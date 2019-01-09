using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos.Character
{
    public struct Character
    {
        /// <summary>
        /// Handle of the body associated with the character.
        /// </summary>
        public int BodyHandle;
        /// <summary>
        /// Character's up direction in the local space of the character's body.
        /// </summary>
        public Vector3 LocalUp;
        /// <summary>
        /// Direction the character is looking in world space. Defines the forward direction for movement.
        /// </summary>
        public Vector3 ViewDirection;
        /// <summary>
        /// Target horizontal velocity. 
        /// X component refers to desired velocity along the strafing direction (perpendicular to the view direction projected down to the surface), 
        /// Y component refers to the desired velocity along the forward direction (aligned with the view direction projected down to the surface).
        /// </summary>
        public Vector2 TargetVelocity;
        /// <summary>
        /// Maximum force the character can apply tangent to the supporting surface to move.
        /// </summary>
        public float MaximumHorizontalForce;
        /// <summary>
        /// Maximum force the character can apply to glue itself to the supporting surface.
        /// </summary>
        public float MaximumVerticalForce;
        /// <summary>
        /// Cosine of the maximum slope angle that the character can treat as a support.
        /// </summary>
        public float CosMaximumSlope;
        /// <summary>
        /// Depth threshold beyond which a contact is considered a support if it the normal allows it.
        /// </summary>
        public float MinimumSupportDepth;
        /// <summary>
        /// Depth threshold beyond which a contact is considered a support if the previous frame had support, even if it isn't deep enough to meet the MinimumSupportDepth.
        /// </summary>
        public float MinimumSupportContinuationDepth;

        /// <summary>
        /// Whether the character is currently supported.
        /// </summary>
        public bool Supported;
        /// <summary>
        /// Collidable supporting the character, if any. Only valid if Supported is true.
        /// </summary>
        public CollidableReference Support;
        /// <summary>
        /// Handle of the character's motion constraint, if any. Only valid if Supported is true.
        /// </summary>
        public int MotionConstraintHandle;
    }

    /// <summary>
    /// System that manages all the characters in a simulation. Responsible for updating movement constraints based on character goals and contact states.
    /// </summary>
    public class CharacterControllers : IDisposable
    {
        Simulation simulation;
        IThreadDispatcher threadDispatcher;
        BufferPool pool;
        IdPool<Buffer<int>> characterIdPool;

        Buffer<int> bodyHandleToCharacterIndex;
        QuickList<Character> characters;

        /// <summary>
        /// Gets the number of characters being controlled.
        /// </summary>
        public int CharacterCount { get { return characters.Count; } }

        /// <summary>
        /// Creates a character controller systme.
        /// </summary>
        /// <param name="pool">Pool to allocate resources from.</param>
        /// <param name="threadDispatcher">Thread dispatcher to be used by the character controller.</param>
        /// <param name="initialCharacterCapacity">Number of characters to initially allocate space for.</param>
        /// <param name="initialBodyHandleCapacity">Number of body handles to initially allocate space for in the body handle->character mapping.</param>
        public CharacterControllers(BufferPool pool, IThreadDispatcher threadDispatcher, int initialCharacterCapacity = 4096, int initialBodyHandleCapacity = 4096)
        {
            this.pool = pool;
            this.threadDispatcher = threadDispatcher;
            characters = new QuickList<Character>(initialCharacterCapacity, pool);
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), initialCharacterCapacity, out characterIdPool);
            ResizeBodyHandleCapacity(initialBodyHandleCapacity);
        }

        /// <summary>
        /// Caches the simulation associated with the characters.
        /// </summary>
        /// <param name="simulation">Simulation to be associated with the characters.</param>
        public void Initialize(Simulation simulation)
        {
            this.simulation = simulation;
        }

        private void ResizeBodyHandleCapacity(int bodyHandleCapacity)
        {
            var oldCapacity = bodyHandleToCharacterIndex.Length;
            pool.Resize(ref bodyHandleToCharacterIndex, bodyHandleCapacity, bodyHandleToCharacterIndex.Length);
            if (bodyHandleToCharacterIndex.Length > oldCapacity)
            {
                Unsafe.InitBlockUnaligned(ref Unsafe.As<int, byte>(ref bodyHandleToCharacterIndex[oldCapacity]), 0xFF, (uint)((bodyHandleToCharacterIndex.Length - oldCapacity) * sizeof(int)));
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Character GetCharacterByIndex(int index)
        {
            return ref characters[index];
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Character AllocateCharacter(int bodyHandle, out int characterIndex)
        {
            characterIndex = characterIdPool.Take();
            characters.EnsureCapacity(characterIndex + 1, pool);
            if (bodyHandle >= bodyHandleToCharacterIndex.Length)
                ResizeBodyHandleCapacity(Math.Max(bodyHandle + 1, bodyHandleToCharacterIndex.Length * 2));
            characterIndex = characters.Count;
            ref var character = ref characters.AllocateUnsafely();
            character = default;
            character.BodyHandle = bodyHandle;
            bodyHandleToCharacterIndex[bodyHandle] = characterIndex;
            return ref character;
        }

        struct SupportCandidate
        {
            public Vector3 OffsetFromCharacter;
            public float Depth;
            public Vector3 OffsetFromSupport;
            public int CharacterIndex;
            public Vector3 Normal;
            public CollidableReference Support;
        }

        struct WorkerCache
        {
            public Buffer<int> CharacterIndexToSupportCandidate;
            public QuickList<SupportCandidate> SupportCandidates;

            public unsafe WorkerCache(int maximumCharacterCount, BufferPool pool)
            {
                pool.Take(maximumCharacterCount, out CharacterIndexToSupportCandidate);
                //We assume unclaimed slots are filled with -1.
                Unsafe.InitBlockUnaligned(CharacterIndexToSupportCandidate.Memory, 0xFF, (uint)CharacterIndexToSupportCandidate.Length * sizeof(int));
                SupportCandidates = new QuickList<SupportCandidate>(maximumCharacterCount, pool);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ref SupportCandidate GetSupportForCharacter(int characterIndex)
            {
                ref var supportIndex = ref CharacterIndexToSupportCandidate[characterIndex];
                if (supportIndex < 0)
                {
                    supportIndex = SupportCandidates.Count;
                    Debug.Assert(SupportCandidates.Span.Length > SupportCandidates.Count, "The support candidates buffer should have been allocated large enough to hold supports for all characters at once.");
                    ref var supportCandidate = ref SupportCandidates.AllocateUnsafely();
                    //New support candidates should be replaced, so set the heuristic to guarantee a change.
                    supportCandidate.Depth = float.MinValue;
                    supportCandidate.CharacterIndex = characterIndex;
                    return ref supportCandidate;
                }
                return ref SupportCandidates[supportIndex];
            }

            public void Dispose(BufferPool pool)
            {
                pool.Return(ref CharacterIndexToSupportCandidate);
            }
        }


        Buffer<WorkerCache> workerCaches;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool TryReportContacts<TManifold>(CollidableReference characterCollidable, CollidableReference supportCollidable, CollidablePair pair, ref TManifold manifold, int workerIndex) where TManifold : struct, IContactManifold
        {
            if (characterCollidable.Mobility == CollidableMobility.Dynamic && characterCollidable.Handle < bodyHandleToCharacterIndex.Length)
            {
                var characterBodyHandle = characterCollidable.Handle;
                var characterIndex = bodyHandleToCharacterIndex[characterBodyHandle];
                if (characterIndex >= 0)
                {
                    //This is actually a character.
                    ref var character = ref characters[characterIndex];
                    //Our job here is to process the manifold into a support representation. That means a single point, normal, and importance heuristic.
                    //Note that we cannot safely pick from the candidates in this function- it is likely executed from a multithreaded context, so all we can do is
                    //output the pair's result into a worker-exclusive buffer.

                    //Contacts with sufficiently negative depth will not be considered support candidates.
                    //Contacts with intermediate depth (above minimum threshold, but still below negative epsilon) may be candidates if the character previously had support.
                    //Contacts with depth above negative epsilon always pass the depth test.

                    //Maximum depth is used to heuristically choose which contact represents the support.
                    //Note that this could be changed to subtly modify the behavior- for example, dotting the movement direction with the support normal and such.
                    //A more careful choice of heuristic could make the character more responsive when trying to 'step' up obstacles.

                    //Note that the body may be inactive during this callback even though it will be activated by new constraints after the narrow phase flushes.
                    //Have to take into account the current potentially inactive location.
                    ref var bodyLocation = ref simulation.Bodies.HandleToLocation[character.BodyHandle];
                    ref var set = ref simulation.Bodies.Sets[bodyLocation.SetIndex];
                    ref var pose = ref set.Poses[bodyLocation.Index];
                    Quaternion.Transform(character.LocalUp, pose.Orientation, out var up);
                    //Note that this branch is compiled out- the generic constraints force type specialization.
                    if (manifold.Convex)
                    {
                        ref var convexManifold = ref Unsafe.As<TManifold, ConvexContactManifold>(ref manifold);
                        var upDot = Vector3.Dot(convexManifold.Normal, up);
                        //The narrow phase generates contacts with normals pointing from B to A by convention.
                        //If the character is collidable B, then we need to negate the comparison.
                        if ((pair.B.Packed == characterCollidable.Packed ? -upDot : upDot) > character.CosMaximumSlope)
                        {
                            //This manifold has a slope that is potentially supportive.
                            //Can the maximum depth contact be used as a support?
                            var maximumDepth = convexManifold.Contact0.Depth;
                            var maximumDepthIndex = 0;
                            for (int i = 1; i < convexManifold.Count; ++i)
                            {
                                ref var candidateDepth = ref Unsafe.Add(ref convexManifold.Contact0, i).Depth;
                                if (candidateDepth > maximumDepth)
                                {
                                    maximumDepth = candidateDepth;
                                    maximumDepthIndex = i;
                                }
                            }
                            if (maximumDepth >= character.MinimumSupportDepth || (character.Supported && maximumDepth > character.MinimumSupportContinuationDepth))
                            {
                                ref var supportCandidate = ref workerCaches[workerIndex].GetSupportForCharacter(characterIndex);
                                if (supportCandidate.Depth < maximumDepth)
                                {
                                    //This support candidate should be replaced.
                                    supportCandidate.Normal = convexManifold.Normal;
                                    supportCandidate.Depth = maximumDepth;
                                    ref var deepestContact = ref Unsafe.Add(ref convexManifold.Contact0, maximumDepthIndex);
                                    var offsetFromB = deepestContact.Offset - convexManifold.OffsetB;
                                    if (pair.B.Packed == characterCollidable.Packed)
                                    {
                                        supportCandidate.OffsetFromCharacter = offsetFromB;
                                        supportCandidate.OffsetFromSupport = deepestContact.Offset;
                                    }
                                    else
                                    {
                                        supportCandidate.OffsetFromCharacter = deepestContact.Offset;
                                        supportCandidate.OffsetFromSupport = offsetFromB;
                                    }
                                    supportCandidate.Support = supportCollidable;
                                }
                            }
                        }
                    }
                    else
                    {
                        ref var nonconvexManifold = ref Unsafe.As<TManifold, NonconvexContactManifold>(ref manifold);
                        //The narrow phase generates contacts with normals pointing from B to A by convention.
                        //If the character is collidable B, then we need to negate the comparison.
                        //This manifold has a slope that is potentially supportive.
                        //Can the maximum depth contact be used as a support?
                        var maximumDepth = float.MinValue;
                        var maximumDepthIndex = -1;
                        for (int i = 0; i < nonconvexManifold.Count; ++i)
                        {
                            ref var candidate = ref Unsafe.Add(ref nonconvexManifold.Contact0, i);
                            if (candidate.Depth > maximumDepth)
                            {
                                //All the nonconvex candidates can have different normals, so we have to perform the (calibrated) normal test on every single one.
                                var upDot = Vector3.Dot(candidate.Normal, up);
                                if ((pair.B.Packed == characterCollidable.Packed ? -upDot : upDot) > character.CosMaximumSlope)
                                {
                                    maximumDepth = candidate.Depth;
                                    maximumDepthIndex = i;
                                }
                            }
                        }
                        if (maximumDepth >= character.MinimumSupportDepth || (character.Supported && maximumDepth > character.MinimumSupportContinuationDepth))
                        {
                            ref var supportCandidate = ref workerCaches[workerIndex].GetSupportForCharacter(characterIndex);
                            if (supportCandidate.Depth < maximumDepth)
                            {
                                //This support candidate should be replaced.
                                ref var deepestContact = ref Unsafe.Add(ref nonconvexManifold.Contact0, maximumDepthIndex);
                                supportCandidate.Normal = deepestContact.Normal;
                                supportCandidate.Depth = maximumDepth;
                                var offsetFromB = deepestContact.Offset - nonconvexManifold.OffsetB;
                                if (pair.B.Packed == characterCollidable.Packed)
                                {
                                    supportCandidate.OffsetFromCharacter = offsetFromB;
                                    supportCandidate.OffsetFromSupport = deepestContact.Offset;
                                }
                                else
                                {
                                    supportCandidate.OffsetFromCharacter = deepestContact.Offset;
                                    supportCandidate.OffsetFromSupport = offsetFromB;
                                }
                                supportCandidate.Support = supportCollidable;
                            }
                        }
                    }
                    return true;
                }
            }
            return false;
        }


        /// <summary>
        /// Reports contacts about a collision to the character system. If the pair does not involve a character, does nothing and returns false.
        /// </summary>
        /// <param name="pair">Pair of objects associated with the contact manifold.</param>
        /// <param name="manifold">Contact manifold between the colliding objects.</param>
        /// <param name="workerIndex">Index of the currently executing worker thread.</param>
        /// <param name="materialProperties">Material properties for this pair. Will be modified if the pair involves a character.</param>
        /// <returns>True if the pair involved a character pair, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryReportContacts<TManifold>(in CollidablePair pair, ref TManifold manifold, int workerIndex, ref PairMaterialProperties materialProperties) where TManifold : struct, IContactManifold
        {
            Debug.Assert(workerCaches.Allocated && workerIndex < workerCaches.Length && workerCaches[workerIndex].SupportCandidates.Span.Allocated,
                "Worker caches weren't properly allocated; did you forget to call PrepareForContacts before collision detection?");
            //It's possible for neither, one, or both collidables to be a character. Check each one, treating the other as a potential support.
            var aIsCharacter = TryReportContacts(pair.A, pair.B, pair, ref manifold, workerIndex);
            var bIsCharacter = TryReportContacts(pair.B, pair.A, pair, ref manifold, workerIndex);
            if (aIsCharacter || bIsCharacter)
            {
                //The character's motion over the surface should be controlled entirely by the horizontal motion constraint.
                //Note- you could use the friction coefficient to change the horizontal motion constraint's maximum force to simulate different environments if you want.
                //That would just require caching a bit more information for the AnalyzeContacts function to use.
                materialProperties.FrictionCoefficient = 0;
                return true;
            }
            return false;
        }

        /// <summary>
        /// Preallocates space for support data collected during the narrow phase. Should be called before the narrow phase executes.
        /// </summary>
        public void PrepareForContacts()
        {
            Debug.Assert(!workerCaches.Allocated, "Worker caches were already allocated; did you forget to call AnalyzeContacts after collision detection to flush the previous frame's results?");
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            pool.Take(threadCount, out workerCaches);
            workerCaches = workerCaches.Slice(0, threadCount);
            for (int i = 0; i < workerCaches.Length; ++i)
            {
                workerCaches[i] = new WorkerCache(characters.Count, pool);
            }
        }


        /// <summary>
        /// Updates all character support states and motion constraints based on the current character goals and all the contacts collected since the last call to AnalyzeContacts. 
        /// Attach to a simulation callback where the most recent contact is available and before the solver executes.
        /// </summary>
        public void AnalyzeContacts()
        {
            Debug.Assert(workerCaches.Allocated, "Worker caches weren't properly allocated; did you forget to call PrepareForContacts before collision detection?");
            ref var workerCache0 = ref workerCaches[0];
            for (int i = 1; i < workerCaches.Length; ++i)
            {
                ref var workerCache = ref workerCaches[i];
                for (int j = 0; j < workerCache.SupportCandidates.Count; ++j)
                {
                    ref var sourceSupportCandidate = ref workerCache.SupportCandidates[j];
                    ref var targetSupportCandidate = ref workerCache0.GetSupportForCharacter(sourceSupportCandidate.CharacterIndex);
                    //If this worker's support candidate is a better choice than the current one, use the worker's candidate.
                    if (targetSupportCandidate.Depth < sourceSupportCandidate.Depth)
                    {
                        targetSupportCandidate = sourceSupportCandidate;
                    }
                }
                workerCache.Dispose(pool);
            }
            //Note that the merged workerCache0 now contains the updated state of all active characters with supports. It does *not* contain updated information for inactive characters
            //or for characters that do not currently have any support.

            //Active characters that previously had support and no longer do should have their constraints removed from the solver. 
            for (int i = 0; i < characters.Count; ++i)
            {
                ref var character = ref characters[i];
                ref var bodyLocation = ref simulation.Bodies.HandleToLocation[character.BodyHandle];
                //Note that this iterates over both active and inactive characters rather than segmenting inactive characters into their own collection.
                //This demands branching, but the expectation is that the vast majority of characters will be active, so there is less value in copying them into stasis.
                if (bodyLocation.SetIndex == 0)
                {
                    //The body is active. We may need to remove the associated constraint from the solver. Remove if any of the following hold:
                    //1) The character was previously supported but is no longer.
                    //2) The character was previously supported by a body, and is now supported by a different body.
                    //3) The character was previously supported by a static, and is now supported by a body.
                    //4) The character was previously supported by a body, and is now supported by a static.
                    var supportCandidateIndex = workerCache0.CharacterIndexToSupportCandidate[i];
                    var shouldRemove = character.Supported && (supportCandidateIndex < 0 || character.Support.Packed != workerCache0.SupportCandidates[supportCandidateIndex].Support.Packed);
                    if (shouldRemove)
                    {
                        //Remove the constraint.
                        simulation.Solver.Remove(character.MotionConstraintHandle);
                    }

                    if (supportCandidateIndex >= 0)
                    {
                        //If a support currently exists and there is still an old constraint, then update it.
                        //If a support currently exists and there is not an old constraint, add the new constraint.
                        ref var supportCandidate = ref workerCache0.SupportCandidates[supportCandidateIndex];

                        //Project the view direction down onto the surface as represented by the contact normala.

                        Matrix3x3 surfaceBasis;
                        surfaceBasis.Y = Vector3.Dot(supportCandidate.OffsetFromCharacter, supportCandidate.Normal) > 0 ? -supportCandidate.Normal : supportCandidate.Normal;
                        //Note negation: we're using a right handed basis where -Z is forward, +Z is backward.
                        surfaceBasis.Z = Vector3.Dot(character.ViewDirection, surfaceBasis.Y) * surfaceBasis.Y - character.ViewDirection;
                        var zLengthSquared = surfaceBasis.Z.LengthSquared();
                        if (zLengthSquared > 1e-12f)
                        {
                            surfaceBasis.Z /= MathF.Sqrt(zLengthSquared);
                        }
                        else
                        {
                            Quaternion.GetQuaternionBetweenNormalizedVectors(Vector3.UnitY, surfaceBasis.Y, out var rotation);
                            Quaternion.TransformUnitZ(rotation, out surfaceBasis.Z);
                        }
                        Vector3x.Cross(surfaceBasis.Y, surfaceBasis.Z, out surfaceBasis.X);
                        Quaternion.CreateFromRotationMatrix(surfaceBasis, out var surfaceBasisQuaternion);
                        if (supportCandidate.Support.Mobility != CollidableMobility.Static)
                        {
                            //The character is supported by a body.
                            var motionConstraint = new DynamicCharacterMotionConstraint
                            {
                                MaximumHorizontalForce = character.MaximumHorizontalForce,
                                MaximumVerticalForce = character.MaximumVerticalForce,
                                OffsetFromCharacterToSupportPoint = supportCandidate.OffsetFromCharacter,
                                OffsetFromSupportToSupportPoint = supportCandidate.OffsetFromSupport,
                                SurfaceBasis = surfaceBasisQuaternion,
                                TargetVelocity = character.TargetVelocity,
                                Depth = supportCandidate.Depth
                            };
                            if (character.Supported && !shouldRemove)
                            {
                                //Already exists, update it.
                                simulation.Solver.ApplyDescriptionWithoutWaking(character.MotionConstraintHandle, ref motionConstraint);
                            }
                            else
                            {
                                //Doesn't exist, add it.
                                character.MotionConstraintHandle = simulation.Solver.Add(character.BodyHandle, supportCandidate.Support.Handle, ref motionConstraint);
                            }
                        }
                        else
                        {
                            //The character is supported by a static.
                            var motionConstraint = new StaticCharacterMotionConstraint
                            {
                                MaximumHorizontalForce = character.MaximumHorizontalForce,
                                MaximumVerticalForce = character.MaximumVerticalForce,
                                OffsetFromCharacterToSupportPoint = supportCandidate.OffsetFromCharacter,
                                SurfaceBasis = surfaceBasisQuaternion,
                                TargetVelocity = character.TargetVelocity,
                                Depth = supportCandidate.Depth
                            };
                            if (character.Supported && !shouldRemove)
                            {
                                //Already exists, update it.
                                simulation.Solver.ApplyDescriptionWithoutWaking(character.MotionConstraintHandle, ref motionConstraint);
                            }
                            else
                            {
                                //Doesn't exist, add it.
                                character.MotionConstraintHandle = simulation.Solver.Add(character.BodyHandle, ref motionConstraint);
                            }
                        }
                        character.Supported = true;
                        character.Support = supportCandidate.Support;
                    }
                    else
                    {
                        character.Supported = false;
                    }
                }
            }

            workerCache0.Dispose(pool);
            pool.Return(ref workerCaches);
        }

        bool disposed;
        /// <summary>
        /// Returns pool-allocated resources.
        /// </summary>
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                characterIdPool.Dispose(pool.SpecializeFor<int>());
                characters.Dispose(pool);
                pool.Return(ref bodyHandleToCharacterIndex);
            }
        }
    }
}
