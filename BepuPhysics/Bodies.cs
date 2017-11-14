using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using BepuUtilities.Collections;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;

namespace BepuPhysics
{
    public struct BodyLocation
    {
        /// <summary>
        /// Index of the set owning the body reference. If the island index is 0, the body is active.
        /// </summary>
        public int SetIndex;
        /// <summary>
        /// Index of the body within its owning set. If the body is active (and so the Island index is -1), this is an index into the Bodies data arrays. 
        /// If it is nonnegative, it is an index into the inactive island 
        /// </summary>
        public int Index;
    }

    /// <summary>
    /// Collection of all allocated bodies.
    /// </summary>
    public class Bodies
    {

        /// <summary>
        /// Remaps a body handle to the actual array index of the body.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public Buffer<BodyLocation> HandleToLocation;
        public IdPool<Buffer<int>> HandlePool;
        /// <summary>
        /// The set of existing bodies. The slot at index 0 contains all active bodies. Later slots, if allocated, contain the bodies associated with inactive islands.
        /// Note that this buffer does not necessarily contain contiguous elements. When a set is removed, a gap remains.
        /// </summary>
        public Buffer<BodySet> Sets;
        /// <summary>
        /// Gets a reference to the active set, stored in the index 0 of the Sets buffer.
        /// </summary>
        /// <returns>Reference to the active body set.</returns>
        public unsafe ref BodySet ActiveSet { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return ref Unsafe.As<byte, BodySet>(ref *Sets.Memory); } }

        /// <summary>
        /// The world transformed inertias of active bodies as of the last update. Note that this is not automatically updated for direct orientation changes or for body memory moves.
        /// It is only updated once during the frame. It should be treated as ephemeral information.
        /// </summary>
        public Buffer<BodyInertia> Inertias;
        protected internal BufferPool pool;

        protected internal Statics statics;
        protected internal Shapes shapes;
        protected internal BroadPhase broadPhase;
        protected internal Solver solver;

        public unsafe Bodies(BufferPool pool, Statics statics, Shapes shapes, BroadPhase broadPhase, Solver solver, int initialBodyCapacity, int initialSetCapacity)
        {
            this.pool = pool;

            //Note that the id pool only grows upon removal, so this is just a heuristic initialization.
            //You could get by with something a lot less aggressive, but it does tend to avoid resizes in the case of extreme churn.
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), initialBodyCapacity, out HandlePool);
            pool.SpecializeFor<BodySet>().Take(initialSetCapacity, out var Sets);
            ActiveSet = new BodySet(initialBodyCapacity, pool);
            this.statics = statics;
            this.shapes = shapes;
            this.broadPhase = broadPhase;
            this.solver = solver;
        }


        void AddCollidableToBroadPhase(int bodyHandle, ref RigidPose pose, ref BodyInertia localInertia, ref Collidable collidable)
        {
            //This body has a collidable; stick it in the broadphase.
            //Note that we have to calculate an initial bounding box for the broad phase to be able to insert it efficiently.
            //(In the event of batch adds, you'll want to use batched AABB calculations or just use cached values.)
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            shapes.UpdateBounds(ref pose, ref collidable.Shape, out var bodyBounds);
            //Note that new body collidables are always assumed to be active.
            collidable.BroadPhaseIndex =
                broadPhase.AddActive(
                    new CollidableReference(IsKinematic(ref localInertia) ? CollidableMobility.Kinematic : CollidableMobility.Dynamic, bodyHandle),
                    ref bodyBounds);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void UpdateCollidableBroadPhaseIndex(int handle, int newBroadPhaseIndex)
        {
            ref var movedOriginalLocation = ref HandleToLocation[handle];
            Sets[movedOriginalLocation.SetIndex].Collidables[movedOriginalLocation.Index].BroadPhaseIndex = newBroadPhaseIndex;
        }
        void RemoveCollidableFromBroadPhase(ref BodyLocation location, ref Collidable collidable)
        {
            var removedBroadPhaseIndex = collidable.BroadPhaseIndex;
            //The below removes a body's collidable from the broad phase and adjusts the broad phase index of any moved leaf.
            if (location.SetIndex == 0)
            {
                //This is an active body. Remove it from the active broad phase structure.
                if (broadPhase.RemoveActiveAt(removedBroadPhaseIndex, out var movedLeaf))
                {
                    //Since we removed an active body, we know that the thing that moved in the broad phase is also an active body.
                    //There is no such thing as an 'active' static object.
                    Debug.Assert(movedLeaf.Mobility != CollidableMobility.Static);
                    UpdateCollidableBroadPhaseIndex(movedLeaf.Handle, removedBroadPhaseIndex);
                }
            }
            else
            {
                Debug.Assert(location.SetIndex > 0 && location.SetIndex < Sets.Length, "All inactive islands should have positive indices.");
                //This is an inactive body. Remove it from the inactive broad phase structure.
                if (broadPhase.RemoveStaticAt(removedBroadPhaseIndex, out var movedLeaf))
                {
                    //When removing an inactive body, the collidable that moves may not be another inactive body. It may be a static.
                    if (movedLeaf.Mobility == CollidableMobility.Static)
                    {
                        statics.UpdateCollidableBroadPhaseIndex(movedLeaf.Handle, removedBroadPhaseIndex);
                    }
                    else
                    {
                        UpdateCollidableBroadPhaseIndex(movedLeaf.Handle, removedBroadPhaseIndex);
                    }
                }
            }

        }
        /// <summary>
        /// Adds a new active body to the simulation.
        /// </summary>
        /// <param name="description">Description of the body to add.</param>
        /// <returns>Handle of the created body.</returns>
        public unsafe int Add(ref BodyDescription description)
        {
            Debug.Assert(HandleToLocation.Allocated, "The backing memory of the bodies set should be initialized before use.");
            var handle = HandlePool.Take();
            Debug.Assert(handle <= HandleToLocation.Length, "It should be impossible for a new handle to end up more than one slot beyond the current handle to index array. " +
                "This would imply some form of resize or compaction bug.");
            if (handle == HandleToLocation.Length)
            {
                //Out of room; need to resize.
                ResizeHandles(HandleToLocation.Length << 1);
            }
            Debug.Assert(Math.Abs(description.Pose.Orientation.Length() - 1) < 1e-6f, "Orientation should be initialized to a unit length quaternion.");

            //All new bodies are active for simplicity. Someday, it may be worth offering an optimized path for inactives, but it adds complexity.
            //(Directly adding inactive bodies can be helpful in some networked open world scenarios.)
            var index = ActiveSet.Add(ref description, handle);
            HandleToLocation[handle] = new BodyLocation { SetIndex = -1, Index = index };

            if (description.Collidable.Shape.Exists)
            {
                AddCollidableToBroadPhase(handle, ref description.Pose, ref description.LocalInertia, ref ActiveSet.Collidables[index]);
            }
            return handle;
        }

        /// <summary>
        /// Removes a body from the set by its location. Assumes that the input location is valid.
        /// </summary>
        /// <param name="location">Location of the body to remove.</param>
        public void RemoveAt(BodyLocation location)
        {
            Debug.Assert(location.SetIndex >= 0 && location.SetIndex < Sets.Length && Sets[location.SetIndex].Allocated, "Target removal must exist.");
            ref var set = ref Sets[location.SetIndex];
            Debug.Assert(location.Index >= 0 && location.Index < set.Count);
            ValidateExistingHandle(set.IndexToHandle[location.Index]);
            ref var collidable = ref set.Collidables[location.Index];
            if (collidable.Shape.Exists)
            {
                //The collidable exists, so it should be removed from the broadphase.
                RemoveCollidableFromBroadPhase(ref location, ref collidable);
            }

            var bodyMoved = set.RemoveAt(location.Index, out var handle, out var movedBodyIndex, out var movedBodyHandle);
            //Note that constraints in inactive islands reference bodies by handle only, so we only need to notify the solver about changes to active bodies.
            if (bodyMoved && location.SetIndex == 0)
            {
                //While the removed body doesn't have any constraints associated with it, the body that gets moved to fill its slot might!
                //We're borrowing the body optimizer's logic here. You could share a bit more- the body layout optimizer has to deal with the same stuff, though it's optimized for swaps.
                //TODO: the logic behind the body memory move really should be moved in here with the more recent designs.
                BodyLayoutOptimizer.UpdateForBodyMemoryMove(movedBodyIndex, location.Index, this, solver);
            }

            Debug.Assert(HandleToLocation[movedBodyHandle].SetIndex == -1 && HandleToLocation[movedBodyHandle].Index == movedBodyIndex);
            HandleToLocation[movedBodyHandle].Index = location.Index;
            HandlePool.Return(handle, pool.SpecializeFor<int>());
            ref var bodyLocation = ref HandleToLocation[handle];
            bodyLocation.SetIndex = -1;
            bodyLocation.Index = -1;
        }

        /// <summary>
        /// Removes a body from the set by its handle.
        /// </summary>
        /// <param name="handle">Handle of the body to remove.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(int handle)
        {
            ValidateExistingHandle(handle);
            RemoveAt(HandleToLocation[handle]);
        }

        /// <summary>
        /// Gets whether the inertia matches that of a kinematic body (that is, all inverse mass and inertia components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        public static bool IsKinematic(ref BodyInertia inertia)
        {
            return inertia.InverseMass == 0 &&
                   inertia.InverseInertiaTensor.M11 == 0 &&
                   inertia.InverseInertiaTensor.M21 == 0 &&
                   inertia.InverseInertiaTensor.M22 == 0 &&
                   inertia.InverseInertiaTensor.M31 == 0 &&
                   inertia.InverseInertiaTensor.M32 == 0 &&
                   inertia.InverseInertiaTensor.M33 == 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateKinematicState(int handle, ref BodyLocation location, ref BodySet set)
        {
            ref var collidable = ref set.Collidables[location.Index];
            var kinematic = IsKinematic(ref set.LocalInertias[location.Index]);
            if (collidable.Shape.Exists)
            {
                var mobility = kinematic ? CollidableMobility.Kinematic : CollidableMobility.Dynamic;
                if (location.SetIndex == 0)
                {
                    broadPhase.activeLeaves[collidable.BroadPhaseIndex] = new CollidableReference(mobility, handle);
                }
                else
                {
                    broadPhase.staticLeaves[collidable.BroadPhaseIndex] = new CollidableReference(mobility, handle);
                }
            }
            set.Activity[location.Index].Kinematic = kinematic;
        }

        /// <summary>
        /// Changes the local mass and inertia tensor associated with a body. Properly handles the transition between kinematic and dynamic.
        /// </summary>
        /// <param name="handle">Handle of the body whose inertia should change.</param>
        /// <param name="inertia">New inertia for the body.</param>
        /// <remarks>
        /// This function is only necessary when the inertia change could potentially result in a transition between dynamic and kinematic states.
        /// If it is guaranteed to be dynamic before and after the change, the inertia can be directly modified without issue.
        /// </remarks>
        public void ChangeLocalInertia(int handle, ref BodyInertia inertia)
        {
            ref var location = ref HandleToLocation[handle];
            ref var set = ref Sets[location.SetIndex];
            set.LocalInertias[location.Index] = inertia;
            UpdateKinematicState(handle, ref location, ref set);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateForShapeChange(int handle, ref BodyLocation location, ref BodySet set, TypedIndex oldShape, TypedIndex newShape)
        {
            if(oldShape.Exists != newShape.Exists)
            {
                if(newShape.Exists)
                {
                    //Add a collidable to the simulation for the new shape.
                    AddCollidableToBroadPhase(handle, ref set.Poses[location.Index], ref set.LocalInertias[location.Index], ref set.Collidables[location.Index]);
                }
                else
                {
                    //Remove the now-unused collidable from the simulation.
                    RemoveCollidableFromBroadPhase(ref location, ref set.Collidables[location.Index]);
                }
            }
        }
        /// <summary>
        /// Changes the shape of a body. Properly handles the transition between shapeless and shapeful.
        /// </summary>
        /// <param name="handle">Handle of the body to change the shape of.</param>
        /// <param name="newShape">Index of the new shape to use for the body.</param>
        public void ChangeShape(int handle, TypedIndex newShape)
        {
            ref var location = ref HandleToLocation[handle];
            ref var set = ref Sets[location.SetIndex];
            ref var collidable = ref set.Collidables[location.Index];
            var oldShape = collidable.Shape;
            collidable.Shape = newShape;
            UpdateForShapeChange(handle, ref location, ref set, oldShape, newShape);
        }

        /// <summary>
        /// Applies a description to a body. Properly handles any transitions between dynamic and kinematic and between shapeless and shapeful.
        /// </summary>
        /// <param name="handle">Handle of the body to receive the description.</param>
        /// <param name="description">Description to apply to the body.</param>
        public void ApplyDescription(int handle, ref BodyDescription description)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle];
            ref var set = ref Sets[location.SetIndex];
            ref var collidable = ref set.Collidables[location.Index];
            var oldShape = collidable.Shape;
            set.ApplyDescriptionByIndex(location.Index, ref description);
            UpdateForShapeChange(handle, ref location, ref set, oldShape, description.Collidable.Shape);           
            UpdateKinematicState(handle, ref location, ref set);
        }

        /// <summary>
        /// Gets the description of a body by handle.
        /// </summary>
        /// <param name="handle">Handle to </param>
        /// <param name="description"></param>
        public void GetDescription(int handle, out BodyDescription description)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle];
            ref var set = ref Sets[location.SetIndex];
            set.GetDescription(location.Index, out description);
        }


        [Conditional("DEBUG")]
        internal void ValidateExistingHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle <= HandlePool.HighestPossiblyClaimedId && HandlePool.HighestPossiblyClaimedId < HandleToLocation.Length,
                "Existing handles must fit within the body handle->index mapping.");
            ref var location = ref HandleToLocation[handle];
            Debug.Assert(location.SetIndex >= 0 && location.SetIndex < Sets.Length);
            ref var set = ref Sets[location.SetIndex];
            Debug.Assert(set.Allocated);
            Debug.Assert(set.Count <= set.IndexToHandle.Length);
            Debug.Assert(location.Index >= 0 && location.Index < set.Count, "Body index must fall within the existing body set.");
            Debug.Assert(set.IndexToHandle[location.Index] == handle, "Handle->index must match index->handle map.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherInertiaForBody(ref BodyInertia source, ref float targetInertiaBase, int targetLaneIndex)
        {
            ref var targetSlot = ref Unsafe.Add(ref targetInertiaBase, targetLaneIndex);
            targetSlot = source.InverseInertiaTensor.M11;
            Unsafe.Add(ref targetSlot, Vector<float>.Count) = source.InverseInertiaTensor.M21;
            Unsafe.Add(ref targetSlot, 2 * Vector<float>.Count) = source.InverseInertiaTensor.M22;
            Unsafe.Add(ref targetSlot, 3 * Vector<float>.Count) = source.InverseInertiaTensor.M31;
            Unsafe.Add(ref targetSlot, 4 * Vector<float>.Count) = source.InverseInertiaTensor.M32;
            Unsafe.Add(ref targetSlot, 5 * Vector<float>.Count) = source.InverseInertiaTensor.M33;
            Unsafe.Add(ref targetSlot, 6 * Vector<float>.Count) = source.InverseMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherPoseForBody(ref RigidPose source, ref float targetPositionBase, ref float targetOrientationBase, int targetLaneIndex)
        {
            ref var targetPositionSlot = ref Unsafe.Add(ref targetPositionBase, targetLaneIndex);
            ref var targetOrientationSlot = ref Unsafe.Add(ref targetOrientationBase, targetLaneIndex);
            targetPositionSlot = source.Position.X;
            Unsafe.Add(ref targetPositionSlot, Vector<float>.Count) = source.Position.Y;
            Unsafe.Add(ref targetPositionSlot, 2 * Vector<float>.Count) = source.Position.Z;
            targetOrientationSlot = source.Orientation.X;
            Unsafe.Add(ref targetOrientationSlot, Vector<float>.Count) = source.Orientation.Y;
            Unsafe.Add(ref targetOrientationSlot, 2 * Vector<float>.Count) = source.Orientation.Z;
            Unsafe.Add(ref targetOrientationSlot, 3 * Vector<float>.Count) = source.Orientation.W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherVelocityForBody(ref BodyVelocity source, ref float targetLinearVelocityBase, ref float targetAngularVelocityBase, int targetLaneIndex)
        {
            ref var targetLinearSlot = ref Unsafe.Add(ref targetLinearVelocityBase, targetLaneIndex);
            ref var targetAngularSlot = ref Unsafe.Add(ref targetAngularVelocityBase, targetLaneIndex);
            targetLinearSlot = source.Linear.X;
            Unsafe.Add(ref targetLinearSlot, Vector<float>.Count) = source.Linear.Y;
            Unsafe.Add(ref targetLinearSlot, 2 * Vector<float>.Count) = source.Linear.Z;
            targetAngularSlot = source.Angular.X;
            Unsafe.Add(ref targetAngularSlot, Vector<float>.Count) = source.Angular.Y;
            Unsafe.Add(ref targetAngularSlot, 2 * Vector<float>.Count) = source.Angular.Z;
        }

        //TODO: In future versions, we will likely store the body position in different forms to allow for extremely large worlds.
        //That will be an opt-in feature. The default implementation will use the FP32 representation, but the user could choose to swap it out for a int64 based representation.
        //This affects other systems- AABB calculation, pose integration, solving, and in extreme (64 bit) cases, the broadphase.
        //We want to insulate other systems from direct knowledge about the implementation of positions when possible.
        //These functions support the solver's needs while hiding absolute positions.
        //In order to support other absolute positions, we'll need alternate implementations of this and other functions.
        //But for the most part, we don't want to pay the overhead of an abstract invocation within the inner loop of the solver. 
        //Given the current limits of C# and the compiler, the best option seems to be a interface implementing struct that provides this functionality.
        //The users would be type specialized by the compiler, avoiding virtual invocation. 
        [MethodImpl(MethodImplOptions.AggressiveInlining)] //Note that this doesn't do anything at the moment- the stackalloc hack blocks inlining.
        internal void GatherInertiaAndPose(ref TwoBodyReferences references, int count,
            out Vector3Wide localPositionB, out QuaternionWide orientationA, out QuaternionWide orientationB,
            out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            unsafe
            {
                //Note that we're storing data into vectors by reinterpreting the underlying memory.
                //Unfortunately, there's a compiler bug: https://github.com/dotnet/coreclr/issues/11804
                //Without intervention, a whole lotta stuff goes sideways when the data stored to registers is loaded from memory, 
                //resulting in uninitialized data flying around. Best case, constraints using this diverged massively as the localPositionB was whoknowswhat,
                //worst case the entire process gets torn down by access violations.
                //The inclusion of this hack convinces the JIT to store the gathered values into stack memory rather than aggressively enregistering it.
                //(Why? I didn't bother checking too closely. Maybe the JIT's more aggressive about looking for type punning in the presence of certain kinds of unsafe code.)
                //TODO: remove this once the fix is in. It blocks inlining.
                var hiPleaseDontRemoveThisWithoutTestingOrEverythingMightExplode = stackalloc byte[0];
            }
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);
            ref var targetInertiaBaseB = ref Unsafe.As<Vector<float>, float>(ref inertiaB.InverseInertiaTensor.M11);
            Vector3Wide positionA, positionB;
            ref var targetPositionBaseA = ref Unsafe.As<Vector<float>, float>(ref positionA.X);
            ref var targetPositionBaseB = ref Unsafe.As<Vector<float>, float>(ref positionB.X);
            ref var targetOrientationBaseA = ref Unsafe.As<Vector<float>, float>(ref orientationA.X);
            ref var targetOrientationBaseB = ref Unsafe.As<Vector<float>, float>(ref orientationB.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);

            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                GatherInertiaForBody(ref Inertias[indexA], ref targetInertiaBaseA, i);
                GatherPoseForBody(ref poses[indexA], ref targetPositionBaseA, ref targetOrientationBaseA, i);
                var indexB = Unsafe.Add(ref indexA, Vector<float>.Count);
                GatherInertiaForBody(ref Inertias[indexB], ref targetInertiaBaseB, i);
                GatherPoseForBody(ref poses[indexB], ref targetPositionBaseB, ref targetOrientationBaseB, i);
            }
            Vector3Wide.Subtract(ref positionB, ref positionA, out localPositionB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherInertia(ref TwoBodyReferences references, int count,
            out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);
            ref var targetInertiaBaseB = ref Unsafe.As<Vector<float>, float>(ref inertiaB.InverseInertiaTensor.M11);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);

            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                GatherInertiaForBody(ref Inertias[indexA], ref targetInertiaBaseA, i);
                var indexB = Unsafe.Add(ref indexA, Vector<float>.Count);
                GatherInertiaForBody(ref Inertias[indexB], ref targetInertiaBaseB, i);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherInertia(ref Vector<int> references, int count,
            out BodyInertias inertiaA)
        {
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references);

            for (int i = 0; i < count; ++i)
            {
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexA, i)], ref targetInertiaBaseA, i);
            }
        }

        //This looks a little different because it's used by AABB calculation, not constraint pairs.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherDataForBounds(ref int start, int count, out RigidPoses poses, out BodyVelocities velocities, out Vector<int> shapeIndices, out Vector<float> maximumExpansion)
        {
            Debug.Assert(count <= Vector<float>.Count);
            ref var targetPositionBase = ref Unsafe.As<Vector<float>, float>(ref poses.Position.X);
            ref var targetOrientationBase = ref Unsafe.As<Vector<float>, float>(ref poses.Orientation.X);
            ref var targetLinearBase = ref Unsafe.As<Vector<float>, float>(ref velocities.LinearVelocity.X);
            ref var targetAngularBase = ref Unsafe.As<Vector<float>, float>(ref velocities.AngularVelocity.X);
            ref var targetShapeBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            ref var targetExpansionBase = ref Unsafe.As<Vector<float>, float>(ref maximumExpansion);
            ref var activeSet = ref ActiveSet;
            for (int i = 0; i < count; ++i)
            {
                var index = Unsafe.Add(ref start, i);
                GatherPoseForBody(ref activeSet.Poses[index], ref targetPositionBase, ref targetOrientationBase, i);
                GatherVelocityForBody(ref activeSet.Velocities[index], ref targetLinearBase, ref targetAngularBase, i);
                ref var collidable = ref activeSet.Collidables[index];
                Unsafe.Add(ref targetShapeBase, i) = collidable.Shape.Index;
                //Not entirely pleased with the fact that this pulls in some logic from bounds calculation.
                Unsafe.Add(ref targetExpansionBase, i) = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
            }
        }


        /// <summary>
        /// Clears all bodies from all sets without releasing any memory that wouldn't be released by a sequence of regular removals.
        /// </summary>
        public unsafe void Clear()
        {
            ActiveSet.Clear();
            //While the top level pool represents active bodies and will persist (as it would after a series of removals),
            //subsequent sets represent inactive bodies. When they are not present, the backing memory is released.
            for (int i = 1; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (set.Allocated)
                    set.Dispose(pool);
            }
            Unsafe.InitBlockUnaligned(HandleToLocation.Memory, 0xFF, (uint)(sizeof(BodyLocation) * HandleToLocation.Length));
            HandlePool.Clear();
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void ResizeHandles(int newCapacity)
        {
            newCapacity = BufferPool<BodyLocation>.GetLowestContainingElementCount(newCapacity);
            if (newCapacity != HandleToLocation.Length)
            {
                var highestPotentialHandle = HandlePool.HighestPossiblyClaimedId + 1;
                pool.SpecializeFor<BodyLocation>().Resize(ref HandleToLocation, newCapacity, highestPotentialHandle);
                if (HandleToLocation.Length > highestPotentialHandle)
                {
                    Unsafe.InitBlockUnaligned(
                      ((BodyLocation*)HandleToLocation.Memory) + highestPotentialHandle, 0xFF,
                      (uint)(sizeof(BodyLocation) * (HandleToLocation.Length - highestPotentialHandle)));
                }
            }
        }
        /// <summary>
        /// Resizes the allocated spans for active body data. Note that this is conservative; it will never orphan existing objects.
        /// </summary>
        /// <param name="capacity">Target body data capacity.</param>
        public void Resize(int capacity)
        {
            var targetBodyCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(capacity, ActiveSet.Count));
            if (ActiveSet.IndexToHandle.Length != targetBodyCapacity)
            {
                ActiveSet.InternalResize(targetBodyCapacity, pool);
            }
            var targetHandleCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(capacity, HandlePool.HighestPossiblyClaimedId + 1));
            if (HandleToLocation.Length != targetHandleCapacity)
            {
                ResizeHandles(targetHandleCapacity);
            }
        }

        /// <summary>
        /// Increases the size of active body buffers if needed to hold the target capacity.
        /// </summary>
        /// <param name="capacity">Target data capacity.</param>
        public void EnsureCapacity(int capacity)
        {
            if (ActiveSet.IndexToHandle.Length < capacity)
            {
                ActiveSet.InternalResize(capacity, pool);
            }
            if (HandleToLocation.Length < capacity)
            {
                ResizeHandles(capacity);
            }
        }


        /// <summary>
        /// Returns all body resources to the pool used to create them.
        /// </summary>
        /// <remarks>The object can be reused if it is reinitialized by using EnsureCapacity or Resize.</remarks>
        public void Dispose()
        {
            for (int i = 0; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (set.Allocated)
                {
                    set.Dispose(pool);
                }
            }
            pool.SpecializeFor<BodySet>().Return(ref Sets);
            pool.SpecializeFor<BodyInertia>().Return(ref Inertias);
            pool.SpecializeFor<BodyLocation>().Return(ref HandleToLocation);
            HandlePool.Dispose(pool.SpecializeFor<int>());
        }

    }
}
