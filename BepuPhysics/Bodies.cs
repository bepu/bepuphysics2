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

        //TODO: Having Inertias publicly exposed seems like a recipe for confusion, given its ephemeral nature. We may want to explicitly delete it after frame execution and
        //never expose it. If the user really wants an up to date world space inertia, it's pretty easy for them to build it from the local inertia and orientation anyway.
        /// <summary>
        /// The world transformed inertias of active bodies as of the last update. Note that this is not automatically updated for direct orientation changes or for body memory moves.
        /// It is only updated once during the frame. It should be treated as ephemeral information.
        /// </summary>
        public Buffer<BodyInertia> Inertias;
        protected internal BufferPool pool;

        protected internal IslandActivator activator;
        protected internal Shapes shapes;
        protected internal BroadPhase broadPhase;
        protected internal Solver solver;

        /// <summary>
        /// Gets or sets the minimum constraint capacity for each body. Future resizes or allocations will obey this minimum, but changing this does not immediately resize existing lists.
        /// </summary>
        public int MinimumConstraintCapacityPerBody { get; set; }


        /// <summary>
        /// Constructs a new bodies collection. Initialize must be called for the instance to be ready for use.
        /// </summary>
        /// <param name="pool">Pool for the collection to pull persistent allocations from.</param>
        /// <param name="shapes">Shapes referenced by the collection's bodies.</param>
        /// <param name="broadPhase">Broad phase containing the body collidables.</param>
        /// <param name="initialBodyCapacity">Initial number of bodies to allocate space for in the active set.</param>
        /// <param name="initialIslandCapacity">Initial number of islands to allocate space for in the Sets buffer.</param>
        /// <param name="initialConstraintCapacityPerBody">Expected number of constraint references per body to allocate space for.</param>
        public unsafe Bodies(BufferPool pool, Shapes shapes, BroadPhase broadPhase,
            int initialBodyCapacity, int initialIslandCapacity, int initialConstraintCapacityPerBody)
        {
            this.pool = pool;

            //Note that the id pool only grows upon removal, so this is just a heuristic initialization.
            //You could get by with something a lot less aggressive, but it does tend to avoid resizes in the case of extreme churn.
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), initialBodyCapacity, out HandlePool);
            ResizeHandles(initialBodyCapacity);
            ResizeSetsCapacity(initialIslandCapacity + 1, 0);
            ActiveSet = new BodySet(initialBodyCapacity, pool);
            this.shapes = shapes;
            this.broadPhase = broadPhase;
            MinimumConstraintCapacityPerBody = initialConstraintCapacityPerBody;
        }

        /// <summary>
        /// Initializes the bodies set. Used to complete bidirectional dependencies.
        /// </summary>
        /// <param name="solver">Solver responsible for the constraints connected to the collection's bodies.</param>
        /// <param name="activator">Island activator to use when bodies undergo transitions requiring that they exist in the active set.</param>
        public void Initialize(Solver solver, IslandActivator activator)
        {
            this.solver = solver;
            this.activator = activator;
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
        void RemoveCollidableFromBroadPhase(int activeBodyIndex, ref Collidable collidable)
        {
            var removedBroadPhaseIndex = collidable.BroadPhaseIndex;
            //The below removes a body's collidable from the broad phase and adjusts the broad phase index of any moved leaf.
            if (broadPhase.RemoveActiveAt(removedBroadPhaseIndex, out var movedLeaf))
            {
                //Note that this is always an active body, so we know that whatever takes the body's place in the broad phase is also an active body.
                //All statics and inactive bodies exist in the static tree.
                Debug.Assert(movedLeaf.Mobility != CollidableMobility.Static);
                UpdateCollidableBroadPhaseIndex(movedLeaf.Handle, removedBroadPhaseIndex);
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
            var index = ActiveSet.Add(ref description, handle, MinimumConstraintCapacityPerBody, pool);
            HandleToLocation[handle] = new BodyLocation { SetIndex = 0, Index = index };

            if (description.Collidable.Shape.Exists)
            {
                AddCollidableToBroadPhase(handle, ref description.Pose, ref description.LocalInertia, ref ActiveSet.Collidables[index]);
            }
            return handle;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void UpdateAttachedConstraintsForBodyMemoryMove(int originalBodyIndex, int newBodyIndex)
        {
            ref var list = ref ActiveSet.Constraints[originalBodyIndex];
            for (int i = 0; i < list.Count; ++i)
            {
                ref var constraint = ref list[i];
                solver.UpdateForBodyMemoryMove(constraint.ConnectingConstraintHandle, constraint.BodyIndexInConstraint, newBodyIndex);
            }
        }

        internal int RemoveFromActiveSet(int activeBodyIndex)
        {
            //Note that this is separated from the main removal because of deactivation. Deactivation doesn't want to truly remove from the *simulation*, just the active set.
            //The constraints, and references to the constraints, are left untouched.
            ref var set = ref ActiveSet;
            Debug.Assert(activeBodyIndex >= 0 && activeBodyIndex < set.Count);
            ValidateExistingHandle(set.IndexToHandle[activeBodyIndex]);
            ref var collidable = ref set.Collidables[activeBodyIndex];
            if (collidable.Shape.Exists)
            {
                //The collidable exists, so it should be removed from the broadphase.
                //This is true even when this function is used in the context of a deactivation. The collidable will be readded to the inactive tree.
                RemoveCollidableFromBroadPhase(activeBodyIndex, ref collidable);
            }

            var bodyMoved = set.RemoveAt(activeBodyIndex, pool, out var handle, out var movedBodyIndex, out var movedBodyHandle);
            if (bodyMoved)
            {
                //While the removed body doesn't have any constraints associated with it, the body that gets moved to fill its slot might!
                UpdateAttachedConstraintsForBodyMemoryMove(movedBodyIndex, activeBodyIndex);
                Debug.Assert(HandleToLocation[movedBodyHandle].SetIndex == 0 && HandleToLocation[movedBodyHandle].Index == movedBodyIndex);
                HandleToLocation[movedBodyHandle].Index = activeBodyIndex;
            }
            return handle;

        }
        /// <summary>
        /// Removes an active body by its index. Any constraints connected to this body will be removed. Assumes that the input location is valid.
        /// </summary>
        /// <param name="activeBodyIndex">Index of the active body.</param>
        public void RemoveAt(int activeBodyIndex)
        {
            //Constraints must be removed; we cannot leave 'orphans' in the solver because they will access invalid data.
            ref var constraints = ref ActiveSet.Constraints[activeBodyIndex];
            for (int i = constraints.Count - 1; i >= 0; --i)
            {
                solver.Remove(constraints[i].ConnectingConstraintHandle);
            }
            constraints.Dispose(pool.SpecializeFor<BodyConstraintReference>());

            var handle = RemoveFromActiveSet(activeBodyIndex);

            HandlePool.Return(handle, pool.SpecializeFor<int>());
            ref var removedBodyLocation = ref HandleToLocation[handle];
            removedBodyLocation.SetIndex = -1;
            removedBodyLocation.Index = -1;
        }

        /// <summary>
        /// Removes a body from the set by its handle. If the body is inactive, all bodies in its island will be forced active.
        /// </summary>
        /// <param name="handle">Handle of the body to remove.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(int handle)
        {
            ValidateExistingHandle(handle);
            activator.ActivateBody(handle);
            RemoveAt(HandleToLocation[handle].Index);
        }

        /// <summary>
        /// Adds a constraint to an active body's constraint list.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to add the constraint to.</param>
        /// <param name="constraintHandle">Handle of the constraint to add.</param>
        /// <param name="indexInConstraint">Index of the body in the constraint.</param>
        internal void AddConstraint(int bodyIndex, int constraintHandle, int indexInConstraint)
        {
            ActiveSet.AddConstraint(bodyIndex, constraintHandle, indexInConstraint, pool);
        }

        /// <summary>
        /// Removes a constraint from an active body's constraint list.
        /// </summary>
        /// <param name="bodyIndex">Index of the active body.</param>
        /// <param name="constraintHandle">Handle of the constraint to remove.</param>
        internal void RemoveConstraint(int bodyIndex, int constraintHandle)
        {
            ActiveSet.RemoveConstraint(bodyIndex, constraintHandle, MinimumConstraintCapacityPerBody, pool);
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
        void UpdateBroadPhaseKinematicState(int handle, ref BodyLocation location, ref BodySet set)
        {
            Debug.Assert(set.Activity[location.Index].Kinematic == IsKinematic(ref set.LocalInertias[location.Index]),
                "Activity's kinematic state should be updated prior to the broad phase update call. This function simply shares its determination.");
            ref var collidable = ref set.Collidables[location.Index];
            var kinematic = set.Activity[location.Index].Kinematic;
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
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                activator.ActivateBody(handle);
            }
            //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move.
            ref var set = ref Sets[location.SetIndex];
            set.LocalInertias[location.Index] = inertia;
            set.Activity[location.Index].Kinematic = IsKinematic(ref inertia);
            UpdateBroadPhaseKinematicState(handle, ref location, ref set);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateForShapeChange(int handle, int activeBodyIndex, TypedIndex oldShape, TypedIndex newShape)
        {
            if (oldShape.Exists != newShape.Exists)
            {
                ref var set = ref ActiveSet;
                if (newShape.Exists)
                {
                    //Add a collidable to the simulation for the new shape.
                    AddCollidableToBroadPhase(handle, ref set.Poses[activeBodyIndex], ref set.LocalInertias[activeBodyIndex], ref set.Collidables[activeBodyIndex]);
                }
                else
                {
                    //Remove the now-unused collidable from the simulation.
                    RemoveCollidableFromBroadPhase(activeBodyIndex, ref set.Collidables[activeBodyIndex]);
                }
            }
        }
        /// <summary>
        /// Changes the shape of a body. Properly handles the transition between shapeless and shapeful. If the body is inactive, it will be forced awake.
        /// </summary>
        /// <param name="handle">Handle of the body to change the shape of.</param>
        /// <param name="newShape">Index of the new shape to use for the body.</param>
        public void ChangeShape(int handle, TypedIndex newShape)
        {
            ref var location = ref HandleToLocation[handle];
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                activator.ActivateBody(handle);
            }
            //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move.
            Debug.Assert(location.SetIndex == 0, "We should be working with an active shape.");
            ref var set = ref ActiveSet;
            ref var collidable = ref set.Collidables[location.Index];
            var oldShape = collidable.Shape;
            collidable.Shape = newShape;
            UpdateForShapeChange(handle, location.Index, oldShape, newShape);
        }

        /// <summary>
        /// Applies a description to a body. Properly handles any transitions between dynamic and kinematic and between shapeless and shapeful. If the body is inactive, it will be forced awake.
        /// </summary>
        /// <param name="handle">Handle of the body to receive the description.</param>
        /// <param name="description">Description to apply to the body.</param>
        public void ApplyDescription(int handle, ref BodyDescription description)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle];
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                activator.ActivateBody(handle);
            }
            //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move.
            ref var set = ref Sets[location.SetIndex];
            ref var collidable = ref set.Collidables[location.Index];
            var oldShape = collidable.Shape;
            set.ApplyDescriptionByIndex(location.Index, ref description);
            UpdateForShapeChange(handle, location.Index, oldShape, description.Collidable.Shape);
            UpdateBroadPhaseKinematicState(handle, ref location, ref set);
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
                var hiPleaseDontRemoveThisWithoutTestingOrEverythingMightExplode = stackalloc byte[1];
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

        internal void ResizeSetsCapacity(int setsCapacity, int potentiallyAllocatedCount)
        {
            Debug.Assert(setsCapacity >= potentiallyAllocatedCount && potentiallyAllocatedCount <= Sets.Length);
            setsCapacity = BufferPool<BodySet>.GetLowestContainingElementCount(setsCapacity);
            if (Sets.Length != setsCapacity)
            {
                var oldCapacity = Sets.Length;
                pool.SpecializeFor<BodySet>().Resize(ref Sets, setsCapacity, potentiallyAllocatedCount);
                if (oldCapacity < Sets.Length)
                    Sets.Clear(oldCapacity, Sets.Length - oldCapacity); //We rely on unused slots being default initialized.
            }
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

        struct ActiveConstraintBodyIndicesEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEach<int>
        {
            public TInnerEnumerator InnerEnumerator;
            public int SourceBodyIndex;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                if (SourceBodyIndex != connectedBodyIndex)
                {
                    //Note that this may report the same body multiple times if it is connected multiple times! That's fine and potentially useful; let the user deal with it.
                    InnerEnumerator.LoopBody(connectedBodyIndex);
                }
            }

        }
        struct ActiveConstraintBodyHandleEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEach<int>
        {
            public Bodies bodies;
            public TInnerEnumerator InnerEnumerator;
            public int SourceBodyIndex;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                if (SourceBodyIndex != connectedBodyIndex)
                {
                    //This enumerator is associated with the public connected bodies enumerator function. The user supplies a handle and expects handles in return, so we 
                    //must convert the solver-provided indices to handles.
                    InnerEnumerator.LoopBody(bodies.ActiveSet.IndexToHandle[connectedBodyIndex]);
                }
            }

        }
        //Note that inactive constraints reference bodies by handles rather than indices.
        struct InactiveConstraintBodyHandleEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEach<int>
        {
            public TInnerEnumerator InnerEnumerator;
            public int SourceBodyHandle;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyHandle)
            {
                if (SourceBodyHandle != connectedBodyHandle)
                {
                    //Since this enumerator is associated with inactive constraints, which directly store body handles rather than body indices, 
                    //we can directly pass the solver-provided handle.
                    InnerEnumerator.LoopBody(connectedBodyHandle);
                }
            }

        }

        /// <summary>
        /// Enumerates all the bodies connected to a given active body.
        /// Bodies which are connected by more than one constraint will be reported multiple times.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to execute on each connected body.</typeparam>
        /// <param name="activeBodyIndex">Index of the active body to enumerate the connections of. This body will not appear in the set of enumerated bodies, even if it is connected to itself somehow.</param>
        /// <param name="enumerator">Enumerator instance to run on each connected body.</param>
        /// <param name="solver">Solver from which to pull constraint body references.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void EnumerateConnectedBodyIndices<TEnumerator>(int activeBodyIndex, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var list = ref ActiveSet.Constraints[activeBodyIndex];
            ActiveConstraintBodyIndicesEnumerator<TEnumerator> constraintBodiesEnumerator;
            constraintBodiesEnumerator.InnerEnumerator = enumerator;
            constraintBodiesEnumerator.SourceBodyIndex = activeBodyIndex;

            //Note reverse iteration. This is useful when performing O(1) removals where the last element is put into the position of the removed element.
            //Non-reversed iteration would result in skipped elements if the loop body removed anything. This relies on convention; any remover should be aware of this order.
            for (int i = list.Count - 1; i >= 0; --i)
            {
                solver.EnumerateConnectedBodies(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
            }
            //Note that we have to assume the enumerator contains state mutated by the internal loop bodies.
            //If it's a value type, those mutations won't be reflected in the original reference. 
            //Copy them back in.
            enumerator = constraintBodiesEnumerator.InnerEnumerator;
        }
        /// <summary>
        /// Enumerates all the bodies connected to a given body.
        /// Bodies which are connected by more than one constraint will be reported multiple times.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to execute on each connected body.</typeparam>
        /// <param name="bodyHandle">Handle of the body to enumerate the connections of. This body will not appear in the set of enumerated bodies, even if it is connected to itself somehow.</param>
        /// <param name="enumerator">Enumerator instance to run on each connected body.</param>
        /// <param name="solver">Solver from which to pull constraint body references.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnumerateConnectedBodies<TEnumerator>(int bodyHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var bodyLocation = ref HandleToLocation[bodyHandle];
            ref var set = ref Sets[bodyLocation.SetIndex];
            ref var list = ref set.Constraints[bodyLocation.Index];
            //In the loops below, we still make use of the reversed iteration. Removing from within the context of an enumerator is a dangerous move, but it is permitted if the user 
            //is careful. By maintaining the same convention across all of these enumerations, it makes it a little easier to do reliably.
            if (bodyLocation.SetIndex == 0)
            {
                //The body is active. Use the active enumerator.
                ActiveConstraintBodyHandleEnumerator<TEnumerator> constraintBodiesEnumerator;
                constraintBodiesEnumerator.InnerEnumerator = enumerator;
                constraintBodiesEnumerator.SourceBodyIndex = bodyHandle;
                constraintBodiesEnumerator.bodies = this;

                for (int i = list.Count - 1; i >= 0; --i)
                {
                    solver.EnumerateConnectedBodies(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
                }
                enumerator = constraintBodiesEnumerator.InnerEnumerator;
            }
            else
            {
                //The body is inactive. Use the inactive enumerator.
                InactiveConstraintBodyHandleEnumerator<TEnumerator> constraintBodiesEnumerator;
                constraintBodiesEnumerator.InnerEnumerator = enumerator;
                constraintBodiesEnumerator.SourceBodyHandle = bodyHandle;

                for (int i = list.Count - 1; i >= 0; --i)
                {
                    solver.EnumerateConnectedBodies(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
                }
                enumerator = constraintBodiesEnumerator.InnerEnumerator;
            }

        }
        /// <summary>
        /// Clears all bodies from all sets without releasing any memory that wouldn't be released by a sequence of regular removals.
        /// </summary>
        public unsafe void Clear()
        {
            ActiveSet.Clear(pool);
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
                var oldCapacity = HandleToLocation.Length;
                pool.SpecializeFor<BodyLocation>().Resize(ref HandleToLocation, newCapacity, Math.Min(oldCapacity, newCapacity));
                if (HandleToLocation.Length > oldCapacity)
                {
                    Unsafe.InitBlockUnaligned(
                      ((BodyLocation*)HandleToLocation.Memory) + oldCapacity, 0xFF,
                      (uint)(sizeof(BodyLocation) * (HandleToLocation.Length - oldCapacity)));
                }
            }
        }
        //Note that these resize and ensure capacity functions affect only the active set.
        //Inactive islands are created with minimal allocations. Since you cannot add to or remove from inactive islands, it is pointless to try to modify their allocation sizes.
        /// <summary>
        /// Reallocates the inertias buffer for the target capacity. Will not shrink below the size of the current active set.
        /// </summary>
        internal void ResizeInertias(int capacity)
        {
            var targetCapacity = BufferPool<BodyInertia>.GetLowestContainingElementCount(Math.Max(capacity, ActiveSet.Count));
            if (Inertias.Length != targetCapacity)
            {
                pool.SpecializeFor<BodyInertia>().Resize(ref Inertias, targetCapacity, Math.Min(Inertias.Length, ActiveSet.Count));
            }
        }
        /// <summary>
        /// Guarantees that the inertias capacity is sufficient for the given capacity.
        /// </summary>
        internal void EnsureInertiasCapacity(int capacity)
        {
            if (capacity < ActiveSet.Count)
                capacity = ActiveSet.Count;
            if (Inertias.Length < capacity)
            {
                pool.SpecializeFor<BodyInertia>().Resize(ref Inertias, capacity, Math.Min(Inertias.Length, ActiveSet.Count));
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
            ResizeInertias(capacity);
            var targetHandleCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(capacity, HandlePool.HighestPossiblyClaimedId + 1));
            if (HandleToLocation.Length != targetHandleCapacity)
            {
                ResizeHandles(targetHandleCapacity);
            }
        }

        /// <summary>
        /// Resizes all active body constraint lists to meet the MinimumConstraintCapacityPerBody. Inactive bodies are untouched.
        /// Resizes are guaranteed to never shrink a list below the current count.
        /// </summary>
        public void ResizeConstraintListCapacities()
        {
            var bodyReferencePool = pool.SpecializeFor<BodyConstraintReference>();
            for (int i = 0; i < ActiveSet.Count; ++i)
            {
                ref var list = ref ActiveSet.Constraints[i];
                var targetCapacity = BufferPool<BodyConstraintReference>.GetLowestContainingElementCount(list.Count > MinimumConstraintCapacityPerBody ? list.Count : MinimumConstraintCapacityPerBody);
                if (list.Span.Length != targetCapacity)
                    list.Resize(targetCapacity, bodyReferencePool);
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
            EnsureInertiasCapacity(capacity);
            if (HandleToLocation.Length < capacity)
            {
                ResizeHandles(capacity);
            }
        }

        /// <summary>
        /// Ensures all active body constraint lists can hold at least MinimumConstraintCapacityPerBody constraints. Inactive bodies are untouched.
        /// </summary>
        public void EnsureConstraintListCapacities()
        {
            var bodyReferencePool = pool.SpecializeFor<BodyConstraintReference>();
            for (int i = 0; i < ActiveSet.Count; ++i)
            {
                ref var list = ref ActiveSet.Constraints[i];
                if (list.Span.Length < MinimumConstraintCapacityPerBody)
                    list.Resize(MinimumConstraintCapacityPerBody, bodyReferencePool);
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
            if (Inertias.Allocated)
                pool.SpecializeFor<BodyInertia>().Return(ref Inertias);
            pool.SpecializeFor<BodyLocation>().Return(ref HandleToLocation);
            HandlePool.Dispose(pool.SpecializeFor<int>());
        }

    }
}
