using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;

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
        public IdPool HandlePool;
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
        public BufferPool Pool { get; private set; }

        internal IslandAwakener awakener;
        internal IslandSleeper sleeper;
        internal Shapes shapes;
        internal BroadPhase broadPhase;
        internal Solver solver;

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
            this.Pool = pool;

            //Note that the id pool only grows upon removal, so this is just a heuristic initialization.
            //You could get by with something a lot less aggressive, but it does tend to avoid resizes in the case of extreme churn.
            HandlePool = new IdPool(initialBodyCapacity, pool);
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
        /// <param name="awakener">Island awakener to use when bodies undergo transitions requiring that they exist in the active set.</param>
        public void Initialize(Solver solver, IslandAwakener awakener, IslandSleeper sleeper)
        {
            this.solver = solver;
            this.awakener = awakener;
            this.sleeper = sleeper;
        }

        void AddCollidableToBroadPhase(int bodyHandle, in RigidPose pose, in BodyInertia localInertia, ref Collidable collidable)
        {
            Debug.Assert(collidable.Shape.Exists);
            //This body has a collidable; stick it in the broadphase.
            //Note that we have to calculate an initial bounding box for the broad phase to be able to insert it efficiently.
            //(In the event of batch adds, you'll want to use batched AABB calculations or just use cached values.)
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            shapes.UpdateBounds(pose, ref collidable.Shape, out var bodyBounds);
            //Note that new body collidables are always assumed to be active.
            collidable.BroadPhaseIndex =
                broadPhase.AddActive(
                    new CollidableReference(IsKinematic(localInertia) ? CollidableMobility.Kinematic : CollidableMobility.Dynamic, bodyHandle),
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
            Debug.Assert(collidable.Shape.Exists && collidable.BroadPhaseIndex >= 0);
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
        public unsafe int Add(in BodyDescription description)
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
            var index = ActiveSet.Add(description, handle, MinimumConstraintCapacityPerBody, Pool);
            HandleToLocation[handle] = new BodyLocation { SetIndex = 0, Index = index };

            if (description.Collidable.Shape.Exists)
            {
                AddCollidableToBroadPhase(handle, description.Pose, description.LocalInertia, ref ActiveSet.Collidables[index]);
            }
            return handle;
        }

        internal int RemoveFromActiveSet(int activeBodyIndex)
        {
            //Note that this is separated from the main removal because of sleeping. Sleeping doesn't want to truly remove from the *simulation*, just the active set.
            //The constraints, and references to the constraints, are left untouched.
            ref var set = ref ActiveSet;
            Debug.Assert(activeBodyIndex >= 0 && activeBodyIndex < set.Count);
            ValidateExistingHandle(set.IndexToHandle[activeBodyIndex]);
            ref var collidable = ref set.Collidables[activeBodyIndex];
            if (collidable.Shape.Exists)
            {
                //The collidable exists, so it should be removed from the broadphase.
                //This is true even when this function is used in the context of a sleep. The collidable will be readded to the static tree.
                RemoveCollidableFromBroadPhase(activeBodyIndex, ref collidable);
            }

            var bodyMoved = set.RemoveAt(activeBodyIndex, Pool, out var handle, out var movedBodyIndex, out var movedBodyHandle);
            if (bodyMoved)
            {
                //While the removed body doesn't have any constraints associated with it, the body that gets moved to fill its slot might!
                solver.UpdateForBodyMemoryMove(movedBodyIndex, activeBodyIndex);
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
            constraints.Dispose(Pool);

            var handle = RemoveFromActiveSet(activeBodyIndex);

            HandlePool.Return(handle, Pool);
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
            awakener.AwakenBody(handle);
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
            ActiveSet.AddConstraint(bodyIndex, constraintHandle, indexInConstraint, Pool);
        }

        /// <summary>
        /// Removes a constraint from an active body's constraint list.
        /// </summary>
        /// <param name="bodyIndex">Index of the active body.</param>
        /// <param name="constraintHandle">Handle of the constraint to remove.</param>
        internal void RemoveConstraintReference(int bodyIndex, int constraintHandle)
        {
            ActiveSet.RemoveConstraintReference(bodyIndex, constraintHandle, MinimumConstraintCapacityPerBody, Pool);
        }

        /// <summary>
        /// Gets whether the inertia matches that of a kinematic body (that is, all inverse mass and inertia components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsKinematic(in BodyInertia inertia)
        {
            return inertia.InverseMass == 0 && HasLockedInertia(inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Gets whether the angular inertia matches that of a kinematic body (that is, all inverse inertia tensor components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool HasLockedInertia(in Symmetric3x3 inertia)
        {
            return inertia.XX == 0 &&
                   inertia.YX == 0 &&
                   inertia.YY == 0 &&
                   inertia.ZX == 0 &&
                   inertia.ZY == 0 &&
                   inertia.ZZ == 0;
        }

        private struct ConnectedDynamicCounter : IForEach<int>
        {
            public Bodies Bodies;
            public int DynamicCount;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int bodyIndex)
            {
                //The solver's connected bodies enumeration directly provides the constraint-stored reference, which is an index in the active set for active constraints and a handle for inactive constraints.
                //We forced the dynamic active at the beginning of BecomeKinematic, so we don't have to worry about the inactive side of things.
                if (!Bodies.IsKinematic(Bodies.ActiveSet.LocalInertias[bodyIndex]))
                    ++DynamicCount;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateForKinematicStateChange(int handle, ref BodyLocation location, ref BodySet set, bool newlyKinematic)
        {
            Debug.Assert(location.SetIndex == 0, "If we're changing kinematic state, we should have already awoken the body.");
            ref var collidable = ref set.Collidables[location.Index];
            if (collidable.Shape.Exists)
            {
                var mobility = IsKinematic(set.LocalInertias[location.Index]) ? CollidableMobility.Kinematic : CollidableMobility.Dynamic;
                if (location.SetIndex == 0)
                {
                    broadPhase.activeLeaves[collidable.BroadPhaseIndex] = new CollidableReference(mobility, handle);
                }
                else
                {
                    broadPhase.staticLeaves[collidable.BroadPhaseIndex] = new CollidableReference(mobility, handle);
                }
            }
            if(newlyKinematic)
            {
                ref var constraints = ref set.Constraints[location.Index];
                ConnectedDynamicCounter enumerator;
                enumerator.Bodies = this;
                for (int i = 0; i < constraints.Count; ++i)
                {
                    ref var constraint = ref constraints[i];
                    enumerator.DynamicCount = 0;
                    solver.EnumerateConnectedBodies(constraint.ConnectingConstraintHandle, ref enumerator);
                    if (enumerator.DynamicCount == 0)
                    {
                        //This constraint connects only kinematic bodies; keeping it in the solver would cause a singularity.
                        solver.Remove(constraint.ConnectingConstraintHandle);
                    }
                }
            }
        }

        /// <summary>
        /// Changes the local mass and inertia tensor associated with a body. Properly handles the transition between kinematic and dynamic.
        /// If the body is becoming kinematic, any constraints which only contain kinematic bodies will be removed.
        /// Wakes up the body.
        /// </summary>
        /// <param name="handle">Handle of the body whose inertia should change.</param>
        /// <param name="localInertia">New local inertia for the body.</param>
        /// <remarks>
        /// This function is only necessary when the inertia change could potentially result in a transition between dynamic and kinematic states.
        /// If it is guaranteed to be dynamic before and after the change, the inertia can be directly modified without issue.
        /// </remarks>
        public void SetLocalInertia(int handle, in BodyInertia localInertia)
        {
            ref var location = ref HandleToLocation[handle];
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                awakener.AwakenBody(handle);
            }
            //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move.
            ref var set = ref Sets[location.SetIndex];
            var newlyKinematic = IsKinematic(localInertia) && !IsKinematic(set.LocalInertias[location.Index]);
            set.LocalInertias[location.Index] = localInertia;
            UpdateForKinematicStateChange(handle, ref location, ref set, newlyKinematic);
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
                    AddCollidableToBroadPhase(handle, set.Poses[activeBodyIndex], set.LocalInertias[activeBodyIndex], ref set.Collidables[activeBodyIndex]);
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
        public void SetShape(int handle, TypedIndex newShape)
        {
            ref var location = ref HandleToLocation[handle];
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                awakener.AwakenBody(handle);
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
        /// Applies a description to a body. Properly handles any transitions between dynamic and kinematic and between shapeless and shapeful.
        /// If the body is becoming kinematic, any constraints which only contain kinematic bodies will be removed. Wakes up the body.
        /// </summary>
        /// <param name="handle">Handle of the body to receive the description.</param>
        /// <param name="description">Description to apply to the body.</param>
        public void ApplyDescription(int handle, in BodyDescription description)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle];
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                awakener.AwakenBody(handle);
            }
            //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move.
            ref var set = ref Sets[location.SetIndex];
            ref var collidable = ref set.Collidables[location.Index];
            var oldShape = collidable.Shape;
            var newlyKinematic = IsKinematic(description.LocalInertia) && !IsKinematic(set.LocalInertias[location.Index]);
            set.ApplyDescriptionByIndex(location.Index, description);
            UpdateForShapeChange(handle, location.Index, oldShape, description.Collidable.Shape);
            UpdateForKinematicStateChange(handle, ref location, ref set, newlyKinematic);
        }

        /// <summary>
        /// Gets the description of a body by handle.
        /// </summary>
        /// <param name="handle">Handle of the body to look up.</param>
        /// <param name="description">Description of the body.</param>
        public void GetDescription(int handle, out BodyDescription description)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle];
            ref var set = ref Sets[location.SetIndex];
            set.GetDescription(location.Index, out description);
        }

        /// <summary>
        /// Gets a reference to a body by its handle.
        /// </summary>
        /// <param name="handle">Handle of the body to grab a reference of.</param>
        /// <returns>Reference to the desired body.</returns>
        public BodyReference GetBodyReference(int handle)
        {
            ValidateExistingHandle(handle);
            return new BodyReference(handle, this);
        }

        /// <summary>
        /// Checks whether a body handle is currently registered with the bodies set.
        /// </summary>
        /// <param name="bodyHandle">Handle to check for.</param>
        /// <returns>True if the handle exists in the collection, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool BodyExists(int bodyHandle)
        {
            //A negative set index marks a body handle as unused.
            return bodyHandle >= 0 && bodyHandle < HandleToLocation.Length && HandleToLocation[bodyHandle].SetIndex >= 0;
        }

        [Conditional("DEBUG")]
        internal void ValidateExistingHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle <= HandlePool.HighestPossiblyClaimedId && HandlePool.HighestPossiblyClaimedId < HandleToLocation.Length,
                "Existing handles must fit within the body handle->index mapping.");
            ref var location = ref HandleToLocation[handle];
            Debug.Assert(location.SetIndex >= 0 && location.SetIndex < Sets.Length, "Body set index must be nonnegative and within the sets buffer length.");
            ref var set = ref Sets[location.SetIndex];
            Debug.Assert(set.Allocated);
            Debug.Assert(set.Count <= set.IndexToHandle.Length);
            Debug.Assert(location.Index >= 0 && location.Index < set.Count, "Body index must fall within the existing body set.");
            Debug.Assert(set.IndexToHandle[location.Index] == handle, "Handle->index must match index->handle map.");
            Debug.Assert(BodyExists(handle), "Body must exist according to the BodyExists test.");
        }

        [Conditional("CHECKMATH")]
        internal void ValidateMotionStates()
        {
            for (int i = 0; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (set.Allocated)
                {
                    for (int j = 0; j < set.Count; ++j)
                    {
                        ref var pose = ref set.Poses[j];
                        ref var velocity = ref set.Velocities[j];
                        try
                        {
                            pose.Position.Validate();
                            pose.Orientation.Validate();
                            velocity.Linear.Validate();
                            velocity.Angular.Validate();
                        }
                        catch
                        {
                            Console.WriteLine($"Validation failed on body {i} of set {j}. Position: {pose.Position}, orientation: {pose.Orientation}, linear: {velocity.Linear}, angular: {velocity.Angular}");
                            throw;
                        }

                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void GatherInertiaForBody(ref BodyInertia source, ref BodyInertias targetSlot)
        {
            GatherScatter.GetFirst(ref targetSlot.InverseInertiaTensor.XX) = source.InverseInertiaTensor.XX;
            GatherScatter.GetFirst(ref targetSlot.InverseInertiaTensor.YX) = source.InverseInertiaTensor.YX;
            GatherScatter.GetFirst(ref targetSlot.InverseInertiaTensor.YY) = source.InverseInertiaTensor.YY;
            GatherScatter.GetFirst(ref targetSlot.InverseInertiaTensor.ZX) = source.InverseInertiaTensor.ZX;
            GatherScatter.GetFirst(ref targetSlot.InverseInertiaTensor.ZY) = source.InverseInertiaTensor.ZY;
            GatherScatter.GetFirst(ref targetSlot.InverseInertiaTensor.ZZ) = source.InverseInertiaTensor.ZZ;
            GatherScatter.GetFirst(ref targetSlot.InverseMass) = source.InverseMass;
        }

        /// <summary>
        /// Gathers inertia for one body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of bodies in the bundle.</param>
        /// <param name="inertiaA">Gathered inertia of body A.</param>
        /// <param name="inertiaB">Gathered inertia of body B.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherInertia(ref Vector<int> references, int count,
            out BodyInertias inertiaA)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references);
            for (int i = 0; i < count; ++i)
            {
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexA, i)], ref GatherScatter.GetOffsetInstance(ref inertiaA, i));
            }
        }

        /// <summary>
        /// Gathers inertia for two body bundles into AOSOA bundles.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of bodies in the bundle.</param>
        /// <param name="inertiaA">Gathered inertia of body A.</param>
        /// <param name="inertiaB">Gathered inertia of body B.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherInertia(ref TwoBodyReferences references, int count, out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            for (int i = 0; i < count; ++i)
            {
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexA, i)], ref GatherScatter.GetOffsetInstance(ref inertiaA, i));
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexB, i)], ref GatherScatter.GetOffsetInstance(ref inertiaB, i));
            }
        }

        /// <summary>
        /// Gathers inertia for three body bundles into AOSOA bundles.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of bodies in the bundle.</param>
        /// <param name="inertiaA">Gathered inertia of body A.</param>
        /// <param name="inertiaB">Gathered inertia of body B.</param>
        /// <param name="inertiaC">Gathered inertia of body C.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherInertia(ref ThreeBodyReferences references, int count, out BodyInertias inertiaA, out BodyInertias inertiaB, out BodyInertias inertiaC)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            for (int i = 0; i < count; ++i)
            {
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexA, i)], ref GatherScatter.GetOffsetInstance(ref inertiaA, i));
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexB, i)], ref GatherScatter.GetOffsetInstance(ref inertiaB, i));
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexC, i)], ref GatherScatter.GetOffsetInstance(ref inertiaC, i));
            }
        }

        /// <summary>
        /// Gathers inertia for four body bundles into AOSOA bundles.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of bodies in the bundle.</param>
        /// <param name="inertiaA">Gathered inertia of body A.</param>
        /// <param name="inertiaB">Gathered inertia of body B.</param>
        /// <param name="inertiaC">Gathered inertia of body C.</param>
        /// <param name="inertiaD">Gathered inertia of body D.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherInertia(ref FourBodyReferences references, int count, out BodyInertias inertiaA, out BodyInertias inertiaB, out BodyInertias inertiaC, out BodyInertias inertiaD)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);
            for (int i = 0; i < count; ++i)
            {
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexA, i)], ref GatherScatter.GetOffsetInstance(ref inertiaA, i));
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexB, i)], ref GatherScatter.GetOffsetInstance(ref inertiaB, i));
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexC, i)], ref GatherScatter.GetOffsetInstance(ref inertiaC, i));
                GatherInertiaForBody(ref Inertias[Unsafe.Add(ref baseIndexD, i)], ref GatherScatter.GetOffsetInstance(ref inertiaD, i));
            }
        }

        /// <summary>
        /// Gathers orientations for two body bundles into AOSOA bundles.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="orientationA">Gathered orientation of body A.</param>
        /// <param name="orientationB">Gathered orientation of body B.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherOrientation(ref TwoBodyReferences references, int count,
            out QuaternionWide orientationA, out QuaternionWide orientationB)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);

            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                QuaternionWide.WriteFirst(poses[indexA].Orientation, ref GatherScatter.GetOffsetInstance(ref orientationA, i));

                ref var indexB = ref Unsafe.Add(ref baseIndexB, i);
                QuaternionWide.WriteFirst(poses[indexB].Orientation, ref GatherScatter.GetOffsetInstance(ref orientationB, i));
            }
        }

        /// <summary>
        /// Gathers orientations for one body bundles into AOSOA bundles.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="orientation">Gathered orientation of bodies in the bundle.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherOrientation(ref Vector<int> references, int count,
            out QuaternionWide orientation)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references);

            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                QuaternionWide.WriteFirst(poses[indexA].Orientation, ref GatherScatter.GetOffsetInstance(ref orientation, i));
            }
        }



        /// <summary>
        /// Gathers pose information for a body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="position">Gathered absolute position of the body.</param>
        /// <param name="orientation">Gathered orientation of the body.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherPose(ref Vector<int> references, int count, out Vector3Wide position, out QuaternionWide orientation)
        {
            //TODO: This function and its users (which should be relatively few) is a problem for large world position precision.
            //It directly reports the position, thereby infecting vectorized logic with the high precision representation.
            //You might be able to redesign the users of this function to not need it, but that comes with its own difficulties
            //(for example, making the grab motor rely on having its goal offset updated every frame by the user).
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndex = ref Unsafe.As<Vector<int>, int>(ref references);

            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndex, i);
                ref var poseA = ref poses[indexA];
                Vector3Wide.WriteFirst(poseA.Position, ref GatherScatter.GetOffsetInstance(ref position, i));
                QuaternionWide.WriteFirst(poseA.Orientation, ref GatherScatter.GetOffsetInstance(ref orientation, i));

            }
        }

        /// <summary>
        /// Gathers orientations and relative positions for a two body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="offsetB">Gathered offsets from body A to body B.</param>
        /// <param name="orientationA">Gathered orientation of body A.</param>
        /// <param name="orientationB">Gathered orientation of body B.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherPose(ref TwoBodyReferences references, int count,
            out Vector3Wide offsetB, out QuaternionWide orientationA, out QuaternionWide orientationB)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);

            Vector3Wide positionA, positionB;
            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                ref var poseA = ref poses[indexA];
                Vector3Wide.WriteFirst(poseA.Position, ref GatherScatter.GetOffsetInstance(ref positionA, i));
                QuaternionWide.WriteFirst(poseA.Orientation, ref GatherScatter.GetOffsetInstance(ref orientationA, i));

                ref var indexB = ref Unsafe.Add(ref baseIndexB, i);
                ref var poseB = ref poses[indexB];
                Vector3Wide.WriteFirst(poseB.Position, ref GatherScatter.GetOffsetInstance(ref positionB, i));
                QuaternionWide.WriteFirst(poseB.Orientation, ref GatherScatter.GetOffsetInstance(ref orientationB, i));
            }
            //TODO: In future versions, we will likely store the body position in different forms to allow for extremely large worlds.
            //That will be an opt-in feature. The default implementation will use the FP32 representation, but the user could choose to swap it out for a fp64 or fixed64 representation.
            //This affects other systems- AABB calculation, pose integration, solving, and in extreme (64 bit) cases, the broadphase.
            //We want to insulate other systems from direct knowledge about the implementation of positions when possible.
            //These functions support the solver's needs while hiding absolute positions.
            //In order to support other absolute positions, we'll need alternate implementations of this and other functions.
            //But for the most part, we don't want to pay the overhead of an abstract invocation within the inner loop of the solver. 
            //Given the current limits of C# and the compiler, the best option seems to be conditional compilation.
            Vector3Wide.Subtract(positionB, positionA, out offsetB);
        }

        /// <summary>
        /// Gathers relative positions for a two body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="ab">Gathered offset from body A to of body B.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherOffsets(ref TwoBodyReferences references, int count, out Vector3Wide ab)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);

            Vector3Wide positionA, positionB;
            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexA, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionA, i));
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexB, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionB, i));
            }
            //Same as other gather case; this is sensitive to changes in the representation of body position. In high precision modes, this'll need to change.
            Vector3Wide.Subtract(positionB, positionA, out ab);
        }

        /// <summary>
        /// Gathers relative positions for a three body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="ab">Gathered offset from body A to of body B.</param>
        /// <param name="ac">Gathered offset from body A to of body C.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherOffsets(ref ThreeBodyReferences references, int count, out Vector3Wide ab, out Vector3Wide ac)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);

            Vector3Wide positionA, positionB, positionC;
            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexA, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionA, i));
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexB, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionB, i));
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexC, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionC, i));
            }
            //Same as two body case; this is sensitive to changes in the representation of body position. In high precision modes, this'll need to change.
            Vector3Wide.Subtract(positionB, positionA, out ab);
            Vector3Wide.Subtract(positionC, positionA, out ac);
        }
        /// <summary>
        /// Gathers relative positions for a four body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="ab">Gathered offset from body A to of body B.</param>
        /// <param name="ac">Gathered offset from body A to of body C.</param>
        /// <param name="ad">Gathered offset from body A to of body D.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherOffsets(ref FourBodyReferences references, int count, out Vector3Wide ab, out Vector3Wide ac, out Vector3Wide ad)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);

            Vector3Wide positionA, positionB, positionC, positionD;
            ref var poses = ref ActiveSet.Poses;
            for (int i = 0; i < count; ++i)
            {
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexA, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionA, i));
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexB, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionB, i));
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexC, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionC, i));
                Vector3Wide.WriteFirst(poses[Unsafe.Add(ref baseIndexD, i)].Position, ref GatherScatter.GetOffsetInstance(ref positionD, i));
            }
            //Same as two body case; this is sensitive to changes in the representation of body position. In high precision modes, this'll need to change.
            Vector3Wide.Subtract(positionB, positionA, out ab);
            Vector3Wide.Subtract(positionC, positionA, out ac);
            Vector3Wide.Subtract(positionD, positionA, out ad);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void GatherVelocities(ref Buffer<BodyVelocity> sources, ref BodyVelocities target, ref int baseIndex, int innerIndex)
        {
            ref var targetSlot = ref GatherScatter.GetOffsetInstance(ref target, innerIndex);
            ref var source = ref sources[Unsafe.Add(ref baseIndex, innerIndex)];
            GatherScatter.GetFirst(ref targetSlot.Linear.X) = source.Linear.X;
            GatherScatter.GetFirst(ref targetSlot.Linear.Y) = source.Linear.Y;
            GatherScatter.GetFirst(ref targetSlot.Linear.Z) = source.Linear.Z;
            GatherScatter.GetFirst(ref targetSlot.Angular.X) = source.Angular.X;
            GatherScatter.GetFirst(ref targetSlot.Angular.Y) = source.Angular.Y;
            GatherScatter.GetFirst(ref targetSlot.Angular.Z) = source.Angular.Z;
        }

        /// <summary>
        /// Gathers velocities for one body bundle and stores it into a velocity bundle.
        /// </summary>
        /// <param name="references">Active set indices of the bodies to gather velocity data for.</param>
        /// <param name="count">Number of bodies in the bundle.</param>
        /// <param name="velocities">Gathered velocities.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(ref Buffer<BodyVelocity> sourceVelocities, ref Vector<int> references, int count, out BodyVelocities velocities)
        {
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndex = ref Unsafe.As<Vector<int>, int>(ref references);
            for (int i = 0; i < count; ++i)
            {
                GatherVelocities(ref sourceVelocities, ref velocities, ref baseIndex, i);
            }
        }

        /// <summary>
        /// Gathers velocities for two body bundles and stores it into velocity bundles.
        /// </summary>
        /// <param name="references">Active set indices of the bodies to gather velocity data for.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="velocitiesA">Gathered velocities of A bodies.</param>
        /// <param name="velocitiesB">Gathered velocities of B bodies.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(ref Buffer<BodyVelocity> sourceVelocities, ref TwoBodyReferences references, int count, out BodyVelocities velocitiesA, out BodyVelocities velocitiesB)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            for (int i = 0; i < count; ++i)
            {
                GatherVelocities(ref sourceVelocities, ref velocitiesA, ref baseIndexA, i);
                GatherVelocities(ref sourceVelocities, ref velocitiesB, ref baseIndexB, i);
            }
        }

        /// <summary>
        /// Gathers velocities for three body bundles and stores it into velocity bundles.
        /// </summary>
        /// <param name="references">Active set indices of the bodies to gather velocity data for.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="velocitiesA">Gathered velocities of A bodies.</param>
        /// <param name="velocitiesB">Gathered velocities of B bodies.</param>
        /// <param name="velocitiesC">Gathered velocities of C bodies.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(ref Buffer<BodyVelocity> sourceVelocities, ref ThreeBodyReferences references, int count,
            out BodyVelocities velocitiesA, out BodyVelocities velocitiesB, out BodyVelocities velocitiesC)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            for (int i = 0; i < count; ++i)
            {
                GatherVelocities(ref sourceVelocities, ref velocitiesA, ref baseIndexA, i);
                GatherVelocities(ref sourceVelocities, ref velocitiesB, ref baseIndexB, i);
                GatherVelocities(ref sourceVelocities, ref velocitiesC, ref baseIndexC, i);
            }
        }

        /// <summary>
        /// Gathers velocities for four body bundles and stores it into velocity bundles.
        /// </summary>
        /// <param name="references">Active set indices of the bodies to gather velocity data for.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="velocitiesA">Gathered velocities of A bodies.</param>
        /// <param name="velocitiesB">Gathered velocities of B bodies.</param>
        /// <param name="velocitiesC">Gathered velocities of C bodies.</param>
        /// <param name="velocitiesD">Gathered velocities of D bodies.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(ref Buffer<BodyVelocity> sourceVelocities, ref FourBodyReferences references, int count,
            out BodyVelocities velocitiesA, out BodyVelocities velocitiesB, out BodyVelocities velocitiesC, out BodyVelocities velocitiesD)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);
            for (int i = 0; i < count; ++i)
            {
                GatherVelocities(ref sourceVelocities, ref velocitiesA, ref baseIndexA, i);
                GatherVelocities(ref sourceVelocities, ref velocitiesB, ref baseIndexB, i);
                GatherVelocities(ref sourceVelocities, ref velocitiesC, ref baseIndexC, i);
                GatherVelocities(ref sourceVelocities, ref velocitiesD, ref baseIndexD, i);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe void ScatterVelocities(ref BodyVelocities sourceVelocities, ref Buffer<BodyVelocity> targets, ref int baseIndex, int innerIndex)
        {
            //TODO: How much value would there be in branching on kinematic state and avoiding a write? Depends a lot on the number of kinematics.
            ref var sourceSlot = ref GatherScatter.GetOffsetInstance(ref sourceVelocities, innerIndex);
            ref var target = ref targets[Unsafe.Add(ref baseIndex, innerIndex)];
            target.Linear.X = GatherScatter.GetFirst(ref sourceSlot.Linear.X);
            target.Linear.Y = GatherScatter.GetFirst(ref sourceSlot.Linear.Y);
            target.Linear.Z = GatherScatter.GetFirst(ref sourceSlot.Linear.Z);
            target.Angular.X = GatherScatter.GetFirst(ref sourceSlot.Angular.X);
            target.Angular.Y = GatherScatter.GetFirst(ref sourceSlot.Angular.Y);
            target.Angular.Z = GatherScatter.GetFirst(ref sourceSlot.Angular.Z);
        }


        /// <summary>
        /// Scatters velocities for one body bundle into the active body set.
        /// </summary>
        /// <param name="sourceVelocities">Velocities of body bundle A to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(ref BodyVelocities sourceVelocities, ref Buffer<BodyVelocity> targetVelocities, ref Vector<int> references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndex = ref Unsafe.As<Vector<int>, int>(ref references);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocities, ref targetVelocities, ref baseIndex, i);
            }
        }

        /// <summary>
        /// Scatters velocities for two body bundles into the active body set.
        /// </summary>
        /// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        /// <param name="sourceVelocitiesA">Velocities of body bundle B to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(ref BodyVelocities sourceVelocitiesA, ref BodyVelocities sourceVelocitiesB, ref Buffer<BodyVelocity> targetVelocities,
            ref TwoBodyReferences references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocitiesA, ref targetVelocities, ref baseIndexA, i);
                ScatterVelocities(ref sourceVelocitiesB, ref targetVelocities, ref baseIndexB, i);
            }
        }

        /// <summary>
        /// Scatters velocities for three body bundles into the active body set.
        /// </summary>
        /// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        /// <param name="sourceVelocitiesB">Velocities of body bundle B to scatter.</param>
        /// <param name="sourceVelocitiesC">Velocities of body bundle A to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(
            ref BodyVelocities sourceVelocitiesA, ref BodyVelocities sourceVelocitiesB, ref BodyVelocities sourceVelocitiesC,
            ref Buffer<BodyVelocity> targetVelocities, ref ThreeBodyReferences references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocitiesA, ref targetVelocities, ref baseIndexA, i);
                ScatterVelocities(ref sourceVelocitiesB, ref targetVelocities, ref baseIndexB, i);
                ScatterVelocities(ref sourceVelocitiesC, ref targetVelocities, ref baseIndexC, i);
            }
        }

        /// <summary>
        /// Scatters velocities for four body bundles into the active body set.
        /// </summary>
        /// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        /// <param name="sourceVelocitiesB">Velocities of body bundle B to scatter.</param>
        /// <param name="sourceVelocitiesC">Velocities of body bundle A to scatter.</param>
        /// <param name="sourceVelocitiesD">Velocities of body bundle B to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(
            ref BodyVelocities sourceVelocitiesA, ref BodyVelocities sourceVelocitiesB, ref BodyVelocities sourceVelocitiesC, ref BodyVelocities sourceVelocitiesD,
            ref Buffer<BodyVelocity> targetVelocities, ref FourBodyReferences references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocitiesA, ref targetVelocities, ref baseIndexA, i);
                ScatterVelocities(ref sourceVelocitiesB, ref targetVelocities, ref baseIndexB, i);
                ScatterVelocities(ref sourceVelocitiesC, ref targetVelocities, ref baseIndexC, i);
                ScatterVelocities(ref sourceVelocitiesD, ref targetVelocities, ref baseIndexD, i);
            }
        }

        internal void ResizeSetsCapacity(int setsCapacity, int potentiallyAllocatedCount)
        {
            Debug.Assert(setsCapacity >= potentiallyAllocatedCount && potentiallyAllocatedCount <= Sets.Length);
            setsCapacity = BufferPool.GetCapacityForCount<BodySet>(setsCapacity);
            if (Sets.Length != setsCapacity)
            {
                var oldCapacity = Sets.Length;
                Pool.ResizeToAtLeast(ref Sets, setsCapacity, potentiallyAllocatedCount);
                if (oldCapacity < Sets.Length)
                    Sets.Clear(oldCapacity, Sets.Length - oldCapacity); //We rely on unused slots being default initialized.
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
            ActiveSet.Clear(Pool);
            //While the top level pool represents active bodies and will persist (as it would after a series of removals),
            //subsequent sets represent inactive bodies. When they are not present, the backing memory is released.
            for (int i = 1; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (set.Allocated)
                    set.Dispose(Pool);
            }
            Unsafe.InitBlockUnaligned(HandleToLocation.Memory, 0xFF, (uint)(sizeof(BodyLocation) * HandleToLocation.Length));
            HandlePool.Clear();
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void ResizeHandles(int newCapacity)
        {
            newCapacity = BufferPool.GetCapacityForCount<BodyLocation>(newCapacity);
            if (newCapacity != HandleToLocation.Length)
            {
                var oldCapacity = HandleToLocation.Length;
                Pool.ResizeToAtLeast(ref HandleToLocation, newCapacity, Math.Min(oldCapacity, newCapacity));
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
            var targetCapacity = BufferPool.GetCapacityForCount<BodyInertia>(Math.Max(capacity, ActiveSet.Count));
            if (Inertias.Length != targetCapacity)
            {
                Pool.ResizeToAtLeast(ref Inertias, targetCapacity, Math.Min(Inertias.Length, ActiveSet.Count));
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
                Pool.ResizeToAtLeast(ref Inertias, capacity, Math.Min(Inertias.Length, ActiveSet.Count));
            }
        }

        /// <summary>
        /// Resizes the allocated spans for active body data. Note that this is conservative; it will never orphan existing objects.
        /// </summary>
        /// <param name="capacity">Target body data capacity.</param>
        public void Resize(int capacity)
        {
            var targetBodyCapacity = BufferPool.GetCapacityForCount<int>(Math.Max(capacity, ActiveSet.Count));
            if (ActiveSet.IndexToHandle.Length != targetBodyCapacity)
            {
                ActiveSet.InternalResize(targetBodyCapacity, Pool);
            }
            ResizeInertias(capacity);
            var targetHandleCapacity = BufferPool.GetCapacityForCount<int>(Math.Max(capacity, HandlePool.HighestPossiblyClaimedId + 1));
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
            for (int i = 0; i < ActiveSet.Count; ++i)
            {
                ref var list = ref ActiveSet.Constraints[i];
                var targetCapacity = BufferPool.GetCapacityForCount<BodyConstraintReference>(list.Count > MinimumConstraintCapacityPerBody ? list.Count : MinimumConstraintCapacityPerBody);
                if (list.Span.Length != targetCapacity)
                    list.Resize(targetCapacity, Pool);
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
                ActiveSet.InternalResize(capacity, Pool);
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
            for (int i = 0; i < ActiveSet.Count; ++i)
            {
                ref var list = ref ActiveSet.Constraints[i];
                if (list.Span.Length < MinimumConstraintCapacityPerBody)
                    list.Resize(MinimumConstraintCapacityPerBody, Pool);
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
                    set.Dispose(Pool);
                }
            }
            Pool.Return(ref Sets);
            if (Inertias.Allocated)
                Pool.Return(ref Inertias);
            Pool.Return(ref HandleToLocation);
            HandlePool.Dispose(Pool);
        }

    }
}
