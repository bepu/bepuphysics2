using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using static BepuUtilities.GatherScatter;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BepuPhysics
{
    /// <summary>
    /// Location of a body in memory.
    /// </summary>
    public struct BodyMemoryLocation
    {
        /// <summary>
        /// Index of the set owning the body reference. If the set index is 0, the body is awake. If the set index is greater than zero, the body is asleep.
        /// </summary>
        public int SetIndex;
        /// <summary>
        /// Index of the body within its owning set.
        /// </summary>
        public int Index;
    }

    /// <summary>
    /// Collection of all allocated bodies.
    /// </summary>
    public partial class Bodies
    {
        /// <summary>
        /// Remaps a body handle integer value to the actual array index of the body.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public Buffer<BodyMemoryLocation> HandleToLocation;
        /// <summary>
        /// Pool from which handles are pulled for new bodies.
        /// </summary>
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
        public unsafe ref BodySet ActiveSet { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return ref *Sets.Memory; } }

        /// <summary>
        /// Gets a reference to a body in the collection.
        /// </summary>
        /// <param name="handle">Handle of the body to pull a reference for.</param>
        /// <returns>Reference to the requested body.</returns>
        /// <remarks>This is an alias for the older <see cref="GetBodyReference"/> and the <see cref="BodyReference"/> constructor. They are all equivalent.</remarks>
        public BodyReference this[BodyHandle handle]
        {
            get
            {
                return new BodyReference(handle, this);
            }
        }


        /// <summary>
        /// Gets the pool used by the bodies collection to allocate and free memory.
        /// </summary>
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

        /// <summary>
        /// Updates the bounds held within the broad phase for the body's current state. Does not expand the bounding box by velocity. If there is no shape associated with the body, this does nothing.
        /// </summary>
        public void UpdateBounds(BodyHandle bodyHandle)
        {
            ref var location = ref HandleToLocation[bodyHandle.Value];
            ref var set = ref Sets[location.SetIndex];
            ref var collidable = ref set.Collidables[location.Index];
            if (collidable.Shape.Exists)
            {
                shapes.UpdateBounds(set.SolverStates[location.Index].Motion.Pose, ref collidable.Shape, out var bodyBounds);
                if (location.SetIndex == 0)
                {
                    broadPhase.UpdateActiveBounds(collidable.BroadPhaseIndex, bodyBounds.Min, bodyBounds.Max);
                }
                else
                {
                    broadPhase.UpdateStaticBounds(collidable.BroadPhaseIndex, bodyBounds.Min, bodyBounds.Max);
                }
            }

        }

        void AddCollidableToBroadPhase(BodyHandle bodyHandle, in RigidPose pose, in BodyInertia localInertia, ref Collidable collidable)
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
        internal void UpdateCollidableBroadPhaseIndex(BodyHandle handle, int newBroadPhaseIndex)
        {
            ref var movedOriginalLocation = ref HandleToLocation[handle.Value];
            Sets[movedOriginalLocation.SetIndex].Collidables[movedOriginalLocation.Index].BroadPhaseIndex = newBroadPhaseIndex;
        }
        void RemoveCollidableFromBroadPhase(ref Collidable collidable)
        {
            var removedBroadPhaseIndex = collidable.BroadPhaseIndex;
            //The below removes a body's collidable from the broad phase and adjusts the broad phase index of any moved leaf.
            if (broadPhase.RemoveActiveAt(removedBroadPhaseIndex, out var movedLeaf))
            {
                //Note that this is always an active body, so we know that whatever takes the body's place in the broad phase is also an active body.
                //All statics and inactive bodies exist in the static tree.
                Debug.Assert(movedLeaf.Mobility != CollidableMobility.Static);
                UpdateCollidableBroadPhaseIndex(movedLeaf.BodyHandle, removedBroadPhaseIndex);
            }
        }
        /// <summary>
        /// Adds a new active body to the simulation.
        /// </summary>
        /// <param name="description">Description of the body to add.</param>
        /// <returns>Handle of the created body.</returns>
        public unsafe BodyHandle Add(in BodyDescription description)
        {
            Debug.Assert(HandleToLocation.Allocated, "The backing memory of the bodies set should be initialized before use.");
            var handleIndex = HandlePool.Take();
            Debug.Assert(handleIndex <= HandleToLocation.Length, "It should be impossible for a new handle to end up more than one slot beyond the current handle to index array. " +
                "This would imply some form of resize or compaction bug.");
            if (handleIndex == HandleToLocation.Length)
            {
                //Out of room; need to resize.
                ResizeHandles(HandleToLocation.Length << 1);
            }
            Debug.Assert(Math.Abs(description.Pose.Orientation.Length() - 1) < 1e-6f, "Orientation should be initialized to a unit length quaternion.");

            //All new bodies are active for simplicity. Someday, it may be worth offering an optimized path for inactives, but it adds complexity.
            //(Directly adding inactive bodies can be helpful in some networked open world scenarios.)
            var handle = new BodyHandle(handleIndex);
            var index = ActiveSet.Add(description, handle, MinimumConstraintCapacityPerBody, Pool);
            HandleToLocation[handleIndex] = new BodyMemoryLocation { SetIndex = 0, Index = index };

            if (description.Collidable.Shape.Exists)
            {
                AddCollidableToBroadPhase(handle, description.Pose, description.LocalInertia, ref ActiveSet.Collidables[index]);
            }
            else
            {
                //Don't want to leak undefined data into the collidable state if there's no shape.
                ActiveSet.Collidables[index].BroadPhaseIndex = -1;
            }
            return handle;
        }

        internal BodyHandle RemoveFromActiveSet(int activeBodyIndex)
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
                RemoveCollidableFromBroadPhase(ref collidable);
            }

            var bodyMoved = set.RemoveAt(activeBodyIndex, out var handle, out var movedBodyIndex, out var movedBodyHandle);
            if (bodyMoved)
            {
                //While the removed body doesn't have any constraints associated with it, the body that gets moved to fill its slot might!
                solver.UpdateForBodyMemoryMove(movedBodyIndex, activeBodyIndex);
                Debug.Assert(HandleToLocation[movedBodyHandle.Value].SetIndex == 0 && HandleToLocation[movedBodyHandle.Value].Index == movedBodyIndex);
                HandleToLocation[movedBodyHandle.Value].Index = activeBodyIndex;
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

            HandlePool.Return(handle.Value, Pool);
            ref var removedBodyLocation = ref HandleToLocation[handle.Value];
            removedBodyLocation.SetIndex = -1;
            removedBodyLocation.Index = -1;
        }

        /// <summary>
        /// Removes a body from the set by its handle. If the body is inactive, all bodies in its island will be forced active.
        /// </summary>
        /// <param name="handle">Handle of the body to remove.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(BodyHandle handle)
        {
            ValidateExistingHandle(handle);
            awakener.AwakenBody(handle);
            RemoveAt(HandleToLocation[handle.Value].Index);
        }

        /// <summary>
        /// Adds a constraint to an active body's constraint list.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to add the constraint to.</param>
        /// <param name="constraintHandle">Handle of the constraint to add.</param>
        /// <param name="indexInConstraint">Index of the body in the constraint.</param>
        internal void AddConstraint(int bodyIndex, ConstraintHandle constraintHandle, int indexInConstraint)
        {
            ActiveSet.AddConstraint(bodyIndex, constraintHandle, indexInConstraint, Pool);
        }

        /// <summary>
        /// Removes a constraint from an active body's constraint list.
        /// </summary>
        /// <param name="bodyIndex">Index of the active body.</param>
        /// <param name="constraintHandle">Handle of the constraint to remove.</param>
        /// <returns>True if the number of constraints remaining attached to the body is 0, false otherwise.</returns>
        internal bool RemoveConstraintReference(int bodyIndex, ConstraintHandle constraintHandle)
        {
            return ActiveSet.RemoveConstraintReference(bodyIndex, constraintHandle, MinimumConstraintCapacityPerBody, Pool);
        }

        /// <summary>
        /// Gets whether the inertia matches that of a kinematic body (that is, all inverse mass and inertia components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool IsKinematic(BodyInertia inertia)
        {
            return IsKinematic(&inertia);
        }

        /// <summary>
        /// Gets whether the inertia matches that of a kinematic body (that is, all inverse mass and inertia components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze. Must be a reference to fixed data; a pointer will be taken.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        /// <remarks>This is not exposed by default because of the risk of a non-obvious GC hole.
        /// It exists because it's a mildly more convenient form than the pointer overload, and every use within the engine references only pinned data.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe static bool IsKinematicUnsafeGCHole(ref BodyInertia inertia)
        {
            return IsKinematic((BodyInertia*)Unsafe.AsPointer(ref inertia));
        }

        /// <summary>
        /// Checks inertia lanes for kinematicity (all inverse mass and inertia values are zero).
        /// </summary>
        /// <param name="inertia">Inertia to examine for kinematicity.</param>
        /// <returns>Mask of lanes which contain zeroed inverse masses and inertias.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<int> IsKinematic(BodyInertiaWide inertia)
        {
            return Vector.Equals(Vector.BitwiseOr(
                Vector.BitwiseOr(Vector.BitwiseOr(inertia.InverseMass, inertia.InverseInertiaTensor.XX), Vector.BitwiseOr(inertia.InverseInertiaTensor.YX, inertia.InverseInertiaTensor.YY)),
                Vector.BitwiseOr(Vector.BitwiseOr(inertia.InverseInertiaTensor.ZX, inertia.InverseInertiaTensor.ZY), inertia.InverseInertiaTensor.ZZ)), Vector<float>.Zero);
        }

        /// <summary>
        /// Gets whether the inertia matches that of a kinematic body (that is, all inverse mass and inertia components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool IsKinematic(BodyInertia* inertia)
        {
            if (Avx.IsSupported)
            {
                var inertiaVector = Avx.LoadVector256((float*)inertia);
                var masked = Avx.CompareEqual(inertiaVector, Vector256<float>.Zero);
                return (Avx.MoveMask(masked) & 0x7F) == 0x7F;
            }
            else
            {
                return inertia->InverseMass == 0 && HasLockedInertia(&inertia->InverseInertiaTensor);
            }
        }

        /// <summary>
        /// Gets whether the angular inertia matches that of a kinematic body (that is, all inverse inertia tensor components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool HasLockedInertia(Symmetric3x3 inertia)
        {
            return HasLockedInertia(&inertia);
        }

        /// <summary>
        /// Gets whether the angular inertia matches that of a kinematic body (that is, all inverse inertia tensor components are zero).
        /// </summary>
        /// <param name="inertia">Body inertia to analyze.</param>
        /// <returns>True if all components of inverse mass and inertia are zero, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool HasLockedInertia(Symmetric3x3* inertia)
        {
            if (Avx.IsSupported)
            {
                var inertiaVector = Avx.LoadVector256((float*)inertia);
                var masked = Avx.CompareEqual(inertiaVector, Vector256<float>.Zero);
                return (Avx.MoveMask(masked) & 0x3F) == 0x3F;
            }
            else
            {
                return inertia->XX == 0 &&
                       inertia->YX == 0 &&
                       inertia->YY == 0 &&
                       inertia->ZX == 0 &&
                       inertia->ZY == 0 &&
                       inertia->ZZ == 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void UpdateForKinematicStateChange(BodyHandle handle, ref BodyMemoryLocation location, ref BodySet set, bool previouslyKinematic, bool currentlyKinematic)
        {
            Debug.Assert(location.SetIndex == 0, "If we're changing kinematic state, we should have already awoken the body.");
            if (previouslyKinematic != currentlyKinematic)
            {
                ref var collidable = ref set.Collidables[location.Index];
                if (collidable.Shape.Exists)
                {
                    //Any collidable references need their encoded mobility updated.
                    var mobility = currentlyKinematic ? CollidableMobility.Kinematic : CollidableMobility.Dynamic;
                    if (location.SetIndex == 0)
                    {
                        broadPhase.activeLeaves[collidable.BroadPhaseIndex] = new CollidableReference(mobility, handle);
                    }
                    else
                    {
                        broadPhase.staticLeaves[collidable.BroadPhaseIndex] = new CollidableReference(mobility, handle);
                    }
                }
                ref var constraints = ref set.Constraints[location.Index];
                if (currentlyKinematic)
                {
                    solver.UpdateReferencesForBodyBecomingKinematic(handle, location.Index);
                }
                else
                {
                    solver.UpdateReferencesForBodyBecomingDynamic(handle, location.Index);
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
        public void SetLocalInertia(BodyHandle handle, in BodyInertia localInertia)
        {
            ref var location = ref HandleToLocation[handle.Value];
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                awakener.AwakenBody(handle);
            }
            //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move.
            ref var set = ref Sets[location.SetIndex];
            ref var inertiaReference = ref set.SolverStates[location.Index].Inertia;
            ref var localInertiaReference = ref set.SolverStates[location.Index].Inertia.Local;
            var nowKinematic = IsKinematic(localInertia);
            var previouslyKinematic = IsKinematicUnsafeGCHole(ref inertiaReference.Local);
            inertiaReference.Local = localInertia;
            //The world inertia is updated on demand and is not 'persistent' data.
            //In the event that the body is now kinematic, it won't be updated by pose integration and such, so it should be initialized to zeroes.
            //Since initializing it to zeroes unconditionally avoids dynamics having some undefined data lingering around in the worst case, might as well.
            inertiaReference.World = default;
            UpdateForKinematicStateChange(handle, ref location, ref set, previouslyKinematic, nowKinematic);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateForShapeChange(BodyHandle handle, int activeBodyIndex, TypedIndex oldShape, TypedIndex newShape)
        {
            if (oldShape.Exists != newShape.Exists)
            {
                ref var set = ref ActiveSet;
                if (newShape.Exists)
                {
                    //Add a collidable to the simulation for the new shape.
                    ref var state = ref set.SolverStates[activeBodyIndex];
                    AddCollidableToBroadPhase(handle, state.Motion.Pose, state.Inertia.Local, ref set.Collidables[activeBodyIndex]);
                }
                else
                {
                    //Remove the now-unused collidable from the simulation.
                    RemoveCollidableFromBroadPhase(ref set.Collidables[activeBodyIndex]);
                }
            }
        }
        /// <summary>
        /// Changes the shape of a body. Properly handles the transition between shapeless and shapeful. If the body is inactive, it will be forced awake.
        /// Updates the bounds of the body in the broad phase.
        /// </summary>
        /// <param name="handle">Handle of the body to change the shape of.</param>
        /// <param name="newShape">Index of the new shape to use for the body.</param>
        public void SetShape(BodyHandle handle, TypedIndex newShape)
        {
            ref var location = ref HandleToLocation[handle.Value];
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
            UpdateBounds(handle);
        }

        /// <summary>
        /// Applies a description to a body. Properly handles any transitions between dynamic and kinematic and between shapeless and shapeful.
        /// If the body is becoming kinematic, any constraints which only contain kinematic bodies will be removed. Wakes up the body.
        /// Updates the bounds of the body in the broad phase.
        /// </summary>
        /// <param name="handle">Handle of the body to receive the description.</param>
        /// <param name="description">Description to apply to the body.</param>
        public void ApplyDescription(BodyHandle handle, in BodyDescription description)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle.Value];
            if (location.SetIndex > 0)
            {
                //The body is inactive. Wake it up.
                awakener.AwakenBody(handle);
            }
            //Note that the HandleToLocation slot reference is still valid; it may have been updated, but handle slots don't move.
            ref var set = ref Sets[location.SetIndex];
            ref var collidable = ref set.Collidables[location.Index];
            var oldShape = collidable.Shape;
            var nowKinematic = IsKinematic(description.LocalInertia);
            var previouslyKinematic = IsKinematicUnsafeGCHole(ref set.SolverStates[location.Index].Inertia.Local);
            set.ApplyDescriptionByIndex(location.Index, description);
            UpdateForShapeChange(handle, location.Index, oldShape, description.Collidable.Shape);
            UpdateForKinematicStateChange(handle, ref location, ref set, previouslyKinematic, nowKinematic);
            UpdateBounds(handle);
        }

        /// <summary>
        /// Gets the description of a body by handle.
        /// </summary>
        /// <param name="handle">Handle of the body to look up.</param>
        /// <param name="description">Description of the body.</param>
        public void GetDescription(BodyHandle handle, out BodyDescription description)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle.Value];
            ref var set = ref Sets[location.SetIndex];
            set.GetDescription(location.Index, out description);
        }

        /// <summary>
        /// Gets the description of a body by handle.
        /// </summary>
        /// <param name="handle">Handle of the body to look up.</param>
        /// <returns>Description of the body.</returns>
        public BodyDescription GetDescription(BodyHandle handle)
        {
            ValidateExistingHandle(handle);
            ref var location = ref HandleToLocation[handle.Value];
            ref var set = ref Sets[location.SetIndex];
            set.GetDescription(location.Index, out var description);
            return description;
        }

        /// <summary>
        /// Gets a reference to a body by its handle.
        /// </summary>
        /// <param name="handle">Handle of the body to grab a reference of.</param>
        /// <returns>Reference to the desired body.</returns>
        /// <remarks>This is an alias for <see cref="this[BodyHandle]"/> and the <see cref="BodyReference"/> constructor. They are all equivalent.</remarks>
        public BodyReference GetBodyReference(BodyHandle handle)
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
        public bool BodyExists(BodyHandle bodyHandle)
        {
            //A negative set index marks a body handle as unused.
            return bodyHandle.Value >= 0 && bodyHandle.Value < HandleToLocation.Length && HandleToLocation[bodyHandle.Value].SetIndex >= 0;
        }

        [Conditional("DEBUG")]
        internal void ValidateExistingHandle(BodyHandle handle)
        {
            Debug.Assert(handle.Value >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle.Value <= HandlePool.HighestPossiblyClaimedId && HandlePool.HighestPossiblyClaimedId < HandleToLocation.Length,
                "Existing handles must fit within the body handle->index mapping.");
            ref var location = ref HandleToLocation[handle.Value];
            Debug.Assert(location.SetIndex >= 0 && location.SetIndex < Sets.Length, "Body set index must be nonnegative and within the sets buffer length.");
            ref var set = ref Sets[location.SetIndex];
            Debug.Assert(set.Allocated);
            Debug.Assert(set.Count <= set.IndexToHandle.Length);
            Debug.Assert(location.Index >= 0 && location.Index < set.Count, "Body index must fall within the existing body set.");
            Debug.Assert(set.IndexToHandle[location.Index].Value == handle.Value, "Handle->index must match index->handle map.");
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
                        ref var state = ref set.SolverStates[j];
                        try
                        {
                            state.Motion.Pose.Position.Validate();
                            state.Motion.Pose.Orientation.ValidateOrientation();
                            state.Motion.Velocity.Linear.Validate();
                            state.Motion.Velocity.Angular.Validate();
                        }
                        catch
                        {
                            Console.WriteLine($"Validation failed on body {i} of set {j}. Position: {state.Motion.Pose.Position}, orientation: {state.Motion.Pose.Orientation}, linear: {state.Motion.Velocity.Linear}, angular: {state.Motion.Velocity.Angular}");
                            throw;
                        }

                    }
                }
            }
        }


        internal void ValidateAwakeMotionStatesByHash(HashDiagnosticType type)
        {
            var instance = InvasiveHashDiagnostics.Instance;
            ref int hash = ref instance.GetHashForType(type);
            ref var set = ref ActiveSet;
            for (int j = 0; j < set.Count; ++j)
            {
                ref var state = ref set.SolverStates[j];
                instance.ContributeToHash(ref hash, state.Motion.Pose.Position);
                instance.ContributeToHash(ref hash, state.Motion.Pose.Orientation);
                instance.ContributeToHash(ref hash, state.Motion.Velocity.Linear);
                instance.ContributeToHash(ref hash, state.Motion.Velocity.Angular);
                instance.ContributeToHash(ref hash, state.Inertia.Local.InverseInertiaTensor);
                instance.ContributeToHash(ref hash, state.Inertia.Local.InverseMass);
                instance.ContributeToHash(ref hash, state.Inertia.World.InverseInertiaTensor);
                instance.ContributeToHash(ref hash, state.Inertia.World.InverseMass);
            }
        }

        internal void ValidateAwakeCollidablesByHash(HashDiagnosticType type)
        {
            var instance = InvasiveHashDiagnostics.Instance;
            ref int hash = ref instance.GetHashForType(type);
            ref var set = ref ActiveSet;
            for (int j = 0; j < set.Count; ++j)
            {
                instance.ContributeToHash(ref hash, set.Collidables[j]);
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
        struct ActiveConstraintBodyHandleEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEach<BodyHandle>
        {
            public Bodies bodies;
            public TInnerEnumerator InnerEnumerator;
            public int SourceBodyIndex;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int encodedBodyIndex)
            {
                var bodyIndex = encodedBodyIndex & BodyReferenceMask;
                if (SourceBodyIndex != bodyIndex)
                {
                    //This enumerator is associated with the public connected bodies enumerator function. The user supplies a handle and expects handles in return, so we 
                    //must convert the solver-provided indices to handles.
                    InnerEnumerator.LoopBody(bodies.ActiveSet.IndexToHandle[bodyIndex]);
                }
            }

        }
        //Note that inactive constraints reference bodies by handles rather than indices.
        //The solver, however, stores them by integer value, rather than typed BodyHandles. That's a little bit gross, but we can do the translation here.
        struct InactiveConstraintBodyHandleEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEach<BodyHandle>
        {
            public TInnerEnumerator InnerEnumerator;
            public BodyHandle SourceBodyHandle;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyHandle)
            {
                if (SourceBodyHandle.Value != connectedBodyHandle)
                {
                    //Since this enumerator is associated with inactive constraints, which directly store body handles rather than body indices, 
                    //we can directly pass the solver-provided handle.
                    InnerEnumerator.LoopBody(new BodyHandle(connectedBodyHandle));
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
                solver.EnumerateConnectedBodyReferences(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnumerateConnectedBodies<TEnumerator>(BodyHandle bodyHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<BodyHandle>
        {
            ref var bodyLocation = ref HandleToLocation[bodyHandle.Value];
            ref var set = ref Sets[bodyLocation.SetIndex];
            ref var list = ref set.Constraints[bodyLocation.Index];
            //In the loops below, we still make use of the reversed iteration. Removing from within the context of an enumerator is a dangerous move, but it is permitted if the user 
            //is careful. By maintaining the same convention across all of these enumerations, it makes it a little easier to do reliably.
            if (bodyLocation.SetIndex == 0)
            {
                //The body is active. Use the active enumerator.
                ActiveConstraintBodyHandleEnumerator<TEnumerator> constraintBodiesEnumerator;
                constraintBodiesEnumerator.InnerEnumerator = enumerator;
                constraintBodiesEnumerator.SourceBodyIndex = bodyLocation.Index;
                constraintBodiesEnumerator.bodies = this;

                for (int i = list.Count - 1; i >= 0; --i)
                {
                    solver.EnumerateConnectedRawBodyReferences(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
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
                    solver.EnumerateConnectedRawBodyReferences(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
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
            Unsafe.InitBlockUnaligned(HandleToLocation.Memory, 0xFF, (uint)(sizeof(BodyMemoryLocation) * HandleToLocation.Length));
            HandlePool.Clear();
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void ResizeHandles(int newCapacity)
        {
            newCapacity = BufferPool.GetCapacityForCount<BodyMemoryLocation>(newCapacity);
            if (newCapacity != HandleToLocation.Length)
            {
                var oldCapacity = HandleToLocation.Length;
                Pool.ResizeToAtLeast(ref HandleToLocation, newCapacity, Math.Min(oldCapacity, newCapacity));
                if (HandleToLocation.Length > oldCapacity)
                {
                    Unsafe.InitBlockUnaligned(
                      HandleToLocation.Memory + oldCapacity, 0xFF,
                      (uint)(sizeof(BodyMemoryLocation) * (HandleToLocation.Length - oldCapacity)));
                }
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
            Pool.Return(ref HandleToLocation);
            HandlePool.Dispose(Pool);
        }

    }
}
