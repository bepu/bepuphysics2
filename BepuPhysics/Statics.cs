using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using BepuPhysics.CollisionDetection;

namespace BepuPhysics
{
    /// <summary>
    /// Defines a type that determines which bodies should be awoken in the bounding box of a static when the static's state changes.
    /// </summary>
    /// <remarks>We cannot find bodies depending on a static through constraints, since statics do not maintain constraint lists.
    /// When a static is added, removed, has SetShape or has ApplyDescription called on it, 
    /// it looks up bodies in its bounding box that could possibly be affected by change and considers waking them up.
    /// This filter can be used to ignore some bodies.</remarks>
    public interface IStaticChangeAwakeningFilter
    {
        /// <summary>
        /// Gets whether to allow awakening for any body. If true, candidates will have the ShouldAwaken function called for them. 
        /// If false, ShouldAwaken will not be called, and no bodies will be awoken.
        /// </summary>
        bool AllowAwakening { get; }

        /// <summary>
        /// Determines whether a body should be forced awake by the state change of a static.
        /// </summary>
        /// <param name="body">Sleeping body under consideration for awakening.</param>
        /// <returns>True if the body should be forced awake, false otherwise.</returns>
        bool ShouldAwaken(BodyReference body);
    }

    /// <summary>
    /// Default awakening filter that only wakes up dynamic bodies. Kinematic bodies do not respond to any kind of dynamic simulation, so they won't respond to the change in statics.
    /// </summary>
    public struct StaticsShouldntAwakenKinematics : IStaticChangeAwakeningFilter
    {
        public bool AllowAwakening => true;
        public bool ShouldAwaken(BodyReference body)
        {
            return !body.Kinematic;
        }
    }

    /// <summary>
    /// Stores data for a static collidable in the simulation. Statics can be posed and collide, but have no velocity and no dynamic behavior.
    /// </summary>
    /// <remarks>Unlike bodies, statics have a very simple access pattern. Most data is referenced together and there are no extreme high frequency data accesses like there are in the solver.
    /// Everything can be conveniently stored within a single location contiguously.</remarks>
    public struct Static
    {
        /// <summary>
        /// Pose of the static collidable.
        /// </summary>
        public RigidPose Pose;

        /// <summary>
        /// Continuous collision detection settings for this collidable. Includes the collision detection mode to use and tuning variables associated with those modes.
        /// </summary>
        /// <remarks>Note that statics cannot move, so there is no difference between <see cref="ContinuousDetectionMode.Discrete"/> and <see cref="ContinuousDetectionMode.Passive"/> for them.
        /// Enabling <see cref="ContinuousDetectionMode.Continuous"/> will still require that pairs associated with the static use swept continuous collision detection.</remarks>
        public ContinuousDetection Continuity;

        /// <summary>
        /// Index of the shape used by the static. While this can be changed, any transition from shapeless->shapeful or shapeful->shapeless must be reported to the broad phase. 
        /// If you need to perform such a transition, consider using <see cref="Statics.SetShape"/> or Statics.ApplyDescription; those functions update the relevant state.
        /// </summary>
        public TypedIndex Shape;
        //Note that statics do not store a 'speculative margin' independently of the contini
        /// <summary>
        /// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters. Under normal circumstances, this should not be set externally.
        /// </summary>
        public int BroadPhaseIndex;
    }

    /// <summary>
    /// Collection of allocated statics.
    /// </summary>
    public class Statics
    {
        //Unlike bodies, there's not a lot of value to tight packing for the sake of enumeration.
        //This uses the same kind of handle indirection just to allow Count to be meaningful,
        //but if there comes a time where maintaining this costs something, it can be pretty easily swapped out for an unmoving index approach.

        /// <summary>
        /// Remaps a static handle integer value to the actual array index of the static.
        /// </summary>
        public Buffer<int> HandleToIndex;
        /// <summary>
        /// Remaps a static index to its handle.
        /// </summary>
        public Buffer<StaticHandle> IndexToHandle;
        /// <summary>
        /// The set of collidables owned by each static. Speculative margins, continuity settings, and shape indices can be changed directly.
        /// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase of the collidable addition or removal.
        /// Consider using <see cref="SetShape"/> or <see cref="ApplyDescription(StaticHandle, in StaticDescription)"/> to handle the bookkeeping changes automatically if changing the shape.
        /// </summary>
        public Buffer<Static> StaticsBuffer;

        public IdPool HandlePool;
        protected BufferPool pool;
        public int Count;

        Shapes shapes;
        Bodies bodies;
        internal BroadPhase broadPhase;
        internal IslandAwakener awakener;

        /// <summary>
        /// Gets a reference to the memory backing a static collidable. The <see cref="StaticReference"/> type is a helper that exposes common operations for statics.
        /// </summary>
        /// <param name="handle">Handle of the static to retrieve a reference for.</param>
        /// <returns>Reference to the memory backing a static collidable.</returns>
        /// <remarks>Equivalent to <see cref="GetStaticReference(StaticHandle)"/>.</remarks>
        public StaticReference this[StaticHandle handle]
        {
            get
            {
                ValidateExistingHandle(handle);
                return new StaticReference(handle, this);
            }
        }

        /// <summary>
        /// Gets a direct reference to the memory backing a static.
        /// </summary>
        /// <param name="handle">Handle of the static to get a reference of.</param>
        /// <returns>Direct reference to the memory backing a static.</returns>
        /// <remarks>This is distinct from the <see cref="this[StaticHandle]"/> indexer in that this returns the direct memory reference. <see cref="StaticReference"/> includes a layer of indirection that can expose more features.</remarks>
        public ref Static GetDirectReference(StaticHandle handle)
        {
            ValidateExistingHandle(handle);
            return ref StaticsBuffer[HandleToIndex[handle.Value]];
        }

        /// <summary>
        /// Gets a reference to the raw memory backing a static collidable.
        /// </summary>
        /// <param name="index">Index of the static to retrieve a memory reference for.</param>
        /// <returns>Direct reference to the memory backing a static collidable.</returns>
        public ref Static this[int index]
        {
            get
            {
                Debug.Assert(index >= 0 && index < Count);
                ValidateExistingHandle(IndexToHandle[index]);
                return ref StaticsBuffer[index];
            }
        }

        public unsafe Statics(BufferPool pool, Shapes shapes, Bodies bodies, BroadPhase broadPhase, int initialCapacity = 4096)
        {
            this.pool = pool;
            InternalResize(Math.Max(1, initialCapacity));

            this.shapes = shapes;
            this.bodies = bodies;
            this.broadPhase = broadPhase;

            HandlePool = new IdPool(initialCapacity, pool);
        }

        unsafe void InternalResize(int targetCapacity)
        {
            Debug.Assert(targetCapacity > 0, "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
            //Note that we base the bundle capacities on the static capacity. This simplifies the conditions on allocation
            targetCapacity = BufferPool.GetCapacityForCount<int>(targetCapacity);
            Debug.Assert(StaticsBuffer.Length != BufferPool.GetCapacityForCount<Static>(targetCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.ResizeToAtLeast(ref StaticsBuffer, targetCapacity, Count);
            pool.ResizeToAtLeast(ref IndexToHandle, targetCapacity, Count);
            pool.ResizeToAtLeast(ref HandleToIndex, targetCapacity, Count);
            //Initialize all the indices beyond the copied region to -1.
            Unsafe.InitBlockUnaligned(HandleToIndex.Memory + Count, 0xFF, (uint)(sizeof(int) * (HandleToIndex.Length - Count)));
            //Note that we do NOT modify the idpool's internal queue size here. We lazily handle that during adds, and during explicit calls to EnsureCapacity, Compact, and Resize.
            //The idpool's internal queue will often be nowhere near as large as the actual static size except in corner cases, so in the usual case, being lazy saves a little space.
            //If the user wants to guarantee zero resizes, EnsureCapacity provides them the option to do so.
        }

        /// <summary>
        /// Checks whether a static handle is currently registered with the statics set.
        /// </summary>
        /// <param name="handle">Handle to check for.</param>
        /// <returns>True if the handle exists in the collection, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool StaticExists(StaticHandle handle)
        {
            if (handle.Value < 0 || handle.Value >= HandleToIndex.Length)
                return false;
            //A negative index marks a static handle as unused.
            return HandleToIndex[handle.Value] >= 0;
        }

        [Conditional("DEBUG")]
        internal void ValidateExistingHandle(StaticHandle handle)
        {
            Debug.Assert(StaticExists(handle), "Handle must exist according to the StaticExists test.");
            Debug.Assert(handle.Value >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle.Value < HandleToIndex.Length && HandleToIndex[handle.Value] >= 0 && IndexToHandle[HandleToIndex[handle.Value]].Value == handle.Value,
                "This static handle doesn't seem to exist, or the mappings are out of sync. If a handle exists, both directions should match.");
        }

        struct SleepingBodyCollector<TFilter> : IBreakableForEach<int> where TFilter : struct, IStaticChangeAwakeningFilter
        {
            Bodies bodies;
            BroadPhase broadPhase;
            BufferPool pool;
            public QuickList<int> SleepingSets;
            public TFilter Filter;

            public SleepingBodyCollector(Bodies bodies, BroadPhase broadPhase, BufferPool pool, ref TFilter filter)
            {
                this.bodies = bodies;
                this.broadPhase = broadPhase;
                this.pool = pool;
                SleepingSets = new QuickList<int>(32, pool);
                Filter = filter;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Dispose()
            {
                SleepingSets.Dispose(pool);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool LoopBody(int leafIndex)
            {
                ref var leaf = ref broadPhase.staticLeaves[leafIndex];
                if (leaf.Mobility != CollidableMobility.Static)
                {
                    if (Filter.ShouldAwaken(bodies[leaf.BodyHandle]))
                        SleepingSets.Add(bodies.HandleToLocation[leaf.BodyHandle.Value].SetIndex, pool);
                }
                return true;
            }
        }

        void AwakenBodiesInBounds<TFilter>(ref BoundingBox bounds, ref TFilter filter) where TFilter : struct, IStaticChangeAwakeningFilter
        {
            if (filter.AllowAwakening)
            {
                var collector = new SleepingBodyCollector<TFilter>(bodies, broadPhase, pool, ref filter);
                broadPhase.StaticTree.GetOverlaps(bounds, ref collector);
                awakener.AwakenSets(ref collector.SleepingSets);
                //Just in case the filter did some internal mutation, preserve the changes.
                filter = collector.Filter;
                collector.Dispose();
            }
        }

        unsafe void AwakenBodiesInExistingBounds<TFilter>(int broadPhaseIndex, ref TFilter filter) where TFilter : struct, IStaticChangeAwakeningFilter
        {
            Debug.Assert(broadPhaseIndex >= 0 && broadPhaseIndex < broadPhase.StaticTree.LeafCount);
            broadPhase.GetStaticBoundsPointers(broadPhaseIndex, out var minPointer, out var maxPointer);
            BoundingBox oldBounds;
            oldBounds.Min = *minPointer;
            oldBounds.Max = *maxPointer;
            AwakenBodiesInBounds(ref oldBounds, ref filter);
        }

        /// <summary>
        /// Removes a static from the set by index. Any sleeping bodies with bounding boxes overlapping the removed static's bounding box and passing the given filter will be forced awake.
        /// </summary>
        /// <param name="index">Index of the static to remove.</param>
        /// <param name="filter">Filter to apply to sleeping bodies near the removed static to see if they should be awoken.</param>
        /// <typeparam name="TAwakeningFilter">Type of the filter to apply to sleeping bodies.</typeparam>
        public void RemoveAt<TAwakeningFilter>(int index, ref TAwakeningFilter filter) where TAwakeningFilter : struct, IStaticChangeAwakeningFilter
        {
            Debug.Assert(index >= 0 && index < Count);
            ValidateExistingHandle(IndexToHandle[index]);
            var handle = IndexToHandle[index];

            ref var collidable = ref this[index];
            Debug.Assert(collidable.Shape.Exists, "Static collidables cannot lack a shape. Their only purpose is colliding.");
            AwakenBodiesInExistingBounds(collidable.BroadPhaseIndex, ref filter);

            var removedBroadPhaseIndex = collidable.BroadPhaseIndex;
            if (broadPhase.RemoveStaticAt(removedBroadPhaseIndex, out var movedLeaf))
            {
                //When a leaf is removed from the broad phase, another leaf will move to take its place in the leaf set.
                //We must update the collidable->leaf index pointer to match the new position of the leaf in the broadphase.
                //There are two possible cases for the moved leaf:
                //1) it is a sleeping body collidable,
                //2) it is a static collidable.
                //The collidable reference we retrieved tells us whether it's a body or a static.
                if (movedLeaf.Mobility == CollidableMobility.Static)
                {
                    //This is a static collidable, not a body.
                    GetDirectReference(movedLeaf.StaticHandle).BroadPhaseIndex = removedBroadPhaseIndex;
                }
                else
                {
                    //This is a sleeping body.
                    bodies.UpdateCollidableBroadPhaseIndex(movedLeaf.BodyHandle, removedBroadPhaseIndex);
                }
            }

            //Move the last static into the removed slot.
            //This does introduce disorder- there may be value in a second overload that preserves order, but it would require large copies.
            //In the event that so many adds and removals are performed at once that they destroy contiguity, it may be better to just
            //explicitly sort after the fact rather than attempt to retain contiguity incrementally. Handle it as a batch, in other words.
            --Count;
            bool staticMoved = index < Count;
            if (staticMoved)
            {
                var movedStaticOriginalIndex = Count;
                //Copy the memory state of the last element down.
                this[index] = StaticsBuffer[movedStaticOriginalIndex];
                //Point the static handles at the new location.
                var lastHandle = IndexToHandle[movedStaticOriginalIndex];
                HandleToIndex[lastHandle.Value] = index;
                IndexToHandle[index] = lastHandle;
            }
            HandlePool.Return(handle.Value, pool);
            HandleToIndex[handle.Value] = -1;
        }

        /// <summary>
        /// Removes a static from the set by index. Any sleeping dynamic bodies with bounding boxes overlapping the removed static's bounding box will be forced active.
        /// </summary>
        /// <param name="index">Index of the static to remove.</param>
        public void RemoveAt(int index)
        {
            var defaultFilter = default(StaticsShouldntAwakenKinematics);
            RemoveAt(index, ref defaultFilter);
        }

        /// <summary>
        /// Removes a static from the set. Any sleeping bodies with bounding boxes overlapping the removed static's bounding box and passing the given filter will be forced active.
        /// </summary>
        /// <param name="handle">Handle of the static to remove.</param>
        /// <param name="filter">Filter to apply to sleeping bodies near the removed static to see if they should be awoken.</param>
        /// <typeparam name="TAwakeningFilter">Type of the filter to apply to sleeping bodies.</typeparam>
        public void Remove<TAwakeningFilter>(StaticHandle handle, ref TAwakeningFilter filter) where TAwakeningFilter : struct, IStaticChangeAwakeningFilter
        {
            ValidateExistingHandle(handle);
            var removedIndex = HandleToIndex[handle.Value];
            RemoveAt(removedIndex, ref filter);
        }

        /// <summary>
        /// Removes a static from the set. Any sleeping dynamic bodies with bounding boxes overlapping the removed static's bounding box will be forced active.
        /// </summary>
        /// <param name="handle">Handle of the static to remove.</param>
        public void Remove(StaticHandle handle)
        {
            var defaultFilter = default(StaticsShouldntAwakenKinematics);
            Remove(handle, ref defaultFilter);
        }

        /// <summary>
        /// Updates the bounds held within the broad phase for the static's current state.
        /// </summary>
        public void UpdateBounds(StaticHandle handle)
        {
            var index = HandleToIndex[handle.Value];
            ref var collidable = ref this[index];
            shapes.UpdateBounds(collidable.Pose, ref collidable.Shape, out var bodyBounds);
            broadPhase.UpdateStaticBounds(collidable.BroadPhaseIndex, bodyBounds.Min, bodyBounds.Max);
        }

        void ComputeNewBoundsAndAwaken<TAwakeningFilter>(in RigidPose pose, TypedIndex shape, ref TAwakeningFilter filter, out BoundingBox bounds) where TAwakeningFilter : struct, IStaticChangeAwakeningFilter
        {
            Debug.Assert(shape.Exists, "Statics must have a shape.");
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            shapes[shape.Type].ComputeBounds(shape.Index, pose, out bounds.Min, out bounds.Max);
            AwakenBodiesInBounds(ref bounds, ref filter);
        }

        internal void ApplyDescriptionByIndexWithoutBroadPhaseModification<TAwakeningFilter>(int index, in StaticDescription description, ref TAwakeningFilter filter, out BoundingBox bounds) where TAwakeningFilter : struct, IStaticChangeAwakeningFilter
        {
            ref var collidable = ref this[index];
            collidable.Pose = description.Pose;
            Debug.Assert(description.Shape.Exists, "Static collidables must have a shape. Their only purpose is colliding.");
            collidable.Continuity = description.Continuity;
            collidable.Shape = description.Shape;

            ComputeNewBoundsAndAwaken(description.Pose, description.Shape, ref filter, out bounds);
        }

        /// <summary>
        /// Adds a new static body to the simulation. All sleeping bodies whose bounding boxes overlap the new static and pass the given filter are forced active.
        /// </summary>
        /// <param name="description">Description of the static to add.</param>
        /// <param name="filter">Filter to apply to sleeping bodies near the new static to see if they should be awoken.</param>
        /// <typeparam name="TAwakeningFilter">Type of the filter to apply to sleeping bodies.</typeparam>
        /// <returns>Handle of the new static.</returns>
        public StaticHandle Add<TAwakeningFilter>(in StaticDescription description, ref TAwakeningFilter filter) where TAwakeningFilter : struct, IStaticChangeAwakeningFilter
        {
            if (Count == HandleToIndex.Length)
            {
                Debug.Assert(HandleToIndex.Allocated, "The backing memory of the bodies set should be initialized before use. Did you dispose and then not call EnsureCapacity/Resize?");
                //Out of room; need to resize.
                var newSize = HandleToIndex.Length << 1;
                InternalResize(newSize);
            }
            Debug.Assert(Math.Abs(description.Pose.Orientation.Length() - 1) < 1e-6f, "Orientation should be initialized to a unit length quaternion.");
            var handle = new StaticHandle(HandlePool.Take());
            var index = Count++;
            HandleToIndex[handle.Value] = index;
            IndexToHandle[index] = handle;
            ApplyDescriptionByIndexWithoutBroadPhaseModification(index, description, ref filter, out var bounds);
            //This is a new add, so we need to add it to the broad phase.
            this[index].BroadPhaseIndex = broadPhase.AddStatic(new CollidableReference(handle), ref bounds);
            return handle;
        }

        /// <summary>
        /// Adds a new static body to the simulation. All sleeping dynamic bodies whose bounding boxes overlap the new static are forced active.
        /// </summary>
        /// <param name="description">Description of the static to add.</param>
        /// <returns>Handle of the new static.</returns>
        public StaticHandle Add(in StaticDescription description)
        {
            var defaultFilter = default(StaticsShouldntAwakenKinematics);
            return Add(description, ref defaultFilter);
        }

        /// <summary>
        /// Changes the shape of a static and updates its bounds in the broad phase. All sleeping bodies with bounding boxes overlapping the old or new static collidable and pass the given filter are forced active.
        /// </summary>
        /// <param name="handle">Handle of the static to change the shape of.</param>
        /// <param name="newShape">Index of the new shape to use for the static.</param>
        /// <param name="filter">Filter to apply to sleeping bodies near the static to see if they should be awoken.</param>
        /// <typeparam name="TAwakeningFilter">Type of the filter to apply to sleeping bodies.</typeparam>
        public void SetShape<TAwakeningFilter>(StaticHandle handle, TypedIndex newShape, ref TAwakeningFilter filter) where TAwakeningFilter : struct, IStaticChangeAwakeningFilter
        {
            ValidateExistingHandle(handle);
            Debug.Assert(newShape.Exists, "Statics must have a shape.");
            var index = HandleToIndex[handle.Value];
            ref var collidable = ref this[index];
            AwakenBodiesInExistingBounds(collidable.BroadPhaseIndex, ref filter);
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            ComputeNewBoundsAndAwaken(collidable.Pose, newShape, ref filter, out var bounds);
            broadPhase.UpdateStaticBounds(collidable.BroadPhaseIndex, bounds.Min, bounds.Max);
        }

        /// <summary>
        /// Changes the shape of a static and updates its bounds in the broad phase. All sleeping dyanmic bodies with bounding boxes overlapping the old or new static collidable are forced active.
        /// </summary>
        /// <param name="handle">Handle of the static to change the shape of.</param>
        /// <param name="newShape">Index of the new shape to use for the static.</param>
        public void SetShape(StaticHandle handle, TypedIndex newShape)
        {
            var defaultFilter = default(StaticsShouldntAwakenKinematics);
            SetShape(handle, newShape, ref defaultFilter);
        }

        /// <summary>
        /// Applies a new description to an existing static object. All sleeping bodies with bounding boxes overlapping the old or new static collidable and pass the given filter are forced active.
        /// Updates the bounds of the static in the broad phase.
        /// </summary>
        /// <param name="handle">Handle of the static to apply the description to.</param>
        /// <param name="description">Description to apply to the static.</param>
        /// <param name="filter">Filter to apply to sleeping bodies near the static to see if they should be awoken.</param>
        /// <typeparam name="TAwakeningFilter">Type of the filter to apply to sleeping bodies.</typeparam>
        public unsafe void ApplyDescription<TAwakeningFilter>(StaticHandle handle, in StaticDescription description, ref TAwakeningFilter filter) where TAwakeningFilter : struct, IStaticChangeAwakeningFilter
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle.Value];
            Debug.Assert(description.Shape.Exists, "Static collidables cannot lack a shape. Their only purpose is colliding.");
            //Wake all bodies up in the old bounds AND the new bounds. Sleeping bodies that may have been resting on the old static need to be aware of the new environment.
            var broadPhaseIndex = this[index].BroadPhaseIndex;
            AwakenBodiesInExistingBounds(broadPhaseIndex, ref filter);
            ApplyDescriptionByIndexWithoutBroadPhaseModification(index, description, ref filter, out var bounds);
            //This applies to an existing static, so we should modify the static's bounds in the broad phase.
            broadPhase.UpdateStaticBounds(broadPhaseIndex, bounds.Min, bounds.Max);
        }

        /// <summary>
        /// Applies a new description to an existing static object. All sleeping bodies with bounding boxes overlapping the old or new static collidable and pass the given filter are forced active.
        /// Updates the bounds of the static in the broad phase.
        /// </summary>
        /// <param name="handle">Handle of the static to apply the description to.</param>
        /// <param name="description">Description to apply to the static.</param>
        public unsafe void ApplyDescription(StaticHandle handle, in StaticDescription description)
        {
            var defaultFilter = default(StaticsShouldntAwakenKinematics);
            ApplyDescription(handle, description, ref defaultFilter);
        }

        /// <summary>
        /// Gets the current description of the static referred to by a given handle.
        /// </summary>
        /// <param name="handle">Handle of the static to look up the description of.</param>
        /// <param name="description">Gathered description of the handle-referenced static.</param>
        public void GetDescription(StaticHandle handle, out StaticDescription description)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle.Value];
            ref var collidable = ref this[index];
            description.Pose = collidable.Pose;
            description.Continuity = collidable.Continuity;
            description.Shape = collidable.Shape;
        }

        /// <summary>
        /// Gets a reference to a static by its handle.
        /// </summary>
        /// <param name="handle">Handle of the static to grab a reference of.</param>
        /// <returns>Reference to the desired static.</returns>
        public StaticReference GetStaticReference(StaticHandle handle)
        {
            ValidateExistingHandle(handle);
            return new StaticReference(handle, this);
        }

        /// <summary>
        /// Clears all bodies from the set without returning any memory to the pool.
        /// </summary>
        public unsafe void Clear()
        {
            Count = 0;
            //Empty out all the index-handle mappings.
            Unsafe.InitBlockUnaligned(HandleToIndex.Memory, 0xFF, (uint)(sizeof(int) * HandleToIndex.Length));
            HandlePool.Clear();
        }

        /// <summary>
        /// Resizes the allocated spans for static data. Note that this is conservative; it will never orphan existing objects.
        /// </summary>
        /// <param name="capacity">Target static data capacity.</param>
        public void Resize(int capacity)
        {
            var targetCapacity = BufferPool.GetCapacityForCount<int>(Math.Max(capacity, Count));
            if (IndexToHandle.Length != targetCapacity)
            {
                InternalResize(targetCapacity);
            }
        }

        /// <summary>
        /// Increases the size of buffers if needed to hold the target capacity.
        /// </summary>
        /// <param name="capacity">Target data capacity.</param>
        public void EnsureCapacity(int capacity)
        {
            if (IndexToHandle.Length < capacity)
            {
                InternalResize(capacity);
            }
        }

        /// <summary>
        /// Returns all static resources to the pool used to create them.
        /// </summary>
        /// <remarks>The object can be reused if it is reinitialized by using EnsureCapacity or Resize.</remarks>
        public void Dispose()
        {
            pool.Return(ref StaticsBuffer);
            pool.Return(ref HandleToIndex);
            pool.Return(ref IndexToHandle);
            HandlePool.Dispose(pool);
        }
    }
}
