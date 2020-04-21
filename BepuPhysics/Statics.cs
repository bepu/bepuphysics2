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
    /// Collection of allocated static collidables.
    /// </summary>
    public class Statics
    {
        /// <summary>
        /// Remaps a static handle integer value to the actual array index of the static.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public Buffer<int> HandleToIndex;
        /// <summary>
        /// Remaps a static index to its handle.
        /// </summary>
        public Buffer<StaticHandle> IndexToHandle;
        /// <summary>
        /// The set of collidables owned by each static. Speculative margins, continuity settings, and shape indices can be changed directly.
        /// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase of the collidable addition or removal.
        /// </summary>
        public Buffer<Collidable> Collidables;

        public Buffer<RigidPose> Poses;
        public IdPool HandlePool;
        protected BufferPool pool;
        public int Count;

        Shapes shapes;
        Bodies bodies;
        internal BroadPhase broadPhase;
        internal IslandAwakener awakener;

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
            Debug.Assert(Poses.Length != BufferPool.GetCapacityForCount<RigidPoses>(targetCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.ResizeToAtLeast(ref Poses, targetCapacity, Count);
            pool.ResizeToAtLeast(ref IndexToHandle, targetCapacity, Count);
            pool.ResizeToAtLeast(ref HandleToIndex, targetCapacity, Count);
            pool.ResizeToAtLeast(ref Collidables, targetCapacity, Count);
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
        public void ValidateExistingHandle(StaticHandle handle)
        {
            Debug.Assert(StaticExists(handle), "Handle must exist according to the StaticExists test.");
            Debug.Assert(handle.Value >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle.Value < HandleToIndex.Length && HandleToIndex[handle.Value] >= 0 && IndexToHandle[HandleToIndex[handle.Value]].Value == handle.Value,
                "This static handle doesn't seem to exist, or the mappings are out of sync. If a handle exists, both directions should match.");
        }

        struct InactiveBodyCollector : IBreakableForEach<int>
        {
            BroadPhase broadPhase;
            BufferPool pool;
            public QuickList<BodyHandle> InactiveBodyHandles;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public InactiveBodyCollector(BroadPhase broadPhase, BufferPool pool)
            {
                this.pool = pool;
                this.broadPhase = broadPhase;
                InactiveBodyHandles = new QuickList<BodyHandle>(32, pool);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Dispose()
            {
                InactiveBodyHandles.Dispose(pool);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool LoopBody(int leafIndex)
            {
                ref var leaf = ref broadPhase.staticLeaves[leafIndex];
                if (leaf.Mobility != CollidableMobility.Static)
                {
                    InactiveBodyHandles.Add(leaf.BodyHandle, pool);
                }
                return true;
            }
        }

        void AwakenBodiesInBounds(ref BoundingBox bounds)
        {
            var collector = new InactiveBodyCollector(broadPhase, pool);
            broadPhase.StaticTree.GetOverlaps(bounds, ref collector);
            for (int i = 0; i < collector.InactiveBodyHandles.Count; ++i)
            {
                awakener.AwakenBody(collector.InactiveBodyHandles[i]);
            }
            collector.Dispose();
        }

        unsafe void AwakenBodiesInExistingBounds(ref Collidable collidable)
        {
            Debug.Assert(collidable.BroadPhaseIndex >= 0 && collidable.BroadPhaseIndex < broadPhase.StaticTree.LeafCount);
            broadPhase.GetStaticBoundsPointers(collidable.BroadPhaseIndex, out var minPointer, out var maxPointer);
            BoundingBox oldBounds;
            oldBounds.Min = *minPointer;
            oldBounds.Max = *maxPointer;
            AwakenBodiesInBounds(ref oldBounds);
        }

        /// <summary>
        /// Removes a static from the set by index. Any inactive bodies with bounding boxes overlapping the removed static's bounding box will be forced active.
        /// </summary>
        /// <param name="index">Index of the static to remove.</param>
        public void RemoveAt(int index)
        {
            Debug.Assert(index >= 0 && index < Count);
            ValidateExistingHandle(IndexToHandle[index]);
            var handle = IndexToHandle[index];

            ref var collidable = ref Collidables[index];
            Debug.Assert(collidable.Shape.Exists, "Static collidables cannot lack a shape. Their only purpose is colliding.");
            AwakenBodiesInExistingBounds(ref collidable);

            var removedBroadPhaseIndex = collidable.BroadPhaseIndex;
            if (broadPhase.RemoveStaticAt(removedBroadPhaseIndex, out var movedLeaf))
            {
                //When a leaf is removed from the broad phase, another leaf will move to take its place in the leaf set.
                //We must update the collidable->leaf index pointer to match the new position of the leaf in the broadphase.
                //There are two possible cases for the moved leaf:
                //1) it is an inactive body collidable,
                //2) it is a static collidable.
                //The collidable reference we retrieved tells us whether it's a body or a static.
                if (movedLeaf.Mobility == CollidableMobility.Static)
                {
                    //This is a static collidable, not a body.
                    Collidables[HandleToIndex[movedLeaf.StaticHandle.Value]].BroadPhaseIndex = removedBroadPhaseIndex;
                }
                else
                {
                    //This is an inactive body.
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
                Poses[index] = Poses[movedStaticOriginalIndex];
                //Note that if you ever treat the world inertias as 'always updated', it would need to be copied here.
                Collidables[index] = Collidables[movedStaticOriginalIndex];
                //Point the static handles at the new location.
                var lastHandle = IndexToHandle[movedStaticOriginalIndex];
                HandleToIndex[lastHandle.Value] = index;
                IndexToHandle[index] = lastHandle;
            }
            HandlePool.Return(handle.Value, pool);
            HandleToIndex[handle.Value] = -1;

        }
        /// <summary>
        /// Removes a static from the set. Any inactive bodies with bounding boxes overlapping the removed static's bounding box will be forced active.
        /// </summary>
        /// <param name="handle">Handle of the static to remove.</param>
        public void Remove(StaticHandle handle)
        {
            ValidateExistingHandle(handle);
            var removedIndex = HandleToIndex[handle.Value];
            RemoveAt(removedIndex);
        }

        /// <summary>
        /// Updates the bounds held within the broad phase for the static's current state.
        /// </summary>
        public void UpdateBounds(StaticHandle handle)
        {
            var index = HandleToIndex[handle.Value];
            ref var collidable = ref Collidables[index];
            shapes.UpdateBounds(Poses[index], ref collidable.Shape, out var bodyBounds);
            broadPhase.UpdateStaticBounds(collidable.BroadPhaseIndex, bodyBounds.Min, bodyBounds.Max);
        }

        void ComputeNewBoundsAndAwaken(in RigidPose pose, TypedIndex shape, out BoundingBox bounds)
        {
            Debug.Assert(shape.Exists, "Statics must have a shape.");
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            shapes[shape.Type].ComputeBounds(shape.Index, pose, out bounds.Min, out bounds.Max);
            AwakenBodiesInBounds(ref bounds);
        }

        internal void ApplyDescriptionByIndexWithoutBroadPhaseModification(int index, in StaticDescription description, out BoundingBox bounds)
        {
            Poses[index] = description.Pose;
            ref var collidable = ref Collidables[index];
            Debug.Assert(description.Collidable.Shape.Exists, "Static collidables must have a shape. Their only purpose is colliding.");
            collidable.Continuity = description.Collidable.Continuity;
            collidable.SpeculativeMargin = description.Collidable.SpeculativeMargin;
            collidable.Shape = description.Collidable.Shape;

            ComputeNewBoundsAndAwaken(description.Pose, description.Collidable.Shape, out bounds);
        }

        /// <summary>
        /// Adds a new static body to the simulation. All inactive bodies whose bounding boxes overlap the new static are forced active.
        /// </summary>
        /// <param name="description">Description of the static to add.</param>
        /// <returns>Handle of the new static.</returns>
        public StaticHandle Add(in StaticDescription description)
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
            ApplyDescriptionByIndexWithoutBroadPhaseModification(index, description, out var bounds);
            //This is a new add, so we need to add it to the broad phase.
            Collidables[index].BroadPhaseIndex = broadPhase.AddStatic(new CollidableReference(handle), ref bounds);
            return handle;
        }

        /// <summary>
        /// Changes the shape of a static.
        /// </summary>
        /// <param name="handle">Handle of the static to change the shape of.</param>
        /// <param name="newShape">Index of the new shape to use for the static.</param>
        public void SetShape(StaticHandle handle, TypedIndex newShape)
        {
            ValidateExistingHandle(handle);
            Debug.Assert(newShape.Exists, "Statics must have a shape.");
            var index = HandleToIndex[handle.Value];
            ref var collidable = ref Collidables[index];
            AwakenBodiesInExistingBounds(ref collidable);
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            ComputeNewBoundsAndAwaken(Poses[index], newShape, out var bounds);
            broadPhase.UpdateStaticBounds(collidable.BroadPhaseIndex, bounds.Min, bounds.Max);
        }

        /// <summary>
        /// Applies a new description to an existing static object. All inactive bodies with bounding boxes overlapping the old or new static collidable are forced active.
        /// </summary>
        /// <param name="handle">Handle of the static to apply the description to.</param>
        /// <param name="description">Description to apply to the static.</param>
        public unsafe void ApplyDescription(StaticHandle handle, in StaticDescription description)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle.Value];
            Debug.Assert(description.Collidable.Shape.Exists, "Static collidables cannot lack a shape. Their only purpose is colliding.");
            //Wake all bodies up in the old bounds AND the new bounds. Inactive bodies that may have been resting on the old static need to be aware of the new environment.
            AwakenBodiesInExistingBounds(ref Collidables[index]);
            ApplyDescriptionByIndexWithoutBroadPhaseModification(index, description, out var bounds);
            //This applies to an existing static, so we should modify the static's bounds in the broad phase.
            broadPhase.UpdateStaticBounds(Collidables[index].BroadPhaseIndex, bounds.Min, bounds.Max);

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
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            description.Pose = Poses[index];
            ref var collidable = ref Collidables[index];
            description.Collidable.Continuity = collidable.Continuity;
            description.Collidable.Shape = collidable.Shape;
            description.Collidable.SpeculativeMargin = collidable.SpeculativeMargin;
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
            pool.Return(ref Poses);
            pool.Return(ref HandleToIndex);
            pool.Return(ref IndexToHandle);
            pool.Return(ref Collidables);
            HandlePool.Dispose(pool);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherPose(ref float targetPositionBase, ref float targetOrientationBase, int targetLaneIndex, int index)
        {
            ref var source = ref Poses[index];
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

        //This looks a little different because it's used by AABB calculation, not constraint pairs.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherDataForBounds(ref int start, int count, out RigidPoses poses, out Vector<int> shapeIndices, out Vector<float> maximumExpansion)
        {
            Debug.Assert(count <= Vector<float>.Count);
            ref var targetPositionBase = ref Unsafe.As<Vector<float>, float>(ref poses.Position.X);
            ref var targetOrientationBase = ref Unsafe.As<Vector<float>, float>(ref poses.Orientation.X);
            ref var targetShapeBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            ref var targetExpansionBase = ref Unsafe.As<Vector<float>, float>(ref maximumExpansion);
            for (int i = 0; i < count; ++i)
            {
                var index = Unsafe.Add(ref start, i);
                GatherPose(ref targetPositionBase, ref targetOrientationBase, i, index);
                ref var collidable = ref Collidables[index];
                Unsafe.Add(ref targetShapeBase, i) = collidable.Shape.Index;
                //Not entirely pleased with the fact that this pulls in some logic from bounds calculation.
                Unsafe.Add(ref targetExpansionBase, i) = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
            }
        }



    }
}
