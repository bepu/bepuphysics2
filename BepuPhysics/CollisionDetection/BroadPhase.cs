using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Numerics;
using BepuPhysics.Trees;
using System.Threading;
using BepuUtilities.Collections;

namespace BepuPhysics.CollisionDetection
{
    public unsafe partial class BroadPhase : IDisposable
    {
        internal Buffer<CollidableReference> activeLeaves;
        internal Buffer<CollidableReference> staticLeaves;
        public BufferPool Pool { get; private set; }
        public Tree ActiveTree;
        public Tree StaticTree;
        Tree.RefitAndRefineMultithreadedContext activeRefineContext;
        //TODO: static trees do not need to do nearly as much work as the active; this will change in the future.
        Tree.RefitAndRefineMultithreadedContext staticRefineContext;

        Action<int> executeRefitAndMarkAction, executeRefineAction;

        public BroadPhase(BufferPool pool, int initialActiveLeafCapacity = 4096, int initialStaticLeafCapacity = 8192)
        {
            Pool = pool;
            ActiveTree = new Tree(pool, initialActiveLeafCapacity);
            StaticTree = new Tree(pool, initialStaticLeafCapacity);
            pool.TakeAtLeast(initialActiveLeafCapacity, out activeLeaves);
            pool.TakeAtLeast(initialStaticLeafCapacity, out staticLeaves);

            activeRefineContext = new Tree.RefitAndRefineMultithreadedContext();
            staticRefineContext = new Tree.RefitAndRefineMultithreadedContext();
            executeRefitAndMarkAction = ExecuteRefitAndMark;
            executeRefineAction = ExecuteRefine;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int Add(CollidableReference collidable, ref BoundingBox bounds, ref Tree tree, BufferPool pool, ref Buffer<CollidableReference> leaves)
        {
            var leafIndex = tree.Add(ref bounds, pool);
            if (leafIndex >= leaves.Length)
            {
                pool.ResizeToAtLeast(ref leaves, tree.LeafCount + 1, leaves.Length);
            }
            leaves[leafIndex] = collidable;
            return leafIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool RemoveAt(int index, ref Tree tree, ref Buffer<CollidableReference> leaves, out CollidableReference movedLeaf)
        {
            Debug.Assert(index >= 0);
            var movedLeafIndex = tree.RemoveAt(index);
            if (movedLeafIndex >= 0)
            {
                movedLeaf = leaves[movedLeafIndex];
                leaves[index] = movedLeaf;
                return true;
            }
            movedLeaf = new CollidableReference();
            return false;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int AddActive(CollidableReference collidable, ref BoundingBox bounds)
        {
            return Add(collidable, ref bounds, ref ActiveTree, Pool, ref activeLeaves);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveActiveAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref ActiveTree, ref activeLeaves, out movedLeaf);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int AddStatic(CollidableReference collidable, ref BoundingBox bounds)
        {
            return Add(collidable, ref bounds, ref StaticTree, Pool, ref staticLeaves);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveStaticAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref StaticTree, ref staticLeaves, out movedLeaf);
        }

        /// <summary>
        /// Gets pointers to the leaf's bounds stored in the broad phase's active tree.
        /// </summary>
        /// <param name="index">Index of the active collidable to examine.</param>
        /// <param name="minPointer">Pointer to the minimum bounds in the tree.</param>
        /// <param name="maxPointer">Pointer to the maximum bounds in the tree.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetActiveBoundsPointers(int index, out Vector3* minPointer, out Vector3* maxPointer)
        {
            ActiveTree.GetBoundsPointers(index, out minPointer, out maxPointer);
        }
        /// <summary>
        /// Gets pointers to the leaf's bounds stored in the broad phase's static tree.
        /// </summary>
        /// <param name="index">Index of the static to examine.</param>
        /// <param name="minPointer">Pointer to the minimum bounds in the tree.</param>
        /// <param name="maxPointer">Pointer to the maximum bounds in the tree.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetStaticBoundsPointers(int index, out Vector3* minPointer, out Vector3* maxPointer)
        {
            StaticTree.GetBoundsPointers(index, out minPointer, out maxPointer);
        }

        /// <summary>
        /// Applies updated bounds to the given active leaf index, refitting the tree to match.
        /// </summary>
        /// <param name="broadPhaseIndex">Index of the leaf to update.</param>
        /// <param name="min">New minimum bounds for the leaf.</param>
        /// <param name="max">New maximum bounds for the leaf.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateActiveBounds(int broadPhaseIndex, Vector3 min, Vector3 max)
        {
            ActiveTree.UpdateBounds(broadPhaseIndex, min, max);
        }
        /// <summary>
        /// Applies updated bounds to the given active leaf index, refitting the tree to match.
        /// </summary>
        /// <param name="broadPhaseIndex">Index of the leaf to update.</param>
        /// <param name="min">New minimum bounds for the leaf.</param>
        /// <param name="max">New maximum bounds for the leaf.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateStaticBounds(int broadPhaseIndex, Vector3 min, Vector3 max)
        {
            StaticTree.UpdateBounds(broadPhaseIndex, min, max);
        }

        int frameIndex;
        int remainingJobCount;
        IThreadDispatcher threadDispatcher;
        void ExecuteRefitAndMark(int workerIndex)
        {
            var threadPool = threadDispatcher.GetThreadMemoryPool(workerIndex);
            while (true)
            {
                var jobIndex = Interlocked.Decrement(ref remainingJobCount);
                if (jobIndex < 0)
                    break;
                if (jobIndex < activeRefineContext.RefitNodes.Count)
                {
                    activeRefineContext.ExecuteRefitAndMarkJob(threadPool, workerIndex, jobIndex);
                }
                else
                {
                    jobIndex -= activeRefineContext.RefitNodes.Count;
                    Debug.Assert(jobIndex >= 0 && jobIndex < staticRefineContext.RefitNodes.Count);
                    staticRefineContext.ExecuteRefitAndMarkJob(threadPool, workerIndex, jobIndex);
                }
            }
        }

        void ExecuteRefine(int workerIndex)
        {
            var threadPool = threadDispatcher.GetThreadMemoryPool(workerIndex);
            var maximumSubtrees = Math.Max(activeRefineContext.MaximumSubtrees, staticRefineContext.MaximumSubtrees);
            var subtreeReferences = new QuickList<int>(maximumSubtrees, threadPool);
            var treeletInternalNodes = new QuickList<int>(maximumSubtrees, threadPool);
            Tree.CreateBinnedResources(threadPool, maximumSubtrees, out var buffer, out var resources);
            while (true)
            {
                var jobIndex = Interlocked.Decrement(ref remainingJobCount);
                if (jobIndex < 0)
                    break;
                if (jobIndex < activeRefineContext.RefinementTargets.Count)
                {
                    activeRefineContext.ExecuteRefineJob(ref subtreeReferences, ref treeletInternalNodes, ref resources, threadPool, jobIndex);
                }
                else
                {
                    jobIndex -= activeRefineContext.RefinementTargets.Count;
                    Debug.Assert(jobIndex >= 0 && jobIndex < staticRefineContext.RefinementTargets.Count);
                    staticRefineContext.ExecuteRefineJob(ref subtreeReferences, ref treeletInternalNodes, ref resources, threadPool, jobIndex);
                }
            }
            subtreeReferences.Dispose(threadPool);
            treeletInternalNodes.Dispose(threadPool);
            threadPool.Return(ref buffer);
        }

        public void Update(IThreadDispatcher threadDispatcher = null)
        {
            if (frameIndex == int.MaxValue)
                frameIndex = 0;
            if (threadDispatcher != null)
            {
                this.threadDispatcher = threadDispatcher;
                activeRefineContext.CreateRefitAndMarkJobs(ref ActiveTree, Pool, threadDispatcher);
                staticRefineContext.CreateRefitAndMarkJobs(ref StaticTree, Pool, threadDispatcher);
                remainingJobCount = activeRefineContext.RefitNodes.Count + staticRefineContext.RefitNodes.Count;
                threadDispatcher.DispatchWorkers(executeRefitAndMarkAction, remainingJobCount);
                activeRefineContext.CreateRefinementJobs(Pool, frameIndex, 1f);
                //TODO: for now, the inactive/static tree is simply updated like another active tree. This is enormously inefficient compared to the ideal-
                //by nature, static and inactive objects do not move every frame!
                //However, the refinement system rarely generates enough work to fill modern beefy machine. Even a million objects might only be 16 refinement jobs.
                //To really get the benefit of incremental updates, the tree needs to be reworked to output finer grained work.
                //Since the jobs are large, reducing the refinement aggressiveness doesn't change much here.
                staticRefineContext.CreateRefinementJobs(Pool, frameIndex, 1f);
                remainingJobCount = activeRefineContext.RefinementTargets.Count + staticRefineContext.RefinementTargets.Count;
                threadDispatcher.DispatchWorkers(executeRefineAction, remainingJobCount);
                activeRefineContext.CleanUpForRefitAndRefine(Pool);
                staticRefineContext.CleanUpForRefitAndRefine(Pool);
                this.threadDispatcher = null;
            }
            else
            {
                ActiveTree.RefitAndRefine(Pool, frameIndex);
                StaticTree.RefitAndRefine(Pool, frameIndex);
            }
            ++frameIndex;
        }

        /// <summary>
        /// Clears out the broad phase's structures without releasing any resources.
        /// </summary>
        public void Clear()
        {
            ActiveTree.Clear();
            StaticTree.Clear();
        }

        void EnsureCapacity(ref Tree tree, ref Buffer<CollidableReference> leaves, int capacity)
        {
            if (tree.Leaves.Length < capacity)
                tree.Resize(Pool, capacity);
            if (leaves.Length < capacity)
                Pool.ResizeToAtLeast(ref leaves, capacity, tree.LeafCount);
        }
        void ResizeCapacity(ref Tree tree, ref Buffer<CollidableReference> leaves, int capacity)
        {
            capacity = Math.Max(capacity, tree.LeafCount);
            if (tree.Leaves.Length != BufferPool.GetCapacityForCount<Leaf>(capacity))
                tree.Resize(Pool, capacity);
            if (leaves.Length != BufferPool.GetCapacityForCount<CollidableReference>(capacity))
                Pool.ResizeToAtLeast(ref leaves, capacity, tree.LeafCount);
        }
        void Dispose(ref Tree tree, ref Buffer<CollidableReference> leaves)
        {
            Pool.Return(ref leaves);
            tree.Dispose(Pool);
        }
        /// <summary>
        /// Ensures that the broad phase structures can hold at least the given number of leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void EnsureCapacity(int activeCapacity, int staticCapacity)
        {
            EnsureCapacity(ref ActiveTree, ref activeLeaves, activeCapacity);
            EnsureCapacity(ref StaticTree, ref staticLeaves, staticCapacity);
        }

        /// <summary>
        /// Resizes the broad phase structures to hold the given number of leaves. Note that this is conservative; it will never orphan any existing leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void Resize(int activeCapacity, int staticCapacity)
        {
            ResizeCapacity(ref ActiveTree, ref activeLeaves, activeCapacity);
            ResizeCapacity(ref StaticTree, ref staticLeaves, staticCapacity);
        }

        /// <summary>
        /// Releases memory used by the broad phase. Leaves the broad phase unusable.
        /// </summary>
        public void Dispose()
        {
            Dispose(ref ActiveTree, ref activeLeaves);
            Dispose(ref StaticTree, ref staticLeaves);
        }

    }
}
