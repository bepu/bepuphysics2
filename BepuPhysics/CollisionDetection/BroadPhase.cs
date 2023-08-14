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
using BepuUtilities.TaskScheduling;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Manages scene acceleration structures for collision detection and queries.
    /// </summary>
    public unsafe partial class BroadPhase : IDisposable
    {
        /// <summary>
        /// Collidable references contained within the <see cref="ActiveTree"/>. Note that values at or beyond the <see cref="ActiveTree"/>.LeafCount are not defined.
        /// </summary>
        public Buffer<CollidableReference> ActiveLeaves;
        /// <summary>
        /// Collidable references contained within the <see cref="StaticTree"/>. Note that values at or beyond <see cref="StaticTree"/>.LeafCount are not defined.
        /// </summary>
        public Buffer<CollidableReference> StaticLeaves;
        /// <summary>
        /// Pool used by the broad phase.
        /// </summary>
        public BufferPool Pool { get; private set; }
        /// <summary>
        /// Tree containing wakeful bodies.
        /// </summary>
        public Tree ActiveTree;
        /// <summary>
        /// Tree containing sleeping bodies and statics.
        /// </summary>
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
            pool.TakeAtLeast(initialActiveLeafCapacity, out ActiveLeaves);
            pool.TakeAtLeast(initialStaticLeafCapacity, out StaticLeaves);

            activeRefineContext = new Tree.RefitAndRefineMultithreadedContext();
            staticRefineContext = new Tree.RefitAndRefineMultithreadedContext();
            executeRefitAndMarkAction = ExecuteRefitAndMark;
            executeRefineAction = ExecuteRefine;

            ActiveRefinementSchedule = DefaultActiveRefinementScheduler;
            StaticRefinementSchedule = DefaultStaticRefinementScheduler;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int Add(CollidableReference collidable, ref BoundingBox bounds, ref Tree tree, BufferPool pool, ref Buffer<CollidableReference> leaves)
        {
            var leafIndex = tree.Add(bounds, pool);
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
            return Add(collidable, ref bounds, ref ActiveTree, Pool, ref ActiveLeaves);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveActiveAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref ActiveTree, ref ActiveLeaves, out movedLeaf);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int AddStatic(CollidableReference collidable, ref BoundingBox bounds)
        {
            return Add(collidable, ref bounds, ref StaticTree, Pool, ref StaticLeaves);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveStaticAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref StaticTree, ref StaticLeaves, out movedLeaf);
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
            var threadPool = threadDispatcher.WorkerPools[workerIndex];
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
            var threadPool = threadDispatcher.WorkerPools[workerIndex];
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
        /// Returns the size and number of refinements to execute during the broad phase.
        /// </summary>
        /// <param name="frameIndex">Index of the frame as tracked by the broad phase.</param>
        /// <param name="tree">Tree being considered for refinement.</param>
        /// <param name="rootRefinementSize">Size of the root refinement. If zero or negative, no root refinement will be performed.</param>
        /// <param name="subtreeRefinementCount">Number of subtree refinements to perform. Can be zero.</param>
        /// <param name="subtreeRefinementSize">Target size of the subtree refinements.</param>
        /// <param name="usePriorityQueue">True if the root refinement should use a priority queue during subtree collection to find larger nodes, false if it should try to collect a more balanced tree.</param>
        public delegate void RefinementScheduler(int frameIndex, in Tree tree, out int rootRefinementSize, out int subtreeRefinementCount, out int subtreeRefinementSize, out bool usePriorityQueue);

        /// <summary>
        /// Gets or sets the refinement schedule to use for the active tree.
        /// </summary>
        public RefinementScheduler ActiveRefinementSchedule { get; set; }
        /// <summary>
        /// Gets or sets the refinement schedule to use for the static tree.
        /// </summary>
        public RefinementScheduler StaticRefinementSchedule { get; set; }

        /// <summary>
        /// Returns the size and number of refinements to execute for the active tree. Used by default.
        /// </summary>
        /// <param name="optimizationFraction">Target fraction of the tree to be optimized.</param>
        /// <param name="rootRefinementPeriod">Period, in timesteps, of refinements applied to the root.</param>
        /// <param name="rootRefinementSizeScale">Multiplier to apply to the square root of the leaf count to get the target root refinement size.</param>
        /// <param name="subtreeRefinementSizeScale">Multiplier to apply to the square root of the leaf count to get the target subtree refinement size.</param>
        /// <param name="nonpriorityPeriod">The period between non-priority queue based root refinements, measured in units of root refinements.</param>
        /// <param name="frameIndex">Index of the frame as tracked by the broad phase.</param>
        /// <param name="tree">Tree being considered for refinement.</param>
        /// <param name="rootRefinementSize">Size of the root refinement. If zero or negative, no root refinement will be performed.</param>
        /// <param name="subtreeRefinementCount">Number of subtree refinements to perform. Can be zero.</param>
        /// <param name="subtreeRefinementSize">Target size of the subtree refinements.</param>
        /// <param name="usePriorityQueue">True if the root refinement should use a priority queue during subtree collection to find larger nodes, false if it should try to collect a more balanced tree.</param>
        public static void DefaultRefinementScheduler(float optimizationFraction, int rootRefinementPeriod, float rootRefinementSizeScale, float subtreeRefinementSizeScale, int nonpriorityPeriod,
           int frameIndex, in Tree tree, out int rootRefinementSize, out int subtreeRefinementCount, out int subtreeRefinementSize, out bool usePriorityQueue)
        {
            var refineRoot = frameIndex % rootRefinementPeriod == 0;
            var targetOptimizedLeafCount = (int)float.Ceiling(tree.LeafCount * optimizationFraction);
            //The square root of the leaf count gets us roughly halfway down the tree. (Each subtree has ~sqrt(LeafCount) leaves, and there are ~sqrt(LeafCount) subtrees.)
            //Root and subtree refinements need to be larger than that: subtrees must be able to exchange nodes, and the root refinement is the intermediary.
            //Another consideration for choosing refinement sizes: larger refinement sizes increase in cost nonlinearly (O(nlogn)).
            //You should choose a size which is large *enough* to get within your quality target and no larger.
            var sqrtLeafCount = float.Sqrt(tree.LeafCount);

            var targetRootRefinementSize = (int)float.Ceiling(sqrtLeafCount * rootRefinementSizeScale);
            subtreeRefinementSize = (int)float.Ceiling(sqrtLeafCount * subtreeRefinementSizeScale);

            //Note that we scale up the cost of the root refinement; it uses a sequentialized priority queue to collect subtrees for refinement and costs more.
            var subtreeRefinementsPerRootRefinement = (int)float.Ceiling(subtreeRefinementSize * float.Log2(subtreeRefinementSize) / (targetRootRefinementSize * float.Log2(targetRootRefinementSize)));
            //If we're refining the root, reduce the number of subtree refinements to avoid cost spikes.
            subtreeRefinementCount = int.Max(0, (int)float.Ceiling((float)targetOptimizedLeafCount / subtreeRefinementSize) - (refineRoot ? subtreeRefinementsPerRootRefinement : 0));

            rootRefinementSize = refineRoot ? targetRootRefinementSize : 0;
            usePriorityQueue = (frameIndex / rootRefinementPeriod) % nonpriorityPeriod != 0;
        }

        /// <summary>
        /// Returns the size and number of refinements to execute for the active tree. Used by default.
        /// </summary>
        /// <param name="frameIndex">Index of the frame as tracked by the broad phase.</param>
        /// <param name="tree">Tree being considered for refinement.</param>
        /// <param name="rootRefinementSize">Size of the root refinement. If zero or negative, no root refinement will be performed.</param>
        /// <param name="subtreeRefinementCount">Number of subtree refinements to perform. Can be zero.</param>
        /// <param name="subtreeRefinementSize">Target size of the subtree refinements.</param>
        /// <param name="usePriorityQueue">True if the root refinement should use a priority queue during subtree collection to find larger nodes, false if it should try to collect a more balanced tree.</param>
        public static void DefaultActiveRefinementScheduler(int frameIndex, in Tree tree, out int rootRefinementSize, out int subtreeRefinementCount, out int subtreeRefinementSize, out bool usePriorityQueue)
        {
            DefaultRefinementScheduler(1f / 20f, 2, 1, 4, 16, frameIndex, tree, out rootRefinementSize, out subtreeRefinementCount, out subtreeRefinementSize, out usePriorityQueue);
        }


        /// <summary>
        /// Returns the size and number of refinements to execute for the active tree. Used by default.
        /// </summary>
        /// <param name="frameIndex">Index of the frame as tracked by the broad phase.</param>
        /// <param name="tree">Tree being considered for refinement.</param>
        /// <param name="rootRefinementSize">Size of the root refinement. If zero or negative, no root refinement will be performed.</param>
        /// <param name="subtreeRefinementCount">Number of subtree refinements to perform. Can be zero.</param>
        /// <param name="subtreeRefinementSize">Target size of the subtree refinements.</param>
        /// <param name="usePriorityQueue">True if the root refinement should use a priority queue during subtree collection to find larger nodes, false if it should try to collect a more balanced tree.</param>
        public static void DefaultStaticRefinementScheduler(int frameIndex, in Tree tree, out int rootRefinementSize, out int subtreeRefinementCount, out int subtreeRefinementSize, out bool usePriorityQueue)
        {
            DefaultRefinementScheduler(1f / 100f, 2, 1, 4, 16, frameIndex, tree, out rootRefinementSize, out subtreeRefinementCount, out subtreeRefinementSize, out usePriorityQueue);
        }

        struct RefinementContext
        {
            public TaskStack* TaskStack;
            public Tree Tree;
            public int TargetTaskCount;
            public int TargetTotalTaskCount;
            public int RootRefinementSize;
            public int SubtreeRefinementCount;
            public int SubtreeRefinementSize;
            public int SubtreeRefinementStartIndex;
            public bool Deterministic;
            public bool UsePriorityQueue;

            //Used for active tree. Didn't split the type because whocares.
            public Buffer<Node> TargetNodes;
        }

        static void ActiveEntrypointTask(long taskId, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var context = ref *(RefinementContext*)untypedContext;
            var pool = dispatcher.WorkerPools[workerIndex];
            context.Tree.Refine2(context.RootRefinementSize, ref context.SubtreeRefinementStartIndex, context.SubtreeRefinementCount, context.SubtreeRefinementSize, pool, dispatcher, context.TaskStack, workerIndex, targetTaskCount: context.TargetTaskCount, deterministic: context.Deterministic, usePriorityQueue: context.UsePriorityQueue);
            //Now refit! Note that we use all but one task. It doesn't affect the performance of a refit much (we're not compute bound), and we can use it to do an incremental cache optimization on the static tree.
            var sourceNodes = context.Tree.Nodes;
            context.Tree.Nodes = context.TargetNodes;
            context.Tree.Refit2WithCacheOptimization(sourceNodes, pool, dispatcher, context.TaskStack, workerIndex, context.TargetTotalTaskCount);
        }

        static void StaticEntrypointTask(long taskId, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var context = ref *(RefinementContext*)untypedContext;
            var pool = dispatcher.WorkerPools[workerIndex];
            context.Tree.Refine2(context.RootRefinementSize, ref context.SubtreeRefinementStartIndex, context.SubtreeRefinementCount, context.SubtreeRefinementSize, pool, dispatcher, context.TaskStack, workerIndex, targetTaskCount: context.TargetTaskCount, deterministic: context.Deterministic, usePriorityQueue: context.UsePriorityQueue);
        }

        int staticSubtreeRefinementStartIndex, activeSubtreeRefinementStartIndex;
        public void Update2(IThreadDispatcher threadDispatcher = null, bool deterministic = false)
        {
            ActiveRefinementSchedule(frameIndex, ActiveTree, out var activeRootRefinementSize, out var activeSubtreeRefinementCount, out var activeSubtreeRefinementSize, out var usePriorityQueueActive);
            StaticRefinementSchedule(frameIndex, StaticTree, out var staticRootRefinementSize, out var staticSubtreeRefinementCount, out var staticSubtreeRefinementSize, out var usePriorityQueueStatic);
            Console.WriteLine($"root size: {activeRootRefinementSize}, subtree count: {activeSubtreeRefinementCount}, subtree size: {activeSubtreeRefinementSize}, usePQ: {usePriorityQueueActive}");
            const int minimumLeafCountForThreading = 256;
            if (threadDispatcher != null && threadDispatcher.ThreadCount > 1 && (ActiveTree.LeafCount >= minimumLeafCountForThreading || StaticTree.LeafCount >= minimumLeafCountForThreading))
            {
                //Distribute tasks for refinement roughly in proportion to their cost.
                //This doesn't need to be perfect.
                //Cost of a refinement is roughly n * log2(n), for n = refinement size.
                var activeCost = float.Log2(activeRootRefinementSize + 1) * activeRootRefinementSize + float.Log2(activeSubtreeRefinementSize + 1) * activeSubtreeRefinementSize * activeSubtreeRefinementCount;
                var staticCost = float.Log2(staticRootRefinementSize + 1) * staticRootRefinementSize + float.Log2(staticSubtreeRefinementSize + 1) * staticSubtreeRefinementSize * staticSubtreeRefinementCount;
                var activeTaskFraction = activeCost / (activeCost + staticCost);
                var targetTotalTaskCount = threadDispatcher.ThreadCount; //could scale this. Empirically, doesn't matter on the CPUs tested so far.
                var targetActiveTaskCount = (int)float.Ceiling(activeTaskFraction * targetTotalTaskCount);
                var taskStack = new TaskStack(Pool, threadDispatcher, threadDispatcher.ThreadCount);
                var activeRefineContext = new RefinementContext
                {
                    TaskStack = &taskStack,
                    Tree = ActiveTree,
                    TargetTotalTaskCount = targetTotalTaskCount,
                    TargetTaskCount = targetActiveTaskCount,
                    RootRefinementSize = activeRootRefinementSize,
                    SubtreeRefinementCount = activeSubtreeRefinementCount,
                    SubtreeRefinementSize = activeSubtreeRefinementSize,
                    SubtreeRefinementStartIndex = activeSubtreeRefinementStartIndex,
                    Deterministic = deterministic,
                    UsePriorityQueue = usePriorityQueueActive,
                    TargetNodes = ActiveTree.LeafCount > 2 ? new Buffer<Node>(ActiveTree.Nodes.Length, Pool) : default,
                };
                var staticRefineContext = new RefinementContext
                {
                    TaskStack = &taskStack,
                    Tree = StaticTree,
                    TargetTotalTaskCount = targetTotalTaskCount,
                    TargetTaskCount = targetTotalTaskCount - targetActiveTaskCount,
                    RootRefinementSize = staticRootRefinementSize,
                    SubtreeRefinementCount = staticSubtreeRefinementCount,
                    SubtreeRefinementSize = staticSubtreeRefinementSize,
                    SubtreeRefinementStartIndex = staticSubtreeRefinementStartIndex,
                    Deterministic = deterministic,
                    UsePriorityQueue = usePriorityQueueStatic,
                };
                Span<Task> tasks = stackalloc Task[2];
                tasks[0] = new Task(&ActiveEntrypointTask, &activeRefineContext);
                tasks[1] = new Task(&StaticEntrypointTask, &staticRefineContext);
                taskStack.AllocateContinuationAndPush(tasks, 0, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(&taskStack));
                TaskStack.DispatchWorkers(threadDispatcher, &taskStack);
                taskStack.Dispose(Pool, threadDispatcher);
                if (ActiveTree.LeafCount > 2) //If no refit was needed, then the target nodes buffer was never allocated.
                {
                    //When using the cache optimizing refit, the tree is modified. Since passed a copy, we need to copy it back.
                    //Static tree doesn't undergo a refit, so no copy required there.
                    ActiveTree.Nodes.Dispose(Pool);
                    ActiveTree.Nodes = activeRefineContext.TargetNodes;
                }
                //The start indices need to be copied back for both.
                activeSubtreeRefinementStartIndex = activeRefineContext.SubtreeRefinementStartIndex;
                staticSubtreeRefinementStartIndex = staticRefineContext.SubtreeRefinementStartIndex;

            }
            else
            {
                StaticTree.Refine2(staticRootRefinementSize, ref staticSubtreeRefinementStartIndex, staticSubtreeRefinementCount, staticSubtreeRefinementSize, Pool);
                //Note we refine *before* refitting. This means the refinement is working with slightly out of date data, but that's okay, the entire point is incremental refinement.
                //The reason to prefer this is that refining scrambles the memory layout a little bit.
                //Refit with cache optimization *after* refinement ensures the rest of the library (and the user) sees the cache optimized version.
                ActiveTree.Refine2(activeRootRefinementSize, ref activeSubtreeRefinementStartIndex, activeSubtreeRefinementCount, activeSubtreeRefinementSize, Pool);
                ActiveTree.Refit2WithCacheOptimization(Pool);
            }
            if (frameIndex == int.MaxValue)
                frameIndex = 0;
            else
                ++frameIndex;
            //StaticTree.Validate();
            //ActiveTree.Validate();
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
            EnsureCapacity(ref ActiveTree, ref ActiveLeaves, activeCapacity);
            EnsureCapacity(ref StaticTree, ref StaticLeaves, staticCapacity);
        }

        /// <summary>
        /// Resizes the broad phase structures to hold the given number of leaves. Note that this is conservative; it will never orphan any existing leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void Resize(int activeCapacity, int staticCapacity)
        {
            ResizeCapacity(ref ActiveTree, ref ActiveLeaves, activeCapacity);
            ResizeCapacity(ref StaticTree, ref StaticLeaves, staticCapacity);
        }

        /// <summary>
        /// Releases memory used by the broad phase. Leaves the broad phase unusable.
        /// </summary>
        public void Dispose()
        {
            Dispose(ref ActiveTree, ref ActiveLeaves);
            Dispose(ref StaticTree, ref StaticLeaves);
        }

    }
}
