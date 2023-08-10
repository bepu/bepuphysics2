using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;
using BepuPhysics.Trees;
using BepuUtilities.TaskScheduling;

namespace BepuPhysics.CollisionDetection
{
    public abstract class CollidableOverlapFinder
    {
        public abstract void DispatchOverlaps(float dt, IThreadDispatcher threadDispatcher = null);
    }

    //The overlap finder requires type knowledge about the narrow phase that the broad phase lacks. Don't really want to infect the broad phase with a bunch of narrow phase dependent 
    //generic parameters, so instead we just explicitly create a type-aware overlap finder to help the broad phase.
    public unsafe class CollidableOverlapFinder<TCallbacks> : CollidableOverlapFinder where TCallbacks : struct, INarrowPhaseCallbacks
    {
        struct SelfOverlapHandler : IOverlapHandler
        {
            public NarrowPhase<TCallbacks> NarrowPhase;
            public Buffer<CollidableReference> Leaves;
            public int WorkerIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public SelfOverlapHandler(Buffer<CollidableReference> leaves, NarrowPhase<TCallbacks> narrowPhase, int workerIndex)
            {
                Leaves = leaves;
                NarrowPhase = narrowPhase;
                WorkerIndex = workerIndex;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Handle(int indexA, int indexB)
            {
                NarrowPhase.HandleOverlap(WorkerIndex, Leaves[indexA], Leaves[indexB]);
            }
        }
        struct IntertreeOverlapHandler : IOverlapHandler
        {
            public NarrowPhase<TCallbacks> NarrowPhase;
            public Buffer<CollidableReference> LeavesA;
            public Buffer<CollidableReference> LeavesB;
            public int WorkerIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public IntertreeOverlapHandler(Buffer<CollidableReference> leavesA, Buffer<CollidableReference> leavesB, NarrowPhase<TCallbacks> narrowPhase, int workerIndex)
            {
                LeavesA = leavesA;
                LeavesB = leavesB;
                NarrowPhase = narrowPhase;
                WorkerIndex = workerIndex;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Handle(int indexA, int indexB)
            {
                NarrowPhase.HandleOverlap(WorkerIndex, LeavesA[indexA], LeavesB[indexB]);
            }
        }
        Tree.MultithreadedSelfTest<SelfOverlapHandler> selfTestContext;
        Tree.MultithreadedIntertreeTest<IntertreeOverlapHandler> intertreeTestContext;
        NarrowPhase<TCallbacks> narrowPhase;
        BroadPhase broadPhase;
        SelfOverlapHandler[] selfHandlers;
        IntertreeOverlapHandler[] intertreeHandlers;
        Action<int> workerAction;
        int nextJobIndex;
        public CollidableOverlapFinder(NarrowPhase<TCallbacks> narrowPhase, BroadPhase broadPhase)
        {
            selfTestContext = new Tree.MultithreadedSelfTest<SelfOverlapHandler>(narrowPhase.Pool);
            intertreeTestContext = new Tree.MultithreadedIntertreeTest<IntertreeOverlapHandler>(narrowPhase.Pool);
            this.narrowPhase = narrowPhase;
            this.broadPhase = broadPhase;
            workerAction = Worker;
        }

        void Worker(int workerIndex)
        {
            Debug.Assert(workerIndex >= 0 && workerIndex < intertreeHandlers.Length && workerIndex < selfHandlers.Length);

            var totalJobCount = selfTestContext.JobCount + intertreeTestContext.JobCount;
            while (true)
            {
                var jobIndex = Interlocked.Increment(ref nextJobIndex);
                if (jobIndex < selfTestContext.JobCount)
                {
                    //This is a self test job.
                    selfTestContext.ExecuteJob(jobIndex, workerIndex);
                }
                else if (jobIndex < totalJobCount)
                {
                    //This is an intertree test job.
                    intertreeTestContext.ExecuteJob(jobIndex - selfTestContext.JobCount, workerIndex);
                }
                else
                {
                    //No more jobs remain;
                    break;
                }
            }
            ref var worker = ref narrowPhase.overlapWorkers[workerIndex];
            worker.Batcher.Flush();
        }

        public override void DispatchOverlaps(float dt, IThreadDispatcher threadDispatcher = null)
        {
            DispatchOverlapsOld(dt, threadDispatcher);
        }
        public void DispatchOverlapsOld(float dt, IThreadDispatcher threadDispatcher = null)
        {
            if (threadDispatcher != null && threadDispatcher.ThreadCount > 1)
            {
                narrowPhase.Prepare(dt, threadDispatcher);
                if (intertreeHandlers == null || intertreeHandlers.Length < threadDispatcher.ThreadCount)
                {
                    //This initialization/resize should occur extremely rarely.
                    selfHandlers = new SelfOverlapHandler[threadDispatcher.ThreadCount];
                    intertreeHandlers = new IntertreeOverlapHandler[threadDispatcher.ThreadCount];
                }
                //Note that the overlap handlers are reinitialized regardless of whether the thread count changed.
                //This is just a simple way to guarantee that the most recent broad phase buffers are used- caching the buffers outside of this execution
                //would be invalid because they may get resized, invalidating the pointers.
                for (int i = 0; i < selfHandlers.Length; ++i)
                {
                    selfHandlers[i] = new SelfOverlapHandler(broadPhase.ActiveLeaves, narrowPhase, i);
                }
                for (int i = 0; i < intertreeHandlers.Length; ++i)
                {
                    intertreeHandlers[i] = new IntertreeOverlapHandler(broadPhase.ActiveLeaves, broadPhase.StaticLeaves, narrowPhase, i);
                }
                Debug.Assert(intertreeHandlers.Length >= threadDispatcher.ThreadCount);
                selfTestContext.PrepareJobs(ref broadPhase.ActiveTree, selfHandlers, threadDispatcher.ThreadCount);
                intertreeTestContext.PrepareJobs(ref broadPhase.ActiveTree, ref broadPhase.StaticTree, intertreeHandlers, threadDispatcher.ThreadCount);
                nextJobIndex = -1;
                var totalJobCount = selfTestContext.JobCount + intertreeTestContext.JobCount;
                threadDispatcher.DispatchWorkers(workerAction, totalJobCount);
                //We dispatch over parts of the tree are not yet analyzed, but the job creation phase may have put some work into the batcher.
                //If the total job count is zero, that means there's no further work to be done (implying the tree was very tiny), but we may need to flush additional jobs in worker 0.
                if (totalJobCount == 0)
                    narrowPhase.overlapWorkers[0].Batcher.Flush();
                //Any workers that we allocated resources for but did not end up using due to a lack of discovered jobs need to be cleaned up. Flushing disposes those resources.
                //(this complexity could be removed if the preparation phase was aware of the job count, but that's somewhat more difficult.)
                for (int i = Math.Max(1, totalJobCount); i < threadDispatcher.ThreadCount; ++i)
                {
                    narrowPhase.overlapWorkers[i].Batcher.Flush();
                }
#if DEBUG
                for (int i = 1; i < threadDispatcher.ThreadCount; ++i)
                {
                    Debug.Assert(!narrowPhase.overlapWorkers[i].Batcher.batches.Allocated, "After execution, there should be no remaining allocated collision batchers.");
                }
#endif
                selfTestContext.CompleteSelfTest();
                intertreeTestContext.CompleteTest();
            }
            else
            {
                narrowPhase.Prepare(dt);
                var selfTestHandler = new SelfOverlapHandler(broadPhase.ActiveLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetSelfOverlaps(ref selfTestHandler);
                var intertreeHandler = new IntertreeOverlapHandler(broadPhase.ActiveLeaves, broadPhase.StaticLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetOverlaps(ref broadPhase.StaticTree, ref intertreeHandler);
                narrowPhase.overlapWorkers[0].Batcher.Flush();

            }

        }

        struct ThreadedIntertreeOverlapHandler : IThreadedOverlapHandler
        {
            public Buffer<CollidableReference> ActiveLeaves;
            public Buffer<CollidableReference> StaticLeaves;
            public void Handle(int indexA, int indexB, int workerIndex, object managedContext)
            {
                var narrowPhase = (NarrowPhase<TCallbacks>)managedContext;
                narrowPhase.HandleOverlap(workerIndex, ActiveLeaves[indexA], StaticLeaves[indexB]);
            }
        }
        struct ThreadedSelfOverlapHandler : IThreadedOverlapHandler
        {
            public Buffer<CollidableReference> Leaves;
            public void Handle(int indexA, int indexB, int workerIndex, object managedContext)
            {
                //var narrowPhase = (NarrowPhase<TCallbacks>)managedContext;
                var narrowPhase = Unsafe.As<NarrowPhase<TCallbacks>>(managedContext);
                narrowPhase.HandleOverlap(workerIndex, Leaves[indexA], Leaves[indexB]);
            }
        }

        unsafe struct SelfContext
        {
            public TaskStack* Stack;
            public ThreadedSelfOverlapHandler* Results;
            public Tree Tree;
            public int TargetTaskCount;
        }
        unsafe struct IntertreeContext
        {
            public TaskStack* Stack;
            public ThreadedIntertreeOverlapHandler* Results;
            public Tree StaticTree;
            public Tree ActiveTree;
            public int TargetTaskCount;
        }
        unsafe static void SelfEntryTask(long taskStartAndEnd, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
        {
            Debug.Assert(dispatcher.ManagedContext != null);
            ref var context = ref *(SelfContext*)untypedContext;
            var pool = dispatcher.WorkerPools[workerIndex];
            context.Tree.GetSelfOverlaps2(ref *context.Results, pool, dispatcher, context.Stack, workerIndex, context.TargetTaskCount);
        }
        unsafe static void IntertreeEntryTask(long taskStartAndEnd, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
        {
            Debug.Assert(dispatcher.ManagedContext != null);
            ref var context = ref *(IntertreeContext*)untypedContext;
            var pool = dispatcher.WorkerPools[workerIndex];
            context.ActiveTree.GetOverlaps2(ref context.StaticTree, ref *context.Results, pool, dispatcher, context.Stack, workerIndex, context.TargetTaskCount);
        }

        public static void WorkerTask(int workerIndex, IThreadDispatcher dispatcher)
        {
            Debug.Assert(dispatcher.ManagedContext != null);
            var taskStack = (TaskStack*)dispatcher.UnmanagedContext;
            PopTaskResult popTaskResult;
            var waiter = new SpinWait();
            while ((popTaskResult = taskStack->TryPopAndRun(workerIndex, dispatcher)) != PopTaskResult.Stop)
            {
                waiter.SpinOnce(-1);
            }
            ((NarrowPhase<TCallbacks>)dispatcher.ManagedContext).overlapWorkers[workerIndex].Batcher.Flush();
        }
        public void DispatchOverlapsNew(float dt, IThreadDispatcher threadDispatcher = null)
        {
            //The number of collisions is usually some constant multiple of the number of active leaves.
            //5-10 collisions per active leaf would represent a very dense simulation.
            //The number of collisions needed to warrant the existence of a thread varies, too, but it's exceptionally unlikely that any you would need more than one thread for 64 collisions.
            //(Those would have to be some very messed up pairs!)
            //So, we'll allow worker counts to scale down with low leaf counts to avoid overhead for tiny simulations.
            var maximumWorkerCount = int.Min(int.Max(1, broadPhase.ActiveTree.LeafCount / 8), threadDispatcher == null ? 1 : threadDispatcher.ThreadCount);
            if (maximumWorkerCount > 1)
            {
                narrowPhase.Prepare(dt, threadDispatcher);

                if (broadPhase.ActiveTree.LeafCount > 0 && broadPhase.StaticTree.LeafCount > 0)
                {
                    var selfResults = new ThreadedSelfOverlapHandler { Leaves = broadPhase.ActiveLeaves };
                    var intertreeResults = new ThreadedIntertreeOverlapHandler { ActiveLeaves = broadPhase.ActiveLeaves, StaticLeaves = broadPhase.StaticLeaves };

                    //Note that we shouldn't reduce the task budget for testing with smallish trees, because it's very possible for the cost of individual narrow phase pairs to be high.
                    //The tree test cost is often small compared to the narrow phase costs, so it's worth keeping quite a few tasks around.
                    var selfTestCostEstimate = broadPhase.ActiveTree.LeafCount;
                    var intertreeTestCostEstimate = int.Max(broadPhase.ActiveTree.LeafCount, broadPhase.StaticTree.LeafCount);

                    var selfTestAsFraction = (float)selfTestCostEstimate / (selfTestCostEstimate + intertreeTestCostEstimate);
                    //Regularize the budgets a bit. Don't let either get too small.
                    const float minimumBudget = 0.25f;
                    var intertreeTestAsFraction = 1f - (selfTestAsFraction * (1f - minimumBudget) + minimumBudget);
                    selfTestAsFraction = 1f - intertreeTestAsFraction;

                    var selfTestTaskTarget = (int)float.Round(maximumWorkerCount * selfTestAsFraction);
                    var intertreeTestTaskTarget = maximumWorkerCount - selfTestTaskTarget;
                    var taskStack = new TaskStack(broadPhase.Pool, threadDispatcher, maximumWorkerCount);
                    var selfContext = new SelfContext { Results = &selfResults, Stack = &taskStack, TargetTaskCount = selfTestTaskTarget, Tree = broadPhase.ActiveTree };
                    var intertreeContext = new IntertreeContext { Results = &intertreeResults, Stack = &taskStack, TargetTaskCount = intertreeTestTaskTarget, ActiveTree = broadPhase.ActiveTree, StaticTree = broadPhase.StaticTree };
                    Span<Task> tasks = stackalloc Task[2];
                    tasks[0] = new Task(&SelfEntryTask, &selfContext);
                    tasks[1] = new Task(&IntertreeEntryTask, &intertreeContext);
                    taskStack.AllocateContinuationAndPush(tasks, 0, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(&taskStack));
                    threadDispatcher.DispatchWorkers(&WorkerTask, maximumWorkerCount: maximumWorkerCount, unmanagedContext: &taskStack, managedContext: narrowPhase);
                    taskStack.Dispose(broadPhase.Pool, threadDispatcher);

                    //var intertreeResults = new ThreadedIntertreeOverlapHandler { ActiveLeaves = broadPhase.ActiveLeaves, StaticLeaves = broadPhase.StaticLeaves };
                    //var taskStack = new TaskStack(broadPhase.Pool, threadDispatcher, maximumWorkerCount);
                    //var intertreeContext = new IntertreeContext { Results = &intertreeResults, Stack = &taskStack, TargetTaskCount = maximumWorkerCount, ActiveTree = broadPhase.ActiveTree, StaticTree = broadPhase.StaticTree };
                    //taskStack.AllocateContinuationAndPush(new Task(&IntertreeEntryTask, &intertreeContext), 0, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(&taskStack));
                    //threadDispatcher.DispatchWorkers(&WorkerTask, maximumWorkerCount: maximumWorkerCount, unmanagedContext: &taskStack, managedContext: narrowPhase);
                    //taskStack.Dispose(broadPhase.Pool, threadDispatcher);
                }
                else if (broadPhase.ActiveTree.LeafCount > 0)
                {
                    var selfResults = new ThreadedSelfOverlapHandler { Leaves = broadPhase.ActiveLeaves };
                    var taskStack = new TaskStack(broadPhase.Pool, threadDispatcher, maximumWorkerCount);
                    var selfContext = new SelfContext { Results = &selfResults, Stack = &taskStack, TargetTaskCount = maximumWorkerCount, Tree = broadPhase.ActiveTree };
                    taskStack.AllocateContinuationAndPush(new Task(&SelfEntryTask, &selfContext), 0, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(&taskStack));
                    threadDispatcher.DispatchWorkers(&WorkerTask, maximumWorkerCount: maximumWorkerCount, unmanagedContext: &taskStack, managedContext: narrowPhase);
                    taskStack.Dispose(broadPhase.Pool, threadDispatcher);
                }
#if DEBUG
                for (int i = 1; i < threadDispatcher.ThreadCount; ++i)
                {
                    Debug.Assert(!narrowPhase.overlapWorkers[i].Batcher.batches.Allocated, "After execution, there should be no remaining allocated collision batchers.");
                }
#endif
            }
            else
            {
                narrowPhase.Prepare(dt);
                var selfTestHandler = new SelfOverlapHandler(broadPhase.ActiveLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetSelfOverlaps2(ref selfTestHandler);
                var intertreeHandler = new IntertreeOverlapHandler(broadPhase.ActiveLeaves, broadPhase.StaticLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetOverlaps(ref broadPhase.StaticTree, ref intertreeHandler);
                narrowPhase.overlapWorkers[0].Batcher.Flush();

            }

        }

    }
}

