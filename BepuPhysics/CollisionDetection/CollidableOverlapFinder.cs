using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;
using BepuPhysics.Trees;

namespace BepuPhysics.CollisionDetection
{
    public abstract class CollidableOverlapFinder
    {
        public abstract void DispatchOverlaps(float dt, IThreadDispatcher threadDispatcher = null);
    }

    //The overlap finder requires type knowledge about the narrow phase that the broad phase lacks. Don't really want to infect the broad phase with a bunch of narrow phase dependent 
    //generic parameters, so instead we just explicitly create a type-aware overlap finder to help the broad phase.
    public class CollidableOverlapFinder<TCallbacks> : CollidableOverlapFinder where TCallbacks : struct, INarrowPhaseCallbacks
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
                    selfHandlers[i] = new SelfOverlapHandler(broadPhase.activeLeaves, narrowPhase, i);
                }
                for (int i = 0; i < intertreeHandlers.Length; ++i)
                {
                    intertreeHandlers[i] = new IntertreeOverlapHandler(broadPhase.activeLeaves, broadPhase.staticLeaves, narrowPhase, i);
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
                var selfTestHandler = new SelfOverlapHandler(broadPhase.activeLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetSelfOverlaps(ref selfTestHandler);
                var intertreeHandler = new IntertreeOverlapHandler(broadPhase.activeLeaves, broadPhase.staticLeaves, narrowPhase, 0);
                broadPhase.ActiveTree.GetOverlaps(ref broadPhase.StaticTree, ref intertreeHandler);
                narrowPhase.overlapWorkers[0].Batcher.Flush();

            }

        }

    }
}

