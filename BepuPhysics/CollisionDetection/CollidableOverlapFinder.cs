using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;
using BepuPhysics.Trees;
using BepuUtilities.TaskScheduling;
using BepuUtilities.Collections;
using System.Runtime.InteropServices;

namespace BepuPhysics.CollisionDetection;

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
        //public int DebugCount;
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
            //++DebugCount;
            NarrowPhase.HandleOverlap(WorkerIndex, Leaves[indexA], Leaves[indexB]);
        }
    }
    struct IntertreeOverlapHandler : IOverlapHandler
    {
        public NarrowPhase<TCallbacks> NarrowPhase;
        public Buffer<CollidableReference> LeavesA;
        public Buffer<CollidableReference> LeavesB;
        public int WorkerIndex;
        //public int DebugCount;
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
            //++DebugCount;
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
        selfTestContext = new Tree.MultithreadedSelfTest<SelfOverlapHandler>();
        intertreeTestContext = new Tree.MultithreadedIntertreeTest<IntertreeOverlapHandler>();
        this.narrowPhase = narrowPhase;
        this.broadPhase = broadPhase;
        workerAction = Worker;

        //VERSION 2
        selfTestContext2 = new Tree.MultithreadedSelfTest<PairCollector>();
        intertreeTestContext2 = new Tree.MultithreadedIntertreeTest<PairCollector>();

        //VERSION 3
        selfTestContext3 = new Tree.MultithreadedSelfTest<PairCollector3>();
        intertreeTestContext3 = new Tree.MultithreadedIntertreeTest<PairCollector3>();
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
        DispatchOverlaps3(dt, threadDispatcher);
        //DispatchOverlaps2(dt, threadDispatcher);
        return;
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
            selfTestContext.PrepareJobs(ref broadPhase.ActiveTree, selfHandlers, threadDispatcher.ThreadCount, 0, narrowPhase.Pool);
            intertreeTestContext.PrepareJobs(ref broadPhase.ActiveTree, ref broadPhase.StaticTree, intertreeHandlers, threadDispatcher.ThreadCount, 0, narrowPhase.Pool);
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
            //var sum = 0;
            //for (int i = 0; i < selfHandlers.Length; ++i)
            //{
            //    sum += selfHandlers[i].DebugCount;
            //    sum += intertreeHandlers[i].DebugCount;
            //}
            //var min = 1.0;
            //var minIndex = 0;
            //var max = 0.0;
            //var maxIndex = 0;
            //for (int i = 0; i < selfHandlers.Length; ++i)
            //{
            //    var threadFraction = (double)(selfHandlers[i].DebugCount + intertreeHandlers[i].DebugCount) / sum;
            //    if (threadFraction > max)
            //    {
            //        max = threadFraction;
            //        maxIndex = i;
            //    }
            //    if (threadFraction < min)
            //    {
            //        min = threadFraction;
            //        minIndex = i;
            //    }
            //}
            //Console.WriteLine();
            //Console.WriteLine($"min: {minIndex}, {min}");
            //Console.WriteLine($"max: {maxIndex}, {max}");

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

    /// <summary>
    /// Stores a list of elements in a chunked format that only allows adding new elements. Avoids resizing and copying when the list grows.
    /// </summary>
    /// <typeparam name="T">Unmanaged type contained by the list.</typeparam>
    public struct ChunkedList<T> where T : unmanaged
    {
        /// <summary>
        /// Contiguous chunks of elements held by the list.
        /// </summary>
        public QuickList<QuickList<T>> Chunks;

        /// <summary>
        /// Gets the total count within the list.
        /// </summary>
        public int Count
        {
            get
            {
                int count = 0;
                for (int i = 0; i < Chunks.Count; ++i)
                {
                    count += Chunks[i].Count;
                }
                return count;
            }
        }

        /// <summary>
        /// Maximum number of chunks that can be created within a chunked list. Each chunk created is twice the size of the previous.
        /// </summary>
        const int MaximumChunkCount = 32;
        public ChunkedList(BufferPool pool, int initialCapacityWithinChunk)
        {
            Chunks = new QuickList<QuickList<T>>(MaximumChunkCount, pool);
            Chunks.AllocateUnsafely() = new QuickList<T>(initialCapacityWithinChunk, pool);
        }

        /// <summary>
        /// Allocates a new element within the chunked list. If a new chunk must be allocated, it will be twice the size of the previous chunk.
        /// </summary>
        /// <param name="pool">Pool to allocate a chunk from if necessary.</param>
        /// <returns>Reference to the allocated value.</returns>
        public ref T Allocate(BufferPool pool)
        {
            ref var chunk = ref Chunks[Chunks.Count - 1];
            if (chunk.Count == chunk.Span.Length)
            {
                Chunks.AllocateUnsafely() = new QuickList<T>(chunk.Span.Length * 2, pool);
                chunk = ref Chunks[Chunks.Count - 1];
            }
            return ref chunk.AllocateUnsafely();
        }

        /// <summary>
        /// Disposes all unmanaged memory allocated by the list.
        /// </summary>
        /// <param name="pool">Pool to return the memory to.</param>
        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < Chunks.Count; ++i)
            {
                Chunks[i].Dispose(pool);
            }
            Chunks.Dispose(pool);
        }
    }

    /// <summary>
    /// Collects pairs of leaf indices which overlap for a given thread in the broad phase test.
    /// </summary>
    unsafe struct PairCollector : IOverlapHandler
    {
        public BufferPool ThreadPool;
        public ChunkedList<CollidablePair>* Pairs;
        public Buffer<CollidableReference> LeavesA;
        public Buffer<CollidableReference> LeavesB;

        public PairCollector(BufferPool threadPool, ChunkedList<CollidablePair>* pairs, Buffer<CollidableReference> leavesA, Buffer<CollidableReference> leavesB)
        {
            ThreadPool = threadPool;
            Pairs = pairs;
            LeavesA = leavesA;
            LeavesB = leavesB;
        }
        public void Handle(int indexA, int indexB)
        {
            Pairs->Allocate(ThreadPool) = new CollidablePair(LeavesA[indexA], LeavesB[indexB]);
        }
    }

    Buffer<ChunkedList<CollidablePair>> threadPairs;
    PairCollector[] selfTestHandlers, intertreeTestHandlers;
    int targetChunkCapacity;
    int previousPairCount;
    Tree.MultithreadedSelfTest<PairCollector> selfTestContext2;
    Tree.MultithreadedIntertreeTest<PairCollector> intertreeTestContext2;
    //Buffer<int> debugPairsExecutedPerThread;
    PaddedCounter jobCounter;


    unsafe static void SelfTestJob(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        overlapFinder.selfTestContext2.ExecuteJob((int)id, workerIndex);
    }
    unsafe static void IntertreeTestJob(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        overlapFinder.intertreeTestContext2.ExecuteJob((int)id, workerIndex);
    }
    unsafe static void PushNarrowPhaseTests(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        //All tree jobs are complete; the handlers contain all requisite information to dispatch narrow phase tests.

        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        var totalPairCount = 0;
        for (int i = 0; i < overlapFinder.threadPairs.Length; ++i)
        {
            totalPairCount += overlapFinder.threadPairs[i].Count;
        }
        overlapFinder.previousPairCount = totalPairCount;
        //Console.WriteLine($"pair count pre: {totalPairCount}");
        //We'd like to pick a job size which is meaty enough to avoid excessive overhead, but not so large that it blocks load balancing.
        //So, base it on the number of pairs we actually have.
        var jobSize = int.Max(1, totalPairCount / (threadDispatcher.ThreadCount * 8));
        var taskCount = int.Min(totalPairCount, threadDispatcher.ThreadCount);

        var taskStack = (TaskStack*)context;
        if (taskCount > 0)
        {
            //Console.WriteLine($"pushing {taskCount} tasks from {workerIndex}");
            Span<Task> tasks = stackalloc Task[taskCount];
            var task = new Task(&NarrowPhaseJob, taskStack, jobSize);
            for (int i = 0; i < tasks.Length; ++i)
            {
                tasks[i] = task;
            }
            taskStack->AllocateContinuationAndPush(tasks, workerIndex, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(taskStack));
        }
        else
        {
            taskStack->RequestStop();
        }
    }


    unsafe static void NarrowPhaseJob(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        //The counter is a ulong encoded as (0 is LSB):
        //[0, 32): pair index within chunk
        //[32, 37): chunk index within chunked list
        //[37, 64): thread index
        //We encoded the job size in the id.
        var jobSize = (int)id;
        ref var counter = ref overlapFinder.jobCounter.Counter;
        int debugLocalPairsExecuted = 0;
        while (true)
        {
            //Try claiming a block of work.
            var currentValue = counter; //CompareExchange has a full fence.
            var pairIndex = (int)(currentValue & 0xFFFF_FFFF);
            var chunkIndex = (int)((currentValue >> 32) & 0x1F);
            var threadIndex = (int)(currentValue >> 37);
            ref var threadChunks = ref overlapFinder.threadPairs[threadIndex];
            ref var chunk = ref threadChunks.Chunks[chunkIndex];
            if (pairIndex == chunk.Count)
            {
                //We've exhausted the chunk. Move on to the next one.
                ++chunkIndex;
                if (chunkIndex == threadChunks.Chunks.Count)
                {
                    //We've exhausted the chunks in this thread. Move to the next thread.
                    //Note that if the thread is empty, we'll try the next one rather than pointlessly doing a compare exchange on an empty set.
                    chunkIndex = 0;
                    for (threadIndex += 1; threadIndex < overlapFinder.threadPairs.Length; ++threadIndex)
                    {
                        threadChunks = ref overlapFinder.threadPairs[threadIndex];
                        if (threadChunks.Chunks[0].Count > 0)
                        {
                            //We found a thread (and chunk) with possibly available work.
                            break;
                        }
                    }
                    if (threadIndex == overlapFinder.threadPairs.Length)
                    {
                        //We've exhausted all pairs in all chunks in all threads. We're done.
                        break;
                    }
                    threadChunks = ref overlapFinder.threadPairs[threadIndex];
                }
                chunk = ref threadChunks.Chunks[chunkIndex];
                pairIndex = 0;
            }
            //Eat part of the chunk, up to the end.
            var startPairIndex = pairIndex;
            pairIndex = int.Min(pairIndex + jobSize, chunk.Count);

            var newCounter = (long)threadIndex << 37 | (long)chunkIndex << 32 | (long)pairIndex;
            if (Interlocked.CompareExchange(ref counter, newCounter, currentValue) == currentValue)
            {
                //We successfully claimed a block of work.
                //Console.WriteLine($"{workerIndex} claimed: thread {threadIndex}, chunk {chunkIndex}, [{startPairIndex}, {pairIndex})");
                for (int i = startPairIndex; i < pairIndex; ++i)
                {
                    var pair = chunk[i];
                    overlapFinder.narrowPhase.HandleOverlap(workerIndex, pair.A, pair.B);
                }
                debugLocalPairsExecuted += pairIndex - startPairIndex;
            }
        }
        //Some lingering pairs may remain in the batcher. Flush them.
        //if (workerIndex > 0)
        //    Console.WriteLine($"worker {workerIndex}: {debugLocalPairsExecuted}");
        //overlapFinder.debugPairsExecutedPerThread[workerIndex] += debugLocalPairsExecuted;
        if (overlapFinder.narrowPhase.overlapWorkers[workerIndex].Batcher.batches.Allocated)
            overlapFinder.narrowPhase.overlapWorkers[workerIndex].Batcher.Flush();
    }


    public void DispatchOverlaps2(float dt, IThreadDispatcher threadDispatcher = null)
    {
        if (threadDispatcher != null && threadDispatcher.ThreadCount > 1)
        {
            narrowPhase.Prepare(dt, threadDispatcher);
            if (selfHandlers == null || selfHandlers.Length < threadDispatcher.ThreadCount)
            {
                //This initialization/resize should occur extremely rarely.
                selfTestHandlers = new PairCollector[threadDispatcher.ThreadCount];
                intertreeTestHandlers = new PairCollector[threadDispatcher.ThreadCount];
            }
            //Decay the initial capacity for chunks slowly over time.
            targetChunkCapacity = int.Max(128, (int)(targetChunkCapacity * 0.95f));
            var effectiveInitialChunkCapacity = 1 << SpanHelper.GetContainingPowerOf2(targetChunkCapacity);
            threadPairs = new Buffer<ChunkedList<CollidablePair>>(threadDispatcher.ThreadCount, narrowPhase.Pool);
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                //Note that we create pairs 
                var pairs = threadPairs.GetPointer(i);
                var threadPool = threadDispatcher.WorkerPools[i];
                *pairs = new ChunkedList<CollidablePair>(threadPool, effectiveInitialChunkCapacity);
                selfTestHandlers[i] = new PairCollector(threadPool, pairs, broadPhase.ActiveLeaves, broadPhase.ActiveLeaves);
                intertreeTestHandlers[i] = new PairCollector(threadPool, pairs, broadPhase.ActiveLeaves, broadPhase.StaticLeaves);
            }
            Debug.Assert(intertreeTestHandlers.Length >= threadDispatcher.ThreadCount);
            selfTestContext2.PrepareJobs(ref broadPhase.ActiveTree, selfTestHandlers, threadDispatcher.ThreadCount, 0, narrowPhase.Pool);
            intertreeTestContext2.PrepareJobs(ref broadPhase.ActiveTree, ref broadPhase.StaticTree, intertreeTestHandlers, threadDispatcher.ThreadCount, 0, narrowPhase.Pool);
            var testTaskCount = selfTestContext2.JobCount + intertreeTestContext2.JobCount;
            var effectiveJobCount = int.Max(1, int.Max(previousPairCount, testTaskCount));
            var taskStack = new TaskStack(narrowPhase.Pool, threadDispatcher, threadDispatcher.ThreadCount);

            //debugPairsExecutedPerThread = new Buffer<int>(threadDispatcher.ThreadCount, narrowPhase.Pool);
            //debugPairsExecutedPerThread.Clear(0, debugPairsExecutedPerThread.Length);
            ContinuationHandle broadPhaseCompleteContinuation = default;
            //The counter is how the narrow phase tests keep track of which pairs have been claimed; we need to reset it each frame.
            jobCounter.Counter = 0;
            var pushTask = new Task(&PushNarrowPhaseTests, &taskStack);
            if (testTaskCount > 0)
                broadPhaseCompleteContinuation = taskStack.AllocateContinuation(testTaskCount, 0, threadDispatcher, pushTask);
            if (selfTestContext2.JobCount > 0)
                taskStack.PushForUnsafely(&SelfTestJob, null, 0, selfTestContext2.JobCount, 0, threadDispatcher, continuation: broadPhaseCompleteContinuation);
            if (intertreeTestContext2.JobCount > 0)
                taskStack.PushForUnsafely(&IntertreeTestJob, null, 0, intertreeTestContext2.JobCount, 0, threadDispatcher, continuation: broadPhaseCompleteContinuation);
            if (!broadPhaseCompleteContinuation.Initialized)
            {
                //If there are no broad phase tasks to perform, we still need to push the continuation.
                taskStack.Push(pushTask, 0, threadDispatcher);
            }
            TaskStack.DispatchWorkers(threadDispatcher, &taskStack, effectiveJobCount, this);

            taskStack.Dispose(narrowPhase.Pool, threadDispatcher);
            //Two paths by which we may need to flush batchers still:
            //1. We dispatch over parts of the tree are not yet analyzed, but the job creation phase may have put some work into the batcher.
            //If the total job count is zero, that means there's no further work to be done (implying the tree was very tiny), but we may need to flush additional jobs in worker 0.
            //2. Any workers that we allocated resources for but did not end up using due to a lack of discovered jobs need to be cleaned up. Flushing disposes those resources.
            //(this complexity could be removed if the preparation phase was aware of the job count, but that's somewhat more difficult.)
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                ref var batcher = ref narrowPhase.overlapWorkers[i].Batcher;
                if (batcher.batches.Allocated)
                    batcher.Flush();
            }
            int totalPairCount = 0;
            for (int i = 0; i < threadPairs.Length; ++i)
            {
                ref var pairs = ref threadPairs[i];
                targetChunkCapacity = int.Max(pairs.Count, targetChunkCapacity);
                totalPairCount += pairs.Count;
                pairs.Dispose(threadDispatcher.WorkerPools[i]);
            }
            previousPairCount = totalPairCount;
            threadPairs.Dispose(narrowPhase.Pool);

            //var sum = 0;
            //for (int i = 0; i < debugPairsExecutedPerThread.Length; ++i)
            //{
            //    sum += debugPairsExecutedPerThread[i];
            //}
            //var min = 1.0;
            //var minIndex = 0;
            //var max = 0.0;
            //var maxIndex = 0;
            //for (int i = 0; i < debugPairsExecutedPerThread.Length; ++i)
            //{
            //    var threadFraction = (double)debugPairsExecutedPerThread[i] / sum;
            //    if (threadFraction > max)
            //    {
            //        max = threadFraction;
            //        maxIndex = i;
            //    }
            //    if (threadFraction < min)
            //    {
            //        min = threadFraction;
            //        minIndex = i;
            //    }
            //}
            //Console.WriteLine();
            //Console.WriteLine($"min: {minIndex}, {min}");
            //Console.WriteLine($"max: {maxIndex}, {max}");
            //Console.WriteLine($"sum: {sum}");
            //debugPairsExecutedPerThread.Dispose(narrowPhase.Pool);

#if DEBUG
            for (int i = 1; i < threadDispatcher.ThreadCount; ++i)
            {
                Debug.Assert(!narrowPhase.overlapWorkers[i].Batcher.batches.Allocated, "After execution, there should be no remaining allocated collision batchers.");
            }
#endif
            selfTestContext2.CompleteSelfTest();
            intertreeTestContext2.CompleteTest();
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









    unsafe static void PrepareSelfTestJob3(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        overlapFinder.selfTestContext3.PrepareJobs(ref overlapFinder.broadPhase.ActiveTree, overlapFinder.selfTestHandlers3, threadDispatcher.ThreadCount, workerIndex, threadDispatcher.WorkerPools[workerIndex]);
    }
    unsafe static void PrepareIntertreeTestJob3(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        var broadPhase = overlapFinder.broadPhase;
        overlapFinder.intertreeTestContext3.PrepareJobs(ref broadPhase.ActiveTree, ref broadPhase.StaticTree, overlapFinder.intertreeTestHandlers3, threadDispatcher.ThreadCount, workerIndex, threadDispatcher.WorkerPools[workerIndex]);
    }
    unsafe static void CompletedPreparation3(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        var selfTest = overlapFinder.selfTestContext3;
        var intertreeTest = overlapFinder.intertreeTestContext3;
        var broadTaskCount = selfTest.JobCount + intertreeTest.JobCount;
        //We'll use a continuation to notify us when all broad jobs are complete by stopping the broad stack.
        ContinuationHandle broadPhaseCompleteContinuation = default;
        var broadStack = overlapFinder.broadStack;
        if (broadTaskCount > 0)
            broadPhaseCompleteContinuation = broadStack->AllocateContinuation(broadTaskCount, workerIndex, threadDispatcher, TaskStack.GetRequestStopTask(broadStack));
        if (selfTest.JobCount > 0)
            broadStack->PushFor(&SelfTestJob3, null, 0, selfTest.JobCount, workerIndex, threadDispatcher, continuation: broadPhaseCompleteContinuation);
        if (intertreeTest.JobCount > 0)
            broadStack->PushFor(&IntertreeTestJob3, null, 0, intertreeTest.JobCount, workerIndex, threadDispatcher, continuation: broadPhaseCompleteContinuation);

        //Go ahead and flush the narrow phase work that the preparation phase generated, if any. If there is any left, then it's not full sized (because that would have been flushed), but that's fine.
        //This is mostly free. At this point, we almost certainly have idling workers.
        overlapFinder.taskAccumulators[workerIndex].FlushToStack(workerIndex, threadDispatcher);

        if (broadTaskCount == 0)
        {
            //The broad phase didn't actually have work to do, so we can just stop it now.
            //Note that this stop was submitted *after* we flushed the stack! That's because the stop is a sync point, and we want to make sure that all the narrow phase work created by the preparation phase is submitted to the narrow phase stack.
            broadStack->RequestStop();
        }
    }

    unsafe static void SelfTestJob3(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        overlapFinder.selfTestContext3.ExecuteJob((int)id, workerIndex);

        //overlapFinder.taskAccumulators[workerIndex].FlushToStack(workerIndex, threadDispatcher);

    }
    unsafe static void IntertreeTestJob3(long id, void* context, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        overlapFinder.intertreeTestContext3.ExecuteJob((int)id, workerIndex);

        //overlapFinder.taskAccumulators[workerIndex].FlushToStack(workerIndex, threadDispatcher);
    }

    unsafe static void NarrowPhaseJob3(long id, void* untypedContext, int workerIndex, IThreadDispatcher threadDispatcher)
    {
        ref var pairsToTest = ref *(QuickList<CollidablePair>*)untypedContext;
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;

        var narrowPhase = overlapFinder.narrowPhase;
        for (int i = 0; i < pairsToTest.Count; ++i)
        {
            var pair = pairsToTest[i];
            narrowPhase.HandleOverlap(workerIndex, pair.A, pair.B);
        }

        overlapFinder.taskAccumulators[workerIndex].PairsTestedOnThread += pairsToTest.Count;
    }

    unsafe static bool TryExecuteNarrowPhaseJobInline(CollidableOverlapFinder<TCallbacks> overlapFinder, int workerIndex, IThreadDispatcher dispatcher)
    {
        var pairs = &overlapFinder.taskAccumulators.GetPointer(workerIndex)->Pairs;
        if (pairs->Count > 0)
        {
            NarrowPhaseJob3(0, pairs, workerIndex, dispatcher);
            //If this is invoked while the broad phase is still working, the already processed pairs should not be double counted, so zero it.
            pairs->Count = 0;
            return true;
        }
        return false;
    }

    unsafe static void Worker3(int workerIndex, IThreadDispatcher threadDispatcher)
    {
        var overlapFinder = (CollidableOverlapFinder<TCallbacks>)threadDispatcher.ManagedContext;
        var broadPhase = overlapFinder.broadPhase;
        var broadStack = overlapFinder.broadStack;

        //The worker stays active for the duration of the dispatch that covers both the broad phase and narrow phase.
        //We'll grab tree testing jobs with priority since those generate the narrow phase jobs.
        var waiter = new SpinWait();
        var narrowStack = overlapFinder.narrowStack;
        while (true)
        {
            var broadResult = broadStack->TryPopAndRun(workerIndex, threadDispatcher);
            if (broadResult == PopTaskResult.Stop)
            {
                //We don't want to keep suffering the communication overhead associated with the broad phase stack if we know there will never be any work left; just decay to narrow only.
                break;
            }
            if (TryExecuteNarrowPhaseJobInline(overlapFinder, workerIndex, threadDispatcher))
            {
                //The thread has a moment. Go ahead and eat inline narrow phase work if it exists.
                waiter.Reset();
                continue;
            }
            if (broadResult == PopTaskResult.Success)
            {
                //We only want to continue to the narrow phase test if there are no pending broad phase tests. Broad phase tests generate narrow phase work.
                waiter.Reset();
                continue;
            }
            //Broad phase had no work, but it's not done. Chomp some narrow phase work if it exists.
            var narrowResult = narrowStack->TryPopAndRun(workerIndex, threadDispatcher);
            //Note that it's impossible for broadResult to be Stop here, so we can't use a lack of narrow phase work to terminate.
            Debug.Assert(narrowResult != PopTaskResult.Stop, "Hey, we assumed that the narrow phase will never get stopped; did you modify something?");
            if (narrowResult == PopTaskResult.Success)
            {
                //We got some narrow phase work. Keep going.
                waiter.Reset();
                continue;
            }
            else
            {
                //There was no broad phase OR narrow phase work available. Rest a moment before sampling again.
                waiter.SpinOnce(-1);
            }

        }
        //All broad phase jobs are completely done now.
        //Go ahead and push any unflushed pairs into the narrow phase without bothering to submit them to the narrow phase stack; this thread clearly has some free time.
        //Note that by submitting these pairs to the narrow phase inline, the next loop doesn't need to wait on a sync.
        //(Consider the alternative: if these pairs were submitted to the narrow phase stack, then the next loop can't know that there are no more narrow phase tasks unless we inserted another sync point.
        //The termination of the broad phase stack with the Stop command serves as the sync point for all narrow phase stack work created by the broad phase.)
        TryExecuteNarrowPhaseJobInline(overlapFinder, workerIndex, threadDispatcher);

        //So, at this point, the narrow phase stack will receive no further jobs. Just keep chomping until it's empty.
        while (true)
        {
            //Chomp some narrow phase work if it exists.
            var narrowResult = narrowStack->TryPopAndRun(workerIndex, threadDispatcher);
            //Note that it's impossible for broadResult to be Stop here, so we can't use a lack of narrow phase work to terminate.
            Debug.Assert(narrowResult != PopTaskResult.Stop, "Hey, we assumed that the narrow phase will never get stopped; did you modify something?");
            if (narrowResult != PopTaskResult.Success)
            {
                //Done!
                break;
            }
        }

        //Flush any remaining batches for this worker inside the narrow phase.
        //Note that this captures every single worker that could have done anything; there will be no need for a postpass that flushes batches.
        //(Even the 'prepare jobs' phase is included within this worker.)
        if (overlapFinder.narrowPhase.overlapWorkers[workerIndex].Batcher.batches.Allocated)
            overlapFinder.narrowPhase.overlapWorkers[workerIndex].Batcher.Flush();
    }


    //TODO: Forcing the struct layout to be larger on the task accumulator reduces false sharing risk.
    [StructLayout(LayoutKind.Sequential, Size = 256)]
    public struct TaskAccumulator(BufferPool threadPool, int maximumTaskSize, int estimatedMaximumTaskCount, TaskStack* narrowPhaseTaskStack)
    {

        public QuickList<CollidablePair> Pairs = new(maximumTaskSize, threadPool);
        /// <summary>
        /// Stores task pair lists in the thread heap for use as task context.
        /// </summary>
        public ChunkedList<QuickList<CollidablePair>> TaskContexts = new(threadPool, estimatedMaximumTaskCount);
        public int MaximumTaskSize = maximumTaskSize;
        public TaskStack* NarrowPhaseTaskStack = narrowPhaseTaskStack;
        /// <summary>
        /// Accumulates the number of pairs tested on this thread. Note that this is not the same thing as the number of pairs submitted to the accumulator; this is added to *after* the submission to the narrow phase.
        /// Those are two different numbers because the narrow phase test may be performed on a different thread than the one that submitted the pair.
        /// It's convenient to track this here because the accumulator is already thread local.
        /// </summary>
        public int PairsTestedOnThread;
        public int FlushCount;

        public void Accumulate(CollidablePair pair, int workerIndex, IThreadDispatcher dispatcher)
        {
            Pairs.AllocateUnsafely() = pair;
            if (Pairs.Count == MaximumTaskSize)
            {
                //The pair list is full. Push a task to handle it.
                FlushToStack(workerIndex, dispatcher);
            }
        }
        public void FlushToStack(int workerIndex, IThreadDispatcher dispatcher)
        {
            if (Pairs.Count > 0)
            {
                ++FlushCount;
                var pool = dispatcher.WorkerPools[workerIndex];
                ref var taskContext = ref TaskContexts.Allocate(pool);
                taskContext = Pairs;
                var task = new Task(&NarrowPhaseJob3, Unsafe.AsPointer(ref taskContext));
                NarrowPhaseTaskStack->Push(task, workerIndex, dispatcher);
                Pairs = new QuickList<CollidablePair>(MaximumTaskSize, pool);
            }
        }

        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < TaskContexts.Chunks.Count; ++i)
            {
                ref var chunk = ref TaskContexts.Chunks[i];
                for (int j = 0; j < chunk.Count; ++j)
                {
                    chunk[j].Dispose(pool);
                }
            }
            TaskContexts.Dispose(pool);
            Pairs.Dispose(pool);
        }

    }


    /// <summary>
    /// Collects pairs of leaf indices which overlap for a given thread in the broad phase test.
    /// </summary>
    unsafe struct PairCollector3(IThreadDispatcher dispatcher, TaskAccumulator* tasks, int workerIndex, Buffer<CollidableReference> leavesA, Buffer<CollidableReference> leavesB) : IOverlapHandler
    {
        public IThreadDispatcher Dispatcher = dispatcher;
        public TaskAccumulator* Tasks = tasks;
        public Buffer<CollidableReference> LeavesA = leavesA;
        public Buffer<CollidableReference> LeavesB = leavesB;
        public int WorkerIndex = workerIndex;

        public void Handle(int indexA, int indexB)
        {
            Tasks->Accumulate(new CollidablePair(LeavesA[indexA], LeavesB[indexB]), WorkerIndex, Dispatcher);
        }

    }




    PairCollector3[] selfTestHandlers3, intertreeTestHandlers3;
    Buffer<TaskAccumulator> taskAccumulators;
    int previousPairCount3;
    int taskSize;
    Tree.MultithreadedSelfTest<PairCollector3> selfTestContext3;
    Tree.MultithreadedIntertreeTest<PairCollector3> intertreeTestContext3;
    TaskStack* broadStack, narrowStack;


    public void DispatchOverlaps3(float dt, IThreadDispatcher threadDispatcher = null)
    {
        if (threadDispatcher != null && threadDispatcher.ThreadCount > 1)
        {
            narrowPhase.Prepare(dt, threadDispatcher);
            if (selfTestHandlers3 == null || selfTestHandlers3.Length < threadDispatcher.ThreadCount)
            {
                //This initialization/resize should occur extremely rarely.
                selfTestHandlers3 = new PairCollector3[threadDispatcher.ThreadCount];
                intertreeTestHandlers3 = new PairCollector3[threadDispatcher.ThreadCount];
            }
            taskAccumulators = new Buffer<TaskAccumulator>(threadDispatcher.ThreadCount, narrowPhase.Pool);
            var broadTaskStack = new TaskStack(narrowPhase.Pool, threadDispatcher, threadDispatcher.ThreadCount);
            var narrowTaskStack = new TaskStack(narrowPhase.Pool, threadDispatcher, threadDispatcher.ThreadCount);
            broadStack = &broadTaskStack;
            narrowStack = &narrowTaskStack;
            const int targetJobsPerThread = 1;
            int maximumTaskSize = int.Max(1, previousPairCount3 / (threadDispatcher.ThreadCount * targetJobsPerThread));
            //Console.WriteLine($"maixmum taskese: {maximumTaskSize}");
            var estimatedMaximumTaskCountPerThread = targetJobsPerThread * 2;
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                //Note that we create pairs 
                var threadAccumulator = taskAccumulators.GetPointer(i);
                var threadPool = threadDispatcher.WorkerPools[i];
                *threadAccumulator = new TaskAccumulator(threadPool, maximumTaskSize, estimatedMaximumTaskCountPerThread, narrowStack);
                selfTestHandlers3[i] = new PairCollector3(threadDispatcher, threadAccumulator, i, broadPhase.ActiveLeaves, broadPhase.ActiveLeaves);
                intertreeTestHandlers3[i] = new PairCollector3(threadDispatcher, threadAccumulator, i, broadPhase.ActiveLeaves, broadPhase.StaticLeaves);
            }
            Debug.Assert(intertreeTestHandlers3.Length >= threadDispatcher.ThreadCount);
            //Submit both self test and intertree test jobs to the broad phase stack; they can run in parallel.
            //They're pretty cheap, but since we've got the fork infrastructure, we might as well use it.
            var completedPreparationContinuation = broadStack->AllocateContinuation(2, 0, threadDispatcher, new Task(&CompletedPreparation3));
            broadStack->PushUnsafely(new Task(&PrepareSelfTestJob3, continuation: completedPreparationContinuation), 0, threadDispatcher);
            broadStack->PushUnsafely(new Task(&PrepareIntertreeTestJob3, continuation: completedPreparationContinuation), 0, threadDispatcher);
            threadDispatcher.DispatchWorkers(&Worker3, managedContext: this);

            narrowTaskStack.Dispose(narrowPhase.Pool, threadDispatcher);
            broadTaskStack.Dispose(narrowPhase.Pool, threadDispatcher);
            int totalPairCount = 0;
            var debugMin = int.MaxValue;
            var debugMinIndex = 0;
            var debugMax = 0;
            var debugMaxIndex = 0;
            int totalFlushCount = 0;
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                ref var accumulator = ref taskAccumulators[i];
                totalPairCount += accumulator.PairsTestedOnThread;

                if (debugMin > accumulator.PairsTestedOnThread)
                {
                    debugMin = accumulator.PairsTestedOnThread;
                    debugMinIndex = i;
                }
                if (debugMax < accumulator.PairsTestedOnThread)
                {
                    debugMax = accumulator.PairsTestedOnThread;
                    debugMaxIndex = i;
                }
                totalFlushCount += accumulator.FlushCount;
                accumulator.Dispose(threadDispatcher.WorkerPools[i]);
            }
            previousPairCount3 = totalPairCount;
            taskAccumulators.Dispose(narrowPhase.Pool);

            //Console.WriteLine();
            //Console.WriteLine($"min: {debugMinIndex}, {debugMin / (double)totalPairCount}");
            //Console.WriteLine($"max: {debugMaxIndex}, {debugMax / (double)totalPairCount}");
            //Console.WriteLine($"sum: {totalPairCount}");
            //Console.WriteLine($"TFC: {totalFlushCount}");

#if DEBUG
            for (int i = 1; i < threadDispatcher.ThreadCount; ++i)
            {
                Debug.Assert(!narrowPhase.overlapWorkers[i].Batcher.batches.Allocated, "After execution, there should be no remaining allocated collision batchers.");
            }
#endif
            selfTestContext3.CompleteSelfTest();
            intertreeTestContext3.CompleteTest();
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
}
[StructLayout(LayoutKind.Explicit, Size = 256 + 8)]
struct PaddedCounter
{
    [FieldOffset(128)]
    public long Counter;
}

