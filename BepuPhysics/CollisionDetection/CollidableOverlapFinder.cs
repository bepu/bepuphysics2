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
        selfTestContext = new Tree.MultithreadedSelfTest<SelfOverlapHandler>(narrowPhase.Pool);
        intertreeTestContext = new Tree.MultithreadedIntertreeTest<IntertreeOverlapHandler>(narrowPhase.Pool);
        this.narrowPhase = narrowPhase;
        this.broadPhase = broadPhase;
        workerAction = Worker;

        //VERSION 2
        selfTestContext2 = new Tree.MultithreadedSelfTest<PairCollector>(narrowPhase.Pool);
        intertreeTestContext2 = new Tree.MultithreadedIntertreeTest<PairCollector>(narrowPhase.Pool);
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
        //DispatchOverlaps2(dt, threadDispatcher);
        //return;
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
            selfTestContext2.PrepareJobs(ref broadPhase.ActiveTree, selfTestHandlers, threadDispatcher.ThreadCount);
            intertreeTestContext2.PrepareJobs(ref broadPhase.ActiveTree, ref broadPhase.StaticTree, intertreeTestHandlers, threadDispatcher.ThreadCount);
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
}
[StructLayout(LayoutKind.Explicit, Size = 256 + 8)]
struct PaddedCounter
{
    [FieldOffset(128)]
    public long Counter;
}

