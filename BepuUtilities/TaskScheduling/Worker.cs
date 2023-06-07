using System;
using System.Diagnostics;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuUtilities.TaskScheduling;

internal unsafe struct Worker
{
    //The worker needs to track allocations made over the course of its lifetime so they can be disposed later.
    public QuickList<nuint> AllocatedJobs;

    public ContinuationBlock* ContinuationHead;
    public int WorkerIndex;

    [Conditional("DEBUG")]
    public void ValidateTasks()
    {
        for (int i = 0; i < AllocatedJobs.Count; ++i)
        {
            var job = (Job*)AllocatedJobs[i];
            for (int j = 0; j < job->Tasks.length; ++j)
            {
                Debug.Assert(job->Tasks[j].Function != null);
            }
        }
    }


    public Worker(int workerIndex, IThreadDispatcher dispatcher, int initialJobCapacity = 128, int continuationBlockCapacity = 128)
    {
        var threadPool = dispatcher.WorkerPools[workerIndex];
        WorkerIndex = workerIndex;
        AllocatedJobs = new QuickList<nuint>(initialJobCapacity, threadPool);
        ContinuationHead = ContinuationBlock.Create(continuationBlockCapacity, threadPool);
    }

    public void Dispose(BufferPool threadPool)
    {
        for (int i = 0; i < AllocatedJobs.Count; ++i)
        {
            ((Job*)AllocatedJobs[i])->Dispose(threadPool);
        }
        AllocatedJobs.Dispose(threadPool);
        ContinuationHead->Dispose(threadPool);
    }

    internal void Reset(BufferPool threadPool)
    {
        for (int i = 0; i < AllocatedJobs.Count; ++i)
        {
            ((Job*)AllocatedJobs[i])->Dispose(threadPool);
        }
        AllocatedJobs.Count = 0;
        var capacity = ContinuationHead->Continuations.length;
        ContinuationHead->Dispose(threadPool);
        ContinuationHead = ContinuationBlock.Create(capacity, threadPool);
    }

    /// <summary>
    /// Pushes a set of tasks onto the stack.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="tag">User tag associated with the job.</param>
    /// <param name="dispatcher">Dispatcher used to pull thread allocations if necessary.</param>
    /// <remarks>If the worker associated with this stack might be active, this function can only be called by the worker.</remarks>
    internal Job* AllocateJob(Span<Task> tasks, ulong tag, IThreadDispatcher dispatcher)
    {
        Debug.Assert(tasks.Length > 0, "Probably shouldn't be trying to push zero tasks.");
        var threadPool = dispatcher.WorkerPools[WorkerIndex];
        //Note that we allocate jobs on the heap directly; it's safe to resize the AllocatedJobs list because it's just storing pointers.
        var job = Job.Create(tasks, tag, threadPool);
        AllocatedJobs.Allocate(threadPool) = (nuint)job;
        return job;
    }


    /// <summary>
    /// Allocates a continuation for a set of tasks.
    /// </summary>
    /// <param name="taskCount">Number of tasks associated with the continuation.</param>
    /// <param name="dispatcher">Dispatcher from which to pull a buffer pool if needed for resizing.</param>
    /// <param name="onCompleted">Function to execute upon completing all associated tasks, if any. Any task with a null <see cref="Task.Function"/> will not be executed.</param>
    /// <returns>Handle of the continuation.</returns>
    public ContinuationHandle AllocateContinuation(int taskCount, IThreadDispatcher dispatcher, Task onCompleted = default)
    {
        if (!ContinuationHead->TryAllocateContinuation(out TaskContinuation* continuation))
        {
            //Couldn't allocate; need to allocate a new block.
            //(The reason for the linked list style allocation is that resizing a buffer- and returning the old buffer- opens up a potential race condition.)
            var newBlock = ContinuationBlock.Create(ContinuationHead->Continuations.length, dispatcher.WorkerPools[WorkerIndex]);
            newBlock->Previous = ContinuationHead;
            ContinuationHead = newBlock;
            var allocated = ContinuationHead->TryAllocateContinuation(out continuation);
            Debug.Assert(allocated, "Just created that block! Is the capacity wrong?");
        }
        continuation->OnCompleted = onCompleted;
        continuation->RemainingTaskCounter = taskCount;
        return new ContinuationHandle(continuation);
    }
}
