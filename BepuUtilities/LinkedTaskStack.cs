using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuUtilities.TestLinkedTaskStack;

/// <summary>
/// Description of a task to be submitted to a <see cref="LinkedTaskStack"/>.
/// </summary>
public unsafe struct Task
{
    /// <summary>
    /// Function to be executed by the task. Takes as arguments the <see cref="Id"/>, <see cref="Context"/> pointer, and executing worker index.
    /// </summary>
    public delegate*<long, void*, int, IThreadDispatcher, void> Function;
    /// <summary>
    /// Context to be passed into the <see cref="Function"/>.
    /// </summary>
    public void* Context;
    /// <summary>
    /// Continuation to be notified after this task completes, if any.
    /// </summary>
    public ContinuationHandle Continuation;
    /// <summary>
    /// User-provided identifier of this task.
    /// </summary>
    public long Id;

    /// <summary>
    /// Creates a new task.
    /// </summary>
    /// <param name="function">Function to be executed by the task. Takes as arguments the <see cref="Id"/>, <see cref="Context"/> pointer, executing worker index, and executing <see cref="IThreadDispatcher"/>.</param>
    /// <param name="context">Context pointer to pass to the <see cref="Function"/>.</param>
    /// <param name="taskId">Id of this task to be passed into the <see cref="Function"/>.</param>
    /// <param name="continuation">Continuation to notify after the completion of this task, if any.</param>
    public Task(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context = null, long taskId = 0, ContinuationHandle continuation = default)
    {
        Function = function;
        Context = context;
        Continuation = continuation;
        Id = taskId;
    }

    /// <summary>
    /// Creates a task from a function.
    /// </summary>
    /// <param name="function">Function to turn into a task.</param>
    public static implicit operator Task(delegate*<long, void*, int, IThreadDispatcher, void> function) => new(function);

    /// <summary>
    /// Runs the task and, if necessary, notifies the associated continuation of its completion.
    /// </summary>
    /// <param name="workerIndex">Worker index to pass to the function.</param> 
    /// <param name="dispatcher">Dispatcher running this task.</param>
    public void Run(int workerIndex, IThreadDispatcher dispatcher)
    {
        Debug.Assert(!Continuation.Completed && (Function != null));
        Function(Id, Context, workerIndex, dispatcher);
        if (Continuation.Initialized)
            Continuation.NotifyTaskCompleted(workerIndex, dispatcher);
    }
}

/// <summary>
/// Describes the result status of a pop attempt.
/// </summary>
public enum PopTaskResult
{
    /// <summary>
    /// A task was successfully popped.
    /// </summary>
    Success,
    /// <summary>
    /// The stack was empty, but may have more tasks in the future.
    /// </summary>
    Empty,
    /// <summary>
    /// The stack has been terminated and all threads seeking work should stop.
    /// </summary>
    Stop
}

/// <summary>
/// Refers to a continuation within a <see cref="LinkedTaskStack"/>.
/// </summary>
public unsafe struct ContinuationHandle : IEquatable<ContinuationHandle>
{
    //This is a bit odd. We're presenting this pointer as a handle, even though it's not.
    //Hiding the implementation detail makes it a little easier to change later if we need to.
    TaskContinuation* continuation;

    internal ContinuationHandle(TaskContinuation* continuation)
    {
        this.continuation = continuation;
    }

    /// <summary>
    /// Gets whether the tasks associated with this continuation have completed. If the continuation has not been initialized, this will always return false.
    /// </summary>
    public bool Completed
    {
        get
        {
            return Initialized && continuation->RemainingTaskCounter <= 0;
        }
    }

    /// <summary>
    /// Retrieves a pointer to the continuation data for <see cref="ContinuationHandle"/>.
    /// </summary>
    /// <returns>Pointer to the continuation backing the given handle.</returns>
    /// <remarks>This should not be used if the continuation handle is not known to be valid. The data pointed to by the data could become invalidated if the continuation completes.</remarks>
    public TaskContinuation* Continuation => continuation;

    /// <summary>
    /// Gets a null continuation handle.
    /// </summary>
    public static ContinuationHandle Null => default;

    /// <summary>
    /// Gets whether this handle ever represented an allocated handle. This does not guarantee that the continuation's associated tasks are active in the <see cref="LinkedTaskStack"/> that it was allocated from.
    /// </summary>
    public bool Initialized => continuation != null;

    /// <summary>
    /// Notifies the continuation that one task was completed.
    /// </summary>
    /// <param name="workerIndex">Worker index to pass to the continuation's delegate, if any.</param>
    /// <param name="dispatcher">Dispatcher to pass to the continuation's delegate, if any.</param>
    public void NotifyTaskCompleted(int workerIndex, IThreadDispatcher dispatcher)
    {
        var continuation = Continuation;
        Debug.Assert(!Completed);
        var counter = Interlocked.Decrement(ref continuation->RemainingTaskCounter);
        Debug.Assert(counter >= 0, "The counter should not go negative. Was notify called too many times?");
        if (counter == 0)
        {
            //This entire job has completed.
            if (continuation->OnCompleted.Function != null)
            {
                continuation->OnCompleted.Function(continuation->OnCompleted.Id, continuation->OnCompleted.Context, workerIndex, dispatcher);
            }
        }
    }

    public bool Equals(ContinuationHandle other) => other.continuation == continuation;

    public override bool Equals([NotNullWhen(true)] object obj) => obj is ContinuationHandle handle && Equals(handle);

    public override int GetHashCode() => (int)continuation;

    public static bool operator ==(ContinuationHandle left, ContinuationHandle right) => left.Equals(right);

    public static bool operator !=(ContinuationHandle left, ContinuationHandle right) => !(left == right);
}



/// <summary>
/// Determines which jobs are allowed to serve a <see cref="LinkedTaskStack.TryPop{TJobFilter}(ref TJobFilter, out Task)"/> request.
/// </summary>
public interface IJobFilter
{
    /// <summary>
    /// Determines whether a job with the given tag should be allowed to serve a <see cref="LinkedTaskStack.TryPop{TJobFilter}(ref TJobFilter, out Task)"/> request.
    /// </summary>
    /// <param name="jobTag">Tag of the candidate job.</param>
    /// <returns>True if the job should be allowed to serve a request, false otherwise.</returns>
    bool AllowJob(ulong jobTag);
}

/// <summary>
/// A filter that will allow pops from any jobs.
/// </summary>
public struct AllowAllJobs : IJobFilter
{
    /// <inheritdoc/>
    public readonly bool AllowJob(ulong jobTag)
    {
        return true;
    }
}

/// <summary>
/// A job filter that wraps a managed delegate.
/// </summary>
public struct DelegateJobFilter : IJobFilter
{
    /// <summary>
    /// Delegate to use as the filter.
    /// </summary>
    public Func<ulong, bool> Filter;
    /// <summary>
    /// Creates a job filter that wraps a delegate.
    /// </summary>
    /// <param name="filter">Delegate to use as the filter.</param>
    public DelegateJobFilter(Func<ulong, bool> filter)
    {
        Filter = filter;
    }
    /// <inheritdoc/>
    public readonly bool AllowJob(ulong jobTag)
    {
        return Filter(jobTag);
    }
}

/// <summary>
/// A job filter that wraps a function pointer.
/// </summary>
public unsafe struct FunctionPointerJobFilter : IJobFilter
{
    /// <summary>
    /// Delegate to use as the filter.
    /// </summary>
    public delegate*<ulong, bool> Filter;
    /// <summary>
    /// Creates a job filter that wraps a delegate.
    /// </summary>
    /// <param name="filter">Delegate to use as the filter.</param>
    public FunctionPointerJobFilter(delegate*<ulong, bool> filter)
    {
        Filter = filter;
    }
    /// <inheritdoc/>
    public readonly bool AllowJob(ulong jobTag)
    {
        return Filter(jobTag);
    }
}
/// <summary>
/// A job filter that requires the job tag to meet or exceed a threshold value.
/// </summary>
public struct MinimumTagFilter : IJobFilter
{
    /// <summary>
    /// Value that a job must match or exceed to be allowed.
    /// </summary>
    public ulong MinimumTagValue;
    /// <summary>
    /// Creates a job filter that requires the job tag to meet or exceed a threshold value.
    /// </summary>
    /// <param name="minimumTagValue">Value that a job must match or exceed to be allowed.</param>
    public MinimumTagFilter(ulong minimumTagValue)
    {
        MinimumTagValue = minimumTagValue;
    }
    /// <inheritdoc/>
    public bool AllowJob(ulong jobTag)
    {
        return jobTag >= MinimumTagValue;
    }
}

/// <summary>
/// A job filter that requires the job tag to match a specific value.
/// </summary>
public struct EqualTagFilter : IJobFilter
{
    /// <summary>
    /// Tag value required to allow a job.
    /// </summary>
    public ulong RequiredTag;

    /// <summary>
    /// Creates a job filter that requires the job tag to match a specific value.
    /// </summary>
    public EqualTagFilter(ulong requiredTag)
    {
        RequiredTag = requiredTag;
    }
    /// <inheritdoc/>
    public bool AllowJob(ulong jobTag)
    {
        return jobTag == RequiredTag;
    }
}


[StructLayout(LayoutKind.Explicit, Size = 292)]
internal unsafe struct Job
{
    [FieldOffset(0)]
    public Buffer<Task> Tasks;
    [FieldOffset(16)]
    public Job* Previous;
    [FieldOffset(24)]
    public ulong Tag;

    [FieldOffset(160)]
    public int Counter;


    public static Job* Create(Span<Task> sourceTasks, ulong tag, BufferPool pool)
    {
        //Note that the job and the buffer of tasks are allocated together as one block.
        //This ensures we only need to perform one 
        var sizeToAllocate = sizeof(Job) + sourceTasks.Length * sizeof(Task);
        pool.Take<byte>(sizeToAllocate, out var rawBuffer);
        Job* job = (Job*)rawBuffer.Memory;
        job->Tasks = new Buffer<Task>(rawBuffer.Memory + sizeof(Job), sourceTasks.Length, rawBuffer.Id);
        job->Tag = tag;
        sourceTasks.CopyTo(job->Tasks);
        job->Counter = sourceTasks.Length;
        job->Previous = null;
        return job;
    }


    /// <summary>
    /// Attempts to pop a task from the job.
    /// </summary>
    /// <param name="task">Task popped from the job, if any.</param>
    /// <returns>True if a task was available to pop, false otherwise.</returns>
    internal bool TryPop(out Task task)
    {

        var newCount = Interlocked.Decrement(ref Counter);
        if (newCount >= 0)
        {
            task = Tasks[newCount];
            Debug.Assert(task.Function != null);
            return true;
        }
        task = default;
        return false;
    }

    public void Dispose(BufferPool pool)
    {
        //The instance is allocated from the same memory as the tasks buffer, so disposing it returns the Job memory too.
        var id = Tasks.Id;
        this = default;
        pool.ReturnUnsafely(id);
    }
}

/// <summary>
/// Stores a block of task continuations that maintains a pointer to previous blocks. 
/// </summary>
internal unsafe struct ContinuationBlock
{
    public ContinuationBlock* Previous;

    public int Count;
    public Buffer<TaskContinuation> Continuations;

    public static ContinuationBlock* Create(int continuationCapacity, BufferPool pool)
    {
        pool.Take<byte>(sizeof(TaskContinuation) * continuationCapacity + sizeof(ContinuationBlock), out var rawBuffer);
        ContinuationBlock* block = (ContinuationBlock*)rawBuffer.Memory;
        block->Continuations = new Buffer<TaskContinuation>(rawBuffer.Memory + sizeof(ContinuationBlock), continuationCapacity, rawBuffer.Id);
        block->Count = 0;
        block->Previous = null;
        return block;
    }

    public bool TryAllocateContinuation(out TaskContinuation* continuation)
    {
        if (Count < Continuations.length)
        {
            continuation = Continuations.Memory + (Count++);
            return true;
        }
        continuation = null;
        return false;
    }

    public void Dispose(BufferPool pool)
    {
        var id = Continuations.Id;
        pool.ReturnUnsafely(id);
        if (Previous != null)
            Previous->Dispose(pool);
        this = default;
    }
}


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

/// <summary>
/// Stores data relevant to tracking task completion and reporting completion for a continuation.
/// </summary>
public unsafe struct TaskContinuation
{
    /// <summary>
    /// Task to run upon completion of the associated task.
    /// </summary>
    public Task OnCompleted;
    /// <summary>
    /// Number of tasks not yet reported as complete in the continuation.
    /// </summary>
    public int RemainingTaskCounter;
}


/// <summary>
/// Manages a linked stack of tasks.
/// </summary>
public unsafe struct LinkedTaskStack
{
    Buffer<Worker> workers;

    [StructLayout(LayoutKind.Explicit, Size = 256 + 16)]
    struct StopPad
    {
        [FieldOffset(128)]
        public volatile bool Stop;
    }
    StopPad padded;

    /// <summary>
    /// Most recently pushed job on the stack. May be null if the stack is empty.
    /// </summary>
    /// <remarks>Pointers and generics don't play well, alas.</remarks>
    volatile nuint head;

    /// <summary>
    /// Constructs a new parallel task stack.
    /// </summary>
    /// <param name="pool">Buffer pool to allocate non-thread allocated resources from.</param>
    /// <param name="dispatcher">Thread dispatcher to pull thread pools from for thread allocations.</param>
    /// <param name="workerCount">Number of workers to allocate space for.</param>
    /// <param name="initialWorkerJobCapacity">Initial number of jobs (groups of tasks submitted together) to allocate space for in each worker.</param>
    /// <param name="continuationBlockCapacity">Number of slots to allocate in each block of continuations in each worker.</param>
    public LinkedTaskStack(BufferPool pool, IThreadDispatcher dispatcher, int workerCount, int initialWorkerJobCapacity = 128, int continuationBlockCapacity = 128)
    {
        pool.Take(workerCount, out workers);
        for (int i = 0; i < workerCount; ++i)
        {
            workers[i] = new Worker(i, dispatcher, initialWorkerJobCapacity, continuationBlockCapacity);
        }
        Reset(dispatcher);
    }

    /// <summary>
    /// Returns the stack to a fresh state without reallocating.
    /// </summary>
    /// <param name="dispatcher">Dispatcher whose thread pools should be used to return any thread allocated resources.</param>
    public void Reset(IThreadDispatcher dispatcher)
    {
        for (int i = 0; i < workers.Length; ++i)
        {
            workers[i].Reset(dispatcher.WorkerPools[workers[i].WorkerIndex]);
        }
        padded.Stop = false;
        head = (nuint)null;
    }

    /// <summary>
    /// Returns unmanaged resources held by the <see cref="LinkedTaskStack"/> to a pool.
    /// </summary>
    /// <param name="pool">Buffer pool to return resources to.</param>
    /// <param name="dispatcher">Dispatcher whose thread pools should be used to return any thread allocated resources.</param>
    public void Dispose(BufferPool pool, IThreadDispatcher dispatcher)
    {
        for (int i = 0; i < workers.Length; ++i)
        {
            workers[i].Dispose(dispatcher.WorkerPools[workers[i].WorkerIndex]);
        }
        pool.Return(ref workers);
    }

    /// <summary>
    /// Gets the approximate number of active tasks. This is not guaranteed to actually measure the true number of tasks at any one point in time.
    /// </summary>
    public int ApproximateTaskCount
    {
        get
        {
            int sum = 0;
            var job = (Job*)this.head;
            while (true)
            {
                if (job == null)
                    break;
                sum += int.Max(0, job->Counter);
                job = job->Previous;
            }
            return sum;
        }
    }
    /// <summary>
    /// Gets the approximate number of active continuations. This is not guaranteed to actually measure the true number of continuations at any one point in time; it checks each worker in sequence, and the continuation counts could vary arbitrarily as the checks proceed.
    /// </summary>
    public int ApproximateContinuationCount
    {
        get
        {
            int sum = 0;
            for (int i = 0; i < workers.Length; ++i)
            {
                var block = workers[i].ContinuationHead;
                while (block != null)
                {
                    sum += block->Count;
                    block = block->Previous;
                }
            }
            return sum;
        }
    }


    /// <summary>
    /// Attempts to allocate a continuation for a set of tasks.
    /// </summary>
    /// <param name="taskCount">Number of tasks associated with the continuation.</param>
    /// <param name="workerIndex">Worker index to allocate the continuation on.</param>
    /// <param name="dispatcher">Dispatcher to use for any per-thread allocations if necessary.</param>
    /// <param name="onCompleted">Function to execute upon completing all associated tasks, if any. Any task with a null <see cref="Task.Function"/> will not be executed.</param>
    /// <returns>Handle of the continuation.</returns>
    public ContinuationHandle AllocateContinuation(int taskCount, int workerIndex, IThreadDispatcher dispatcher, Task onCompleted = default)
    {
        return workers[workerIndex].AllocateContinuation(taskCount, dispatcher, onCompleted);
    }

    /// <summary>
    /// Attempts to pop a task.
    /// </summary>
    /// <param name="filter">Filter to apply to jobs. Only allowed jobs can have tasks popped from them.</param>
    /// <param name="task">Popped task, if any.</param>
    /// <typeparam name="TJobFilter">Type of the job filter used in the pop.</typeparam>
    /// <returns>Result status of the pop attempt.</returns>
    public PopTaskResult TryPop<TJobFilter>(ref TJobFilter filter, out Task task) where TJobFilter : IJobFilter
    {
        //Note that this implementation does not need to lock against anything. We just follow the pointer.
        var job = (Job*)head;
        while (true)
        {
            if (job == null)
            {
                //There is no job to pop from.
                task = default;
                return padded.Stop ? PopTaskResult.Stop : PopTaskResult.Empty;
            }
            //Try to pop a task from the current job.
            if (!filter.AllowJob(job->Tag))
            {
                //This job isn't allowed for this pop; go to the next one.
                job = job->Previous;
                continue;
            }
            if (job->TryPop(out task))
            {
                Debug.Assert(task.Function != null);
                return PopTaskResult.Success;
            }
            else
            {
                //There was no task available in this job, which means the sampled job should be removed from the stack.
                //Note that other threads might be doing the same thing; we must use an interlocked operation to try to swap the head.
                //If this fails, the head has changed before we could remove it and the current empty job will persists in the stack until some other dequeue finds it.
                //That's okay.
                Interlocked.CompareExchange(ref head, (nuint)job->Previous, (nuint)job);
                job = (Job*)head;
            }
        }
    }

    /// <summary>
    /// Attempts to pop a task.
    /// </summary>
    /// <param name="task">Popped task, if any.</param>
    /// <returns>Result status of the pop attempt.</returns>
    public PopTaskResult TryPop(out Task task)
    {
        AllowAllJobs filter = default;
        return TryPop(ref filter, out task);
    }

    /// <summary>
    /// Attempts to pop a task and run it.
    /// </summary>
    /// <param name="filter">Filter to apply to jobs. Only allowed jobs can have tasks popped from them.</param>
    /// <param name="workerIndex">Index of the worker to pass into the task function.</param>
    /// <param name="dispatcher">Thread dispatcher running this task stack.</param>
    /// <typeparam name="TJobFilter">Type of the job filter used in the pop.</typeparam>
    /// <returns>Result status of the pop attempt.</returns>
    public PopTaskResult TryPopAndRun<TJobFilter>(ref TJobFilter filter, int workerIndex, IThreadDispatcher dispatcher) where TJobFilter : IJobFilter
    {
        var result = TryPop(ref filter, out var task);
        if (result == PopTaskResult.Success)
        {
            task.Run(workerIndex, dispatcher);
        }
        return result;
    }

    /// <summary>
    /// Attempts to pop a task and run it.
    /// </summary>
    /// <param name="workerIndex">Index of the worker to pass into the task function.</param>
    /// <param name="dispatcher">Thread dispatcher running this task stack.</param>
    /// <returns>Result status of the pop attempt.</returns>
    public PopTaskResult TryPopAndRun(int workerIndex, IThreadDispatcher dispatcher)
    {
        AllowAllJobs filter = default;
        return TryPopAndRun(ref filter, workerIndex, dispatcher);
    }


    /// <summary>
    /// Pushes a set of tasks onto the task stack. This function is not thread safe.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="tag">User-defined tag data for the submitted job.</param>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops that could affect the specified worker.</remarks>
    public void PushUnsafely(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
    {
        Job* job = workers[workerIndex].AllocateJob(tasks, tag, dispatcher);
        job->Previous = (Job*)head;
        head = (nuint)job;
    }
    /// <summary>
    /// Pushes a task onto the task stack. This function is not thread safe.
    /// </summary>
    /// <param name="task">Task to push.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops that could affect the specified worker.</remarks>
    public unsafe void PushUnsafely(Task task, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
    {
        PushUnsafely(new Span<Task>(&task, 1), workerIndex, dispatcher, tag);
    }

    /// <summary>
    /// Pushes a set of tasks onto the task stack.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="tag">User-defined tag data for the submitted job.</param>
    /// <returns>True if the push succeeded, false if it was contested.</returns>
    public void Push(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
    {
        Job* job = workers[workerIndex].AllocateJob(tasks, tag, dispatcher);

        while (true)
        {
            //Pre-set the previous pointer so that it's visible when the job is swapped in.
            //Note that if the head pointer changes between the first set attempt and the swap, the previous pointer will be wrong and we must try again.
            job->Previous = (Job*)head;
            if ((nuint)job->Previous == Interlocked.CompareExchange(ref head, (nuint)job, (nuint)job->Previous))
                break;
        }
    }


    /// <summary>
    /// Pushes a set of tasks to the stack with a created continuation.
    /// </summary>
    /// <param name="tasks">Tasks composing the job. A continuation will be assigned internally; no continuation should be present on any of the provided tasks.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    /// <param name="onComplete">Task to run upon completion of all the submitted tasks, if any.</param>
    /// <returns>Handle of the continuation created for these tasks.</returns>
    /// <remarks>Note that this will keep trying until task submission succeeds.</remarks>
    public ContinuationHandle AllocateContinuationAndPush(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0, Task onComplete = default)
    {
        var continuationHandle = AllocateContinuation(tasks.Length, workerIndex, dispatcher, onComplete);
        for (int i = 0; i < tasks.Length; ++i)
        {
            ref var task = ref tasks[i];
            Debug.Assert(!task.Continuation.Initialized, "This function creates a continuation for the tasks");
            task.Continuation = continuationHandle;
        }
        Push(tasks, workerIndex, dispatcher, tag);
        return continuationHandle;
    }

    /// <summary>
    /// Waits for a continuation to be completed.
    /// </summary>
    /// <remarks>Instead of spinning the entire time, this may pop and execute pending tasks to fill the gap.</remarks>
    /// <param name="filter">Filter to apply to jobs. Only allowed jobs can have tasks popped from them.</param>
    /// <param name="continuation">Continuation to wait on.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the executing worker.</param>
    /// <typeparam name="TJobFilter">Type of the job filter used in the pop.</typeparam>
    public void WaitForCompletion<TJobFilter>(ref TJobFilter filter, ContinuationHandle continuation, int workerIndex, IThreadDispatcher dispatcher) where TJobFilter : IJobFilter
    {
        var waiter = new SpinWait();
        Debug.Assert(continuation.Initialized, "This codepath should only run if the continuation was allocated earlier.");
        while (!continuation.Completed)
        {
            var result = TryPop(ref filter, out var fillerTask);
            if (result == PopTaskResult.Stop)
            {
                return;
            }
            if (result == PopTaskResult.Success)
            {
                fillerTask.Run(workerIndex, dispatcher);
                waiter.Reset();
            }
            else
            {
                waiter.SpinOnce(-1);
            }
        }
    }

    /// <summary>
    /// Waits for a continuation to be completed.
    /// </summary>
    /// <remarks>Instead of spinning the entire time, this may pop and execute pending tasks to fill the gap.</remarks>
    /// <param name="continuation">Continuation to wait on.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the executing worker.</param>
    public void WaitForCompletion(ContinuationHandle continuation, int workerIndex, IThreadDispatcher dispatcher)
    {
        AllowAllJobs filter = default;
        WaitForCompletion(ref filter, continuation, workerIndex, dispatcher);
    }

    /// <summary>
    /// Pushes a set of tasks to the worker stack and returns when all tasks are complete.
    /// </summary>
    /// <param name="tasks">Tasks composing the job. A continuation will be assigned internally; no continuation should be present on any of the provided tasks.</param>
    /// <param name="workerIndex">Index of the worker executing this function.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="filter">Filter applied to jobs considered for filling the calling thread's wait for other threads to complete.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    /// <typeparam name="TJobFilter">Type of the job filter used in the pop.</typeparam>
    /// <remarks>Note that this will keep working until all tasks are run. It may execute tasks unrelated to the requested tasks while waiting on other workers to complete constituent tasks.</remarks>
    public void RunTasks<TJobFilter>(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher, ref TJobFilter filter, ulong tag = 0) where TJobFilter : IJobFilter
    {
        if (tasks.Length == 0)
            return;
        ContinuationHandle continuationHandle = default;
        if (tasks.Length > 1)
        {
            //Note that we only submit tasks to the stack for tasks beyond the first. The current thread is responsible for at least task 0.
            var taskCount = tasks.Length - 1;
            Span<Task> tasksToPush = stackalloc Task[taskCount];
            ref var worker = ref workers[workerIndex];
            continuationHandle = worker.AllocateContinuation(taskCount, dispatcher);
            for (int i = 0; i < tasksToPush.Length; ++i)
            {
                var task = tasks[i + 1];
                Debug.Assert(!task.Continuation.Initialized, $"None of the source tasks should have continuations when provided to {nameof(RunTasks)}.");
                task.Continuation = continuationHandle;
                tasksToPush[i] = task;
            }
            Push(tasksToPush, workerIndex, dispatcher, tag);
        }
        //Tasks [1, count) are submitted to the stack and may now be executing on other workers.
        //The thread calling the for loop should not relinquish its timeslice. It should immediately begin working on task 0.
        var task0 = tasks[0];
        Debug.Assert(!task0.Continuation.Initialized, $"None of the source tasks should have continuations when provided to {nameof(RunTasks)}.");
        task0.Function(task0.Id, task0.Context, workerIndex, dispatcher);

        if (tasks.Length > 1)
        {
            //Task 0 is done; this thread should seek out other work until the job is complete.
            WaitForCompletion(ref filter, continuationHandle, workerIndex, dispatcher);
        }
    }


    /// <summary>
    /// Pushes a set of tasks to the worker stack and returns when all tasks are complete.
    /// </summary>
    /// <param name="tasks">Tasks composing the job. A continuation will be assigned internally; no continuation should be present on any of the provided tasks.</param>
    /// <param name="workerIndex">Index of the worker executing this function.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    /// <remarks>Note that this will keep working until all tasks are run. It may execute tasks unrelated to the requested tasks while waiting on other workers to complete constituent tasks.</remarks>
    public void RunTasks(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
    {
        AllowAllJobs filter = default;
        RunTasks(tasks, workerIndex, dispatcher, ref filter, tag);
    }

    /// <summary>
    /// Requests that all workers stop. The next time a worker runs out of tasks to run, if it sees a stop command, it will be reported.
    /// </summary>
    public void RequestStop()
    {
        padded.Stop = true;
    }


    /// <summary>
    /// Pushes a for loop onto the task stack. Does not take a lock.
    /// </summary>
    /// <param name="function">Function to execute on each iteration of the loop.</param>
    /// <param name="context">Context pointer to pass into each task execution.</param>
    /// <param name="inclusiveStartIndex">Inclusive start index of the loop range.</param>
    /// <param name="iterationCount">Number of iterations to perform.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    /// <param name="continuation">Continuation associated with the loop tasks, if any.</param>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops that could affect the specified worker.</remarks>
    public void PushForUnsafely(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0, ContinuationHandle continuation = default)
    {
        Span<Task> tasks = stackalloc Task[iterationCount];
        for (int i = 0; i < tasks.Length; ++i)
        {
            tasks[i] = new Task { Function = function, Context = context, Id = i + inclusiveStartIndex, Continuation = continuation };
        }
        PushUnsafely(tasks, workerIndex, dispatcher, tag);
    }

    /// <summary>
    /// Pushes a for loop onto the task stack.
    /// </summary>
    /// <param name="function">Function to execute on each iteration of the loop.</param>
    /// <param name="context">Context pointer to pass into each task execution.</param>
    /// <param name="inclusiveStartIndex">Inclusive start index of the loop range.</param>
    /// <param name="iterationCount">Number of iterations to perform.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param> 
    /// <param name="continuation">Continuation associated with the loop tasks, if any.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    /// <remarks>This function will not attempt to run any iterations of the loop itself.</remarks>
    public void PushFor(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0, ContinuationHandle continuation = default)
    {
        Span<Task> tasks = stackalloc Task[iterationCount];
        for (int i = 0; i < tasks.Length; ++i)
        {
            tasks[i] = new Task { Function = function, Context = context, Id = i + inclusiveStartIndex, Continuation = continuation };
        }
        Push(tasks, workerIndex, dispatcher, tag);
    }

    /// <summary>
    /// Submits a set of tasks representing a for loop over the given indices and returns when all loop iterations are complete.
    /// </summary>
    /// <param name="function">Function to execute on each iteration of the loop.</param>
    /// <param name="context">Context pointer to pass into each iteration of the loop.</param>
    /// <param name="inclusiveStartIndex">Inclusive start index of the loop range.</param>
    /// <param name="iterationCount">Number of iterations to perform.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="filter">Filter applied to jobs considered for filling the calling thread's wait for other threads to complete.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    /// <typeparam name="TJobFilter">Type of the job filter used in the pop.</typeparam>
    public void For<TJobFilter>(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, int workerIndex, IThreadDispatcher dispatcher,
        ref TJobFilter filter, ulong tag = 0) where TJobFilter : IJobFilter
    {
        if (iterationCount <= 0)
            return;
        Span<Task> tasks = stackalloc Task[iterationCount];
        for (int i = 0; i < tasks.Length; ++i)
        {
            tasks[i] = new Task(function, context, inclusiveStartIndex + i);
        }
        RunTasks(tasks, workerIndex, dispatcher, ref filter, tag);
    }

    /// <summary>
    /// Submits a set of tasks representing a for loop over the given indices and returns when all loop iterations are complete.
    /// </summary>
    /// <param name="function">Function to execute on each iteration of the loop.</param>
    /// <param name="context">Context pointer to pass into each iteration of the loop.</param>
    /// <param name="inclusiveStartIndex">Inclusive start index of the loop range.</param>
    /// <param name="iterationCount">Number of iterations to perform.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted tasks.</param>
    public void For(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
    {
        AllowAllJobs filter = default;
        For(function, context, inclusiveStartIndex, iterationCount, workerIndex, dispatcher, ref filter, tag);
    }
}
