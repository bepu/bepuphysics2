using System;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace BepuUtilities.TestStack;

/// <summary>
/// Description of a task to be submitted to a <see cref="ParallelTaskStack"/>.
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
        Debug.Assert(!Continuation.Completed);
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
/// Refers to a continuation within a <see cref="ParallelTaskStack"/>.
/// </summary>
public unsafe struct ContinuationHandle : IEquatable<ContinuationHandle>
{
    uint index;
    uint encodedVersion;
    /// <summary>
    /// Source worker task stack.
    /// </summary>
    internal WorkerTaskStack* WorkerStack;

    internal ContinuationHandle(uint index, int version, WorkerTaskStack* workerStack)
    {
        this.index = index;
        encodedVersion = (uint)version | 1u << 31;
        WorkerStack = workerStack;
    }

    internal uint Index
    {
        get
        {
            Debug.Assert(Initialized, "If you're trying to pull a continuation id from a continuation handle, it should have been initialized.");
            return index;
        }
    }

    internal int Version
    {
        get
        {
            Debug.Assert(Initialized, "If you're trying to pull a continuation id from a continuation handle, it should have been initialized.");
            return (int)(encodedVersion & ((1u << 31) - 1));
        }
    }

    /// <summary>
    /// Gets whether the tasks associated with this continuation have completed.
    /// </summary>
    public bool Completed => WorkerStack->IsComplete(this);

    /// <summary>
    /// Retrieves a pointer to the continuation data for <see cref="ContinuationHandle"/>.
    /// </summary>
    /// <returns>Pointer to the continuation backing the given handle.</returns>
    /// <remarks>This should not be used if the continuation handle is not known to be valid. The data pointed to by the data could become invalidated if the continuation completes.</remarks>
    public TaskContinuation* Continuation => WorkerStack->GetContinuation(this);

    /// <summary>
    /// Gets a null continuation handle.
    /// </summary>
    public static ContinuationHandle Null => default;

    /// <summary>
    /// Gets whether the continuation associated with this handle was allocated and has outstanding tasks.
    /// </summary>
    public bool Exists
    {
        get
        {
            if (!Initialized || WorkerStack == null)
                return false;
            ref var continuation = ref WorkerStack->Continuations[Index];
            return continuation.Version == Version && continuation.RemainingTaskCounter > 0;
        }
    }

    /// <summary>
    /// Gets whether this handle ever represented an allocated handle. This does not guarantee that the continuation's associated tasks are active in the <see cref="ParallelTaskStack"/> that it was allocated from.
    /// </summary>
    public bool Initialized => encodedVersion >= 1u << 31;

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
            //Free this continuation slot.
            var waiter = new SpinWait();
            while (Interlocked.CompareExchange(ref WorkerStack->ContinuationLocker, 1, 0) != 0)
            {
                waiter.SpinOnce(-1);
            }
            //We have the lock.
            WorkerStack->ContinuationIndexPool.ReturnUnsafely((int)Index);
            --WorkerStack->ContinuationCount;
            WorkerStack->ContinuationLocker = 0;
        }
    }

    public bool Equals(ContinuationHandle other) => other.index == index && other.encodedVersion == encodedVersion && other.WorkerStack == WorkerStack;

    public override bool Equals([NotNullWhen(true)] object obj) => obj is ContinuationHandle handle && Equals(handle);

    public override int GetHashCode() => (int)(index ^ (encodedVersion << 24));

    public static bool operator ==(ContinuationHandle left, ContinuationHandle right) => left.Equals(right);

    public static bool operator !=(ContinuationHandle left, ContinuationHandle right) => !(left == right);
}

internal unsafe struct WorkerTaskStack
{
    public QuickList<Task> Tasks;
    public Buffer<TaskContinuation> Continuations;
    public IdPool ContinuationIndexPool;
    public int ContinuationCount;
    public int WorkerIndex;
    public volatile int TaskLocker;
    public volatile int ContinuationLocker;

    public WorkerTaskStack(int workerIndex, IThreadDispatcher dispatcher, int initialTaskCapacity = 64, int initialContinuationCapacity = 16)
    {
        var threadPool = dispatcher.WorkerPools[workerIndex];
        WorkerIndex = workerIndex;
        Tasks = new QuickList<Task>(initialTaskCapacity, threadPool);
        threadPool.Take(initialContinuationCapacity, out Continuations);
#if DEBUG
        //While you shouldn't *need* to clear continuations, it can be useful for debug purposes.
        Continuations.Clear(0, Continuations.length);
        Tasks.Span.Clear(0, Tasks.Span.length);
#endif
        ContinuationIndexPool = new IdPool(initialContinuationCapacity, threadPool);
    }

    public void Dispose(BufferPool threadPool)
    {
        Tasks.Dispose(threadPool);
        threadPool.Return(ref Continuations);
        ContinuationIndexPool.Dispose(threadPool);
    }

    /// <summary>
    /// Attempts to pop a task from the stack.
    /// </summary>
    /// <param name="task">Task popped from the stack, if any.</param>
    /// <returns>True if a task was available to pop, false otherwise.</returns>
    internal bool TryPop(out Task task)
    {
        //Quick early out to avoid lock expense.
        if (Tasks.Count == 0)
        {
            task = default;
            return false;
        }
        var waiter = new SpinWait();
        //Unlike the pushes, we only return false if there is no task. Just simplifies things a bit. Don't want to have 'try' with two different meanings.
        //(There's some argument for there not being *any* try-because-contested functions. Bit of a holdover from how things used to work.)
        while (true)
        {
            if (Interlocked.CompareExchange(ref TaskLocker, 1, 0) != 0)
            {
                waiter.SpinOnce();
                continue;
            }
            try
            {
                //We have the lock.
                return Tasks.TryPop(out task);
            }
            finally
            {
                TaskLocker = 0;
            }
        }
    }

    /// <summary>
    /// Pushes a set of tasks onto the stack. Does not acquire a lock.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Dispatcher used to pull thread allocations if necessary.</param>
    /// <remarks>If the worker associated with this stack might be active, this function can only be called by the worker.</remarks>
    internal void PushUnsafely(Span<Task> tasks, IThreadDispatcher dispatcher)
    {
        Debug.Assert(tasks.Length > 0, "Probably shouldn't be trying to push zero tasks.");
        var newTaskCount = Tasks.Count + tasks.Length;

        if (Tasks.Span.length < newTaskCount)
        {
            //Doing a resize within a lock is *really* not great. This is something to be avoided via preallocation whenever possible.
            //BUT:
            //1. It should be trivial to make these resizes effectively never happen.
            //2. In the cases where it happens anyway, suffering the pain of a resize in a lock is the lesser of two evils.
            //In the case where we *didn't* resize, we'd have to either stall on the allocation attempt or cycle on other tasks until continuations are freed up.
            //But there's no guarantee that running tasks will actually free up continuations on net- they could make more!
            //So having the fallback plan of expanding storage for more continuations avoids deadlock prone options.
            //Note that *ONLY THE OWNING WORKER* can ever validly perform this resize!
            //We have to use the thread's buffer pool, and only the thread can validly access that pool.
            Tasks.Resize(int.Max(newTaskCount, Tasks.Span.length * 2), dispatcher.WorkerPools[WorkerIndex]);
#if DEBUG
            //While you shouldn't *need* to clear tasks, it can be useful for debug purposes.
            Tasks.Span.Clear(Tasks.Count, Tasks.Span.length - Tasks.Count);
#endif
        }

        Tasks.AddRangeUnsafely(tasks);
    }
    /// <summary>
    /// Pushes a task onto the task stack. Does not acquire a lock.
    /// </summary>
    /// <param name="task">Task to push</param>
    /// <param name="dispatcher">Dispatcher used to pull thread allocations if necessary.</param>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops.</remarks>
    public void PushUnsafely(Task task, IThreadDispatcher dispatcher)
    {
        PushUnsafely(new Span<Task>(&task, 1), dispatcher);
    }

    /// <summary>
    /// Tries to push a set of tasks to the task stack if the stack is uncontested.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Dispatcher used to pull thread allocations if necessary.</param>
    /// <returns>True if the push succeeded, false if the push was contested.</returns>
    /// <remarks>This must only be called from the associated worker if the worker is potentially active.</remarks>
    public bool TryPush(Span<Task> tasks, IThreadDispatcher dispatcher)
    {
        if (Interlocked.CompareExchange(ref TaskLocker, 1, 0) != 0)
            return false;
        try
        {
            //We have the lock.
            PushUnsafely(tasks, dispatcher);
            return true;
        }
        finally
        {
            TaskLocker = 0;
        }
    }

    /// <summary>
    /// Push a set of tasks to the task stack.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Dispatcher used to pull thread allocations if necessary.</param>
    /// <remarks>This must only be called from the associated worker if the worker is potentially active.</remarks>
    public void Push(Span<Task> tasks, IThreadDispatcher dispatcher)
    {
        var waiter = new SpinWait();
        while (!TryPush(tasks, dispatcher))
        {
            waiter.SpinOnce(-1);
        }
    }


    /// <summary>
    /// Attempts to allocate a continuation for a set of tasks.
    /// </summary>
    /// <param name="taskCount">Number of tasks associated with the continuation.</param>
    /// <param name="dispatcher">Dispatcher from which to pull a buffer pool if needed for resizing.</param>
    /// <param name="onCompleted">Function to execute upon completing all associated tasks, if any. Any task with a null <see cref="Task.Function"/> will not be executed.</param>
    /// <param name="continuationHandle">Handle of the continuation if allocation is successful.</param>
    /// <returns>True if the continuation was allocated, false if the attempt was contested..</returns>
    public bool TryAllocateContinuation(int taskCount, IThreadDispatcher dispatcher, out ContinuationHandle continuationHandle, Task onCompleted = default)
    {
        continuationHandle = default;
        if (Interlocked.CompareExchange(ref ContinuationLocker, 1, 0) != 0)
            return false;
        try
        {
            //We have the lock.
            if (ContinuationCount == Continuations.length)
            {
                //Doing a resize within a lock is *really* not great. This is something to be avoided via preallocation whenever possible.
                //BUT:
                //1. It should be trivial to make these resizes effectively never happen.
                //2. In the cases where it happens anyway, suffering the pain of a resize in a lock is the lesser of two evils.
                //In the case where we *didn't* resize, we'd have to either stall on the allocation attempt or cycle on other tasks until continuations are freed up.
                //But there's no guarantee that running tasks will actually free up continuations on net- they could make more!
                //So having the fallback plan of expanding storage for more continuations avoids deadlock prone options.
                //Note that *ONLY THE OWNING WORKER* can ever validly perform this resize!
                //We have to use the thread's buffer pool, and only the thread can validly access that pool.
                var workerPool = dispatcher.WorkerPools[WorkerIndex];
                workerPool.ResizeToAtLeast(ref Continuations, ContinuationCount * 2, ContinuationCount);
                ContinuationIndexPool.Resize(Continuations.length, workerPool);
#if DEBUG
                //While you shouldn't *need* to clear continuations, it can be useful for debug purposes.
                Continuations.Clear(ContinuationCount, Continuations.length - ContinuationCount);
#endif
            }
            var index = ContinuationIndexPool.Take();
            ++ContinuationCount;
            ref var continuation = ref Continuations[index];
            //Note that the version number could be based on undefined data initially. That's actually fine; all we care about is whether it is different.
            //Note mask to leave a valid bit for encoding in the handle.
            var newVersion = (continuation.Version + 1) & (~(1 << 31));
            continuation.OnCompleted = onCompleted;
            continuation.Version = newVersion;
            continuation.RemainingTaskCounter = taskCount;
            continuationHandle = new ContinuationHandle((uint)index, newVersion, (WorkerTaskStack*)Unsafe.AsPointer(ref this));
            return true;
        }
        finally
        {
            ContinuationLocker = 0;
        }
    }


    /// <summary>
    /// Allocates a continuation for a set of tasks.
    /// </summary>
    /// <param name="taskCount">Number of tasks associated with the continuation.</param>
    /// <param name="dispatcher">Dispatcher from which to pull any thread allocations if necessary.</param>
    /// <param name="onCompleted">Task to execute upon completing all associated tasks, if any. Any task with a null <see cref="Task.Function"/> will not be executed.</param>
    /// <returns>Handle of the allocated continuation.</returns>
    /// <remarks>Note that this will keep trying until allocation succeeds. If something is blocking allocation, such as insufficient room in the continuations buffer and there are no workers consuming tasks, this will block forever.</remarks>
    public ContinuationHandle AllocateContinuation(int taskCount, IThreadDispatcher dispatcher, Task onCompleted = default)
    {
        var waiter = new SpinWait();
        ContinuationHandle handle;
        while (!TryAllocateContinuation(taskCount, dispatcher, out handle, onCompleted))
        {
            waiter.SpinOnce(-1);
        }
        return handle;
    }

    /// <summary>
    /// Retrieves a pointer to the continuation data for <see cref="ContinuationHandle"/>.
    /// </summary>
    /// <param name="continuationHandle">Handle to look up the associated continuation for.</param>
    /// <returns>Pointer to the continuation backing the given handle.</returns>
    /// <remarks>This should not be used if the continuation handle is not known to be valid. The data pointed to by the data could become invalidated if the continuation completes.</remarks>
    public TaskContinuation* GetContinuation(ContinuationHandle continuationHandle)
    {
        Debug.Assert(continuationHandle.Initialized, "This continuation handle was never initialized.");
        Debug.Assert(continuationHandle.Index < Continuations.length, "This continuation refers to an invalid index.");
        if (continuationHandle.Index >= Continuations.length || !continuationHandle.Initialized)
            return null;
        var continuation = Continuations.Memory + continuationHandle.Index;
        Debug.Assert(continuation->Version == continuationHandle.Version, "This continuation no longer refers to an active continuation.");
        if (continuation->Version != continuationHandle.Version)
            return null;
        return Continuations.Memory + continuationHandle.Index;
    }

    /// <summary>
    /// Checks whether all tasks associated with this continuation have completed.
    /// </summary>
    /// <param name="continuationHandle">Continuation to check for completion.</param>
    /// <returns>True if all tasks associated with a continuation have completed, false otherwise.</returns>
    public bool IsComplete(ContinuationHandle continuationHandle)
    {
        Debug.Assert(continuationHandle.Initialized, "This continuation handle was never initialized.");
        Debug.Assert(continuationHandle.Index < Continuations.length, "This continuation refers to an invalid index.");
        if (continuationHandle.Index >= Continuations.length || !continuationHandle.Initialized)
            return false;
        ref var continuation = ref Continuations[continuationHandle.Index];
        return continuation.Version > continuationHandle.Version || continuation.RemainingTaskCounter == 0;
    }

    internal void Reset()
    {
        Tasks.Count = 0;
        Debug.Assert(TaskLocker == 0, "There appears to be a thread actively working still. That's invalid.");
#if DEBUG
        //While you shouldn't *need* to clear continuations, it can be useful for debug purposes.
        Continuations.Clear(0, Continuations.length);
#endif
        ContinuationCount = 0;
        Debug.Assert(ContinuationLocker == 0, "There appears to be a thread actively working still. That's invalid.");
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
    /// Version of this continuation.
    /// </summary>
    public int Version;
    /// <summary>
    /// Number of tasks not yet reported as complete in the continuation.
    /// </summary>
    public int RemainingTaskCounter;
}


/// <summary>
/// Manages task stacks for parallel workers. Supports work stealing between workers.
/// </summary>
public unsafe struct ParallelTaskStack
{
    Buffer<WorkerTaskStack> workerStacks;

    [StructLayout(LayoutKind.Explicit, Size = 256 + 16)]
    struct StopPad
    {
        [FieldOffset(128)]
        public volatile bool Stop;
    }
    StopPad padded;

    /// <summary>
    /// Constructs a new parallel task stack.
    /// </summary>
    /// <param name="pool">Buffer pool to allocate non-thread allocated resources from.</param>
    /// <param name="dispatcher">Thread dispatcher to pull thread pools from for thread allocations.</param>
    /// <param name="workerCount">Number of workers to allocate space for.</param>
    /// <param name="initialWorkerTaskCapacity">Initial number of tasks to allocate space for in each worker. Tasks are individual chunks of scheduled work..</param>
    /// <param name="initialWorkerContinuationCapacity">Initial number of continuations to allocate space for in each worker.</param>
    public ParallelTaskStack(BufferPool pool, IThreadDispatcher dispatcher, int workerCount, int initialWorkerTaskCapacity = 64, int initialWorkerContinuationCapacity = 16)
    {
        pool.Take(workerCount, out workerStacks);
        for (int i = 0; i < workerCount; ++i)
        {
            workerStacks[i] = new WorkerTaskStack(i, dispatcher, initialWorkerTaskCapacity, initialWorkerContinuationCapacity);
        }
        Reset();
    }

    /// <summary>
    /// Returns the stack to a fresh state without reallocating.
    /// </summary>
    public void Reset()
    {
        for (int i = 0; i < workerStacks.Length; ++i)
        {
            workerStacks[i].Reset();
        }
        padded.Stop = false;
    }

    /// <summary>
    /// Returns unmanaged resources held by the <see cref="ParallelTaskStack"/> to a pool.
    /// </summary>
    /// <param name="pool">Buffer pool to return resources to.</param>
    public void Dispose(BufferPool pool)
    {
        for (int i = 0; i < workerStacks.Length; ++i)
        {
            workerStacks[i].Reset();
        }
        pool.Return(ref workerStacks);
    }

    /// <summary>
    /// Gets the approximate number of active tasks. This is not guaranteed to actually measure the true number of tasks at any one point in time; it checks each worker in sequence, and the task counts could vary arbitrarily as the checks proceed.
    /// </summary>
    public int ApproximateTaskCount
    {
        get
        {
            int sum = 0;
            for (int i = 0; i < workerStacks.Length; ++i)
            {
                sum += workerStacks[i].Tasks.Count;
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
            for (int i = 0; i < workerStacks.Length; ++i)
            {
                sum += workerStacks[i].ContinuationCount;
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
    /// <param name="continuationHandle">Handle of the continuation if allocation is successful.</param>
    /// <returns>True if the allocation succeeded, false if it was contested..</returns>
    public bool TryAllocateContinuation(int taskCount, int workerIndex, IThreadDispatcher dispatcher, out ContinuationHandle continuationHandle, Task onCompleted = default)
    {
        return workerStacks[workerIndex].TryAllocateContinuation(taskCount, dispatcher, out continuationHandle, onCompleted);
    }

    /// <summary>
    /// Allocates a continuation for a set of tasks.
    /// </summary>
    /// <param name="taskCount">Number of tasks associated with the continuation.</param>
    /// <param name="workerIndex">Worker index to allocate the continuation on.</param>
    /// <param name="dispatcher">Dispatcher to use for any per-thread allocations if necessary.</param>
    /// <param name="onCompleted">Task to execute upon completing all associated tasks, if any. Any task with a null <see cref="Task.Function"/> will not be executed.</param>
    /// <returns>Handle of the allocated continuation.</returns>
    public ContinuationHandle AllocateContinuation(int taskCount, int workerIndex, IThreadDispatcher dispatcher, Task onCompleted = default)
    {
        return workerStacks[workerIndex].AllocateContinuation(taskCount, dispatcher, onCompleted);
    }

    /// <summary>
    /// Attempts to pop a task.
    /// </summary>
    /// <param name="workerIndex">Index of the worker executing this function.</param>
    /// <param name="task">Popped task, if any.</param>
    /// <returns>Result status of the pop attempt.</returns>
    public PopTaskResult TryPop(int workerIndex, out Task task)
    {
        //Walk through the worker stacks looking for work. Start with the current worker.
        int peekedWorkerIndex = workerIndex;
        for (int i = 0; i < workerStacks.length; ++i)
        {
            if (workerStacks[peekedWorkerIndex].TryPop(out task))
            {
                return PopTaskResult.Success;
            }
            ++peekedWorkerIndex;
            if (peekedWorkerIndex >= workerStacks.length)
                peekedWorkerIndex -= workerStacks.length;
        }
        //No worker had anything!
        task = default;
        return padded.Stop ? PopTaskResult.Stop : PopTaskResult.Empty;
    }

    /// <summary>
    /// Attempts to pop a task and run it.
    /// </summary>
    /// <param name="workerIndex">Index of the worker to pass into the task function.</param>
    /// <param name="dispatcher">Thread dispatcher running this task stack.</param>
    /// <returns>Result status of the pop attempt.</returns>
    public PopTaskResult TryPopAndRun(int workerIndex, IThreadDispatcher dispatcher)
    {
        var result = TryPop(workerIndex, out var task);
        if (result == PopTaskResult.Success)
        {
            task.Run(workerIndex, dispatcher);
        }
        return result;
    }

    /// <summary>
    /// Pushes a set of tasks onto a worker's task stack. Does not acquire a lock.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops that could affect the specified worker.</remarks>
    public void PushUnsafely(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher)
    {
        workerStacks[workerIndex].PushUnsafely(tasks, dispatcher);
    }
    /// <summary>
    /// Pushes a task onto a worker's task stack. Does not acquire a lock.
    /// </summary>
    /// <param name="task">Task to push.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops that could affect the specified worker.</remarks>
    public unsafe void PushUnsafely(Task task, int workerIndex, IThreadDispatcher dispatcher)
    {
        workerStacks[workerIndex].PushUnsafely(new Span<Task>(&task, 1), dispatcher);
    }

    /// <summary>
    /// Tries to pushes a set of tasks onto a worker's task stack.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <returns>True if the push succeeded, false if it was contested.</returns>
    public bool TryPush(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher)
    {
        return workerStacks[workerIndex].TryPush(tasks, dispatcher);
    }

    /// <summary>
    /// Pushes a set of tasks onto a worker's task stack.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    public void Push(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher)
    {
        workerStacks[workerIndex].Push(tasks, dispatcher);
    }

    /// <summary>
    /// Pushes a set of tasks to the stack with a created continuation.
    /// </summary>
    /// <param name="tasks">Tasks composing the job. A continuation will be assigned internally; no continuation should be present on any of the provided tasks.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    /// <param name="onComplete">Task to run upon completion of all the submitted tasks, if any.</param>
    /// <returns>Handle of the continuation created for these tasks.</returns>
    /// <remarks>Note that this will keep trying until task submission succeeds.</remarks>
    public ContinuationHandle AllocateContinuationAndPush(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher, Task onComplete = default)
    {
        var continuationHandle = AllocateContinuation(tasks.Length, workerIndex, dispatcher, onComplete);
        for (int i = 0; i < tasks.Length; ++i)
        {
            ref var task = ref tasks[i];
            Debug.Assert(!task.Continuation.Initialized, "This function creates a continuation for the tasks");
            task.Continuation = continuationHandle;
        }
        Push(tasks, workerIndex, dispatcher);
        return continuationHandle;
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
        var waiter = new SpinWait();
        Debug.Assert(continuation.Initialized, "This codepath should only run if the continuation was allocated earlier.");
        while (!continuation.Completed)
        {
            var result = TryPop(workerIndex, out var fillerTask);
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
    /// Pushes a set of tasks to the worker stack and returns when all tasks are complete.
    /// </summary>
    /// <param name="tasks">Tasks composing the job. A continuation will be assigned internally; no continuation should be present on any of the provided tasks.</param>
    /// <param name="workerIndex">Index of the worker executing this function.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <remarks>Note that this will keep working until all tasks are run. It may execute tasks unrelated to the requested tasks while waiting on other workers to complete constituent tasks.</remarks>
    public void RunTasks(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher)
    {
        if (tasks.Length == 0)
            return;
        ContinuationHandle continuationHandle = default;
        if (tasks.Length > 1)
        {
            //Note that we only submit tasks to the stack for tasks beyond the first. The current thread is responsible for at least task 0.
            var taskCount = tasks.Length - 1;
            Span<Task> tasksToPush = stackalloc Task[taskCount];
            ref var worker = ref workerStacks[workerIndex];
            continuationHandle = worker.AllocateContinuation(taskCount, dispatcher);
            for (int i = 0; i < tasksToPush.Length; ++i)
            {
                var task = tasks[i + 1];
                Debug.Assert(continuationHandle.WorkerStack == workerStacks.Memory + workerIndex);
                Debug.Assert(!task.Continuation.Initialized, $"None of the source tasks should have continuations when provided to {nameof(RunTasks)}.");
                task.Continuation = continuationHandle;
                tasksToPush[i] = task;
            }
            worker.Push(tasksToPush, dispatcher);
        }
        //Tasks [1, count) are submitted to the stack and may now be executing on other workers.
        //The thread calling the for loop should not relinquish its timeslice. It should immediately begin working on task 0.
        var task0 = tasks[0];
        Debug.Assert(!task0.Continuation.Initialized, $"None of the source tasks should have continuations when provided to {nameof(RunTasks)}.");
        task0.Function(task0.Id, task0.Context, workerIndex, dispatcher);

        if (tasks.Length > 1)
        {
            //Task 0 is done; this thread should seek out other work until the job is complete.
            WaitForCompletion(continuationHandle, workerIndex, dispatcher);
        }
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
    /// <param name="continuation">Continuation associated with the loop tasks, if any.</param>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops that could affect the specified worker.</remarks>
    public void PushForUnsafely(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, int workerIndex, IThreadDispatcher dispatcher, ContinuationHandle continuation = default)
    {
        Span<Task> tasks = stackalloc Task[iterationCount];
        for (int i = 0; i < tasks.Length; ++i)
        {
            tasks[i] = new Task { Function = function, Context = context, Id = i + inclusiveStartIndex, Continuation = continuation };
        }
        PushUnsafely(tasks, workerIndex, dispatcher);
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
    /// <remarks>This function will not usually attempt to run any iterations of the loop itself. It tries to push the loop tasks onto the stack.<para/>
    /// If the task stack is full, this will opt to run the tasks inline while waiting for room.</remarks>
    public void PushFor(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, int workerIndex, IThreadDispatcher dispatcher, ContinuationHandle continuation = default)
    {
        Span<Task> tasks = stackalloc Task[iterationCount];
        for (int i = 0; i < tasks.Length; ++i)
        {
            tasks[i] = new Task { Function = function, Context = context, Id = i + inclusiveStartIndex, Continuation = continuation };
        }
        Push(tasks, workerIndex, dispatcher);
    }

    /// <summary>
    /// Submits a set of tasks representing a for loop over the given indices and returns when all loop iterations are complete.
    /// </summary>
    /// <param name="function">Function to execute on each iteration of the loop.</param>
    /// <param name="context">Context pointer to pass into each iteration of the loop.</param>
    /// <param name="inclusiveStartIndex">Inclusive start index of the loop range.</param>
    /// <param name="iterationCount">Number of iterations to perform.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the tasks onto.</param>
    public void For(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, int workerIndex, IThreadDispatcher dispatcher)
    {
        if (iterationCount <= 0)
            return;
        Span<Task> tasks = stackalloc Task[iterationCount];
        for (int i = 0; i < tasks.Length; ++i)
        {
            tasks[i] = new Task(function, context, inclusiveStartIndex + i);
        }
        RunTasks(tasks, workerIndex, dispatcher);
    }
}
