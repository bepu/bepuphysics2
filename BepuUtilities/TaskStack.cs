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
    /// The pop was contested.
    /// </summary>
    Contested,
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
/// Describes the result of a task enqueue attempt.
/// </summary>
public enum PushTaskResult
{
    /// <summary>
    /// The tasks were successfully pushed.
    /// </summary>
    Success,
    /// <summary>
    /// The push attempt was blocked by concurrent access.
    /// </summary>
    Contested,
    /// <summary>
    /// The push attempt was blocked because no space remained in the tasks buffer.
    /// </summary>
    Full,
}


/// <summary>
/// Refers to a continuation within a <see cref="TaskStack"/>.
/// </summary>
public unsafe struct ContinuationHandle : IEquatable<ContinuationHandle>
{
    uint index;
    uint encodedVersion;
    /// <summary>
    /// Source worker task stack.
    /// </summary>
    internal TaskStackContinuations* Continuations;

    internal ContinuationHandle(uint index, int version, TaskStackContinuations* continuations)
    {
        this.index = index;
        encodedVersion = (uint)version | 1u << 31;
        Continuations = continuations;
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
    public bool Completed => Continuations->IsComplete(this);

    /// <summary>
    /// Retrieves a pointer to the continuation data for <see cref="ContinuationHandle"/>.
    /// </summary>
    /// <returns>Pointer to the continuation backing the given handle.</returns>
    /// <remarks>This should not be used if the continuation handle is not known to be valid. The data pointed to by the data could become invalidated if the continuation completes.</remarks>
    public TaskContinuation* Continuation => Continuations->GetContinuation(this);

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
            if (!Initialized || Continuations == null)
                return false;
            ref var continuation = ref Continuations->Continuations[Index];
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
            while (Interlocked.CompareExchange(ref Continuations->ContinuationLocker, 1, 0) != 0)
            {
                waiter.SpinOnce(-1);
            }
            //We have the lock.
            Continuations->ContinuationIndexPool.ReturnUnsafely((int)Index);
            --Continuations->ContinuationCount;
            Continuations->ContinuationLocker = 0;
        }
    }

    public bool Equals(ContinuationHandle other) => other.index == index && other.encodedVersion == encodedVersion && other.Continuations == Continuations;

    public override bool Equals([NotNullWhen(true)] object obj) => obj is ContinuationHandle handle && Equals(handle);

    public override int GetHashCode() => (int)(index ^ (encodedVersion << 24));

    public static bool operator ==(ContinuationHandle left, ContinuationHandle right) => left.Equals(right);

    public static bool operator !=(ContinuationHandle left, ContinuationHandle right) => !(left == right);
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

unsafe struct TaskStackContinuations
{
    public Buffer<TaskContinuation> Continuations;
    public IdPool ContinuationIndexPool;
    public int ContinuationCount;
    public volatile int ContinuationLocker;

    public TaskStackContinuations(BufferPool pool, int maximumCapacity)
    {
        pool.Take(maximumCapacity, out Continuations);
        ContinuationIndexPool = new IdPool(maximumCapacity, pool);
    }

    public void Dispose(BufferPool pool)
    {
        pool.Return(ref Continuations);
        ContinuationIndexPool.Dispose(pool);
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
}

/// <summary>
/// Manages task stacks for parallel workers. Supports work stealing between workers.
/// </summary>
public unsafe struct TaskStack
{
    [StructLayout(LayoutKind.Explicit, Size = 256 + 16)]
    struct StopPad
    {
        [FieldOffset(128)]
        public volatile bool Stop;
    }
    StopPad padded;


    Buffer<TaskStackContinuations> continuations;
    public QuickList<Task> Tasks;
    public volatile int TaskLocker;

    /// <summary>
    /// Constructs a new parallel task stack.
    /// </summary>
    /// <param name="pool">Buffer pool to allocate non-thread allocated resources from.</param>
    /// <param name="dispatcher">Thread dispatcher calling this TaskStack.</param>
    /// <param name="maximumTaskCapacity">Maximum number of tasks to allocate space for in each worker. Tasks are individual chunks of scheduled work..</param>
    /// <param name="maximumContinuationCapacity">Maximum number of continuations to allocate space for in each worker.</param>
    public TaskStack(BufferPool pool, IThreadDispatcher dispatcher, int maximumTaskCapacity = 512, int maximumContinuationCapacity = 512)
    {
        Tasks = new QuickList<Task>(maximumTaskCapacity, pool);
        pool.Take(1, out continuations);
        continuations[0] = new TaskStackContinuations(pool, maximumContinuationCapacity);
#if DEBUG
        //While you shouldn't *need* to clear continuations, it can be useful for debug purposes.
        continuations[0].Continuations.Clear(0, continuations[0].Continuations.length);
        Tasks.Span.Clear(0, Tasks.Span.length);
#endif
        Reset();
    }

    /// <summary>
    /// Returns unmanaged resources held by the <see cref="TaskStack"/> to a pool.
    /// </summary>
    /// <param name="pool">Buffer pool to return resources to.</param>
    public void Dispose(BufferPool pool)
    {
        Tasks.Dispose(pool);
        continuations[0].Dispose(pool);
    }

    /// <summary>
    /// Returns the stack to a fresh state without reallocating.
    /// </summary>
    public void Reset()
    {
        Tasks.Count = 0;
        Debug.Assert(TaskLocker == 0, "There appears to be a thread actively working still. That's invalid.");
#if DEBUG
        //While you shouldn't *need* to clear continuations, it can be useful for debug purposes.
        continuations[0].Continuations.Clear(0, continuations[0].Continuations.length);
#endif
        continuations[0].ContinuationCount = 0;
        Debug.Assert(continuations[0].ContinuationLocker == 0, "There appears to be a thread actively working still. That's invalid.");
        padded.Stop = false;
    }

    /// <summary>
    /// Gets the number of active tasks.
    /// </summary>
    public int TaskCount
    {
        get
        {
            return Tasks.Count;
        }
    }
    /// <summary>
    /// Gets the number of active continuations.
    /// </summary>
    public int ContinuationCount
    {
        get
        {
            return continuations[0].ContinuationCount;
        }
    }

    /// <summary>
    /// Attempts to pop a task from the stack.
    /// </summary>
    /// <param name="task">Task popped from the stack, if any.</param>
    /// <returns>True if a task was available to pop, false otherwise.</returns>
    public PopTaskResult TryPop(out Task task)
    {
        //Quick early out to avoid lock expense.
        if (Tasks.Count == 0)
        {
            task = default;
            return padded.Stop ? PopTaskResult.Stop : PopTaskResult.Empty;
        }
        if (Interlocked.CompareExchange(ref TaskLocker, 1, 0) != 0)
        {
            task = default;
            return PopTaskResult.Contested;
        }
        try
        {
            //We have the lock.
            if (Tasks.TryPop(out task))
            {
                return PopTaskResult.Success;
            }
            return PopTaskResult.Empty;
        }
        finally
        {
            TaskLocker = 0;
        }
    }

    /// <summary>
    /// Pushes a set of tasks onto the stack. Does not acquire a lock.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <returns>True if there was room to complete the push, false otherwise.</returns>
    public bool TryPushUnsafely(Span<Task> tasks)
    {
        Debug.Assert(tasks.Length > 0, "Probably shouldn't be trying to push zero tasks.");
        if (tasks.Length + Tasks.Count > Tasks.Span.Length)
        {
            return false;
        }
        Tasks.AddRangeUnsafely(tasks);
        return true;
    }

    /// <summary>
    /// Pushes a task onto the task stack. Does not acquire a lock.
    /// </summary>
    /// <param name="task">Task to push</param>
    /// <returns>True if there was room to complete the push, false otherwise.</returns>
    public unsafe bool TryPushUnsafely(Task task)
    {
        return TryPushUnsafely(new Span<Task>(&task, 1));
    }

    /// <summary>
    /// Tries to push a set of tasks to the task stack if the stack is uncontested.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <returns>Status of the push attempt.</returns>
    public PushTaskResult TryPush(Span<Task> tasks)
    {
        if (Interlocked.CompareExchange(ref TaskLocker, 1, 0) != 0)
            return PushTaskResult.Contested;
        try
        {
            //We have the lock.
            return TryPushUnsafely(tasks) ? PushTaskResult.Success : PushTaskResult.Full;
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
    /// <param name="workerIndex">Worker index of the currently executing thread.</param>
    /// <param name="dispatcher">Currently executing thread dispatcher.</param>
    /// <remarks>Runs filler tasks if the push is blocked by a lack of capacity.</remarks>
    public void Push(Span<Task> tasks, int workerIndex, IThreadDispatcher dispatcher)
    {
        var waiter = new SpinWait();
        PushTaskResult result;
        while ((result = TryPush(tasks)) != PushTaskResult.Success)
        {
            if (result == PushTaskResult.Contested)
                waiter.SpinOnce(-1);
            else if (result == PushTaskResult.Full)
            {
                waiter.Reset();
                while (true)
                {
                    var popResult = TryPopAndRun(workerIndex, dispatcher);
                    Debug.Assert(popResult != PopTaskResult.Stop, "Should not try to push onto a stopped stack.");
                    if (popResult == PopTaskResult.Stop) return;
                    if (popResult != PopTaskResult.Contested) break;
                    waiter.SpinOnce(-1);
                }
                waiter.Reset();
            }
        }
    }

    /// <summary>
    /// Attempts to allocate a continuation for a set of tasks.
    /// </summary>
    /// <param name="taskCount">Number of tasks associated with the continuation.</param>
    /// <param name="onCompleted">Function to execute upon completing all associated tasks, if any. Any task with a null <see cref="Task.Function"/> will not be executed.</param>
    /// <param name="continuationHandle">Handle of the continuation if allocation is successful.</param>
    /// <returns>True if the continuation was allocated, false if the attempt was contested..</returns>
    public bool TryAllocateContinuation(int taskCount, out ContinuationHandle continuationHandle, Task onCompleted = default)
    {
        continuationHandle = default;
        ref var continuations = ref this.continuations[0];
        if (Interlocked.CompareExchange(ref continuations.ContinuationLocker, 1, 0) != 0)
            return false;
        try
        {
            //We have the lock.            
            var index = continuations.ContinuationIndexPool.Take();
            ++continuations.ContinuationCount;
            ref var continuation = ref continuations.Continuations[index];
            //Note that the version number could be based on undefined data initially. That's actually fine; all we care about is whether it is different.
            //Note mask to leave a valid bit for encoding in the handle.
            var newVersion = (continuation.Version + 1) & (~(1 << 31));
            continuation.OnCompleted = onCompleted;
            continuation.Version = newVersion;
            continuation.RemainingTaskCounter = taskCount;
            continuationHandle = new ContinuationHandle((uint)index, newVersion, this.continuations.Memory);
            return true;
        }
        finally
        {
            continuations.ContinuationLocker = 0;
        }
    }

    /// <summary>
    /// Allocates a continuation for a set of tasks.
    /// </summary>
    /// <param name="taskCount">Number of tasks associated with the continuation.</param>
    /// <param name="workerIndex">Index of the worker to pass into the task function.</param>
    /// <param name="dispatcher">Thread dispatcher running this task stack.</param>
    /// <param name="onCompleted">Task to execute upon completing all associated tasks, if any. Any task with a null <see cref="Task.Function"/> will not be executed.</param>
    /// <returns>Handle of the allocated continuation.</returns>
    /// <remarks>Note that this will keep trying until allocation succeeds.</remarks>
    public ContinuationHandle AllocateContinuation(int taskCount, int workerIndex, IThreadDispatcher dispatcher, Task onCompleted = default)
    {
        var waiter = new SpinWait();
        ContinuationHandle handle;
        while (!TryAllocateContinuation(taskCount, out handle, onCompleted))
        {
            var result = TryPopAndRun(workerIndex, dispatcher);
            Debug.Assert(result != PopTaskResult.Stop, "Should not attempt to allocate a continuation after a stop command.");
            if (result == PopTaskResult.Stop)
                return default;
            if (result == PopTaskResult.Success)
                waiter.Reset();
            else
                waiter.SpinOnce(-1);
        }
        return handle;
    }


    /// <summary>
    /// Attempts to pop a task and run it.
    /// </summary>
    /// <param name="workerIndex">Index of the worker to pass into the task function.</param>
    /// <param name="dispatcher">Thread dispatcher running this task stack.</param>
    /// <returns>Result status of the pop attempt.</returns>
    public PopTaskResult TryPopAndRun(int workerIndex, IThreadDispatcher dispatcher)
    {
        var result = TryPop(out var task);
        if (result == PopTaskResult.Success)
        {
            task.Run(workerIndex, dispatcher);
        }
        return result;
    }

    /// <summary>
    /// Pushes a set of tasks to the stack with a created continuation.
    /// </summary>
    /// <param name="tasks">Tasks composing the job. A continuation will be assigned internally; no continuation should be present on any of the provided tasks.</param>
    /// <param name="dispatcher">Thread dispatcher calling this TaskStack.</param>
    /// <param name="workerIndex">Index of the currently executing worker.</param>
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
    /// <param name="dispatcher">Thread dispatcher calling this TaskStack.</param>
    /// <param name="workerIndex">Index of the executing worker.</param>
    public void WaitForCompletion(ContinuationHandle continuation, int workerIndex, IThreadDispatcher dispatcher)
    {
        var waiter = new SpinWait();
        Debug.Assert(continuation.Initialized, "This codepath should only run if the continuation was allocated earlier.");
        while (!continuation.Completed)
        {
            var result = TryPop(out var fillerTask);
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
    /// <param name="dispatcher">Thread dispatcher calling this TaskStack.</param>
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
            continuationHandle = AllocateContinuation(taskCount, workerIndex, dispatcher);
            for (int i = 0; i < tasksToPush.Length; ++i)
            {
                var task = tasks[i + 1];
                Debug.Assert(continuationHandle.Continuations == this.continuations.Memory);
                Debug.Assert(!task.Continuation.Initialized, $"None of the source tasks should have continuations when provided to {nameof(RunTasks)}.");
                task.Continuation = continuationHandle;
                tasksToPush[i] = task;
            }
            Push(tasksToPush, workerIndex, dispatcher);
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
    /// <param name="continuation">Continuation associated with the loop tasks, if any.</param>
    /// <returns>True if there was enough room on the stack to push, false otherwise.</returns>
    /// <remarks>This must not be used while other threads could be performing task pushes or pops that could affect the specified worker.</remarks>
    public bool TryPushForUnsafely(delegate*<long, void*, int, IThreadDispatcher, void> function, void* context, int inclusiveStartIndex, int iterationCount, ContinuationHandle continuation = default)
    {
        Span<Task> tasks = stackalloc Task[iterationCount];
        for (int i = 0; i < tasks.Length; ++i)
        {
            tasks[i] = new Task { Function = function, Context = context, Id = i + inclusiveStartIndex, Continuation = continuation };
        }
        return TryPushUnsafely(tasks);
    }

    /// <summary>
    /// Pushes a for loop onto the task stack.
    /// </summary>
    /// <param name="function">Function to execute on each iteration of the loop.</param>
    /// <param name="context">Context pointer to pass into each task execution.</param>
    /// <param name="inclusiveStartIndex">Inclusive start index of the loop range.</param>
    /// <param name="iterationCount">Number of iterations to perform.</param>
    /// <param name="dispatcher">Thread dispatcher calling this TaskStack.</param>
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
    /// <param name="dispatcher">Thread dispatcher calling this TaskStack.</param>
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
