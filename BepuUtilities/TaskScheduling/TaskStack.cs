using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities.TaskScheduling;

/// <summary>
/// Manages a linked stack of tasks.
/// </summary>
public unsafe struct TaskStack
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
    public TaskStack(BufferPool pool, IThreadDispatcher dispatcher, int workerCount, int initialWorkerJobCapacity = 128, int continuationBlockCapacity = 128)
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
    /// Returns unmanaged resources held by the <see cref="TaskStack"/> to a pool.
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
            var job = (Job*)head;
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
    public void PushUnsafely(Task task, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
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
    /// Pushes a task onto the task stack.
    /// </summary>
    /// <param name="task">Task composing the job.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the task onto.</param>
    /// <param name="tag">User-defined tag data for the submitted job.</param>
    /// <returns>True if the push succeeded, false if it was contested.</returns>
    public void Push(Task task, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
    {
        Push(new Span<Task>(ref task), workerIndex, dispatcher, tag);
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
    /// Pushes a task to the stack with a created continuation.
    /// </summary>
    /// <param name="task">Task composing the job. A continuation will be assigned internally; no continuation should be present on any of the provided task.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="workerIndex">Index of the worker stack to push the task onto.</param>
    /// <param name="tag">User tag associated with the task's job.</param>
    /// <param name="onComplete">Task to run upon completion of all the submitted task, if any.</param>
    /// <returns>Handle of the continuation created for these task.</returns>
    public ContinuationHandle AllocateContinuationAndPush(Task task, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0, Task onComplete = default)
    {
        return AllocateContinuationAndPush(new Span<Task>(ref task), workerIndex, dispatcher, tag, onComplete);
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
    /// Pushes a task to the worker stack and returns when it completes.
    /// </summary>
    /// <param name="task">Task composing the job. A continuation will be assigned internally; no continuation should be present on the task.</param>
    /// <param name="workerIndex">Index of the worker executing this function.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted task.</param>
    /// <remarks>Note that this will keep working until the task completes. It may execute tasks unrelated to the requested task while waiting on other workers.</remarks>
    public void RunTask(Task task, int workerIndex, IThreadDispatcher dispatcher, ulong tag = 0)
    {
        RunTasks(new Span<Task>(ref task), workerIndex, dispatcher, tag);
    }

    /// <summary>
    /// Pushes a task to the worker stack and returns when all tasks are complete.
    /// </summary>
    /// <param name="task">Task composing the job. A continuation will be assigned internally; no continuation should be present on the task.</param>
    /// <param name="workerIndex">Index of the worker executing this function.</param>
    /// <param name="dispatcher">Thread dispatcher to allocate thread data from if necessary.</param>
    /// <param name="filter">Filter applied to jobs considered for filling the calling thread's wait for other threads to complete.</param>
    /// <param name="tag">User tag associated with the job spanning the submitted task.</param>
    /// <typeparam name="TJobFilter">Type of the job filter used in the pop.</typeparam>
    /// <remarks>Note that this will keep working the task completes. It may execute tasks unrelated to the requested task while waiting on other workers to complete constituent tasks.</remarks>
    public void RunTask<TJobFilter>(Task task, int workerIndex, IThreadDispatcher dispatcher, ref TJobFilter filter, ulong tag = 0) where TJobFilter : IJobFilter
    {
        RunTasks(new Span<Task>(ref task), workerIndex, dispatcher, ref filter, tag);
    }

    /// <summary>
    /// Requests that all workers stop. The next time a worker runs out of tasks to run, if it sees a stop command, it will be reported.
    /// </summary>
    public void RequestStop()
    {
        padded.Stop = true;
    }

    /// <summary>
    /// Convenience function for requesting a stop. Requires the context to be the expected <see cref="TaskStack"/>.
    /// </summary>
    /// <param name="id">Id of the task.</param>
    /// <param name="untypedContext"><see cref="TaskStack"/> to be stopped.</param>
    /// <param name="workerIndex">Index of the worker executing this task.</param>
    /// <param name="dispatcher">Dispatcher associated with the execution.</param>
    public static void RequestStopTaskFunction(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ((TaskStack*)untypedContext)->RequestStop();
    }

    /// <summary>
    /// Convenience function for getting a task representing a stop request.
    /// </summary>
    /// <param name="stack">Stack to be stopped.</param>
    /// <returns>Task representing a stop request.</returns>
    public static Task GetRequestStopTask(TaskStack* stack) => new(&RequestStopTaskFunction, stack);

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

    /// <summary>
    /// Worker function that pops tasks from the stack and executes them.
    /// </summary>
    /// <param name="workerIndex">Index of the worker calling this function.</param>
    /// <param name="dispatcher">Thread dispatcher responsible for the invocation.</param>
    public static void DispatchWorkerFunction(int workerIndex, IThreadDispatcher dispatcher)
    {
        var taskStack = (TaskStack*)dispatcher.UnmanagedContext;
        PopTaskResult popTaskResult;
        var waiter = new SpinWait();
        while ((popTaskResult = taskStack->TryPopAndRun(workerIndex, dispatcher)) != PopTaskResult.Stop)
        {
            waiter.SpinOnce(-1);
        }
    }

    /// <summary>
    /// Dispatches workers to execute tasks from the given stack.
    /// </summary>
    /// <param name="taskStack">Task stack to pull work from.</param>
    /// <param name="dispatcher">Thread dispatcher to dispatch workers with.</param>
    /// <param name="maximumWorkerCount">Maximum number of workers to spin up for the dispatch.</param>
    /// <param name="managedContext">Managed context to include in this dispatch, if any.</param>
    public static void DispatchWorkers(IThreadDispatcher dispatcher, TaskStack* taskStack, int maximumWorkerCount = int.MaxValue, object managedContext = null)
    {
        dispatcher.DispatchWorkers(&DispatchWorkerFunction, maximumWorkerCount, taskStack, managedContext);
    }
}
