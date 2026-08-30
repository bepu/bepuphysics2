using System.Diagnostics;

namespace BepuUtilities.TaskScheduling;

/// <summary>
/// Description of a task to be submitted to a <see cref="TaskStack"/>.
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
        Debug.Assert(!Continuation.Completed && Function != null);
        Function(Id, Context, workerIndex, dispatcher);
        if (Continuation.Initialized)
            Continuation.NotifyTaskCompleted(workerIndex, dispatcher);
    }
}
