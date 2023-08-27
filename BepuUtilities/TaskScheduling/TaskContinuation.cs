namespace BepuUtilities.TaskScheduling;

/// <summary>
/// Stores data relevant to tracking task completion and reporting completion for a continuation.
/// </summary>
public struct TaskContinuation
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
