namespace BepuUtilities.TaskScheduling;

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
