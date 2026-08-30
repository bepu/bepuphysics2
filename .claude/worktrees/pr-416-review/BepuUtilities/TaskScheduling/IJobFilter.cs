using System;

namespace BepuUtilities.TaskScheduling;

/// <summary>
/// Determines which jobs are allowed to serve a <see cref="TaskStack.TryPop{TJobFilter}(ref TJobFilter, out Task)"/> request.
/// </summary>
public interface IJobFilter
{
    /// <summary>
    /// Determines whether a job with the given tag should be allowed to serve a <see cref="TaskStack.TryPop{TJobFilter}(ref TJobFilter, out Task)"/> request.
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