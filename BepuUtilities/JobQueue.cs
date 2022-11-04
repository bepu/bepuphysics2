using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using BepuUtilities.Memory;

namespace BepuUtilities;

/// <summary>
/// Refers to a job within a <see cref="JobQueue"/>.
/// </summary>
public struct JobHandle
{
    internal long JobIndex;
}

/// <summary>
/// Description of one task within a job to be submitted to a <see cref="JobQueue"/>.
/// </summary>
public unsafe struct TaskDescription
{
    /// <summary>
    /// Function to be executed by the task. Takes as arguments the <see cref="TaskId"/>, <see cref="Context"/> pointer, and executing worker index.
    /// </summary>
    public delegate*<int, void*, int, void> Function;
    /// <summary>
    /// Context to be passed into the <see cref="Function"/>.
    /// </summary>
    public void* Context;
    /// <summary>
    /// Identifier of this task within the job.
    /// </summary>
    public int TaskId;
}



public unsafe struct JobQueue
{
    internal struct Task
    {
        public delegate*<int, void*, int, void> Function;
        public void* Context;
        /// <summary>
        /// Identifier of this task within the job.
        /// </summary>
        public int TaskId;
        /// <summary>
        /// Index of the job to which this task belongs.
        /// </summary>
        public int JobIndex;
    }
    struct Job
    {
        public int TaskStart;
        public int TaskEnd;
        public int CompletionCounter;
        /// <summary>
        /// Because we use a ring buffer, we will eventually reuse slots.
        /// Jobs are given long identifiers to track them over time. This version will store the number of times the queue has wrapped around the jobs buffer before creating this task.
        /// </summary>
        public int Version;
    }

    public enum DequeueResult
    {
        /// <summary>
        /// A task was successfully dequeued.
        /// </summary>
        Success,
        /// <summary>
        /// The dequeue attempt was contested.
        /// </summary>
        Contested,
        /// <summary>
        /// The job queue was empty, but may have more tasks in the future.
        /// </summary>
        Empty,
        /// <summary>
        /// The job queue has been terminated and all threads seeking work should stop.
        /// </summary>
        Stop
    }



    Buffer<Job> jobs;

    Buffer<Task> tasks;

    int jobMask, jobShift;

    //TODO: Careful about false sharing.
    long taskIndex;
    long allocatedTaskIndex;
    long writtenTaskIndex;

    volatile int locker;

    /// <summary>
    /// Constructs a new job queue.
    /// </summary>
    /// <param name="pool">Buffer pool to allocate resources from.</param>
    /// <param name="maximumJobCapacity">Maximum number of jobs to allocate space for. If more jobs exist at any one moment, attempts to create new jobs will have to stall until space is available. Rounds up to the nearest equal or larger power of 2.</param>
    /// <param name="maximumTaskCapacity">Maximum number of tasks to allocate space for. Tasks are groupings of jobs that can be waited on.</param>
    public JobQueue(BufferPool pool, int maximumJobCapacity = 1024, int maximumTaskCapacity = 256)
    {
        maximumJobCapacity = (int)BitOperations.RoundUpToPowerOf2((uint)maximumJobCapacity);
        pool.Take(maximumJobCapacity, out jobs);
        pool.Take(maximumJobCapacity, out tasks);
        tasks.Clear(0, tasks.Length);
        var jobMask = jobs.length - 1;
        var jobShift = BitOperations.TrailingZeroCount(jobs.length);
    }

    public void Dispose(BufferPool pool)
    {
        pool.Return(ref jobs);
        pool.Return(ref tasks);
    }

    public DequeueResult TryDequeue(out delegate*<int, void*, int, void> function, out void* context, out int taskId)
    {
        function = default;
        context = default;
        taskId = default;
        if (Interlocked.CompareExchange(ref locker, 1, 0) != 0)
            return DequeueResult.Contested;
        try
        {
            //We have the lock.
            var nextTaskIndex = taskIndex;
            if (nextTaskIndex >= writtenTaskIndex)
                return DequeueResult.Empty;
            var task = tasks[(int)(nextTaskIndex & jobMask)];
            if (task.JobIndex < 0)
                return DequeueResult.Stop;
            //There's an actual job!
            function = task.Function;
            context = task.Context;
            taskId = task.TaskId;
            ++taskIndex;
            return DequeueResult.Success;
        }
        finally
        {
            locker = 0;
        }
    }

    /// <summary>
    /// Checks whether all tasks composing a job have completed.
    /// </summary>
    /// <param name="jobHandle">Job to check for completion.</param>
    /// <returns>True if the job has completed, false otherwise.</returns>
    public bool TaskIsComplete(JobHandle jobHandle)
    {
        var version = jobHandle.JobIndex >> jobShift;
        ref var job = ref jobs[(int)(jobHandle.JobIndex & jobMask)];
        return job.Version == version && job.CompletionCounter == 0;
    }

    /// <summary>
    /// Tries to appends a task to the job queue if the ring buffer is uncontested. Will return false if contested or if there is no room left.
    /// </summary>
    /// <param name="tasks">Tasks composing the job.</param>
    /// <param name="jobHandle">Handle representing the created job if the enqueue succeeded, invalid handle otherwise.</param>
    public bool TryEnqueueJob(Span<TaskDescription> tasks, out JobHandle jobHandle)
    {
        jobHandle.JobIndex = -1;
        if (Interlocked.CompareExchange(ref locker, 1, 0) != 0)
            return false;
        try
        {
            //We have the lock.
            Debug.Assert(this.tasks[(int)writtenTaskIndex].JobIndex >= 0, "No more jobs should be written after a stop command.");
            var taskStartIndex = allocatedTaskIndex;
            var taskEndIndex = taskStartIndex + tasks.Length;
            allocatedTaskIndex = taskEndIndex;
            var longLength = (long)this.tasks.length;
            if (taskEndIndex - taskIndex > longLength)
            {
                //We've run out of space in the ring buffer. If we tried to write, we'd overwrite jobs that haven't yet been completed.
                return false;
            }
            //We can actually write the jobs.
            Debug.Assert(BitOperations.IsPow2(jobs.Length));
            long mask = longLength - 1;
            //jobHandle.JobIndex = ; //TODO: Not done!
            var jobIndex = (int)(jobHandle.JobIndex & mask);
            for (int i = 0; i < tasks.Length; ++i)
            {
                var taskIndex = (int)((i + taskStartIndex) & mask);
                var sourceJob = tasks[i];
                ref var targetJob = ref this.tasks[taskIndex];
                targetJob.Function = sourceJob.Function;
                targetJob.Context = sourceJob.Context;
                targetJob.TaskId = sourceJob.TaskId;
                targetJob.JobIndex = jobIndex;
            }
            Interlocked.Exchange(ref writtenTaskIndex, taskEndIndex); //This is not assuming 64 bits. Mildly goofy considering the rest.
            return true;
        }
        finally
        {
            locker = 0;
        }
    }


    ///// <summary>
    ///// Tries to enqueues the stop command. 
    ///// </summary>
    ///// <returns>True if the stop could be pushed onto the queue, false otherwise.</returns>
    //public bool TryEnqueueStop()
    //{
    //    Span<TaskDescription> stopJob = stackalloc TaskDescription[1];
    //    stopJob[0] = new TaskDescription { Function = null, TaskId = -1 };
    //    return TryEnqueueJob(stopJob, out _);
    //}


    /// <summary>
    /// Submits a set of tasks representing a for loop over the given indices and returns when all loop iterations are complete.
    /// </summary>
    /// <param name="function">Function to execute on each iteration of the loop.</param>
    /// <param name="context">Context pointer to pass into each iteration of the loop.</param>
    /// <param name="inclusiveStartIndex">Inclusive start index of the loop range.</param>
    /// <param name="exclusiveEndIndex">Exclusive end index of the loop range.</param>
    /// <param name="workerIndex">Index of the currently executing worker.</param>
    /// <remarks>This function attempts to minimize completion latency. The calling thread will not be used to execute any task unrelated to the for loop job.
    /// This prevents other parallel jobs from making the total execution time of this for loop longer than if the calling thread simply executed the entire loop itself.</remarks>
    public void For(delegate*<int, void*, int, void> function, void* context, int inclusiveStartIndex, int exclusiveEndIndex, int workerIndex)
    {
        var taskCount = exclusiveEndIndex - inclusiveStartIndex;
        if (taskCount <= 0)
            return;
        if (taskCount > 1)
        {
            //Note that we only submit tasks to the queue for tasks beyond the first. The current thread is responsible for at least task 0.
            Span<TaskDescription> taskDescriptions = stackalloc TaskDescription[taskCount - 1];
            for (int i = 0; i < taskDescriptions.Length; ++i)
            {
                taskDescriptions[i] = new TaskDescription { Function = function, Context = context, TaskId = i + 1 + inclusiveStartIndex };
            }
            SpinWait waiter = new SpinWait();
            while (!TryEnqueueJob(taskDescriptions, out var jobHandle))
            {
                waiter.SpinOnce(-1); //TODO: We're biting the bullet on yields/sleep(0) here. May not be ideal for the use case; investigate.
            }
        }
        //Tasks [1, count) are submitted to the queue and may now be executing on other workers.
        //The thread calling the for loop should not relinquish its timeslice. It should immediately begin working on task 0.
        function(inclusiveStartIndex, context, workerIndex);

        if (taskCount > 1)
        {
            //Task 0 is done; this thread should seek out other work. Critically, this thread should pull *only* tasks which were related to the for loop job.
            //If we were to take other tasks, 
        }

    }

}
