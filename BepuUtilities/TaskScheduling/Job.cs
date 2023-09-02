using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities.TaskScheduling;

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
