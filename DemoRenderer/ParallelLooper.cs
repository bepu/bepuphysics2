using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace DemoRenderer
{
    /// <summary>
    /// Function called by the <see cref="ParallelLooper"/> for each job.
    /// </summary>
    /// <param name="jobIndex">Index of the job to execute.</param>
    /// <param name="workerIndex">Index of the worker executing the job.</param>
    public delegate void LooperAction(int jobIndex, int workerIndex);

    /// <summary>
    /// Function called by the <see cref="ParallelLooper"/> when a worker determines that there is no more work available.
    /// </summary>
    /// <param name="workerIndex">Index of the worker that finished all available work.</param>
    public delegate void LooperWorkerDone(int workerIndex);

    /// <summary>
    /// Simple multithreaded for loop provider built on an IThreadDispatcher. Performs an atomic operation for every object in the loop, so pre-chunking the works into jobs is important.
    /// </summary>
    /// <remarks>This helps avoid some unnecessary allocations associated with the TPL implementation. While a little garbage from the renderer in the demos isn't exactly a catastrophe,
    /// having zero allocations under normal execution makes it easier to notice when the physics simulation itself is allocating inappropriately.</remarks>
    public class ParallelLooper
    {
        Action<int> dispatcherWorker;
        public IThreadDispatcher Dispatcher { get; set; }

        public ParallelLooper()
        {
            dispatcherWorker = Worker;
        }

        void Worker(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref start);
                if (index >= end)
                    break;
                iteration?.Invoke(index, workerIndex);
            }
            workerDone?.Invoke(workerIndex);

        }

        int start, end;
        LooperAction iteration;
        LooperWorkerDone workerDone;

        public void For(int start, int exclusiveEnd, LooperAction workAction, LooperWorkerDone workerDone = null)
        {
            if (Dispatcher == null)
            {
                for (int i = start; i < exclusiveEnd; ++i)
                {
                    workAction?.Invoke(i, 0);
                }
                workerDone?.Invoke(0);
            }
            else
            {
                this.start = start - 1;
                this.end = exclusiveEnd;
                this.iteration = workAction;
                this.workerDone = workerDone;
                Dispatcher.DispatchWorkers(dispatcherWorker);
                this.iteration = null;
                this.workerDone = workerDone;
            }
        }
    }
}
