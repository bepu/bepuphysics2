using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;

namespace DemoRenderer
{
    /// <summary>
    /// Simple multithreaded for loop provider built on an IThreadDispatcher. Performs an atomic operation for every object in the loop, so pre-chunking the works into jobs is important.
    /// </summary>
    /// <remarks>This helps avoid some unnecessary allocations associated with the TPL implementation. While a little garbage from the renderer in the demos isn't exactly a catastrophe,
    /// having zero allocations under normal execution makes it easier to notice when the physics simulation itself is allocating inappropriately.</remarks>
    public class ParallelLooper
    {
        Action<int> workerDelegate;
        public IThreadDispatcher Dispatcher { get; set; }
        
        public ParallelLooper()
        {
            workerDelegate = Worker;
        }

        void Worker(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref start);
                if (index >= end)
                    break;
                work(index);
            }
        }

        int start, end;
        Action<int> work;

        public void For(int start, int exclusiveEnd, Action<int> work)
        {
            if(Dispatcher == null)
            {
                for (int i = start; i < exclusiveEnd; ++i)
                {
                    work(i);
                }
            }
            else
            {
                this.start = start - 1;
                this.end = exclusiveEnd;
                this.work = work;
                Dispatcher.DispatchWorkers(workerDelegate);
                this.work = null;
            }
        }
    }
}
