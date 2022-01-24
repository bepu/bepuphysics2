using System;
using System.Diagnostics;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities
{
    /// <summary>
    /// Provides a <see cref="IThreadDispatcher"/> implementation. Not reentrant.
    /// </summary>
    public class ThreadDispatcher5 : IThreadDispatcher, IDisposable
    {
        int threadCount;
        /// <summary>
        /// Gets the number of threads to dispatch work on.
        /// </summary>
        public int ThreadCount => threadCount;
        struct Worker
        {
            public Thread Thread;
            public AutoResetEvent Signal;
        }

        Worker[] workers;
        AutoResetEvent finished;

        BufferPool[] bufferPools;

        /// <summary>
        /// Creates a new thread dispatcher with the given number of threads.
        /// </summary>
        /// <param name="threadCount">Number of threads to dispatch on each invocation.</param>
        public ThreadDispatcher5(int threadCount)
        {
            this.threadCount = threadCount;
            workers = new Worker[threadCount - 1];
            for (int i = 0; i < workers.Length; ++i)
            {
                workers[i] = new Worker { Thread = new Thread(WorkerLoop), Signal = new AutoResetEvent(false) };
                workers[i].Thread.IsBackground = true;
                workers[i].Thread.Start((workers[i].Signal, i + 1));
            }
            finished = new AutoResetEvent(false);
            bufferPools = new BufferPool[threadCount];
            for (int i = 0; i < bufferPools.Length; ++i)
            {
                bufferPools[i] = new BufferPool();
            }
        }

        void DispatchThread(Action<int> localAction, int jobIndex, int workerIndex)
        {
            Debug.Assert(workerBody != null);
            localAction(jobIndex);

            if (Interlocked.Decrement(ref remainingWorkerCounter) == 0)
            {
                finished.Set();
            }
        }

        volatile Action<int> workerBody;
        int jobIndex;
        int jobCount;
        int remainingWorkerCounter;
        object locker = new object();
        void WorkerLoop(object untypedSignal)
        {
            (AutoResetEvent signal, int workerIndex) = ((AutoResetEvent, int))untypedSignal;
            while (true)
            {
                if (disposed)
                    return;
                signal.WaitOne();
                while (true)
                {
                    Action<int> localAction;
                    int jobIndex;
                    lock (locker)
                    {
                        localAction = workerBody;
                        jobIndex = this.jobIndex++;
                        if (localAction == null || jobIndex >= jobCount)
                            break;
                    }
                    DispatchThread(localAction, jobIndex, workerIndex);
                }
            }
        }

        void SignalThreads(int maximumWorkerCount)
        {
            //Worker 0 is not signalled; it's the executing thread.
            //So if we want 4 total executing threads, we should signal 3 workers.
            int maximumWorkersToSignal = maximumWorkerCount - 1;
            var workersToSignal = maximumWorkersToSignal < workers.Length ? maximumWorkersToSignal : workers.Length;
            for (int i = 0; i < workersToSignal; ++i)
            {
                workers[i].Signal.Set();
            }
        }

        public void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                Debug.Assert(this.workerBody == null);
                lock (locker)
                {
                    jobIndex = 1; //Just make the inline thread worker 0. While the other threads might start executing first, the user should never rely on the dispatch order.
                    jobCount = Math.Min(maximumWorkerCount, threadCount);
                    remainingWorkerCounter = jobCount;
                    this.workerBody = workerBody;
                }
                SignalThreads(maximumWorkerCount);
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(workerBody, 0, 0);
                finished.WaitOne();
                this.workerBody = null;
            }
            else if (maximumWorkerCount == 1)
            {
                workerBody(0);
            }
        }

        volatile bool disposed;

        /// <summary>
        /// Waits for all pending work to complete and then disposes all workers.
        /// </summary>
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                SignalThreads(threadCount);
                for (int i = 0; i < bufferPools.Length; ++i)
                {
                    bufferPools[i].Clear();
                }
                foreach (var worker in workers)
                {
                    worker.Thread.Join();
                    worker.Signal.Dispose();
                }
            }
        }

        public BufferPool GetThreadMemoryPool(int workerIndex)
        {
            return bufferPools[workerIndex];
        }
    }

}
