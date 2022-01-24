using System;
using System.Diagnostics;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities
{
    /// <summary>
    /// Provides a <see cref="IThreadDispatcher"/> implementation. Not reentrant.
    /// </summary>
    public class ThreadDispatcher : IThreadDispatcher, IDisposable
    {
        int threadCount;
        /// <summary>
        /// Gets the number of threads to dispatch work on.
        /// </summary>
        public int ThreadCount => threadCount;
        struct Worker
        {
            public Thread Thread;
        }

        Worker[] workers;
        ManualResetEventSlim resetEvent;
        AutoResetEvent finished;

        BufferPool[] bufferPools;

        /// <summary>
        /// Creates a new thread dispatcher with the given number of threads.
        /// </summary>
        /// <param name="threadCount">Number of threads to dispatch on each invocation.</param>
        public ThreadDispatcher(int threadCount)
        {
            this.threadCount = threadCount;
            resetEvent = new ManualResetEventSlim(false);
            workers = new Worker[threadCount - 1];
            for (int i = 0; i < workers.Length; ++i)
            {
                workers[i] = new Worker { Thread = new Thread(WorkerLoop) };
                workers[i].Thread.IsBackground = true;
                workers[i].Thread.Start(i + 1);
            }
            finished = new AutoResetEvent(false);
            bufferPools = new BufferPool[threadCount];
            for (int i = 0; i < bufferPools.Length; ++i)
            {
                bufferPools[i] = new BufferPool();
            }
        }

        void DispatchThread(int workerIndex)
        {
            while (true)
            {
                var localJobIndex = Interlocked.Increment(ref jobIndex);
                if (localJobIndex < jobCount)
                {
                    if (localJobIndex == jobCount - 1)
                    {
                        //No further jobs are available, so workers should start waiting again.
                        resetEvent.Reset();
                    }
                    Debug.Assert(workerBody != null);
                    workerBody(localJobIndex);
                    if (Interlocked.Decrement(ref remainingJobCounter) == 0)
                    {
                        finished.Set();
                    }
                }
                else
                {
                    return;
                }
            }
        }

        volatile Action<int> workerBody;
        int jobIndex;
        int remainingJobCounter;
        int jobCount;

        void WorkerLoop(object untypedSignal)
        {
            var workerIndex = (int)untypedSignal;
            while (true)
            {
                resetEvent.Wait();
                if (disposed)
                    return;
                DispatchThread(workerIndex);
            }
        }

        void SignalThreads(int jobCount)
        {
            this.jobCount = jobCount;
            jobIndex = -1;
            remainingJobCounter = jobCount;
            resetEvent.Set();
        }

        public void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                Debug.Assert(this.workerBody == null);
                this.workerBody = workerBody;
                SignalThreads(Math.Min(threadCount, maximumWorkerCount));
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(0);
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
                }
            }
        }

        public BufferPool GetThreadMemoryPool(int workerIndex)
        {
            return bufferPools[workerIndex];
        }
    }

}
