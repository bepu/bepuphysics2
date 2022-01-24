using System;
using System.Diagnostics;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities
{

    /// <summary>
    /// Provides a <see cref="IThreadDispatcher"/> implementation. Not reentrant.
    /// </summary>
    public class ThreadDispatcher2 : IThreadDispatcher, IDisposable
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
        AutoResetEvent finished;
        BufferPool[] bufferPools;

        /// <summary>
        /// Creates a new thread dispatcher with the given number of threads.
        /// </summary>
        /// <param name="threadCount">Number of threads to dispatch on each invocation.</param>
        /// <param name="workerPoolBlockAllocationSize">Size of allocation blocks in the worker thread buffer pools.</param>
        public ThreadDispatcher2(int threadCount, int workerPoolBlockAllocationSize = 16384)
        {
            this.threadCount = threadCount;
            workers = new Worker[threadCount - 1];
            for (int i = 0; i < workers.Length; ++i)
            {
                workers[i] = new Worker { Thread = new Thread(WorkerLoop) };
                workers[i].Thread.IsBackground = true;
                //The main thread will be used as worker 0.
                workers[i].Thread.Start(i + 1);
            }
            finished = new AutoResetEvent(false);
            bufferPools = new BufferPool[threadCount];
            for (int i = 0; i < bufferPools.Length; ++i)
            {
                bufferPools[i] = new BufferPool(workerPoolBlockAllocationSize);
            }
        }

        volatile Action<int> workerBody;
        int remainingUnclaimedJobCount;
        int remainingUncompletedJobCount;

        object waiter = new object();

        void DispatchThread(int workerIndex)
        {
            int jobIndex;
            lock (waiter)
            {
                //Getting here either means 1) this is the startup and there is no job to do, or 2) the local thread just executed a workerBody and we should increment completed work.
                if (workerBody != null)
                {
                    //Just finished work; mark it as completed.
                    if (--remainingUncompletedJobCount == 0)
                    {
                        workerBody = null;
                        finished.Set();
                        return;
                    }
                }
                if (remainingUnclaimedJobCount == 0 || disposed)
                {
                    return;
                }
                //Grab the next job.
                jobIndex = --remainingUnclaimedJobCount;
            }
            workerBody(jobIndex);
        }

        void WorkerLoop(object untypedWorkerIndex)
        {
            var workerIndex = (int)untypedWorkerIndex;
            while (true)
            {
                int jobIndex;
                lock (waiter)
                {
                    //Getting here either means 1) this is the startup and there is no job to do, or 2) the local thread just executed a workerBody and we should increment completed work.
                    if (workerBody != null)
                    {
                        //Just finished work; mark it as completed.
                        if (--remainingUncompletedJobCount == 0)
                        {
                            workerBody = null;
                            finished.Set();
                        }
                    }
                    while (workerBody == null || remainingUnclaimedJobCount == 0)
                    {
                        if (disposed)
                            return;
                        Monitor.Wait(waiter);
                    }
                    //Grab the next job.
                    jobIndex = --remainingUnclaimedJobCount;
                }
                workerBody(jobIndex);
            }
        }

        public void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                lock (waiter)
                {
                    remainingUnclaimedJobCount = maximumWorkerCount < threadCount ? maximumWorkerCount : threadCount;
                    remainingUncompletedJobCount = remainingUnclaimedJobCount;
                    Debug.Assert(this.workerBody == null);
                    this.workerBody = workerBody;
                    Monitor.PulseAll(waiter);
                }
                //Calling thread does work. No reason to spin up another worker and block this one!
                //DispatchThread(0);
                finished.WaitOne();
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
                lock (waiter)
                {
                    Debug.Assert(this.workerBody == null);
                    Monitor.PulseAll(waiter);
                }
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
