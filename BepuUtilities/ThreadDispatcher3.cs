using System;
using System.Diagnostics;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities
{

    /// <summary>
    /// Provides a <see cref="IThreadDispatcher"/> implementation. Not reentrant.
    /// </summary>
    public class ThreadDispatcher3 : IThreadDispatcher, IDisposable
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
        public ThreadDispatcher3(int threadCount, int workerPoolBlockAllocationSize = 16384)
        {
            this.threadCount = threadCount;
            semaphore = new SemaphoreSlim(0);
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

        SemaphoreSlim semaphore;

        void DispatchThread(int workerIndex)
        {
            int jobIndex;
            int completedJobCount = 0;
            while ((jobIndex = Interlocked.Decrement(ref remainingUnclaimedJobCount)) >= 0)
            {
                workerBody(jobIndex);
                ++completedJobCount;
            }
            //No more jobs are available.
            if (Interlocked.Add(ref remainingUncompletedJobCount, -completedJobCount) == 0)
            {
                //All jobs are done, no worker is still executing anything.
                workerBody = null;
                finished.Set();
            }
        }
        void WorkerLoop(object untypedWorkerIndex)
        {
            var workerIndex = (int)untypedWorkerIndex;
            while (true)
            {
                semaphore.Wait();
                if (disposed)
                    return;
                DispatchThread(workerIndex);
            }
        }

        void SignalThreads(int jobCount, Action<int> workerBody)
        {
            remainingUnclaimedJobCount = jobCount < threadCount ? jobCount : threadCount;
            remainingUncompletedJobCount = remainingUnclaimedJobCount;
            Debug.Assert(this.workerBody == null);
            this.workerBody = workerBody;
            semaphore.Release(remainingUnclaimedJobCount);
        }

        public void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                SignalThreads(maximumWorkerCount, workerBody);
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(0);
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
                Debug.Assert(workerBody == null);
                SignalThreads(threadCount - 1, null);
                finished.WaitOne();
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
