using System;
using System.Diagnostics;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities
{
    /// <summary>
    /// Provides a <see cref="IThreadDispatcher"/> implementation. Not reentrant.
    /// </summary>
    public class ThreadDispatcher6 : IThreadDispatcher, IDisposable
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
        SemaphoreSlim semaphore;
        AutoResetEvent finished;

        BufferPool[] bufferPools;

        /// <summary>
        /// Creates a new thread dispatcher with the given number of threads.
        /// </summary>
        /// <param name="threadCount">Number of threads to dispatch on each invocation.</param>
        public ThreadDispatcher6(int threadCount)
        {
            this.threadCount = threadCount;
            semaphore = new SemaphoreSlim(0, threadCount);
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

        void DispatchThread(int jobIndex, int workerIndex)
        {
            Debug.Assert(workerBody != null);
            workerBody(jobIndex);

            if (Interlocked.Decrement(ref remainingWorkerCounter) == -1)
            {
                finished.Set();
            }
        }

        volatile Action<int> workerBody;
        int jobIndex;
        int remainingWorkerCounter;

        void WorkerLoop(object untypedSignal)
        {
            var workerIndex = (int)untypedSignal;
            while (true)
            {
                semaphore.Wait();
                if (disposed)
                    return;
                var localJobIndex = Interlocked.Increment(ref jobIndex);
                DispatchThread(localJobIndex, workerIndex);
            }
        }

        void SignalThreads(int maximumWorkerCount)
        {
            //Worker 0 is not signalled; it's the executing thread.
            //So if we want 4 total executing threads, we should signal 3 workers.
            int maximumWorkersToSignal = maximumWorkerCount - 1;
            var workersToSignal = maximumWorkersToSignal < workers.Length ? maximumWorkersToSignal : workers.Length;
            remainingWorkerCounter = workersToSignal;
            jobIndex = 0;
            semaphore.Release(workersToSignal);
        }

        public void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                Debug.Assert(this.workerBody == null);
                this.workerBody = workerBody;
                SignalThreads(maximumWorkerCount);
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(0, 0);
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
