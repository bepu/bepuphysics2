using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;
using BepuUtilities.Memory;

namespace BepuUtilities
{
    /// <summary>
    /// Provides a <see cref="IThreadDispatcher"/> implementation. Not reentrant.
    /// </summary>
    public unsafe class ThreadDispatcher : IThreadDispatcher, IDisposable
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

        /// <inheritdoc/>
        public WorkerBufferPools WorkerPools { get; private set; }

        /// <summary>
        /// Creates a new thread dispatcher with the given number of threads.
        /// </summary>
        /// <param name="threadCount">Number of threads to dispatch on each invocation.</param>
        /// <param name="threadPoolBlockAllocationSize">Size of memory blocks to allocate for thread pools.</param>
        public ThreadDispatcher(int threadCount, int threadPoolBlockAllocationSize = 16384)
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
            WorkerPools = new WorkerBufferPools(threadCount, threadPoolBlockAllocationSize);
        }

        void DispatchThread(int workerIndex)
        {
            switch (workerType)
            {
                case WorkerType.Managed: managedWorker(workerIndex); break;
                case WorkerType.ManagedWithUnmanagedContext: managedWithContextWorker(workerIndex, context); break;
                case WorkerType.Unmanaged: unmanagedWorker(workerIndex, context); break;
            }

            if (Interlocked.Decrement(ref remainingWorkerCounter) == -1)
            {
                finished.Set();
            }
        }

        enum WorkerType
        {
            /// <summary>
            /// Action{int}
            /// </summary>
            Managed,
            /// <summary>
            /// <see cref="ThreadDispatcherWorker"/>
            /// </summary>
            ManagedWithUnmanagedContext,
            /// <summary>
            /// Includes unmanaged context by default because these jobs can only be static functions and *some* context is probably required.
            /// delegate*{int, void*, void}
            /// </summary>
            Unmanaged,
        }

        volatile WorkerType workerType;
        volatile Action<int> managedWorker;
        volatile ThreadDispatcherWorker managedWithContextWorker;
        volatile delegate*<int, void*, void> unmanagedWorker;
        volatile void* context;

        int remainingWorkerCounter;

        void WorkerLoop(object untypedSignal)
        {
            var (signal, workerIndex) = ((AutoResetEvent, int))untypedSignal;
            while (true)
            {
                signal.WaitOne();
                if (disposed)
                    return;
                DispatchThread(workerIndex);
            }
        }

        void SignalThreads(int maximumWorkerCount)
        {
            //Worker 0 is not signalled; it's the executing thread.
            //So if we want 4 total executing threads, we should signal 3 workers.
            int maximumWorkersToSignal = maximumWorkerCount - 1;
            var workersToSignal = maximumWorkersToSignal < workers.Length ? maximumWorkersToSignal : workers.Length;
            remainingWorkerCounter = workersToSignal;
            for (int i = 0; i < workersToSignal; ++i)
            {
                workers[i].Signal.Set();
            }
        }

        /// <inheritdoc/>
        public void DispatchWorkers(delegate*<int, void*, void> workerBody, void* context, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                Debug.Assert(this.managedWorker == null && this.managedWithContextWorker == null && this.unmanagedWorker == null && this.context == null);
                workerType = WorkerType.Unmanaged;
                unmanagedWorker = workerBody;
                this.context = context;
                SignalThreads(maximumWorkerCount);
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(0);
                finished.WaitOne();
                unmanagedWorker = null;
                this.context = null;
            }
            else if (maximumWorkerCount == 1)
            {
                workerBody(0, context);
            }
        }

        /// <inheritdoc/>
        public void DispatchWorkers(ThreadDispatcherWorker workerBody, void* context, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                Debug.Assert(this.managedWorker == null && this.managedWithContextWorker == null && this.unmanagedWorker == null && this.context == null);
                workerType = WorkerType.ManagedWithUnmanagedContext;
                this.managedWithContextWorker = workerBody;
                this.context = context;
                SignalThreads(maximumWorkerCount);
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(0);
                finished.WaitOne();
                this.managedWithContextWorker = null;
                this.context = null;
            }
            else if (maximumWorkerCount == 1)
            {
                workerBody(0, context);
            }
        }

        /// <inheritdoc/>
        public void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue)
        {
            if (maximumWorkerCount > 1)
            {
                Debug.Assert(this.managedWorker == null && this.managedWithContextWorker == null && this.unmanagedWorker == null && this.context == null);
                workerType = WorkerType.ManagedWithUnmanagedContext;
                this.managedWorker = workerBody;
                SignalThreads(maximumWorkerCount);
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(0);
                finished.WaitOne();
                this.managedWorker = null;
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
                foreach (var worker in workers)
                {
                    worker.Thread.Join();
                    worker.Signal.Dispose();
                }
                WorkerPools.Dispose();
            }
        }

    }

}
