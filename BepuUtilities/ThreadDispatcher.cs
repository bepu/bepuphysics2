﻿using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
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
        /// <inheritdoc/>
        public void* UnmanagedContext => unmanagedContext;
        /// <inheritdoc/>
        public object ManagedContext => managedContext;

        /// <summary>
        /// Creates a new thread dispatcher with the given number of threads.
        /// </summary>
        /// <param name="threadCount">Number of threads to dispatch on each invocation.</param>
        /// <param name="threadPoolBlockAllocationSize">Size of memory blocks to allocate for thread pools.</param>
        public ThreadDispatcher(int threadCount, int threadPoolBlockAllocationSize = 16384)
        {
            if (threadCount <= 0)
                throw new ArgumentOutOfRangeException(nameof(threadCount), "Thread count must be positive.");
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
                case WorkerType.Unmanaged: unmanagedWorker(workerIndex, this); break;
            }

            if (Interlocked.Decrement(ref remainingWorkerCounter.Value) == -1)
            {
                finished.Set();
            }
        }

        enum WorkerType
        {
            //We've gone back and forth many times on how many types exist. 2 at the moment, but will it be 4 again next week? Who knows!
            Managed,
            Unmanaged,
        }

        volatile WorkerType workerType;
        volatile Action<int> managedWorker;
        volatile delegate*<int, IThreadDispatcher, void> unmanagedWorker;
        volatile void* unmanagedContext;
        volatile object managedContext;

        //We'd like to avoid the thread readonly values above being adjacent to the thread readwrite counter.
        //If they were in the same cache line, it would cause a bit of extra contention for no reason.
        //(It's not *that* big of a deal since the counter is only touched once per worker, but padding this also costs nothing.)
        //In a class, we don't control layout, so wrap the counter in a beefy struct.
        //128B padding is used for the sake of architectures that might try prefetching cache line pairs and running into sync problems.
        [StructLayout(LayoutKind.Explicit, Size = 256)]
        struct Counter
        {
            [FieldOffset(128)]
            public int Value;
        }

        Counter remainingWorkerCounter;

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
            remainingWorkerCounter.Value = workersToSignal;
            for (int i = 0; i < workersToSignal; ++i)
            {
                workers[i].Signal.Set();
            }
        }

        /// <inheritdoc/>
        public void DispatchWorkers(delegate*<int, IThreadDispatcher, void> workerBody, int maximumWorkerCount = int.MaxValue, void* unmanagedContext = null, object managedContext = null)
        {
            Debug.Assert(this.managedWorker == null && this.unmanagedWorker == null && this.managedContext == null && this.unmanagedContext == null);
            this.unmanagedContext = unmanagedContext;
            this.managedContext = managedContext;
            if (maximumWorkerCount > 1)
            {
                workerType = WorkerType.Unmanaged;
                this.unmanagedWorker = workerBody;
                SignalThreads(maximumWorkerCount);
                //Calling thread does work. No reason to spin up another worker and block this one!
                DispatchThread(0);
                finished.WaitOne();
                this.unmanagedWorker = null;
            }
            else if (maximumWorkerCount == 1)
            {
                workerBody(0, this);
            }
            this.unmanagedContext = null;
            this.managedContext = null;
        }

        //While we *could* pass in the IThreadDispatcher for the managed side of things, it is typically best to just expect closures. Simplifies some stuff.
        //(The fact that we supply context at all is a bit of a shrug.)
        /// <inheritdoc/>
        public void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue, void* unmanagedContext = null, object managedContext = null)
        {
            Debug.Assert(this.managedWorker == null && this.unmanagedWorker == null && this.managedContext == null && this.unmanagedContext == null);
            this.unmanagedContext = unmanagedContext;
            this.managedContext = managedContext;
            if (maximumWorkerCount > 1)
            {
                workerType = WorkerType.Managed;
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
            this.unmanagedContext = null;
            this.managedContext = null;
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
