using BepuUtilities.Memory;
using System;

namespace BepuUtilities
{
    /// <summary>
    /// Provides multithreading dispatch primitives, a thread count, and per thread resource pools for the simulation to use.
    /// </summary>
    /// <remarks>
    /// <para>Note that the simulation does not require a true load balancing forloop implementation. All that's needed is a way to jumpstart some threads.
    /// All systems which use multithreading tend to have some form of domain specific load balancing that a general purpose thread pool or parallel for loop implementation 
    /// couldn't match. The simulation also tends to keep the number of dispatches as low as it can. Combined, these two things reduce the importance of a
    /// very highly optimized dispatcher.</para>
    /// <para>This is important when a user wants to share some other thread pool, but doesn't have the time to guarantee extremely high performance and high quality
    /// load balancing. Instead of worrying about that, they can just wrap whatever implementation they happen to have and it'll probably work fine.</para>
    /// </remarks>
    public interface IThreadDispatcher
    {
        /// <summary>
        /// Gets the number of workers available in the thread dispatcher.
        /// </summary>
        /// <remarks>Note that some systems (like the solver) expect the ThreadCount to be backed by truly independent threads capable of progression even when one is blocked.
        /// If the ThreadCount doesn't represent independent threads, deadlocks will occur.
        /// </remarks>
        int ThreadCount { get; }

        /// <summary>
        /// Dispatches all the available workers.
        /// </summary>
        /// <param name="workerBody">Delegate to be invoked on every worker.</param>
        /// <param name="maximumWorkerCount">Maximum number of workers to dispatch.</param>
        void DispatchWorkers(Action<int> workerBody, int maximumWorkerCount = int.MaxValue);

        /// <summary>
        /// Gets the memory pool associated with a given worker index. It is guaranteed that no other workers will share the same pool for the duration of the worker's execution.
        /// </summary>
        /// <remarks>All usages of the memory pool within the simulation are guaranteed to return thread pool memory before the function returns. In other words,
        /// thread memory pools are used for strictly ephemeral memory, and it will never be held by the simulation outside the scope of a function that 
        /// takes the IThreadDispatcher as input.</remarks>
        /// <param name="workerIndex">Index of the worker to grab the pool for.</param>
        /// <returns>The memory pool for the specified worker index.</returns>
        BufferPool GetThreadMemoryPool(int workerIndex);
    }
}
