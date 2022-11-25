using System;

namespace BepuUtilities.Memory;

/// <summary>
/// Collection of arena pools used by worker threads.
/// </summary> 
/// <remarks>Returns to an <see cref="ArenaPool"/> are not guaranteed to free memory because it does not carry enough information about allocations to do so.
/// To free up memory after use, the arena pool as a whole must be cleared using <see cref="ArenaPool.Clear"/>.<para/>
/// Use <see cref="Clear"/> to clear all pools together.</remarks>
public class WorkerBufferPools : IDisposable
{
    ArenaPool[] pools;
    /// <summary>
    /// Internal pool from which arena pools are allocated from.
    /// </summary>
    /// <remarks>This pool will be accessed from multiple threads through the arena pools. Accessing it while workers are using the worker pools without using the <see cref="Locker"/> could lead to race conditions.<para/>
    /// If no external pool was provided to the <see cref="WorkerBufferPools"/> constructor, <see cref="Dispose"/> will dispose the internally created <see cref="BufferPool"/>. If an external pool was provided, calling <see cref="Dispose"/> will not dispose it.</remarks>
    public IUnmanagedMemoryPool Pool { get; private set; }
    /// <summary>
    /// True if the backing pool was created by the <see cref="WorkerBufferPools"/> and should be disposed with it, false otherwise.
    /// </summary>
    bool poolIsLocallyOwned;
    /// <summary>
    /// Locker used by subpools to control access to the backing pool.
    /// </summary>
    public object Locker { get; private set; }

    /// <summary>
    /// Gets the pool associated with this worker.
    /// </summary>
    /// <param name="workerIndex">Worker index of the pool to look up.</param>
    /// <returns>Pool associated with the given worker.</returns>
    public ArenaPool this[int workerIndex] => pools[workerIndex];

    /// <summary>
    /// Gets or sets the default block capacity for any newly created arena subpools.
    /// </summary>
    public int DefaultBlockCapacity { get; set; }


    /// <summary>
    /// Creates a new set of worker pools.
    /// </summary>
    /// <param name="backingPool">Central pool from which worker pools allocate from. If null, a <see cref="BufferPool"/> will be created. The created pool will be disposed by <see cref="Dispose"/>, but if an external pool is provided, calling <see cref="Dispose"/> will not dispose it.</param>
    /// <param name="initialWorkerCount">Initial number of workers to allocate space for.</param>
    /// <param name="defaultBlockCapacity">Default block capacity in thread pools.</param>
    public WorkerBufferPools(int initialWorkerCount, IUnmanagedMemoryPool backingPool = null, int defaultBlockCapacity = 16384)
    {
        poolIsLocallyOwned = backingPool == null;
        Pool = poolIsLocallyOwned ? new BufferPool() : backingPool;
        Locker = new object();
        pools = new ArenaPool[initialWorkerCount];
        DefaultBlockCapacity = defaultBlockCapacity;
        for (int i = 0; i < pools.Length; ++i)
        {
            pools[i] = new ArenaPool(Pool, defaultBlockCapacity, Locker);
        }
    }

    /// <summary>
    /// Ensures that at least the given amount of memory is available in the given number of worker pools.
    /// </summary>
    /// <param name="workerCount">Number of workers to preallocate for.</param>
    /// <param name="preallocationSize">Capacity to preallocate in the specified worker pools.</param>
    /// <remarks>This does not take any locks and should not be called if any other threads may be using any of the involved pools.</remarks>
    public void EnsureCapacity(int workerCount, int preallocationSize)
    {
        if (workerCount > pools.Length)
        {
            var oldSize = pools.Length;
            Array.Resize(ref pools, workerCount);
            for (int i = oldSize; i < pools.Length; ++i)
            {
                pools[i] = new ArenaPool(Pool, DefaultBlockCapacity, Locker);
            }
        }
        for (int i = 0; i < workerCount; ++i)
        {
            pools[i].EnsureCapacityUnsafely(preallocationSize);
        }
    }

    /// <summary>
    /// Ensures that at least the given amount of memory is available in all worker pools.
    /// </summary>
    /// <param name="preallocationSize">Capacity to preallocate in all worker pools.</param>
    /// <remarks>This does not take any locks and should not be called if any other threads may be using any of the involved pools.</remarks>
    public void EnsureCapacity(int preallocationSize)
    {
        EnsureCapacity(pools.Length, preallocationSize);
    }

    /// <summary>
    /// Clears all allocations from worker arena pools. Pools can still be used after being cleared.
    /// </summary>
    /// <remarks>This does not take any locks and should not be called if any other threads may be using any of the involved pools.</remarks>
    public void Clear()
    {
        for (int i = 0; i < pools.Length; ++i)
        {
            pools[i].Clear();
        }
    }

    /// <summary>
    /// Disposes all worker arena pools. Pools cannot be used after being disposed.
    /// </summary>
    /// <remarks>This will dispose the <see cref="Pool"/> only if no external pool was provided to the constructor.</remarks>
    public void Dispose()
    {
        for (int i = 0; i < pools.Length; ++i)
        {
            pools[i].Dispose();
        }
        if (poolIsLocallyOwned)
            Pool.Dispose();
    }



}
