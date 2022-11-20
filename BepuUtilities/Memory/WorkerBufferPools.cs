using System;

namespace BepuUtilities.Memory;

/// <summary>
/// Collection of arena pools used by worker threads.
/// </summary>
public class WorkerBufferPools : IDisposable
{
    ArenaPool[] pools;
    /// <summary>
    /// Central pool from which subpools allocate.
    /// </summary>
    public BufferPool Pool { get; private set; }
    /// <summary>
    /// Locker used by subpools to control access to the central pool.
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
    /// <param name="pool">Central pool from which worker pools allocate from.</param>
    /// <param name="initialWorkerCount">Initial number of workers to allocate space for.</param>
    /// <param name="defaultBlockCapacity">Default block capacity in thread pools.</param>
    public WorkerBufferPools(BufferPool pool, int initialWorkerCount, int defaultBlockCapacity = 16384)
    {
        Pool = pool;
        Locker = new object();
        pools = new ArenaPool[initialWorkerCount];
        DefaultBlockCapacity = defaultBlockCapacity;
        for (int i = 0; i < pools.Length; ++i)
        {
            pools[i] = new ArenaPool(pool, defaultBlockCapacity, Locker);
        }
    }

    /// <summary>
    /// Preallocates for a given number of workers.
    /// </summary>
    /// <param name="workerCount">Number of workers to preallocate for.</param>
    /// <param name="preallocationSize">Capacity to preallocate in the specified worker pools.</param>
    public void Preallocate(int workerCount, int preallocationSize)
    {
        if (workerCount > pools.Length)
        {
            var oldSize = pools.Length;
            Array.Resize(ref pools, workerCount);
            for (int i = oldSize; i < pools.Length; ++i)
            {
                pools[i] = new ArenaPool(Pool, DefaultBlockCapacity);
            }
        }
        for (int i = 0; i < workerCount; ++i)
        {
            pools[i].EnsurePreallocatedSpaceUnsafely(preallocationSize);
        }
    }

    /// <summary>
    /// Preallocates space in all worker pools.
    /// </summary>
    /// <param name="preallocationSize">Capacity to preallocate in all worker pools.</param>
    public void Preallocate(int preallocationSize)
    {
        Preallocate(pools.Length, preallocationSize);
    }

    /// <summary>
    /// Clears all allocations from worker arena pools. Pools can still be used after being cleared.
    /// </summary>
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
    public void Dispose()
    {
        for (int i = 0; i < pools.Length; ++i)
        {
            pools[i].Dispose();
        }
    }



}
