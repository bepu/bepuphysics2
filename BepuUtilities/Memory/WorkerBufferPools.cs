using System;

namespace BepuUtilities.Memory;

/// <summary>
/// Collection of pools used by worker threads.
/// </summary> 
public class WorkerBufferPools : IDisposable
{
    BufferPool[] pools;

    /// <summary>

    /// <summary>
    /// Gets the pool associated with this worker.
    /// </summary>
    /// <param name="workerIndex">Worker index of the pool to look up.</param>
    /// <returns>Pool associated with the given worker.</returns>
    public BufferPool this[int workerIndex] => pools[workerIndex];

    /// <summary>
    /// Gets or sets the default block capacity for any newly created arena subpools.
    /// </summary>
    public int DefaultBlockCapacity { get; set; }


    /// <summary>
    /// Creates a new set of worker pools.
    /// </summary>
   /// <param name="initialWorkerCount">Initial number of workers to allocate space for.</param>
    /// <param name="defaultBlockCapacity">Default block capacity in thread pools.</param>
    public WorkerBufferPools(int initialWorkerCount, int defaultBlockCapacity = 16384)
    {
        pools = new BufferPool[initialWorkerCount];
        DefaultBlockCapacity = defaultBlockCapacity;
        for (int i = 0; i < pools.Length; ++i)
        {
            pools[i] = new BufferPool(defaultBlockCapacity);
        }
    }

    /// <summary>
    /// Clears all allocations from worker pools. Pools can still be used after being cleared.
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
    /// Disposes all worker pools. Pools cannot be used after being disposed.
    /// </summary>
    public void Dispose()
    {
        for (int i = 0; i < pools.Length; ++i)
        {
            pools[i].Clear();
        }
    }

    /// <summary>
    /// Gets the total number of bytes allocated from native memory by all worker pools. Includes memory that is not currently in use by external allocators.
    /// </summary>
    /// <returns>Total number of bytes allocated from native memory by all worker pools. Includes memory that is not currently in use by external allocators.</returns>
    public ulong GetTotalAllocatedByteCount()
    {
        ulong sum = 0;
        for (int i = 0; i < pools.Length; ++i)
        {
            sum += pools[i].GetTotalAllocatedByteCount();
        }
        return sum;
    }
}
