using BepuUtilities.Collections;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Memory;

/// <summary>
/// Arena allocator built to serve a single thread. Pulls resources from a central buffer pool when necessary.
/// </summary>
public class ArenaPool : IUnmanagedMemoryPool, IDisposable
{
    /// <summary>
    /// Gets or sets the central pool backing this arena allocator. Resources are pulled from this pool when the thread pool's blocks are depleted.
    /// </summary>
    public IUnmanagedMemoryPool Pool { get; set; }
    /// <summary>
    /// Gets the locker object used to protect accesses to the cetnral buffer pool.
    /// </summary>
    public object Locker { get; private set; }
    /// <summary>
    /// Gets or sets the default capacity within blocks allocated by the pool.
    /// </summary>
    public int DefaultBlockCapacity { get; set; }

    struct Block
    {
        public int Count;
        public Buffer<byte> Data;

        public Block(Buffer<byte> data)
        {
            Data = data;
            Count = 0;
        }

        public bool TryAllocate(int sizeInBytes, out Buffer<byte> allocation)
        {
            //Following the pattern set by the bufferpool, use beefy alignment:
            var startLocation = (Count + 127) & (~127);
            var newCount = startLocation + sizeInBytes;
            if (Data.Length >= newCount)
            {
                allocation = Data.Slice(startLocation, sizeInBytes);
                Count = newCount;
                return true;
            }
            allocation = default;
            return false;
        }
    }
#if DEBUG
    internal HashSet<int> outstandingIds;
#if LEAKDEBUG
    internal Dictionary<string, HashSet<int>> outstandingAllocators;
#endif
#endif

    QuickList<Block> blocks;

    /// <summary>
    /// Creates a new arena thread pool.
    /// </summary>
    /// <param name="pool">Central pool to allocate blocks from </param>
    /// <param name="locker">Locker object used to protect accesses to the central pool. If no locker is specified, the pool reference is used.</param>
    public ArenaPool(IUnmanagedMemoryPool pool, int defaultBlockCapacity = 16384, object locker = null)
    {
        Pool = pool;
        Locker = locker == null ? pool : locker;
        DefaultBlockCapacity = defaultBlockCapacity;
        blocks = new QuickList<Block>(8, pool);
        //We check for allocated blocks before allocating one, so we need to clear the memory up front.
        blocks.Span.Clear(0, blocks.Span.Length);
#if DEBUG
        outstandingIds = new HashSet<int>();
#if LEAKDEBUG
        outstandingAllocators = new Dictionary<string, HashSet<int>>();
#endif
#endif
    }

    /// <summary>
    /// Ensures that there is a given minimum amount of continguous space available in the pool. This does not acquire a lock to read from the internal pool.
    /// </summary>
    /// <param name="sizeInBytes">Size of the block to use. If negative, <see cref="DefaultBlockCapacity"/> will be used instead.</param>
    public void EnsurePreallocatedSpaceUnsafely(int sizeInBytes = -1)
    {
        if (sizeInBytes < 0)
            sizeInBytes = DefaultBlockCapacity;
        if (blocks.Count > 0)
        {
            ref var block = ref blocks[blocks.Count - 1];
            if (block.Data.Length - block.Count >= sizeInBytes)
            {
                //There's enough room already; no need to allocate.
                return;
            }
        }
        //We need a new block.
        Pool.Take<byte>(sizeInBytes, out var data);
        blocks.Allocate(Pool) = new Block(data);

    }

    /// <summary>
    /// Clears all block allocations from the pool. The pool can still be used.
    /// </summary>
    public void Clear()
    {
#if DEBUG
        outstandingIds.Clear();
#if LEAKDEBUG
        outstandingAllocators.Clear();
#endif
#endif
        for (int i = 0; i < blocks.Count; ++i)
        {
            Pool.Return(ref blocks[i].Data);
        }
        blocks.Count = 0;
    }

    /// <summary>
    /// Clears all allocations from the pool, including any used by the pool itself. The pool can no longer be used.
    /// </summary>
    public void Dispose()
    {
        if (blocks.Span.Allocated)
        {
            Clear();
            blocks.Dispose(Pool);
        }
    }

    const int maximumBitsForBlock = 1;
    const int maximumBitsForIndices = 31 - maximumBitsForBlock;
    const int maximumBitsForIndex = maximumBitsForIndices / 2;
    const int countInBlockMask = (1 << maximumBitsForIndex) - 1;
    const int previousCountInBlockMask = countInBlockMask << maximumBitsForIndex;
    const int blockMask = ~(countInBlockMask | previousCountInBlockMask);
    const int lowerBlockMask = (1 << maximumBitsForBlock) - 1;

    /// <inheritdoc/>
    public unsafe void TakeAtLeast<T>(int count, out Buffer<T> buffer) where T : unmanaged
    {
        var sizeInBytes = Unsafe.SizeOf<T>() * count;
        var blockIndex = blocks.Count - 1;
        if (blocks.Count == 0 || !blocks[blockIndex].TryAllocate(sizeInBytes, out Buffer<byte> allocation))
        {
            //No room; need a new block.
            var newBlockCapacityInBytes = Math.Max(DefaultBlockCapacity, sizeInBytes);
            //Check to see if there's already a block allocated that we can use.
            if (blocks.Span.Length > blocks.Count && blocks.Span[blocks.Count].Data.Length >= sizeInBytes)
            {
                ++blocks.Count;
                Debug.Assert(blocks[blocks.Count - 1].Count == 0 && blocks[blocks.Count - 1].Data.Memory != null);
            }
            else
            {
                //Need a new block.
                Buffer<byte> blockData;
                lock (Locker)
                {
                    Pool.Take(newBlockCapacityInBytes, out blockData);
                    if (blocks.Span.Length == blocks.Count)
                        blocks.EnsureCapacity(blocks.Count * 2, Pool);
                }
                blocks.AllocateUnsafely() = new Block(blockData);
            }
            blockIndex = blocks.Count - 1;
            var succeeded = blocks[blockIndex].TryAllocate(sizeInBytes, out allocation);
            Debug.Assert(succeeded, "We just allocated that block, it should hold everything requested!");
        }
        buffer = allocation.As<T>();
        var newCount = blocks[blockIndex].Count;
        var previousCount = newCount - sizeInBytes;
        var bitpackedBlockIndex = lowerBlockMask & blockIndex;
        var bitpackedStartIndex = countInBlockMask & newCount;
        var bitpackedPreviousIndex = countInBlockMask & previousCount;
        buffer.Id = (bitpackedBlockIndex << maximumBitsForIndices) | (bitpackedPreviousIndex << maximumBitsForIndex) | bitpackedStartIndex;
        if (blockIndex >= (1 << maximumBitsForBlock) || previousCount >= (1 << maximumBitsForIndex) || newCount >= (1 << maximumBitsForIndex))
        {
            //Bit index 31 being set is code for 'unrepresentable'.
            buffer.Id |= 1 << 31;
        }
#if DEBUG
        if (buffer.Id >= 0) //Don't include unrepresentable ids in the tracked set.
        {
            const int maximumOutstandingCount = 1 << 26;
            Debug.Assert(outstandingIds.Count < maximumOutstandingCount,
                $"Do you actually truly really need to have {maximumOutstandingCount} allocations taken from this pool, or is this a memory leak?");
            Debug.Assert(outstandingIds.Add(buffer.Id), "Should not be able to request the same slot twice.");
#if LEAKDEBUG
            var allocator = new StackTrace().ToString();
            if (!outstandingAllocators.TryGetValue(allocator, out var idsForAllocator))
            {
                idsForAllocator = new HashSet<int>();
                outstandingAllocators.Add(allocator, idsForAllocator);
            }
            const int maximumReasonableOutstandingAllocationsForAllocator = 1 << 25;
            Debug.Assert(idsForAllocator.Count < maximumReasonableOutstandingAllocationsForAllocator, "Do you actually have that many allocations for this one allocator?");
            idsForAllocator.Add(buffer.Id);
#endif
        }
#endif
    }
    /// <inheritdoc/>
    public void Take<T>(int count, out Buffer<T> buffer) where T : unmanaged
    {
        TakeAtLeast(count, out buffer);
    }

    /// <inheritdoc/>
    /// <remarks>Unlike a <see cref="BufferPool"/>, the <see cref="ArenaPool"/> will not generally free up space in response to calls to <see cref="Return{T}(ref Buffer{T})"/>.
    /// If the deallocated buffer is the last allocated buffer for a given block, the pool may choose to bump the allocation pointer back, but it is not guaranteed.</remarks>
    public void Return<T>(ref Buffer<T> buffer) where T : unmanaged
    {
        if (buffer.Id >= 0)
        {
            //This was a representable id.
#if DEBUG
            Debug.Assert(outstandingIds.Remove(buffer.Id),
                "This buffer id must have been taken from the pool previously.");
#if LEAKDEBUG
            bool found = false;
            foreach (var pair in outstandingAllocators)
            {
                if (pair.Value.Remove(buffer.Id))
                {
                    found = true;
                    if (pair.Value.Count == 0)
                    {
                        outstandingAllocators.Remove(pair.Key);
                        break;
                    }
                }
            }
            Debug.Assert(found, "Allocator set must contain the buffer id.");
#endif
#endif
            //Was it the most recently allocated buffer (in that block) such that we can pop it off like a stack?
            var blockIndex = (buffer.Id & blockMask) >> maximumBitsForIndices;
            var countInBlock = buffer.Id & countInBlockMask;
            var previousCountInBlock = (buffer.Id & countInBlockMask) >> maximumBitsForIndex;
            ref var block = ref blocks[blockIndex];
            if (block.Count == countInBlock)
            {
                //Deallocating is as simple as just resetting to the previous value.
                block.Count = previousCountInBlock;
                while (blocks.Count > 0 && blocks[blocks.Count - 1].Count == 0)
                {
                    //Push the block count as far as it can go.
                    --blocks.Count;
                }
            }
        }
        buffer = default;
    }

    /// <inheritdoc/>
    public int GetCapacityForCount<T>(int count) where T : unmanaged
    {
        return count;
    }

}
