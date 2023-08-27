﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Unmanaged memory pool that suballocates from memory blocks pulled from the native heap.
    /// </summary>
    public class BufferPool : IUnmanagedMemoryPool
    {
        unsafe struct PowerPool
        {
            public byte*[] Blocks;
            /// <summary>
            /// Pool of slots available to this power level.
            /// </summary>
            public ManagedIdPool Slots;
#if DEBUG
            internal HashSet<int> outstandingIds;
#if LEAKDEBUG
            internal Dictionary<string, HashSet<int>> outstandingAllocators;
#endif
#endif

            public readonly int SuballocationsPerBlock;
            public readonly int SuballocationsPerBlockShift;
            public readonly int SuballocationsPerBlockMask;
            public readonly int Power;
            public readonly int SuballocationSize;
            public readonly int BlockSize;
            public int BlockCount;

            internal const int IdPowerShift = 26;
            /// <summary>
            /// Byte alignment to enforce for all block allocations within the buffer pool.
            /// </summary>
            /// <remarks>Since this only applies at the level of blocks, we can use a pretty beefy value without much concern.</remarks>
            public const int BlockAlignment = 128;

            public PowerPool(int power, int minimumBlockSize, int expectedPooledCount)
            {
                Power = power;
                SuballocationSize = 1 << power;
                BlockSize = Math.Max(SuballocationSize, minimumBlockSize);
                Slots = new ManagedIdPool(expectedPooledCount);
                SuballocationsPerBlock = BlockSize / SuballocationSize;
                SuballocationsPerBlockShift = SpanHelper.GetContainingPowerOf2(SuballocationsPerBlock);
                SuballocationsPerBlockMask = (1 << SuballocationsPerBlockShift) - 1;
                Blocks = new byte*[1];
                BlockCount = 0;

#if DEBUG
                outstandingIds = new HashSet<int>();
#if LEAKDEBUG
                outstandingAllocators = new Dictionary<string, HashSet<int>>();
#endif
#endif
            }

            void Resize(int newSize)
            {
                var newBlocks = new byte*[newSize];
                Array.Copy(Blocks, newBlocks, Blocks.Length);
                Blocks = newBlocks;
            }

            void AllocateBlock(int blockIndex)
            {
#if DEBUG
                for (int i = 0; i < blockIndex; ++i)
                {
                    Debug.Assert(Blocks[i] != null, "If we are allocating a block, all previous blocks should be allocated already.");
                }
#endif
                Debug.Assert(Blocks[blockIndex] == null);
                //Suballocations from the block will always occur on pow2 boundaries, so the only way for a suballocation to violate this alignment is if an individual 
                //suballocation is smaller than the alignment- in which case it doesn't require the alignment to be that wide. Also, since the alignment and 
                //suballocations are both pow2 sized, they won't drift out of sync.
                Blocks[blockIndex] = (byte*)NativeMemory.AlignedAlloc((nuint)BlockSize, BlockAlignment);
                BlockCount = blockIndex + 1;
            }

            public void EnsureCapacity(int capacity)
            {
                var neededBlockCount = (int)Math.Ceiling((double)capacity / BlockSize);
                if (BlockCount < neededBlockCount)
                {
                    if (neededBlockCount > Blocks.Length)
                    {
                        Resize(neededBlockCount);
                    }
                    for (int i = BlockCount; i < neededBlockCount; ++i)
                    {
                        AllocateBlock(i);
                    }
                    BlockCount = neededBlockCount;
                }

            }

            public unsafe readonly byte* GetStartPointerForSlot(int slot)
            {
                var blockIndex = slot >> SuballocationsPerBlockShift;
                var indexInBlock = slot & SuballocationsPerBlockMask;
                return Blocks[blockIndex] + indexInBlock * SuballocationSize;
            }

            public unsafe void Take(out Buffer<byte> buffer)
            {
                var slot = Slots.Take();
                var blockIndex = slot >> SuballocationsPerBlockShift;
                if (blockIndex >= Blocks.Length)
                {
                    Resize((int)BitOperations.RoundUpToPowerOf2((uint)(blockIndex + 1)));
                }
                if (blockIndex >= BlockCount)
                {
                    AllocateBlock(blockIndex);
                }

                var indexInBlock = slot & SuballocationsPerBlockMask;
                buffer = new Buffer<byte>(Blocks[blockIndex] + indexInBlock * SuballocationSize, SuballocationSize, (Power << IdPowerShift) | slot);
                Debug.Assert(buffer.Id >= 0 && Power >= 0 && Power < 32, "Slot/power should be safely encoded in a 32 bit integer.");
#if DEBUG
                const int maximumOutstandingCount = 1 << 26;
                Debug.Assert(outstandingIds.Count < maximumOutstandingCount,
                    $"Do you actually truly really need to have {maximumOutstandingCount} allocations taken from this power pool, or is this a memory leak?");
                Debug.Assert(outstandingIds.Add(slot), "Should not be able to request the same slot twice.");
#if LEAKDEBUG
                var allocator = new StackTrace().ToString();
                if (!outstandingAllocators.TryGetValue(allocator, out var idsForAllocator))
                {
                    idsForAllocator = new HashSet<int>();
                    outstandingAllocators.Add(allocator, idsForAllocator);
                }
                const int maximumReasonableOutstandingAllocationsForAllocator = 1 << 25;
                Debug.Assert(idsForAllocator.Count < maximumReasonableOutstandingAllocationsForAllocator, "Do you actually have that many allocations for this one allocator?");
                idsForAllocator.Add(slot);
#endif
#endif
            }

            [Conditional("DEBUG")]
            internal unsafe void ValidateBufferIsContained<T>(ref Buffer<T> typedBuffer) where T : unmanaged
            {
                var buffer = typedBuffer.As<byte>();
                //There are a lot of ways to screw this up. Try to catch as many as possible!
                var slotIndex = buffer.Id & ((1 << IdPowerShift) - 1);
                var blockIndex = slotIndex >> SuballocationsPerBlockShift;
                var indexInAllocatorBlock = slotIndex & SuballocationsPerBlockMask;
                Debug.Assert(buffer.Length <= SuballocationSize,
                  "A buffer taken from a pool should have a specific size.");
                Debug.Assert(blockIndex >= 0 && blockIndex < BlockCount,
                    "The block pointed to by a returned buffer should actually exist within the pool.");
                var memoryOffset = buffer.Memory - Blocks[blockIndex];
                Debug.Assert(memoryOffset >= 0 && memoryOffset < BlockSize,
                    "If a raw buffer points to a given block as its source, the address should be within the block's memory region.");
                Debug.Assert(Blocks[blockIndex] + indexInAllocatorBlock * SuballocationSize == buffer.Memory,
                    "The implied address of a buffer in its block should match its actual address.");
                Debug.Assert(buffer.Length + indexInAllocatorBlock * SuballocationSize <= BlockSize,
                    "The extent of the buffer should fit within the block.");
            }

            public readonly unsafe void Return(int slotIndex)
            {
#if DEBUG 
                Debug.Assert(outstandingIds.Remove(slotIndex),
                    "This buffer id must have been taken from the pool previously.");
#if LEAKDEBUG
                bool found = false;
                foreach (var pair in outstandingAllocators)
                {
                    if (pair.Value.Remove(slotIndex))
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
                Slots.Return(slotIndex);
            }

            public void Clear()
            {
#if DEBUG
                //We'll assume that the caller understands that the outstanding buffers are invalidated, so should not be returned again.
                outstandingIds.Clear();
#if LEAKDEBUG
                outstandingAllocators.Clear();
#endif
#endif
                for (int i = 0; i < BlockCount; ++i)
                {
                    NativeMemory.AlignedFree(Blocks[i]);
                    Blocks[i] = null;
                }
                Slots.Clear();
                BlockCount = 0;
            }

        }

        private PowerPool[] pools = new PowerPool[SpanHelper.MaximumSpanSizePower + 1];
        private int minimumBlockSize;

        /// <summary>
        /// Creates a new buffer pool.
        /// </summary>
        /// <param name="minimumBlockAllocationSize">Minimum size of individual block allocations. Must be a power of 2.
        /// Pools with single allocations larger than the minimum will use the minimum value necessary to hold one element.
        /// Buffers will be suballocated from blocks.</param>
        /// <param name="expectedPooledResourceCount">Number of suballocations to preallocate reference space for.
        /// This does not preallocate actual blocks, just the space to hold references that are waiting in the pool.</param>
        public BufferPool(int minimumBlockAllocationSize = 131072, int expectedPooledResourceCount = 16)
        {
            if (((minimumBlockAllocationSize - 1) & minimumBlockAllocationSize) != 0)
                throw new ArgumentException("Block allocation size must be a power of 2.");
            minimumBlockSize = minimumBlockAllocationSize;
            for (int power = 0; power <= SpanHelper.MaximumSpanSizePower; ++power)
            {
                pools[power] = new PowerPool(power, minimumBlockSize, expectedPooledResourceCount);
            }
        }

        /// <summary>
        /// Ensures that the pool associated with a given power has at least a certain amount of capacity, measured in bytes.
        /// </summary>
        /// <param name="byteCount">Minimum number of bytes to require for the power pool.</param>
        /// <param name="power">Power associated with the pool to check.</param>
        public void EnsureCapacityForPower(int byteCount, int power)
        {
            SpanHelper.ValidatePower(power);
            pools[power].EnsureCapacity(byteCount);
        }

        /// <summary>
        /// Gets the capacity allocated for a power.
        /// </summary>
        /// <param name="power">Power to check.</param>
        /// <returns>Allocated capacity for the given power.</returns>
        public int GetCapacityForPower(int power)
        {
            SpanHelper.ValidatePower(power);
            return pools[power].BlockCount * pools[power].BlockSize;
        }

        /// <summary>
        /// Computes the total number of bytes allocated from native memory in this buffer pool.
        /// Includes allocated memory regardless of whether it currently has outstanding references.
        /// </summary>
        /// <returns>Total number of bytes allocated from native memory in this buffer pool.</returns>
        public ulong GetTotalAllocatedByteCount()
        {
            ulong sum = 0;
            for (int i = 0; i < pools.Length; ++i)
            {
                sum += (ulong)pools[i].BlockCount * (ulong)pools[i].BlockSize;
            }
            return sum;
        }

        /// <summary>
        /// Takes a buffer large enough to contain a number of elements of a given type. Capacity may be larger than requested.
        /// </summary>
        /// <typeparam name="T">Type of the elements in the buffer.</typeparam>
        /// <param name="count">Desired minimum capacity of the buffer in typed elements.</param>
        /// <param name="buffer">Buffer large enough to contain the requested number of elements.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeAtLeast<T>(int count, out Buffer<T> buffer) where T : unmanaged
        {
            //Avoid returning a zero length span because 1 byte / Unsafe.SizeOf<T>() happens to be zero.
            if (count == 0)
                count = 1;
            TakeForPower(SpanHelper.GetContainingPowerOf2(count * Unsafe.SizeOf<T>()), out var rawBuffer);
            buffer = rawBuffer.As<T>();
        }

        /// <summary>
        /// Takes a typed buffer of the requested size from the pool.
        /// </summary>
        /// <typeparam name="T">Type of the instances in the buffer.</typeparam>
        /// <param name="count">Desired capacity of the buffer in typed elements.</param>
        /// <param name="buffer">Typed buffer of the requested size.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take<T>(int count, out Buffer<T> buffer) where T : unmanaged
        {
            TakeAtLeast(count, out buffer);
            buffer.length = count;
        }

        /// <summary>
        /// Takes a buffer large enough to contain a number of bytes given by a power, where the number of bytes is 2^power.
        /// </summary>
        /// <param name="power">Number of bytes that should fit within the buffer as an exponent, where the number of bytes is 2^power.</param>
        /// <param name="buffer">Buffer that can hold the bytes.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out Buffer<byte> buffer)
        {
            Debug.Assert(power >= 0 && power <= SpanHelper.MaximumSpanSizePower);
            pools[power].Take(out buffer);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void DecomposeId(int bufferId, out int powerIndex, out int slotIndex)
        {
            powerIndex = bufferId >> PowerPool.IdPowerShift;
            slotIndex = bufferId & ((1 << PowerPool.IdPowerShift) - 1);
        }

        /// <inheritdoc/>       
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ReturnUnsafely(int id)
        {
            DecomposeId(id, out var powerIndex, out var slotIndex);
            pools[powerIndex].Return(slotIndex);
        }

        /// <inheritdoc/>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Return<T>(ref Buffer<T> buffer) where T : unmanaged
        {
#if DEBUG
            DecomposeId(buffer.Id, out var powerIndex, out var slotIndex);
            pools[powerIndex].ValidateBufferIsContained(ref buffer);
#endif
            ReturnUnsafely(buffer.Id);
            buffer = default;
        }

        /// <inheritdoc/>  
        public void ResizeToAtLeast<T>(ref Buffer<T> buffer, int targetSize, int copyCount) where T : unmanaged
        {
            //Only do anything if the new size is actually different from the current size.
            Debug.Assert(copyCount <= targetSize && copyCount <= buffer.Length, "Can't copy more elements than exist in the source or target buffers.");
            targetSize = GetCapacityForCount<T>(targetSize);
            if (!buffer.Allocated)
            {
                Debug.Assert(buffer.Length == 0, "If a buffer is pointing at null, then it should be default initialized and have a length of zero too.");
                //This buffer is not allocated; just return a new one. No copying to be done.
                TakeAtLeast(targetSize, out buffer);
            }
            else
            {
                var originalAllocatedSizeInBytes = 1 << (buffer.Id >> PowerPool.IdPowerShift);
                var originalAllocatedSize = originalAllocatedSizeInBytes / Unsafe.SizeOf<T>();
                Debug.Assert(originalAllocatedSize >= buffer.Length, "The original allocated capacity must be sufficient for the buffer's observed length. Did the buffer get corrupted? Is this buffer reference from uninitialized memory?");
                if (targetSize > originalAllocatedSize)
                {
                    //The original allocation isn't big enough to hold the new size; allocate a new buffer.
                    TakeAtLeast(targetSize, out Buffer<T> newBuffer);
                    buffer.CopyTo(0, newBuffer, 0, copyCount);
                    ReturnUnsafely(buffer.Id);
                    buffer = newBuffer;
                }
                else
                {
                    //Original allocation is large enough to hold the new size; just bump the size up.
                    //The expectation for this function is to bump up to the next power of 2, given the 'AtLeast' suffix, so just expose the full original size.
                    //No need for copying.
                    buffer.length = originalAllocatedSize;
                }
            }
        }

        /// <inheritdoc/>  
        public void Resize<T>(ref Buffer<T> buffer, int targetSize, int copyCount) where T : unmanaged
        {
            ResizeToAtLeast(ref buffer, targetSize, copyCount);
            buffer.length = targetSize;
        }

        /// <summary>
        /// Issues debug assertions that all pools are empty.
        /// </summary>
        [Conditional("DEBUG")]
        public void AssertEmpty()
        {
#if DEBUG
            for (int i = 0; i < pools.Length; ++i)
            {
                var pool = pools[i];
                if (pool.outstandingIds.Count > 0)
                {
                    Debug.WriteLine($"Power pool {i} contains allocations.");
#if LEAKDEBUG
                    foreach (var allocator in pool.outstandingAllocators)
                    {
                        Debug.WriteLine($"{allocator.Key}   ALLOCATION COUNT: {allocator.Value.Count}");
                    }
#endif
                    Debug.Assert(pool.outstandingIds.Count == 0);
                }
            }
#endif
        }

        /// <summary>
        /// Returns all allocations in the pool to sources. Any outstanding buffers will be invalidated silently.
        /// The pool will remain in a usable state after clearing.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i < pools.Length; ++i)
            {
                pools[i].Clear();
            }
        }

        /// <summary>
        /// Returns all allocations in the pool to sources. Any outstanding buffers will be invalidated silently.
        /// Equivalent to <see cref="Clear"/> for <see cref="BufferPool"/>.
        /// </summary>
        void IDisposable.Dispose()
        {
            Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetCapacityForCount<T>(int count)
        {
            if (count == 0)
                count = 1;
            Debug.Assert(BitOperations.RoundUpToPowerOf2((ulong)(count * Unsafe.SizeOf<T>())) < int.MaxValue, "This function assumes that counts aren't going to overflow a signed 32 bit integer.");
            return ((int)BitOperations.RoundUpToPowerOf2((uint)(count * Unsafe.SizeOf<T>()))) / Unsafe.SizeOf<T>();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int IUnmanagedMemoryPool.GetCapacityForCount<T>(int count)
        {
            return GetCapacityForCount<T>(count);
        }

#if DEBUG
        ~BufferPool()
        {
            var totalBlockCount = 0;
            for (int i = 0; i < pools.Length; ++i)
            {
                totalBlockCount += pools[i].BlockCount;
            }
            //If block count is zero, pinned just returns true since that's the default. If there's a nonzero number of blocks, then they have to be explicitly unpinned
            //in order for a finalizer to be valid.
            Debug.Assert(totalBlockCount == 0, "Memory leak warning! Don't let a buffer pool die without clearing it!");
        }
#endif

    }
}