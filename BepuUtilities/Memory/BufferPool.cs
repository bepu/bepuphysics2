﻿using BepuUtilities.Collections;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Unmanaged memory pool that creates pinned blocks of memory for use in spans.
    /// </summary>
    /// <remarks>This currently works by allocating large managed arrays and pinning them under the assumption that they'll end up in the large object heap.</remarks>
    public class BufferPool : IUnmanagedMemoryPool, IDisposable
    {
        unsafe struct Block
        {
            public byte[] Array;
            public GCHandle Handle;
            public byte* Pointer;

            public Block(int blockSize)
            {
                //While the runtime does have some alignment guarantees, we hedge against the possibility that the runtime could change (or another runtime is in use),
                //or that the runtime isn't aligning to a size sufficiently large for wide SIMD types. I suspect that the combination of the jit's tendency to use unaligned 
                //instructions regardless and modern processors' performance on unaligned instructions will make this basically irrelevant, but it costs roughly nothing.
                //Suballocations from the block will always occur on pow2 boundaries, so the only way for a suballocation to violate this alignment is if an individual 
                //suballocation is smaller than the alignment- in which case it doesn't require the alignment to be that wide. Also, since the alignment and 
                //suballocations are both pow2 sized, they won't drift out of sync.
                int alignment = Vector<float>.Count * sizeof(float);
                Array = new byte[blockSize + alignment];
                Handle = GCHandle.Alloc(Array, GCHandleType.Pinned);
                Pointer = (byte*)Handle.AddrOfPinnedObject();
                var mask = alignment - 1;
                var offset = (uint)Pointer & mask;
                Pointer += alignment - offset;
            }


            public byte* Allocate(int indexInBlock, int suballocationSize)
            {
                Debug.Assert(Allocated);
                Debug.Assert(Pinned);
                Debug.Assert(indexInBlock >= 0 && indexInBlock * suballocationSize < Array.Length);
                return Pointer + indexInBlock * suballocationSize;
            }

            public bool Allocated
            {
                get
                {
                    return Array != null;
                }
            }

            public bool Pinned
            {
                get
                {
                    return Array != null && Handle.IsAllocated;
                }
                set
                {

                    Debug.Assert(Array != null);
                    if (value)
                    {
                        Debug.Assert(!Handle.IsAllocated);
                        Handle = GCHandle.Alloc(Array);
                        Pointer = (byte*)Handle.AddrOfPinnedObject();
                    }
                    else
                    {
                        Debug.Assert(Handle.IsAllocated);
                        Handle.Free();
                        Pointer = null;
                    }
                }
            }

            /// <summary>
            /// Unpins and drops the reference to the underlying array.
            /// </summary>
            public void Clear()
            {
                Debug.Assert(Array != null);
                //It's not guaranteed that the array is actually pinned if we support unpinning.
                if (Handle.IsAllocated)
                {
                    Pinned = false;
                }
                Array = null;
            }

        }

        struct PowerPool
        {
            public Block[] Blocks;
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

            public PowerPool(int power, int minimumBlockSize, int expectedPooledCount)
            {
                Power = power;
                SuballocationSize = 1 << power;
                BlockSize = Math.Max(SuballocationSize, minimumBlockSize);
                Slots = new ManagedIdPool(expectedPooledCount);
                SuballocationsPerBlock = BlockSize / SuballocationSize;
                SuballocationsPerBlockShift = SpanHelper.GetContainingPowerOf2(SuballocationsPerBlock);
                SuballocationsPerBlockMask = (1 << SuballocationsPerBlockShift) - 1;
                Blocks = new Block[1];
                BlockCount = 0;

#if DEBUG
                outstandingIds = new HashSet<int>();
#if LEAKDEBUG
                outstandingAllocators = new Dictionary<string, HashSet<int>>();
#endif
#endif
            }

            public void EnsureCapacity(int capacity)
            {
                var neededBlockCount = (int)Math.Ceiling((double)capacity / BlockSize);
                if (BlockCount < neededBlockCount)
                {
                    if (neededBlockCount > Blocks.Length)
                        Array.Resize(ref Blocks, neededBlockCount);
                    for (int i = BlockCount; i < neededBlockCount; ++i)
                    {
                        Blocks[i] = new Block(BlockSize);
                    }
                    BlockCount = neededBlockCount;
                }

            }

            public unsafe byte* GetStartPointerForSlot(int slot)
            {
                var blockIndex = slot >> SuballocationsPerBlockShift;
                var indexInBlock = slot & SuballocationsPerBlockMask;
                return Blocks[blockIndex].Pointer + indexInBlock * SuballocationSize;
            }

            public unsafe void Take(out RawBuffer buffer)
            {
                var slot = Slots.Take();
                var blockIndex = slot >> SuballocationsPerBlockShift;
                if (blockIndex >= Blocks.Length)
                {
                    Array.Resize(ref Blocks, 1 << SpanHelper.GetContainingPowerOf2(blockIndex + 1));
                }
                if (blockIndex >= BlockCount)
                {
#if DEBUG
                    for (int i = 0; i < blockIndex; ++i)
                    {
                        Debug.Assert(Blocks[i].Allocated, "If a block index is found to exceed the current block count, then every block preceding the index should be allocated.");
                    }
#endif
                    BlockCount = blockIndex + 1;
                    Debug.Assert(!Blocks[blockIndex].Allocated);
                    Blocks[blockIndex] = new Block(BlockSize);
                }

                var indexInBlock = slot & SuballocationsPerBlockMask;
                buffer = new RawBuffer(Blocks[blockIndex].Allocate(indexInBlock, SuballocationSize), SuballocationSize, (Power << IdPowerShift) | slot);
                Debug.Assert(buffer.Id >= 0 && Power >= 0 && Power < 32, "Slot/power should be safely encoded in a 32 bit integer.");
#if DEBUG
                const int maximumOutstandingCount = 1 << 20;
                Debug.Assert(outstandingIds.Count <= maximumOutstandingCount,
                    $"Do you actually truly really need to have {maximumOutstandingCount} allocations taken from this power pool, or is this a memory leak?");
                Debug.Assert(outstandingIds.Add(slot), "Should not be able to request the same slot twice.");
#if LEAKDEBUG
                var allocator = new StackTrace().ToString();
                if (!outstandingAllocators.TryGetValue(allocator, out var idsForAllocator))
                {
                    idsForAllocator = new HashSet<int>();
                    outstandingAllocators.Add(allocator, idsForAllocator);
                }
                Debug.Assert(idsForAllocator.Count < (1 << 25), "Do you actually have that many allocations for this one allocator?");
                idsForAllocator.Add(slot);
#endif
#endif
            }

            [Conditional("DEBUG")]
            internal unsafe void ValidateBufferIsContained(ref RawBuffer buffer)
            {
                //There are a lot of ways to screw this up. Try to catch as many as possible!
                var slotIndex = buffer.Id & ((1 << IdPowerShift) - 1);
                var blockIndex = slotIndex >> SuballocationsPerBlockShift;
                var indexInAllocatorBlock = slotIndex & SuballocationsPerBlockMask;
                Debug.Assert(buffer.Length <= SuballocationSize,
                  "A buffer taken from a pool should have a specific size.");
                Debug.Assert(blockIndex >= 0 && blockIndex < BlockCount,
                    "The block pointed to by a returned buffer should actually exist within the pool.");
                var memoryOffset = buffer.Memory - Blocks[blockIndex].Pointer;
                Debug.Assert(memoryOffset >= 0 && memoryOffset < Blocks[blockIndex].Array.Length,
                    "If a raw buffer points to a given block as its source, the address should be within the block's memory region.");
                Debug.Assert(Blocks[blockIndex].Pointer + indexInAllocatorBlock * SuballocationSize == buffer.Memory,
                    "The implied address of a buffer in its block should match its actual address.");
                Debug.Assert(buffer.Length + indexInAllocatorBlock * SuballocationSize <= Blocks[blockIndex].Array.Length,
                    "The extent of the buffer should fit within the block.");
            }

            public unsafe void Return(int slotIndex)
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
                    Blocks[i].Clear();
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
        /// Buffers will be suballocated from blocks.
        /// Use a value larger than the large object heap cutoff (85000 bytes as of this writing in the microsoft runtime)
        /// to avoid interfering with generational garbage collection.</param>
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
            ValidatePinnedState(true);
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
        /// Takes a buffer large enough to contain a number of bytes. Capacity may be larger than requested.
        /// </summary>
        /// <param name="count">Desired minimum capacity of the buffer in bytes.</param>
        /// <param name="buffer">Buffer that can hold the bytes.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeAtLeast(int count, out RawBuffer buffer)
        {
            TakeForPower(SpanHelper.GetContainingPowerOf2(count), out buffer);
        }

        /// <summary>
        /// Takes a buffer of the requested size from the pool.
        /// </summary>
        /// <param name="count">Desired capacity of the buffer in bytes.</param>
        /// <param name="buffer">Buffer of the requested size.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out RawBuffer buffer)
        {
            TakeAtLeast(count, out buffer);
            buffer.Length = count;
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
            TakeAtLeast(count * Unsafe.SizeOf<T>(), out var rawBuffer);
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
        /// <param name="count">Number of bytes that should fit within the buffer as an exponent, where the number of bytes is 2^power.</param>
        /// <param name="buffer">Buffer that can hold the bytes.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out RawBuffer buffer)
        {
            ValidatePinnedState(true);
            Debug.Assert(power >= 0 && power <= SpanHelper.MaximumSpanSizePower);
            pools[power].Take(out buffer);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void DecomposeId(int bufferId, out int powerIndex, out int slotIndex)
        {
            powerIndex = bufferId >> PowerPool.IdPowerShift;
            slotIndex = bufferId & ((1 << PowerPool.IdPowerShift) - 1);
        }

        /// <summary>
        /// Returns a buffer to the pool by id.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        /// <remarks>Typed buffer pools zero out the passed-in buffer by convention.
        /// This costs very little and avoids a wide variety of bugs (either directly or by forcing fast failure). For consistency, BufferPool.Return does the same thing.
        /// This "Unsafe" overload should be used only in cases where there's a reason to bypass the clear; the naming is intended to dissuade casual use.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ReturnUnsafely(int id)
        {
            ValidatePinnedState(true);
            DecomposeId(id, out var powerIndex, out var slotIndex);
            pools[powerIndex].Return(slotIndex);
        }

        /// <summary>
        /// Returns a buffer to the pool.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Return(ref RawBuffer buffer)
        {
#if DEBUG
            DecomposeId(buffer.Id, out var powerIndex, out var slotIndex);
            pools[powerIndex].ValidateBufferIsContained(ref buffer);
#endif
            ReturnUnsafely(buffer.Id);
            buffer = default;
        }
        /// <summary>
        /// Returns a buffer to the pool.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Return<T>(ref Buffer<T> buffer) where T : unmanaged
        {
            ReturnUnsafely(buffer.Id);
            buffer = default;
        }

        //The resizes aren't particularly clever. They are more aggressive about reallocating than they need to be, but
        //the assumption is that they will be used only when it's already known that a resize is necessary.
        /// <summary>
        /// Resizes a buffer to the smallest size available in the pool which contains the target size. Copies a subset of elements into the new buffer.
        /// Final buffer size is at least as large as the target size and may be larger.
        /// </summary>
        /// <param name="buffer">Buffer reference to resize.</param>
        /// <param name="targetSize">Number of bytes to resize the buffer for.</param>
        /// <param name="copyCount">Number of bytes to copy into the new buffer from the old buffer.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ResizeToAtLeast(ref RawBuffer buffer, int targetSize, int copyCount)
        {
            Debug.Assert(copyCount <= buffer.Length, "Can't copy more than the capacity of the buffer.");
            Debug.Assert(copyCount <= targetSize, "Can't copy more than the target size.");
            //Only do anything if the new size is actually different from the current size.
            targetSize = 1 << (SpanHelper.GetContainingPowerOf2(targetSize));
            if (buffer.Allocated)
            {
                DecomposeId(buffer.Id, out var powerIndex, out var slotIndex);
                var currentSize = 1 << powerIndex;
                if (currentSize != targetSize || pools[powerIndex].GetStartPointerForSlot(slotIndex) != buffer.Memory)
                {
                    TakeAtLeast(targetSize, out var newBuffer);
                    Buffer.MemoryCopy(buffer.Memory, newBuffer.Memory, buffer.Length, copyCount);
                    pools[powerIndex].Return(slotIndex);
                    buffer = newBuffer;
                }
                else
                {
                    //While the allocation size is equal to the target size, the buffer might not be.
                    //Fortunately, if the allocation stays the same size and the buffer start is at its original location, we can skip doing any work.
                    //(With more work, you could expand *backwards*, we just didn't bother since this is exceptionally rare anyway.
                    //The typed codepath doesn't bother doing this at all, and that's fine.)
                    buffer.Length = targetSize;
                }
            }
            else
            {
                //Nothing to return or copy.
                TakeAtLeast(targetSize, out buffer);
            }
        }

        /// <summary>
        /// Resizes a buffer to the target size. Copies a subset of elements into the new buffer.
        /// </summary>
        /// <param name="buffer">Buffer reference to resize.</param>
        /// <param name="targetSize">Number of bytes to resize the buffer for.</param>
        /// <param name="copyCount">Number of bytes to copy into the new buffer from the old buffer.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Resize(ref RawBuffer buffer, int targetSize, int copyCount)
        {
            ResizeToAtLeast(ref buffer, targetSize, copyCount);
            buffer.Length = targetSize;
        }

        /// <summary>
        /// Resizes a typed buffer to the smallest size available in the pool which contains the target size. Copies a subset of elements into the new buffer. 
        /// Final buffer size is at least as large as the target size and may be larger.
        /// </summary>
        /// <typeparam name="T">Type of the buffer to resize.</typeparam>
        /// <param name="buffer">Buffer reference to resize.</param>
        /// <param name="targetSize">Number of elements to resize the buffer for.</param>
        /// <param name="copyCount">Number of elements to copy into the new buffer from the old buffer.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ResizeToAtLeast<T>(ref Buffer<T> buffer, int targetSize, int copyCount) where T : unmanaged
        {
            //Only do anything if the new size is actually different from the current size.
            Debug.Assert(copyCount <= targetSize && copyCount <= buffer.Length, "Can't copy more elements than exist in the source or target buffers.");
            targetSize = GetCapacityForCount<T>(targetSize);
            if (buffer.Length != targetSize) //Note that we don't check for allocated status- for buffers, a length of 0 is the same as being unallocated.
            {
                TakeAtLeast(targetSize, out Buffer<T> newBuffer);
                if (buffer.Length > 0)
                {
                    //Don't bother copying from or re-pooling empty buffers. They're uninitialized.
                    buffer.CopyTo(0, ref newBuffer, 0, copyCount);
                    ReturnUnsafely(buffer.Id);
                }
                else
                {
                    Debug.Assert(copyCount == 0, "Should not be trying to copy elements from an empty span.");
                }
                buffer = newBuffer;
            }
        }


        /// <summary>
        /// Resizes a buffer to the target size. Copies a subset of elements into the new buffer.
        /// </summary>
        /// <typeparam name="T">Type of the buffer to resize.</typeparam>
        /// <param name="buffer">Buffer reference to resize.</param>
        /// <param name="targetSize">Number of elements to resize the buffer for.</param>
        /// <param name="copyCount">Number of elements to copy into the new buffer from the old buffer.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize<T>(ref Buffer<T> buffer, int targetSize, int copyCount) where T : unmanaged
        {
            ResizeToAtLeast(ref buffer, targetSize, copyCount);
            buffer.length = targetSize;
        }

        [Conditional("LEAKDEBUG")]
        void ValidatePinnedState(bool pinned)
        {
            for (int i = 0; i < pools.Length; ++i)
            {
                var pool = pools[i];
                for (int j = 0; j < pool.BlockCount; ++j)
                {
                    Debug.Assert(pool.Blocks[j].Pinned == pinned, $"For this operation, all blocks must share the same pinned state of {pinned}.");
                }
            }
        }

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
        /// Gets or sets whether the BufferPool's backing resources are pinned. If no blocks are allocated internally, this returns true.
        /// Setting this to false invalidates all outstanding pointers, and any attempt to take or return buffers while unpinned will fail (though not necessarily immediately).
        /// The only valid operations while unpinned are setting Pinned to true and clearing the pool.
        /// </summary>
        public bool Pinned
        {
            get
            {
                //If no blocks exist, we just call it pinned- that's the default state.
                bool pinned = true;
                for (int i = 0; i < pools.Length; ++i)
                {
                    if (pools[i].BlockCount > 0)
                    {
                        pinned = pools[i].Blocks[0].Pinned;
                        break;
                    }
                }
                ValidatePinnedState(pinned);
                return pinned;
            }
            set
            {
                void ChangePinnedState(bool pinned)
                {
                    for (int i = 0; i < pools.Length; ++i)
                    {
                        var pool = pools[i];
                        for (int j = 0; j < pool.BlockCount; ++j)
                        {
                            pool.Blocks[j].Pinned = pinned;
                        }
                    }
                }
                if (value)
                {
                    if (!Pinned)
                    {
                        ChangePinnedState(true);
                    }
                }
                else
                {
                    if (Pinned)
                    {
                        ChangePinnedState(false);
                    }
                }
            }
        }

        /// <summary>
        /// Unpins and drops reference to all memory. Any outstanding buffers will be invalidated silently.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i < pools.Length; ++i)
            {
                pools[i].Clear();
            }
        }

        void IDisposable.Dispose()
        {
            Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetCapacityForCount<T>(int count)
        {
            if (count == 0)
                count = 1;
            return (1 << SpanHelper.GetContainingPowerOf2(count * Unsafe.SizeOf<T>())) / Unsafe.SizeOf<T>();
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
            Debug.Assert(totalBlockCount == 0 || !Pinned, "Memory leak warning! Don't let a buffer pool die without unpinning it!");
        }
#endif

    }
}