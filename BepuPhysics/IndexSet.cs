using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Collection of unique indices supporting add, remove, and contains operations.
    /// </summary>
    public struct IndexSet
    {
        /// <summary>
        /// Since we only ever need to support add, remove and contains checks, and because the indices are guaranteed unique,
        /// we can just use packed bitfields. Each bit represents one index's containment state.
        /// </summary>
        /// <remarks> 
        /// This can grow up to the number of (indexCount / 8) bytes in the worst case, but that is much, much smaller than using a dictionary or set.
        /// 16384 bodies would only take 2KB. Even if you have 1000 batches all at that size, it's a pretty unconcerning amount of storage.
        /// (And to be clear, 1000 batches is a crazy pathological number. Most simulations will have less than 20 batches.)
        /// </remarks>
        internal Buffer<ulong> flags;

        const int shift = 6;
        const int mask = 63;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int GetBundleCapacity(int count)
        {
            return (count + mask) >> shift;
        }

        public IndexSet(BufferPool pool, int initialCapacity)
        {
            //Remember; the bundles are 64 flags wide. A default of 128 supports up to 8192 indices without needing resizing...
            flags = new Buffer<ulong>();
            InternalResize(pool, initialCapacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InternalResize(BufferPool pool, int capacity)
        {
            InternalResizeForBundleCount(pool, GetBundleCapacity(capacity));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InternalResizeForBundleCount(BufferPool pool, int bundleCapacity)
        {
            var copyRegionLength = Math.Min(bundleCapacity, flags.Length);
            pool.ResizeToAtLeast(ref flags, bundleCapacity, copyRegionLength);
            //Since the pool's data is not guaranteed to be clean and the indices rely on it being clean, we must clear any memory beyond the copied region.
            if (flags.Length > copyRegionLength)
                flags.Clear(copyRegionLength, flags.Length - copyRegionLength);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(int index)
        {
            var packedIndex = index >> shift;
            return packedIndex < flags.Length && (flags[packedIndex] & (1ul << (index & mask))) > 0;
        }

        /// <summary>
        /// Gets whether the batch could hold the specified indices.
        /// </summary>
        /// <param name="indexList">List of indices to check for in the batch.</param>
        /// <param name="indexCount">Number of referenced indices.</param>
        /// <returns>True if none of the indices are present in the set, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool CanFit(ref int indexList, int indexCount)
        {
            for (int i = 0; i < indexCount; ++i)
            {
                var index = Unsafe.Add(ref indexList, i);
                if (Contains(index))
                {
                    return false;
                }
            }
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddUnsafely(int index, int bundleIndex)
        {
            ref var bundle = ref flags[bundleIndex];
            var slot = 1ul << (index & mask);
            Debug.Assert((bundle & slot) == 0, "Cannot add if it's already present!");
            //Not much point in branching to stop a single instruction that doesn't change the result.
            bundle |= slot;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(int index, BufferPool pool)
        {
            var bundleIndex = index >> shift;
            if (bundleIndex >= flags.Length)
            {
                //Note that the bundle index may be larger than two times the current capacity, since indices are not guaranteed to be appended.
                InternalResizeForBundleCount(pool, 1 << SpanHelper.GetContainingPowerOf2(bundleIndex + 1));
            }
            AddUnsafely(index, bundleIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddUnsafely(int index)
        {
            AddUnsafely(index, index >> shift);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(int index)
        {
            Debug.Assert((flags[index >> shift] & (1ul << (index & mask))) > 0, "If you try to remove a index, it should be present.");
            flags[index >> shift] &= ~(1ul << (index & mask));
        }

        public void Clear()
        {
            flags.Clear(0, flags.Length);
        }

        public void EnsureCapacity(int indexCapacity, BufferPool pool)
        {
            if ((flags.Length << shift) < indexCapacity)
            {
                InternalResize(pool, indexCapacity);
            }
        }

        //While we expose a compaction and resize, using it requires care. It would be a mistake to, for example, shrink beyond the current bodies indices size.
        public void Compact(int indexCapacity, BufferPool pool)
        {
            var desiredBundleCount = BufferPool.GetCapacityForCount<ulong>(GetBundleCapacity(indexCapacity));
            if (flags.Length > desiredBundleCount)
            {
                InternalResizeForBundleCount(pool, desiredBundleCount);
            }
        }
        public void Resize(int indexCapacity, BufferPool pool)
        {
            var desiredBundleCount = BufferPool.GetCapacityForCount<ulong>(GetBundleCapacity(indexCapacity));
            if (flags.Length != desiredBundleCount)
            {
                InternalResizeForBundleCount(pool, desiredBundleCount);
            }
        }
        /// <summary>
        /// Disposes the internal buffer.
        /// </summary>
        /// <remarks>The instance can be reused after a Dispose if EnsureCapacity or Resize is called.
        /// That's a little meaningless given that the instance is a value type, but hey, you don't have to new another one, that's something.</remarks>
        public void Dispose(BufferPool pool)
        {
            Debug.Assert(flags.Length > 0, "Cannot double-dispose.");
            pool.Return(ref flags);
            flags = new Buffer<ulong>();
        }
    }
}
