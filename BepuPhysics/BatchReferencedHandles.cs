using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Contains the set of body handles referenced by a constraint batch.
    /// </summary>
    struct BatchReferencedHandles
    {
        /// <summary>
        /// Since we only ever need to support add, remove and contains checks, and because body handles are guaranteed unique,
        /// we can just use packed bitfields. Each bit represents one body handle's containment state.
        /// </summary>
        /// <remarks> 
        /// This can grow up to the number of (bodies / 8) bytes in the worst case, but that is much, much smaller than using a dictionary or set.
        /// 16384 bodies would only take 2KB. Even if you have 1000 batches all at that size, it's a pretty unconcerning amount of storage.
        /// (And to be clear, 1000 batches is a crazy pathological number. Most simulations will have less than 20 batches.)
        /// </remarks>
        internal Buffer<ulong> packedHandles;

        const int shift = 6;
        const int mask = 63;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int GetSizeInLongs(int count)
        {
            return (count >> shift) + ((count & mask) > 0 ? 1 : 0);
        }

        public BatchReferencedHandles(BufferPool pool, int initialHandleCapacity)
        {
            //Remember; the bundles are 64 bodies wide. A default of 128 supports up to 8192 handles without needing resizing...
            packedHandles = new Buffer<ulong>();
            InternalResize(pool, initialHandleCapacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InternalResize(BufferPool pool, int handleCapacity)
        {
            InternalResizeForBundleCount(pool, GetSizeInLongs(handleCapacity));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InternalResizeForBundleCount(BufferPool pool, int bundleCapacity)
        {
            var copyRegionLength = Math.Min(bundleCapacity, packedHandles.Length);
            pool.SpecializeFor<ulong>().Resize(ref packedHandles, bundleCapacity, copyRegionLength);
            //Since the pool's data is not guaranteed to be clean and the handles rely on it being clean, we must clear any memory beyond the copied region.
            if (packedHandles.Length > copyRegionLength)
                packedHandles.Clear(copyRegionLength, packedHandles.Length - copyRegionLength);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(int handle)
        {
            var packedIndex = handle >> shift;
            return packedIndex < packedHandles.Length && (packedHandles[packedIndex] & (1ul << (handle & mask))) > 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(int handle, BufferPool pool)
        {
            var bundleIndex = handle >> shift;
            if (bundleIndex >= packedHandles.Length)
            {
                //Note that the bundle index may be larger than two times the current capacity, since handles are not guaranteed to be appended.
                InternalResizeForBundleCount(pool, 1 << SpanHelper.GetContainingPowerOf2(bundleIndex + 1));
            }
            ref var bundle = ref packedHandles[bundleIndex];
            var slot = 1ul << (handle & mask);
            Debug.Assert((bundle & slot) == 0, "Cannot add if it's already present!");
            //Not much point in branching to stop a single instruction that doesn't change the result.
            bundle |= slot;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(int handle)
        {
            Debug.Assert((packedHandles[handle >> shift] & (1ul << (handle & mask))) > 0, "If you remove a handle, it should be present.");
            packedHandles[handle >> shift] &= ~(1ul << (handle & mask));
        }

        public void Clear()
        {
            packedHandles.Clear(0, packedHandles.Length);
        }

        public void EnsureCapacity(int handleCount, BufferPool pool)
        {
            if ((packedHandles.Length << shift) < handleCount)
            {
                InternalResize(pool, handleCount);
            }
        }

        //While we expose a compaction and resize, using it requires care. It would be a mistake to shrink beyond the current bodies handles size.
        public void Compact(int handleCount, BufferPool pool)
        {
            var desiredBundleCount = BufferPool<ulong>.GetLowestContainingElementCount(GetSizeInLongs(handleCount));
            if (packedHandles.Length > desiredBundleCount)
            {
                InternalResizeForBundleCount(pool, desiredBundleCount);
            }
        }
        public void Resize(int handleCount, BufferPool pool)
        {
            var desiredBundleCount = BufferPool<ulong>.GetLowestContainingElementCount(GetSizeInLongs(handleCount));
            if (packedHandles.Length != desiredBundleCount)
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
            Debug.Assert(packedHandles.Length > 0, "Cannot double-dispose.");
            pool.SpecializeFor<ulong>().Return(ref packedHandles);
            packedHandles = new Buffer<ulong>();
        }
    }
}
