using BepuUtilities.Collections;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Manages a pool of identifier values. Grabbing an id from the pool picks a number that has been picked and returned before, 
    /// or if none of those are available, the minimum value greater than any existing id.
    /// </summary>
    public struct IdPool
    {
        //TODO: You could make this quite a bit more memory efficient in the case where a bunch of ids are requested, then returned.
        //It would require a little more bookkeeping effort, though; it's likely that either taking or returning ids would slow down a little.

        //Note that all availableIds are guaranteed to be less than nextIndex.
        //[0, nextIndex) contains all currently used ids and ids contained within availableIds.
        private int nextIndex;

        int availableIdCount;
        Buffer<int> availableIds;

        /// <summary>
        /// Gets the highest value which any index claimed thus far could possibly have.
        /// This is not necessarily the current highest claimed index; this value may represent an earlier claim that has already been released.
        /// -1 if nothing has ever been claimed.
        /// </summary>
        public int HighestPossiblyClaimedId
        {
            get { return nextIndex - 1; }
        }

        /// <summary>
        /// Gets the number of previously returned ids waiting in the pool.
        /// </summary>
        public int AvailableIdCount => availableIdCount;

        public int Capacity => availableIds.Length;

        /// <summary>
        /// Gets whether the id pool has backing resources allocated to it and is ready to use.
        /// </summary>
        public bool Allocated => availableIds.Allocated;

        public IdPool(int initialCapacity, IUnmanagedMemoryPool pool)
        {
            nextIndex = 0;
            availableIdCount = 0;
            pool.TakeAtLeast(initialCapacity, out availableIds);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Take()
        {
            Debug.Assert(availableIds.Allocated);
            if (availableIdCount > 0)
                return availableIds[--availableIdCount];
            return nextIndex++;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(int id, IUnmanagedMemoryPool pool)
        {
            Debug.Assert(availableIds.Allocated);
            if (availableIdCount == availableIds.Length)
            {
                var oldAvailableIds = availableIds;
                pool.TakeAtLeast(Math.Max(availableIdCount * 2, availableIds.Length), out availableIds);
                oldAvailableIds.CopyTo(0, availableIds, 0, availableIdCount);
                pool.Return(ref oldAvailableIds);
            }
            ReturnUnsafely(id);
        }

        /// <summary>
        /// Returns an id to the pool without checking if a resize is required on the available id stack.
        /// </summary>
        /// <param name="id">Id to return.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ReturnUnsafely(int id)
        {
            Debug.Assert(availableIds.Allocated && availableIds.Length > availableIdCount);
            availableIds[availableIdCount++] = id;
        }

        /// <summary>
        /// Resets the IdPool.
        /// </summary>
        public void Clear()
        {
            nextIndex = 0;
            availableIdCount = 0;
        }

        void InternalResize(int newSize, IUnmanagedMemoryPool pool)
        {
            var oldAvailableIds = availableIds;
            pool.TakeAtLeast(newSize, out availableIds);
            Debug.Assert(oldAvailableIds.Length != availableIds.Length, "Did you really mean to resize this? Nothing changed!");
            oldAvailableIds.CopyTo(0, availableIds, 0, availableIdCount);
            pool.Return(ref oldAvailableIds);
        }

        /// <summary>
        /// Ensures that the underlying id queue can hold at least a certain number of ids.
        /// </summary>
        /// <param name="count">Number of elements to preallocate space for in the available ids queue.</param>
        /// <param name="pool">Pool to pull resized spans from.</param>
        public void EnsureCapacity(int count, IUnmanagedMemoryPool pool)
        {
            if (!availableIds.Allocated)
            {
                //If this was disposed, we must explicitly rehydrate it.
                this = new IdPool(count, pool);
            }
            else
            {
                if (availableIds.Length < count)
                    InternalResize(count, pool);
            }
        }

        /// <summary>
        /// Shrinks the available ids queue to the smallest size that can fit the given count and the current available id count.
        /// </summary>
        /// <param name="minimumCount">Number of elements to guarantee space for in the available ids queue.</param>
        public void Compact(int minimumCount, IUnmanagedMemoryPool pool)
        {
            Debug.Assert(availableIds.Allocated);
            var targetLength = BufferPool.GetCapacityForCount<int>(Math.Max(minimumCount, availableIdCount));
            if (availableIds.Length > targetLength)
            {
                InternalResize(targetLength, pool);
            }
        }

        /// <summary>
        /// Resizes the underlying buffer to the smallest size required to hold the given count and the current available id count.
        /// </summary>
        /// <param name="count">Number of elements to guarantee space for in the available ids queue.</param>
        public void Resize(int count, IUnmanagedMemoryPool pool)
        {
            if (!availableIds.Allocated)
            {
                //If this was disposed, we must explicitly rehydrate it.
                this = new IdPool(count, pool);
            }
            else
            {
                var targetLength = BufferPool.GetCapacityForCount<int>(Math.Max(count, availableIdCount));
                if (availableIds.Length != targetLength)
                {
                    InternalResize(targetLength, pool);
                }
            }
        }

        /// <summary>
        /// Returns underlying memory to the pool.
        /// </summary>
        /// <remarks>The IdPool can be reused only if EnsureCapacity or Resize is called.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose(IUnmanagedMemoryPool pool)
        {
            pool.Return(ref availableIds);
            //This simplifies reuse and makes it harder to use invalid data.
            this = default;
        }
    }
}
