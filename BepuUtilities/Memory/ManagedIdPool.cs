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
    /// <remarks>This contrasts with the IdPool which operates on unmanaged memory. This version only exists to support use cases where the unmanaged version can't be used-
    /// for example, in the BufferPool. While the implementation can be shared, doing so involves creating enough supporting infrastructure that it's simpler to have a managed-only version.</remarks>
    public class ManagedIdPool
    {
        private int nextIndex;

        int availableIdCount;
        int[] availableIds;

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

        /// <summary>
        /// Gets the capacity of the id pool for returned ids.
        /// </summary>        
        /// public int Capacity => availableIds.Length;


        public ManagedIdPool(int initialCapacity)
        {
            nextIndex = 0;
            availableIdCount = 0;
            Debug.Assert(initialCapacity > 0);
            availableIds = new int[initialCapacity];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Take()
        {
            if (availableIdCount > 0)
                return availableIds[--availableIdCount];
            return nextIndex++;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(int id)
        {
            if (availableIdCount == availableIds.Length)
            {
                Debug.Assert(availableIdCount > 0);
                InternalResize(availableIdCount * 2);
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
            Debug.Assert(availableIds.Length > availableIdCount);
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

        void InternalResize(int newSize)
        {
            Debug.Assert(availableIds.Length != newSize, "Did you really mean to resize this? Nothing changed!");
            Array.Resize(ref availableIds, newSize);
        }

        /// <summary>
        /// Ensures that the underlying id queue can hold at least a certain number of ids.
        /// </summary>
        /// <param name="count">Number of elements to preallocate space for in the available ids queue.</param>
        public void EnsureCapacity(int count)
        {
            if (availableIds.Length < count)
                InternalResize(count);
        }

        /// <summary>
        /// Shrinks the available ids queue to the smallest size that can fit the given count and the current available id count.
        /// </summary>
        /// <param name="minimumCount">Number of elements to guarantee space for in the available ids queue.</param>
        public void Compact(int minimumCount)
        {
            var targetLength = minimumCount > availableIdCount ? minimumCount : availableIdCount;
            if (availableIds.Length > targetLength)
            {
                InternalResize(targetLength);
            }
        }

        /// <summary>
        /// Resizes the underlying buffer to the smallest size required to hold the given count and the current available id count.
        /// </summary>
        /// <param name="count">Number of elements to guarantee space for in the available ids queue.</param>
        public void Resize(int count)
        {
            var targetLength = count > availableIdCount ? count : availableIdCount;
            if (availableIds.Length != targetLength)
            {
                InternalResize(targetLength);
            }
        }
    }
}
