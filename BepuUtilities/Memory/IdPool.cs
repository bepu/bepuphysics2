using BepuUtilities.Collections;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Manages a pool of identifier values. Grabbing an id from the pool picks a number that has been picked and returned before, 
    /// or if none of those are available, the minimum value greater than any existing id.
    /// </summary>
    public struct IdPool<TSpan> where TSpan : ISpan<int>
    {
        //TODO: You could make this quite a bit more memory efficient in the case where a bunch of ids are requested, then returned.
        //It would require a little more bookkeeping effort, though; it's likely that either taking or returning ids would slow down a little.
        private int nextIndex;

        /// <summary>
        /// Gets the highest value which any index claimed thus far could possibly have.
        /// This is not necessarily the current highest claimed index; this value may represent an earlier claim that has already been released.
        /// -1 if nothing has ever been claimed.
        /// </summary>
        public int HighestPossiblyClaimedId
        {
            get { return nextIndex - 1; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create<TPool>(TPool pool, int initialCapacity, out IdPool<TSpan> idPool) where TPool : IMemoryPool<int, TSpan>
        {
            idPool.nextIndex = 0;
            QuickList<int, TSpan>.Create(pool, initialCapacity, out idPool.AvailableIds);
        }

        //Note that all availableIds are guaranteed to be less than nextIndex.
        //[0, nextIndex) contains all currently used ids and ids contained within availableIds.
        public QuickList<int, TSpan> AvailableIds;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Take()
        {
            if (AvailableIds.TryPop(out var id))
                return id;
            return nextIndex++;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return<TPool>(int id, TPool pool) where TPool : IMemoryPool<int, TSpan>
        {
            AvailableIds.Add(id, pool);
        }

        /// <summary>
        /// Returns an id to the pool without checking if a resize is required on the available id stack.
        /// </summary>
        /// <param name="id">Id to return.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ReturnUnsafely(int id)
        {
            AvailableIds.AddUnsafely(id);
        }

        /// <summary>
        /// Resets the IdPool without returning any resources to the underlying memory pool.
        /// </summary>
        public void Clear()
        {
            nextIndex = 0;
            AvailableIds.Count = 0;
        }

        /// <summary>
        /// Ensures that the underlying id queue can hold at least a certain number of ids.
        /// </summary>
        /// <param name="count">Number of elements to preallocate space for in the available ids queue.</param>
        /// <param name="pool">Pool to pull resized spans from.</param>
        public void EnsureCapacity<TPool>(int count, TPool pool) where TPool : IMemoryPool<int, TSpan>
        {
            if (!AvailableIds.Span.Allocated)
            {
                //If this was disposed, we must explicitly rehydrate it.
                QuickList<int, TSpan>.Create(pool, count, out AvailableIds);
            }
            else
            {
                AvailableIds.EnsureCapacity(count, pool);
            }
        }

        /// <summary>
        /// Shrinks the available ids queue to the smallest size that can fit the given count and the current available id count.
        /// </summary>
        /// <param name="queuedCount">Number of elements to guarantee space for in the available ids queue.</param>
        public void Compact<TPool>(int queuedCount, TPool pool) where TPool : IMemoryPool<int, TSpan>
        {
            var targetLength = BufferPool<int>.GetLowestContainingElementCount(Math.Max(queuedCount, AvailableIds.Count));
            if (AvailableIds.Span.Length > targetLength)
            {
                AvailableIds.Resize(targetLength, pool);
            }
        }
        /// <summary>
        /// Resizes the underlying buffer to the smallest size required to hold the given count and the current available id count.
        /// </summary>
        /// <param name="count">Number of elements to guarantee space for in the available ids queue.</param>
        public void Resize<TPool>(int count, TPool pool) where TPool : IMemoryPool<int, TSpan>
        {
            if (!AvailableIds.Span.Allocated)
            {
                //If this was disposed, we must explicitly rehydrate it.
                QuickList<int, TSpan>.Create(pool, count, out AvailableIds);
                return;
            }
            var targetLength = BufferPool<int>.GetLowestContainingElementCount(Math.Max(count, AvailableIds.Count));
            if (AvailableIds.Span.Length != targetLength)
            {
                AvailableIds.Resize(targetLength, pool);
            }
        }

        /// <summary>
        /// Returns underlying memory to the pool.
        /// </summary>
        /// <remarks>The IdPool can be reused only if EnsureCapacity or Resize is called.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose<TPool>(TPool pool) where TPool : IMemoryPool<int, TSpan>
        {
            AvailableIds.Dispose(pool);
            //This simplifies reuse and makes it harder to use invalid data.
            nextIndex = 0;
            AvailableIds = new QuickList<int, TSpan>();
        }
    }
}
