using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Provides storage for reusable arrays with power-of-2 lengths.
    /// </summary>    
    /// <remarks>
    /// This is a very simple performance-oriented pool that doesn't put much effort into safety.
    /// Incorrect usage will usually be caught in debug mode, but otherwise bad things could happen silently.
    /// </remarks>
    /// <typeparam name="T">Type of resource contained in the buffers.</typeparam>
    public class ArrayPool<T> : IMemoryPool<T, Array<T>>
    {
        private Stack<T[]>[] pools = new Stack<T[]>[SpanHelper.MaximumSpanSizePower + 1];
#if DEBUG
        private HashSet<T[]> outstandingResources = new HashSet<T[]>();
#endif

        /// <summary>
        /// Constructs a new array pool.
        /// </summary>
        /// <param name="expectedPooledResourceCount">Number of arrays to preallocate reference space for.
        /// This does not preallocate actual arrays, just the space to hold references that are waiting in the pool.</param>
        public ArrayPool(int expectedPooledResourceCount = 8)
        {
            for (int i = 0; i < pools.Length; ++i)
            {
                pools[i] = new Stack<T[]>();
            }
        }


        /// <summary>
        /// Ensures that there are at least the specified number of buffers allocated for the given batch index.
        /// </summary>
        /// <param name="power">Index of the power to ensure the count of.</param>
        /// <param name="count">Minimum number of elements that need to exist in the specified pool.</param>
        public void EnsureArrayCount(int power, int count)
        {
            while (pools[power].Count < count)
                pools[power].Push(new T[1 << power]);
        }

        /// <summary>
        /// Gets the number of buffers for a pool index.
        /// </summary>
        /// <param name="power">Index of the pool to count.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetArrayCount(int power)
        {
            return pools[power].Count;
        }

        /// <summary>
        /// Drops all buffer references.
        /// Does not affect outstanding references.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i <= SpanHelper.MaximumSpanSizePower; ++i)
            {
                pools[i].Clear();
            }
        }

        /// <summary>
        /// Takes an array that can hold the given number of elements.
        /// </summary>
        /// <param name="count">Number of elements that must fit in the retrieved span.</param>
        /// <param name="span">Span that can hold the number of elements specified.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out Array<T> span)
        {
            TakeForPower(SpanHelper.GetContainingPowerOf2(count), out span); ;
        }

        /// <summary>
        /// Takes an array of size equal to 2^power.
        /// </summary>
        /// <param name="power">Exponent that defines the size of the span. The span's length will be 2^power.</param>
        /// <param name="array">Span for the given power.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out Array<T> array)
        {
            ValidateCount();
            Debug.Assert(power >= 0 && power <= SpanHelper.MaximumSpanSizePower, $"Pool index should be from 0 to {SpanHelper.MaximumSpanSizePower} inclusive.");
            T[] toReturn;
            if (pools[power].Count > 0)
            {
                toReturn = pools[power].Pop();
            }
            else
                toReturn = new T[1 << power];
#if DEBUG
            outstandingResources.Add(toReturn);
#endif
            array = new Array<T>(toReturn);
        }

        /// <summary>
        /// Releases a array back to the pool without clearing it out.
        /// </summary>
        /// <param name="array">Array to return to the pool.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(ref Array<T> array)
        {
            ValidateCount();
#if DEBUG
            Debug.Assert(outstandingResources.Remove(array.Memory), "The buffer being returned must come from this pool, and buffers should only be returned once.");
            
#endif
            pools[SpanHelper.GetContainingPowerOf2(array.Length)].Push(array.Memory);
            array = new Array<T>();
        }

        [Conditional("DEBUG")]
        void ValidateCount()
        {
#if DEBUG
            //Note the relatively low number of outstanding resources as a threshold compared to the pointer-based pool.
            //While it's not really a problem if a user wants to suballocate ten thousand buffers from a big pinned block,
            //it's really not kind to the GC to have many thousands of array references floating around.
            //If you find yourself hitting this limit, consider if there's another way before just bumping up this number-
            //could you use the pointer-based pool instead? Is there some way around having tons of regions?
            const int maximumOutstandingResources = 1000;
            Debug.Assert(outstandingResources.Count < maximumOutstandingResources, $"Do you REALLY have {maximumOutstandingResources} outstanding resources? Or is this a memory leak?");
#endif
        }        
    }
}
