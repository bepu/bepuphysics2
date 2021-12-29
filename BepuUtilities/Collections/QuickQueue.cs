using System;
using System.Diagnostics;
using BepuUtilities.Memory;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Numerics;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Container supporting double ended queue behaviors built on top of pooled arrays.
    /// </summary>
    /// <remarks>
    /// Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care,
    /// it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, 
    /// it rarely checks input for errors,
    /// the enumerator doesn't check for mid-enumeration modification,
    /// it allows unsafe addition that can break if the user doesn't manage the capacity,
    /// it works on top of an abstracted memory blob which might internally be a pointer that could be rugpulled, 
    /// it does not (and is incapable of) checking that provided memory gets returned to the same pool that it came from.
    /// </remarks>
    /// <typeparam name="T">Type of the elements in the queue.</typeparam>
    public struct QuickQueue<T> where T : unmanaged
    {
        /// <summary>
        /// Number of elements in the queue.
        /// </summary>
        public int Count;

        /// <summary>
        /// Index of the first element in the queue.
        /// </summary>
        public int FirstIndex;

        /// <summary>
        /// Index of the last element in the queue.
        /// </summary>
        public int LastIndex;

        /// <summary>
        /// Mask based on the current span length used to do fast modulo operations; requires that the span has a power of 2 length.
        /// </summary>
        public int CapacityMask;

        /// <summary>
        /// Gets the backing memory containing the elements of the queue.
        /// Indices from FirstIndex to LastIndex inclusive hold actual data. All other data is undefined.
        /// Watch out for wrap around; LastIndex can be less than FirstIndex even when count > 0!
        /// </summary>
        public Buffer<T> Span;

        /// <summary>
        /// Gets the backing array index for the logical queue index.
        /// </summary>
        /// <param name="queueIndex">Index in the logical queue.</param>
        /// <returns>The index in in the backing array corresponding to the given logical queue index.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetBackingArrayIndex(int queueIndex)
        {
            return (FirstIndex + queueIndex) & CapacityMask;
        }

        /// <summary>
        /// Gets an element at the given index in the queue.
        /// 0 gets the element at the FirstIndex. Count-1 would get the element at LastIndex.
        /// </summary>
        /// <param name="index">Index to grab an element from.</param>
        /// <returns>Element at the given index in the queue.</returns>
        public ref T this[int index]
        {
            //TODO: Check inlining.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ValidateIndex(index);
                return ref Span[GetBackingArrayIndex(index)];
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int GetCapacityMask(int spanLength)
        {
            return (1 << BitOperations.Log2((uint)spanLength)) - 1;
        }

        /// <summary>
        /// Creates a new queue.
        /// </summary>
        /// <param name="initialSpan">Span to use as backing memory to begin with.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickQueue(ref Buffer<T> initialSpan)
        {
            Span = initialSpan;
            Count = 0;
            CapacityMask = GetCapacityMask(Span.Length);
            FirstIndex = 0;
            LastIndex = CapacityMask;
            ValidateSpanCapacity(ref initialSpan, CapacityMask);
        }
        /// <summary>
        /// Creates a new queue.
        /// </summary>
        /// <param name="pool">Pool to pull a span from.</param>
        /// <param name="minimumInitialCount">The minimum size of the region to be pulled from the pool. Actual span may be larger.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickQueue(int minimumInitialCount, IUnmanagedMemoryPool pool)
        {
            pool.TakeAtLeast(minimumInitialCount, out Span);
            Count = 0;
            CapacityMask = GetCapacityMask(Span.Length);
            FirstIndex = 0;
            LastIndex = CapacityMask;
            ValidateSpanCapacity(ref Span, CapacityMask);
        }


        /// <summary>
        /// Swaps out the queue's backing memory span for a new span.
        /// If the new span is smaller, the queue's count is truncated and the extra elements are dropped. 
        /// The old span is not cleared or returned to any pool; if it needs to be pooled or cleared, the user must handle it.
        /// </summary>
        /// <param name="newSpan">New span to use.</param>
        /// <param name="oldSpan">Previous span used for elements.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize(ref Buffer<T> newSpan, out Buffer<T> oldSpan)
        {
            Validate();
            Debug.Assert(Span.Length != newSpan.Length, "Resizing without changing the size is pretty peculiar. Is something broken?");
            var oldQueue = this;
            //Truncate length.
            if (oldQueue.Count > newSpan.Length)
                Count = newSpan.Length;
            else
                Count = oldQueue.Count;
            LastIndex = Count - 1;
            FirstIndex = 0;
            CapacityMask = GetCapacityMask(newSpan.Length);
            Span = newSpan;

            oldSpan = oldQueue.Span;

            //There is no guarantee that the count is equal to Elements.Length, so both cases must be covered.
            if (oldQueue.LastIndex >= oldQueue.FirstIndex)
            {
                //The indices are in order and the queue has at least one element in it, so just do one contiguous copy.
                oldSpan.CopyTo(oldQueue.FirstIndex, Span, 0, Count);
            }
            else if (Count > 0)
            {
                //The last index is before the first index, meaning the elements wrap around the end of the span. Do a copy for each contiguous region.
                var firstToEndLength = oldSpan.Length - oldQueue.FirstIndex;
                oldSpan.CopyTo(oldQueue.FirstIndex, Span, 0, firstToEndLength);
                oldSpan.CopyTo(0, Span, firstToEndLength, oldQueue.LastIndex + 1);
            }

        }

        /// <summary>
        /// Resizes the queue's backing array for the given size.
        /// Any elements that do not fit in the resized span are dropped and the count is truncated.
        /// </summary>
        /// <param name="newSize">Minimum number of elements required in the new backing array. Actual capacity of the created span may exceed this size.</param>
        /// <param name="pool">Pool to pull a new span from and return the old span to.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize(int newSize, IUnmanagedMemoryPool pool)
        {
            var targetCapacity = pool.GetCapacityForCount<T>(newSize);
            if (targetCapacity != Span.Length)
            {
                var oldQueue = this;
                pool.TakeAtLeast<T>(newSize, out var newSpan);
                Resize(ref newSpan, out var oldSpan);
                oldQueue.Dispose(pool);
            }
        }

        /// <summary>
        /// Returns the resources associated with the queue to pools. Any managed references still contained within the queue are cleared (and some unmanaged resources may also be cleared).
        /// </summary>
        /// <param name="pool">Pool used for element spans.</param>   
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose(IUnmanagedMemoryPool pool)
        {
            pool.Return(ref Span);
        }
        /// <summary>
        /// Ensures that the queue has enough room to hold the specified number of elements.
        /// </summary>
        /// <param name="count">Number of elements to hold.</param>
        /// <param name="pool">Pool to pull a new span from and return the old span to.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnsureCapacity(int count, IUnmanagedMemoryPool pool)
        {
            if (count >= CapacityMask)
            {
                Resize(count, pool);
            }
        }

        /// <summary>
        /// Compacts the internal buffer to the minimum size required for the number of elements in the queue.
        /// </summary>
        /// <param name="pool">Pool to pull from if necessary.</param>
        public void Compact(IUnmanagedMemoryPool pool)
        {
            Validate();
            var targetCapacity = pool.GetCapacityForCount<T>(Count);
            if (targetCapacity < Span.Length)
                Resize(targetCapacity, pool);
        }

        /// <summary>
        /// Enqueues the element to the end of the queue, incrementing the last index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnqueueUnsafely(in T element)
        {
            Validate();
            ValidateUnsafeAdd();
            Span[(LastIndex = ((LastIndex + 1) & CapacityMask))] = element;
            ++Count;
        }

        /// <summary>
        /// Enqueues the element to the start of the queue, decrementing the first index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnqueueFirstUnsafely(in T element)
        {
            Validate();
            ValidateUnsafeAdd();
            Span[(FirstIndex = ((FirstIndex - 1) & CapacityMask))] = element;
            ++Count;
        }
        
        /// <summary>
        /// Enqueues the element to the end of the queue, incrementing the last index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Enqueue(in T element, IUnmanagedMemoryPool pool)
        {
            Validate();
            if (Count == Span.Length)
                Resize(Span.Length * 2, pool);
            EnqueueUnsafely(element);
        }

        /// <summary>
        /// Enqueues the element to the start of the queue, decrementing the first index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnqueueFirst(in T element, IUnmanagedMemoryPool pool)
        {
            Validate();
            if (Count == Span.Length)
                Resize(Span.Length * 2, pool);
            EnqueueFirstUnsafely(element);
        }

        /// <summary>
        /// Dequeues an element from the start of the queue, incrementing the first index.
        /// </summary>
        /// <returns>Element removed from the queue.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T Dequeue()
        {
            Validate();
            if (Count == 0)
                throw new InvalidOperationException("The queue is empty.");
            var element = Span[FirstIndex];
            DeleteFirst();
            return element;

        }

        /// <summary>
        /// Dequeues an element from the end of the queue, decrementing the last index.
        /// </summary>
        /// <returns>Element removed from the queue.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T DequeueLast()
        {
            Validate();
            if (Count == 0)
                throw new InvalidOperationException("The queue is empty.");
            var element = Span[LastIndex];
            DeleteLast();
            return element;

        }

        /// <summary>
        /// Attempts to dequeue an element from the start of the queue, incrementing the first index.
        /// </summary>
        /// <param name="element">Element removed from the queue, if any.</param>
        /// <returns>True if an element was available to remove, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryDequeue(out T element)
        {
            Validate();
            if (Count > 0)
            {
                element = Span[FirstIndex];
                DeleteFirst();
                return true;
            }
            element = default;
            return false;

        }

        /// <summary>
        /// Attempts to dequeue an element from the end of the queue, decrementing the last index.
        /// </summary>
        /// <param name="element">Element removed from the queue, if any.</param>
        /// <returns>True if an element was available to remove, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryDequeueLast(out T element)
        {
            Validate();
            if (Count > 0)
            {
                element = Span[LastIndex];
                DeleteLast();
                return true;
            }
            element = default(T);
            return false;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void DeleteFirst()
        {
            Span[FirstIndex] = default(T);
            FirstIndex = (FirstIndex + 1) & CapacityMask;
            --Count;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void DeleteLast()
        {
            Span[LastIndex] = default(T);
            LastIndex = (LastIndex - 1) & CapacityMask;
            --Count;
        }

        /// <summary>
        /// Removes the element at the given index, preserving the order of the queue.
        /// </summary>
        /// <param name="queueIndex">Index in the queue to remove. The index is in terms of the conceptual queue, not the backing array.</param>
        public void RemoveAt(int queueIndex)
        {
            Validate();
            ValidateIndex(queueIndex);
            var arrayIndex = GetBackingArrayIndex(queueIndex);
            if (LastIndex == arrayIndex)
            {
                DeleteLast();
                return;
            }
            if (FirstIndex == arrayIndex)
            {
                DeleteFirst();
                return;
            }
            //It's internal.

            //Four possible cases:
            //1) Queue wraps around end and arrayIndex is in [0, lastIndex),
            //2) Queue wraps around end and arrayIndex is in (firstIndex, arrayLength),
            //3) Queue is contiguous and arrayIndex is closer to lastIndex than firstIndex, or
            //4) Queue is contiguous and arrayIndex is closer to firstIndex than lastIndex
            //In cases #1 and #3, we should move [arrayIndex + 1, lastIndex] to [arrayIndex, lastIndex - 1], deleting the last element.
            //In cases #2 and #4, we should move [firstIndex, arrayIndex - 1] to [firstIndex + 1, arrayIndex], deleting the first element.

            if ((FirstIndex > LastIndex && arrayIndex < LastIndex) || //Case 1
                (FirstIndex < LastIndex && (LastIndex - arrayIndex) < (arrayIndex - FirstIndex))) //Case 3
            {
                Span.CopyTo(arrayIndex + 1, Span, arrayIndex, LastIndex - arrayIndex);
                DeleteLast();
            }
            else
            {
                Span.CopyTo(FirstIndex, Span, FirstIndex + 1, arrayIndex - FirstIndex);
                DeleteFirst();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClearSpan(ref Buffer<T> span, int firstIndex, int lastIndex, int count)
        {
            if (lastIndex >= firstIndex)
            {
                span.Clear(firstIndex, count);
            }
            else if (count > 0)
            {
                span.Clear(firstIndex, span.Length - firstIndex);
                span.Clear(0, lastIndex + 1);
            }
        }

        /// <summary>
        /// Clears the queue by setting the count to zero and explicitly setting all relevant indices in the backing array to default values.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            Validate();
            ClearSpan(ref Span, FirstIndex, LastIndex, Count);
            Count = 0;
            FirstIndex = 0;
            LastIndex = CapacityMask;
        }

        /// <summary>
        /// Clears the queue without changing any of the values in the backing array. Be careful about using this if the queue contains reference types.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void FastClear()
        {
            Count = 0;
            FirstIndex = 0;
            LastIndex = CapacityMask;
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(Span, Count, FirstIndex, CapacityMask);
        }

        public struct Enumerator : IEnumerator<T>
        {
            private Buffer<T> span;
            private readonly int count;
            private readonly int firstIndex;
            private readonly int capacityMask;
            private int index;

            public Enumerator(in Buffer<T> span, int count, int firstIndex, int capacityMask)
            {
                this.span = span;
                this.count = count;
                this.firstIndex = firstIndex;
                this.capacityMask = capacityMask;
                index = -1;
            }

            public T Current
            {
                get { return span[(firstIndex + index) & capacityMask]; }
            }

            public void Dispose()
            {
            }

            object System.Collections.IEnumerator.Current
            {
                get { return Current; }
            }

            public bool MoveNext()
            {
                return ++index < count;
            }

            public void Reset()
            {
                index = -1;
            }
        }

        [Conditional("DEBUG")]
        void ValidateIndex(int index)
        {
            Debug.Assert(index >= 0 && index < Count, "Index must be nonnegative and less than the number of elements in the queue.");
        }

        [Conditional("DEBUG")]
        static void ValidateSpanCapacity(ref Buffer<T> span, int capacityMask)
        {
            Debug.Assert((1 << BitOperations.Log2((uint)span.Length)) - 1 == capacityMask,
                "Capacity mask should be the largest power of 2 that fits in the allocated span, minus one. This is necessary for efficient modulo operations.");
        }

        [Conditional("DEBUG")]
        void ValidateUnsafeAdd()
        {
            Debug.Assert(Count <= CapacityMask, "Unsafe adders can only be used if the capacity is guaranteed to hold the new size.");
        }

        [Conditional("DEBUG")]
        private void Validate()
        {
            Debug.Assert(Span.Length > 0, "Any QuickQueue in use should have a nonzero length Span. Was this instance default constructed without further initialization?");
            ValidateSpanCapacity(ref Span, CapacityMask);
        }

    }
}
