using System;
using System.Collections.Generic;
using System.Diagnostics;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Container supporting constant time adds and removes while preserving fast iteration times.
    /// Offers very direct access to information at the cost of safety.
    /// </summary>
    /// <remarks>
    /// <para>
    /// Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care,
    /// it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, 
    /// it rarely checks input for errors,
    /// the enumerator doesn't check for mid-enumeration modification,
    /// it allows unsafe addition that can break if the user doesn't manage the capacity,
    /// it works on top of an abstracted memory blob which might internally be a pointer that could be rugpulled, 
    /// it does not (and is incapable of) checking that provided memory gets returned to the same pool that it came from.
    /// </para>
    /// <para>Note that the implementation is extremely simple. It uses single-step linear probing under the assumption of very low collision rates.
    /// A generous table capacity is recommended; this trades some memory for simplicity and runtime performance.</para></remarks>
    /// <typeparam name="T">Type of element held by the container.</typeparam>
    /// <typeparam name="TEqualityComparer">Type of the equality tester and hash calculator used.</typeparam>
    public struct QuickSet<T, TEqualityComparer> where TEqualityComparer : IEqualityComparerRef<T> where T : unmanaged
    {
        /// <summary>
        /// Gets the number of elements in the set.
        /// </summary>
        public int Count;
        /// <summary>
        /// Mask for use in performing fast modulo operations for hashes. Requires that the table span is a power of 2.
        /// </summary>
        public int TableMask;
        /// <summary>
        /// Desired size of the table relative to the size of the key/value spans in terms of a power of 2. Table capacity target will be elementCapacityTarget * 2^TablePowerOffset.
        /// </summary>
        public int TablePowerOffset;

        /// <summary>
        /// Backing memory of the set's table. Values are distributed according to the EqualityComparer's hash function.
        /// Slots containing 0 are unused and point to nothing. Slots containing higher values are equal to one plus the index of an element in the Span.
        /// </summary>
        public Buffer<int> Table;

        /// <summary>
        /// Backing memory containing the elements of the set.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public Buffer<T> Span;

        /// <summary>
        /// Equality comparer used to compare and hash elements.
        /// </summary>
        public TEqualityComparer EqualityComparer;


        /// <summary>
        /// Gets or sets an element at the given index in the list representation of the set.
        /// </summary>
        /// <param name="index">Index to grab an element from.</param>
        /// <returns>Element at the given index in the set.</returns>
        public ref T this[int index]
        {
            //TODO: Check inlining
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(index >= 0 && index < Count, "Index should be within the set's size.");
                return ref Span[index];
            }
        }

        /// <summary>
        /// Creates a new set.
        /// </summary>
        /// <param name="initialSpan">Span to use as backing memory of the set elements.</param>
        /// <param name="initialTableSpan">Span to use as backing memory of the table. Must be zeroed out.</param>
        /// <param name="comparer">Comparer to use for the set.</param>
        /// <param name="tablePowerOffset">Target size of the table relative to the number of stored elements.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickSet(ref Buffer<T> initialSpan, ref Buffer<int> initialTableSpan, TEqualityComparer comparer, int tablePowerOffset = 2)
        {
            ValidateSpanCapacity(ref initialSpan, ref initialTableSpan);
            Span = initialSpan;
            Table = initialTableSpan;
            Count = 0;
            TableMask = Table.Length - 1;
            TablePowerOffset = tablePowerOffset;
            EqualityComparer = comparer;
            Debug.Assert(EqualityComparer != null);
            ValidateTableIsCleared(ref initialTableSpan);
        }

        /// <summary>
        /// Creates a new set using a default constructed equality comparer.
        /// </summary>
        /// <param name="initialSpan">Span to use as backing memory of the set elements.</param>
        /// <param name="initialTableSpan">Span to use as backing memory of the table. Must be zeroed.</param>
        /// <param name="tablePowerOffset">Target size of the table relative to the number of stored elements.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickSet(ref Buffer<T> initialSpan, ref Buffer<int> initialTableSpan, int tablePowerOffset = 2)
            : this(ref initialSpan, ref initialTableSpan, default, tablePowerOffset)
        {
        }

        /// <summary>
        /// Creates a new set.
        /// </summary>
        /// <param name="pool">Pool to pull spans from.</param>   
        /// <param name="initialCapacity">Initial target size of the key and value spans. The size of the initial buffer will be at least as large as the initialCapacity.</param>
        /// <param name="tableSizePower">Target capacity relative to the initial capacity in terms of a power of 2. The size of the initial table buffer will be at least 2^tableSizePower times larger than the initial capacity.</param>
        /// <param name="comparer">Comparer to use in the set.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickSet(int initialCapacity, int tableSizePower, IUnmanagedMemoryPool pool, TEqualityComparer comparer)
        {
            pool.TakeAtLeast<T>(initialCapacity, out var span);
            pool.TakeAtLeast<int>(span.Length << tableSizePower, out var tableSpan);
            //No guarantee that the table is clean; clear it.
            tableSpan.Clear(0, tableSpan.Length);
            this = new QuickSet<T, TEqualityComparer>(ref span, ref tableSpan, comparer, tableSizePower);
        }

        /// <summary>
        /// Creates a new set with a default constructed comparer.
        /// </summary>
        /// <param name="pool">Pool to pull spans from.</param>   
        /// <param name="initialCapacity">Initial target size of the key and value spans. The size of the initial buffer will be at least as large as the initialCapacity.</param>
        /// <param name="tableSizePower">Target capacity relative to the initial capacity in terms of a power of 2. The size of the initial table buffer will be at least 2^tableSizePower times larger than the initial capacity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickSet(int initialCapacity, int tableSizePower, IUnmanagedMemoryPool pool)
            : this(initialCapacity, tableSizePower, pool, default)
        {
        }

        /// <summary>
        /// Creates a new set with a default constructed comparer.
        /// </summary>
        /// <param name="pool">Pool to pull spans from.</param>   
        /// <param name="initialCapacity">Initial target size of the key and value spans. The size of the initial buffer will be at least as large as the initialCapacity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickSet(int initialCapacity, IUnmanagedMemoryPool pool)
            : this(initialCapacity, 2, pool, default)
        {
        }

        /// <summary>
        /// Swaps out the set's backing memory span for a new span.
        /// If the new span is smaller, the set's count is truncated and the extra elements are dropped. 
        /// The old span is not cleared or returned to any pool; if it needs to be pooled or cleared, the user must handle it.
        /// </summary>      
        /// <param name="newSpan">New span to use for elements.</param>
        /// <param name="newTableSpan">New span to use for the table. Must be zeroed.</param>
        /// <param name="oldSpan">Previous span used for elements.</param>
        /// <param name="oldTableSpan">Previous span used for the table.</param>
        public void Resize(ref Buffer<T> newSpan, ref Buffer<int> newTableSpan, out Buffer<T> oldSpan, out Buffer<int> oldTableSpan)
        {
            ValidateSpanCapacity(ref newSpan, ref newTableSpan);
            ValidateTableIsCleared(ref newTableSpan);
            var oldSet = this;
            Span = newSpan;
            Table = newTableSpan;
            Count = 0;
            TableMask = newTableSpan.Length - 1;
            var newCount = oldSet.Count > newSpan.Length ? newSpan.Length : oldSet.Count;

            //Unfortunately we can't really do a straight copy; the backing table relies on modulo operations.
            //Technically, we could copy the regular set and then rely on a partial add to take care of the rest, but bleh!
            //Should really attempt to avoid resizes on sets and dictionaries whenever possible anyway. It ain't fast.
            for (int i = 0; i < newCount; ++i)
            {
                //We assume that ref adds will get inlined reasonably here. That's not actually guaranteed, but we'll bite the bullet.
                //(You could technically branch on the Unsafe.SizeOf<T>, which should result in a compile time specialized zero overhead implementation... but meh!)
                AddUnsafely(ref oldSet.Span[i]);
            }
            oldSpan = oldSet.Span;
            oldTableSpan = oldSet.Table;

        }

        /// <summary>
        /// Resizes the set's backing array for the given size.
        /// If the new span is smaller, the set's count is truncated and the extra elements are dropped. 
        /// </summary>
        /// <param name="newSize">Minimum size of the new object memory block. Actual size may be larger.</param>
        /// <param name="pool">Pool to take spans from.</param>   
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize(int newSize, IUnmanagedMemoryPool pool)
        {
            var targetCapacity = pool.GetCapacityForCount<T>(newSize);
            if (targetCapacity != Span.Length)
            {
                var oldSet = this;
                pool.TakeAtLeast<T>(newSize, out var newSpan);
                pool.TakeAtLeast<int>(newSpan.Length << TablePowerOffset, out var newTableSpan);
                //There is no guarantee that the table retrieved from the pool is clean. Clear it!
                newTableSpan.Clear(0, newTableSpan.Length);
                Resize(ref newSpan, ref newTableSpan, out var oldSpan, out var oldTableSpan);
                oldSet.Dispose(pool);
            }
        }

        /// <summary>
        /// Returns the resources associated with the set to pools. Any managed references still contained within the set are cleared (and some unmanaged resources may also be cleared).
        /// </summary>
        /// <param name="pool">Pool used for element spans.</param>   
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose(IUnmanagedMemoryPool pool)
        {
            pool.Return(ref Span);
            pool.Return(ref Table);
        }

        /// <summary>
        /// Ensures that the set has enough room to hold the specified number of elements.
        /// </summary>     
        /// <param name="pool">Pool used for spans.</param>   
        /// <param name="count">Number of elements to hold.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnsureCapacity(int count, IUnmanagedMemoryPool pool)
        {
            if (count > Span.Length)
            {
                Resize(count, pool);
            }
        }

        /// <summary>
        /// Shrinks the internal buffers to the smallest acceptable size and releases the old buffers to the pools.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        public void Compact(IUnmanagedMemoryPool pool)
        {
            Validate();
            var targetCapacity = pool.GetCapacityForCount<T>(Count);
            if (targetCapacity < Span.Length)
                Resize(Count, pool);
        }

        /// <summary>
        /// Gets the index of the element in the table.
        /// </summary>
        /// <param name="element">Element to look up.</param>
        /// <param name="tableIndex">Index of the element in the redirect table, or if it is not present, the index of where it would be added.</param>
        /// <param name="elementIndex">The index of the element in the elements array, if it exists; -1 otherwise.</param>
        /// <returns>True if the element is present in the set, false if it is not.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool GetTableIndices(ref T element, out int tableIndex, out int elementIndex)
        {
            Validate();
            //The table lengths are guaranteed to be a power of 2, so the modulo is a simple binary operation.
            tableIndex = HashHelper.Rehash(EqualityComparer.Hash(ref element)) & TableMask;
            //0 in the table means 'not taken'; all other values are offset by 1 upward. That is, 1 is actually index 0, 2 is actually index 1, and so on.
            //This is preferred over using a negative number for flagging since clean buffers will contain all 0's.
            while ((elementIndex = Table[tableIndex]) > 0)
            {
                //This table index is taken. Is this the specified element?
                //Remember to decode the object index.
                if (EqualityComparer.Equals(ref Span[--elementIndex], ref element))
                {
                    return true;
                }
                tableIndex = (tableIndex + 1) & TableMask;
            }
            elementIndex = -1;
            return false;
        }

        /// <summary>
        /// Gets the index of the element in the set if it exists.
        /// </summary>
        /// <param name="element">Element to get the index of.</param>
        /// <returns>The index of the element if the element exists in the set, -1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element)
        {
            GetTableIndices(ref element, out int tableIndex, out int objectIndex);
            return objectIndex;
        }

        /// <summary>
        /// Gets the index of the element in the set if it exists.
        /// </summary>
        /// <param name="element">Element to get the index of.</param>
        /// <returns>The index of the element if the element exists in the set, -1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref T element)
        {
            GetTableIndices(ref element, out int tableIndex, out int objectIndex);
            return objectIndex;
        }

        /// <summary>
        /// Checks if a given element already belongs to the set.
        /// </summary>
        /// <param name="element">Element to test for.</param>
        /// <returns>True if the element already belongs to the set, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(T element)
        {
            return GetTableIndices(ref element, out int tableIndex, out int objectIndex);
        }

        /// <summary>
        /// Checks if a given element already belongs to the set.
        /// </summary>
        /// <param name="element">Element to test for.</param>
        /// <returns>True if the element already belongs to the set, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(ref T element)
        {
            return GetTableIndices(ref element, out int tableIndex, out int objectIndex);
        }

        /// <summary>
        /// Adds an element to the set. If a version of the element (same hash code, 'equal' by comparer) is already present,
        /// it is replaced by the given version.
        /// Does not resize in the event that the capacity is exceeded.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present and was instead replaced.</returns>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)] //TODO: Test performance of full chain inline.
        public bool AddAndReplaceUnsafely(ref T element)
        {
            Validate();
            if (GetTableIndices(ref element, out int tableIndex, out int elementIndex))
            {
                //Already present!
                Span[elementIndex] = element;
                return false;
            }

            //It wasn't in the set. Add it!
            ValidateUnsafeAdd();
            Span[Count] = element;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = ++Count;
            return true;
        }

        /// <summary>
        /// Adds an element to the set. If a version of the element (same hash code, 'equal' by comparer) is already present,
        /// it is replaced by the given version.
        /// Does not resize in the event that the capacity is exceeded.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present and was instead replaced.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddAndReplaceUnsafely(T element)
        {
            return AddAndReplaceUnsafely(ref element);
        }

        /// <summary>
        /// Adds an element to the set if it is not already present.
        /// Does not resize in the event that the capacity is exceeded.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present.</returns>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)] //TODO: Test performance of full chain inline.
        public bool AddUnsafely(ref T element)
        {
            Validate();
            if (GetTableIndices(ref element, out int tableIndex, out int elementIndex))
            {
                //Already present!
                return false;
            }

            //It wasn't in the set. Add it!
            ValidateUnsafeAdd();
            Span[Count] = element;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = ++Count;
            return true;
        }

        /// <summary>
        /// Adds an element to the set if it is not already present.
        /// Does not resize in the event that the capacity is exceeded.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddUnsafely(T element)
        {
            return AddUnsafely(ref element);
        }

        /// <summary>
        /// Adds an element to the set. If a version of the element (same hash code, 'equal' by comparer) is already present,
        /// it is replaced by the given version.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        /// <returns>True if the element was added to the set, false if the element was already present and was instead replaced.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddAndReplace(ref T element, IUnmanagedMemoryPool pool)
        {
            if (Count == Span.Length)
            {
                //There's no room left; resize.
                Resize(Count * 2, pool);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }
            return AddAndReplaceUnsafely(ref element);
        }


        /// <summary>
        /// Adds an element to the set if it is not already present.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        /// <returns>True if the element was added to the set, false if the element was already present.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Add(ref T element, IUnmanagedMemoryPool pool)
        {
            if (Count == Span.Length)
            {
                //There's no room left; resize.
                Resize(Count * 2, pool);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }
            return AddUnsafely(ref element);
        }


        /// <summary>
        /// Adds an element to the set. If a version of the element (same hash code, 'equal' by comparer) is already present,
        /// it is replaced by the given version.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        /// <returns>True if the element was added to the set, false if the element was already present and was instead replaced.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddAndReplace(T element, IUnmanagedMemoryPool pool)
        {
            return AddAndReplaceUnsafely(ref element);
        }


        /// <summary>
        /// Adds an element to the set if it is not already present.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        /// <returns>True if the element was added to the set, false if the element was already present.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Add(T element, IUnmanagedMemoryPool pool)
        {
            return Add(ref element, pool);
        }

        //Note: the reason this is named "FastRemove" instead of just "Remove" despite it being the only remove present is that
        //there may later exist an order preserving "Remove". That would be a very sneaky breaking change.

        /// <summary>
        /// Removes an element from the dictionary according to its table and element index. Can only be used if the table and element index are valid.
        /// </summary>
        /// <param name="tableIndex">Index of the table entry associated with the existing element to remove.</param>
        /// <param name="elementIndex">Index of the existing element to remove in the contiguous element array.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void FastRemove(int tableIndex, int elementIndex)
        {
            Debug.Assert(GetTableIndices(ref Span[elementIndex], out var debugTableIndex, out var debugElementIndex) && debugTableIndex == tableIndex && debugElementIndex == elementIndex,
                 "The table index and element index used to directly remove must match an actual element.");

            //Add and remove must both maintain a property:
            //All items are either at their desired index (as defined by the hash), or they are contained in a contiguous block clockwise from the desired index.
            //Removals seek to fill the gap they create by searching clockwise to find items which can be moved backward.
            //Search clockwise for an item to fill this slot. The search must continue until a gap is found.
            int moveCandidateIndex;
            int gapIndex = tableIndex;
            //Search clockwise.
            while ((moveCandidateIndex = Table[tableIndex = (tableIndex + 1) & TableMask]) > 0)
            {
                //This slot contains something. What is its actual index?
                --moveCandidateIndex;
                int desiredIndex = HashHelper.Rehash(EqualityComparer.Hash(ref Span[moveCandidateIndex])) & TableMask;

                //Would this element be closer to its actual index if it was moved to the gap?
                //To find out, compute the clockwise distance from the gap and the clockwise distance from the ideal location.

                var distanceFromGap = (tableIndex - gapIndex) & TableMask;
                var distanceFromIdeal = (tableIndex - desiredIndex) & TableMask;
                if (distanceFromGap <= distanceFromIdeal)
                {
                    //The distance to the gap is less than or equal the distance to the ideal location, so just move to the gap.
                    Table[gapIndex] = Table[tableIndex];
                    gapIndex = tableIndex;
                }

            }
            //Clear the table gap left by the removal.
            Table[gapIndex] = 0;
            //Swap the final element into the removed object's element array index, if the removed object wasn't the last object.
            --Count;
            if (elementIndex < Count)
            {
                Span[elementIndex] = Span[Count];
                //Locate the swapped object in the table and update its index.
                GetTableIndices(ref Span[elementIndex], out tableIndex, out int oldElementIndex);
                Table[tableIndex] = elementIndex + 1; //Remember the encoding! all indices offset by 1.
            }
            //Clear the final slot in the elements set.
            Span[Count] = default;
        }

        /// <summary>
        /// Removes an element from the set if belongs to the set.
        /// Does not preserve the order of elements in the set.
        /// </summary>
        /// <param name="element">Element to remove.</param>
        /// <returns>True if the element was found and removed, false otherwise.</returns>
        public bool FastRemove(ref T element)
        {
            Validate();
            //Find it.
            if (GetTableIndices(ref element, out int tableIndex, out int elementIndex))
            {
                //We found the object!
                FastRemove(tableIndex, elementIndex);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the set if belongs to the set.
        /// Does not preserve the order of elements in the set.
        /// </summary>
        /// <param name="element">Element to remove.</param>
        /// <returns>True if the element was found and removed, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool FastRemove(T element)
        {
            return FastRemove(ref element);
        }


        /// <summary>
        /// Removes all elements from the set.
        /// </summary>
        public void Clear()
        {
            //While it may be appealing to remove individual elements from the set when the set is sparse,
            //using a brute force clear over the entire table is almost always faster. And it's a lot simpler!
            Table.Clear(0, Table.Length);
            Span.Clear(0, Count);
            Count = 0;
        }

        /// <summary>
        /// Removes all elements from the set without modifying the contents of the elements array.
        /// </summary>
        public void FastClear()
        {
            Table.Clear(0, Table.Length);
            Count = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static implicit operator Span<T>(in QuickSet<T, TEqualityComparer> set)
        {
            return new Span<T>(set.Span.Memory, set.Count);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static implicit operator ReadOnlySpan<T>(in QuickSet<T, TEqualityComparer> set)
        {
            return new ReadOnlySpan<T>(set.Span.Memory, set.Count);
        }

        public Enumerator GetEnumerator()
        {
            Validate();
            return new Enumerator(ref Span, Count);
        }


        public struct Enumerator : IEnumerator<T>
        {
            private readonly Buffer<T> backingArray;
            private readonly int count;
            private int index;

            public Enumerator(ref Buffer<T> backingArray, int count)
            {
                this.backingArray = backingArray;
                this.count = count;

                index = -1;
            }

            public T Current
            {
                get { return backingArray[index]; }
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
        static void ValidateSpanCapacity(ref Buffer<T> span, ref Buffer<int> tableSpan)
        {
            Debug.Assert(tableSpan.Length >= span.Length, "The table span must be at least as large as the element span.");
            Debug.Assert((tableSpan.Length & (tableSpan.Length - 1)) == 0, "QuickSets depend upon power of 2 backing table span sizes for efficient modulo operations.");
        }

        [Conditional("DEBUG")]
        private void Validate()
        {
            Debug.Assert(Span.Length != 0 && Table.Length != 0, "The QuickSet must have its internal buffers and pools available; default-constructed or disposed sets should not be used.");
            ValidateSpanCapacity(ref Span, ref Table);
        }

        [Conditional("DEBUG")]
        void ValidateUnsafeAdd()
        {
            Debug.Assert(Count < Span.Length, "Unsafe adders can only be used if the capacity is guaranteed to hold the new size.");
        }

        [Conditional("DEBUG")]
        void ValidateTableIsCleared(ref Buffer<int> span)
        {
            for (int i = 0; i < span.Length; ++i)
            {
                Debug.Assert(span[i] == 0, "The table provided to the set must be cleared.");
            }
        }



    }
}
