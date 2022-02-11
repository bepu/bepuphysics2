using System;
using System.Collections.Generic;
using System.Diagnostics;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Contains basic helpers for hashing.
    /// </summary>
    public static class HashHelper
    {
        /// <summary>
        /// Redistributes a hash. Useful for converting unique but contiguous hashes into a semirandom distribution.
        /// </summary>
        /// <param name="hash">Hash to redistribute.</param>
        /// <returns>Hashed hash.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Rehash(int hash)
        {
            //This rehash aims to address two problems:
            //1) Many common keys, such as ints and longs, will result in contiguous hash codes. 
            //Contiguous hash codes result in contiguous table entries, which destroy this implementation's linear probing performance.
            //2) Many common hashes have significantly patterned input, such as having all 0's in the lower bits. Since this implementation uses pow2-sized tables, 
            //patterns which could align with the pow2 table size can cause massive numbers of collisions.
            //So, we apply an additional scrambling pass on the hash to get rid of most such patterns. This won't stop a malicious attacker, but most coincidences should be avoided.
            //Keep in mind that this implementation is performance critical- there's no time for a bunch of rounds.
            //The initial multiplication serves to avoid some contiguity-induced patterning, the 
            //following xor'd rotations take care of most bit distribution patterning, and together it's super cheap. 
            //(You may be familiar with these rotation constants from SHA2 rounds.)

            const int a = 6;
            const int b = 13;
            const int c = 25;
            uint uhash = (uint)hash * 982451653u;
            var redongled =
                ((uhash << a) | (uhash >> (32 - a))) ^
                ((uhash << b) | (uhash >> (32 - b))) ^
                ((uhash << c) | (uhash >> (32 - c)));
            return (int)redongled;
        }
    }
    /// <summary>
    /// Container supporting constant time adds and removes of key-value pairs while preserving fast iteration times.
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
    /// <typeparam name="TKey">Type of key held by the container.</typeparam>
    /// <typeparam name="TValue">Type of value held by the container.</typeparam>
    /// <typeparam name="TEqualityComparer">Type of the equality tester and hash calculator used.</typeparam>
    public struct QuickDictionary<TKey, TValue, TEqualityComparer> where TKey : unmanaged where TValue : unmanaged where TEqualityComparer : IEqualityComparerRef<TKey>
    {
        /// <summary>
        /// Gets the number of elements in the dictionary.
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
        /// Backing memory of the dictionary's table. Values are distributed according to the EqualityComparer's hash function.
        /// Slots containing 0 are unused and point to nothing. Slots containing higher values are equal to one plus the index of an element in the Span.
        /// </summary>
        public Buffer<int> Table;

        /// <summary>
        /// Backing memory containing the keys of the dictionary.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public Buffer<TKey> Keys;

        /// <summary>
        /// Backing memory containing the values of the dictionary.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public Buffer<TValue> Values;

        /// <summary>
        /// Equality comparer used to compare and hash keys.
        /// </summary>
        public TEqualityComparer EqualityComparer;

        /// <summary>
        /// Gets or sets a key-value pair at the given index in the list representation.
        /// </summary>
        /// <param name="index">Index to grab a pair from.</param>
        /// <returns>Pair at the given index in the dictionary.</returns>
        public KeyValuePair<TKey, TValue> this[int index]
        {
            //You would think that such a trivial accessor would inline without any external suggestion.
            //Sometimes, yes. Sometimes, no. :(
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(index >= 0 && index < Count, "Index should be within the dictionary's size.");
                return new KeyValuePair<TKey, TValue>(Keys[index], Values[index]);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                Debug.Assert(index >= 0 && index < Count, "Index should be within the dictionary's size.");
                Keys[index] = value.Key;
                Values[index] = value.Value;
            }
        }


        /// <summary>
        /// Creates a new dictionary.
        /// </summary>
        /// <param name="initialKeySpan">Span to use as backing memory of the dictionary keys.</param>
        /// <param name="initialValueSpan">Span to use as backing memory of the dictionary values.</param>
        /// <param name="initialTableSpan">Span to use as backing memory of the table. Must be zeroed.</param>
        /// <param name="comparer">Comparer to use for the dictionary.</param>
        /// <param name="tablePowerOffset">Target size of the table relative to the number of stored elements.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickDictionary(ref Buffer<TKey> initialKeySpan, ref Buffer<TValue> initialValueSpan, ref Buffer<int> initialTableSpan, TEqualityComparer comparer, int tablePowerOffset = 2)
        {
            ValidateSpanCapacity(ref initialKeySpan, ref initialValueSpan, ref initialTableSpan);
            Keys = initialKeySpan;
            Values = initialValueSpan;
            Table = initialTableSpan;
            Count = 0;
            TableMask = Table.Length - 1;
            TablePowerOffset = tablePowerOffset;
            EqualityComparer = comparer;
            Debug.Assert(EqualityComparer != null);
            ValidateTableIsCleared(ref initialTableSpan);
        }

        /// <summary>
        /// Creates a new dictionary with a default constructed comparer.
        /// </summary>
        /// <param name="initialKeySpan">Span to use as backing memory of the dictionary keys.</param>
        /// <param name="initialValueSpan">Span to use as backing memory of the dictionary values.</param>
        /// <param name="initialTableSpan">Span to use as backing memory of the table. Must be zeroed.</param>
        /// <param name="comparer">Comparer to use for the dictionary.</param>
        /// <param name="tablePowerOffset">Target size of the table relative to the number of stored elements.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickDictionary(ref Buffer<TKey> initialKeySpan, ref Buffer<TValue> initialValueSpan, ref Buffer<int> initialTableSpan, int tablePowerOffset = 2)
            : this(ref initialKeySpan, ref initialValueSpan, ref initialTableSpan, default, tablePowerOffset)
        {
        }

        /// <summary>
        /// Creates a new dictionary.
        /// </summary>
        /// <param name="initialCapacity">Initial target size of the key and value spans. The size of the initial buffer will be at least as large as the initialCapacity.</param>
        /// <param name="tableSizePower">Target capacity relative to the initial capacity in terms of a power of 2. The size of the initial table buffer will be at least 2^tableSizePower times larger than the initial capacity.</param>
        /// <param name="comparer">Comparer to use in the dictionary.</param>
        /// <param name="pool">Pool used for spans.</param>   
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickDictionary(int initialCapacity, int tableSizePower, IUnmanagedMemoryPool pool, TEqualityComparer comparer)
        {
            pool.TakeAtLeast<TKey>(initialCapacity, out var keySpan);
            pool.TakeAtLeast<TValue>(keySpan.Length, out var valueSpan);
            pool.TakeAtLeast<int>(keySpan.Length << tableSizePower, out var tableSpan);
            //No guarantee that the table is clean; clear it.
            tableSpan.Clear(0, tableSpan.Length);
            this = new QuickDictionary<TKey, TValue, TEqualityComparer>(ref keySpan, ref valueSpan, ref tableSpan, comparer, tableSizePower);
        }

        /// <summary>
        /// Creates a new dictionary with a default constructed comparer.
        /// </summary>
        /// <param name="initialCapacity">Initial target size of the key and value spans. The size of the initial buffer will be at least as large as the initialCapacity.</param>
        /// <param name="tableSizePower">Target capacity relative to the initial capacity in terms of a power of 2. The size of the initial table buffer will be at least 2^tableSizePower times larger than the initial capacity.</param>
        /// <param name="comparer">Comparer to use in the dictionary.</param>
        /// <param name="pool">Pool used for spans.</param>   
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickDictionary(int initialCapacity, int tableSizePower, IUnmanagedMemoryPool pool)
            : this(initialCapacity, tableSizePower, pool, default)
        {
        }

        /// <summary>
        /// Creates a new dictionary with a default constructed comparer.
        /// </summary>
        /// <param name="initialCapacity">Initial target size of the key and value spans. The size of the initial buffer will be at least as large as the initialCapacity.</param>
        /// <param name="comparer">Comparer to use in the dictionary.</param>
        /// <param name="pool">Pool used for spans.</param>   
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickDictionary(int initialCapacity, IUnmanagedMemoryPool pool)
            : this(initialCapacity, 2, pool, default)
        {
        }


        /// <summary>
        /// Swaps out the dictionary's backing memory span for a new span.
        /// If the new span is smaller, the dictionary's count is truncated and the extra elements are dropped. 
        /// The old span is not cleared or returned to any pool; if it needs to be pooled or cleared, the user must handle it.
        /// </summary>
        /// <param name="newKeySpan">New span to use for keys.</param>
        /// <param name="newValueSpan">New span to use for values.</param>
        /// <param name="newTableSpan">New span to use for the table. Must be zeroed.</param>
        /// <param name="oldKeySpan">Previous span used for keys.</param>
        /// <param name="oldValueSpan">Previous span used for values.</param>
        /// <param name="oldTableSpan">Previous span used for the table.</param>
        public void Resize(ref Buffer<TKey> newKeySpan, ref Buffer<TValue> newValueSpan, ref Buffer<int> newTableSpan,
            out Buffer<TKey> oldKeySpan, out Buffer<TValue> oldValueSpan, out Buffer<int> oldTableSpan)
        {
            ValidateSpanCapacity(ref newKeySpan, ref newValueSpan, ref newTableSpan);
            ValidateTableIsCleared(ref newTableSpan);
            var oldDictionary = this;
            Keys = newKeySpan;
            Values = newValueSpan;
            Table = newTableSpan;
            Count = 0;
            TableMask = newTableSpan.Length - 1;
            var newCount = oldDictionary.Count > newKeySpan.Length ? newKeySpan.Length : oldDictionary.Count;

            //Unfortunately we can't really do a straight copy; the backing table relies on modulo operations.
            //Technically, we could copy the regular dictionary and then rely on a partial add to take care of the rest, but bleh!
            //Should really attempt to avoid resizes on sets and dictionaries whenever possible anyway. It ain't fast.
            for (int i = 0; i < newCount; ++i)
            {
                //We assume that ref adds will get inlined reasonably here. That's not actually guaranteed, but we'll bite the bullet.
                //(You could technically branch on the Unsafe.SizeOf<T>, which should result in a compile time specialized zero overhead implementation... but meh!)
                AddUnsafely(ref oldDictionary.Keys[i], oldDictionary.Values[i]);
            }
            oldKeySpan = oldDictionary.Keys;
            oldValueSpan = oldDictionary.Values;
            oldTableSpan = oldDictionary.Table;

        }

        /// <summary>
        /// Resizes the dictionary's backing array for the given size.
        /// If the new span is smaller, the dictionary's count is truncated and the extra elements are dropped. 
        /// </summary>
        /// <param name="newSize">Minimum size of the new object memory block. Actual size may be larger.</param>
        /// <param name="pool">Pool used for spans.</param>   
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize(int newSize, IUnmanagedMemoryPool pool)
        {
            var targetKeyCapacity = pool.GetCapacityForCount<TKey>(newSize);
            if (targetKeyCapacity != Keys.Length)
            {
                pool.TakeAtLeast<TKey>(newSize, out var newKeySpan);
                pool.TakeAtLeast<TValue>(newKeySpan.Length, out var newValueSpan);
                pool.TakeAtLeast<int>(newKeySpan.Length << TablePowerOffset, out var newTableSpan);
                //There is no guarantee that the table retrieved from the pool is clean. Clear it!
                newTableSpan.Clear(0, newTableSpan.Length);
                var oldDictionary = this;
                Resize(ref newKeySpan, ref newValueSpan, ref newTableSpan, out var oldKeySpan, out var oldValueSpan, out var oldTableSpan);
                oldDictionary.Dispose(pool);
            }
        }

        /// <summary>
        /// Returns the resources associated with the dictionary to pools.
        /// </summary>
        /// <param name="keyPool">Pool used for key spans.</param>   
        /// <param name="valuePool">Pool used for value spans.</param>   
        /// <param name="tablePool">Pool used for table spans.</param>
        /// <typeparam name="TKeyPool">Type of the pool used for key spans.</typeparam>
        /// <typeparam name="TValuePool">Type of the pool used for value spans.</typeparam>
        /// <typeparam name="TTablePool">Type of the pool used for table spans.</typeparam>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose(IUnmanagedMemoryPool pool)
        {
            pool.Return(ref Keys);
            pool.Return(ref Values);
            pool.Return(ref Table);
        }

        /// <summary>
        /// Ensures that the dictionary has enough room to hold the specified number of elements.
        /// </summary>     
        /// <param name="pool">Pool used for spans.</param>   
        /// <param name="count">Number of elements to hold.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnsureCapacity(int count, IUnmanagedMemoryPool pool)
        {
            if (count > Keys.Length)
            {
                Resize(count, pool);
            }
        }

        /// <summary>
        /// Shrinks the internal buffers to the smallest acceptable size and releases the old buffers to the pools.
        /// </summary>
        /// <param name="pool">Pool used for spans.</param>
        public void Compact(IUnmanagedMemoryPool pool)
        {
            Validate();
            var targetKeyCapacity = pool.GetCapacityForCount<TKey>(Count);
            if (targetKeyCapacity < Keys.Length)
                Resize(Count, pool);
        }



        /// <summary>
        /// Gets the index of the element in the table.
        /// </summary>
        /// <param name="element">Element to look up.</param>
        /// <param name="tableIndex">Index of the element in the redirect table, or if it is not present, the index of where it would be added.</param>
        /// <param name="elementIndex">The index of the element in the element arrays, if it exists; -1 otherwise.</param>
        /// <returns>True if the element is present in the dictionary, false if it is not.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool GetTableIndices(ref TKey element, out int tableIndex, out int elementIndex)
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
                if (EqualityComparer.Equals(ref Keys[--elementIndex], ref element))
                {
                    return true;
                }
                tableIndex = (tableIndex + 1) & TableMask;
            }
            elementIndex = -1;
            return false;
        }

        /// <summary>
        /// Gets the index of the key in the dictionary values list if it exists.
        /// </summary>
        /// <param name="key">Key to get the index of.</param>
        /// <returns>The index of the key if the key exists in the dictionary, -1 otherwise.</returns>
        public int IndexOf(TKey key)
        {
            Validate();
            GetTableIndices(ref key, out int tableIndex, out int objectIndex);
            return objectIndex;
        }


        /// <summary>
        /// Gets the index of the key in the dictionary values list if it exists.
        /// </summary>
        /// <param name="key">Key to get the index of.</param>
        /// <returns>The index of the key if the key exists in the dictionary, -1 otherwise.</returns>
        public int IndexOf(ref TKey key)
        {
            Validate();
            GetTableIndices(ref key, out int tableIndex, out int objectIndex);
            return objectIndex;
        }

        /// <summary>
        /// Checks if a given key already belongs to the dictionary.
        /// </summary>
        /// <param name="key">Key to test for.</param>
        /// <returns>True if the key already belongs to the dictionary, false otherwise.</returns>
        public bool ContainsKey(TKey key)
        {
            Validate();
            return GetTableIndices(ref key, out int tableIndex, out int objectIndex);
        }

        /// <summary>
        /// Checks if a given key already belongs to the dictionary.
        /// </summary>
        /// <param name="key">Key to test for.</param>
        /// <returns>True if the key already belongs to the dictionary, false otherwise.</returns>
        public bool ContainsKey(ref TKey key)
        {
            Validate();
            return GetTableIndices(ref key, out int tableIndex, out int objectIndex);
        }

        /// <summary>
        /// Tries to retrieve the value associated with a key if it exists.
        /// </summary>
        /// <param name="key">Key to look up.</param>
        /// <param name="value">Value associated with the specified key.</param>
        /// <returns>True if a value was found, false otherwise.</returns>
        public bool TryGetValue(TKey key, out TValue value)
        {
            Validate();
            if (GetTableIndices(ref key, out int tableIndex, out int elementIndex))
            {
                value = Values[elementIndex];
                return true;
            }
            value = default;
            return false;
        }

        /// <summary>
        /// Tries to retrieve the value associated with a key if it exists.
        /// </summary>
        /// <param name="key">Key to look up.</param>
        /// <param name="value">Value associated with the specified key.</param>
        /// <returns>True if a value was found, false otherwise.</returns>
        public bool TryGetValue(ref TKey key, out TValue value)
        {
            Validate();
            if (GetTableIndices(ref key, out int tableIndex, out int elementIndex))
            {
                value = Values[elementIndex];
                return true;
            }
            value = default;
            return false;
        }

        /// <summary>
        /// Attempts to find the index of the given key. If it is present, outputs the index and returns true. If it is not present, it allocates a slot for it, outputs the index of that new slot, and returns false.
        /// If a new slot is allocated, the value stored in the slot is undefined.
        /// </summary>
        /// <param name="key">Key to find or allocate a slot for.</param>
        /// <param name="slotIndex">Index of the found or allocated slot.</param>
        /// <returns>True if the key was already present in the dictionary, false otherwise.</returns>
        public bool FindOrAllocateSlotUnsafely(ref TKey key, out int slotIndex)
        {
            Validate();
            ValidateUnsafeAdd();
            if (GetTableIndices(ref key, out int tableIndex, out slotIndex))
                return true;
            //It wasn't in the dictionary. Allocate it!
            slotIndex = Count++;
            Keys[slotIndex] = key;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = Count;
            return false;
        }

        /// <summary>
        /// Attempts to find the index of the given key. If it is present, outputs the index and returns true. If it is not present, it allocates a slot for it, outputs the index of that new slot, and returns false.
        /// If a new slot is allocated, the value stored in the slot is undefined.
        /// </summary>
        /// <param name="key">Key to find or allocate a slot for.</param>
        /// <param name="slotIndex">Index of the found or allocated slot.</param>
        /// <returns>True if the key was already present in the dictionary, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool FindOrAllocateSlotUnsafely(TKey key, out int slotIndex)
        {
            return FindOrAllocateSlotUnsafely(ref key, out slotIndex);
        }

        /// <summary>
        /// Attempts to find the index of the given key. If it is present, outputs the index and returns true. If it is not present, it allocates a slot for it, outputs the index of that new slot, and returns false.
        /// If a new slot is allocated, the value stored in the slot is undefined.
        /// </summary>
        /// <param name="key">Key to find or allocate a slot for.</param>
        /// <param name="pool">Pool used to resize the container if necessary to allocate.</param>
        /// <param name="slotIndex">Index of the found or allocated slot.</param>
        /// <returns>True if the key was already present in the dictionary, false otherwise.</returns>
        public bool FindOrAllocateSlot(ref TKey key, BufferPool pool, out int slotIndex)
        {
            Validate();
            if (Count == Keys.Length)
            {
                //There's no room left; resize.
                Resize(Count * 2, pool);
                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }
            if (GetTableIndices(ref key, out int tableIndex, out slotIndex))
                return true;
            //It wasn't in the dictionary. Allocate it!
            slotIndex = Count++;
            Keys[slotIndex] = key;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = Count;
            return false;
        }

        /// <summary>
        /// Attempts to find the index of the given key. If it is present, outputs the index and returns true. If it is not present, it allocates a slot for it, outputs the index of that new slot, and returns false.
        /// If a new slot is allocated, the value stored in the slot is undefined.
        /// </summary>
        /// <param name="key">Key to find or allocate a slot for.</param>
        /// <param name="pool">Pool used to resize the container if necessary to allocate.</param>
        /// <param name="slotIndex">Index of the found or allocated slot.</param>
        /// <returns>True if the key was already present in the dictionary, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool FindOrAllocateSlot(TKey key, BufferPool pool, out int slotIndex)
        {
            return FindOrAllocateSlot(ref key, pool, out slotIndex);
        }

        /// <summary>
        /// Adds a pair to the dictionary. If a version of the key (same hash code, 'equal' by comparer) is already present,
        /// the existing pair is replaced by the given version.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <returns>True if the pair was added to the dictionary, false if the key was already present and its pair was replaced.</returns>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)] //TODO: Test performance of full chain inline.
        public bool AddAndReplaceUnsafely(ref TKey key, in TValue value)
        {
            Validate();
            ValidateUnsafeAdd();

            if (GetTableIndices(ref key, out int tableIndex, out int elementIndex))
            {
                //Already present!
                Keys[elementIndex] = key;
                Values[elementIndex] = value;
                return false;
            }

            //It wasn't in the dictionary. Add it!
            Keys[Count] = key;
            Values[Count] = value;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = ++Count;
            return true;
        }

        /// <summary>
        /// Adds a pair to the dictionary. If a version of the key (same hash code, 'equal' by comparer) is already present,
        /// the existing pair is replaced by the given version.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <returns>True if the pair was added to the dictionary, false if the key was already present and its pair was replaced.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddAndReplaceUnsafely(TKey key, in TValue value)
        {
            return AddAndReplaceUnsafely(ref key, value);
        }

        /// <summary>
        /// Adds a pair to the dictionary if it is not already present.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <returns>True if the pair was added to the dictionary, false if the key was already present.</returns>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)] //TODO: Test performance of full chain inline.
        public bool AddUnsafely(ref TKey key, in TValue value)
        {
            Validate();
            ValidateUnsafeAdd();
            if (GetTableIndices(ref key, out int tableIndex, out int elementIndex))
            {
                //Already present!
                return false;
            }

            //It wasn't in the dictionary. Add it!
            Keys[Count] = key;
            Values[Count] = value;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = ++Count;
            return true;
        }

        /// <summary>
        /// Adds a pair to the dictionary if it is not already present.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <returns>True if the pair was added to the dictionary, false if the key was already present.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddUnsafely(TKey key, in TValue value)
        {
            return AddUnsafely(ref key, value);
        }

        /// <summary>
        /// Adds a pair to the dictionary. If a version of the key (same hash code, 'equal' by comparer) is already present,
        /// the existing pair is replaced by the given version.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        /// <returns>True if the pair was added to the dictionary, false if the key was already present and its pair was replaced.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddAndReplace(ref TKey key, in TValue value, IUnmanagedMemoryPool pool)
        {
            if (Count == Keys.Length)
            {
                //There's no room left; resize.
                Resize(Count * 2, pool);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }
            return AddAndReplaceUnsafely(ref key, value);
        }

        /// <summary>
        /// Adds a pair to the dictionary. If a version of the key (same hash code, 'equal' by comparer) is already present,
        /// the existing pair is replaced by the given version.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        /// <returns>True if the pair was added to the dictionary, false if the key was already present and its pair was replaced.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AddAndReplace(TKey key, in TValue value, IUnmanagedMemoryPool pool)
        {
            return AddAndReplace(ref key, value, pool);
        }

        /// <summary>
        /// Adds a pair to the dictionary if it is not already present.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <param name="pool">Pool used for spans.</param>   
        /// <typeparam name="TPool">Type of the pool used for spans.</typeparam>
        /// <returns>True if the pair was added to the dictionary, false if the key was already present.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Add(ref TKey key, in TValue value, IUnmanagedMemoryPool pool)
        {
            Validate();

            if (Count == Keys.Length)
            {
                //There's no room left; resize.
                Resize(Count * 2, pool);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }
            return AddUnsafely(ref key, value);
        }

        /// <summary>
        /// Adds a pair to the dictionary if it is not already present.
        /// </summary>
        /// <param name="key">Key of the pair to add.</param>
        /// <param name="value">Value of the pair to add.</param>
        /// <param name="pool">Pool to pull resources from and to return resources to.</param>   
        /// <typeparam name="TPool">Type of the pool to use.</typeparam>
        /// <returns>True if the pair was added to the dictionary, false if the key was already present.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Add(TKey key, in TValue value, IUnmanagedMemoryPool pool)
        {
            return Add(ref key, value, pool);
        }

        //Note: the reason this is named "FastRemove" instead of just "Remove" despite it being the only remove present is that
        //there may later exist an order preserving "Remove". That would be a very sneaky breaking change.

        /// <summary>
        /// Removes an element from the dictionary according to its table and element index. Can only be used if the table and element index are valid.
        /// </summary>
        /// <param name="tableIndex">Index of the table entry associated with the existing element to remove.</param>
        /// <param name="elementIndex">Index of the existing element to remove in the contiguous key/value arrays.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void FastRemove(int tableIndex, int elementIndex)
        {
            Debug.Assert(GetTableIndices(ref Keys[elementIndex], out var debugTableIndex, out var debugElementIndex) && debugTableIndex == tableIndex && debugElementIndex == elementIndex,
                "The table index and element index used to directly remove must match an actual key.");
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
                int desiredIndex = HashHelper.Rehash(EqualityComparer.Hash(ref Keys[moveCandidateIndex])) & TableMask;

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
                Keys[elementIndex] = Keys[Count];
                Values[elementIndex] = Values[Count];
                //Locate the swapped object in the table and update its index.
                GetTableIndices(ref Keys[elementIndex], out tableIndex, out int oldObjectIndex);
                Table[tableIndex] = elementIndex + 1; //Remember the encoding! all indices offset by 1.
            }
            //Clear the final slot in the elements set.
            Keys[Count] = default;
            Values[Count] = default;
        }

        /// <summary>
        /// Removes a pair associated with a key from the dictionary if belongs to the dictionary.
        /// Does not preserve the order of elements in the dictionary.
        /// </summary>
        /// <param name="key">Key of the pair to remove.</param>
        /// <returns>True if the key was found and removed, false otherwise.</returns>
        public bool FastRemove(ref TKey key)
        {
            Validate();
            //Find it.
            if (GetTableIndices(ref key, out int tableIndex, out int elementIndex))
            {
                //We found the object!
                FastRemove(tableIndex, elementIndex);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes a pair associated with a key from the dictionary if belongs to the dictionary.
        /// Does not preserve the order of elements in the dictionary.
        /// </summary>
        /// <param name="key">Key of the pair to remove.</param>
        /// <returns>True if the key was found and removed, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool FastRemove(TKey key)
        {
            return FastRemove(ref key);
        }

        /// <summary>
        /// Removes all elements from the dictionary.
        /// </summary>
        public void Clear()
        {
            //While it may be appealing to remove individual elements from the dictionary when sparse,
            //using a brute force clear over the entire table is almost always faster. And it's a lot simpler!
            Table.Clear(0, Table.Length);
            Keys.Clear(0, Count);
            Values.Clear(0, Count);
            Count = 0;
        }

        /// <summary>
        /// Removes all elements from the dictionary without modifying the contents of the keys or values arrays.
        /// </summary>
        public void FastClear()
        {
            Table.Clear(0, Table.Length);
            Count = 0;
        }

        /// <summary>
        /// Gets the keys and values wrapped in spans.
        /// </summary>
        /// <param name="keys">Keys from the dictionary.</param>
        /// <param name="values">Values from the dictionary.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AsSpans(out Span<TKey> keys, out Span<TValue> values)
        {
            keys = new Span<TKey>(Keys.Memory, Count);
            values = new Span<TValue>(Values.Memory, Count);
        }

        public Enumerator GetEnumerator()
        {
            Validate();
            return new Enumerator(ref Keys, ref Values, Count);
        }

        public struct Enumerator : IEnumerator<KeyValuePair<TKey, TValue>>
        {
            private readonly Buffer<TKey> keys;
            private readonly Buffer<TValue> values;
            private readonly int count;
            private int index;

            public Enumerator(ref Buffer<TKey> keys, ref Buffer<TValue> values, int count)
            {
                this.keys = keys;
                this.values = values;
                this.count = count;

                index = -1;
            }

            public KeyValuePair<TKey, TValue> Current
            {
                get { return new KeyValuePair<TKey, TValue>(keys[index], values[index]); }
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
        static void ValidateSpanCapacity(ref Buffer<TKey> keySpan, ref Buffer<TValue> valueSpan, ref Buffer<int> tableSpan)
        {
            Debug.Assert(tableSpan.Length >= keySpan.Length, "The table span must be at least as large as the key span.");
            Debug.Assert(valueSpan.Length >= keySpan.Length, "The value span must be at least as large as the key span.");
            Debug.Assert((tableSpan.Length & (tableSpan.Length - 1)) == 0, "Dictionaries depend upon power of 2 backing table span sizes for efficient modulo operations.");
        }

        [Conditional("DEBUG")]
        private void Validate()
        {
            Debug.Assert(Keys.Length != 0 && Values.Length != 0 && Table.Length != 0, "The QuickDictionary must have its internal buffers and pools available; default-constructed or disposed QuickDictionary should not be used.");
            ValidateSpanCapacity(ref Keys, ref Values, ref Table);
        }


        [Conditional("DEBUG")]
        void ValidateUnsafeAdd()
        {
            Debug.Assert(Count < Keys.Length, "Unsafe adders can only be used if the capacity is guaranteed to hold the new size.");
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
