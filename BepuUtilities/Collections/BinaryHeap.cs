using BepuUtilities.Memory;
using System.Diagnostics;

namespace BepuUtilities.Collections;

/// <summary>
/// A max-heap priority queue that stores values with associated priorities, where higher priorities are dequeued first.
/// </summary>
/// <typeparam name="T">The type of value stored in the heap. Must be unmanaged.</typeparam>
public struct BinaryHeap<T> where T: unmanaged
{
    /// <summary>
    /// Represents an entry in the heap containing a value and its associated priority.
    /// </summary>
    public struct HeapEntry
    {
        /// <summary>
        /// The value stored in this entry.
        /// </summary>
        public T Value;
        /// <summary>
        /// The priority of this entry. Higher values are dequeued first.
        /// </summary>
        public float Priority;
    }

    /// <summary>
    /// The backing buffer containing all heap entries.
    /// </summary>
    public Buffer<HeapEntry> Entries;
    /// <summary>
    /// The number of entries currently in the heap.
    /// </summary>
    public int Count;

    /// <summary>
    /// Creates a new binary heap using the provided buffer for storage.
    /// </summary>
    /// <param name="entries">The buffer to use for storing heap entries.</param>
    public BinaryHeap(Buffer<HeapEntry> entries)
    {
        Entries = entries;
        Count = 0;
    }

    /// <summary>
    /// Creates a new binary heap with the specified capacity, allocating storage from the given pool.
    /// </summary>
    /// <param name="capacity">The maximum number of entries the heap can hold.</param>
    /// <param name="pool">The buffer pool to allocate storage from.</param>
    public BinaryHeap(int capacity, BufferPool pool) : this(new Buffer<HeapEntry>(capacity, pool)) { }

    /// <summary>
    /// Returns the heap's buffer to the specified pool and clears the reference.
    /// </summary>
    /// <param name="pool">The buffer pool to return the storage to.</param>
    public void Dispose(BufferPool pool)
    {
        pool.Return(ref Entries);
    }

    /// <summary>
    /// Inserts a value into the heap with the specified priority.
    /// </summary>
    /// <param name="valueToInsert">The value to insert.</param>
    /// <param name="priority">The priority of the value. Higher values are dequeued first.</param>
    public void Insert(T valueToInsert, float priority)
    {
        int index = Count;
        ++Count;
        Debug.Assert(Count <= Entries.Length, "New entry does not fit in the heap; this function does not perform resizing.");
        //Sift up.
        while (index > 0)
        {
            var parentIndex = (index - 1) >> 1;
            var parent = Entries[parentIndex];
            if (parent.Priority < priority)
            {
                //Pull the parent down.
                Entries[index] = parent;
                index = parentIndex;
            }
            else
            {
                //Found the insertion spot.
                break;
            }
        }
        ref var entry = ref Entries[index];
        entry.Value = valueToInsert;
        entry.Priority = priority;
    }

    /// <summary>
    /// Removes and returns the entry with the highest priority from the heap.
    /// </summary>
    /// <returns>The heap entry with the highest priority.</returns>
    public HeapEntry Pop()
    {
        var entry = Entries[0];
        --Count;
        var priority = Entries[Count].Priority;

        //Pull the elements up to fill in the gap.
        int index = 0;
        while (true)
        {
            var childIndexA = (index << 1) + 1;
            var childIndexB = (index << 1) + 2;
            if (childIndexB < Count)
            {
                //Both children are available.
                //Try swapping with the largest one.
                var childA = Entries[childIndexA];
                var childB = Entries[childIndexB];
                if (childA.Priority > childB.Priority)
                {
                    if (priority > childA.Priority)
                    {
                        break;
                    }
                    Entries[index] = Entries[childIndexA];
                    index = childIndexA;
                }
                else
                {
                    if (priority > childB.Priority)
                    {
                        break;
                    }
                    Entries[index] = Entries[childIndexB];
                    index = childIndexB;
                }
            }
            else if (childIndexA < Count)
            {
                //Only one child was available.
                ref var childA = ref Entries[childIndexA];
                if (priority > childA.Priority)
                {
                    break;
                }
                Entries[index] = Entries[childIndexA];
                index = childIndexA;
            }
            else
            {
                //The children were beyond the heap.
                break;
            }
        }
        //Move the last entry into position.
        Entries[index] = Entries[Count];
        return entry;
    }
}
