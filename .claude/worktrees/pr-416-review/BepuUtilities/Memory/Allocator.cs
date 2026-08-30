using BepuUtilities.Collections;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Represents a chunk of abstract memory supporting allocations and deallocations.
    /// Never moves any memory.
    /// </summary>
    /// <remarks>Uses an extremely simple ring buffer that makes no attempt to skip groups of allocations. Not particularly efficient.</remarks>
    public class Allocator : IDisposable
    {
        BufferPool pool;
        long capacity;
        /// <summary>
        /// Gets or sets the capacity of the allocator.
        /// </summary>
        public long Capacity
        {
            get { return capacity; }
            set
            {
                if (value < 0)
                    throw new ArgumentException("Capacity must be positive.");
                if (value < capacity)
                {
                    for (int i = 0; i < allocations.Count; ++i)
                    {
                        if (value < allocations.Values[i].End)
                            throw new ArgumentException("Can't reduce capacity below any existing allocation endpoint.");
                    }
                }
                capacity = value;
            }
        }
        /// <summary>
        /// Index in allocations that we should start at during the next allocation attempt.
        /// </summary>
        int searchStartIndex;

        public struct Allocation
        {
            public long Start, End;
            public ulong Previous;
            public ulong Next;
        }

        private QuickDictionary<ulong, Allocation, PrimitiveComparer<ulong>> allocations;

        /// <summary>
        /// Creates a new allocator.
        /// </summary>
        /// <param name="capacity">Size of the memory handled by the allocator in elements.</param>
        /// <param name="initialAllocationCapacity">Estimated number of allocations to allocate room for in the internal structures.</param>
        /// <param name="pool">Pool to pull internal resources from.</param>
        public Allocator(long capacity, BufferPool pool, int initialAllocationCapacity = 128)
        {
            this.pool = pool;
            this.Capacity = capacity;
            allocations = new QuickDictionary<ulong, Allocation, PrimitiveComparer<ulong>>(initialAllocationCapacity, 2, pool);
        }

        /// <summary>
        /// Checks if the id is currently allocated.
        /// </summary>
        /// <param name="id">Id to check for.</param>
        /// <returns>True if the id is present in the allocations set, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(ulong id)
        {
            return allocations.ContainsKey(id);
        }

        /// <summary>
        /// Gets the allocation region associated with the given allocation id if it is present.
        /// </summary>
        /// <param name="allocationId">Allocation id to look up the allocation for.</param>
        /// <param name="allocation">Allocation associated with the id, if present.</param>
        /// <returns>True if the allocationId was present in the allocator, false otherwise.</returns>
        public bool TryGetAllocationRegion(ulong allocationId, out Allocation allocation)
        {
            return allocations.TryGetValue(allocationId, out allocation);
        }

        /// <summary>
        /// Checks if a block of memory can fit into the current state of the allocator.
        /// </summary>
        /// <param name="size">Size of the memory to test.</param>
        /// <param name="ignoredIds">Ids of allocations to treat as nonexistent for the purposes of the test.</param>
        /// <returns>True if the size could fit, false if out of memory or if memory was too fragmented to find a spot.</returns>
        public bool CanFit<TPredicate>(long size, ref TPredicate ignoredIds) where TPredicate : struct, IPredicate<ulong>
        {
            if (allocations.Count == 0)
            {
                return size <= Capacity;
            }
            int allocationIndex = searchStartIndex;
            var initialId = allocations.Keys[allocationIndex];
            while (true)
            {
                var allocation = allocations.Values[allocationIndex];
                int nextAllocationIndex;
                Allocation nextAllocation;

                //Skip any subsequent allocations that are ignored.
                ulong nextAllocationId = allocation.Next;
                do
                {
                    nextAllocationIndex = allocations.IndexOf(nextAllocationId);
                    nextAllocation = allocations.Values[nextAllocationIndex];
                    nextAllocationId = nextAllocation.Next;
                } while (ignoredIds.Matches(ref nextAllocationId));

                if (nextAllocation.Start < allocation.End)
                {
                    //Wrapped around, so the gap goes from here to the end of the memory block, and from the beginning of the memory block to the next allocation.
                    //But we need contiguous space so the two areas have to be tested independently.
                    if (Capacity - allocation.End >= size)
                    {
                        return true;
                    }
                    else
                    {
                        if (nextAllocation.Start >= size)
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    //The next allocation is in order.
                    if (nextAllocation.Start - allocation.End >= size)
                    {
                        return true;
                    }

                }
                //If we get here, no open space was found.
                //Move on to the next spot.
                allocationIndex = nextAllocationIndex;

                //Have we already wrapped around?
                if (allocations.Keys[allocationIndex] == initialId)
                {
                    //Wrapped around without finding any space.
                    return false;
                }
            }
        }

        struct IgnoreNothing : IPredicate<ulong>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Matches(ref ulong item)
            {
                return false;
            }
        }
        /// <summary>
        /// Checks if a block of memory can fit into the current state of the allocator.
        /// </summary>
        /// <param name="size">Size of the memory to test.</param>
        /// <returns>True if the size could fit, false if out of memory or if memory was too fragmented to find a spot.</returns>
        public bool CanFit(long size)
        {
            var predicate = default(IgnoreNothing);
            return CanFit(size, ref predicate);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddAllocation(ulong id, long start, long end,
            ref Allocation allocation, ref Allocation nextAllocation)
        {
            var newAllocation = new Allocation
            {
                Next = allocation.Next,
                Previous = nextAllocation.Previous,
                Start = start,
                End = end
            };
            //Note that the pointer modifications come BEFORE the new addition.
            //This avoids a potential pointer invalidation caused by a resize in the allocations dictionary.
            allocation.Next = id;
            nextAllocation.Previous = id;
            //About to add a new allocation. We had space here this time, so there's a high chance we'll have some more space next time. Point the search to this index.
            searchStartIndex = allocations.Count;
            allocations.Add(id, newAllocation, pool);
        }
        /// <summary>
        /// Attempts to allocate a range of memory.
        /// </summary>
        /// <param name="id">Unique id of the memory to allocate.</param>
        /// <param name="size">Size of the memory to allocate.</param>
        /// <param name="outputStart">Starting index of the allocated memory, if successful.</param>
        /// <returns>True if the allocation succeeded, false if out of memory or if memory was too fragmented to find a spot.</returns>
        public bool Allocate(ulong id, long size, out long outputStart)
        {
            Debug.Assert(!allocations.ContainsKey(id), "Id must not already be present.");
            if (allocations.Count == 0)
            {
                //If it's the first allocation, then the next and previous pointers should circle around.
                if (size <= Capacity)
                {
                    outputStart = 0;
                    allocations.Add(id, new Allocation { Start = 0, End = size, Next = id, Previous = id }, pool);
                    searchStartIndex = 0;
                    return true;
                }
                outputStart = 0;
                return false;
            }
            Debug.Assert(searchStartIndex >= 0 && searchStartIndex < allocations.Count, "Search start index must be within the allocation set!");
            int allocationIndex = searchStartIndex;
            var initialId = allocations.Keys[allocationIndex];
            while (true)
            {
                var allocation = allocations.Values[allocationIndex];
                int nextAllocationIndex = allocations.IndexOf(allocation.Next);
                var nextAllocation = allocations.Values[nextAllocationIndex];
                if (nextAllocation.Start < allocation.End)
                {
                    //Wrapped around, so the gap goes from here to the end of the memory block, and from the beginning of the memory block to the next allocation.
                    //But we need contiguous space so the two areas have to be tested independently.
                    if (Capacity - allocation.End >= size)
                    {
                        AddAllocation(id, outputStart = allocation.End, allocation.End + size, ref allocations.Values[allocationIndex], ref allocations.Values[nextAllocationIndex]);
                        return true;
                    }
                    else
                    {
                        if (nextAllocation.Start >= size)
                        {
                            AddAllocation(id, outputStart = 0, size, ref allocations.Values[allocationIndex], ref allocations.Values[nextAllocationIndex]);
                            return true;
                        }
                    }
                }
                else
                {
                    //The next allocation is in order.
                    if (nextAllocation.Start - allocation.End >= size)
                    {
                        AddAllocation(id, outputStart = allocation.End, allocation.End + size, ref allocations.Values[allocationIndex], ref allocations.Values[nextAllocationIndex]);
                        return true;
                    }

                }
                //If we get here, no open space was found.
                //Move on to the next spot.
                allocationIndex = nextAllocationIndex;

                //Have we already wrapped around?
                if (allocations.Keys[allocationIndex] == initialId)
                {
                    //Wrapped around without finding any space.
                    outputStart = 0;
                    return false;
                }
            }
        }

        /// <summary>
        /// Removes the memory associated with the id from the pool.
        /// </summary>
        /// <param name="id">Id to remove.</param>
        /// <returns>True of the id was found, false otherwise.</returns>
        public bool Deallocate(ulong id)
        {
            Allocation allocation;
            if (allocations.TryGetValue(id, out allocation))
            {
                if (allocation.Previous != id)
                {
                    var previousIndex = allocations.IndexOf(allocation.Previous);
                    Debug.Assert(allocations.Values[previousIndex].Next == id, "Previous and current must agree about their relationship.");
                    //Make the previous allocation point to the next allocation to get rid of the current allocation.
                    allocations.Values[previousIndex].Next = allocation.Next;

                    var nextIndex = allocations.IndexOf(allocation.Next);
                    Debug.Assert(allocations.Values[nextIndex].Previous == id, "Next and current must agree about their relationship.");
                    //Make the next allocation point to the previous allocation to get rid of the current allocation.
                    allocations.Values[nextIndex].Previous = allocation.Previous;

                }
                else
                {
                    Debug.Assert(allocation.Next == id, "The next index should be itself too, if previous was itself.");
                    Debug.Assert(allocations.Count == 1, "The only time where the previous allocation is itself should be when there is only a single allocation.");
                    //If there are no more allocations, then the searchStartIndex is irrelevant. It won't be used on the next allocation.
                }
                allocations.FastRemove(id);
                //By removing this id, a promising place to look for an allocation next time is the position next to the previous allocation!
                //Note that we request the previous index here again, because the previousIndex might change during the call to FastRemove.
                //If there are no elements remaining, the index will be invalid, and that's fine- if there are no elements then the next allocation will not use the searchStartIndex.
                searchStartIndex = allocations.IndexOf(allocation.Previous);
                Debug.Assert(allocations.Count == 0 || (searchStartIndex >= 0 && searchStartIndex < allocations.Count), "Search start index must be within the allocation set!");
                return true;
            }
            return false;
        }


        /// <summary>
        /// Gets the size of the largest contiguous area and the total free space in the allocator.
        /// Not very efficient; runs in linear time for the number of allocations.
        /// </summary>
        /// <param name="largestContiguous">Largest contiguous region in the allocator. The allocator can hold an allocation up to this size.</param>
        /// <param name="totalFreeSpace">Total free space in the allocator.</param>
        public void GetLargestContiguousSize(out long largestContiguous, out long totalFreeSpace)
        {
            if (allocations.Count == 0)
            {
                totalFreeSpace = Capacity;
                largestContiguous = Capacity;
                return;
            }
            largestContiguous = 0;
            totalFreeSpace = 0;
            for (int i = 0; i < allocations.Count; ++i)
            {
                Allocation nextAllocation;
                allocations.TryGetValue(allocations.Values[i].Next, out nextAllocation);
                var toNext = nextAllocation.Start - allocations.Values[i].End;
                if (toNext < 0)
                {
                    //The next allocation requires a wrap, so the actual contiguous area is only from our end to the end of the pool,
                    //and then a second region from 0 to the next allocation.
                    var adjacent = Capacity - allocations.Values[i].End;
                    var wrapped = nextAllocation.Start;
                    if (largestContiguous < adjacent)
                        largestContiguous = adjacent;
                    if (largestContiguous < wrapped)
                        largestContiguous = wrapped;
                    totalFreeSpace += adjacent + wrapped;
                }
                else
                {
                    if (largestContiguous < toNext)
                        largestContiguous = toNext;
                    totalFreeSpace += toNext;
                }
            }
        }

        /// <summary>
        /// Finds the first allocation with empty space before it and pulls it forward to close the gap. Assumes the ability to perform synchronous reallocation.
        /// </summary>
        /// <param name="id">Id of the allocation to be moved, if any.</param>
        /// <param name="size">Size of the moved allocation.</param>
        /// <param name="oldStart">Old starting location of the allocation.</param>
        /// <param name="newStart">New starting location of the allocation.</param>
        /// <returns>True if a compaction was performed, false otherwise.</returns>
        public bool IncrementalCompact(out ulong id, out long size, out long oldStart, out long newStart)
        {
            //Find the allocation nearest to the zero index. Identify it by checking for the previous allocation requiring a wraparound.
            //Start at the beginning of the list since it's marginally more likely to be there than at the end of the list where new allocations get appended.
            for (int i = 0; i < allocations.Count; ++i)
            {
                Allocation previousAllocation;
                allocations.TryGetValue(allocations.Values[i].Previous, out previousAllocation);
                if (previousAllocation.End > allocations.Values[i].Start)
                {
                    //Found the beginning of the list! This index is the first index.
                    //Now, scan forward through the allocation links looking for the first gap.
                    var index = i;
                    var previousEnd = 0L;
                    //Note that we stop before wrapping.
                    for (int iterationIndex = 0; iterationIndex < allocations.Count; ++iterationIndex)
                    {
                        searchStartIndex = index; //If the traversal ends, we want to have this index cached so that the next allocation will start at the end of the contiguous block.
                        Debug.Assert(searchStartIndex >= 0 && searchStartIndex < allocations.Count, "Search start index must be within the allocation set!");
                        if (allocations.Values[index].Start > previousEnd)
                        {
                            //Found a gap.
                            id = allocations.Keys[index];
                            size = allocations.Values[index].End - allocations.Values[index].Start;
                            oldStart = allocations.Values[index].Start;
                            newStart = previousEnd;
                            //Actually perform the move.
                            allocations.Values[index].Start = newStart;
                            allocations.Values[index].End = newStart + size;
                            return true;
                        }
                        //Haven't found a gap yet. Move to the next.
                        previousEnd = allocations.Values[index].End;
                        index = allocations.IndexOf(allocations.Values[index].Next);
                    }
                    break;
                }
            }
            id = 0;
            size = 0;
            oldStart = 0;
            newStart = 0;
            return false;

            //Note: a slightly fancier allocator could 1) track the start and 2) coalesce allocations such that this entire process would become O(1). 
            //Something to consider if this allocator ever bottlenecks.
        }

        /// <summary>
        /// Attempts to resize a given allocation to a new size. If the new size is smaller, the start index remains unchanged.
        /// </summary>
        /// <param name="id">Id of the allocation to resize.</param>
        /// <param name="size">New desired size of the allocation.</param>
        /// <param name="oldStart">Old start location of the allocation.</param>
        /// <param name="newStart">New start location of the allocation.</param>
        /// <returns>True if the resize was successful. False if there was insufficient room for the larger allocation.</returns>
        public bool Resize(ulong id, long size, out long oldStart, out long newStart)
        {
            var allocationIndex = allocations.IndexOf(id);
            Debug.Assert(allocationIndex >= 0, "The allocation must be present inside the allocator to resize it!");
            //Ref locals would be so nice.
            var allocation = allocations.Values[allocationIndex];
            oldStart = allocation.Start;
            var currentSize = allocation.End - allocation.Start;
            Debug.Assert(size != currentSize, "Why are you calling resize if the new size is the same as the old one?");

            if (size < currentSize)
            {
                //We can resize without worrying about redoing an allocation.
                //Note that we always shrink the interval by moving the end closer to the start, even though that might
                //increase fragmentation. However, by only moving the endpoint, we eliminate the need to move the interval.
                //Externally, this means resource uploads are avoided.
                //Conceptually, the incremental compaction algorithm already induces a bias toward 0. In other words,
                //temporarily introducing fragmentation doesn't matter because the incremental compaction algorithm ends up 
                //doing the same amount of work either way. So we might as well avoid doing double-moves.
                allocations.Values[allocationIndex].End = allocation.Start + size;
                newStart = allocation.Start;
                return true;
            }
            //The size is increasing (unless the above assertion was hit!).
            //This requires a reallocation.
            var success = Deallocate(id);
            Debug.Assert(success, "Sanity check: you just looked this allocation up, yet the deallocation failed. Did you introduce a race condition?");
            if (!Allocate(id, size, out newStart))
            {
                //Failed to find a location that fits the requested size. Allocate at the old size.
                success = Allocate(id, currentSize, out newStart);
                Debug.Assert(success, "You just deallocated a region of this size, so the allocation must succeed. Did you introduce a race condition?");
                return false;
            }
            return true;
        }

        [Conditional("DEBUG")]
        private void ValidatePointers()
        {
            if (allocations.Count == 0)
                return;
            var initialId = allocations.Keys[0];
            ulong backwardId = initialId;
            ulong forwardId = initialId;
            for (int i = 0; i < allocations.Count; ++i)
            {
                var backwardIndex = allocations.IndexOf(backwardId);
                backwardId = allocations.Values[backwardIndex].Previous;

                var forwardIndex = allocations.IndexOf(forwardId);
                forwardId = allocations.Values[forwardIndex].Next;
            }
            Debug.Assert(initialId == backwardId && initialId == forwardId, "We should be able to walk back to the starting id in exactly allocations.Count steps in either direction.");
        }

        public void Dispose()
        {
            allocations.Dispose(pool);
        }
    }
}
