using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;

namespace BEPUutilitiesTests
{
    public static class AllocatorTests
    {

        public static void TestChurnStability()
        {
            var pool = new BufferPool();
            var allocator = new Allocator(2048, pool);
            var random = new Random(5);
            ulong idCounter = 0;
            var allocatedIds = new QuickList<ulong>(8, pool);
            var unallocatedIds = new QuickList<ulong>(8, pool);
            for (int i = 0; i < 512; ++i)
            {
                long start;
                var id = idCounter++;
                //allocator.ValidatePointers();
                if (allocator.Allocate(id, 1 + random.Next(5), out start))
                {
                    allocatedIds.Add(id, pool);
                }
                else
                {
                    unallocatedIds.Add(id, pool);
                }
                //allocator.ValidatePointers();
            }
            for (int timestepIndex = 0; timestepIndex < 100000; ++timestepIndex)
            {
                //First add and remove a bunch randomly.
                for (int i = random.Next(Math.Min(allocatedIds.Count, 15)); i >= 0; --i)
                {
                    var indexToRemove = random.Next(allocatedIds.Count);
                    //allocator.ValidatePointers();
                    var deallocated = allocator.Deallocate(allocatedIds[indexToRemove]);
                    Debug.Assert(deallocated);
                    //allocator.ValidatePointers();
                    unallocatedIds.Add(allocatedIds[indexToRemove], pool);
                    allocatedIds.FastRemoveAt(indexToRemove);
                }
                for (int i = random.Next(Math.Min(unallocatedIds.Count, 15)); i >= 0; --i)
                {
                    var indexToAllocate = random.Next(unallocatedIds.Count);
                    //allocator.ValidatePointers();
                    if (allocator.Allocate(unallocatedIds[indexToAllocate], random.Next(3), out long start))
                    {
                        //allocator.ValidatePointers();
                        allocatedIds.Add(unallocatedIds[indexToAllocate], pool);
                        unallocatedIds.FastRemoveAt(indexToAllocate);
                    }
                    //allocator.ValidatePointers();
                }
                //Check to ensure that everything's still coherent.
                for (int i = 0; i < allocatedIds.Count; ++i)
                {
                    Debug.Assert(allocator.Contains(allocatedIds[i]));
                }
                for (int i = 0; i < unallocatedIds.Count; ++i)
                {
                    Debug.Assert(!allocator.Contains(unallocatedIds[i]));
                }
            }
            //Wind it down.
            for (int i = 0; i < allocatedIds.Count; ++i)
            {
                var deallocated = allocator.Deallocate(allocatedIds[i]);
                Debug.Assert(deallocated);
            }
            //Confirm cleanup.
            for (int i = 0; i < allocatedIds.Count; ++i)
            {
                Debug.Assert(!allocator.Contains(allocatedIds[i]));
            }
            for (int i = 0; i < unallocatedIds.Count; ++i)
            {
                Debug.Assert(!allocator.Contains(unallocatedIds[i]));
            }
            pool.Clear();
        }
    }
}
