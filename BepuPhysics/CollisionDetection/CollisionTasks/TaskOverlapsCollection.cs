using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    /// <summary>
    /// Stores the child detected within a pair instance as a series of lists. Each childA is given its own list.
    /// </summary>
    public unsafe struct PairOverlapsCollection
    {
        Buffer<int> overlaps;
        int count;
        public int OverlapCount;
        public int ChildCount;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref int AllocateChild(int childIndex, BufferPool pool, int minimumChildCapacity, out int* childOverlapCount)
        {
            var newCount = count + 2;
            var targetCapacity = newCount + minimumChildCapacity;
            if (overlaps.Length < targetCapacity)
            {
                pool.Resize(ref overlaps, targetCapacity, count);
            }
            var pointer = (int*)overlaps.Memory;
            //The header of each list contains the child index and the count.
            pointer[count] = childIndex;
            childOverlapCount = pointer + count + 1;
            *childOverlapCount = 0;
            count = newCount;
            ++ChildCount;
            return ref pointer[count];
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref int Allocate(int* childOverlapCount, BufferPool pool)
        {
            if (overlaps.Length == count)
            {
                pool.Resize(ref overlaps, count * 2, count);
            }
            ++*childOverlapCount;
            ++OverlapCount;
            return ref overlaps[count++];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool VisitNextChild(ref int traversalIndex, out int childIndex, out int childOverlapCount)
        {
            Debug.Assert(traversalIndex <= count);
            if (traversalIndex == count)
            {
                childIndex = default;
                childOverlapCount = default;
                return false;
            }
            childIndex = overlaps[traversalIndex++];
            childOverlapCount = overlaps[traversalIndex++];
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetOverlap(ref int traversalIndex)
        {
            Debug.Assert(traversalIndex >= 2 && traversalIndex < count);
            var result = ((int*)overlaps.Memory)[traversalIndex];
            ++traversalIndex;
            return result;
        }

        public void Dispose(BufferPool pool)
        {
            pool.Return(ref overlaps);
        }
    }

    public struct TaskOverlapsCollection
    {
        //Note that we use multiple separately allocated lists rather than one big contiguous one.
        //This is to keep the door open for simultaneous traversals that would add overlaps from multiple pairs in an interleaved way.
        Buffer<PairOverlapsCollection> overlaps;
        public readonly int PairCount;
        int initialAllocationCountPerPair;
        BufferPool pool;
        public TaskOverlapsCollection(BufferPool pool, int pairCount, int initialAllocationCountPerPair = 64)
        {
            PairCount = pairCount;
            this.initialAllocationCountPerPair = initialAllocationCountPerPair;
            this.pool = pool;
            pool.Take(pairCount, out overlaps);
            //We lazily initialize overlap lists, so the capacities need to start at zero.
            overlaps.Clear(0, pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref PairOverlapsCollection GetPairOverlaps(int pairIndex)
        {
            Debug.Assert(pairIndex < PairCount);
            return ref overlaps[pairIndex];
        }

        public void Dispose()
        {
            for (int i = 0; i < PairCount; ++i)
            {
                overlaps[i].Dispose(pool);
            }
            pool.Return(ref overlaps);
        }
    }
}
