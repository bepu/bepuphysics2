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
    public struct TaskOverlapsCollection
    {
        //Note that we use multiple separately allocated lists rather than one big contiguous one.
        //This is to keep the door open for simultaneous traversals that would add overlaps from multiple pairs in an interleaved way.
        Buffer<QuickList<(int childA, int childB), Buffer<(int, int)>>> overlaps;
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
        public ref QuickList<(int childA, int childB), Buffer<(int, int)>> GetOverlapsForPair(int pairIndex)
        {
            Debug.Assert(pairIndex < PairCount);
            return ref overlaps[pairIndex];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref (int, int) AllocatePair(int pairIndex)
        {
            Debug.Assert(pairIndex < PairCount);
            ref var list = ref overlaps[pairIndex];
            if (list.Span.Length == list.Count)
            {
                //The list needs to be resized (or initially allocated).
                list.Resize(MathHelper.Max(initialAllocationCountPerPair, list.Count * 2), pool.SpecializeFor<(int, int)>());
            }
            return ref list.AllocateUnsafely();
        }

        public void Dispose()
        {
            var childPairPool = pool.SpecializeFor<(int, int)>();
            for (int i = 0; i < PairCount; ++i)
            {
                overlaps[i].Dispose(childPairPool);
            }
            pool.Return(ref overlaps);
        }
    }
}
