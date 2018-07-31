using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public unsafe struct PairsToTestForOverlap
    {
        public void* Container;
        public Vector3 Min;
        public Vector3 Max;
    }

    public unsafe struct ChildOverlapsCollection
    {
        Buffer<int> overlaps;
        int count;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref int Allocate(BufferPool pool)
        {
            if (overlaps.Length == count)
            {
                pool.Resize(ref overlaps, MathHelper.Max(64, count * 2), count);
            }
            return ref overlaps[count++];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose(BufferPool pool)
        {
            if (overlaps.Allocated)
                pool.Return(ref overlaps);
        }
    }
    /// <summary>
    /// Stores the child detected within a pair instance as a series of lists. Each childA is given its own list.
    /// </summary>
    public unsafe struct PairOverlapsCollection
    {
        internal Buffer<ChildOverlapsCollection> childOverlaps;
        internal int childCount;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairOverlapsCollection(int childCount, BufferPool pool)
        {
            pool.Take(childCount, out childOverlaps);
            //We test the overlap length, so just zero init the memory.
            childOverlaps.Clear(0, childCount);
            this.childCount = childCount;
        }

        public ref ChildOverlapsCollection GetChildOverlaps(int childIndex)
        {
            return ref childOverlaps[childIndex];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < childCount; ++i)
            {
                childOverlaps[i].Dispose(pool);
            }
            pool.Return(ref childOverlaps);
        }
    }

    public struct TaskOverlapsCollection
    {
        Buffer<PairOverlapsCollection> pairOverlaps;
        int pairCount;
        public TaskOverlapsCollection(BufferPool pool, int pairCount)
        {
            this.pairCount = pairCount;
            pool.Take(pairCount, out pairOverlaps);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref PairOverlapsCollection GetPairOverlaps(int pairIndex)
        {
            Debug.Assert(pairIndex < pairCount);
            return ref pairOverlaps[pairIndex];
        }

        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < pairCount; ++i)
            {
                pairOverlaps[i].Dispose(pool);
            }
            pool.Return(ref pairOverlaps);
        }
    }
}
