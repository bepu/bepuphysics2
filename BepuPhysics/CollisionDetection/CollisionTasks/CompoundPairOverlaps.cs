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
    public interface ICollisionTaskSubpairOverlaps
    {
        ref int Allocate(BufferPool pool);
    }

    public interface ICollisionTaskOverlaps<TSubpairOverlaps> where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
    {
        ref TSubpairOverlaps GetOverlapsForPair(int subpairIndex);
    }

    public unsafe struct ChildOverlapsCollection : ICollisionTaskSubpairOverlaps
    {
        public Buffer<int> Overlaps;
        public int Count;
        public int ChildIndex;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref int Allocate(BufferPool pool)
        {
            if (Overlaps.Length == Count)
            {
                pool.ResizeToAtLeast(ref Overlaps, MathHelper.Max(64, Count * 2), Count);
            }
            return ref Overlaps[Count++];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Dispose(BufferPool pool)
        {
            if (Overlaps.Allocated)
                pool.Return(ref Overlaps);
        }
    }

    public struct CompoundPairOverlaps : ICollisionTaskOverlaps<ChildOverlapsCollection>
    {
        Buffer<ChildOverlapsCollection> childOverlaps;
        internal Buffer<OverlapQueryForPair> pairQueries;
        Buffer<(int start, int count)> pairRegions;
        int pairCount;
        int subpairCount;
        public CompoundPairOverlaps(BufferPool pool, int pairCapacity, int subpairCapacity)
        {
            this.pairCount = 0;
            this.subpairCount = 0;
            pool.Take(subpairCapacity, out childOverlaps);
            pool.Take(subpairCapacity, out pairQueries);
            //We rely on the length being zero to begin with for lazy initialization.
            childOverlaps.Clear(0, subpairCapacity);
            pool.Take(pairCapacity, out pairRegions);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CreatePairOverlaps(int childrenInPair)
        {
            pairRegions[pairCount++] = (subpairCount, childrenInPair);
            subpairCount += childrenInPair;
            Debug.Assert(pairCount <= pairRegions.Length);
            Debug.Assert(subpairCount <= childOverlaps.Length);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ChildOverlapsCollection GetOverlapsForPair(int subpairIndex)
        {
            return ref childOverlaps[subpairIndex];
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetPairOverlaps(int pairIndex, out Buffer<ChildOverlapsCollection> pairOverlaps, out Buffer<OverlapQueryForPair> pairQueries)
        {
            ref var region = ref pairRegions[pairIndex];
            childOverlaps.Slice(region.start, region.count, out pairOverlaps);
            this.pairQueries.Slice(region.start, region.count, out pairQueries);
        }

        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < subpairCount; ++i)
            {
                childOverlaps[i].Dispose(pool);
            }
            pool.Return(ref pairQueries);
            pool.Return(ref childOverlaps);
            pool.Return(ref pairRegions);
        }
    }
}
