using BepuUtilities;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public unsafe struct ConvexCompoundOverlaps : ICollisionTaskSubpairOverlaps
    {
        public Buffer<int> Overlaps;
        public int Count;
        
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

    public struct ConvexCompoundTaskOverlaps : ICollisionTaskOverlaps<ConvexCompoundOverlaps>
    {
        Buffer<ConvexCompoundOverlaps> overlaps;
        internal Buffer<OverlapQueryForPair> subpairQueries;
        public ConvexCompoundTaskOverlaps(BufferPool pool, int pairCount)
        {
            pool.Take(pairCount, out overlaps);
            pool.Take(pairCount, out subpairQueries);
            //We rely on the length being zero to begin with for lazy initialization.
            overlaps.Clear(0, pairCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexCompoundOverlaps GetOverlapsForPair(int pairIndex)
        {
            return ref overlaps[pairIndex];
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref OverlapQueryForPair GetQueryForPair(int pairIndex)
        {
            return ref subpairQueries[pairIndex];
        }

        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < overlaps.Length; ++i)
            {
                overlaps[i].Dispose(pool);
            }
            pool.Return(ref subpairQueries);
            pool.Return(ref overlaps);
        }

    }
}
