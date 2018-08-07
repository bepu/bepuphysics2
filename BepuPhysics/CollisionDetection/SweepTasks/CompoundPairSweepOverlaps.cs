using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct CompoundPairSweepOverlaps
    {
        Buffer<ChildOverlapsCollection> childOverlaps;
        public readonly int ChildCount;
        public CompoundPairSweepOverlaps(BufferPool pool, int childCount)
        {
            ChildCount = childCount;
            pool.Take(childCount, out childOverlaps);
            //We rely on the length being zero to begin with for lazy initialization.
            childOverlaps.Clear(0, childCount);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ChildOverlapsCollection GetOverlapsForChild(int pairIndex)
        {
            return ref childOverlaps[pairIndex];
        }

        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < ChildCount; ++i)
            {
                childOverlaps[i].Dispose(pool);
            }
            pool.Return(ref childOverlaps);
        }
    }
}
