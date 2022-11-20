using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    internal struct SleepingPair
    {
        public CollidablePair Pair;
        public ConstraintCache Cache;
    }


    internal struct SleepingSet
    {
        public bool Allocated => Pairs.Span.Allocated;

        public QuickList<SleepingPair> Pairs;

        public void Dispose(BufferPool pool)
        {
            Pairs.Dispose(pool);
        }
    }

    internal struct SleepingSetBuilder
    {
        public QuickList<SleepingPair> Pairs;
        public SleepingSetBuilder(BufferPool pool, int initialPairCapacity)
        {
            Pairs = new QuickList<SleepingPair>(initialPairCapacity, pool);
        }

        public int Add(BufferPool pool, CollidablePair pair, in ConstraintCache cache)
        {
            var pairIndex = Pairs.Count;
            ref var entry = ref Pairs.Allocate(pool);
            entry.Pair = pair;
            entry.Cache = cache;
            return pairIndex;
        }

        public void FinalizeSet(BufferPool pool, out SleepingSet set)
        {
            //Repackage the gathered caches into a smaller format for longer term storage.
            //This adds a little extra cost, but it 
            //1) avoids the need for most incremental resizes during inactive set construction by sharing allocations and
            //2) minimizes the memory required for the inactive set. Using the same format as the active set would require about a kilobyte per set, 
            //which gets expensive when you have tens of thousands of isolated islands!

            if (Pairs.Count > 0)
            {
                set.Pairs = new QuickList<SleepingPair>(Pairs.Count, pool);
                set.Pairs.AddRangeUnsafely(Pairs.Span, 0, Pairs.Count);
                Pairs.Count = 0;
            }
            else
            {
                //No pairs -> no set required.
                set = new SleepingSet();
            }
        }

        public void Dispose(BufferPool pool)
        {
            Pairs.Dispose(pool);
        }
    }
}
