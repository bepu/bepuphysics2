using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Demos.SpecializedTests
{
    static class SuballocationTests
    {
        public struct TestPredicate : IPredicate<int>
        {
            public int ToCompare;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Matches(ref int x)
            {
                return x == ToCompare;
            }


        }
        public static void Test()
        {
            var pool = new BufferPool();
            for (int i = 3; i < 7; ++i)
            {
                pool.EnsureCapacityForPower(1 << 19, i);
            }

            var random = new Random(5);
            const int listCount = 1000;
            var lists = new QuickList<int>[listCount];
            for (int i = 0; i < 1000; ++i)
            {
                QuickList<int>.Create(pool, 0 + random.Next(9), out lists[i]);
                ref var list = ref lists[i];
                const int anchorSize = 128;
                for (int j = 0; j < 1000; ++j)
                {
                    var removeProbability = 0.5f + 0.5f * (list.Count - anchorSize) / anchorSize;
                    var p = random.NextDouble();
                    if (p < removeProbability)
                    {
                        Debug.Assert(list.Count > 0);
                        //Note that adds can invalidate the start.
                        if (p < removeProbability * 0.5)
                        {
                            //Remove an element that is actually present.
                            var toRemoveIndex = random.Next(list.Count);
                            var predicate = new TestPredicate { ToCompare = list[toRemoveIndex] };
                            var removed = list.FastRemove(ref predicate);
                            Debug.Assert(removed, "If we selected an element from the list, it should be removable.");
                        }
                        else
                        {
                            var toRemove = -(1 + random.Next(16));
                            var predicate = new TestPredicate { ToCompare = toRemove };
                            var removed = list.FastRemove(ref predicate);
                            Debug.Assert(!removed, "Shouldn't be able to remove things that were never added!");
                        }
                    }
                    else
                    {
                        var toAdd = random.Next(256);
                        list.Add(toAdd, pool);
                    }
                }
            }
            for (int i = 0; i < listCount; ++i)
            {
                lists[i].Dispose(pool);
            }
            pool.Clear();
        }
    }
}
