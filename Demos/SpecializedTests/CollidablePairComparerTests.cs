using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class CollidablePairComparerTests
    {
        public static void Test()
        {
            //var random = new Random(5);
            //var comparer = new CollidablePairComparer();
            //for (int i = 0; i < 10000; ++i)
            //{
            //    var a = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
            //    var b = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
            //    var pair1 = new CollidablePair(a, b);
            //    var pair2 = new CollidablePair(b, a);
            //    Debug.Assert(comparer.Hash(ref pair1) == comparer.Hash(ref pair2));
            //    Debug.Assert(comparer.Equals(ref pair1, ref pair2));
            //}
            //for (int i = 0; i < 10000; ++i)
            //{
            //    var a = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
            //    var b = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
            //    var pair1 = new CollidablePair(a, b);
            //    CollidablePair pair2;
            //    do
            //    {
            //        var a2 = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
            //        var b2 = new CollidableReference((CollidableMobility)random.Next(3), random.Next(1 << 30));
            //        pair2 = new CollidablePair(a2, b2);
            //    } while (
            //    (pair2.A.Packed == pair1.A.Packed && pair2.B.Packed == pair1.B.Packed) ||
            //    (pair2.B.Packed == pair1.A.Packed && pair2.A.Packed == pair1.B.Packed));
            //    Debug.Assert(!comparer.Equals(ref pair1, ref pair2));
            //}


            const int iterationCount = 1000;
            const int perLayerCollidableCount = 900;
            const int layerCount = 10;
            int[] creationRemap = new int[perLayerCollidableCount * (layerCount - 1)];
            int[] lookupRemap = new int[creationRemap.Length];
            for (int i = 0; i < creationRemap.Length; ++i)
            {
                creationRemap[i] = i;
                lookupRemap[i] = i;
            }

            BufferPool pool = new BufferPool();
            QuickDictionary<CollidablePair, int, CollidablePairComparer>.Create(pool, creationRemap.Length, 1,
                out var dictionary);

            var random = new Random(5);
            for (int i = 0; i < creationRemap.Length - 1; ++i)
            {
                {
                    var temp = creationRemap[i];
                    var swapTarget = random.Next(i + 1, creationRemap.Length);
                    creationRemap[i] = creationRemap[swapTarget];
                    creationRemap[swapTarget] = temp;
                }
                {
                    var temp = lookupRemap[i];
                    var swapTarget = random.Next(i + 1, lookupRemap.Length);
                    lookupRemap[i] = lookupRemap[swapTarget];
                    lookupRemap[swapTarget] = temp;
                }
            }

            int accumulator = 0;
            double totalTime = 0;
            const int warmupIterations = 128;
            for (int iterationIndex = 0; iterationIndex < iterationCount + warmupIterations; ++iterationIndex)
            {
                dictionary.Clear();
                for (int i = 0; i < creationRemap.Length; ++i)
                {
                    var index = creationRemap[i];
                    var pair = new CollidablePair
                    {
                        A = new CollidableReference(CollidableMobility.Kinematic, index),
                        B = new CollidableReference(CollidableMobility.Dynamic, index + perLayerCollidableCount)
                    };
                    dictionary.AddUnsafely(ref pair, ref index);
                }
                CacheBlaster.Blast();
                //Prewarm the remap into cache to more closely mirror the behavior in the narrow phase.
                for (int i = 0; i < lookupRemap.Length; ++i)
                {
                    accumulator += lookupRemap[i];
                }
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < lookupRemap.Length; ++i)
                {
                    var collidableIndex = lookupRemap[i];
                    var pair = new CollidablePair
                    {
                        A = new CollidableReference(CollidableMobility.Kinematic, collidableIndex),
                        B = new CollidableReference(CollidableMobility.Dynamic, collidableIndex + perLayerCollidableCount)
                    };
                    var dictionaryIndex = dictionary.IndexOf(ref pair);
                    accumulator += dictionaryIndex;
                }
                var end = Stopwatch.GetTimestamp();
                if (iterationIndex >= warmupIterations)
                    totalTime += (end - start) / (double)Stopwatch.Frequency;
            }
            Console.WriteLine($"Time per lookup (ns): {1e9 * totalTime / (iterationCount * creationRemap.Length)}, acc{accumulator}");

            pool.Clear();
        }
    }
}
