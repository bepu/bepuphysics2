using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;

namespace Demos.SpecializedTests
{
    public static class IslandCacheConvergence
    {
        struct IslandBlob : ISwapHeuristic
        {
            public void Reset()
            {
                bodyIndex = 0;
            }
            int bodyIndex;
            static ArrayPool<int> pool = new ArrayPool<int>();
            public void Swap(int[] bodies, int islandCount)
            {
                //Pick a body. Find all bodies of its type.
                //Put them all in the same spot, starting at the body with the lowest index.
                if (bodyIndex >= bodies.Length)
                    bodyIndex = 0;
                var bodyType = bodies[bodyIndex];
                int lowestIndex = -1;
                QuickList<int, Array<int>>.Create(pool, 128, out var islandBodyIndices);
                for (int i = bodies.Length - 1; i >= 0; --i)
                {
                    if (bodies[i] == bodyType)
                    {
                        lowestIndex = i;
                        islandBodyIndices.Add(i, pool);
                    }
                }
                var baseIndex = Math.Min(bodies.Length - islandBodyIndices.Count, lowestIndex);
                Array.Sort(islandBodyIndices.Span.Memory, 0, islandBodyIndices.Count);
                for (int i = 0; i < islandBodyIndices.Count; ++i)
                {
                    IslandCacheConvergence.Swap(ref bodies[baseIndex + i], ref bodies[islandBodyIndices[i]]);
                }
                bodyIndex += islandBodyIndices.Count;
                islandBodyIndices.Dispose(pool);
            }
        }

        struct BlindLocal : ISwapHeuristic
        {
            public void Reset()
            {
                swapIndex = 0;
            }
            int swapIndex;
            public void Swap(int[] bodies, int islandCount)
            {
                //Pick a body. Does it have any connections that are currently positioned to the right?
                //This will pointlessly shuffle all island neighbors to the right.
                //The goal here is just super simplicity.
                //(This is roughly equivalent to checking a body's connections, seeing if any are to the right, and if they are, pulling them to be a neighbor.
                //This is NOT a globally stable heuristic- even after it 'converges' it will continually reshuffle the elements within islands. 
                //If you had full island information, you could grab only one and stick it onto the end of the currently gathered island set....)
                var nextIndex = (swapIndex + 1) % bodies.Length;
                for (int i = (nextIndex + 1) % bodies.Length; i < bodies.Length; ++i)
                {
                    if (bodies[i] == bodies[swapIndex])
                    {
                        IslandCacheConvergence.Swap(ref bodies[i], ref bodies[nextIndex]);
                        //Just do one.
                        break;
                    }
                }
                swapIndex = nextIndex;


            }
        }

        interface ISwapHeuristic
        {
            void Swap(int[] bodies, int islandCount);
            void Reset();
        }

        private static void Swap(ref int a, ref int b)
        {
            var temp = a;
            a = b;
            b = temp;
        }
        private static void Scramble(Random random, int[] bodies)
        {
            for (int i = bodies.Length - 1; i >= 0; --i)
            {
                Swap(ref bodies[i], ref bodies[random.Next(i)]);
            }
        }
        private static void Test<T>(int testCount, int islandCount, int islandMaximumBodyCount, int maximumIterationCount, ref T swapHeuristic) where T : ISwapHeuristic
        {
            var random = new Random(5);
            int[] islandBodyCounts = new int[islandCount];
            Console.WriteLine($"{testCount} tests with {islandCount} islands of 1 to {islandMaximumBodyCount} bodies each.");
            for (int testIndex = 0; testIndex < testCount; ++testIndex)
            {
                int totalBodyCount = 0;
                for (int islandIndex = 0; islandIndex < islandCount; ++islandIndex)
                {
                    var bodyCount = 1 + random.Next(islandMaximumBodyCount);
                    islandBodyCounts[islandIndex] = bodyCount;
                    totalBodyCount += bodyCount;
                }
                var bodies = new int[totalBodyCount];
                int bodyIndex = 0;
                for (int islandIndex = 0; islandIndex < islandCount; ++islandIndex)
                {
                    for (int islandBodyIndex = 0; islandBodyIndex < islandBodyCounts[islandIndex]; ++islandBodyIndex)
                    {
                        bodies[bodyIndex++] = islandIndex;
                    }
                }
                Scramble(random, bodies);
                int convergenceIterationCount = -1;
                for (int iterationIndex = 0; iterationIndex < maximumIterationCount; ++iterationIndex)
                {
                    swapHeuristic.Swap(bodies, islandCount);

                    for (int i = 0; i < islandBodyCounts.Length; ++i)
                    {
                        Debug.Assert(bodies.Count(x => x == i) == islandBodyCounts[i], "Hey yo heuristic be broke.");
                    }

                    int currentIsland = bodies[0];
                    int bodyCount = 1;
                    bool iterationCausedConvergence = true;
                    for (int i = 1; i < bodies.Length; ++i)
                    {
                        if (bodies[i] == currentIsland)
                        {
                            ++bodyCount;
                        }
                        else
                        {
                            if (bodyCount != islandBodyCounts[currentIsland])
                            {
                                //Not yet converged; the island isn't contiguous.
                                iterationCausedConvergence = false;
                                break;
                            }
                            else
                            {
                                bodyCount = 1;
                                currentIsland = bodies[i];
                            }
                        }
                    }
                    if (iterationCausedConvergence)
                    {
                        convergenceIterationCount = iterationIndex;
                        break;
                    }
                }
                swapHeuristic.Reset();

                if (convergenceIterationCount >= 0)
                    Console.WriteLine($"Test {testIndex}, {bodies.Length} bodies, converged. Iteration count: {convergenceIterationCount}");
                else
                    Console.WriteLine($"Test {testIndex}, {bodies.Length} bodies, did not converge.");
            }
        }

        public static void Test()
        {
            var blindLocal = new BlindLocal();
            Test(5, 30, 100, 100000, ref blindLocal);
            var islandBlob = new IslandBlob();
            Test(5, 30, 100, 100000, ref islandBlob);
        }
    }
}
