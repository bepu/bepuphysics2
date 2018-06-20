using BepuPhysics;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Trees;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using System.Runtime.CompilerServices;

namespace Demos.SpecializedTests
{
    public static class IntertreeThreadingTests
    {
        static void GetRandomLocation(Random random, ref BoundingBox locationBounds, out Vector3 location)
        {
            location = (locationBounds.Max - locationBounds.Min) * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) + locationBounds.Min;
        }
        struct OverlapHandler : IOverlapHandler
        {
            public List<(int a, int b)> Pairs;
            public void Handle(int indexA, int indexB)
            {
                Pairs.Add((indexA, indexB));
            }
        }

        static void GetBoundsForLeaf(in Tree tree, int leafIndex, out BoundingBox bounds)
        {
            ref var leaf = ref tree.Leaves[leafIndex];
            ref var node = ref tree.Nodes[leaf.NodeIndex];
            bounds = leaf.ChildIndex == 0 ? new BoundingBox(node.A.Min, node.A.Max) : new BoundingBox(node.B.Min, node.B.Max);
        }

        static void SortPairs(List<(int a, int b)> pairs)
        {
            for (int i = 0; i < pairs.Count; ++i)
            {
                if (pairs[i].b < pairs[i].a)
                {
                    pairs[i] = (pairs[i].b, pairs[i].a);
                }
            }
            Comparison<(int, int)> comparison = (a, b) =>
            {
                var combinedA = ((ulong)a.Item1 << 32) | ((uint)a.Item2);
                var combinedB = ((ulong)b.Item1 << 32) | ((uint)b.Item2);
                return combinedA.CompareTo(combinedB);
            };
            pairs.Sort(comparison);
        }

        static void TestTrees(BufferPool pool, IThreadDispatcher threadDispatcher, Random random)
        {
            var treeA = new Tree(pool, 1);
            var treeB = new Tree(pool, 1);

            var aBounds = new BoundingBox(new Vector3(-40, 0, -40), new Vector3(40, 0, 40));
            var aOffset = new Vector3(3f, 3f, 3f);
            var aCount = 1024;
            var bBounds = new BoundingBox(new Vector3(-5, -2, -5), new Vector3(5, 2, 5));
            var bOffset = new Vector3(0.5f, 0.5f, 0.5f);
            var bCount = 3;
            for (int i = 0; i < aCount; ++i)
            {
                GetRandomLocation(random, ref aBounds, out var center);
                var bounds = new BoundingBox(center - aOffset, center + aOffset);
                treeA.Add(ref bounds, pool);
            }
            for (int i = 0; i < bCount; ++i)
            {
                GetRandomLocation(random, ref bBounds, out var center);
                var bounds = new BoundingBox(center - bOffset, center + bOffset);
                treeB.Add(ref bounds, pool);
            }
            
            {
                var indexToRemove = 1;
                GetBoundsForLeaf(treeB, indexToRemove, out var removedBounds);
                treeB.RemoveAt(indexToRemove);
                treeA.Add(ref removedBounds, pool);
            }
            
            var singleThreadedResults = new OverlapHandler { Pairs = new List<(int a, int b)>() };
            treeA.GetOverlaps(ref treeB, ref singleThreadedResults);
            SortPairs(singleThreadedResults.Pairs);
            for (int i = 0; i < 10; ++i)
            {
                treeA.RefitAndRefine(pool, i);
                treeB.RefitAndRefine(pool, i);
            }
            treeA.Validate();
            treeB.Validate();

            var context = new Tree.MultithreadedIntertreeTest<OverlapHandler>(pool);
            var handlers = new OverlapHandler[threadDispatcher.ThreadCount];
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                handlers[i].Pairs = new List<(int a, int b)>();
            }
            context.PrepareJobs(ref treeA, ref treeB, handlers, threadDispatcher.ThreadCount);
            threadDispatcher.DispatchWorkers(context.PairTest);
            context.CompleteTest();
            List<(int a, int b)> multithreadedResults = new List<(int, int)>();
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                multithreadedResults.AddRange(handlers[i].Pairs);
            }
            SortPairs(multithreadedResults);

            if (singleThreadedResults.Pairs.Count != multithreadedResults.Count)
            {
                throw new Exception("Single threaded vs multithreaded counts don't match.");
            }
            for (int i = 0; i < singleThreadedResults.Pairs.Count; ++i)
            {
                var singleThreadedPair = singleThreadedResults.Pairs[i];
                var multithreadedPair = multithreadedResults[i];
                if (singleThreadedPair.a != multithreadedPair.a ||
                    singleThreadedPair.b != multithreadedPair.b)
                {
                    throw new Exception("Single threaded vs multithreaded results don't match.");
                }
            }

            //Single and multithreaded variants produce the same results. But do they match a brute force test?
            Tree smaller, larger;
            if (treeA.LeafCount < treeB.LeafCount)
            {
                smaller = treeA;
                larger = treeB;
            }
            else
            {
                smaller = treeB;
                larger = treeA;
            }
            var bruteResultsEnumerator = new BruteForceResultsEnumerator();
            bruteResultsEnumerator.Pairs = new List<(int a, int b)>();
            for (int i = 0; i < smaller.LeafCount; ++i)
            {
                GetBoundsForLeaf(smaller, i, out var bounds);
                bruteResultsEnumerator.QuerySourceIndex = i;
                larger.GetOverlaps(bounds, ref bruteResultsEnumerator);
            }
            SortPairs(bruteResultsEnumerator.Pairs);

            if (singleThreadedResults.Pairs.Count != bruteResultsEnumerator.Pairs.Count)
            {
                throw new Exception("Brute force vs intertree counts don't match.");
            }
            for (int i = 0; i < singleThreadedResults.Pairs.Count; ++i)
            {
                var singleThreadedPair = singleThreadedResults.Pairs[i];
                var bruteForcePair = bruteResultsEnumerator.Pairs[i];
                if (singleThreadedPair.a != bruteForcePair.a ||
                    singleThreadedPair.b != bruteForcePair.b)
                {
                    throw new Exception("Brute force vs intertree results don't match.");
                }
            }

            treeA.Dispose(pool);
            treeB.Dispose(pool);
        }

        struct BruteForceResultsEnumerator : IBreakableForEach<int>
        {
            public List<(int a, int b)> Pairs;
            public int QuerySourceIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool LoopBody(int foundIndex)
            {
                Pairs.Add((QuerySourceIndex, foundIndex));
                return true;
            }
        }

        public static void Test()
        {
            var random = new Random(5);
            var pool = new BufferPool();
            var threadDispatcher = new SimpleThreadDispatcher(Environment.ProcessorCount);
            for (int i = 0; i < 1000; ++i)
            {
                TestTrees(pool, threadDispatcher, random);
            }
            pool.Clear();
            threadDispatcher.Dispose();
        }
    }
}
