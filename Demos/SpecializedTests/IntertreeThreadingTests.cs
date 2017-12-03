using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

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

        static void TestTrees(BufferPool pool, IThreadDispatcher threadDispatcher, Random random)
        {
            var treeA = new Tree(pool, 1);
            var treeB = new Tree(pool, 1);

            var aBounds = new BoundingBox(new Vector3(-30, 0, -30), new Vector3(30, 0, 30));
            var aOffset = new Vector3(3f, 3f, 3f);
            var aCount = 1025;
            var bBounds = new BoundingBox(new Vector3(-5, -2, -5), new Vector3(5, 2, 5));
            var bOffset = new Vector3(0.5f, 0.5f, 0.5f);
            var bCount = 2;
            for (int i = 0; i < aCount; ++i)
            {
                GetRandomLocation(random, ref aBounds, out var center);
                var bounds = new BoundingBox(center - aOffset, center + aOffset);
                treeA.Add(ref bounds);
            }
            for (int i = 0; i < bCount; ++i)
            {
                GetRandomLocation(random, ref bBounds, out var center);
                var bounds = new BoundingBox(center - bOffset, center + bOffset);
                treeB.Add(ref bounds);
            }
            var singleThreadedResults = new OverlapHandler { Pairs = new List<(int a, int b)>() };
            treeA.GetOverlaps(treeB, ref singleThreadedResults);
            for (int i = 0; i < singleThreadedResults.Pairs.Count; ++i)
            {
                if (singleThreadedResults.Pairs[i].b < singleThreadedResults.Pairs[i].a)
                {
                    singleThreadedResults.Pairs[i] = (singleThreadedResults.Pairs[i].b, singleThreadedResults.Pairs[i].a);
                }
            }
            for (int i = 0; i < 10; ++i)
            {
                treeA.RefitAndRefine(i);
                treeB.RefitAndRefine(i);
            }
            treeA.Validate();
            treeB.Validate();

            var context = new Tree.MultithreadedIntertreeTest<OverlapHandler>(pool);
            var handlers = new OverlapHandler[threadDispatcher.ThreadCount];
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                handlers[i].Pairs = new List<(int a, int b)>();
            }
            context.PrepareJobs(treeA, treeB, handlers, threadDispatcher.ThreadCount);
            threadDispatcher.DispatchWorkers(context.PairTest);
            context.CompleteTest();
            List<(int a, int b)> multithreadedResults = new List<(int, int)>();
            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
            {
                multithreadedResults.AddRange(handlers[i].Pairs);
            }
            for (int i = 0; i < multithreadedResults.Count; ++i)
            {
                if (multithreadedResults[i].b < multithreadedResults[i].a)
                {
                    multithreadedResults[i] = (multithreadedResults[i].b, multithreadedResults[i].a);
                }
            }
            Comparison<(int, int)> comparison = (a, b) =>
            {
                var combinedA = ((ulong)a.Item1 << 32) | ((uint)a.Item2);
                var combinedB = ((ulong)b.Item1 << 32) | ((uint)b.Item2);
                return combinedA.CompareTo(combinedB);
            };
            singleThreadedResults.Pairs.Sort(comparison);
            multithreadedResults.Sort(comparison);
            if (singleThreadedResults.Pairs.Count != multithreadedResults.Count)
            {
                throw new Exception("Result counts don't match.");
            }
            for (int i = 0; i < singleThreadedResults.Pairs.Count; ++i)
            {
                var singleThreadedPair = singleThreadedResults.Pairs[i];
                var multithreadedPair = multithreadedResults[i];
                if (singleThreadedPair.a != multithreadedPair.a ||
                    singleThreadedPair.b != multithreadedPair.b)
                {
                    throw new Exception("Results don't match.");
                }
            }
            treeA.Dispose();
            treeB.Dispose();
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
