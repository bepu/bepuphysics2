using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using BepuPhysics.Trees;

namespace Demos.SpecializedTests
{
    public static class TreeTest
    {
        public static void Test()
        {
            var pool = new BufferPool();
            var tree = new Tree(pool, 128);

            const int leafCountAlongXAxis = 11;
            const int leafCountAlongYAxis = 13;
            const int leafCountAlongZAxis = 15;
            var leafCount = leafCountAlongXAxis * leafCountAlongYAxis * leafCountAlongZAxis;
            pool.Take<BoundingBox>(leafCount, out var leafBounds);

            const float boundsSpan = 2;
            const float spanRange = 2;
            const float boundsSpacing = 3;
            var random = new Random(5);
            for (int i = 0; i < leafCountAlongXAxis; ++i)
            {
                for (int j = 0; j < leafCountAlongYAxis; ++j)
                {
                    for (int k = 0; k < leafCountAlongZAxis; ++k)
                    {
                        var index = leafCountAlongXAxis * leafCountAlongYAxis * k + leafCountAlongXAxis * j + i;
                        leafBounds[index].Min = new Vector3(i, j, k) * boundsSpacing;
                        leafBounds[index].Max = leafBounds[index].Min + new Vector3(boundsSpan) +
                            spanRange * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());

                    }
                }
            }

            var prebuiltCount = Math.Max(leafCount / 2, 1);

            tree.SweepBuild(pool, leafBounds.Slice(prebuiltCount));
            tree.Validate();


            for (int i = prebuiltCount; i < leafCount; ++i)
            {
                tree.Add(ref leafBounds[i], pool);
            }
            tree.Validate();

            pool.TakeAtLeast<int>(leafCount, out var handleToLeafIndex);
            pool.TakeAtLeast<int>(leafCount, out var leafIndexToHandle);
            for (int i = 0; i < leafCount; ++i)
            {
                handleToLeafIndex[i] = i;
                leafIndexToHandle[i] = i;
            }

            const int iterations = 100000;
            const int maximumChangesPerIteration = 20;

            var threadDispatcher = new SimpleThreadDispatcher(Environment.ProcessorCount);
            var refineContext = new Tree.RefitAndRefineMultithreadedContext();
            var selfTestContext = new Tree.MultithreadedSelfTest<OverlapHandler>(pool);
            var overlapHandlers = new OverlapHandler[threadDispatcher.ThreadCount];
            Action<int> pairTestAction = selfTestContext.PairTest;
            var removedLeafHandles = new QuickList<int>(leafCount, pool);
            for (int i = 0; i < iterations; ++i)
            {
                var changeCount = random.Next(maximumChangesPerIteration);
                for (int j = 0; j <= changeCount; ++j)
                {
                    var addedFraction = tree.LeafCount / (float)leafCount;
                    if (random.NextDouble() < addedFraction)
                    {
                        //Remove a leaf.
                        var leafIndexToRemove = random.Next(tree.LeafCount);
                        var handleToRemove = leafIndexToHandle[leafIndexToRemove];
                        var movedLeafIndex = tree.RemoveAt(leafIndexToRemove);
                        if (movedLeafIndex >= 0)
                        {
                            var movedHandle = leafIndexToHandle[movedLeafIndex];
                            handleToLeafIndex[movedHandle] = leafIndexToRemove;
                            leafIndexToHandle[leafIndexToRemove] = movedHandle;
                            leafIndexToHandle[movedLeafIndex] = -1;
                        }
                        else
                        {
                            //The removed leaf was the last one. This leaf index is no longer associated with any existing leaf.
                            leafIndexToHandle[leafIndexToRemove] = -1;
                        }
                        handleToLeafIndex[handleToRemove] = -1;

                        removedLeafHandles.AddUnsafely(handleToRemove);

                        tree.Validate();
                    }
                    else
                    {
                        //Add a leaf.
                        var indexInRemovedList = random.Next(removedLeafHandles.Count);
                        var handleToAdd = removedLeafHandles[indexInRemovedList];
                        removedLeafHandles.FastRemoveAt(indexInRemovedList);
                        var leafIndex = tree.Add(ref leafBounds[handleToAdd], pool);
                        leafIndexToHandle[leafIndex] = handleToAdd;
                        handleToLeafIndex[handleToAdd] = leafIndex;

                        tree.Validate();
                    }
                }

                tree.Refit();
                tree.Validate();

                tree.RefitAndRefine(pool, i);
                tree.Validate();

                var handler = new OverlapHandler();
                tree.GetSelfOverlaps(ref handler);
                tree.Validate();

                refineContext.RefitAndRefine(ref tree, pool, threadDispatcher, i);
                tree.Validate();
                for (int k = 0; k < threadDispatcher.ThreadCount; ++k)
                {
                    overlapHandlers[k] = new OverlapHandler();
                }
                selfTestContext.PrepareJobs(ref tree, overlapHandlers, threadDispatcher.ThreadCount);
                threadDispatcher.DispatchWorkers(pairTestAction);
                selfTestContext.CompleteSelfTest();
                tree.Validate();

                if (i % 50 == 0)
                {
                    Console.WriteLine($"Cost: {tree.MeasureCostMetric()}");
                    Console.WriteLine($"Cache Quality: {tree.MeasureCacheQuality()}");
                    Console.WriteLine($"Overlap Count: {handler.OverlapCount}");
                }
            }

            threadDispatcher.Dispose();
            pool.Clear();


        }

        struct OverlapHandler : IOverlapHandler
        {
            public int OverlapCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Handle(int indexA, int indexB)
            {
                ++OverlapCount;
            }
        }

    }
}
