using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        /// <summary>
        /// Caches input and output for the multithreaded execution of a tree's refit and refinement operations.
        /// </summary>
        public class RefitAndRefineMultithreadedContext
        {
            Tree Tree;

            int RefitNodeIndex;
            public QuickList<int> RefitNodes;
            float RefitCostChange;

            int RefinementLeafCountThreshold;
            Buffer<QuickList<int>> RefinementCandidates;
            Action<int> RefitAndMarkAction;

            int RefineIndex;
            public QuickList<int> RefinementTargets;
            public int MaximumSubtrees;
            Action<int> RefineAction;

            IThreadDispatcher threadDispatcher;

            public RefitAndRefineMultithreadedContext()
            {
                RefitAndMarkAction = RefitAndMarkForWorker;
                RefineAction = RefineForWorker;
            }


            public unsafe void CreateRefitAndMarkJobs(ref Tree tree, BufferPool pool, IThreadDispatcher threadDispatcher)
            {
                if (tree.leafCount <= 2)
                {
                    //If there are 2 or less leaves, then refit/refine doesn't do anything at all.
                    //(The root node has no parent, so it does not have a bounding box, and the SAH won't change no matter how we swap the children of the root.)
                    //Avoiding this case also gives the other codepath a guarantee that it will be working with nodes with two children.
                    RefitNodes = default;
                    return;
                }
                this.threadDispatcher = threadDispatcher;
                Tree = tree;
                //Note that we create per-thread refinement candidates. That's because candidates are found during the multithreaded refit and mark phase, and 
                //we don't want to spend the time doing sync work. The candidates are then pruned down to a target single target set for the refine pass.
                pool.Take(threadDispatcher.ThreadCount, out RefinementCandidates);
                Tree.GetRefitAndMarkTuning(out MaximumSubtrees, out var estimatedRefinementCandidateCount, out RefinementLeafCountThreshold);
                //Note that the number of refit nodes is not necessarily bound by MaximumSubtrees. It is just a heuristic estimate. Resizing has to be supported.
                RefitNodes = new QuickList<int>(MaximumSubtrees, pool);
                //Note that we haven't rigorously guaranteed a refinement count maximum, so it's possible that the workers will need to resize the per-thread refinement candidate lists.
                for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                {
                    RefinementCandidates[i] = new QuickList<int>(estimatedRefinementCandidateCount, threadDispatcher.GetThreadMemoryPool(i));
                }

                int multithreadingLeafCountThreshold = Tree.leafCount / (threadDispatcher.ThreadCount * 2);
                if (multithreadingLeafCountThreshold < RefinementLeafCountThreshold)
                    multithreadingLeafCountThreshold = RefinementLeafCountThreshold;
                CollectNodesForMultithreadedRefit(0, multithreadingLeafCountThreshold, ref RefitNodes, RefinementLeafCountThreshold, ref RefinementCandidates[0],
                    pool, threadDispatcher.GetThreadMemoryPool(0));

                RefitNodeIndex = -1;
            }

            public unsafe void CreateRefinementJobs(BufferPool pool, int frameIndex, float refineAggressivenessScale = 1)
            {
                if (Tree.leafCount <= 2)
                {
                    //If there are 2 or less leaves, then refit/refine doesn't do anything at all.
                    //(The root node has no parent, so it does not have a bounding box, and the SAH won't change no matter how we swap the children of the root.)
                    //Avoiding this case also gives the other codepath a guarantee that it will be working with nodes with two children.
                    RefinementTargets = default;
                    return;
                }
                //Condense the set of candidates into a set of targets.
                int refinementCandidatesCount = 0;
                for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                {
                    refinementCandidatesCount += RefinementCandidates[i].Count;
                }
                Tree.GetRefineTuning(frameIndex, refinementCandidatesCount, refineAggressivenessScale, RefitCostChange,
                    out var targetRefinementCount, out var period, out var offset);
                RefinementTargets = new QuickList<int>(targetRefinementCount, pool);

                //Note that only a subset of all refinement *candidates* will become refinement *targets*.
                //We start at a semirandom offset and then skip through the set to accumulate targets.
                //The number of candidates that become targets is based on the refinement aggressiveness,
                //tuned by both user input (the scale) and on the volatility of the tree (RefitCostChange).
                var currentCandidatesIndex = 0;
                int index = offset;
                for (int i = 0; i < targetRefinementCount - 1; ++i)
                {
                    index += period;
                    //Wrap around if the index doesn't fit.
                    while (index >= RefinementCandidates[currentCandidatesIndex].Count)
                    {
                        index -= RefinementCandidates[currentCandidatesIndex].Count;
                        ++currentCandidatesIndex;
                        if (currentCandidatesIndex >= threadDispatcher.ThreadCount)
                            currentCandidatesIndex -= threadDispatcher.ThreadCount;
                    }
                    Debug.Assert(index < RefinementCandidates[currentCandidatesIndex].Count && index >= 0);
                    var nodeIndex = RefinementCandidates[currentCandidatesIndex][index];
                    Debug.Assert(Tree.Metanodes[nodeIndex].RefineFlag == 0, "Refinement target search shouldn't run into the same node twice!");
                    RefinementTargets.AddUnsafely(nodeIndex);
                    Tree.Metanodes[nodeIndex].RefineFlag = 1;
                }
                //Note that the root node is only refined if it was not picked as a target earlier.
                if (Tree.Metanodes[0].RefineFlag != 1)
                {
                    RefinementTargets.AddUnsafely(0);
                    Tree.Metanodes[0].RefineFlag = 1;
                }
                RefineIndex = -1;
            }

            public unsafe void CleanUpForRefitAndRefine(BufferPool pool)
            {
                if (Tree.leafCount <= 2)
                {
                    //If there are 2 or less leaves, then refit/refine doesn't do anything at all.
                    return;
                }
                //Note that we defer the refine flag clear until after the refinements complete. If we did it within the refine action itself, 
                //it would introduce nondeterminism by allowing refines to progress based on their order of completion.
                for (int i = 0; i < RefinementTargets.Count; ++i)
                {
                    Tree.Metanodes[RefinementTargets[i]].RefineFlag = 0;
                }

                for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                {
                    //Note the use of the thread memory pool. Each thread allocated their own memory for the list since resizes were possible.
                    RefinementCandidates[i].Dispose(threadDispatcher.GetThreadMemoryPool(i));
                }
                pool.Return(ref RefinementCandidates);
                RefitNodes.Dispose(pool);
                RefinementTargets.Dispose(pool);
                Tree = default;
                this.threadDispatcher = null;
            }

            public unsafe void RefitAndRefine(ref Tree tree, BufferPool pool, IThreadDispatcher threadDispatcher, int frameIndex,
                float refineAggressivenessScale = 1)
            {
                CreateRefitAndMarkJobs(ref tree, pool, threadDispatcher);
                threadDispatcher.DispatchWorkers(RefitAndMarkAction, RefitNodes.Count);
                CreateRefinementJobs(pool, frameIndex, refineAggressivenessScale);
                threadDispatcher.DispatchWorkers(RefineAction, RefinementTargets.Count);
                CleanUpForRefitAndRefine(pool);
            }

            unsafe void CollectNodesForMultithreadedRefit(int nodeIndex,
                int multithreadingLeafCountThreshold, ref QuickList<int> refitAndMarkTargets,
                int refinementLeafCountThreshold, ref QuickList<int> refinementCandidates, BufferPool pool, BufferPool threadPool)
            {
                ref var node = ref Tree.Nodes[nodeIndex];
                ref var metanode = ref Tree.Metanodes[nodeIndex];
                ref var children = ref node.A;
                Debug.Assert(metanode.RefineFlag == 0);
                Debug.Assert(Tree.leafCount > 2);
                for (int i = 0; i < 2; ++i)
                {
                    ref var child = ref Unsafe.Add(ref children, i);
                    if (child.Index >= 0)
                    {
                        //Each node stores how many children are involved in the multithreaded refit.
                        //This allows the postphase to climb the tree in a thread safe way.
                        ++metanode.RefineFlag;
                        if (child.LeafCount <= multithreadingLeafCountThreshold)
                        {
                            if (child.LeafCount <= refinementLeafCountThreshold)
                            {
                                //It's possible that a wavefront node is this high in the tree, so it has to be captured here because the postpass won't find it.
                                refinementCandidates.Add(child.Index, threadPool);
                                //Encoding the child index tells the thread to use RefitAndMeasure instead of RefitAndMark since this was a wavefront node.
                                refitAndMarkTargets.Add(Encode(child.Index), pool);
                            }
                            else
                            {
                                refitAndMarkTargets.Add(child.Index, pool);
                            }
                        }
                        else
                        {
                            CollectNodesForMultithreadedRefit(child.Index, multithreadingLeafCountThreshold, ref refitAndMarkTargets, refinementLeafCountThreshold, ref refinementCandidates, pool, threadPool);
                        }
                    }
                }
            }

            public unsafe void ExecuteRefitAndMarkJob(BufferPool threadPool, int workerIndex, int refitIndex)
            {
                var nodeIndex = RefitNodes[refitIndex];
                bool shouldUseMark;
                if (nodeIndex < 0)
                {
                    //Node was already marked as a wavefront. Should proceed with a RefitAndMeasure instead of RefitAndMark.
                    nodeIndex = Encode(nodeIndex);
                    shouldUseMark = false;
                }
                else
                {
                    shouldUseMark = true;
                }

                ref var node = ref Tree.Nodes[nodeIndex];
                ref var metanode = ref Tree.Metanodes[nodeIndex];
                Debug.Assert(metanode.Parent >= 0, "The root should not be marked for refit.");
                ref var parent = ref Tree.Nodes[metanode.Parent];
                ref var childInParent = ref Unsafe.Add(ref parent.A, metanode.IndexInParent);
                if (shouldUseMark)
                {
                    var costChange = Tree.RefitAndMark(ref childInParent, RefinementLeafCountThreshold, ref RefinementCandidates[workerIndex], threadPool);
                    metanode.LocalCostChange = costChange;
                }
                else
                {
                    var costChange = Tree.RefitAndMeasure(ref childInParent);
                    metanode.LocalCostChange = costChange;
                }


                //int foundLeafCount;
                //Tree.Validate(RefitNodes.Elements[refitNodeIndex], node->Parent, node->IndexInParent, ref *boundingBoxInParent, out foundLeafCount);


                //Walk up the tree.
                node = ref parent;
                metanode = ref Tree.Metanodes[metanode.Parent];
                while (true)
                {

                    if (Interlocked.Decrement(ref metanode.RefineFlag) == 0)
                    {
                        //Compute the child contributions to this node's volume change.
                        ref var children = ref node.A;
                        metanode.LocalCostChange = 0;
                        for (int i = 0; i < 2; ++i)
                        {
                            ref var child = ref Unsafe.Add(ref children, i);
                            if (child.Index >= 0)
                            {
                                ref var childMetadata = ref Tree.Metanodes[child.Index];
                                metanode.LocalCostChange += childMetadata.LocalCostChange;
                                //Clear the refine flag (unioned).
                                childMetadata.RefineFlag = 0;

                            }
                        }

                        //This thread is the last thread to visit this node, so it must handle this node.
                        //Merge all the child bounding boxes into one. 
                        if (metanode.Parent < 0)
                        {
                            //Root node.
                            //Don't bother including the root's change in volume.
                            //Refinement can't change the root's bounds, so the fact that the world got bigger or smaller
                            //doesn't really have any bearing on how much refinement should be done.
                            //We do, however, need to divide by root volume so that we get the change in cost metric rather than volume.
                            var merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
                            for (int i = 0; i < 2; ++i)
                            {
                                ref var child = ref Unsafe.Add(ref children, i);
                                BoundingBox.CreateMerged(child.Min, child.Max, merged.Min, merged.Max, out merged.Min, out merged.Max);
                            }
                            var postmetric = ComputeBoundsMetric(ref merged);
                            if (postmetric > 1e-9f)
                                RefitCostChange = metanode.LocalCostChange / postmetric;
                            else
                                RefitCostChange = 0;
                            //Clear the root's refine flag (unioned).
                            metanode.RefineFlag = 0;
                            break;
                        }
                        else
                        {
                            parent = ref Tree.Nodes[metanode.Parent];
                            childInParent = ref Unsafe.Add(ref parent.A, metanode.IndexInParent);
                            var premetric = ComputeBoundsMetric(ref childInParent.Min, ref childInParent.Max);
                            childInParent.Min = new Vector3(float.MaxValue);
                            childInParent.Max = new Vector3(float.MinValue);
                            for (int i = 0; i < 2; ++i)
                            {
                                ref var child = ref Unsafe.Add(ref children, i);
                                BoundingBox.CreateMerged(child.Min, child.Max, childInParent.Min, childInParent.Max, out childInParent.Min, out childInParent.Max);
                            }
                            var postmetric = ComputeBoundsMetric(ref childInParent.Min, ref childInParent.Max);
                            metanode.LocalCostChange += postmetric - premetric;
                            node = ref parent;
                            metanode = ref Tree.Metanodes[metanode.Parent];
                        }
                    }
                    else
                    {
                        //This thread wasn't the last to visit this node, so it should die. Some other thread will handle it later.
                        break;
                    }
                }
            }
            public unsafe void RefitAndMarkForWorker(int workerIndex)
            {
                if (RefitNodes.Count == 0)
                    return;
                //Since resizes may occur, we have to use the thread's buffer pool.
                //The main thread already created the refinement candidate list using the worker's pool.
                var threadPool = threadDispatcher.GetThreadMemoryPool(workerIndex);
                int refitIndex;
                Debug.Assert(Tree.leafCount > 2);
                while ((refitIndex = Interlocked.Increment(ref RefitNodeIndex)) < RefitNodes.Count)
                {
                    ExecuteRefitAndMarkJob(threadPool, workerIndex, refitIndex);
                }


            }

            public unsafe void ExecuteRefineJob(ref QuickList<int> subtreeReferences, ref QuickList<int> treeletInternalNodes, ref BinnedResources resources, BufferPool threadPool, int refineIndex)
            {
                Tree.BinnedRefine(RefinementTargets[refineIndex], ref subtreeReferences, MaximumSubtrees, ref treeletInternalNodes, ref resources, threadPool);
                subtreeReferences.Count = 0;
                treeletInternalNodes.Count = 0;
            }

            public unsafe void RefineForWorker(int workerIndex)
            {
                if (RefinementTargets.Count == 0)
                    return;
                var threadPool = threadDispatcher.GetThreadMemoryPool(workerIndex);
                var subtreeCountEstimate = (int)BitOperations.RoundUpToPowerOf2((uint)MaximumSubtrees);
                var subtreeReferences = new QuickList<int>(subtreeCountEstimate, threadPool);
                var treeletInternalNodes = new QuickList<int>(subtreeCountEstimate, threadPool);

                CreateBinnedResources(threadPool, MaximumSubtrees, out var buffer, out var resources);

                int refineIndex;
                while ((refineIndex = Interlocked.Increment(ref RefineIndex)) < RefinementTargets.Count)
                {
                    ExecuteRefineJob(ref subtreeReferences, ref treeletInternalNodes, ref resources, threadPool, refineIndex);
                }

                subtreeReferences.Dispose(threadPool);
                treeletInternalNodes.Dispose(threadPool);
                threadPool.Return(ref buffer);


            }
        }

        unsafe void CheckForRefinementOverlaps(int nodeIndex, ref QuickList<int> refinementTargets)
        {
            ref var node = ref Nodes[nodeIndex];
            ref var children = ref node.A;
            for (int childIndex = 0; childIndex < 2; ++childIndex)
            {
                ref var child = ref Unsafe.Add(ref children, childIndex);
                if (child.Index >= 0)
                {
                    for (int i = 0; i < refinementTargets.Count; ++i)
                    {
                        if (refinementTargets[i] == child.Index)
                            Console.WriteLine("Found a refinement target in the children of a refinement target.");
                    }

                    CheckForRefinementOverlaps(child.Index, ref refinementTargets);
                }

            }
        }

    }
}
