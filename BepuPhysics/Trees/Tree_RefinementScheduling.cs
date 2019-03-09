using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {

        unsafe float RefitAndMeasure(ref NodeChild child)
        {
            var node = nodes + child.Index;

            //All nodes are guaranteed to have at least 2 children.
            Debug.Assert(leafCount >= 2);

            var premetric = ComputeBoundsMetric(ref child.Min, ref child.Max);
            float childChange = 0;
            ref var a = ref node->A;
            if (a.Index >= 0)
            {
                childChange += RefitAndMeasure(ref a);
            }
            ref var b = ref node->B;
            if (b.Index >= 0)
            {
                childChange += RefitAndMeasure(ref b);
            }
            BoundingBox.CreateMerged(a.Min, a.Max, b.Min, b.Max, out child.Min, out child.Max);

            var postmetric = ComputeBoundsMetric(ref child.Min, ref child.Max);
            return postmetric - premetric + childChange; //TODO: would clamping produce a superior result?

        }

        unsafe float RefitAndMark(ref NodeChild child, int leafCountThreshold, ref QuickList<int> refinementCandidates, BufferPool pool)
        {
            Debug.Assert(leafCountThreshold > 1);

            var node = nodes + child.Index;
            Debug.Assert(metanodes[child.Index].RefineFlag == 0);
            float childChange = 0;

            var premetric = ComputeBoundsMetric(ref child.Min, ref child.Max);
            //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
            //Add them to a list of refinement candidates.
            //Note that leaves are not included, since they can't be refinement candidates.
            ref var a = ref node->A;
            if (a.Index >= 0)
            {
                if (a.LeafCount <= leafCountThreshold)
                {
                    refinementCandidates.Add(a.Index, pool);
                    childChange += RefitAndMeasure(ref a);
                }
                else
                {
                    childChange += RefitAndMark(ref a, leafCountThreshold, ref refinementCandidates, pool);
                }
            }
            ref var b = ref node->B;
            if (b.Index >= 0)
            {
                if (b.LeafCount <= leafCountThreshold)
                {
                    refinementCandidates.Add(b.Index, pool);
                    childChange += RefitAndMeasure(ref b);
                }
                else
                {
                    childChange += RefitAndMark(ref b, leafCountThreshold, ref refinementCandidates, pool);
                }
            }

            BoundingBox.CreateMerged(a.Min, a.Max, b.Min, b.Max, out child.Min, out child.Max);


            var postmetric = ComputeBoundsMetric(ref child.Min, ref child.Max);

            return postmetric - premetric + childChange; //TODO: Would clamp provide better results?



        }

        unsafe float RefitAndMark(int leafCountThreshold, ref QuickList<int> refinementCandidates, BufferPool pool)
        {
            Debug.Assert(LeafCount > 2, "There's no reason to refit a tree with 2 or less elements. Nothing would happen.");

            var children = &nodes->A;
            float childChange = 0;
            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            for (int i = 0; i < 2; ++i)
            {
                //Note: these conditions mean the root will never be considered a wavefront node. That's acceptable;
                //it will be included regardless.
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    if (child.LeafCount <= leafCountThreshold)
                    {
                        //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
                        //Since we don't traverse into these children, there is no need to check the parent's leaf count.
                        refinementCandidates.Add(child.Index, pool);
                        childChange += RefitAndMeasure(ref child);
                    }
                    else
                    {
                        childChange += RefitAndMark(ref child, leafCountThreshold, ref refinementCandidates, pool);
                    }
                }
                BoundingBox.CreateMerged(child.Min, child.Max, merged.Min, merged.Max, out merged.Min, out merged.Max);
            }

            var postmetric = ComputeBoundsMetric(ref merged);

            //Note that the root's own change is not included.
            //This cost change is used to determine whether or not to refine.
            //Since refines are unable to change the volume of the root, there's
            //no point in including it in the volume change.
            //It does, however, normalize the child volume changes into a cost metric.
            if (postmetric >= 1e-10)
            {
                return childChange / postmetric;
            }
            return 0;
        }




        unsafe void ValidateRefineFlags(int index)
        {
            var metanode = metanodes + index;
            if (metanode->RefineFlag != 0)
                Console.WriteLine("Bad refine flag");

            var children = &nodes[index].A;
            for (int i = 0; i < 2; ++i)
            {
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    ValidateRefineFlags(child.Index);
                }
            }
        }

        void GetRefitAndMarkTuning(out int maximumSubtrees, out int estimatedRefinementCandidateCount, out int refinementLeafCountThreshold)
        {
            maximumSubtrees = (int)(Math.Sqrt(leafCount) * 3);
            estimatedRefinementCandidateCount = (leafCount * 2) / maximumSubtrees;

            refinementLeafCountThreshold = Math.Min(leafCount, maximumSubtrees);
        }

        void GetRefineTuning(int frameIndex, int refinementCandidatesCount, float refineAggressivenessScale, float costChange,
            out int targetRefinementCount, out int refinementPeriod, out int refinementOffset)
        {
            Debug.Assert(!float.IsNaN(costChange) && !float.IsInfinity(costChange),
            "If the change in heuristic cost hits invalid values, it's likely that there are invalid poses or velocities. A bugged velocity input or constraint triggering an explosion is likely.");
            var refineAggressiveness = Math.Max(0, costChange * refineAggressivenessScale);
            float refinePortion = Math.Min(1, refineAggressiveness * 0.25f);

            var targetRefinementScale = Math.Min(nodeCount, Math.Max(2, (float)Math.Ceiling(refinementCandidatesCount * 0.03f)) + refinementCandidatesCount * refinePortion);
            //Note that the refinementCandidatesCount is used as a maximum instead of refinementCandidates + 1 for simplicity, since there's a chance
            //that the root would already be a refinementCandidate. Doesn't really have a significant effect either way.
            refinementPeriod = Math.Max(1, (int)(refinementCandidatesCount / targetRefinementScale));
            refinementOffset = (int)((frameIndex * 236887691L + 104395303L) % Math.Max(1, refinementCandidatesCount));
            targetRefinementCount = Math.Min(refinementCandidatesCount, (int)targetRefinementScale);
        }

        public int GetCacheOptimizeTuning(int maximumSubtrees, float costChange, float cacheOptimizeAggressivenessScale)
        {
            //TODO: Using cost change as the heuristic for cache optimization isn't a great idea. They don't always or even frequently correlate.
            //The best heuristic would be directly measuring the degree of adjacency. We could do that in the refit. I'm not addressing this yet
            //because there's a good chance the cache optimization approach will change significantly (for example, refit outputting into a new tree with heuristically perfect layout).
            var cacheOptimizeAggressiveness = Math.Max(0, costChange * cacheOptimizeAggressivenessScale);
            float cacheOptimizePortion = Math.Min(1, 0.03f + 85f * (maximumSubtrees / (float)leafCount) * cacheOptimizeAggressiveness);
            //float cacheOptimizePortion = Math.Min(1, 0.03f + cacheOptimizeAggressiveness * 0.5f);
            //Console.WriteLine($"cache optimization portion: {cacheOptimizePortion}");
            return (int)Math.Ceiling(cacheOptimizePortion * nodeCount);
        }



        public unsafe void RefitAndRefine(BufferPool pool, int frameIndex, float refineAggressivenessScale = 1, float cacheOptimizeAggressivenessScale = 1)
        {
            //Don't proceed if the tree has no refitting or refinement required. This also guarantees that any nodes that do exist have two children.
            if (leafCount <= 2)
                return;
            GetRefitAndMarkTuning(out int maximumSubtrees, out int estimatedRefinementCandidateCount, out int leafCountThreshold);
            var refinementCandidates = new QuickList<int>(estimatedRefinementCandidateCount, pool);

            //Collect the refinement candidates.
            var costChange = RefitAndMark(leafCountThreshold, ref refinementCandidates, pool);


            GetRefineTuning(frameIndex, refinementCandidates.Count, refineAggressivenessScale, costChange, out int targetRefinementCount, out int period, out int offset);


            var refinementTargets = new QuickList<int>(targetRefinementCount, pool);

            int index = offset;
            for (int i = 0; i < targetRefinementCount - 1; ++i)
            {
                index += period;
                if (index >= refinementCandidates.Count)
                    index -= refinementCandidates.Count;
                refinementTargets.AddUnsafely(refinementCandidates[index]);
                Debug.Assert(metanodes[refinementCandidates[index]].RefineFlag == 0, "Refinement target search shouldn't run into the same node twice!");
                metanodes[refinementCandidates[index]].RefineFlag = 1;
            }
            refinementCandidates.Dispose(pool);
            if (metanodes->RefineFlag == 0)
            {
                refinementTargets.AddUnsafely(0);
                metanodes->RefineFlag = 1;
            }

            //Refine all marked targets.

            var subtreeReferences = new QuickList<int>(maximumSubtrees, pool);
            var treeletInternalNodes = new QuickList<int>(maximumSubtrees, pool);

            CreateBinnedResources(pool, maximumSubtrees, out var buffer, out var resources);

            for (int i = 0; i < refinementTargets.Count; ++i)
            {

                subtreeReferences.Count = 0;
                treeletInternalNodes.Count = 0;
                BinnedRefine(refinementTargets[i], ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref resources, pool);
                //TODO: Should this be moved into a post-loop? It could permit some double work, but that's not terrible.
                //It's not invalid from a multithreading perspective, either- setting the refine flag to zero is essentially an unlock.
                //If other threads don't see it updated due to cache issues, it doesn't really matter- it's not a signal or anything like that.
                metanodes[refinementTargets[i]].RefineFlag = 0;

            }

            pool.Return(ref buffer);
            subtreeReferences.Dispose(pool);
            treeletInternalNodes.Dispose(pool);
            refinementTargets.Dispose(pool);

            var cacheOptimizeCount = GetCacheOptimizeTuning(maximumSubtrees, costChange, cacheOptimizeAggressivenessScale);

            var startIndex = (int)(((long)frameIndex * cacheOptimizeCount) % nodeCount);

            //We could wrap around. But we could also not do that because it doesn't really matter!
            var end = Math.Min(NodeCount, startIndex + cacheOptimizeCount);
            for (int i = startIndex; i < end; ++i)
            {
                IncrementalCacheOptimize(i);
            }

        }




    }
}
