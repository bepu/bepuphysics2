using BepuPhysics.Constraints.Contact;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BepuPhysics.Trees;

public partial struct Tree
{
    /// <summary>
    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// </summary>
    /// <param name="refinementStartIndex">Index used to distribute refinements over multiple executions.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    internal void Refine2(ref int refinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, int rootRefinementSize, BufferPool pool)
    {
        //1. The root gets traversed by every traversal, so refinement always refines the region close to the root.
        //2. Subtrees dangling off the root region are refined incrementally over time.
        //3. It's important for refinement to be able to exchange nodes across subtrees, so the root region varies a bit between refinements.
        //   That's implemented by varying the depth that the root refinement traverses for its subtrees.

        //Identify all candidates for refinement.
        //We'll define a candidate as any node which has a total leaf count less than or equal to the subtreeRefinementSize, and its parent does not.
        var maximumRefinementCandidateCount = 2 * (LeafCount + subtreeRefinementSize - 1) / subtreeRefinementSize;
        var subtreeRefinementCandidates = new QuickList<int>(maximumRefinementCandidateCount, pool);
        var stack = new QuickList<int>(int.Max(rootRefinementSize, subtreeRefinementSize), pool);
        var rootRefinementSubtrees = new QuickList<NodeChild>(rootRefinementSize, pool);
        var rootRefinementNodeIndices = new QuickList<int>(rootRefinementSize, pool);
        stack.AllocateUnsafely() = 0;
        while (stack.TryPop(out var nodeToVisit))
        {
            rootRefinementNodeIndices.AllocateUnsafely() = nodeToVisit;
            ref var node = ref Nodes[nodeToVisit];
            if (node.B.Index < 0)
            {
                //Root refinement *may* encounter leaves before subtree refinement candidates. They have to be included in the root's refinement.
                rootRefinementSubtrees.AllocateUnsafely() = node.B;
            }
            else
            {
                //Note that refinement candidates are not pushed onto the stack, so we don't need to check to see if the parent has more than subtreeRefinementSize leaves-
                //it definitely does, or else it would have been the refinement candidate. Or it's the root.
                //Note the order of stack pushes: if the tree is in DFS order, we want to next visit child A, so push it second.
                if (node.B.LeafCount <= subtreeRefinementSize)
                    subtreeRefinementCandidates.AllocateUnsafely() = node.B.Index;
                else
                    stack.AllocateUnsafely() = node.B.Index;
            }
            if (node.A.Index < 0)
            {
                rootRefinementSubtrees.AllocateUnsafely() = node.A;
            }
            else
            {
                if (node.A.LeafCount <= subtreeRefinementSize)
                    subtreeRefinementCandidates.AllocateUnsafely() = node.A.Index;
                else
                    stack.AllocateUnsafely() = node.A.Index;
            }
        }

        //There's no guarantee we have enough actual subtrees to serve the request.
        var effectiveSubtreeRefinementCount = int.Min(subtreeRefinementCount, subtreeRefinementCandidates.Count);
        //Refine the desired number of subtrees contiguously, starting at the refinement tracker index and wrapping.
        refinementStartIndex %= subtreeRefinementCandidates.Count;
        var subtreeRefinementTargets = new QuickList<int>(effectiveSubtreeRefinementCount, pool);
        for (int i = 0; i < effectiveSubtreeRefinementCount; ++i)
        {
            var candidate = subtreeRefinementCandidates[refinementStartIndex];
            subtreeRefinementTargets.AllocateUnsafely() = candidate;
            //Note that refinement targets are *also* subtrees for the root refinement.
            //Add the NodeChild associated with the refinement target to the root refinement list.
            ref var metanode = ref Metanodes[candidate];
            if (metanode.Parent >= 0)
            {
                ref var childOfParent = ref Unsafe.Add(ref Nodes[metanode.Parent].A, metanode.IndexInParent);
                rootRefinementSubtrees.AllocateUnsafely() = childOfParent;
            }
            ++refinementStartIndex;
            if (refinementStartIndex >= subtreeRefinementCandidates.Count)
                refinementStartIndex -= subtreeRefinementCandidates.Count;
        }

        //We now have the set of subtree refinement targets, plus an incomplete collection of root refinement nodes.
        //In multithreaded-land, we could kick off jobs for every subtree refinement right now and continue on with the root refinement work.
        //Root refinement is currently in cache, so we'll do that now.
        //For every subtree refinement candidate that was *not* selected, we should continue to traverse.
        var remainingRootSubtreesRequired = rootRefinementSize - rootRefinementSubtrees.Count;
        var traversalRestartCount = subtreeRefinementCandidates.Count - effectiveSubtreeRefinementCount;
        if (traversalRestartCount > 0)
        {
            //Distribute the remaining root subtrees required over the set of non-target refinement candidates.
            var baseAmountPerCandidate = remainingRootSubtreesRequired / traversalRestartCount;
            var remainder = remainingRootSubtreesRequired - baseAmountPerCandidate * traversalRestartCount;
            //Start at the end of the selected region of refinement candidates- we updated the refinementStartIndex earlier, so it's in the right spot.
            var refinementCandidateIndex = refinementStartIndex;
            int largestRestartedTraversalLeafCount = 0;
            var traversalRestarts = new QuickList<(int Index, int MaximumLeafCount)>(traversalRestartCount, pool);
            var subtreeSurplus = 0;
            for (int i = 0; i < traversalRestartCount; ++i)
            {
                var targetAmountForCandidate = baseAmountPerCandidate + subtreeSurplus;
                if (i < remainder) ++targetAmountForCandidate;
                var candidate = subtreeRefinementCandidates[refinementCandidateIndex];
                ref var candidateNode = ref Nodes[candidate];
                //The number of subtrees we can actually collect from this candidate is limited to the number of leaves it has.
                //There's no guarantee the tree is perfectly balanced, so if we wanted to guarantee the number of root refinement subtrees,
                //we'd have to redistribute any 'extra' we accumulate (due to a lack of local leaves) to other candidates.
                //We do make a very lazy attempt at redistributing: if we can't fit the full amount, we'll carry over the rest to the next candidate.
                //If we can't allocate all the budget, shrugohwell. Don't really want to iterate.
                var candidateLeafCount = candidateNode.A.LeafCount + candidateNode.B.LeafCount;
                var amountForCandidate = int.Min(targetAmountForCandidate, candidateLeafCount);
                largestRestartedTraversalLeafCount = int.Max(largestRestartedTraversalLeafCount, amountForCandidate);
                subtreeSurplus += targetAmountForCandidate - amountForCandidate;
                traversalRestarts.AllocateUnsafely() = (candidate, amountForCandidate);
                ++refinementCandidateIndex;
                if (refinementCandidateIndex >= subtreeRefinementCandidates.Count)
                    refinementCandidateIndex -= subtreeRefinementCandidates.Count;
            }
            //Now we have all the traversal restart locations for the root refinement subtree collection: use them!
            //TODO: While we created an intermediate "traversalRestarts" list for ease of generalizing to multiple threads, everything could have just
            //been shoved directly into a traversal restart stack instead.
            var traversalRestartStack = new QuickList<(int nodeToVisit, int maximumLeafCount)>(largestRestartedTraversalLeafCount, pool);
            for (int i = 0; i < traversalRestarts.Count; ++i)
            {
                ref var traversalRestart = ref traversalRestarts[i];
                Debug.Assert(stack.Count == 0 && stack.Span.Length >= traversalRestart.MaximumLeafCount);
                traversalRestartStack.AllocateUnsafely() = traversalRestart;
                while (traversalRestartStack.TryPop(out var entry))
                {
                    ref var node = ref Nodes[entry.nodeToVisit];
                    rootRefinementNodeIndices.AllocateUnsafely() = entry.nodeToVisit;
                    var maximumLeafCount = entry.maximumLeafCount;
                    var aLeafCount = int.Min(node.A.LeafCount, maximumLeafCount / 2);
                    var bLeafCount = maximumLeafCount - aLeafCount;
                    Debug.Assert(bLeafCount <= node.B.LeafCount, "The node visited in this traversal should never see a target leaf count that exceeds what could fit in the children.");
                    Debug.Assert(aLeafCount > 0 && bLeafCount > 0, "Splitting subtree budget between children should never yield zero sized subtrees.");
                    //B pushed first so A is popped first; some degree of consistent local DFS ordering.
                    if (bLeafCount > 1)
                        traversalRestartStack.AllocateUnsafely() = (node.B.Index, bLeafCount);
                    else
                        rootRefinementSubtrees.AllocateUnsafely() = node.B;
                    if (aLeafCount > 1)
                        traversalRestartStack.AllocateUnsafely() = (node.A.Index, aLeafCount);
                    else
                        rootRefinementSubtrees.AllocateUnsafely() = node.A;
                }
            }
            traversalRestarts.Dispose(pool);
            traversalRestartStack.Dispose(pool);
        }

        //We now have the set of root refinement subtrees. Root refine!
        //TODO: The nodes collected during the root refinement are not ordered, so may destroy existing cache coherency. Sorting it would help.
        Debug.Assert(rootRefinementNodeIndices.Count == rootRefinementSubtrees.Count - 1);
        var refinementNodesAllocation = new Buffer<Node>(int.Max(rootRefinementNodeIndices.Count, subtreeRefinementSize), pool);
        var refinementMetanodesAllocation = new Buffer<Metanode>(refinementNodesAllocation.Length, pool);

        var rootRefinementNodes = refinementNodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        var rootRefinementMetanodes = refinementMetanodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        //TODO: Need to use the BinnedBuildNode path with TLeaves = LeavesHandledInPostpass.
        BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, rootRefinementMetanodes, default, null, pool);
        ReifyRefinement(rootRefinementNodeIndices, rootRefinementNodes, rootRefinementMetanodes);

        //Root refine is done; execute all the subtree refinements.
        var subtreeRefinementNodeIndices = new QuickList<int>(subtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(subtreeRefinementSize, pool);
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            //Accumulate nodes and leaves with a prepass.
            stack.AllocateUnsafely() = subtreeRefinementTargets[i];
            while (stack.TryPop(out var nodeToVisit))
            {
                ref var node = ref Nodes[nodeToVisit];
                subtreeRefinementNodeIndices.AllocateUnsafely() = nodeToVisit;
                if (node.B.Index >= 0)
                    stack.AllocateUnsafely() = node.B.Index;
                else
                    subtreeRefinementLeaves.AllocateUnsafely() = node.B;
                if (node.A.Index >= 0)
                    stack.AllocateUnsafely() = node.A.Index;
                else
                    subtreeRefinementLeaves.AllocateUnsafely() = node.A;
            }


            var refinementNodes = refinementNodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
            var refinementMetanodes = refinementMetanodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
            //TODO: Need to use the BinnedBuildNode path with TLeaves = LeavesHandledInPostpass.
            BinnedBuild(subtreeRefinementLeaves, refinementNodes, refinementMetanodes, default, null, pool);
            ReifyRefinement(subtreeRefinementNodeIndices, refinementNodes, refinementMetanodes);
        }
        subtreeRefinementNodeIndices.Dispose(pool);
        subtreeRefinementLeaves.Dispose(pool);
        refinementNodesAllocation.Dispose(pool);
        refinementMetanodesAllocation.Dispose(pool);
    }

    void ReifyRefinement(QuickList<int> refinementNodeIndices, Buffer<Node> refinementNodes, Buffer<Metanode> refinementMetanodes)
    {
        for (int i = 0; i < refinementNodeIndices.Count; ++i)
        {
            //refinementNodeIndices maps "refinement index space" to "real index space"; we can use it to update child pointers to the real locations.
            var realNodeIndex = refinementNodeIndices[i];
            ref var refinedNode = ref refinementNodes[i];
            ref var refinedMetanode = ref refinementMetanodes[i];
            //Map child indices, and update leaf references.
            if (refinedNode.A.Index >= 0)
                refinedNode.A.Index = refinementNodeIndices[refinedNode.A.Index];
            else
                Leaves[Encode(refinedNode.A.Index)] = new Leaf(realNodeIndex, 0);
            if (refinedNode.B.Index >= 0)
                refinedNode.B.Index = refinementNodeIndices[refinedNode.B.Index];
            else
                Leaves[Encode(refinedNode.B.Index)] = new Leaf(realNodeIndex, 1);
            if (refinedMetanode.Parent >= 0)
                refinedMetanode.Parent = refinementNodeIndices[refinedMetanode.Parent];
            Nodes[realNodeIndex] = refinedNode;
            Metanodes[realNodeIndex] = refinedMetanode;
        }
    }
}
