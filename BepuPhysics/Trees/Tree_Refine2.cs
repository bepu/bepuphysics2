using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Xml.Linq;

namespace BepuPhysics.Trees;

public partial struct Tree
{
    const int flagForRootRefinementSubtree = 1 << 30;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AppendRootRefinementInternalSubtree(ref QuickList<NodeChild> rootSubtrees, in NodeChild child)
    {
        ref var allocatedChild = ref rootSubtrees.AllocateUnsafely();
        allocatedChild = child;
        //Internal nodes used as subtrees by the root refinement are flagged.
        Debug.Assert(allocatedChild.Index < flagForRootRefinementSubtree, "The use of an upper index bit as flag means the binned refiner cannot handle trees with billions of children.");
        if (allocatedChild.Index >= 0)
            allocatedChild.Index |= flagForRootRefinementSubtree;

    }

    /// <summary>
    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// </summary>
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="targetSubtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    public void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int targetSubtreeRefinementSize, BufferPool pool)
    {
        //1. The root gets traversed by every traversal, so refinement always refines the region close to the root.
        //2. Subtrees dangling off the root region are refined incrementally over time.
        //3. It's important for refinement to be able to exchange nodes across subtrees, so the root region varies a bit between refinements.
        //   That's implemented by varying the depth that the root refinement traverses for its subtrees.

        //Identify all candidates for refinement.
        //We'll define a candidate as any node which has a total leaf count less than or equal to the subtreeRefinementSize, and its parent does not.
        var maximumRefinementCandidateCount = 2 * (LeafCount + targetSubtreeRefinementSize - 1) / targetSubtreeRefinementSize;
        var subtreeRefinementCandidates = new QuickList<int>(maximumRefinementCandidateCount, pool);
        var stack = new QuickList<int>(int.Max(rootRefinementSize, targetSubtreeRefinementSize), pool);
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
                if (node.B.LeafCount <= targetSubtreeRefinementSize)
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
                if (node.A.LeafCount <= targetSubtreeRefinementSize)
                    subtreeRefinementCandidates.AllocateUnsafely() = node.A.Index;
                else
                    stack.AllocateUnsafely() = node.A.Index;
            }
        }

        //There's no guarantee we have enough actual subtrees to serve the request.
        var effectiveSubtreeRefinementCount = int.Min(subtreeRefinementCount, subtreeRefinementCandidates.Count);
        //Refine the desired number of subtrees contiguously, starting at the refinement tracker index and wrapping.
        subtreeRefinementStartIndex %= subtreeRefinementCandidates.Count;
        var subtreeRefinementTargets = new QuickList<int>(effectiveSubtreeRefinementCount, pool);
        for (int i = 0; i < effectiveSubtreeRefinementCount; ++i)
        {
            var candidate = subtreeRefinementCandidates[subtreeRefinementStartIndex];
            Debug.Assert(Nodes[candidate].A.LeafCount + Nodes[candidate].B.LeafCount <= targetSubtreeRefinementSize);
            subtreeRefinementTargets.AllocateUnsafely() = candidate;
            //Note that refinement targets are *also* subtrees for the root refinement.
            //Add the NodeChild associated with the refinement target to the root refinement list.
            ref var metanode = ref Metanodes[candidate];
            if (metanode.Parent >= 0)
            {
                ref var childOfParent = ref Unsafe.Add(ref Nodes[metanode.Parent].A, metanode.IndexInParent);
                AppendRootRefinementInternalSubtree(ref rootRefinementSubtrees, childOfParent);
            }
            ++subtreeRefinementStartIndex;
            if (subtreeRefinementStartIndex >= subtreeRefinementCandidates.Count)
                subtreeRefinementStartIndex -= subtreeRefinementCandidates.Count;
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
            var refinementCandidateIndex = subtreeRefinementStartIndex;
            int largestRestartedTraversalLeafCount = 0;
            var traversalRestarts = new QuickList<(int Index, int MaximumLeafCount)>(traversalRestartCount, pool);
            var subtreeSurplus = 0;
            for (int i = 0; i < traversalRestartCount; ++i)
            {
                var targetAmountForCandidate = baseAmountPerCandidate;
                if (i < remainder) ++targetAmountForCandidate;
                var candidate = subtreeRefinementCandidates[refinementCandidateIndex];
                ref var candidateNode = ref Nodes[candidate];
                //The number of subtrees we can actually collect from this candidate is limited to the number of leaves it has.
                //There's no guarantee the tree is perfectly balanced, so if we wanted to guarantee the number of root refinement subtrees,
                //we'd have to redistribute any 'extra' we accumulate (due to a lack of local leaves) to other candidates.
                //We do make a very lazy attempt at redistributing: if we can't fit the full amount, we'll carry over the rest to the next candidate.
                //If we can't allocate all the budget, shrugohwell. Don't really want to iterate.
                var candidateLeafCount = candidateNode.A.LeafCount + candidateNode.B.LeafCount;
                var amountForCandidate = int.Min(targetAmountForCandidate + subtreeSurplus, candidateLeafCount);
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
                    int smallerLeafCount = int.Min(node.A.LeafCount, node.B.LeafCount);
                    var targetLeafCountForSmaller = int.Min(smallerLeafCount, (maximumLeafCount + 1) / 2);
                    var targetLeafCountForLarger = maximumLeafCount - targetLeafCountForSmaller;
                    var aIsSmaller = node.A.LeafCount < node.B.LeafCount;
                    var aLeafCount = aIsSmaller ? targetLeafCountForSmaller : targetLeafCountForLarger;
                    var bLeafCount = aIsSmaller ? targetLeafCountForLarger : targetLeafCountForSmaller;
                    Debug.Assert(bLeafCount <= node.B.LeafCount, "The node visited in this traversal should never see a target leaf count that exceeds what could fit in the children.");
                    Debug.Assert(aLeafCount > 0 && bLeafCount > 0, "Splitting subtree budget between children should never yield zero sized subtrees.");
                    //B pushed first so A is popped first; some degree of consistent local DFS ordering.
                    if (bLeafCount > 1)
                        traversalRestartStack.AllocateUnsafely() = (node.B.Index, bLeafCount);
                    else
                        AppendRootRefinementInternalSubtree(ref rootRefinementSubtrees, node.B);
                    if (aLeafCount > 1)
                        traversalRestartStack.AllocateUnsafely() = (node.A.Index, aLeafCount);
                    else
                        AppendRootRefinementInternalSubtree(ref rootRefinementSubtrees, node.A);
                }
            }
            traversalRestarts.Dispose(pool);
            traversalRestartStack.Dispose(pool);
        }
        subtreeRefinementCandidates.Dispose(pool);

        //We now have the set of root refinement subtrees. Root refine!
        //The nodes collected during the root refinement are not ordered, so may destroy existing cache coherency.
        //Sorting *may* help in some cases, but not enough to warrant it by default.
        //((Span<int>)rootRefinementNodeIndices).Sort();
        Debug.Assert(rootRefinementNodeIndices.Count == rootRefinementSubtrees.Count - 1);
        var refinementNodesAllocation = new Buffer<Node>(int.Max(rootRefinementNodeIndices.Count, targetSubtreeRefinementSize), pool);
        var refinementMetanodesAllocation = new Buffer<Metanode>(refinementNodesAllocation.Length, pool);

        //Validate();
        var rootRefinementNodes = refinementNodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        var rootRefinementMetanodes = refinementMetanodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
        BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, rootRefinementMetanodes, default, null, pool);
        ReifyRootRefinement(rootRefinementNodeIndices, rootRefinementNodes);
        rootRefinementSubtrees.Dispose(pool);
        rootRefinementNodeIndices.Dispose(pool);

        //Validate();

        //Root refine is done; execute all the subtree refinements.
        var subtreeRefinementNodeIndices = new QuickList<int>(targetSubtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(targetSubtreeRefinementSize, pool);
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            //Accumulate nodes and leaves with a prepass.
            Debug.Assert(stack.Count == 0 && subtreeRefinementNodeIndices.Count == 0);
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

            var refinementNodes = refinementNodesAllocation.Slice(0, subtreeRefinementNodeIndices.Count);
            var refinementMetanodes = refinementMetanodesAllocation.Slice(0, subtreeRefinementNodeIndices.Count);
            //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
            BinnedBuild(subtreeRefinementLeaves, refinementNodes, refinementMetanodes, default, null, pool);
            ReifyRefinement(subtreeRefinementNodeIndices, refinementNodes);

            subtreeRefinementNodeIndices.Count = 0;
            subtreeRefinementLeaves.Count = 0;
        }
        subtreeRefinementTargets.Dispose(pool);
        stack.Dispose(pool);
        subtreeRefinementNodeIndices.Dispose(pool);
        subtreeRefinementLeaves.Dispose(pool);
        refinementNodesAllocation.Dispose(pool);
        refinementMetanodesAllocation.Dispose(pool);
        //Validate();
    }

    void ReifyRootRefinementNodeChild(ref int index, ref QuickList<int> refinementNodeIndices, int realNodeIndex, int childIndexInParent)
    {
        //Root refinements mark internal subtrees with a flag in the second to last index.
        if (index < 0)
        {
            //The child is a leaf.
            Leaves[Encode(index)] = new Leaf(realNodeIndex, childIndexInParent);
        }
        else
        {
            //The child is an internal node.
            if ((uint)index < flagForRootRefinementSubtree)
            {
                //The child is an internal node that is part of the refinement; remap its index to point at the real memory location.
                index = refinementNodeIndices[index];
            }
            else
            {
                //The child is an internal node that is *not* part of the refinement: it's a subtree endpoint.
                //No remapping is required, but we do need to strip off the 'this is a subtree endpoint of the refinement' flag.
                index &= ~flagForRootRefinementSubtree;
            }
            //Just as leaves need to be updated to point at the new node state, parent pointers for internal nodes need be updated too.
            //Note that this touches memory associated with nodes that weren't included in the refinement.
            //This is only safe if the subtree refinement either occurs sequentially with root refinement, or the subtree refinement doesn't touch the subtree refinement root's metanode.
            ref var childMetanode = ref Metanodes[index];
            childMetanode.Parent = realNodeIndex;
            childMetanode.IndexInParent = childIndexInParent;
        }
    }

    void ReifyRootRefinement(QuickList<int> refinementNodeIndices, Buffer<Node> refinementNodes)
    {
        for (int i = 0; i < refinementNodeIndices.Count; ++i)
        {
            //refinementNodeIndices maps "refinement index space" to "real index space"; we can use it to update child pointers to the real locations.
            var realNodeIndex = refinementNodeIndices[i];
            ref var refinedNode = ref refinementNodes[i];
            //Map child indices, and update leaf references.
            ReifyRootRefinementNodeChild(ref refinedNode.A.Index, ref refinementNodeIndices, realNodeIndex, 0);
            ReifyRootRefinementNodeChild(ref refinedNode.B.Index, ref refinementNodeIndices, realNodeIndex, 1);
            Nodes[realNodeIndex] = refinedNode;
            Debug.Assert(Metanodes[realNodeIndex].Parent < 0 || Unsafe.Add(ref Nodes[Metanodes[realNodeIndex].Parent].A, Metanodes[realNodeIndex].IndexInParent).LeafCount == refinedNode.A.LeafCount + refinedNode.B.LeafCount);
            Debug.Assert(Metanodes[realNodeIndex].Parent < 0 || Unsafe.Add(ref Nodes[Metanodes[realNodeIndex].Parent].A, Metanodes[realNodeIndex].IndexInParent).Index == realNodeIndex);
        }
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void ReifyRefinementNodeChild(ref int index, ref QuickList<int> refinementNodeIndices, int realNodeIndex, int childIndexInParent)
    {
        //Root refinements mark internal subtrees with a flag in the second to last index.
        if (index < 0)
        {
            //The child is a leaf.
            Leaves[Encode(index)] = new Leaf(realNodeIndex, childIndexInParent);
        }
        else
        {
            //The child is an internal node that is part of the refinement; remap its index to point at the real memory location.
            index = refinementNodeIndices[index];
            //Just as leaves need to be updated to point at the new node state, parent pointers for internal nodes need be updated too.
            //Note that this touches memory associated with nodes that weren't included in the refinement.
            //This is only safe if the subtree refinement either occurs sequentially with root refinement, or the subtree refinement doesn't touch the subtree refinement root's metanode.
            ref var childMetanode = ref Metanodes[index];
            childMetanode.Parent = realNodeIndex;
            childMetanode.IndexInParent = childIndexInParent;
        }
    }

    void ReifyRefinement(QuickList<int> refinementNodeIndices, Buffer<Node> refinementNodes)
    {
        for (int i = 0; i < refinementNodeIndices.Count; ++i)
        {
            //refinementNodeIndices maps "refinement index space" to "real index space"; we can use it to update child pointers to the real locations.
            var realNodeIndex = refinementNodeIndices[i];
            ref var refinedNode = ref refinementNodes[i];
            //Map child indices, and update leaf references.
            //Root refinements mark internal subtrees with a flag in the second to last index.
            ReifyRefinementNodeChild(ref refinedNode.A.Index, ref refinementNodeIndices, realNodeIndex, 0);
            ReifyRefinementNodeChild(ref refinedNode.B.Index, ref refinementNodeIndices, realNodeIndex, 1);
            Nodes[realNodeIndex] = refinedNode;
            Debug.Assert(Metanodes[realNodeIndex].Parent < 0 || Unsafe.Add(ref Nodes[Metanodes[realNodeIndex].Parent].A, Metanodes[realNodeIndex].IndexInParent).Index == realNodeIndex);
        }
    }

    bool CheckChildForSubtreeRefinementAccumulation(int subtreeRefinementSize, int accumulatedLeftLeaves, in NodeChild child, int nextLeafCount, ref int startIndex, ref int nextNodeIndex, ref QuickList<int> refinementTargets)
    {
        if (child.Index < 0)
        {
            //This child is a leaf; it can't be a refinement target.
            startIndex = nextLeafCount;
            return true;
        }
        if (startIndex >= accumulatedLeftLeaves && child.LeafCount <= subtreeRefinementSize)
        {
            //This is the candidate!
            refinementTargets.AllocateUnsafely() = child.Index;
            startIndex = nextLeafCount;
            return true;
        }
        //The child isn't the candidate, but it is where we should go next.
        nextNodeIndex = child.Index;
        return false;
    }

    int FindSubtreeRefinementTargets(int startIndex, int subtreeRefinementSize, int targetSubtreeRefinementCount, ref QuickList<int> refinementTargets)
    {
        //The goal here is to select candidates which have two properties:
        //1. The total number of leaves to the left of the node is less than or equal to the startIndex.
        //2. The total number of leaves in the subtree below the node is less than or equal to the targetSubtreeRefinementSize.
        //We can perform this search stacklessly!
        //(You could be a little more clever with the traversal here instead of restarting at the root every time, but the targetSubtreeRefinementCount should be <32, so shrug.)
        Debug.Assert(LeafCount >= 2, "Subtree finding assumes that the root is complete.");
        Debug.Assert(refinementTargets.Span.Length >= targetSubtreeRefinementCount, "The candidates list should hold the target refinement count.");
        Debug.Assert(subtreeRefinementSize > 2, "Subtree refinements must actually cover some refineable region!");
        int traversedLeaves = 0;
        while (refinementTargets.Count < targetSubtreeRefinementCount && traversedLeaves < LeafCount)
        {
            int nextNodeIndex = 0;
            int accumulatedLeftLeaves = 0;
            var previousStart = startIndex;
            while (true)
            {
                ref var node = ref Nodes[nextNodeIndex];
                var nextLeafCount = accumulatedLeftLeaves + node.A.LeafCount;
                if (startIndex < nextLeafCount)
                {
                    //The startIndex is to the left of the midpoint. We need to consider the left child.
                    if (CheckChildForSubtreeRefinementAccumulation(subtreeRefinementSize, accumulatedLeftLeaves, node.A, nextLeafCount, ref startIndex, ref nextNodeIndex, ref refinementTargets))
                        break;
                }
                else
                {
                    accumulatedLeftLeaves = nextLeafCount;
                    nextLeafCount = accumulatedLeftLeaves + node.B.LeafCount;

                    if (startIndex < nextLeafCount)
                    {
                        //The startIndex is to the right of the midpoint. We need to consider the right child.
                        if (CheckChildForSubtreeRefinementAccumulation(subtreeRefinementSize, accumulatedLeftLeaves, node.B, nextLeafCount, ref startIndex, ref nextNodeIndex, ref refinementTargets))
                            break;
                    }
                }
            }
            var leavesTraversedInLastAttempt = startIndex - previousStart;
            traversedLeaves += leavesTraversedInLastAttempt;
        }
        return startIndex;
    }

    void FindSubtreeRefinementTargets3(int nodeIndex, int leftLeafCount, int subtreeRefinementSize, int targetSubtreeRefinementCount, ref int startIndex, int endIndex, ref QuickList<int> refinementTargets)
    {
        //If we've used up the target region already, just quit.
        if (startIndex >= endIndex || refinementTargets.Count == targetSubtreeRefinementCount)
            return;

        ref var node = ref Nodes[nodeIndex];
        var midpoint = leftLeafCount + node.A.LeafCount;
        if (startIndex < midpoint)
        {
            //Go left!
            if (node.A.LeafCount <= subtreeRefinementSize)
            {
                //This is a candidate!
                if (node.A.LeafCount > 2) //Only include subtrees if they could be meaningfully refined.
                    refinementTargets.AllocateUnsafely() = node.A.Index;
                startIndex += node.A.LeafCount;
            }
            else
            {
                //Too big to be a candidate; traverse further.
                FindSubtreeRefinementTargets3(node.A.Index, leftLeafCount, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, endIndex, ref refinementTargets);
            }
        }

        //If we've used up the target region already, just quit.
        if (startIndex >= endIndex || refinementTargets.Count == targetSubtreeRefinementCount)
            return;

        //Note that A's traversal may have modified startIndex such that B should now be traversed.
        if (startIndex >= midpoint)
        {
            //Go right!
            if (node.B.LeafCount <= subtreeRefinementSize)
            {
                //This is a candidate!
                if (node.B.LeafCount > 2) //Only include subtrees if they could be meaningfully refined.
                    refinementTargets.AllocateUnsafely() = node.B.Index;
                startIndex += node.B.LeafCount;
            }
            else
            {
                //Too big to be a candidate; traverse further.
                FindSubtreeRefinementTargets3(node.B.Index, leftLeafCount + node.A.LeafCount, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, endIndex, ref refinementTargets);
            }
        }
    }
    void FindSubtreeRefinementTargets3(int subtreeRefinementSize, int targetSubtreeRefinementCount, ref int startIndex, ref QuickList<int> refinementTargets)
    {
        var initialStart = startIndex;
        FindSubtreeRefinementTargets3(0, 0, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, LeafCount, ref refinementTargets);
        if (startIndex >= LeafCount && refinementTargets.Count < targetSubtreeRefinementCount)
        {
            //Hit the end of the tree. Reset.
            startIndex = 0;
            var remainingLeaves = LeafCount - initialStart;
            FindSubtreeRefinementTargets3(0, 0, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, remainingLeaves, ref refinementTargets);
        }
    }


    int FindSubtreeRefinementTargets2(int startIndex, int subtreeRefinementSize, int targetSubtreeRefinementCount, ref QuickList<int> refinementTargets)
    {
        //The goal here is to select candidates which have two properties:
        //1. The total number of leaves to the left of the node is less than or equal to the startIndex.
        //2. The total number of leaves in the subtree below the node is less than or equal to the targetSubtreeRefinementSize.
        //We can perform this search stacklessly!
        //Note an important detail: nodes beyond the first encountered meeting the threshold leaf count for being a candidate cannot be considered.
        //Doing so would allow very small subtrees to be picked if the "leaves counted on the left" condition is met late.
        //Instead, when encountering a node at the threshold size, we must either accept it or move on to the next node.
        Debug.Assert(LeafCount >= 2, "Subtree finding assumes that the root is complete.");
        Debug.Assert(refinementTargets.Span.Length >= targetSubtreeRefinementCount, "The candidates list should hold the target refinement count.");
        Debug.Assert(subtreeRefinementSize > 2, "Subtree refinements must actually cover some refineable region!");
        int traversedLeaves = 0;

        int nextNodeIndex = 0;
        int accumulatedLeftLeaves = 0;
        while (refinementTargets.Count < targetSubtreeRefinementCount && traversedLeaves < LeafCount)
        {
            ref var node = ref Nodes[nextNodeIndex];
            var nodeTotalLeafCount = node.A.LeafCount + node.B.LeafCount;
            //Should we go left or right?
            var leafMidpointCount = accumulatedLeftLeaves + node.A.LeafCount;
            if (startIndex >= accumulatedLeftLeaves && startIndex < leafMidpointCount)
            {
                //Go left.
                if (node.A.LeafCount <= subtreeRefinementSize)
                {
                    //A is small enough that it can be a refinement.
                    if (node.A.Index >= 0)
                        refinementTargets.AllocateUnsafely() = node.A.Index;
                    //The traversal may continue to find additional refinement targets. This will bump it over to the right child on the next iteration.
                    startIndex = leafMidpointCount;
                }
                else
                {
                    //A remains large enough that it cannot yet be a refinement.
                    nextNodeIndex = node.A.Index;
                }
            }
            else
            {
                //Go right.
                accumulatedLeftLeaves += node.A.LeafCount;
                traversedLeaves += node.A.LeafCount;
                Debug.Assert(startIndex >= leafMidpointCount && startIndex < accumulatedLeftLeaves + nodeTotalLeafCount);
                if (node.B.LeafCount <= subtreeRefinementSize)
                {
                    //B is small enough that it can be a refinement.
                    if (node.B.Index >= 0)
                        refinementTargets.AllocateUnsafely() = node.B.Index;
                    //The traversal may continue to find additional refinement targets. 
                    accumulatedLeftLeaves += node.B.LeafCount;
                    traversedLeaves += node.B.LeafCount;
                    startIndex = accumulatedLeftLeaves + node.B.LeafCount;
                    //We'll need to bump to the parent since we're done with this node- but note that this child might have been child B of the parent, and so on.
                    //Follow parent pointers until we find a node that has another child.
                    var bumpUpNodeIndex = Metanodes[nextNodeIndex].Parent;
                    var previousTotalLeafCount = nodeTotalLeafCount;
                    while (true)
                    {
                        if (bumpUpNodeIndex < 0)
                        {
                            //Hit the root. We've reached the end of the tree, apparently. Wrap around; restart the traversal.
                            startIndex = 0;
                            accumulatedLeftLeaves = 0;
                            nextNodeIndex = 0;
                            break;
                        }
                        //We need to back out an entire node's worth of leaves; this accumulator is how we track how far we've made it in the tree.
                        //If we don't back these nodes out, the tree will think it's further ahead than it should be, and it won't traverse the right child to get to the startIndex.
                        accumulatedLeftLeaves -= previousTotalLeafCount;
                        ref var bumpUpNode = ref Nodes[bumpUpNodeIndex];
                        previousTotalLeafCount = bumpUpNode.A.LeafCount + bumpUpNode.B.LeafCount;
                        ref var metanode = ref Metanodes[bumpUpNodeIndex];
                        if (metanode.IndexInParent == 0)
                        {
                            //The node is the left child of the parent, so the parent is the next node to visit.
                            nextNodeIndex = metanode.Parent;
                            break;
                        }
                        //Still need to go further.
                        bumpUpNodeIndex = metanode.Parent;

                    }
                }
                else
                {
                    //B remains large enough that it cannot yet be a refinement.
                    nextNodeIndex = node.B.Index;
                }
            }
        }
        return startIndex;
    }

    bool IsNodeChildSubtreeRefinementTarget(in QuickList<int> subtreeRefinements, in NodeChild child, int parentTotalLeafCount, int subtreeRefinementSize)
    {
        //First check if it *could* be one by checking the leaf count threshold.
        if (child.LeafCount <= subtreeRefinementSize && parentTotalLeafCount > subtreeRefinementSize)
        {
            //It may be a subtree refinement. Do a deeper test!
            var vectorCount = BundleIndexing.GetBundleCount(subtreeRefinements.Count);
            var search = new Vector<int>(child.Index);
            var vectors = subtreeRefinements.Span.As<Vector<int>>();
            for (int i = 0; i < vectorCount; ++i)
            {
                if (Vector.EqualsAny(search, vectors[i]))
                    return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Checks if a child should be a subtree in the root refinement. If so, it's added to the list. Otherwise, it's pushed onto the stack.
    /// </summary>
    /// <returns>The amount of surplus leaf budget accumulated by pushing this child. The value will be nonzero only if the child was a subtree refinement target.</returns>
    private int TryPushChildForRootRefinement(
        int subtreeRefinementSize, in QuickList<int> subtreeRefinementRoots, int nodeTotalLeafCount, int subtreeBudget, in NodeChild child, ref QuickList<(int nodeIndex, int subtreeBudget)> stack, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        //We automatically accept any child as a subtree for the refinement process if:
        //1. It's a leaf node, or
        //2. This traversal path has used up its node budget.  
        Debug.Assert(subtreeBudget >= 0);
        if (subtreeBudget == 1)
        {
            //rootRefinementSubtrees.AllocateUnsafely() = child;
            ref var allocatedChild = ref rootRefinementSubtrees.AllocateUnsafely();
            allocatedChild = child;
            allocatedChild.Index |= flagForRootRefinementSubtree;
        }
        else
        {
            //Internal node; is it a subtree refinement?
            if (IsNodeChildSubtreeRefinementTarget(subtreeRefinementRoots, child, nodeTotalLeafCount, subtreeRefinementSize))
            {
                //Yup!
                ref var allocatedChild = ref rootRefinementSubtrees.AllocateUnsafely();
                allocatedChild = child;
                //Internal nodes used as subtrees by the root refinement are flagged so that the reification process knows to stop.
                Debug.Assert(allocatedChild.Index < flagForRootRefinementSubtree, "The use of an upper index bit as flag means the binned refiner cannot handle trees with billions of children.");
                allocatedChild.Index |= flagForRootRefinementSubtree;
                //Return the budget so it can be used on other nodes that follow in the traversal. Don't really care *where* it goes.
                return subtreeBudget;
            }
            else
            {
                //Not a subtree refinement, and we know we have budget remaining.
                stack.AllocateUnsafely() = (child.Index, subtreeBudget);
            }
        }
        return 0;
    }

    /// <summary>
    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// </summary>
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="subtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    public void Refine3(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool)
    {
        //No point refining anything with two leaves. This condition also avoids having to special case for an incomplete root node.
        if (LeafCount <= 2)
            return;
        //We used a vectorized containment test later, so make sure to pad out the refinement target list.
        var subtreeRefinementCapacity = BundleIndexing.GetBundleCount(subtreeRefinementCount) * Vector<int>.Count;
        var subtreeRefinementTargets = new QuickList<int>(subtreeRefinementCapacity, pool);
        //Console.WriteLine($"Starting at {subtreeRefinementStartIndex}");
        //subtreeRefinementStartIndex = FindSubtreeRefinementTargets2(subtreeRefinementStartIndex, subtreeRefinementSize, subtreeRefinementCount, ref subtreeRefinementTargets);
        FindSubtreeRefinementTargets3(subtreeRefinementSize, subtreeRefinementCount, ref subtreeRefinementStartIndex, ref subtreeRefinementTargets);
        //Fill the trailing slots in the list with -1 to avoid matches.
        ((Span<int>)subtreeRefinementTargets.Span)[subtreeRefinementTargets.Count..].Fill(-1);

        //for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        //{
        //    Console.Write($"{Nodes[subtreeRefinementTargets[i]].A.LeafCount + Nodes[subtreeRefinementTargets[i]].B.LeafCount}, ");
        //}
        //Console.WriteLine();

        //We now know which nodes are the roots of subtree refinements; the root refinement can avoid traversing through them.
        var rootStack = new QuickList<(int nodeIndex, int subtreeBudget)>(rootRefinementSize, pool);
        var rootRefinementSubtrees = new QuickList<NodeChild>(rootRefinementSize, pool);
        var rootRefinementNodeIndices = new QuickList<int>(rootRefinementSize, pool);
        rootStack.AllocateUnsafely() = (0, rootRefinementSize);
        int surplusSubtreeBudget = 0;
        while (rootStack.TryPop(out var nodeToVisit))
        {
            rootRefinementNodeIndices.AllocateUnsafely() = nodeToVisit.nodeIndex;
            ref var node = ref Nodes[nodeToVisit.nodeIndex];
            var nodeTotalLeafCount = node.A.LeafCount + node.B.LeafCount;
            //The "budget" is just a mechanism for bottoming out the traversal at the same depth as a BFS, but during a depth first traversal.
            var effectiveNodeBudget = int.Min(nodeTotalLeafCount, nodeToVisit.subtreeBudget + surplusSubtreeBudget);
            var lowerSubtreeBudget = int.Min((effectiveNodeBudget + 1) / 2, int.Min(node.A.LeafCount, node.B.LeafCount));
            var higherSubtreeBudget = effectiveNodeBudget - lowerSubtreeBudget;
            var useSmallerForA = lowerSubtreeBudget == node.A.LeafCount;
            var aSubtreeBudget = useSmallerForA ? lowerSubtreeBudget : higherSubtreeBudget;
            var bSubtreeBudget = useSmallerForA ? higherSubtreeBudget : lowerSubtreeBudget;

            //If the effective budgets exceed the originally scheduled amount, then we've used up part of our surplus.
            surplusSubtreeBudget += nodeToVisit.subtreeBudget - (aSubtreeBudget + bSubtreeBudget);

            surplusSubtreeBudget += TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargets, nodeTotalLeafCount, bSubtreeBudget, node.B, ref rootStack, ref rootRefinementSubtrees);
            surplusSubtreeBudget += TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargets, nodeTotalLeafCount, aSubtreeBudget, node.A, ref rootStack, ref rootRefinementSubtrees);
            //surplusSubtreeBudget = 0;
        }

        //Now that we have a list of nodes to refine, we can run the root refinement.
        Debug.Assert(rootRefinementNodeIndices.Count == rootRefinementSubtrees.Count - 1);
        var refinementNodesAllocation = new Buffer<Node>(int.Max(rootRefinementNodeIndices.Count, subtreeRefinementSize), pool);
        var refinementMetanodesAllocation = new Buffer<Metanode>(refinementNodesAllocation.Length, pool);

        var rootRefinementNodes = refinementNodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        var rootRefinementMetanodes = refinementMetanodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
        BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, rootRefinementMetanodes, default, null, pool);
        ReifyRootRefinement(rootRefinementNodeIndices, rootRefinementNodes);
        rootRefinementSubtrees.Dispose(pool);
        rootRefinementNodeIndices.Dispose(pool);

        rootStack.Dispose(pool);

        var subtreeRefinementNodeIndices = new QuickList<int>(subtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(subtreeRefinementSize, pool);
        var subtreeStack = new QuickList<int>(subtreeRefinementSize, pool);
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            //Accumulate nodes and leaves with a prepass.
            Debug.Assert(subtreeStack.Count == 0 && subtreeRefinementNodeIndices.Count == 0);
            subtreeStack.AllocateUnsafely() = subtreeRefinementTargets[i];
            while (subtreeStack.TryPop(out var nodeToVisit))
            {
                ref var node = ref Nodes[nodeToVisit];
                subtreeRefinementNodeIndices.AllocateUnsafely() = nodeToVisit;
                if (node.B.Index >= 0)
                    subtreeStack.AllocateUnsafely() = node.B.Index;
                else
                    subtreeRefinementLeaves.AllocateUnsafely() = node.B;
                if (node.A.Index >= 0)
                    subtreeStack.AllocateUnsafely() = node.A.Index;
                else
                    subtreeRefinementLeaves.AllocateUnsafely() = node.A;
            }

            var refinementNodes = refinementNodesAllocation.Slice(0, subtreeRefinementNodeIndices.Count);
            var refinementMetanodes = refinementMetanodesAllocation.Slice(0, subtreeRefinementNodeIndices.Count);
            //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
            BinnedBuild(subtreeRefinementLeaves, refinementNodes, refinementMetanodes, default, null, pool);
            ReifyRefinement(subtreeRefinementNodeIndices, refinementNodes);

            subtreeRefinementNodeIndices.Count = 0;
            subtreeRefinementLeaves.Count = 0;
        }

        subtreeRefinementNodeIndices.Dispose(pool);
        subtreeRefinementLeaves.Dispose(pool);
        subtreeRefinementTargets.Dispose(pool);
        subtreeStack.Dispose(pool);
        refinementNodesAllocation.Dispose(pool);
        refinementMetanodesAllocation.Dispose(pool);
    }

}
