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


    void FindSubtreeRefinementTargets(int nodeIndex, int leftLeafCount, int subtreeRefinementSize, int targetSubtreeRefinementCount, ref int startIndex, int endIndex, ref QuickList<int> refinementTargets)
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
                FindSubtreeRefinementTargets(node.A.Index, leftLeafCount, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, endIndex, ref refinementTargets);
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
                FindSubtreeRefinementTargets(node.B.Index, leftLeafCount + node.A.LeafCount, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, endIndex, ref refinementTargets);
            }
        }
    }
    void FindSubtreeRefinementTargets(int subtreeRefinementSize, int targetSubtreeRefinementCount, ref int startIndex, ref QuickList<int> refinementTargets)
    {
        var initialStart = startIndex;
        FindSubtreeRefinementTargets(0, 0, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, LeafCount, ref refinementTargets);
        if (startIndex >= LeafCount && refinementTargets.Count < targetSubtreeRefinementCount)
        {
            //Hit the end of the tree. Reset.
            startIndex = 0;
            var remainingLeaves = LeafCount - initialStart;
            FindSubtreeRefinementTargets(0, 0, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, remainingLeaves, ref refinementTargets);
        }
    }



    bool IsNodeChildSubtreeRefinementTarget(Buffer<Vector<int>> subtreeRefinementBundles, in NodeChild child, int parentTotalLeafCount, int subtreeRefinementSize)
    {
        //First check if it *could* be one by checking the leaf count threshold.
        if (child.LeafCount <= subtreeRefinementSize && parentTotalLeafCount > subtreeRefinementSize)
        {
            //It may be a subtree refinement. Do a deeper test!
            var search = new Vector<int>(child.Index);
            for (int i = 0; i < subtreeRefinementBundles.Length; ++i)
            {
                if (Vector.EqualsAny(search, subtreeRefinementBundles[i]))
                    return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Checks if a child should be a subtree in the root refinement. If so, it's added to the list. Otherwise, it's pushed onto the stack.
    /// </summary>
    private void TryPushChildForRootRefinement(
         int subtreeRefinementSize, Buffer<Vector<int>> subtreeRefinementRootBundles, int nodeTotalLeafCount, int subtreeBudget, in NodeChild child, ref QuickList<(int nodeIndex, int subtreeBudget)> stack, ref QuickList<NodeChild> rootRefinementSubtrees)
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
            if (IsNodeChildSubtreeRefinementTarget(subtreeRefinementRootBundles, child, nodeTotalLeafCount, subtreeRefinementSize))
            {
                //Yup!
                ref var allocatedChild = ref rootRefinementSubtrees.AllocateUnsafely();
                allocatedChild = child;
                //Internal nodes used as subtrees by the root refinement are flagged so that the reification process knows to stop.
                Debug.Assert(allocatedChild.Index < flagForRootRefinementSubtree, "The use of an upper index bit as flag means the binned refiner cannot handle trees with billions of children.");
                allocatedChild.Index |= flagForRootRefinementSubtree;
            }
            else
            {
                //Not a subtree refinement, and we know we have budget remaining.
                stack.AllocateUnsafely() = (child.Index, subtreeBudget);
            }
        }
    }

    unsafe void CollectSubtreesForRootRefinement(int rootRefinementSize, int subtreeRefinementSize, BufferPool pool, in QuickList<int> subtreeRefinementTargets, ref QuickList<int> rootRefinementNodeIndices, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        var rootStack = new QuickList<(int nodeIndex, int subtreeBudget)>(rootRefinementSize, pool);
        rootStack.AllocateUnsafely() = (0, rootRefinementSize);
        var subtreeRefinementTargetBundles = new Buffer<Vector<int>>(subtreeRefinementTargets.Span.Memory, BundleIndexing.GetBundleCount(subtreeRefinementTargets.Count));
        while (rootStack.TryPop(out var nodeToVisit))
        {
            rootRefinementNodeIndices.AllocateUnsafely() = nodeToVisit.nodeIndex;
            ref var node = ref Nodes[nodeToVisit.nodeIndex];
            var nodeTotalLeafCount = node.A.LeafCount + node.B.LeafCount;
            Debug.Assert(nodeToVisit.subtreeBudget <= nodeTotalLeafCount);
            var lowerSubtreeBudget = int.Min((nodeToVisit.subtreeBudget + 1) / 2, int.Min(node.A.LeafCount, node.B.LeafCount));
            var higherSubtreeBudget = nodeToVisit.subtreeBudget - lowerSubtreeBudget;
            var useSmallerForA = lowerSubtreeBudget == node.A.LeafCount;
            var aSubtreeBudget = useSmallerForA ? lowerSubtreeBudget : higherSubtreeBudget;
            var bSubtreeBudget = useSmallerForA ? higherSubtreeBudget : lowerSubtreeBudget;

            TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargetBundles, nodeTotalLeafCount, bSubtreeBudget, node.B, ref rootStack, ref rootRefinementSubtrees);
            TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargetBundles, nodeTotalLeafCount, aSubtreeBudget, node.A, ref rootStack, ref rootRefinementSubtrees);
        }
        rootStack.Dispose(pool);
    }

    void CollectSubtreesForSubtreeRefinement(int refinementTargetRootNodeIndex, Buffer<int> subtreeStackBuffer, ref QuickList<int> subtreeRefinementNodeIndices, ref QuickList<NodeChild> subtreeRefinementLeaves)
    {
        Debug.Assert(subtreeRefinementLeaves.Count == 0 && subtreeRefinementNodeIndices.Count == 0);
        var subtreeStack = new QuickList<int>(subtreeStackBuffer);
        subtreeStack.AllocateUnsafely() = refinementTargetRootNodeIndex;
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
    public void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool)
    {
        //No point refining anything with two leaves. This condition also avoids having to special case for an incomplete root node.
        if (LeafCount <= 2)
            return;
        //Clamp refinement sizes to avoid pointless overallocations when the user supplies odd inputs.
        rootRefinementSize = int.Min(rootRefinementSize, LeafCount);
        subtreeRefinementSize = int.Min(subtreeRefinementSize, LeafCount);
        //We used a vectorized containment test later, so make sure to pad out the refinement target list.
        var subtreeRefinementCapacity = BundleIndexing.GetBundleCount(subtreeRefinementCount) * Vector<int>.Count;
        var subtreeRefinementTargets = new QuickList<int>(subtreeRefinementCapacity, pool);
        FindSubtreeRefinementTargets(subtreeRefinementSize, subtreeRefinementCount, ref subtreeRefinementStartIndex, ref subtreeRefinementTargets);
        //Fill the trailing slots in the list with -1 to avoid matches.
        ((Span<int>)subtreeRefinementTargets.Span)[subtreeRefinementTargets.Count..].Fill(-1);

        //We now know which nodes are the roots of subtree refinements; the root refinement can avoid traversing through them.
        var rootRefinementSubtrees = new QuickList<NodeChild>(rootRefinementSize, pool);
        var rootRefinementNodeIndices = new QuickList<int>(rootRefinementSize, pool);
        CollectSubtreesForRootRefinement(rootRefinementSize, subtreeRefinementSize, pool, subtreeRefinementTargets, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);

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


        var subtreeRefinementNodeIndices = new QuickList<int>(subtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(subtreeRefinementSize, pool);
        var subtreeStackBuffer = new Buffer<int>(subtreeRefinementSize, pool);
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            //Accumulate nodes and leaves with a prepass.
            CollectSubtreesForSubtreeRefinement(subtreeRefinementTargets[i], subtreeStackBuffer, ref subtreeRefinementNodeIndices, ref subtreeRefinementLeaves);

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
        subtreeStackBuffer.Dispose(pool);
        refinementNodesAllocation.Dispose(pool);
        refinementMetanodesAllocation.Dispose(pool);
    }

}
