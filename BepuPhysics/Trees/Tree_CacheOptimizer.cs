using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees;

partial struct Tree
{
    unsafe void SwapNodes(int indexA, int indexB)
    {
        ref var a = ref Nodes[indexA];
        ref var b = ref Nodes[indexB];
        ref var metaA = ref Metanodes[indexA];
        ref var metaB = ref Metanodes[indexB];

        Helpers.Swap(ref a, ref b);
        Helpers.Swap(ref metaA, ref metaB);

        if (metaA.Parent == indexA)
        {
            //The original B's parent was A.
            //That parent has moved.
            metaA.Parent = indexB;
        }
        else if (metaB.Parent == indexB)
        {
            //The original A's parent was B.
            //That parent has moved.
            metaB.Parent = indexA;
        }
        Unsafe.Add(ref Nodes[metaA.Parent].A, metaA.IndexInParent).Index = indexA;
        Unsafe.Add(ref Nodes[metaB.Parent].A, metaB.IndexInParent).Index = indexB;


        //Update the parent pointers of the children.
        ref var children = ref a.A;
        for (int i = 0; i < 2; ++i)
        {
            ref var child = ref Unsafe.Add(ref children, i);
            if (child.Index >= 0)
            {
                Metanodes[child.Index].Parent = indexA;
            }
            else
            {
                var leafIndex = Encode(child.Index);
                Leaves[leafIndex] = new Leaf(indexA, i);
            }
        }
        children = ref b.A;
        for (int i = 0; i < 2; ++i)
        {
            ref var child = ref Unsafe.Add(ref children, i);
            if (child.Index >= 0)
            {
                Metanodes[child.Index].Parent = indexB;
            }
            else
            {
                var leafIndex = Encode(child.Index);
                Leaves[leafIndex] = new Leaf(indexB, i);
            }
        }

    }

    /// <summary>
    /// Computes the index where the given node would be located if the tree were in depth first traversal order.
    /// </summary>
    /// <param name="nodeIndex">Node index to find the location for in a depth first traversal order.</param>
    /// <returns>Target index of the node if the tree were in depth first traversal order.</returns>
    public int ComputeCacheOptimalLocation(int nodeIndex)
    {
        //We want a DFS traversal order, so walk back up to the root. Count all nodes to the left of the current node.
        int leftNodeCount = 0;
        int chainedNodeIndex = nodeIndex;
        while (true)
        {
            ref var metanode = ref Metanodes[chainedNodeIndex];
            var parent = metanode.Parent;
            if (parent < 0)
                break;
            chainedNodeIndex = parent;
            ++leftNodeCount;
            if (metanode.IndexInParent == 1)
            {
                leftNodeCount += Nodes[parent].A.LeafCount - 1;
            }
        }
        return leftNodeCount;
    }

    unsafe void CacheOptimize(int nodeIndex, ref int nextIndex)
    {
        ref var node = ref Nodes[nodeIndex];
        ref var children = ref node.A;
        for (int i = 0; i < 2; ++i)
        {
            ref var child = ref Unsafe.Add(ref children, i);
            if (child.Index >= 0)
            {
                Debug.Assert(nextIndex >= 0 && nextIndex < NodeCount,
                    "Swap target should be within the node set. If it's not, the initial node was probably not in global optimum position.");
                if (child.Index != nextIndex)
                    SwapNodes(child.Index, nextIndex);
                Debug.Assert(child.Index != nextIndex);
                ++nextIndex;
                CacheOptimize(child.Index, ref nextIndex);
            }
        }
    }

    /// <summary>
    /// Begins a cache optimization at the given node and proceeds all the way to the bottom of the tree.
    /// Requires that the targeted node is already at the global optimum position.
    /// </summary>
    /// <param name="nodeIndex">Node to begin the optimization process at.</param>
    public unsafe void CacheOptimize(int nodeIndex)
    {
        if (LeafCount <= 2)
        {
            //Don't bother cache optimizing if there are only two leaves. There's no work to be done, and it supplies a guarantee to the rest of the optimization logic
            //so that we don't have to check per-node child counts.
            return;
        }
        var targetIndex = nodeIndex + 1;

        CacheOptimize(nodeIndex, ref targetIndex);
    }

    private unsafe void CacheOptimizedLimitedSubtreeInternal(int sourceNodeIndex, int targetNodeIndex, int nodeOptimizationCount)
    {
        if (sourceNodeIndex != targetNodeIndex)
            SwapNodes(targetNodeIndex, sourceNodeIndex);
        --nodeOptimizationCount;
        if (nodeOptimizationCount == 0)
            return;
        ref var node = ref Nodes[targetNodeIndex];
        var lowerNodeCount = int.Min(node.A.LeafCount, node.B.LeafCount) - 1;
        var lowerTargetNodeCount = int.Min(lowerNodeCount, (nodeOptimizationCount + 1) / 2);
        var higherNodeCount = nodeOptimizationCount - lowerTargetNodeCount;
        var aIsSmaller = node.A.LeafCount < node.B.LeafCount;
        var nodeOptimizationCountA = aIsSmaller ? lowerTargetNodeCount : higherNodeCount;
        var nodeOptimizationCountB = aIsSmaller ? higherNodeCount : lowerTargetNodeCount;
        if (nodeOptimizationCountA > 0)
            CacheOptimizedLimitedSubtreeInternal(node.A.Index, targetNodeIndex + 1, nodeOptimizationCountA);
        if (nodeOptimizationCountB > 0)
            CacheOptimizedLimitedSubtreeInternal(node.B.Index, targetNodeIndex + node.A.LeafCount, nodeOptimizationCountB);
    }

    /// <summary>
    /// Starts a cache optimization process at the target node index and the nodeOptimizationCount closest nodes in the tree.
    /// </summary>
    /// <param name="nodeIndex">Node index to start the optimization process at.</param>
    /// <param name="nodeOptimizationCount">Number of nodes to move.</param>
    /// <remarks>This optimizer will move the targeted node index to the globally optimal location if necessary.</remarks>
    public unsafe void CacheOptimizeLimitedSubtree(int nodeIndex, int nodeOptimizationCount)
    {
        if (LeafCount <= 2)
            return;
        var targetNodeIndex = ComputeCacheOptimalLocation(nodeIndex);
        ref var originalNode = ref Nodes[nodeIndex];
        var effectiveNodeOptimizationCount = int.Min(originalNode.A.LeafCount + originalNode.B.LeafCount - 1, nodeOptimizationCount);
        CacheOptimizedLimitedSubtreeInternal(nodeIndex, targetNodeIndex, effectiveNodeOptimizationCount);
    }

    /// <summary>
    /// Puts all nodes starting from the given node index into depth first traversal order.
    /// If the count is larger than the number of nodes beneath the starting node, the optimization will stop early.
    /// </summary>
    /// <param name="startingNodeIndex">Node index to start the optimization at.</param>
    /// <param name="targetCount">Number of nodes to try to optimize.</param>
    /// <returns>Number of nodes optimized.</returns>
    public unsafe int CacheOptimizeRegion(int startingNodeIndex, int targetCount)
    {
        if (LeafCount <= 2)
            return 0;
        var targetNodeIndex = ComputeCacheOptimalLocation(startingNodeIndex);
        if (startingNodeIndex != targetNodeIndex)
            SwapNodes(targetNodeIndex, startingNodeIndex);

        ref var startNode = ref Nodes[targetNodeIndex];
        //Note minus 2: visiting the parent of the last node is sufficient to put the last node into position.
        var nodeCount = int.Min(startNode.A.LeafCount + startNode.B.LeafCount - 2, targetCount);
        for (int i = 0; i < nodeCount; ++i)
        {
            int parentIndex = targetNodeIndex + i;
            ref var node = ref Nodes[parentIndex];
            //Put both children into depth first order before continuing.
            var targetLocationA = parentIndex + 1;

            var targetLocationB = parentIndex + node.A.LeafCount;
            if (node.A.Index >= 0 && node.A.Index != targetLocationA)
                SwapNodes(node.A.Index, targetLocationA);
            if (node.B.Index >= 0 && node.B.Index != targetLocationB)
                SwapNodes(node.B.Index, targetLocationB);
        }
        return nodeCount;
    }
}
