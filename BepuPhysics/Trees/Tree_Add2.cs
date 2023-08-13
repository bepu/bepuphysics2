using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Collections.Generic;

namespace BepuPhysics.Trees;

partial struct Tree
{

    /// <summary>
    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    /// </summary>
    /// <param name="bounds">Extents of the leaf bounds.</param>
    /// <param name="pool">Resource pool to use if resizing is required.</param>
    /// <returns>Index of the leaf allocated in the tree's leaf array.</returns>
    public unsafe int Add2(BoundingBox bounds, BufferPool pool)
    {
        //The rest of the function assumes we have sufficient room. We don't want to deal with invalidated pointers mid-add.
        if (Leaves.Length == LeafCount)
        {
            //Note that, while we add 1, the underlying pool will request the next higher power of 2 in bytes that can hold it.
            //Since we're already at capacity, that will be ~double the size.
            Resize(pool, LeafCount + 1);
        }

        if (LeafCount < 2)
        {
            //The root is partial.
            ref var leafChild = ref Unsafe.Add(ref Nodes[0].A, LeafCount);
            leafChild = Unsafe.As<BoundingBox, NodeChild>(ref bounds);
            var leafIndex = AddLeaf(0, LeafCount);
            leafChild.Index = Encode(leafIndex);
            leafChild.LeafCount = 1;
            return leafIndex;
        }

        //The tree is complete; traverse to find the best place to insert the leaf.

        int nodeIndex = 0;
        var bounds4 = Unsafe.As<BoundingBox, BoundingBox4>(ref bounds);
        while (true)
        {
            ref var node = ref Nodes[nodeIndex];
            //Choose whichever child requires less bounds expansion. If they're tied, choose the one with the least leaf count.
            BoundingBox.CreateMergedUnsafe(bounds4, node.A, out var mergedA);
            BoundingBox.CreateMergedUnsafe(bounds4, node.B, out var mergedB);
            var boundsIncreaseA = ComputeBoundsMetric(mergedA) - ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref node.A));
            var boundsIncreaseB = ComputeBoundsMetric(mergedB) - ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref node.B));
            var useA = boundsIncreaseA == boundsIncreaseB ? node.A.LeafCount < node.B.LeafCount : boundsIncreaseA < boundsIncreaseB;
            ref var merged = ref Unsafe.As<BoundingBox4, NodeChild>(ref useA ? ref mergedA : ref mergedB);
            ref var chosenChild = ref useA ? ref node.A : ref node.B;
            if (chosenChild.LeafCount == 1)
            {
                //The merge target is a leaf. We'll need a new internal node.
                var newNodeIndex = AllocateNode();
                ref var newNode = ref Nodes[newNodeIndex];
                ref var newMetanode = ref Metanodes[newNodeIndex];
                //Initialize the metanode of the new node.
                newMetanode.Parent = nodeIndex;
                newMetanode.IndexInParent = useA ? 0 : 1;
                //Create the new child for the inserted leaf.
                newNode.A = Unsafe.As<BoundingBox, NodeChild>(ref bounds);
                newNode.A.LeafCount = 1;
                var newLeafIndex = AddLeaf(newNodeIndex, 0);
                newNode.A.Index = Encode(newLeafIndex);
                //Move the leaf that used to be in the parent down into its new slot.
                newNode.B = chosenChild;
                //Update the moved leaf's location in the leaves. (Note that AddLeaf handled the leaves for the just-inserted leaf.)
                Leaves[Encode(chosenChild.Index)] = new Leaf(newNodeIndex, 1);
                //Update the parent's child reference to point to the new node. Note that the chosenChild still points to the slot we want.
                chosenChild = merged;
                chosenChild.LeafCount = 2;
                chosenChild.Index = newNodeIndex;
                return newLeafIndex;
            }
            else
            {
                //Just traversing into an internal node. (This could be microoptimized a wee bit. Similar above.)
                var index = chosenChild.Index;
                var leafCount = chosenChild.LeafCount + 1;
                chosenChild = merged;
                chosenChild.Index = index;
                chosenChild.LeafCount = leafCount;
                nodeIndex = index;
            }

        }
    }


    /// <summary>
    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    /// </summary>
    /// <param name="bounds">Extents of the leaf bounds.</param>
    /// <param name="pool">Resource pool to use if resizing is required.</param>
    /// <returns>Index of the leaf allocated in the tree's leaf array.</returns>
    public unsafe int Add3(BoundingBox bounds, BufferPool pool)
    {
        //The rest of the function assumes we have sufficient room. We don't want to deal with invalidated pointers mid-add.
        if (Leaves.Length == LeafCount)
        {
            //Note that, while we add 1, the underlying pool will request the next higher power of 2 in bytes that can hold it.
            //Since we're already at capacity, that will be ~double the size.
            Resize(pool, LeafCount + 1);
        }

        if (LeafCount < 2)
        {
            //The root is partial.
            ref var leafChild = ref Unsafe.Add(ref Nodes[0].A, LeafCount);
            leafChild = Unsafe.As<BoundingBox, NodeChild>(ref bounds);
            var leafIndex = AddLeaf(0, LeafCount);
            leafChild.Index = Encode(leafIndex);
            leafChild.LeafCount = 1;
            return leafIndex;
        }
        else
        {
            //The tree is complete; traverse to find the best place to insert the leaf and collect subtrees on the way.
            var estimatedMaximumTraversalDepth = int.Log2(LeafCount + 1) * 4;
            var subtrees = new QuickList<NodeChild>(estimatedMaximumTraversalDepth, pool);
            var nodeIndices = new QuickList<int>(estimatedMaximumTraversalDepth, pool);

            var leafIndex = AllocateLeaf();
            ref var leafSubtree = ref subtrees.AllocateUnsafely();
            leafSubtree = Unsafe.As<BoundingBox, NodeChild>(ref bounds);
            leafSubtree.Index = Encode(leafIndex);
            leafSubtree.LeafCount = 1;

            int nodeIndex = 0;
            var bounds4 = Unsafe.As<BoundingBox, BoundingBox4>(ref bounds);
            while (true)
            {
                nodeIndices.Allocate(pool) = nodeIndex;
                ref var node = ref Nodes[nodeIndex];
                //Choose whichever child requires less bounds expansion. If they're tied, choose the one with the least leaf count.
                BoundingBox.CreateMergedUnsafe(bounds4, node.A, out var mergedA);
                BoundingBox.CreateMergedUnsafe(bounds4, node.B, out var mergedB);
                var boundsIncreaseA = ComputeBoundsMetric(mergedA) - ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref node.A));
                var boundsIncreaseB = ComputeBoundsMetric(mergedB) - ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref node.B));
                var useA = boundsIncreaseA == boundsIncreaseB ? node.A.LeafCount < node.B.LeafCount : boundsIncreaseA < boundsIncreaseB;
                //Push the subtree that we didn't choose into the subtrees set for refinement.
                ref var newSubtree = ref subtrees.Allocate(pool);
                newSubtree = useA ? node.B : node.A;
                //ReifyRootRefinement needs a way to distinguish between node indices pointing at *within refinement* nodes, versus original subtrees.
                //We distinguish it here (as we do in the refinement operation) by including a flag in the upper bits of the index for internal nodes.
                if (newSubtree.Index >= 0)
                    newSubtree.Index |= flagForRootRefinementSubtree;

                ref var merged = ref Unsafe.As<BoundingBox4, NodeChild>(ref useA ? ref mergedA : ref mergedB);
                ref var chosenChild = ref useA ? ref node.A : ref node.B;
                if (chosenChild.LeafCount == 1)
                {
                    //Bottomed out.
                    subtrees.Allocate(pool) = useA ? node.A : node.B;
                    break;
                }
                else
                {
                    //Just traversing into an internal node.
                    nodeIndex = chosenChild.Index;
                }
            }
            //Once the tree has at least 2 leaves, it's guaranteed that the node count will be leafCount - 1, so there's going to be a new node.
            nodeIndices.Allocate(pool) = AllocateNode();

            //We can now run a binned build over all the touched nodes.
            //Note that we ignore metanodes for the binned build; we reify the results in a postpass.
            var nodes = new Buffer<Node>(nodeIndices.Count, pool);
            BinnedBuild(subtrees.Span.Slice(subtrees.Count), nodes, default, default, pool);

            ReifyRootRefinement(0, nodeIndices.Count, nodeIndices, nodes, this);

            nodeIndices.Dispose(pool);
            subtrees.Dispose(pool);
            nodes.Dispose(pool);


            return leafIndex;
        }
    }
}