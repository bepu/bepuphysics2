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
    private struct InsertShouldNotRotate { }
    private struct InsertShouldRotateTopDown { }
    private struct InsertShouldRotateBottomUp { }

    /// <summary>
    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    /// </summary>
    /// <param name="bounds">Extents of the leaf bounds.</param>
    /// <param name="pool">Resource pool to use if resizing is required.</param>
    /// <returns>Index of the leaf allocated in the tree's leaf array.</returns>
    /// <remarks>This performs no incremental refinement. When acting on the same tree, it's slightly cheaper than <see cref="Add"/>, 
    /// but the quality of the tree depends on insertion order.<para/>
    /// Pathological insertion orders can result in a maximally imbalanced tree, quadratic insertion times across the full tree build, and query performance linear in the number of leaves.<para/>
    /// This is typically best reserved for cases where the insertion order is known to be randomized or otherwise conducive to building decent trees.</remarks>
    public unsafe int AddWithoutRefinement(BoundingBox bounds, BufferPool pool)
    {
        return Add<InsertShouldNotRotate>(bounds, pool);
    }

    /// <summary>
    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    /// </summary>
    /// <param name="bounds">Extents of the leaf bounds.</param>
    /// <param name="pool">Resource pool to use if resizing is required.</param>
    /// <returns>Index of the leaf allocated in the tree's leaf array.</returns>
    /// <remarks>Performs incrementally refining tree rotations down along the insertion path, unlike <see cref="AddWithoutRefinement"/>.<para/>
    /// For a given tree, this is slightly slower than <see cref="AddWithoutRefinement"/> and slightly faster than <see cref="AddWithBottomUpRefinement"/>.<para/>
    /// Trees built with repeated insertions of this kind tend to have decent quality, but slightly worse than <see cref="AddWithBottomUpRefinement"/>.</remarks>
    public unsafe int Add(BoundingBox bounds, BufferPool pool)
    {
        return Add<InsertShouldRotateTopDown>(bounds, pool);
    }

    /// <summary>
    /// Adds a leaf to the tree with the given bounding box and returns the index of the added leaf.
    /// </summary>
    /// <param name="bounds">Extents of the leaf bounds.</param>
    /// <param name="pool">Resource pool to use if resizing is required.</param>
    /// <returns>Index of the leaf allocated in the tree's leaf array.</returns>
    /// <remarks>Performs incrementally refining tree rotations up along the insertion path, unlike <see cref="AddWithoutRefinement"/>.<para/>
    /// Trees built with repeated insertions of this kind tend to have slightly better quality than <see cref="Add"/>, but it is also slightly more expensive.</remarks>
    public unsafe int AddWithBottomUpRefinement(BoundingBox bounds, BufferPool pool)
    {
        return Add<InsertShouldRotateBottomUp>(bounds, pool);
    }

    private unsafe int Add<TShouldRotate>(BoundingBox bounds, BufferPool pool) where TShouldRotate : unmanaged
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
        var newNodeIndex = AllocateNode();
        //We only ever insert into child A, and it's guaranteed to belong to a new node, so we don't have to wait to add the leaf.
        var newLeafIndex = AddLeaf(newNodeIndex, 0);
        while (true)
        {
            //Note: rotating from the top down produces a tree that's lower quality that rotating from the bottom up.
            //In context, that's fine; insertion just needs to produce a tree that isn't megatrash/stackoverflowy, and refinement will take care of the rest.
            //The advantage is that top down is a little faster.
            if (typeof(TShouldRotate) == typeof(InsertShouldRotateTopDown))
                TryRotateNode(nodeIndex);
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
                ref var newNode = ref Nodes[newNodeIndex];
                ref var newMetanode = ref Metanodes[newNodeIndex];
                //Initialize the metanode of the new node.
                newMetanode.Parent = nodeIndex;
                newMetanode.IndexInParent = useA ? 0 : 1;
                //Create the new child for the inserted leaf.
                newNode.A = Unsafe.As<BoundingBox, NodeChild>(ref bounds);
                newNode.A.LeafCount = 1;
                newNode.A.Index = Encode(newLeafIndex);
                //Move the leaf that used to be in the parent down into its new slot.
                newNode.B = chosenChild;
                //Update the moved leaf's location in the leaves. (Note that AddLeaf handled the leaves for the just-inserted leaf.)
                Leaves[Encode(chosenChild.Index)] = new Leaf(newNodeIndex, 1);
                //Update the parent's child reference to point to the new node. Note that the chosenChild still points to the slot we want.
                chosenChild = merged;
                chosenChild.LeafCount = 2;
                chosenChild.Index = newNodeIndex;
                break;
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

        if (typeof(TShouldRotate) == typeof(InsertShouldRotateBottomUp))
        {
            var parentIndex = Leaves[newLeafIndex].NodeIndex;
            while (parentIndex >= 0)
            {
                TryRotateNode(parentIndex);
                parentIndex = Metanodes[parentIndex].Parent;
            }
        }
        return newLeafIndex;
    }

    private unsafe void TryRotateNode(int rotationRootIndex)
    {
        ref var root = ref Nodes[rotationRootIndex];
        var costA = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref root.A));
        var costB = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref root.B));
        float leftRotationCostChange = 0;
        bool leftUsesA = false;
        float rightRotationCostChange = 0;
        bool rightUsesA = false;
        if (root.A.Index >= 0)
        {
            //Try a right rotation. root.B will merge with the better of A's children, while the worse of A's children will take the place of root.A.
            ref var a = ref Nodes[root.A.Index];
            BoundingBox.CreateMergedUnsafe(a.A, root.B, out var aaB);
            BoundingBox.CreateMergedUnsafe(a.B, root.B, out var abB);
            var costAAB = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref aaB));
            var costABB = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref abB));
            rightUsesA = costAAB < costABB;
            rightRotationCostChange = float.Min(costAAB, costABB) - costA;
        }
        if (root.B.Index >= 0)
        {
            //Try a left rotation. root.A will merge with the better of B's children, while the worse of B's children will take the place of root.B.
            ref var b = ref Nodes[root.B.Index];
            BoundingBox.CreateMergedUnsafe(root.A, b.A, out var baB);
            BoundingBox.CreateMergedUnsafe(root.A, b.B, out var bbB);
            var costBAB = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref baB));
            var costBBB = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref bbB));
            leftUsesA = costBAB < costBBB;
            leftRotationCostChange = float.Min(costBAB, costBBB) - costB;
        }
        if (float.Min(leftRotationCostChange, rightRotationCostChange) < 0)
        {
            //A rotation is worth it.
            if (leftRotationCostChange < rightRotationCostChange)
            {
                //Left rotation wins!
                var nodeIndexToReplace = root.B.Index;
                ref var nodeToReplace = ref Nodes[nodeIndexToReplace];
                var childToShiftUp = leftUsesA ? nodeToReplace.B : nodeToReplace.A;
                var childToShiftLeft = leftUsesA ? nodeToReplace.A : nodeToReplace.B;
                nodeToReplace.A = root.A;
                nodeToReplace.B = childToShiftLeft;
                BoundingBox.CreateMergedUnsafe(nodeToReplace.A, nodeToReplace.B, out root.A);
                root.A.Index = nodeIndexToReplace;
                root.A.LeafCount = nodeToReplace.A.LeafCount + nodeToReplace.B.LeafCount;
                root.B = childToShiftUp;
                Metanodes[nodeIndexToReplace] = new Metanode { Parent = rotationRootIndex, IndexInParent = 0 };
                if (childToShiftUp.Index < 0) Leaves[Encode(childToShiftUp.Index)] = new Leaf(rotationRootIndex, 1); else Metanodes[childToShiftUp.Index] = new Metanode { Parent = rotationRootIndex, IndexInParent = 1 };
                if (nodeToReplace.A.Index < 0) Leaves[Encode(nodeToReplace.A.Index)] = new Leaf(nodeIndexToReplace, 0); else Metanodes[nodeToReplace.A.Index] = new Metanode { Parent = nodeIndexToReplace, IndexInParent = 0 };
                if (nodeToReplace.B.Index < 0) Leaves[Encode(nodeToReplace.B.Index)] = new Leaf(nodeIndexToReplace, 1); else Metanodes[nodeToReplace.B.Index] = new Metanode { Parent = nodeIndexToReplace, IndexInParent = 1 };
            }
            else
            {
                //Right rotation wins!
                var nodeIndexToReplace = root.A.Index;
                ref var nodeToReplace = ref Nodes[nodeIndexToReplace];
                var childToShiftUp = rightUsesA ? nodeToReplace.B : nodeToReplace.A;
                var childToShiftRight = rightUsesA ? nodeToReplace.A : nodeToReplace.B;
                nodeToReplace.A = childToShiftRight;
                nodeToReplace.B = root.B;
                BoundingBox.CreateMergedUnsafe(nodeToReplace.A, nodeToReplace.B, out root.B);
                root.B.Index = nodeIndexToReplace;
                root.B.LeafCount = nodeToReplace.A.LeafCount + nodeToReplace.B.LeafCount;
                root.A = childToShiftUp;
                Metanodes[nodeIndexToReplace] = new Metanode { Parent = rotationRootIndex, IndexInParent = 1 };
                if (childToShiftUp.Index < 0) Leaves[Encode(childToShiftUp.Index)] = new Leaf(rotationRootIndex, 0); else Metanodes[childToShiftUp.Index] = new Metanode { Parent = rotationRootIndex, IndexInParent = 0 };
                if (nodeToReplace.A.Index < 0) Leaves[Encode(nodeToReplace.A.Index)] = new Leaf(nodeIndexToReplace, 0); else Metanodes[nodeToReplace.A.Index] = new Metanode { Parent = nodeIndexToReplace, IndexInParent = 0 };
                if (nodeToReplace.B.Index < 0) Leaves[Encode(nodeToReplace.B.Index)] = new Leaf(nodeIndexToReplace, 1); else Metanodes[nodeToReplace.B.Index] = new Metanode { Parent = nodeIndexToReplace, IndexInParent = 1 };
            }
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static float ComputeBoundsMetric(ref BoundingBox bounds)
    {
        return ComputeBoundsMetric(ref bounds.Min, ref bounds.Max);
    }
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static float ComputeBoundsMetric(ref Vector3 min, ref Vector3 max)
    {
        //Note that we just use the SAH. While we are primarily interested in volume queries for the purposes of collision detection, the topological difference
        //between a volume heuristic and surface area heuristic isn't huge. There is, however, one big annoying issue that volume heuristics run into:
        //all bounding boxes with one extent equal to zero have zero cost. Surface area approaches avoid this hole simply.
        var offset = max - min;
        //Note that this is merely proportional to surface area. Being scaled by a constant factor is irrelevant.
        return offset.X * offset.Y + offset.Y * offset.Z + offset.X * offset.Z;
    }
}