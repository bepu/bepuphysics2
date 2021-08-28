using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;



namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        //TODO: Note that this heuristic does not fully capture the cost of a node.
        //It assumes that traversing a node with 2 children is about the same as traversing a node with 8 children.
        //While this may be closer to true that it appears at first glance due to the very high cost of cache misses versus trivial ALU work,
        //it's probably not *identical*.
        //The builders also use this approximation.
        public unsafe float MeasureCostMetric()
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            ref var rootNode = ref Nodes[0];
            ref var rootChildren = ref rootNode.A;

            var merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < leafCount; ++i)
            {
                ref var child = ref Unsafe.Add(ref rootChildren, i);
                BoundingBox.CreateMerged(child.Min, child.Max, merged.Min, merged.Max, out merged.Min, out merged.Max);
            }
            float rootMetric = ComputeBoundsMetric(ref merged);

            const float leafCost = 1;
            const float internalNodeCost = 1;

            if (leafCount > 2)
            {
                float totalCost = 0;
                for (int i = 0; i < nodeCount; ++i)
                {
                    ref var node = ref Nodes[i];
                    ref var children = ref node.A;
                    for (int childIndex = 0; childIndex < 2; ++childIndex)
                    {
                        ref var child = ref Unsafe.Add(ref children, childIndex);
                        if (child.Index >= 0)
                        {
                            //Internal node.
                            totalCost += internalNodeCost * ComputeBoundsMetric(ref child.Min, ref child.Max);
                        }
                        else
                        {
                            //Leaf node.
                            totalCost += leafCost * ComputeBoundsMetric(ref child.Min, ref child.Max);
                        }

                    }
                }
                return totalCost / rootMetric;
            }
            return 0;

        }

        readonly unsafe void Validate(int nodeIndex, int expectedParentIndex, int expectedIndexInParent, ref Vector3 expectedMin, ref Vector3 expectedMax, out int foundLeafCount)
        {
            ref var node = ref Nodes[nodeIndex];
            ref var metanode = ref Metanodes[nodeIndex];
            if (metanode.Parent != expectedParentIndex)
                throw new Exception($"Bad parent index on node {nodeIndex}");
            if (metanode.IndexInParent != expectedIndexInParent)
                throw new Exception($"Bad index in parent on node {nodeIndex}");
            if (metanode.RefineFlag != 0)
                throw new Exception($"Nonzero refine flag on node {nodeIndex}");
            ref var children = ref node.A;
            foundLeafCount = 0;
            var badMinValue = new Vector3(float.MaxValue);
            var badMaxValue = new Vector3(float.MinValue);
            var mergedMin = badMinValue; //Note- using isolated vectors instead of actual BoundingBox here to avoid a compiler bug: https://github.com/dotnet/coreclr/issues/12950
            var mergedMax = badMaxValue;
            var childCount = Math.Min(leafCount, 2);
            for (int i = 0; i < childCount; ++i)
            {
                ref var child = ref Unsafe.Add(ref children, i);
                if (child.Min == badMinValue || child.Max == badMaxValue)
                    throw new Exception($"Node {nodeIndex} child {i} has a bad bounding box.");
                BoundingBox.CreateMerged(mergedMin, mergedMax, child.Min, child.Max, out mergedMin, out mergedMax);
                if (child.Index >= 0)
                {
                    if (child.Index >= nodeCount)
                        throw new Exception($"Implied existence of node {child} is outside of count {nodeCount}.");
                    Validate(child.Index, nodeIndex, i, ref child.Min, ref child.Max, out int childFoundLeafCount);
                    if (childFoundLeafCount != child.LeafCount)
                        throw new Exception($"Bad leaf count for child {i} of node {nodeIndex}.");
                    foundLeafCount += childFoundLeafCount;
                }
                else
                {
                    ++foundLeafCount;
                    if (child.LeafCount != 1)
                    {
                        throw new Exception($"Bad leaf count on {nodeIndex} child {i}, it's a leaf but leafCount is {child.LeafCount}.");
                    }
                    var leafIndex = Encode(child.Index);
                    if (leafIndex < 0 || leafIndex >= leafCount)
                        throw new Exception("Bad node-contained leaf index.");
                    if (Leaves[leafIndex].NodeIndex != nodeIndex || Leaves[leafIndex].ChildIndex != i)
                    {
                        throw new Exception("Mismatch between node-held leaf pointer and leaf's pointers.");
                    }
                }
            }
            if (foundLeafCount == 0 && (leafCount > 0 || expectedParentIndex >= 0))
            {
                //The only time foundLeafCount can be zero is if this is the root node in an empty tree.
                throw new Exception("Bad leaf count.");
            }
            var metric = ComputeBoundsMetric(ref mergedMin, ref mergedMax);
            if (foundLeafCount > 0 && //If there are no leaves, then calling the bounds 'bad' is silly. For this to happen, 
                (float.IsNaN(metric) || float.IsInfinity(metric)))
            {
                throw new Exception($"Bad bounds: {metric} SAH. {mergedMin}, {mergedMax}.");
            }
            if (expectedParentIndex >= 0 && //Not a root node,
                (mergedMin != expectedMin || mergedMax != expectedMax))
            {
                throw new Exception($"{nodeIndex} bounds {mergedMin}, {mergedMax}, expected ({expectedMin}, {expectedMax}).");
            }
        }

        readonly unsafe void ValidateLeafNodeIndices()
        {
            for (int i = 0; i < leafCount; ++i)
            {
                if (Leaves[i].NodeIndex < 0)
                {
                    throw new Exception($"Leaf {i} has negative node index: {Leaves[i].NodeIndex}.");
                }
                if (Leaves[i].NodeIndex >= nodeCount)
                {
                    throw new Exception($"Leaf {i} points to a node outside the node set, {Leaves[i].NodeIndex} >= {nodeCount}.");
                }
            }
        }

        readonly unsafe void ValidateLeaves()
        {
            ValidateLeafNodeIndices();

            for (int i = 0; i < leafCount; ++i)
            {
                if (Encode(Unsafe.Add(ref Nodes[Leaves[i].NodeIndex].A, Leaves[i].ChildIndex).Index) != i)
                {
                    throw new Exception($"Leaf {i} data does not agree with node about parenthood.");
                }
            }
        }

        public readonly unsafe void Validate()
        {
            if (nodeCount < 0)
            {
                throw new Exception($"Invalid negative node count of {nodeCount}");
            }
            else if (nodeCount > Nodes.Length)
            {
                throw new Exception($"Invalid node count of {nodeCount}, larger than nodes array length {Nodes.Length}.");
            }
            if (LeafCount > 0 && (Metanodes[0].Parent != -1 || Metanodes[0].IndexInParent != -1))
            {
                throw new Exception($"Invalid parent pointers on root.");
            }
            if ((nodeCount != 1 && leafCount < 2) || (nodeCount != LeafCount - 1 && leafCount >= 2))
            {
                throw new Exception($"Invalid node count versus leaf count.");
            }

            var standInBounds = new BoundingBox();

            Validate(0, -1, -1, ref standInBounds.Min, ref standInBounds.Max, out int foundLeafCount);
            if (foundLeafCount != leafCount)
                throw new Exception($"{foundLeafCount} leaves found in tree, expected {leafCount}.");

            ValidateLeaves();

        }

        readonly unsafe int ComputeMaximumDepth(ref Node node, int currentDepth)
        {
            ref var children = ref node.A;
            int maximum = currentDepth;
            int nextDepth = currentDepth + 1;
            var childCount = Math.Min(leafCount, 2);
            for (int i = 0; i < childCount; ++i)
            {
                ref var child = ref Unsafe.Add(ref children, i);
                if (child.Index >= 0)
                {
                    var candidate = ComputeMaximumDepth(ref Nodes[child.Index], nextDepth);
                    if (candidate > maximum)
                        maximum = candidate;
                }
            }
            return maximum;
        }

        public readonly unsafe int ComputeMaximumDepth()
        {
            return ComputeMaximumDepth(ref Nodes[0], 0);
        }

        readonly unsafe void MeasureCacheQuality(int nodeIndex, out int foundNodes, out float nodeScore, out int scorableNodeCount)
        {
            ref var node = ref Nodes[nodeIndex];
            ref var children = ref node.A;
            nodeScore = 0;
            scorableNodeCount = 0;
            foundNodes = 0;
            int correctlyPositionedImmediateChildren = 0;
            int immediateInternalChildren = 0;
            int expectedChildIndex = nodeIndex + 1;
            var childCount = Math.Min(leafCount, 2);
            for (int i = 0; i < childCount; ++i)
            {
                ref var child = ref Unsafe.Add(ref children, i);
                if (child.Index >= 0)
                {
                    ++immediateInternalChildren;
                    if (child.Index == expectedChildIndex)
                    {
                        ++correctlyPositionedImmediateChildren;
                    }
                    MeasureCacheQuality(child.Index, out int childFoundNodes, out float childNodeScore, out int childScorableNodes);
                    foundNodes += childFoundNodes;
                    expectedChildIndex += childFoundNodes;
                    nodeScore += childNodeScore;
                    scorableNodeCount += childScorableNodes;
                }

            }

            ++foundNodes;
            //Include this node.
            if (immediateInternalChildren > 0)
            {
                nodeScore += correctlyPositionedImmediateChildren / (float)immediateInternalChildren;
                ++scorableNodeCount;
            }
        }
        public readonly unsafe float MeasureCacheQuality()
        {
            MeasureCacheQuality(0, out int foundNodes, out float nodeScore, out int scorableNodeCount);
            return scorableNodeCount > 0 ? nodeScore / scorableNodeCount : 1;

        }

        public readonly unsafe float MeasureCacheQuality(int nodeIndex)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeCount)
                throw new ArgumentException("Measurement target index must be nonnegative and less than node count.");
            MeasureCacheQuality(nodeIndex, out int foundNodes, out float nodeScore, out int scorableNodeCount);
            return scorableNodeCount > 0 ? nodeScore / scorableNodeCount : 1;
        }

    }
}
