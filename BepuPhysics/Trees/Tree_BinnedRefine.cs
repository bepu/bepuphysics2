using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    //TODO: This type was built on assumptions which are no longer valid today; could avoid this complexity with virtually zero overhead now.
    //This will likely be revisited during the larger tree refinement revamp.
    public unsafe struct BinnedResources
    {
        //TODO:
        //Note that these preallocations will be significantly larger than L1 for a 256-leaf treelet.
        //That's not ideal. 
        //It's largely because we handle all three axes simultaneously. While that's useful to some degree for low-lanecount SIMD reasons,
        //we should look into alternative forms later. For example, gather->wide SIMD->analyze on a per axis basis.
        //Also, it's not clear that precalculating centroids is a good idea- memory for ALU is rarely wise; we already load the bounding boxes.
        public BoundingBox* BoundingBoxes;
        public int* LeafCounts;
        public int* IndexMap;
        public Vector3* Centroids;

        public SubtreeHeapEntry* SubtreeHeapEntries;
        public Node* StagingNodes;
        public int* RefineFlags;

        //The binning process requires a lot of auxiliary memory.
        //Rather than continually reallocating it with stackalloc
        //and incurring its zeroing cost (at least until that's improved),
        //create the resources at the beginning of the refinement and reuse them.

        //Subtree related reusable resources.
        public int* SubtreeBinIndicesX;
        public int* SubtreeBinIndicesY;
        public int* SubtreeBinIndicesZ;
        public int* TempIndexMap;

        public int* ALeafCountsX;
        public int* ALeafCountsY;
        public int* ALeafCountsZ;
        public BoundingBox* AMergedX;
        public BoundingBox* AMergedY;
        public BoundingBox* AMergedZ;

        //Bin-space reusable resources.
        public BoundingBox* BinBoundingBoxesX;
        public BoundingBox* BinBoundingBoxesY;
        public BoundingBox* BinBoundingBoxesZ;
        public int* BinLeafCountsX;
        public int* BinLeafCountsY;
        public int* BinLeafCountsZ;
        public int* BinSubtreeCountsX;
        public int* BinSubtreeCountsY;
        public int* BinSubtreeCountsZ;

        public int* BinStartIndices;
        public int* BinSubtreeCountsSecondPass;


    }


    partial struct Tree
    {
        const int MaximumBinCount = 64;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe byte* Suballocate(byte* Memory, ref int memoryAllocated, int byteCount)
        {
            //When creating binned resources, we suballocate from a single buffer. This is partially an artifact of the old implementation, but 
            //it would also be an awful lot of bufferPool.Take invocations. By doing it explicitly and inline, we can reduce a little overhead.
            //(And we only have to return one buffer at the end, rather than... four hundred)
            var newSize = memoryAllocated + (byteCount + 15) & ~0xF; //Maintaining 16 byte alignment.
            var toReturn = Memory + memoryAllocated;
            memoryAllocated = newSize;
            return toReturn;

        }
        public static unsafe void CreateBinnedResources(BufferPool bufferPool, int maximumSubtreeCount, out Buffer<byte> buffer, out BinnedResources resources)
        {
            //TODO: This is a holdover from the pre-BufferPool tree design. It's pretty ugly. While some preallocation is useful (there's no reason to suffer the overhead of 
            //pulling things out of the BufferPool over and over and over again), the degree to which this preallocates has a negative impact on L1 cache for subtree refines.
            int nodeCount = maximumSubtreeCount - 1;
            int bytesRequired =
                16 * (3 + 3 + 1) + sizeof(BoundingBox) * (maximumSubtreeCount + 3 * nodeCount + 3 * MaximumBinCount) +
                16 * (6 + 3 + 8) + sizeof(int) * (maximumSubtreeCount * 6 + nodeCount * 3 + MaximumBinCount * 8) +
                16 * (1) + sizeof(Vector3) * maximumSubtreeCount +
                16 * (1) + sizeof(SubtreeHeapEntry) * maximumSubtreeCount +
                16 * (1) + sizeof(Node) * nodeCount +
                16 * (1) + sizeof(int) * nodeCount;

            bufferPool.TakeAtLeast(bytesRequired, out buffer);
            var memory = buffer.Memory;
            int memoryAllocated = 0;

            resources.BoundingBoxes = (BoundingBox*)Suballocate(memory, ref memoryAllocated, sizeof(BoundingBox) * maximumSubtreeCount);
            resources.LeafCounts = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * maximumSubtreeCount);
            resources.IndexMap = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * maximumSubtreeCount);
            resources.Centroids = (Vector3*)Suballocate(memory, ref memoryAllocated, sizeof(Vector3) * maximumSubtreeCount);
            resources.SubtreeHeapEntries = (SubtreeHeapEntry*)Suballocate(memory, ref memoryAllocated, sizeof(SubtreeHeapEntry) * maximumSubtreeCount);
            resources.StagingNodes = (Node*)Suballocate(memory, ref memoryAllocated, sizeof(Node) * nodeCount);
            resources.RefineFlags = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * nodeCount);

            resources.SubtreeBinIndicesX = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * maximumSubtreeCount);
            resources.SubtreeBinIndicesY = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * maximumSubtreeCount);
            resources.SubtreeBinIndicesZ = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * maximumSubtreeCount);
            resources.TempIndexMap = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * maximumSubtreeCount);

            resources.ALeafCountsX = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * nodeCount);
            resources.ALeafCountsY = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * nodeCount);
            resources.ALeafCountsZ = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * nodeCount);
            resources.AMergedX = (BoundingBox*)Suballocate(memory, ref memoryAllocated, sizeof(BoundingBox) * nodeCount);
            resources.AMergedY = (BoundingBox*)Suballocate(memory, ref memoryAllocated, sizeof(BoundingBox) * nodeCount);
            resources.AMergedZ = (BoundingBox*)Suballocate(memory, ref memoryAllocated, sizeof(BoundingBox) * nodeCount);

            resources.BinBoundingBoxesX = (BoundingBox*)Suballocate(memory, ref memoryAllocated, sizeof(BoundingBox) * MaximumBinCount);
            resources.BinBoundingBoxesY = (BoundingBox*)Suballocate(memory, ref memoryAllocated, sizeof(BoundingBox) * MaximumBinCount);
            resources.BinBoundingBoxesZ = (BoundingBox*)Suballocate(memory, ref memoryAllocated, sizeof(BoundingBox) * MaximumBinCount);
            resources.BinLeafCountsX = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);
            resources.BinLeafCountsY = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);
            resources.BinLeafCountsZ = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsX = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsY = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsZ = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);
            resources.BinStartIndices = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsSecondPass = (int*)Suballocate(memory, ref memoryAllocated, sizeof(int) * MaximumBinCount);

            Debug.Assert(memoryAllocated <= buffer.Length, "The allocated buffer should be large enough for all the suballocations.");
        }


        unsafe void FindPartitionBinned(ref BinnedResources resources, int start, int count,
               out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            //Initialize the per-axis candidate maps.
            var localIndexMap = resources.IndexMap + start;
            BoundingBox centroidBoundingBox;
            centroidBoundingBox.Min = resources.Centroids[localIndexMap[0]];
            centroidBoundingBox.Max = centroidBoundingBox.Min;

            for (int i = 1; i < count; ++i)
            {
                var centroid = resources.Centroids + localIndexMap[i];
                centroidBoundingBox.Min = Vector3.Min(*centroid, centroidBoundingBox.Min);
                centroidBoundingBox.Max = Vector3.Max(*centroid, centroidBoundingBox.Max);
            }

            //Bin along all three axes simultaneously.
            var nullBoundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            var span = centroidBoundingBox.Max - centroidBoundingBox.Min;
            const float epsilon = 1e-12f;
            if (span.X < epsilon && span.Y < epsilon && span.Z < epsilon)
            {
                //All axes are degenerate.
                //This is the one situation in which we can end up with all objects in the same bin.
                //To stop this, just short circuit.
                splitIndex = count / 2;
                a = nullBoundingBox;
                b = nullBoundingBox;
                leafCountA = 0;
                leafCountB = 0;
                for (int i = 0; i < splitIndex; ++i)
                {
                    BoundingBox.CreateMerged(a, resources.BoundingBoxes[localIndexMap[i]], out a);
                    leafCountA += resources.LeafCounts[localIndexMap[i]];
                }
                for (int i = splitIndex; i < count; ++i)
                {
                    BoundingBox.CreateMerged(b, resources.BoundingBoxes[localIndexMap[i]], out b);
                    leafCountB += resources.LeafCounts[localIndexMap[i]];
                }
                splitIndex += start;
                return;
            }


            //There is no real value in having tons of bins when there are very few children.
            //At low counts, many of them even end up empty.
            //You can get huge speed boosts by simply dropping the bin count adaptively.
            var binCount = (int)Math.Min(MaximumBinCount, Math.Max(count * .25f, 2));

            //Take into account zero-width cases.
            //This will result in degenerate axes all being dumped into the first bin.
            var inverseBinSize = new Vector3(
                span.X > epsilon ? binCount / span.X : 0,
                span.Y > epsilon ? binCount / span.Y : 0,
                span.Z > epsilon ? binCount / span.Z : 0);
            //inverseBinSize = new Vector3(inverseBinSize.X, inverseBinSize.Y, inverseBinSize.Z);

            //If the span along an axis is too small, just ignore it.
            var maximumBinIndex = new Vector3(binCount - 1);

            //Initialize bin information.
            for (int i = 0; i < binCount; ++i)
            {
                resources.BinBoundingBoxesX[i] = nullBoundingBox;
                resources.BinBoundingBoxesY[i] = nullBoundingBox;
                resources.BinBoundingBoxesZ[i] = nullBoundingBox;

                resources.BinSubtreeCountsX[i] = 0;
                resources.BinSubtreeCountsY[i] = 0;
                resources.BinSubtreeCountsZ[i] = 0;

                resources.BinLeafCountsX[i] = 0;
                resources.BinLeafCountsY[i] = 0;
                resources.BinLeafCountsZ[i] = 0;
            }

            //var startAllocateToBins = Stopwatch.GetTimestamp();
            //Allocate subtrees to bins for all axes simultaneously.
            for (int i = 0; i < count; ++i)
            {
                var subtreeIndex = localIndexMap[i];
                var binIndices = Vector3.Min((resources.Centroids[subtreeIndex] - centroidBoundingBox.Min) * inverseBinSize, maximumBinIndex);
                var x = (int)binIndices.X;
                var y = (int)binIndices.Y;
                var z = (int)binIndices.Z;

                resources.SubtreeBinIndicesX[i] = x;
                resources.SubtreeBinIndicesY[i] = y;
                resources.SubtreeBinIndicesZ[i] = z;

                var leafCount = resources.LeafCounts + subtreeIndex;
                var subtreeBoundingBox = resources.BoundingBoxes + subtreeIndex;

                resources.BinLeafCountsX[x] += *leafCount;
                resources.BinLeafCountsY[y] += *leafCount;
                resources.BinLeafCountsZ[z] += *leafCount;

                ++resources.BinSubtreeCountsX[x];
                ++resources.BinSubtreeCountsY[y];
                ++resources.BinSubtreeCountsZ[z];

                BoundingBox.CreateMerged(resources.BinBoundingBoxesX[x], *subtreeBoundingBox, out resources.BinBoundingBoxesX[x]);
                BoundingBox.CreateMerged(resources.BinBoundingBoxesY[y], *subtreeBoundingBox, out resources.BinBoundingBoxesY[y]);
                BoundingBox.CreateMerged(resources.BinBoundingBoxesZ[z], *subtreeBoundingBox, out resources.BinBoundingBoxesZ[z]);
            }

            //Determine split axes for all axes simultaneously.
            //Sweep from low to high.
            var lastIndex = binCount - 1;

            resources.ALeafCountsX[0] = resources.BinLeafCountsX[0];
            resources.ALeafCountsY[0] = resources.BinLeafCountsY[0];
            resources.ALeafCountsZ[0] = resources.BinLeafCountsZ[0];
            resources.AMergedX[0] = resources.BinBoundingBoxesX[0];
            resources.AMergedY[0] = resources.BinBoundingBoxesY[0];
            resources.AMergedZ[0] = resources.BinBoundingBoxesZ[0];
            for (int i = 1; i < lastIndex; ++i)
            {
                var previousIndex = i - 1;
                resources.ALeafCountsX[i] = resources.BinLeafCountsX[i] + resources.ALeafCountsX[previousIndex];
                resources.ALeafCountsY[i] = resources.BinLeafCountsY[i] + resources.ALeafCountsY[previousIndex];
                resources.ALeafCountsZ[i] = resources.BinLeafCountsZ[i] + resources.ALeafCountsZ[previousIndex];
                BoundingBox.CreateMerged(resources.AMergedX[previousIndex], resources.BinBoundingBoxesX[i], out resources.AMergedX[i]);
                BoundingBox.CreateMerged(resources.AMergedY[previousIndex], resources.BinBoundingBoxesY[i], out resources.AMergedY[i]);
                BoundingBox.CreateMerged(resources.AMergedZ[previousIndex], resources.BinBoundingBoxesZ[i], out resources.AMergedZ[i]);
            }

            //Sweep from high to low.
            BoundingBox bMergedX = nullBoundingBox;
            BoundingBox bMergedY = nullBoundingBox;
            BoundingBox bMergedZ = nullBoundingBox;
            int bLeafCountX = 0;
            int bLeafCountY = 0;
            int bLeafCountZ = 0;

            int bestAxis = 0;
            float cost = float.MaxValue;
            var binSplitIndex = 0;
            a = nullBoundingBox;
            b = nullBoundingBox;
            leafCountA = 0;
            leafCountB = 0;


            for (int i = lastIndex; i >= 1; --i)
            {
                int aIndex = i - 1;
                BoundingBox.CreateMerged(bMergedX, resources.BinBoundingBoxesX[i], out bMergedX);
                BoundingBox.CreateMerged(bMergedY, resources.BinBoundingBoxesY[i], out bMergedY);
                BoundingBox.CreateMerged(bMergedZ, resources.BinBoundingBoxesZ[i], out bMergedZ);
                bLeafCountX += resources.BinLeafCountsX[i];
                bLeafCountY += resources.BinLeafCountsY[i];
                bLeafCountZ += resources.BinLeafCountsZ[i];


                //It's possible for a lot of bins in a row to be unpopulated. In that event, the metric isn't defined; don't bother calculating it.
                float costCandidateX, costCandidateY, costCandidateZ;
                if (bLeafCountX > 0 && resources.ALeafCountsX[aIndex] > 0)
                {
                    var metricAX = ComputeBoundsMetric(ref resources.AMergedX[aIndex]);
                    var metricBX = ComputeBoundsMetric(ref bMergedX);
                    costCandidateX = resources.ALeafCountsX[aIndex] * metricAX + bLeafCountX * metricBX;
                }
                else
                    costCandidateX = float.MaxValue;
                if (bLeafCountY > 0 && resources.ALeafCountsY[aIndex] > 0)
                {
                    var metricAY = ComputeBoundsMetric(ref resources.AMergedY[aIndex]);
                    var metricBY = ComputeBoundsMetric(ref bMergedY);
                    costCandidateY = resources.ALeafCountsY[aIndex] * metricAY + bLeafCountY * metricBY;
                }
                else
                    costCandidateY = float.MaxValue;
                if (bLeafCountZ > 0 && resources.ALeafCountsZ[aIndex] > 0)
                {
                    var metricAZ = ComputeBoundsMetric(ref resources.AMergedZ[aIndex]);
                    var metricBZ = ComputeBoundsMetric(ref bMergedZ);
                    costCandidateZ = resources.ALeafCountsZ[aIndex] * metricAZ + bLeafCountZ * metricBZ;
                }
                else
                    costCandidateZ = float.MaxValue;
                if (costCandidateX < costCandidateY && costCandidateX < costCandidateZ)
                {
                    if (costCandidateX < cost)
                    {
                        bestAxis = 0;
                        cost = costCandidateX;
                        binSplitIndex = i;
                        a = resources.AMergedX[aIndex];
                        b = bMergedX;
                        leafCountA = resources.ALeafCountsX[aIndex];
                        leafCountB = bLeafCountX;
                    }
                }
                else if (costCandidateY < costCandidateZ)
                {
                    if (costCandidateY < cost)
                    {
                        bestAxis = 1;
                        cost = costCandidateY;
                        binSplitIndex = i;
                        a = resources.AMergedY[aIndex];
                        b = bMergedY;
                        leafCountA = resources.ALeafCountsY[aIndex];
                        leafCountB = bLeafCountY;
                    }
                }
                else
                {
                    if (costCandidateZ < cost)
                    {
                        bestAxis = 2;
                        cost = costCandidateZ;
                        binSplitIndex = i;
                        a = resources.AMergedZ[aIndex];
                        b = bMergedZ;
                        leafCountA = resources.ALeafCountsZ[aIndex];
                        leafCountB = bLeafCountZ;
                    }
                }

            }


            int* bestBinSubtreeCounts;
            int* bestSubtreeBinIndices;
            switch (bestAxis)
            {
                case 0:
                    bestBinSubtreeCounts = resources.BinSubtreeCountsX;
                    bestSubtreeBinIndices = resources.SubtreeBinIndicesX;
                    break;
                case 1:
                    bestBinSubtreeCounts = resources.BinSubtreeCountsY;
                    bestSubtreeBinIndices = resources.SubtreeBinIndicesY;
                    break;
                default:
                    bestBinSubtreeCounts = resources.BinSubtreeCountsZ;
                    bestSubtreeBinIndices = resources.SubtreeBinIndicesZ;
                    break;
            }
            //Rebuild the index map.

            resources.BinStartIndices[0] = 0;
            resources.BinSubtreeCountsSecondPass[0] = 0;

            for (int i = 1; i < binCount; ++i)
            {
                resources.BinStartIndices[i] = resources.BinStartIndices[i - 1] + bestBinSubtreeCounts[i - 1];
                resources.BinSubtreeCountsSecondPass[i] = 0;
            }

            //var startIndexMapTime = Stopwatch.GetTimestamp();

            for (int i = 0; i < count; ++i)
            {
                var index = bestSubtreeBinIndices[i];
                resources.TempIndexMap[resources.BinStartIndices[index] + resources.BinSubtreeCountsSecondPass[index]++] = localIndexMap[i];
            }

            //Update the real index map.
            for (int i = 0; i < count; ++i)
            {
                localIndexMap[i] = resources.TempIndexMap[i];
            }
            //Transform the split index into object indices.
            splitIndex = resources.BinStartIndices[binSplitIndex] + start;

        }



        unsafe void SplitSubtreesIntoChildrenBinned(ref BinnedResources resources,
            int start, int count,
            int stagingNodeIndex, ref int stagingNodesCount, out float childrenTreeletsCost)
        {
            Debug.Assert(count > 2);
            FindPartitionBinned(ref resources, start, count, out int splitIndex, out BoundingBox aBounds, out BoundingBox bBounds, out int leafCountA, out int leafCountB);

            //Recursion bottomed out. 
            var stagingNode = resources.StagingNodes + stagingNodeIndex;

            var stagingChildren = &stagingNode->A;
            ref var a = ref stagingNode->A;
            ref var b = ref stagingNode->B;
            a.Min = aBounds.Min;
            a.Max = aBounds.Max;
            b.Min = bBounds.Min;
            b.Max = bBounds.Max;
            a.LeafCount = leafCountA;
            b.LeafCount = leafCountB;

            int subtreeCountA = splitIndex - start;
            int subtreeCountB = start + count - splitIndex;
            float costA, costB;
            if (subtreeCountA > 1)
            {
                a.Index = CreateStagingNodeBinned(ref resources, start, subtreeCountA,
                    ref stagingNodesCount, out costA);
                costA += ComputeBoundsMetric(ref aBounds); //An internal node was created; measure its cost.
            }
            else
            {
                Debug.Assert(subtreeCountA == 1);
                //Only one subtree. Don't create another node.
                a.Index = Encode(resources.IndexMap[start]);
                costA = 0;
            }
            if (subtreeCountB > 1)
            {
                b.Index = CreateStagingNodeBinned(ref resources, splitIndex, subtreeCountB,
                    ref stagingNodesCount, out costB);
                costB += ComputeBoundsMetric(ref bBounds); //An internal node was created; measure its cost.
            }
            else
            {
                Debug.Assert(subtreeCountB == 1);
                //Only one subtree. Don't create another node.
                b.Index = Encode(resources.IndexMap[splitIndex]);
                costB = 0;
            }
            childrenTreeletsCost = costA + costB;
        }

        unsafe int CreateStagingNodeBinned(
            ref BinnedResources resources, int start, int count,
            ref int stagingNodeCount, out float childTreeletsCost)
        {
            var stagingNodeIndex = stagingNodeCount++;
            var stagingNode = resources.StagingNodes + stagingNodeIndex;

            if (count <= 2)
            {
                //No need to do any sorting. This node can fit every remaining subtree.
                var localIndexMap = resources.IndexMap + start;
                var stagingNodeChildren = &stagingNode->A;
                for (int i = 0; i < count; ++i)
                {
                    var subtreeIndex = localIndexMap[i];
                    ref var child = ref stagingNodeChildren[i];
                    ref var bounds = ref resources.BoundingBoxes[subtreeIndex];
                    child.Min = bounds.Min;
                    child.Max = bounds.Max;
                    child.LeafCount = resources.LeafCounts[subtreeIndex];
                    child.Index = Encode(subtreeIndex);
                }
                //Because subtrees do not change in size, they cannot change the cost.
                childTreeletsCost = 0;
                return stagingNodeIndex;
            }

            SplitSubtreesIntoChildrenBinned(ref resources, start, count, stagingNodeIndex, ref stagingNodeCount, out childTreeletsCost);

            return stagingNodeIndex;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void ReifyChildren(int internalNodeIndex, Node* stagingNodes,
            ref QuickList<int> subtrees, ref QuickList<int> treeletInternalNodes, ref int nextInternalNodeIndexToUse)
        {
            Debug.Assert(subtrees.Count > 1);
            ref var internalNode = ref Nodes[internalNodeIndex];
            ref var internalNodeChildren = ref internalNode.A;
            for (int i = 0; i < 2; ++i)
            {
                ref var child = ref Unsafe.Add(ref internalNodeChildren, i);
                if (child.Index >= 0)
                {
                    child.Index = ReifyStagingNode(internalNodeIndex, i, stagingNodes, child.Index,
                        ref subtrees, ref treeletInternalNodes, ref nextInternalNodeIndexToUse);
                }
                else
                {
                    //It's a subtree. Update its pointers.
                    var subtreeIndex = subtrees[Encode(child.Index)];
                    child.Index = subtreeIndex;
                    if (subtreeIndex >= 0)
                    {
                        Debug.Assert(subtreeIndex >= 0 && subtreeIndex < nodeCount);
                        //Subtree is an internal node. Update its parent pointers.
                        ref var metanode = ref Metanodes[subtreeIndex];
                        metanode.IndexInParent = i;
                        metanode.Parent = internalNodeIndex;

                    }
                    else
                    {
                        //Subtree is a leaf node. Update its parent pointers.
                        var leafIndex = Encode(subtreeIndex);
                        Debug.Assert(leafIndex >= 0 && leafIndex < LeafCount);
                        Leaves[leafIndex] = new Leaf(internalNodeIndex, i);
                    }
                }
            }
        }

        unsafe int ReifyStagingNode(int parent, int indexInParent, Node* stagingNodes, int stagingNodeIndex,
           ref QuickList<int> subtrees, ref QuickList<int> treeletInternalNodes,
           ref int nextInternalNodeIndexToUse)
        {
            int internalNodeIndex;
            Debug.Assert(nextInternalNodeIndexToUse < treeletInternalNodes.Count,
                "Binary trees should never run out of available internal nodes when reifying staging nodes; no nodes are created or destroyed during the process.");

            //There is an internal node that we can use.
            //Note that we remove from the end to guarantee that the treelet root does not change location.
            //The CollectSubtrees function guarantees that the treelet root is enqueued first.
            internalNodeIndex = treeletInternalNodes[nextInternalNodeIndexToUse++];

            //To make the staging node real, it requires an accurate parent pointer, index in parent, and child indices.
            //Copy the staging node into the real tree.
            //We take the staging node's child bounds, child indices, leaf counts, and child count.
            //The parent and index in parent are provided by the caller.
            ref var stagingNode = ref stagingNodes[stagingNodeIndex];
            ref var internalNode = ref Nodes[internalNodeIndex];
            internalNode = stagingNode;
            ref var metanode = ref Metanodes[internalNodeIndex];
            metanode.RefineFlag = 0; //The staging node could have contained arbitrary refine flag data.
            metanode.Parent = parent;
            metanode.IndexInParent = indexInParent;


            ReifyChildren(internalNodeIndex, stagingNodes, ref subtrees, ref treeletInternalNodes, ref nextInternalNodeIndexToUse);
            return internalNodeIndex;
        }

        unsafe void ReifyStagingNodes(int treeletRootIndex, Node* stagingNodes,
            ref QuickList<int> subtrees, ref QuickList<int> treeletInternalNodes, ref int nextInternalNodeIndexToUse)
        {
            //We take the staging node's child bounds, child indices, leaf counts, and child count.
            //The parent and index in parent of the treelet root CANNOT BE TOUCHED.
            //When running on multiple threads, another thread may modify the Parent and IndexInParent of the treelet root.
            ref var internalNode = ref Nodes[treeletRootIndex];
            internalNode.A = stagingNodes->A;
            internalNode.B = stagingNodes->B;
            ReifyChildren(treeletRootIndex, stagingNodes, ref subtrees, ref treeletInternalNodes, ref nextInternalNodeIndexToUse);
        }



        public unsafe void BinnedRefine(int nodeIndex,
            ref QuickList<int> subtreeReferences, int maximumSubtrees,
            ref QuickList<int> treeletInternalNodes,
            ref BinnedResources resources, BufferPool pool)
        {
            Debug.Assert(subtreeReferences.Count == 0, "The subtree references list should be empty since it's about to get filled.");
            Debug.Assert(subtreeReferences.Span.Length >= maximumSubtrees, "Subtree references list should have a backing array large enough to hold all possible subtrees.");
            Debug.Assert(treeletInternalNodes.Count == 0, "The treelet internal nodes list should be empty since it's about to get filled.");
            Debug.Assert(treeletInternalNodes.Span.Length >= maximumSubtrees - 1, "Internal nodes queue should have a backing array large enough to hold all possible treelet internal nodes.");
            CollectSubtrees(nodeIndex, maximumSubtrees, resources.SubtreeHeapEntries, ref subtreeReferences, ref treeletInternalNodes, out float originalTreeletCost);
            Debug.Assert(treeletInternalNodes.Count == subtreeReferences.Count - 2,
                "Given that this is a binary tree, the number of subtree references found must match the internal nodes traversed to reach them. Note that the treelet root is excluded.");
            Debug.Assert(subtreeReferences.Count <= maximumSubtrees);

            //TODO: There's no reason to use a priority queue based node selection process for MOST treelets. It's only useful for the root node treelet.
            //For the others, we can use a much cheaper collection scheme.
            //CollectSubtreesDirect(nodeIndex, maximumSubtrees, ref subtreeReferences, ref treeletInternalNodes, out originalTreeletCost);

            //Gather necessary information from nodes.
            for (int i = 0; i < subtreeReferences.Count; ++i)
            {
                resources.IndexMap[i] = i;
                if (subtreeReferences[i] >= 0)
                {
                    //It's an internal node.
                    ref var subtreeMetanode = ref Metanodes[subtreeReferences[i]];
                    ref var parentNode = ref Nodes[subtreeMetanode.Parent];
                    ref var owningChild = ref Unsafe.Add(ref parentNode.A, subtreeMetanode.IndexInParent);
                    ref var targetBounds = ref resources.BoundingBoxes[i];
                    targetBounds.Min = owningChild.Min;
                    targetBounds.Max = owningChild.Max;
                    resources.Centroids[i] = owningChild.Min + owningChild.Max;
                    resources.LeafCounts[i] = owningChild.LeafCount;
                }
                else
                {
                    //It's a leaf node.
                    ref var leaf = ref Leaves[Encode(subtreeReferences[i])];
                    ref var owningChild = ref Unsafe.Add(ref Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                    ref var targetBounds = ref resources.BoundingBoxes[i];
                    targetBounds.Min = owningChild.Min;
                    targetBounds.Max = owningChild.Max;
                    resources.Centroids[i] = owningChild.Min + owningChild.Max;
                    resources.LeafCounts[i] = 1;
                }
            }

            //Now perform a top-down sweep build.
            //TODO: this staging creation section is really the only part that is sweep-specific. The rest is common to any other kind of subtree-collection based refinement. 
            //If you end up making others, keep this in mind.
            int stagingNodeCount = 0;


            CreateStagingNodeBinned(ref resources, 0, subtreeReferences.Count, ref stagingNodeCount, out float newTreeletCost);
            //Copy the refine flag over from the treelet root so that it persists.
            resources.RefineFlags[0] = Metanodes[nodeIndex].RefineFlag;


            //ValidateStaging(stagingNodes, sweepSubtrees, ref subtreeReferences, parent, indexInParent);

            //Note that updating only when it improves often results in a suboptimal local minimum. Allowing it to worsen locally often leads to better global scores.
            if (true)//newTreeletCost < originalTreeletCost)
            {
                //The refinement is an actual improvement.
                //Apply the staged nodes to real nodes!
                int nextInternalNodeIndexToUse = 0;
                ReifyStagingNodes(nodeIndex, resources.StagingNodes, ref subtreeReferences, ref treeletInternalNodes, ref nextInternalNodeIndexToUse);
            }
        }


    }
}
