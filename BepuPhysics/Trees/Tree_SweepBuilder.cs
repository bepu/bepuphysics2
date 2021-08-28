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
    partial struct Tree
    {
        internal unsafe struct SweepResources
        {
            public BoundingBox* Bounds;
            public int* IndexMap;
            public int* IndexMapX;
            public int* IndexMapY;
            public int* IndexMapZ;
            public float* CentroidsX;
            public float* CentroidsY;
            public float* CentroidsZ;
            public BoundingBox* Merged;
        }

        unsafe struct IndexMapComparer : IComparerRef<int>
        {
            public float* Centroids;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref int a, ref int b)
            {
                var centroidA = Centroids[a];
                var centroidB = Centroids[b];
                return centroidA.CompareTo(centroidB);
            }
        }


        readonly unsafe void FindPartitionForAxis(BoundingBox* boundingBoxes, BoundingBox* aMerged, float* centroids, int* indexMap, int count,
            out int splitIndex, out float cost, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            Debug.Assert(count > 1);
            //TODO: Note that sorting at every level isn't necessary. Like in one of the much older spatial splitting implementations we did, you can just sort once, and thereafter
            //just do an O(n) operation to shuffle leaf data to the relevant location on each side of the partition. That allows us to punt all sort work to a prestep.
            //There, we could throw an optimized parallel sort at it. Or just do the three axes independently, hidden alongside some other work maybe.
            //I suspect the usual problems with parallel sorts would be mitigated somewhat by having three of them going on at the same time- more chances for load balancing.
            //Also note that, at each step, both the above partitioning scheme and the sort result in a contiguous block of data to work on.
            //If you're already doing a gather like that, you might as well throw wider SIMD at the problem. This version only goes up to 3 wide, which is unfortunate for AVX2 and AVX512.
            //With those changes, we can probably get the sweep builder to be faster than v1's insertion builder- it's almost there already.
            //(You'll also want to bench it against similarly simd accelerated binned approaches for use in incremental refinement. If it's not much slower, the extra quality benefits
            //might make it faster on net by virtue of speeding up self-tests, which are a dominant cost.)
            var comparer = new IndexMapComparer { Centroids = centroids };
            QuickSort.Sort(ref indexMap[0], 0, count - 1, ref comparer);

            //Search for the best split.
            //Sweep across from low to high, caching the merged size and leaf count at each point.
            //Index N includes every subtree from 0 to N, inclusive. So index 0 contains subtree 0's information.
            var lastIndex = count - 1;

            aMerged[0] = boundingBoxes[indexMap[0]];
            for (int i = 1; i < lastIndex; ++i)
            {
                var index = indexMap[i];
                BoundingBox.CreateMerged(aMerged[i - 1], boundingBoxes[index], out aMerged[i]);
            }

            //Sweep from high to low.
            var bMerged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            cost = float.MaxValue;
            splitIndex = 0;
            a = bMerged;
            b = bMerged;
            leafCountA = 0;
            leafCountB = 0;
            for (int i = lastIndex; i >= 1; --i)
            {
                int aIndex = i - 1;
                var subtreeIndex = indexMap[i];
                BoundingBox.CreateMerged(bMerged, boundingBoxes[subtreeIndex], out bMerged);

                //Note the modifications to the cost function compared to raw SAH.
                //First, we include a very mildly quadratic term for the counts so that skewed distributions are penalized.
                //This penalty is weak enough that it should effectively never come into play except in pathological cases, like all bounding boxes overlapping.
                //Second, we include an extremely small (the smallest normal floating point number) baseline to the evaluated bounds metric.
                //This ensures that even a set of perfectly overlapping zero bounds will use a midpoint split rather than a skewed split.
                const float normalEpsilon = 1.1754943508e-38f;
                var aCost = i * (1f + i * 0.001f) * (normalEpsilon + ComputeBoundsMetric(ref aMerged[aIndex]));
                var bCount = count - i;
                var bCost = bCount * (1f + bCount * 0.001f) * (normalEpsilon + ComputeBoundsMetric(ref bMerged));

                var totalCost = aCost + bCost;
                if (totalCost < cost)
                {
                    cost = totalCost;
                    splitIndex = i;
                    a = aMerged[aIndex];
                    b = bMerged;
                    leafCountA = i;
                    leafCountB = count - i;
                }

            }

        }

        unsafe readonly void FindPartition(ref SweepResources leaves, int start, int count,
               out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            //A variety of potential microoptimizations exist here.

            //Initialize the per-axis candidate maps.
            for (int i = 0; i < count; ++i)
            {
                var originalValue = leaves.IndexMap[i + start];
                leaves.IndexMapX[i] = originalValue;
                leaves.IndexMapY[i] = originalValue;
                leaves.IndexMapZ[i] = originalValue;
            }

            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsX, leaves.IndexMapX, count,
                out int xSplitIndex, out float xCost, out BoundingBox xA, out BoundingBox xB, out int xLeafCountA, out int xLeafCountB);
            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsY, leaves.IndexMapY, count,
                out int ySplitIndex, out float yCost, out BoundingBox yA, out BoundingBox yB, out int yLeafCountA, out int yLeafCountB);
            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsZ, leaves.IndexMapZ, count,
                out int zSplitIndex, out float zCost, out BoundingBox zA, out BoundingBox zB, out int zLeafCountA, out int zLeafCountB);

            int* bestIndexMap;
            if (xCost <= yCost && xCost <= zCost)
            {
                splitIndex = xSplitIndex;
                a = xA;
                b = xB;
                leafCountA = xLeafCountA;
                leafCountB = xLeafCountB;
                bestIndexMap = leaves.IndexMapX;
            }
            else if (yCost <= zCost)
            {
                splitIndex = ySplitIndex;
                a = yA;
                b = yB;
                leafCountA = yLeafCountA;
                leafCountB = yLeafCountB;
                bestIndexMap = leaves.IndexMapY;
            }
            else
            {
                splitIndex = zSplitIndex;
                a = zA;
                b = zB;
                leafCountA = zLeafCountA;
                leafCountB = zLeafCountB;
                bestIndexMap = leaves.IndexMapZ;
            }
            for (int i = 0; i < count; ++i)
            {
                leaves.IndexMap[i + start] = bestIndexMap[i];
            }

            splitIndex += start;


        }

        unsafe void SplitLeavesIntoChildren(ref SweepResources leaves, int start, int count, int nodeIndex)
        {
            Debug.Assert(count >= 2);
            FindPartition(ref leaves, start, count, out int splitIndex, out BoundingBox aBounds, out BoundingBox bBounds, out int leafCountA, out int leafCountB);

            ref var node = ref Nodes[nodeIndex];

            ref var a = ref node.A;
            ref var b = ref node.B;
            a.Min = aBounds.Min;
            a.Max = aBounds.Max;
            b.Min = bBounds.Min;
            b.Max = bBounds.Max;

            a.LeafCount = leafCountA;
            b.LeafCount = leafCountB;

            if (leafCountA > 1)
            {
                a.Index = CreateSweepBuilderNode(nodeIndex, 0, ref leaves, start, leafCountA);
            }
            else
            {
                Debug.Assert(leafCountA == 1);
                //Only one leaf. Don't create another node.
                var leafIndex = leaves.IndexMap[start];
                Leaves[leafIndex] = new Leaf(nodeIndex, 0);
                a.Index = Encode(leafIndex);
            }
            if (leafCountB > 1)
            {
                b.Index = CreateSweepBuilderNode(nodeIndex, 1, ref leaves, splitIndex, leafCountB);
            }
            else
            {
                Debug.Assert(leafCountB == 1);
                //Only one leaf. Don't create another node.
                var leafIndex = leaves.IndexMap[splitIndex];
                Leaves[leafIndex] = new Leaf(nodeIndex, 1);
                b.Index = Encode(leafIndex);
            }
        }

        unsafe int CreateSweepBuilderNode(int parentIndex, int indexInParent,
            ref SweepResources leaves, int start, int count)
        {
            var nodeIndex = AllocateNode();
            ref var metanode = ref Metanodes[nodeIndex];
            metanode.Parent = parentIndex;
            metanode.IndexInParent = indexInParent;
            metanode.RefineFlag = 0;

            if (count <= 2)
            {
                //No need to do any sorting. This node can fit every remaining subtree.
                ref var children = ref Nodes[nodeIndex].A;
                for (int i = 0; i < count; ++i)
                {
                    //The sweep builder preallocated space for leaves and set the leafCount to match.
                    //The index map tells us which of those original leaves to create.
                    var leafIndex = leaves.IndexMap[i + start];
                    Leaves[leafIndex] = new Leaf(nodeIndex, i);
                    ref var child = ref Unsafe.Add(ref children, i);
                    child.Min = leaves.Bounds[leafIndex].Min;
                    child.Max = leaves.Bounds[leafIndex].Max;
                    child.Index = Encode(leafIndex);
                    child.LeafCount = 1;
                }
                return nodeIndex;
            }



            SplitLeavesIntoChildren(ref leaves, start, count, nodeIndex);


            return nodeIndex;

        }


        public unsafe void SweepBuild(BufferPool pool, Buffer<BoundingBox> leafBounds)
        {
            if (leafBounds.Length <= 0)
                throw new ArgumentException("Length must be positive.");
            if (LeafCount != 0)
                throw new InvalidOperationException("Cannot build a tree that already contains nodes.");
            //The tree is built with an empty node at the root to make insertion work more easily.
            //As long as that is the case (and as long as this is not a constructor),
            //we must clear it out.
            nodeCount = 0;

            //Guarantee that no resizes will occur during the build.
            if (Leaves.Length < leafBounds.Length)
            {
                Resize(pool, leafBounds.Length);
            }
            leafCount = leafBounds.Length;


            pool.TakeAtLeast<int>(leafBounds.Length, out var indexMap);
            pool.TakeAtLeast<int>(leafBounds.Length, out var indexMapX);
            pool.TakeAtLeast<int>(leafBounds.Length, out var indexMapY);
            pool.TakeAtLeast<int>(leafBounds.Length, out var indexMapZ);
            pool.TakeAtLeast<float>(leafBounds.Length, out var centroidsX);
            pool.TakeAtLeast<float>(leafBounds.Length, out var centroidsY);
            pool.TakeAtLeast<float>(leafBounds.Length, out var centroidsZ);
            pool.TakeAtLeast<BoundingBox>(leafBounds.Length, out var merged);
            SweepResources leaves;
            leaves.Bounds = leafBounds.Memory;
            leaves.IndexMap = indexMap.Memory;
            leaves.IndexMapX = indexMapX.Memory;
            leaves.IndexMapY = indexMapY.Memory;
            leaves.IndexMapZ = indexMapZ.Memory;
            leaves.CentroidsX = centroidsX.Memory;
            leaves.CentroidsY = centroidsY.Memory;
            leaves.CentroidsZ = centroidsZ.Memory;
            leaves.Merged = merged.Memory;


            for (int i = 0; i < leafBounds.Length; ++i)
            {
                var bounds = leaves.Bounds[i];
                //The index map relates an index in traversal back to the original leaf location.
                leaves.IndexMap[i] = i;
                //Per-axis index maps don't need to be initialized here. They're filled in at the time of use.

                var centroid = bounds.Min + bounds.Max;
                centroidsX[i] = centroid.X;
                centroidsY[i] = centroid.Y;
                centroidsZ[i] = centroid.Z;
            }

            //Now perform a top-down sweep build.
            CreateSweepBuilderNode(-1, -1, ref leaves, 0, leafBounds.Length);


            //Return resources.            
            pool.ReturnUnsafely(centroidsX.Id);
            pool.ReturnUnsafely(centroidsY.Id);
            pool.ReturnUnsafely(centroidsZ.Id);
            pool.ReturnUnsafely(indexMap.Id);
            pool.ReturnUnsafely(indexMapX.Id);
            pool.ReturnUnsafely(indexMapY.Id);
            pool.ReturnUnsafely(indexMapZ.Id);
            pool.Return(ref merged);

        }
    }
}
