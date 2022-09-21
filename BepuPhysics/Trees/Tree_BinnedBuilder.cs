using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;
using System.Threading.Tasks.Sources;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        const int MaximumBinCountRevamp = 64;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe Int4 Truncate(Vector4 v)
        {
            Int4 discrete;
            if (Vector128.IsHardwareAccelerated)
            {
                Vector128.Store(Vector128.ConvertToInt32(v.AsVector128()), (int*)&discrete);
            }
            else
            {
                discrete.X = (int)v.X;
                discrete.Y = (int)v.Y;
                discrete.Z = (int)v.Z;
                discrete.W = (int)v.W;
            }
            return discrete;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void BuildNode(BoundingBox4 a, BoundingBox4 b, Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<int> indices, int nodeIndex, int parentNodeIndex, int childIndexInParent, int aCount, int bCount, out int aIndex, out int bIndex)
        {
            ref var metanode = ref metanodes[0];
            metanode.Parent = parentNodeIndex;
            metanode.IndexInParent = childIndexInParent;
            metanode.RefineFlag = 0;
            ref var node = ref nodes[0];
            aIndex = aCount == 1 ? indices[0] : nodeIndex + 1;
            bIndex = bCount == 1 ? indices[^1] : nodeIndex + aCount;//parentNodeIndex + 1 + (aCount - 1)
            node.A = Unsafe.As<BoundingBox4, NodeChild>(ref a);
            node.B = Unsafe.As<BoundingBox4, NodeChild>(ref b);
            node.A.Index = aIndex;
            node.A.LeafCount = aCount;
            node.B.Index = bIndex;
            node.B.LeafCount = bCount;
        }

        internal static float ComputeBoundsMetric(BoundingBox4 bounds)
        {
            return ComputeBoundsMetric(bounds.Min, bounds.Max);
        }
        internal static float ComputeBoundsMetric(Vector4 min, Vector4 max)
        {
            //Note that we just use the SAH. While we are primarily interested in volume queries for the purposes of collision detection, the topological difference
            //between a volume heuristic and surface area heuristic isn't huge. There is, however, one big annoying issue that volume heuristics run into:
            //all bounding boxes with one extent equal to zero have zero cost. Surface area approaches avoid this hole simply.
            var offset = max - min;
            //Note that this is merely proportional to surface area. Being scaled by a constant factor is irrelevant.
            return offset.X * offset.Y + offset.Y * offset.Z + offset.Z * offset.X;

        }
        struct Bins
        {
            public Buffer<BoundingBox4> BinBoundingBoxes;
            public Buffer<BoundingBox4> BinBoundingBoxesScan;
            public Buffer<int> BinLeafCounts;
        }

        struct BoundsComparerX : IComparerRef<BoundingBox4>
        {
            public int Compare(ref BoundingBox4 a, ref BoundingBox4 b) => (a.Min.X + a.Max.X) > (b.Min.X + b.Max.X) ? -1 : 1;
        }
        struct BoundsComparerY : IComparerRef<BoundingBox4>
        {
            public int Compare(ref BoundingBox4 a, ref BoundingBox4 b) => (a.Min.Y + a.Max.Y) > (b.Min.Y + b.Max.Y) ? -1 : 1;
        }
        struct BoundsComparerZ : IComparerRef<BoundingBox4>
        {
            public int Compare(ref BoundingBox4 a, ref BoundingBox4 b) => (a.Min.Z + a.Max.Z) > (b.Min.Z + b.Max.Z) ? -1 : 1;
        }

        //static void ValidateNode(int nodeIndex, Buffer<Node> nodes, out Vector3 min, out Vector3 max)
        //{
        //    if (nodes[nodeIndex].A.Index >= 0)
        //    {
        //        ValidateNode(nodes[nodeIndex].A.Index, nodes, out var aMin, out var aMax);
        //        Debug.Assert(nodes[nodeIndex].A.Min == aMin);
        //        Debug.Assert(nodes[nodeIndex].A.Max == aMax);
        //    }
        //    if (nodes[nodeIndex].B.Index >= 0)
        //    {
        //        ValidateNode(nodes[nodeIndex].B.Index, nodes, out var bMin, out var bMax);
        //        Debug.Assert(nodes[nodeIndex].B.Min == bMin);
        //        Debug.Assert(nodes[nodeIndex].B.Max == bMax);
        //    }
        //    min = Vector3.Min(nodes[nodeIndex].A.Min, nodes[nodeIndex].A.Min);
        //    max = Vector3.Max(nodes[nodeIndex].B.Max, nodes[nodeIndex].B.Max);
        //}

        static unsafe void MicroSweepForBinnedBuilder(Vector4 centroidMin, Vector4 centroidMax, Buffer<int> indices, Buffer<BoundingBox4> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, int nodeIndex, int parentNodeIndex, int childIndexInParent, Bins bins)
        {
            //This is a very small scale sweep build.
            var leafCount = indices.Length;
            if (leafCount == 2)
            {
                BuildNode(boundingBoxes[0], boundingBoxes[1], nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, out _, out _);
                return;
            }
            var centroidSpan = centroidMax - centroidMin;
            if (centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z)
            {
                var comparer = new BoundsComparerX();
                QuickSort.Sort(ref boundingBoxes[0], ref indices[0], 0, leafCount - 1, ref comparer);
            }
            else if (centroidSpan.Y > centroidSpan.Z)
            {
                var comparer = new BoundsComparerY();
                QuickSort.Sort(ref boundingBoxes[0], ref indices[0], 0, leafCount - 1, ref comparer);
            }
            else
            {
                var comparer = new BoundsComparerZ();
                QuickSort.Sort(ref boundingBoxes[0], ref indices[0], 0, leafCount - 1, ref comparer);
            }

            Debug.Assert(leafCount <= MaximumBinCountRevamp, "We're reusing the bin resources under the assumption that this is only ever called when there are less leaves than maximum bins.");
            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            bins.BinBoundingBoxesScan[0] = boundingBoxes[0];
            for (int i = 1; i < leafCount; ++i)
            {
                ref var previousScanBounds = ref bins.BinBoundingBoxesScan[i - 1];
                ref var scanBounds = ref bins.BinBoundingBoxesScan[i];
                ref var bounds = ref boundingBoxes[i];
                scanBounds.Min = Vector4.Min(bounds.Min, previousScanBounds.Min);
                scanBounds.Max = Vector4.Max(bounds.Max, previousScanBounds.Max);
            }

            float bestSAH = float.MaxValue;
            int bestSplit = 1;
            //The split index is going to end up in child B.
            var lastLeafIndex = leafCount - 1;
            BoundingBox4 accumulatedBoundingBoxB = boundingBoxes[lastLeafIndex];
            Unsafe.SkipInit(out BoundingBox4 bestBoundsB);
            int accumulatedLeafCountB = 1;
            for (int splitIndexCandidate = lastLeafIndex; splitIndexCandidate >= 1; --splitIndexCandidate)
            {
                var previousIndex = splitIndexCandidate - 1;
                var leafCountA = leafCount - accumulatedLeafCountB;
                var sahCandidate = ComputeBoundsMetric(bins.BinBoundingBoxesScan[previousIndex]) * leafCountA + ComputeBoundsMetric(accumulatedBoundingBoxB) * accumulatedLeafCountB;
                if (sahCandidate < bestSAH)
                {
                    bestSAH = sahCandidate;
                    bestSplit = splitIndexCandidate;
                    bestBoundsB = accumulatedBoundingBoxB;
                }
                ref var bounds = ref boundingBoxes[splitIndexCandidate - 1];
                accumulatedBoundingBoxB.Min = Vector4.Min(bounds.Min, accumulatedBoundingBoxB.Min);
                accumulatedBoundingBoxB.Max = Vector4.Max(bounds.Max, accumulatedBoundingBoxB.Max);
                ++accumulatedLeafCountB;
            }

            var bestBoundsA = bins.BinBoundingBoxesScan[bestSplit - 1];

            var aCount = bestSplit;
            var bCount = leafCount - bestSplit;
            {
                //if (leafCount == 2)
                //{
                //    Debug.Assert(bestBoundsA.Min == boundingBoxes[0].Min);
                //    Debug.Assert(bestBoundsA.Max == boundingBoxes[0].Max);
                //    Debug.Assert(bestBoundsB.Min == boundingBoxes[1].Min);
                //    Debug.Assert(bestBoundsB.Max == boundingBoxes[1].Max);
                //}
                //for (int i = 0; i < leafCount; ++i)
                //{
                //    var bounds = i < bestSplit ? bestBoundsA : bestBoundsB;
                //    var containedA = Vector128.LessThanOrEqual(boundingBoxes[i].Max.AsVector128(), bounds.Max.AsVector128()) & Vector128.GreaterThanOrEqual(boundingBoxes[i].Min.AsVector128(), bounds.Min.AsVector128());
                //    var mask = containedA.ExtractMostSignificantBits() & 0b111;
                //    Debug.Assert(mask == 0b111, "All children must be contained within their parent.");
                //}
                //BoundingBox4 debugBoundsA = boundingBoxes[0];
                //for (int i = 1; i < aCount; ++i)
                //{
                //    ref var bounds = ref boundingBoxes[i];
                //    debugBoundsA.Min = Vector4.Min(debugBoundsA.Min, bounds.Min);
                //    debugBoundsA.Max = Vector4.Max(debugBoundsA.Max, bounds.Max);
                //}
                //BoundingBox4 debugBoundsB = boundingBoxes[aCount];
                //for (int i = aCount + 1; i < leafCount; ++i)
                //{
                //    ref var bounds = ref boundingBoxes[i];
                //    debugBoundsB.Min = Vector4.Min(debugBoundsB.Min, bounds.Min);
                //    debugBoundsB.Max = Vector4.Max(debugBoundsB.Max, bounds.Max);
                //}
                //Debug.Assert(bestBoundsA.Min == debugBoundsA.Min);
                //Debug.Assert(bestBoundsA.Max == debugBoundsA.Max);
                //Debug.Assert(bestBoundsB.Min == debugBoundsB.Min);
                //Debug.Assert(bestBoundsB.Max == debugBoundsB.Max);
                BuildNode(bestBoundsA, bestBoundsB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, aCount, bCount, out var aIndex, out var bIndex);
                if (aCount > 1)
                {
                    var aBounds = boundingBoxes.Slice(aCount);
                    BoundingBox4 centroidBoundsA = aBounds[0];
                    for (int i = 1; i < aCount; ++i)
                    {
                        ref var bounds = ref aBounds[i];
                        centroidBoundsA.Min = Vector4.Min(centroidBoundsA.Min, bounds.Min);
                        centroidBoundsA.Max = Vector4.Max(centroidBoundsA.Max, bounds.Max);
                    }
                    MicroSweepForBinnedBuilder(centroidBoundsA.Min, centroidBoundsA.Max, indices.Slice(aCount), aBounds, nodes.Slice(1, aCount - 1), metanodes.Slice(1, aCount - 1), aIndex, nodeIndex, 0, bins);
                }
                if (bCount > 1)
                {
                    var bBounds = boundingBoxes.Slice(aCount, bCount);
                    BoundingBox4 centroidBoundsB = bBounds[0];
                    for (int i = 0; i < bCount; ++i)
                    {
                        ref var bounds = ref bBounds[i];
                        centroidBoundsB.Min = Vector4.Min(centroidBoundsB.Min, bounds.Min);
                        centroidBoundsB.Max = Vector4.Max(centroidBoundsB.Max, bounds.Max);
                    }
                    MicroSweepForBinnedBuilder(centroidBoundsB.Min, centroidBoundsB.Max, indices.Slice(aCount, bCount), bBounds, nodes.Slice(aCount, bCount - 1), metanodes.Slice(aCount, bCount - 1), bIndex, nodeIndex, 1, bins);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe int ComputeBinIndex(Vector4 centroidMin, bool useX, bool useY, Vector128<int> permuteMask, int axisIndex, Vector4 offsetToBinIndex, Vector4 maximumBinIndex, in BoundingBox4 box)
        {
            var centroid = box.Min + box.Max;
            var binIndicesForLeafContinuous = Vector4.Min(maximumBinIndex, (centroid - centroidMin) * offsetToBinIndex);
            //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
            //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
            int binIndex;
            //To extract the desired lane, we need to use a variable shuffle mask. At the time of writing, the Vector128 cross platform shuffle did not like variable masks.
            if (Avx.IsSupported)
            {
                binIndex = (int)Vector128.ToScalar(Avx.PermuteVar(binIndicesForLeafContinuous.AsVector128(), permuteMask));
            }
            else if (Vector128.IsHardwareAccelerated)
            {
                binIndex = (int)Vector128.GetElement(binIndicesForLeafContinuous.AsVector128(), axisIndex);
            }
            else
            {
                binIndex = (int)(useX ? binIndicesForLeafContinuous.X : useY ? binIndicesForLeafContinuous.Y : binIndicesForLeafContinuous.Z);
            }

            return binIndex;
        }

        static unsafe void BinnedBuilderInternal(Buffer<int> indices, Buffer<BoundingBox4> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, int nodeIndex, int parentNodeIndex, int childIndexInParent, in Bins bins)
        {
            var centroidMin = new Vector4(float.MaxValue);
            var centroidMax = new Vector4(float.MinValue);
            var leafCount = indices.Length;

            for (int i = 0; i < leafCount; ++i)
            {
                ref var box = ref boundingBoxes[i];
                //Note that centroids never bother scaling by 0.5. It's fine as long as we're consistent.
                var centroid = box.Min + box.Max;
                centroidMin = Vector4.Min(centroidMin, centroid);
                centroidMax = Vector4.Max(centroidMax, centroid);
            }
            var centroidSpan = centroidMax - centroidMin;
            var axisIsDegenerate = Vector128.LessThanOrEqual(centroidSpan.AsVector128(), Vector128.Create(1e-12f));
            if ((Vector128.ExtractMostSignificantBits(axisIsDegenerate) & 0b111) == 0b111)
            {
                //This node is completely degenerate; there is no 'good' ordering of the children. Pick a split in the middle and shrug.
                //This shouldn't happen unless something is badly wrong with the input; no point in optimizing it.
                var countA = indices.Length / 2;
                var countB = indices.Length - countA;
                //Still have to compute the child bounding boxes, because the centroid bounds span being zero doesn't imply that the full bounds are zero.
                BoundingBox4 boundsA, boundsB;
                boundsA.Min = new Vector4(float.MaxValue);
                boundsA.Max = new Vector4(float.MinValue);
                boundsB.Min = new Vector4(float.MaxValue);
                boundsB.Max = new Vector4(float.MinValue);
                for (int i = 0; i < countA; ++i)
                {
                    ref var bounds = ref boundingBoxes[i];
                    boundsA.Min = Vector4.Min(bounds.Min, boundsA.Min);
                    boundsA.Max = Vector4.Max(bounds.Max, boundsA.Max);
                }
                for (int i = countA; i < indices.Length; ++i)
                {
                    ref var bounds = ref boundingBoxes[i];
                    boundsB.Min = Vector4.Min(bounds.Min, boundsB.Min);
                    boundsB.Max = Vector4.Max(bounds.Max, boundsB.Max);
                }
                BuildNode(boundsA, boundsB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, countA, countB, out var aIndex, out var bIndex);
                if (countA > 1)
                    BinnedBuilderInternal(indices.Slice(countA), boundingBoxes.Slice(countA), nodes.Slice(1, countA - 1), metanodes.Slice(1, countA - 1), aIndex, nodeIndex, 0, bins);
                if (countB > 1)
                    BinnedBuilderInternal(indices.Slice(countA, countB), boundingBoxes.Slice(countA, countB), nodes.Slice(countA, countB - 1), metanodes.Slice(countA, countB - 1), bIndex, nodeIndex, 1, bins);
                return;
            }
            if (leafCount == 2)
            {
                BuildNode(boundingBoxes[0], boundingBoxes[1], nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, out _, out _);
                return;
            }

            if (leafCount <= 32)
            {
                MicroSweepForBinnedBuilder(centroidMin, centroidMax, indices, boundingBoxes, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, bins);
                return;
            }

            var useX = centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z;
            var useY = centroidSpan.Y > centroidSpan.Z;
            //These will be used conditionally based on what hardware acceleration is available. Pretty minor detail.
            var permuteMask = Vector128.Create(useX ? 0 : useY ? 1 : 2, 0, 0, 0);
            var axisIndex = useX ? 0 : useY ? 1 : 2;

            var binCount = Math.Min(MaximumBinCountRevamp, Math.Max((int)(leafCount * .1f), 16));
            Debug.Assert(bins.BinBoundingBoxes.Length >= binCount);

            var offsetToBinIndex = new Vector4(binCount) / centroidSpan;
            //Avoid letting NaNs into the offsetToBinIndex scale.
            offsetToBinIndex = Vector128.ConditionalSelect(axisIsDegenerate, Vector128<float>.Zero, offsetToBinIndex.AsVector128()).AsVector4();


            for (int i = 0; i < binCount; ++i)
            {
                ref var boxX = ref bins.BinBoundingBoxes[i];
                boxX.Min = new Vector4(float.MaxValue);
                boxX.Max = new Vector4(float.MinValue);
                bins.BinLeafCounts[i] = 0;
            }

            //Unfortunately, unrolling has a slight benefit here.
            var maximumBinIndex = new Vector4(binCount - 1);

            //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
            //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
            for (int i = 0; i < leafCount; ++i)
            {
                ref var box = ref boundingBoxes[i];
                var binIndex = ComputeBinIndex(centroidMin, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
                ref var xBounds = ref bins.BinBoundingBoxes[binIndex];
                xBounds.Min = Vector4.Min(xBounds.Min, box.Min);
                xBounds.Max = Vector4.Max(xBounds.Max, box.Max);
                ++bins.BinLeafCounts[binIndex];
            }

            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            bins.BinBoundingBoxesScan[0] = bins.BinBoundingBoxes[0];
            for (int i = 1; i < binCount; ++i)
            {
                var previousIndex = i - 1;
                ref var xBounds = ref bins.BinBoundingBoxes[i];
                ref var xScanBounds = ref bins.BinBoundingBoxesScan[i];
                ref var xPreviousScanBounds = ref bins.BinBoundingBoxesScan[previousIndex];
                xScanBounds.Min = Vector4.Min(xBounds.Min, xPreviousScanBounds.Min);
                xScanBounds.Max = Vector4.Max(xBounds.Max, xPreviousScanBounds.Max);
            }
            var leftBoundsX = bins.BinBoundingBoxes[0];
            Debug.Assert(
                leftBoundsX.Min.X > float.MinValue && leftBoundsX.Min.Y > float.MinValue && leftBoundsX.Min.Z > float.MinValue,
                "Bin 0 should have been updated in all cases because it is aligned with the minimum bin, and the centroid span isn't degenerate.");

            float bestSAH = float.MaxValue;
            int bestSplit = 1;
            //The split index is going to end up in child B.
            var lastBinIndex = binCount - 1;
            BoundingBox4 accumulatedBoundingBoxB;
            accumulatedBoundingBoxB = bins.BinBoundingBoxes[lastBinIndex];
            BoundingBox4 bestBoundingBoxB;
            bestBoundingBoxB = bins.BinBoundingBoxes[lastBinIndex];
            int accumulatedLeafCountB = bins.BinLeafCounts[lastBinIndex];
            for (int splitIndexCandidate = lastBinIndex; splitIndexCandidate >= 1; --splitIndexCandidate)
            {
                var previousIndex = splitIndexCandidate - 1;
                var leafCountA = leafCount - accumulatedLeafCountB;
                var sahCandidate = ComputeBoundsMetric(bins.BinBoundingBoxesScan[previousIndex]) * leafCountA + ComputeBoundsMetric(accumulatedBoundingBoxB) * accumulatedLeafCountB;
                if (sahCandidate < bestSAH)
                {
                    bestSAH = sahCandidate;
                    bestSplit = splitIndexCandidate;
                    bestBoundingBoxB = accumulatedBoundingBoxB;
                }
                ref var xBounds = ref bins.BinBoundingBoxes[previousIndex];
                accumulatedBoundingBoxB.Min = Vector4.Min(xBounds.Min, accumulatedBoundingBoxB.Min);
                accumulatedBoundingBoxB.Max = Vector4.Max(xBounds.Max, accumulatedBoundingBoxB.Max);
                accumulatedLeafCountB += bins.BinLeafCounts[previousIndex];
            }

            //Choose the best SAH from all axes and split the indices/bounds into two halves for the children to operate on.
            var bCount = 0;
            var aCount = 0;
            var splitIndex = bestSplit;
            var bestboundsA = bins.BinBoundingBoxesScan[bestSplit - 1];
            var bestboundsB = bestBoundingBoxB;
            //Now we have the split index between bins. Go back through and sort the indices and bounds into two halves.
            while (aCount + bCount < leafCount)
            {
                ref var box = ref boundingBoxes[aCount];
                var binIndex = ComputeBinIndex(centroidMin, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
                if (binIndex >= splitIndex)
                {
                    //Belongs to B. Swap it.
                    var targetIndex = leafCount - bCount - 1;
                    Helpers.Swap(ref indices[targetIndex], ref indices[aCount]);
                    if (Vector256.IsHardwareAccelerated)
                    {
                        var targetMemory = (byte*)(boundingBoxes.Memory + targetIndex);
                        var aCountMemory = (byte*)(boundingBoxes.Memory + aCount);
                        var targetVector = Vector256.Load(targetMemory);
                        var aCountVector = Vector256.Load(aCountMemory);
                        Vector256.Store(aCountVector, targetMemory);
                        Vector256.Store(targetVector, aCountMemory);
                    }
                    else
                    {
                        Helpers.Swap(ref boundingBoxes[targetIndex], ref boundingBoxes[aCount]);
                    }
                    ++bCount;
                    //(Note that we still need to examine what we just swapped into the slot! It may belong to B too!)
                }
                else
                {
                    //Belongs to A, no movement necessary.
                    ++aCount;
                }
            }

            {
                Debug.Assert(aCount + bCount == leafCount);
                BuildNode(bestboundsA, bestboundsB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, aCount, bCount, out var aIndex, out var bIndex);
                if (aCount > 1)
                    BinnedBuilderInternal(indices.Slice(aCount), boundingBoxes.Slice(aCount), nodes.Slice(1, aCount - 1), metanodes.Slice(1, aCount - 1), aIndex, nodeIndex, 0, bins);
                if (bCount > 1)
                    BinnedBuilderInternal(indices.Slice(aCount, bCount), boundingBoxes.Slice(aCount, bCount), nodes.Slice(aCount, bCount - 1), metanodes.Slice(aCount, bCount - 1), bIndex, nodeIndex, 1, bins);
            }
        }

        public static unsafe void BinnedBuilder(Buffer<int> indices, Buffer<BoundingBox> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, BufferPool pool)
        {
            var leafCount = indices.Length;
            Debug.Assert(boundingBoxes.Length >= leafCount, "The bounding boxes provided must cover the range of indices provided.");
            Debug.Assert(nodes.Length >= leafCount - 1, "The output nodes must be able to contain the nodes created for the leaves.");
            if (leafCount == 0)
                return;
            if (leafCount == 1)
            {
                //If there's only one leaf, the tree has a special format: the root node has only one child.
                ref var root = ref nodes[0];
                root.A.Min = boundingBoxes[0].Min;
                root.A.Index = indices[0]; //Node that we assume the indices are already encoded. This function works with subtree refinements as well which can manage either leaves or internals.
                root.A.Max = boundingBoxes[0].Max;
                root.A.LeafCount = 1;
                root.B = default;
                return;
            }
            boundingBoxes = boundingBoxes.Slice(indices.Length);
            nodes = nodes.Slice(leafCount - 1);

            var binBoundsMemory = stackalloc BoundingBox4[MaximumBinCountRevamp * 2];
            Bins bins;
            bins.BinBoundingBoxes = new Buffer<BoundingBox4>(binBoundsMemory, MaximumBinCountRevamp);
            bins.BinBoundingBoxesScan = new Buffer<BoundingBox4>(binBoundsMemory + MaximumBinCountRevamp, MaximumBinCountRevamp);

            var binLeafCountsMemory = stackalloc int[MaximumBinCountRevamp];
            bins.BinLeafCounts = new Buffer<int>(binLeafCountsMemory, MaximumBinCountRevamp);

            //While we could avoid a recursive implementation, the overhead is low compared to the per-iteration cost.
            BinnedBuilderInternal(indices, boundingBoxes.As<BoundingBox4>(), nodes, metanodes, 0, -1, -1, bins);
        }

    }
}
