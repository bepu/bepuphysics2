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
using System.Threading.Tasks.Sources;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
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
        static void BuildParentNode(BoundingBox4 a, BoundingBox4 b, Buffer<Node> nodes, int parentNodeIndex, int firstChildCount, int secondChildCount, out int aIndex, out int bIndex)
        {
            ref var parentNode = ref nodes[0];
            aIndex = parentNodeIndex + 1;
            bIndex = parentNodeIndex + firstChildCount;//parentNodeIndex + 1 + (firstChildCount - 1)
            parentNode.A = Unsafe.As<BoundingBox4, NodeChild>(ref a);
            parentNode.B = Unsafe.As<BoundingBox4, NodeChild>(ref b);
            parentNode.A.Index = aIndex;
            parentNode.A.LeafCount = firstChildCount;
            parentNode.B.Index = bIndex;
            parentNode.B.LeafCount = secondChildCount;
        }

        struct Bins
        {
            public Buffer<BoundingBox4> BinBoundingBoxesX;
            public Buffer<BoundingBox4> BinBoundingBoxesY;
            public Buffer<BoundingBox4> BinBoundingBoxesZ;
            public Buffer<BoundingBox4> BinBoundingBoxesScanX;
            public Buffer<BoundingBox4> BinBoundingBoxesScanY;
            public Buffer<BoundingBox4> BinBoundingBoxesScanZ;

            public Buffer<int> BinLeafCountsX;
            public Buffer<int> BinLeafCountsY;
            public Buffer<int> BinLeafCountsZ;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float ComputeBoundsMetric(BoundingBox4 bounds)
        {
            return ComputeBoundsMetric(bounds.Min, bounds.Max);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float ComputeBoundsMetric(Vector4 min, Vector4 max)
        {
            //Note that we just use the SAH. While we are primarily interested in volume queries for the purposes of collision detection, the topological difference
            //between a volume heuristic and surface area heuristic isn't huge. There is, however, one big annoying issue that volume heuristics run into:
            //all bounding boxes with one extent equal to zero have zero cost. Surface area approaches avoid this hole simply.
            if (Vector128.IsHardwareAccelerated)
            {
                var span = max - min;
                var shuffled = Vector128.Shuffle(span.AsVector128(), Vector128.Create(1, 2, 0, 0));
                var zeroedUpper = shuffled.WithElement(3, 0);
                return Vector128.Dot(span.AsVector128(), zeroedUpper);
            }
            else
            {
                var offset = max - min;
                //Note that this is merely proportional to surface area. Being scaled by a constant factor is irrelevant.
                return offset.X * offset.Y + offset.Y * offset.Z + offset.Z * offset.X;
            }
        }

        static unsafe void BinnedBuilderInternal(Buffer<int> indices, Buffer<BoundingBox4> boundingBoxes, Buffer<Node> nodes, int parentNodeIndex, in Bins bins)
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
            var binCount = Math.Min(MaximumBinCount, Math.Max((int)(leafCount * 0.25f), 4));
            Debug.Assert(bins.BinBoundingBoxesX.Length >= binCount);
            Debug.Assert(bins.BinBoundingBoxesY.Length >= binCount);
            Debug.Assert(bins.BinBoundingBoxesZ.Length >= binCount);
            var centroidSpan = centroidMax - centroidMin;
            var offsetToBinIndex = new Vector4(binCount) / centroidSpan;
            //Avoid letting NaNs into the offsetToBinIndex scale.
            var axisIsDegenerate = Vector128.LessThanOrEqual(centroidSpan.AsVector128(), Vector128.Create(1e-12f));
            offsetToBinIndex = Vector128.ConditionalSelect(axisIsDegenerate, Vector128<float>.Zero, offsetToBinIndex.AsVector128()).AsVector4();

            if ((Vector128.ExtractMostSignificantBits(axisIsDegenerate) & 0b111) == 0b111)
            {
                //This node is completely degenerate; there is no 'good' ordering of the children. Pick a split in the middle and shrug.
                //This shouldn't happen unless something is badly wrong with the input; no point in optimizing it.
                var midpoint = indices.Length / 2;
                var secondCount = indices.Length - midpoint;
                //Still have to compute the child bounding boxes, because the centroid bounds span being zero doesn't imply that the full bounds are zero.
                BoundingBox4 boundsA, boundsB;
                boundsA.Min = new Vector4(float.MaxValue);
                boundsA.Max = new Vector4(float.MinValue);
                boundsB.Min = new Vector4(float.MaxValue);
                boundsB.Max = new Vector4(float.MinValue);
                for (int i = 0; i < midpoint; ++i)
                {
                    ref var bounds = ref boundingBoxes[i];
                    boundsA.Min = Vector4.Min(bounds.Min, boundsA.Min);
                    boundsA.Max = Vector4.Max(bounds.Max, boundsA.Max);
                }
                for (int i = midpoint; i < indices.Length; ++i)
                {
                    ref var bounds = ref boundingBoxes[i];
                    boundsB.Min = Vector4.Min(bounds.Min, boundsB.Min);
                    boundsB.Max = Vector4.Max(bounds.Max, boundsB.Max);
                }
                BuildParentNode(boundsA, boundsB, nodes, parentNodeIndex, midpoint, secondCount, out var aIndex, out var bIndex);
                BinnedBuilderInternal(indices.Slice(midpoint), boundingBoxes.Slice(midpoint), nodes.Slice(1, midpoint - 1), aIndex, bins);
                BinnedBuilderInternal(indices.Slice(midpoint, secondCount), boundingBoxes.Slice(midpoint, secondCount), nodes.Slice(midpoint, secondCount - 1), bIndex, bins);
            }

            for (int i = 0; i < binCount; ++i)
            {
                ref var boxX = ref bins.BinBoundingBoxesX[i];
                ref var boxY = ref bins.BinBoundingBoxesY[i];
                ref var boxZ = ref bins.BinBoundingBoxesZ[i];
                boxX.Min = new Vector4(float.MaxValue);
                boxX.Max = new Vector4(float.MinValue);
                boxY.Min = new Vector4(float.MaxValue);
                boxY.Max = new Vector4(float.MinValue);
                boxZ.Min = new Vector4(float.MaxValue);
                boxZ.Max = new Vector4(float.MinValue);
                bins.BinLeafCountsX[i] = 0;
                bins.BinLeafCountsY[i] = 0;
                bins.BinLeafCountsZ[i] = 0;
            }

            var maximumBinIndex = new Vector4(binCount - 1);
            for (int i = 0; i < leafCount; ++i)
            {
                ref var box = ref boundingBoxes[i];
                var centroid = box.Min + box.Max;
                var binIndicesForLeafContinuous = Vector4.Min(maximumBinIndex, (centroid - centroidMin) * offsetToBinIndex);
                //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
                //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
                var binIndicesForLeaf = Truncate(binIndicesForLeafContinuous);
                ref var xBounds = ref bins.BinBoundingBoxesX[binIndicesForLeaf.X];
                ref var yBounds = ref bins.BinBoundingBoxesY[binIndicesForLeaf.Y];
                ref var zBounds = ref bins.BinBoundingBoxesZ[binIndicesForLeaf.Z];
                xBounds.Min = Vector4.Min(xBounds.Min, box.Min);
                xBounds.Max = Vector4.Max(xBounds.Max, box.Max);
                yBounds.Min = Vector4.Min(yBounds.Min, box.Min);
                yBounds.Max = Vector4.Max(yBounds.Max, box.Max);
                zBounds.Min = Vector4.Min(zBounds.Min, box.Min);
                zBounds.Max = Vector4.Max(zBounds.Max, box.Max);
                ++bins.BinLeafCountsX[binIndicesForLeaf.X];
                ++bins.BinLeafCountsY[binIndicesForLeaf.Y];
                ++bins.BinLeafCountsZ[binIndicesForLeaf.Z];
            }

            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            bins.BinBoundingBoxesScanX[0] = bins.BinBoundingBoxesX[0];
            bins.BinBoundingBoxesScanY[0] = bins.BinBoundingBoxesY[0];
            bins.BinBoundingBoxesScanZ[0] = bins.BinBoundingBoxesZ[0];
            for (int i = 1; i < binCount; ++i)
            {
                var previousIndex = i - 1;
                ref var xBounds = ref bins.BinBoundingBoxesX[i];
                ref var yBounds = ref bins.BinBoundingBoxesY[i];
                ref var zBounds = ref bins.BinBoundingBoxesZ[i];
                ref var xScanBounds = ref bins.BinBoundingBoxesScanX[i];
                ref var yScanBounds = ref bins.BinBoundingBoxesScanY[i];
                ref var zScanBounds = ref bins.BinBoundingBoxesScanZ[i];
                ref var xPreviousScanBounds = ref bins.BinBoundingBoxesScanX[previousIndex];
                ref var yPreviousScanBounds = ref bins.BinBoundingBoxesScanY[previousIndex];
                ref var zPreviousScanBounds = ref bins.BinBoundingBoxesScanZ[previousIndex];
                xScanBounds.Min = Vector4.Min(xBounds.Min, xPreviousScanBounds.Min);
                xScanBounds.Max = Vector4.Max(xBounds.Max, xPreviousScanBounds.Max);
                yScanBounds.Min = Vector4.Min(yBounds.Min, yPreviousScanBounds.Min);
                yScanBounds.Max = Vector4.Max(yBounds.Max, yPreviousScanBounds.Max);
                zScanBounds.Min = Vector4.Min(zBounds.Min, zPreviousScanBounds.Min);
                zScanBounds.Max = Vector4.Max(zBounds.Max, zPreviousScanBounds.Max);
            }
            var leftBoundsX = bins.BinBoundingBoxesX[0];
            var leftBoundsY = bins.BinBoundingBoxesY[0];
            var leftBoundsZ = bins.BinBoundingBoxesZ[0];
            Debug.Assert(
                leftBoundsX.Min.X > float.MinValue && leftBoundsX.Min.Y > float.MinValue && leftBoundsX.Min.Z > float.MinValue &&
                leftBoundsY.Min.X > float.MinValue && leftBoundsY.Min.Y > float.MinValue && leftBoundsY.Min.Z > float.MinValue &&
                leftBoundsZ.Min.X > float.MinValue && leftBoundsZ.Min.Y > float.MinValue && leftBoundsZ.Min.Z > float.MinValue,
                "Bin 0 should have been updated in all cases because it is aligned with the minimum bin, and the centroid span isn't degenerate.");

            float bestX = float.MaxValue, bestY = float.MaxValue, bestZ = float.MaxValue;
            int bestSplitX = 1, bestSplitY = 1, bestSplitZ = 1;
            //The split index is going to end up in child B.
            var lastBinIndex = binCount - 1;
            BoundingBox4 accumulatedBoundingBoxBX, accumulatedBoundingBoxBY, accumulatedBoundingBoxBZ;
            accumulatedBoundingBoxBX = bins.BinBoundingBoxesX[lastBinIndex];
            accumulatedBoundingBoxBY = bins.BinBoundingBoxesY[lastBinIndex];
            accumulatedBoundingBoxBZ = bins.BinBoundingBoxesZ[lastBinIndex];
            BoundingBox4 bestBoundingBoxBX, bestBoundingBoxBY, bestBoundingBoxBZ;
            bestBoundingBoxBX = bins.BinBoundingBoxesX[lastBinIndex];
            bestBoundingBoxBY = bins.BinBoundingBoxesY[lastBinIndex];
            bestBoundingBoxBZ = bins.BinBoundingBoxesZ[lastBinIndex];
            int accumulatedLeafCountBX = bins.BinLeafCountsX[lastBinIndex];
            int accumulatedLeafCountBY = bins.BinLeafCountsY[lastBinIndex];
            int accumulatedLeafCountBZ = bins.BinLeafCountsZ[lastBinIndex];
            for (int splitIndexCandidate = lastBinIndex; splitIndexCandidate >= 1; --splitIndexCandidate)
            {
                var previousIndex = splitIndexCandidate - 1;
                var leafCountAX = leafCount - accumulatedLeafCountBX;
                var leafCountAY = leafCount - accumulatedLeafCountBY;
                var leafCountAZ = leafCount - accumulatedLeafCountBZ;
                var sahX = ComputeBoundsMetric(bins.BinBoundingBoxesScanX[previousIndex]) * leafCountAX + ComputeBoundsMetric(accumulatedBoundingBoxBX) * accumulatedLeafCountBX;
                var sahY = ComputeBoundsMetric(bins.BinBoundingBoxesScanY[previousIndex]) * leafCountAY + ComputeBoundsMetric(accumulatedBoundingBoxBY) * accumulatedLeafCountBY;
                var sahZ = ComputeBoundsMetric(bins.BinBoundingBoxesScanZ[previousIndex]) * leafCountAZ + ComputeBoundsMetric(accumulatedBoundingBoxBZ) * accumulatedLeafCountBZ;
                if (sahX < bestX)
                {
                    bestX = sahX;
                    bestSplitX = splitIndexCandidate;
                    bestBoundingBoxBX = accumulatedBoundingBoxBX;
                }
                if (sahY < bestY)
                {
                    bestY = sahY;
                    bestSplitY = splitIndexCandidate;
                    bestBoundingBoxBY = accumulatedBoundingBoxBY;
                }
                if (sahZ < bestZ)
                {
                    bestZ = sahZ;
                    bestSplitZ = splitIndexCandidate;
                    bestBoundingBoxBZ = accumulatedBoundingBoxBZ;
                }
                ref var xBounds = ref bins.BinBoundingBoxesX[splitIndexCandidate];
                ref var yBounds = ref bins.BinBoundingBoxesY[splitIndexCandidate];
                ref var zBounds = ref bins.BinBoundingBoxesZ[splitIndexCandidate];
                accumulatedBoundingBoxBX.Min = Vector4.Min(xBounds.Min, accumulatedBoundingBoxBX.Min);
                accumulatedBoundingBoxBX.Max = Vector4.Max(xBounds.Max, accumulatedBoundingBoxBX.Max);
                accumulatedBoundingBoxBY.Min = Vector4.Min(yBounds.Min, accumulatedBoundingBoxBY.Min);
                accumulatedBoundingBoxBY.Max = Vector4.Max(yBounds.Max, accumulatedBoundingBoxBY.Max);
                accumulatedBoundingBoxBZ.Min = Vector4.Min(zBounds.Min, accumulatedBoundingBoxBZ.Min);
                accumulatedBoundingBoxBZ.Max = Vector4.Max(zBounds.Max, accumulatedBoundingBoxBZ.Max);
                accumulatedLeafCountBX += bins.BinLeafCountsX[splitIndexCandidate];
                accumulatedLeafCountBY += bins.BinLeafCountsY[splitIndexCandidate];
                accumulatedLeafCountBZ += bins.BinLeafCountsZ[splitIndexCandidate];
            }

            //Choose the best SAH from all axes and split the indices/bounds into two halves for the children to operate on.
            int splitIndex;
            BoundingBox4 bestboundsA, bestboundsB;
            var bCount = 0;
            var aCount = 0;
            if (bestX < bestY && bestX < bestZ)
            {
                splitIndex = bestSplitX;
                bestboundsA = bins.BinBoundingBoxesScanX[bestSplitX - 1];
                bestboundsB = bestBoundingBoxBX;
                //Now we have the split index between bins. Go back through and sort the indices and bounds into two halves.
                while(aCount + bCount < leafCount)
                {
                    ref var box = ref boundingBoxes[aCount];
                    var centroid = box.Min + box.Max;
                    var binIndicesForLeafContinuous = Vector4.Min(maximumBinIndex, (centroid - centroidMin) * offsetToBinIndex);
                    //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
                    //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
                    var binIndex = (int)binIndicesForLeafContinuous.X;
                    if (binIndex >= splitIndex)
                    {
                        //Belongs to B. Swap it.
                        var targetIndex = leafCount - bCount - 1;
                        Helpers.Swap(ref indices[targetIndex], ref indices[aCount]);
                        Helpers.Swap(ref boundingBoxes[targetIndex], ref boundingBoxes[aCount]);
                        ++bCount;
                        //(Note that we still need to examine what we just swapped into the slot! It may belong to B too!)
                    }
                    else
                    {
                        //Belongs to A, no movement necessary.
                        ++aCount;
                    }
                }
            }
            else if (bestY < bestZ)
            {
                splitIndex = bestSplitY;
                bestboundsA = bins.BinBoundingBoxesScanY[bestSplitY - 1];
                bestboundsB = bestBoundingBoxBY;
                while (aCount + bCount < leafCount)
                {
                    ref var box = ref boundingBoxes[aCount];
                    var centroid = box.Min + box.Max;
                    var binIndicesForLeafContinuous = Vector4.Min(maximumBinIndex, (centroid - centroidMin) * offsetToBinIndex);
                    //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
                    //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
                    var binIndex = (int)binIndicesForLeafContinuous.Y;
                    if (binIndex >= splitIndex)
                    {
                        //Belongs to B. Swap it.
                        var targetIndex = leafCount - bCount - 1;
                        Helpers.Swap(ref indices[targetIndex], ref indices[aCount]);
                        Helpers.Swap(ref boundingBoxes[targetIndex], ref boundingBoxes[aCount]);
                        ++bCount;
                        //(Note that we still need to examine what we just swapped into the slot! It may belong to B too!)
                    }
                    else
                    {
                        //Belongs to A, no movement necessary.
                        ++aCount;
                    }
                }
            }
            else
            {
                splitIndex = bestSplitZ;
                bestboundsA = bins.BinBoundingBoxesScanZ[bestSplitZ - 1];
                bestboundsB = bestBoundingBoxBZ;
                while (aCount + bCount < leafCount)
                {
                    ref var box = ref boundingBoxes[aCount];
                    var centroid = box.Min + box.Max;
                    var binIndicesForLeafContinuous = Vector4.Min(maximumBinIndex, (centroid - centroidMin) * offsetToBinIndex);
                    //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
                    //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
                    var binIndex = (int)binIndicesForLeafContinuous.Z;
                    if (binIndex >= splitIndex)
                    {
                        //Belongs to B. Swap it.
                        var targetIndex = leafCount - bCount - 1;
                        Helpers.Swap(ref indices[targetIndex], ref indices[aCount]);
                        Helpers.Swap(ref boundingBoxes[targetIndex], ref boundingBoxes[aCount]);
                        ++bCount;
                        //(Note that we still need to examine what we just swapped into the slot! It may belong to B too!)
                    }
                    else
                    {
                        //Belongs to A, no movement necessary.
                        ++aCount;
                    }
                }
            }

            {
                BuildParentNode(bestboundsA, bestboundsB, nodes, parentNodeIndex, leafCount - bCount, bCount, out var aIndex, out var bIndex);
            }

        }

        public static unsafe void BinnedBuilder(Buffer<int> indices, Buffer<BoundingBox> boundingBoxes, Buffer<Node> nodes, BufferPool pool)
        {
            var leafCount = indices.Length;
            Debug.Assert(boundingBoxes.Length >= leafCount, "The bounding boxes provided must cover the range of indices provided.");
            Debug.Assert(nodes.Length > leafCount - 1, "The output nodes must be able to contain the nodes created for the leaves.");
            if (leafCount == 0)
                return;
            if (leafCount == 1)
            {
                //If there's only one leaf, the tree has a special format: the root node has only one child.
                ref var root = ref nodes[0];
                root.A.Min = boundingBoxes[0].Min;
                root.A.Index = Encode(indices[0]);
                root.A.Max = boundingBoxes[0].Max;
                root.A.LeafCount = 1;
                root.B = default;
                return;
            }
            boundingBoxes = boundingBoxes.Slice(indices.Length);
            nodes = nodes.Slice(leafCount - 1);

            var binBoundsMemory = stackalloc BoundingBox4[MaximumBinCount * 6];
            Bins bins;
            bins.BinBoundingBoxesX = new Buffer<BoundingBox4>(binBoundsMemory, MaximumBinCount);
            bins.BinBoundingBoxesY = new Buffer<BoundingBox4>(binBoundsMemory + MaximumBinCount, MaximumBinCount);
            bins.BinBoundingBoxesZ = new Buffer<BoundingBox4>(binBoundsMemory + MaximumBinCount * 2, MaximumBinCount);
            bins.BinBoundingBoxesScanX = new Buffer<BoundingBox4>(binBoundsMemory + MaximumBinCount * 3, MaximumBinCount);
            bins.BinBoundingBoxesScanY = new Buffer<BoundingBox4>(binBoundsMemory + MaximumBinCount * 4, MaximumBinCount);
            bins.BinBoundingBoxesScanZ = new Buffer<BoundingBox4>(binBoundsMemory + MaximumBinCount * 5, MaximumBinCount);

            var binLeafCountsMemory = stackalloc int[MaximumBinCount * 3];
            bins.BinLeafCountsX = new Buffer<int>(binLeafCountsMemory, MaximumBinCount);
            bins.BinLeafCountsY = new Buffer<int>(binLeafCountsMemory + MaximumBinCount, MaximumBinCount);
            bins.BinLeafCountsZ = new Buffer<int>(binLeafCountsMemory + MaximumBinCount * 2, MaximumBinCount);

            //While we could avoid a recursive implementation, the overhead is low compared to the per-iteration cost.
            BinnedBuilderInternal(indices, boundingBoxes.As<BoundingBox4>(), nodes, 0, bins);
        }

    }
}
