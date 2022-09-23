using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;
using System.Threading.Tasks.Sources;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void BuildNode(
            BoundingBox4 a, BoundingBox4 b,
            int leafCountA, int leafCountB,
            Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<int> indices,
            int nodeIndex, int parentNodeIndex, int childIndexInParent, int subtreeCountA, int subtreeCountB, out int aIndex, out int bIndex)
        {
            ref var metanode = ref metanodes[0];
            metanode.Parent = parentNodeIndex;
            metanode.IndexInParent = childIndexInParent;
            metanode.RefineFlag = 0;
            ref var node = ref nodes[0];
            aIndex = subtreeCountA == 1 ? indices[0] : nodeIndex + 1;
            bIndex = subtreeCountB == 1 ? indices[^1] : nodeIndex + subtreeCountA;//parentNodeIndex + 1 + (subtreeCountA - 1)
            node.A = Unsafe.As<BoundingBox4, NodeChild>(ref a);
            node.B = Unsafe.As<BoundingBox4, NodeChild>(ref b);
            node.A.Index = aIndex;
            node.A.LeafCount = leafCountA;
            node.B.Index = bIndex;
            node.B.LeafCount = leafCountB;
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

        interface ILeafCountBuffer<T> where T : unmanaged, ILeafCountBuffer<T>
        {
            int this[int index] { get; set; }

            T Slice(int startIndex, int count);


        }

        /// <summary>
        /// An implicit buffer where every slot contains a 1.
        /// </summary>
        struct UnitLeafCount : ILeafCountBuffer<UnitLeafCount>
        {
            public int this[int index] { get => 1; set { } }

            public UnitLeafCount Slice(int startIndex, int count)
            {
                return this;
            }
        }

        /// <summary>
        /// Leaf counts buffer with actual values.
        /// </summary>
        struct LeafCountBuffer : ILeafCountBuffer<LeafCountBuffer>
        {
            public Buffer<int> LeafCounts;
            public int this[int index] { get => LeafCounts[index]; set => LeafCounts[index] = value; }

            public LeafCountBuffer Slice(int startIndex, int count) => new() { LeafCounts = LeafCounts.Slice(startIndex, count) };
        }

        struct Bins
        {
            public Buffer<BoundingBox4> BinBoundingBoxes;
            public Buffer<BoundingBox4> BinBoundingBoxesScan;
            public Buffer<int> BinLeafCounts;

            public int MinimumBinCount;
            public int MaximumBinCount;
            public float LeafToBinMultiplier;
            public int MicrosweepThreshold;

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

        static unsafe void MicroSweepForBinnedBuilder<TLeafCounts>(Vector4 centroidMin, Vector4 centroidMax, Buffer<int> indices, TLeafCounts leafCounts, Buffer<BoundingBox4> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, int nodeIndex, int parentNodeIndex, int childIndexInParent, Bins bins)
            where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            //This is a very small scale sweep build.
            var subtreeCount = indices.Length;
            if (subtreeCount == 2)
            {
                BuildNode(boundingBoxes[0], boundingBoxes[1], leafCounts[0], leafCounts[1], nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, out _, out _);
                return;
            }
            var centroidSpan = centroidMax - centroidMin;

            if (Vector256.IsHardwareAccelerated || Vector128.IsHardwareAccelerated)
            {
                //Repurpose the bins memory so we don't need to allocate any extra. The bins aren't in use right now anyway.
                int paddedKeyCount = Vector256.IsHardwareAccelerated ? ((subtreeCount + 7) / 8) * 8 : ((subtreeCount + 3) / 4) * 4;

                Debug.Assert(Unsafe.SizeOf<BoundingBox4>() * bins.BinBoundingBoxes.Length >= (paddedKeyCount * 2 + subtreeCount) * Unsafe.SizeOf<int>(),
                    "The bins should preallocate enough space to handle the needs of microsweeps. They reuse the same allocations.");
                var keys = new Buffer<float>(bins.BinBoundingBoxes.Memory, paddedKeyCount);
                var targetIndices = new Buffer<int>(keys.Memory + paddedKeyCount, paddedKeyCount);

                //Compute the axis centroids up front to avoid having to recompute them during a sort.
                if (centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z)
                {
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        ref var bounds = ref boundingBoxes[i];
                        keys[i] = bounds.Min.X + bounds.Max.X;
                    }
                }
                else if (centroidSpan.Y > centroidSpan.Z)
                {
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        ref var bounds = ref boundingBoxes[i];
                        keys[i] = bounds.Min.Y + bounds.Max.Y;
                    }
                }
                else
                {
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        ref var bounds = ref boundingBoxes[i];
                        keys[i] = bounds.Min.Z + bounds.Max.Z;
                    }
                }
                for (int i = subtreeCount; i < paddedKeyCount; ++i)
                {
                    keys[i] = float.MaxValue;
                }
                VectorizedSorts.VectorCountingSort(keys, targetIndices, subtreeCount);

                //Now that we know the target indices, copy things back.
                //Have to copy things into a temporary cache to avoid overwrites since we didn't do any shuffling during the sort.
                //Note that we can now reuse the keys memory.              
                if (typeof(TLeafCounts) != typeof(LeafCountBuffer))
                {
                    //There aren't any leaf counts that we need to copy; they're all just 1 anyway.
                    Debug.Assert(typeof(TLeafCounts) == typeof(UnitLeafCount));
                    var indicesCache = new Buffer<int>(bins.BinBoundingBoxes.Memory, subtreeCount);
                    var boundingBoxCache = bins.BinBoundingBoxesScan;
                    boundingBoxes.CopyTo(0, boundingBoxCache, 0, subtreeCount);
                    indices.CopyTo(0, indicesCache, 0, subtreeCount);
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        var targetIndex = targetIndices[i];
                        boundingBoxes[targetIndex] = boundingBoxCache[i];
                        indices[targetIndex] = indicesCache[i];
                    }
                }
                else
                {
                    //There are actual leaf counts we need to worry about on top of the rest!
                    var indicesCache = new Buffer<int>(bins.BinBoundingBoxes.Memory, subtreeCount);
                    var leafCountCache = new Buffer<int>(targetIndices.Memory + subtreeCount, subtreeCount);
                    var boundingBoxCache = bins.BinBoundingBoxesScan;
                    boundingBoxes.CopyTo(0, boundingBoxCache, 0, subtreeCount);
                    indices.CopyTo(0, indicesCache, 0, subtreeCount);
                    var leafCountBuffer = Unsafe.As<TLeafCounts, LeafCountBuffer>(ref leafCounts).LeafCounts;
                    leafCountBuffer.CopyTo(0, leafCountCache, 0, subtreeCount);
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        var targetIndex = targetIndices[i];
                        boundingBoxes[targetIndex] = boundingBoxCache[i];
                        indices[targetIndex] = indicesCache[i];
                        leafCountBuffer[targetIndex] = leafCountCache[i];
                    }
                }
            }
            else
            {
                //No vectorization supported. Fall back to poopymode!
                if (typeof(TLeafCounts) != typeof(LeafCountBuffer))
                {
                    if (centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z)
                    {
                        var comparer = new BoundsComparerX();
                        QuickSort.Sort(ref boundingBoxes[0], ref indices[0], 0, subtreeCount - 1, ref comparer);
                    }
                    else if (centroidSpan.Y > centroidSpan.Z)
                    {
                        var comparer = new BoundsComparerY();
                        QuickSort.Sort(ref boundingBoxes[0], ref indices[0], 0, subtreeCount - 1, ref comparer);
                    }
                    else
                    {
                        var comparer = new BoundsComparerZ();
                        QuickSort.Sort(ref boundingBoxes[0], ref indices[0], 0, subtreeCount - 1, ref comparer);
                    }
                }
                else
                {
                    //There are leaf counts that we need to sort alongside the rest. This is a pretty low value codepath, so we'll just create a targetIndices buffer.
                    var targetIndices = new Buffer<int>(bins.BinBoundingBoxes.Memory, subtreeCount);
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        targetIndices[i] = i;
                    }
                    if (centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z)
                    {
                        var comparer = new BoundsComparerX();
                        QuickSort.Sort(ref boundingBoxes[0], ref targetIndices[0], 0, subtreeCount - 1, ref comparer);
                    }
                    else if (centroidSpan.Y > centroidSpan.Z)
                    {
                        var comparer = new BoundsComparerY();
                        QuickSort.Sort(ref boundingBoxes[0], ref targetIndices[0], 0, subtreeCount - 1, ref comparer);
                    }
                    else
                    {
                        var comparer = new BoundsComparerZ();
                        QuickSort.Sort(ref boundingBoxes[0], ref targetIndices[0], 0, subtreeCount - 1, ref comparer);
                    }
                    //Apply the swaps to indices and leaf counts.
                    var indicesCache = new Buffer<int>(targetIndices.Memory + subtreeCount, subtreeCount);
                    var leafCountCache = new Buffer<int>(indicesCache.Memory + subtreeCount, subtreeCount);
                    indices.CopyTo(0, indicesCache, 0, subtreeCount);
                    var leafCountBuffer = Unsafe.As<TLeafCounts, LeafCountBuffer>(ref leafCounts).LeafCounts;
                    leafCountBuffer.CopyTo(0, leafCountCache, 0, subtreeCount);
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        var targetIndex = targetIndices[i];
                        leafCountBuffer[targetIndex] = leafCountCache[i];
                        indices[targetIndex] = indicesCache[i];
                    }
                }
            }

            Debug.Assert(subtreeCount <= bins.MaximumBinCount || subtreeCount < bins.MicrosweepThreshold, "We're reusing the bin resources under the assumption that this is only ever called when there are less leaves than maximum bins.");
            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            bins.BinBoundingBoxesScan[0] = boundingBoxes[0];
            int totalLeafCount = typeof(TLeafCounts) == typeof(LeafCountBuffer) ? leafCounts[0] : subtreeCount;
            for (int i = 1; i < subtreeCount; ++i)
            {
                var previousIndex = i - 1;
                ref var previousScanBounds = ref bins.BinBoundingBoxesScan[previousIndex];
                ref var scanBounds = ref bins.BinBoundingBoxesScan[i];
                ref var bounds = ref boundingBoxes[i];
                scanBounds.Min = Vector4.Min(bounds.Min, previousScanBounds.Min);
                scanBounds.Max = Vector4.Max(bounds.Max, previousScanBounds.Max);
                if (typeof(TLeafCounts) == typeof(LeafCountBuffer))
                    totalLeafCount += leafCounts[i];
            }

            float bestSAH = float.MaxValue;
            int bestSplit = 1;
            //The split index is going to end up in child B.
            var lastSubtreeIndex = subtreeCount - 1;
            BoundingBox4 accumulatedBoundingBoxB = boundingBoxes[lastSubtreeIndex];
            Unsafe.SkipInit(out BoundingBox4 bestBoundsB);
            int accumulatedLeafCountB = 1;
            int bestLeafCountB = 0;
            for (int splitIndexCandidate = lastSubtreeIndex; splitIndexCandidate >= 1; --splitIndexCandidate)
            {
                var previousIndex = splitIndexCandidate - 1;
                var sahCandidate =
                    ComputeBoundsMetric(bins.BinBoundingBoxesScan[previousIndex]) * (totalLeafCount - accumulatedLeafCountB) +
                    ComputeBoundsMetric(accumulatedBoundingBoxB) * accumulatedLeafCountB;
                if (sahCandidate < bestSAH)
                {
                    bestSAH = sahCandidate;
                    bestSplit = splitIndexCandidate;
                    bestBoundsB = accumulatedBoundingBoxB;
                    if (typeof(TLeafCounts) == typeof(LeafCountBuffer))
                        bestLeafCountB = accumulatedLeafCountB;
                }
                ref var bounds = ref boundingBoxes[previousIndex];
                accumulatedBoundingBoxB.Min = Vector4.Min(bounds.Min, accumulatedBoundingBoxB.Min);
                accumulatedBoundingBoxB.Max = Vector4.Max(bounds.Max, accumulatedBoundingBoxB.Max);
                accumulatedLeafCountB += leafCounts[previousIndex];
            }

            var bestBoundsA = bins.BinBoundingBoxesScan[bestSplit - 1];
            var subtreeCountA = bestSplit;
            var subtreeCountB = subtreeCount - bestSplit;
            var bestLeafCountA = typeof(TLeafCounts) == typeof(UnitLeafCount) ? subtreeCountA : totalLeafCount - bestLeafCountB;
            if (typeof(TLeafCounts) == typeof(UnitLeafCount))
                bestLeafCountB = subtreeCountB;

            BuildNode(bestBoundsA, bestBoundsB, bestLeafCountA, bestLeafCountB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, subtreeCountA, subtreeCountB, out var aIndex, out var bIndex);
            if (subtreeCountA > 1)
            {
                var aBounds = boundingBoxes.Slice(subtreeCountA);
                BoundingBox4 centroidBoundsA = aBounds[0];
                for (int i = 1; i < subtreeCountA; ++i)
                {
                    ref var bounds = ref aBounds[i];
                    centroidBoundsA.Min = Vector4.Min(centroidBoundsA.Min, bounds.Min);
                    centroidBoundsA.Max = Vector4.Max(centroidBoundsA.Max, bounds.Max);
                }
                MicroSweepForBinnedBuilder(centroidBoundsA.Min, centroidBoundsA.Max, indices.Slice(subtreeCountA), leafCounts.Slice(0, subtreeCountA), aBounds, nodes.Slice(1, subtreeCountA - 1), metanodes.Slice(1, subtreeCountA - 1), aIndex, nodeIndex, 0, bins);
            }
            if (subtreeCountB > 1)
            {
                var bBounds = boundingBoxes.Slice(subtreeCountA, subtreeCountB);
                BoundingBox4 centroidBoundsB = bBounds[0];
                for (int i = 0; i < subtreeCountB; ++i)
                {
                    ref var bounds = ref bBounds[i];
                    centroidBoundsB.Min = Vector4.Min(centroidBoundsB.Min, bounds.Min);
                    centroidBoundsB.Max = Vector4.Max(centroidBoundsB.Max, bounds.Max);
                }
                MicroSweepForBinnedBuilder(centroidBoundsB.Min, centroidBoundsB.Max, indices.Slice(subtreeCountA, subtreeCountB), leafCounts.Slice(subtreeCountA, subtreeCountB), bBounds, nodes.Slice(subtreeCountA, subtreeCountB - 1), metanodes.Slice(subtreeCountA, subtreeCountB - 1), bIndex, nodeIndex, 1, bins);
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

        static unsafe void BinnedBuilderInternal<TLeafCounts>(Buffer<int> indices, TLeafCounts leafCounts, Buffer<BoundingBox4> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes,
            int nodeIndex, int parentNodeIndex, int childIndexInParent, in Bins bins)
            where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            var subtreeCount = indices.Length;
            if (subtreeCount == 2)
            {
                BuildNode(boundingBoxes[0], boundingBoxes[1], leafCounts[0], leafCounts[1], nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, out _, out _);
                return;
            }
            var centroidMin = new Vector4(float.MaxValue);
            var centroidMax = new Vector4(float.MinValue);

            for (int i = 0; i < subtreeCount; ++i)
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
                var degenerateSubtreeCountA = indices.Length / 2;
                var degenerateSubtreeCountB = indices.Length - degenerateSubtreeCountA;
                //Still have to compute the child bounding boxes, because the centroid bounds span being zero doesn't imply that the full bounds are zero.
                BoundingBox4 boundsA, boundsB;
                boundsA.Min = new Vector4(float.MaxValue);
                boundsA.Max = new Vector4(float.MinValue);
                boundsB.Min = new Vector4(float.MaxValue);
                boundsB.Max = new Vector4(float.MinValue);
                int degenerateLeafCountA = 0, degenerateLeafCountB = 0;
                for (int i = 0; i < degenerateSubtreeCountA; ++i)
                {
                    ref var bounds = ref boundingBoxes[i];
                    boundsA.Min = Vector4.Min(bounds.Min, boundsA.Min);
                    boundsA.Max = Vector4.Max(bounds.Max, boundsA.Max);
                    degenerateLeafCountA += leafCounts[i];
                }
                for (int i = degenerateSubtreeCountA; i < indices.Length; ++i)
                {
                    ref var bounds = ref boundingBoxes[i];
                    boundsB.Min = Vector4.Min(bounds.Min, boundsB.Min);
                    boundsB.Max = Vector4.Max(bounds.Max, boundsB.Max);
                    degenerateLeafCountB += leafCounts[i];
                }
                BuildNode(boundsA, boundsB, degenerateLeafCountA, degenerateLeafCountB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, degenerateSubtreeCountA, degenerateSubtreeCountB, out var aIndex, out var bIndex);
                if (degenerateSubtreeCountA > 1)
                    BinnedBuilderInternal(indices.Slice(degenerateSubtreeCountA), leafCounts.Slice(0, degenerateSubtreeCountA), boundingBoxes.Slice(degenerateSubtreeCountA), nodes.Slice(1, degenerateSubtreeCountA - 1), metanodes.Slice(1, degenerateSubtreeCountA - 1), aIndex, nodeIndex, 0, bins);
                if (degenerateSubtreeCountB > 1)
                    BinnedBuilderInternal(indices.Slice(degenerateSubtreeCountA, degenerateSubtreeCountB), leafCounts.Slice(degenerateSubtreeCountA, degenerateSubtreeCountB), boundingBoxes.Slice(degenerateSubtreeCountA, degenerateSubtreeCountB), nodes.Slice(degenerateSubtreeCountA, degenerateSubtreeCountB - 1), metanodes.Slice(degenerateSubtreeCountA, degenerateSubtreeCountB - 1), bIndex, nodeIndex, 1, bins);
                return;
            }

            if (subtreeCount <= bins.MicrosweepThreshold)
            {
                MicroSweepForBinnedBuilder(centroidMin, centroidMax, indices, leafCounts, boundingBoxes, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, bins);
                return;
            }

            var useX = centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z;
            var useY = centroidSpan.Y > centroidSpan.Z;
            //These will be used conditionally based on what hardware acceleration is available. Pretty minor detail.
            var permuteMask = Vector128.Create(useX ? 0 : useY ? 1 : 2, 0, 0, 0);
            var axisIndex = useX ? 0 : useY ? 1 : 2;

            var binCount = Math.Min(bins.MaximumBinCount, Math.Max((int)(subtreeCount * bins.LeafToBinMultiplier), bins.MinimumBinCount));
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
            for (int i = 0; i < subtreeCount; ++i)
            {
                ref var box = ref boundingBoxes[i];
                var binIndex = ComputeBinIndex(centroidMin, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
                ref var xBounds = ref bins.BinBoundingBoxes[binIndex];
                xBounds.Min = Vector4.Min(xBounds.Min, box.Min);
                xBounds.Max = Vector4.Max(xBounds.Max, box.Max);
                bins.BinLeafCounts[binIndex] += leafCounts[i];
            }

            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            bins.BinBoundingBoxesScan[0] = bins.BinBoundingBoxes[0];
            int totalLeafCount = typeof(TLeafCounts) == typeof(LeafCountBuffer) ? bins.BinLeafCounts[0] : subtreeCount;
            for (int i = 1; i < binCount; ++i)
            {
                var previousIndex = i - 1;
                ref var xBounds = ref bins.BinBoundingBoxes[i];
                ref var xScanBounds = ref bins.BinBoundingBoxesScan[i];
                ref var xPreviousScanBounds = ref bins.BinBoundingBoxesScan[previousIndex];
                xScanBounds.Min = Vector4.Min(xBounds.Min, xPreviousScanBounds.Min);
                xScanBounds.Max = Vector4.Max(xBounds.Max, xPreviousScanBounds.Max);
                if (typeof(TLeafCounts) == typeof(LeafCountBuffer))
                    totalLeafCount += bins.BinLeafCounts[i];
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
            int bestLeafCountB = 0;
            for (int splitIndexCandidate = lastBinIndex; splitIndexCandidate >= 1; --splitIndexCandidate)
            {
                var previousIndex = splitIndexCandidate - 1;
                var sahCandidate = ComputeBoundsMetric(bins.BinBoundingBoxesScan[previousIndex]) * (totalLeafCount - accumulatedLeafCountB) + ComputeBoundsMetric(accumulatedBoundingBoxB) * accumulatedLeafCountB;

                if (sahCandidate < bestSAH)
                {
                    bestSAH = sahCandidate;
                    bestSplit = splitIndexCandidate;
                    bestBoundingBoxB = accumulatedBoundingBoxB;
                    if (typeof(TLeafCounts) == typeof(LeafCountBuffer))
                        bestLeafCountB = accumulatedLeafCountB;
                }
                ref var xBounds = ref bins.BinBoundingBoxes[previousIndex];
                accumulatedBoundingBoxB.Min = Vector4.Min(xBounds.Min, accumulatedBoundingBoxB.Min);
                accumulatedBoundingBoxB.Max = Vector4.Max(xBounds.Max, accumulatedBoundingBoxB.Max);
                accumulatedLeafCountB += bins.BinLeafCounts[previousIndex];
            }

            //Choose the best SAH from all axes and split the indices/bounds into two halves for the children to operate on.
            var subtreeCountB = 0;
            var subtreeCountA = 0;
            var splitIndex = bestSplit;
            var bestboundsA = bins.BinBoundingBoxesScan[bestSplit - 1];
            var bestboundsB = bestBoundingBoxB;
            //Now we have the split index between bins. Go back through and sort the indices and bounds into two halves.
            while (subtreeCountA + subtreeCountB < subtreeCount)
            {
                ref var box = ref boundingBoxes[subtreeCountA];
                var binIndex = ComputeBinIndex(centroidMin, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
                if (binIndex >= splitIndex)
                {
                    //Belongs to B. Swap it.
                    var targetIndex = subtreeCount - subtreeCountB - 1;
                    Helpers.Swap(ref indices[targetIndex], ref indices[subtreeCountA]);
                    if (typeof(TLeafCounts) == typeof(LeafCountBuffer))
                    {
                        var tempTarget = leafCounts[targetIndex];
                        var tempACount = leafCounts[subtreeCountA];
                        leafCounts[subtreeCountA] = tempTarget;
                        leafCounts[targetIndex] = tempACount;
                    }
                    if (Vector256.IsHardwareAccelerated)
                    {
                        var targetMemory = (byte*)(boundingBoxes.Memory + targetIndex);
                        var aCountMemory = (byte*)(boundingBoxes.Memory + subtreeCountA);
                        var targetVector = Vector256.Load(targetMemory);
                        var aCountVector = Vector256.Load(aCountMemory);
                        Vector256.Store(aCountVector, targetMemory);
                        Vector256.Store(targetVector, aCountMemory);
                    }
                    else
                    {
                        Helpers.Swap(ref boundingBoxes[targetIndex], ref boundingBoxes[subtreeCountA]);
                    }
                    ++subtreeCountB;
                    //(Note that we still need to examine what we just swapped into the slot! It may belong to B too!)
                }
                else
                {
                    //Belongs to A, no movement necessary.
                    ++subtreeCountA;
                }
            }

            var leafCountB = typeof(TLeafCounts) == typeof(UnitLeafCount) ? subtreeCountB : bestLeafCountB;
            var leafCountA = typeof(TLeafCounts) == typeof(UnitLeafCount) ? subtreeCountA : totalLeafCount - leafCountB;

            {
                Debug.Assert(subtreeCountA + subtreeCountB == subtreeCount);
                BuildNode(bestboundsA, bestboundsB, leafCountA, leafCountB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, subtreeCountA, subtreeCountB, out var aIndex, out var bIndex);
                if (subtreeCountA > 1)
                    BinnedBuilderInternal(indices.Slice(subtreeCountA), leafCounts.Slice(0, subtreeCountA), boundingBoxes.Slice(subtreeCountA), nodes.Slice(1, subtreeCountA - 1), metanodes.Slice(1, subtreeCountA - 1), aIndex, nodeIndex, 0, bins);
                if (subtreeCountB > 1)
                    BinnedBuilderInternal(indices.Slice(subtreeCountA, subtreeCountB), leafCounts.Slice(subtreeCountA, subtreeCountB), boundingBoxes.Slice(subtreeCountA, subtreeCountB), nodes.Slice(subtreeCountA, subtreeCountB - 1), metanodes.Slice(subtreeCountA, subtreeCountB - 1), bIndex, nodeIndex, 1, bins);
            }
        }

        public static unsafe void BinnedBuilder(Buffer<int> indices, Buffer<BoundingBox> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, BufferPool pool,
            int minimumBinCount = 16, int maximumBinCount = 64, float leafToBinMultiplier = 1 / 16f, int microsweepThreshold = 64)
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

            //Don't let the user pick values that will just cause an explosion.
            Debug.Assert(minimumBinCount >= 2 && maximumBinCount >= 2, "At least two bins are required. In release mode, this will be clamped up to 2, but where did lower values come from?");
            minimumBinCount = Math.Max(2, minimumBinCount);
            maximumBinCount = Math.Max(2, maximumBinCount);
            //The microsweep uses the same resources as the bin allocations, so expand to hold whichever is larger.
            var allocatedBinCount = Math.Max(maximumBinCount, microsweepThreshold);
            var binBoundsMemory = stackalloc BoundingBox4[allocatedBinCount * 2 + 1];
            //Should be basically irrelevant, but just in case it's not on some platform, align the allocation.
            binBoundsMemory = (BoundingBox4*)(((ulong)binBoundsMemory + 31ul) & (~31ul));

            Bins bins;
            bins.BinBoundingBoxes = new Buffer<BoundingBox4>(binBoundsMemory, allocatedBinCount);
            bins.BinBoundingBoxesScan = new Buffer<BoundingBox4>(binBoundsMemory + allocatedBinCount, allocatedBinCount);

            var binLeafCountsMemory = stackalloc int[allocatedBinCount * 2];
            bins.BinLeafCounts = new Buffer<int>(binLeafCountsMemory, allocatedBinCount);

            bins.MinimumBinCount = minimumBinCount;
            bins.MaximumBinCount = maximumBinCount;
            bins.LeafToBinMultiplier = leafToBinMultiplier;
            bins.MicrosweepThreshold = microsweepThreshold;

            var leafCounts = new UnitLeafCount();

            //While we could avoid a recursive implementation, the overhead is low compared to the per-iteration cost.
            BinnedBuilderInternal(indices, leafCounts, boundingBoxes.As<BoundingBox4>(), nodes, metanodes, 0, -1, -1, bins);
        }

    }
}
