using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.ComponentModel;
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
        struct LeavesHandledInPostPass { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void BuildNode<TLeaves>(
            BoundingBox4 a, BoundingBox4 b,
            int leafCountA, int leafCountB,
            Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<int> indices,
            int nodeIndex, int parentNodeIndex, int childIndexInParent, int subtreeCountA, int subtreeCountB, ref TLeaves leaves, out int aIndex, out int bIndex)
            where TLeaves : unmanaged
        {
            Debug.Assert(typeof(TLeaves) == typeof(LeavesHandledInPostPass) || typeof(TLeaves) == typeof(Buffer<Leaf>), "While we didn't bother with an interface here, we assume one of two types only.");
            ref var metanode = ref metanodes[0];
            metanode.Parent = parentNodeIndex;
            metanode.IndexInParent = childIndexInParent;
            metanode.RefineFlag = 0;
            ref var node = ref nodes[0];
            if (subtreeCountA == 1)
            {
                aIndex = indices[0];
                if (typeof(TLeaves) == typeof(Buffer<Leaf>))
                {
                    Debug.Assert(aIndex < 0, "During building, any subtreeCount of 1 should imply a leaf.");
                    //This is a leaf node, and this is a direct builder execution, so write to the leaf data.
                    Unsafe.As<TLeaves, Buffer<Leaf>>(ref leaves)[Encode(aIndex)] = new Leaf(nodeIndex, 0);
                }
            }
            else
            {
                aIndex = nodeIndex + 1;
            }
            if (subtreeCountB == 1)
            {
                bIndex = indices[^1];
                if (typeof(TLeaves) == typeof(Buffer<Leaf>))
                {
                    Debug.Assert(bIndex < 0, "During building, any subtreeCount of 1 should imply a leaf.");
                    //This is a leaf node, and this is a direct builder execution, so write to the leaf data.
                    Unsafe.As<TLeaves, Buffer<Leaf>>(ref leaves)[Encode(bIndex)] = new Leaf(nodeIndex, 1);
                }
            }
            else
            {
                bIndex = nodeIndex + subtreeCountA; //parentNodeIndex + 1 + (subtreeCountA - 1)
            }
            node.A = Unsafe.As<BoundingBox4, NodeChild>(ref a);
            node.B = Unsafe.As<BoundingBox4, NodeChild>(ref b);
            node.A.Index = aIndex;
            node.A.LeafCount = leafCountA;
            node.B.Index = bIndex;
            node.B.LeafCount = leafCountB;
        }

        internal static float ComputeBoundsMetric(BoundingBox4 bounds) => ComputeBoundsMetric(bounds.Min, bounds.Max);
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
            public UnitLeafCount Slice(int startIndex, int count) => this;
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

        interface IBinnedBuilderThreading
        {
            void GetBins(int workerIndex, out Buffer<BoundingBox4> binBoundingBoxes, out Buffer<BoundingBox4> binBoundingBoxesScan, out Buffer<int> binLeafCounts);
        }


        struct Context<TLeafCounts, TLeaves, TThreading>
            where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
            where TLeaves : unmanaged
            where TThreading : unmanaged, IBinnedBuilderThreading
        {
            public int MinimumBinCount;
            public int MaximumBinCount;
            public float LeafToBinMultiplier;
            public int MicrosweepThreshold;

            public Buffer<int> Indices;
            public TLeafCounts LeafCounts;
            public TLeaves Leaves;
            public Buffer<BoundingBox4> BoundingBoxes;
            public Buffer<Node> Nodes;
            public Buffer<Metanode> Metanodes;

            public TThreading Threading;

            public Context(int minimumBinCount, int maximumBinCount, float leafToBinMultiplier, int microsweepThreshold,
                Buffer<int> indices, TLeafCounts leafCounts, TLeaves leaves, Buffer<BoundingBox4> boundingBoxes,
                Buffer<Node> nodes, Buffer<Metanode> metanodes, TThreading threading)
            {
                MinimumBinCount = minimumBinCount;
                MaximumBinCount = maximumBinCount;
                LeafToBinMultiplier = leafToBinMultiplier;
                MicrosweepThreshold = microsweepThreshold;
                Indices = indices;
                LeafCounts = leafCounts;
                Leaves = leaves;
                BoundingBoxes = boundingBoxes;
                Nodes = nodes;
                Metanodes = metanodes;
                Threading = threading;
            }
        }

        struct BoundsComparerX : IComparerRef<BoundingBox4> { public int Compare(ref BoundingBox4 a, ref BoundingBox4 b) => (a.Min.X + a.Max.X) > (b.Min.X + b.Max.X) ? -1 : 1; }
        struct BoundsComparerY : IComparerRef<BoundingBox4> { public int Compare(ref BoundingBox4 a, ref BoundingBox4 b) => (a.Min.Y + a.Max.Y) > (b.Min.Y + b.Max.Y) ? -1 : 1; }
        struct BoundsComparerZ : IComparerRef<BoundingBox4> { public int Compare(ref BoundingBox4 a, ref BoundingBox4 b) => (a.Min.Z + a.Max.Z) > (b.Min.Z + b.Max.Z) ? -1 : 1; }

        static unsafe void MicroSweepForBinnedBuilder<TLeafCounts, TLeaves, TThreading>(
            Vector4 centroidMin, Vector4 centroidMax, Buffer<int> indices, TLeafCounts leafCounts, ref TLeaves leaves,
            Buffer<BoundingBox4> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, int nodeIndex, int parentNodeIndex, int childIndexInParent, Context<TLeafCounts, TLeaves, TThreading>* context, int workerIndex)
            where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts> where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            //This is a very small scale sweep build.
            var subtreeCount = indices.Length;
            if (subtreeCount == 2)
            {
                BuildNode(boundingBoxes[0], boundingBoxes[1], leafCounts[0], leafCounts[1], nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, ref leaves, out _, out _);
                return;
            }
            var centroidSpan = centroidMax - centroidMin;
            context->Threading.GetBins(workerIndex, out var binBoundingBoxes, out var binBoundingBoxesScan, out var binLeafCounts);

            if (Vector256.IsHardwareAccelerated || Vector128.IsHardwareAccelerated)
            {
                //Repurpose the bins memory so we don't need to allocate any extra. The bins aren't in use right now anyway.
                int paddedKeyCount = Vector256.IsHardwareAccelerated ? ((subtreeCount + 7) / 8) * 8 : ((subtreeCount + 3) / 4) * 4;

                Debug.Assert(Unsafe.SizeOf<BoundingBox4>() * binBoundingBoxes.Length >= (paddedKeyCount * 2 + subtreeCount) * Unsafe.SizeOf<int>(),
                    "The bins should preallocate enough space to handle the needs of microsweeps. They reuse the same allocations.");
                var keys = new Buffer<float>(binBoundingBoxes.Memory, paddedKeyCount);
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
                    var indicesCache = new Buffer<int>(binBoundingBoxes.Memory, subtreeCount);
                    var boundingBoxCache = binBoundingBoxesScan;
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
                    var indicesCache = new Buffer<int>(binBoundingBoxes.Memory, subtreeCount);
                    var leafCountCache = new Buffer<int>(targetIndices.Memory + subtreeCount, subtreeCount);
                    var boundingBoxCache = binBoundingBoxesScan;
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
                    var targetIndices = new Buffer<int>(binBoundingBoxes.Memory, subtreeCount);
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

            Debug.Assert(subtreeCount <= context->MaximumBinCount || subtreeCount < context->MicrosweepThreshold, "We're reusing the bin resources under the assumption that this is only ever called when there are less leaves than maximum bins.");
            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            binBoundingBoxesScan[0] = boundingBoxes[0];
            int totalLeafCount = typeof(TLeafCounts) == typeof(LeafCountBuffer) ? leafCounts[0] : subtreeCount;
            for (int i = 1; i < subtreeCount; ++i)
            {
                var previousIndex = i - 1;
                ref var previousScanBounds = ref binBoundingBoxesScan[previousIndex];
                ref var scanBounds = ref binBoundingBoxesScan[i];
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
                    ComputeBoundsMetric(binBoundingBoxesScan[previousIndex]) * (totalLeafCount - accumulatedLeafCountB) +
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

            var bestBoundsA = binBoundingBoxesScan[bestSplit - 1];
            var subtreeCountA = bestSplit;
            var subtreeCountB = subtreeCount - bestSplit;
            var bestLeafCountA = typeof(TLeafCounts) == typeof(UnitLeafCount) ? subtreeCountA : totalLeafCount - bestLeafCountB;
            if (typeof(TLeafCounts) == typeof(UnitLeafCount))
                bestLeafCountB = subtreeCountB;

            BuildNode(bestBoundsA, bestBoundsB, bestLeafCountA, bestLeafCountB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, subtreeCountA, subtreeCountB, ref leaves, out var aIndex, out var bIndex);
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
                MicroSweepForBinnedBuilder(centroidBoundsA.Min, centroidBoundsA.Max, indices.Slice(subtreeCountA), leafCounts.Slice(0, subtreeCountA), ref leaves, aBounds, nodes.Slice(1, subtreeCountA - 1), metanodes.Slice(1, subtreeCountA - 1), aIndex, nodeIndex, 0, context, workerIndex);
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
                MicroSweepForBinnedBuilder(centroidBoundsB.Min, centroidBoundsB.Max, indices.Slice(subtreeCountA, subtreeCountB), leafCounts.Slice(subtreeCountA, subtreeCountB), ref leaves, bBounds, nodes.Slice(subtreeCountA, subtreeCountB - 1), metanodes.Slice(subtreeCountA, subtreeCountB - 1), bIndex, nodeIndex, 1, context, workerIndex);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static unsafe int ComputeBinIndex(Vector4 centroidMin, bool useX, bool useY, Vector128<int> permuteMask, int axisIndex, Vector4 offsetToBinIndex, Vector4 maximumBinIndex, in BoundingBox4 box)
        {
            var centroid = box.Min + box.Max;
            var binIndicesForLeafContinuous = Vector4.Min(maximumBinIndex, (centroid - centroidMin) * offsetToBinIndex);
            //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
            //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
            //To extract the desired lane, we need to use a variable shuffle mask. At the time of writing, the Vector128 cross platform shuffle did not like variable masks.
            if (Avx.IsSupported)
                return (int)Vector128.ToScalar(Avx.PermuteVar(binIndicesForLeafContinuous.AsVector128(), permuteMask));
            else if (Vector128.IsHardwareAccelerated)
                return (int)Vector128.GetElement(binIndicesForLeafContinuous.AsVector128(), axisIndex);
            else
                return (int)(useX ? binIndicesForLeafContinuous.X : useY ? binIndicesForLeafContinuous.Y : binIndicesForLeafContinuous.Z);
        }

        struct SingleThreaded : IBinnedBuilderThreading
        {
            public Buffer<BoundingBox4> BinBoundingBoxes;
            public Buffer<BoundingBox4> BinBoundingBoxesScan;
            public Buffer<int> BinLeafCounts;

            public void GetBins(int workerIndex, out Buffer<BoundingBox4> binBoundingBoxes, out Buffer<BoundingBox4> binBoundingBoxesScan, out Buffer<int> binLeafCounts)
            {
                binBoundingBoxes = BinBoundingBoxes;
                binBoundingBoxesScan = BinBoundingBoxesScan;
                binLeafCounts = BinLeafCounts;
            }
        }

        struct BinSubtreesWorkerContext
        {
            public Buffer<BoundingBox4> BinBoundingBoxes;
            public Buffer<int> BinLeafCounts;
        }

        /// <summary>
        /// Stores resources required by a worker to dispatch and manage multithreaded work.
        /// </summary>
        unsafe struct BinnedBuildWorkerContext<TLeafCounts> where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            /// <summary>
            /// Stores per-worker prepass bounds accumulated over multiple tasks. If there are less tasks than workers, then only the lower contiguous region of these bounds are used.
            /// </summary>
            public Buffer<BoundingBox4> PrepassWorkers;
            public Buffer<BinSubtreesWorkerContext> BinSubtreesWorkers;
            public Buffer<BoundingBox4> BinBoundingBoxesScan;

            public int BinCount;
            public Vector4 CentroidBoundsMin;
            public bool UseX, UseY;
            public Vector128<int> PermuteMask;
            public int AxisIndex;
            public Vector4 OffsetToBinIndex;
            public Vector4 MaximumBinIndex;

            /// <summary>
            /// Bounds associated with the node that invoked this multithreaded job.
            /// </summary>
            public Buffer<BoundingBox4> Bounds;
            /// <summary>
            /// Leaf counts associated with the node that invoked this multithreaded job.
            /// </summary>
            public TLeafCounts LeafCounts;

            public int SlotsPerTaskBase;
            public int SlotRemainder;
            public bool TaskCountFitsInWorkerCount;

            public BinnedBuildWorkerContext(Buffer<BoundingBox4> prepassWorkers, Buffer<BinSubtreesWorkerContext> binSubtreesWorkers)
            {
                //We let the caller preallocate the bounds to avoid an individual allocation for every worker.
                PrepassWorkers = prepassWorkers;
                BinSubtreesWorkers = binSubtreesWorkers;
            }

            public void GetSlotInterval(long taskId, out int start, out int count)
            {
                var remainderedTaskCount = Math.Min(SlotRemainder, taskId);
                var earlySlotCount = (SlotsPerTaskBase + 1) * remainderedTaskCount;
                var lateSlotCount = SlotsPerTaskBase * (taskId - remainderedTaskCount);
                start = (int)(earlySlotCount + lateSlotCount);
                count = taskId > SlotRemainder ? SlotsPerTaskBase : SlotsPerTaskBase + 1;
            }
        }
        unsafe struct MultithreadBinnedBuildContext<TLeafCounts> : IBinnedBuilderThreading where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            public TaskQueue* Queue;
            /// <summary>
            /// Maximum number of tasks any one job submission should create.
            /// If you have far more tasks than there are workers, adding more tasks just adds overhead without additional workstealing advantages.
            /// </summary>
            public int MaximumTaskCountPerSubmission;
            public Buffer<BinnedBuildWorkerContext<TLeafCounts>> Workers;


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int SetupTaskCounts(int slotCount, int slotsPerTaskTarget, int workerIndex)
            {
                Debug.Assert(BitOperations.IsPow2(slotsPerTaskTarget), "This mask trick requires power of 2 task size targets.");
                var mask = slotsPerTaskTarget - 1;
                int taskCount = Math.Min(MaximumTaskCountPerSubmission, (slotCount + mask) & mask);
                ref var worker = ref Workers[workerIndex];
                worker.SlotsPerTaskBase = slotCount / taskCount;
                worker.SlotRemainder = slotCount - taskCount * worker.SlotsPerTaskBase;
                worker.TaskCountFitsInWorkerCount = taskCount <= Workers.Length;
                return taskCount;
            }

            public void GetBins(int workerIndex, out Buffer<BoundingBox4> binBoundingBoxes, out Buffer<BoundingBox4> binBoundingBoxesScan, out Buffer<int> binLeafCounts)
            {
                ref var worker = ref Workers[workerIndex];
                ref var cache0 = ref worker.BinSubtreesWorkers[0];
                binBoundingBoxes = cache0.BinBoundingBoxes;
                binBoundingBoxesScan = worker.BinBoundingBoxesScan;
                binLeafCounts = cache0.BinLeafCounts;
            }
        }

        //These should be powers of 2 for maskhack reasons.
        const int SubtreesPerThreadForCentroidPrepass = 2048;
        const int SubtreesPerThreadForBinning = 512;
        const int SubtreesPerThreadForNodeJob = 512;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static BoundingBox4 ComputeCentroidBounds(Buffer<BoundingBox4> bounds)
        {
            BoundingBox4 centroidBounds;
            centroidBounds.Min = new Vector4(float.MaxValue);
            centroidBounds.Max = new Vector4(float.MinValue);
            for (int i = 0; i < bounds.Length; ++i)
            {
                ref var box = ref bounds[i];
                //Note that centroids never bother scaling by 0.5. It's fine as long as we're consistent.
                var centroid = box.Min + box.Max;
                centroidBounds.Min = Vector4.Min(centroidBounds.Min, centroid);
                centroidBounds.Max = Vector4.Max(centroidBounds.Max, centroid);
            }
            return centroidBounds;
        }

        unsafe static void CentroidPrepassWorker<TLeafCounts>(long taskId, void* untypedContext, int workerIndex) where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            ref var context = ref *(BinnedBuildWorkerContext<TLeafCounts>*)untypedContext;
            context.GetSlotInterval(taskId, out var start, out var count);
            var centroidBounds = ComputeCentroidBounds(context.Bounds.Slice(start, count));
            if (context.TaskCountFitsInWorkerCount)
            {
                //There were less tasks than workers; directly write into the slot without bothering to merge.
                context.PrepassWorkers[(int)taskId] = centroidBounds;
            }
            else
            {
                ref var workerBounds = ref context.PrepassWorkers[workerIndex];
                workerBounds.Min = Vector4.Min(workerBounds.Min, centroidBounds.Min);
                workerBounds.Max = Vector4.Max(workerBounds.Max, centroidBounds.Max);
            }
        }

        unsafe static BoundingBox4 MultithreadedCentroidPrepass<TLeafCounts>(MultithreadBinnedBuildContext<TLeafCounts>* context, int subtreeCount, int workerIndex) where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            var taskCount = context->SetupTaskCounts(subtreeCount, SubtreesPerThreadForCentroidPrepass, workerIndex);
            //Don't bother initializing more slots than we have tasks. Note that this requires special handling on the task level;
            //if we have less tasks than workers, then the task needs to distinguish that fact.
            var activeWorkerCount = Math.Min(context->Workers.Length, taskCount);
            ref var worker = ref context->Workers[workerIndex];
            if (taskCount > context->Workers.Length)
            {
                //Potentially multiple tasks per worker; we must preinitialize slots.
                for (int i = 0; i < activeWorkerCount; ++i)
                {
                    ref var workerBounds = ref worker.PrepassWorkers[i];
                    workerBounds.Min = new Vector4(float.MaxValue);
                    workerBounds.Max = new Vector4(float.MinValue);
                }
            }
            context->Queue->For(&CentroidPrepassWorker<TLeafCounts>, context, 0, taskCount, workerIndex);

            var centroidBounds = worker.PrepassWorkers[0];
            for (int i = 1; i < activeWorkerCount; ++i)
            {
                ref var workerBounds = ref worker.PrepassWorkers[i];
                centroidBounds.Min = Vector4.Min(workerBounds.Min, centroidBounds.Min);
                centroidBounds.Max = Vector4.Max(workerBounds.Max, centroidBounds.Max);
            }
            return centroidBounds;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void BinSubtrees<TLeafCounts>(Vector4 centroidBoundsMin,
            bool useX, bool useY, Vector128<int> permuteMask, int axisIndex, Vector4 offsetToBinIndex, Vector4 maximumBinIndex,
            Buffer<BoundingBox4> bounds, TLeafCounts leafCounts,
            Buffer<BoundingBox4> binBoundingBoxes, Buffer<int> binLeafCounts) where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
            //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
            for (int i = 0; i < bounds.Length; ++i)
            {
                ref var box = ref bounds[i];
                var binIndex = ComputeBinIndex(centroidBoundsMin, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
                ref var xBounds = ref binBoundingBoxes[binIndex];
                xBounds.Min = Vector4.Min(xBounds.Min, box.Min);
                xBounds.Max = Vector4.Max(xBounds.Max, box.Max);
                binLeafCounts[binIndex] += leafCounts[i];
            }
        }
        unsafe static void BinSubtreesWorker<TLeafCounts>(long taskId, void* untypedContext, int workerIndex) where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            ref var context = ref *(BinnedBuildWorkerContext<TLeafCounts>*)untypedContext;
            //Note that if we have more workers than tasks, we use the task id to index into the caches (and initialize the data here rather then before dispatching).
            ref var worker = ref context.BinSubtreesWorkers[context.TaskCountFitsInWorkerCount ? (int)taskId : workerIndex];
            if (context.TaskCountFitsInWorkerCount)
            {
                for (int i = 0; i < context.BinCount; ++i)
                {
                    ref var binBounds = ref worker.BinBoundingBoxes[i];
                    binBounds.Min = new Vector4(float.MaxValue);
                    binBounds.Max = new Vector4(float.MinValue);
                    worker.BinLeafCounts[i] = 0;
                }
            }
            context.GetSlotInterval(taskId, out var start, out var count);
            BinSubtrees(context.CentroidBoundsMin, context.UseX, context.UseY, context.PermuteMask, context.AxisIndex, context.OffsetToBinIndex, context.MaximumBinIndex,
                context.Bounds.Slice(start, count), context.LeafCounts.Slice(start, count), worker.BinBoundingBoxes, worker.BinLeafCounts);
        }

        unsafe static void MultithreadedBinSubtrees<TLeafCounts>(MultithreadBinnedBuildContext<TLeafCounts>* context,
            Vector4 centroidBoundsMin, bool useX, bool useY, Vector128<int> permuteMask, int axisIndex, Vector4 offsetToBinIndex, Vector4 maximumBinIndex,
            Buffer<BoundingBox4> bounds, TLeafCounts leafCounts, int binCount, int workerIndex) where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts>
        {
            var taskCount = context->SetupTaskCounts(bounds.Length, SubtreesPerThreadForBinning, workerIndex);
            //Don't bother initializing more slots than we have tasks. Note that this requires special handling on the task level;
            //if we have less tasks than workers, then the task needs to distinguish that fact.
            var activeWorkerCount = Math.Min(context->Workers.Length, taskCount);
            ref var worker = ref context->Workers[workerIndex];
            if (!worker.TaskCountFitsInWorkerCount)
            {
                //If there are more tasks than workers, then we need to preinitialize all the worker caches.
                for (int cacheIndex = 0; cacheIndex < activeWorkerCount; ++cacheIndex)
                {
                    ref var cache = ref worker.BinSubtreesWorkers[cacheIndex];
                    for (int i = 0; i < binCount; ++i)
                    {
                        ref var binBounds = ref cache.BinBoundingBoxes[i];
                        binBounds.Min = new Vector4(float.MaxValue);
                        binBounds.Max = new Vector4(float.MinValue);
                        cache.BinLeafCounts[i] = 0;
                    }
                }
            }
            worker.BinCount = binCount;
            worker.CentroidBoundsMin = centroidBoundsMin;
            worker.UseX = useX;
            worker.UseY = useY;
            worker.PermuteMask = permuteMask;
            worker.AxisIndex = axisIndex;
            worker.OffsetToBinIndex = offsetToBinIndex;
            worker.MaximumBinIndex = maximumBinIndex;

            context->Queue->For(&BinSubtreesWorker<TLeafCounts>, context, 0, taskCount, workerIndex);

            //Unless the number of threads and bins is really huge, there's no value in attempting to multithread the final compression.
            //(Parallel reduction is an option, but even then... I suspect the single threaded version will be faster. And it's way simpler.)
            //Merge everything into cache 0. That's the source of bins for the worker that invoked this dispatch.
            ref var cache0 = ref worker.BinSubtreesWorkers[0];
            for (int cacheIndex = 1; cacheIndex < activeWorkerCount; ++cacheIndex)
            {
                ref var cache = ref worker.BinSubtreesWorkers[cacheIndex];
                for (int binIndex = 0; binIndex < binCount; ++binIndex)
                {
                    ref var b0 = ref cache0.BinBoundingBoxes[binIndex];
                    ref var bi = ref cache.BinBoundingBoxes[binIndex];
                    b0.Min = Vector4.Min(b0.Min, bi.Min);
                    b0.Max = Vector4.Min(b0.Max, bi.Max);
                    cache0.BinLeafCounts[binIndex] += cache.BinLeafCounts[binIndex];
                }
            }
        }

        unsafe struct NodePushTaskContext<TLeafCounts, TLeaves, TThreading>
            where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts> where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            public Context<TLeafCounts, TLeaves, TThreading>* Context;
            public int NodeIndex;
            public int ParentNodeIndex;
            //Subtree region start index and subtree count are both encoded into the task id.
        }
        unsafe static void BinnedBuilderNodeWorker<TLeafCounts, TLeaves, TThreading>(long taskId, void* context, int workerIndex)
            where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts> where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            var subtreeRegionStartIndex = (int)taskId;
            var subtreeCount = (int)(taskId >> 32);
            var wrappedContext = (NodePushTaskContext<TLeafCounts, TLeaves, TThreading>*)context;
            //Note that child index is always 1 because we only ever push child B.
            BinnedBuilderInternal(subtreeRegionStartIndex, wrappedContext->NodeIndex, subtreeCount, wrappedContext->ParentNodeIndex, 1, wrappedContext->Context, workerIndex);
        }

        static unsafe void BinnedBuilderInternal<TLeafCounts, TLeaves, TThreading>(
            int subtreeRegionStartIndex, int nodeIndex, int subtreeCount, int parentNodeIndex, int childIndexInParent, Context<TLeafCounts, TLeaves, TThreading>* context, int workerIndex)
            where TLeafCounts : unmanaged, ILeafCountBuffer<TLeafCounts> where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            var indices = context->Indices.Slice(subtreeRegionStartIndex, subtreeCount);
            var boundingBoxes = context->BoundingBoxes.Slice(subtreeRegionStartIndex, subtreeCount);
            var leafCounts = context->LeafCounts.Slice(subtreeRegionStartIndex, subtreeCount);
            var nodeCount = subtreeCount - 1;
            var nodes = context->Nodes.Slice(nodeIndex, nodeCount);
            var metanodes = context->Metanodes.Slice(nodeIndex, nodeCount);
            if (subtreeCount == 2)
            {
                BuildNode(boundingBoxes[0], boundingBoxes[1], leafCounts[0], leafCounts[1], nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, ref context->Leaves, out _, out _);
                return;
            }
            BoundingBox4 centroidBounds;
            if (typeof(TThreading) == typeof(SingleThreaded) || subtreeCount < SubtreesPerThreadForCentroidPrepass)
            {
                centroidBounds = ComputeCentroidBounds(boundingBoxes);
            }
            else
            {
                centroidBounds = MultithreadedCentroidPrepass(
                    (MultithreadBinnedBuildContext<TLeafCounts>*)Unsafe.AsPointer(ref Unsafe.As<TThreading, MultithreadBinnedBuildContext<TLeafCounts>>(ref context->Threading)), subtreeCount, workerIndex);
            }
            var centroidSpan = centroidBounds.Max - centroidBounds.Min;
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
                BuildNode(boundsA, boundsB, degenerateLeafCountA, degenerateLeafCountB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, degenerateSubtreeCountA, degenerateSubtreeCountB, ref context->Leaves, out var aIndex, out var bIndex);
                if (degenerateSubtreeCountA > 1)
                    BinnedBuilderInternal(subtreeRegionStartIndex, aIndex, degenerateSubtreeCountA, nodeIndex, 0, context, workerIndex);
                if (degenerateSubtreeCountB > 1)
                    BinnedBuilderInternal(subtreeRegionStartIndex + degenerateSubtreeCountA, bIndex, degenerateSubtreeCountB, nodeIndex, 1, context, workerIndex);
                return;
            }

            //Note that we don't bother even trying to internally multithread microsweeps. They *should* be small, and should only show up deeper in the recursion process.
            if (subtreeCount <= context->MicrosweepThreshold)
            {
                MicroSweepForBinnedBuilder(centroidBounds.Min, centroidBounds.Max, indices, leafCounts, ref context->Leaves, boundingBoxes, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, context, workerIndex);
                return;
            }

            var useX = centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z;
            var useY = centroidSpan.Y > centroidSpan.Z;
            //These will be used conditionally based on what hardware acceleration is available. Pretty minor detail.
            var permuteMask = Vector128.Create(useX ? 0 : useY ? 1 : 2, 0, 0, 0);
            var axisIndex = useX ? 0 : useY ? 1 : 2;

            var binCount = Math.Min(context->MaximumBinCount, Math.Max((int)(subtreeCount * context->LeafToBinMultiplier), context->MinimumBinCount));
            context->Threading.GetBins(workerIndex, out var binBoundingBoxes, out var binBoundingBoxesScan, out var binLeafCounts);
            Debug.Assert(binBoundingBoxes.Length >= binCount);

            var offsetToBinIndex = new Vector4(binCount) / centroidSpan;
            //Avoid letting NaNs into the offsetToBinIndex scale.
            offsetToBinIndex = Vector128.ConditionalSelect(axisIsDegenerate, Vector128<float>.Zero, offsetToBinIndex.AsVector128()).AsVector4();

            for (int i = 0; i < binCount; ++i)
            {
                ref var boxX = ref binBoundingBoxes[i];
                boxX.Min = new Vector4(float.MaxValue);
                boxX.Max = new Vector4(float.MinValue);
                binLeafCounts[i] = 0;
            }

            var maximumBinIndex = new Vector4(binCount - 1);
            if (typeof(TThreading) == typeof(SingleThreaded) || subtreeCount < SubtreesPerThreadForBinning)
            {
                BinSubtrees(centroidBounds.Min, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, boundingBoxes, leafCounts, binBoundingBoxes, binLeafCounts);
            }
            else
            {
                MultithreadedBinSubtrees(
                   (MultithreadBinnedBuildContext<TLeafCounts>*)Unsafe.AsPointer(ref Unsafe.As<TThreading, MultithreadBinnedBuildContext<TLeafCounts>>(ref context->Threading)),
                   centroidBounds.Min, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, boundingBoxes, leafCounts, binCount, workerIndex);
            }

            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            binBoundingBoxesScan[0] = binBoundingBoxes[0];
            int totalLeafCount = typeof(TLeafCounts) == typeof(LeafCountBuffer) ? binLeafCounts[0] : subtreeCount;
            for (int i = 1; i < binCount; ++i)
            {
                var previousIndex = i - 1;
                ref var xBounds = ref binBoundingBoxes[i];
                ref var xScanBounds = ref binBoundingBoxesScan[i];
                ref var xPreviousScanBounds = ref binBoundingBoxesScan[previousIndex];
                xScanBounds.Min = Vector4.Min(xBounds.Min, xPreviousScanBounds.Min);
                xScanBounds.Max = Vector4.Max(xBounds.Max, xPreviousScanBounds.Max);
                if (typeof(TLeafCounts) == typeof(LeafCountBuffer))
                    totalLeafCount += binLeafCounts[i];
            }
            var leftBoundsX = binBoundingBoxes[0];
            Debug.Assert(
                leftBoundsX.Min.X > float.MinValue && leftBoundsX.Min.Y > float.MinValue && leftBoundsX.Min.Z > float.MinValue,
                "Bin 0 should have been updated in all cases because it is aligned with the minimum bin, and the centroid span isn't degenerate.");

            float bestSAH = float.MaxValue;
            int bestSplit = 1;
            //The split index is going to end up in child B.
            var lastBinIndex = binCount - 1;
            BoundingBox4 accumulatedBoundingBoxB;
            accumulatedBoundingBoxB = binBoundingBoxes[lastBinIndex];
            BoundingBox4 bestBoundingBoxB;
            bestBoundingBoxB = binBoundingBoxes[lastBinIndex];
            int accumulatedLeafCountB = binLeafCounts[lastBinIndex];
            int bestLeafCountB = 0;
            for (int splitIndexCandidate = lastBinIndex; splitIndexCandidate >= 1; --splitIndexCandidate)
            {
                var previousIndex = splitIndexCandidate - 1;
                var sahCandidate = ComputeBoundsMetric(binBoundingBoxesScan[previousIndex]) * (totalLeafCount - accumulatedLeafCountB) + ComputeBoundsMetric(accumulatedBoundingBoxB) * accumulatedLeafCountB;

                if (sahCandidate < bestSAH)
                {
                    bestSAH = sahCandidate;
                    bestSplit = splitIndexCandidate;
                    bestBoundingBoxB = accumulatedBoundingBoxB;
                    if (typeof(TLeafCounts) == typeof(LeafCountBuffer))
                        bestLeafCountB = accumulatedLeafCountB;
                }
                ref var xBounds = ref binBoundingBoxes[previousIndex];
                accumulatedBoundingBoxB.Min = Vector4.Min(xBounds.Min, accumulatedBoundingBoxB.Min);
                accumulatedBoundingBoxB.Max = Vector4.Max(xBounds.Max, accumulatedBoundingBoxB.Max);
                accumulatedLeafCountB += binLeafCounts[previousIndex];
            }

            //Choose the best SAH from all axes and split the indices/bounds into two halves for the children to operate on.
            var subtreeCountB = 0;
            var subtreeCountA = 0;
            var splitIndex = bestSplit;
            var bestboundsA = binBoundingBoxesScan[bestSplit - 1];
            var bestboundsB = bestBoundingBoxB;
            //Now we have the split index between bins. Go back through and sort the indices and bounds into two halves.
            while (subtreeCountA + subtreeCountB < subtreeCount)
            {
                ref var box = ref boundingBoxes[subtreeCountA];
                var binIndex = ComputeBinIndex(centroidBounds.Min, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
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
            Debug.Assert(subtreeCountA + subtreeCountB == subtreeCount);
            BuildNode(bestboundsA, bestboundsB, leafCountA, leafCountB, nodes, metanodes, indices, nodeIndex, parentNodeIndex, childIndexInParent, subtreeCountA, subtreeCountB, ref context->Leaves, out var nodeChildIndexA, out var nodeChildIndexB);

            var shouldPushBOntoMultithreadedQueue = typeof(TThreading) != typeof(SingleThreaded) && subtreeCountA >= SubtreesPerThreadForNodeJob && subtreeCountB >= SubtreesPerThreadForNodeJob;
            ContinuationHandle nodeBContinuation = default;
            if (shouldPushBOntoMultithreadedQueue)
            {
                //Both of the children are large. Push child B onto the multithreaded execution stack so it can run at the same time as child A (potentially).
                Debug.Assert(SubtreesPerThreadForNodeJob > 1, "The job threshold for a new node should be large enough that there's no need for a subtreeCountB > 1 test.");
                ref var threading = ref Unsafe.As<TThreading, MultithreadBinnedBuildContext<TLeafCounts>>(ref context->Threading);
                //Allocate the parameters to send to the worker on the local stack. Note that we have to preserve the stack for this to work; see the later WaitForCompletion.
                NodePushTaskContext<TLeafCounts, TLeaves, TThreading> wrappedContext;
                wrappedContext.Context = context;
                wrappedContext.NodeIndex = nodeChildIndexB;
                wrappedContext.ParentNodeIndex = nodeIndex;
                //Note that we use the task id to store subtree start and subtree count. Don't have to do that, but no reason not to use it.
                var task = new Task(&BinnedBuilderNodeWorker<TLeafCounts, TLeaves, TThreading>, &wrappedContext, (long)(subtreeRegionStartIndex + subtreeCountA) | ((long)subtreeCountB << 32));
                nodeBContinuation = threading.Queue->AllocateContinuationAndEnqueueTasks(new Span<Task>(&task, 1), workerIndex);
            }
            if (subtreeCountA > 1)
                BinnedBuilderInternal(subtreeRegionStartIndex, nodeChildIndexA, subtreeCountA, nodeIndex, 0, context, workerIndex);
            if (!shouldPushBOntoMultithreadedQueue && subtreeCountB > 1)
                BinnedBuilderInternal(subtreeRegionStartIndex + subtreeCountA, nodeChildIndexB, subtreeCountB, nodeIndex, 1, context, workerIndex);
            if (shouldPushBOntoMultithreadedQueue)
            {
                //We want to keep the stack at this level alive until the memory we allocated for the node push completes.
                //Note that WaitForCompletion will execute pending work; this isn't just busywaiting the current thread.
                //In addition to letting us use the local stack to store some arguments for the other thread, this wait means that all children have completed when this function returns.
                //That makes knowing when to stop the queue easier.
                Debug.Assert(nodeBContinuation.Initialized);
                Unsafe.As<TThreading, MultithreadBinnedBuildContext<TLeafCounts>>(ref context->Threading).Queue->WaitForCompletion(nodeBContinuation, workerIndex);
            }
        }




        static unsafe void BinnedBuilderInternal(Buffer<int> encodedLeafIndices, Buffer<BoundingBox> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<Leaf> leaves,
            IThreadDispatcher dispatcher, TaskQueue* taskQueue, int workerCount, int minimumBinCount, int maximumBinCount, float leafToBinMultiplier, int microsweepThreshold)
        {
            var subtreeCount = encodedLeafIndices.Length;
            Debug.Assert(boundingBoxes.Length >= subtreeCount, "The bounding boxes provided must cover the range of indices provided.");
            Debug.Assert(nodes.Length >= subtreeCount - 1, "The output nodes must be able to contain the nodes created for the leaves.");
            if (subtreeCount == 0)
                return;
            if (subtreeCount == 1)
            {
                //If there's only one leaf, the tree has a special format: the root node has only one child.
                ref var root = ref nodes[0];
                root.A.Min = boundingBoxes[0].Min;
                root.A.Index = encodedLeafIndices[0]; //Node that we assume the indices are already encoded. This function works with subtree refinements as well which can manage either leaves or internals.
                root.A.Max = boundingBoxes[0].Max;
                root.A.LeafCount = 1;
                root.B = default;
                return;
            }
            boundingBoxes = boundingBoxes.Slice(encodedLeafIndices.Length);
            nodes = nodes.Slice(subtreeCount - 1);

            //Don't let the user pick values that will just cause an explosion.
            Debug.Assert(minimumBinCount >= 2 && maximumBinCount >= 2, "At least two bins are required. In release mode, this will be clamped up to 2, but where did lower values come from?");
            minimumBinCount = Math.Max(2, minimumBinCount);
            maximumBinCount = Math.Max(2, maximumBinCount);
            //The microsweep uses the same resources as the bin allocations, so expand to hold whichever is larger.
            var allocatedBinCount = Math.Max(maximumBinCount, microsweepThreshold);

            if (dispatcher == null && taskQueue == null)
            {
                //Use the single threaded path.
                var binBoundsMemory = stackalloc BoundingBox4[allocatedBinCount * 2 + 1];
                //Should be basically irrelevant, but just in case it's not on some platform, align the allocation.
                binBoundsMemory = (BoundingBox4*)(((ulong)binBoundsMemory + 31ul) & (~31ul));

                var binLeafCountsMemory = stackalloc int[allocatedBinCount];
                SingleThreaded threading;
                threading.BinBoundingBoxes = new Buffer<BoundingBox4>(binBoundsMemory, allocatedBinCount);
                threading.BinBoundingBoxesScan = new Buffer<BoundingBox4>(binBoundsMemory + allocatedBinCount, allocatedBinCount);
                threading.BinLeafCounts = new Buffer<int>(binLeafCountsMemory, allocatedBinCount);
                var context = new Context<UnitLeafCount, Buffer<Leaf>, SingleThreaded>(
                    minimumBinCount, maximumBinCount, leafToBinMultiplier, microsweepThreshold, encodedLeafIndices,
                    new UnitLeafCount(), leaves, boundingBoxes.As<BoundingBox4>(), nodes, metanodes, threading);
                BinnedBuilderInternal(0, 0, subtreeCount, -1, -1, &context, 0);
            }
        }

        public static unsafe void BinnedBuilder(Buffer<int> encodedLeafIndices, Buffer<BoundingBox> boundingBoxes, Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<Leaf> leaves,
            int minimumBinCount = 16, int maximumBinCount = 64, float leafToBinMultiplier = 1 / 16f, int microsweepThreshold = 64)
        {
            BinnedBuilderInternal(encodedLeafIndices, boundingBoxes, nodes, metanodes, leaves, null, null, 0, minimumBinCount, maximumBinCount, leafToBinMultiplier, microsweepThreshold);
        }
    }
}
