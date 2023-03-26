using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuUtilities.TestLinkedTaskStack;
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
using System.Threading;
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
            Buffer<NodeChild> subtrees, Buffer<Node> nodes, Buffer<Metanode> metanodes,
            int nodeIndex, int parentNodeIndex, int childIndexInParent, int subtreeCountA, int subtreeCountB, ref TLeaves leaves, out int aIndex, out int bIndex)
            where TLeaves : unmanaged
        {
            Debug.Assert(typeof(TLeaves) == typeof(LeavesHandledInPostPass) || typeof(TLeaves) == typeof(Buffer<Leaf>), "While we didn't bother with an interface here, we assume one of two types only.");
            ref var metanode = ref metanodes[0];
            metanode.Parent = parentNodeIndex;
            metanode.IndexInParent = childIndexInParent;
            metanode.RefineFlag = 0;
            ref var node = ref nodes[0];
            node.A = Unsafe.As<BoundingBox4, NodeChild>(ref a);
            node.B = Unsafe.As<BoundingBox4, NodeChild>(ref b);
            if (subtreeCountA == 1)
            {
                aIndex = subtrees[0].Index;
                Debug.Assert(leafCountA == 1);
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
                node.A.Index = aIndex;
            }
            if (subtreeCountB == 1)
            {
                bIndex = subtrees[^1].Index;
                Debug.Assert(leafCountB == 1);
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
                node.B.Index = bIndex;
            }
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

        interface IBinnedBuilderThreading
        {
            void GetBins(int workerIndex,
                out Buffer<BoundingBox4> binBoundingBoxes, out Buffer<BoundingBox4> binCentroidBoundingBoxes,
                out Buffer<BoundingBox4> binBoundingBoxesScan, out Buffer<BoundingBox4> binCentroidBoundingBoxesScan, out Buffer<int> binLeafCounts);
        }


        struct Context<TLeaves, TThreading>
            where TLeaves : unmanaged
            where TThreading : unmanaged, IBinnedBuilderThreading
        {
            public int MinimumBinCount;
            public int MaximumBinCount;
            public float LeafToBinMultiplier;
            public int MicrosweepThreshold;

            public TLeaves Leaves;
            public Buffer<NodeChild> SubtreesPing;
            public Buffer<NodeChild> SubtreesPong;
            public Buffer<Node> Nodes;
            public Buffer<Metanode> Metanodes;

            public Buffer<byte> BinIndices;

            public TThreading Threading;

            public Context(int minimumBinCount, int maximumBinCount, float leafToBinMultiplier, int microsweepThreshold,
                Buffer<NodeChild> subtreesPing, Buffer<NodeChild> subtreesPong, TLeaves leaves, Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<byte> binIndices, TThreading threading)
            {
                MinimumBinCount = minimumBinCount;
                MaximumBinCount = maximumBinCount;
                LeafToBinMultiplier = leafToBinMultiplier;
                MicrosweepThreshold = microsweepThreshold;
                SubtreesPing = subtreesPing;
                SubtreesPong = subtreesPong;
                BinIndices = binIndices;
                Leaves = leaves;
                Nodes = nodes;
                Metanodes = metanodes;
                Threading = threading;
            }
        }

        struct BoundsComparerX : IComparerRef<NodeChild> { public int Compare(ref NodeChild a, ref NodeChild b) => (a.Min.X + a.Max.X) > (b.Min.X + b.Max.X) ? -1 : 1; }
        struct BoundsComparerY : IComparerRef<NodeChild> { public int Compare(ref NodeChild a, ref NodeChild b) => (a.Min.Y + a.Max.Y) > (b.Min.Y + b.Max.Y) ? -1 : 1; }
        struct BoundsComparerZ : IComparerRef<NodeChild> { public int Compare(ref NodeChild a, ref NodeChild b) => (a.Min.Z + a.Max.Z) > (b.Min.Z + b.Max.Z) ? -1 : 1; }

        public struct NodeTimes
        {
            public double Total;
            public double CentroidPrepass;
            public double Binning;
            public double Partition;
            public bool MTPrepass;
            public bool MTBinning;
            public bool MTPartition;
            public int TargetTaskCount;
            public int SubtreeCount;
        }

        public static NodeTimes[] Times;

        static unsafe void MicroSweepForBinnedBuilder<TLeaves, TThreading>(
            Vector4 centroidMin, Vector4 centroidMax, ref TLeaves leaves,
            Buffer<NodeChild> subtrees, Buffer<Node> nodes, Buffer<Metanode> metanodes, int nodeIndex, int parentNodeIndex, int childIndexInParent, Context<TLeaves, TThreading>* context, int workerIndex)
            where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            //This is a very small scale sweep build.
            var subtreeCount = subtrees.Length;
            if (subtreeCount == 2)
            {
                ref var subtreeA = ref subtrees[0];
                ref var subtreeB = ref subtrees[1];
                BuildNode(Unsafe.As<NodeChild, BoundingBox4>(ref subtreeA), Unsafe.As<NodeChild, BoundingBox4>(ref subtreeB), subtreeA.LeafCount, subtreeB.LeafCount, subtrees,
                    nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, ref leaves, out _, out _);
                return;
            }
            var centroidSpan = centroidMax - centroidMin;
            context->Threading.GetBins(workerIndex, out var binBoundingBoxes, out var binCentroidBoundingBoxes, out var binBoundingBoxesScan, out var binCentroidBoundingBoxesScan, out var binLeafCounts);

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
                        ref var bounds = ref subtrees[i];
                        keys[i] = bounds.Min.X + bounds.Max.X;
                    }
                }
                else if (centroidSpan.Y > centroidSpan.Z)
                {
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        ref var bounds = ref subtrees[i];
                        keys[i] = bounds.Min.Y + bounds.Max.Y;
                    }
                }
                else
                {
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        ref var bounds = ref subtrees[i];
                        keys[i] = bounds.Min.Z + bounds.Max.Z;
                    }
                }
                for (int i = subtreeCount; i < paddedKeyCount; ++i)
                {
                    keys[i] = float.MaxValue;
                }
                VectorizedSorts.VectorCountingSort(keys, targetIndices, subtreeCount);

                //Now that we know the target indices, copy things into position.
                //Have to copy things into a temporary cache to avoid overwrites since we didn't do any shuffling during the sort.
                //Note that we can now reuse the keys memory.              
                var subtreeCache = binBoundingBoxesScan.As<NodeChild>();
                subtrees.CopyTo(0, subtreeCache, 0, subtreeCount);
                for (int i = 0; i < subtreeCount; ++i)
                {
                    var targetIndex = targetIndices[i];
                    subtrees[targetIndex] = subtreeCache[i];
                }
            }
            else
            {
                //No vectorization supported. Fall back to poopymode!
                if (centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z)
                {
                    var comparer = new BoundsComparerX();
                    QuickSort.Sort(ref subtrees[0], 0, subtreeCount - 1, ref comparer);
                }
                else if (centroidSpan.Y > centroidSpan.Z)
                {
                    var comparer = new BoundsComparerY();
                    QuickSort.Sort(ref subtrees[0], 0, subtreeCount - 1, ref comparer);
                }
                else
                {
                    var comparer = new BoundsComparerZ();
                    QuickSort.Sort(ref subtrees[0], 0, subtreeCount - 1, ref comparer);
                }
            }

            Debug.Assert(subtreeCount <= context->MaximumBinCount || subtreeCount <= context->MicrosweepThreshold, "We're reusing the bin resources under the assumption that this is only ever called when there are less leaves than maximum bins.");
            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            var boundingBoxes = subtrees.As<BoundingBox4>();
            binBoundingBoxesScan[0] = boundingBoxes[0];
            int totalLeafCount = subtrees[0].LeafCount;
            for (int i = 1; i < subtreeCount; ++i)
            {
                var previousIndex = i - 1;
                ref var previousScanBounds = ref binBoundingBoxesScan[previousIndex];
                ref var scanBounds = ref binBoundingBoxesScan[i];
                ref var bounds = ref boundingBoxes[i];
                scanBounds.Min = Vector4.Min(bounds.Min, previousScanBounds.Min);
                scanBounds.Max = Vector4.Max(bounds.Max, previousScanBounds.Max);
                totalLeafCount += subtrees[i].LeafCount;
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
                    bestLeafCountB = accumulatedLeafCountB;
                }
                ref var bounds = ref boundingBoxes[previousIndex];
                accumulatedBoundingBoxB.Min = Vector4.Min(bounds.Min, accumulatedBoundingBoxB.Min);
                accumulatedBoundingBoxB.Max = Vector4.Max(bounds.Max, accumulatedBoundingBoxB.Max);
                accumulatedLeafCountB += subtrees[previousIndex].LeafCount;
            }

            var bestBoundsA = binBoundingBoxesScan[bestSplit - 1];
            var subtreeCountA = bestSplit;
            var subtreeCountB = subtreeCount - bestSplit;
            var bestLeafCountA = totalLeafCount - bestLeafCountB;

            BuildNode(bestBoundsA, bestBoundsB, bestLeafCountA, bestLeafCountB, subtrees, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, subtreeCountA, subtreeCountB, ref leaves, out var aIndex, out var bIndex);
            if (subtreeCountA > 1)
            {
                var aBounds = boundingBoxes.Slice(subtreeCountA);
                var initialCentroid = aBounds.Memory->Min + aBounds.Memory->Max;
                BoundingBox4 centroidBoundsA;
                centroidBoundsA.Min = initialCentroid;
                centroidBoundsA.Max = initialCentroid;
                for (int i = 1; i < subtreeCountA; ++i)
                {
                    ref var bounds = ref aBounds[i];
                    var centroid = bounds.Min + bounds.Max;
                    centroidBoundsA.Min = Vector4.Min(centroidBoundsA.Min, centroid);
                    centroidBoundsA.Max = Vector4.Max(centroidBoundsA.Max, centroid);
                }
                MicroSweepForBinnedBuilder(centroidBoundsA.Min, centroidBoundsA.Max, ref leaves, subtrees.Slice(subtreeCountA), nodes.Slice(1, subtreeCountA - 1), metanodes.Slice(1, subtreeCountA - 1), aIndex, nodeIndex, 0, context, workerIndex);
            }
            if (subtreeCountB > 1)
            {
                var bBounds = boundingBoxes.Slice(subtreeCountA, subtreeCountB);
                var initialCentroid = bBounds.Memory->Min + bBounds.Memory->Max;
                BoundingBox4 centroidBoundsB;
                centroidBoundsB.Min = initialCentroid;
                centroidBoundsB.Max = initialCentroid;
                for (int i = 1; i < subtreeCountB; ++i)
                {
                    ref var bounds = ref bBounds[i];
                    var centroid = bounds.Min + bounds.Max;
                    centroidBoundsB.Min = Vector4.Min(centroidBoundsB.Min, centroid);
                    centroidBoundsB.Max = Vector4.Max(centroidBoundsB.Max, centroid);
                }
                MicroSweepForBinnedBuilder(centroidBoundsB.Min, centroidBoundsB.Max, ref leaves, subtrees.Slice(subtreeCountA, subtreeCountB), nodes.Slice(subtreeCountA, subtreeCountB - 1), metanodes.Slice(subtreeCountA, subtreeCountB - 1), bIndex, nodeIndex, 1, context, workerIndex);
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
            public Buffer<BoundingBox4> BinCentroidBoundingBoxes;
            public Buffer<BoundingBox4> BinBoundingBoxesScan;
            public Buffer<BoundingBox4> BinCentroidBoundingBoxesScan;
            public Buffer<int> BinLeafCounts;

            public SingleThreaded(Buffer<byte> binAllocationBuffer, int binCapacity)
            {
                int start = 0;
                BinBoundingBoxes = Suballocate<BoundingBox4>(binAllocationBuffer, ref start, binCapacity);
                BinCentroidBoundingBoxes = Suballocate<BoundingBox4>(binAllocationBuffer, ref start, binCapacity);
                BinBoundingBoxesScan = Suballocate<BoundingBox4>(binAllocationBuffer, ref start, binCapacity);
                BinCentroidBoundingBoxesScan = Suballocate<BoundingBox4>(binAllocationBuffer, ref start, binCapacity);
                BinLeafCounts = Suballocate<int>(binAllocationBuffer, ref start, binCapacity);
            }

            public void GetBins(int workerIndex,
                out Buffer<BoundingBox4> binBoundingBoxes, out Buffer<BoundingBox4> binCentroidBoundingBoxes,
                out Buffer<BoundingBox4> binBoundingBoxesScan, out Buffer<BoundingBox4> binCentroidBoundingBoxesScan, out Buffer<int> binLeafCounts)
            {
                binBoundingBoxes = BinBoundingBoxes;
                binCentroidBoundingBoxes = BinCentroidBoundingBoxes;
                binBoundingBoxesScan = BinBoundingBoxesScan;
                binCentroidBoundingBoxesScan = BinCentroidBoundingBoxesScan;
                binLeafCounts = BinLeafCounts;
            }
        }

        static Buffer<T> Suballocate<T>(Buffer<byte> buffer, ref int start, int count) where T : unmanaged
        {
            var size = count * Unsafe.SizeOf<T>();
            var previousStart = start;
            start += size;
            return buffer.Slice(previousStart, size).As<T>();
        }

        /// <summary>
        /// Stores resources required by a worker to dispatch and manage multithreaded work.
        /// </summary>
        /// <remarks>
        /// Some of the resources cached here are technically redundant with the storage used for workers and ends up involving an extra bin scan on a multithreaded test,
        /// but the cost associated with doing so is... low. The complexity cost of trying to use the memory allocated for workers is not low.
        /// </remarks>
        unsafe struct BinnedBuildWorkerContext
        {
            /// <summary>
            /// Bins associated with this worker for the duration of a node. This allocation will persist across the build.
            /// </summary>
            public Buffer<BoundingBox4> BinBoundingBoxes;
            /// <summary>
            /// Centroid bound bins associated with this worker for the duration of a node. This allocation will persist across the build.
            /// </summary>
            public Buffer<BoundingBox4> BinCentroidBoundingBoxes;
            /// <summary>
            /// Bins associated with this worker for use in the SAH scan. This allocation will persist across the build.
            /// </summary>
            public Buffer<BoundingBox4> BinBoundingBoxesScan;
            /// <summary>
            /// Centroid bound bins associated with this worker for use in the SAH scan. This allocation will persist across the build.
            /// </summary>
            public Buffer<BoundingBox4> BinCentroidBoundingBoxesScan;
            /// <summary>
            /// Bin leaf counts associated with this worker for the duration of a node. This allocation will persist across the build.
            /// </summary>
            public Buffer<int> BinLeafCounts;

            public BinnedBuildWorkerContext(Buffer<byte> binAllocationBuffer, ref int binStart, int binCapacity)
            {
                BinBoundingBoxes = Suballocate<BoundingBox4>(binAllocationBuffer, ref binStart, binCapacity);
                BinCentroidBoundingBoxes = Suballocate<BoundingBox4>(binAllocationBuffer, ref binStart, binCapacity);
                BinBoundingBoxesScan = Suballocate<BoundingBox4>(binAllocationBuffer, ref binStart, binCapacity);
                BinCentroidBoundingBoxesScan = Suballocate<BoundingBox4>(binAllocationBuffer, ref binStart, binCapacity);
                BinLeafCounts = Suballocate<int>(binAllocationBuffer, ref binStart, binCapacity);
            }
        }
        unsafe struct MultithreadBinnedBuildContext : IBinnedBuilderThreading
        {
            public LinkedTaskStack* TaskStack;
            /// <summary>
            /// The number of subtrees present at the root of the build.
            /// </summary>
            public int OriginalSubtreeCount;
            /// <summary>
            /// The target number of tasks that would be used for the root node. Later nodes will tend to target smaller numbers of tasks on the assumption that other parallel nodes will provide enough work to fill in the gaps.
            /// </summary>
            public int TopLevelTargetTaskCount;
            public Buffer<BinnedBuildWorkerContext> Workers;

            public void GetBins(int workerIndex,
                out Buffer<BoundingBox4> binBoundingBoxes, out Buffer<BoundingBox4> binCentroidBoundingBoxes,
                out Buffer<BoundingBox4> binBoundingBoxesScan, out Buffer<BoundingBox4> binCentroidBoundingBoxesScan, out Buffer<int> binLeafCounts)
            {
                ref var worker = ref Workers[workerIndex];
                binBoundingBoxes = worker.BinBoundingBoxes;
                binCentroidBoundingBoxes = worker.BinCentroidBoundingBoxes;
                binBoundingBoxesScan = worker.BinBoundingBoxesScan;
                binCentroidBoundingBoxesScan = worker.BinCentroidBoundingBoxesScan;
                binLeafCounts = worker.BinLeafCounts;
            }

            public int GetTargetTaskCount(int subtreeCount)
            {
                return (int)float.Ceiling(TopLevelTargetTaskCount * (float)subtreeCount / OriginalSubtreeCount);
            }
        }

        const int MinimumSubtreesPerThreadForCentroidPrepass = 65536;
        const int MinimumSubtreesPerThreadForBinning = 65536;
        const int MinimumSubtreesPerThreadForPartitioning = 65536;
        const int MinimumSubtreesPerThreadForNodeJob = 1024;

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

        struct SharedTaskData
        {
            public int WorkerCount;
            public int TaskCount;

            public int SubtreeStartIndex;
            public int SubtreeCount;

            public int SlotsPerTaskBase;
            public int SlotRemainder;
            public bool TaskCountFitsInWorkerCount;

            public SharedTaskData(int workerCount, int subtreeStartIndex, int slotCount,
                int minimumSlotsPerTask, int targetTaskCount)
            {
                WorkerCount = workerCount;
                var taskSize = int.Max(minimumSlotsPerTask, slotCount / targetTaskCount);
                TaskCount = (slotCount + taskSize - 1) / taskSize;
                SubtreeStartIndex = subtreeStartIndex;
                SubtreeCount = slotCount;
                SlotsPerTaskBase = slotCount / TaskCount;
                SlotRemainder = slotCount - TaskCount * SlotsPerTaskBase;
                TaskCountFitsInWorkerCount = TaskCount <= WorkerCount;
            }

            public void GetSlotInterval(long taskId, out int start, out int count)
            {
                var remainderedTaskCount = int.Min(SlotRemainder, (int)taskId);
                var earlySlotCount = (SlotsPerTaskBase + 1) * remainderedTaskCount;
                var lateSlotCount = SlotsPerTaskBase * (taskId - remainderedTaskCount);
                start = SubtreeStartIndex + (int)(earlySlotCount + lateSlotCount);
                count = taskId >= SlotRemainder ? SlotsPerTaskBase : SlotsPerTaskBase + 1;
            }
        }
        unsafe struct CentroidPrepassTaskContext
        {
            public SharedTaskData TaskData;
            /// <summary>
            /// Stores per-worker prepass bounds accumulated over multiple tasks. If there are less tasks than workers, then only the lower contiguous region of these bounds are used.
            /// This allocation is ephemeral; it is allocated from the current worker when needed. 
            /// Note that the allocation occurs on the loop dispatching thread: the workers that help with the loop do not have to allocate anything themselves.
            /// </summary>
            public Buffer<BoundingBox4> PrepassWorkers;
            /// <summary>
            /// Buffer containing all subtrees in the node.
            /// </summary>
            public Buffer<NodeChild> Subtrees;

            public CentroidPrepassTaskContext(BufferPool pool, SharedTaskData taskData, Buffer<NodeChild> subtrees)
            {
                TaskData = taskData;
                pool.Take(int.Min(taskData.WorkerCount, taskData.TaskCount), out PrepassWorkers);
                Debug.Assert(PrepassWorkers.Length >= 2);
                Subtrees = subtrees;
            }

            public void Dispose(BufferPool pool) => pool.Return(ref PrepassWorkers);
        }
        unsafe static void CentroidPrepassWorker(long taskId, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var context = ref *(CentroidPrepassTaskContext*)untypedContext;
            Debug.Assert(context.TaskData.WorkerCount > 1 && context.TaskData.TaskCount > 1 && context.TaskData.WorkerCount < 100);
            context.TaskData.GetSlotInterval(taskId, out var start, out var count);
            var centroidBounds = ComputeCentroidBounds(context.Subtrees.Slice(start, count).As<BoundingBox4>());
            if (context.TaskData.TaskCountFitsInWorkerCount)
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

        unsafe static BoundingBox4 MultithreadedCentroidPrepass(MultithreadBinnedBuildContext* context, Buffer<NodeChild> subtrees, int targetTaskCount, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var worker = ref context->Workers[workerIndex];
            var workerPool = dispatcher.WorkerPools[workerIndex];
            var taskContext = new CentroidPrepassTaskContext(workerPool, new SharedTaskData(context->Workers.Length, 0, subtrees.Length, MinimumSubtreesPerThreadForCentroidPrepass, targetTaskCount), subtrees);
            Debug.Assert(taskContext.TaskData.TaskCount > 1, "This codepath shouldn't be used if there's only one task!");
            Debug.Assert(taskContext.TaskData.WorkerCount > 1 && taskContext.TaskData.WorkerCount < 100);
            var taskCount = taskContext.TaskData.TaskCount;
            //Don't bother initializing more slots than we have tasks. Note that this requires special handling on the task level;
            //if we have less tasks than workers, then the task needs to distinguish that fact.
            var activeWorkerCount = int.Min(taskContext.TaskData.WorkerCount, taskCount);
            if (taskCount > taskContext.TaskData.WorkerCount)
            {
                //Potentially multiple tasks per worker; we must preinitialize slots.
                for (int i = 0; i < activeWorkerCount; ++i)
                {
                    ref var workerBounds = ref taskContext.PrepassWorkers[i];
                    workerBounds.Min = new Vector4(float.MaxValue);
                    workerBounds.Max = new Vector4(float.MinValue);
                }
            }
            Debug.Assert(taskContext.TaskData.TaskCount > 0 && taskContext.TaskData.WorkerCount > 0);
            context->TaskStack->For(&CentroidPrepassWorker, &taskContext, 0, taskCount, workerIndex, dispatcher);

            var centroidBounds = taskContext.PrepassWorkers[0];
            for (int i = 1; i < activeWorkerCount; ++i)
            {
                ref var workerBounds = ref taskContext.PrepassWorkers[i];
                centroidBounds.Min = Vector4.Min(workerBounds.Min, centroidBounds.Min);
                centroidBounds.Max = Vector4.Max(workerBounds.Max, centroidBounds.Max);
            }
            taskContext.Dispose(workerPool);
            return centroidBounds;
        }

        struct BinSubtreesWorkerContext
        {
            public Buffer<BoundingBox4> BinBoundingBoxes;
            public Buffer<BoundingBox4> BinCentroidBoundingBoxes;
            public Buffer<int> BinLeafCounts;
        }
        unsafe struct BinSubtreesTaskContext
        {
            public SharedTaskData TaskData;
            /// <summary>
            /// Bins associated with any workers that end up contributing to this worker's dispatch of a binning loop. If there are less tasks than workers, then only the lower contiguous region of these bounds are used.
            /// This allocation is ephemeral; it is allocated from the current worker when needed. 
            /// Note that the allocation occurs on the loop dispatching thread: the workers that help with the loop do not have to allocate anything themselves.
            /// </summary>
            public Buffer<BinSubtreesWorkerContext> BinSubtreesWorkers;
            /// <summary>
            /// Whether a given worker contributed to the subtree binning process. If this worker did not contribute, there's no reason to merge its bins.
            /// This allocation is ephemeral; it is allocated from the current worker when needed. 
            /// Note that the allocation occurs on the loop dispatching thread: the workers that help with the loop do not have to allocate anything themselves.
            /// </summary>
            public Buffer<bool> WorkerHelpedWithBinning;

            /// <summary>
            /// Buffer containing all subtrees in this node.
            /// </summary>
            public Buffer<NodeChild> Subtrees;

            /// <summary>
            /// Stores the bin indices of all subtrees in the node.
            /// </summary>
            public Buffer<byte> BinIndices;

            public int BinCount;
            public bool UseX, UseY;
            public Vector128<int> PermuteMask;
            public int AxisIndex;
            public Vector4 CentroidBoundsMin;
            public Vector4 OffsetToBinIndex;
            public Vector4 MaximumBinIndex;

            public BinSubtreesTaskContext(BufferPool pool, SharedTaskData taskData, Buffer<NodeChild> subtrees, Buffer<byte> binIndices,
                int binCount, bool useX, bool useY, Vector128<int> permuteMask, int axisIndex,
                Vector4 centroidBoundsMin, Vector4 offsetToBinIndex, Vector4 maximumBinIndex)
            {
                TaskData = taskData;
                Subtrees = subtrees;
                BinIndices = binIndices;
                BinCount = binCount;
                UseX = useX;
                UseY = useY;
                PermuteMask = permuteMask;
                AxisIndex = axisIndex;
                CentroidBoundsMin = centroidBoundsMin;
                OffsetToBinIndex = offsetToBinIndex;
                MaximumBinIndex = maximumBinIndex;
                var effectiveWorkerCount = int.Min(taskData.WorkerCount, taskData.TaskCount);
                //Pull one allocation from the pool instead of 1 + workerCount * 2. Slight reduction in overhead. Note that this means we only need to return one buffer of the associated id at the end!
                var allocationSize = (sizeof(BinSubtreesWorkerContext) + (sizeof(BoundingBox4) * 2 + sizeof(int)) * binCount + sizeof(bool) * taskData.WorkerCount) * effectiveWorkerCount;
                pool.Take<byte>(allocationSize, out var allocation);
                int start = 0;
                BinSubtreesWorkers = Suballocate<BinSubtreesWorkerContext>(allocation, ref start, effectiveWorkerCount);
                for (int i = 0; i < effectiveWorkerCount; ++i)
                {
                    ref var worker = ref BinSubtreesWorkers[i];
                    worker.BinBoundingBoxes = Suballocate<BoundingBox4>(allocation, ref start, BinCount);
                    worker.BinCentroidBoundingBoxes = Suballocate<BoundingBox4>(allocation, ref start, BinCount);
                    worker.BinLeafCounts = Suballocate<int>(allocation, ref start, BinCount);
                }
                WorkerHelpedWithBinning = Suballocate<bool>(allocation, ref start, effectiveWorkerCount);
                WorkerHelpedWithBinning.Clear(0, effectiveWorkerCount);
            }
            public void Dispose(BufferPool pool) => pool.Return(ref BinSubtreesWorkers); //Only need to return the main buffer because all the other allocations share the same id!
        }
        //these type-level booleans let the compiler avoid branching in the binning loop. The bin indices buffer is not guaranteed to exist.
        //i apologize
        /// <summary>
        /// Marks a <see cref="BinSubtrees"/> call as requiring the bin indices to be written to the binIndices buffer.
        /// </summary>
        private struct DoWriteBinIndices { }
        /// <summary>
        /// Marks a <see cref="BinSubtrees"/> call as not allowing the bin indices to be written to the binIndices buffer.
        /// </summary>
        private struct DoNotWriteBinIndices { }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void BinSubtrees<TShouldWriteBinIndices>(Vector4 centroidBoundsMin,
            bool useX, bool useY, Vector128<int> permuteMask, int axisIndex, Vector4 offsetToBinIndex, Vector4 maximumBinIndex,
            Buffer<NodeChild> subtrees, Buffer<BoundingBox4> binBoundingBoxes, Buffer<BoundingBox4> binCentroidBoundingBoxes, Buffer<int> binLeafCounts, Buffer<byte> binIndices)
            where TShouldWriteBinIndices : unmanaged
        {
            //Note that we don't store out any of the indices into per-bin lists here. We only *really* want two final groups for the children,
            //and we can easily compute those by performing another scan. It requires recomputing the bin indices, but that's really not much of a concern.
            for (int i = 0; i < subtrees.Length; ++i)
            {
                ref var subtree = ref subtrees[i];
                ref var box = ref Unsafe.As<NodeChild, BoundingBox4>(ref subtree);
                var binIndex = ComputeBinIndex(centroidBoundsMin, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
                if (typeof(TShouldWriteBinIndices) == typeof(DoWriteBinIndices))
                    binIndices[i] = (byte)binIndex;
                ref var binBounds = ref binBoundingBoxes[binIndex];
                binBounds.Min = Vector4.Min(binBounds.Min, box.Min);
                binBounds.Max = Vector4.Max(binBounds.Max, box.Max);
                //The binning phase also keeps track of *centroid* bounding boxes so that we don't have to do a dedicated centroid prepass for each node.
                //(A centroid prepass would require touching every single subtree again, and, for large trees, that's a lot of uncached (or distant) memory accesses.)
                var centroid = box.Min + box.Max;
                ref var binCentroidBounds = ref binCentroidBoundingBoxes[binIndex];
                binCentroidBounds.Min = Vector4.Min(binCentroidBounds.Min, centroid);
                binCentroidBounds.Max = Vector4.Max(binCentroidBounds.Max, centroid);
                binLeafCounts[binIndex] += subtree.LeafCount;
            }
        }
        unsafe static void BinSubtreesWorker(long taskId, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var context = ref *(BinSubtreesTaskContext*)untypedContext;
            Debug.Assert(context.TaskData.WorkerCount > 1 && context.TaskData.TaskCount > 1 && context.TaskData.WorkerCount < 100);
            //Note that if we have more workers than tasks, we use the task id to index into the caches (and initialize the data here rather then before dispatching).
            var effectiveWorkerIndex = context.TaskData.TaskCountFitsInWorkerCount ? (int)taskId : workerIndex;
            ref var worker = ref context.BinSubtreesWorkers[effectiveWorkerIndex];
            context.WorkerHelpedWithBinning[effectiveWorkerIndex] = true;
            if (context.TaskData.TaskCountFitsInWorkerCount)
            {
                for (int i = 0; i < context.BinCount; ++i)
                {
                    ref var binBounds = ref worker.BinBoundingBoxes[i];
                    binBounds.Min = new Vector4(float.MaxValue);
                    binBounds.Max = new Vector4(float.MinValue);
                    ref var binCentroidBounds = ref worker.BinCentroidBoundingBoxes[i];
                    binCentroidBounds.Min = new Vector4(float.MaxValue);
                    binCentroidBounds.Max = new Vector4(float.MinValue);
                    worker.BinLeafCounts[i] = 0;
                }
            }
            context.TaskData.GetSlotInterval(taskId, out var start, out var count);
            //We always write bin indices, because threading always has a bufferpool available to allocate bin indices from.
            Debug.Assert(context.BinIndices.Allocated);
            BinSubtrees<DoWriteBinIndices>(context.CentroidBoundsMin, context.UseX, context.UseY, context.PermuteMask, context.AxisIndex, context.OffsetToBinIndex, context.MaximumBinIndex,
                context.Subtrees.Slice(start, count), worker.BinBoundingBoxes, worker.BinCentroidBoundingBoxes, worker.BinLeafCounts, context.BinIndices.Slice(start, count));
        }

        unsafe static void MultithreadedBinSubtrees(MultithreadBinnedBuildContext* context,
            Vector4 centroidBoundsMin, bool useX, bool useY, Vector128<int> permuteMask, int axisIndex, Vector4 offsetToBinIndex, Vector4 maximumBinIndex,
            Buffer<NodeChild> subtrees, Buffer<byte> subtreeBinIndices, int binCount, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var worker = ref context->Workers[workerIndex];
            var workerPool = dispatcher.WorkerPools[workerIndex];
            var taskContext = new BinSubtreesTaskContext(
                workerPool,
                new SharedTaskData(context->Workers.Length, 0, subtrees.Length, MinimumSubtreesPerThreadForBinning, context->GetTargetTaskCount(subtrees.Length)),
                subtrees, subtreeBinIndices, binCount, useX, useY, permuteMask, axisIndex, centroidBoundsMin, offsetToBinIndex, maximumBinIndex);

            //Don't bother initializing more slots than we have tasks. Note that this requires special handling on the task level;
            //if we have less tasks than workers, then the task needs to distinguish that fact.
            var activeWorkerCount = int.Min(context->Workers.Length, taskContext.TaskData.TaskCount);
            if (!taskContext.TaskData.TaskCountFitsInWorkerCount)
            {
                //If there are more tasks than workers, then we need to preinitialize all the worker caches.
                for (int cacheIndex = 0; cacheIndex < activeWorkerCount; ++cacheIndex)
                {
                    ref var cache = ref taskContext.BinSubtreesWorkers[cacheIndex];
                    for (int i = 0; i < binCount; ++i)
                    {
                        ref var binBounds = ref cache.BinBoundingBoxes[i];
                        binBounds.Min = new Vector4(float.MaxValue);
                        binBounds.Max = new Vector4(float.MinValue);
                        ref var binCentroidBounds = ref cache.BinCentroidBoundingBoxes[i];
                        binCentroidBounds.Min = new Vector4(float.MaxValue);
                        binCentroidBounds.Max = new Vector4(float.MinValue);
                        cache.BinLeafCounts[i] = 0;
                    }
                }
            }

            context->TaskStack->For(&BinSubtreesWorker, &taskContext, 0, taskContext.TaskData.TaskCount, workerIndex, dispatcher);

            //Unless the number of threads and bins is really huge, there's no value in attempting to multithread the final compression.
            //(Parallel reduction is an option, but even then... I suspect the single threaded version will be faster. And it's way simpler.)
            //Note that we have a separate merging target from the caches; that just makes resource management easier.
            //We can dispose the worker stuff immediately after this merge.
            //(Consider what happens in the case where the single threaded path is used: you need an allocation! would you allocate a bunch of multithreaded workers for it?
            //That's not an irrelevant case, either. *Most* nodes will be too small to warrant internal multithreading.)
            ref var cache0 = ref taskContext.BinSubtreesWorkers[0];
            cache0.BinBoundingBoxes.CopyTo(0, worker.BinBoundingBoxes, 0, cache0.BinBoundingBoxes.Length);
            cache0.BinCentroidBoundingBoxes.CopyTo(0, worker.BinCentroidBoundingBoxes, 0, cache0.BinCentroidBoundingBoxes.Length);
            cache0.BinLeafCounts.CopyTo(0, worker.BinLeafCounts, 0, cache0.BinLeafCounts.Length);
            for (int cacheIndex = 1; cacheIndex < activeWorkerCount; ++cacheIndex)
            {
                //Only bother merging from workers that actually did anything.
                if (taskContext.WorkerHelpedWithBinning[cacheIndex])
                {
                    ref var cache = ref taskContext.BinSubtreesWorkers[cacheIndex];
                    for (int binIndex = 0; binIndex < binCount; ++binIndex)
                    {
                        ref var b0 = ref worker.BinBoundingBoxes[binIndex];
                        ref var bi = ref cache.BinBoundingBoxes[binIndex];
                        b0.Min = Vector4.Min(b0.Min, bi.Min);
                        b0.Max = Vector4.Max(b0.Max, bi.Max);
                        ref var bc0 = ref worker.BinCentroidBoundingBoxes[binIndex];
                        ref var bci = ref cache.BinCentroidBoundingBoxes[binIndex];
                        bc0.Min = Vector4.Min(bc0.Min, bci.Min);
                        bc0.Max = Vector4.Max(bc0.Max, bci.Max);
                        worker.BinLeafCounts[binIndex] += cache.BinLeafCounts[binIndex];
                    }
                }
            }
            taskContext.Dispose(workerPool);
        }

        [StructLayout(LayoutKind.Explicit, Size = 264)]
        struct PartitionCounters
        {
            //Padding to avoid shared cache lines.
            [FieldOffset(128)]
            public int SubtreeCountA;
            [FieldOffset(134)]
            public int SubtreeCountB;
        }
        unsafe struct PartitionTaskContext
        {
            public SharedTaskData TaskData;

            /// <summary>
            /// Buffer containing all subtrees in this node.
            /// </summary>
            public Buffer<NodeChild> Subtrees;
            /// <summary>
            /// Buffer that will contain the partitioned subtrees pulled from <see cref="Subtrees"/>.
            /// </summary>
            public Buffer<NodeChild> SubtreesNext;
            /// <summary>
            /// Buffer containing bin indices for all subtrees in the node (encoded with one byte per subtree).
            /// </summary>
            public Buffer<byte> BinIndices;
            public int BinSplitIndex;

            public PartitionCounters Counters;

            public PartitionTaskContext(SharedTaskData taskData, Buffer<NodeChild> subtrees, Buffer<NodeChild> subtreesNext, Buffer<byte> binIndices, int binSplitIndex)
            {
                TaskData = taskData;
                Subtrees = subtrees;
                SubtreesNext = subtreesNext;
                BinIndices = binIndices;
                BinSplitIndex = binSplitIndex;

                Counters = new PartitionCounters();
            }
        }

        unsafe static void PartitionSubtreesWorker(long taskId, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var context = ref *(PartitionTaskContext*)untypedContext;
            Buffer<byte> binIndices = context.BinIndices;
            context.TaskData.GetSlotInterval(taskId, out var start, out var count);
            //We don't really want to trigger interlocked operation for *every single subtree*, but we also don't want to allocate a bunch of memory.
            //Compromise! Stackalloc enough memory to cover sub-batches of the worker's subtrees, and do interlocked operations at the end of each batch.
            //Note that the main limit to the batch size is the amount of memory in cache.
            const int batchSize = 16384;
            byte* slotBelongsToA = stackalloc byte[batchSize];

            var batchCount = (count + batchSize - 1) / batchSize;
            var boundingBoxes = context.Subtrees.As<BoundingBox4>();
            var subtrees = context.Subtrees;
            var subtreesNext = context.SubtreesNext;
            var splitIndexBundle = new Vector<byte>((byte)context.BinSplitIndex);
            for (int batchIndex = 0; batchIndex < batchCount; ++batchIndex)
            {
                var localCountA = 0;
                var batchStart = start + batchIndex * batchSize;
                var countInBatch = int.Min(start + count - batchStart, batchSize);

                int scalarLoopStartIndex;
                if (Vector<byte>.IsSupported)
                {
                    //Note that the original data is loaded as bytes, but we need wider storage to handle the counts- which could conceivably go up to batchSize.
                    Vector<ushort> localCountABundle = Vector<ushort>.Zero;
                    scalarLoopStartIndex = (countInBatch / Vector<byte>.Count) * Vector<byte>.Count;
                    for (int indexInBatch = 0; indexInBatch < scalarLoopStartIndex; indexInBatch += Vector<byte>.Count)
                    {
                        var subtreeIndex = indexInBatch + batchStart;
                        var binIndicesBundle = *(Vector<byte>*)(binIndices.Memory + subtreeIndex);
                        var belongsToABundle = Vector.LessThan(binIndicesBundle, splitIndexBundle);
                        *(Vector<byte>*)(slotBelongsToA + indexInBatch) = belongsToABundle;
                        var increment = Vector.BitwiseAnd(belongsToABundle, Vector<byte>.One);
                        Vector.Widen(increment, out var low, out var high);
                        localCountABundle += low + high;
                    }
                    localCountA = Vector.Sum(localCountABundle);
                }
                else
                    scalarLoopStartIndex = 0;
                for (int indexInBatch = scalarLoopStartIndex; indexInBatch < countInBatch; ++indexInBatch)
                {
                    var subtreeIndex = indexInBatch + batchStart;
                    var binIndex = binIndices[subtreeIndex];
                    var belongsToA = binIndex < context.BinSplitIndex;
                    slotBelongsToA[indexInBatch] = belongsToA ? (byte)0xFF : (byte)0;
                    if (belongsToA) ++localCountA;
                }

                var localCountB = countInBatch - localCountA;
                var startIndexA = Interlocked.Add(ref context.Counters.SubtreeCountA, localCountA) - localCountA;
                var startIndexB = subtrees.Length - Interlocked.Add(ref context.Counters.SubtreeCountB, localCountB);

                int recountA = 0;
                int recountB = 0;
                for (int indexInBatch = 0; indexInBatch < countInBatch; ++indexInBatch)
                {
                    var targetIndex = slotBelongsToA[indexInBatch] != 0 ? startIndexA + recountA++ : startIndexB + recountB++;
                    subtreesNext[targetIndex] = subtrees[batchStart + indexInBatch];
                }

            }
        }

        unsafe static (int subtreeCountA, int subtreeCountB) MultithreadedPartition(MultithreadBinnedBuildContext* context,
            Buffer<NodeChild> subtrees, Buffer<NodeChild> subtreesNext, Buffer<byte> binIndices, int binSplitIndex, int targetTaskCount, int workerIndex, IThreadDispatcher dispatcher)
        {
            ref var worker = ref context->Workers[workerIndex];
            var workerPool = dispatcher.WorkerPools[workerIndex];
            var taskContext = new PartitionTaskContext(
                new SharedTaskData(context->Workers.Length, 0, subtrees.Length, MinimumSubtreesPerThreadForPartitioning, targetTaskCount),
                subtrees, subtreesNext, binIndices, binSplitIndex);

            context->TaskStack->For(&PartitionSubtreesWorker, &taskContext, 0, taskContext.TaskData.TaskCount, workerIndex, dispatcher);
            return (taskContext.Counters.SubtreeCountA, taskContext.Counters.SubtreeCountB);
        }

        unsafe struct NodePushTaskContext<TLeaves, TThreading>
            where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            public Context<TLeaves, TThreading>* Context;
            public int NodeIndex;
            public int ParentNodeIndex;
            public BoundingBox4 CentroidBounds;
            //Subtree region start index, subtree count, and usePongBuffer status are all encoded into the task id.
        }
        unsafe static void BinnedBuilderNodeWorker<TLeaves, TThreading>(long taskId, void* context, int workerIndex, IThreadDispatcher dispatcher)
            where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            var subtreeRegionStartIndex = (int)taskId;
            var subtreeCount = (int)((taskId >> 32) & 0x7FFF_FFFF);
            var usePongBuffer = (ulong)taskId >= (1UL << 63);
            var nodePushContext = (NodePushTaskContext<TLeaves, TThreading>*)context;
            //Note that child index is always 1 because we only ever push child B.
            BinnedBuildNode(usePongBuffer, subtreeRegionStartIndex, nodePushContext->NodeIndex, subtreeCount, nodePushContext->ParentNodeIndex, 1, nodePushContext->CentroidBounds, nodePushContext->Context, workerIndex, dispatcher);
        }

        static unsafe void BinnedBuildNode<TLeaves, TThreading>(
            bool usePongBuffer, int subtreeRegionStartIndex, int nodeIndex, int subtreeCount, int parentNodeIndex, int childIndexInParent,
            BoundingBox4 centroidBounds, Context<TLeaves, TThreading>* context, int workerIndex, IThreadDispatcher dispatcher)
            where TLeaves : unmanaged where TThreading : unmanaged, IBinnedBuilderThreading
        {
            var subtrees = (usePongBuffer ? context->SubtreesPong : context->SubtreesPing).Slice(subtreeRegionStartIndex, subtreeCount);
            var subtreeBinIndices = context->BinIndices.Allocated ? context->BinIndices.Slice(subtreeRegionStartIndex, subtreeCount) : default;
            //leaf counts, indices, and bounds are packed together, but it's useful to have a bounds-only representation so that the merging processes don't have to worry about dealing with the fourth lanes.
            var boundingBoxes = subtrees.As<BoundingBox4>();
            var nodeCount = subtreeCount - 1;
            var nodes = context->Nodes.Slice(nodeIndex, nodeCount);
            var metanodes = context->Metanodes.Slice(nodeIndex, nodeCount);
            if (subtreeCount == 2)
            {
                BuildNode(boundingBoxes[0], boundingBoxes[1], subtrees[0].LeafCount, subtrees[1].LeafCount, subtrees, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, 1, 1, ref context->Leaves, out _, out _);
                return;
            }
            var targetTaskCount = typeof(TThreading) == typeof(SingleThreaded) ? 1 :
                ((MultithreadBinnedBuildContext*)Unsafe.AsPointer(ref Unsafe.As<TThreading, MultithreadBinnedBuildContext>(ref context->Threading)))->GetTargetTaskCount(subtreeCount);
            if (nodeIndex == 0)
            {
                //The first node doesn't have a parent, and so isn't given centroid bounds. We have to compute them.
                if (targetTaskCount == 1 || subtreeCount < MinimumSubtreesPerThreadForCentroidPrepass)
                {
                    centroidBounds = ComputeCentroidBounds(boundingBoxes);
                }
                else
                {
                    centroidBounds = MultithreadedCentroidPrepass(
                        (MultithreadBinnedBuildContext*)Unsafe.AsPointer(ref Unsafe.As<TThreading, MultithreadBinnedBuildContext>(ref context->Threading)), subtrees, targetTaskCount, workerIndex, dispatcher);
                }
            }
            var centroidSpan = centroidBounds.Max - centroidBounds.Min;
            var axisIsDegenerate = Vector128.LessThanOrEqual(centroidSpan.AsVector128(), Vector128.Create(1e-12f));
            if ((Vector128.ExtractMostSignificantBits(axisIsDegenerate) & 0b111) == 0b111)
            {
                //This node is completely degenerate; there is no 'good' ordering of the children. Pick a split in the middle and shrug.
                //This shouldn't happen unless something is badly wrong with the input; no point in optimizing it.
                var degenerateSubtreeCountA = subtrees.Length / 2;
                var degenerateSubtreeCountB = subtrees.Length - degenerateSubtreeCountA;
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
                    degenerateLeafCountA += subtrees[i].LeafCount;
                }
                for (int i = degenerateSubtreeCountA; i < subtrees.Length; ++i)
                {
                    ref var bounds = ref boundingBoxes[i];
                    boundsB.Min = Vector4.Min(bounds.Min, boundsB.Min);
                    boundsB.Max = Vector4.Max(bounds.Max, boundsB.Max);
                    degenerateLeafCountB += subtrees[i].LeafCount;
                }
                BuildNode(boundsA, boundsB, degenerateLeafCountA, degenerateLeafCountB, subtrees, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, degenerateSubtreeCountA, degenerateSubtreeCountB, ref context->Leaves, out var aIndex, out var bIndex);
                if (degenerateSubtreeCountA > 1)
                    BinnedBuildNode(usePongBuffer, subtreeRegionStartIndex, aIndex, degenerateSubtreeCountA, nodeIndex, 0, boundsA, context, workerIndex, dispatcher);
                if (degenerateSubtreeCountB > 1)
                    BinnedBuildNode(usePongBuffer, subtreeRegionStartIndex + degenerateSubtreeCountA, bIndex, degenerateSubtreeCountB, nodeIndex, 1, boundsB, context, workerIndex, dispatcher);
                return;
            }

            //Note that we don't bother even trying to internally multithread microsweeps. They *should* be small, and should only show up deeper in the recursion process.
            if (subtreeCount <= context->MicrosweepThreshold)
            {
                MicroSweepForBinnedBuilder(centroidBounds.Min, centroidBounds.Max, ref context->Leaves, subtrees, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, context, workerIndex);
                return;
            }

            var useX = centroidSpan.X > centroidSpan.Y && centroidSpan.X > centroidSpan.Z;
            var useY = centroidSpan.Y > centroidSpan.Z;
            //These will be used conditionally based on what hardware acceleration is available. Pretty minor detail.
            var permuteMask = Vector128.Create(useX ? 0 : useY ? 1 : 2, 0, 0, 0);
            var axisIndex = useX ? 0 : useY ? 1 : 2;

            var binCount = int.Min(context->MaximumBinCount, int.Max((int)(subtreeCount * context->LeafToBinMultiplier), context->MinimumBinCount));

            var offsetToBinIndex = new Vector4(binCount) / centroidSpan;
            //Avoid letting NaNs into the offsetToBinIndex scale.
            offsetToBinIndex = Vector128.ConditionalSelect(axisIsDegenerate, Vector128<float>.Zero, offsetToBinIndex.AsVector128()).AsVector4();

            var maximumBinIndex = new Vector4(binCount - 1);
            context->Threading.GetBins(workerIndex, out var binBoundingBoxes, out var binCentroidBoundingBoxes, out var binBoundingBoxesScan, out var binCentroidBoundingBoxesScan, out var binLeafCounts);
            Debug.Assert(binBoundingBoxes.Length >= binCount);
            for (int i = 0; i < binCount; ++i)
            {
                ref var binBounds = ref binBoundingBoxes[i];
                binBounds.Min = new Vector4(float.MaxValue);
                binBounds.Max = new Vector4(float.MinValue);
                ref var binCentroidBounds = ref binCentroidBoundingBoxes[i];
                binCentroidBounds.Min = new Vector4(float.MaxValue);
                binCentroidBounds.Max = new Vector4(float.MinValue);
                binLeafCounts[i] = 0;
            }
            if (targetTaskCount == 1 || subtreeCount < MinimumSubtreesPerThreadForBinning)
            {
                //If the subtree bin indices buffer isn't available, then the binning process can't write to them! That'll happen if:
                //single threaded execution,
                //no bufferpool provided,
                //tree size too large for stack allocation.
                if (subtreeBinIndices.Allocated)
                    BinSubtrees<DoWriteBinIndices>(centroidBounds.Min, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, subtrees, binBoundingBoxes, binCentroidBoundingBoxes, binLeafCounts, subtreeBinIndices);
                else
                    BinSubtrees<DoNotWriteBinIndices>(centroidBounds.Min, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, subtrees, binBoundingBoxes, binCentroidBoundingBoxes, binLeafCounts, subtreeBinIndices);
            }
            else
            {
                MultithreadedBinSubtrees(
                   (MultithreadBinnedBuildContext*)Unsafe.AsPointer(ref Unsafe.As<TThreading, MultithreadBinnedBuildContext>(ref context->Threading)),
                   centroidBounds.Min, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, subtrees, subtreeBinIndices, binCount, workerIndex, dispatcher);
            }

            //Identify the split index by examining the SAH of very split option.
            //Premerge from left to right so we have a sorta-summed area table to cheaply look up all possible child A bounds as we scan.
            binBoundingBoxesScan[0] = binBoundingBoxes[0];
            binCentroidBoundingBoxesScan[0] = binCentroidBoundingBoxes[0];
            int totalLeafCount = binLeafCounts[0];
            for (int i = 1; i < binCount; ++i)
            {
                var previousIndex = i - 1;
                ref var bounds = ref binBoundingBoxes[i];
                ref var scanBounds = ref binBoundingBoxesScan[i];
                ref var previousScanBounds = ref binBoundingBoxesScan[previousIndex];
                scanBounds.Min = Vector4.Min(bounds.Min, previousScanBounds.Min);
                scanBounds.Max = Vector4.Max(bounds.Max, previousScanBounds.Max);
                ref var binCentroidBoundingBox = ref binCentroidBoundingBoxes[i];
                ref var binCentroidBoundingBoxScan = ref binCentroidBoundingBoxesScan[i];
                ref var previousCentroidBoundingBoxScan = ref binCentroidBoundingBoxesScan[previousIndex];
                binCentroidBoundingBoxScan.Min = Vector4.Min(binCentroidBoundingBox.Min, previousCentroidBoundingBoxScan.Min);
                binCentroidBoundingBoxScan.Max = Vector4.Max(binCentroidBoundingBox.Max, previousCentroidBoundingBoxScan.Max);
                totalLeafCount += binLeafCounts[i];
            }
            var leftBoundsX = binBoundingBoxes[0];
            Debug.Assert(
                leftBoundsX.Min.X > float.MinValue && leftBoundsX.Min.Y > float.MinValue && leftBoundsX.Min.Z > float.MinValue,
                "Bin 0 should have been updated in all cases because it is aligned with the minimum bin, and the centroid span isn't degenerate.");

            float bestSAH = float.MaxValue;
            int splitIndex = 1;
            //The split index is going to end up in child B.
            var lastBinIndex = binCount - 1;
            var accumulatedBoundingBoxB = binBoundingBoxes[lastBinIndex];
            var accumulatedCentroidBoundingBoxB = binCentroidBoundingBoxes[lastBinIndex];
            BoundingBox4 bestBoundingBoxB, bestCentroidBoundingBoxB;
            bestBoundingBoxB = accumulatedBoundingBoxB;
            bestCentroidBoundingBoxB = accumulatedCentroidBoundingBoxB;
            int accumulatedLeafCountB = binLeafCounts[lastBinIndex];
            int bestLeafCountB = 0;
            for (int splitIndexCandidate = lastBinIndex; splitIndexCandidate >= 1; --splitIndexCandidate)
            {
                var previousIndex = splitIndexCandidate - 1;
                var sahCandidate = ComputeBoundsMetric(binBoundingBoxesScan[previousIndex]) * (totalLeafCount - accumulatedLeafCountB) + ComputeBoundsMetric(accumulatedBoundingBoxB) * accumulatedLeafCountB;

                if (sahCandidate < bestSAH)
                {
                    bestSAH = sahCandidate;
                    splitIndex = splitIndexCandidate;
                    bestBoundingBoxB = accumulatedBoundingBoxB;
                    bestLeafCountB = accumulatedLeafCountB;
                    bestCentroidBoundingBoxB = accumulatedCentroidBoundingBoxB;
                }
                ref var bounds = ref binBoundingBoxes[previousIndex];
                accumulatedBoundingBoxB.Min = Vector4.Min(bounds.Min, accumulatedBoundingBoxB.Min);
                accumulatedBoundingBoxB.Max = Vector4.Max(bounds.Max, accumulatedBoundingBoxB.Max);
                ref var centroidBoundsForBin = ref binCentroidBoundingBoxes[previousIndex];
                accumulatedCentroidBoundingBoxB.Min = Vector4.Min(centroidBoundsForBin.Min, accumulatedCentroidBoundingBoxB.Min);
                accumulatedCentroidBoundingBoxB.Max = Vector4.Max(centroidBoundsForBin.Max, accumulatedCentroidBoundingBoxB.Max);
                accumulatedLeafCountB += binLeafCounts[previousIndex];
            }

            //var debugHuhStartTime = Stopwatch.GetTimestamp();
            var subtreeCountB = 0;
            var subtreeCountA = 0;
            var bestBoundingBoxA = binBoundingBoxesScan[splitIndex - 1];
            var bestCentroidBoundingBoxA = binCentroidBoundingBoxesScan[splitIndex - 1];

            //Split the indices/bounds into two halves for the children to operate on.
            if (context->SubtreesPong.Allocated)
            {
                Debug.Assert(subtreeBinIndices.Allocated);
                //If the current buffer is pong, then write to ping, and vice versa.
                var subtreesNext = (usePongBuffer ? context->SubtreesPing : context->SubtreesPong).Slice(subtreeRegionStartIndex, subtreeCount);
                if (targetTaskCount == 1 || subtreeCount < MinimumSubtreesPerThreadForPartitioning)
                {
                    for (int i = 0; i < subtreeCount; ++i)
                    {
                        var targetIndex = subtreeBinIndices[i] >= splitIndex ? subtreeCount - ++subtreeCountB : subtreeCountA++;
                        subtreesNext[targetIndex] = subtrees[i];
                    }
                }
                else
                {
                    (subtreeCountA, subtreeCountB) = MultithreadedPartition(
                       (MultithreadBinnedBuildContext*)Unsafe.AsPointer(ref Unsafe.As<TThreading, MultithreadBinnedBuildContext>(ref context->Threading)),
                       subtrees, subtreesNext, subtreeBinIndices, splitIndex, targetTaskCount, workerIndex, dispatcher);
                }
                subtrees = subtreesNext;
                usePongBuffer = !usePongBuffer;
            }
            else
            {
                //There is no pong buffer allocated. We allow this for lower memory allocation, but the implementation is strictly sequential and slower.
                while (subtreeCountA + subtreeCountB < subtreeCount)
                {
                    ref var box = ref boundingBoxes[subtreeCountA];
                    var binIndex = ComputeBinIndex(centroidBounds.Min, useX, useY, permuteMask, axisIndex, offsetToBinIndex, maximumBinIndex, box);
                    if (binIndex >= splitIndex)
                    {
                        //Belongs to B. Swap it.
                        var targetIndex = subtreeCount - subtreeCountB - 1;
                        if (Vector256.IsHardwareAccelerated)
                        {
                            var targetMemory = (byte*)(subtrees.Memory + targetIndex);
                            var aCountMemory = (byte*)(subtrees.Memory + subtreeCountA);
                            var targetVector = Vector256.Load(targetMemory);
                            var aCountVector = Vector256.Load(aCountMemory);
                            Vector256.Store(aCountVector, targetMemory);
                            Vector256.Store(targetVector, aCountMemory);
                        }
                        else
                        {
                            Helpers.Swap(ref subtrees[targetIndex], ref subtrees[subtreeCountA]);
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
            }

            var leafCountB = bestLeafCountB;
            var leafCountA = totalLeafCount - leafCountB;
            Debug.Assert(subtreeCountA + subtreeCountB == subtreeCount);
            BuildNode(bestBoundingBoxA, bestBoundingBoxB, leafCountA, leafCountB, subtrees, nodes, metanodes, nodeIndex, parentNodeIndex, childIndexInParent, subtreeCountA, subtreeCountB, ref context->Leaves, out var nodeChildIndexA, out var nodeChildIndexB);

            var shouldPushBOntoMultithreadedQueue = typeof(TThreading) != typeof(SingleThreaded) && subtreeCountA >= MinimumSubtreesPerThreadForNodeJob && subtreeCountB >= MinimumSubtreesPerThreadForNodeJob;
            ContinuationHandle nodeBContinuation = default;
            if (shouldPushBOntoMultithreadedQueue)
            {
                //Both of the children are large. Push child B onto the multithreaded execution stack so it can run at the same time as child A (potentially).
                Debug.Assert(MinimumSubtreesPerThreadForNodeJob > 1, "The job threshold for a new node should be large enough that there's no need for a subtreeCountB > 1 test.");
                ref var threading = ref Unsafe.As<TThreading, MultithreadBinnedBuildContext>(ref context->Threading);
                //Allocate the parameters to send to the worker on the local stack. Note that we have to preserve the stack for this to work; see the later WaitForCompletion.
                NodePushTaskContext<TLeaves, TThreading> nodePushContext;
                nodePushContext.Context = context;
                nodePushContext.NodeIndex = nodeChildIndexB;
                nodePushContext.ParentNodeIndex = nodeIndex;
                nodePushContext.CentroidBounds = bestCentroidBoundingBoxB;
                //Note that we use the task id to store subtree start, subtree count, and the pong buffer flag. Don't have to do that, but no reason not to use it.
                Debug.Assert((uint)subtreeCountB < (1u << 31), "The task id encodes start, count, and a pong flag, so we don't have room for a full 32 bits of count.");
                var task = new Task(&BinnedBuilderNodeWorker<TLeaves, TThreading>, &nodePushContext, (long)(subtreeRegionStartIndex + subtreeCountA) | ((long)subtreeCountB << 32) | (usePongBuffer ? 1L << 63 : 0));
                nodeBContinuation = threading.TaskStack->AllocateContinuationAndPush(new Span<Task>(&task, 1), workerIndex, dispatcher);
            }
            if (subtreeCountA > 1)
                BinnedBuildNode(usePongBuffer, subtreeRegionStartIndex, nodeChildIndexA, subtreeCountA, nodeIndex, 0, bestCentroidBoundingBoxA, context, workerIndex, dispatcher);
            if (!shouldPushBOntoMultithreadedQueue && subtreeCountB > 1)
                BinnedBuildNode(usePongBuffer, subtreeRegionStartIndex + subtreeCountA, nodeChildIndexB, subtreeCountB, nodeIndex, 1, bestCentroidBoundingBoxB, context, workerIndex, dispatcher);
            if (shouldPushBOntoMultithreadedQueue)
            {
                //We want to keep the stack at this level alive until the memory we allocated for the node push completes.
                //Note that WaitForCompletion will execute pending work; this isn't just busywaiting the current thread.
                //In addition to letting us use the local stack to store some arguments for the other thread, this wait means that all children have completed when this function returns.
                //That makes knowing when to stop the queue easier.
                Debug.Assert(nodeBContinuation.Initialized);
                Unsafe.As<TThreading, MultithreadBinnedBuildContext>(ref context->Threading).TaskStack->WaitForCompletion(nodeBContinuation, workerIndex, dispatcher);
            }
        }

        /// <summary>
        /// Runs a binned build across the input <see cref="NodeChild"/> buffer.
        /// </summary>
        /// <param name="subtrees">Subtrees (either leaves or nodes) to run the builder over. The builder may make in-place modifications to the input buffer; the input buffer should not be assumed to be in a valid state after the builder runs.</param>
        /// <param name="subtreesPong">A parallel buffer to subtrees which is used as a scratch buffer during execution. If a default initialized buffer is provided, a slower sequential in-place fallback will be used.</param>
        /// <param name="nodes">Buffer holding the nodes created by the build process.<para/>
        /// Nodes are created in a depth first ordering with respect to the input buffer.</param>
        /// <param name="metanodes">Buffer holding the metanodes created by the build process.<para/>
        /// Metanodes, like nodes, are created in a depth first ordering with respect to the input buffer.
        /// Metanodes are in the same order and in the same slots; they simply contain data about nodes that most traversals don't need to know about.</param>
        /// <param name="leaves">Buffer holding the leaf references created by the build process.<para/>
        /// The indices written by the build process are those defined in the inputs; any <see cref="NodeChild.Index"/> that is negative is encoded according to <see cref="Tree.Encode(int)"/> and points into the leaf buffer.</param>
        /// <param name="binIndices">Buffer to be used for caching bin indices during execution. If subtreesPong is defined, binIndices must also be defined, and vice versa.</param>
        /// <param name="dispatcher">Thread dispatcher used to accelerate the build process.</param>
        /// <param name="taskStackPointer">Task stack used to accelerate the build process. Can be null; one will be created if not provided.</param>
        /// <param name="workerCount">Number of workers to be used in the builder.</param>
        /// <param name="pool">Buffer pool used to preallocate temporary resources for building.</param>
        /// <param name="minimumBinCount">Minimum number of bins the builder should use per node.</param>
        /// <param name="maximumBinCount">Maximum number of bins the builder should use per node. Must be no higher than 255.</param>
        /// <param name="leafToBinMultiplier">Multiplier to apply to the subtree count within a node to decide the bin count. Resulting value will then be clamped by the minimum/maximum bin counts.</param>
        /// <param name="microsweepThreshold">Threshold at or under which the binned builder resorts to local counting sort sweeps.</param>
        static unsafe void BinnedBuilderInternal(Buffer<NodeChild> subtrees, Buffer<NodeChild> subtreesPong, Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<Leaf> leaves, Buffer<byte> binIndices,
            IThreadDispatcher dispatcher, LinkedTaskStack* taskStackPointer, int workerCount, BufferPool pool, int minimumBinCount, int maximumBinCount, float leafToBinMultiplier, int microsweepThreshold)
        {
            var subtreeCount = subtrees.Length;
            if (nodes.Length < subtreeCount - 1)
                throw new ArgumentException($"The nodes buffer is too small to hold all the nodes that will be necessary for the input subtrees.");
            if (maximumBinCount > 255)
                throw new ArgumentException($"Maximum bin count must fit in a byte (maximum of 255).");
            if (subtreesPong.Allocated != binIndices.Allocated)
                throw new ArgumentException("The parameters subtreesPong and binIndices must both be allocated or unallocated.");
            if (subtreeCount == 0)
                return;
            if (subtreeCount == 1)
            {
                //If there's only one leaf, the tree has a special format: the root node has only one child.
                ref var root = ref nodes[0];
                root.A = subtrees[0];
                root.B = default;
                return;
            }
            nodes = nodes.Slice(subtreeCount - 1);

            //Don't let the user pick values that will just cause an explosion.
            Debug.Assert(minimumBinCount >= 2 && maximumBinCount >= 2, "At least two bins are required. In release mode, this will be clamped up to 2, but where did lower values come from?");
            minimumBinCount = int.Max(2, minimumBinCount);
            maximumBinCount = int.Max(2, maximumBinCount);
            //The microsweep uses the same resources as the bin allocations, so expand to hold whichever is larger.
            var allocatedBinCount = int.Max(maximumBinCount, microsweepThreshold);


            if (dispatcher == null && taskStackPointer == null)
            {
                //Use the single threaded path.
                var allocatedByteCount = allocatedBinCount * 4 * sizeof(BoundingBox4) + allocatedBinCount * sizeof(int);
                var binBoundsMemoryAllocation = stackalloc byte[allocatedByteCount + 32];
                //Should be basically irrelevant, but just in case it's not on some platform, align the allocation.
                binBoundsMemoryAllocation = (byte*)(((ulong)binBoundsMemoryAllocation + 31ul) & (~31ul));
                var binBoundsMemory = new Buffer<byte>(binBoundsMemoryAllocation, allocatedByteCount);

                var threading = new SingleThreaded(binBoundsMemory, allocatedBinCount);
                var context = new Context<Buffer<Leaf>, SingleThreaded>(
                    minimumBinCount, maximumBinCount, leafToBinMultiplier, microsweepThreshold,
                     subtrees, default, leaves, nodes, metanodes, binIndices, threading);
                BinnedBuildNode(false, 0, 0, subtreeCount, -1, -1, default, &context, 0, null);
            }
            else
            {
                //Multithreaded execution of some type.
                if (dispatcher != null)
                {
                    //There's a task queue; we should use a multithreaded dispatch.
                    //While we could allocate on the stack with reasonable safety in the single threaded path, that's not very reasonable for the multithreaded path.
                    //Each worker thread could be given a node job which executes asynchronously with respect to other node jobs.
                    //Those node jobs could spawn multithreaded work that other workers assist with.
                    //Each of those jobs needs its own context for those workers, and the number of jobs is not 1:1 with the workers.
                    //We'll handle such dispatch-required allocations from worker pools. Here, we just preallocate stuff for the first level across all workers.
                    pool.Take<byte>(allocatedBinCount * workerCount * (sizeof(BoundingBox4) * 4 + sizeof(int)), out var workerBinsAllocation);

                    BinnedBuildWorkerContext* workerContextsPointer = stackalloc BinnedBuildWorkerContext[workerCount];
                    var workerContexts = new Buffer<BinnedBuildWorkerContext>(workerContextsPointer, workerCount);

                    int binAllocationStart = 0;
                    for (int i = 0; i < workerCount; ++i)
                    {
                        workerContexts[i] = new BinnedBuildWorkerContext(workerBinsAllocation, ref binAllocationStart, allocatedBinCount);
                    }

                    LinkedTaskStack taskStack = default;
                    bool createdTaskQueueLocally = taskStackPointer == null;
                    if (taskStackPointer == null)
                    {
                        taskStack = new LinkedTaskStack(pool, dispatcher, dispatcher.ThreadCount);
                        taskStackPointer = &taskStack;
                    }
                    var threading = new MultithreadBinnedBuildContext
                    {
                        TopLevelTargetTaskCount = dispatcher.ThreadCount,
                        OriginalSubtreeCount = subtrees.Length,
                        TaskStack = taskStackPointer,
                        Workers = workerContexts,
                    };
                    var context = new Context<Buffer<Leaf>, MultithreadBinnedBuildContext>(
                        minimumBinCount, maximumBinCount, leafToBinMultiplier, microsweepThreshold,
                        subtrees, subtreesPong, leaves, nodes, metanodes, binIndices, threading);

                    taskStackPointer->PushUnsafely(new Task(&BinnedBuilderWorkerEntry<Buffer<Leaf>>, &context), 0, dispatcher);
                    dispatcher.DispatchWorkers(&BinnedBuilderWorkerFunction<Buffer<Leaf>>, unmanagedContext: taskStackPointer);

                    if (createdTaskQueueLocally)
                        taskStack.Dispose(pool, dispatcher);
                    pool.Return(ref workerBinsAllocation);
                }
                else
                {
                    //TODO: TaskStackPointer-only path?
                    throw new NotImplementedException("Haven't yet implemented this path, or decided if it should really exist!");
                }
            }
        }

        unsafe static void BinnedBuilderWorkerEntry<TLeaves>(long taskId, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
            where TLeaves : unmanaged
        {
            var context = (Context<TLeaves, MultithreadBinnedBuildContext>*)untypedContext;
            BinnedBuildNode(false, 0, 0, context->SubtreesPing.Length, -1, -1, default, context, workerIndex, dispatcher);
            //Once the entry point returns, all workers should stop because it won't return unless both nodes are done.
            context->Threading.TaskStack->RequestStop();
        }

        unsafe static void BinnedBuilderWorkerFunction<TLeaves>(int workerIndex, IThreadDispatcher dispatcher)
            where TLeaves : unmanaged
        {
            var taskQueue = (LinkedTaskStack*)dispatcher.UnmanagedContext;
            PopTaskResult popTaskResult;
            var waiter = new SpinWait();
            while ((popTaskResult = taskQueue->TryPopAndRun(workerIndex, dispatcher)) != PopTaskResult.Stop)
            {
                waiter.SpinOnce(-1);
            }
        }

        public static unsafe void BinnedBuilder(Buffer<NodeChild> subtrees, Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<Leaf> leaves,
            IThreadDispatcher threadDispatcher, BufferPool pool, int minimumBinCount = 16, int maximumBinCount = 64, float leafToBinMultiplier = 1 / 16f, int microsweepThreshold = 64)
        {
            pool.Take(subtrees.Length, out Buffer<NodeChild> subtreesPong);
            pool.Take(subtrees.Length, out Buffer<byte> binIndices);
            BinnedBuilderInternal(subtrees, subtreesPong, nodes, metanodes, leaves, binIndices, threadDispatcher, null, threadDispatcher.ThreadCount, pool, minimumBinCount, maximumBinCount, leafToBinMultiplier, microsweepThreshold);
            pool.Return(ref subtreesPong);
            pool.Return(ref binIndices);
        }

        /// <summary>
        /// Runs a binned build across the subtrees buffer.
        /// </summary>
        /// <param name="subtrees">Subtrees (either leaves or nodes) to run the builder over. The builder may make in-place modifications to the input buffer; the input buffer should not be assumed to be in a valid state after the builder runs.</param>
        /// <param name="nodes">Buffer holding the nodes created by the build process.<para/>
        /// Nodes are created in a depth first ordering with respect to the input buffer.</param>
        /// <param name="metanodes">Buffer holding the metanodes created by the build process.<para/>
        /// Metanodes, like nodes, are created in a depth first ordering with respect to the input buffer.
        /// Metanodes are in the same order and in the same slots; they simply contain data about nodes that most traversals don't need to know about.</param>
        /// <param name="leaves">Buffer holding the leaf references created by the build process.<para/>
        /// The indices written by the build process are those defined in the inputs; any <see cref="NodeChild.Index"/> that is negative is encoded according to <see cref="Tree.Encode(int)"/> and points into the leaf buffer.</param>
        /// <param name="pool">Buffer pool used to preallocate a pingpong buffer if the number of subtrees exceeds maximumSubtreeStackAllocationCount. If null, stack allocation or a slower in-place partitioning will be used.</param>
        /// <param name="maximumSubtreeStackAllocationCount">Maximum number of subtrees to try putting on the stack for the binned builder's pong buffers.<para/>
        /// Subtree counts larger than this threshold will either resort to a buffer pool allocation (if available) or slower in-place partition operations.</param>
        /// <param name="minimumBinCount">Minimum number of bins the builder should use per node.</param>
        /// <param name="maximumBinCount">Maximum number of bins the builder should use per node.</param>
        /// <param name="leafToBinMultiplier">Multiplier to apply to the subtree count within a node to decide the bin count. Resulting value will then be clamped by the minimum/maximum bin counts.</param>
        /// <param name="microsweepThreshold">Threshold at or under which the binned builder resorts to local counting sort sweeps.</param>
        public static unsafe void BinnedBuilder(Buffer<NodeChild> subtrees, Buffer<Node> nodes, Buffer<Metanode> metanodes, Buffer<Leaf> leaves,
            BufferPool pool = null, int maximumSubtreeStackAllocationCount = 4096, int minimumBinCount = 16, int maximumBinCount = 64, float leafToBinMultiplier = 1 / 16f, int microsweepThreshold = 64)
        {
            Buffer<NodeChild> subtreesPong;
            Buffer<byte> binIndices;
            bool requiresReturn = false;
            if (subtrees.Length <= maximumSubtreeStackAllocationCount)
            {
                var subtreesPongMemory = stackalloc NodeChild[subtrees.Length];
                subtreesPong = new Buffer<NodeChild>(subtreesPongMemory, subtrees.Length);
                var binIndicesMemory = stackalloc byte[subtrees.Length];
                binIndices = new Buffer<byte>(binIndicesMemory, subtrees.Length);
            }
            else if (pool != null)
            {
                pool.Take(subtrees.Length, out subtreesPong);
                pool.Take(subtrees.Length, out binIndices);
                requiresReturn = true;
            }
            else
            {
                binIndices = default;
                subtreesPong = default;
            }
            BinnedBuilderInternal(subtrees, subtreesPong, nodes, metanodes, leaves, binIndices, null, null, 0, null, minimumBinCount, maximumBinCount, leafToBinMultiplier, microsweepThreshold);

            if (requiresReturn)
            {
                pool.Return(ref binIndices);
                pool.Return(ref subtreesPong);
            }
        }
    }
}
