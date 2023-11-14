using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuUtilities.TaskScheduling;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BepuPhysics.Trees
{
    /// <summary>
    /// Overlap callback for tree overlap queries.
    /// </summary>
    public interface IOverlapHandler
    {
        /// <summary>
        /// Handles an overlap between leaves.
        /// </summary>
        /// <param name="indexA">Index of the first leaf in the overlap.</param>
        /// <param name="indexB">Index of the second leaf in the overlap.</param>
        void Handle(int indexA, int indexB);
    }
    /// <summary>
    /// Overlap callback for tree overlap queries. Used in multithreaded contexts.
    /// </summary>
    public interface IThreadedOverlapHandler
    {
        /// <summary>
        /// Handles an overlap between leaves.
        /// </summary>
        /// <param name="indexA">Index of the first leaf in the overlap.</param>
        /// <param name="indexB">Index of the second leaf in the overlap.</param>
        /// <param name="workerIndex">Index of the worker reporting the overlap.</param>
        /// <param name="managedContext">Managed context provided by the multithreaded dispatch.</param>
        void Handle(int indexA, int indexB, int workerIndex, object managedContext);
    }

    partial struct Tree
    {

        //TODO: This contains a lot of empirically tested implementations on much older runtimes.
        //I suspect results would be different on modern versions of ryujit. In particular, recursion is very unlikely to be the fastest approach.
        //(I don't immediately recall what made the non-recursive version slower last time- it's possible that it was making use of stackalloc and I hadn't yet realized that it 
        //requires zeroing, or something along those lines.)

        //Note that all of these implementations make use of a fully generic handler. It could be dumping to a list, or it could be directly processing the results- at this
        //level of abstraction we don't know or care. It's up to the user to use a handler which maximizes performance if they want it. We'll be using this in the broad phase.
        readonly void DispatchTestForLeaf<TOverlapHandler>(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (nodeIndex < 0)
            {
                results.Handle(leafIndex, Encode(nodeIndex));
            }
            else
            {
                TestLeafAgainstNode(leafIndex, ref leafChild, nodeIndex, ref results);
            }
        }
        readonly void TestLeafAgainstNode<TOverlapHandler>(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref TOverlapHandler results)
            where TOverlapHandler : IOverlapHandler
        {
            ref var node = ref Nodes[nodeIndex];
            ref var a = ref node.A;
            ref var b = ref node.B;
            //Despite recursion, leafBounds should remain in L1- it'll be used all the way down the recursion from here.
            //However, while we likely loaded child B when we loaded child A, there's no guarantee that it will stick around.
            //Reloading that in the event of eviction would require more work than keeping the derived data on the stack.
            //TODO: this is some pretty questionable microtuning. It's not often that the post-leaf-found recursion will be long enough to evict L1. Definitely test it.
            var bIndex = b.Index;
            var aIntersects = BoundingBox.IntersectsUnsafe(leafChild, a);
            var bIntersects = BoundingBox.IntersectsUnsafe(leafChild, b);
            if (aIntersects)
            {
                DispatchTestForLeaf(leafIndex, ref leafChild, a.Index, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForLeaf(leafIndex, ref leafChild, bIndex, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly void DispatchTestForNodes<TOverlapHandler>(ref NodeChild a, ref NodeChild b, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (a.Index >= 0)
            {
                if (b.Index >= 0)
                {
                    GetOverlapsBetweenDifferentNodes(ref Nodes[a.Index], ref Nodes[b.Index], ref results);
                }
                else
                {
                    //leaf B versus node A.
                    TestLeafAgainstNode(Encode(b.Index), ref b, a.Index, ref results);
                }
            }
            else if (b.Index >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode(Encode(a.Index), ref a, b.Index, ref results);
            }
            else
            {
                //Two leaves.
                results.Handle(Encode(a.Index), Encode(b.Index));
            }
        }

        readonly void GetOverlapsBetweenDifferentNodes<TOverlapHandler>(ref Node a, ref Node b, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            //There are no shared children, so test them all.
            ref var aa = ref a.A;
            ref var ab = ref a.B;
            ref var ba = ref b.A;
            ref var bb = ref b.B;
            var aaIntersects = BoundingBox.IntersectsUnsafe(aa, ba);
            var abIntersects = BoundingBox.IntersectsUnsafe(aa, bb);
            var baIntersects = BoundingBox.IntersectsUnsafe(ab, ba);
            var bbIntersects = BoundingBox.IntersectsUnsafe(ab, bb);

            if (aaIntersects)
            {
                DispatchTestForNodes(ref aa, ref ba, ref results);
            }
            if (abIntersects)
            {
                DispatchTestForNodes(ref aa, ref bb, ref results);
            }
            if (baIntersects)
            {
                DispatchTestForNodes(ref ab, ref ba, ref results);
            }
            if (bbIntersects)
            {
                DispatchTestForNodes(ref ab, ref bb, ref results);
            }
        }

        readonly void GetOverlapsInNode<TOverlapHandler>(ref Node node, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {

            ref var a = ref node.A;
            ref var b = ref node.B;
            var ab = BoundingBox.IntersectsUnsafe(a, b);
            if (a.Index >= 0)
                GetOverlapsInNode(ref Nodes[a.Index], ref results);
            if (b.Index >= 0)
                GetOverlapsInNode(ref Nodes[b.Index], ref results);
            //Test all different nodes.
            if (ab)
            {
                DispatchTestForNodes(ref a, ref b, ref results);
            }
        }

        public readonly void GetSelfOverlaps<TOverlapHandler>(ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            //If there are less than two leaves, there can't be any overlap.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (LeafCount < 2)
                return;

            GetOverlapsInNode(ref Nodes[0], ref results);
        }

        void GetOverlapsWithLeaf<TOverlapHandler>(ref TOverlapHandler results, NodeChild leaf, int nodeToTest, ref QuickList<int> stack) where TOverlapHandler : IOverlapHandler
        {
            var leafIndex = Encode(leaf.Index);
            Debug.Assert(stack.Count == 0);
            while (true)
            {
                ref var node = ref Nodes[nodeToTest];
                var a = BoundingBox.IntersectsUnsafe(leaf, node.A);
                var b = BoundingBox.IntersectsUnsafe(leaf, node.B);
                var aIsInternal = node.A.Index >= 0;
                var bIsInternal = node.B.Index >= 0;
                var intersectedInternalA = a && aIsInternal;
                var intersectedInternalB = b && bIsInternal;
                if (intersectedInternalA && intersectedInternalB)
                {
                    nodeToTest = node.A.Index;
                    stack.AllocateUnsafely() = node.B.Index;
                }
                else
                {
                    if (a && !aIsInternal)
                    {
                        results.Handle(leafIndex, Encode(node.A.Index));
                    }
                    if (b && !bIsInternal)
                    {
                        results.Handle(leafIndex, Encode(node.B.Index));
                    }
                    if (intersectedInternalA || intersectedInternalB)
                    {
                        nodeToTest = intersectedInternalA ? node.A.Index : node.B.Index;
                    }
                    else
                    {
                        //The current traversal step doesn't offer a next step; pull from the stack.
                        if (!stack.TryPop(out nodeToTest))
                        {
                            //Nothing left to test against this leaf! Done!
                            break;
                        }
                    }
                }

                //ref var node = ref Nodes[nodeToTest];
                //var a = BoundingBox.IntersectsUnsafe(leaf, node.A);
                //var b = BoundingBox.IntersectsUnsafe(leaf, node.B);
                //var aIsInternal = node.A.Index >= 0;
                //var bIsInternal = node.B.Index >= 0;
                //var intersectedInternalA = a && aIsInternal;
                //var intersectedInternalB = b && bIsInternal;
                //if (a && !aIsInternal)
                //    results.Handle(leafIndex, Encode(node.A.Index));
                //if (b && !bIsInternal)
                //    results.Handle(leafIndex, Encode(node.B.Index));

                //if (intersectedInternalA && intersectedInternalB)
                //    stack.AllocateUnsafely() = node.B.Index;
                //else if (intersectedInternalA || intersectedInternalB)
                //    nodeToTest = intersectedInternalA ? node.A.Index : node.B.Index;
                //else if (!stack.TryPop(out nodeToTest))
                //    break;


            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static uint EncodeParentIndex(int leafParentIndex, bool childIsB)
        {
            var encoded = (uint)leafParentIndex;
            if (childIsB)
                encoded |= 1u << 31;
            return encoded;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static Vector128<int> Encode(Vector128<int> indices)
        {
            return Vector128<int>.AllBitsSet - indices;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static ref NodeChild GetLeafChild(ref Tree tree, uint encodedLeafParentIndex)
        {
            var parentNodeIndex = encodedLeafParentIndex & 0x7FFF_FFFF;
            var leafIsChildB = encodedLeafParentIndex > 0x7FFF_FFFF;
            ref var parent = ref tree.Nodes[parentNodeIndex];
            return ref leafIsChildB ? ref parent.B : ref parent.A;
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //static void LeftPack(Vector128<int> mask, Vector128<int> a, Vector128<int> b, out Vector128<int> packedA, out Vector128<int> packedB, out int count)
        //{
        //    var bitmask = Vector128.ExtractMostSignificantBits(mask);
        //    count = BitOperations.PopCount(bitmask);
        //    switch (bitmask)
        //    {
        //        //     0000 requires no shuffle.
        //        //     0001 requires no shuffle.
        //        case 0b0010: packedA = Vector128.Shuffle(a, Vector128.Create(1, 0, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(1, 0, 0, 0)); break;
        //        //     0011 requires no shuffle.
        //        case 0b0100: packedA = Vector128.Shuffle(a, Vector128.Create(2, 0, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(2, 0, 0, 0)); break;
        //        case 0b0101: packedA = Vector128.Shuffle(a, Vector128.Create(0, 2, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(0, 2, 0, 0)); break;
        //        case 0b0110: packedA = Vector128.Shuffle(a, Vector128.Create(1, 2, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(1, 2, 0, 0)); break;
        //        //     0111 requires no shuffle.
        //        case 0b1000: packedA = Vector128.Shuffle(a, Vector128.Create(3, 0, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(3, 0, 0, 0)); break;
        //        case 0b1001: packedA = Vector128.Shuffle(a, Vector128.Create(0, 3, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(0, 3, 0, 0)); break;
        //        case 0b1010: packedA = Vector128.Shuffle(a, Vector128.Create(1, 3, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(1, 3, 0, 0)); break;
        //        case 0b1011: packedA = Vector128.Shuffle(a, Vector128.Create(0, 1, 3, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(0, 1, 3, 0)); break;
        //        case 0b1100: packedA = Vector128.Shuffle(a, Vector128.Create(2, 3, 0, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(2, 3, 0, 0)); break;
        //        case 0b1101: packedA = Vector128.Shuffle(a, Vector128.Create(0, 2, 3, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(0, 2, 3, 0)); break;
        //        case 0b1110: packedA = Vector128.Shuffle(a, Vector128.Create(1, 2, 3, 0)); packedB = Vector128.Shuffle(b, Vector128.Create(1, 2, 3, 0)); break;
        //        //     1111 requires no shuffle.
        //        default: packedA = a; packedB = b; break;
        //    }

        //}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector128<int> GetLeftPackMask(Vector128<int> mask, out int count)
        {
            if (!Avx2.IsSupported) throw new NotSupportedException("No fallback exists! This should never be visible!");

            var bitmask = Vector128.ExtractMostSignificantBits(mask);
            //This depends upon an optimization that preallocates the constant array in fixed memory. Bit of a turbohack that won't work on other runtimes.
            //The lookup table includes one entry for each of the 256 possible bitmasks. Each lane requires 3 bits to define the source for a shuffle mask.
            //3 bits, 8 lanes, 256 bitmasks means only 768 bytes.
            ReadOnlySpan<byte> lookupTable = new byte[] {
                //0000       0001         0010         0011         0100         0101         0110         0111
                0b1110_0100, 0b1110_0100, 0b1110_0101, 0b1110_0100, 0b1110_0110, 0b1110_1000, 0b1110_1001, 0b1110_0100,
                //1000       1001         1010         1011         1100         1101         1110         1111
                0b1110_0111, 0b1110_1100, 0b1110_1101, 0b1111_0100, 0b1110_1110, 0b1111_1000, 0b1111_1001, 0b1110_0100 };
            var encodedLeftPackMask = Unsafe.Add(ref Unsafe.AsRef(in lookupTable[0]), bitmask);

            count = BitOperations.PopCount(bitmask);
            //Broadcast, variable shift.
            return Avx2.ShiftRightLogicalVariable(Vector128.Create((int)encodedLeftPackMask), Vector128.Create(0u, 2, 4, 6));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void LeftPack(Vector128<int> mask, Vector128<int> a, Vector128<int> b, out Vector128<int> packedA, out Vector128<int> packedB, out int count)
        {
            var permuteMask = GetLeftPackMask(mask, out count);
            packedA = Avx.PermuteVar(a.AsSingle(), permuteMask).AsInt32();
            packedB = Avx.PermuteVar(b.AsSingle(), permuteMask).AsInt32();

        }

        struct IndexPair
        {
            public int A;
            public int B;
        }

        unsafe struct NodeLeafPair
        {
            public NodeChild* LeafParent;
            public int NodeIndex;
        }

        unsafe void AddCrossoverResult<TOverlapHandler>(ref NodeChild a, ref NodeChild b, ref QuickList<IndexPair> crossovers, ref QuickList<NodeLeafPair> nodeLeaf, ref TOverlapHandler results, BufferPool pool) where TOverlapHandler : IOverlapHandler
        {
            if (a.Index >= 0 && b.Index >= 0)
            {
                crossovers.Allocate(pool) = new IndexPair { A = a.Index, B = b.Index };
            }
            else if (a.Index < 0 && b.Index < 0)
            {
                results.Handle(Encode(a.Index), Encode(b.Index));
            }
            else
            {
                nodeLeaf.Allocate(pool) = a.Index >= 0
                    ? new NodeLeafPair { LeafParent = (NodeChild*)Unsafe.AsPointer(ref b), NodeIndex = a.Index }
                    : new NodeLeafPair { LeafParent = (NodeChild*)Unsafe.AsPointer(ref a), NodeIndex = b.Index };
            }
        }

        void ExecuteCrossoverBatch<TOverlapHandler>(ref QuickList<IndexPair> crossovers, ref QuickList<NodeLeafPair> nodeLeaf, ref TOverlapHandler results, BufferPool pool) where TOverlapHandler : IOverlapHandler
        {
            while (crossovers.TryPop(out var pair))
            {
                ref var a = ref Nodes[pair.A];
                ref var b = ref Nodes[pair.B];
                //There are no shared children, so test them all.
                ref var aa = ref a.A;
                ref var ab = ref a.B;
                ref var ba = ref b.A;
                ref var bb = ref b.B;
                var aaIntersects = BoundingBox.IntersectsUnsafe(aa, ba);
                var abIntersects = BoundingBox.IntersectsUnsafe(aa, bb);
                var baIntersects = BoundingBox.IntersectsUnsafe(ab, ba);
                var bbIntersects = BoundingBox.IntersectsUnsafe(ab, bb);

                if (aaIntersects)
                {
                    AddCrossoverResult(ref aa, ref ba, ref crossovers, ref nodeLeaf, ref results, pool);
                }
                if (abIntersects)
                {
                    AddCrossoverResult(ref aa, ref bb, ref crossovers, ref nodeLeaf, ref results, pool);
                }
                if (baIntersects)
                {
                    AddCrossoverResult(ref ab, ref ba, ref crossovers, ref nodeLeaf, ref results, pool);
                }
                if (bbIntersects)
                {
                    AddCrossoverResult(ref ab, ref bb, ref crossovers, ref nodeLeaf, ref results, pool);
                }
            }
        }
        unsafe void ExecuteNodeLeafBatch<TOverlapHandler>(ref QuickList<NodeLeafPair> nodeLeaf, ref TOverlapHandler results, BufferPool pool) where TOverlapHandler : IOverlapHandler
        {
            while (nodeLeaf.TryPop(out var pair))
            {
                ref var leafChild = ref *pair.LeafParent;
                ref var node = ref Nodes[pair.NodeIndex];
                ref var a = ref node.A;
                ref var b = ref node.B;
                var bIndex = b.Index;
                var aIntersects = BoundingBox.IntersectsUnsafe(leafChild, a);
                var bIntersects = BoundingBox.IntersectsUnsafe(leafChild, b);
                if (aIntersects)
                {
                    if (a.Index < 0)
                    {
                        results.Handle(Encode(leafChild.Index), Encode(a.Index));
                    }
                    else
                    {
                        nodeLeaf.Allocate(pool) = new NodeLeafPair { LeafParent = pair.LeafParent, NodeIndex = a.Index };
                    }
                }
                if (bIntersects)
                {
                    if (b.Index < 0)
                    {
                        results.Handle(Encode(leafChild.Index), Encode(b.Index));
                    }
                    else
                    {
                        nodeLeaf.Allocate(pool) = new NodeLeafPair { LeafParent = pair.LeafParent, NodeIndex = b.Index };
                    }
                }
            }
        }

        void FlushLeafLeaf<TOverlapHandler>(ref QuickList<IndexPair> leafLeaf, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            for (int leafLeafIndex = 0; leafLeafIndex < leafLeaf.Count; ++leafLeafIndex)
            {
                var pair = leafLeaf[leafLeafIndex];
                results.Handle(Encode(pair.A), Encode(pair.B));
            }
            leafLeaf.Count = 0;
        }

        public unsafe void GetSelfOverlapsBatched<TOverlapHandler>(ref TOverlapHandler results, BufferPool pool) where TOverlapHandler : IOverlapHandler
        {
            const int crossoverBatchSizeTarget = 16;
            const int nodeLeafBatchSizeTarget = 16;
            var crossovers = new QuickList<IndexPair>(crossoverBatchSizeTarget * 16, pool);
            var nodeLeaf = new QuickList<NodeLeafPair>(nodeLeafBatchSizeTarget * 16, pool);
            for (int i = NodeCount - 1; i >= 0; --i)
            {
                ref var node = ref Nodes[i];
                ref var a = ref node.A;
                ref var b = ref node.B;
                if (BoundingBox.IntersectsUnsafe(a, b))
                {
                    if (a.Index >= 0 && b.Index >= 0)
                    {
                        crossovers.Allocate(pool) = new IndexPair { A = a.Index, B = b.Index };
                    }
                    else if (a.Index < 0 && b.Index < 0)
                    {
                        results.Handle(Encode(a.Index), Encode(b.Index));
                    }
                    else
                    {
                        //Leaf-node.
                        nodeLeaf.Allocate(pool) = a.Index >= 0
                            ? new NodeLeafPair { LeafParent = (NodeChild*)Unsafe.AsPointer(ref b), NodeIndex = a.Index }
                            : new NodeLeafPair { LeafParent = (NodeChild*)Unsafe.AsPointer(ref a), NodeIndex = b.Index };
                    }
                }
                if (crossovers.Count >= crossoverBatchSizeTarget)
                {
                    ExecuteCrossoverBatch(ref crossovers, ref nodeLeaf, ref results, pool);
                }
                if (nodeLeaf.Count >= nodeLeafBatchSizeTarget)
                {
                    ExecuteNodeLeafBatch(ref nodeLeaf, ref results, pool);
                }
            }
            //Flush any remaining pairs.
            ExecuteCrossoverBatch(ref crossovers, ref nodeLeaf, ref results, pool);
            ExecuteNodeLeafBatch(ref nodeLeaf, ref results, pool);
            crossovers.Dispose(pool);
            nodeLeaf.Dispose(pool);
        }





        readonly void GetSelfOverlaps2<TOverlapHandler>(ref TOverlapHandler results, int start, int end) where TOverlapHandler : IOverlapHandler
        {
            Debug.Assert(end >= 0 && end <= NodeCount && start >= 0 && start < NodeCount);
            for (int i = end - 1; i >= start; --i)
            {
                ref var node = ref Nodes[i];
                ref var a = ref node.A;
                ref var b = ref node.B;
                var ab = BoundingBox.IntersectsUnsafe(a, b);
                if (ab)
                {
                    DispatchTestForNodes(ref a, ref b, ref results);
                }
            }
        }

        /// <summary>
        /// Reports all bounding box overlaps between leaves in the tree to the given <typeparamref name="TOverlapHandler"/>.
        /// </summary>
        /// <param name="results">Handler to report results to.</param>
        public readonly void GetSelfOverlaps2<TOverlapHandler>(ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            GetSelfOverlaps2(ref results, 0, NodeCount);
        }

        unsafe struct SelfTestContext<TOverlapHandler> where TOverlapHandler : unmanaged, IThreadedOverlapHandler
        {
            public Tree Tree;
            public int LoopTaskCount;
            public int LeafThresholdForTask;
            public TOverlapHandler* Results;
            public TaskStack* TaskStack;
        }
        unsafe struct WrappedOverlapHandler<TOverlapHandler> : IOverlapHandler where TOverlapHandler : unmanaged, IThreadedOverlapHandler
        {
            public int WorkerIndex;
            public object ManagedContext;
            public TOverlapHandler* Inner;
            public void Handle(int indexA, int indexB)
            {
                Inner->Handle(indexA, indexB, WorkerIndex, ManagedContext);
            }
        }

        unsafe static void LoopEntryTask<TOverlapHandler>(long taskStartAndEnd, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
        {
            var taskStart = (int)taskStartAndEnd;
            var taskEnd = (int)(taskStartAndEnd >> 32);
            ref var context = ref *(SelfTestContext<TOverlapHandler>*)untypedContext;
            var wrapped = new WrappedOverlapHandler<TOverlapHandler> { Inner = context.Results, WorkerIndex = workerIndex, ManagedContext = dispatcher.ManagedContext };
            context.Tree.GetSelfOverlaps2(ref wrapped, taskStart, taskEnd);
        }



        private unsafe void GetSelfOverlaps2<TOverlapHandler>(ref TOverlapHandler results, BufferPool pool,
        int workerIndex, TaskStack* taskStack, IThreadDispatcher threadDispatcher, bool internallyDispatch, int workerCount, int targetTaskBudget, object managedContext = null) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
        {
            if (targetTaskBudget < 0)
                targetTaskBudget = threadDispatcher.ThreadCount;
            targetTaskBudget *= 16;
            targetTaskBudget = int.Min(NodeCount, targetTaskBudget);

            const int leafThresholdForTask = 256;

            var resultsCopy = results;
            var context = new SelfTestContext<TOverlapHandler> { Tree = this, LoopTaskCount = targetTaskBudget, LeafThresholdForTask = leafThresholdForTask, Results = &resultsCopy, TaskStack = taskStack };

            //Go ahead and submit very large early nodes as independent tasks to help with load balancing.
            //(This isn't guaranteed, or even intended, to catch all large individual nodes. It's just an easy way to get some of them.)
            var earlyIsolatedNodeIntervalEnd = 0;
            const int maximumIsolatedNodeCapacity = 32;
            int isolatedNodeCapacity = int.Min(maximumIsolatedNodeCapacity, targetTaskBudget / 4);
            var earlyIsolatedNodesMemory = stackalloc int[isolatedNodeCapacity];
            var earlyIsolatedNodes = new QuickList<int>(new Buffer<int>(earlyIsolatedNodesMemory, isolatedNodeCapacity));
            for (int i = 0; i < NodeCount && earlyIsolatedNodes.Count < isolatedNodeCapacity; ++i)
            {
                ref var node = ref Nodes[i];
                ref var a = ref node.A;
                ref var b = ref node.B;
                if (int.Max(a.LeafCount, b.LeafCount) > leafThresholdForTask)
                {
                    if (BoundingBox.IntersectsUnsafe(a, b))
                    {
                        //Note that this technically does double work on the bounds test with the way we're submitting this as a task. Don't care; it's constant bounded nanoseconds.
                        earlyIsolatedNodes.AllocateUnsafely() = i;
                    }
                }
                else
                {
                    earlyIsolatedNodeIntervalEnd = i;
                    break;
                }
            }

            var remainingNodeCount = NodeCount - earlyIsolatedNodeIntervalEnd;
            var regularLoopTaskCount = targetTaskBudget - earlyIsolatedNodes.Count;
            var nodesPerTaskBase = remainingNodeCount / regularLoopTaskCount;
            var remainder = remainingNodeCount - nodesPerTaskBase * regularLoopTaskCount;
            var tasks = new Buffer<Task>(targetTaskBudget, pool);
            int previousEnd = earlyIsolatedNodeIntervalEnd;
            for (int i = 0; i < regularLoopTaskCount; ++i)
            {
                var taskStart = previousEnd;
                var nodeCountForTask = i < remainder ? nodesPerTaskBase + 1 : nodesPerTaskBase;
                var taskEnd = previousEnd + nodeCountForTask;
                previousEnd = taskEnd;
                tasks[i] = new Task(&LoopEntryTask<TOverlapHandler>, &context, (uint)taskStart | (((long)taskEnd) << 32));
            }
            //Stick the early isolated nodes at the end so they're popped first.
            for (int i = 0; i < earlyIsolatedNodes.Count; ++i)
            {
                var taskStart = earlyIsolatedNodes[i];
                tasks[tasks.Length - i - 1] = new Task(&LoopEntryTask<TOverlapHandler>, &context, (uint)taskStart | (((long)(taskStart + 1)) << 32));
            }
            if (internallyDispatch)
            {
                //There isn't an active dispatch, so we need to do it.
                taskStack->AllocateContinuationAndPush(tasks, workerIndex, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(taskStack));
                TaskStack.DispatchWorkers(threadDispatcher, taskStack, workerCount, managedContext);
            }
            else
            {
                //We're executing from within a multithreaded dispatch already, so we can simply run the tasks and trust that other threads are ready to steal.
                taskStack->RunTasks(tasks, workerIndex, threadDispatcher);
            }
            tasks.Dispose(pool);
            //Have to copy back the results; it's a value type.
            results = resultsCopy;
        }

        /// <summary>
        /// Reports all bounding box overlaps between leaves in the tree to the given <typeparamref name="TOverlapHandler"/>. Uses the thread dispatcher to parallelize overlap testing.
        /// </summary>
        /// <param name="results">Handler to report results to.</param>
        /// <param name="pool">Pool used for ephemeral allocations.</param>
        /// <param name="threadDispatcher">Thread dispatcher used during the overlap testing.</param>
        /// <param name="managedContext">Managed context to provide to the overlap handler, if any.</param>
        public unsafe void GetSelfOverlaps2<TOverlapHandler>(ref TOverlapHandler results, BufferPool pool, IThreadDispatcher threadDispatcher, object managedContext = null) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
        {
            var taskStack = new TaskStack(pool, threadDispatcher, threadDispatcher.ThreadCount);
            GetSelfOverlaps2(ref results, pool, 0, &taskStack, threadDispatcher, true, threadDispatcher.ThreadCount, threadDispatcher.ThreadCount, managedContext);
            taskStack.Dispose(pool, threadDispatcher);
        }

        /// <summary>
        /// Reports all bounding box overlaps between leaves in the tree to the given <typeparamref name="TOverlapHandler"/>.
        /// <para/>Pushes tasks into the provided <see cref="TaskStack"/>. Does not dispatch threads internally; this is intended to be used as a part of a caller-managed dispatch.
        /// </summary>
        /// <param name="results">Handler to report results to.</param>
        /// <param name="pool">Pool used for ephemeral allocations.</param>
        /// <param name="threadDispatcher">Thread dispatcher used during the overlap test.</param>
        /// <param name="taskStack"><see cref="TaskStack"/> that the overlap test will push tasks onto as needed.</param>
        /// <param name="workerIndex">Index of the worker calling the function.</param>
        /// <param name="targetTaskCount">Number of tasks the overlap testing should try to create during execution. If negative, uses <see cref="IThreadDispatcher.ThreadCount"/>.</param>
        /// <remarks>This does not dispatch workers on the <see cref="IThreadDispatcher"/> directly. If the overlap handler requires managed context, that should be provided by whatever dispatched the workers.</remarks>
        public unsafe void GetSelfOverlaps2<TOverlapHandler>(ref TOverlapHandler results, BufferPool pool,
             IThreadDispatcher threadDispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount = -1) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
        {
            GetSelfOverlaps2(ref results, pool, workerIndex, taskStack, threadDispatcher, false, threadDispatcher.ThreadCount, targetTaskCount);
        }
    }
}
