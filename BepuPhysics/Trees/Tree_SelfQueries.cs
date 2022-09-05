using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Reflection.Metadata.Ecma335;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Security.Cryptography;

namespace BepuPhysics.Trees
{
    public interface IOverlapHandler
    {
        void Handle(int indexA, int indexB);
    }


    partial struct Tree
    {
        //TODO: This contains a lot of empirically tested implementations on much older runtimes.
        //I suspect results would be different on modern versions of ryujit. In particular, recursion is very unlikely to be the fastest approach.
        //(I don't immediately recall what made the non-recursive version slower last time- it's possible that it was making use of stackalloc and I hadn't yet realized that it 
        //requires zeroing, or something along those lines.)

        //Note that all of these implementations make use of a fully generic handler. It could be dumping to a list, or it could be directly processing the results- at this
        //level of abstraction we don't know or care. It's up to the user to use a handler which maximizes performance if they want it. We'll be using this in the broad phase.
        unsafe void DispatchTestForLeaf<TOverlapHandler>(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
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
        unsafe void TestLeafAgainstNode<TOverlapHandler>(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref TOverlapHandler results)
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
        unsafe void DispatchTestForNodes<TOverlapHandler>(ref NodeChild a, ref NodeChild b, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
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

        unsafe void GetOverlapsBetweenDifferentNodes<TOverlapHandler>(ref Node a, ref Node b, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
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

        unsafe void GetOverlapsInNode<TOverlapHandler>(ref Node node, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
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

        public unsafe void GetSelfOverlaps<TOverlapHandler>(ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            //If there are less than two leaves, there can't be any overlap.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (LeafCount < 2)
                return;

            GetOverlapsInNode(ref Nodes[0], ref results);
        }

        unsafe void GetOverlapsWithLeaf<TOverlapHandler>(ref TOverlapHandler results, NodeChild leaf, int nodeToTest, ref QuickList<int> stack) where TOverlapHandler : IOverlapHandler
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
        struct StackEntry
        {
            public int A;
            public int B;
        }

        struct LeafNodeStackEntry
        {
            public uint EncodedLeafParentIndex;
            public int NodeIndex;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public LeafNodeStackEntry(int leafParentIndex, bool leafChildIsB, int nodeIndex)
            {
                EncodedLeafParentIndex = (uint)leafParentIndex;
                if (leafChildIsB)
                    EncodedLeafParentIndex |= 1u << 31;
                NodeIndex = nodeIndex;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ref NodeChild GetChild(ref Tree tree)
            {
                var parentNodeIndex = EncodedLeafParentIndex & 0x7FFF_FFFF;
                var leafIsChildB = EncodedLeafParentIndex > 0x7FFF_FFFF;
                ref var parent = ref tree.Nodes[parentNodeIndex];
                return ref leafIsChildB ? ref parent.B : ref parent.A;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transpose(
            Vector256<float> m0, Vector256<float> m1, Vector256<float> m2, Vector256<float> m3, Vector256<float> m4, Vector256<float> m5, Vector256<float> m6, Vector256<float> m7,
            out Vector256<float> t0, out Vector256<float> t1, out Vector256<float> t2, out Vector256<float> t3, out Vector256<float> t4, out Vector256<float> t5, out Vector256<float> t6, out Vector256<float> t7)
        {

            if (Avx.IsSupported)
            {
                var n0 = Avx.UnpackLow(m0, m1);
                var n1 = Avx.UnpackLow(m2, m3);
                var n2 = Avx.UnpackLow(m4, m5);
                var n3 = Avx.UnpackLow(m6, m7);
                var n4 = Avx.UnpackHigh(m0, m1);
                var n5 = Avx.UnpackHigh(m2, m3);
                var n6 = Avx.UnpackHigh(m4, m5);
                var n7 = Avx.UnpackHigh(m6, m7);

                var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                var o7 = Avx.Shuffle(n6, n7, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                t0 = Avx.Permute2x128(o0, o1, 0 | (2 << 4));
                t1 = Avx.Permute2x128(o4, o5, 0 | (2 << 4));
                t2 = Avx.Permute2x128(o2, o3, 0 | (2 << 4));
                t3 = Avx.Permute2x128(o6, o7, 0 | (2 << 4));

                t4 = Avx.Permute2x128(o0, o1, 1 | (3 << 4));
                t5 = Avx.Permute2x128(o4, o5, 1 | (3 << 4));
                t6 = Avx.Permute2x128(o2, o3, 1 | (3 << 4));
                t7 = Avx.Permute2x128(o6, o7, 1 | (3 << 4));
            }
            else
            {
                throw new NotSupportedException("No fallback exists! This should never be visible!");
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector256<int> Intersects(
            Vector256<float> minAX, Vector256<float> minAY, Vector256<float> minAZ, Vector256<float> maxAX, Vector256<float> maxAY, Vector256<float> maxAZ,
            Vector256<float> minBX, Vector256<float> minBY, Vector256<float> minBZ, Vector256<float> maxBX, Vector256<float> maxBY, Vector256<float> maxBZ)
        {
            return (
                Vector256.GreaterThanOrEqual(maxAX, minBX) & Vector256.GreaterThanOrEqual(maxAY, minBY) & Vector256.GreaterThanOrEqual(maxAZ, minBZ) &
                Vector256.GreaterThanOrEqual(maxBX, minAX) & Vector256.GreaterThanOrEqual(maxBY, minAY) & Vector256.GreaterThanOrEqual(maxBZ, minBZ)).As<float, int>();
        }

        public unsafe void GetSelfOverlapsContiguousPrepass<TOverlapHandler>(ref TOverlapHandler results, BufferPool pool) where TOverlapHandler : IOverlapHandler
        {
            //If there are less than two leaves, there can't be any overlap.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (LeafCount < 2)
                return;

            //A recursive self test will at some point visit all nodes with certainty. Instead of framing it as a recursive test at all, do a prepass that's just a contiguous iteration.
            pool.Take<int>(NodeCount, out var crossoverStackA);
            pool.Take<int>(NodeCount, out var crossoverStackB);
            int crossoverStackCount = 0;
            QuickList<LeafNodeStackEntry> leafNodeStack = new(NodeCount, pool);
            for (int i = 0; i < NodeCount; ++i)
            {
                ref var node = ref Nodes[i];
                var a = node.A.Index;
                var b = node.B.Index;
                var aIsInternal = a >= 0;
                var bIsInternal = b >= 0;
                if (aIsInternal && bIsInternal)
                {
                    if (BoundingBox.IntersectsUnsafe(node.A, node.B))
                    {
                        crossoverStackA[crossoverStackCount] = a;
                        crossoverStackB[crossoverStackCount] = b;
                        ++crossoverStackCount;
                    }
                }
                else if (aIsInternal || bIsInternal)
                {
                    //One is a leaf, one is internal.
                    leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(i, aIsInternal, aIsInternal ? a : b);
                }
                else
                {
                    //Both are leaves.
                    results.Handle(Encode(a), Encode(b));
                }
            }

            //Now, we need to complete all crossovers and leaf-node tests. Note that leaf-node tests can only generate leaf-node or leaf-leaf tests, they never produce crossovers.
            //In contrast, crossovers can generate leaf-node tests. So do crossovers first so we'll have every leaf-node test in the stack ready to go.
            if (crossoverStackCount > 0)
            {
                if (Vector256.IsHardwareAccelerated)
                {
                    //In a given iteration, the maximum number of leaf-leaf intersections is 8 lanes * 4 child combinations per lane.
                    var toReportA = stackalloc int[32];
                    var toReportB = stackalloc int[32];                    

                    while (crossoverStackCount > 0)
                    {
                        //Note that we do not have a 'nextCrossover' entry held over from the previous iteration. Instead, we directly yoink off the stack. Serves the same purpose.
                        //Pick the appropriate location to load from in the stack.
                        var nextLaneCount = 8 < crossoverStackCount ? 8 : crossoverStackCount;
                        var startIndex = crossoverStackCount - nextLaneCount;
                        var nextCrossover0 = crossoverStackA.Memory + startIndex;
                        var nextCrossover1 = crossoverStackB.Memory + startIndex;

                        //Each crossover execution loads two nodes, each having two children, for each lane.
                        //Gather all of it and transpose it for vectorized operations.
                        //Note that we're not actually getting much benefit on the actual intersection test here;
                        //the goal of going wide is to reduce the burden of branch misprediction by brute forcing it with vectorization.
                        var n0Pointer0 = (float*)(Nodes.Memory + nextCrossover0[0]);
                        var n0Pointer1 = (float*)(Nodes.Memory + nextCrossover0[1]);
                        var n0Pointer2 = (float*)(Nodes.Memory + nextCrossover0[2]);
                        var n0Pointer3 = (float*)(Nodes.Memory + nextCrossover0[3]);
                        var n0Pointer4 = (float*)(Nodes.Memory + nextCrossover0[4]);
                        var n0Pointer5 = (float*)(Nodes.Memory + nextCrossover0[5]);
                        var n0Pointer6 = (float*)(Nodes.Memory + nextCrossover0[6]);
                        var n0Pointer7 = (float*)(Nodes.Memory + nextCrossover0[7]);

                        var n0A0 = Vector256.Load(n0Pointer0);
                        var n0A1 = 1 < nextLaneCount ? Vector256.Load(n0Pointer1) : Vector256<float>.AllBitsSet;
                        var n0A2 = 2 < nextLaneCount ? Vector256.Load(n0Pointer2) : Vector256<float>.AllBitsSet;
                        var n0A3 = 3 < nextLaneCount ? Vector256.Load(n0Pointer3) : Vector256<float>.AllBitsSet;
                        var n0A4 = 4 < nextLaneCount ? Vector256.Load(n0Pointer4) : Vector256<float>.AllBitsSet;
                        var n0A5 = 5 < nextLaneCount ? Vector256.Load(n0Pointer5) : Vector256<float>.AllBitsSet;
                        var n0A6 = 6 < nextLaneCount ? Vector256.Load(n0Pointer6) : Vector256<float>.AllBitsSet;
                        var n0A7 = 7 < nextLaneCount ? Vector256.Load(n0Pointer7) : Vector256<float>.AllBitsSet;

                        Transpose(n0A0, n0A1, n0A2, n0A3, n0A4, n0A5, n0A6, n0A7,
                            out var n0AMinX, out var n0AMinY, out var n0AMinZ, out var n0AIndex,
                            out var n0AMaxX, out var n0AMaxY, out var n0AMaxZ, out _);

                        var n0B0 = Vector256.Load(n0Pointer0 + 8);
                        var n0B1 = 1 < nextLaneCount ? Vector256.Load(n0Pointer1 + 8) : Vector256<float>.AllBitsSet;
                        var n0B2 = 2 < nextLaneCount ? Vector256.Load(n0Pointer2 + 8) : Vector256<float>.AllBitsSet;
                        var n0B3 = 3 < nextLaneCount ? Vector256.Load(n0Pointer3 + 8) : Vector256<float>.AllBitsSet;
                        var n0B4 = 4 < nextLaneCount ? Vector256.Load(n0Pointer4 + 8) : Vector256<float>.AllBitsSet;
                        var n0B5 = 5 < nextLaneCount ? Vector256.Load(n0Pointer5 + 8) : Vector256<float>.AllBitsSet;
                        var n0B6 = 6 < nextLaneCount ? Vector256.Load(n0Pointer6 + 8) : Vector256<float>.AllBitsSet;
                        var n0B7 = 7 < nextLaneCount ? Vector256.Load(n0Pointer7 + 8) : Vector256<float>.AllBitsSet;

                        Transpose(n0B0, n0B1, n0B2, n0B3, n0B4, n0B5, n0A6, n0B7,
                            out var n0BMinX, out var n0BMinY, out var n0BMinZ, out var n0BIndex,
                            out var n0BMaxX, out var n0BMaxY, out var n0BMaxZ, out _);

                        var n1Pointer0 = (float*)(Nodes.Memory + nextCrossover1[0]);
                        var n1Pointer1 = (float*)(Nodes.Memory + nextCrossover1[1]);
                        var n1Pointer2 = (float*)(Nodes.Memory + nextCrossover1[2]);
                        var n1Pointer3 = (float*)(Nodes.Memory + nextCrossover1[3]);
                        var n1Pointer4 = (float*)(Nodes.Memory + nextCrossover1[4]);
                        var n1Pointer5 = (float*)(Nodes.Memory + nextCrossover1[5]);
                        var n1Pointer6 = (float*)(Nodes.Memory + nextCrossover1[6]);
                        var n1Pointer7 = (float*)(Nodes.Memory + nextCrossover1[7]);

                        var n1A0 = Vector256.Load(n1Pointer0);
                        var n1A1 = 1 < nextLaneCount ? Vector256.Load(n1Pointer1) : Vector256<float>.AllBitsSet;
                        var n1A2 = 2 < nextLaneCount ? Vector256.Load(n1Pointer2) : Vector256<float>.AllBitsSet;
                        var n1A3 = 3 < nextLaneCount ? Vector256.Load(n1Pointer3) : Vector256<float>.AllBitsSet;
                        var n1A4 = 4 < nextLaneCount ? Vector256.Load(n1Pointer4) : Vector256<float>.AllBitsSet;
                        var n1A5 = 5 < nextLaneCount ? Vector256.Load(n1Pointer5) : Vector256<float>.AllBitsSet;
                        var n1A6 = 6 < nextLaneCount ? Vector256.Load(n1Pointer6) : Vector256<float>.AllBitsSet;
                        var n1A7 = 7 < nextLaneCount ? Vector256.Load(n1Pointer7) : Vector256<float>.AllBitsSet;

                        Transpose(n1A0, n1A1, n1A2, n1A3, n1A4, n1A5, n1A6, n1A7,
                            out var n1AMinX, out var n1AMinY, out var n1AMinZ, out var n1AIndex,
                            out var n1AMaxX, out var n1AMaxY, out var n1AMaxZ, out _);

                        var n1B0 = Vector256.Load(n1Pointer0 + 8);
                        var n1B1 = 1 < nextLaneCount ? Vector256.Load(n1Pointer1 + 8) : Vector256<float>.AllBitsSet;
                        var n1B2 = 2 < nextLaneCount ? Vector256.Load(n1Pointer2 + 8) : Vector256<float>.AllBitsSet;
                        var n1B3 = 3 < nextLaneCount ? Vector256.Load(n1Pointer3 + 8) : Vector256<float>.AllBitsSet;
                        var n1B4 = 4 < nextLaneCount ? Vector256.Load(n1Pointer4 + 8) : Vector256<float>.AllBitsSet;
                        var n1B5 = 5 < nextLaneCount ? Vector256.Load(n1Pointer5 + 8) : Vector256<float>.AllBitsSet;
                        var n1B6 = 6 < nextLaneCount ? Vector256.Load(n1Pointer6 + 8) : Vector256<float>.AllBitsSet;
                        var n1B7 = 7 < nextLaneCount ? Vector256.Load(n1Pointer7 + 8) : Vector256<float>.AllBitsSet;

                        Transpose(n1B0, n1B1, n1B2, n1B3, n1B4, n1B5, n1B6, n1B7,
                            out var n1BMinX, out var n1BMinY, out var n1BMinZ, out var n1BIndex,
                            out var n1BMaxX, out var n1BMaxY, out var n1BMaxZ, out _);

                        var aaIntersects = Intersects(
                            n0AMinX, n0AMinY, n0AMinZ, n0AMaxX, n0AMaxY, n0AMaxZ,
                            n1AMinX, n1AMinY, n1AMinZ, n1AMaxX, n1AMaxY, n1AMaxZ);
                        var abIntersects = Intersects(
                            n0AMinX, n0AMinY, n0AMinZ, n0AMaxX, n0AMaxY, n0AMaxZ,
                            n1BMinX, n1BMinY, n1BMinZ, n1BMaxX, n1BMaxY, n1BMaxZ);
                        var baIntersects = Intersects(
                            n0BMinX, n0BMinY, n0BMinZ, n0BMaxX, n0BMaxY, n0BMaxZ,
                            n1AMinX, n1AMinY, n1AMinZ, n1AMaxX, n1AMaxY, n1AMaxZ);
                        var bbIntersects = Intersects(
                            n0BMinX, n0BMinY, n0BMinZ, n0BMaxX, n0BMaxY, n0BMaxZ,
                            n1BMinX, n1BMinY, n1BMinZ, n1BMaxX, n1BMaxY, n1BMaxZ);

                        var n0AIsInternal = Vector256.GreaterThanOrEqual(n0AIndex.As<float, int>(), Vector256<int>.Zero);
                        var n0BIsInternal = Vector256.GreaterThanOrEqual(n0BIndex.As<float, int>(), Vector256<int>.Zero);
                        var n1AIsInternal = Vector256.GreaterThanOrEqual(n1AIndex.As<float, int>(), Vector256<int>.Zero);
                        var n1BIsInternal = Vector256.GreaterThanOrEqual(n1BIndex.As<float, int>(), Vector256<int>.Zero);

                        var reportAA = Vector256.AndNot(Vector256.AndNot(aaIntersects, n0AIsInternal), n1AIsInternal);
                        var reportAB = Vector256.AndNot(Vector256.AndNot(abIntersects, n0AIsInternal), n1BIsInternal);
                        var reportBA = Vector256.AndNot(Vector256.AndNot(baIntersects, n0BIsInternal), n1AIsInternal);
                        var reportBB = Vector256.AndNot(Vector256.AndNot(bbIntersects, n0BIsInternal), n1BIsInternal);
                        var reportAnyPair = reportAA | reportAB | reportBA | reportBB;
                        if (Vector256.LessThanAny(reportAnyPair, Vector256<int>.Zero))
                        {
                            //At least one leaf-leaf test is reported.
                            //For each report vector, encode the indices, left pack for reported lanes, and store into the toReport buffers.
                            var aaBitMask = Vector256.ExtractMostSignificantBits(reportAA);
                            var reportCountAA = BitOperations.PopCount(aaBitMask);
                            var abBitMask = Vector256.ExtractMostSignificantBits(reportBA);
                            var reportCountAB = BitOperations.PopCount(abBitMask);
                            var baBitMask = Vector256.ExtractMostSignificantBits(reportBA);
                            var reportCountBA = BitOperations.PopCount(baBitMask);
                            var bbBitMask = Vector256.ExtractMostSignificantBits(reportBB);
                            var reportCountBB = BitOperations.PopCount(bbBitMask);

                            var reportCount = 0;
                            //Reporting itself is sequentialized; exposing the vectorized context to the callback is grossbad.
                            for (int i = 0; i < reportCount; ++i)
                            {
                                results.Handle(toReportA[i], toReportB[i]);
                            }
                        }

                        var pushNode0AVersusLeaf1A = aaIntersects & Vector256.AndNot(n0AIsInternal, n1AIsInternal);
                        var pushNode0AVersusLeaf1B = abIntersects & Vector256.AndNot(n0AIsInternal, n1BIsInternal);
                        var pushNode0BVersusLeaf1A = baIntersects & Vector256.AndNot(n0BIsInternal, n1AIsInternal);
                        var pushNode0BVersusLeaf1B = bbIntersects & Vector256.AndNot(n0BIsInternal, n1BIsInternal);
                        var pushAnyNode0VersusLeaf1 = pushNode0AVersusLeaf1A | pushNode0AVersusLeaf1B | pushNode0BVersusLeaf1A | pushNode0BVersusLeaf1B;

                        var pushLeaf0AVersusNode1A = aaIntersects & Vector256.AndNot(n1AIsInternal, n0AIsInternal);
                        var pushLeaf0AVersusNode1B = abIntersects & Vector256.AndNot(n1BIsInternal, n0AIsInternal);
                        var pushLeaf0BVersusNode1A = baIntersects & Vector256.AndNot(n1AIsInternal, n0BIsInternal);
                        var pushLeaf0BVersusNode1B = bbIntersects & Vector256.AndNot(n1BIsInternal, n0BIsInternal);
                        var pushAnyLeaf0VersusNode1 = pushLeaf0AVersusNode1A | pushLeaf0AVersusNode1B | pushLeaf0BVersusNode1A | pushLeaf0BVersusNode1B;

                        if (Vector256.LessThanAny(pushAnyNode0VersusLeaf1 | pushAnyLeaf0VersusNode1, Vector256<int>.Zero))
                        {
                            //At least one node-leaf push is needed.
                        }

                        var aaWantsToPushCrossover = aaIntersects & n0AIsInternal & n1AIsInternal;
                        var abWantsToPushCrossover = abIntersects & n0AIsInternal & n1BIsInternal;
                        var baWantsToPushCrossover = baIntersects & n0BIsInternal & n1AIsInternal;
                        var bbWantsToPushCrossover = bbIntersects & n0BIsInternal & n1BIsInternal;
                        var pushAnyCrossover = aaWantsToPushCrossover | abWantsToPushCrossover | baWantsToPushCrossover | bbWantsToPushCrossover;
                        if (Vector256.LessThanAny(pushAnyCrossover, Vector256<int>.Zero))
                        {
                            //At least one push for crossovers.
                            //Similar to leaf-leaf reporting; left pack the indices and store them into the push buffers.
                        }

                    }


                }
                //while (true)
                //{

                //    GetOverlapsBetweenDifferentNodes(ref Nodes[nextCrossover.A], ref Nodes[nextCrossover.B], ref results);
                //    //None of the candidates generated a next step, so grab from the stack.
                //    if (!crossoverStack.TryPop(out nextCrossover))
                //        break;



                //    ////Possible sources of stack entry:
                //    ////1) AA intersection, both internal
                //    ////2) AB intersection, both internal
                //    ////3) BA intersection, both internal
                //    ////4) BB intersection, both internal
                //    //var parent0 = nextCrossover.A;
                //    //var parent1 = nextCrossover.B;
                //    //ref var n0 = ref Nodes[parent0];
                //    //ref var n1 = ref Nodes[parent1];
                //    //var aaIntersects = BoundingBox.IntersectsUnsafe(n0.A, n1.A);
                //    //var abIntersects = BoundingBox.IntersectsUnsafe(n0.A, n1.B);
                //    //var baIntersects = BoundingBox.IntersectsUnsafe(n0.B, n1.A);
                //    //var bbIntersects = BoundingBox.IntersectsUnsafe(n0.B, n1.B);
                //    //var n0A = n0.A.Index;
                //    //var n0B = n0.B.Index;
                //    //var n1A = n1.A.Index;
                //    //var n1B = n1.B.Index;
                //    //var n0AIsInternal = n0A >= 0;
                //    //var n0BIsInternal = n0B >= 0;
                //    //var n1AIsInternal = n1A >= 0;
                //    //var n1BIsInternal = n1B >= 0;
                //    ////The first test which generates a stack candidate gets the nextTest; the rest get pushed to the stack.
                //    //int previousStackGeneratorCount = 0;
                //    //if (aaIntersects)
                //    //{
                //    //    if (n0AIsInternal && n1AIsInternal)
                //    //    {
                //    //        ++previousStackGeneratorCount;
                //    //        nextCrossover.A = n0A;
                //    //        nextCrossover.B = n1A;
                //    //    }
                //    //    else
                //    //    {
                //    //        //At least one is a leaf.
                //    //        if (n0AIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent1, false, n0A);
                //    //        else if (n1AIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent0, false, n1A);
                //    //        else //Both are leaves.
                //    //            results.Handle(Encode(n0A), Encode(n1A));
                //    //    }
                //    //}
                //    //if (abIntersects)
                //    //{
                //    //    if (n0AIsInternal && n1BIsInternal)
                //    //    {
                //    //        if (previousStackGeneratorCount++ == 0)
                //    //        {
                //    //            nextCrossover.A = n0A;
                //    //            nextCrossover.B = n1B;
                //    //        }
                //    //        else
                //    //        {
                //    //            ref var stackEntry = ref crossoverStack.AllocateUnsafely();
                //    //            stackEntry.A = n0A;
                //    //            stackEntry.B = n1B;
                //    //        }
                //    //    }
                //    //    else
                //    //    {
                //    //        //At least one is a leaf.
                //    //        if (n0AIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent1, true, n0A);
                //    //        else if (n1BIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent0, false, n1B);
                //    //        else //Both are leaves.
                //    //            results.Handle(Encode(n0A), Encode(n1B));
                //    //    }
                //    //}
                //    //if (baIntersects)
                //    //{
                //    //    if (n0BIsInternal && n1AIsInternal)
                //    //    {
                //    //        if (previousStackGeneratorCount++ == 0)
                //    //        {
                //    //            nextCrossover.A = n0B;
                //    //            nextCrossover.B = n1A;
                //    //        }
                //    //        else
                //    //        {
                //    //            ref var stackEntry = ref crossoverStack.AllocateUnsafely();
                //    //            stackEntry.A = n0B;
                //    //            stackEntry.B = n1A;
                //    //        }
                //    //    }
                //    //    else
                //    //    {
                //    //        //At least one is a leaf.
                //    //        if (n0BIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent1, false, n0B);
                //    //        else if (n1AIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent0, true, n1A);
                //    //        else //Both are leaves.
                //    //            results.Handle(Encode(n0B), Encode(n1A));
                //    //    }
                //    //}
                //    //if (bbIntersects)
                //    //{
                //    //    if (n0BIsInternal && n1BIsInternal)
                //    //    {
                //    //        if (previousStackGeneratorCount++ == 0)
                //    //        {
                //    //            nextCrossover.A = n0B;
                //    //            nextCrossover.B = n1B;
                //    //        }
                //    //        else
                //    //        {
                //    //            ref var stackEntry = ref crossoverStack.AllocateUnsafely();
                //    //            stackEntry.A = n0B;
                //    //            stackEntry.B = n1B;
                //    //        }
                //    //    }
                //    //    else
                //    //    {
                //    //        //At least one is a leaf.
                //    //        if (n0BIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent1, true, n0B);
                //    //        else if (n1BIsInternal)
                //    //            leafNodeStack.AllocateUnsafely() = new LeafNodeStackEntry(parent0, true, n1B);
                //    //        else //Both are leaves.
                //    //            results.Handle(Encode(n0B), Encode(n1B));
                //    //    }
                //    //}
                //    //if (previousStackGeneratorCount == 0)
                //    //{
                //    //    //None of the candidates generated a next step, so grab from the stack.
                //    //    if (!crossoverStack.TryPop(out nextCrossover))
                //    //        break;
                //    //}
                //}
            }
            pool.Return(ref crossoverStackA);
            pool.Return(ref crossoverStackB);
            QuickList<int> stack = new(NodeCount, pool);

            for (int i = 0; i < leafNodeStack.Count; ++i)
            {
                var leafNodeTarget = leafNodeStack[i];
                ref var leafChild = ref leafNodeTarget.GetChild(ref this);
                var leafIndex = Encode(leafChild.Index);
                var nodeToTest = leafNodeTarget.NodeIndex;
                Debug.Assert(stack.Count == 0);
                while (true)
                {
                    //Now process all the leaf nodes!
                    ref var node = ref Nodes[nodeToTest];
                    var a = node.A.Index;
                    var b = node.B.Index;
                    var aIntersected = BoundingBox.IntersectsUnsafe(leafChild, node.A);
                    var bIntersected = BoundingBox.IntersectsUnsafe(leafChild, node.B);
                    var aIsInternal = a >= 0;
                    var bIsInternal = b >= 0;
                    var intersectedInternalA = aIntersected && aIsInternal;
                    var intersectedInternalB = bIntersected && bIsInternal;
                    if (intersectedInternalA && intersectedInternalB)
                    {
                        nodeToTest = a;
                        stack.AllocateUnsafely() = b;
                    }
                    else
                    {
                        if (aIntersected && !aIsInternal)
                        {
                            results.Handle(leafIndex, Encode(a));
                        }
                        if (bIntersected && !bIsInternal)
                        {
                            results.Handle(leafIndex, Encode(b));
                        }
                        if (intersectedInternalA || intersectedInternalB)
                        {
                            nodeToTest = intersectedInternalA ? a : b;
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
                }
            }
            leafNodeStack.Dispose(pool);
            stack.Dispose(pool);
        }




    }
}
