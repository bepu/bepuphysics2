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
            var encodedLeftPackMask = Unsafe.Add(ref Unsafe.AsRef(lookupTable[0]), bitmask);

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

        public unsafe void GetSelfOverlapsContiguousPrepass<TOverlapHandler>(ref TOverlapHandler results, BufferPool pool) where TOverlapHandler : IOverlapHandler
        {
            //If there are less than two leaves, there can't be any overlap.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (LeafCount < 2)
                return;

            //A recursive self test will at some point visit all nodes with certainty. Instead of framing it as a recursive test at all, do a prepass that's just a contiguous iteration.
            //Include a little buffer to avoid overruns by vectorized operations.
            var stackSize = ((NodeCount + 7) / 8 + 1) * 8;
            pool.Take<int>(stackSize, out var crossoverStackA);
            pool.Take<int>(stackSize, out var crossoverStackB);
            pool.Take<uint>(stackSize, out var nodeLeafStackA);
            pool.Take<uint>(stackSize, out var nodeLeafStackB);
            int crossoverStackCount = 0;
            int nodeLeafStackCount = 0;
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
                    nodeLeafStackA[nodeLeafStackCount] = (uint)i;
                    nodeLeafStackB[nodeLeafStackCount] = (uint)i | (1u << 31);
                    ++nodeLeafStackCount;
                }
                else
                {
                    //Both are leaves.
                    results.Handle(Encode(a), Encode(b));
                }
            }

            //Now, we need to complete all crossovers and leaf-node tests. Note that leaf-node tests can only generate leaf-node or leaf-leaf tests, they never produce crossovers.
            //In contrast, crossovers can generate leaf-node tests. So do crossovers first so we'll have every leaf-node test in the stack ready to go.
            while (crossoverStackCount > 0)
            {
                var toVisitIndex = --crossoverStackCount;
                var index0 = crossoverStackA[toVisitIndex];
                var index1 = crossoverStackB[toVisitIndex];
                ref var n0 = ref Nodes[index0];
                ref var n1 = ref Nodes[index1];

                var aaIntersects = BoundingBox.IntersectsUnsafe(n0.A, n1.A);
                var abIntersects = BoundingBox.IntersectsUnsafe(n0.A, n1.B);
                var baIntersects = BoundingBox.IntersectsUnsafe(n0.B, n1.A);
                var bbIntersects = BoundingBox.IntersectsUnsafe(n0.B, n1.B);

                var intersects = Vector128.Create(aaIntersects ? -1 : 0, abIntersects ? -1 : 0, baIntersects ? -1 : 0, bbIntersects ? -1 : 0);
                var indices = Vector128.Create(n0.A.Index, n0.B.Index, n1.A.Index, n1.B.Index);
                var n0Indices = Vector128.Shuffle(indices, Vector128.Create(0, 0, 1, 1));
                var n1Indices = Vector128.Shuffle(indices, Vector128.Create(2, 3, 2, 3));
                var n0Internal = Vector128.GreaterThan(n0Indices, Vector128<int>.Zero);
                var n1Internal = Vector128.GreaterThan(n1Indices, Vector128<int>.Zero);

                var leafLeaf = Vector128.AndNot(Vector128.AndNot(intersects, n0Internal), n1Internal);
                var nodeLeaf = intersects & (n0Internal ^ n1Internal);
                var crossover = intersects & n0Internal & n1Internal;

                LeftPack(leafLeaf, Encode(n0Indices), Encode(n1Indices), out var leafleafToPush0, out var leafleafToPush1, out var leafLeafCount);
                var parentEncoded0 = Vector128.BitwiseOr(Vector128.Create(index0), Vector128.Create(0, 0, 1 << 31, 1 << 31));
                var parentEncoded1 = Vector128.BitwiseOr(Vector128.Create(index1), Vector128.Create(0, 1 << 31, 0, 1 << 31));
                LeftPack(nodeLeaf, parentEncoded0, parentEncoded1, out var nodeLeafToPush0, out var nodeLeafToPush1, out var nodeLeafCount);
                LeftPack(crossover, n0Indices, n1Indices, out var crossoverToPush0, out var crossoverToPush1, out var crossoverCount);

                if (leafLeafCount > 0)
                {
                    results.Handle(leafleafToPush0[0], leafleafToPush1[0]);
                    if (leafLeafCount > 1) results.Handle(leafleafToPush0[1], leafleafToPush1[1]);
                    if (leafLeafCount > 2) results.Handle(leafleafToPush0[2], leafleafToPush1[2]);
                    if (leafLeafCount > 3) results.Handle(leafleafToPush0[3], leafleafToPush1[3]);
                }
                if (nodeLeafCount > 0)
                {
                    Vector128.Store(nodeLeafToPush0.AsUInt32(), nodeLeafStackA.Memory + nodeLeafStackCount);
                    Vector128.Store(nodeLeafToPush1.AsUInt32(), nodeLeafStackB.Memory + nodeLeafStackCount);
                    nodeLeafStackCount += nodeLeafCount;
                }
                if (crossoverCount > 0)
                {
                    Vector128.Store(crossoverToPush0, crossoverStackA.Memory + crossoverStackCount);
                    Vector128.Store(crossoverToPush1, crossoverStackB.Memory + crossoverStackCount);
                    crossoverStackCount += crossoverCount;
                }

                //Console.WriteLine($"new stackcounts: nodeleaf {nodeLeafStackCount}, crossover {crossoverStackCount}; new leafleaf {leafleafCount}, nodeleaf {nodeLeafCount}, crossover {crossoverCount}");
            }
            //Console.WriteLine("End crossovers");

            pool.Return(ref crossoverStackA);
            pool.Return(ref crossoverStackB);
            QuickList<int> stack = new(NodeCount, pool);

            for (int i = 0; i < nodeLeafStackCount; ++i)
            {
                ref var childA = ref GetLeafChild(ref this, nodeLeafStackA[i]);
                ref var childB = ref GetLeafChild(ref this, nodeLeafStackB[i]);
                Debug.Assert((childA.Index < 0) ^ (childB.Index < 0), "One and only one of the two children must be a leaf.");
                ref var leafChild = ref childA.Index < 0 ? ref childA : ref childB;
                stack.AllocateUnsafely() = childA.Index < 0 ? childB.Index : childA.Index;
                var leafIndex = Encode(leafChild.Index);
                Debug.Assert(stack.Count == 0);
                while (stack.TryPop(out var nodeToTest))
                {
                    ref var node = ref Nodes[nodeToTest];
                    var a = node.A.Index;
                    var b = node.B.Index;
                    var aIntersected = BoundingBox.IntersectsUnsafe(leafChild, node.A);
                    var bIntersected = BoundingBox.IntersectsUnsafe(leafChild, node.B);

                    if (bIntersected)
                    {
                        if (b >= 0)
                            stack.AllocateUnsafely() = b;
                        else
                            results.Handle(leafIndex, Encode(b));
                    }
                    if (aIntersected)
                    {
                        if (a >= 0)
                            stack.AllocateUnsafely() = a;
                        else
                            results.Handle(leafIndex, Encode(a));
                    }

                }
            }
            pool.Return(ref nodeLeafStackA);
            pool.Return(ref nodeLeafStackB);
            stack.Dispose(pool);
        }




    }
}
