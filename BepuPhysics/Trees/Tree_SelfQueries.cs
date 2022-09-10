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
        static ref NodeChild GetLeafChild(ref Tree tree, uint encodedLeafParentIndex)
        {
            var parentNodeIndex = encodedLeafParentIndex & 0x7FFF_FFFF;
            var leafIsChildB = encodedLeafParentIndex > 0x7FFF_FFFF;
            ref var parent = ref tree.Nodes[parentNodeIndex];
            return ref leafIsChildB ? ref parent.B : ref parent.A;
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
                Vector256.GreaterThanOrEqual(maxBX, minAX) & Vector256.GreaterThanOrEqual(maxBY, minAY) & Vector256.GreaterThanOrEqual(maxBZ, minAZ)).AsInt32();
        }

        //public static void CreateLeftPackLookupTable8()
        //{
        //    var lookupTable = new byte[768 + 1]; //last write will be the last 3 bytes plus one, so add a little buffer.
        //    for (int i = 0; i < 256; ++i)
        //    {
        //        var accumulator = 0;
        //        int index = 0;
        //        for (int j = 0; j < 8; ++j)
        //        {
        //            if ((i & (1 << j)) != 0)
        //            {
        //                accumulator |= j << (index * 3);
        //                ++index;
        //            }
        //        }
        //        Unsafe.As<byte, int>(ref lookupTable[i * 3]) = accumulator;
        //    }
        //    Console.WriteLine("new byte[] { ");
        //    for (int i = 0; i < 768; ++i)
        //    {
        //        Console.Write($"{lookupTable[i]}, ");
        //        if (i % 32 == 31)
        //            Console.WriteLine();
        //    }
        //    Console.WriteLine("}");

        //}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector256<int> GetLeftPackMask(Vector256<int> mask, out int count)
        {
            if (!Avx2.IsSupported || !Bmi2.X64.IsSupported) throw new NotSupportedException("No fallback exists! This should never be visible!");

            var bitmask = Vector256.ExtractMostSignificantBits(mask);
            //This depends upon an optimization that preallocates the constant array in fixed memory. Bit of a turbohack that won't work on other runtimes.
            //The lookup table includes one entry for each of the 256 possible bitmasks. Each lane requires 3 bits to define the source for a shuffle mask.
            //3 bits, 8 lanes, 256 bitmasks means only 768 bytes.
            ReadOnlySpan<byte> lookupTable = new byte[] {
                0, 0, 0, 0, 0, 0, 1, 0, 0, 8, 0, 0, 2, 0, 0, 16, 0, 0, 17, 0, 0, 136, 0, 0, 3, 0, 0, 24, 0, 0, 25, 0,
                0, 200, 0, 0, 26, 0, 0, 208, 0, 0, 209, 0, 0, 136, 6, 0, 4, 0, 0, 32, 0, 0, 33, 0, 0, 8, 1, 0, 34, 0, 0, 16,
                1, 0, 17, 1, 0, 136, 8, 0, 35, 0, 0, 24, 1, 0, 25, 1, 0, 200, 8, 0, 26, 1, 0, 208, 8, 0, 209, 8, 0, 136, 70, 0,
                5, 0, 0, 40, 0, 0, 41, 0, 0, 72, 1, 0, 42, 0, 0, 80, 1, 0, 81, 1, 0, 136, 10, 0, 43, 0, 0, 88, 1, 0, 89, 1,
                0, 200, 10, 0, 90, 1, 0, 208, 10, 0, 209, 10, 0, 136, 86, 0, 44, 0, 0, 96, 1, 0, 97, 1, 0, 8, 11, 0, 98, 1, 0, 16,
                11, 0, 17, 11, 0, 136, 88, 0, 99, 1, 0, 24, 11, 0, 25, 11, 0, 200, 88, 0, 26, 11, 0, 208, 88, 0, 209, 88, 0, 136, 198, 2,
                6, 0, 0, 48, 0, 0, 49, 0, 0, 136, 1, 0, 50, 0, 0, 144, 1, 0, 145, 1, 0, 136, 12, 0, 51, 0, 0, 152, 1, 0, 153, 1,
                0, 200, 12, 0, 154, 1, 0, 208, 12, 0, 209, 12, 0, 136, 102, 0, 52, 0, 0, 160, 1, 0, 161, 1, 0, 8, 13, 0, 162, 1, 0, 16,
                13, 0, 17, 13, 0, 136, 104, 0, 163, 1, 0, 24, 13, 0, 25, 13, 0, 200, 104, 0, 26, 13, 0, 208, 104, 0, 209, 104, 0, 136, 70, 3,
                53, 0, 0, 168, 1, 0, 169, 1, 0, 72, 13, 0, 170, 1, 0, 80, 13, 0, 81, 13, 0, 136, 106, 0, 171, 1, 0, 88, 13, 0, 89, 13,
                0, 200, 106, 0, 90, 13, 0, 208, 106, 0, 209, 106, 0, 136, 86, 3, 172, 1, 0, 96, 13, 0, 97, 13, 0, 8, 107, 0, 98, 13, 0, 16,
                107, 0, 17, 107, 0, 136, 88, 3, 99, 13, 0, 24, 107, 0, 25, 107, 0, 200, 88, 3, 26, 107, 0, 208, 88, 3, 209, 88, 3, 136, 198, 26,
                7, 0, 0, 56, 0, 0, 57, 0, 0, 200, 1, 0, 58, 0, 0, 208, 1, 0, 209, 1, 0, 136, 14, 0, 59, 0, 0, 216, 1, 0, 217, 1,
                0, 200, 14, 0, 218, 1, 0, 208, 14, 0, 209, 14, 0, 136, 118, 0, 60, 0, 0, 224, 1, 0, 225, 1, 0, 8, 15, 0, 226, 1, 0, 16,
                15, 0, 17, 15, 0, 136, 120, 0, 227, 1, 0, 24, 15, 0, 25, 15, 0, 200, 120, 0, 26, 15, 0, 208, 120, 0, 209, 120, 0, 136, 198, 3,
                61, 0, 0, 232, 1, 0, 233, 1, 0, 72, 15, 0, 234, 1, 0, 80, 15, 0, 81, 15, 0, 136, 122, 0, 235, 1, 0, 88, 15, 0, 89, 15,
                0, 200, 122, 0, 90, 15, 0, 208, 122, 0, 209, 122, 0, 136, 214, 3, 236, 1, 0, 96, 15, 0, 97, 15, 0, 8, 123, 0, 98, 15, 0, 16,
                123, 0, 17, 123, 0, 136, 216, 3, 99, 15, 0, 24, 123, 0, 25, 123, 0, 200, 216, 3, 26, 123, 0, 208, 216, 3, 209, 216, 3, 136, 198, 30,
                62, 0, 0, 240, 1, 0, 241, 1, 0, 136, 15, 0, 242, 1, 0, 144, 15, 0, 145, 15, 0, 136, 124, 0, 243, 1, 0, 152, 15, 0, 153, 15,
                0, 200, 124, 0, 154, 15, 0, 208, 124, 0, 209, 124, 0, 136, 230, 3, 244, 1, 0, 160, 15, 0, 161, 15, 0, 8, 125, 0, 162, 15, 0, 16,
                125, 0, 17, 125, 0, 136, 232, 3, 163, 15, 0, 24, 125, 0, 25, 125, 0, 200, 232, 3, 26, 125, 0, 208, 232, 3, 209, 232, 3, 136, 70, 31,
                245, 1, 0, 168, 15, 0, 169, 15, 0, 72, 125, 0, 170, 15, 0, 80, 125, 0, 81, 125, 0, 136, 234, 3, 171, 15, 0, 88, 125, 0, 89, 125,
                0, 200, 234, 3, 90, 125, 0, 208, 234, 3, 209, 234, 3, 136, 86, 31, 172, 15, 0, 96, 125, 0, 97, 125, 0, 8, 235, 3, 98, 125, 0, 16,
                235, 3, 17, 235, 3, 136, 88, 31, 99, 125, 0, 24, 235, 3, 25, 235, 3, 200, 88, 31, 26, 235, 3, 208, 88, 31, 209, 88, 31, 136, 198, 250 };
            var encodedLeftPackMask = Unsafe.As<byte, int>(ref Unsafe.Add(ref Unsafe.AsRef(lookupTable[0]), bitmask * 3));

            count = BitOperations.PopCount(bitmask);
            //Broadcast, variable shift.
            return Avx2.ShiftRightLogicalVariable(Vector256.Create(encodedLeftPackMask), Vector256.Create(0u, 3, 6, 9, 12, 15, 18, 21));


            //var bitmask = Vector256.ExtractMostSignificantBits(mask);
            ////From https://stackoverflow.com/a/36951611, courtesy of Peter Cordes.
            ////pdep/pext are apparently quite slow pre-Zen3, unfortunately. This is just a proof of correctness.
            //ulong expanded_mask = Bmi2.X64.ParallelBitDeposit(bitmask, 0x0101010101010101);  // unpack each bit to a byte
            //expanded_mask *= 0xFF;    // mask |= mask<<1 | mask<<2 | ... | mask<<7;
            //                          // ABC... -> AAAAAAAABBBBBBBBCCCCCCCC...: replicate each bit to fill its byte

            //ulong identity_indices = 0x0706050403020100;    // the identity shuffle for vpermps, packed to one index per byte
            //ulong wanted_indices = Bmi2.X64.ParallelBitExtract(identity_indices, expanded_mask);

            //count = BitOperations.PopCount(bitmask);
            //return Avx2.ConvertToVector256Int32(Vector128.CreateScalarUnsafe(wanted_indices).AsByte());

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector256<int> Encode(Vector256<int> index)
        {
            return Vector256<int>.AllBitsSet - index;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector256<int> Encode(Vector256<float> index)
        {
            return Vector256<int>.AllBitsSet - index.AsInt32();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector256<int> EncodeLeafChildBForStack(Vector256<int> parentIndex)
        {
            return parentIndex | Vector256.Create(1 << 31);
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
            if (crossoverStackCount > 0)
            {
                if (Vector256.IsHardwareAccelerated)
                {
                    //In a given iteration, the maximum number of leaf-leaf intersections is 8 lanes * 4 child combinations per lane.
                    var toReportA = stackalloc int[32];
                    var toReportB = stackalloc int[32];
                    var shouldReport = stackalloc int[32];

                    while (crossoverStackCount > 0)
                    {
                        //Console.WriteLine($"START iteration, crossoverStack: {crossoverStackCount}, nodeLeafStack: {nodeLeafStackCount}");
                        //Note that we do not have a 'nextCrossover' entry held over from the previous iteration. Instead, we directly yoink off the stack. Serves the same purpose.
                        //Pick the appropriate location to load from in the stack.
                        var nextLaneCount = 8 < crossoverStackCount ? 8 : crossoverStackCount;
                        var startIndex = crossoverStackCount - nextLaneCount;
                        var nextCrossover0 = crossoverStackA.Memory + startIndex;
                        var nextCrossover1 = crossoverStackB.Memory + startIndex;
                        crossoverStackCount = startIndex;

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

                        var aaIntersects = Intersects(
                            n0AMinX, n0AMinY, n0AMinZ, n0AMaxX, n0AMaxY, n0AMaxZ,
                            n1AMinX, n1AMinY, n1AMinZ, n1AMaxX, n1AMaxY, n1AMaxZ);

                        var n0B0 = Vector256.Load(n0Pointer0 + 8);
                        var n0B1 = 1 < nextLaneCount ? Vector256.Load(n0Pointer1 + 8) : Vector256<float>.AllBitsSet;
                        var n0B2 = 2 < nextLaneCount ? Vector256.Load(n0Pointer2 + 8) : Vector256<float>.AllBitsSet;
                        var n0B3 = 3 < nextLaneCount ? Vector256.Load(n0Pointer3 + 8) : Vector256<float>.AllBitsSet;
                        var n0B4 = 4 < nextLaneCount ? Vector256.Load(n0Pointer4 + 8) : Vector256<float>.AllBitsSet;
                        var n0B5 = 5 < nextLaneCount ? Vector256.Load(n0Pointer5 + 8) : Vector256<float>.AllBitsSet;
                        var n0B6 = 6 < nextLaneCount ? Vector256.Load(n0Pointer6 + 8) : Vector256<float>.AllBitsSet;
                        var n0B7 = 7 < nextLaneCount ? Vector256.Load(n0Pointer7 + 8) : Vector256<float>.AllBitsSet;

                        Transpose(n0B0, n0B1, n0B2, n0B3, n0B4, n0B5, n0B6, n0B7,
                            out var n0BMinX, out var n0BMinY, out var n0BMinZ, out var n0BIndex,
                            out var n0BMaxX, out var n0BMaxY, out var n0BMaxZ, out _);

                        var baIntersects = Intersects(
                            n0BMinX, n0BMinY, n0BMinZ, n0BMaxX, n0BMaxY, n0BMaxZ,
                            n1AMinX, n1AMinY, n1AMinZ, n1AMaxX, n1AMaxY, n1AMaxZ);

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

                        var abIntersects = Intersects(
                            n0AMinX, n0AMinY, n0AMinZ, n0AMaxX, n0AMaxY, n0AMaxZ,
                            n1BMinX, n1BMinY, n1BMinZ, n1BMaxX, n1BMaxY, n1BMaxZ);
                        var bbIntersects = Intersects(
                            n0BMinX, n0BMinY, n0BMinZ, n0BMaxX, n0BMaxY, n0BMaxZ,
                            n1BMinX, n1BMinY, n1BMinZ, n1BMaxX, n1BMaxY, n1BMaxZ);

                        var n0AIsInternal = Vector256.GreaterThanOrEqual(n0AIndex.AsInt32(), Vector256<int>.Zero);
                        var n0BIsInternal = Vector256.GreaterThanOrEqual(n0BIndex.AsInt32(), Vector256<int>.Zero);
                        var n1AIsInternal = Vector256.GreaterThanOrEqual(n1AIndex.AsInt32(), Vector256<int>.Zero);
                        var n1BIsInternal = Vector256.GreaterThanOrEqual(n1BIndex.AsInt32(), Vector256<int>.Zero);

                        var reportAA = Vector256.AndNot(Vector256.AndNot(aaIntersects, n0AIsInternal), n1AIsInternal);
                        var reportAB = Vector256.AndNot(Vector256.AndNot(abIntersects, n0AIsInternal), n1BIsInternal);
                        var reportBA = Vector256.AndNot(Vector256.AndNot(baIntersects, n0BIsInternal), n1AIsInternal);
                        var reportBB = Vector256.AndNot(Vector256.AndNot(bbIntersects, n0BIsInternal), n1BIsInternal);
                        var reportAnyPair = reportAA | reportAB | reportBA | reportBB;
                        if (Vector256.LessThanAny(reportAnyPair, Vector256<int>.Zero))
                        {
                            //At least one leaf-leaf test is reported.
                            //For each report vector, encode the indices, left pack for reported lanes, and store into the toReport buffers.
                            var aaShuffle = GetLeftPackMask(reportAA, out int aaCount);
                            var abShuffle = GetLeftPackMask(reportAB, out int abCount);
                            var baShuffle = GetLeftPackMask(reportBA, out int baCount);
                            var bbShuffle = GetLeftPackMask(reportBB, out int bbCount);

                            var encodedN0A = Encode(n0AIndex);
                            var encodedN0B = Encode(n0BIndex);
                            var encodedN1A = Encode(n1AIndex);
                            var encodedN1B = Encode(n1BIndex);
                            var reportCount = 0;
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN0A, aaShuffle), toReportA);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN1A, aaShuffle), toReportB);
                            reportCount += aaCount;
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN0A, abShuffle), toReportA + reportCount);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN1B, abShuffle), toReportB + reportCount);
                            reportCount += abCount;
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN0B, baShuffle), toReportA + reportCount);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN1A, baShuffle), toReportB + reportCount);
                            reportCount += baCount;
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN0B, bbShuffle), toReportA + reportCount);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedN1B, bbShuffle), toReportB + reportCount);
                            reportCount += bbCount;

                            //Reporting itself is sequentialized; exposing the vectorized context to the callback is grossbad.
                            for (int i = 0; i < reportCount; ++i)
                            {
                                //Console.WriteLine($"reporting: {toReportA[i]}, {toReportB[i]}");
                                results.Handle(toReportA[i], toReportB[i]);
                            }
                            //Console.WriteLine($"total leaf-leaf iteration: {reportCount}");
                        }

                        var pushNodeLeaf0AVersus1A = aaIntersects & (n0AIsInternal ^ n1AIsInternal);
                        var pushNodeLeaf0AVersus1B = abIntersects & (n0AIsInternal ^ n1BIsInternal);
                        var pushNodeLeaf0BVersus1A = baIntersects & (n0BIsInternal ^ n1AIsInternal);
                        var pushNodeLeaf0BVersus1B = bbIntersects & (n0BIsInternal ^ n1BIsInternal);
                        var pushAnyNodeLeaf = pushNodeLeaf0AVersus1A | pushNodeLeaf0AVersus1B | pushNodeLeaf0BVersus1A | pushNodeLeaf0BVersus1B;

                        if (Vector256.LessThanAny(pushAnyNodeLeaf, Vector256<int>.Zero))
                        {
                            //At least one node-leaf push is needed.
                            var shuffle0A1A = GetLeftPackMask(pushNodeLeaf0AVersus1A, out int count0A1A);
                            var shuffle0A1B = GetLeftPackMask(pushNodeLeaf0AVersus1B, out int count0A1B);
                            var shuffle0B1A = GetLeftPackMask(pushNodeLeaf0BVersus1A, out int count0B1A);
                            var shuffle0B1B = GetLeftPackMask(pushNodeLeaf0BVersus1B, out int count0B1B);

                            var encodedForStack0A = Vector256.Load(nextCrossover0);
                            var encodedForStack0B = EncodeLeafChildBForStack(encodedForStack0A);
                            var encodedForStack1A = Vector256.Load(nextCrossover1);
                            var encodedForStack1B = EncodeLeafChildBForStack(encodedForStack1A);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack0A, shuffle0A1A), (int*)nodeLeafStackA.Memory + nodeLeafStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack1A, shuffle0A1A), (int*)nodeLeafStackB.Memory + nodeLeafStackCount);
                            nodeLeafStackCount += count0A1A;
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack0A, shuffle0A1B), (int*)nodeLeafStackA.Memory + nodeLeafStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack1B, shuffle0A1B), (int*)nodeLeafStackB.Memory + nodeLeafStackCount);
                            nodeLeafStackCount += count0A1B;
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack0B, shuffle0B1A), (int*)nodeLeafStackA.Memory + nodeLeafStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack1A, shuffle0B1A), (int*)nodeLeafStackB.Memory + nodeLeafStackCount);
                            nodeLeafStackCount += count0B1A;
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack0B, shuffle0B1B), (int*)nodeLeafStackA.Memory + nodeLeafStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(encodedForStack1B, shuffle0B1B), (int*)nodeLeafStackB.Memory + nodeLeafStackCount);
                            nodeLeafStackCount += count0B1B;
                        }

                        var aaWantsToPushCrossover = aaIntersects & n0AIsInternal & n1AIsInternal;
                        var abWantsToPushCrossover = abIntersects & n0AIsInternal & n1BIsInternal;
                        var baWantsToPushCrossover = baIntersects & n0BIsInternal & n1AIsInternal;
                        var bbWantsToPushCrossover = bbIntersects & n0BIsInternal & n1BIsInternal;
                        var pushAnyCrossover = aaWantsToPushCrossover | abWantsToPushCrossover | baWantsToPushCrossover | bbWantsToPushCrossover;
                        if (Vector256.LessThanAny(pushAnyCrossover, Vector256<int>.Zero))
                        {
                            //At least one push for crossovers.
                            var aaShuffle = GetLeftPackMask(aaWantsToPushCrossover, out int aaCount);
                            var abShuffle = GetLeftPackMask(abWantsToPushCrossover, out int abCount);
                            var baShuffle = GetLeftPackMask(baWantsToPushCrossover, out int baCount);
                            var bbShuffle = GetLeftPackMask(bbWantsToPushCrossover, out int bbCount);

                            Vector256.Store(Avx2.PermuteVar8x32(n0AIndex.AsInt32(), aaShuffle), crossoverStackA.Memory + crossoverStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(n1AIndex.AsInt32(), aaShuffle), crossoverStackB.Memory + crossoverStackCount);
                            crossoverStackCount += aaCount;
                            Vector256.Store(Avx2.PermuteVar8x32(n0AIndex.AsInt32(), abShuffle), crossoverStackA.Memory + crossoverStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(n1BIndex.AsInt32(), abShuffle), crossoverStackB.Memory + crossoverStackCount);
                            crossoverStackCount += abCount;
                            Vector256.Store(Avx2.PermuteVar8x32(n0BIndex.AsInt32(), baShuffle), crossoverStackA.Memory + crossoverStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(n1AIndex.AsInt32(), baShuffle), crossoverStackB.Memory + crossoverStackCount);
                            crossoverStackCount += baCount;
                            Vector256.Store(Avx2.PermuteVar8x32(n0BIndex.AsInt32(), bbShuffle), crossoverStackA.Memory + crossoverStackCount);
                            Vector256.Store(Avx2.PermuteVar8x32(n1BIndex.AsInt32(), bbShuffle), crossoverStackB.Memory + crossoverStackCount);
                            crossoverStackCount += bbCount;

                            //Console.WriteLine($"crossover count for iteration: {aaCount + abCount + baCount + bbCount}, new stack count {crossoverStackCount}");
                        }
                    }
                }
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
                var nodeToTest = childA.Index < 0 ? childB.Index : childA.Index;
                var leafIndex = Encode(leafChild.Index);
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
            pool.Return(ref nodeLeafStackA);
            pool.Return(ref nodeLeafStackB);
            stack.Dispose(pool);
        }




    }
}
