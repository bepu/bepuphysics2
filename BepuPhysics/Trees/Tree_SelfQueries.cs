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

                var n0AIndex = n0.A.Index;
                var n0BIndex = n0.B.Index;
                var n1AIndex = n1.A.Index;
                var n1BIndex = n1.B.Index;
                var n0AInternal = n0AIndex >= 0;
                var n0BInternal = n0BIndex >= 0;
                var n1AInternal = n1AIndex >= 0;
                var n1BInternal = n1BIndex >= 0;

                var aaLeafLeaf = aaIntersects & !(n0AInternal | n1AInternal);
                var abLeafLeaf = abIntersects & !(n0AInternal | n1BInternal);
                var baLeafLeaf = baIntersects & !(n0BInternal | n1AInternal);
                var bbLeafLeaf = bbIntersects & !(n0BInternal | n1BInternal);

                var aaNodeLeaf = aaIntersects & (n0AInternal ^ n1AInternal);
                var abNodeLeaf = abIntersects & (n0AInternal ^ n1BInternal);
                var baNodeLeaf = baIntersects & (n0BInternal ^ n1AInternal);
                var bbNodeLeaf = bbIntersects & (n0BInternal ^ n1BInternal);

                var aaPushCrossover = aaIntersects & n0AInternal & n1AInternal;
                var abPushCrossover = abIntersects & n0AInternal & n1BInternal;
                var baPushCrossover = baIntersects & n0BInternal & n1AInternal;
                var bbPushCrossover = bbIntersects & n0BInternal & n1BInternal;

                if (aaLeafLeaf) results.Handle(Encode(n0AIndex), Encode(n1AIndex));
                if (abLeafLeaf) results.Handle(Encode(n0AIndex), Encode(n1BIndex));
                if (baLeafLeaf) results.Handle(Encode(n0BIndex), Encode(n1AIndex));
                if (bbLeafLeaf) results.Handle(Encode(n0BIndex), Encode(n1BIndex));

                if (aaNodeLeaf) { nodeLeafStackA[nodeLeafStackCount] = (uint)index0; nodeLeafStackB[nodeLeafStackCount] = (uint)index1; ++nodeLeafStackCount; }
                if (abNodeLeaf) { nodeLeafStackA[nodeLeafStackCount] = (uint)index0; nodeLeafStackB[nodeLeafStackCount] = (uint)index1 | (1u << 31); ++nodeLeafStackCount; }
                if (baNodeLeaf) { nodeLeafStackA[nodeLeafStackCount] = (uint)index0 | (1u << 31); nodeLeafStackB[nodeLeafStackCount] = (uint)index1; ++nodeLeafStackCount; }
                if (bbNodeLeaf) { nodeLeafStackA[nodeLeafStackCount] = (uint)index0 | (1u << 31); nodeLeafStackB[nodeLeafStackCount] = (uint)index1 | (1u << 31); ++nodeLeafStackCount; }

                if (aaPushCrossover) { crossoverStackA[crossoverStackCount] = n0AIndex; crossoverStackB[crossoverStackCount] = n1AIndex; ++crossoverStackCount; }
                if (abPushCrossover) { crossoverStackA[crossoverStackCount] = n0AIndex; crossoverStackB[crossoverStackCount] = n1BIndex; ++crossoverStackCount; }
                if (baPushCrossover) { crossoverStackA[crossoverStackCount] = n0BIndex; crossoverStackB[crossoverStackCount] = n1AIndex; ++crossoverStackCount; }
                if (bbPushCrossover) { crossoverStackA[crossoverStackCount] = n0BIndex; crossoverStackB[crossoverStackCount] = n1BIndex; ++crossoverStackCount; }

                //var leafleafCount = (aaLeafLeaf ? 1 : 0) + (abLeafLeaf ? 1 : 0) + (baLeafLeaf ? 1 : 0) + (bbLeafLeaf ? 1 : 0);
                //var nodeLeafCount = (aaNodeLeaf ? 1 : 0) + (abNodeLeaf ? 1 : 0) + (baNodeLeaf ? 1 : 0) + (bbNodeLeaf ? 1 : 0);
                //var crossoverCount = (aaPushCrossover ? 1 : 0) + (abPushCrossover ? 1 : 0) + (baPushCrossover ? 1 : 0) + (bbPushCrossover ? 1 : 0);
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
