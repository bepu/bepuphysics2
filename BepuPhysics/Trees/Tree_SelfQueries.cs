using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

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
        unsafe void DispatchTestForLeaf<TOverlapHandler>(int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (nodeIndex < 0)
            {
                results.Handle(leafIndex, Encode(nodeIndex));
            }
            else
            {
                TestLeafAgainstNode(leafIndex, ref leafMin, ref leafMax, nodeIndex, ref results);
            }
        }
        unsafe void TestLeafAgainstNode<TOverlapHandler>(int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, ref TOverlapHandler results)
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
            var aIntersects = BoundingBox.Intersects(leafMin, leafMax, a.Min, a.Max);
            var bIntersects = BoundingBox.Intersects(leafMin, leafMax, b.Min, b.Max);
            if (aIntersects)
            {
                DispatchTestForLeaf(leafIndex, ref leafMin, ref leafMax, a.Index, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForLeaf(leafIndex, ref leafMin, ref leafMax, bIndex, ref results);
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
                    TestLeafAgainstNode(Encode(b.Index), ref b.Min, ref b.Max, a.Index, ref results);
                }
            }
            else if (b.Index >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode(Encode(a.Index), ref a.Min, ref a.Max, b.Index, ref results);
            }
            else
            {
                //Two leaves.
                results.Handle(Encode(a.Index), Encode(b.Index));
            }
        }


        ///// <summary>
        ///// Intersects two node children. 
        ///// The referenced node children must not be in unpinned managed memory.
        ///// </summary>
        ///// <param name="childA">First child to compare.</param>
        ///// <param name="childB">Second child to compare.</param>
        ///// <returns>True if the children overlap, false otherwise.</returns>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //unsafe static bool Intersects(in NodeChild childA, in NodeChild childB)
        //{
        //    return BoundingBox.Intersects(childA, childB);
        //    //if (Vector256.IsHardwareAccelerated && Avx.IsSupported)
        //    //{
        //    //    var a = Vector256.LoadUnsafe(ref Unsafe.AsRef(childA.Min.X));
        //    //    var b = Vector256.LoadUnsafe(ref Unsafe.AsRef(childB.Min.X));
        //    //    var min = Avx.Permute2x128(a, b, (0) | (2 << 4)); //(aMin, aMax) (bMin, bMax) -> (aMin, bMin)
        //    //    var max = Avx.Permute2x128(a, b, (3) | (1 << 4)); //(aMin, aMax) (bMin, bMax) -> (bMax, aMax)
        //    //    var noIntersection = Vector256.LessThan(max, min);
        //    //    return (Vector256.ExtractMostSignificantBits(noIntersection) & 0b1110111) == 0;
        //    //}
        //    //else
        //    //if (Vector128.IsHardwareAccelerated)
        //    //{
        //    //    //THIS IS A POTENTIAL GC HOLE IF CHILDREN ARE PASSED FROM UNPINNED MANAGED MEMORY
        //    //    var aMin = Vector128.LoadUnsafe(ref Unsafe.AsRef(childA.Min.X));
        //    //    var aMax = Vector128.LoadUnsafe(ref Unsafe.AsRef(childA.Max.X));
        //    //    var bMin = Vector128.LoadUnsafe(ref Unsafe.AsRef(childB.Min.X));
        //    //    var bMax = Vector128.LoadUnsafe(ref Unsafe.AsRef(childB.Max.X));
        //    //    var noIntersectionOnAxes = Vector128.LessThan(aMax, bMin) | Vector128.LessThan(bMax, aMin);
        //    //    return (Vector128.ExtractMostSignificantBits(noIntersectionOnAxes) & 0b111) == 0;
        //    //}
        //    //else
        //    {
        //        return BoundingBox.Intersects(childA.Min, childA.Max, childB.Min, childB.Max);
        //    }
        //}

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

        struct StackEntry
        {
            public int A;
            public int B;
        }

        unsafe void GetOverlapsWithLeaf<TOverlapHandler>(ref TOverlapHandler results, NodeChild leaf, int nodeToTest, ref QuickList<int> stack, BufferPool pool) where TOverlapHandler : IOverlapHandler
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
                    stack.Allocate(pool) = node.B.Index;
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
            }
        }

        public unsafe void GetSelfOverlaps2<TOverlapHandler>(ref TOverlapHandler results, BufferPool pool) where TOverlapHandler : IOverlapHandler
        {
            //If there are less than two leaves, there can't be any overlap.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (LeafCount < 2)
                return;

            QuickList<StackEntry> stack = new(Math.Min(1024, Math.Max(0, NodeCount / 4)), pool);
            QuickList<int> leafStack = new(Math.Min(512, Math.Max(0, NodeCount / 16)), pool);
            StackEntry nextTest = default;

            while (true)
            {
                if (nextTest.A == nextTest.B)
                {
                    //Self test.
                    //Possible sources of further stack entries in a self test:
                    //1) Child A and B are both internal and their bounds intersect.
                    //2) Child A is internal.
                    //3) Child B is internal.
                    //We can also spawn leaf-subtree tests if:
                    //1) Child A and B have intersecting bounds, and only one of them is a leaf.
                    //We can spawn direct leaf tests if:
                    //1) Child A and B have intersecting bounds, and both are leaves.
                    //Note that we never need to push an entry with differing A and B indices to the stack *from a self test*. There can only be one such entry created from any self test, and it's always visited next.
                    //Non-self tests will generate *only* results with differing A and B indices.
                    ref var node = ref Nodes[nextTest.A];
                    var abIntersect = BoundingBox.IntersectsUnsafe(node.A, node.B);
                    var aIsInternal = node.A.Index >= 0;
                    var bIsInternal = node.B.Index >= 0;
                    if (abIntersect && aIsInternal && bIsInternal)
                    {
                        //Both children internal and their bounds intersect.
                        //Their intersection should be the next visited.
                        nextTest.A = node.A.Index;
                        nextTest.B = node.B.Index;

                        //The two child self tests get pushed to the stack.
                        ref var stackA = ref stack.Allocate(pool);
                        stackA.A = node.A.Index;
                        stackA.B = node.A.Index;
                        ref var stackB = ref stack.Allocate(pool);
                        stackB.A = node.B.Index;
                        stackB.B = node.B.Index;
                    }
                    else
                    {
                        if (abIntersect)
                        {
                            Debug.Assert(!aIsInternal || !bIsInternal, "Just in case you shuffle logic around in the future: this clause assumes at least one leaf.");
                            //The children have overlapping bounds, but at least one is a leaf.
                            //Note that this path cannot generate any stack entries.
                            //At least one child is a leaf, so there is at most one self-test.
                            //The a-b test involves at least one leaf as well, which means it will use the GetOverlapsWithLeaf path.
                            //So the one possible self-test will be put into the nextTest.
                            if (aIsInternal || bIsInternal)
                            {
                                //One's a leaf, one's internal.
                                if (aIsInternal)
                                {
                                    nextTest.A = node.A.Index;
                                    nextTest.B = node.A.Index;
                                    GetOverlapsWithLeaf(ref results, node.B, node.A.Index, ref leafStack, pool);
                                }
                                else
                                {
                                    nextTest.A = node.B.Index;
                                    nextTest.B = node.B.Index;
                                    GetOverlapsWithLeaf(ref results, node.A, node.B.Index, ref leafStack, pool);
                                }
                            }
                            else
                            {
                                //Both children are leaves. They have overlapping bounds, so...
                                results.Handle(Encode(node.A.Index), Encode(node.B.Index));

                                //If both children are leaves, then there's no other source for the next visit, so pop.
                                if (!stack.TryPop(out nextTest))
                                    break;
                            }
                        }
                        else
                        {
                            //No a-b test, but possibly self tests.
                            if (aIsInternal && bIsInternal)
                            {
                                nextTest.A = node.A.Index;
                                nextTest.B = node.A.Index;
                                ref var stackB = ref stack.Allocate(pool);
                                stackB.A = node.B.Index;
                                stackB.B = node.B.Index;
                            }
                            else if (aIsInternal || bIsInternal)
                            {
                                var nextSelfTestIndex = aIsInternal ? node.A.Index : node.B.Index;
                                nextTest.A = nextSelfTestIndex;
                                nextTest.B = nextSelfTestIndex;
                            }
                            else
                            {
                                //There was no A-B test and both children are leaves. No new tests available, so grab from the stack.
                                if (!stack.TryPop(out nextTest))
                                    break;
                            }
                        }
                    }
                }
                else
                {
                    //Not a self test!
                    //Possible sources of stack entry:
                    //1) AA intersection, both internal
                    //2) AB intersection, both internal
                    //3) BA intersection, both internal
                    //4) BB intersection, both internal
                    ref var n0 = ref Nodes[nextTest.A];
                    ref var n1 = ref Nodes[nextTest.B];
                    var aaIntersects = BoundingBox.IntersectsUnsafe(n0.A, n1.A);
                    var abIntersects = BoundingBox.IntersectsUnsafe(n0.A, n1.B);
                    var baIntersects = BoundingBox.IntersectsUnsafe(n0.B, n1.A);
                    var bbIntersects = BoundingBox.IntersectsUnsafe(n0.B, n1.B);
                    var n0AIsInternal = n0.A.Index >= 0;
                    var n0BIsInternal = n0.B.Index >= 0;
                    var n1AIsInternal = n1.A.Index >= 0;
                    var n1BIsInternal = n1.B.Index >= 0;
                    //The first test which generates a stack candidate gets the nextTest; the rest get pushed to the stack.
                    int previousStackGeneratorCount = 0;
                    if (aaIntersects)
                    {
                        if (n0AIsInternal && n1AIsInternal)
                        {
                            ++previousStackGeneratorCount;
                            nextTest.A = n0.A.Index;
                            nextTest.B = n1.A.Index;
                        }
                        else
                        {
                            //At least one is a leaf.
                            if (n0AIsInternal)
                                GetOverlapsWithLeaf(ref results, n1.A, n0.A.Index, ref leafStack, pool);
                            else if (n1AIsInternal)
                                GetOverlapsWithLeaf(ref results, n0.A, n1.A.Index, ref leafStack, pool);
                            else //Both are leaves.
                                results.Handle(Encode(n0.A.Index), Encode(n1.A.Index));
                        }
                    }
                    if (abIntersects)
                    {
                        if (n0AIsInternal && n1BIsInternal)
                        {
                            if (previousStackGeneratorCount++ == 0)
                            {
                                nextTest.A = n0.A.Index;
                                nextTest.B = n1.B.Index;
                            }
                            else
                            {
                                ref var stackEntry = ref stack.Allocate(pool);
                                stackEntry.A = n0.A.Index;
                                stackEntry.B = n1.B.Index;
                            }
                        }
                        else
                        {
                            //At least one is a leaf.
                            if (n0AIsInternal)
                                GetOverlapsWithLeaf(ref results, n1.B, n0.A.Index, ref leafStack, pool);
                            else if (n1BIsInternal)
                                GetOverlapsWithLeaf(ref results, n0.A, n1.B.Index, ref leafStack, pool);
                            else //Both are leaves.
                                results.Handle(Encode(n0.A.Index), Encode(n1.B.Index));
                        }
                    }
                    if (baIntersects)
                    {
                        if (n0BIsInternal && n1AIsInternal)
                        {
                            if (previousStackGeneratorCount++ == 0)
                            {
                                nextTest.A = n0.B.Index;
                                nextTest.B = n1.A.Index;
                            }
                            else
                            {
                                ref var stackEntry = ref stack.Allocate(pool);
                                stackEntry.A = n0.B.Index;
                                stackEntry.B = n1.A.Index;
                            }
                        }
                        else
                        {
                            //At least one is a leaf.
                            if (n0BIsInternal)
                                GetOverlapsWithLeaf(ref results, n1.A, n0.B.Index, ref leafStack, pool);
                            else if (n1AIsInternal)
                                GetOverlapsWithLeaf(ref results, n0.B, n1.A.Index, ref leafStack, pool);
                            else //Both are leaves.
                                results.Handle(Encode(n0.B.Index), Encode(n1.A.Index));
                        }
                    }
                    if (bbIntersects)
                    {
                        if (n0BIsInternal && n1BIsInternal)
                        {
                            if (previousStackGeneratorCount++ == 0)
                            {
                                nextTest.A = n0.B.Index;
                                nextTest.B = n1.B.Index;
                            }
                            else
                            {
                                ref var stackEntry = ref stack.Allocate(pool);
                                stackEntry.A = n0.B.Index;
                                stackEntry.B = n1.B.Index;
                            }
                        }
                        else
                        {
                            //At least one is a leaf.
                            if (n0BIsInternal)
                                GetOverlapsWithLeaf(ref results, n1.B, n0.B.Index, ref leafStack, pool);
                            else if (n1BIsInternal)
                                GetOverlapsWithLeaf(ref results, n0.B, n1.B.Index, ref leafStack, pool);
                            else //Both are leaves.
                                results.Handle(Encode(n0.B.Index), Encode(n1.B.Index));
                        }
                    }
                    if (previousStackGeneratorCount == 0)
                    {
                        //None of the candidates generated a next step, so grab from the stack.
                        if (!stack.TryPop(out nextTest))
                            break;
                    }
                }

            }
            leafStack.Dispose(pool);
            stack.Dispose(pool);

        }

    }
}
