using BepuUtilities;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        unsafe void DispatchTestForNodeAgainstLeaf<TOverlapHandler>(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (nodeIndex < 0)
            {
                results.Handle(Encode(nodeIndex), leafIndex);
            }
            else
            {
                TestNodeAgainstLeaf(nodeIndex, leafIndex, ref leafChild, ref results);
            }
        }
        private unsafe void TestNodeAgainstLeaf<TOverlapHandler>(int nodeIndex, int leafIndex, ref NodeChild leafChild, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
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
                DispatchTestForNodeAgainstLeaf(leafIndex, ref leafChild, a.Index, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForNodeAgainstLeaf(leafIndex, ref leafChild, bIndex, ref results);
            }
        }

        unsafe void DispatchTestForLeafAgainstNode<TOverlapHandler>(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref Tree treeB, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (nodeIndex < 0)
            {
                results.Handle(leafIndex, Encode(nodeIndex));
            }
            else
            {
                TestLeafAgainstNode(leafIndex, ref leafChild, nodeIndex, ref treeB, ref results);
            }
        }
        unsafe void TestLeafAgainstNode<TOverlapHandler>(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref Tree treeB, ref TOverlapHandler results)
            where TOverlapHandler : IOverlapHandler
        {
            ref var node = ref treeB.Nodes[nodeIndex];
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
                DispatchTestForLeafAgainstNode(leafIndex, ref leafChild, a.Index, ref treeB, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForLeafAgainstNode(leafIndex, ref leafChild, bIndex, ref treeB, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void DispatchTestForNodes<TOverlapHandler>(ref NodeChild a, ref NodeChild b, ref Tree treeB, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (a.Index >= 0)
            {
                if (b.Index >= 0)
                {
                    GetOverlapsBetweenDifferentNodes(ref Nodes[a.Index], ref treeB.Nodes[b.Index], ref treeB, ref results);
                }
                else
                {
                    //leaf B versus node A. Note that we have to maintain order; treeB nodes always should be in the second slot.
                    TestNodeAgainstLeaf(a.Index, Encode(b.Index), ref b, ref results);
                }
            }
            else if (b.Index >= 0)
            {
                //leaf A versus node B. Note that we have to maintain order; treeB nodes always should be in the second slot.
                TestLeafAgainstNode(Encode(a.Index), ref a, b.Index, ref treeB, ref results);
            }
            else
            {
                //Two leaves.
                results.Handle(Encode(a.Index), Encode(b.Index));
            }
        }

        private unsafe void GetOverlapsBetweenDifferentNodes<TOverlapHandler>(ref Node a, ref Node b, ref Tree treeB, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
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
                DispatchTestForNodes(ref aa, ref ba, ref treeB, ref results);
            }
            if (abIntersects)
            {
                DispatchTestForNodes(ref aa, ref bb, ref treeB, ref results);
            }
            if (baIntersects)
            {
                DispatchTestForNodes(ref ab, ref ba, ref treeB, ref results);
            }
            if (bbIntersects)
            {
                DispatchTestForNodes(ref ab, ref bb, ref treeB, ref results);
            }
        }

        public unsafe void GetOverlaps<TOverlapHandler>(ref Tree treeB, ref TOverlapHandler overlapHandler) where TOverlapHandler : struct, IOverlapHandler
        {
            if (LeafCount == 0 || treeB.LeafCount == 0)
                return;
            if (LeafCount == 1 && treeB.LeafCount >= 2)
            {
                //Tree A is degenerate; needs a special case.
                ref var a = ref Nodes[0];
                ref var b = ref treeB.Nodes[0];
                var aaIntersects = BoundingBox.IntersectsUnsafe(a.A, b.A);
                var abIntersects = BoundingBox.IntersectsUnsafe(a.A, b.B);
                if (aaIntersects)
                {
                    DispatchTestForNodes(ref a.A, ref b.A, ref treeB, ref overlapHandler);
                }
                if (abIntersects)
                {
                    DispatchTestForNodes(ref a.A, ref b.B, ref treeB, ref overlapHandler);
                }
                return;
            }
            if (LeafCount >= 2 && treeB.LeafCount == 1)
            {
                //Tree B is degenerate; needs a special case.
                ref var a = ref Nodes[0];
                ref var b = ref treeB.Nodes[0];
                var aaIntersects = BoundingBox.IntersectsUnsafe(a.A, b.A);
                var baIntersects = BoundingBox.IntersectsUnsafe(a.B, b.A);
                if (aaIntersects)
                {
                    DispatchTestForNodes(ref a.A, ref b.A, ref treeB, ref overlapHandler);
                }
                if (baIntersects)
                {
                    DispatchTestForNodes(ref a.B, ref b.A, ref treeB, ref overlapHandler);
                }
                return;
            }
            if (LeafCount == 1 && treeB.LeafCount == 1)
            {
                //Both degenerate.
                if (BoundingBox.IntersectsUnsafe(Nodes[0].A, treeB.Nodes[0].A))
                {
                    DispatchTestForNodes(ref Nodes[0].A, ref treeB.Nodes[0].A, ref treeB, ref overlapHandler);
                }
                return;
            }
            //Both trees have complete nodes; we can use a general case.
            GetOverlapsBetweenDifferentNodes(ref Nodes[0], ref treeB.Nodes[0], ref treeB, ref overlapHandler);

        }

    }
}
