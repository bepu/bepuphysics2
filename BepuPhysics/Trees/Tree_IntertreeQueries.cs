using BepuUtilities;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    partial class Tree
    {
        unsafe void DispatchTestForNodeAgainstLeaf<TOverlapHandler>(int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (nodeIndex < 0)
            {
                results.Handle(Encode(nodeIndex), leafIndex);
            }
            else
            {
                TestNodeAgainstLeaf(nodeIndex, leafIndex, ref leafMin, ref leafMax, ref results);
            }
        }
        private unsafe void TestNodeAgainstLeaf<TOverlapHandler>(int nodeIndex, int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            var node = nodes + nodeIndex;
            ref var a = ref node->A;
            ref var b = ref node->B;
            //Despite recursion, leafBounds should remain in L1- it'll be used all the way down the recursion from here.
            //However, while we likely loaded child B when we loaded child A, there's no guarantee that it will stick around.
            //Reloading that in the event of eviction would require more work than keeping the derived data on the stack.
            //TODO: this is some pretty questionable microtuning. It's not often that the post-leaf-found recursion will be long enough to evict L1. Definitely test it.
            var bIndex = b.Index;
            var aIntersects = BoundingBox.Intersects(ref leafMin, ref leafMax, ref a.Min, ref a.Max);
            var bIntersects = BoundingBox.Intersects(ref leafMin, ref leafMax, ref b.Min, ref b.Max);
            if (aIntersects)
            {
                DispatchTestForNodeAgainstLeaf(leafIndex, ref leafMin, ref leafMax, a.Index, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForNodeAgainstLeaf(leafIndex, ref leafMin, ref leafMax, bIndex, ref results);
            }
        }

        unsafe void DispatchTestForLeafAgainstNode<TOverlapHandler>(int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, Tree treeB, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (nodeIndex < 0)
            {
                results.Handle(leafIndex, Encode(nodeIndex));
            }
            else
            {
                TestLeafAgainstNode(leafIndex, ref leafMin, ref leafMax, nodeIndex, treeB, ref results);
            }
        }
        unsafe void TestLeafAgainstNode<TOverlapHandler>(int leafIndex, ref Vector3 leafMin, ref Vector3 leafMax, int nodeIndex, Tree treeB, ref TOverlapHandler results)
            where TOverlapHandler : IOverlapHandler
        {
            var node = treeB.nodes + nodeIndex;
            ref var a = ref node->A;
            ref var b = ref node->B;
            //Despite recursion, leafBounds should remain in L1- it'll be used all the way down the recursion from here.
            //However, while we likely loaded child B when we loaded child A, there's no guarantee that it will stick around.
            //Reloading that in the event of eviction would require more work than keeping the derived data on the stack.
            //TODO: this is some pretty questionable microtuning. It's not often that the post-leaf-found recursion will be long enough to evict L1. Definitely test it.
            var bIndex = b.Index;
            var aIntersects = BoundingBox.Intersects(ref leafMin, ref leafMax, ref a.Min, ref a.Max);
            var bIntersects = BoundingBox.Intersects(ref leafMin, ref leafMax, ref b.Min, ref b.Max);
            if (aIntersects)
            {
                DispatchTestForLeafAgainstNode(leafIndex, ref leafMin, ref leafMax, a.Index, treeB, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForLeafAgainstNode(leafIndex, ref leafMin, ref leafMax, bIndex, treeB, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void DispatchTestForNodes<TOverlapHandler>(ref NodeChild a, ref NodeChild b, Tree treeB, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            if (a.Index >= 0)
            {
                if (b.Index >= 0)
                {
                    GetOverlapsBetweenDifferentNodes(nodes + a.Index, treeB.nodes + b.Index, treeB, ref results);
                }
                else
                {
                    //leaf B versus node A. Note that we have to maintain order; treeB nodes always should be in the second slot.
                    TestNodeAgainstLeaf(a.Index, Encode(b.Index), ref b.Min, ref b.Max, ref results);
                }
            }
            else if (b.Index >= 0)
            {
                //leaf A versus node B. Note that we have to maintain order; treeB nodes always should be in the second slot.
                TestLeafAgainstNode(Encode(a.Index), ref a.Min, ref a.Max, b.Index, treeB, ref results);
            }
            else
            {
                //Two leaves.
                results.Handle(Encode(a.Index), Encode(b.Index));
            }
        }

        private unsafe void GetOverlapsBetweenDifferentNodes<TOverlapHandler>(Node* a, Node* b, Tree treeB, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            ref var aa = ref a->A;
            ref var ab = ref a->B;
            ref var ba = ref b->A;
            ref var bb = ref b->B;
            var aaIntersects = Intersects(ref aa, ref ba);
            var abIntersects = Intersects(ref aa, ref bb);
            var baIntersects = Intersects(ref ab, ref ba);
            var bbIntersects = Intersects(ref ab, ref bb);

            if (aaIntersects)
            {
                DispatchTestForNodes(ref aa, ref ba, treeB, ref results);
            }
            if (abIntersects)
            {
                DispatchTestForNodes(ref aa, ref bb, treeB, ref results);
            }
            if (baIntersects)
            {
                DispatchTestForNodes(ref ab, ref ba, treeB, ref results);
            }
            if (bbIntersects)
            {
                DispatchTestForNodes(ref ab, ref bb, treeB, ref results);
            }
        }


        public unsafe void GetOverlaps<TOverlapHandler>(Tree treeB, ref TOverlapHandler overlapHandler) where TOverlapHandler : struct, IOverlapHandler
        {
            if (leafCount == 0 || treeB.leafCount == 0)
                return;
            if (leafCount >= 2 && treeB.leafCount >= 2)
            {
                //Both trees have complete nodes; we can use a general case.
                GetOverlapsBetweenDifferentNodes(nodes, treeB.nodes, treeB, ref overlapHandler);
            }
            else if (leafCount == 1 && treeB.leafCount >= 2)
            {
                //Tree A is degenerate; needs a special case.
                var a = nodes;
                var b = treeB.nodes;
                var aaIntersects = Intersects(ref a->A, ref b->A);
                var abIntersects = Intersects(ref a->A, ref b->B);
                if (aaIntersects)
                {
                    DispatchTestForNodes(ref a->A, ref b->A, treeB, ref overlapHandler);
                }
                if (abIntersects)
                {
                    DispatchTestForNodes(ref a->A, ref b->B, treeB, ref overlapHandler);
                }
            }
            else if (leafCount >= 2 && treeB.leafCount == 1)
            {
                //Tree B is degenerate; needs a special case.
                var a = nodes;
                var b = treeB.nodes;
                var aaIntersects = Intersects(ref a->A, ref b->A);
                var baIntersects = Intersects(ref a->B, ref b->A);
                if (aaIntersects)
                {
                    DispatchTestForNodes(ref a->A, ref b->A, treeB, ref overlapHandler);
                }
                if (baIntersects)
                {
                    DispatchTestForNodes(ref a->B, ref b->A, treeB, ref overlapHandler);
                }
            }
            else
            {
                Debug.Assert(leafCount == 1 && treeB.leafCount == 1);
                if (Intersects(ref nodes->A, ref treeB.nodes->A))
                {
                    DispatchTestForNodes(ref nodes->A, ref treeB.nodes->A, treeB, ref overlapHandler);
                }
            }
        }

    }
}
