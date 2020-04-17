using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool Intersects(in NodeChild a, in NodeChild b)
        {
            return BoundingBox.Intersects(a.Min, a.Max, b.Min, b.Max);
        }

        unsafe void GetOverlapsBetweenDifferentNodes<TOverlapHandler>(ref Node a, ref Node b, ref TOverlapHandler results) where TOverlapHandler : IOverlapHandler
        {
            //There are no shared children, so test them all.
            ref var aa = ref a.A;
            ref var ab = ref a.B;
            ref var ba = ref b.A;
            ref var bb = ref b.B;
            var aaIntersects = Intersects(aa, ba);
            var abIntersects = Intersects(aa, bb);
            var baIntersects = Intersects(ab, ba);
            var bbIntersects = Intersects(ab, bb);

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

            var ab = Intersects(a, b);

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

    }
}
