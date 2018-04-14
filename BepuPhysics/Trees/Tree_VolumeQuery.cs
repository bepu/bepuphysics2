using BepuUtilities;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    partial class Tree
    {
        //TODO: This is far from an optimal implementation of a volume query. The entire tree structure is slated for a revamp that covers
        //a refinement rewrite, a memory layout scrubbing, and some significant effort in query codegen. This recursive approach is just a simple stopgap.

        //There is also value in a 'breakable' variant of these queries. It's much clearer for ray casts where you have an obvious 'progression' that you can halt meaningfully,
        //but there are some use cases even for volumes where being able to quit early could be useful. We'll revisit that later once we actually have something shipped.
        unsafe void GetOverlapsWithNode<TEnumerator>(int nodeIndex, ref BoundingBox query, ref TEnumerator results) where TEnumerator : IForEach<int>
        {
            Debug.Assert(LeafCount >= 2);
            var node = nodes + nodeIndex;

            if (BoundingBox.Intersects(ref query.Min, ref query.Max, ref node->A.Min, ref node->A.Max))
            {
                if (node->A.Index >= 0)
                {
                    GetOverlapsWithNode(node->A.Index, ref query, ref results);
                }
                else
                {
                    results.LoopBody(Encode(node->A.Index));
                }
            }
            if (BoundingBox.Intersects(ref query.Min, ref query.Max, ref node->B.Min, ref node->B.Max))
            {
                if (node->B.Index >= 0)
                {
                    GetOverlapsWithNode(node->B.Index, ref query, ref results);
                }
                else
                {
                    results.LoopBody(Encode(node->B.Index));
                }
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetOverlaps<TEnumerator>(ref BoundingBox boundingBox, ref TEnumerator results) where TEnumerator : IForEach<int>
        {
            if (leafCount > 1)
            {
                GetOverlapsWithNode(0, ref boundingBox, ref results);
            }
            else if (leafCount == 1)
            {
                Debug.Assert(nodes->A.Index < 0, "If the root only has one child, it must be a leaf.");
                if (BoundingBox.Intersects(ref boundingBox.Min, ref boundingBox.Max, ref nodes->A.Min, ref nodes->A.Max))
                {
                    results.LoopBody(Encode(nodes->A.Index));
                }
                return;
            }
            //If the leaf count is zero, then there's nothing to test against.

        }


    }
}
