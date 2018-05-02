using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    partial class Tree
    {
        //TODO: This is far from an optimal implementation of a volume query. The entire tree structure is slated for a revamp that covers
        //a refinement rewrite, a memory layout scrubbing, and some significant effort in query codegen. This recursive approach is just a simple stopgap.

        //There is also value in a 'breakable' variant of these queries. It's much clearer for ray casts where you have an obvious 'progression' that you can halt meaningfully,
        //but there are some use cases even for volumes where being able to quit early could be useful. We'll revisit that later once we actually have something shipped.
        unsafe void GetOverlapsWithNode<TEnumerator>(int nodeIndex, ref BoundingBox query, ref TEnumerator results) where TEnumerator : IBreakableForEach<int>
        {
            Debug.Assert(LeafCount >= 2);
            var node = nodes + nodeIndex;

            if (BoundingBox.Intersects(query.Min,query.Max, node->A.Min, node->A.Max))
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
            if (BoundingBox.Intersects(query.Min, query.Max, node->B.Min, node->B.Max))
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
        public unsafe void GetOverlaps<TEnumerator>(ref BoundingBox boundingBox, ref TEnumerator results) where TEnumerator : IBreakableForEach<int>
        {
            if (leafCount > 1)
            {
                GetOverlapsWithNode(0, ref boundingBox, ref results);
            }
            else if (leafCount == 1)
            {
                Debug.Assert(nodes->A.Index < 0, "If the root only has one child, it must be a leaf.");
                if (BoundingBox.Intersects(boundingBox.Min, boundingBox.Max, nodes->A.Min, nodes->A.Max))
                {
                    results.LoopBody(Encode(nodes->A.Index));
                }
                return;
            }
            //If the leaf count is zero, then there's nothing to test against.

        }

        unsafe void GetOverlaps2<TEnumerator>(int nodeIndex, in Vector3 min, in Vector3 max, int* stack, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            Debug.Assert((nodeIndex >= 0 && nodeIndex < nodeCount) || (Encode(nodeIndex) >= 0 && Encode(nodeIndex) < leafCount));
            Debug.Assert(leafCount >= 2, "This implementation assumes all nodes are filled.");

            int stackEnd = 0;
            while (true)
            {
                if (nodeIndex < 0)
                {
                    //This is actually a leaf node.
                    var leafIndex = Encode(nodeIndex);
                    if (!leafEnumerator.LoopBody(leafIndex))
                        return;
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        return;
                    nodeIndex = stack[--stackEnd];
                }
                else
                {
                    var node = nodes + nodeIndex;
                    var aIntersected = BoundingBox.Intersects(node->A.Min, node->A.Max, min, max);
                    var bIntersected = BoundingBox.Intersects(node->B.Min, node->B.Max, min, max);

                    if (aIntersected)
                    {
                        nodeIndex = node->A.Index;
                        if (bIntersected)
                        {
                            //Visit the earlier AABB intersection first.
                            Debug.Assert(stackEnd < TraversalStackCapacity - 1, "At the moment, we use a fixed size stack. Until we have explicitly tracked depths, watch out for excessive depth traversals.");
                            stack[stackEnd++] = node->B.Index;

                        }
                    }
                    else if (bIntersected)
                    {
                        nodeIndex = node->B.Index;
                    }
                    else
                    {
                        //No intersection. Need to pull from the stack to get a new target.
                        if (stackEnd == 0)
                            return;
                        nodeIndex = stack[--stackEnd];
                    }
                }
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetOverlaps2<TEnumerator>(in Vector3 min, in Vector3 max, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            if (leafCount > 1)
            {
                //TODO: Explicitly tracking depth in the tree during construction/refinement is practically required to guarantee correctness.
                //While it's exceptionally rare that any tree would have more than 256 levels, the worst case of stomping stack memory is not acceptable in the long run.
                var stack = stackalloc int[TraversalStackCapacity];
                GetOverlaps2(0, min, max, stack, ref leafEnumerator);
            }
            else if (leafCount == 1)
            {
                Debug.Assert(nodes->A.Index < 0, "If the root only has one child, it must be a leaf.");
                if (BoundingBox.Intersects(min, max, nodes->A.Min, nodes->A.Max))
                {
                    leafEnumerator.LoopBody(Encode(nodes->A.Index));
                }
                return;
            }
            //If the leaf count is zero, then there's nothing to test against.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetOverlaps2<TEnumerator>(in BoundingBox boundingBox, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            GetOverlaps2(boundingBox.Min, boundingBox.Max, ref leafEnumerator);
        }


    }
}
