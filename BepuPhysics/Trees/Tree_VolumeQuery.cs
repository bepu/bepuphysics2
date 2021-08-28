using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        unsafe readonly void GetOverlaps<TEnumerator>(int nodeIndex, in Vector3 min, in Vector3 max, int* stack, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
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
                    ref var node = ref Nodes[nodeIndex];
                    var aIntersected = BoundingBox.Intersects(node.A.Min, node.A.Max, min, max);
                    var bIntersected = BoundingBox.Intersects(node.B.Min, node.B.Max, min, max);

                    if (aIntersected)
                    {
                        nodeIndex = node.A.Index;
                        if (bIntersected)
                        {
                            //Visit the earlier AABB intersection first.
                            Debug.Assert(stackEnd < TraversalStackCapacity - 1, "At the moment, we use a fixed size stack. Until we have explicitly tracked depths, watch out for excessive depth traversals.");
                            stack[stackEnd++] = node.B.Index;

                        }
                    }
                    else if (bIntersected)
                    {
                        nodeIndex = node.B.Index;
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
        public readonly unsafe void GetOverlaps<TEnumerator>(in Vector3 min, in Vector3 max, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            if (leafCount > 1)
            {
                //TODO: Explicitly tracking depth in the tree during construction/refinement is practically required to guarantee correctness.
                //While it's exceptionally rare that any tree would have more than 256 levels, the worst case of stomping stack memory is not acceptable in the long run.
                var stack = stackalloc int[TraversalStackCapacity];
                GetOverlaps(0, min, max, stack, ref leafEnumerator);
            }
            else if (leafCount == 1)
            {
                Debug.Assert(Nodes[0].A.Index < 0, "If the root only has one child, it must be a leaf.");
                if (BoundingBox.Intersects(min, max, Nodes[0].A.Min, Nodes[0].A.Max))
                {
                    leafEnumerator.LoopBody(Encode(Nodes[0].A.Index));
                }
                return;
            }
            //If the leaf count is zero, then there's nothing to test against.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly unsafe void GetOverlaps<TEnumerator>(in BoundingBox boundingBox, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            GetOverlaps(boundingBox.Min, boundingBox.Max, ref leafEnumerator);
        }


    }
}
