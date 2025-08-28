using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        unsafe readonly void GetOverlaps<TEnumerator>(int nodeIndex, BoundingBox boundingBox, Buffer<int> stack, BufferPool pool, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            Debug.Assert((nodeIndex >= 0 && nodeIndex < NodeCount) || (Encode(nodeIndex) >= 0 && Encode(nodeIndex) < LeafCount));
            Debug.Assert(LeafCount >= 2, "This implementation assumes all nodes are filled.");

            int stackEnd = 0;
            while (true)
            {
                if (nodeIndex < 0)
                {
                    //This is actually a leaf node.
                    var leafIndex = Encode(nodeIndex);
                    if (!leafEnumerator.LoopBody(leafIndex))
                        break;
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        break;
                    nodeIndex = stack[--stackEnd];
                }
                else
                {
                    ref var node = ref Nodes[nodeIndex];
                    var aIntersected = BoundingBox.IntersectsUnsafe(node.A, boundingBox);
                    var bIntersected = BoundingBox.IntersectsUnsafe(node.B, boundingBox);

                    if (aIntersected)
                    {
                        nodeIndex = node.A.Index;
                        if (bIntersected)
                        {
                            if (stackEnd == stack.Length)
                            {
                                if (stack.Length == TraversalStackCapacity)
                                {
                                    // First allocation is on the stack.
                                    pool.TakeAtLeast<int>(TraversalStackCapacity * 2, out var newStack);
                                    stack.CopyTo(0, newStack, 0, TraversalStackCapacity);
                                    stack = newStack;
                                }
                                else
                                    pool.Resize(ref stack, stackEnd * 2, stackEnd);
                            }
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
                            break;
                        nodeIndex = stack[--stackEnd];
                    }
                }
            }
            if (stack.Length > TraversalStackCapacity)
            {
                // We rented a larger stack at some point. Return it.
                pool.Return(ref stack);
            }
        }

        /// <summary>
        /// Finds and processes all leaves with bounding boxes that overlap the specified axis-aligned bounding box. The <paramref name="leafEnumerator"/> is invoked
        /// for each overlapping element.
        /// </summary>
        /// <typeparam name="TEnumerator">The type of the <see cref="IBreakableForEach{T}"/> enumerator used to process the overlapping elements.</typeparam>
        /// <param name="boundingBox">Query to test against the bounding volume hierarchy.</param>
        /// <param name="pool">The buffer pool used for temporary allocations during the operation. Only used if the tree is pathologically deep; stack memory is used preferentially.</param>
        /// <param name="leafEnumerator">A reference to the enumerator that processes the indices of overlapping elements. The enumerator can
        /// terminate early by returning <see langword="false"/> from its iteration.</param>
        public readonly unsafe void GetOverlaps<TEnumerator>(BoundingBox boundingBox, BufferPool pool, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            if (LeafCount > 1)
            {
                var stack = stackalloc int[TraversalStackCapacity];
                GetOverlaps(0, boundingBox, new Buffer<int>(stack, TraversalStackCapacity), pool, ref leafEnumerator);
            }
            else if (LeafCount == 1)
            {
                Debug.Assert(Nodes[0].A.Index < 0, "If the root only has one child, it must be a leaf.");
                if (BoundingBox.IntersectsUnsafe(boundingBox, Nodes[0].A))
                {
                    leafEnumerator.LoopBody(Encode(Nodes[0].A.Index));
                }
                return;
            }
            //If the leaf count is zero, then there's nothing to test against.
        }

        /// <summary>
        /// Finds and processes all leaves with bounding boxes that overlap the specified axis-aligned bounding box. The <paramref name="leafEnumerator"/> is invoked
        /// for each overlapping element.
        /// </summary>
        /// <typeparam name="TEnumerator">The type of the <see cref="IBreakableForEach{T}"/> enumerator used to process the overlapping elements.</typeparam>
        /// <param name="min">The minimum corner of the axis-aligned bounding box.</param>
        /// <param name="max">The maximum corner of the axis-aligned bounding box.</param>
        /// <param name="pool">The buffer pool used for temporary allocations during the operation. Only used if the tree is pathologically deep; stack memory is used preferentially.</param>
        /// <param name="leafEnumerator">A reference to the enumerator that processes the indices of overlapping elements. The enumerator can
        /// terminate early by returning <see langword="false"/> from its iteration.</param>
        public readonly void GetOverlaps<TEnumerator>(Vector3 min, Vector3 max, BufferPool pool, ref TEnumerator leafEnumerator) where TEnumerator : IBreakableForEach<int>
        {
            GetOverlaps(new BoundingBox(min, max), pool, ref leafEnumerator);
        }


    }
}
