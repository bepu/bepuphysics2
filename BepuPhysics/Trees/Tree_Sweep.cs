using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    public interface ISweepLeafTester
    {
        void TestLeaf(int leafIndex, ref float maximumT);
    }
    partial struct Tree
    {
        readonly unsafe void Sweep<TLeafTester>(int nodeIndex, Vector3 expansion, Vector3 origin, Vector3 direction, TreeRay* treeRay, Buffer<int> stack, BufferPool pool, ref TLeafTester leafTester) where TLeafTester : ISweepLeafTester
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
                    leafTester.TestLeaf(leafIndex, ref treeRay->MaximumT);
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        break;
                    nodeIndex = stack[--stackEnd];
                }
                else
                {
                    ref var node = ref Nodes[nodeIndex];
                    var minA = node.A.Min - expansion;
                    var maxA = node.A.Max + expansion;
                    var aIntersected = Intersects(minA, maxA, treeRay, out var tA);
                    var minB = node.B.Min - expansion;
                    var maxB = node.B.Max + expansion;
                    var bIntersected = Intersects(minB, maxB, treeRay, out var tB);

                    if (aIntersected)
                    {
                        if (bIntersected)
                        {
                            //Visit the earlier AABB intersection first.
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
                            if (tA < tB)
                            {
                                nodeIndex = node.A.Index;
                                stack[stackEnd++] = node.B.Index;
                            }
                            else
                            {
                                nodeIndex = node.B.Index;
                                stack[stackEnd++] = node.A.Index;
                            }
                        }
                        else
                        {
                            //Single intersection cases don't require an explicit stack entry.
                            nodeIndex = node.A.Index;
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

        internal readonly unsafe void Sweep<TLeafTester>(Vector3 expansion, Vector3 origin, Vector3 direction, TreeRay* treeRay, BufferPool pool, ref TLeafTester sweepTester) where TLeafTester : ISweepLeafTester
        {
            if (LeafCount == 0)
                return;

            if (LeafCount == 1)
            {
                //If the first node isn't filled, we have to use a special case.
                if (Intersects(Nodes[0].A.Min - expansion, Nodes[0].A.Max + expansion, treeRay, out var tA))
                {
                    sweepTester.TestLeaf(0, ref treeRay->MaximumT);
                }
            }
            else
            {
                var stack = stackalloc int[TraversalStackCapacity];
                Sweep(0, expansion, origin, direction, treeRay, new Buffer<int>(stack, TraversalStackCapacity), pool, ref sweepTester);
            }
        }

        /// <summary>
        /// Converts a bounding box defined by minimum and maximum corners into a centroid and half-extent representation.
        /// </summary>
        /// <param name="min">The minimum corner of the bounding box.</param>
        /// <param name="max">The maximum corner of the bounding box.</param>
        /// <param name="origin">The computed centroid of the bounding box.</param>
        /// <param name="expansion">The computed half-extents of the bounding box.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConvertBoxToCentroidWithExtent(Vector3 min, Vector3 max, out Vector3 origin, out Vector3 expansion)
        {
            var halfMin = 0.5f * min;
            var halfMax = 0.5f * max;
            expansion = halfMax - halfMin;
            origin = halfMax + halfMin;
        }

        /// <summary>
        /// Performs a sweep test of an axis-aligned bounding box against the tree and invokes the <paramref name="sweepTester"/> for each intersecting leaf.
        /// </summary>
        /// <typeparam name="TLeafTester">The type of the <see cref="ISweepLeafTester"/> used to process the intersecting leaves.</typeparam>
        /// <param name="min">The minimum corner of the axis-aligned bounding box to sweep.</param>
        /// <param name="max">The maximum corner of the axis-aligned bounding box to sweep.</param>
        /// <param name="direction">The direction of the sweep.</param>
        /// <param name="maximumT">The maximum parametric distance along the sweep direction to test.</param>
        /// <param name="sweepTester">A reference to the tester that processes the indices of intersecting leaves.</param>
        /// <param name="pool">The buffer pool used for temporary allocations during the operation. Only used if the tree is pathologically deep; stack memory is used preferentially.</param>
        public readonly unsafe void Sweep<TLeafTester>(Vector3 min, Vector3 max, Vector3 direction, float maximumT, BufferPool pool, ref TLeafTester sweepTester) where TLeafTester : ISweepLeafTester
        {
            ConvertBoxToCentroidWithExtent(min, max, out var origin, out var expansion);
            TreeRay.CreateFrom(origin, direction, maximumT, out var treeRay);
            Sweep(expansion, origin, direction, &treeRay, pool, ref sweepTester);
        }
        /// <summary>
        /// Performs a sweep test of a bounding box against the tree and invokes the <paramref name="sweepTester"/> for each intersecting leaf.
        /// </summary>
        /// <typeparam name="TLeafTester">The type of the <see cref="ISweepLeafTester"/> used to process the intersecting leaves.</typeparam>
        /// <param name="boundingBox">The bounding box to sweep.</param>
        /// <param name="direction">The direction of the sweep.</param>
        /// <param name="maximumT">The maximum parametric distance along the sweep direction to test.</param>
        /// <param name="sweepTester">A reference to the tester that processes the indices of intersecting leaves.</param>
        /// <param name="pool">The buffer pool used for temporary allocations during the operation. Only used if the tree is pathologically deep; stack memory is used preferentially.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void Sweep<TLeafTester>(in BoundingBox boundingBox, Vector3 direction, float maximumT, BufferPool pool, ref TLeafTester sweepTester) where TLeafTester : ISweepLeafTester
        {
            Sweep(boundingBox.Min, boundingBox.Max, direction, maximumT, pool, ref sweepTester);
        }

    }
}
