using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Trees
{
    public interface ISweepLeafTester
    {
        unsafe void TestLeaf(int leafIndex, ref float maximumT);
    }
    partial struct Tree
    {
        readonly unsafe void Sweep<TLeafTester>(int nodeIndex, in Vector3 expansion, in Vector3 origin, in Vector3 direction, TreeRay* treeRay, int* stack, ref TLeafTester leafTester) where TLeafTester : ISweepLeafTester
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
                    leafTester.TestLeaf(leafIndex, ref treeRay->MaximumT);
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        return;
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
                            Debug.Assert(stackEnd < TraversalStackCapacity - 1, "At the moment, we use a fixed size stack. Until we have explicitly tracked depths, watch out for excessive depth traversals.");
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
                            return;
                        nodeIndex = stack[--stackEnd];
                    }
                }
            }

        }

        internal readonly unsafe void Sweep<TLeafTester>(in Vector3 expansion, in Vector3 origin, in Vector3 direction, TreeRay* treeRay, ref TLeafTester sweepTester) where TLeafTester : ISweepLeafTester
        {
            if (leafCount == 0)
                return;

            if (leafCount == 1)
            {
                //If the first node isn't filled, we have to use a special case.
                if (Intersects(Nodes[0].A.Min - expansion, Nodes[0].A.Max + expansion, treeRay, out var tA))
                {
                    sweepTester.TestLeaf(0, ref treeRay->MaximumT);
                }
            }
            else
            {
                //TODO: Explicitly tracking depth in the tree during construction/refinement is practically required to guarantee correctness.
                //While it's exceptionally rare that any tree would have more than 256 levels, the worst case of stomping stack memory is not acceptable in the long run.
                var stack = stackalloc int[TraversalStackCapacity];
                Sweep(0, expansion, origin, direction, treeRay, stack, ref sweepTester);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConvertBoxToCentroidWithExtent(in Vector3 min, in Vector3 max, out Vector3 origin, out Vector3 expansion)
        {
            var halfMin = 0.5f * min;
            var halfMax = 0.5f * max;
            expansion = halfMax - halfMin;
            origin = halfMax + halfMin;
        }

        public readonly unsafe void Sweep<TLeafTester>(in Vector3 min, in Vector3 max, in Vector3 direction, float maximumT, ref TLeafTester sweepTester) where TLeafTester : ISweepLeafTester
        {
            ConvertBoxToCentroidWithExtent(min, max, out var origin, out var expansion);
            TreeRay.CreateFrom(origin, direction, maximumT, out var treeRay);
            Sweep(expansion, origin, direction, &treeRay, ref sweepTester);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly unsafe void Sweep<TLeafTester>(in BoundingBox boundingBox, in Vector3 direction, float maximumT, ref TLeafTester sweepTester) where TLeafTester : ISweepLeafTester
        {
            Sweep(boundingBox.Min, boundingBox.Max, direction, maximumT, ref sweepTester);
        }

    }
}
