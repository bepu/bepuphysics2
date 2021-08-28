using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool Intersects(in Vector3 min, in Vector3 max, TreeRay* ray, out float t)
        {
            var t0 = min * ray->InverseDirection - ray->OriginOverDirection;
            var t1 = max * ray->InverseDirection - ray->OriginOverDirection;
            var tExit = Vector3.Max(t0, t1);
            var tEntry = Vector3.Min(t0, t1);
            //TODO: Note the use of broadcast and SIMD min/max here. This is much faster than using branches to compute minimum elements, since the branches
            //get mispredicted extremely frequently. Also note 4-wide operations; they're actually faster than using Vector2 or Vector3 due to some unnecessary codegen as of this writing.
            var earliestExit = Vector4.Min(Vector4.Min(new Vector4(ray->MaximumT), new Vector4(tExit.X)), Vector4.Min(new Vector4(tExit.Y), new Vector4(tExit.Z))).X;
            t = Vector4.Max(Vector4.Max(new Vector4(tEntry.X), Vector4.Zero), Vector4.Max(new Vector4(tEntry.Y), new Vector4(tEntry.Z))).X;
            return t <= earliestExit;
        }


        internal readonly unsafe void RayCast<TLeafTester>(int nodeIndex, TreeRay* treeRay, RayData* rayData, int* stack, ref TLeafTester leafTester) where TLeafTester : IRayLeafTester
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
                    leafTester.TestLeaf(leafIndex, rayData, &treeRay->MaximumT);
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        return;
                    nodeIndex = stack[--stackEnd];
                }
                else
                {
                    ref var node = ref Nodes[nodeIndex];
                    var aIntersected = Intersects(node.A.Min, node.A.Max, treeRay, out var tA);
                    var bIntersected = Intersects(node.B.Min, node.B.Max, treeRay, out var tB);

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

        internal const int TraversalStackCapacity = 256;

        internal readonly unsafe void RayCast<TLeafTester>(TreeRay* treeRay, RayData* rayData, ref TLeafTester leafTester) where TLeafTester : IRayLeafTester
        {
            if (leafCount == 0)
                return;

            if (leafCount == 1)
            {
                //If the first node isn't filled, we have to use a special case.
                if (Intersects(Nodes[0].A.Min, Nodes[0].A.Max, treeRay, out var tA))
                {
                    leafTester.TestLeaf(0, rayData, &treeRay->MaximumT);
                }
            }
            else
            {
                //TODO: Explicitly tracking depth in the tree during construction/refinement is practically required to guarantee correctness.
                //While it's exceptionally rare that any tree would have more than 256 levels, the worst case of stomping stack memory is not acceptable in the long run.
                var stack = stackalloc int[TraversalStackCapacity];
                RayCast(0, treeRay, rayData, stack, ref leafTester);
            }
        }

        public readonly unsafe void RayCast<TLeafTester>(in Vector3 origin, in Vector3 direction, ref float maximumT, ref TLeafTester leafTester, int id = 0) where TLeafTester : IRayLeafTester
        {
            TreeRay.CreateFrom(origin, direction, maximumT, id, out var rayData, out var treeRay);
            RayCast(&treeRay, &rayData, ref leafTester);
            //The maximumT could have been mutated by the leaf tester. Propagate that change. This is important for when we jump between tree traversals and such.
            maximumT = treeRay.MaximumT;
        }

    }
}
