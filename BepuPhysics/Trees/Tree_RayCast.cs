using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Trees
{
    partial class Tree
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static bool Intersects(ref Vector3 min, ref Vector3 max, TreeRay* ray, out float t)
        {
            var t0 = min * ray->InverseDirection - ray->OriginOverDirection;
            var t1 = max * ray->InverseDirection - ray->OriginOverDirection;
            var tExit = Vector3.Max(t0, t1);
            var tEntry = Vector3.Min(t0, t1);
            //TODO: Some potential microoptimization opportunities here, especially with platform specific intrinsics.
            var earliestExit = tExit.X < tExit.Y ? tExit.X : tExit.Y;
            t = tEntry.X > tEntry.Y ? tEntry.X : tEntry.Y;
            if (tExit.Z < earliestExit)
                earliestExit = tExit.Z;
            if (tEntry.Z > t)
                t = tEntry.Z;
            return t < earliestExit && t < ray->MaximumT;
        }

        
        internal unsafe void RayCast<TLeafTester>(int nodeIndex, TreeRay* treeRay, RayData* rayData, int* stack, ref TLeafTester leafTester) where TLeafTester : ILeafTester
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
                    leafTester.RayTest(leafIndex, rayData, &treeRay->MaximumT);
                    //Leaves have no children; have to pull from the stack to get a new target.
                    if (stackEnd == 0)
                        return;
                    nodeIndex = stack[--stackEnd];
                }
                else
                {
                    var node = nodes + nodeIndex;
                    var aIntersected = Intersects(ref node->A.Min, ref node->A.Max, treeRay, out var tA);
                    var bIntersected = Intersects(ref node->B.Min, ref node->B.Max, treeRay, out var tB);

                    if (aIntersected && bIntersected)
                    {
                        //Visit the earlier AABB intersection first.
                        if (tA < tB)
                        {
                            nodeIndex = node->A.Index;
                            stack[stackEnd++] = node->B.Index;
                        }
                        else
                        {
                            nodeIndex = node->B.Index;
                            stack[stackEnd++] = node->A.Index;
                        }
                    }
                    //Single intersection cases don't require an explicit stack entry.
                    else if (aIntersected)
                    {
                        nodeIndex = node->A.Index;
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
    }
}
