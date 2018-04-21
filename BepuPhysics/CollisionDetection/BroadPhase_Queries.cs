using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    partial class BroadPhase
    {
        struct LeafTester<TRayTester> : ILeafTester where TRayTester : IBroadPhaseRayTester
        {
            public TRayTester RayTester;
            public Buffer<CollidableReference> Leaves;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void RayTest(int leafIndex, RayData* rayData, float* maximumT)
            {
                RayTester.RayTest(Leaves[leafIndex], rayData, maximumT);
            }
        }

        public unsafe void RayCast<TRayTester>(ref Vector3 origin, ref Vector3 direction, float maximumT, ref TRayTester rayTester, int id = 0) where TRayTester : IBroadPhaseRayTester
        {
            TreeRay.CreateFrom(ref origin, ref direction, maximumT, id, out var rayData, out var treeRay);
            LeafTester<TRayTester> tester;
            tester.RayTester = rayTester;
            tester.Leaves = activeLeaves;
            ActiveTree.RayCast(&treeRay, &rayData, ref tester);
            tester.Leaves = staticLeaves;
            StaticTree.RayCast(&treeRay, &rayData, ref tester);
        }

    }
}
