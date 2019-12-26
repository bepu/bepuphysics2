using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Defines a type that can act as a callback for broad phase sweep tests.
    /// </summary>
    public interface IBroadPhaseSweepTester
    {
        unsafe void Test(CollidableReference collidable, ref float maximumT);
    }

    partial class BroadPhase
    {
        struct RayLeafTester<TRayTester> : IRayLeafTester where TRayTester : IBroadPhaseRayTester
        {
            public TRayTester LeafTester;
            public Buffer<CollidableReference> Leaves;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void TestLeaf(int leafIndex, RayData* rayData, float* maximumT)
            {
                LeafTester.RayTest(Leaves[leafIndex], rayData, maximumT);
            }
        }

        /// <summary>
        /// Finds any intersections between a ray and leaf bounding boxes.
        /// </summary>
        /// <typeparam name="TRayTester">Type of the callback to execute on ray-leaf bounding box intersections.</typeparam>
        /// <param name="origin">Origin of the ray to cast.</param>
        /// <param name="direction">Direction of the ray to cast.</param>
        /// <param name="maximumT">Maximum length of the ray traversal in units of the direction's length.</param>
        /// <param name="rayTester">Callback to execute on ray-leaf bounding box intersections.</param>
        /// <param name="id">User specified id of the ray.</param>
        public unsafe void RayCast<TRayTester>(in Vector3 origin, in Vector3 direction, float maximumT, ref TRayTester rayTester, int id = 0) where TRayTester : IBroadPhaseRayTester
        {
            TreeRay.CreateFrom(origin, direction, maximumT, id, out var rayData, out var treeRay);
            RayLeafTester<TRayTester> tester;
            tester.LeafTester = rayTester;
            tester.Leaves = activeLeaves;
            ActiveTree.RayCast(&treeRay, &rayData, ref tester);
            tester.Leaves = staticLeaves;
            StaticTree.RayCast(&treeRay, &rayData, ref tester);
            //The sweep tester probably relies on mutation to function; copy any mutations back to the original reference.
            rayTester = tester.LeafTester;
        }

        struct SweepLeafTester<TSweepTester> : ISweepLeafTester where TSweepTester : IBroadPhaseSweepTester
        {
            public TSweepTester LeafTester;
            public Buffer<CollidableReference> Leaves;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void TestLeaf(int leafIndex, ref float maximumT)
            {
                LeafTester.Test(Leaves[leafIndex], ref maximumT);
            }
            
        }

        /// <summary>
        /// Finds any intersections between a swept bounding box and leaf bounding boxes.
        /// </summary>
        /// <typeparam name="TSweepTester">Type of the callback to execute on sweep-leaf bounding box intersections.</typeparam>
        /// <param name="min">Minimum bounds of the box to sweep.</param>
        /// <param name="max">Maximum bounds of the box to sweep.</param>
        /// <param name="direction">Direction along which to sweep the bounding box.</param>
        /// <param name="maximumT">Maximum length of the sweep in units of the direction's length.</param>
        /// <param name="sweepTester">Callback to execute on sweep-leaf bounding box intersections.</param>
        public unsafe void Sweep<TSweepTester>(in Vector3 min, in Vector3 max, in Vector3 direction, float maximumT, ref TSweepTester sweepTester) where TSweepTester : IBroadPhaseSweepTester
        {
            Tree.ConvertBoxToCentroidWithExtent(min, max, out var origin, out var expansion);
            TreeRay.CreateFrom(origin, direction, maximumT, out var treeRay);
            SweepLeafTester<TSweepTester> tester;
            tester.LeafTester = sweepTester;
            tester.Leaves = activeLeaves;
            ActiveTree.Sweep(expansion, origin, direction, &treeRay, ref tester);
            tester.Leaves = staticLeaves;
            StaticTree.Sweep(expansion, origin, direction, &treeRay, ref tester);
            //The sweep tester probably relies on mutation to function; copy any mutations back to the original reference.
            sweepTester = tester.LeafTester;
        }

        /// <summary>
        /// Finds any intersections between a swept bounding box and leaf bounding boxes.
        /// </summary>
        /// <typeparam name="TSweepTester">Type of the callback to execute on sweep-leaf bounding box intersections.</typeparam>
        /// <param name="boundingBox">Bounding box to sweep.</param>
        /// <param name="direction">Direction along which to sweep the bounding box.</param>
        /// <param name="maximumT">Maximum length of the sweep in units of the direction's length.</param>
        /// <param name="sweepTester">Callback to execute on sweep-leaf bounding box intersections.</param>
        public unsafe void Sweep<TSweepTester>(in BoundingBox boundingBox, in Vector3 direction, float maximumT, ref TSweepTester sweepTester) where TSweepTester : IBroadPhaseSweepTester
        {
            Sweep(boundingBox.Min, boundingBox.Max, direction, maximumT, ref sweepTester);
        }

        struct BoxQueryEnumerator<TInnerEnumerator> : IBreakableForEach<int> where TInnerEnumerator : IBreakableForEach<CollidableReference>
        {
            public TInnerEnumerator Enumerator;
            public Buffer<CollidableReference> Leaves;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool LoopBody(int i)
            {
                return Enumerator.LoopBody(Leaves[i]);
            }
        }

        /// <summary>
        /// Finds any overlaps between a bounding box and leaf bounding boxes.
        /// </summary>
        /// <typeparam name="TOverlapEnumerator">Type of the enumerator to call for overlaps.</typeparam>
        /// <param name="min">Minimum bounds of the query box.</param>
        /// <param name="max">Maximum bounds of the query box.</param>
        /// <param name="overlapEnumerator">Enumerator to call for overlaps.</param>
        public unsafe void GetOverlaps<TOverlapEnumerator>(in Vector3 min, in Vector3 max, ref TOverlapEnumerator overlapEnumerator) where TOverlapEnumerator : IBreakableForEach<CollidableReference>
        {
            BoxQueryEnumerator<TOverlapEnumerator> enumerator;
            enumerator.Enumerator = overlapEnumerator;
            enumerator.Leaves = activeLeaves;
            ActiveTree.GetOverlaps(min, max, ref enumerator);
            enumerator.Leaves = staticLeaves;
            StaticTree.GetOverlaps(min, max, ref enumerator);
            //Enumeration could have mutated the enumerator; preserve those modifications.
            overlapEnumerator = enumerator.Enumerator;
        }

        /// <summary>
        /// Finds any overlaps between a bounding box and leaf bounding boxes.
        /// </summary>
        /// <typeparam name="TOverlapEnumerator">Type of the enumerator to call for overlaps.</typeparam>
        /// <param name="boundingBox">Query box bounds.</param>
        /// <param name="overlapEnumerator">Enumerator to call for overlaps.</param>
        public unsafe void GetOverlaps<TOverlapEnumerator>(in BoundingBox boundingBox, ref TOverlapEnumerator overlapEnumerator) where TOverlapEnumerator : IBreakableForEach<CollidableReference>
        {
            BoxQueryEnumerator<TOverlapEnumerator> enumerator;
            enumerator.Enumerator = overlapEnumerator;
            enumerator.Leaves = activeLeaves;
            ActiveTree.GetOverlaps(boundingBox, ref enumerator);
            enumerator.Leaves = staticLeaves;
            StaticTree.GetOverlaps(boundingBox, ref enumerator);
            //Enumeration could have mutated the enumerator; preserve those modifications.
            overlapEnumerator = enumerator.Enumerator;
        }
    }
}
