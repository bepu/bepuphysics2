using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public unsafe struct PairsToTestForOverlap
    {
        public void* Container;
        public Vector3 Min;
        public Vector3 Max;
    }
    public struct ConvexMeshOverlapFinder<TConvex, TConvexWide> : IConvexCompoundOverlapFinder where TConvex : struct, IConvexShape where TConvexWide : struct, IShapeWide<TConvex>
    {
        public unsafe void FindLocalOverlaps(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out ConvexCompoundTaskOverlaps overlaps)
        {
            overlaps = new ConvexCompoundTaskOverlaps(pool, pairCount);
            var pairsToTest = stackalloc PairsToTestForOverlap[pairCount];

            Vector3Wide offsetB = default;
            QuaternionWide orientationA = default;
            QuaternionWide orientationB = default;
            Vector3Wide relativeLinearVelocityA = default;
            Vector3Wide angularVelocityA = default;
            Vector3Wide angularVelocityB = default;
            Vector<float> maximumAllowedExpansion = default;
            TConvexWide convexWide = default;
            for (int i = 0; i < pairCount; i += Vector<float>.Count)
            {
                var count = pairCount - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;

                //Compute the local bounding boxes using wide operations for the expansion work.
                //Doing quite a bit of gather work (and still quite a bit of scalar work). Very possible that a scalar path could win. TODO: test that.
                for (int j = 0; j < count; ++j)
                {
                    var pairIndex = i + j;
                    ref var pair = ref pairs[pairIndex];
                    pairsToTest[pairIndex].Container = pair.B;
                    Vector3Wide.WriteFirst(pair.OffsetB, ref GatherScatter.GetOffsetInstance(ref offsetB, j));
                    QuaternionWide.WriteFirst(pair.OrientationA, ref GatherScatter.GetOffsetInstance(ref orientationA, j));
                    QuaternionWide.WriteFirst(pair.OrientationB, ref GatherScatter.GetOffsetInstance(ref orientationB, j));
                    Vector3Wide.WriteFirst(pair.RelativeLinearVelocityA, ref GatherScatter.GetOffsetInstance(ref relativeLinearVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityA, ref GatherScatter.GetOffsetInstance(ref angularVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityB, ref GatherScatter.GetOffsetInstance(ref angularVelocityB, j));
                    Unsafe.Add(ref Unsafe.As<Vector<float>, float>(ref maximumAllowedExpansion), j) = pair.MaximumExpansion;

                    GatherScatter.GetOffsetInstance(ref convexWide, j).WriteFirst(ref Unsafe.AsRef<TConvex>(pair.A));
                }

                QuaternionWide.Conjugate(orientationB, out var inverseOrientationB);
                QuaternionWide.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
                QuaternionWide.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);
                QuaternionWide.TransformWithoutOverlap(relativeLinearVelocityA, inverseOrientationB, out var localRelativeLinearVelocityA);

                convexWide.GetBounds(ref localOrientationA, out var maximumRadius, out var maximumAngularExpansion, out var min, out var max);

                Vector3Wide.Negate(localOffsetB, out var localPositionA);
                BoundingBoxHelpers.ExpandLocalBoundingBoxes(ref min, ref max, Vector<float>.Zero, localPositionA, localRelativeLinearVelocityA, angularVelocityA, angularVelocityB, dt,
                    maximumRadius, maximumAngularExpansion, maximumAllowedExpansion);

                for (int j = 0; j < count; ++j)
                {
                    ref var pairToTest = ref pairsToTest[i + j];
                    Vector3Wide.ReadSlot(ref min, j, out pairToTest.Min);
                    Vector3Wide.ReadSlot(ref max, j, out pairToTest.Max);
                }
            }

            //Doesn't matter what mesh instance is used for the function; just using it as a source of the function.
            Unsafe.AsRef<Mesh>(pairsToTest->Container).FindLocalOverlaps<ConvexCompoundTaskOverlaps, ConvexCompoundOverlaps>(pairsToTest, pairCount, pool, ref overlaps);

        }

    }
}
