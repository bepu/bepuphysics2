using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    unsafe struct SubpairData
    {
        public BoundsTestedPair* Pair;
        public CompoundChild* Child;
    }
    public struct CompoundPairOverlapFinder<TCompoundA, TCompoundB> : ICompoundPairOverlapFinder
        where TCompoundA : struct, ICompoundShape
        where TCompoundB : struct, IBoundsQueryableCompound
    {

        public unsafe void FindLocalOverlaps(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out CompoundPairOverlaps overlaps)
        {
            var totalCompoundChildCount = 0;
            for (int i = 0; i < pairCount; ++i)
            {
                totalCompoundChildCount += Unsafe.AsRef<TCompoundA>(pairs[i].A).ChildCount;
            }
            overlaps = new CompoundPairOverlaps(pool, pairCount, totalCompoundChildCount);
            ref var pairsToTest = ref overlaps.pairQueries;
            var subpairData = stackalloc SubpairData[totalCompoundChildCount];
            int nextSubpairIndex = 0;
            for (int i = 0; i < pairCount; ++i)
            {
                ref var pair = ref pairs[i];
                ref var compoundA = ref Unsafe.AsRef<TCompoundA>(pair.A);
                overlaps.CreatePairOverlaps(compoundA.ChildCount);
                for (int j = 0; j < compoundA.ChildCount; ++j)
                {
                    var subpairIndex = nextSubpairIndex++;
                    overlaps.GetOverlapsForPair(subpairIndex).ChildIndex = j;
                    pairsToTest[subpairIndex].Container = pair.B;
                    ref var subpair = ref subpairData[subpairIndex];
                    subpair.Pair = (BoundsTestedPair*)Unsafe.AsPointer(ref pair);
                    subpair.Child = (CompoundChild*)Unsafe.AsPointer(ref compoundA.GetChild(j));
                }
            }

            Unsafe.SkipInit(out Vector3Wide offsetB);
            Unsafe.SkipInit(out QuaternionWide orientationA);
            Unsafe.SkipInit(out QuaternionWide orientationB);
            Unsafe.SkipInit(out Vector3Wide relativeLinearVelocityA);
            Unsafe.SkipInit(out Vector3Wide angularVelocityA);
            Unsafe.SkipInit(out Vector3Wide angularVelocityB);
            Unsafe.SkipInit(out Vector<float> maximumAllowedExpansion);
            Unsafe.SkipInit(out Vector<float> maximumRadius);
            Unsafe.SkipInit(out Vector<float> maximumAngularExpansion);
            Unsafe.SkipInit(out RigidPoseWide localPosesA);
            Unsafe.SkipInit(out Vector3Wide mins);
            Unsafe.SkipInit(out Vector3Wide maxes);
            for (int i = 0; i < totalCompoundChildCount; i += Vector<float>.Count)
            {
                var count = totalCompoundChildCount - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;

                //Compute the local bounding boxes using wide operations for the expansion work.
                //Doing quite a bit of gather work (and still quite a bit of scalar work). Very possible that a scalar path could win. TODO: test that.
                for (int j = 0; j < count; ++j)
                {
                    var subpairIndex = i + j;
                    ref var subpair = ref subpairData[subpairIndex];
                    Vector3Wide.WriteFirst(subpair.Pair->OffsetB, ref GatherScatter.GetOffsetInstance(ref offsetB, j));
                    QuaternionWide.WriteFirst(subpair.Pair->OrientationA, ref GatherScatter.GetOffsetInstance(ref orientationA, j));
                    QuaternionWide.WriteFirst(subpair.Pair->OrientationB, ref GatherScatter.GetOffsetInstance(ref orientationB, j));
                    Vector3Wide.WriteFirst(subpair.Pair->RelativeLinearVelocityA, ref GatherScatter.GetOffsetInstance(ref relativeLinearVelocityA, j));
                    Vector3Wide.WriteFirst(subpair.Pair->AngularVelocityA, ref GatherScatter.GetOffsetInstance(ref angularVelocityA, j));
                    Vector3Wide.WriteFirst(subpair.Pair->AngularVelocityB, ref GatherScatter.GetOffsetInstance(ref angularVelocityB, j));
                    Unsafe.Add(ref Unsafe.As<Vector<float>, float>(ref maximumAllowedExpansion), j) = subpair.Pair->MaximumExpansion;

                    RigidPoseWide.WriteFirst(subpair.Child->LocalPose, ref GatherScatter.GetOffsetInstance(ref localPosesA, j));
                }

                QuaternionWide.Conjugate(orientationB, out var toLocalB);
                QuaternionWide.ConcatenateWithoutOverlap(orientationA, toLocalB, out var localOrientationsA);
                QuaternionWide.ConcatenateWithoutOverlap(localPosesA.Orientation, localOrientationsA, out var localChildOrientationsA);
                QuaternionWide.TransformWithoutOverlap(localPosesA.Position, localOrientationsA, out var localOffsetA);
                QuaternionWide.TransformWithoutOverlap(offsetB, toLocalB, out var localOffsetB);
                Vector3Wide.Subtract(localOffsetA, localOffsetB, out var localPositionsA);

                for (int j = 0; j < count; ++j)
                {
                    var shapeIndex = subpairData[i + j].Child->ShapeIndex;
                    QuaternionWide.ReadFirst(GatherScatter.GetOffsetInstance(ref localChildOrientationsA, j), out var localChildOrientationA);
                    shapes[shapeIndex.Type].ComputeBounds(shapeIndex.Index, localChildOrientationA,
                        out GatherScatter.Get(ref maximumRadius, j),
                        out GatherScatter.Get(ref maximumAngularExpansion, j), out var min, out var max);
                    Vector3Wide.WriteFirst(min, ref GatherScatter.GetOffsetInstance(ref mins, j));
                    Vector3Wide.WriteFirst(max, ref GatherScatter.GetOffsetInstance(ref maxes, j));

                }

                QuaternionWide.TransformWithoutOverlap(relativeLinearVelocityA, toLocalB, out var localRelativeLinearVelocityA);
                Vector3Wide.Length(localOffsetA, out var radiusA);
                BoundingBoxHelpers.ExpandLocalBoundingBoxes(ref mins, ref maxes, radiusA, localPositionsA, localRelativeLinearVelocityA, angularVelocityA, angularVelocityB, dt,
                    maximumRadius, maximumAngularExpansion, maximumAllowedExpansion);

                for (int j = 0; j < count; ++j)
                {
                    ref var pairToTest = ref pairsToTest[i + j];
                    Vector3Wide.ReadSlot(ref mins, j, out pairToTest.Min);
                    Vector3Wide.ReadSlot(ref maxes, j, out pairToTest.Max);
                }
            }
            //Doesn't matter what mesh/compound instance is used for the function; just using it as a source of the function.
            Debug.Assert(totalCompoundChildCount > 0);
            Unsafe.AsRef<TCompoundB>(pairsToTest[0].Container).FindLocalOverlaps<CompoundPairOverlaps, ChildOverlapsCollection>(ref pairsToTest, pool, shapes, ref overlaps);

        }

    }
}
