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
    public struct CompoundMeshOverlapFinder : ICompoundMeshOverlapFinder<Compound, Mesh>
    {
        unsafe struct SubpairData
        {
            public BoundsTestedPair* Pair;
            public CompoundChild* Child;
        }
        public unsafe void FindLocalOverlaps(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out TaskOverlapsCollection overlaps)
        {
            var totalCompoundChildCount = 0;
            for (int i = 0; i < pairCount; ++i)
            {
                totalCompoundChildCount += Unsafe.AsRef<Compound>(pairs[i].A).Children.Length;
            }
            overlaps = new TaskOverlapsCollection(pool, pairCount, totalCompoundChildCount);
            var pairsToTest = stackalloc PairsToTestForOverlap[totalCompoundChildCount];
            var subpairData = stackalloc SubpairData[totalCompoundChildCount];
            int nextSubpairIndex = 0;
            for (int i = 0; i < pairCount; ++i)
            {
                ref var pair = ref pairs[i];
                ref var compound = ref Unsafe.AsRef<Compound>(pair.A);
                ref var mesh = ref Unsafe.AsRef<Mesh>(pair.B);
                overlaps.CreatePairOverlaps(compound.Children.Length, pool);
                for (int j = 0; j < compound.Children.Length; ++j)
                {
                    var subpairIndex = nextSubpairIndex++;
                    overlaps.GetChildOverlaps(subpairIndex).ChildIndex = j;
                    pairsToTest[subpairIndex].Container = pair.B;
                    ref var subpair = ref subpairData[subpairIndex];
                    subpair.Pair = (BoundsTestedPair*)Unsafe.AsPointer(ref pair);
                    subpair.Child = (CompoundChild*)Unsafe.AsPointer(ref compound.Children[j]);
                }
            }

            Vector3Wide offsetB = default;
            QuaternionWide orientationA = default;
            QuaternionWide orientationB = default;
            Vector3Wide relativeLinearVelocityA = default;
            Vector3Wide angularVelocityA = default;
            Vector3Wide angularVelocityB = default;
            Vector<float> maximumAllowedExpansion = default;
            Vector<float> maximumRadius = default;
            Vector<float> maximumAngularExpansion = default;
            RigidPoses localPosesA;
            Vector3Wide mins = default;
            Vector3Wide maxes = default;
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

                    RigidPoses.WriteFirst(subpair.Child->LocalPose, ref GatherScatter.GetOffsetInstance(ref localPosesA, j));
                }

                QuaternionWide.ConcatenateWithoutOverlap(localPosesA.Orientation, orientationA, out var childOrientationA);
                QuaternionWide.TransformWithoutOverlap(localPosesA.Position, orientationA, out var childPositionA);
                Vector3Wide.Subtract(childPositionA, offsetB, out childPositionA);
                QuaternionWide.Conjugate(orientationB, out var toLocalB);
                QuaternionWide.TransformWithoutOverlap(childPositionA, toLocalB, out var localPositionsA);
                QuaternionWide.ConcatenateWithoutOverlap(childOrientationA, toLocalB, out var localOrientationsA);

                for (int j = 0; j < count; ++j)
                {
                    var shapeIndex = subpairData[i + j].Child->ShapeIndex;
                    Vector3Wide.ReadSlot(ref localPositionsA, j, out var localPositionA);
                    QuaternionWide.ReadFirst(GatherScatter.GetOffsetInstance(ref localOrientationsA, j), out var localOrientationA);
                    shapes[shapeIndex.Type].ComputeBounds(shapeIndex.Index, localOrientationA,
                        out GatherScatter.Get(ref maximumRadius, j),
                        out GatherScatter.Get(ref maximumAngularExpansion, j), out var min, out var max);
                    Vector3Wide.WriteFirst(min, ref GatherScatter.GetOffsetInstance(ref mins, j));
                    Vector3Wide.WriteFirst(max, ref GatherScatter.GetOffsetInstance(ref maxes, j));

                }

                BoundingBoxHelpers.ExpandLocalBoundingBoxes(ref mins, ref maxes, localPositionsA, localOrientationsA, toLocalB, relativeLinearVelocityA, angularVelocityA,
                    angularVelocityB, dt, maximumRadius, maximumAngularExpansion, maximumAllowedExpansion);
            }

            //Doesn't matter what mesh instance is used for the function; just using it as a source of the function.
            Debug.Assert(totalCompoundChildCount > 0);
            Unsafe.AsRef<Mesh>(pairsToTest->Container).FindLocalOverlaps(pairsToTest, totalCompoundChildCount, pool, ref overlaps);

        }
    }
}
