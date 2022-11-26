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
    public struct MeshPairOverlapFinder<TMeshA, TMeshB> : ICompoundPairOverlapFinder
        where TMeshA : struct, IHomogeneousCompoundShape<Triangle, TriangleWide>
        where TMeshB : struct, IHomogeneousCompoundShape<Triangle, TriangleWide>
    {

        public unsafe void FindLocalOverlaps(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out CompoundPairOverlaps overlaps)
        {
            var totalCompoundChildCount = 0;
            for (int i = 0; i < pairCount; ++i)
            {
                totalCompoundChildCount += Unsafe.AsRef<TMeshA>(pairs[i].A).ChildCount;
            }
            overlaps = new CompoundPairOverlaps(pool, pairCount, totalCompoundChildCount);
            ref var pairsToTest = ref overlaps.pairQueries;
            int nextSubpairIndex = 0;
            for (int i = 0; i < pairCount; ++i)
            {
                ref var pair = ref pairs[i];
                ref var meshA = ref Unsafe.AsRef<TMeshA>(pair.A);
                overlaps.CreatePairOverlaps(meshA.ChildCount);
                for (int j = 0; j < meshA.ChildCount; ++j)
                {
                    var subpairIndex = nextSubpairIndex++;
                    overlaps.GetOverlapsForPair(subpairIndex).ChildIndex = j;
                    pairsToTest[subpairIndex].Container = pair.B;
                }
            }

            Unsafe.SkipInit(out TriangleWide triangles);
            nextSubpairIndex = 0;
            for (int i = 0; i < pairCount; ++i)
            {
                ref var pair = ref pairs[i];
                Vector3Wide.Broadcast(pair.OffsetB, out var offsetB);
                QuaternionWide.Broadcast(pair.OrientationA, out var orientationA);
                QuaternionWide.Broadcast(pair.OrientationB, out var orientationB);
                Vector3Wide.Broadcast(pair.RelativeLinearVelocityA, out var relativeLinearVelocityA);
                Vector3Wide.Broadcast(pair.AngularVelocityA, out var angularVelocityA);
                Vector3Wide.Broadcast(pair.AngularVelocityB, out var angularVelocityB);
                var maximumAllowedExpansion = new Vector<float>(pair.MaximumExpansion);

                QuaternionWide.Conjugate(orientationB, out var toLocalB);
                QuaternionWide.ConcatenateWithoutOverlap(orientationA, toLocalB, out var localOrientationA);
                QuaternionWide.TransformWithoutOverlap(offsetB, toLocalB, out var localOffsetB);
                Vector3Wide.Negate(localOffsetB, out var localOffsetA);

                ref var meshA = ref Unsafe.AsRef<TMeshA>(pair.A);
                for (int j = 0; j < meshA.ChildCount; j += Vector<float>.Count)
                {
                    var count = meshA.ChildCount - j;
                    if (count > Vector<float>.Count)
                        count = Vector<float>.Count;
                    for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                    {
                        meshA.GetLocalChild(j + innerIndex, ref GatherScatter.GetOffsetInstance(ref triangles, innerIndex));
                    }
                    triangles.GetBounds(ref localOrientationA, count, out var maximumRadius, out var maximumAngularExpansion, out var min, out var max);

                    QuaternionWide.TransformWithoutOverlap(relativeLinearVelocityA, toLocalB, out var localRelativeLinearVelocityA);
                    Vector3Wide.Length(localOffsetA, out var radiusA);
                    BoundingBoxHelpers.ExpandLocalBoundingBoxes(ref min, ref max, radiusA, localOffsetA, localRelativeLinearVelocityA, angularVelocityA, angularVelocityB, dt,
                        maximumRadius, maximumAngularExpansion, maximumAllowedExpansion);

                    for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                    {
                        ref var pairToTest = ref pairsToTest[nextSubpairIndex++];
                        Vector3Wide.ReadSlot(ref min, innerIndex, out pairToTest.Min);
                        Vector3Wide.ReadSlot(ref max, innerIndex, out pairToTest.Max);
                    }
                }
            }

            //Doesn't matter what mesh/compound instance is used for the function; just using it as a source of the function.
            Debug.Assert(totalCompoundChildCount > 0);
            Unsafe.AsRef<TMeshB>(pairsToTest[0].Container).FindLocalOverlaps<CompoundPairOverlaps, ChildOverlapsCollection>(ref pairsToTest, pool, shapes, ref overlaps);

        }

    }
}
