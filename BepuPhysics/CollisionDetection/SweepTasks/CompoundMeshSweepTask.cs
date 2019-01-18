using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public class CompoundMeshSweepTask<TCompound, TMesh, TOverlapFinder> : SweepTask
        where TCompound : struct, ICompoundShape
        where TMesh : struct, IHomogeneousCompoundShape<Triangle, TriangleWide>
        where TOverlapFinder : struct, ICompoundPairSweepOverlapFinder<TCompound, TMesh>
    {
        public CompoundMeshSweepTask()
        {
            ShapeTypeIndexA = default(TCompound).TypeId;
            ShapeTypeIndexB = default(TMesh).TypeId;
        }

        protected unsafe override bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB,
            float maximumT, float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool flipRequired, ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            ref var mesh = ref Unsafe.AsRef<TMesh>(shapeDataB);
            TOverlapFinder overlapFinder = default;
            t0 = float.MaxValue;
            t1 = float.MaxValue;
            hitLocation = new Vector3();
            hitNormal = new Vector3();
            ref var compound = ref Unsafe.AsRef<TCompound>(shapeDataA);
            overlapFinder.FindOverlaps(ref compound, orientationA, velocityA, ref mesh, offsetB, orientationB, velocityB, maximumT, shapes, pool, out var overlaps);
            for (int i = 0; i < overlaps.ChildCount; ++i)
            {
                ref var childOverlaps = ref overlaps.GetOverlapsForChild(i);
                for (int j = 0; j < childOverlaps.Count; ++j)
                {
                    var triangleIndex = childOverlaps.Overlaps[j];
                    if (filter.AllowTest(flipRequired ? triangleIndex : childOverlaps.ChildIndex, flipRequired ? childOverlaps.ChildIndex : triangleIndex))
                    {
                        mesh.GetLocalChild(triangleIndex, out var triangle);
                        ref var compoundChild = ref compound.GetChild(childOverlaps.ChildIndex);
                        var compoundChildType = compoundChild.ShapeIndex.Type;
                        var task = sweepTasks.GetTask(compoundChildType, Triangle.Id);
                        var triangleCenter = (triangle.A + triangle.B + triangle.C) * (1f / 3f);
                        triangle.A -= triangleCenter;
                        triangle.B -= triangleCenter;
                        triangle.C -= triangleCenter;
                        shapes[compoundChildType].GetShapeData(compoundChild.ShapeIndex.Index, out var compoundChildShapeData, out _);
                        if (task.Sweep(
                            compoundChildShapeData, compoundChildType, compoundChild.LocalPose, orientationA, velocityA,
                            Unsafe.AsPointer(ref triangle), Triangle.Id, new RigidPose(triangleCenter, Quaternion.Identity), offsetB, orientationB, velocityB,
                            maximumT, minimumProgression, convergenceThreshold, maximumIterationCount,
                            out var t0Candidate, out var t1Candidate, out var hitLocationCandidate, out var hitNormalCandidate))
                        {
                            //Note that we use t1 to determine whether to accept the new location. In other words, we're choosing to keep sweeps that have the earliest time of intersection.
                            //(t0 is *not* intersecting for any initially separated pair.)
                            if (t1Candidate < t1)
                            {
                                t0 = t0Candidate;
                                t1 = t1Candidate;
                                hitLocation = hitLocationCandidate;
                                hitNormal = hitNormalCandidate;
                            }
                        }
                    }
                }
            }
            overlaps.Dispose(pool);
            return t1 < float.MaxValue;
        }

        protected override unsafe bool PreorderedTypeSweep(void* shapeDataA, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA, void* shapeDataB, in RigidPose localPoseB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT, float minimumProgression, float convergenceThreshold, int maximumIterationCount, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            throw new NotImplementedException("Compounds and meshes can never be nested; this should never be called.");
        }
    }
}
