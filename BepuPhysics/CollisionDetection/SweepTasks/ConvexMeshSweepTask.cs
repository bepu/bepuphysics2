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
    public class ConvexMeshSweepTask<TConvex, TConvexWide, TMesh> : SweepTask
        where TConvex : struct, IConvexShape
        where TConvexWide : struct, IShapeWide<TConvex>
        where TMesh : IMeshShape
    {
        public ConvexMeshSweepTask()
        {
            ShapeTypeIndexA = default(TConvex).TypeId;
            ShapeTypeIndexB = default(TMesh).TypeId;
        }


        protected override unsafe bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool flipRequired, ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            ref var mesh = ref Unsafe.AsRef<TMesh>(shapeDataB);
            t0 = float.MaxValue;
            t1 = float.MaxValue;
            hitLocation = new Vector3();
            hitNormal = new Vector3();
            var task = sweepTasks.GetTask(ShapeTypeIndexA, Triangle.Id);
            if (task != null)
            {
                BoundingBoxHelpers.GetBoundingBoxForSweep(ref Unsafe.AsRef<TConvex>(shapeDataA), orientationA, velocityA, offsetB, orientationB, velocityB, maximumT,
                    out var sweep, out var min, out var max);
                
                ChildOverlapsCollection overlaps = default;
                mesh.FindLocalOverlaps<ChildOverlapsCollection>(min, max, sweep, maximumT, pool, shapes, Unsafe.AsPointer(ref overlaps));
                for (int i = 0; i < overlaps.Count; ++i)
                {
                    var childIndex = overlaps.Overlaps[i];
                    if (filter.AllowTest(flipRequired ? 0 : childIndex, flipRequired ? childIndex : 0))
                    {
                        mesh.GetLocalTriangle(childIndex, out var triangle);
                        var triangleCenter = (triangle.A + triangle.B + triangle.C) * (1f / 3f);
                        triangle.A -= triangleCenter;
                        triangle.B -= triangleCenter;
                        triangle.C -= triangleCenter;
                        if (task.Sweep(
                            shapeDataA, ShapeTypeIndexA, new RigidPose(Vector3.Zero, Quaternion.Identity), orientationA, velocityA,
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
                overlaps.Dispose(pool);
            }
            return t1 < float.MaxValue;
        }

        protected override unsafe bool PreorderedTypeSweep(void* shapeDataA, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA, void* shapeDataB, in RigidPose localPoseB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT, float minimumProgression, float convergenceThreshold, int maximumIterationCount, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            throw new NotImplementedException("Meshes can never be nested; this should never be called.");
        }
    }
}
