using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public class ConvexHomogeneousCompoundSweepTask<TConvex, TConvexWide, TCompound, TChildType, TChildTypeWide, TOverlapFinder> : SweepTask
        where TConvex : unmanaged, IConvexShape
        where TConvexWide : unmanaged, IShapeWide<TConvex>
        where TCompound : unmanaged, IHomogeneousCompoundShape<TChildType, TChildTypeWide>
        where TChildType : unmanaged, IConvexShape
        where TChildTypeWide : unmanaged, IShapeWide<TChildType>
        where TOverlapFinder : struct, IConvexCompoundSweepOverlapFinder<TConvex, TCompound>

    {
        public ConvexHomogeneousCompoundSweepTask()
        {
            ShapeTypeIndexA = default(TConvex).TypeId;
            ShapeTypeIndexB = default(TCompound).TypeId;
        }


        protected override unsafe bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool flipRequired, ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            ref var compound = ref Unsafe.AsRef<TCompound>(shapeDataB);
            t0 = float.MaxValue;
            t1 = float.MaxValue;
            hitLocation = new Vector3();
            hitNormal = new Vector3();
            var task = sweepTasks.GetTask(ShapeTypeIndexA, default(TChildType).TypeId);
            if (task != null)
            {
                default(TOverlapFinder).FindOverlaps(ref Unsafe.AsRef<TConvex>(shapeDataA), orientationA, velocityA, ref compound, offsetB, orientationB, velocityB, maximumT, shapes, pool, out var overlaps);
                for (int i = 0; i < overlaps.Count; ++i)
                {
                    var childIndex = overlaps.Overlaps[i];
                    if (filter.AllowTest(flipRequired ? 0 : childIndex, flipRequired ? childIndex : 0))
                    {
                        compound.GetPosedLocalChild(childIndex, out var childShape, out var childPose);
                        if (task.Sweep(
                            shapeDataA, ShapeTypeIndexA, RigidPose.Identity, orientationA, velocityA,
                            Unsafe.AsPointer(ref childShape), Triangle.Id, childPose, offsetB, orientationB, velocityB,
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
            throw new NotImplementedException("Compounds can never be nested; this should never be called.");
        }
    }
}
