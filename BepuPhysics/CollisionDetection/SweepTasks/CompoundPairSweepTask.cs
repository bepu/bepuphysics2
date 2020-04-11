using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public class CompoundPairSweepTask<TCompoundA, TCompoundB, TOverlapFinder> : SweepTask
        where TCompoundA : unmanaged, ICompoundShape
        where TCompoundB : unmanaged, ICompoundShape
        where TOverlapFinder : struct, ICompoundPairSweepOverlapFinder<TCompoundA, TCompoundB>
    {
        public CompoundPairSweepTask()
        {
            ShapeTypeIndexA = default(TCompoundA).TypeId;
            ShapeTypeIndexB = default(TCompoundB).TypeId;
        }

        protected override unsafe bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB,
            float maximumT, float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool flipRequired, ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            ref var a = ref Unsafe.AsRef<TCompoundA>(shapeDataA);
            ref var b = ref Unsafe.AsRef<TCompoundB>(shapeDataB);
            t0 = float.MaxValue;
            t1 = float.MaxValue;
            hitLocation = new Vector3();
            hitNormal = new Vector3();
            default(TOverlapFinder).FindOverlaps(
                ref a, orientationA, velocityA,
                ref b, offsetB, orientationB, velocityB, maximumT, shapes, pool, out var overlaps);
            for (int i = 0; i < overlaps.ChildCount; ++i)
            {
                ref var childOverlaps = ref overlaps.GetOverlapsForChild(i);
                ref var childA = ref a.GetChild(childOverlaps.ChildIndex);
                var childTypeA = childA.ShapeIndex.Type;
                shapes[childTypeA].GetShapeData(childA.ShapeIndex.Index, out var childShapeDataA, out _);
                for (int j = 0; j < childOverlaps.Count; ++j)
                {
                    var childIndexB = childOverlaps.Overlaps[j];
                    ref var childB = ref b.GetChild(childIndexB);
                    var childTypeB = childB.ShapeIndex.Type;
                    shapes[childTypeB].GetShapeData(childB.ShapeIndex.Index, out var childShapeDataB, out _);
                    if (filter.AllowTest(
                        flipRequired ? childIndexB : childOverlaps.ChildIndex, 
                        flipRequired ? childOverlaps.ChildIndex : childIndexB))
                    {
                        var task = sweepTasks.GetTask(childTypeA, childTypeB);
                        if (task != null && task.Sweep(
                                childShapeDataA, childTypeA, childA.LocalPose, orientationA, velocityA,
                                childShapeDataB, childTypeB, childB.LocalPose, offsetB, orientationB, velocityB,
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
            throw new NotImplementedException("Compounds cannot be nested; this should never be called.");
        }
    }
}
