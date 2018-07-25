using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public class CompoundConvexSweepTask<TShapeB, TShapeWideB> : SweepTask
              where TShapeB : struct, IConvexShape
              where TShapeWideB : struct, IShapeWide<TShapeB>
    {
        public CompoundConvexSweepTask()
        {
            ShapeTypeIndexA = default(Compound).TypeId;
            ShapeTypeIndexB = default(TShapeB).TypeId;
        }

        public override unsafe bool Sweep(
            void* shapeDataA, int shapeTypeA, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in RigidPose localPoseB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            throw new NotImplementedException("Compounds cannot be nested; this should never be called.");
        }
        static unsafe bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, int shapeTypeA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool flipRequired, ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
            where TSweepFilter : ISweepFilter
        {
            ref var compound = ref Unsafe.AsRef<Compound>(shapeDataA);
            t0 = float.MaxValue;
            t1 = float.MaxValue;
            hitLocation = new Vector3();
            hitNormal = new Vector3();
            for (int i = 0; i < compound.Children.Length; ++i)
            {
                if (filter.AllowTest(flipRequired ? 0 : i, flipRequired ? i : 0))
                {
                    ref var child = ref compound.Children[i];
                    var childType = child.ShapeIndex.Type;
                    shapes[childType].GetShapeData(child.ShapeIndex.Index, out var childShapeData, out _);
                    var task = sweepTasks.GetTask(childType, shapeTypeB);
                    if (task != null && task.Sweep(
                        childShapeData, childType, child.LocalPose, orientationA, velocityA,
                        shapeDataB, shapeTypeB, new RigidPose() { Orientation = Quaternion.Identity }, offsetB, orientationB, velocityB,
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
            return t1 < float.MaxValue;
        }
        public override unsafe bool Sweep<TSweepFilter>(
            void* shapeDataA, int shapeTypeA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            Debug.Assert((shapeTypeA == ShapeTypeIndexA && shapeTypeB == ShapeTypeIndexB) || (shapeTypeA == ShapeTypeIndexA && shapeTypeB == ShapeTypeIndexB),
                "Types must match expected types.");
            var flipRequired = shapeTypeB == ShapeTypeIndexA;
            if (flipRequired)
            {
                var hit = PreorderedTypeSweep(
                    shapeDataB, shapeTypeB, orientationB, velocityB,
                    shapeDataA, shapeTypeA, -offsetB, orientationA, velocityA,
                    maximumT, minimumProgression, convergenceThreshold, maximumIterationCount,
                    flipRequired, ref filter, shapes, sweepTasks,
                    out t0, out t1, out hitLocation, out hitNormal);
                hitNormal = -hitNormal;
                hitLocation = hitLocation + offsetB;
                return hit;
            }
            else
            {
                return PreorderedTypeSweep(
                    shapeDataA, shapeTypeA, orientationA, velocityA,
                    shapeDataB, shapeTypeB, offsetB, orientationB, velocityB,
                    maximumT, minimumProgression, convergenceThreshold, maximumIterationCount,
                    flipRequired, ref filter, shapes, sweepTasks,
                    out t0, out t1, out hitLocation, out hitNormal);
            }
        }
    }
}
