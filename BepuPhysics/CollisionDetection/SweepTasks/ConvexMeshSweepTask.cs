using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuPhysics.Collidables;
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

        public override unsafe bool Sweep(
            void* shapeDataA, int shapeTypeA, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in RigidPose localPoseB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            throw new NotImplementedException("Meshes cannot be nested; this should never be called.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void Expand(in Vector3 expansion, ref Vector3 min, ref Vector3 max)
        {
            var minExpansion = Vector3.Min(default, expansion);
            var maxExpansion = Vector3.Max(default, expansion);
            min += minExpansion;
            max += maxExpansion;
        }

        unsafe bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, int shapeTypeA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool flipRequired, ref TSweepFilter filter, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
            where TSweepFilter : ISweepFilter
        {
            Debug.Assert(shapeTypeA == ShapeTypeIndexA && shapeTypeB == ShapeTypeIndexB);
            ref var mesh = ref Unsafe.AsRef<TMesh>(shapeDataB);
            t0 = float.MaxValue;
            t1 = float.MaxValue;
            hitLocation = new Vector3();
            hitNormal = new Vector3();
            var task = sweepTasks.GetTask(shapeTypeA, Triangle.Id);
            if (task != null)
            {
                Quaternion.Conjugate(orientationB, out var inverseOrientationB);
                Quaternion.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
                Quaternion.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);
                Quaternion.TransformWithoutOverlap(velocityA.Linear - velocityB.Linear, inverseOrientationB, out var localRelativeLinearVelocityA);

                ref var shape = ref Unsafe.AsRef<TConvex>(shapeDataA);
                shape.ComputeBounds(localOrientationA, out var min, out var max);
                shape.ComputeAngularExpansionData(out var maximumRadius, out var maximumAngularExpansion);
                //Note that this angular velocity is not in the local space of the mesh. This is simply used to figure out how much local angular expansion to apply to the convex.
                //Consider what happens when two bodies have the same angular velocity- their relative rotation does not change, so there is no need for local angular expansion.
                //The primary bounds expansion only makes use of the magnitude, so the fact that it's not truly in local space is irrelevant.
                var netAngularVelocity = velocityA.Angular - velocityB.Angular;
                BoundingBoxBatcher.GetAngularBoundsExpansion(netAngularVelocity, maximumT, maximumRadius, maximumAngularExpansion, out var angularExpansion);
                min += angularExpansion;
                max += angularExpansion;

                //If any mesh/compound in the batch has angular velocity, we need to compute the bounding box expansion caused by the resulting nonlinear path.
                //(This is equivalent to expanding the bounding boxes of the mesh/compound shapes to account for their motion. It's just much simpler to expand only the incoming convex.
                //Conceptually, you can think of this as if we're fixing our frame of reference on the mesh/compound, and watching how the convex moves. 
                //In the presence of mesh/compound angular velocity, a stationary convex will trace a circular arc.)
                var angularSpeedBSquared = Vector3.Dot(velocityB.Angular, velocityB.Angular);
                if (angularSpeedBSquared > 0)
                {
                    //We need to expand the bounding box by the extent of the circular arc which the convex traces due to the mesh/compound's angular motion.
                    //We'll create two axes and measure the extent of the arc along them.
                    //Note that arcX and arcY are invalid if radius or angular velocity magnitude is zero. We'll handle that with a mask.
                    var radius = offsetB.Length();
                    var arcX = offsetB / radius;
                    Vector3x.Cross(velocityB.Angular, arcX, out var arcY);
                    arcY /= arcY.Length();
                    var angularSpeedB = (float)Math.Sqrt(angularSpeedBSquared);
                    var angularDisplacement = angularSpeedB * maximumT;
                    //minX is just 0 because of the chosen frame of reference.
                    var maxX = MathHelper.Cos(MathHelper.Min(MathHelper.Pi, angularDisplacement));
                    var sinTheta = MathHelper.Sin(angularDisplacement);
                    var minY = MathHelper.Min(sinTheta, 0);
                    var maxY = MathHelper.Sin(MathHelper.Min(angularDisplacement, MathHelper.PiOver2));

                    var expansionMaxX = arcX * maxX;
                    var expansionMinY = arcY * minY;
                    var expansionMaxY = arcY * maxY;
                    Expand(expansionMaxX, ref min, ref max);
                    Expand(expansionMinY, ref min, ref max);
                    Expand(expansionMaxY, ref min, ref max);
                    //TODO: Convexes that belong to a compound will also need to include expansion caused by the child motion.
                }
                min -= offsetB;
                max -= offsetB;

                QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), 128, out var childIndices);
                mesh.FindLocalOverlaps(min, max, localRelativeLinearVelocityA, maximumT, pool, ref childIndices);
                for (int i = 0; i < childIndices.Count; ++i)
                {
                    var childIndex = childIndices[i];
                    if (filter.AllowTest(flipRequired ? 0 : childIndex, flipRequired ? childIndex : 0))
                    {
                        mesh.GetLocalTriangle(childIndex, out var triangle);
                        var triangleCenter = (triangle.A + triangle.B + triangle.C) * (1f / 3f);
                        triangle.A -= triangleCenter;
                        triangle.B -= triangleCenter;
                        triangle.C -= triangleCenter;
                        if (task.Sweep(
                            shapeDataA, shapeTypeA, new RigidPose(Vector3.Zero, Quaternion.Identity), orientationA, velocityA,
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
                childIndices.Dispose(pool.SpecializeFor<int>());
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
                    flipRequired, ref filter, sweepTasks, pool,
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
                    flipRequired, ref filter, sweepTasks, pool,
                    out t0, out t1, out hitLocation, out hitNormal);
            }
        }
    }
}
