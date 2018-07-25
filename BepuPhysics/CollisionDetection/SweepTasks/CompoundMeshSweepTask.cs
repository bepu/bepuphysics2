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
    public interface ICompoundMeshSweepOverlapFinder<TCompound, TMesh> where TCompound : ICompoundShape where TMesh : IMeshShape
    {
        void FindOverlaps(ref TCompound compound, in Quaternion compoundOrientation, in BodyVelocity compoundVelocity,
            ref TMesh mesh, in Vector3 meshOffset, in Quaternion meshOrientation, in BodyVelocity meshVelocity, BufferPool pool, ref QuickList<(int, int), Buffer<(int, int)>> childOverlaps);
    }
    public class CompoundMeshSweepTask<TCompound, TMesh, TOverlapFinder> : SweepTask
        where TCompound : struct, ICompoundShape
        where TMesh : IMeshShape
        where TOverlapFinder : ICompoundMeshSweepOverlapFinder<TCompound, TMesh>
    {
        public CompoundMeshSweepTask()
        {
            ShapeTypeIndexA = default(TCompound).TypeId;
            ShapeTypeIndexB = default(TMesh).TypeId;
        }

        public override unsafe bool Sweep(
            void* shapeDataA, int shapeTypeA, in RigidPose localPoseA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in RigidPose localPoseB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
        {
            throw new NotImplementedException("Meshes and compounds cannot be nested; this should never be called.");
        }


        unsafe bool PreorderedTypeSweep<TSweepFilter>(
            void* shapeDataA, int shapeTypeA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            bool flipRequired, ref TSweepFilter filter, Shapes shapes, SweepTaskRegistry sweepTasks, BufferPool pool, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal)
            where TSweepFilter : ISweepFilter
        {
            Debug.Assert(shapeTypeA == ShapeTypeIndexA && shapeTypeB == ShapeTypeIndexB);
            ref var mesh = ref Unsafe.AsRef<TMesh>(shapeDataB);
            TOverlapFinder overlapFinder = default;
            t0 = float.MaxValue;
            t1 = float.MaxValue;
            hitLocation = new Vector3();
            hitNormal = new Vector3();
            ref var compound = ref Unsafe.AsRef<TCompound>(shapeDataA);
            QuickList<(int CompoundChildIndex, int MeshTriangleIndex), Buffer<(int, int)>>.Create(pool.SpecializeFor<(int, int)>(), 128, out var childOverlaps);
            overlapFinder.FindOverlaps(ref compound, orientationA, velocityA, ref mesh, offsetB, orientationB, velocityB, pool, ref childOverlaps);
            for (int i = 0; i < childOverlaps.Count; ++i)
            {
                ref var overlap = ref childOverlaps[i];
                if (filter.AllowTest(flipRequired ? overlap.MeshTriangleIndex : overlap.CompoundChildIndex, flipRequired ? overlap.CompoundChildIndex : overlap.MeshTriangleIndex))
                {
                    mesh.GetLocalTriangle(overlap.MeshTriangleIndex, out var triangle);
                    ref var compoundChild = ref compound.GetChild(overlap.CompoundChildIndex);
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
            childOverlaps.Dispose(pool.SpecializeFor<(int, int)>());

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
                    flipRequired, ref filter, shapes, sweepTasks, pool,
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
                    flipRequired, ref filter, shapes, sweepTasks, pool,
                    out t0, out t1, out hitLocation, out hitNormal);
            }
        }
    }
}
