using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    //At the moment, this is basically an unused abstraction. But, if you wanted, this allows you to use a special cased overlap finder in certain cases.
    public interface IConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB> where TShapeA : struct, IConvexShape where TCompoundB : struct, IBoundsQueryableCompound
    {
        unsafe void FindOverlaps(ref TShapeA shapeA, in Quaternion orientationA, in BodyVelocity velocityA,
              ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
              Shapes shapes, BufferPool pool, out ChildOverlapsCollection overlaps);
    }

    public struct ConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB> : IConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB>
        where TShapeA : struct, IConvexShape where TCompoundB : struct, IBoundsQueryableCompound
    {
        public unsafe void FindOverlaps(ref TShapeA shapeA, in Quaternion orientationA, in BodyVelocity velocityA,
            ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, out ChildOverlapsCollection overlaps)
        {
            Quaternion.Conjugate(orientationB, out var inverseOrientationB);
            Quaternion.TransformWithoutOverlap(velocityA.Linear - velocityB.Linear * maximumT, inverseOrientationB, out var sweep);
            Quaternion.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
            Quaternion.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);

            shapeA.ComputeAngularExpansionData(out var maximumRadiusA, out var maximumAngularExpansionA);
            BoundingBoxHelpers.GetAngularBoundsExpansion(velocityA.Angular, maximumT, maximumRadiusA, maximumAngularExpansionA, out var angularExpansionA);
            //We assume a worst case scenario for angular expansion based on the size of the compound. Note that the resulting expansion is applied to the convex;
            //for the purposes of testing, that's equivalent to expanding every compound child.
            compoundB.ComputeAngularExpansionData(out var maximumRadiusB, out var maximumAngularExpansionB);
            BoundingBoxHelpers.GetAngularBoundsExpansion(velocityB.Angular, maximumT, maximumRadiusB, maximumAngularExpansionB, out var angularExpansionB);
            var combinedAngularExpansion = angularExpansionA + angularExpansionB;

            shapeA.ComputeBounds(localOrientationA, out var min, out var max);
            min = min - localOffsetB - combinedAngularExpansion;
            max = max - localOffsetB + combinedAngularExpansion;
            
            overlaps = default;
            compoundB.FindLocalOverlaps<ChildOverlapsCollection>(min, max, sweep, maximumT, pool, shapes, Unsafe.AsPointer(ref overlaps));
        }
    }
}
