using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    //At the moment, this is basically an unused abstraction. But, if you wanted, this allows you to use a special cased overlap finder in certain cases.
    public interface IConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB> where TShapeA : struct, IConvexShape where TCompoundB : struct, IBoundsQueryableCompound
    {
        static abstract void FindOverlaps(ref TShapeA shapeA, Quaternion orientationA, in BodyVelocity velocityA,
              ref TCompoundB compoundB, Vector3 offsetB, Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
              Shapes shapes, BufferPool pool, out ChildOverlapsCollection overlaps);
    }

    public struct ConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB> : IConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB>
        where TShapeA : struct, IConvexShape where TCompoundB : struct, IBoundsQueryableCompound
    {
        public static unsafe void FindOverlaps(ref TShapeA shapeA, Quaternion orientationA, in BodyVelocity velocityA,
            ref TCompoundB compoundB, Vector3 offsetB, Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, out ChildOverlapsCollection overlaps)
        {
            BoundingBoxHelpers.GetLocalBoundingBoxForSweep(ref shapeA, orientationA, velocityA, offsetB, orientationB, velocityB, maximumT, out var sweep, out var min, out var max);
            
            overlaps = default;
            //The sweep is already the displacement over maximumT, so the overlap query's ray covers it at t = 1. Passing maximumT would scale the displacement by maximumT again.
            compoundB.FindLocalOverlaps<ChildOverlapsCollection>(min, max, sweep, 1, pool, shapes, Unsafe.AsPointer(ref overlaps));
        }
    }
}
