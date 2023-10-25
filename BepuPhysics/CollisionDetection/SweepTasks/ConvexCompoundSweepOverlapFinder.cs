using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    //At the moment, this is basically an unused abstraction. But, if you wanted, this allows you to use a special cased overlap finder in certain cases.
    public interface IConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB> where TShapeA : struct, IConvexShape where TCompoundB : struct, IBoundsQueryableCompound
    {
        unsafe void FindOverlaps(ref TShapeA shapeA, in Quaternion orientationA, in BodyVelocity velocityA,
              ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
              Shapes shapes, BufferPool pool, out ChildOverlapsCollection overlaps);

        unsafe void FindOverlaps<TLeafTester>(ref TShapeA shapeA, in Quaternion orientationA, in BodyVelocity velocityA,
            ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, ref TLeafTester leafTester)
            where TLeafTester : ISweepLeafTester;
    }

    public struct ConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB> : IConvexCompoundSweepOverlapFinder<TShapeA, TCompoundB>
        where TShapeA : struct, IConvexShape where TCompoundB : struct, IBoundsQueryableCompound
    {
        public unsafe void FindOverlaps(ref TShapeA shapeA, in Quaternion orientationA, in BodyVelocity velocityA,
            ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, out ChildOverlapsCollection overlaps)
        {
            overlaps = default;

            ShapeTreeSweepLeafTester<ChildOverlapsCollection> enumerator;
            enumerator.Pool = pool;
            enumerator.Overlaps = Unsafe.AsPointer(ref overlaps);

            BoundingBoxHelpers.GetLocalBoundingBoxForSweep(ref shapeA, orientationA, velocityA, offsetB, orientationB, velocityB, maximumT, out var sweep, out var min, out var max);

            FindOverlaps(ref shapeA, orientationA, velocityA,
                ref compoundB, offsetB, orientationB, velocityB, maximumT,
                shapes, pool, ref enumerator);
        }

        public unsafe void FindOverlaps<TLeafTester>(ref TShapeA shapeA, in Quaternion orientationA, in BodyVelocity velocityA,
            ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, ref TLeafTester leafTester)
            where TLeafTester : ISweepLeafTester
        {
            BoundingBoxHelpers.GetLocalBoundingBoxForSweep(ref shapeA, orientationA, velocityA, offsetB, orientationB, velocityB, maximumT, out var sweep, out var min, out var max);

            compoundB.FindLocalOverlaps(min, max, sweep, maximumT, pool, shapes, ref leafTester);
        }
    }
}
