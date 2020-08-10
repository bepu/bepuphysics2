using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    //At the moment, this is basically an unused abstraction. But, if you wanted, this allows you to use a special cased overlap finder in certain cases.
    public interface ICompoundPairSweepOverlapFinder<TCompoundA, TCompoundB> where TCompoundA : struct, ICompoundShape where TCompoundB : struct, IBoundsQueryableCompound
    {
        unsafe void FindOverlaps(ref TCompoundA compoundA, in Quaternion orientationA, in BodyVelocity velocityA,
              ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
              Shapes shapes, BufferPool pool, out CompoundPairSweepOverlaps overlaps);
    }

    public struct CompoundPairSweepOverlapFinder<TCompoundA, TCompoundB> : ICompoundPairSweepOverlapFinder<TCompoundA, TCompoundB>
        where TCompoundA : struct, ICompoundShape
        where TCompoundB : struct, IBoundsQueryableCompound
    {
        public unsafe void FindOverlaps(
            ref TCompoundA compoundA, in Quaternion orientationA, in BodyVelocity velocityA,
            ref TCompoundB compoundB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            Shapes shapes, BufferPool pool, out CompoundPairSweepOverlaps overlaps)
        {
            overlaps = new CompoundPairSweepOverlaps(pool, compoundA.ChildCount);
            for (int i = 0; i < compoundA.ChildCount; ++i)
            {
                ref var child = ref compoundA.GetChild(i);
                BoundingBoxHelpers.GetLocalBoundingBoxForSweep(
                    child.ShapeIndex, shapes, child.LocalPose, orientationA, velocityA,
                    offsetB, orientationB, velocityB, maximumT, out var sweep, out var min, out var max);
                ref var childOverlaps = ref overlaps.GetOverlapsForChild(i);
                childOverlaps.ChildIndex = i;
                compoundB.FindLocalOverlaps<ChildOverlapsCollection>(min, max, sweep, maximumT, pool, shapes, Unsafe.AsPointer(ref childOverlaps));
            }
        }
    }
}
