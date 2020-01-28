using BepuPhysics.Collidables;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct ConvexCompoundContinuations<TCompound> : IConvexCompoundContinuationHandler<NonconvexReduction> where TCompound : ICompoundShape
    {
        public CollisionContinuationType CollisionContinuationType => CollisionContinuationType.NonconvexReduction;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexReduction CreateContinuation<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, int childCount, in BoundsTestedPair pair, in OverlapQueryForPair pairQuery, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks
        {
            return ref collisionBatcher.NonconvexReductions.CreateContinuation(childCount, collisionBatcher.Pool, out continuationIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref NonconvexReduction continuation, int continuationChildIndex, in BoundsTestedPair pair, int shapeTypeA, int childIndex,
            out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var compoundChild = ref Unsafe.AsRef<TCompound>(pair.B).GetChild(childIndex);
            ref var continuationChild = ref continuation.Children[continuationChildIndex];
            Compound.GetRotatedChildPose(compoundChild.LocalPose, pair.OrientationB, out childPoseB);
            childTypeB = compoundChild.ShapeIndex.Type;
            collisionBatcher.Shapes[childTypeB].GetShapeData(compoundChild.ShapeIndex.Index, out childShapeDataB, out _);
            if (pair.FlipMask < 0)
            {
                continuationChild.ChildIndexA = childIndex;
                continuationChild.ChildIndexB = 0;
                continuationChild.OffsetA = childPoseB.Position;
                continuationChild.OffsetB = default;
            }
            else
            {
                continuationChild.ChildIndexA = 0;
                continuationChild.ChildIndexB = childIndex;
                continuationChild.OffsetA = default;
                continuationChild.OffsetB = childPoseB.Position;
            }
        }

    }
}
