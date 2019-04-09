using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public unsafe struct CompoundPairContinuations<TCompoundA, TCompoundB> : ICompoundPairContinuationHandler<NonconvexReduction>
        where TCompoundA : ICompoundShape
        where TCompoundB : ICompoundShape
    {
        public CollisionContinuationType CollisionContinuationType => CollisionContinuationType.NonconvexReduction;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexReduction CreateContinuation<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, int totalChildCount, ref Buffer<ChildOverlapsCollection> pairOverlaps, ref Buffer<OverlapQueryForPair> pairQueries, in BoundsTestedPair pair, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks
        {
            return ref collisionBatcher.NonconvexReductions.CreateContinuation(totalChildCount, collisionBatcher.Pool, out continuationIndex);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetChildAData<TCallbacks>(ref CollisionBatcher<TCallbacks> collisionBatcher, ref NonconvexReduction continuation, in BoundsTestedPair pair, int childIndexA,
            out RigidPose childPoseA, out int childTypeA, out void* childShapeDataA)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var compoundA = ref Unsafe.AsRef<TCompoundA>(pair.A);
            ref var compoundChildA = ref compoundA.GetChild(childIndexA);
            Compound.GetRotatedChildPose(compoundChildA.LocalPose, pair.OrientationA, out childPoseA);
            childTypeA = compoundChildA.ShapeIndex.Type;
            collisionBatcher.Shapes[childTypeA].GetShapeData(compoundChildA.ShapeIndex.Index, out childShapeDataA, out _);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref NonconvexReduction continuation, int continuationChildIndex, in BoundsTestedPair pair, int childIndexA, int childTypeA, int childIndexB, in RigidPose childPoseA,
            out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var continuationChild = ref continuation.Children[continuationChildIndex];

            ref var compoundB = ref Unsafe.AsRef<TCompoundB>(pair.B);
            ref var compoundChildB = ref compoundB.GetChild(childIndexB);
            childTypeB = compoundChildB.ShapeIndex.Type;
            collisionBatcher.Shapes[childTypeB].GetShapeData(compoundChildB.ShapeIndex.Index, out childShapeDataB, out _);

            Compound.GetRotatedChildPose(compoundChildB.LocalPose, pair.OrientationB, out childPoseB);
            if (pair.FlipMask < 0)
            {
                continuationChild.ChildIndexA = childIndexB;
                continuationChild.ChildIndexB = childIndexA;
                continuationChild.OffsetA = childPoseB.Position;
                continuationChild.OffsetB = childPoseA.Position;
            }
            else
            {
                continuationChild.ChildIndexA = childIndexA;
                continuationChild.ChildIndexB = childIndexB;
                continuationChild.OffsetA = childPoseA.Position;
                continuationChild.OffsetB = childPoseB.Position;
            }
        }

    }
}
