using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public unsafe struct CompoundMeshContinuations<TCompound, TMesh> : ICompoundPairContinuationHandler<CompoundMeshReduction>
        where TCompound : ICompoundShape
        where TMesh : IHomogeneousCompoundShape<Triangle, TriangleWide>
    {
        public CollisionContinuationType CollisionContinuationType => CollisionContinuationType.CompoundMeshReduction;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CompoundMeshReduction CreateContinuation<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, int totalChildCount, ref Buffer<ChildOverlapsCollection> pairOverlaps, ref Buffer<OverlapQueryForPair> pairQueries, in BoundsTestedPair pair, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var continuation = ref collisionBatcher.CompoundMeshReductions.CreateContinuation(totalChildCount, collisionBatcher.Pool, out continuationIndex);
            //Pass ownership of the triangle and region buffers to the continuation. It'll dispose of the buffer.
            collisionBatcher.Pool.Take(totalChildCount, out continuation.Triangles);
            collisionBatcher.Pool.Take(pairOverlaps.Length, out continuation.ChildManifoldRegions);
            collisionBatcher.Pool.Take(pairOverlaps.Length, out continuation.QueryBounds);
            continuation.RegionCount = pairOverlaps.Length;
            continuation.MeshOrientation = pair.OrientationB;
            //TODO: This is not flexible with respect to different mesh types. Not a problem right now, but it will be in the future.
            continuation.Mesh = (Mesh*)pair.B;
            //A flip is required in mesh reduction whenever contacts are being generated as if the triangle is in slot B, which is whenever this pair has *not* been flipped.
            continuation.RequiresFlip = pair.FlipMask == 0;

            //All regions must be assigned ahead of time. Some trailing regions may be empty, so the dispatch may occur before all children are visited in the later loop.
            //That would result in potentially uninitialized values in region counts.
            int nextContinuationChildIndex = 0;
            Debug.Assert(pairOverlaps.Length == pairQueries.Length);
            for (int j = 0; j < pairOverlaps.Length; ++j)
            {
                ref var childOverlaps = ref pairOverlaps[j];
                continuation.ChildManifoldRegions[j] = (nextContinuationChildIndex, childOverlaps.Count);
                nextContinuationChildIndex += childOverlaps.Count;
                ref var continuationBounds = ref continuation.QueryBounds[j];
                ref var sourceBounds = ref pairQueries[j];
                continuationBounds.Min = sourceBounds.Min;
                continuationBounds.Max = sourceBounds.Max;
            }
            return ref continuation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetChildAData<TCallbacks>(ref CollisionBatcher<TCallbacks> collisionBatcher, ref CompoundMeshReduction continuation, in BoundsTestedPair pair, int childIndexA,
            out RigidPose childPoseA, out int childTypeA, out void* childShapeDataA)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var compound = ref Unsafe.AsRef<TCompound>(pair.A);
            ref var compoundChild = ref compound.GetChild(childIndexA);
            Compound.GetRotatedChildPose(compoundChild.LocalPose, pair.OrientationA, out childPoseA);
            childTypeA = compoundChild.ShapeIndex.Type;
            collisionBatcher.Shapes[childTypeA].GetShapeData(compoundChild.ShapeIndex.Index, out childShapeDataA, out _);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref CompoundMeshReduction continuation, int continuationChildIndex, in BoundsTestedPair pair, int childIndexA, int childTypeA, int childIndexB, in RigidPose childPoseA,
            out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks
        {
            //Note that the triangles list persists until the continuation completes, which means the memory will be validly accessible for all of the spawned collision tasks.
            //In other words, we can pass a pointer to it to avoid the need for additional batcher shape copying.
            ref var triangle = ref continuation.Triangles[continuationChildIndex];
            childShapeDataB = Unsafe.AsPointer(ref triangle);
            childTypeB = triangle.TypeId;
            Unsafe.AsRef<TMesh>(pair.B).GetLocalChild(childIndexB, out continuation.Triangles[continuationChildIndex]);
            ref var continuationChild = ref continuation.Inner.Children[continuationChildIndex];
            //In meshes, the triangle's vertices already contain the offset, so there is no additional offset.                                 
            childPoseB = new RigidPose(default, pair.OrientationB);
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
