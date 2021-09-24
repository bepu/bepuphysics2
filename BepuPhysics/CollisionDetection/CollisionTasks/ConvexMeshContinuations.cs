using BepuPhysics.Collidables;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct ConvexMeshContinuations<TMesh> : IConvexCompoundContinuationHandler<MeshReduction> where TMesh : IHomogeneousCompoundShape<Triangle, TriangleWide>
    {
        public CollisionContinuationType CollisionContinuationType => CollisionContinuationType.MeshReduction;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref MeshReduction CreateContinuation<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, int childCount, in BoundsTestedPair pair, in OverlapQueryForPair pairQuery, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var continuation = ref collisionBatcher.MeshReductions.CreateContinuation(childCount, collisionBatcher.Pool, out continuationIndex);
            //Pass ownership of the triangle and region buffers to the continuation. It'll dispose of the buffer.
            collisionBatcher.Pool.Take(childCount, out continuation.Triangles);
            continuation.MeshOrientation = pair.OrientationB;
            //A flip is required in mesh reduction whenever contacts are being generated as if the triangle is in slot B, which is whenever this pair has *not* been flipped.
            continuation.RequiresFlip = pair.FlipMask == 0;
            continuation.QueryBounds.Min = pairQuery.Min;
            continuation.QueryBounds.Max = pairQuery.Max;
            //TODO: This is not flexible with respect to different mesh types. Not a problem right now, but it will be in the future.
            continuation.Mesh = pairQuery.Container;
            return ref continuation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref MeshReduction continuation, int continuationChildIndex, in BoundsTestedPair pair, int shapeTypeA, int childIndex,
            out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks
        {
            //Note that the triangles list persists until the continuation completes, which means the memory will be validly accessible for all of the spawned collision tasks.
            //In other words, we can pass a pointer to it to avoid the need for additional batcher shape copying.
            ref var triangle = ref continuation.Triangles[continuationChildIndex];
            childShapeDataB = Unsafe.AsPointer(ref triangle);
            childTypeB = triangle.TypeId;
            Unsafe.AsRef<TMesh>(pair.B).GetLocalChild(childIndex, out continuation.Triangles[continuationChildIndex]);
            ref var continuationChild = ref continuation.Inner.Children[continuationChildIndex];
            //Triangles already have their local pose baked into their vertices, so we just need the orientation.
            childPoseB = new RigidPose(default, pair.OrientationB);
            continuationChild.OffsetA = default;
            continuationChild.OffsetB = default;
            if (pair.FlipMask < 0)
            {
                continuationChild.ChildIndexA = childIndex;
                continuationChild.ChildIndexB = 0;
            }
            else
            {
                continuationChild.ChildIndexA = 0;
                continuationChild.ChildIndexB = childIndex;
            }
        }

    }
}
