using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct ConvexMeshContinuations<TMesh> : IConvexCompoundContinuationHandler<MeshReduction> where TMesh : IMeshShape
    {
        public CollisionContinuationType CollisionContinuationType => CollisionContinuationType.MeshReduction;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MeshReduction CreateContinuation<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, int childCount, BufferPool pool, in BoundsTestedPair pair, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var continuation = ref collisionBatcher.MeshReductions.CreateContinuation(childCount, pool, out continuationIndex);
            //Pass ownership of the triangle and region buffers to the continuation. It'll dispose of the buffer.
            pool.Take(childCount, out continuation.Triangles);
            continuation.MeshOrientation = pair.OrientationB;
            //A flip is required in mesh reduction whenever contacts are being generated as if the triangle is in slot B, which is whenever this pair has *not* been flipped.
            continuation.RequiresFlip = pair.FlipMask == 0;
            return ref continuation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref MeshReduction continuation, int continuationChildIndex, in BoundsTestedPair pair, int childIndex,
            out int compoundChildType, out void* compoundChildShapeData, out Vector3 convexToChild, out Quaternion childOrientation)
            where TCallbacks : struct, ICollisionCallbacks
        {
            //Note that the triangles list persists until the continuation completes, which means the memory will be validly accessible for all of the spawned collision tasks.
            //In other words, we can pass a pointer to it to avoid the need for additional batcher shape copying.
            ref var triangle = ref continuation.Triangles[continuationChildIndex];
            compoundChildShapeData = Unsafe.AsPointer(ref triangle);
            compoundChildType = triangle.TypeId;
            Unsafe.AsRef<TMesh>(pair.B).GetLocalTriangle(childIndex, out continuation.Triangles[continuationChildIndex]);
            ref var continuationChild = ref continuation.Inner.Children[continuationChildIndex];
            convexToChild = pair.OffsetB;
            continuationChild.OffsetA = default;
            continuationChild.OffsetB = default;
            childOrientation = pair.OrientationB;
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
