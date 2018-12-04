﻿using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public unsafe struct MeshPairContinuations<TMeshA, TMeshB> : ICompoundPairContinuationHandler<CompoundMeshReduction>
        where TMeshA : IMeshShape
        where TMeshB : IMeshShape
    {
        public CollisionContinuationType CollisionContinuationType => CollisionContinuationType.CompoundMeshReduction;

        int triangleAStartIndex;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CompoundMeshReduction CreateContinuation<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, int totalChildCount, ref Buffer<ChildOverlapsCollection> pairOverlaps, in BoundsTestedPair pair, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var continuation = ref collisionBatcher.CompoundMeshReductions.CreateContinuation(totalChildCount, collisionBatcher.Pool, out continuationIndex);
            //Pass ownership of the triangle and region buffers to the continuation. It'll dispose of the buffer.
            //Note that this expands the triangles set by pairOverlaps.Length- we're going to store the triangles of mesh A in the surplus space. A bit of a hack, but simple and cheap.
            triangleAStartIndex = totalChildCount;
            collisionBatcher.Pool.Take(totalChildCount + pairOverlaps.Length, out continuation.Triangles);
            collisionBatcher.Pool.Take(pairOverlaps.Length, out continuation.ChildManifoldRegions);
            continuation.RegionCount = pairOverlaps.Length;
            continuation.MeshOrientation = pair.OrientationB;
            //A flip is required in mesh reduction whenever contacts are being generated as if the triangle is in slot B, which is whenever this pair has *not* been flipped.
            continuation.RequiresFlip = pair.FlipMask == 0;

            //All regions must be assigned ahead of time. Some trailing regions may be empty, so the dispatch may occur before all children are visited in the later loop.
            //That would result in potentially uninitialized values in region counts.
            int nextContinuationChildIndex = 0;
            for (int j = 0; j < pairOverlaps.Length; ++j)
            {
                ref var childOverlaps = ref pairOverlaps[j];
                continuation.ChildManifoldRegions[j] = (nextContinuationChildIndex, childOverlaps.Count);
                nextContinuationChildIndex += childOverlaps.Count;
            }
            return ref continuation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetChildAData<TCallbacks>(ref CollisionBatcher<TCallbacks> collisionBatcher, ref CompoundMeshReduction continuation, in BoundsTestedPair pair, int childIndexA,
            out RigidPose childPoseA, out int childTypeA, out void* childShapeDataA)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var triangle = ref continuation.Triangles[triangleAStartIndex++];
            childShapeDataA = Unsafe.AsPointer(ref triangle);
            childTypeA = triangle.TypeId;
            Unsafe.AsRef<TMeshA>(pair.A).GetLocalTriangle(childIndexA, out triangle);
            childPoseA = new RigidPose(default, pair.OrientationA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref CompoundMeshReduction continuation, int continuationChildIndex, in BoundsTestedPair pair, int childIndexA, int childIndexB, in RigidPose childPoseA,
            out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks
        {
            //Note that the triangles list persists until the continuation completes, which means the memory will be validly accessible for all of the spawned collision tasks.
            //In other words, we can pass a pointer to it to avoid the need for additional batcher shape copying.
            ref var triangle = ref continuation.Triangles[continuationChildIndex];
            childShapeDataB = Unsafe.AsPointer(ref triangle);
            childTypeB = triangle.TypeId;
            Unsafe.AsRef<TMeshB>(pair.B).GetLocalTriangle(childIndexB, out continuation.Triangles[continuationChildIndex]);
            ref var continuationChild = ref continuation.Inner.Children[continuationChildIndex];
            //In meshes, the triangle's vertices already contain the offset, so there is no additional offset.                                 
            childPoseB = new RigidPose(default, pair.OrientationB);
            continuationChild.OffsetA = default;
            continuationChild.OffsetB = default;
            if (pair.FlipMask < 0)
            {
                continuationChild.ChildIndexA = childIndexB;
                continuationChild.ChildIndexB = childIndexA;
            }
            else
            {
                continuationChild.ChildIndexA = childIndexA;
                continuationChild.ChildIndexB = childIndexB;
            }
        }

    }
}