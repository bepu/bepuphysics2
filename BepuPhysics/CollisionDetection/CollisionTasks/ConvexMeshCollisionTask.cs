using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public interface IConvexCompoundOverlapFinder
    {
        void FindLocalOverlaps(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out ConvexCompoundTaskOverlaps overlaps);
    }

    public class ConvexMeshCollisionTask<TConvex, TMesh, TOverlapFinder> : CollisionTask
        where TConvex : struct, IConvexShape
        where TMesh : struct, IMeshShape
        where TOverlapFinder : IConvexCompoundOverlapFinder
    {
        public ConvexMeshCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(TConvex).TypeId;
            ShapeTypeIndexB = default(TMesh).TypeId;
            SubtaskGenerator = true;
            PairType = CollisionTaskPairType.BoundsTestedPair;
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            var pairs = batch.Buffer.As<BoundsTestedPair>();
            TOverlapFinder overlapFinder = default;

            //We perform all necessary bounding box computations and lookups up front. This helps avoid some instruction pipeline pressure at the cost of some extra data cache requirements.
            //Because of this, you need to be careful with the batch size on this collision task.
            overlapFinder.FindLocalOverlaps(ref pairs, batch.Count, batcher.Pool, batcher.Shapes, batcher.Dt, out var overlaps);
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var pairOverlaps = ref overlaps.GetOverlapsForSubpair(i);
                if (pairOverlaps.Count > 0)
                {
                    ref var pair = ref pairs[i];
                    ref var mesh = ref Unsafe.AsRef<TMesh>(pair.B);
                    ref var continuation = ref batcher.MeshReductions.CreateContinuation(pairOverlaps.Count, batcher.Pool, out var continuationIndex);
                    //Pass ownership of the triangle and region buffers to the continuation. It'll dispose of the buffer.
                    batcher.Pool.Take(pairOverlaps.Count, out continuation.Triangles);
                    continuation.MeshOrientation = pair.OrientationB;
                    //A flip is required in mesh reduction whenever contacts are being generated as if the triangle is in slot B, which is whenever this pair has *not* been flipped.
                    continuation.RequiresFlip = pair.FlipMask == 0;

                    int nextContinuationChildIndex = 0;
                    for (int j = 0; j < pairOverlaps.Count; ++j)
                    {
                        var triangleIndex = pairOverlaps.Overlaps[j];                        
                        //Note that we have to take into account whether we flipped the shapes to match the expected memory layout.
                        //The caller expects results according to the submitted pair order, not the batcher's memory layout order.
                        int childA, childB;
                        if (pair.FlipMask < 0)
                        {
                            childA = triangleIndex;
                            childB = 0;
                        }
                        else
                        {
                            childA = 0;
                            childB = triangleIndex;
                        }
                        if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                        {
                            var continuationChildIndex = nextContinuationChildIndex++;
                            var continuationInfo = new PairContinuation(pair.Continuation.PairId, childA, childB,
                                CollisionContinuationType.MeshReduction, continuationIndex, continuationChildIndex);
                            ref var continuationChild = ref continuation.Inner.Children[continuationChildIndex];
                            continuationChild.OffsetA = default;
                            continuationChild.ChildIndexA = childA;
                            continuationChild.OffsetB = default;
                            continuationChild.ChildIndexB = childB;

                            //Note that the triangles list persists until the continuation completes, which means the memory will be validly accessible for all of the spawned collision tasks.
                            //In other words, we can pass a pointer to it to avoid the need for additional batcher shape copying.
                            ref var triangle = ref continuation.Triangles[continuationChildIndex];
                            mesh.GetLocalTriangle(triangleIndex, out triangle);
                            if (pair.FlipMask < 0)
                            {
                                //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                batcher.AddDirectly(triangle.TypeId, ShapeTypeIndexA, Unsafe.AsPointer(ref triangle), pair.A,
                                    -pair.OffsetB, pair.OrientationB, pair.OrientationA, pair.SpeculativeMargin, continuationInfo);
                            }
                            else
                            {
                                batcher.AddDirectly(ShapeTypeIndexA, triangle.TypeId, pair.A, Unsafe.AsPointer(ref triangle),
                                    pair.OffsetB, pair.OrientationA, pair.OrientationB, pair.SpeculativeMargin, continuationInfo);
                            }
                        }
                        else
                        {
                            continuation.OnChildCompletedEmpty(ref pair.Continuation, ref batcher);
                        }

                    }
                }


            }
            overlaps.Dispose(batcher.Pool);
            //Note that the triangle lists are not disposed here. Those are handed off to the continuations for further analysis.
        }
    }
}
