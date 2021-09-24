using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public interface ICompoundPairOverlapFinder
    {
        void FindLocalOverlaps(ref Buffer<BoundsTestedPair> pairs, int pairCount, BufferPool pool, Shapes shapes, float dt, out CompoundPairOverlaps overlaps);
    }

    public unsafe interface ICompoundPairContinuationHandler<TContinuation> where TContinuation : struct, ICollisionTestContinuation
    {
        CollisionContinuationType CollisionContinuationType { get; }

        ref TContinuation CreateContinuation<TCallbacks>(ref CollisionBatcher<TCallbacks> collisionBatcher, int childCount, ref Buffer<ChildOverlapsCollection> pairOverlaps, ref Buffer<OverlapQueryForPair> pairQueries, in BoundsTestedPair pair, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks;

        void GetChildAData<TCallbacks>(ref CollisionBatcher<TCallbacks> collisionBatcher, ref TContinuation continuation, in BoundsTestedPair pair, int childIndexA, out RigidPose childPoseA, out int childTypeA, out void* childShapeDataA)
            where TCallbacks : struct, ICollisionCallbacks;

        void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref TContinuation continuation, int continuationChildIndex, in BoundsTestedPair pair, int childIndexA, int childTypeA, int childIndexB,
            in RigidPose childPoseA, out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks;
    }

    public class CompoundPairCollisionTask<TCompoundA, TCompoundB, TOverlapFinder, TContinuationHandler, TContinuation> : CollisionTask
        where TCompoundA : unmanaged, IShape, IBoundsQueryableCompound
        where TCompoundB : unmanaged, IShape, IBoundsQueryableCompound
        where TOverlapFinder : struct, ICompoundPairOverlapFinder
        where TContinuationHandler : struct, ICompoundPairContinuationHandler<TContinuation>
        where TContinuation : struct, ICollisionTestContinuation
    {
        public CompoundPairCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(TCompoundA).TypeId;
            ShapeTypeIndexB = default(TCompoundB).TypeId;
            SubtaskGenerator = true;
            PairType = CollisionTaskPairType.BoundsTestedPair;
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            var pairs = batch.Buffer.As<BoundsTestedPair>();
            Unsafe.SkipInit(out TOverlapFinder overlapFinder);
            Unsafe.SkipInit(out TContinuationHandler continuationHandler);
            //We perform all necessary bounding box computations and lookups up front. This helps avoid some instruction pipeline pressure at the cost of some extra data cache requirements.
            //Because of this, you need to be careful with the batch size on this collision task.
            overlapFinder.FindLocalOverlaps(ref pairs, batch.Count, batcher.Pool, batcher.Shapes, batcher.Dt, out var overlaps);

            for (int pairIndex = 0; pairIndex < batch.Count; ++pairIndex)
            {
                overlaps.GetPairOverlaps(pairIndex, out var pairOverlaps, out var subpairQueries);
                var totalOverlapCountForPair = pairOverlaps[0].Count;
                for (int j = 1; j < pairOverlaps.Length; ++j)
                {
                    totalOverlapCountForPair += pairOverlaps[j].Count;
                }
                if (totalOverlapCountForPair > 0)
                {
                    ref var pair = ref pairs[pairIndex];
                    ref var continuation = ref continuationHandler.CreateContinuation(ref batcher, totalOverlapCountForPair, ref pairOverlaps, ref subpairQueries, pair, out var continuationIndex);

                    var nextContinuationChildIndex = 0;
                    for (int j = 0; j < pairOverlaps.Length; ++j)
                    {
                        ref var childOverlaps = ref pairOverlaps[j];
                        if (childOverlaps.Count == 0)
                            continue;
                        continuationHandler.GetChildAData(ref batcher, ref continuation, pair, childOverlaps.ChildIndex, out var childPoseA, out var childTypeA, out var childShapeDataA);
                        //Note that we defer the region assignment until after the loop rather than using the triangleCount as the region count.
                        //That's because the user callback could cull some of the subpairs.
                        for (int k = 0; k < childOverlaps.Count; ++k)
                        {
                            var originalChildIndexB = childOverlaps.Overlaps[k];
                            //Note that we have to take into account whether we flipped the shapes to match the expected memory layout.
                            //The caller expects results according to the submitted pair order, not the batcher's memory layout order.
                            int childA, childB;
                            if (pair.FlipMask < 0)
                            {
                                childA = originalChildIndexB;
                                childB = childOverlaps.ChildIndex;
                            }
                            else
                            {
                                childA = childOverlaps.ChildIndex;
                                childB = originalChildIndexB;
                            }
                            var continuationChildIndex = nextContinuationChildIndex++;
                            var subpairContinuation = new PairContinuation(pair.Continuation.PairId, childA, childB,
                                continuationHandler.CollisionContinuationType, continuationIndex, continuationChildIndex);
                            if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                            {
                                continuationHandler.ConfigureContinuationChild(ref batcher, ref continuation, continuationChildIndex, pair, childOverlaps.ChildIndex, childTypeA, originalChildIndexB,
                                    childPoseA, out var childPoseB, out var childTypeB, out var childShapeDataB);

                                var childAToChildB = pair.OffsetB + childPoseB.Position - childPoseA.Position;
                                if (pair.FlipMask < 0)
                                {
                                    //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                    batcher.AddDirectly(childTypeB, childTypeA, childShapeDataB, childShapeDataA,
                                        -childAToChildB, childPoseB.Orientation, childPoseA.Orientation, pair.SpeculativeMargin, subpairContinuation);
                                }
                                else
                                {
                                    batcher.AddDirectly(childTypeA, childTypeB, childShapeDataA, childShapeDataB,
                                        childAToChildB, childPoseA.Orientation, childPoseB.Orientation, pair.SpeculativeMargin, subpairContinuation);
                                }
                            }
                            else
                            {
                                continuation.OnChildCompletedEmpty(ref subpairContinuation, ref batcher);
                            }
                        }
                    }
                }


            }
            overlaps.Dispose(batcher.Pool);
            //Note that the triangle lists are not disposed here. Those are handed off to the continuations for further analysis.
        }
    }
}
