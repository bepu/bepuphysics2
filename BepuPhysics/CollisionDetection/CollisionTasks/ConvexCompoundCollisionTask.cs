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
    public unsafe interface IConvexCompoundContinuationHandler<TContinuation> where TContinuation : struct, ICollisionTestContinuation
    {
        CollisionContinuationType CollisionContinuationType { get; }

        ref TContinuation CreateContinuation<TCallbacks>(ref CollisionBatcher<TCallbacks> collisionBatcher, int childCount, in BoundsTestedPair pair, in OverlapQueryForPair queryForPair, out int continuationIndex)
            where TCallbacks : struct, ICollisionCallbacks;

        void ConfigureContinuationChild<TCallbacks>(
            ref CollisionBatcher<TCallbacks> collisionBatcher, ref TContinuation continuation, int continuationChildIndex, in BoundsTestedPair pair, int shapeTypeA, int childIndex,
            out RigidPose childPoseB, out int childTypeB, out void* childShapeDataB)
            where TCallbacks : struct, ICollisionCallbacks;
    }

    public class ConvexCompoundCollisionTask<TConvex, TCompound, TOverlapFinder, TContinuationHandler, TContinuation> : CollisionTask
        where TConvex : unmanaged, IConvexShape
        where TCompound : unmanaged, IShape, IBoundsQueryableCompound
        where TOverlapFinder : struct, IConvexCompoundOverlapFinder
        where TContinuationHandler : struct, IConvexCompoundContinuationHandler<TContinuation>
        where TContinuation : struct, ICollisionTestContinuation
    {
        public ConvexCompoundCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(TConvex).TypeId;
            ShapeTypeIndexB = default(TCompound).TypeId;
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
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var pairOverlaps = ref overlaps.GetOverlapsForPair(i);
                ref var pairQuery = ref overlaps.GetQueryForPair(i);
                if (pairOverlaps.Count > 0)
                {
                    ref var pair = ref pairs[i];
                    ref var compound = ref Unsafe.AsRef<TCompound>(pair.B);
                    ref var continuation = ref continuationHandler.CreateContinuation(ref batcher, pairOverlaps.Count, pair, pairQuery, out var continuationIndex);

                    int nextContinuationChildIndex = 0;
                    for (int j = 0; j < pairOverlaps.Count; ++j)
                    {
                        var childIndex = pairOverlaps.Overlaps[j];
                        //Note that we have to take into account whether we flipped the shapes to match the expected memory layout.
                        //The caller expects results according to the submitted pair order, not the batcher's memory layout order.
                        int childA, childB;
                        if (pair.FlipMask < 0)
                        {
                            childA = childIndex;
                            childB = 0;
                        }
                        else
                        {
                            childA = 0;
                            childB = childIndex;
                        }
                        var continuationChildIndex = nextContinuationChildIndex++;
                        var subpairContinuation = new PairContinuation(pair.Continuation.PairId, childA, childB,
                            continuationHandler.CollisionContinuationType, continuationIndex, continuationChildIndex);
                        if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                        {
                            continuationHandler.ConfigureContinuationChild(ref batcher, ref continuation, continuationChildIndex, pair, ShapeTypeIndexA, childIndex,
                                out var compoundChildPose, out var compoundChildType, out var compoundChildShapeData);

                            var convexToChild = compoundChildPose.Position + pair.OffsetB;
                            if (pair.FlipMask < 0)
                            {
                                //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                batcher.AddDirectly(compoundChildType, ShapeTypeIndexA, compoundChildShapeData, pair.A,
                                    -convexToChild, compoundChildPose.Orientation, pair.OrientationA, pair.SpeculativeMargin, subpairContinuation);
                            }
                            else
                            {
                                batcher.AddDirectly(ShapeTypeIndexA, compoundChildType, pair.A, compoundChildShapeData,
                                    convexToChild, pair.OrientationA, compoundChildPose.Orientation, pair.SpeculativeMargin, subpairContinuation);
                            }
                        }
                        else
                        {
                            continuation.OnChildCompletedEmpty(ref subpairContinuation, ref batcher);
                        }

                    }
                }
            }
            overlaps.Dispose(batcher.Pool);
        }
    }
}
