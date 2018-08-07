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
    public class ConvexCompoundCollisionTask<TConvex, TCompound, TOverlapFinder> : CollisionTask
        where TConvex : struct, IConvexShape
        where TCompound : struct, ICompoundShape
        where TOverlapFinder : IConvexCompoundOverlapFinder
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
                    ref var compound = ref Unsafe.AsRef<TCompound>(pair.B);
                    ref var continuation = ref batcher.NonconvexReductions.CreateContinuation(pairOverlaps.Count, batcher.Pool, out var continuationIndex);

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
                            CollisionContinuationType.NonconvexReduction, continuationIndex, continuationChildIndex);
                        if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                        {
                            ref var compoundChild = ref compound.GetChild(childIndex);
                            ref var continuationChild = ref continuation.Children[continuationChildIndex];
                            Compound.GetRotatedChildPose(compoundChild.LocalPose, pair.OrientationB, out var rotatedChildPose);
                            var compoundChildType = compoundChild.ShapeIndex.Type;
                            batcher.Shapes[compoundChildType].GetShapeData(compoundChild.ShapeIndex.Index, out var compoundChildShapeData, out _);
                            continuationChild.ChildIndexA = childA;
                            continuationChild.ChildIndexB = childB;

                            var convexToChild = pair.OffsetB + rotatedChildPose.Position; 
                            if (pair.FlipMask < 0)
                            {
                                continuationChild.OffsetA = rotatedChildPose.Position;
                                continuationChild.OffsetB = default;
                                //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                batcher.AddDirectly(compoundChildType, ShapeTypeIndexA, compoundChildShapeData, pair.A,
                                    -convexToChild, rotatedChildPose.Orientation, pair.OrientationA, pair.SpeculativeMargin, subpairContinuation);
                            }
                            else
                            {
                                continuationChild.OffsetA = default;
                                continuationChild.OffsetB = rotatedChildPose.Position;
                                batcher.AddDirectly(ShapeTypeIndexA, compoundChildType, pair.A, compoundChildShapeData,
                                    convexToChild, pair.OrientationA, rotatedChildPose.Orientation, pair.SpeculativeMargin, subpairContinuation);
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
