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
    public class CompoundPairCollisionTask<TCompoundA, TCompoundB, TOverlapFinder> : CollisionTask
        where TCompoundA : struct, ICompoundShape
        where TCompoundB : struct, ICompoundShape
        where TOverlapFinder : struct, ICompoundPairOverlapFinder
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
            TOverlapFinder overlapFinder = default;

            //We perform all necessary bounding box computations and lookups up front. This helps avoid some instruction pipeline pressure at the cost of some extra data cache requirements.
            //Because of this, you need to be careful with the batch size on this collision task.
            overlapFinder.FindLocalOverlaps(ref pairs, batch.Count, batcher.Pool, batcher.Shapes, batcher.Dt, out var overlaps);
            for (int pairIndex = 0; pairIndex < batch.Count; ++pairIndex)
            {
                overlaps.GetPairOverlaps(pairIndex, out var pairOverlaps);
                var totalOverlapCountForPair = pairOverlaps[0].Count;
                for (int j = 1; j < pairOverlaps.Length; ++j)
                {
                    totalOverlapCountForPair += pairOverlaps[j].Count;
                }
                if (totalOverlapCountForPair > 0)
                {
                    ref var pair = ref pairs[pairIndex];
                    ref var compoundA = ref Unsafe.AsRef<TCompoundA>(pair.A);
                    ref var compoundB = ref Unsafe.AsRef<TCompoundB>(pair.B);
                    ref var continuation = ref batcher.NonconvexReductions.CreateContinuation(totalOverlapCountForPair, batcher.Pool, out var continuationIndex);

                    var nextContinuationChildIndex = 0;
                    for (int j = 0; j < pairOverlaps.Length; ++j)
                    {
                        ref var childOverlaps = ref pairOverlaps[j];
                        if (childOverlaps.Count == 0)
                            continue;
                        ref var compoundChildA = ref compoundA.GetChild(childOverlaps.ChildIndex);
                        Compound.GetRotatedChildPose(compoundChildA.LocalPose, pair.OrientationA, out var rotatedChildPoseA);
                        var childTypeA = compoundChildA.ShapeIndex.Type;
                        batcher.Shapes[childTypeA].GetShapeData(compoundChildA.ShapeIndex.Index, out var compoundChildShapeDataA, out _);
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
                                CollisionContinuationType.NonconvexReduction, continuationIndex, continuationChildIndex);
                            if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                            {
                                ref var continuationChild = ref continuation.Children[continuationChildIndex];
                                continuationChild.ChildIndexA = childA;
                                continuationChild.ChildIndexB = childB;

                                ref var compoundChildB = ref compoundB.GetChild(originalChildIndexB);
                                var childTypeB = compoundChildB.ShapeIndex.Type;
                                batcher.Shapes[childTypeB].GetShapeData(compoundChildB.ShapeIndex.Index, out var compoundChildShapeDataB, out _);

                                Compound.GetRotatedChildPose(compoundChildB.LocalPose, pair.OrientationB, out var rotatedChildPoseB);
                                var childAToChildB = pair.OffsetB + rotatedChildPoseB.Position - rotatedChildPoseA.Position; 
                                if (pair.FlipMask < 0)
                                {
                                    continuationChild.OffsetA = rotatedChildPoseB.Position;
                                    continuationChild.OffsetB = rotatedChildPoseA.Position;
                                    //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                    batcher.AddDirectly(childTypeB, childTypeA, compoundChildShapeDataB, compoundChildShapeDataA,
                                        -childAToChildB, rotatedChildPoseB.Orientation, rotatedChildPoseA.Orientation, pair.SpeculativeMargin, subpairContinuation);
                                }
                                else
                                {
                                    continuationChild.OffsetA = rotatedChildPoseA.Position;
                                    continuationChild.OffsetB = rotatedChildPoseB.Position;
                                    batcher.AddDirectly(childTypeA, childTypeB, compoundChildShapeDataA, compoundChildShapeDataB,
                                        childAToChildB, rotatedChildPoseA.Orientation, rotatedChildPoseB.Orientation, pair.SpeculativeMargin, subpairContinuation);
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
