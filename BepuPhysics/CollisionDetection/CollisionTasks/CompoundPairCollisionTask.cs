using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public class CompoundPairCollisionTask : CollisionTask
    {
        public CompoundPairCollisionTask()
        {
            BatchSize = 8;
            ShapeTypeIndexA = default(Compound).TypeId;
            ShapeTypeIndexB = default(Compound).TypeId;
            SubtaskGenerator = true;
            PairType = CollisionTaskPairType.FliplessPair;
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            var testPairs = batch.Buffer.As<FliplessPair>();
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var pair = ref testPairs[i];
                ref var a = ref Unsafe.AsRef<Compound>(pair.A);
                ref var b = ref Unsafe.AsRef<Compound>(pair.B);
                Debug.Assert(pair.Continuation.ChildA == 0 && pair.Continuation.ChildB == 0 && pair.Continuation.Type == CollisionContinuationType.Direct,
                    "Compound-involving pairs cannot be marked as children of compound pairs. Convex-convex children of such pairs will be.");
                ref var continuation = ref batcher.NonconvexReductions.CreateContinuation(a.Children.Length * b.Children.Length, batcher.Pool, out var continuationIndex);
                int nextContinuationChildIndex = 0;
                for (int childAIndex = 0; childAIndex < a.Children.Length; ++childAIndex)
                {
                    ref var childA = ref a.Children[childAIndex];
                    Compound.GetRotatedChildPose(childA.LocalPose, pair.OrientationA, out var childAPose);
                    for (int childBIndex = 0; childBIndex < b.Children.Length; ++childBIndex)
                    {
                        if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childAIndex, childBIndex))
                        {
                            ref var childB = ref b.Children[childBIndex];
                            //You could avoid recalculating this pose for every childA, but the value is limited and it adds nontrivial complexity. Only bother if it shows up.
                            Compound.GetRotatedChildPose(childB.LocalPose, pair.OrientationB, out var childBPose);

                            var childShapeType = childB.ShapeIndex.Type;
                            batcher.Shapes[childShapeType].GetShapeData(childB.ShapeIndex.Index, out var childShapePointer, out var childShapeSize);

                            var continuationChildIndex = nextContinuationChildIndex++;
                            var subpairContinuation = new PairContinuation(pair.Continuation.PairId, childAIndex, childBIndex,
                                CollisionContinuationType.NonconvexReduction, continuationIndex, continuationChildIndex);
                            ref var continuationChild = ref batcher.NonconvexReductions.Continuations[continuationIndex].Children[continuationChildIndex];

                            continuationChild.OffsetA = childAPose.Position;
                            continuationChild.ChildIndexA = childAIndex;
                            continuationChild.OffsetB = childBPose.Position;
                            continuationChild.ChildIndexB = childBIndex;

                            batcher.Add(childA.ShapeIndex, childB.ShapeIndex, 
                                childBPose.Position + pair.OffsetB - childAPose.Position, childAPose.Orientation, childBPose.Orientation, pair.SpeculativeMargin, subpairContinuation);

                        }
                        else
                        {
                            continuation.OnChildCompletedEmpty(ref pair.Continuation, ref batcher);
                        }
                    }
                }
            }
        }
    }
}
