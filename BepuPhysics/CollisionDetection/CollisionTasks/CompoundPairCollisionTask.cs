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
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            var testPairs = batch.Buffer.As<TestPair<Compound, Compound>>();
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var pair = ref Buffer<TestPair<Compound, Compound>>.Get(ref batch.Buffer, i);
                Debug.Assert(pair.Shared.Continuation.ChildA == 0 && pair.Shared.Continuation.ChildB == 0 && pair.Shared.Continuation.Type == CollisionContinuationType.Direct,
                    "Compound-involving pairs cannot be marked as children of compound pairs. Convex-convex children of such pairs will be.");
                ref var continuation = ref batcher.NonconvexReductions.CreateContinuation(pair.A.Children.Length * pair.B.Children.Length, batcher.Pool, out var continuationIndex);
                Debug.Assert(pair.Shared.FlipMask == 0, "Compound-compound should be unflippable; they're the same shape type.");
                int nextContinuationChildIndex = 0;
                for (int childAIndex = 0; childAIndex < pair.A.Children.Length; ++childAIndex)
                {
                    ref var childA = ref pair.A.Children[childAIndex];
                    Compound.GetRotatedChildPose(childA.LocalPose, pair.Shared.PoseA.Orientation, out var childAPose);
                    RigidPose childAWorldPose;
                    childAWorldPose.Orientation = childAPose.Orientation;
                    childAWorldPose.Position = childAPose.Position + pair.Shared.PoseA.Position;
                    for (int childBIndex = 0; childBIndex < pair.B.Children.Length; ++childBIndex)
                    {
                        if (batcher.Callbacks.AllowCollisionTesting(pair.Shared.Continuation.PairId, childAIndex, childBIndex))
                        {
                            ref var childB = ref pair.B.Children[childBIndex];
                            //You could avoid recalculating this pose for every childA, but the value is limited and it adds nontrivial complexity. Only bother if it shows up.
                            Compound.GetRotatedChildPose(childB.LocalPose, pair.Shared.PoseB.Orientation, out var childBPose);

                            var childShapeType = childB.ShapeIndex.Type;
                            batcher.Shapes[childShapeType].GetShapeData(childB.ShapeIndex.Index, out var childShapePointer, out var childShapeSize);

                            var continuationChildIndex = nextContinuationChildIndex++;
                            var continuationInfo = new PairContinuation(pair.Shared.Continuation.PairId, childAIndex, childBIndex,
                                CollisionContinuationType.NonconvexReduction, continuationIndex, continuationChildIndex);
                            ref var continuationChild = ref batcher.NonconvexReductions.Continuations[continuationIndex].Children[continuationChildIndex];
                            
                            continuationChild.OffsetA = childAPose.Position;
                            continuationChild.ChildIndexA = childAIndex;
                            continuationChild.OffsetB = childBPose.Position;
                            continuationChild.ChildIndexB = childBIndex;
                            //Move the child into world space to be consistent with the other convex.
                            childBPose.Position += pair.Shared.PoseB.Position;
                            batcher.Add(childA.ShapeIndex, childB.ShapeIndex, ref childAWorldPose, ref childBPose, pair.Shared.SpeculativeMargin, ref continuationInfo);

                        }
                        else
                        {
                            continuation.OnChildCompletedEmpty(ref pair.Shared.Continuation, ref batcher);
                        }
                    }
                }
            }
        }
    }
}
