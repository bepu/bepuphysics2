using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public class ConvexCompoundCollisionTask<TConvex> : CollisionTask where TConvex : struct, IConvexShape
    {
        public ConvexCompoundCollisionTask()
        {
            BatchSize = 8;
            ShapeTypeIndexA = default(TConvex).TypeId;
            ShapeTypeIndexB = default(Compound).TypeId;
            SubtaskGenerator = true;
            PairType = CollisionTaskPairType.StandardPair;
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            var testPairs = batch.Buffer.As<CollisionPair>();
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var pair = ref testPairs[i];
                Debug.Assert(pair.Continuation.ChildA == 0 && pair.Continuation.ChildB == 0 && pair.Continuation.Type == CollisionContinuationType.Direct,
                    "Compound-involving pairs cannot be marked as children of compound pairs. Convex-convex children of such pairs will be.");
                ref var b = ref Unsafe.AsRef<Compound>(pair.B);
                ref var continuation = ref batcher.NonconvexReductions.CreateContinuation(b.Children.Length, batcher.Pool, out var continuationIndex);
                int nextContinuationChildIndex = 0;
                for (int j = 0; j < b.Children.Length; ++j)
                {
                    //Note that we have to take into account whether we flipped the shapes to match the expected memory layout.
                    //The caller expects results according to the submitted pair order, not the batcher's memory layout order.
                    int childA, childB;
                    if (pair.FlipMask < 0)
                    {
                        childA = j;
                        childB = 0;
                    }
                    else
                    {
                        childA = 0;
                        childB = j;
                    }
                    var continuationChildIndex = nextContinuationChildIndex++;
                    var subpairContinuation = new PairContinuation(pair.Continuation.PairId, childA, childB,
                        CollisionContinuationType.NonconvexReduction, continuationIndex, continuationChildIndex);
                    if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                    {
                        ref var child = ref b.Children[j];
                        Compound.GetRotatedChildPose(child.LocalPose, pair.OrientationB, out var childPose);

                        var childShapeType = child.ShapeIndex.Type;
                        batcher.Shapes[childShapeType].GetShapeData(child.ShapeIndex.Index, out var childShapePointer, out var childShapeSize);
                        
                        ref var continuationChild = ref batcher.NonconvexReductions.Continuations[continuationIndex].Children[continuationChildIndex];

                        ref var a = ref Unsafe.AsRef<TConvex>(pair.A);
                        continuationChild.ChildIndexA = childA;
                        continuationChild.ChildIndexB = childB;
                        if (pair.FlipMask < 0)
                        {
                            //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the compound collision task's flip.
                            continuationChild.OffsetA = childPose.Position;
                            continuationChild.OffsetB = default;
                            batcher.AddDirectly(child.ShapeIndex.Type, a.TypeId, childShapePointer, pair.A,
                                -childPose.Position - pair.OffsetB, childPose.Orientation, pair.OrientationA, pair.SpeculativeMargin, subpairContinuation);
                        }
                        else
                        {
                            continuationChild.OffsetA = default;
                            continuationChild.OffsetB = childPose.Position;
                            //Move the child into world space to be consistent with the other convex.
                            batcher.AddDirectly(a.TypeId, child.ShapeIndex.Type, pair.A, childShapePointer,
                                childPose.Position + pair.OffsetB, pair.OrientationA, childPose.Orientation, pair.SpeculativeMargin, subpairContinuation);
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
}
