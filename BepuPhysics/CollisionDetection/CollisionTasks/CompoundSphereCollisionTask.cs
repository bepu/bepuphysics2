using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{

    public static class CompoundCollisionTaskCommon
    {
        //TODO: No vectorization benefit. Need some empirical tuning.
        public const int CompoundConvexBatchSize = 8;

        //Note that this does not attempt to generalize over all compound types. The basic compound type has a special low overhead form- it performs no 
        //pruning. Assumption is that this type is used in cases where there aren't many children and most/all of them will need to be tested anyway.
        public unsafe static void ExecuteBatch<TConvex, TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
            where TConvex : struct, IConvexShape
            where TCallbacks : struct, ICollisionCallbacks
        {
            var testPairs = batch.Buffer.As<TestPair<TConvex, Compound>>();
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var pair = ref Buffer<TestPair<TConvex, Compound>>.Get(ref batch.Buffer, i);
                Debug.Assert(pair.Shared.Source.ChildA == 0 && pair.Shared.Source.ChildB == 0 && pair.Shared.Source.Type == PairReportType.Direct,
                    "Compound-involving pairs cannot be marked as children of compound pairs. Convex-convex children of such pairs will be.");
                for (int j = 0; j < pair.B.Children.Length; ++j)
                {
                    ref var child = ref pair.B.Children[j];
                    Compound.GetWorldPose(ref child.LocalPose, ref pair.Shared.PoseB, out var childWorldPose);
                    var childShapeType = child.ShapeIndex.Type;
                    batcher.Shapes[childShapeType].GetShapeData(child.ShapeIndex.Index, out var childShapePointer, out var childShapeSize);
                    //Note that we can safely take a pointer to the pair-stored shape:
                    //1) It's stored in a buffer, which is guaranteed GC safe
                    //2) The data contained is copied by the time Add returns, so there's no concern about invalid pointers getting stored.
                    batcher.Add(pair.A.TypeId, child.ShapeIndex.Type, Unsafe.SizeOf<TConvex>(), childShapeSize, Unsafe.AsPointer(ref pair.A), childShapePointer,
                        ref pair.Shared.PoseA, ref childWorldPose, pair.Shared.Source.PairId, 0, j, PairReportType.NonconvexReduction);
                }

            }
        }
    }
    public class ConvexCompoundCollisionTask<TConvex> : CollisionTask where TConvex : struct, IConvexShape
    {
        public ConvexCompoundCollisionTask()
        {
            BatchSize = CompoundCollisionTaskCommon.CompoundConvexBatchSize;
            ShapeTypeIndexA = default(TConvex).TypeId;
            ShapeTypeIndexB = default(Compound).TypeId;
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            CompoundCollisionTaskCommon.ExecuteBatch<TConvex, TCallbacks>(ref batch, ref batcher);
        }
    }
}
