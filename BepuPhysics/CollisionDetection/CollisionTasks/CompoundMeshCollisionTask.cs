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
    public interface ICompoundMeshOverlapFinder<TCompound, TMesh>
        where TCompound : struct, ICompoundShape
        where TMesh : struct, IMeshShape
    {
        void FindLocalOverlaps(ref Buffer<BoundsTestedPair> pairs, BufferPool pool, ref TaskOverlapsCollection overlaps);
    }


    public class CompoundMeshCollisionTask<TCompound, TMesh, TOverlapFinder> : CollisionTask
        where TCompound : struct, ICompoundShape
        where TMesh : struct, IMeshShape
        where TOverlapFinder : struct, ICompoundMeshOverlapFinder<TCompound, TMesh>
    {
        public CompoundMeshCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(TCompound).TypeId;
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
            var overlaps = new TaskOverlapsCollection(batcher.Pool, batch.Count);
            overlapFinder.FindLocalOverlaps(ref pairs, batcher.Pool, ref overlaps);
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var pairOverlaps = ref overlaps.GetOverlapsForPair(i);
                if (pairOverlaps.Count > 0)
                {
                    ref var pair = ref pairs[i];
                    ref var continuation = ref batcher.MeshReductions.CreateContinuation(pairOverlaps.Count, batcher.Pool, out var continuationIndex);
                    continuation.MeshOrientation = pair.OrientationB;
                    //Pass ownership of the triangles to the continuation. It'll dispose of the buffer.
                    batcher.Pool.Take(pairOverlaps.Count, out continuation.Triangles);
                    ref var compound = ref Unsafe.AsRef<TCompound>(pair.A);
                    ref var mesh = ref Unsafe.AsRef<TMesh>(pair.B);
                    //A flip is required in mesh reduction whenever contacts are being generated as if the triangle is in slot B, which is whenever this pair has *not* been flipped.
                    continuation.RequiresFlip = pair.FlipMask == 0;

                    int nextContinuationChildIndex = 0;
                    for (int j = 0; j < pairOverlaps.Count; ++j)
                    {
                        ref var overlap = ref pairOverlaps[j];
                        //Note that we have to take into account whether we flipped the shapes to match the expected memory layout.
                        //The caller expects results according to the submitted pair order, not the batcher's memory layout order.
                        int childA, childB;
                        if (pair.FlipMask < 0)
                        {
                            childA = overlap.childB;
                            childB = overlap.childA;
                        }
                        else
                        {
                            childA = overlap.childA;
                            childB = overlap.childB;
                        }
                        if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                        {
                            ref var compoundChild = ref compound.GetChild(overlap.childA);
                            var compoundChildType = compoundChild.ShapeIndex.Type;

                            var continuationChildIndex = nextContinuationChildIndex++;
                            var continuationInfo = new PairContinuation(pair.Continuation.PairId, childA, childB,
                                CollisionContinuationType.MeshReduction, continuationIndex, continuationChildIndex);
                            ref var continuationChild = ref continuation.Inner.Children[continuationChildIndex];
                            continuationChild.ChildIndexA = childA;
                            continuationChild.ChildIndexB = childB;

                            batcher.Shapes[compoundChildType].GetShapeData(compoundChild.ShapeIndex.Index, out var compoundChildShapeData, out _);
                            //Note that the triangles list persists until the continuation completes, which means the memory will be validly accessible for all of the spawned collision tasks.
                            //In other words, we can pass a pointer to it to avoid the need for additional batcher shape copying.
                            ref var triangle = ref continuation.Triangles[continuationChildIndex];
                            mesh.GetLocalTriangle(overlap.childB, out triangle);
                            if (pair.FlipMask < 0)
                            {
                                //In meshes, the triangle's vertices already contain the offset, so there is no additional offset.
                                continuationChild.OffsetA = default;
                                continuationChild.OffsetB = compoundChild.LocalPose.Position;
                                //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                batcher.AddDirectly(triangle.TypeId, compoundChildType, Unsafe.AsPointer(ref triangle), compoundChildShapeData,
                                    -pair.OffsetB, pair.OrientationB, pair.OrientationA, pair.SpeculativeMargin, continuationInfo);
                            }
                            else
                            {
                                continuationChild.OffsetA = compoundChild.LocalPose.Position;
                                continuationChild.OffsetB = default;
                                batcher.AddDirectly(compoundChildType, triangle.TypeId, compoundChildShapeData, Unsafe.AsPointer(ref triangle),
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
            overlaps.Dispose();
            //Note that the triangle lists are not disposed here. Those are handed off to the continuations for further analysis.
        }
    }
}
