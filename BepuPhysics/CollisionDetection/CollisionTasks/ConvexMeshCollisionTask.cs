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

    public class ConvexMeshCollisionTask<TConvex, TConvexWide, TMesh> : CollisionTask
        where TConvex : struct, IConvexShape
        where TConvexWide : struct, IShapeWide<TConvex>
        where TMesh : struct, IMeshShape
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
            //It doesn't matter which mesh instance is used to invoke the mesh functions, so we just grab a representative.
            //(The only reason this exists is a lack of language expressiveness- static interface functions, for example, would eliminate this.)
            ref var meshFunctions = ref Unsafe.AsRef<TMesh>(pairs[0].B);
            TConvexWide convexWide = default;
            Vector3Wide offsetB = default;
            QuaternionWide orientationA = default;
            QuaternionWide orientationB = default;
            Vector3Wide relativeLinearVelocityA = default;
            Vector3Wide angularVelocityA = default;
            Vector3Wide angularVelocityB = default;
            Vector<float> maximumAllowedExpansion = default;
            batcher.Pool.Take<IntPtr>(Vector<float>.Count, out var meshes);
            batcher.Pool.Take<QuickList<int, Buffer<int>>>(Vector<float>.Count, out var meshTriangleIndices);
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                //Overallocating on child indices doesn't really matter. A few kilobytes of wasted space is nothing and doesn't impact memory bandwidth or anything, and avoiding 
                //resizes is handy.
                QuickList<int, Buffer<int>>.Create(batcher.Pool.SpecializeFor<int>(), 128, out meshTriangleIndices[i]);
            }
            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            {
                var count = batch.Count - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;

                //Create the bounding boxes in the rigid local space of the mesh.
                //TODO: This is not necessarily a win. If the compound/mesh type we're querying doesn't benefit from vectorization, 
                //it may be better to just do the bounding box calculations scalar-style. Empirical question.

                for (int j = 0; j < count; ++j)
                {
                    ref var pair = ref pairs[i + j];
                    meshes[j] = (IntPtr)pair.B;
                    Debug.Assert(pair.Continuation.ChildA == 0 && pair.Continuation.ChildB == 0 && pair.Continuation.Type == CollisionContinuationType.Direct,
                        "Mesh-involving pairs cannot be marked as children of other pairs.");
                    GatherScatter.GetOffsetInstance(ref convexWide, j).WriteFirst(ref Unsafe.AsRef<TConvex>(pair.A));
                    Vector3Wide.WriteFirst(pair.OffsetB, ref GatherScatter.GetOffsetInstance(ref offsetB, j));
                    QuaternionWide.WriteFirst(pair.OrientationA, ref GatherScatter.GetOffsetInstance(ref orientationA, j));
                    QuaternionWide.WriteFirst(pair.OrientationB, ref GatherScatter.GetOffsetInstance(ref orientationB, j));
                    Vector3Wide.WriteFirst(pair.RelativeLinearVelocityA, ref GatherScatter.GetOffsetInstance(ref relativeLinearVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityA, ref GatherScatter.GetOffsetInstance(ref angularVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityB, ref GatherScatter.GetOffsetInstance(ref angularVelocityB, j));
                    Unsafe.Add(ref Unsafe.As<Vector<float>, float>(ref maximumAllowedExpansion), j) = pair.MaximumExpansion;
                }
                BoundingBoxHelpers.GetLocalBoundingBox<TConvex, TConvexWide>(ref convexWide, orientationA, angularVelocityA,
                    offsetB, orientationB, relativeLinearVelocityA, angularVelocityB, batcher.Dt, maximumAllowedExpansion, 
                    out var min, out var max);

                meshFunctions.FindLocalOverlaps(ref meshes, ref min, ref max, count, batcher.Pool, ref meshTriangleIndices);

                for (int j = 0; j < count; ++j)
                {
                    ref var triangleIndices = ref meshTriangleIndices[j];
                    if (triangleIndices.Count > 0)
                    {

                        ref var pair = ref pairs[i + j];
                        ref var continuation = ref batcher.MeshReductions.CreateContinuation(triangleIndices.Count, batcher.Pool, out var continuationIndex);
                        continuation.MeshOrientation = pair.OrientationB;
                        //Pass ownership of the triangles to the continuation. It'll dispose of the buffer.
                        batcher.Pool.Take(triangleIndices.Count, out continuation.Triangles);
                        ref var mesh = ref Unsafe.AsRef<TMesh>(pair.B);
                        //A flip is required in mesh reduction whenever contacts are being generated as if the triangle is in slot B, which is whenever this pair has *not* been flipped.
                        continuation.RequiresFlip = pair.FlipMask == 0;

                        int nextContinuationChildIndex = 0;
                        for (int k = 0; k < triangleIndices.Count; ++k)
                        {
                            var triangleIndex = triangleIndices[k];
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
                                //In meshes, the triangle's vertices already contain the offset, so there is no additional offset.
                                continuationChild.OffsetA = default;
                                continuationChild.ChildIndexA = childA;
                                continuationChild.OffsetB = default;
                                continuationChild.ChildIndexB = childB;

                                //Note that the triangles list persists until the continuation completes, which means the memory will be validly accessible for all of the spawned collision tasks.
                                //In other words, we can pass a pointer to it to avoid the need for additional batcher shape copying.
                                ref var triangle = ref continuation.Triangles[continuationChildIndex];
                                mesh.GetLocalTriangle(triangleIndex, out triangle);
                                ref var convex = ref Unsafe.AsRef<TConvex>(pair.A);
                                if (pair.FlipMask < 0)
                                {
                                    //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                    batcher.AddDirectly(triangle.TypeId, convex.TypeId, Unsafe.AsPointer(ref triangle), pair.A,
                                        -pair.OffsetB, pair.OrientationB, pair.OrientationA, pair.SpeculativeMargin, continuationInfo);
                                }
                                else
                                {
                                    batcher.AddDirectly(convex.TypeId, triangle.TypeId, pair.A, Unsafe.AsPointer(ref triangle),
                                        pair.OffsetB, pair.OrientationA, pair.OrientationB, pair.SpeculativeMargin, continuationInfo);
                                }
                            }
                            else
                            {
                                continuation.OnChildCompletedEmpty(ref pair.Continuation, ref batcher);
                            }
                        }
                        triangleIndices.Count = 0;
                    }
                }


            }
            batcher.Pool.ReturnUnsafely(meshes.Id);
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                meshTriangleIndices[i].Dispose(batcher.Pool.SpecializeFor<int>());
            }
            batcher.Pool.ReturnUnsafely(meshTriangleIndices.Id);
            //Note that the triangle lists are not disposed here. Those are handed off to the continuations for further analysis.
        }
    }
}
