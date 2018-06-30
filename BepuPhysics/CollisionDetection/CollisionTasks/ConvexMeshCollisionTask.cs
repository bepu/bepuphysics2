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
    public unsafe interface IMeshShape : IShape
    {
        void FindOverlaps(in Vector3 min, in Vector3 max, BufferPool pool, out QuickList<Triangle, Buffer<Triangle>> overlaps);
        void FindOverlaps(ref Buffer<IntPtr> meshes, in Vector3Wide min, in Vector3Wide max, int count, BufferPool pool,
            ref Buffer<QuickList<Triangle, Buffer<Triangle>>> overlaps, ref Buffer<QuickList<int, Buffer<int>>> childIndices);
    }

    public unsafe struct ConvexMeshTestPair<TConvex> where TConvex : struct, IConvexShape
    {
        public TConvex Convex;
        //Meshes are fairly large, and we require that they are stored in a shapes set, so we can just store a pointer on a per-pair basis.
        public void* Mesh;
        public int FlipMask;
        public Vector3 OffsetToMesh;
        public Quaternion ConvexOrientation;
        public Quaternion MeshOrientation;
        public Vector3 RelativeLinearVelocityA;
        public Vector3 AngularVelocityA;
        public Vector3 AngularVelocityB;
        public float MaximumExpansion;
        public float SpeculativeMargin;
        public PairContinuation Continuation;
    }


    public class ConvexMeshCollisionTask<TConvex, TConvexWide, TMesh, TMeshOverlapFinder> : CollisionTask
        where TConvex : struct, IConvexShape
        where TConvexWide : struct, IShapeWide<TConvex>
        where TMesh : struct, IMeshShape
    {
        public ConvexMeshCollisionTask()
        {
            BatchSize = 8;
            ShapeTypeIndexA = default(TConvex).TypeId;
            ShapeTypeIndexB = default(TMesh).TypeId;
            SubtaskGenerator = true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void Expand(in Vector3Wide expansion, ref Vector3Wide min, ref Vector3Wide max)
        {
            Vector3Wide.Min(Vector<float>.Zero, expansion, out var minExpansion);
            Vector3Wide.Max(Vector<float>.Zero, expansion, out var maxExpansion);
            Vector3Wide.Add(min, minExpansion, out min);
            Vector3Wide.Add(max, maxExpansion, out max);
        }

        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            var pairs = batch.Buffer.As<ConvexMeshTestPair<TConvex>>();
            //It doesn't matter which mesh instance is used to invoke the mesh functions, so we just grab a representative.
            //(The only reason this exists is a lack of language expressiveness- static interface functions, for example, would eliminate this.)
            ref var meshFunctions = ref Unsafe.AsRef<TMesh>(pairs[0].Mesh);
            TConvexWide convexWide = default;
            Vector3Wide offsetB = default;
            QuaternionWide orientationA = default;
            QuaternionWide orientationB = default;
            Vector3Wide relativeLinearVelocityA = default;
            Vector3Wide angularVelocityA = default;
            Vector3Wide angularVelocityB = default;
            Vector<float> maximumAllowedExpansion = default;
            batcher.Pool.Take<IntPtr>(Vector<float>.Count, out var meshes);
            batcher.Pool.Take<QuickList<Triangle, Buffer<Triangle>>>(Vector<float>.Count, out var meshTriangles);
            batcher.Pool.Take<QuickList<int, Buffer<int>>>(Vector<float>.Count, out var meshTriangleIndices);
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
                    meshes[j] = (IntPtr)pair.Mesh;
                    Debug.Assert(pair.Continuation.ChildA == 0 && pair.Continuation.ChildB == 0 && pair.Continuation.Type == CollisionContinuationType.Direct,
                        "Mesh-involving pairs cannot be marked as children of other pairs.");
                    GatherScatter.GetOffsetInstance(ref convexWide, j).WriteFirst(ref pair.Convex);
                    Vector3Wide.WriteFirst(pair.OffsetToMesh, ref GatherScatter.GetOffsetInstance(ref offsetB, j));
                    QuaternionWide.WriteFirst(pair.ConvexOrientation, ref GatherScatter.GetOffsetInstance(ref orientationA, j));
                    QuaternionWide.WriteFirst(pair.MeshOrientation, ref GatherScatter.GetOffsetInstance(ref orientationB, j));
                    Vector3Wide.WriteFirst(pair.RelativeLinearVelocityA, ref GatherScatter.GetOffsetInstance(ref relativeLinearVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityA, ref GatherScatter.GetOffsetInstance(ref angularVelocityA, j));
                    Vector3Wide.WriteFirst(pair.AngularVelocityB, ref GatherScatter.GetOffsetInstance(ref angularVelocityB, j));
                    Unsafe.Add(ref Unsafe.As<Vector<float>, float>(ref maximumAllowedExpansion), j) = pair.MaximumExpansion;
                }
                QuaternionWide.Conjugate(orientationB, out var inverseOrientationB);
                QuaternionWide.TransformWithoutOverlap(offsetB, inverseOrientationB, out var localOffsetB);
                QuaternionWide.ConcatenateWithoutOverlap(orientationA, inverseOrientationB, out var localOrientationA);
                QuaternionWide.TransformWithoutOverlap(relativeLinearVelocityA, inverseOrientationB, out var localRelativeLinearVelocityA);

                convexWide.GetBounds(ref localOrientationA, out var maximumRadius, out var maximumAngularExpansion, out var min, out var max);
                //Note that this angular velocity is not in the local space of the mesh. This is simply used to figure out how much local angular expansion to apply to the convex.
                //Consider what happens when two bodies have the same angular velocity- their relative rotation does not change, so there is no need for local angular expansion.
                //The primary bounds expansion only makes use of the magnitude, so the fact that it's not truly in local space is irrelevant.
                Vector3Wide.Subtract(angularVelocityA, angularVelocityB, out var netAngularVelocity);
                BoundingBoxBatcher.GetBoundsExpansion(ref localRelativeLinearVelocityA, ref netAngularVelocity, batcher.Dt,
                    ref maximumRadius, ref maximumAngularExpansion, out var minExpansion, out var maxExpansion);

                //If any mesh/compound in the batch has angular velocity, we need to compute the bounding box expansion caused by the resulting nonlinear path.
                //(This is equivalent to expanding the bounding boxes of the mesh/compound shapes to account for their motion. It's just much simpler to expand only the incoming convex.
                //Conceptually, you can think of this as if we're fixing our frame of reference on the mesh/compound, and watching how the convex moves. 
                //In the presence of mesh/compound angular velocity, a stationary convex will trace a circular arc.)
                Vector3Wide.LengthSquared(angularVelocityB, out var angularSpeedBSquared);
                if (Vector.GreaterThanAny(angularSpeedBSquared, Vector<float>.Zero))
                {
                    //We need to expand the bounding box by the extent of the circular arc which the convex traces due to the mesh/compound's angular motion.
                    //We'll create two axes and measure the extent of the arc along them.
                    //Note that arcX and arcY are invalid if radius or angular velocity magnitude is zero. We'll handle that with a mask.
                    Vector3Wide.Length(offsetB, out var radius);
                    Vector3Wide.Scale(offsetB, Vector<float>.One / radius, out var arcX);
                    Vector3Wide.CrossWithoutOverlap(angularVelocityB, arcX, out var arcY);
                    Vector3Wide.Normalize(arcY, out arcY);
                    var angularSpeedB = Vector.SquareRoot(angularSpeedBSquared);
                    var angularDisplacement = angularSpeedB * batcher.Dt;
                    //minX is just 0 because of the chosen frame of reference.
                    MathHelper.Cos(Vector.Min(new Vector<float>(MathHelper.Pi), angularDisplacement), out var maxX);
                    MathHelper.Sin(angularDisplacement, out var sinTheta);
                    var minY = Vector.Min(Vector<float>.Zero, sinTheta);
                    MathHelper.Sin(Vector.Min(angularDisplacement, new Vector<float>(MathHelper.PiOver2)), out var maxY);

                    Vector3Wide.Scale(arcX, maxX, out var expansionMaxX);
                    Vector3Wide.Scale(arcY, minY, out var expansionMinY);
                    Vector3Wide.Scale(arcY, maxY, out var expansionMaxY);
                    Expand(expansionMaxX, ref minExpansion, ref maxExpansion);
                    Expand(expansionMinY, ref minExpansion, ref maxExpansion);
                    Expand(expansionMaxY, ref minExpansion, ref maxExpansion);
                    //TODO: Convexes that belong to a compound will also need to include expansion caused by the child motion.
                }

                //Clamp the expansion to the pair imposed limit. Discrete pairs don't need to look beyond their speculative margin.
                Vector3Wide.Min(maximumAllowedExpansion, maxExpansion, out maxExpansion);
                Vector3Wide.Max(-maximumAllowedExpansion, minExpansion, out minExpansion);

                Vector3Wide.Add(minExpansion, min, out min);
                Vector3Wide.Add(maxExpansion, max, out max);

                meshFunctions.FindOverlaps(ref meshes, min, max, count, batcher.Pool, ref meshTriangles, ref meshTriangleIndices);


                for (int j = 0; j < count; ++j)
                {
                    ref var triangles = ref meshTriangles[j];
                    if (triangles.Count > 0)
                    {
                        ref var pair = ref pairs[i + j];
                        ref var triangleIndices = ref meshTriangleIndices[j];
                        ref var continuation = ref batcher.MeshReductions.CreateContinuation(triangles.Count, batcher.Pool, out var continuationIndex);
                        continuation.MeshOrientation = pair.MeshOrientation;
                        //Pass ownership of the triangles to the continuation. It'll dispose of the buffer.
                        continuation.Triangles = triangles.Span;

                        int nextContinuationChildIndex = 0;
                        for (int k = 0; k < triangles.Count; ++k)
                        {
                            //Note that we have to take into account whether we flipped the shapes to match the expected memory layout.
                            //The caller expects results according to the submitted pair order, not the batcher's memory layout order.
                            int childA, childB;
                            if (pair.FlipMask < 0)
                            {
                                childA = triangleIndices[k];
                                childB = 0;
                            }
                            else
                            {
                                childA = 0;
                                childB = triangleIndices[k];
                            }
                            if (batcher.Callbacks.AllowCollisionTesting(pair.Continuation.PairId, childA, childB))
                            {
                                ref var triangle = ref triangles[k];

                                //Note that we can safely take a pointer to the pair-stored shape:
                                //1) It's stored in a buffer, which is guaranteed GC safe
                                //2) The data contained is copied by the time Add returns, so there's no concern about invalid pointers getting stored.
                                var continuationChildIndex = nextContinuationChildIndex++;
                                var continuationInfo = new PairContinuation(pair.Continuation.PairId, childA, childB,
                                    CollisionContinuationType.NonconvexReduction, continuationIndex, continuationChildIndex);
                                ref var continuationChild = ref batcher.NonconvexReductions.Continuations[continuationIndex].Children[continuationChildIndex];
                                //In meshes, the triangle's vertices already contain the offset, so there is no additional offset.
                                continuationChild.OffsetA = default;
                                continuationChild.ChildIndexA = childA;
                                continuationChild.OffsetB = default;
                                continuationChild.ChildIndexB = childB;

                                if (pair.FlipMask < 0)
                                {
                                    //By reversing the order of the parameters, the manifold orientation is flipped. This compensates for the flip induced by order requirements on this task.                          
                                    //batcher.Add(triangle.TypeId, pair.Convex.TypeId, Unsafe.SizeOf<Triangle>(), Unsafe.SizeOf<TConvex>(), Unsafe.AsPointer(ref triangle), Unsafe.AsPointer(ref pair.Convex),
                                    //    -pair.OffsetToMesh, pair.MeshOrientation, pair.ConvexOrientation, pair.SpeculativeMargin, ref continuationInfo);
                                }
                                else
                                {
                                    //batcher.Add(pair.Convex.TypeId, triangle.TypeId, Unsafe.SizeOf<TConvex>(), Unsafe.SizeOf<Triangle>(), Unsafe.AsPointer(ref pair.Convex), Unsafe.AsPointer(ref triangle),
                                    //    pair.OffsetToMesh, pair.ConvexOrientation, pair.MeshOrientation, pair.SpeculativeMargin, ref continuationInfo);
                                }
                            }
                            else
                            {
                                continuation.OnChildCompletedEmpty(ref pair.Continuation, ref batcher);
                            }
                        }
                    }
                }


            }
            batcher.Pool.ReturnUnsafely(meshes.Id);
            batcher.Pool.ReturnUnsafely(meshTriangleIndices.Id);
            //Note that the actual triangle lists are not disposed here. Those are handed off to the continuations for further analysis.
            batcher.Pool.ReturnUnsafely(meshTriangles.Id);
        }
    }
}
