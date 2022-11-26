using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Stores references to the points composing one of a convex hull's faces.
    /// </summary>
    public struct HullFace
    {
        public Buffer<int> OriginalVertexMapping;
        public Buffer<int> VertexIndices;

        /// <summary>
        /// Gets the number of vertices in the face.
        /// </summary>
        public int VertexCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return VertexIndices.Length; }
        }

        /// <summary>
        /// Gets the index of the vertex associated with the given face vertex index in the source point set.
        /// </summary>
        /// <param name="index">Index into the face's vertex list.</param>
        /// <returns>Index of the vertex associated with the given face vertex index in the source point set.</returns>
        public int this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return OriginalVertexMapping[VertexIndices[index]]; }
        }
    }

    /// <summary>
    /// Raw data representing a convex hull.
    /// </summary>
    /// <remarks>This is not yet transformed into a runtime format. It requires additional processing to be used in a ConvexHull shape; see ConvexHullHelper.ProcessHull.</remarks>
    public struct HullData
    {
        /// <summary>
        /// Mapping of points on the convex hull back to the original point set.
        /// </summary>
        public Buffer<int> OriginalVertexMapping;
        /// <summary>
        /// List of indices composing the faces of the hull. Individual faces indexed by the FaceIndices.
        /// </summary>
        public Buffer<int> FaceVertexIndices;
        /// <summary>
        /// Starting index in the FaceVertexIndices for each face.
        /// </summary>
        public Buffer<int> FaceStartIndices;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetFace(int faceIndex, out HullFace face)
        {
            var nextFaceIndex = faceIndex + 1;
            var start = FaceStartIndices[faceIndex];
            var end = nextFaceIndex == FaceStartIndices.Length ? FaceVertexIndices.Length : FaceStartIndices[nextFaceIndex];
            FaceVertexIndices.Slice(start, end - start, out face.VertexIndices);
            face.OriginalVertexMapping = OriginalVertexMapping;
        }

        public void Dispose(BufferPool pool)
        {
            pool.Return(ref OriginalVertexMapping);
            pool.Return(ref FaceVertexIndices);
            pool.Return(ref FaceStartIndices);
        }
    }

    /// <summary>
    /// Helper methods to create and process convex hulls from point clouds.
    /// </summary>
    public static class ConvexHullHelper
    {
        static void FindExtremeFace(in Vector3Wide basisX, in Vector3Wide basisY, in Vector3Wide basisOrigin, in EdgeEndpoints sourceEdgeEndpoints, ref Buffer<Vector3Wide> pointBundles, in Vector<int> indexOffsets, int pointCount,
            ref Buffer<Vector<float>> projectedOnX, ref Buffer<Vector<float>> projectedOnY, in Vector<float> planeEpsilon, ref QuickList<int> vertexIndices, out Vector3 faceNormal)
        {
            Debug.Assert(projectedOnX.Length >= pointBundles.Length && projectedOnY.Length >= pointBundles.Length && vertexIndices.Count == 0 && vertexIndices.Span.Length >= pointBundles.Length * Vector<float>.Count);
            //Find the candidate-basisOrigin which has the smallest angle with basisY when projected onto the plane spanned by basisX and basisY.
            //angle = atan(y / x)
            //tanAngle = y / x
            //x is guaranteed to be nonnegative, so its sign doesn't change.
            //tanAngle is monotonically increasing with respect to y / x, so a higher angle corresponds to a higher y/x, always.
            //We can then compare samples 0 and 1 using:
            //tanAngle0 > tanAngle1
            //y0 / x0 > y1 / x1
            //y0 * x1 > y1 * x0
            Vector3Wide.Subtract(pointBundles[0], basisOrigin, out var toCandidate);
            ref var x = ref projectedOnX[0];
            ref var y = ref projectedOnY[0];
            Vector3Wide.Dot(basisX, toCandidate, out x);
            //If x is negative, that means some numerical issue has resulted in a point beyond the bounding plane that generated this face request.
            //We'll treat it as if it's on the plane. (The reason we bother with this clamp is the sign assumption built into our angle comparison, detailed above.)
            x = Vector.Max(Vector<float>.Zero, x);
            Vector3Wide.Dot(basisY, toCandidate, out y);
            var bestY = y;
            var bestX = x;
            //Ignore the source edge.
            var edgeIndexA = new Vector<int>(sourceEdgeEndpoints.A);
            var edgeIndexB = new Vector<int>(sourceEdgeEndpoints.B);
            var pointCountBundle = new Vector<int>(pointCount);
            //Note that any slot that would have been coplanar with the generating face *and* behind the edge (that is, a vertex almost certainly associated with the generating face) is ignored.
            //Without this condition, it's possible for numerical cycles to occur where a face finds itself over and over again.
            var ignoreSlot = Vector.BitwiseOr(
                Vector.BitwiseOr(Vector.GreaterThanOrEqual(indexOffsets, pointCountBundle), Vector.BitwiseAnd(Vector.LessThanOrEqual(bestX, planeEpsilon), Vector.LessThanOrEqual(bestY, planeEpsilon))),
                Vector.BitwiseOr(Vector.Equals(indexOffsets, edgeIndexA), Vector.Equals(indexOffsets, edgeIndexB)));
            bestX = Vector.ConditionalSelect(ignoreSlot, Vector<float>.One, bestX);
            bestY = Vector.ConditionalSelect(ignoreSlot, new Vector<float>(float.MinValue), bestY);
            var bestIndices = indexOffsets;
            for (int i = 1; i < pointBundles.Length; ++i)
            {
                Vector3Wide.Subtract(pointBundles[i], basisOrigin, out toCandidate);
                x = ref projectedOnX[i];
                y = ref projectedOnY[i];
                Vector3Wide.Dot(basisX, toCandidate, out x);
                x = Vector.Max(Vector<float>.Zero, x); //Same as earlier- protect against numerical error finding points beyond the bounding plane.
                Vector3Wide.Dot(basisY, toCandidate, out y);

                var candidateIndices = indexOffsets + new Vector<int>(i << BundleIndexing.VectorShift);
                ignoreSlot = Vector.BitwiseOr(
                    Vector.BitwiseOr(Vector.GreaterThanOrEqual(candidateIndices, pointCountBundle), Vector.BitwiseAnd(Vector.LessThanOrEqual(x, planeEpsilon), Vector.LessThanOrEqual(y, planeEpsilon))),
                    Vector.BitwiseOr(Vector.Equals(candidateIndices, edgeIndexA), Vector.Equals(candidateIndices, edgeIndexB)));
                var useCandidate = Vector.AndNot(Vector.GreaterThan(y * bestX, bestY * x), ignoreSlot);
                bestY = Vector.ConditionalSelect(useCandidate, y, bestY);
                bestX = Vector.ConditionalSelect(useCandidate, x, bestX);
                bestIndices = Vector.ConditionalSelect(useCandidate, candidateIndices, bestIndices);
            }
            var bestYNarrow = bestY[0];
            var bestXNarrow = bestX[0];
            var bestIndexNarrow = bestIndices[0];
            for (int i = 1; i < Vector<float>.Count; ++i)
            {
                var candidateNumerator = bestY[i];
                var candidateDenominator = bestX[i];
                if (candidateNumerator * bestXNarrow > bestYNarrow * candidateDenominator)
                {
                    bestYNarrow = candidateNumerator;
                    bestXNarrow = candidateDenominator;
                    bestIndexNarrow = bestIndices[i];
                }
            }
            //We now have the best index, but there may have been multiple vertices on the same plane. Capture all of them at once by doing a second pass over the results.
            //The plane normal we want to examine is (-bestY, bestX) / ||(-bestY, bestX)||.
            //(This isn't wonderfully fast, but it's fairly simple. The alternatives are things like incrementally combining coplanar triangles as they are discovered
            //or using a postpass that looks for coplanar triangles after they've been created.)
            //Rotate the offset to point outward.
            var projectedPlaneNormalNarrow = Vector2.Normalize(new Vector2(-bestYNarrow, bestXNarrow));
            Vector2Wide.Broadcast(projectedPlaneNormalNarrow, out var projectedPlaneNormal);
            Vector3Wide.ReadFirst(basisX, out var basisXNarrow);
            Vector3Wide.ReadFirst(basisY, out var basisYNarrow);
            faceNormal = basisXNarrow * projectedPlaneNormalNarrow.X + basisYNarrow * projectedPlaneNormalNarrow.Y;

            //if (sourceEdgeEndpoints.A != sourceEdgeEndpoints.B)
            //{
            //    BundleIndexing.GetBundleIndices(sourceEdgeEndpoints.A, out var bundleA, out var innerA);
            //    BundleIndexing.GetBundleIndices(sourceEdgeEndpoints.B, out var bundleB, out var innerB);
            //    BundleIndexing.GetBundleIndices(bestIndexNarrow, out var bundleC, out var innerC);
            //    Vector3Wide.ReadSlot(ref pointBundles[bundleA], innerA, out var a);
            //    Vector3Wide.ReadSlot(ref pointBundles[bundleB], innerB, out var b);
            //    Vector3Wide.ReadSlot(ref pointBundles[bundleC], innerC, out var c);
            //    var faceNormalFromCross = Vector3.Normalize(Vector3.Cross(c - a, b - a));
            //    var testDot = Vector3.Dot(faceNormalFromCross, faceNormal);
            //    var faceNormalError = faceNormal - faceNormalFromCross;
            //    faceNormal = faceNormalFromCross;
            //}

            Vector3Wide.Broadcast(faceNormal, out var faceNormalWide);
            for (int i = 0; i < pointBundles.Length; ++i)
            {
                var dot = projectedOnX[i] * projectedPlaneNormal.X + projectedOnY[i] * projectedPlaneNormal.Y;
                //var dot2 = Vector3Wide.Dot(pointBundles[i] - basisOrigin, faceNormalWide);
                //var error = dot2 - dot;
                //if (Vector.GreaterThanAny(Vector.Abs(error), planeEpsilon))
                //{
                //    for (int j = 0; j < Vector<float>.Count; ++j)
                //    {
                //        if (MathF.Abs(error[j]) > planeEpsilon[j])
                //        {
                //            Console.WriteLine($"error: {error[j]}");
                //        }
                //    }
                //}
                var coplanar = Vector.LessThanOrEqual(Vector.Abs(dot), planeEpsilon);
                if (Vector.LessThanAny(coplanar, Vector<int>.Zero))
                {
                    var bundleBaseIndex = i << BundleIndexing.VectorShift;
                    var localIndexMaximum = pointCount - bundleBaseIndex;
                    if (localIndexMaximum > Vector<int>.Count)
                        localIndexMaximum = Vector<int>.Count;
                    for (int j = 0; j < localIndexMaximum; ++j)
                    {
                        if (coplanar[j] < 0)
                        {
                            vertexIndices.AllocateUnsafely() = bundleBaseIndex + j;
                        }
                    }
                }
            }
            //Vector3Wide.ReadFirst(basisX, out var basisXNarrow);
            //Vector3Wide.ReadFirst(basisY, out var basisYNarrow);
            //faceNormal = basisXNarrow * projectedPlaneNormalNarrow.X + basisYNarrow * projectedPlaneNormalNarrow.Y;
        }


        static int FindNextIndexForFaceHull(Vector2 start, Vector2 previousEdgeDirection, float planeEpsilon, ref QuickList<Vector2> facePoints)
        {
            //Use a AOS version since the number of points on a given face will tend to be very small in most cases.
            //Same idea as the 3d version- find the next edge which is closest to the previous edge. Not going to worry about collinear points here for now.
            var bestIndex = -1;
            float best = -float.MaxValue;
            float bestDistanceSquared = 0;
            var startToCandidate = facePoints[0] - start;
            var xDirection = new Vector2(previousEdgeDirection.Y, -previousEdgeDirection.X);
            var candidateX = Vector2.Dot(startToCandidate, xDirection);
            var candidateY = Vector2.Dot(startToCandidate, previousEdgeDirection);
            var currentEdgeDirectionX = previousEdgeDirection.X;
            var currentEdgeDirectionY = previousEdgeDirection.Y;
            if (candidateX > 0)
            {
                best = candidateY / candidateX;
                bestIndex = 0;
                bestDistanceSquared = candidateX * candidateX + candidateY * candidateY;
                var inverseBestDistance = 1f / MathF.Sqrt(bestDistanceSquared);
                currentEdgeDirectionX = candidateX * inverseBestDistance;
                currentEdgeDirectionY = candidateY * inverseBestDistance;
            }
            for (int i = 1; i < facePoints.Count; ++i)
            {
                startToCandidate = facePoints[i] - start;
                candidateY = Vector2.Dot(startToCandidate, previousEdgeDirection);
                candidateX = Vector2.Dot(startToCandidate, xDirection);
                //Any points that are collinear *with the previous edge* cannot be a part of the current edge without numerical failure; the previous edge should include them.
                if (candidateX <= 0)
                {
                    Debug.Assert(candidateY <= 0,
                        "Previous edge should include any collinear points, so this edge should not see any further collinear points beyond its start." +
                        "If you run into this, it implies you've found some content that violates the convex huller's assumptions, and I'd appreciate it if you reported it on github.com/bepu/bepuphysics2/issues!" +
                        "A .obj or other simple demos-compatible reproduction case would help me fix it.");
                    continue;
                }
                //We accept a candidate if it is either:
                //1) collinear with the previous best by the plane epsilon test BUT is more distant, or
                //2) has a greater angle than the previous best.
                var planeOffset = -candidateX * currentEdgeDirectionY + candidateY * currentEdgeDirectionX;
                if (MathF.Abs(planeOffset) < planeEpsilon)
                {
                    //The candidate is collinear. Only accept it if it's further away.
                    if (candidateX * candidateX + candidateY * candidateY <= bestDistanceSquared)
                    {
                        continue;
                    }
                }
                else if (candidateY < best * candidateX) //candidateY / candidateX < best, given candidate X > 0; just avoiding a division for bulk testing.
                {
                    //Candidate is a smaller angle. Rejected.
                    continue;
                }
                best = candidateY / candidateX;
                bestDistanceSquared = candidateX * candidateX + candidateY * candidateY;
                var inverseBestDistance = 1f / MathF.Sqrt(bestDistanceSquared);
                currentEdgeDirectionX = candidateX * inverseBestDistance;
                currentEdgeDirectionY = candidateY * inverseBestDistance;
                bestIndex = i;
            }
            //Note that this can return -1 if all points were on top of the start.
            return bestIndex;
        }

        static void ReduceFace(ref QuickList<int> faceVertexIndices, Vector3 faceNormal, Span<Vector3> points, float planeEpsilon, ref QuickList<Vector2> facePoints, ref Buffer<bool> allowVertex, ref QuickList<int> reducedIndices)
        {
            Debug.Assert(facePoints.Count == 0 && reducedIndices.Count == 0 && facePoints.Span.Length >= faceVertexIndices.Count && reducedIndices.Span.Length >= faceVertexIndices.Count);
            for (int i = faceVertexIndices.Count - 1; i >= 0; --i)
            {
                if (!allowVertex[faceVertexIndices[i]])
                    faceVertexIndices.RemoveAt(i);
            }
            if (faceVertexIndices.Count <= 3)
            {
                //Too small to require computing a hull. Copy directly.
                for (int i = 0; i < faceVertexIndices.Count; ++i)
                {
                    reducedIndices.AllocateUnsafely() = faceVertexIndices[i];
                }
                if (faceVertexIndices.Count == 3)
                {
                    //No point in running a full reduction, but we do need to check the winding of the triangle.
                    ref var a = ref points[reducedIndices[0]];
                    ref var b = ref points[reducedIndices[1]];
                    ref var c = ref points[reducedIndices[2]];
                    //Counterclockwise should result in face normal pointing outward.
                    var ab = b - a;
                    var ac = c - a;
                    var uncalibratedNormal = Vector3.Cross(ab, ac);
                    if (uncalibratedNormal.LengthSquared() < 1e-14f)
                    {
                        //The face is degenerate.
                        if (ab.LengthSquared() > 1e-14f)
                        {
                            allowVertex[reducedIndices[2]] = false;
                            reducedIndices.FastRemoveAt(2);
                        }
                        else if (ac.LengthSquared() > 1e-14f)
                        {
                            allowVertex[reducedIndices[1]] = false;
                            reducedIndices.FastRemoveAt(1);
                        }
                        else
                        {
                            allowVertex[reducedIndices[1]] = false;
                            allowVertex[reducedIndices[2]] = false;
                            reducedIndices.Count = 1;
                        }
                    }
                    else
                    {
                        if (Vector3.Dot(faceNormal, uncalibratedNormal) < 0)
                            Helpers.Swap(ref reducedIndices[0], ref reducedIndices[1]);
                    }
                }
                return;
            }
            Helpers.BuildOrthonormalBasis(faceNormal, out var basisX, out var basisY);
            Vector2 centroid = default;
            for (int i = 0; i < faceVertexIndices.Count; ++i)
            {
                ref var source = ref points[faceVertexIndices[i]];
                ref var facePoint = ref facePoints.AllocateUnsafely();
                facePoint = new Vector2(Vector3.Dot(basisX, source), Vector3.Dot(basisY, source));
                centroid += facePoint;
            }
            centroid /= faceVertexIndices.Count;
            var greatestDistanceSquared = -1f;
            var initialIndex = 0;
            for (int i = 0; i < faceVertexIndices.Count; ++i)
            {
                ref var facePoint = ref facePoints[i];
                var distanceSquared = (facePoint - centroid).LengthSquared();
                if (greatestDistanceSquared < distanceSquared)
                {
                    greatestDistanceSquared = distanceSquared;
                    initialIndex = i;
                }
            }

            if (greatestDistanceSquared < 1e-14f)
            {
                //The face is degenerate.
                for (int i = 0; i < faceVertexIndices.Count; ++i)
                {
                    allowVertex[faceVertexIndices[i]] = false;
                }
                return;
            }
            var greatestDistance = (float)Math.Sqrt(greatestDistanceSquared);
            var initialOffsetDirection = (facePoints[initialIndex] - centroid) / greatestDistance;
            var previousEdgeDirection = new Vector2(initialOffsetDirection.Y, -initialOffsetDirection.X);
            reducedIndices.AllocateUnsafely() = faceVertexIndices[initialIndex];

            var previousEndIndex = initialIndex;
            while (true)
            {
                //This can return -1 in the event of a completely degenerate face.
                var nextIndex = FindNextIndexForFaceHull(facePoints[previousEndIndex], previousEdgeDirection, planeEpsilon, ref facePoints);
                if (nextIndex == -1 || nextIndex == initialIndex)
                {
                    break;
                }
                reducedIndices.AllocateUnsafely() = faceVertexIndices[nextIndex];
                previousEdgeDirection = Vector2.Normalize(facePoints[nextIndex] - facePoints[previousEndIndex]);
                previousEndIndex = nextIndex;
            }

            //Ignore any vertices which were not on the outer boundary of the face.
            for (int i = 0; i < faceVertexIndices.Count; ++i)
            {
                var index = faceVertexIndices[i];
                if (!reducedIndices.Contains(index))
                {
                    allowVertex[index] = false;
                }
            }
        }

        [StructLayout(LayoutKind.Explicit)]
        public struct EdgeEndpoints : IEqualityComparerRef<EdgeEndpoints>
        {
            [FieldOffset(0)]
            public int A;
            [FieldOffset(4)]
            public int B;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Equals(ref EdgeEndpoints a, ref EdgeEndpoints b)
            {
                return Unsafe.As<int, long>(ref a.A) == Unsafe.As<int, long>(ref b.A) || (a.A == b.B && a.B == b.A);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Hash(ref EdgeEndpoints item)
            {
                return item.A ^ item.B;
            }

            public override string ToString()
            {
                return $"({A}, {B})";
            }
        }

        /// <summary>
        /// Tracks the faces associated with a detected surface edge. 
        /// During hull calculation, surface edges that have only one face associated with them should have an outstanding entry in the edges to visit set.
        /// Surface edges can never validly have more than two faces associated with them. Two means that an edge is 'complete', or should be.
        /// Detecting a third edge implies an error condition. By tracking *which* faces were associated with an edge, we can attempt to fix the error.
        /// This type of error tends to occur when there is a numerical disagreement about what vertices are coplanar with a face.
        /// One iteration could find what it thinks is a complete face, and a later iteration ends up finding more vertices *including* the ones already contained in the previous face.
        /// That's a recipe for excessive edge faces, but it's also a direct indicator that we should merge the involved faces for being close enough to coplanar that the error happened in the first place.
        /// </summary>
        struct EdgeFaceIndices
        {
            public int FaceA;
            public int FaceB;
            public bool Complete => FaceA >= 0 && FaceB >= 0;

            public EdgeFaceIndices(int initialFaceIndex)
            {
                FaceA = initialFaceIndex;
                FaceB = -1;
            }


        }

        static void RemoveFaceFromEdge(int a, int b, int faceIndex, ref QuickDictionary<EdgeEndpoints, EdgeFaceIndices, EdgeEndpoints> facesForEdges)
        {
            EdgeEndpoints edge;
            edge.A = a;
            edge.B = b;
            var exists = facesForEdges.GetTableIndices(ref edge, out _, out var index);
            Debug.Assert(exists, "Whoa something weird happened here! A face was created, but there's a missing edge for its face?");
            ref var facesForEdge = ref facesForEdges.Values[index];
            Debug.Assert(faceIndex == facesForEdge.FaceA || faceIndex == facesForEdge.FaceB, "If you're trying to remove a face index from the edge, it better be in the edge!");
            if (facesForEdge.FaceA == faceIndex)
            {
                facesForEdge.FaceA = facesForEdge.FaceB;
                facesForEdge.FaceB = -1;
                if (facesForEdge.FaceA == -1)
                {
                    //This edge no longer has any faces associated with it. 
                    facesForEdges.FastRemove(ref edge);
                }
            }
            else
            {
                facesForEdge.FaceB = -1;
            }
            //Note that we do *not* add the edge back into the 'edges to test' list when transitioning from 2 faces to 1 face for the edge.
            //This function is invoked when an edge is being deleted because it's related to a deleted face.
            //There are two possible outcomes:
            //1. The completion of the merge will see a face added back to this edge,
            //2. it's an orphaned internal edge that should not be tested.
        }

        internal struct EarlyFace
        {
            public QuickList<int> VertexIndices;
            public bool Deleted;
            public Vector3 Normal;
        }

        struct EdgeToTest
        {
            public EdgeEndpoints Endpoints;
            public Vector3 FaceNormal;
            public int FaceIndex;
        }


        static void AddFace(ref QuickList<EarlyFace> faces, BufferPool pool, Vector3 normal, QuickList<int> vertexIndices)
        {
            ref var face = ref faces.Allocate(pool);
            face = new EarlyFace { Normal = normal, VertexIndices = new QuickList<int>(vertexIndices.Count, pool) };
            face.VertexIndices.AddRangeUnsafely(vertexIndices);
        }

        static void AddFaceToEdgesAndTestList(BufferPool pool,
            ref QuickList<int> reducedFaceIndices,
            ref QuickList<EdgeToTest> edgesToTest,
            ref QuickDictionary<EdgeEndpoints, EdgeFaceIndices, EdgeEndpoints> facesForEdges,
            Vector3 faceNormal, int newFaceIndex)
        {
            facesForEdges.EnsureCapacity(facesForEdges.Count + reducedFaceIndices.Count, pool);
            var previousIndex = reducedFaceIndices[reducedFaceIndices.Count - 1];
            for (int i = 0; i < reducedFaceIndices.Count; ++i)
            {
                EdgeEndpoints endpoints;
                endpoints.A = previousIndex;
                endpoints.B = reducedFaceIndices[i];
                previousIndex = endpoints.B;
                if (facesForEdges.GetTableIndices(ref endpoints, out var tableIndex, out var elementIndex))
                {
                    ref var edgeFaceCount = ref facesForEdges.Values[elementIndex];
                    Debug.Assert(edgeFaceCount.FaceB == -1,
                        "While we let execution continue, this is an error condition and implies overlapping triangles are being generated." +
                        "This tends to happen when there are many near-coplanar vertices, so numerical tolerances across different faces cannot consistently agree.");
                    edgeFaceCount.FaceB = newFaceIndex;
                }
                else
                {
                    //This edge is not yet claimed by any edge. Claim it for the new face and add the edge for further testing.
                    EdgeToTest nextEdgeToTest;
                    nextEdgeToTest.Endpoints = endpoints;
                    nextEdgeToTest.FaceNormal = faceNormal;
                    nextEdgeToTest.FaceIndex = newFaceIndex;
                    facesForEdges.Keys[facesForEdges.Count] = nextEdgeToTest.Endpoints;
                    facesForEdges.Values[facesForEdges.Count] = new EdgeFaceIndices(newFaceIndex);
                    //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
                    facesForEdges.Table[tableIndex] = ++facesForEdges.Count;
                    edgesToTest.Allocate(pool) = nextEdgeToTest;
                }
            }
        }

        static void AddIfNotPresent(ref QuickList<int> list, int value, BufferPool pool)
        {
            if (!list.Contains(value))
                list.Allocate(pool) = value;
        }

        //public struct DebugStep
        //{
        //    public EdgeEndpoints SourceEdge;
        //    public List<int> Raw;
        //    public List<int> Reduced;
        //    public bool[] AllowVertex;
        //    public Vector3 FaceNormal;
        //    public Vector3 BasisX;
        //    public Vector3 BasisY;
        //    public List<int> FaceStarts;
        //    public List<int> FaceIndices;
        //    public bool[] FaceDeleted;
        //    public int[] MergedFaceIndices;
        //    public int FaceIndex;
        //    public Vector3[] FaceNormals;

        //    internal DebugStep(EdgeEndpoints sourceEdge, ref QuickList<int> raw, Vector3 faceNormal, Vector3 basisX, Vector3 basisY, ref QuickList<int> reduced, ref Buffer<bool> allowVertex, ref QuickList<EarlyFace> faces, Span<int> mergedFaceIndices, int faceIndex)
        //    {
        //        SourceEdge = sourceEdge;
        //        FaceNormal = faceNormal;
        //        BasisX = basisX;
        //        BasisY = basisY;
        //        Raw = new List<int>();
        //        for (int i = 0; i < raw.Count; ++i)
        //        {
        //            Raw.Add(raw[i]);
        //        }
        //        Reduced = new List<int>();
        //        for (int i = 0; i < reduced.Count; ++i)
        //        {
        //            Reduced.Add(reduced[i]);
        //        }
        //        AllowVertex = new bool[allowVertex.Length];
        //        for (int i = 0; i < allowVertex.Length; ++i)
        //        {
        //            AllowVertex[i] = allowVertex[i];
        //        }
        //        FaceStarts = new List<int>(faces.Count);
        //        FaceIndices = new List<int>();
        //        FaceDeleted = new bool[faces.Count];
        //        FaceNormals = new Vector3[faces.Count];
        //        for (int i = 0; i < faces.Count; ++i)
        //        {
        //            ref var face = ref faces[i];
        //            FaceStarts.Add(FaceIndices.Count);
        //            for (int j = 0; j < face.VertexIndices.Count; ++j)
        //                FaceIndices.Add(face.VertexIndices[j]);
        //            FaceDeleted[i] = face.Deleted;
        //            FaceNormals[i] = face.Normal;
        //        }
        //        MergedFaceIndices = mergedFaceIndices.ToArray();
        //        FaceIndex = faceIndex;
        //    }
        //}
        ///// <summary>
        ///// Computes the convex hull of a set of points.
        ///// </summary>
        ///// <param name="points">Point set to compute the convex hull of.</param>
        ///// <param name="pool">Buffer pool to pull memory from when creating the hull.</param>
        ///// <param name="hullData">Convex hull of the input point set.</param>
        //public static void ComputeHull(Span<Vector3> points, BufferPool pool, out HullData hullData)
        //{
        //    ComputeHull(points, pool, out hullData, out _);
        //}


        /// <summary>
        /// Computes the convex hull of a set of points.
        /// </summary>
        /// <param name="points">Point set to compute the convex hull of.</param>
        /// <param name="pool">Buffer pool to pull memory from when creating the hull.</param>
        /// <param name="hullData">Convex hull of the input point set.</param>
        public static void ComputeHull(Span<Vector3> points, BufferPool pool, out HullData hullData)//, out List<DebugStep> steps)
        {
            //steps = new List<DebugStep>();
            if (points.Length <= 0)
            {
                hullData = default;
                return;
            }
            if (points.Length <= 3)
            {
                //If the input is too small to actually form a volumetric hull, just output the input directly.
                pool.Take(points.Length, out hullData.OriginalVertexMapping);
                for (int i = 0; i < points.Length; ++i)
                {
                    hullData.OriginalVertexMapping[i] = i;
                }
                if (points.Length == 3)
                {
                    pool.Take(1, out hullData.FaceStartIndices);
                    pool.Take(3, out hullData.FaceVertexIndices);
                    hullData.FaceStartIndices[0] = 0;
                    //No volume, so winding doesn't matter.
                    hullData.FaceVertexIndices[0] = 0;
                    hullData.FaceVertexIndices[1] = 1;
                    hullData.FaceVertexIndices[2] = 2;
                }
                else
                {
                    hullData.FaceStartIndices = default;
                    hullData.FaceVertexIndices = default;
                }
                return;
            }
            var pointBundleCount = BundleIndexing.GetBundleCount(points.Length);
            pool.Take<Vector3Wide>(pointBundleCount, out var pointBundles);
            //While it's not asymptotically optimal in general, gift wrapping is simple and easy to productively vectorize.
            //As a first step, create an AOSOA version of the input data.
            Vector3 centroid = default;
            for (int i = 0; i < points.Length; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                ref var p = ref points[i];
                Vector3Wide.WriteSlot(p, innerIndex, ref pointBundles[bundleIndex]);
                centroid += p;
            }
            centroid /= points.Length;
            //Fill in the last few slots with the centroid.
            //We avoid doing a bunch of special case work on the last partial bundle by just assuming it has a few extra redundant internal points. 
            var bundleSlots = pointBundles.Length * Vector<float>.Count;
            for (int i = points.Length; i < bundleSlots; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                Vector3Wide.WriteSlot(centroid, innerIndex, ref pointBundles[bundleIndex]);
            }

            //Find a starting point. We'll use the one furthest from the centroid.
            Vector3Wide.Broadcast(centroid, out var centroidBundle);
            Helpers.FillVectorWithLaneIndices(out var mostDistantIndicesBundle);
            var indexOffsetBundle = mostDistantIndicesBundle;
            Vector3Wide.DistanceSquared(pointBundles[0], centroidBundle, out var distanceSquaredBundle);
            for (int i = 1; i < pointBundles.Length; ++i)
            {
                var bundleIndices = new Vector<int>(i << BundleIndexing.VectorShift) + indexOffsetBundle;
                Vector3Wide.DistanceSquared(pointBundles[i], centroidBundle, out var distanceSquaredCandidate);
                mostDistantIndicesBundle = Vector.ConditionalSelect(Vector.GreaterThan(distanceSquaredCandidate, distanceSquaredBundle), bundleIndices, mostDistantIndicesBundle);
                distanceSquaredBundle = Vector.Max(distanceSquaredBundle, distanceSquaredCandidate);
            }
            var bestDistanceSquared = distanceSquaredBundle[0];
            var initialIndex = mostDistantIndicesBundle[0];
            for (int i = 1; i < Vector<int>.Count; ++i)
            {
                var distanceCandidate = distanceSquaredBundle[i];
                if (distanceCandidate > bestDistanceSquared)
                {
                    bestDistanceSquared = distanceCandidate;
                    initialIndex = mostDistantIndicesBundle[i];
                }
            }
            BundleIndexing.GetBundleIndices(initialIndex, out var mostDistantBundleIndex, out var mostDistantInnerIndex);
            Vector3Wide.ReadSlot(ref pointBundles[mostDistantBundleIndex], mostDistantInnerIndex, out var initialVertex);

            //All further points will be found by picking an plane on which to project all vertices down onto, and then measuring the angle on that plane.
            //We pick to basis directions along which to measure. For the second point, we choose a perpendicular direction arbitrarily.
            var initialToCentroid = centroid - initialVertex;
            var initialDistance = initialToCentroid.Length();
            if (initialDistance < 1e-7f)
            {
                //The point set lacks any volume or area.
                pool.Take(1, out hullData.OriginalVertexMapping);
                hullData.OriginalVertexMapping[0] = 0;
                hullData.FaceStartIndices = default;
                hullData.FaceVertexIndices = default;
                pool.Return(ref pointBundles);
                return;
            }
            Vector3Wide.Broadcast(initialToCentroid / initialDistance, out var initialBasisX);
            Helpers.FindPerpendicular(initialBasisX, out var initialBasisY); //(broadcasted before FindPerpendicular just because we didn't have a non-bundle version)
            Vector3Wide.Broadcast(initialVertex, out var initialVertexBundle);
            pool.Take<Vector<float>>(pointBundles.Length, out var projectedOnX);
            pool.Take<Vector<float>>(pointBundles.Length, out var projectedOnY);
            var planeEpsilonNarrow = MathF.Sqrt(bestDistanceSquared) * 1e-5f;
            var normalCoplanarityEpsilon = 1f - 1e-5f;
            var planeEpsilon = new Vector<float>(planeEpsilonNarrow);
            var rawFaceVertexIndices = new QuickList<int>(pointBundles.Length * Vector<float>.Count, pool);
            var initialSourceEdge = new EdgeEndpoints { A = initialIndex, B = initialIndex };
            FindExtremeFace(initialBasisX, initialBasisY, initialVertexBundle, initialSourceEdge, ref pointBundles, indexOffsetBundle, points.Length,
               ref projectedOnX, ref projectedOnY, planeEpsilon, ref rawFaceVertexIndices, out var initialFaceNormal);
            Debug.Assert(rawFaceVertexIndices.Count >= 2);
            var facePoints = new QuickList<Vector2>(points.Length, pool);
            var reducedFaceIndices = new QuickList<int>(points.Length, pool);
            //Points found to not be on the face hull are ignored by future executions. 
            pool.Take<bool>(points.Length, out var allowVertex);
            for (int i = 0; i < points.Length; ++i)
                allowVertex[i] = true;

            ReduceFace(ref rawFaceVertexIndices, initialFaceNormal, points, planeEpsilonNarrow, ref facePoints, ref allowVertex, ref reducedFaceIndices);

            var faces = new QuickList<EarlyFace>(points.Length, pool);
            var edgesToTest = new QuickList<EdgeToTest>(points.Length, pool);
            var facesForEdges = new QuickDictionary<EdgeEndpoints, EdgeFaceIndices, EdgeEndpoints>(points.Length, pool);
            var facesNeedingMerge = new QuickList<int>(32, pool);
            if (reducedFaceIndices.Count >= 3)
            {
                //The initial face search found an actual face! That's a bit surprising since we didn't start from an edge offset, but rather an arbitrary direction.
                //Handle it anyway.
                for (int i = 0; i < reducedFaceIndices.Count; ++i)
                {
                    ref var edgeToAdd = ref edgesToTest.Allocate(pool);
                    edgeToAdd.Endpoints.A = reducedFaceIndices[i == 0 ? reducedFaceIndices.Count - 1 : i - 1];
                    edgeToAdd.Endpoints.B = reducedFaceIndices[i];
                    edgeToAdd.FaceNormal = initialFaceNormal;
                    edgeToAdd.FaceIndex = 0;
                    facesForEdges.Add(ref edgeToAdd.Endpoints, new EdgeFaceIndices(0), pool);
                }
                //Since an actual face was found, we go ahead and output it into the face set.
                AddFace(ref faces, pool, initialFaceNormal, reducedFaceIndices);
            }
            else
            {
                Debug.Assert(reducedFaceIndices.Count == 2,
                    "The point set size was verified to be at least 4 earlier, so even in degenerate cases, a second point should be found by the face search.");
                //No actual face was found. That's expected; the arbitrary direction we used for the basis doesn't likely line up with any edges.
                ref var edgeToAdd = ref edgesToTest.Allocate(pool);
                edgeToAdd.Endpoints.A = reducedFaceIndices[0];
                edgeToAdd.Endpoints.B = reducedFaceIndices[1];
                edgeToAdd.FaceNormal = initialFaceNormal;
                edgeToAdd.FaceIndex = -1;
                var edgeOffset = points[edgeToAdd.Endpoints.B] - points[edgeToAdd.Endpoints.A];
                var basisY = Vector3.Cross(edgeOffset, edgeToAdd.FaceNormal);
                var basisX = Vector3.Cross(edgeOffset, basisY);
                if (Vector3.Dot(basisX, edgeToAdd.FaceNormal) > 0)
                    Helpers.Swap(ref edgeToAdd.Endpoints.A, ref edgeToAdd.Endpoints.B);
            }
            //Vector3Wide.ReadFirst(initialBasisX, out var debugInitialBasisX);
            //Vector3Wide.ReadFirst(initialBasisY, out var debugInitialBasisY);
            //steps.Add(new DebugStep(initialSourceEdge, ref rawFaceVertexIndices, initialFaceNormal, debugInitialBasisX, debugInitialBasisY, ref reducedFaceIndices, ref allowVertex, ref faces, default, reducedFaceIndices.Count >= 3 ? 0 : -1));

            int facesDeletedCount = 0;

            while (edgesToTest.Count > 0)
            {
                edgesToTest.Pop(out var edgeToTest);
                //Make sure the new edge hasn't already been filled by another traversal.
                var faceCountIndex = facesForEdges.IndexOf(edgeToTest.Endpoints);
                if (faceCountIndex >= 0 && facesForEdges.Values[faceCountIndex].Complete)
                    continue;

                ref var edgeA = ref points[edgeToTest.Endpoints.A];
                ref var edgeB = ref points[edgeToTest.Endpoints.B];
                var edgeOffset = edgeB - edgeA;
                //The face normal points outward, and the edges should be wound counterclockwise.
                //basisY should point away from the source face.
                var basisY = Vector3.Cross(edgeOffset, edgeToTest.FaceNormal);
                //basisX should point inward.
                var basisX = Vector3.Cross(edgeOffset, basisY);
                basisX = Vector3.Normalize(basisX);
                basisY = Vector3.Normalize(basisY);
                Vector3Wide.Broadcast(basisX, out var basisXBundle);
                Vector3Wide.Broadcast(basisY, out var basisYBundle);
                Vector3Wide.Broadcast(edgeA, out var basisOrigin);
                rawFaceVertexIndices.Count = 0;
                FindExtremeFace(basisXBundle, basisYBundle, basisOrigin, edgeToTest.Endpoints, ref pointBundles, indexOffsetBundle, points.Length, ref projectedOnX, ref projectedOnY, planeEpsilon, ref rawFaceVertexIndices, out var faceNormal);
                reducedFaceIndices.Count = 0;
                facePoints.Count = 0;
                ReduceFace(ref rawFaceVertexIndices, faceNormal, points, planeEpsilonNarrow, ref facePoints, ref allowVertex, ref reducedFaceIndices);
                if (reducedFaceIndices.Count < 3)
                {
                    //steps.Add(new DebugStep(edgeToTest.Endpoints, ref rawFaceVertexIndices, faceNormal, basisX, basisY, ref reducedFaceIndices, ref allowVertex, ref faces, default, -1));
                    //Degenerate face found; don't bother creating work for it.
                    continue;
                }
                var faceCountPriorToAdd = faces.Count;

                facesNeedingMerge.Count = 0;
                while (true)
                {
                    //This implementation bites the bullet pretty hard on numerical problems. They most frequently arise from near coplanar vertices.
                    //It's possible that two iterations see different subsets of 'coplanar' vertices, either causing two faces with near equal normal or
                    //even causing an edge to have more than two faces associated with it.
                    //Before making any modifications to the existing data, iterate over all the edges in the current face to check for any such edges.

                    //While the usual flow here will be to reduce the vertices we just found and accept the result immediately,
                    //it's possible for the just-detected face to end up coplanar with an existing face because of numerical issues that prevented detecting the whole face earlier.
                    //Rather than using more precise math, we detect the failure and merge faces after the fact.
                    //Unfortunately, this can happen recursively! Merging two redundant faces could reveal a third face that also needs to be merged.
                    //This is uncommon, but it needs to be covered.
                    //So, we just stick everything into a while loop and let it iterate until it's done.
                    //The good news is that the merging process won't loop forever- every merge results in a face that is at least as large as any contributor, and any internal points
                    //that get reduced out of consideration are marked with allowVertex[whateverInteriorPointIndex] = false.
                    int previousFaceMergeCount = facesNeedingMerge.Count;
                    var previousEndpointIndex = reducedFaceIndices[reducedFaceIndices.Count - 1];
                    for (int i = 0; i < reducedFaceIndices.Count; ++i)
                    {
                        EdgeEndpoints edgeEndpoints;
                        edgeEndpoints.A = previousEndpointIndex;
                        edgeEndpoints.B = reducedFaceIndices[i];
                        previousEndpointIndex = edgeEndpoints.B;

                        if (facesForEdges.GetTableIndices(ref edgeEndpoints, out var tableIndex, out var elementIndex))
                        {
                            //There is already at least one face associated with this edge. Do we need to merge?
                            ref var edgeFaces = ref facesForEdges.Values[elementIndex];
                            Debug.Assert(edgeFaces.FaceA >= 0);
                            var aDot = Vector3.Dot(faces[edgeFaces.FaceA].Normal, faceNormal);
                            if (edgeFaces.FaceB >= 0)
                            {
                                //The edge already has two faces associated with it. This is definitely a numerical error; separate coplanar faces were generated.
                                //We *must* merge at least one pair of faces.
                                //Note that technically both faces can end up being included, even though we've already tested A and B;
                                //the new face could be a bridge that's close enough to both, even if they were barely too far from each other.
                                var bDot = Vector3.Dot(faces[edgeFaces.FaceB].Normal, faceNormal);
                                if (aDot >= normalCoplanarityEpsilon && bDot >= normalCoplanarityEpsilon)
                                {
                                    AddIfNotPresent(ref facesNeedingMerge, edgeFaces.FaceA, pool);
                                    AddIfNotPresent(ref facesNeedingMerge, edgeFaces.FaceB, pool);
                                }
                                else
                                {
                                    //If both aren't merge candidates, then just pick the one that's closer.
                                    AddIfNotPresent(ref facesNeedingMerge, aDot > bDot ? edgeFaces.FaceA : edgeFaces.FaceB, pool);
                                }
                            }
                            else
                            {
                                //Only one face already present. Check if it's coplanar.
                                if (aDot >= normalCoplanarityEpsilon)
                                    AddIfNotPresent(ref facesNeedingMerge, edgeFaces.FaceA, pool);
                            }
                        }
                    }

                    if (facesNeedingMerge.Count > previousFaceMergeCount)
                    {
                        //This iteration has found more faces which should be merged together.
                        //Accumulate the vertices.
                        for (int i = previousFaceMergeCount; i < facesNeedingMerge.Count; ++i)
                        {
                            ref var sourceFace = ref faces[facesNeedingMerge[i]];
                            for (int j = 0; j < sourceFace.VertexIndices.Count; ++j)
                            {
                                if (!rawFaceVertexIndices.Contains(sourceFace.VertexIndices[j]))
                                {
                                    rawFaceVertexIndices.Allocate(pool) = sourceFace.VertexIndices[j];
                                }
                            }
                        }

                        //Re-reduce.
                        reducedFaceIndices.Count = 0;
                        facePoints.Count = 0;
                        ReduceFace(ref rawFaceVertexIndices, faceNormal, points, planeEpsilonNarrow, ref facePoints, ref allowVertex, ref reducedFaceIndices);
                    }
                    else
                    {
                        //No more faces need to be merged! Go ahead and apply the changes.
                        for (int i = 0; i < facesNeedingMerge.Count; ++i)
                        {
                            //Note that we do not merge into an existing face; for simplicity, we just merge into the face we're going to append and mark the old faces as being deleted.
                            //Note that we do *not* remove deleted faces from the faces list. That would break all the face indices that we've accumulated.
                            var faceIndex = facesNeedingMerge[i];
                            ref var faceToRemove = ref faces[faceIndex];
                            faceToRemove.Deleted = true;

                            //For every edge of any face we're merging, remove the face from the edge lists.
                            var previousIndex = faceToRemove.VertexIndices[faceToRemove.VertexIndices.Count - 1];
                            for (int j = 0; j < faceToRemove.VertexIndices.Count; ++j)
                            {
                                RemoveFaceFromEdge(previousIndex, faceToRemove.VertexIndices[j], faceIndex, ref facesForEdges);
                                previousIndex = faceToRemove.VertexIndices[j];
                            }
                            //Remove any references to this face in the edges to test.
                            int removedEdgesToTestCount = 0;
                            for (int j = 0; j < edgesToTest.Count; ++j)
                            {
                                if (edgesToTest[j].FaceIndex == faceIndex)
                                {
                                    ++removedEdgesToTestCount;
                                }
                                else if (removedEdgesToTestCount > 0)
                                {
                                    //Scoot edges to test back.
                                    edgesToTest[j - removedEdgesToTestCount] = edgesToTest[j];
                                }
                            }
                            edgesToTest.Count -= removedEdgesToTestCount;
                        }

                        //Retain a count of the deleted faces so the post process can handle them more easily.
                        facesDeletedCount += facesNeedingMerge.Count;

                        AddFace(ref faces, pool, faceNormal, reducedFaceIndices);
                        AddFaceToEdgesAndTestList(pool, ref reducedFaceIndices, ref edgesToTest, ref facesForEdges, faceNormal, faceCountPriorToAdd);
                        //steps.Add(new DebugStep(edgeToTest.Endpoints, ref rawFaceVertexIndices, faceNormal, basisX, basisY, ref reducedFaceIndices, ref allowVertex, ref faces, facesNeedingMerge, faceCountPriorToAdd));
                        break;
                    }
                }
            }

            edgesToTest.Dispose(pool);
            facesForEdges.Dispose(pool);
            facePoints.Dispose(pool);
            reducedFaceIndices.Dispose(pool);
            rawFaceVertexIndices.Dispose(pool);
            pool.Return(ref allowVertex);
            pool.Return(ref projectedOnX);
            pool.Return(ref projectedOnY);
            pool.Return(ref pointBundles);

            //for (int i = 0; i < faces.Count; ++i)
            //{
            //    for (int j = i + 1; j < faces.Count; ++j)
            //    {
            //        var dot = Vector3.Dot(faces[i].Normal, faces[j].Normal);
            //        var bothFacesExist = !faces[i].Deleted && !faces[j].Deleted;
            //        if (dot >= normalCoplanarityEpsilon && bothFacesExist)
            //        {
            //            Console.WriteLine($"Dot {dot} on faces {i} and {j}");
            //        }
            //    }
            //}

            if (facesDeletedCount > 0)
            {
                //During execution, some faces were found to be coplanar and got merged. To avoid breaking face index references during execution, we left those triangles in place.
                //Now, though, we need to get rid of any faces that are marked deleted!
                int shift = 0;
                for (int i = 0; i < faces.Count; ++i)
                {
                    if (faces[i].Deleted)
                    {
                        //Removing the face from the list, so we gotta dispose it now rather than later.
                        faces[i].VertexIndices.Dispose(pool);
                        ++shift;
                    }
                    else if (shift > 0)
                    {
                        faces[i - shift] = faces[i];
                    }
                }
                Debug.Assert(facesDeletedCount == shift);
                faces.Count -= facesDeletedCount;
            }

            //Create a reduced hull point set from the face vertex references.
            int totalIndexCount = 0;
            for (int i = 0; i < faces.Count; ++i)
            {
                totalIndexCount += faces[i].VertexIndices.Count;
            }
            pool.Take(faces.Count, out hullData.FaceStartIndices);
            pool.Take(totalIndexCount, out hullData.FaceVertexIndices);
            var nextStartIndex = 0;
            pool.Take<int>(points.Length, out var originalToHullIndexMapping);
            var hullToOriginalIndexMapping = new QuickList<int>(points.Length, pool);
            for (int i = 0; i < points.Length; ++i)
            {
                originalToHullIndexMapping[i] = -1;
            }
            for (int i = 0; i < faces.Count; ++i)
            {
                var source = faces[i].VertexIndices;
                hullData.FaceStartIndices[i] = nextStartIndex;
                for (int j = 0; j < source.Count; ++j)
                {
                    var originalVertexIndex = source[j];
                    ref var originalToHull = ref originalToHullIndexMapping[originalVertexIndex];
                    if (originalToHull < 0)
                    {
                        //This vertex hasn't been seen yet.
                        originalToHull = hullToOriginalIndexMapping.Count;
                        hullToOriginalIndexMapping.AllocateUnsafely() = originalVertexIndex;
                    }
                    hullData.FaceVertexIndices[nextStartIndex + j] = originalToHull;
                }
                nextStartIndex += source.Count;
            }

            pool.Take(hullToOriginalIndexMapping.Count, out hullData.OriginalVertexMapping);
            hullToOriginalIndexMapping.Span.CopyTo(0, hullData.OriginalVertexMapping, 0, hullToOriginalIndexMapping.Count);

            pool.Return(ref originalToHullIndexMapping);
            hullToOriginalIndexMapping.Dispose(pool);
            for (int i = 0; i < faces.Count; ++i)
            {
                faces[i].VertexIndices.Dispose(pool);
            }
            faces.Dispose(pool);
            facesNeedingMerge.Dispose(pool);
        }


        /// <summary>
        /// Processes hull data into a runtime usable convex hull shape. Recenters the convex hull's points around its center of mass.
        /// </summary>
        /// <param name="points">Point array into which the hull data indexes.</param>
        /// <param name="hullData">Raw input data to process.</param>
        /// <param name="pool">Pool used to allocate resources for the hullShape.</param>
        /// <param name="hullShape">Convex hull shape created from the input data.</param>
        /// <param name="center">Computed center of mass of the convex hull before its points were recentered onto the origin.</param>
        public static void CreateShape(Span<Vector3> points, HullData hullData, BufferPool pool, out Vector3 center, out ConvexHull hullShape)
        {
            hullShape = default;
            if (hullData.OriginalVertexMapping.Length < 3)
            {
                center = default;
                if (hullData.OriginalVertexMapping.Length > 0)
                {
                    for (int i = 0; i < hullData.OriginalVertexMapping.Length; ++i)
                    {
                        center += points[hullData.OriginalVertexMapping[i]];
                    }
                    center /= hullData.OriginalVertexMapping.Length;
                }
                return;
            }
            var pointBundleCount = BundleIndexing.GetBundleCount(hullData.OriginalVertexMapping.Length);
            pool.Take(pointBundleCount, out hullShape.Points);

            float volume = 0;
            center = default;
            for (int faceIndex = 0; faceIndex < hullData.FaceStartIndices.Length; ++faceIndex)
            {
                hullData.GetFace(faceIndex, out var face);
                for (int subtriangleIndex = 2; subtriangleIndex < face.VertexCount; ++subtriangleIndex)
                {
                    var a = points[face[0]];
                    var b = points[face[subtriangleIndex - 1]];
                    var c = points[face[subtriangleIndex]];
                    var volumeContribution = MeshInertiaHelper.ComputeTetrahedronVolume(a, b, c);
                    volume += volumeContribution;
                    center += (a + b + c) * volumeContribution;
                }
            }
            //Division by 4 since we accumulated (a + b + c), rather than the actual tetrahedral center (a + b + c + 0) / 4.
            center /= volume * 4;

            var lastIndex = hullData.OriginalVertexMapping.Length - 1;
            for (int bundleIndex = 0; bundleIndex < hullShape.Points.Length; ++bundleIndex)
            {
                ref var bundle = ref hullShape.Points[bundleIndex];
                for (int innerIndex = 0; innerIndex < Vector<float>.Count; ++innerIndex)
                {
                    var index = (bundleIndex << BundleIndexing.VectorShift) + innerIndex;
                    //We duplicate the last vertices in the hull. It has no impact on performance; the vertex bundles are executed all or nothing.
                    if (index > lastIndex)
                        index = lastIndex;
                    ref var point = ref points[hullData.OriginalVertexMapping[index]];
                    Vector3Wide.WriteSlot(point - center, innerIndex, ref bundle);
                }
            }

            //Create the face->vertex mapping.
            pool.Take(hullData.FaceStartIndices.Length, out hullShape.FaceToVertexIndicesStart);
            hullData.FaceStartIndices.CopyTo(0, hullShape.FaceToVertexIndicesStart, 0, hullShape.FaceToVertexIndicesStart.Length);
            pool.Take(hullData.FaceVertexIndices.Length, out hullShape.FaceVertexIndices);
            for (int i = 0; i < hullShape.FaceVertexIndices.Length; ++i)
            {
                BundleIndexing.GetBundleIndices(hullData.FaceVertexIndices[i], out var bundleIndex, out var innerIndex);
                ref var faceVertex = ref hullShape.FaceVertexIndices[i];
                faceVertex.BundleIndex = (ushort)bundleIndex;
                faceVertex.InnerIndex = (ushort)innerIndex;
            }

            //Create bounding planes.
            var faceBundleCount = BundleIndexing.GetBundleCount(hullShape.FaceToVertexIndicesStart.Length);
            pool.Take(faceBundleCount, out hullShape.BoundingPlanes);
            for (int i = 0; i < hullShape.FaceToVertexIndicesStart.Length; ++i)
            {
                hullShape.GetVertexIndicesForFace(i, out var faceVertexIndices);
                Debug.Assert(faceVertexIndices.Length >= 3, "We only allow the creation of convex hulls around point sets with, at minimum, some area, so all faces should have at least 3 points.");
                //Note that we sum up contributions from all the constituent triangles.
                //This avoids hitting any degenerate face triangles and smooths out small numerical deviations.
                //(It's mathematically equivalent to taking a weighted average by area, since the magnitude of the cross product is proportional to area.)
                Vector3 faceNormal = default;
                hullShape.GetPoint(faceVertexIndices[0], out var facePivot);
                hullShape.GetPoint(faceVertexIndices[1], out var faceVertex);
                var previousOffset = faceVertex - facePivot;
                for (int j = 2; j < faceVertexIndices.Length; ++j)
                {
                    //Normal points outward.
                    hullShape.GetPoint(faceVertexIndices[j], out faceVertex);
                    var offset = faceVertex - facePivot;
                    faceNormal += Vector3.Cross(previousOffset, offset);
                    previousOffset = offset;
                }
                var length = faceNormal.Length();
                Debug.Assert(length > 1e-10f, "Convex hull procedure should not output degenerate faces.");
                faceNormal /= length;
                BundleIndexing.GetBundleIndices(i, out var boundingPlaneBundleIndex, out var boundingPlaneInnerIndex);
                ref var boundingBundle = ref hullShape.BoundingPlanes[boundingPlaneBundleIndex];
                ref var boundingOffsetBundle = ref GatherScatter.GetOffsetInstance(ref boundingBundle, boundingPlaneInnerIndex);
                Vector3Wide.WriteFirst(faceNormal, ref boundingOffsetBundle.Normal);
                GatherScatter.GetFirst(ref boundingOffsetBundle.Offset) = Vector3.Dot(facePivot, faceNormal);
            }

            //Clear any trailing bounding plane data to keep it from contributing.
            var boundingPlaneCapacity = hullShape.BoundingPlanes.Length * Vector<float>.Count;
            for (int i = hullShape.FaceToVertexIndicesStart.Length; i < boundingPlaneCapacity; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                ref var offsetInstance = ref GatherScatter.GetOffsetInstance(ref hullShape.BoundingPlanes[bundleIndex], innerIndex);
                Vector3Wide.WriteFirst(default, ref offsetInstance.Normal);
                GatherScatter.GetFirst(ref offsetInstance.Offset) = float.MinValue;
            }
        }

        /// <summary>
        /// Creates a convex hull shape out of an input point set. Recenters the convex hull's points around its center of mass.
        /// </summary>
        /// <param name="points">Points to use to create the hull.</param>
        /// <param name="pool">Buffer pool used for temporary allocations and the output data structures.</param>
        /// <param name="hullData">Intermediate hull data that got processed into the convex hull.</param>
        /// <param name="center">Computed center of mass of the convex hull before its points were recentered onto the origin.</param>
        /// <param name="convexHull">Convex hull shape of the input point set.</param>
        public static void CreateShape(Span<Vector3> points, BufferPool pool, out HullData hullData, out Vector3 center, out ConvexHull convexHull)
        {
            ComputeHull(points, pool, out hullData);
            CreateShape(points, hullData, pool, out center, out convexHull);
        }

        /// <summary>
        /// Creates a convex hull shape out of an input point set. Recenters the convex hull's points around its center of mass.
        /// </summary>
        /// <param name="points">Points to use to create the hull.</param>
        /// <param name="pool">Buffer pool used for temporary allocations and the output data structures.</param>
        /// <param name="center">Computed center of mass of the convex hull before its points were recentered onto the origin.</param>
        /// <param name="convexHull">Convex hull shape of the input point set.</param>
        public static void CreateShape(Span<Vector3> points, BufferPool pool, out Vector3 center, out ConvexHull convexHull)
        {
            ComputeHull(points, pool, out var hullData);
            CreateShape(points, hullData, pool, out center, out convexHull);
            //Empty input point sets won't allocate.
            if (hullData.OriginalVertexMapping.Allocated)
                hullData.Dispose(pool);
        }


        /// <summary>
        /// Creates a transformed copy of a convex hull.
        /// </summary>
        /// <param name="source">Source convex hull to copy.</param>
        /// <param name="transform">Transform to apply to the hull points.</param>
        /// <param name="targetPoints">Transformed points in the copy target hull.</param>
        /// <param name="targetBoundingPlanes">Transformed bounding planes in the copy target hull.</param>
        public static void CreateTransformedCopy(in ConvexHull source, in Matrix3x3 transform, Buffer<Vector3Wide> targetPoints, Buffer<HullBoundingPlanes> targetBoundingPlanes)
        {
            if (targetPoints.Length < source.Points.Length)
                throw new ArgumentException("Target points buffer cannot hold the copy.", nameof(targetPoints));
            if (targetBoundingPlanes.Length < source.BoundingPlanes.Length)
                throw new ArgumentException("Target bounding planes buffer cannot hold the copy.", nameof(targetBoundingPlanes));
            Matrix3x3Wide.Broadcast(transform, out var transformWide);
            for (int i = 0; i < source.Points.Length; ++i)
            {
                Matrix3x3Wide.TransformWithoutOverlap(source.Points[i], transformWide, out targetPoints[i]);
            }
            Matrix3x3.Invert(transform, out var inverse);
            Matrix3x3Wide.Broadcast(inverse, out var inverseWide);
            for (int i = 0; i < source.BoundingPlanes.Length; ++i)
            {
                Matrix3x3Wide.TransformByTransposedWithoutOverlap(source.BoundingPlanes[i].Normal, inverseWide, out var normal);
                Vector3Wide.Normalize(normal, out targetBoundingPlanes[i].Normal);
            }

            for (int faceIndex = 0; faceIndex < source.FaceToVertexIndicesStart.Length; ++faceIndex)
            {
                //This isn't exactly an optimal implementation- it uses a pretty inefficient gather, but any optimization can wait for it being a problem.
                var vertexIndex = source.FaceVertexIndices[source.FaceToVertexIndicesStart[faceIndex]];
                BundleIndexing.GetBundleIndices(faceIndex, out var bundleIndex, out var indexInBundle);
                Vector3Wide.ReadSlot(ref targetPoints[vertexIndex.BundleIndex], vertexIndex.InnerIndex, out var point);
                Vector3Wide.ReadSlot(ref targetBoundingPlanes[bundleIndex].Normal, indexInBundle, out var normal);
                GatherScatter.Get(ref targetBoundingPlanes[bundleIndex].Offset, indexInBundle) = Vector3.Dot(point, normal);
            }

            //Clear any trailing bounding plane data to keep it from contributing.
            var boundingPlaneCapacity = targetBoundingPlanes.Length * Vector<float>.Count;
            for (int i = source.FaceToVertexIndicesStart.Length; i < boundingPlaneCapacity; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                ref var offsetInstance = ref GatherScatter.GetOffsetInstance(ref targetBoundingPlanes[bundleIndex], innerIndex);
                Vector3Wide.WriteFirst(default, ref offsetInstance.Normal);
                GatherScatter.GetFirst(ref offsetInstance.Offset) = float.MinValue;
            }
        }

        /// <summary>
        /// Creates a transformed copy of a convex hull. FaceVertexIndices and FaceToVertexIndicesStart buffers from the source are reused in the copy target.
        /// Note that disposing two convex hulls with the same buffers will cause errors; disposal must be handled carefully to avoid double freeing the shared buffers.
        /// </summary>
        /// <param name="source">Source convex hull to copy.</param>
        /// <param name="transform">Transform to apply to the hull points.</param>
        /// <param name="pool">Pool from which to allocate the new hull's points and bounding planes buffers.</param>
        /// <param name="target">Target convex hull to copy into. FaceVertexIndices and FaceToVertexIndicesStart buffers are reused from the source.</param>
        public static void CreateTransformedShallowCopy(in ConvexHull source, in Matrix3x3 transform, BufferPool pool, out ConvexHull target)
        {
            pool.Take(source.Points.Length, out target.Points);
            pool.Take(source.BoundingPlanes.Length, out target.BoundingPlanes);
            CreateTransformedCopy(source, transform, target.Points, target.BoundingPlanes);
            target.FaceVertexIndices = source.FaceVertexIndices;
            target.FaceToVertexIndicesStart = source.FaceToVertexIndicesStart;
        }

        /// <summary>
        /// Creates a transformed copy of a convex hull. Unique FaceVertexIndices and FaceToVertexIndicesStart buffers are allocated for the copy target.
        /// </summary>
        /// <param name="source">Source convex hull to copy.</param>
        /// <param name="transform">Transform to apply to the hull points.</param>
        /// <param name="pool">Pool from which to allocate the new hull's buffers.</param>
        /// <param name="target">Target convex hull to copy into.</param>
        public static void CreateTransformedCopy(in ConvexHull source, in Matrix3x3 transform, BufferPool pool, out ConvexHull target)
        {
            pool.Take(source.Points.Length, out target.Points);
            pool.Take(source.BoundingPlanes.Length, out target.BoundingPlanes);
            pool.Take(source.FaceVertexIndices.Length, out target.FaceVertexIndices);
            pool.Take(source.FaceToVertexIndicesStart.Length, out target.FaceToVertexIndicesStart);
            CreateTransformedCopy(source, transform, target.Points, target.BoundingPlanes);
            source.FaceVertexIndices.CopyTo(0, target.FaceVertexIndices, 0, target.FaceVertexIndices.Length);
            source.FaceToVertexIndicesStart.CopyTo(0, target.FaceToVertexIndicesStart, 0, target.FaceToVertexIndicesStart.Length);
        }

    }
}
