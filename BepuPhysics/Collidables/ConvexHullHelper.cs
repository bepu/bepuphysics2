//#define DEBUG_STEPS
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;


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
            //The other allocations may not exist if the hull is degenerate.
            if (FaceVertexIndices.Allocated)
                pool.Return(ref FaceVertexIndices);
            if (FaceStartIndices.Allocated)
                pool.Return(ref FaceStartIndices);
        }
    }

    /// <summary>
    /// Helper methods to create and process convex hulls from point clouds.
    /// </summary>
    public static class ConvexHullHelper
    {
        static void FindExtremeFace(
            in Vector3Wide basisX, in Vector3Wide basisY, in Vector3Wide basisOrigin, in EdgeEndpoints sourceEdgeEndpoints, ref Buffer<Vector3Wide> pointBundles, in Vector<int> indexOffsets, Buffer<int> allowVertices, int pointCount,
            ref Buffer<Vector<float>> projectedOnX, ref Buffer<Vector<float>> projectedOnY, Vector<float> planeEpsilon, ref QuickList<int> vertexIndices, out Vector3 faceNormal)
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
            //Note that any slot that would have been coplanar with the generating face *and* behind the edge (that is, a vertex almost certainly associated with the generating face) is ignored.
            //Without this condition, it's possible for numerical cycles to occur where a face finds itself over and over again.
            var allowVertexBundles = allowVertices.As<Vector<int>>();
            var ignoreSlot = Vector.BitwiseOr(
                Vector.BitwiseOr(
                    Vector.OnesComplement(allowVertexBundles[0]),
                    Vector.BitwiseAnd(Vector.LessThanOrEqual(bestX, planeEpsilon), Vector.LessThanOrEqual(bestY, planeEpsilon))),
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
                    Vector.BitwiseOr(
                        Vector.OnesComplement(allowVertexBundles[i]),
                        Vector.BitwiseAnd(Vector.LessThanOrEqual(x, planeEpsilon), Vector.LessThanOrEqual(y, planeEpsilon))),
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
            //Note: in unusual corner cases, the above may have accepted zero candidates resulting in a bestXNarrow = 1 and bestYNarrow = float.MinValue.
            //Catching that and ensuring that a reasonable face normal is output avoids a bad face.
            var candidateNormalDirection = new Vector2(-bestYNarrow, bestXNarrow);
            var length = candidateNormalDirection.Length();
            var projectedPlaneNormalNarrow = float.IsFinite(length) ? candidateNormalDirection / length : new Vector2(1, 0);
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
            var negatedPlaneEpsilon = -planeEpsilon;
            for (int i = 0; i < pointBundles.Length; ++i)
            {
                var dot = projectedOnX[i] * projectedPlaneNormal.X + projectedOnY[i] * projectedPlaneNormal.Y;
                var coplanar = Vector.GreaterThan(dot, negatedPlaneEpsilon);
                if (Vector.LessThanAny(coplanar, Vector<int>.Zero))
                {
                    var bundleBaseIndex = i << BundleIndexing.VectorShift;
                    var localIndexMaximum = pointCount - bundleBaseIndex;
                    if (localIndexMaximum > Vector<int>.Count)
                        localIndexMaximum = Vector<int>.Count;
                    for (int j = 0; j < localIndexMaximum; ++j)
                    {
                        var vertexIndex = bundleBaseIndex + j;
                        if (coplanar[j] < 0 && allowVertices[vertexIndex] != 0)
                        {
                            vertexIndices.AllocateUnsafely() = vertexIndex;
                        }
                    }
                }
            }
            //Vector3Wide.ReadFirst(basisX, out var basisXNarrow);
            //Vector3Wide.ReadFirst(basisY, out var basisYNarrow);
            //faceNormal = basisXNarrow * projectedPlaneNormalNarrow.X + basisYNarrow * projectedPlaneNormalNarrow.Y;
        }

        /// <summary>
        /// Finds the next index in the 2D hull of a face on the 3D hull using gift wrapping.
        /// </summary>
        /// <param name="start">Start location of the next edge to identify.</param>
        /// <param name="previousEdgeDirection">2D direction of the previously identified edge.</param>
        /// <param name="planeEpsilon">Epsilon within which to consider points to be coplanar (or here, in the 2D case, collinear).</param>
        /// <param name="facePoints">Points composing the hull face projected onto the face's 2D basis.</param>
        /// <returns>Index of the point in facePoints which is the end point for the next edge segment as identified by gift wrapping.</returns>
        static int FindNextIndexForFaceHull(Vector2 start, Vector2 previousEdgeDirection, float planeEpsilon, ref QuickList<Vector2> facePoints)
        {
            //Find the candidate-basisOrigin which has the smallest angle with basisY when projected onto the plane spanned by basisX and basisY.
            //angle = atan(y / x)
            //tanAngle = y / x
            //x is guaranteed to be nonnegative, so its sign doesn't change.
            //tanAngle is monotonically increasing with respect to y / x, so a higher angle corresponds to a higher y/x, always.
            //We can then compare samples 0 and 1 using:
            //tanAngle0 > tanAngle1
            //y0 / x0 > y1 / x1
            //y0 * x1 > y1 * x0
            var basisX = new Vector2(previousEdgeDirection.Y, -previousEdgeDirection.X);
            var basisY = -previousEdgeDirection;
            var bestX = 1f;
            var bestY = float.MaxValue;
            int bestIndex = -1;
            for (int i = 0; i < facePoints.Count; ++i)
            {
                var candidate = facePoints[i];
                var toCandidate = candidate - start;
                //If x is negative, that means some numerical issue has resulted in a point beyond the bounding plane that generated this face request.
                //We'll treat it as if it's on the plane. (The reason we bother with this clamp is the sign assumption built into our angle comparison, detailed above.)
                var x = float.Max(0, Vector2.Dot(toCandidate, basisX));
                var y = Vector2.Dot(toCandidate, basisY);

                //Note that any slot that would have been coplanar with the generating face *and* behind the edge (that is, a vertex almost certainly associated with the generating face) is ignored.
                //Without this condition, it's possible for numerical cycles to occur where a face finds itself over and over again.
                var ignoreSlot = x <= planeEpsilon && y >= -planeEpsilon;
                var useCandidate = (y * bestX < bestY * x) && !ignoreSlot;
                if (useCandidate)
                {
                    bestY = y;
                    bestX = x;
                    bestIndex = i;
                }
            }
            //If no next index was identified, then the face is degenerate.
            //Stop now to prevent the postpass from identifying some nonsense derived from a garbage plane.
            if (bestIndex == -1)
                return -1;

            //We now have the best index, but there may have been multiple vertices on the same plane. Capture all of them at once by doing a second pass over the results.
            //Note that incrementally tracking distance during the above loop is more complex than it first appears; we want the most distant point within the plane epsilon around best angle,
            //but we don't know the best angle until after the loop terminates. A distant point early in the list could be kicked out by a later change in the plane angle. A postpass makes that easy to discover.
            //The plane normal we want to examine is (-bestY, bestX) / ||(-bestY, bestX)||.
            //Rotate the offset to point outward.
            //Note: in unusual corner cases, the above may have accepted zero candidates resulting in a bestXNarrow = 1 and bestYNarrow = float.MinValue.
            //Catching that and ensuring that a reasonable face normal is output avoids a bad face.
            var projectedBestEdgeDirection = new Vector2(bestX, bestY);
            var length = projectedBestEdgeDirection.Length();
            //Note that the projected face normal is in terms of basisX and basisY, not the original basis facePoints are built on.
            projectedBestEdgeDirection = float.IsFinite(length) ? projectedBestEdgeDirection / length : new Vector2(1, 0);
            //Transform the projected normal back into the basis of facePoints.
            var edgeDirection = basisX * projectedBestEdgeDirection.X + basisY * projectedBestEdgeDirection.Y;
            var faceNormal = new Vector2(-edgeDirection.Y, edgeDirection.X);

            float distance = 0;
            int mostDistantIndex = -1;
            for (int i = 0; i < facePoints.Count; ++i)
            {
                var candidate = facePoints[i];
                var toCandidate = candidate - start;
                var alongNormal = Vector2.Dot(toCandidate, faceNormal);
                if (alongNormal > -planeEpsilon)
                {
                    var alongEdge = Vector2.Dot(toCandidate, edgeDirection);
                    if (alongEdge > distance)
                    {
                        distance = alongEdge;
                        mostDistantIndex = i;
                    }
                }
            }
            return mostDistantIndex == -1 ? bestIndex : mostDistantIndex;


        }

        static void ReduceFace(ref QuickList<int> faceVertexIndices, Vector3 faceNormal, Span<Vector3> points, float planeEpsilon, ref QuickList<Vector2> facePoints, ref Buffer<int> allowVertex, ref QuickList<int> reducedIndices)
        {
            Debug.Assert(facePoints.Count == 0 && reducedIndices.Count == 0 && facePoints.Span.Length >= faceVertexIndices.Count && reducedIndices.Span.Length >= faceVertexIndices.Count);
            for (int i = faceVertexIndices.Count - 1; i >= 0; --i)
            {
                //TODO: This isn't really necessary (conditioning on a small change).
                //Face merges may see this codepath because the original rawFaceVertexIndices may contain now-disallowed vertices.
                //We don't really need to *track* those, though; we could just use the reducedIndices and then this would never be required.
                //It's mostly a matter of legacy- previously, we accumulated everything without asking about whether it was allowed, and relied on ReduceFace to clean it up.
                //That opened a door for an infinite loop, so it got changed.
                if (allowVertex[faceVertexIndices[i]] == 0)
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
                            allowVertex[reducedIndices[2]] = 0;
                            reducedIndices.FastRemoveAt(2);
                        }
                        else if (ac.LengthSquared() > 1e-14f)
                        {
                            allowVertex[reducedIndices[1]] = 0;
                            reducedIndices.FastRemoveAt(1);
                        }
                        else
                        {
                            allowVertex[reducedIndices[1]] = 0;
                            allowVertex[reducedIndices[2]] = 0;
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
                    allowVertex[faceVertexIndices[i]] = 0;
                }
                return;
            }
            var greatestDistance = (float)Math.Sqrt(greatestDistanceSquared);
            var initialOffsetDirection = (facePoints[initialIndex] - centroid) / greatestDistance;
            var previousEdgeDirection = new Vector2(initialOffsetDirection.Y, -initialOffsetDirection.X);
            reducedIndices.AllocateUnsafely() = faceVertexIndices[initialIndex];

            var previousEndIndex = initialIndex;
            for (int i = 0; i < facePoints.Count; ++i)
            {
                var nextIndex = FindNextIndexForFaceHull(facePoints[previousEndIndex], previousEdgeDirection, planeEpsilon, ref facePoints);
                //This can return -1 in the event of a completely degenerate face.
                if (nextIndex == -1 || reducedIndices.Contains(faceVertexIndices[nextIndex]))
                {
                    if (nextIndex >= 0)
                    {
                        //Wrapped around to a repeated index.
                        //Note that hitting a repeated index is not necessarily because we found the initial index again; the initial index may have been numerically undiscoverable.
                        //In this case, we don't actually want our initial index to be in the reduced indices.
                        //In fact, we don't want *any* of the indices that aren't part of the identified face cycle, so look up the first index in the cycle and remove anything before that.
                        var cycleStartIndex = reducedIndices.IndexOf(faceVertexIndices[nextIndex]);
                        Debug.Assert(cycleStartIndex >= 0);
                        if (cycleStartIndex > 0)
                        {
                            //Note that order matters; can't do a last element swapping remove.
                            reducedIndices.Span.CopyTo(cycleStartIndex, reducedIndices.Span, 0, reducedIndices.Count - cycleStartIndex);
                            reducedIndices.Count -= cycleStartIndex;
                        }
                    }
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
                    allowVertex[index] = 0;
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

        internal struct EarlyFace
        {
            public QuickList<int> VertexIndices;
            public Vector3 Normal;
        }

        struct EdgeToTest
        {
            public EdgeEndpoints Endpoints;
            public Vector3 FaceNormal;
        }


        static void AddFace(ref QuickList<EarlyFace> faces, BufferPool pool, Vector3 normal, QuickList<int> vertexIndices)
        {
            ref var face = ref faces.Allocate(pool);
            face = new EarlyFace { Normal = normal, VertexIndices = new QuickList<int>(vertexIndices.Count, pool) };
            face.VertexIndices.AddRangeUnsafely(vertexIndices);
        }

        static void AddFaceEdgesToTestList(BufferPool pool,
            ref QuickList<int> reducedFaceIndices,
            ref QuickList<EdgeToTest> edgesToTest,
            ref QuickDictionary<EdgeEndpoints, int, EdgeEndpoints> edgeFaceCounts,
            Vector3 faceNormal, int newFaceIndex)
        {
            var previousIndex = reducedFaceIndices[reducedFaceIndices.Count - 1];
            for (int i = 0; i < reducedFaceIndices.Count; ++i)
            {
                EdgeEndpoints endpoints;
                endpoints.A = previousIndex;
                endpoints.B = reducedFaceIndices[i];
                previousIndex = endpoints.B;
                if (!edgeFaceCounts.FindOrAllocateSlot(ref endpoints, pool, out var slotIndex))
                {
                    EdgeToTest nextEdgeToTest;
                    nextEdgeToTest.Endpoints = endpoints;
                    nextEdgeToTest.FaceNormal = faceNormal;
                    edgesToTest.Allocate(pool) = nextEdgeToTest;
                    edgeFaceCounts.Values[slotIndex] = 1;
                }
                else
                {
                    //No need to test this edge; it's already been submitted by a different face.
                    edgeFaceCounts.Values[slotIndex]++;
                }
            }
        }

#if DEBUG_STEPS
        public struct DebugStep
        {
            public EdgeEndpoints SourceEdge;
            public int[] Raw;
            public int[] Reduced;
            public int[] OverwrittenOriginal;
            public List<int[]> DeletedFaces;
            public bool[] AllowVertex;
            public Vector3 FaceNormal;
            public Vector3 BasisX;
            public Vector3 BasisY;
            public List<int> FaceStarts;
            public List<int> FaceIndices;
            public int FaceIndex;
            public Vector3[] FaceNormals;


            internal DebugStep(EdgeEndpoints sourceEdge, QuickList<int> rawVertexIndices, Vector3 faceNormal, Vector3 basisX, Vector3 basisY, QuickList<int> reducedVertexIndices, int faceIndex)
            {
                SourceEdge = sourceEdge;
                FaceNormal = faceNormal;
                BasisX = basisX;
                BasisY = basisY;
                Raw = ((Span<int>)rawVertexIndices).ToArray();
                Reduced = ((Span<int>)reducedVertexIndices).ToArray();
                OverwrittenOriginal = null;
                FaceIndex = faceIndex;
                DeletedFaces = new List<int[]>();
            }

            internal DebugStep FillHistory(Buffer<int> allowVertex, QuickList<EarlyFace> faces)
            {
                FaceStarts = new List<int>(faces.Count);
                FaceIndices = new List<int>();
                FaceNormals = new Vector3[faces.Count];
                for (int i = 0; i < faces.Count; ++i)
                {
                    ref var face = ref faces[i];
                    FaceStarts.Add(FaceIndices.Count);
                    for (int j = 0; j < face.VertexIndices.Count; ++j)
                        FaceIndices.Add(face.VertexIndices[j]);
                    FaceNormals[i] = face.Normal;
                }
                AllowVertex = new bool[allowVertex.Length];
                for (int i = 0; i < allowVertex.Length; ++i)
                {
                    AllowVertex[i] = allowVertex[i] != 0;
                }
                return this;
            }

            /// <summary>
            /// Records the vertex indices corresponding to a face that was overwritten by a new face created by merging a newly-discovered face and an existing face due to normal similarity.
            /// </summary>
            /// <param name="faceVertexIndices">Face vertex indices of the original face that's being overwritten.</param>
            internal void RecordOverwrittenFace(QuickList<int> faceVertexIndices)
            {
                OverwrittenOriginal = ((Span<int>)faceVertexIndices).ToArray();
            }

            /// <summary>
            /// Records the vertex indices corresponding to a face that was deleted for being associated with now-disallowed vertices downstream of a face merge.
            /// </summary>
            /// <param name="faceVertexIndices">Vertices of the face that was deleted.</param>
            internal void RecordDeletedFace(QuickList<int> faceVertexIndices)
            {
                DeletedFaces.Add(((Span<int>)faceVertexIndices).ToArray());
            }

            internal void UpdateForFaceMerge(QuickList<int> rawFaceVertexIndices, QuickList<int> reducedVertexIndices, Buffer<int> allowVertex, int mergedFaceIndex)
            {
                Raw = ((Span<int>)rawFaceVertexIndices).ToArray();
                Reduced = ((Span<int>)reducedVertexIndices).ToArray();
                FaceIndex = mergedFaceIndex;
            }
        }
        /// <summary>
        /// Computes the convex hull of a set of points.
        /// </summary>
        /// <param name="points">Point set to compute the convex hull of.</param>
        /// <param name="pool">Buffer pool to pull memory from when creating the hull.</param>
        /// <param name="hullData">Convex hull of the input point set.</param>
        public static void ComputeHull(Span<Vector3> points, BufferPool pool, out HullData hullData)
        {
            ComputeHull(points, pool, out hullData, out _);
        }
#endif

        /// <summary>
        /// Computes the convex hull of a set of points.
        /// </summary>
        /// <param name="points">Point set to compute the convex hull of.</param>
        /// <param name="pool">Buffer pool to pull memory from when creating the hull.</param>
        /// <param name="hullData">Convex hull of the input point set.</param>
#if DEBUG_STEPS
        public static void ComputeHull(Span<Vector3> points, BufferPool pool, out HullData hullData, out List<DebugStep> steps)
#else
        public static void ComputeHull(Span<Vector3> points, BufferPool pool, out HullData hullData)
#endif
        {
#if DEBUG_STEPS
            steps = new List<DebugStep>();
#endif
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
            // Currently using two forms of epsilon for coplanar point testing:
            // 1. A 'slab' epsilon, specifying a constant width of the slab around the plane within which points are considered coplanar, and
            // 2. A face coplanarity epsilon, which increases the slab width based on the distance from the measurement point.
            // The face coplanarity epsilon captures points which could be member of faces that will be considered coplanar by the later face merging phase.
            // If we expect they're going to show up as coplanar later, there's not much reason to create separate faces for them now.
            // (This can simplify away microgeometry, but that's often actually desirable.)
            var planeSlabEpsilonNarrow = MathF.Sqrt(bestDistanceSquared) * 1e-4f;
            var normalCoplanarityEpsilon = 1f - 1e-6f;
            var planeSlabEpsilon = new Vector<float>(planeSlabEpsilonNarrow);
            var rawFaceVertexIndices = new QuickList<int>(pointBundles.Length * Vector<float>.Count, pool);
            var initialSourceEdge = new EdgeEndpoints { A = initialIndex, B = initialIndex };

            //Points found to not be on the face hull are ignored by future executions.
            //Note that it's stored in integers instead of bools; it can be directly loaded as a mask during vectorized operations.
            //0 means the vertex is disallowed, -1 means the vertex is allowed.
            pool.Take<int>(pointBundleCount * Vector<int>.Count, out var allowVertices);
            ((Span<int>)allowVertices).Slice(0, points.Length).Fill(-1);
            for (int i = points.Length; i < allowVertices.Length; ++i)
                allowVertices[i] = 0;

            FindExtremeFace(initialBasisX, initialBasisY, initialVertexBundle, initialSourceEdge, ref pointBundles, indexOffsetBundle, allowVertices, points.Length,
               ref projectedOnX, ref projectedOnY, planeSlabEpsilon, ref rawFaceVertexIndices, out var initialFaceNormal);
            Debug.Assert(rawFaceVertexIndices.Count >= 2);
            var facePoints = new QuickList<Vector2>(points.Length, pool);
            var reducedFaceIndices = new QuickList<int>(points.Length, pool);


            ReduceFace(ref rawFaceVertexIndices, initialFaceNormal, points, planeSlabEpsilonNarrow, ref facePoints, ref allowVertices, ref reducedFaceIndices);

            var faces = new QuickList<EarlyFace>(points.Length, pool);
            var edgesToTest = new QuickList<EdgeToTest>(points.Length, pool);
            var edgeFaceCounts = new QuickDictionary<EdgeEndpoints, int, EdgeEndpoints>(points.Length, pool);
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
                var edgeOffset = points[edgeToAdd.Endpoints.B] - points[edgeToAdd.Endpoints.A];
                var basisY = Vector3.Cross(edgeOffset, edgeToAdd.FaceNormal);
                var basisX = Vector3.Cross(edgeOffset, basisY);
                if (Vector3.Dot(basisX, edgeToAdd.FaceNormal) > 0)
                    Helpers.Swap(ref edgeToAdd.Endpoints.A, ref edgeToAdd.Endpoints.B);
            }
#if DEBUG_STEPS
            Vector3Wide.ReadFirst(initialBasisX, out var debugInitialBasisX);
            Vector3Wide.ReadFirst(initialBasisY, out var debugInitialBasisY);
            steps.Add(new DebugStep(initialSourceEdge, rawFaceVertexIndices, initialFaceNormal, debugInitialBasisX, debugInitialBasisY, reducedFaceIndices, reducedFaceIndices.Count >= 3 ? 0 : -1).FillHistory(allowVertices, faces));
#endif

            while (edgesToTest.Count > 0)
            {
                edgesToTest.Pop(out var edgeToTest);
                if (edgeFaceCounts.TryGetValue(ref edgeToTest.Endpoints, out var edgeFaceCount) && edgeFaceCount >= 2)
                {
                    //This edge is already part of two faces; no need to test it further.
                    continue;
                }

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
                FindExtremeFace(basisXBundle, basisYBundle, basisOrigin, edgeToTest.Endpoints, ref pointBundles, indexOffsetBundle, allowVertices, points.Length, ref projectedOnX, ref projectedOnY, planeSlabEpsilon, ref rawFaceVertexIndices, out var faceNormal);
                reducedFaceIndices.Count = 0;
                facePoints.Count = 0;
                ReduceFace(ref rawFaceVertexIndices, faceNormal, points, planeSlabEpsilonNarrow, ref facePoints, ref allowVertices, ref reducedFaceIndices);

                if (reducedFaceIndices.Count < 3)
                {
#if DEBUG_STEPS
                    steps.Add(new DebugStep(edgeToTest.Endpoints, rawFaceVertexIndices, faceNormal, basisX, basisY, reducedFaceIndices, -1).FillHistory(allowVertices, faces));
#endif
                    //Degenerate face found; don't bother creating work for it.
                    continue;
                }
                // Brute force scan all the faces to see if the new face is coplanar with any of them.
#if DEBUG_STEPS
                var step = new DebugStep(edgeToTest.Endpoints, rawFaceVertexIndices, faceNormal, basisX, basisY, reducedFaceIndices, faces.Count);
                Console.WriteLine($"step count: {steps.Count}");
#endif
                bool mergedFace = false;
                for (int i = 0; i < faces.Count; ++i)
                {
                    ref var face = ref faces[i];
                    if (Vector3.Dot(face.Normal, faceNormal) > normalCoplanarityEpsilon)
                    {
#if DEBUG_STEPS
                        Console.WriteLine($"Merging face {i} with new face, dot {Vector3.Dot(face.Normal, faceNormal)}:");
                        Console.WriteLine($"Existing face:  {face.Normal}");
                        Console.WriteLine($"Candidate:      {faceNormal}");
#endif
                        // The new face is coplanar with an existing face. Merge the new face into the old face.
                        rawFaceVertexIndices.EnsureCapacity(reducedFaceIndices.Count + face.VertexIndices.Count, pool);
                        rawFaceVertexIndices.Count = reducedFaceIndices.Count;
                        reducedFaceIndices.Span.CopyTo(0, rawFaceVertexIndices.Span, 0, reducedFaceIndices.Count);
                        for (int j = 0; j < face.VertexIndices.Count; ++j)
                        {
                            var vertexIndex = face.VertexIndices[j];
                            // Only testing the original set of reduced face indices for duplicates when merging; we know the face's point set isn't redundant.
                            if (allowVertices[vertexIndex] != 0 && !reducedFaceIndices.Contains(vertexIndex))
                            {
                                rawFaceVertexIndices.AllocateUnsafely() = vertexIndex;
                            }
                        }
                        // Rerun reduction for the merged face.
#if DEBUG_STEPS
                        step.RecordOverwrittenFace(face.VertexIndices);
#endif
                        face.VertexIndices.Count = 0;
                        facePoints.Count = 0;
                        face.VertexIndices.EnsureCapacity(rawFaceVertexIndices.Count, pool);
                        ReduceFace(ref rawFaceVertexIndices, faceNormal, points, planeSlabEpsilonNarrow, ref facePoints, ref allowVertices, ref face.VertexIndices);
#if DEBUG_STEPS
                        step.UpdateForFaceMerge(rawFaceVertexIndices, face.VertexIndices, allowVertices, i);
#endif
                        mergedFace = true;

                        // It's possible for the merged face to have invalidated a previous face that wouldn't necessarily be detected as something to merge.
                        break;
                    }
                }
                if (!mergedFace)
                {
                    var faceCountPriorToAdd = faces.Count;
                    AddFace(ref faces, pool, faceNormal, reducedFaceIndices);
                    AddFaceEdgesToTestList(pool, ref reducedFaceIndices, ref edgesToTest, ref edgeFaceCounts, faceNormal, faceCountPriorToAdd);
                }
                // Check all faces for use of disallowed vertices.
                var deletedFaceCount = 0;
                for (int i = 0; i < faces.Count; ++i)
                {
                    ref var face = ref faces[i];
                    bool deletedFace = false;
                    for (int j = 0; j < face.VertexIndices.Count; ++j)
                    {
                        if (allowVertices[face.VertexIndices[j]] == 0)
                        {
                            ++deletedFaceCount;
                            deletedFace = true;
                            break;
                        }
                    }
                    if (deletedFace)
                    {
#if DEBUG_STEPS
                        Console.WriteLine($"Deleting face {i}");
                        step.RecordDeletedFace(face.VertexIndices);
#endif
                        // Edges may have been exposed by the deletion of the face.
                        // Adjust the edge-face counts.
                        for (int j = 0; j < face.VertexIndices.Count; ++j)
                        {
                            var previousIndex = face.VertexIndices[j == 0 ? face.VertexIndices.Count - 1 : j - 1];
                            var nextIndex = face.VertexIndices[j];
                            // NOTE A CRITICAL SUBTLETY:
                            // The edge endpoints are flipped from the usual submission order.
                            // That's because the usual submission is trying to find faces *outside* the current face (since it just got added).
                            // Here, we're leaving a void and we want to fill it.
                            var endpoints = new EdgeEndpoints { A = nextIndex, B = previousIndex };
                            if (edgeFaceCounts.GetTableIndices(ref endpoints, out var tableIndex, out var elementIndex))
                            {
                                ref var countForEdge = ref edgeFaceCounts.Values[elementIndex];
                                if (allowVertices[endpoints.A] != 0 && allowVertices[endpoints.B] != 0)
                                {
                                    // This edge connects still-valid vertices, and by removing a face from it, it's conceivable that we've opened a hole.
                                    // Note that the face normal we're using here is not actually 'correct'; it should be the face normal of the *other* face on this edge.
                                    // We're shrugging about this because the deleted face should still be able to offer a normal that fills the hole...
                                    edgesToTest.Add(new EdgeToTest { Endpoints = endpoints, FaceNormal = face.Normal }, pool);
                                }
                            }
                        }

                        face.VertexIndices.Dispose(pool);
                    }
                    if (!deletedFace && deletedFaceCount > 0)
                    {
                        // Shift the face back to fill in the gap.
                        faces[i - deletedFaceCount] = faces[i];
                    }
                }
                faces.Count -= deletedFaceCount;
#if DEBUG_STEPS
                step.FillHistory(allowVertices, faces);
                steps.Add(step);
                if (steps.Count > 500)
                    break;
#endif
            }

            edgesToTest.Dispose(pool);
            facePoints.Dispose(pool);
            reducedFaceIndices.Dispose(pool);
            rawFaceVertexIndices.Dispose(pool);
            pool.Return(ref allowVertices);
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
        }


        /// <summary>
        /// Processes hull data into a runtime usable convex hull shape. Recenters the convex hull's points around its center of mass.
        /// </summary>
        /// <param name="points">Point array into which the hull data indexes.</param>
        /// <param name="hullData">Raw input data to process.</param>
        /// <param name="pool">Pool used to allocate resources for the hullShape.</param>
        /// <param name="hullShape">Convex hull shape created from the input data.</param>
        /// <param name="center">Computed center of mass of the convex hull before its points were recentered onto the origin.</param>
        /// <returns>True if the shape was created successfully, false otherwise. If false, the hull probably had no volume and would not have worked properly as a shape.</returns>
        public static bool CreateShape(Span<Vector3> points, HullData hullData, BufferPool pool, out Vector3 center, out ConvexHull hullShape)
        {
            Debug.Assert(points.Length > 0, "Convex hulls need to have a nonzero number of points!");
            hullShape = default;
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
                    var centroid = a + b + c;
                    center += centroid * volumeContribution;
                }
            }
            //Division by 4 since we accumulated (a + b + c), rather than the actual tetrahedral center (a + b + c + 0) / 4.
            center /= volume * 4;
            if (float.IsNaN(center.X) || float.IsNaN(center.Y) || float.IsNaN(center.Z) || hullData.FaceStartIndices.Length == 2)
            {
                //The convex hull seems to have no volume.
                //While you could try treating it as coplanar (like we once tried; see commit history just prior to the commit that added this message):
                //1. Ray tests won't work. They rely on bounding planes. It would require a special case for degenerate hulls.
                //2. Inertia won't work. You could resolve that with a special case, but it doesn't fix ray tests.
                //3. Edge-on contact generation may produce lower quality contacts.
                //So, pretty worthless overall without major changes.
                hullShape.Points.Dispose(pool);
                center = default;
                Debug.Assert(!hullShape.Points.Allocated && !hullShape.FaceToVertexIndicesStart.Allocated && !hullShape.BoundingPlanes.Allocated && !hullShape.FaceVertexIndices.Allocated, "Hey! You moved something around and forgot to dispose!");
                return false;
            }

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
            return true;
        }

        /// <summary>
        /// Creates a convex hull shape out of an input point set. Recenters the convex hull's points around its center of mass.
        /// </summary>
        /// <param name="points">Points to use to create the hull.</param>
        /// <param name="pool">Buffer pool used for temporary allocations and the output data structures.</param>
        /// <param name="hullData">Intermediate hull data that got processed into the convex hull.</param>
        /// <param name="center">Computed center of mass of the convex hull before its points were recentered onto the origin.</param>
        /// <param name="convexHull">Convex hull shape of the input point set.</param>
        /// <returns>True if the shape was created successfully, false otherwise. If false, the hull probably had no volume and would not have worked properly as a shape.</returns>
        public static bool CreateShape(Span<Vector3> points, BufferPool pool, out HullData hullData, out Vector3 center, out ConvexHull convexHull)
        {
            ComputeHull(points, pool, out hullData);
            return CreateShape(points, hullData, pool, out center, out convexHull);
        }

        /// <summary>
        /// Creates a convex hull shape out of an input point set. Recenters the convex hull's points around its center of mass.
        /// </summary>
        /// <param name="points">Points to use to create the hull.</param>
        /// <param name="pool">Buffer pool used for temporary allocations and the output data structures.</param>
        /// <param name="center">Computed center of mass of the convex hull before its points were recentered onto the origin.</param>
        /// <param name="convexHull">Convex hull shape of the input point set.</param>
        /// <returns>True if the shape was created successfully, false otherwise. If false, the hull probably had no volume and would not have worked properly as a shape.</returns>
        public static bool CreateShape(Span<Vector3> points, BufferPool pool, out Vector3 center, out ConvexHull convexHull)
        {
            ComputeHull(points, pool, out var hullData);
            var result = CreateShape(points, hullData, pool, out center, out convexHull);
            //Empty input point sets won't allocate; don't try to dispose them.
            if (hullData.OriginalVertexMapping.Allocated)
                hullData.Dispose(pool);
            return result;
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
