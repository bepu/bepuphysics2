using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
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
            //angle = acos(y / ||(x, y)||)
            //cosAngle = y / ||(x, y)||
            //cosAngle^2 = y^2 / ||(x, y)||^2
            //We can then compare samples 0 and 1 using:
            //sign(y0) * y0^2 * ||(x1,y1)||^2 > sign(y1) * y1^2 * ||(x0,y0)||^2
            //with no divisions, square roots, or trigonometry.
            Vector3Wide.Subtract(pointBundles[0], basisOrigin, out var toCandidate);
            ref var x = ref projectedOnX[0];
            ref var y = ref projectedOnY[0];
            Vector3Wide.Dot(basisX, toCandidate, out x);
            //If x is negative, that means some numerical issue has resulted in a point beyond the bounding plane that generated this face request.
            //We'll treat it as if it's on the plane.
            x = Vector.Max(Vector<float>.Zero, x);
            Vector3Wide.Dot(basisY, toCandidate, out y);
            var bestNumerators = y * y;
            var bestDenominators = bestNumerators + x * x;
            bestNumerators = Vector.ConditionalSelect(Vector.LessThan(y, Vector<float>.Zero), -bestNumerators, bestNumerators);
            //Ignore the source edge.
            var edgeIndexA = new Vector<int>(sourceEdgeEndpoints.A);
            var edgeIndexB = new Vector<int>(sourceEdgeEndpoints.B);
            var pointCountBundle = new Vector<int>(pointCount);
            const float minimumDistanceSquaredScalar = 1e-14f;
            var minimumDistanceSquared = new Vector<float>(minimumDistanceSquaredScalar);
            var ignoreSlot = Vector.BitwiseOr(
                Vector.BitwiseOr(Vector.GreaterThanOrEqual(indexOffsets, pointCountBundle), Vector.LessThan(bestDenominators, minimumDistanceSquared)),
                Vector.BitwiseOr(Vector.Equals(indexOffsets, edgeIndexA), Vector.Equals(indexOffsets, edgeIndexB)));
            bestDenominators = Vector.ConditionalSelect(ignoreSlot, Vector<float>.One, bestDenominators);
            bestNumerators = Vector.ConditionalSelect(ignoreSlot, new Vector<float>(float.MinValue), bestNumerators);
            var bestIndices = indexOffsets;
            for (int i = 1; i < pointBundles.Length; ++i)
            {
                Vector3Wide.Subtract(pointBundles[i], basisOrigin, out toCandidate);
                x = ref projectedOnX[i];
                y = ref projectedOnY[i];
                Vector3Wide.Dot(basisX, toCandidate, out x);
                x = Vector.Max(Vector<float>.Zero, x); //Same as earlier- protect against numerical error finding points beyond the bounding plane.
                Vector3Wide.Dot(basisY, toCandidate, out y);
                var candidateNumerator = y * y;
                var candidateDenominator = candidateNumerator + x * x;
                candidateNumerator = Vector.ConditionalSelect(Vector.LessThan(y, Vector<float>.Zero), -candidateNumerator, candidateNumerator);

                var candidateIndices = indexOffsets + new Vector<int>(i << BundleIndexing.VectorShift);
                ignoreSlot = Vector.BitwiseOr(
                    Vector.BitwiseOr(Vector.GreaterThanOrEqual(candidateIndices, pointCountBundle), Vector.LessThan(candidateDenominator, minimumDistanceSquared)),
                    Vector.BitwiseOr(Vector.Equals(candidateIndices, edgeIndexA), Vector.Equals(candidateIndices, edgeIndexB)));
                var useCandidate = Vector.AndNot(Vector.GreaterThan(candidateNumerator * bestDenominators, bestNumerators * candidateDenominator), ignoreSlot);
                bestIndices = Vector.ConditionalSelect(useCandidate, candidateIndices, bestIndices);
                bestNumerators = Vector.ConditionalSelect(useCandidate, candidateNumerator, bestNumerators);
                bestDenominators = Vector.ConditionalSelect(useCandidate, candidateDenominator, bestDenominators);
            }
            var bestNumerator = bestNumerators[0];
            var bestDenominator = bestDenominators[0];
            var bestIndex = bestIndices[0];
            for (int i = 1; i < Vector<float>.Count; ++i)
            {
                var candidateNumerator = bestNumerators[i];
                var candidateDenominator = bestDenominators[i];
                if (candidateNumerator * bestDenominator > bestNumerator * candidateDenominator)
                {
                    bestNumerator = candidateNumerator;
                    bestDenominator = candidateDenominator;
                    bestIndex = bestIndices[i];
                }
            }
            //We now have the best index, but there may have been multiple vertices on the same plane. Capture all of them at once by doing a second pass over the results.
            //The plane normal we want to examine is (-bestY, bestX) / ||(-bestY, bestX)||.
            //(This isn't wonderfully fast, but it's fairly simple. The alternatives are things like incrementally combining coplanar triangles as they are discovered
            //or using a postpass that looks for coplanar triangles after they've been created.)
            BundleIndexing.GetBundleIndices(bestIndex, out var bestBundleIndex, out int bestInnerIndex);
            var bestX = projectedOnX[bestBundleIndex][bestInnerIndex];
            var bestY = projectedOnY[bestBundleIndex][bestInnerIndex];
            //Rotate the offset to point outward.
            var projectedPlaneNormalNarrow = Vector2.Normalize(new Vector2(-bestY, bestX));
            Vector2Wide.Broadcast(projectedPlaneNormalNarrow, out var projectedPlaneNormal);
            for (int i = 0; i < pointBundles.Length; ++i)
            {
                var dot = projectedOnX[i] * projectedPlaneNormal.X + projectedOnY[i] * projectedPlaneNormal.Y;
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
            Vector3Wide.ReadFirst(basisX, out var basisXNarrow);
            Vector3Wide.ReadFirst(basisY, out var basisYNarrow);
            faceNormal = basisXNarrow * projectedPlaneNormalNarrow.X + basisYNarrow * projectedPlaneNormalNarrow.Y;
        }


        static int FindNextIndexForFaceHull(in Vector2 start, in Vector2 previousEdgeDirection, ref QuickList<Vector2> facePoints)
        {
            //Use a AOS version since the number of points on a given face will tend to be very small in most cases.
            //Same idea as the 3d version- find the next edge which is closest to the previous edge. Not going to worry about collinear points here for now.
            var bestIndex = -1;
            float bestNumerator = -float.MaxValue;
            float bestDenominator = 1;
            var startToCandidate = facePoints[0] - start;
            var candidateDenominator = startToCandidate.LengthSquared();
            var dot = Vector2.Dot(startToCandidate, previousEdgeDirection);
            var candidateNumerator = dot * dot;
            candidateNumerator = dot < 0 ? -candidateNumerator : candidateNumerator;
            if (candidateDenominator > 0)
            {
                bestNumerator = candidateNumerator;
                bestDenominator = candidateDenominator;
                bestIndex = 0;
            }
            for (int i = 0; i < facePoints.Count; ++i)
            {
                startToCandidate = facePoints[i] - start;
                dot = Vector2.Dot(startToCandidate, previousEdgeDirection);
                var absCandidateNumerator = dot * dot;
                candidateNumerator = dot < 0 ? -absCandidateNumerator : absCandidateNumerator;
                candidateDenominator = startToCandidate.LengthSquared();
                //Watch out for collinear points. If the angle is the same, then pick the more distant point.
                var candidate = candidateNumerator * bestDenominator;
                var currentBest = bestNumerator * candidateDenominator;
                var epsilon = 1e-7f * absCandidateNumerator * bestDenominator;
                if (candidate > currentBest - epsilon)
                {
                    //Candidate and current best angle may be extremely similar.
                    //Only use the candidate if it's further away.
                    if (candidate < currentBest + epsilon && candidateDenominator <= bestDenominator)
                    {
                        continue;
                    }
                    bestNumerator = candidateNumerator;
                    bestDenominator = candidateDenominator;
                    bestIndex = i;
                }
            }
            //Note that this can return -1 if all points were on top of the start.
            return bestIndex;
        }

        static void ReduceFace(ref QuickList<int> faceVertexIndices, in Vector3 faceNormal, ref Buffer<Vector3> points, ref QuickList<Vector2> facePoints, ref Buffer<bool> allowVertex, ref QuickList<int> reducedIndices)
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
            Helpers.BuildOrthnormalBasis(faceNormal, out var basisX, out var basisY);
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
                var nextIndex = FindNextIndexForFaceHull(facePoints[previousEndIndex], previousEdgeDirection, ref facePoints);
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
        struct EdgeToTest
        {
            public EdgeEndpoints Endpoints;
            public Vector3 FaceNormal;
            public int FaceIndex;
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

        //    public DebugStep(EdgeEndpoints sourceEdge, ref QuickList<int> raw, in Vector3 faceNormal, in Vector3 basisX, in Vector3 basisY)
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
        //        Reduced = default;
        //        AllowVertex = default;
        //    }

        //    public void AddReduced(ref QuickList<int> reduced, ref Buffer<bool> allowVertex)
        //    {
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
        //    }


        //}

        /// <summary>
        /// Computes the convex hull of a set of points.
        /// </summary>
        /// <param name="points">Point set to compute the convex hull of.</param>
        /// <param name="pool">Buffer pool to pull memory from when creating the hull.</param>
        /// <param name="hullData">Convex hull of the input point set.</param>
        public static void ComputeHull(Buffer<Vector3> points, BufferPool pool, out HullData hullData)
        {
            if (points.Length <= 0)
            {
                hullData = default;
                //steps = new List<DebugStep>();
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
                //steps = new List<DebugStep>();
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
                //steps = new List<DebugStep>();
                pool.Return(ref pointBundles);
                return;
            }
            Vector3Wide.Broadcast(initialToCentroid / initialDistance, out var initialBasisX);
            Helpers.FindPerpendicular(initialBasisX, out var initialBasisY); //(broadcasted before FindPerpendicular just because we didn't have a non-bundle version)
            Vector3Wide.Broadcast(initialVertex, out var initialVertexBundle);
            pool.Take<Vector<float>>(pointBundles.Length, out var projectedOnX);
            pool.Take<Vector<float>>(pointBundles.Length, out var projectedOnY);
            var planeEpsilon = new Vector<float>((float)Math.Sqrt(bestDistanceSquared) * 1e-6f);
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

            Vector3Wide.ReadFirst(initialBasisX, out var debugInitialBasisX);
            Vector3Wide.ReadFirst(initialBasisY, out var debugInitialBasisY);
            //steps = new List<DebugStep>();
            //var step = new DebugStep(initialSourceEdge, ref rawFaceVertexIndices, initialFaceNormal, debugInitialBasisX, debugInitialBasisY);

            ReduceFace(ref rawFaceVertexIndices, initialFaceNormal, ref points, ref facePoints, ref allowVertex, ref reducedFaceIndices);
            //step.AddReduced(ref reducedFaceIndices, ref allowVertex);
            //steps.Add(step);

            var earlyFaceIndices = new QuickList<int>(points.Length, pool);
            var earlyFaceStartIndices = new QuickList<int>(points.Length, pool);

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
                    edgeToAdd.FaceIndex = 0;
                    edgeFaceCounts.AddRef(ref edgeToAdd.Endpoints, 1, pool);
                }
                //Since an actual face was found, we go ahead and output it into the face set.
                earlyFaceStartIndices.Allocate(pool) = earlyFaceIndices.Count;
                earlyFaceIndices.AddRange(ref reducedFaceIndices.Span, 0, reducedFaceIndices.Count, pool);
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

            while (edgesToTest.Count > 0)
            {
                edgesToTest.Pop(out var edgeToTest);
                //Make sure the new edge hasn't already been filled by another traversal.
                var faceCountIndex = edgeFaceCounts.IndexOf(edgeToTest.Endpoints);
                if (faceCountIndex >= 0 && edgeFaceCounts.Values[faceCountIndex] >= 2)
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
                //step = new DebugStep(edgeToTest.Endpoints, ref rawFaceVertexIndices, faceNormal, basisX, basisY);
                reducedFaceIndices.Count = 0;
                facePoints.Count = 0;
                ReduceFace(ref rawFaceVertexIndices, faceNormal, ref points, ref facePoints, ref allowVertex, ref reducedFaceIndices);
                if(reducedFaceIndices.Count < 3)
                {
                    //Degenerate face found; don't bother creating work for it.
                    continue;
                }
                //step.AddReduced(ref reducedFaceIndices, ref allowVertex);
                //steps.Add(step);

                var newFaceIndex = earlyFaceStartIndices.Count;
                earlyFaceStartIndices.Allocate(pool) = earlyFaceIndices.Count;
                earlyFaceIndices.AddRange(ref reducedFaceIndices.Span, 0, reducedFaceIndices.Count, pool);

                edgeFaceCounts.EnsureCapacity(edgeFaceCounts.Count + reducedFaceIndices.Count, pool);
                for (int i = 0; i < reducedFaceIndices.Count; ++i)
                {
                    EdgeToTest nextEdgeToTest;
                    nextEdgeToTest.Endpoints.A = reducedFaceIndices[i == 0 ? reducedFaceIndices.Count - 1 : i - 1];
                    nextEdgeToTest.Endpoints.B = reducedFaceIndices[i];
                    nextEdgeToTest.FaceNormal = faceNormal;
                    nextEdgeToTest.FaceIndex = newFaceIndex;
                    if (edgeFaceCounts.GetTableIndices(ref nextEdgeToTest.Endpoints, out var tableIndex, out var elementIndex))
                    {
                        ref var edgeFaceCount = ref edgeFaceCounts.Values[elementIndex];
                        //Debug.Assert(edgeFaceCount == 1, 
                        //    "While we let execution continue, this is an error condition and implies overlapping triangles are being generated." + 
                        //    "This tends to happen when there are many near-coplanar vertices, so numerical tolerances across different faces cannot consistently agree.");
                        ++edgeFaceCount;
                    }
                    else
                    {
                        //This edge is not yet claimed by any edge. Claim it for the new face and add the edge for further testing.
                        edgeFaceCounts.Keys[edgeFaceCounts.Count] = nextEdgeToTest.Endpoints;
                        edgeFaceCounts.Values[edgeFaceCounts.Count] = 1;
                        //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
                        edgeFaceCounts.Table[tableIndex] = ++edgeFaceCounts.Count;
                        edgesToTest.Allocate(pool) = nextEdgeToTest;
                    }
                }
            }

            edgesToTest.Dispose(pool);
            edgeFaceCounts.Dispose(pool);
            facePoints.Dispose(pool);
            reducedFaceIndices.Dispose(pool);
            rawFaceVertexIndices.Dispose(pool);
            pool.Return(ref allowVertex);
            pool.Return(ref projectedOnX);
            pool.Return(ref projectedOnY);
            pool.Return(ref pointBundles);

            //Create a reduced hull point set from the face vertex references.
            pool.Take(earlyFaceStartIndices.Count, out hullData.FaceStartIndices);
            pool.Take(earlyFaceIndices.Count, out hullData.FaceVertexIndices);
            earlyFaceStartIndices.Span.CopyTo(0, ref hullData.FaceStartIndices, 0, earlyFaceStartIndices.Count);
            pool.Take<int>(points.Length, out var originalToHullIndexMapping);
            var hullToOriginalIndexMapping = new QuickList<int>(points.Length, pool);
            for (int i = 0; i < points.Length; ++i)
            {
                originalToHullIndexMapping[i] = -1;
            }
            for (int i = 0; i < earlyFaceStartIndices.Count; ++i)
            {
                var start = earlyFaceStartIndices[i];
                var nextIndex = i + 1;
                var end = earlyFaceStartIndices.Count == nextIndex ? earlyFaceIndices.Count : earlyFaceStartIndices[nextIndex];
                for (int j = start; j < end; ++j)
                {
                    var originalVertexIndex = earlyFaceIndices[j];
                    ref var originalToHull = ref originalToHullIndexMapping[originalVertexIndex];
                    if (originalToHull < 0)
                    {
                        //This vertex hasn't been seen yet.
                        originalToHull = hullToOriginalIndexMapping.Count;
                        hullToOriginalIndexMapping.AllocateUnsafely() = originalVertexIndex;
                    }
                    hullData.FaceVertexIndices[j] = originalToHull;
                }
            }

            pool.Take(hullToOriginalIndexMapping.Count, out hullData.OriginalVertexMapping);
            hullToOriginalIndexMapping.Span.CopyTo(0, ref hullData.OriginalVertexMapping, 0, hullToOriginalIndexMapping.Count);

            pool.Return(ref originalToHullIndexMapping);
            hullToOriginalIndexMapping.Dispose(pool);
            earlyFaceIndices.Dispose(pool);
            earlyFaceStartIndices.Dispose(pool);
        }

        public struct RawHullTriangleSource : ITriangleSource
        {
            Buffer<Vector3> points;
            HullData hullData;
            int faceIndex;
            int subtriangleIndex;

            public RawHullTriangleSource(in Buffer<Vector3> points, in HullData hullData)
            {
                this.points = points;
                this.hullData = hullData;
                faceIndex = 0;
                subtriangleIndex = 2;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetNextTriangle(out Vector3 a, out Vector3 b, out Vector3 c)
            {
                //This isn't quite as direct or fast as it could be, but it's fairly simple without requiring a redundant implementation.
                if (faceIndex < hullData.FaceStartIndices.Length)
                {
                    hullData.GetFace(faceIndex, out var face);
                    a = points[face[0]];
                    b = points[face[subtriangleIndex - 1]];
                    c = points[face[subtriangleIndex]];
                    ++subtriangleIndex;
                    if (subtriangleIndex == face.VertexIndices.Length)
                    {
                        subtriangleIndex = 2;
                        ++faceIndex;
                    }
                    return true;
                }
                a = default;
                b = default;
                c = default;
                return false;
            }
        }

        /// <summary>
        /// Processes hull data into a runtime usable convex hull shape. Recenters the convex hull's points around its center of mass.
        /// </summary>
        /// <param name="points">Point array into which the hull data indexes.</param>
        /// <param name="hullData">Raw input data to process.</param>
        /// <param name="pool">Pool used to allocate resources for the hullShape.</param>
        /// <param name="hullShape">Convex hull shape created from the input data.</param>
        /// <param name="center">Computed center of mass of the convex hull before its points were recentered onto the origin.</param>
        public static void CreateShape(Buffer<Vector3> points, HullData hullData, BufferPool pool, out Vector3 center, out ConvexHull hullShape)
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

            var triangleSource = new RawHullTriangleSource(points, hullData);
            MeshInertiaHelper.ComputeClosedCenterOfMass(ref triangleSource, out _, out center);

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
            hullData.FaceStartIndices.CopyTo(0, ref hullShape.FaceToVertexIndicesStart, 0, hullShape.FaceToVertexIndicesStart.Length);
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
        public static void CreateShape(Buffer<Vector3> points, BufferPool pool, out HullData hullData, out Vector3 center, out ConvexHull convexHull)
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
        public static void CreateShape(Buffer<Vector3> points, BufferPool pool, out Vector3 center, out ConvexHull convexHull)
        {
            ComputeHull(points, pool, out var hullData);
            CreateShape(points, hullData, pool, out center, out convexHull);
            hullData.Dispose(pool);
        }
    }
}
