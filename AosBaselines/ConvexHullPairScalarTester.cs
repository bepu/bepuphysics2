using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace AosBaselines;

using HullDepthRefiner = ScalarDepthRefiner<ConvexHull, HullSupportScalar, ConvexHull, HullSupportScalar>;

/// <summary>
/// Scalar AoS port of ConvexHullPairTester, bitwise identical per lane.
/// The wide tester's contact generation is already scalar per slot (vectorized within hulls), so that part is copied nearly verbatim;
/// the ported pieces are the wide setup math, the DepthRefiner (see ScalarDepthRefiner), and face picking.
/// </summary>
public static class ConvexHullPairScalarTester
{
    struct CachedEdge
    {
        public Vector3 Vertex;
        public Vector3 EdgePlaneNormal;
        public float MaximumContainmentDot;
    }

    /// <summary>
    /// Mirrors ConvexHullTestHelper.PickRepresentativeFace with scalar per-pair inputs. The within-hull wide loops are identical.
    /// Internal so the other hull pair scalar testers can share it; the body is frozen.
    /// </summary>
    internal static void PickRepresentativeFace(ref ConvexHull hull, in Vector3 slotLocalNormal, in Vector3 closestOnHull, float slotBoundingPlaneEpsilon,
        out Vector3 slotFaceNormal, out int bestFaceIndex)
    {
        Helpers.FillVectorWithLaneIndices(out var slotOffsetIndices);
        Vector3Wide.Broadcast(slotLocalNormal, out var slotLocalNormalBundle);
        ref var boundingPlaneBundle = ref hull.BoundingPlanes[0];
        var slotBoundingPlaneEpsilonBundle = new Vector<float>(slotBoundingPlaneEpsilon);
        var negatedSlotBoundingPlaneEpsilonBundle = -slotBoundingPlaneEpsilonBundle;
        Vector3Wide.Broadcast(closestOnHull, out var slotClosestOnHull);
        Vector3Wide.Dot(boundingPlaneBundle.Normal, slotLocalNormalBundle, out var bestFaceDotBundle);
        Vector3Wide.Dot(boundingPlaneBundle.Normal, slotClosestOnHull, out var closestOnHullDot);
        var bestPlaneErrorBundle = Vector.Abs(closestOnHullDot - boundingPlaneBundle.Offset);
        var bestIndices = slotOffsetIndices;
        for (int i = 1; i < hull.BoundingPlanes.Length; ++i)
        {
            var slotIndices = new Vector<int>(i << BundleIndexing.VectorShift) + slotOffsetIndices;
            boundingPlaneBundle = ref hull.BoundingPlanes[i];
            Vector3Wide.Dot(boundingPlaneBundle.Normal, slotLocalNormalBundle, out var dot);
            Vector3Wide.Dot(boundingPlaneBundle.Normal, slotClosestOnHull, out closestOnHullDot);
            var candidateError = Vector.Abs(closestOnHullDot - boundingPlaneBundle.Offset);
            var errorImprovement = bestPlaneErrorBundle - candidateError;
            var useCandidate = Vector.BitwiseOr(
                Vector.GreaterThanOrEqual(errorImprovement, slotBoundingPlaneEpsilonBundle),
                Vector.BitwiseAnd(
                    Vector.GreaterThan(errorImprovement, negatedSlotBoundingPlaneEpsilonBundle),
                    Vector.GreaterThan(dot, bestFaceDotBundle)));
            bestFaceDotBundle = Vector.ConditionalSelect(useCandidate, dot, bestFaceDotBundle);
            bestPlaneErrorBundle = Vector.ConditionalSelect(useCandidate, candidateError, bestPlaneErrorBundle);
            bestIndices = Vector.ConditionalSelect(useCandidate, slotIndices, bestIndices);
        }
        var bestFaceDot = bestFaceDotBundle[0];
        var bestPlaneError = bestPlaneErrorBundle[0];
        bestFaceIndex = bestIndices[0];
        var negatedSlotBoundingPlaneEpsilon = -slotBoundingPlaneEpsilon;
        for (int i = 1; i < Vector<float>.Count; ++i)
        {
            var dot = bestFaceDotBundle[i];
            var error = bestPlaneErrorBundle[i];
            var improvement = bestPlaneError - error;
            if (improvement >= slotBoundingPlaneEpsilon || (improvement >= negatedSlotBoundingPlaneEpsilon && dot > bestFaceDot))
            {
                bestFaceDot = dot;
                bestPlaneError = error;
                bestFaceIndex = bestIndices[i];
            }
        }
        BundleIndexing.GetBundleIndices(bestFaceIndex, out var faceBundleIndex, out var faceInnerIndex);
        Vector3Wide.ReadSlot(ref hull.BoundingPlanes[faceBundleIndex].Normal, faceInnerIndex, out slotFaceNormal);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void PlaceCandidateInSlot(in ManifoldCandidateScalar candidate, int contactIndex,
        Vector3 faceCenterB, Vector3 faceBX, Vector3 faceBY, float depth,
        in Matrix3x3 orientationB, Vector3 offsetB, ref Convex4ManifoldScalar manifold)
    {
        var localPosition = candidate.X * faceBX + candidate.Y * faceBY + faceCenterB;
        Matrix3x3.Transform(localPosition, orientationB, out var position);
        position += offsetB;
        Unsafe.Add(ref manifold.OffsetA0, contactIndex) = position;
        Unsafe.Add(ref manifold.Depth0, contactIndex) = depth;
        Unsafe.Add(ref manifold.FeatureId0, contactIndex) = candidate.FeatureId;
        Unsafe.Add(ref manifold.Contact0Exists, contactIndex) = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static unsafe void RemoveCandidateAt(ManifoldCandidateScalar* candidates, float* depths, int removalIndex, ref int candidateCount)
    {
        var lastIndex = candidateCount - 1;
        if (removalIndex < lastIndex)
        {
            candidates[removalIndex] = candidates[lastIndex];
            depths[removalIndex] = depths[lastIndex];
        }
        --candidateCount;
    }

    /// <summary>
    /// Copy of the library's scalar ManifoldCandidateHelper.Reduce, retargeted to write a scalar manifold instead of a wide manifold slot.
    /// All arithmetic (including Vector3.Dot intrinsics) matches the library version exactly, since the wide tester calls that same scalar code.
    /// Internal so the other hull pair scalar testers can share it; the body is frozen.
    /// </summary>
    internal static unsafe void Reduce(ManifoldCandidateScalar* candidates, int candidateCount,
        Vector3 faceNormalA, float inverseFaceNormalADotLocalNormal, Vector3 faceCenterA, Vector3 faceCenterB, Vector3 tangentBX, Vector3 tangentBY,
        float epsilonScale, float minimumDepth, in Matrix3x3 rotationToWorld, Vector3 worldOffsetB, ref Convex4ManifoldScalar manifold)
    {
        if (candidateCount == 0)
        {
            return;
        }
        //Note that this does NOT assign the world normal in the manifold.

        //Calculate the depths of all candidates, and prune those below the depth threshold.
        var dotAxis = faceNormalA * inverseFaceNormalADotLocalNormal;
        var faceCenterAToFaceCenterB = faceCenterB - faceCenterA;
        var baseDot = Vector3.Dot(faceCenterAToFaceCenterB, dotAxis);
        var xDot = Vector3.Dot(tangentBX, dotAxis);
        var yDot = Vector3.Dot(tangentBY, dotAxis);
        var candidateDepths = stackalloc float[candidateCount];
        for (int i = candidateCount - 1; i >= 0; --i)
        {
            ref var candidate = ref candidates[i];
            ref var candidateDepth = ref candidateDepths[i];
            candidateDepth = baseDot + candidate.X * xDot + candidate.Y * yDot;
            if (candidateDepth < minimumDepth)
            {
                RemoveCandidateAt(candidates, candidateDepths, i, ref candidateCount);
            }
        }
        if (candidateCount <= 4)
        {
            //No reduction is necessary; just place the contacts into the manifold.
            for (int i = 0; i < candidateCount; ++i)
            {
                PlaceCandidateInSlot(candidates[i], i, faceCenterB, tangentBX, tangentBY, candidateDepths[i], rotationToWorld, worldOffsetB, ref manifold);
            }
            return;
        }

        var bestScore0 = float.MinValue;
        var bestIndex0 = 0;
        //While depth is the dominant heuristic, extremity is used as a bias to keep initial contact selection a little more consistent in near-equal cases.
        var extremityScale = epsilonScale * 1e-2f;
        var extremityX = 0.7946897654f * extremityScale;
        var extremityY = 0.60701579614f * extremityScale;
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref candidates[i];
            ref var candidateDepth = ref candidateDepths[i];
            float candidateScore = candidateDepth;
            if (candidateDepth >= 0)
            {
                var extremity = candidate.X * extremityX + candidate.Y * extremityY;
                if (extremity < 0)
                    extremity = -extremity;
                candidateScore += extremity;
            }
            if (candidateScore > bestScore0)
            {
                bestScore0 = candidateScore;
                bestIndex0 = i;
            }
        }
        var candidate0 = candidates[bestIndex0];
        var depth0 = candidateDepths[bestIndex0];
        PlaceCandidateInSlot(candidate0, 0, faceCenterB, tangentBX, tangentBY, depth0, rotationToWorld, worldOffsetB, ref manifold);
        RemoveCandidateAt(candidates, candidateDepths, bestIndex0, ref candidateCount);

        //Find the most distant point from the starting contact.
        var maximumDistanceSquared = -1f;
        var bestIndex1 = 0;
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref candidates[i];
            var offsetX = candidate.X - candidate0.X;
            var offsetY = candidate.Y - candidate0.Y;
            var distanceSquared = offsetX * offsetX + offsetY * offsetY;
            if (distanceSquared > maximumDistanceSquared)
            {
                maximumDistanceSquared = distanceSquared;
                bestIndex1 = i;
            }
        }
        if (maximumDistanceSquared < 1e-6f * epsilonScale * epsilonScale)
        {
            //There's no point in additional contacts if the distance between the first and second candidates is zero.
            return;
        }
        var candidate1 = candidates[bestIndex1];
        var depth1 = candidateDepths[bestIndex1];
        PlaceCandidateInSlot(candidate1, 1, faceCenterB, tangentBX, tangentBY, depth1, rotationToWorld, worldOffsetB, ref manifold);
        RemoveCandidateAt(candidates, candidateDepths, bestIndex1, ref candidateCount);

        //Now identify two more points, maximizing signed triangle area magnitudes relative to the edge formed by the first two contacts.
        var edgeOffsetX = candidate1.X - candidate0.X;
        var edgeOffsetY = candidate1.Y - candidate0.Y;
        var minSignedArea = 0f;
        var maxSignedArea = 0f;
        var bestIndex2 = 0;
        var bestIndex3 = 0;
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref candidates[i];
            var candidateOffsetX = candidate.X - candidate0.X;
            var candidateOffsetY = candidate.Y - candidate0.Y;
            var signedArea = candidateOffsetX * edgeOffsetY - candidateOffsetY * edgeOffsetX;
            //Penalize speculative contacts; they are not as important in general.
            if (candidateDepths[i] < 0)
                signedArea *= 0.25f;

            if (signedArea < minSignedArea)
            {
                minSignedArea = signedArea;
                bestIndex2 = i;
            }
            if (signedArea > maxSignedArea)
            {
                maxSignedArea = signedArea;
                bestIndex3 = i;
            }
        }

        var areaEpsilon = maximumDistanceSquared * maximumDistanceSquared * 1e-6f;
        if (minSignedArea * minSignedArea > areaEpsilon)
        {
            PlaceCandidateInSlot(candidates[bestIndex2], 2, faceCenterB, tangentBX, tangentBY, candidateDepths[bestIndex2], rotationToWorld, worldOffsetB, ref manifold);
        }
        if (maxSignedArea * maxSignedArea > areaEpsilon)
        {
            PlaceCandidateInSlot(candidates[bestIndex3], 3, faceCenterB, tangentBX, tangentBY, candidateDepths[bestIndex3], rotationToWorld, worldOffsetB, ref manifold);
        }
    }

    public static unsafe void Test(ref ConvexHull a, ref ConvexHull b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        //Contact existence is cleared up front like the wide version; non-contact fields are only defined when the corresponding contact exists.
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 rA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 rB);
        ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);

        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, rB);
        var localOffsetA = -localOffsetB;
        var centerDistance = ScalarMath.Length(localOffsetA);
        var initialNormal = localOffsetA * (1f / centerDistance);
        if (centerDistance < 1e-8f)
        {
            initialNormal.X = 0f;
            initialNormal.Y = 1f;
            initialNormal.Z = 0f;
        }
        //EstimateEpsilonScale mirror: uses the first point of each hull.
        Vector3Wide.ReadSlot(ref a.Points[0], 0, out var firstPointA);
        Vector3Wide.ReadSlot(ref b.Points[0], 0, out var firstPointB);
        var aEpsilonScale = (MathF.Abs(firstPointA.X) + MathF.Abs(firstPointA.Y) + MathF.Abs(firstPointA.Z)) * (1f / 3f);
        var bEpsilonScale = (MathF.Abs(firstPointB.X) + MathF.Abs(firstPointB.Y) + MathF.Abs(firstPointB.Z)) * (1f / 3f);
        var epsilonScale = float.MinNative(aEpsilonScale, bEpsilonScale);
        var depthThreshold = -speculativeMargin;
        HullDepthRefiner.FindMinimumDepth(b, a, localOffsetA, bLocalOrientationA, initialNormal, 1e-5f * epsilonScale, depthThreshold,
            out var depth, out var localNormal, out var closestOnB);

        if (depth < depthThreshold)
        {
            //No contacts generated.
            return;
        }

        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, bLocalOrientationA);
        var negatedLocalNormalInA = -localNormalInA;
        var negatedOffsetToClosestOnA = localNormal * depth;
        var closestOnA = closestOnB - negatedOffsetToClosestOnA;
        var aToClosestOnA = closestOnA - localOffsetA;
        var closestOnAInA = ScalarMath.TransformByTransposed(aToClosestOnA, bLocalOrientationA);

        var boundingPlaneEpsilon = 1e-3f * epsilonScale;

        PickRepresentativeFace(ref a, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out var slotFaceNormalAInA, out var bestFaceIndexA);
        Matrix3x3.Transform(slotFaceNormalAInA, bLocalOrientationA, out var slotFaceNormalA);
        PickRepresentativeFace(ref b, localNormal, closestOnB, boundingPlaneEpsilon, out var slotFaceNormalB, out var bestFaceIndexB);
        Helpers.BuildOrthonormalBasis(slotFaceNormalB, out var bFaceX, out var bFaceY);

        //From here down this mirrors the wide tester's per-slot scalar contact generation.
        var slotLocalNormal = localNormal;
        var slotLocalOffsetA = localOffsetA;
        ref var slotBLocalOrientationA = ref bLocalOrientationA;

        a.GetVertexIndicesForFace(bestFaceIndexA, out var faceVertexIndicesA);
        b.GetVertexIndicesForFace(bestFaceIndexB, out var faceVertexIndicesB);

        //Create cached edge data for A.
        var cachedEdges = stackalloc CachedEdge[faceVertexIndicesA.Length];
        var previousIndexA = faceVertexIndicesA[faceVertexIndicesA.Length - 1];
        Vector3Wide.ReadSlot(ref a.Points[previousIndexA.BundleIndex], previousIndexA.InnerIndex, out var previousVertexA);
        Matrix3x3.Transform(previousVertexA, slotBLocalOrientationA, out previousVertexA);
        previousVertexA += slotLocalOffsetA;
        for (int i = 0; i < faceVertexIndicesA.Length; ++i)
        {
            ref var edge = ref cachedEdges[i];
            edge.MaximumContainmentDot = float.MinValue;
            var indexA = faceVertexIndicesA[i];
            Vector3Wide.ReadSlot(ref a.Points[indexA.BundleIndex], indexA.InnerIndex, out edge.Vertex);
            Matrix3x3.Transform(edge.Vertex, slotBLocalOrientationA, out edge.Vertex);
            edge.Vertex += slotLocalOffsetA;
            //Note flipped cross order; local normal points from B to A.
            edge.EdgePlaneNormal = Vector3.Cross(slotLocalNormal, edge.Vertex - previousVertexA);
            previousVertexA = edge.Vertex;
        }
        var maximumCandidateCount = Math.Max(Math.Max(faceVertexIndicesA.Length, faceVertexIndicesB.Length), Math.Min(faceVertexIndicesA.Length * 2, faceVertexIndicesB.Length * 2));
        var candidates = stackalloc ManifoldCandidateScalar[maximumCandidateCount];
        var candidateCount = 0;
        var previousIndexB = faceVertexIndicesB[faceVertexIndicesB.Length - 1];
        //Clip face B's edges against A's face, and test A's vertices against B's face.
        Vector3Wide.ReadSlot(ref b.Points[previousIndexB.BundleIndex], previousIndexB.InnerIndex, out var bFaceOrigin);
        var previousVertexB = bFaceOrigin;
        for (int faceVertexIndexB = 0; faceVertexIndexB < faceVertexIndicesB.Length; ++faceVertexIndexB)
        {
            var indexB = faceVertexIndicesB[faceVertexIndexB];
            Vector3Wide.ReadSlot(ref b.Points[indexB.BundleIndex], indexB.InnerIndex, out var vertexB);

            var edgeOffsetB = vertexB - previousVertexB;
            var edgePlaneNormalB = Vector3.Cross(edgeOffsetB, slotLocalNormal);

            var latestEntry = float.MinValue;
            var earliestExit = float.MaxValue;
            for (int faceVertexIndexA = 0; faceVertexIndexA < faceVertexIndicesA.Length; ++faceVertexIndexA)
            {
                ref var edgeA = ref cachedEdges[faceVertexIndexA];

                //Check containment in this B edge.
                var edgeBToEdgeA = edgeA.Vertex - previousVertexB;
                var containmentDot = Vector3.Dot(edgeBToEdgeA, edgePlaneNormalB);
                if (edgeA.MaximumContainmentDot < containmentDot)
                    edgeA.MaximumContainmentDot = containmentDot;

                var numerator = Vector3.Dot(edgeBToEdgeA, edgeA.EdgePlaneNormal);
                var denominator = Vector3.Dot(edgeA.EdgePlaneNormal, edgeOffsetB);

                //A plane is being 'entered' if the ray direction opposes the face normal.
                if (denominator < 0)
                {
                    if (numerator < latestEntry * denominator)
                        latestEntry = numerator / denominator;
                }
                else if (denominator > 0)
                {
                    if (numerator < earliestExit * denominator)
                        earliestExit = numerator / denominator;
                }
                else if (numerator < 0)
                {
                    //The B edge is parallel and outside the edge A, so there can be no intersection.
                    earliestExit = float.MinValue;
                    latestEntry = float.MaxValue;
                }
            }
            //We now have bounds on B's edge.
            if (latestEntry <= earliestExit)
            {
                latestEntry = latestEntry < 0 ? 0 : latestEntry;
                earliestExit = earliestExit > 1 ? 1 : earliestExit;
                var startId = (previousIndexB.BundleIndex << BundleIndexing.VectorShift) + previousIndexB.InnerIndex;
                var endId = (indexB.BundleIndex << BundleIndexing.VectorShift) + indexB.InnerIndex;
                var baseFeatureId = (startId ^ endId) << 8;
                if (earliestExit >= latestEntry && candidateCount < maximumCandidateCount)
                {
                    //Create max contact.
                    var point = edgeOffsetB * earliestExit + previousVertexB - bFaceOrigin;
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, bFaceX);
                    candidate.Y = Vector3.Dot(point, bFaceY);
                    candidate.FeatureId = baseFeatureId + endId;
                }
                if (latestEntry < earliestExit && latestEntry > 0 && candidateCount < maximumCandidateCount)
                {
                    //Create min contact.
                    var point = edgeOffsetB * latestEntry + previousVertexB - bFaceOrigin;
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, bFaceX);
                    candidate.Y = Vector3.Dot(point, bFaceY);
                    candidate.FeatureId = baseFeatureId + startId;
                }
            }
            previousIndexB = indexB;
            previousVertexB = vertexB;
        }
        //We've now analyzed every edge of B. Check for vertices from A to add.
        var inverseLocalNormalADotFaceNormalB = 1f / Vector3.Dot(slotLocalNormal, slotFaceNormalB);
        for (int i = 0; i < faceVertexIndicesA.Length && candidateCount < maximumCandidateCount; ++i)
        {
            ref var edge = ref cachedEdges[i];
            if (edge.MaximumContainmentDot <= 0)
            {
                //This vertex was contained by all b edge plane normals. Include it, projected onto B's surface.
                var bFaceToVertexA = edge.Vertex - bFaceOrigin;
                var distance = Vector3.Dot(bFaceToVertexA, slotFaceNormalB) * inverseLocalNormalADotFaceNormalB;
                var bFaceToProjectedVertexA = bFaceToVertexA - slotLocalNormal * distance;

                var newContactIndex = candidateCount++;
                ref var candidate = ref candidates[newContactIndex];
                candidate.X = Vector3.Dot(bFaceX, bFaceToProjectedVertexA);
                candidate.Y = Vector3.Dot(bFaceY, bFaceToProjectedVertexA);
                candidate.FeatureId = i;
            }
        }
        Reduce(candidates, candidateCount, slotFaceNormalA, 1f / Vector3.Dot(slotFaceNormalA, slotLocalNormal), cachedEdges[0].Vertex, bFaceOrigin, bFaceX, bFaceY,
            epsilonScale, depthThreshold, rB, offsetB, ref manifold);
        Matrix3x3.Transform(localNormal, rB, out manifold.Normal);
    }
}
