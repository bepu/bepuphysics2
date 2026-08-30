using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace AosBaselines;

using CylinderHullDepthRefiner = ScalarDepthRefiner<ConvexHull, HullSupportScalar, Cylinder, CylinderSupportScalar>;

/// <summary>
/// Scalar AoS port of CylinderConvexHullTester, bitwise identical per lane.
/// ProjectOntoCap/IntersectLineCircle/InsertContact and the whole cap/side contact generation are already per-slot scalar in the
/// wide tester and are copied verbatim (Vector2/Vector3.Dot intrinsics included); the ported pieces are the wide setup, the
/// feature identification (cap center, interior points, side edge center), and the DepthRefiner.
/// </summary>
public struct CylinderConvexHullScalarTester : IShapeHullScalarTester<Cylinder>
{
    //Verbatim copy of the wide tester's scalar helper.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ProjectOntoCap(Vector3 capCenter, in Matrix3x3 cylinderOrientation, float inverseLocalNormalDotAY, Vector3 localNormal, Vector3 point, out Vector2 projected)
    {
        var pointToCapCenter = capCenter - point;
        var t = Vector3.Dot(pointToCapCenter, cylinderOrientation.Y) * inverseLocalNormalDotAY;
        var projectionOffsetB = localNormal * t;
        var projectedPoint = point - projectionOffsetB;
        var capCenterToProjectedPoint = projectedPoint - capCenter;
        projected = new Vector2(
            Vector3.Dot(capCenterToProjectedPoint, cylinderOrientation.X),
            Vector3.Dot(capCenterToProjectedPoint, cylinderOrientation.Z));
    }

    //Verbatim copy of the wide tester's scalar helper (note: this differs from the cylinder-cylinder tester's variant).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static bool IntersectLineCircle(in Vector2 linePosition, in Vector2 lineDirection, float radius, out float tMin, out float tMax)
    {
        var a = Vector2.Dot(lineDirection, lineDirection);
        var inverseA = 1f / a;
        var b = Vector2.Dot(linePosition, lineDirection);
        var c = Vector2.Dot(linePosition, linePosition);
        var radiusSquared = radius * radius;
        c -= radiusSquared;
        var d = b * b - a * c;
        if (d < 0)
        {
            tMin = 0;
            tMax = 0;
            return false;
        }
        var tOffset = (float)Math.Sqrt(d) * inverseA;
        var tBase = -b * inverseA;
        if (a < 1e-12f && a > -1e-12f)
        {
            //If the projected line direction is zero, just compress the interval to tBase.
            tMin = tBase;
            tMax = tBase;
        }
        else
        {
            tMin = tBase - tOffset;
            tMax = tBase + tOffset;
        }
        if (tMin < 0)
            tMin = 0;
        if (tMax > 1)
            tMax = 1;
        return true;
    }

    //Scalar retarget of the wide tester's InsertContact (which is scalar math writing into wide manifold slots).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void InsertContact(Vector3 slotSideEdgeCenter, Vector3 slotCylinderEdgeAxis, float t,
        Vector3 hullFaceOrigin, Vector3 slotHullFaceNormal, float inverseDepthDenominator,
        in Matrix3x3 slotHullOrientation, Vector3 slotOffsetB, int featureId,
        ref Vector3 contactOffsetA, ref float contactDepth, ref int contactFeatureId, ref bool contactExists)
    {
        //Create max contact.
        var localPoint = slotSideEdgeCenter + slotCylinderEdgeAxis * t;
        //depth = dot(faceCenterB - pointOnFaceA, faceNormalB) / dot(faceNormalB, normal)
        var depth = Vector3.Dot(hullFaceOrigin - localPoint, slotHullFaceNormal) * inverseDepthDenominator;
        Matrix3x3.Transform(localPoint, slotHullOrientation, out var offsetA);
        offsetA += slotOffsetB;
        contactOffsetA = offsetA;
        contactDepth = depth;
        contactFeatureId = featureId;
        contactExists = true;
    }

    /// <summary>
    /// Mirrors the per-lane semantics of BoxCylinderTester.GenerateInteriorPoints (wide select/abs/mul math; no dots, so
    /// componentwise mirroring is exact; MathF.Max/Min mirror Vector.Max/Min's IEEE semantics).
    /// </summary>
    static void GenerateInteriorPoints(in Cylinder cylinder, Vector3 cylinderLocalNormal, Vector3 localClosestOnCylinder,
        out Vector2 interior0, out Vector2 interior1, out Vector2 interior2, out Vector2 interior3)
    {
        const float interpolationMin = 0.9999f;
        const float inverseInterpolationSpan = 1f / 0.00005f;
        var parallelWeight = MathF.Max(0f, MathF.Min(1f, (MathF.Abs(cylinderLocalNormal.Y) - interpolationMin) * inverseInterpolationSpan));
        var deepestWeight = 1f - parallelWeight;
        var replaceX = MathF.Abs(localClosestOnCylinder.X) > MathF.Abs(localClosestOnCylinder.Z);
        var replace0 = localClosestOnCylinder.X > 0f & replaceX;
        var replace1 = localClosestOnCylinder.X <= 0f & replaceX;
        var replace2 = localClosestOnCylinder.Z > 0f & !replaceX;
        var replace3 = localClosestOnCylinder.Z <= 0f & !replaceX;
        var scaledRadius = parallelWeight * cylinder.Radius;
        interior0 = replace0 ? new Vector2(deepestWeight * localClosestOnCylinder.X + scaledRadius, deepestWeight * localClosestOnCylinder.Z) : new Vector2(cylinder.Radius, 0f);
        interior1 = replace1 ? new Vector2(deepestWeight * localClosestOnCylinder.X - scaledRadius, deepestWeight * localClosestOnCylinder.Z) : new Vector2(-cylinder.Radius, 0f);
        interior2 = replace2 ? new Vector2(deepestWeight * localClosestOnCylinder.X, deepestWeight * localClosestOnCylinder.Z + scaledRadius) : new Vector2(0f, cylinder.Radius);
        interior3 = replace3 ? new Vector2(deepestWeight * localClosestOnCylinder.X, deepestWeight * localClosestOnCylinder.Z - scaledRadius) : new Vector2(0f, -cylinder.Radius);
    }

    public static unsafe void Test(in Cylinder a, ref ConvexHull b, float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 cylinderOrientation);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 hullOrientation);
        ScalarMath.MultiplyByTranspose(cylinderOrientation, hullOrientation, out var hullLocalCylinderOrientation);

        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, hullOrientation);
        var localOffsetA = -localOffsetB;
        var centerDistance = ScalarMath.Length(localOffsetA);
        var initialNormal = localOffsetA * (1f / centerDistance);
        if (centerDistance < 1e-8f)
        {
            initialNormal.X = 0f;
            initialNormal.Y = 1f;
            initialNormal.Z = 0f;
        }
        var hullEpsilonScale = HullScalarHelpers.EstimateEpsilonScale(ref b);
        var epsilonScale = MathF.Min(MathF.Max(a.HalfLength, a.Radius), hullEpsilonScale);
        var depthThreshold = -speculativeMargin;
        CylinderHullDepthRefiner.FindMinimumDepth(b, a, localOffsetA, hullLocalCylinderOrientation, initialNormal, 1e-5f * epsilonScale, depthThreshold,
            out var depth, out var localNormal, out var closestOnHull);

        if (depth < depthThreshold)
        {
            //No contacts generated.
            return;
        }

        //Identify the cylinder feature.
        var closestOnCylinderOffset = localNormal * depth;
        var closestOnCylinder = closestOnHull - closestOnCylinderOffset;
        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, hullLocalCylinderOrientation);
        var inverseLocalNormalDotCapNormal = 1f / localNormalInA.Y;
        var useCap = MathF.Abs(localNormalInA.Y) > 0.70710678118f;
        Vector3 capCenter = default;
        Vector2 interior0 = default, interior1 = default, interior2 = default, interior3 = default;
        if (useCap)
        {
            var useBottom = localNormalInA.Y > 0f;
            capCenter = hullLocalCylinderOrientation.Y * (useBottom ? -a.HalfLength : a.HalfLength);
            capCenter = capCenter + localOffsetA;

            var hullLocalCylinderToClosestOnCylinder = closestOnCylinder - localOffsetA;
            var cylinderLocalCylinderToClosestOnCylinder = ScalarMath.TransformByTransposed(hullLocalCylinderToClosestOnCylinder, hullLocalCylinderOrientation);
            GenerateInteriorPoints(a, localNormalInA, cylinderLocalCylinderToClosestOnCylinder, out interior0, out interior1, out interior2, out interior3);
        }

        Vector3 cylinderSideEdgeCenter = default;
        if (!useCap)
        {
            //If the contact is on the cylinder's side, use the closestOnHull-derived position rather than resampling the support function.
            var cylinderToClosestOnCylinder = closestOnCylinder - localOffsetA;
            var cylinderLocalClosestOnCylinderY = ScalarMath.Dot(cylinderToClosestOnCylinder, hullLocalCylinderOrientation.Y);
            var cylinderEdgeCenterToClosestOnCylinder = hullLocalCylinderOrientation.Y * cylinderLocalClosestOnCylinderY;
            cylinderSideEdgeCenter = closestOnCylinder - cylinderEdgeCenterToClosestOnCylinder;
        }
        //We can create up to 2 contacts per hull edge; the wide bound uses the hull's full FaceVertexIndices length.
        int maximumCandidateCount = Math.Max(4, b.FaceVertexIndices.Length * 2);
        var candidates = stackalloc ManifoldCandidateScalar[maximumCandidateCount];
        var boundingPlaneEpsilon = 1e-3f * epsilonScale;

        ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnHull, boundingPlaneEpsilon, out var slotHullFaceNormal, out var bestFaceIndex);
        b.GetVertexIndicesForFace(bestFaceIndex, out var faceVertexIndices);
        var slotLocalNormal = localNormal;

        if (useCap)
        {
            var candidateCount = 0;
            //The cap is the representative feature. Clip the hull's edges against the cap's circle, and test the cylinder's
            //heuristically chosen 'vertices' against the hull edges for containment. Per-slot scalar region; verbatim.
            var slotCapCenter = capCenter;
            var slotInverseLocalNormalDotCapNormal = inverseLocalNormalDotCapNormal;

            var interiorPointsX = new Vector4(interior0.X, interior1.X, interior2.X, interior3.X);
            var interiorPointsY = new Vector4(interior0.Y, interior1.Y, interior2.Y, interior3.Y);
            var slotRadius = a.Radius;
            ref var slotCylinderOrientation = ref hullLocalCylinderOrientation;

            var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
            Vector3Wide.ReadSlot(ref b.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
            ProjectOntoCap(slotCapCenter, slotCylinderOrientation, slotInverseLocalNormalDotCapNormal, slotLocalNormal, hullFaceOrigin, out var previousVertex);
            var maximumInteriorContainmentDots = Vector4.Zero;

            for (int i = 0; i < faceVertexIndices.Length; ++i)
            {
                var index = faceVertexIndices[i];
                Vector3Wide.ReadSlot(ref b.Points[index.BundleIndex], index.InnerIndex, out var hullVertex);
                ProjectOntoCap(slotCapCenter, slotCylinderOrientation, slotInverseLocalNormalDotCapNormal, slotLocalNormal, hullVertex, out var vertex);

                //Test all the cap's interior points against this edge's plane normal (which, since we've projected the vertex, is just a perp dot product).
                var hullEdgeOffset = vertex - previousVertex;
                var previousStartX = new Vector4(previousVertex.X);
                var previousStartY = new Vector4(previousVertex.Y);
                var hullEdgeOffsetX = new Vector4(hullEdgeOffset.X);
                var hullEdgeOffsetY = new Vector4(hullEdgeOffset.Y);
                var interiorPointContainmentDots = (interiorPointsX - previousStartX) * hullEdgeOffsetY - (interiorPointsY - previousStartY) * hullEdgeOffsetX;
                //If we're generating contacts with the bottom cap, then the visible winding of the hull is flipped and the containment signs will be negated.
                if (slotInverseLocalNormalDotCapNormal > 0)
                    interiorPointContainmentDots *= -1;
                maximumInteriorContainmentDots = Vector4.Max(interiorPointContainmentDots, maximumInteriorContainmentDots);

                //Test the projected hull edge against the cap.
                if (IntersectLineCircle(previousVertex, hullEdgeOffset, slotRadius, out var tMin, out var tMax))
                {
                    //We now have a convex hull edge interval. Add contacts for it.
                    //Create max contact if max >= min.
                    //Create min if min < max and min > 0.
                    var startId = (previousIndex.BundleIndex << BundleIndexing.VectorShift) + previousIndex.InnerIndex;
                    var endId = (index.BundleIndex << BundleIndexing.VectorShift) + index.InnerIndex;
                    var baseFeatureId = (startId ^ endId) << 8;
                    if (tMax >= tMin && candidateCount < maximumCandidateCount)
                    {
                        //Create max contact.
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        Unsafe.As<float, Vector2>(ref candidate.X) = hullEdgeOffset * tMax + previousVertex;
                        candidate.FeatureId = baseFeatureId + endId;

                    }
                    if (tMin < tMax && tMin > 0 && candidateCount < maximumCandidateCount)
                    {
                        //Create min contact.
                        var newContactIndex = candidateCount++;
                        ref var candidate = ref candidates[newContactIndex];
                        Unsafe.As<float, Vector2>(ref candidate.X) = hullEdgeOffset * tMin + previousVertex;
                        candidate.FeatureId = baseFeatureId + startId;

                    }
                }

                previousIndex = index;
                previousVertex = vertex;
            }

            if (candidateCount < maximumCandidateCount)
            {
                //Try adding the cylinder 'vertex' contacts.
                //We took the maximum of all interior-hulledgeplane tests; if a vertex is outside any edge plane, the maximum dot will be positive.
                if (maximumInteriorContainmentDots.X <= 0)
                {
                    ref var candidate = ref candidates[candidateCount++];
                    candidate.X = interiorPointsX.X;
                    candidate.Y = interiorPointsY.X;
                    candidate.FeatureId = 0;
                }
                if (candidateCount == maximumCandidateCount)
                    goto SkipVertexCandidates;
                if (maximumInteriorContainmentDots.Y <= 0)
                {
                    ref var candidate = ref candidates[candidateCount++];
                    candidate.X = interiorPointsX.Y;
                    candidate.Y = interiorPointsY.Y;
                    candidate.FeatureId = 1;
                }
                if (candidateCount == maximumCandidateCount)
                    goto SkipVertexCandidates;
                if (maximumInteriorContainmentDots.Z <= 0)
                {
                    ref var candidate = ref candidates[candidateCount++];
                    candidate.X = interiorPointsX.Z;
                    candidate.Y = interiorPointsY.Z;
                    candidate.FeatureId = 2;
                }
                if (candidateCount < maximumCandidateCount && maximumInteriorContainmentDots.W <= 0)
                {
                    ref var candidate = ref candidates[candidateCount++];
                    candidate.X = interiorPointsX.W;
                    candidate.Y = interiorPointsY.W;
                    candidate.FeatureId = 3;
                }
            SkipVertexCandidates:;
            }
            //We have found all contacts for this hull slot. There may be more contacts than we want (4), so perform a reduction.
            //Note that we're working on the cylinder's cap, so the parameters get flipped around. Gets pushed back onto the hull in the postpass.
            ConvexHullPairScalarTester.Reduce(candidates, candidateCount, slotHullFaceNormal, -1f / Vector3.Dot(slotLocalNormal, slotHullFaceNormal), hullFaceOrigin, slotCapCenter,
                hullLocalCylinderOrientation.X, hullLocalCylinderOrientation.Z, epsilonScale, depthThreshold, hullOrientation, offsetB, ref manifold);
        }
        else
        {
            //The side edge is the representative feature. Clip the cylinder's side edge against the hull edges; similar to capsule-hull.
            var slotCylinderEdgeAxis = hullLocalCylinderOrientation.Y;
            var slotSideEdgeCenter = cylinderSideEdgeCenter;
            var previousIndex = faceVertexIndices[faceVertexIndices.Length - 1];
            Vector3Wide.ReadSlot(ref b.Points[previousIndex.BundleIndex], previousIndex.InnerIndex, out var hullFaceOrigin);
            var previousVertex = hullFaceOrigin;
            var latestEntryNumerator = float.MaxValue;
            var latestEntryDenominator = -1f;
            var earliestExitNumerator = float.MaxValue;
            var earliestExitDenominator = 1f;
            for (int i = 0; i < faceVertexIndices.Length; ++i)
            {
                var index = faceVertexIndices[i];
                Vector3Wide.ReadSlot(ref b.Points[index.BundleIndex], index.InnerIndex, out var vertex);

                var edgeOffset = vertex - previousVertex;
                var edgePlaneNormal = Vector3.Cross(edgeOffset, slotLocalNormal);

                //t = dot(pointOnPlane - capsuleCenter, planeNormal) / dot(planeNormal, rayDirection)
                var cylinderSideToHullEdgeStart = previousVertex - slotSideEdgeCenter;
                var numerator = Vector3.Dot(cylinderSideToHullEdgeStart, edgePlaneNormal);
                var denominator = Vector3.Dot(edgePlaneNormal, slotCylinderEdgeAxis);
                previousVertex = vertex;

                //A plane is being 'entered' if the ray direction opposes the face normal.
                var edgePlaneNormalLengthSquared = edgePlaneNormal.LengthSquared();
                var denominatorSquared = denominator * denominator;

                const float min = 1e-5f;
                const float max = 3e-4f;
                const float inverseSpan = 1f / (max - min);
                if (denominatorSquared > min * edgePlaneNormalLengthSquared)
                {
                    if (denominatorSquared < max * edgePlaneNormalLengthSquared)
                    {
                        //As the angle between the axis and edge plane approaches zero, the axis should unrestrict.
                        var restrictWeight = (denominatorSquared / edgePlaneNormalLengthSquared - min) * inverseSpan;
                        if (restrictWeight < 0)
                            restrictWeight = 0;
                        else if (restrictWeight > 1)
                            restrictWeight = 1;
                        var unrestrictedNumerator = a.HalfLength * denominator;
                        if (denominator < 0)
                            unrestrictedNumerator = -unrestrictedNumerator;
                        numerator = restrictWeight * numerator + (1 - restrictWeight) * unrestrictedNumerator;
                    }
                    if (denominator < 0)
                    {
                        if (numerator * latestEntryDenominator > latestEntryNumerator * denominator)
                        {
                            latestEntryNumerator = numerator;
                            latestEntryDenominator = denominator;
                        }
                    }
                    else // if (denominator > 0)
                    {
                        if (numerator * earliestExitDenominator < earliestExitNumerator * denominator)
                        {
                            earliestExitNumerator = numerator;
                            earliestExitDenominator = denominator;
                        }
                    }
                }
            }
            var slotSideEdgeHalfLength = a.HalfLength;
            var latestEntry = latestEntryNumerator / latestEntryDenominator;
            var earliestExit = earliestExitNumerator / earliestExitDenominator;
            var inverseDepthDenominator = 1f / Vector3.Dot(slotHullFaceNormal, slotLocalNormal);
            var negatedEdgeLength = -slotSideEdgeHalfLength;
            if (latestEntry < negatedEdgeLength)
                latestEntry = negatedEdgeLength;
            if (latestEntry > slotSideEdgeHalfLength)
                latestEntry = slotSideEdgeHalfLength;
            if (earliestExit < negatedEdgeLength)
                earliestExit = negatedEdgeLength;
            if (earliestExit > slotSideEdgeHalfLength)
                earliestExit = slotSideEdgeHalfLength;
            InsertContact(
                slotSideEdgeCenter, slotCylinderEdgeAxis, earliestExit,
                hullFaceOrigin, slotHullFaceNormal, inverseDepthDenominator, hullOrientation, offsetB, 0,
                ref manifold.OffsetA0, ref manifold.Depth0, ref manifold.FeatureId0, ref manifold.Contact0Exists);
            if (earliestExit - latestEntry > slotSideEdgeHalfLength * 1e-3f)
            {
                InsertContact(
                    slotSideEdgeCenter, slotCylinderEdgeAxis, latestEntry,
                    hullFaceOrigin, slotHullFaceNormal, inverseDepthDenominator, hullOrientation, offsetB, 1,
                    ref manifold.OffsetA1, ref manifold.Depth1, ref manifold.FeatureId1, ref manifold.Contact1Exists);
            }
            else
            {
                manifold.Contact1Exists = false;
            }
            manifold.Contact2Exists = false;
            manifold.Contact3Exists = false;
        }
        //Push the manifold onto the hull. The reduction does not assign the normal. Fill it in.
        Matrix3x3.Transform(localNormal, hullOrientation, out manifold.Normal);
        var offset0 = manifold.Normal * manifold.Depth0;
        var offset1 = manifold.Normal * manifold.Depth1;
        var offset2 = manifold.Normal * manifold.Depth2;
        var offset3 = manifold.Normal * manifold.Depth3;
        manifold.OffsetA0 = manifold.OffsetA0 + offset0;
        manifold.OffsetA1 = manifold.OffsetA1 + offset1;
        manifold.OffsetA2 = manifold.OffsetA2 + offset2;
        manifold.OffsetA3 = manifold.OffsetA3 + offset3;
    }
}
