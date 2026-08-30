using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

using BoxCylinderDepthRefiner = ScalarDepthRefiner<Cylinder, CylinderSupportScalar, Box, BoxSupportScalar>;

/// <summary>
/// Scalar AoS port of BoxCylinderTester, bitwise identical per lane. Works in the cylinder's (b's) local space like the wide
/// implementation; the refiner treats the cylinder as its shape A and the box as its shape B, matching the wide call.
/// Structure and comments deliberately track BoxCylinderTester; see that file for the algorithm's reasoning.
/// </summary>
public static class BoxCylinderScalarTester
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Dot2(Vector2 a, Vector2 b) => a.X * b.X + a.Y * b.Y;

    /// <summary>
    /// Mirror of BoxCylinderTester.IntersectLineCircle. Note that this is NOT the cylinder-cylinder or cylinder-hull variant:
    /// it guards the direction length with a 2e-38 max (MathF.Max mirrors Vector.Max's IEEE semantics), reports an
    /// intersected flag (d >= 0), and clamps the discriminant to zero instead of branching.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void IntersectLineCircle(Vector2 linePosition, Vector2 lineDirection, float radius, out float tMin, out float tMax, out bool intersected)
    {
        var a = Dot2(lineDirection, lineDirection);
        a = MathF.Max(a, 2e-38f); //Guard against division by zero.
        var inverseA = 1f / a;
        var b = Dot2(linePosition, lineDirection);
        var c = Dot2(linePosition, linePosition);
        var radiusSquared = radius * radius;
        c -= radiusSquared;
        var d = b * b - a * c;
        intersected = d >= 0f;
        var tOffset = MathF.Sqrt(MathF.Max(0f, d)) * inverseA;
        var tBase = -b * inverseA;
        tMin = tBase - tOffset;
        tMax = tBase + tOffset;
    }

    /// <summary>
    /// Mirror of BoxCylinderTester.AddCandidateForEdge. The wide AddCandidate scatter reduces to a branchy append per lane.
    /// The allowFeatureContacts mask is the cap-lane gate, which is always true on this scalar path.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void AddCandidateForEdge(Vector2 edgeStart, Vector2 edgeOffset, float tMin, float tMax, bool intersected, int edgeId,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        //There's an edge id (0, 1, 2, 3), and then a flag for min or max (0 or 4).
        Unsafe.SkipInit(out CandidateAos candidate);
        candidate.Depth = 0f;
        candidate.FeatureId = edgeId;
        candidate.X = edgeStart.X + edgeOffset.X * tMin;
        candidate.Y = edgeStart.Y + edgeOffset.Y * tMin;
        //If tMin is overlapping the previous edge's tMax (or this edge's tMax!), don't bother including it.
        if (intersected & tMin < tMax & tMin > 0f)
        {
            candidates[candidateCount++] = candidate;
        }
        candidate.FeatureId = edgeId + 4;
        candidate.X = edgeStart.X + edgeOffset.X * tMax;
        candidate.Y = edgeStart.Y + edgeOffset.Y * tMax;
        if (intersected & tMax > 0f)
        {
            candidates[candidateCount++] = candidate;
        }
    }

    /// <summary>
    /// Mirrors the per-lane semantics of BoxCylinderTester.GenerateInteriorPoints (wide select/abs/mul math; no dots, so
    /// componentwise mirroring is exact; MathF.Max/Min mirror Vector.Max/Min's IEEE semantics). Identical to the copy in
    /// CylinderConvexHullScalarTester (which is private in a frozen file, so it is duplicated here).
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

    /// <summary>
    /// Mirror of BoxCylinderTester.TryAddInteriorPoint.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void TryAddInteriorPoint(Vector2 point, int featureId,
        Vector2 edge0010, float edge0010PlaneMin, float edge0010PlaneMax,
        Vector2 edge1011, float edge1011PlaneMin, float edge1011PlaneMax,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        var edge0010Dot = point.X * edge0010.Y - point.Y * edge0010.X;
        var edge1011Dot = point.X * edge1011.Y - point.Y * edge1011.X;
        var contained =
            edge0010Dot >= edge0010PlaneMin & edge0010Dot <= edge0010PlaneMax &
            edge1011Dot >= edge1011PlaneMin & edge1011Dot <= edge1011PlaneMax;
        if (contained)
        {
            Unsafe.SkipInit(out CandidateAos candidate);
            candidate.X = point.X;
            candidate.Y = point.Y;
            candidate.Depth = 0f;
            candidate.FeatureId = featureId;
            candidates[candidateCount++] = candidate;
        }
    }

    /// <summary>
    /// Mirror of CylinderPairTester.ProjectOntoCapB (the copy in CylinderPairScalarTester is private in a frozen file).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector2 ProjectOntoCapB(float capCenterBY, float inverseLocalNormalY, Vector3 localNormal, Vector3 point)
    {
        var tAOnB = (point.Y - capCenterBY) * inverseLocalNormalY;
        return new Vector2(point.X - localNormal.X * tAOnB, point.Z - localNormal.Z * tAOnB);
    }

    /// <summary>
    /// Scalar mirror of the wide ManifoldCandidateHelper.Reduce; identical to BoxPairScalarTester's copy (that one is private,
    /// and the harness rules freeze that file, so the mirror is duplicated here). The wide bundle-level count squishing only
    /// skips candidates that are masked out per lane anyway, so per-lane results are identical.
    /// </summary>
    static void Reduce(Span<CandidateAos> candidates, int candidateCount,
        Vector3 faceNormalA, float inverseFaceNormalADotNormal, Vector3 faceCenterBToFaceCenterA, Vector3 tangentBX, Vector3 tangentBY,
        float epsilonScale, float minimumDepth,
        out CandidateAos contact0, out CandidateAos contact1, out CandidateAos contact2, out CandidateAos contact3,
        out bool contact0Exists, out bool contact1Exists, out bool contact2Exists, out bool contact3Exists)
    {
        //ComputeDepthsForReduction: cast a ray from the point on face B toward the plane of face A along the contact normal.
        var dotAxis = faceNormalA * inverseFaceNormalADotNormal;
        var negativeBaseDot = ScalarMath.Dot(faceCenterBToFaceCenterA, dotAxis);
        var xDot = ScalarMath.Dot(tangentBX, dotAxis);
        var yDot = ScalarMath.Dot(tangentBY, dotAxis);
        ref var candidatesBase = ref MemoryMarshal.GetReference(candidates);
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref Unsafe.Add(ref candidatesBase, i);
            candidate.Depth = candidate.X * xDot + candidate.Y * yDot - negativeBaseDot;
        }

        contact0 = default;
        contact1 = default;
        contact2 = default;
        contact3 = default;
        if (candidateCount == 0)
        {
            contact0Exists = false;
            contact1Exists = false;
            contact2Exists = false;
            contact3Exists = false;
            return;
        }

        //Index-tracking select chains; see BoxPairScalarTester.Reduce for commentary.
        const float extremityScale = 1e-2f;
        var bestScore = -float.MaxValue;
        var bestIndex = 0f;
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref Unsafe.Add(ref candidatesBase, i);
            var candidateExists = ScalarMath.GreaterMask(candidate.Depth, minimumDepth);
            var extremity = MathF.Abs(candidate.X * 0.7946897654f + candidate.Y * 0.60701579614f);
            var candidateScore = candidate.Depth + ScalarMath.Select(ScalarMath.GreaterOrEqualMask(candidate.Depth, 0f), extremity * extremityScale, 0f);
            var candidateIsHighestScore = candidateExists & ScalarMath.GreaterMask(candidateScore, bestScore);
            bestIndex = ScalarMath.Select(candidateIsHighestScore, i, bestIndex);
            bestScore = ScalarMath.Select(candidateIsHighestScore, candidateScore, bestScore);
        }
        contact0Exists = bestScore > -float.MaxValue;
        if (contact0Exists)
            contact0 = Unsafe.Add(ref candidatesBase, (int)bestIndex);

        //Find the most distant point from the starting contact.
        var maxDistanceSquared = 0f;
        var mostDistantIndex = 0f;
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref Unsafe.Add(ref candidatesBase, i);
            var offsetX = candidate.X - contact0.X;
            var offsetY = candidate.Y - contact0.Y;
            var distanceSquared = offsetX * offsetX + offsetY * offsetY;
            var candidateExists = ScalarMath.GreaterMask(candidate.Depth, minimumDepth);
            var candidateIsMostDistant = ScalarMath.GreaterMask(distanceSquared, maxDistanceSquared) & candidateExists;
            mostDistantIndex = ScalarMath.Select(candidateIsMostDistant, i, mostDistantIndex);
            maxDistanceSquared = ScalarMath.Select(candidateIsMostDistant, distanceSquared, maxDistanceSquared);
        }
        if (maxDistanceSquared > 0f)
            contact1 = Unsafe.Add(ref candidatesBase, (int)mostDistantIndex);
        contact1Exists = maxDistanceSquared > epsilonScale * epsilonScale * 1e-6f;

        //Pick the points with the largest magnitude negative and positive signed areas relative to the edge formed by the first two contacts.
        var edgeOffsetX = contact1.X - contact0.X;
        var edgeOffsetY = contact1.Y - contact0.Y;
        var minSignedArea = 0f;
        var maxSignedArea = 0f;
        var minAreaIndex = 0f;
        var maxAreaIndex = 0f;
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref Unsafe.Add(ref candidatesBase, i);
            var candidateOffsetX = candidate.X - contact0.X;
            var candidateOffsetY = candidate.Y - contact0.Y;
            var signedArea = candidateOffsetX * edgeOffsetY - candidateOffsetY * edgeOffsetX;
            //Penalize speculative contacts; they are not as important in general.
            signedArea = ScalarMath.Select(ScalarMath.LessMask(candidate.Depth, 0f), 0.25f * signedArea, signedArea);

            var candidateExists = ScalarMath.GreaterMask(candidate.Depth, minimumDepth);
            var isMinArea = ScalarMath.LessMask(signedArea, minSignedArea) & candidateExists;
            minAreaIndex = ScalarMath.Select(isMinArea, i, minAreaIndex);
            minSignedArea = ScalarMath.Select(isMinArea, signedArea, minSignedArea);
            var isMaxArea = ScalarMath.GreaterMask(signedArea, maxSignedArea) & candidateExists;
            maxAreaIndex = ScalarMath.Select(isMaxArea, i, maxAreaIndex);
            maxSignedArea = ScalarMath.Select(isMaxArea, signedArea, maxSignedArea);
        }
        //Selection happened iff the accumulator moved off its strict-compare initial value, mirroring the chains' select conditions.
        if (minSignedArea < 0f)
            contact2 = Unsafe.Add(ref candidatesBase, (int)minAreaIndex);
        if (maxSignedArea > 0f)
            contact3 = Unsafe.Add(ref candidatesBase, (int)maxAreaIndex);

        var epsilon = maxDistanceSquared * maxDistanceSquared * 1e-6f;
        contact2Exists = minSignedArea * minSignedArea > epsilon;
        contact3Exists = maxSignedArea * maxSignedArea > epsilon;
    }

    public static void Test(
        in Box a, in Cylinder b, float speculativeMargin,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 worldRA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 worldRB);
        //Work in b's local space.
        ScalarMath.MultiplyByTranspose(worldRA, worldRB, out var rA);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, worldRB);
        var localOffsetA = -localOffsetB;

        var length = ScalarMath.Length(localOffsetA);
        var localNormal = localOffsetA * (1f / length);
        if (length < 1e-10f)
        {
            localNormal.X = 0f;
            localNormal.Y = 1f;
            localNormal.Z = 0f;
        }

        //We now have a decent estimate for the local normal. Refine it to a local minimum.
        var depthThreshold = -speculativeMargin;
        var epsilonScale = float.MinNative(float.MaxNative(a.HalfWidth, float.MaxNative(a.HalfHeight, a.HalfLength)), float.MaxNative(b.HalfLength, b.Radius));
        BoxCylinderDepthRefiner.FindMinimumDepth(
            b, a, localOffsetA, rA, localNormal, epsilonScale * 1e-6f, depthThreshold,
            out var depth, out localNormal, out var closestOnB, maximumIterations: 25);

        if (depth < depthThreshold)
        {
            //The depth is lower than the speculative margin; no contacts.
            return;
        }

        //We generate contacts according to the dominant features along the collision normal.
        //The possible pairs are: Face A-Cap B, Face A-Side B.

        //Identify the box face.
        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, rA);
        var absLocalNormalInA = Vector3.Abs(localNormalInA);
        var useX = absLocalNormalInA.X > absLocalNormalInA.Y & absLocalNormalInA.X > absLocalNormalInA.Z;
        var useY = absLocalNormalInA.Y > absLocalNormalInA.Z & !useX;
        var useXMask = ScalarMath.Mask(useX);
        var useYMask = ScalarMath.Mask(useY);
        var boxFaceNormal = ScalarMath.Select(useYMask, rA.Y, ScalarMath.Select(useXMask, rA.X, rA.Z));
        var boxFaceX = ScalarMath.Select(useYMask, rA.Z, ScalarMath.Select(useXMask, rA.Y, rA.X));
        var boxFaceY = ScalarMath.Select(useYMask, rA.X, ScalarMath.Select(useXMask, rA.Z, rA.Y));
        var negateFace = useX ? localNormalInA.X > 0f : (useY ? localNormalInA.Y > 0f : localNormalInA.Z > 0f);
        var negateFaceMask = ScalarMath.Mask(negateFace);
        //Whole-vector negate-selects; bitwise identical to the wide componentwise ConditionallyNegate.
        boxFaceNormal = ScalarMath.Select(negateFaceMask, -boxFaceNormal, boxFaceNormal);
        boxFaceX = ScalarMath.Select(negateFaceMask, -boxFaceX, boxFaceX);
        boxFaceY = ScalarMath.Select(negateFaceMask, -boxFaceY, boxFaceY);
        var boxFaceHalfWidth = ScalarMath.Select(useXMask, a.HalfHeight, ScalarMath.Select(useYMask, a.HalfLength, a.HalfWidth));
        var boxFaceHalfHeight = ScalarMath.Select(useXMask, a.HalfLength, ScalarMath.Select(useYMask, a.HalfWidth, a.HalfHeight));
        var boxFaceNormalOffset = ScalarMath.Select(useXMask, a.HalfWidth, ScalarMath.Select(useYMask, a.HalfHeight, a.HalfLength));
        var boxFaceCenterOffset = boxFaceNormal * boxFaceNormalOffset;
        var boxFaceCenter = boxFaceCenterOffset + localOffsetA;
        var boxFaceXOffset = boxFaceX * boxFaceHalfWidth;
        var boxFaceYOffset = boxFaceY * boxFaceHalfHeight;
        var v00 = boxFaceCenter - boxFaceXOffset;
        v00 = v00 - boxFaceYOffset;
        var v11 = boxFaceCenter + boxFaceXOffset;
        v11 = v11 + boxFaceYOffset;

        var capCenterBY = ScalarMath.Select(ScalarMath.LessMask(localNormal.Y, 0f), -b.HalfLength, b.HalfLength);

        var useCap = MathF.Abs(localNormal.Y) > 0.70710678118f;

        var faceNormalDotLocalNormal = ScalarMath.Dot(boxFaceNormal, localNormal);
        var inverseFaceNormalDotLocalNormal = 1f / faceNormalDotLocalNormal;

        if (useCap)
        {
            //This lane needs a cap-face manifold.
            Span<CandidateAos> candidates = stackalloc CandidateAos[12];
            int candidateCount = 0;

            //Project the edges down onto the cap's plane.
            var inverseLocalNormalY = 1f / localNormal.Y;
            var v01 = boxFaceCenter - boxFaceXOffset;
            v01 = v01 + boxFaceYOffset;
            var v10 = boxFaceCenter + boxFaceXOffset;
            v10 = v10 - boxFaceYOffset;
            var p00 = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v00);
            var p01 = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v01);
            var p10 = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v10);
            var p11 = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, v11);
            //Note that winding is important; we'll be choosing contacts based on the intervals.
            var edge0010 = p10 - p00;
            var edge1011 = p11 - p10;
            var edge1101 = p01 - p11;
            var edge0100 = p00 - p01;
            IntersectLineCircle(p00, edge0010, b.Radius, out var tMin0010, out var tMax0010, out var intersected0010);
            IntersectLineCircle(p01, edge0100, b.Radius, out var tMin0100, out var tMax0100, out var intersected0100);
            IntersectLineCircle(p10, edge1011, b.Radius, out var tMin1011, out var tMax1011, out var intersected1011);
            IntersectLineCircle(p11, edge1101, b.Radius, out var tMin1101, out var tMax1101, out var intersected1101);

            tMin0010 = MathF.Min(MathF.Max(tMin0010, 0f), 1f);
            tMax0010 = MathF.Min(MathF.Max(tMax0010, 0f), 1f);
            tMin1101 = MathF.Min(MathF.Max(tMin1101, 0f), 1f);
            tMax1101 = MathF.Min(MathF.Max(tMax1101, 0f), 1f);
            tMin0100 = MathF.Min(MathF.Max(tMin0100, 0f), 1f);
            tMax0100 = MathF.Min(MathF.Max(tMax0100, 0f), 1f);
            tMin1011 = MathF.Min(MathF.Max(tMin1011, 0f), 1f);
            tMax1011 = MathF.Min(MathF.Max(tMax1011, 0f), 1f);

            AddCandidateForEdge(p00, edge0010, tMin0010, tMax0010, intersected0010, 0, candidates, ref candidateCount);
            AddCandidateForEdge(p01, edge0100, tMin0100, tMax0100, intersected0100, 1, candidates, ref candidateCount);
            AddCandidateForEdge(p10, edge1011, tMin1011, tMax1011, intersected1011, 2, candidates, ref candidateCount);
            AddCandidateForEdge(p11, edge1101, tMin1101, tMax1101, intersected1101, 3, candidates, ref candidateCount);

            GenerateInteriorPoints(b, localNormal, closestOnB, out var interior0, out var interior1, out var interior2, out var interior3);

            //Test the four points against the edge plane. Note that signs depend on the orientation of the cylinder.
            var edge0010Plane0 = p00.X * edge0010.Y - p00.Y * edge0010.X;
            var edge0010Plane1 = p01.X * edge0010.Y - p01.Y * edge0010.X;
            var edge1011Plane0 = p10.X * edge1011.Y - p10.Y * edge1011.X;
            var edge1011Plane1 = p00.X * edge1011.Y - p00.Y * edge1011.X;
            var edge0010PlaneMin = MathF.Min(edge0010Plane0, edge0010Plane1);
            var edge0010PlaneMax = MathF.Max(edge0010Plane0, edge0010Plane1);
            var edge1011PlaneMin = MathF.Min(edge1011Plane0, edge1011Plane1);
            var edge1011PlaneMax = MathF.Max(edge1011Plane0, edge1011Plane1);
            TryAddInteriorPoint(interior0, 8, edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, candidates, ref candidateCount);
            TryAddInteriorPoint(interior1, 9, edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, candidates, ref candidateCount);
            TryAddInteriorPoint(interior2, 10, edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, candidates, ref candidateCount);
            TryAddInteriorPoint(interior3, 11, edge0010, edge0010PlaneMin, edge0010PlaneMax, edge1011, edge1011PlaneMin, edge1011PlaneMax, candidates, ref candidateCount);

            var capCenterToBoxFaceCenter = new Vector3(boxFaceCenter.X, boxFaceCenter.Y - capCenterBY, boxFaceCenter.Z);
            var tangentBX = new Vector3(1f, 0f, 0f);
            var tangentBY = new Vector3(0f, 0f, 1f);
            Reduce(candidates, candidateCount, boxFaceNormal, inverseFaceNormalDotLocalNormal, capCenterToBoxFaceCenter, tangentBX, tangentBY, epsilonScale, depthThreshold,
                out var candidate0, out var candidate1, out var candidate2, out var candidate3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            Vector3 localContact;
            localContact.X = candidate0.X;
            localContact.Y = capCenterBY;
            localContact.Z = candidate0.Y;
            var aToLocalContact = localContact + localOffsetB;
            Matrix3x3.Transform(aToLocalContact, worldRB, out manifold.OffsetA0);
            localContact.X = candidate1.X;
            localContact.Z = candidate1.Y;
            aToLocalContact = localContact + localOffsetB;
            Matrix3x3.Transform(aToLocalContact, worldRB, out manifold.OffsetA1);
            localContact.X = candidate2.X;
            localContact.Z = candidate2.Y;
            aToLocalContact = localContact + localOffsetB;
            Matrix3x3.Transform(aToLocalContact, worldRB, out manifold.OffsetA2);
            localContact.X = candidate3.X;
            localContact.Z = candidate3.Y;
            aToLocalContact = localContact + localOffsetB;
            Matrix3x3.Transform(aToLocalContact, worldRB, out manifold.OffsetA3);
            manifold.FeatureId0 = candidate0.FeatureId;
            manifold.FeatureId1 = candidate1.FeatureId;
            manifold.FeatureId2 = candidate2.FeatureId;
            manifold.FeatureId3 = candidate3.FeatureId;
            manifold.Depth0 = candidate0.Depth;
            manifold.Depth1 = candidate1.Depth;
            manifold.Depth2 = candidate2.Depth;
            manifold.Depth3 = candidate3.Depth;
        }
        else
        {
            //This lane needs a side-face manifold.
            //Intersect the single edge of B with the edge planes of face A.
            //Note that the edge planes are skewed to follow the local normal. Equivalent to projecting the side edge onto face A.
            var edgeNormalX = Vector3.Cross(boxFaceX, localNormal); //Points up
            var edgeNormalY = Vector3.Cross(boxFaceY, localNormal); //Points right
            //Center of the side line is just (closestOnB.X, 0, closestOnB.Z), sideLineDirection is just (0, 1, 0).
            //t = dot(sideLineStart - pointOnFaceEdge, edgeNormal) / dot(sideLineDirection, edgeNormal)
            const float negativeOne = -1f;
            var xDenominator = negativeOne / edgeNormalX.Y;
            var yDenominator = negativeOne / edgeNormalY.Y;
            var edgeNormalXLengthSquared = ScalarMath.Dot(edgeNormalX, edgeNormalX);
            var edgeNormalYLengthSquared = ScalarMath.Dot(edgeNormalY, edgeNormalY);
            var inverseEdgeNormalXLengthSquared = 1f / edgeNormalXLengthSquared;
            var inverseEdgeNormalYLengthSquared = 1f / edgeNormalYLengthSquared;
            Vector3 v00ToSideLine, v11ToSideLine;
            v00ToSideLine.X = closestOnB.X - v00.X;
            v00ToSideLine.Y = -v00.Y;
            v00ToSideLine.Z = closestOnB.Z - v00.Z;
            v11ToSideLine.X = closestOnB.X - v11.X;
            v11ToSideLine.Y = -v11.Y;
            v11ToSideLine.Z = closestOnB.Z - v11.Z;

            var bottomNumerator = ScalarMath.Dot(edgeNormalX, v00ToSideLine);
            var leftNumerator = ScalarMath.Dot(edgeNormalY, v00ToSideLine);
            var topNumerator = ScalarMath.Dot(edgeNormalX, v11ToSideLine);
            var rightNumerator = ScalarMath.Dot(edgeNormalY, v11ToSideLine);
            var xInvalid = ScalarMath.EqualMask(edgeNormalX.Y, 0f);
            var yInvalid = ScalarMath.EqualMask(edgeNormalY.Y, 0f);
            var tX0 = bottomNumerator * xDenominator;
            var tX1 = topNumerator * xDenominator;
            var tY0 = leftNumerator * yDenominator;
            var tY1 = rightNumerator * yDenominator;
            //As the side aligns with one of the edge directions, unrestrict that axis to avoid numerical noise; see the wide version.
            const float lowerThresholdAngle = 0.01f;
            const float upperThresholdAngle = 0.02f;
            const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
            const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
            const float interpolationMin = upperThreshold;
            const float inverseInterpolationSpan = 1f / (upperThreshold - lowerThreshold);
            var unrestrictWeightX = MathF.Max(0f, MathF.Min(1f, (interpolationMin - edgeNormalX.Y * edgeNormalX.Y * inverseEdgeNormalXLengthSquared) * inverseInterpolationSpan));
            var unrestrictWeightY = MathF.Max(0f, MathF.Min(1f, (interpolationMin - edgeNormalY.Y * edgeNormalY.Y * inverseEdgeNormalYLengthSquared) * inverseInterpolationSpan));
            var regularWeightX = 1f - unrestrictWeightX;
            var regularWeightY = 1f - unrestrictWeightY;
            var negativeHalfLength = -b.HalfLength;
            var tXMin = ScalarMath.Select(xInvalid, float.MinValue, unrestrictWeightX * negativeHalfLength + regularWeightX * MathF.Min(tX0, tX1));
            var tXMax = ScalarMath.Select(xInvalid, float.MaxValue, unrestrictWeightX * b.HalfLength + regularWeightX * MathF.Max(tX0, tX1));
            var tYMin = ScalarMath.Select(yInvalid, float.MinValue, unrestrictWeightY * negativeHalfLength + regularWeightY * MathF.Min(tY0, tY1));
            var tYMax = ScalarMath.Select(yInvalid, float.MaxValue, unrestrictWeightY * b.HalfLength + regularWeightY * MathF.Max(tY0, tY1));
            //Shouldn't need to make contact generation conditional here. The closest points are guaranteed to be on these chosen features;
            //they might just be in the same spot. We do clamp for numerical reasons. Note the asymmetric clamp order, mirrored exactly.
            var tMax = MathF.Min(MathF.Max(negativeHalfLength, MathF.Min(tXMax, tYMax)), b.HalfLength);
            var tMin = MathF.Min(MathF.Max(negativeHalfLength, MathF.Max(tXMin, tYMin)), b.HalfLength);

            Vector3 localContact0, localContact1;
            localContact0.X = localContact1.X = closestOnB.X;
            localContact0.Y = tMin;
            localContact1.Y = tMax;
            localContact0.Z = localContact1.Z = closestOnB.Z;
            Matrix3x3.Transform(localContact0, worldRB, out var contact0);
            Matrix3x3.Transform(localContact1, worldRB, out var contact1);
            contact0 = contact0 + offsetB;
            contact1 = contact1 + offsetB;
            manifold.OffsetA0 = contact0;
            manifold.OffsetA1 = contact1;
            manifold.FeatureId0 = 0;
            manifold.FeatureId1 = 1;
            //depth = dot(pointOnFaceB - faceCenterA, faceNormalA) / dot(faceNormalA, normal)
            var boxFaceToContact0 = localContact0 - boxFaceCenter;
            var boxFaceToContact1 = localContact1 - boxFaceCenter;
            var contact0Dot = ScalarMath.Dot(boxFaceToContact0, boxFaceNormal);
            var contact1Dot = ScalarMath.Dot(boxFaceToContact1, boxFaceNormal);
            var depth0 = contact0Dot * inverseFaceNormalDotLocalNormal;
            var depth1 = contact1Dot * inverseFaceNormalDotLocalNormal;
            manifold.Depth0 = depth0;
            manifold.Depth1 = depth1;
            manifold.Contact0Exists = depth0 >= depthThreshold;
            manifold.Contact1Exists = depth1 >= depthThreshold & tMax > tMin;
            manifold.Contact2Exists = false;
            manifold.Contact3Exists = false;
        }

        Matrix3x3.Transform(localNormal, worldRB, out manifold.Normal);
    }
}
