using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

using TriangleCylinderDepthRefiner = ScalarDepthRefiner<Cylinder, CylinderSupportScalar, Triangle, PretransformedTriangleSupportScalar>;

/// <summary>
/// Scalar AoS port of TriangleCylinderTester, bitwise identical per lane. Works in the cylinder's (b's) local space like the
/// wide implementation; the triangle is pretransformed by rA and recentered on its centroid, and the refiner treats the
/// cylinder as its shape A and the pretransformed triangle as its shape B, matching the wide call. The triangle-face prepass
/// can skip the refiner entirely per lane (the wide code passes skip lanes as terminated lanes, freezing their state, so the
/// scalar branch is exact). Structure and comments deliberately track TriangleCylinderTester.
/// </summary>
public static class TriangleCylinderScalarTester
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Dot2(Vector2 a, Vector2 b) => a.X * b.X + a.Y * b.Y;

    /// <summary>
    /// Mirror of BoxCylinderTester.IntersectLineCircle (the wide variant this tester calls). Note that this is NOT the
    /// cylinder-cylinder or cylinder-hull variant: it guards the direction length with a 2e-38 max (MathF.Max mirrors
    /// Vector.Max's IEEE semantics), reports an intersected flag (d >= 0), and clamps the discriminant to zero instead of
    /// branching. Identical to the copy in BoxCylinderScalarTester (private in a frozen file, so duplicated here).
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
    /// Mirror of BoxCylinderTester.AddCandidateForEdge; the wide AddCandidate scatter reduces to a branchy append per lane.
    /// The allowFeatureContacts mask is the cap-lane gate, always true on this scalar path.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void AddCandidateForEdge(Vector2 edgeStart, Vector2 edgeOffset, float tMin, float tMax, bool intersected, int edgeId,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        //There's an edge id (0, 1, 2), and then a flag for min or max (0 or 4).
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
    /// componentwise mirroring is exact; MathF.Max/Min mirror Vector.Max/Min's IEEE semantics). Identical to the copies in
    /// CylinderConvexHullScalarTester and BoxCylinderScalarTester (private in frozen files, so duplicated here).
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
    /// Mirror of TriangleCylinderTester.TryAddInteriorPoint. Containment differs from BoxCylinderTester's interval variant:
    /// all three perp-dot signs must match (the sum of GreaterThan masks is 0 or -3), since edge plane signs depend on the
    /// cylinder's orientation. The allowContact mask is the useCapTriangleFace gate, always true on this scalar path.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void TryAddInteriorPoint(Vector2 point, int featureId,
        Vector2 projectedA, Vector2 projectedAB, Vector2 projectedB, Vector2 projectedBC, Vector2 projectedC, Vector2 projectedCA,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        var ap = point - projectedA;
        var bp = point - projectedB;
        var cp = point - projectedC;
        //Signs are dependent on the cylinder orientation. Rather than dealing with that, we just look for whether all three signs are the same.
        //If the point is outside the triangle, it can't be outside all three edges at the same time.
        var abDot = ap.X * projectedAB.Y - ap.Y * projectedAB.X;
        var bcDot = bp.X * projectedBC.Y - bp.Y * projectedBC.X;
        var caDot = cp.X * projectedCA.Y - cp.Y * projectedCA.X;
        var sum = (abDot > 0f ? -1 : 0) + (bcDot > 0f ? -1 : 0) + (caDot > 0f ? -1 : 0);
        if (sum == 0 | sum == -3)
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
    /// Mirror of CylinderPairTester.ProjectOntoCapB (the copies in CylinderPairScalarTester/BoxCylinderScalarTester are
    /// private in frozen files, so it is duplicated here).
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
    /// skips candidates that are masked out per lane anyway, so per-lane results are identical (this also covers the wide
    /// maximumCandidateCount of 6 vs 10 in this tester: a pure bundle loop-bound detail with no scalar counterpart).
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
        in Triangle a, in Cylinder b, float speculativeMargin,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 worldRA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 worldRB);
        //Work in b's local space.
        ScalarMath.MultiplyByTranspose(worldRA, worldRB, out var rA);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, worldRB);

        //Pretransform and recenter the triangle so that the DepthRefiner can work with it (and to avoid unnecessary transforms).
        Triangle triangle;
        Matrix3x3.Transform(a.A, rA, out triangle.A);
        Matrix3x3.Transform(a.B, rA, out triangle.B);
        Matrix3x3.Transform(a.C, rA, out triangle.C);
        var centroid = triangle.A + triangle.B;
        centroid = triangle.C + centroid;
        centroid = centroid * (1f / 3f);
        triangle.A = triangle.A - centroid;
        triangle.B = triangle.B - centroid;
        triangle.C = triangle.C - centroid;
        var localTriangleCenter = centroid - localOffsetB;

        var length = ScalarMath.Length(localTriangleCenter);
        var initialNormal = localTriangleCenter * (1f / length);
        if (length < 1e-10f)
        {
            initialNormal.X = 0f;
            initialNormal.Y = 1f;
            initialNormal.Z = 0f;
        }

        var triangleAB = triangle.B - triangle.A;
        var triangleBC = triangle.C - triangle.B;
        var triangleCA = triangle.A - triangle.C;
        //We'll be using B-local triangle vertices quite a bit, so cache them.
        var triangleA = triangle.A + localTriangleCenter;
        var triangleB = triangle.B + localTriangleCenter;
        var triangleC = triangle.C + localTriangleCenter;
        var triangleNormal = Vector3.Cross(triangleAB, triangleCA);
        var triangleNormalLength = ScalarMath.Length(triangleNormal);
        triangleNormal = triangleNormal * (1f / triangleNormalLength);

        //Check if the cylinder's position is within the triangle and below the triangle plane. If so, we can ignore it.
        var cylinderToTriangleDot = ScalarMath.Dot(triangleNormal, localTriangleCenter);
        var cylinderBelowPlane = cylinderToTriangleDot >= 0f;
        var edgePlaneAB = Vector3.Cross(triangleAB, triangleNormal);
        var edgePlaneBC = Vector3.Cross(triangleBC, triangleNormal);
        var edgePlaneCA = Vector3.Cross(triangleCA, triangleNormal);
        //Is the cylinder position within the triangle bounds?
        var abPlaneTest = ScalarMath.Dot(edgePlaneAB, triangleA);
        var bcPlaneTest = ScalarMath.Dot(edgePlaneBC, triangleB);
        var caPlaneTest = ScalarMath.Dot(edgePlaneCA, triangleC);
        var cylinderInsideTriangleEdgePlanes = abPlaneTest <= 0f & bcPlaneTest <= 0f & caPlaneTest <= 0f;
        var cylinderInsideAndBelowTriangle = cylinderInsideTriangleEdgePlanes & cylinderBelowPlane;

        //ComputeNondegenerateTriangleMask mirror: LengthSquared sums of squares are association-safe.
        var abLengthSquared = triangleAB.LengthSquared();
        var caLengthSquared = triangleCA.LengthSquared();
        var triangleEpsilonScale = MathF.Sqrt(float.MaxNative(abLengthSquared, caLengthSquared));
        var nondegenerate = triangleNormalLength > TriangleWide.DegenerateTriangleEpsilon * triangleEpsilonScale;
        //Note that degenerate triangles are ignored completely; they don't have a well defined normal.
        if (!nondegenerate | cylinderInsideAndBelowTriangle)
        {
            //No contacts generated.
            return;
        }

        //Create a simplex entry for the triangle face normal.
        var negatedTriangleNormal = -triangleNormal;
        var cylinderSupportAlongNegatedTriangleNormal = CylinderSupportScalar.ComputeLocalSupport(b, negatedTriangleNormal);
        var negatedTriangleNormalSupport = cylinderSupportAlongNegatedTriangleNormal - localTriangleCenter;
        var triangleFaceDepth = ScalarMath.Dot(negatedTriangleNormalSupport, negatedTriangleNormal);

        //Check if the extreme point on the cylinder is contained within the bounds of the triangle face. If it is, there is no need for a full depth refinement.
        var closestToA = triangleA - cylinderSupportAlongNegatedTriangleNormal;
        var closestToB = triangleB - cylinderSupportAlongNegatedTriangleNormal;
        var closestToC = triangleC - cylinderSupportAlongNegatedTriangleNormal;
        var extremeABPlaneTest = ScalarMath.Dot(edgePlaneAB, closestToA);
        var extremeBCPlaneTest = ScalarMath.Dot(edgePlaneBC, closestToB);
        var extremeCAPlaneTest = ScalarMath.Dot(edgePlaneCA, closestToC);
        var triangleNormalIsMinimal =
            (cylinderInsideTriangleEdgePlanes & !cylinderBelowPlane) &
            extremeABPlaneTest <= 0f &
            extremeBCPlaneTest <= 0f &
            extremeCAPlaneTest <= 0f;

        var depthThreshold = -speculativeMargin;
        var skipDepthRefine = triangleNormalIsMinimal;
        Vector3 localNormal, closestOnB;
        float depth;
        var epsilonScale = MathF.Max(b.HalfLength, b.Radius);
        if (!skipDepthRefine)
        {
            TriangleCylinderDepthRefiner.FindMinimumDepth(b, triangle, localTriangleCenter, rA, initialNormal, 1e-5f * epsilonScale, depthThreshold,
                out depth, out localNormal, out closestOnB);
        }
        else
        {
            //No depth refine ran; the extreme point prepass did everything we needed. Just use the initial normal.
            localNormal = negatedTriangleNormal;
            closestOnB = cylinderSupportAlongNegatedTriangleNormal;
            depth = triangleFaceDepth;
        }

        //If the cylinder is too far away or if it's on the backside of the triangle, don't generate any contacts.
        var faceNormalADotNormal = ScalarMath.Dot(triangleNormal, localNormal);
        if (faceNormalADotNormal > -TriangleWide.BackfaceNormalDotRejectionThreshold | depth < depthThreshold)
        {
            //No contacts generated.
            return;
        }

        //Swap over to an edge case if the normal is not face aligned. For other shapes we tend to take the closest feature regardless, but here we're favoring the face a bit more by choosing a lower threshold.
        var useTriangleEdgeCase = MathF.Abs(faceNormalADotNormal) < 0.2f;

        //We generate contacts according to the dominant features along the collision normal.
        var capCenterBY = ScalarMath.Select(ScalarMath.LessMask(localNormal.Y, 0f), -b.HalfLength, b.HalfLength);

        var useCap = MathF.Abs(localNormal.Y) > 0.70710678118f;

        Vector3 localOffsetB0 = default;
        Vector3 localOffsetB1 = default;
        Vector3 localOffsetB2 = default;
        Vector3 localOffsetB3 = default;
        if (useCap)
        {
            //This lane needs a cap-face manifold.
            Span<CandidateAos> candidates = stackalloc CandidateAos[10];
            int candidateCount = 0;

            //Project the edges down onto the cap's plane.
            var inverseLocalNormalY = 1f / localNormal.Y;
            var pA = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, triangleA);
            var pB = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, triangleB);
            var pC = ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, triangleC);
            var projectedAB = pB - pA;
            var projectedBC = pC - pB;
            var projectedCA = pA - pC;
            IntersectLineCircle(pA, projectedAB, b.Radius, out var tMinAB, out var tMaxAB, out var intersectedAB);
            IntersectLineCircle(pB, projectedBC, b.Radius, out var tMinBC, out var tMaxBC, out var intersectedBC);
            IntersectLineCircle(pC, projectedCA, b.Radius, out var tMinCA, out var tMaxCA, out var intersectedCA);

            tMinAB = MathF.Min(MathF.Max(tMinAB, 0f), 1f);
            tMaxAB = MathF.Min(MathF.Max(tMaxAB, 0f), 1f);
            tMinBC = MathF.Min(MathF.Max(tMinBC, 0f), 1f);
            tMaxBC = MathF.Min(MathF.Max(tMaxBC, 0f), 1f);
            tMinCA = MathF.Min(MathF.Max(tMinCA, 0f), 1f);
            tMaxCA = MathF.Min(MathF.Max(tMaxCA, 0f), 1f);

            //While we projected onto the cylinder cap to do the triangle edge intersections, we use the triangle to create the contact manifold.
            //Arbitrarily pick the triangle center as the origin of the tangent space, and AB as the x axis.
            var triangleABLength = ScalarMath.Length(triangleAB);
            var triangleTangentX = triangleAB * (1f / triangleABLength);
            var triangleTangentY = Vector3.Cross(triangleTangentX, triangleNormal);

            Vector2 tangentA, tangentB, tangentC;
            tangentA.X = ScalarMath.Dot(triangle.A, triangleTangentX);
            tangentA.Y = ScalarMath.Dot(triangle.A, triangleTangentY);
            tangentB.X = ScalarMath.Dot(triangle.B, triangleTangentX);
            tangentB.Y = ScalarMath.Dot(triangle.B, triangleTangentY);
            tangentC.X = ScalarMath.Dot(triangle.C, triangleTangentX);
            tangentC.Y = ScalarMath.Dot(triangle.C, triangleTangentY);

            var tangentAB = tangentB - tangentA;
            var tangentBC = tangentC - tangentB;
            var tangentCA = tangentA - tangentC;

            AddCandidateForEdge(tangentA, tangentAB, tMinAB, tMaxAB, intersectedAB, 0, candidates, ref candidateCount);
            AddCandidateForEdge(tangentB, tangentBC, tMinBC, tMaxBC, intersectedBC, 1, candidates, ref candidateCount);
            AddCandidateForEdge(tangentC, tangentCA, tMinCA, tMaxCA, intersectedCA, 2, candidates, ref candidateCount);

            var useCapTriangleFace = !useTriangleEdgeCase;
            if (useCapTriangleFace)
            {
                //Project the points on the cylinder down to the triangle. Note that this is only valid if the normal is not perpendicular to the face normal.
                GenerateInteriorPoints(b, localNormal, closestOnB, out var interiorOnCylinder0, out var interiorOnCylinder1, out var interiorOnCylinder2, out var interiorOnCylinder3);
                //pointOnTrianglePlane = pointOnCylinder + localNormal * t
                //y = sign(localNormal.Y) * b.HalfLength
                //pointOnCylinder = (interiorOnCylinderN.X, y, interiorOnCylinderN.Y)
                //t = dot(localTriangleCenter - pointOnCylinder, triangleNormal) / dot(triangleNormal, localNormal)
                var inverseDenominator = -1f / faceNormalADotNormal;
                var yOffset = localTriangleCenter.Y - capCenterBY;
                var xOffset0 = localTriangleCenter.X - interiorOnCylinder0.X;
                var zOffset0 = localTriangleCenter.Z - interiorOnCylinder0.Y;
                var xOffset1 = localTriangleCenter.X - interiorOnCylinder1.X;
                var zOffset1 = localTriangleCenter.Z - interiorOnCylinder1.Y;
                var xOffset2 = localTriangleCenter.X - interiorOnCylinder2.X;
                var zOffset2 = localTriangleCenter.Z - interiorOnCylinder2.Y;
                var xOffset3 = localTriangleCenter.X - interiorOnCylinder3.X;
                var zOffset3 = localTriangleCenter.Z - interiorOnCylinder3.Y;
                var t0 = (xOffset0 * localNormal.X + yOffset * localNormal.Y + zOffset0 * localNormal.Z) * inverseDenominator;
                var t1 = (xOffset1 * localNormal.X + yOffset * localNormal.Y + zOffset1 * localNormal.Z) * inverseDenominator;
                var t2 = (xOffset2 * localNormal.X + yOffset * localNormal.Y + zOffset2 * localNormal.Z) * inverseDenominator;
                var t3 = (xOffset3 * localNormal.X + yOffset * localNormal.Y + zOffset3 * localNormal.Z) * inverseDenominator;
                //Projecting into the triangle's *tangent space* directly.
                //pointInTriangleTangentSpace = (dot(pointOnCylinder + localNormal * t, tangentX), dot(pointOnCylinder + localNormal * t, tangentY))
                Vector2 tangentLocalNormal;
                tangentLocalNormal.X = ScalarMath.Dot(localNormal, triangleTangentX);
                tangentLocalNormal.Y = ScalarMath.Dot(localNormal, triangleTangentY);
                Vector2 interior0, interior1, interior2, interior3;
                var yOnTangentX = yOffset * triangleTangentX.Y;
                var yOnTangentY = yOffset * triangleTangentY.Y;
                interior0.X = tangentLocalNormal.X * t0 - xOffset0 * triangleTangentX.X - yOnTangentX - zOffset0 * triangleTangentX.Z;
                interior0.Y = tangentLocalNormal.Y * t0 - xOffset0 * triangleTangentY.X - yOnTangentY - zOffset0 * triangleTangentY.Z;
                interior1.X = tangentLocalNormal.X * t1 - xOffset1 * triangleTangentX.X - yOnTangentX - zOffset1 * triangleTangentX.Z;
                interior1.Y = tangentLocalNormal.Y * t1 - xOffset1 * triangleTangentY.X - yOnTangentY - zOffset1 * triangleTangentY.Z;
                interior2.X = tangentLocalNormal.X * t2 - xOffset2 * triangleTangentX.X - yOnTangentX - zOffset2 * triangleTangentX.Z;
                interior2.Y = tangentLocalNormal.Y * t2 - xOffset2 * triangleTangentY.X - yOnTangentY - zOffset2 * triangleTangentY.Z;
                interior3.X = tangentLocalNormal.X * t3 - xOffset3 * triangleTangentX.X - yOnTangentX - zOffset3 * triangleTangentX.Z;
                interior3.Y = tangentLocalNormal.Y * t3 - xOffset3 * triangleTangentY.X - yOnTangentY - zOffset3 * triangleTangentY.Z;

                //Test the four points against the edge planes. Note that signs depend on the orientation of the cylinder.
                TryAddInteriorPoint(interior0, 8, tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, candidates, ref candidateCount);
                TryAddInteriorPoint(interior1, 9, tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, candidates, ref candidateCount);
                TryAddInteriorPoint(interior2, 10, tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, candidates, ref candidateCount);
                TryAddInteriorPoint(interior3, 11, tangentA, tangentAB, tangentB, tangentBC, tangentC, tangentCA, candidates, ref candidateCount);
            }

            Vector3 capNormal;
            capNormal.X = 0f;
            capNormal.Y = ScalarMath.Select(ScalarMath.LessMask(localNormal.Y, 0f), 1f, -1f);
            capNormal.Z = 0f;
            Vector3 triangleCenterToCapCenter;
            triangleCenterToCapCenter.X = -localTriangleCenter.X;
            triangleCenterToCapCenter.Y = capCenterBY - localTriangleCenter.Y;
            triangleCenterToCapCenter.Z = -localTriangleCenter.Z;
            Reduce(candidates, candidateCount, capNormal, -capNormal.Y / localNormal.Y, triangleCenterToCapCenter, triangleTangentX, triangleTangentY, epsilonScale, depthThreshold,
                out var candidate0, out var candidate1, out var candidate2, out var candidate3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            localOffsetB0.X = triangleTangentX.X * candidate0.X + triangleTangentY.X * candidate0.Y + localTriangleCenter.X;
            localOffsetB0.Y = triangleTangentX.Y * candidate0.X + triangleTangentY.Y * candidate0.Y + localTriangleCenter.Y;
            localOffsetB0.Z = triangleTangentX.Z * candidate0.X + triangleTangentY.Z * candidate0.Y + localTriangleCenter.Z;
            localOffsetB1.X = triangleTangentX.X * candidate1.X + triangleTangentY.X * candidate1.Y + localTriangleCenter.X;
            localOffsetB1.Y = triangleTangentX.Y * candidate1.X + triangleTangentY.Y * candidate1.Y + localTriangleCenter.Y;
            localOffsetB1.Z = triangleTangentX.Z * candidate1.X + triangleTangentY.Z * candidate1.Y + localTriangleCenter.Z;
            localOffsetB2.X = triangleTangentX.X * candidate2.X + triangleTangentY.X * candidate2.Y + localTriangleCenter.X;
            localOffsetB2.Y = triangleTangentX.Y * candidate2.X + triangleTangentY.Y * candidate2.Y + localTriangleCenter.Y;
            localOffsetB2.Z = triangleTangentX.Z * candidate2.X + triangleTangentY.Z * candidate2.Y + localTriangleCenter.Z;
            localOffsetB3.X = triangleTangentX.X * candidate3.X + triangleTangentY.X * candidate3.Y + localTriangleCenter.X;
            localOffsetB3.Y = triangleTangentX.Y * candidate3.X + triangleTangentY.Y * candidate3.Y + localTriangleCenter.Y;
            localOffsetB3.Z = triangleTangentX.Z * candidate3.X + triangleTangentY.Z * candidate3.Y + localTriangleCenter.Z;

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
            //This lane uses the cylinder side.
            var useSideEdgeCase = useTriangleEdgeCase;
            float depthTMin, depthTMax, cylinderTMin, cylinderTMax;

            //Identify the dominant edge based on alignment with the local normal.
            var abEdgeAlignment = ScalarMath.Dot(edgePlaneAB, localNormal);
            var bcEdgeAlignment = ScalarMath.Dot(edgePlaneBC, localNormal);
            var caEdgeAlignment = ScalarMath.Dot(edgePlaneCA, localNormal);

            var max = MathF.Max(abEdgeAlignment, MathF.Max(bcEdgeAlignment, caEdgeAlignment));
            var abIsDominant = max == abEdgeAlignment;
            var bcIsDominant = max == bcEdgeAlignment;

            var dominantEdgeStart = abIsDominant ? triangleA : triangleC;
            dominantEdgeStart = bcIsDominant ? triangleB : dominantEdgeStart;
            var dominantEdgeOffset = abIsDominant ? triangleAB : triangleCA;
            dominantEdgeOffset = bcIsDominant ? triangleBC : dominantEdgeOffset;

            //If the contacts are near an edge and the cylinder side is aligned with that edge, expand the contact interval; see the wide implementation's comments.
            const float lowerSinAngleThreshold = 0.01f;
            const float upperSinAngleThreshold = 0.02f;
            var dominantEdgeDotHorizontalNormal = dominantEdgeOffset.Z * localNormal.X - dominantEdgeOffset.X * localNormal.Z;
            var dominantEdgeDotHorizontalNormalSquared = dominantEdgeDotHorizontalNormal * dominantEdgeDotHorizontalNormal;
            var dominantEdgeLengthSquared = dominantEdgeOffset.LengthSquared();
            var horizontalNormalLengthSquared = localNormal.X * localNormal.X + localNormal.Z * localNormal.Z;
            var interpolationScale = dominantEdgeLengthSquared * horizontalNormalLengthSquared;
            const float interpolationMin = lowerSinAngleThreshold * lowerSinAngleThreshold;
            const float inverseInterpolationSpan = 1f / (upperSinAngleThreshold * upperSinAngleThreshold - lowerSinAngleThreshold * lowerSinAngleThreshold);
            var restrictWeight = MathF.Max(0f, MathF.Min(1f, (dominantEdgeDotHorizontalNormalSquared / interpolationScale - interpolationMin) * inverseInterpolationSpan));

            //Either triangle edge-cylinder side, or triangle face-cylinder side.
            if (useSideEdgeCase)
            {
                //Test the dominant edge of the triangle versus the plane formed by the cylinder side edge and the local normal.
                //(0,1,0) x localNormal = (-localNormal.Z, 0, localNormal.X)
                var cylinderEdgeToDominantEdgeStartX = dominantEdgeStart.X - closestOnB.X;
                var cylinderEdgeToDominantEdgeStartZ = dominantEdgeStart.Z - closestOnB.Z;
                var numerator = cylinderEdgeToDominantEdgeStartX * localNormal.Z - cylinderEdgeToDominantEdgeStartZ * localNormal.X;
                var edgeT = numerator / dominantEdgeDotHorizontalNormal;

                //As the unrestrict weight increases, expand the interval on the triangle edge until it covers the entire cylinder edge.
                var inverseEdgeOffsetLengthSquared = 1f / dominantEdgeLengthSquared;
                var tCenter = -(cylinderEdgeToDominantEdgeStartX * dominantEdgeOffset.X + dominantEdgeStart.Y * dominantEdgeOffset.Y + cylinderEdgeToDominantEdgeStartZ * dominantEdgeOffset.Z) * inverseEdgeOffsetLengthSquared;
                var projectedExtentOffset = b.HalfLength * MathF.Abs(dominantEdgeOffset.Y) * inverseEdgeOffsetLengthSquared;
                cylinderTMin = tCenter - projectedExtentOffset;
                cylinderTMax = tCenter + projectedExtentOffset;
                //Note that the edgeT value is ignored once the denominator is small enough. Avoids division by zero propagation.
                var regularContribution = restrictWeight * (dominantEdgeDotHorizontalNormalSquared < interpolationMin ? tCenter : edgeT);
                var unrestrictWeight = 1f - restrictWeight;
                cylinderTMin = regularContribution + unrestrictWeight * cylinderTMin;
                cylinderTMax = regularContribution + unrestrictWeight * cylinderTMax;

                cylinderTMin = MathF.Min(MathF.Max(0f, cylinderTMin), 1f);
                cylinderTMax = MathF.Min(MathF.Max(0f, cylinderTMax), 1f);

                //Compute depth by projecting back to the cylinder plane.
                var inverseDepthDenominator = -1f / horizontalNormalLengthSquared;
                var depthBase = (cylinderEdgeToDominantEdgeStartX * localNormal.X + cylinderEdgeToDominantEdgeStartZ * localNormal.Z) * inverseDepthDenominator;
                var tDepthScale = (dominantEdgeOffset.X * localNormal.X + dominantEdgeOffset.Z * localNormal.Z) * inverseDepthDenominator;
                depthTMin = depthBase + tDepthScale * cylinderTMin;
                depthTMax = depthBase + tDepthScale * cylinderTMax;

                var minOffset = dominantEdgeOffset * cylinderTMin;
                var maxOffset = dominantEdgeOffset * cylinderTMax;
                localOffsetB0.X = dominantEdgeStart.X + minOffset.X;
                localOffsetB0.Y = dominantEdgeStart.Y + minOffset.Y;
                localOffsetB0.Z = dominantEdgeStart.Z + minOffset.Z;
                localOffsetB1.X = dominantEdgeStart.X + maxOffset.X;
                localOffsetB1.Y = dominantEdgeStart.Y + maxOffset.Y;
                localOffsetB1.Z = dominantEdgeStart.Z + maxOffset.Z;
            }
            else
            {
                //Triangle face-cylinder side.
                //Project the cylinder edge down onto the triangle plane. We know it's numerically valid because useTriangleEdgeCase is false for this lane.
                var inverseDenominator = 1f / faceNormalADotNormal;
                var xzContribution = (localTriangleCenter.X - closestOnB.X) * triangleNormal.X + (localTriangleCenter.Z - closestOnB.Z) * triangleNormal.Z;
                var tMinToTriangle = (xzContribution + (localTriangleCenter.Y + b.HalfLength) * triangleNormal.Y) * inverseDenominator;
                var tMaxToTriangle = (xzContribution + (localTriangleCenter.Y - b.HalfLength) * triangleNormal.Y) * inverseDenominator;
                Vector3 minOnTriangle, maxOnTriangle;
                minOnTriangle.X = tMinToTriangle * localNormal.X + closestOnB.X;
                minOnTriangle.Y = tMinToTriangle * localNormal.Y - b.HalfLength;
                minOnTriangle.Z = tMinToTriangle * localNormal.Z + closestOnB.Z;
                maxOnTriangle.X = tMaxToTriangle * localNormal.X + closestOnB.X;
                maxOnTriangle.Y = tMaxToTriangle * localNormal.Y + b.HalfLength;
                maxOnTriangle.Z = tMaxToTriangle * localNormal.Z + closestOnB.Z;
                var minToMax = maxOnTriangle - minOnTriangle;
                //We now have points on the surface of the triangle. Use them as a ray to intersect the triangle's edge planes.
                var numeratorAB = (triangleA.X - minOnTriangle.X) * edgePlaneAB.X + (triangleA.Y - minOnTriangle.Y) * edgePlaneAB.Y + (triangleA.Z - minOnTriangle.Z) * edgePlaneAB.Z;
                var numeratorBC = (triangleB.X - minOnTriangle.X) * edgePlaneBC.X + (triangleB.Y - minOnTriangle.Y) * edgePlaneBC.Y + (triangleB.Z - minOnTriangle.Z) * edgePlaneBC.Z;
                var numeratorCA = (triangleC.X - minOnTriangle.X) * edgePlaneCA.X + (triangleC.Y - minOnTriangle.Y) * edgePlaneCA.Y + (triangleC.Z - minOnTriangle.Z) * edgePlaneCA.Z;
                var denominatorAB = minToMax.X * edgePlaneAB.X + minToMax.Y * edgePlaneAB.Y + minToMax.Z * edgePlaneAB.Z;
                var denominatorBC = minToMax.X * edgePlaneBC.X + minToMax.Y * edgePlaneBC.Y + minToMax.Z * edgePlaneBC.Z;
                var denominatorCA = minToMax.X * edgePlaneCA.X + minToMax.Y * edgePlaneCA.Y + minToMax.Z * edgePlaneCA.Z;
                //Protect against division by zero. This preserves sign and allows values to go to relatively enormous values.
                const float threshold = 1e-30f;
                const float negativeThreshold = -threshold;
                //A ray is 'exiting' if the ray's direction is aligned with the edge normal, entering otherwise.
                var exitingAB = denominatorAB <= 0f;
                var exitingBC = denominatorBC <= 0f;
                var exitingCA = denominatorCA <= 0f;
                denominatorAB = MathF.Abs(denominatorAB) < threshold ? (exitingAB ? negativeThreshold : threshold) : denominatorAB;
                denominatorBC = MathF.Abs(denominatorBC) < threshold ? (exitingBC ? negativeThreshold : threshold) : denominatorBC;
                denominatorCA = MathF.Abs(denominatorCA) < threshold ? (exitingCA ? negativeThreshold : threshold) : denominatorCA;
                var edgeTAB = numeratorAB / denominatorAB;
                var edgeTBC = numeratorBC / denominatorBC;
                var edgeTCA = numeratorCA / denominatorCA;

                //Take the first exit and last entry to create the interval of intersection.
                var entryAB = exitingAB ? float.MinValue : edgeTAB;
                var entryBC = exitingBC ? float.MinValue : edgeTBC;
                var entryCA = exitingCA ? float.MinValue : edgeTCA;
                var exitAB = exitingAB ? edgeTAB : float.MaxValue;
                var exitBC = exitingBC ? edgeTBC : float.MaxValue;
                var exitCA = exitingCA ? edgeTCA : float.MaxValue;

                //If the dominant edge is parallel with the cylinder side edge, then unrestrict the interval.
                //Entry T values are unrestricted to 0, exit T values are unrestricted to 1.
                var caIsDominant = !abIsDominant & !bcIsDominant;
                entryAB = abIsDominant ? entryAB * restrictWeight : entryAB;
                entryBC = bcIsDominant ? entryBC * restrictWeight : entryBC;
                entryCA = caIsDominant ? entryCA * restrictWeight : entryCA;
                var unrestrictWeight = 1f - restrictWeight;
                exitAB = abIsDominant ? exitAB * restrictWeight + unrestrictWeight : exitAB;
                exitBC = bcIsDominant ? exitBC * restrictWeight + unrestrictWeight : exitBC;
                exitCA = caIsDominant ? exitCA * restrictWeight + unrestrictWeight : exitCA;

                var sideTriangleCylinderTMin = MathF.Max(entryAB, MathF.Max(entryBC, entryCA));
                var sideTriangleCylinderTMax = MathF.Min(exitAB, MathF.Min(exitBC, exitCA));

                //Numerical error can invert the interval; this will typically happen in a vertex case.
                //Choose the vertex by examining which edges contributed the intersections forming the bounds of the interval.
                var useVertexFallback = sideTriangleCylinderTMax < sideTriangleCylinderTMin;
                var abContributedBound = edgeTAB == sideTriangleCylinderTMin | edgeTAB == sideTriangleCylinderTMax;
                var bcContributedBound = edgeTBC == sideTriangleCylinderTMin | edgeTBC == sideTriangleCylinderTMax;
                var caContributedBound = edgeTCA == sideTriangleCylinderTMin | edgeTCA == sideTriangleCylinderTMax;
                var useA = caContributedBound & abContributedBound;
                var useB = abContributedBound & bcContributedBound;
                var vertexFallback = useA ? triangleA : triangleC;
                vertexFallback = useB ? triangleB : vertexFallback;

                //Bound the interval to the cylinder's extent.
                cylinderTMin = MathF.Max(0f, MathF.Min(1f, sideTriangleCylinderTMin));
                cylinderTMax = MathF.Max(0f, MathF.Min(1f, sideTriangleCylinderTMax));
                localOffsetB0.X = minOnTriangle.X + minToMax.X * cylinderTMin;
                localOffsetB0.Y = minOnTriangle.Y + minToMax.Y * cylinderTMin;
                localOffsetB0.Z = minOnTriangle.Z + minToMax.Z * cylinderTMin;
                localOffsetB1.X = minOnTriangle.X + minToMax.X * cylinderTMax;
                localOffsetB1.Y = minOnTriangle.Y + minToMax.Y * cylinderTMax;
                localOffsetB1.Z = minOnTriangle.Z + minToMax.Z * cylinderTMax;

                if (useVertexFallback)
                    localOffsetB0 = vertexFallback;

                //Ray cast back to the cylinder's side to compute the depth for the contact.
                var inverseDepthDenominator = 1f / (localNormal.X * localNormal.X + localNormal.Z * localNormal.Z);
                depthTMin = (localNormal.X * (closestOnB.X - localOffsetB0.X) + localNormal.Z * (closestOnB.Z - localOffsetB0.Z)) * inverseDepthDenominator;
                depthTMax = (localNormal.X * (closestOnB.X - localOffsetB1.X) + localNormal.Z * (closestOnB.Z - localOffsetB1.Z)) * inverseDepthDenominator;
            }
            manifold.FeatureId0 = 0;
            manifold.FeatureId1 = 1;
            manifold.Depth0 = depthTMin;
            manifold.Depth1 = depthTMax;
            manifold.Contact0Exists = depthTMin > depthThreshold;
            manifold.Contact1Exists = depthTMax > depthThreshold & cylinderTMax > cylinderTMin;
            manifold.Contact2Exists = false;
            manifold.Contact3Exists = false;
        }

        Matrix3x3.Transform(localNormal, worldRB, out manifold.Normal);

        var localOffsetA0 = localOffsetB0 + localOffsetB;
        var localOffsetA1 = localOffsetB1 + localOffsetB;
        var localOffsetA2 = localOffsetB2 + localOffsetB;
        var localOffsetA3 = localOffsetB3 + localOffsetB;
        Matrix3x3.Transform(localOffsetA0, worldRB, out manifold.OffsetA0);
        Matrix3x3.Transform(localOffsetA1, worldRB, out manifold.OffsetA1);
        Matrix3x3.Transform(localOffsetA2, worldRB, out manifold.OffsetA2);
        Matrix3x3.Transform(localOffsetA3, worldRB, out manifold.OffsetA3);

        //Mesh reductions also make use of a face contact flag in the feature id.
        if (faceNormalADotNormal < -MeshReduction.MinimumDotForFaceCollision)
            manifold.FeatureId0 += MeshReduction.FaceCollisionFlag;
    }
}
