using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

/// <summary>
/// Scalar AoS port of BoxTriangleTester, bitwise identical per lane to the wide implementation.
/// Structure and comments deliberately track BoxTriangleTester line by line; see that file for the algorithm's reasoning.
/// Conventions follow BoxPairScalarTester: by-value phase methods with no inlining attributes, ScalarMath ordered dots and
/// mask selects, float.MinNative/MaxNative for Vector.Min/Max mirrors, branchy candidate appends.
/// </summary>
public static class BoxTriangleScalarTester
{
    //Mirrors TestBoxEdgesAgainstTriangleEdge (and its inner helper TestBoxEdgeAgainstTriangleEdge) for one triangle edge.
    //The wide helper swizzles its out parameters per box axis; the swizzles are applied by hand here:
    //A.X block -> normal (0, eZ*il, -eY*il); A.Y block -> (eZ*il, 0, -eX*il); A.Z block -> (eY*il, -eX*il, 0).
    //Note that the running depth update is Vector.Min (MinNative), NOT a select on the compare mask: they differ at (+0, -0).
    static void TestBoxEdgesAgainstTriangleEdge(
        float halfWidth, float halfHeight, float halfLength,
        Vector3 edge, Vector3 triangleCenter, Vector3 vA, Vector3 vB, Vector3 vC,
        out float depth, out Vector3 localNormal)
    {
        var x2 = edge.X * edge.X;
        var y2 = edge.Y * edge.Y;
        var z2 = edge.Z * edge.Z;
        {
            //A.X x edge
            var nY = edge.Z;
            var nZ = -edge.Y;
            var calibrationDot = triangleCenter.Y * nY + triangleCenter.Z * nZ;
            var length = MathF.Sqrt(y2 + z2);
            var inverseLength = ScalarMath.Select(ScalarMath.LessMask(calibrationDot, 0f), 1f, -1f) / length;
            nY *= inverseLength;
            nZ *= inverseLength;
            var extremeA = MathF.Abs(nY) * halfHeight + MathF.Abs(nZ) * halfLength;
            var nVA = vA.Y * nY + vA.Z * nZ;
            var nVB = vB.Y * nY + vB.Z * nZ;
            var nVC = vC.Y * nY + vC.Z * nZ;
            var minB = float.MinNative(nVA, float.MinNative(nVB, nVC));
            var maxB = float.MaxNative(nVA, float.MaxNative(nVB, nVC));
            depth = float.MinNative(extremeA - minB, maxB + extremeA);
            depth = ScalarMath.Select(ScalarMath.LessMask(length, 1e-7f), float.MaxValue, depth);
            localNormal = new Vector3(0f, nY, nZ);
        }
        {
            //A.Y x edge; the wide code routes the helper's (X, Y, Z) outputs into (.Y, .X, .Z).
            var nX = edge.Z;
            var nZ = -edge.X;
            var calibrationDot = triangleCenter.X * nX + triangleCenter.Z * nZ;
            var length = MathF.Sqrt(x2 + z2);
            var inverseLength = ScalarMath.Select(ScalarMath.LessMask(calibrationDot, 0f), 1f, -1f) / length;
            nX *= inverseLength;
            nZ *= inverseLength;
            var extremeA = MathF.Abs(nX) * halfWidth + MathF.Abs(nZ) * halfLength;
            var nVA = vA.X * nX + vA.Z * nZ;
            var nVB = vB.X * nX + vB.Z * nZ;
            var nVC = vC.X * nX + vC.Z * nZ;
            var minB = float.MinNative(nVA, float.MinNative(nVB, nVC));
            var maxB = float.MaxNative(nVA, float.MaxNative(nVB, nVC));
            var d = float.MinNative(extremeA - minB, maxB + extremeA);
            d = ScalarMath.Select(ScalarMath.LessMask(length, 1e-7f), float.MaxValue, d);
            var useCandidate = ScalarMath.LessMask(d, depth);
            localNormal = ScalarMath.Select(useCandidate, new Vector3(nX, 0f, nZ), localNormal);
            depth = float.MinNative(depth, d);
        }
        {
            //A.Z x edge; helper outputs routed into (.Z, .X, .Y).
            var nX = edge.Y;
            var nY = -edge.X;
            var calibrationDot = triangleCenter.X * nX + triangleCenter.Y * nY;
            var length = MathF.Sqrt(x2 + y2);
            var inverseLength = ScalarMath.Select(ScalarMath.LessMask(calibrationDot, 0f), 1f, -1f) / length;
            nX *= inverseLength;
            nY *= inverseLength;
            var extremeA = MathF.Abs(nX) * halfWidth + MathF.Abs(nY) * halfHeight;
            var nVA = vA.X * nX + vA.Y * nY;
            var nVB = vB.X * nX + vB.Y * nY;
            var nVC = vC.X * nX + vC.Y * nY;
            var minB = float.MinNative(nVA, float.MinNative(nVB, nVC));
            var maxB = float.MaxNative(nVA, float.MaxNative(nVB, nVC));
            var d = float.MinNative(extremeA - minB, maxB + extremeA);
            d = ScalarMath.Select(ScalarMath.LessMask(length, 1e-7f), float.MaxValue, d);
            var useCandidate = ScalarMath.LessMask(d, depth);
            localNormal = ScalarMath.Select(useCandidate, new Vector3(nX, nY, 0f), localNormal);
            depth = float.MinNative(depth, d);
        }
    }

    //Mirrors both wide Select overloads: normal blends on the compare mask, depth takes the running Vector.Min.
    static void Select(ref float depth, ref Vector3 normal, float depthCandidate, Vector3 normalCandidate)
    {
        var useCandidate = ScalarMath.LessMask(depthCandidate, depth);
        normal = ScalarMath.Select(useCandidate, normalCandidate, normal);
        depth = float.MinNative(depth, depthCandidate);
    }

    /// <summary>
    /// SAT phases: 9 edge-edge axes, 3 box face axes, triangle face axis. The returned triangleNormal is the normalized
    /// UNCALIBRATED normal (the candidate fed to the select was the calibrated one; downstream uses the uncalibrated form).
    /// </summary>
    static void TestSat(Box a, Vector3 vA, Vector3 vB, Vector3 vC, Vector3 localTriangleCenter, Vector3 ab, Vector3 bc, Vector3 ca, float minimumDepth,
        out float depth, out Vector3 localNormal, out Vector3 triangleNormal, out float triangleNormalLength)
    {
        TestBoxEdgesAgainstTriangleEdge(a.HalfWidth, a.HalfHeight, a.HalfLength, ab, localTriangleCenter, vA, vB, vC, out depth, out localNormal);
        TestBoxEdgesAgainstTriangleEdge(a.HalfWidth, a.HalfHeight, a.HalfLength, bc, localTriangleCenter, vA, vB, vC, out var depthCandidate, out var localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
        TestBoxEdgesAgainstTriangleEdge(a.HalfWidth, a.HalfHeight, a.HalfLength, ca, localTriangleCenter, vA, vB, vC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

        //The SAT depth is a running minimum: once strictly below the speculative margin the reject decision is final, and every
        //remaining output is unobservable on the rejection path (the caller's allowContacts conjunction is false regardless of
        //the normal/degeneracy terms), so bailing here is bitwise-output-identical. NaN depths compare false and fall through.
        //(The wide path can only skip when the whole bundle misses — this is the structural per-pair AoS early-out advantage.)
        if (depth < minimumDepth)
        {
            triangleNormal = default;
            triangleNormalLength = 0f;
            return;
        }

        //Test face normals of A. Working in local space of A means potential axes are just (1,0,0) etc.
        var xNormalSign = ScalarMath.Select(ScalarMath.LessMask(localTriangleCenter.X, 0f), 1f, -1f);
        var yNormalSign = ScalarMath.Select(ScalarMath.LessMask(localTriangleCenter.Y, 0f), 1f, -1f);
        var zNormalSign = ScalarMath.Select(ScalarMath.LessMask(localTriangleCenter.Z, 0f), 1f, -1f);

        //GetDepthForInterval per axis: depth = Min(boxExtreme - minB, maxB + boxExtreme).
        var minBX = float.MinNative(vA.X, float.MinNative(vB.X, vC.X));
        var maxBX = float.MaxNative(vA.X, float.MaxNative(vB.X, vC.X));
        var faceAXDepth = float.MinNative(a.HalfWidth - minBX, maxBX + a.HalfWidth);
        var minBY = float.MinNative(vA.Y, float.MinNative(vB.Y, vC.Y));
        var maxBY = float.MaxNative(vA.Y, float.MaxNative(vB.Y, vC.Y));
        var faceAYDepth = float.MinNative(a.HalfHeight - minBY, maxBY + a.HalfHeight);
        var minBZ = float.MinNative(vA.Z, float.MinNative(vB.Z, vC.Z));
        var maxBZ = float.MaxNative(vA.Z, float.MaxNative(vB.Z, vC.Z));
        var faceAZDepth = float.MinNative(a.HalfLength - minBZ, maxBZ + a.HalfLength);
        Select(ref depth, ref localNormal, faceAXDepth, new Vector3(xNormalSign, 0f, 0f));
        Select(ref depth, ref localNormal, faceAYDepth, new Vector3(0f, yNormalSign, 0f));
        Select(ref depth, ref localNormal, faceAZDepth, new Vector3(0f, 0f, zNormalSign));
        if (depth < minimumDepth)
        {
            triangleNormal = default;
            triangleNormalLength = 0f;
            return;
        }

        //Test face normal of B.
        triangleNormal = Vector3.Cross(ab, ca);
        triangleNormalLength = ScalarMath.Length(triangleNormal);
        triangleNormal = triangleNormal * (1f / triangleNormalLength);
        //Calibrate the normal to point from B to A for the candidate; the face depth uses the uncalibrated normal.
        var trianglePlaneOffset = ScalarMath.Dot(triangleNormal, localTriangleCenter);
        var negatedTriangleNormal = -triangleNormal;
        var calibratedTriangleNormal = ScalarMath.Select(ScalarMath.GreaterMask(trianglePlaneOffset, 0f), negatedTriangleNormal, triangleNormal);
        var triangleFaceDepth =
            MathF.Abs(triangleNormal.X) * a.HalfWidth + MathF.Abs(triangleNormal.Y) * a.HalfHeight + MathF.Abs(triangleNormal.Z) * a.HalfLength - MathF.Abs(trianglePlaneOffset);
        Select(ref depth, ref localNormal, triangleFaceDepth, calibratedTriangleNormal);
    }

    //Mirrors BoxTriangleTester.Add: candidate X/Y are ordered dots on the triangle tangent plane; append is branchy but the
    //candidate values are computed unconditionally, matching the wide masked-store semantics (unstored values are unobservable).
    static void Add(Vector3 pointOnTriangle, Vector3 triangleCenter, Vector3 triangleTangentX, Vector3 triangleTangentY, int featureId,
        bool exists, Span<CandidateAos> candidates, ref int candidateCount)
    {
        var offset = pointOnTriangle - triangleCenter;
        Unsafe.SkipInit(out CandidateAos candidate);
        candidate.X = ScalarMath.Dot(offset, triangleTangentX);
        candidate.Y = ScalarMath.Dot(offset, triangleTangentY);
        candidate.FeatureId = featureId;
        if (exists)
        {
            Unsafe.Add(ref MemoryMarshal.GetReference(candidates), candidateCount) = candidate;
            ++candidateCount;
        }
    }

    static void ClipTriangleEdgeAgainstPlanes(Vector3 edgeDirection, Vector3 triangleEdgeStartToBoxEdgeAnchor0, Vector3 triangleEdgeStartToBoxEdgeAnchor1,
        Vector3 boxEdgePlaneNormal, out float min, out float max)
    {
        var distance0 = ScalarMath.Dot(triangleEdgeStartToBoxEdgeAnchor0, boxEdgePlaneNormal);
        var distance1 = ScalarMath.Dot(triangleEdgeStartToBoxEdgeAnchor1, boxEdgePlaneNormal);
        var velocity = ScalarMath.Dot(boxEdgePlaneNormal, edgeDirection);
        var inverseVelocity = 1f / velocity;

        //If the distances to the planes have opposing signs, then the start must be between the two.
        var edgeStartIsInside = ScalarMath.LessOrEqualMask(distance0 * distance1, 0f);
        var dontUseFallback = ScalarMath.GreaterMask(MathF.Abs(velocity), 1e-15f);
        var t0 = distance0 * inverseVelocity;
        var t1 = distance1 * inverseVelocity;
        //If the edge direction and plane surface is parallel, then the interval is defined entirely by whether the edge starts inside or outside.
        const float largeNegative = -float.MaxValue;
        const float largePositive = float.MaxValue;
        min = ScalarMath.Select(dontUseFallback, float.MinNative(t0, t1), ScalarMath.Select(edgeStartIsInside, largeNegative, largePositive));
        max = ScalarMath.Select(dontUseFallback, float.MaxNative(t0, t1), ScalarMath.Select(edgeStartIsInside, largePositive, largeNegative));
    }

    static void ClipTriangleEdgeAgainstBoxFace(Vector3 edgeStart, Vector3 edgeDirection, int edgeId,
        Vector3 boxVertex00, Vector3 boxVertex11, Vector3 edgePlaneNormalX, Vector3 edgePlaneNormalY,
        Vector3 triangleCenter, Vector3 triangleTangentX, Vector3 triangleTangentY,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        var triangleEdgeStartToV00 = boxVertex00 - edgeStart;
        var triangleEdgeStartToV11 = boxVertex11 - edgeStart;
        ClipTriangleEdgeAgainstPlanes(edgeDirection, triangleEdgeStartToV00, triangleEdgeStartToV11, edgePlaneNormalX, out var minX, out var maxX);
        ClipTriangleEdgeAgainstPlanes(edgeDirection, triangleEdgeStartToV00, triangleEdgeStartToV11, edgePlaneNormalY, out var minY, out var maxY);
        var min = float.MaxNative(minX, minY);
        var max = float.MinNative(1f, float.MinNative(maxX, maxY));
        var minLocation = edgeStart + edgeDirection * min;
        var maxLocation = edgeStart + edgeDirection * max;

        //If 0<min<1 && (max-min)>epsilon for an edge, use the min intersection as a contact; the epsilon is fixed since t is normalized to the edge.
        //The capacity test against 6 must use the count BEFORE the min append for min, and AFTER it for max, matching the wide sequencing.
        var minExists = candidateCount < 6 & max - min >= 1e-5f & min < 1f & min > 0f;
        Add(minLocation, triangleCenter, triangleTangentX, triangleTangentY, edgeId, minExists, candidates, ref candidateCount);

        var maxExists = candidateCount < 6 & max >= min & max <= 1f & max >= 0f;
        Add(maxLocation, triangleCenter, triangleTangentX, triangleTangentY, edgeId + 8, maxExists, candidates, ref candidateCount);
    }

    static void ClipTriangleEdgesAgainstBoxFace(
        Vector3 vA, Vector3 vB, Vector3 vC, Vector3 triangleCenter, Vector3 triangleTangentX, Vector3 triangleTangentY,
        Vector3 ab, Vector3 bc, Vector3 ca,
        Vector3 boxVertex00, Vector3 boxVertex11, Vector3 boxTangentX, Vector3 boxTangentY, Vector3 contactNormal,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        //Clipping happens on the contact normal plane, so the edge plane normals embed the box edge while being perpendicular to the contact normal.
        var edgePlaneNormalX = Vector3.Cross(boxTangentY, contactNormal);
        var edgePlaneNormalY = Vector3.Cross(boxTangentX, contactNormal);

        //Edge feature ids: 4 + [0, 1, 2] per triangle edge; +8 for the max endpoint of an interval.
        ClipTriangleEdgeAgainstBoxFace(vA, ab, 4, boxVertex00, boxVertex11, edgePlaneNormalX, edgePlaneNormalY, triangleCenter, triangleTangentX, triangleTangentY, candidates, ref candidateCount);
        ClipTriangleEdgeAgainstBoxFace(vB, bc, 5, boxVertex00, boxVertex11, edgePlaneNormalX, edgePlaneNormalY, triangleCenter, triangleTangentX, triangleTangentY, candidates, ref candidateCount);
        ClipTriangleEdgeAgainstBoxFace(vC, ca, 6, boxVertex00, boxVertex11, edgePlaneNormalX, edgePlaneNormalY, triangleCenter, triangleTangentX, triangleTangentY, candidates, ref candidateCount);
    }

    static void AddBoxVertex(Vector3 a, Vector3 b, Vector3 v, Vector3 triangleNormal, Vector3 contactNormal, float inverseNormalDot,
        Vector3 abEdgePlaneNormal, Vector3 bcEdgePlaneNormal, Vector3 caEdgePlaneNormal,
        Vector3 triangleCenter, Vector3 triangleX, Vector3 triangleY, int featureId,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        //Cast a ray from the box vertex down to the triangle along the contact normal.
        var pointOnTriangleToBoxVertex = v - a;
        var planeDistance = ScalarMath.Dot(triangleNormal, pointOnTriangleToBoxVertex);
        var offset = contactNormal * (planeDistance * inverseNormalDot);
        //Contact normal points from triangle to box by convention, so we have to subtract.
        var vOnPlane = v - offset;

        //Test the unprojected location against the edge normals (which point inward).
        var aToV = vOnPlane - a;
        var bToV = vOnPlane - b;
        var abDot = ScalarMath.Dot(aToV, abEdgePlaneNormal);
        var bcDot = ScalarMath.Dot(bToV, bcEdgePlaneNormal);
        var caDot = ScalarMath.Dot(aToV, caEdgePlaneNormal);

        var contained = abDot >= 0f & bcDot >= 0f & caDot >= 0f;
        Add(vOnPlane, triangleCenter, triangleX, triangleY, featureId, contained, candidates, ref candidateCount);
    }

    static void AddBoxVertices(Vector3 a, Vector3 b, Vector3 ab, Vector3 bc, Vector3 ca, Vector3 triangleNormal, Vector3 contactNormal,
        Vector3 v00, Vector3 v01, Vector3 v10, Vector3 v11,
        Vector3 triangleCenter, Vector3 triangleX, Vector3 triangleY, int baseFeatureId, int featureIdX, int featureIdY,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        var normalDot = ScalarMath.Dot(triangleNormal, contactNormal);
        //Cases where the triangle normal faces away from the contact normal don't generate contacts anyway.
        var inverseNormalDot = MathF.Abs(normalDot) > 1e-10f ? 1f / normalDot : float.MaxValue;

        //Edge plane normals point inward for these tests.
        var abEdgePlaneNormal = Vector3.Cross(ab, triangleNormal);
        var bcEdgePlaneNormal = Vector3.Cross(bc, triangleNormal);
        var caEdgePlaneNormal = Vector3.Cross(ca, triangleNormal);

        //Insertion order (v00, v01, v10, v11) matters for Reduce's first-strict-winner tie-breaking; no capacity checks needed here
        //(the buffer holds 6 and this pass adds at most 4).
        AddBoxVertex(a, b, v00, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
            triangleCenter, triangleX, triangleY, baseFeatureId, candidates, ref candidateCount);
        AddBoxVertex(a, b, v01, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
            triangleCenter, triangleX, triangleY, baseFeatureId + featureIdY, candidates, ref candidateCount);
        AddBoxVertex(a, b, v10, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
            triangleCenter, triangleX, triangleY, baseFeatureId + featureIdX, candidates, ref candidateCount);
        AddBoxVertex(a, b, v11, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
            triangleCenter, triangleX, triangleY, baseFeatureId + featureIdX + featureIdY, candidates, ref candidateCount);
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

    static void TransformContactToManifold(CandidateAos rawContact, Vector3 faceCenterB, Vector3 tangentBX, Vector3 tangentBY,
        out Vector3 manifoldOffsetA, out float manifoldDepth, out int manifoldFeatureId)
    {
        manifoldOffsetA = tangentBX * rawContact.X;
        var y = tangentBY * rawContact.Y;
        manifoldOffsetA += y;
        manifoldOffsetA += faceCenterB;
        manifoldDepth = rawContact.Depth;
        manifoldFeatureId = rawContact.FeatureId;
    }

    public static void Test(
        in Box a, in Triangle b, float speculativeMargin,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        Unsafe.SkipInit(out manifold);
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 worldRA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 worldRB);
        ScalarMath.MultiplyByTranspose(worldRB, worldRA, out var rB);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, worldRA);
        Matrix3x3.Transform(b.A, rB, out var vA);
        vA += localOffsetB;
        Matrix3x3.Transform(b.B, rB, out var vB);
        vB += localOffsetB;
        Matrix3x3.Transform(b.C, rB, out var vC);
        vC += localOffsetB;

        var localTriangleCenter = (vA + vB + vC) * (1f / 3f);

        var ab = vB - vA;
        var bc = vC - vB;
        var ca = vA - vC;

        var minimumDepth = -speculativeMargin;
        TestSat(a, vA, vB, vC, localTriangleCenter, ab, bc, ca, minimumDepth, out var depth, out var localNormal, out var triangleNormal, out var triangleNormalLength);

        //If the local normal points against the triangle normal, then it's on the backside and should not collide.
        var normalDot = ScalarMath.Dot(localNormal, triangleNormal);
        var abLengthSquared = ScalarMath.Dot(ab, ab);
        var caLengthSquared = ScalarMath.Dot(ca, ca);
        //TriangleWide.ComputeNondegenerateTriangleMask.
        var triangleEpsilonScale = MathF.Sqrt(float.MaxNative(abLengthSquared, caLengthSquared));
        var nondegenerate = triangleNormalLength > TriangleWide.DegenerateTriangleEpsilon * triangleEpsilonScale;
        var allowContacts = nondegenerate & normalDot >= TriangleWide.BackfaceNormalDotRejectionThreshold & depth >= minimumDepth;
        if (!allowContacts)
        {
            //A per-lane wide rejection produces zero candidates, so every exists flag comes out false; nothing else is observable.
            manifold.Contact0Exists = false;
            manifold.Contact1Exists = false;
            manifold.Contact2Exists = false;
            manifold.Contact3Exists = false;
            return;
        }

        //Choose the box face whose axis has the minimum dot with the collision normal (normal is calibrated triangle->box).
        var absNX = MathF.Abs(localNormal.X);
        var absNY = MathF.Abs(localNormal.Y);
        var absNZ = MathF.Abs(localNormal.Z);
        var xBiggerThanY = absNX > absNY;
        var xBiggerThanZ = absNX > absNZ;
        var yBiggerThanZ = absNY > absNZ;
        var useAX = xBiggerThanY & xBiggerThanZ;
        //Vector.AndNot(a, b) = a & ~b.
        var useAY = yBiggerThanZ & !useAX;
        var useAZ = !(useAX | useAY);

        var normalIsNegativeX = localNormal.X < 0f;
        var normalIsNegativeY = localNormal.Y < 0f;
        var normalIsNegativeZ = localNormal.Z < 0f;

        //If we're using face with normal X, the tangent axes are Z and Y; normal Y -> X and Z; normal Z -> X and Y.
        var boxTangentX = new Vector3(useAY | useAZ ? 1f : 0f, 0f, useAX ? 1f : 0f);
        var boxTangentY = new Vector3(0f, useAX | useAZ ? 1f : 0f, useAY ? 1f : 0f);
        var boxFaceNormal = new Vector3(
            useAX ? normalIsNegativeX ? 1f : -1f : 0f,
            useAY ? normalIsNegativeY ? 1f : -1f : 0f,
            useAZ ? normalIsNegativeZ ? 1f : -1f : 0f);

        var halfExtentX = useAX ? a.HalfLength : a.HalfWidth;
        var halfExtentY = useAY ? a.HalfLength : a.HalfHeight;
        var halfExtentZ = useAX ? a.HalfWidth : useAY ? a.HalfHeight : a.HalfLength;
        var boxFaceCenter = boxFaceNormal * halfExtentZ;

        //Feature ids: box vertex contacts use axis ids (X=0, Y=1, Z=2); triangle edge contacts use 4 + edge index (+8 for max endpoints).
        var axisIdTangentX = useAX ? 2 : 0;
        var axisIdTangentY = useAY ? 2 : 1;
        //The wide code selects localXId (0) versus zero for the X case, which is a constant 0 either way.
        var axisIdNormal = useAX ? 0 : useAY ? normalIsNegativeY ? 1 : 0 : normalIsNegativeZ ? 2 : 0;

        //The minimum across the maxes avoids huge-size-disparity epsilons killing valid contacts.
        var epsilonScale = float.MinNative(
            float.MaxNative(a.HalfWidth, float.MaxNative(a.HalfHeight, a.HalfLength)),
            triangleEpsilonScale);

        //2d parameterization of the triangle surface for contact reduction.
        var triangleTangentX = ab * (1f / MathF.Sqrt(abLengthSquared));
        var triangleTangentY = Vector3.Cross(triangleTangentX, triangleNormal);

        //At most 6 candidates: up to 4 box vertices plus at most 2 per triangle edge, capacity-gated on the edge path.
        Span<CandidateAos> candidates = stackalloc CandidateAos[6];
        int candidateCount = 0;

        //Box vertex candidates first (so the vertex pass never needs capacity tests).
        var boxEdgeOffsetX = boxTangentX * halfExtentX;
        var boxEdgeOffsetY = boxTangentY * halfExtentY;
        var positiveX = boxFaceCenter + boxEdgeOffsetX;
        var negativeX = boxFaceCenter - boxEdgeOffsetX;
        var boxVertex00 = negativeX - boxEdgeOffsetY;
        var boxVertex01 = negativeX + boxEdgeOffsetY;
        var boxVertex10 = positiveX - boxEdgeOffsetY;
        var boxVertex11 = positiveX + boxEdgeOffsetY;
        AddBoxVertices(vA, vB, ab, bc, ca, triangleNormal, localNormal, boxVertex00, boxVertex01, boxVertex10, boxVertex11,
            localTriangleCenter, triangleTangentX, triangleTangentY, axisIdNormal, axisIdTangentX, axisIdTangentY, candidates, ref candidateCount);

        //Triangle edges clipped against the box face; also picks up triangle vertices within the face bounds.
        ClipTriangleEdgesAgainstBoxFace(vA, vB, vC, localTriangleCenter, triangleTangentX, triangleTangentY, ab, bc, ca,
            boxVertex00, boxVertex11, boxTangentX, boxTangentY, localNormal, candidates, ref candidateCount);

        var faceCenterBToFaceCenterA = boxFaceCenter - localTriangleCenter;
        var faceNormalDotNormal = ScalarMath.Dot(boxFaceNormal, localNormal);
        Reduce(candidates, candidateCount, boxFaceNormal, 1f / faceNormalDotNormal, faceCenterBToFaceCenterA, triangleTangentX, triangleTangentY, epsilonScale, minimumDepth,
            out var contact0, out var contact1, out var contact2, out var contact3,
            out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

        //Transform the contacts into the manifold; the basis moves into world rotation so individual contacts don't need transforming.
        Matrix3x3.Transform(triangleTangentX, worldRA, out var worldTangentBX);
        Matrix3x3.Transform(triangleTangentY, worldRA, out var worldTangentBY);
        Matrix3x3.Transform(localTriangleCenter, worldRA, out var worldTriangleCenter);
        Matrix3x3.Transform(localNormal, worldRA, out manifold.Normal);
        TransformContactToManifold(contact0, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA0, out manifold.Depth0, out manifold.FeatureId0);
        TransformContactToManifold(contact1, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA1, out manifold.Depth1, out manifold.FeatureId1);
        TransformContactToManifold(contact2, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA2, out manifold.Depth2, out manifold.FeatureId2);
        TransformContactToManifold(contact3, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA3, out manifold.Depth3, out manifold.FeatureId3);
        //Mark the manifold as a triangle face collision when the normal is close enough; added to contact 0's feature id unconditionally.
        var faceFlag = normalDot >= MeshReduction.MinimumDotForFaceCollision ? MeshReduction.FaceCollisionFlag : 0;
        manifold.FeatureId0 += faceFlag;
    }
}
