using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

/// <summary>
/// Scalar AoS port of TrianglePairTester, bitwise identical per lane to the wide implementation.
/// Structure and comments deliberately track TrianglePairTester line by line; see that file for the algorithm's reasoning.
/// Works in A's LOCAL space: rB = MultiplyByTranspose(worldRB, worldRA) — note the reversed argument order versus the
/// B-local-space testers — and the final basis transform uses worldRA.
/// Conventions follow BoxTriangleScalarTester: ScalarMath ordered dots and mask selects, MathF.Min/Max for the IEEE
/// Vector.Min/Max mirrors, branchy candidate appends carrying depth (AddCandidateWithDepth mirror).
/// </summary>
public static class TrianglePairScalarTester
{
    //Mirrors TrianglePairTester.GetIntervalForNormal.
    static void GetIntervalForNormal(Vector3 a, Vector3 b, Vector3 c, Vector3 normal, out float min, out float max)
    {
        var dA = ScalarMath.Dot(normal, a);
        var dB = ScalarMath.Dot(normal, b);
        var dC = ScalarMath.Dot(normal, c);
        min = MathF.Min(dA, MathF.Min(dB, dC));
        max = MathF.Max(dA, MathF.Max(dB, dC));
    }

    //Mirrors TrianglePairTester.GetDepthForNormal.
    static float GetDepthForNormal(Vector3 aA, Vector3 bA, Vector3 cA, Vector3 aB, Vector3 bB, Vector3 cB, Vector3 normal)
    {
        GetIntervalForNormal(aA, bA, cA, normal, out var minA, out var maxA);
        GetIntervalForNormal(aB, bB, cB, normal, out var minB, out var maxB);
        return MathF.Min(maxA - minB, maxB - minA);
    }

    //Mirrors TrianglePairTester.TestEdgeEdge. The normal is normalized but not calibrated here.
    static void TestEdgeEdge(
        Vector3 edgeDirectionA, Vector3 edgeDirectionB,
        Vector3 aA, Vector3 bA, Vector3 cA, Vector3 aB, Vector3 bB, Vector3 cB,
        out float depth, out Vector3 normal)
    {
        normal = Vector3.Cross(edgeDirectionA, edgeDirectionB);
        var normalLength = ScalarMath.Length(normal);
        normal = normal * (1f / normalLength);
        depth = GetDepthForNormal(aA, bA, cA, aB, bB, cB, normal);
        //Protect against bad normals.
        depth = ScalarMath.Select(ScalarMath.LessMask(normalLength, 1e-10f), float.MaxValue, depth);
    }

    //Mirrors both wide Select overloads: normal blends on the strict-less mask, depth takes the running Vector.Min (IEEE).
    static void Select(ref float depth, ref Vector3 normal, float depthCandidate, Vector3 normalCandidate)
    {
        var useCandidate = ScalarMath.LessMask(depthCandidate, depth);
        normal = ScalarMath.Select(useCandidate, normalCandidate, normal);
        depth = MathF.Min(depth, depthCandidate);
    }

    //Mirror of ManifoldCandidateHelper.AddCandidateWithDepth for one lane: masked scatter becomes a branchy append.
    //The candidate's values (including Depth) are computed unconditionally by the callers, matching the wide flow.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void AddCandidateWithDepth(Span<CandidateAos> candidates, ref int candidateCount, in CandidateAos candidate, bool exists)
    {
        if (exists)
        {
            Unsafe.Add(ref MemoryMarshal.GetReference(candidates), candidateCount) = candidate;
            ++candidateCount;
        }
    }

    //Mirrors TrianglePairTester.TryAddTriangleAVertex. No capacity check needed: this pass runs first and adds at most 3.
    static void TryAddTriangleAVertex(Vector3 vertex, Vector2 flattenedVertex, int vertexId,
        Vector3 tangentBX, Vector3 tangentBY, Vector3 triangleCenterB, Vector3 contactNormal, Vector3 faceNormalB,
        Vector2 edgeAB, Vector2 edgeBC, Vector2 edgeCA, Vector2 bA, Vector2 bB,
        bool allowContacts, float inverseContactNormalDotFaceNormalB, float minimumDepth,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        //Test edge plane sign for all three edges of B in the flattened space; strict > 0 containment.
        var bAToVertex = flattenedVertex - bA;
        var bBToVertex = flattenedVertex - bB;
        var abEdgePlaneDot = bAToVertex.Y * edgeAB.X - bAToVertex.X * edgeAB.Y;
        var bcEdgePlaneDot = bBToVertex.Y * edgeBC.X - bBToVertex.X * edgeBC.Y;
        var caEdgePlaneDot = bAToVertex.Y * edgeCA.X - bAToVertex.X * edgeCA.Y;
        var abContained = abEdgePlaneDot > 0f;
        var bcContained = bcEdgePlaneDot > 0f;
        var caContained = caEdgePlaneDot > 0f;
        var contained = abContained & bcContained & caContained;

        //Cast a ray from triangle A's vertex along the contact normal up to the plane of triangle B.
        var offset = triangleCenterB - vertex;
        var distance = ScalarMath.Dot(offset, faceNormalB);
        Unsafe.SkipInit(out CandidateAos candidate);
        candidate.Depth = distance * inverseContactNormalDotFaceNormalB;
        var unprojectedVertex = contactNormal * candidate.Depth;
        unprojectedVertex = unprojectedVertex + vertex;

        var offsetOnB = unprojectedVertex - triangleCenterB;
        candidate.X = ScalarMath.Dot(offsetOnB, tangentBX);
        candidate.Y = ScalarMath.Dot(offsetOnB, tangentBY);
        candidate.FeatureId = vertexId;
        AddCandidateWithDepth(candidates, ref candidateCount, candidate, candidate.Depth >= minimumDepth & (allowContacts & contained));
    }

    //Mirrors TrianglePairTester.ClipEdge, including the 1e-20 sign-preserving parallel guard.
    static void ClipEdge(
        Vector2 edgeStartB, Vector2 edgeOffsetB,
        Vector2 edgeStartA, Vector2 edgeOffsetA, float inverseEdgeLengthSquaredA, float edgeStartADotNormal, float edgeOffsetADotNormal,
        out bool intersectionExists, out float tB, out float depthContributionA)
    {
        var edgePlaneNormalDot = (edgeStartA.X - edgeStartB.X) * edgeOffsetA.Y - (edgeStartA.Y - edgeStartB.Y) * edgeOffsetA.X;
        var velocity = edgeOffsetB.X * edgeOffsetA.Y - edgeOffsetB.Y * edgeOffsetA.X;
        const float parallelThreshold = 1e-20f;
        var parallel = ScalarMath.LessMask(MathF.Abs(velocity), parallelThreshold);
        var denominator = ScalarMath.Select(parallel, ScalarMath.Select(ScalarMath.LessMask(velocity, 0f), -parallelThreshold, parallelThreshold), velocity);
        tB = edgePlaneNormalDot / denominator;
        //To be valid, an intersection must be within both edge bounds.
        var intersectionPointX = tB * edgeOffsetB.X + edgeStartB.X;
        var intersectionPointY = tB * edgeOffsetB.Y + edgeStartB.Y;
        var tA = ((intersectionPointX - edgeStartA.X) * edgeOffsetA.X + (intersectionPointY - edgeStartA.Y) * edgeOffsetA.Y) * inverseEdgeLengthSquaredA;
        intersectionExists = tA >= 0f & tA <= 1f;
        depthContributionA = edgeStartADotNormal + edgeOffsetADotNormal * tA;
    }

    //Mirrors TrianglePairTester.ClipBEdgeAgainstABounds. allowContacts arrives as the caller's
    //stillCouldUseClippingContacts lane (always true here since the scalar caller branches), and is narrowed locally.
    static void ClipBEdgeAgainstABounds(
        Vector2 aA, Vector2 aB, Vector2 aC,
        Vector2 edgeOffsetABOnA, Vector2 edgeOffsetBCOnA, Vector2 edgeOffsetCAOnA,
        float inverseEdgeOffsetABOnALengthSquared, float inverseEdgeOffsetBCOnALengthSquared, float inverseEdgeOffsetCAOnALengthSquared,
        float aDotNormalOnA, float bDotNormalOnA, float cDotNormalOnA,
        float abDotNormalOnA, float bcDotNormalOnA, float caDotNormalOnA,
        Vector2 flatEdgeStartB, Vector2 flatEdgeOffsetB, Vector3 edgeStartB, Vector3 edgeOffsetB,
        int entryId, int exitIdOffset,
        Vector3 triangleCenterB, Vector3 tangentBX, Vector3 tangentBY,
        Vector3 localNormal, float minimumDepth, bool allowContacts, Span<CandidateAos> candidates, ref int candidateCount)
    {
        ClipEdge(flatEdgeStartB, flatEdgeOffsetB, aA, edgeOffsetABOnA, inverseEdgeOffsetABOnALengthSquared, aDotNormalOnA, abDotNormalOnA, out var abIntersected, out var tAB, out var depthContributionABOnA);
        ClipEdge(flatEdgeStartB, flatEdgeOffsetB, aB, edgeOffsetBCOnA, inverseEdgeOffsetBCOnALengthSquared, bDotNormalOnA, bcDotNormalOnA, out var bcIntersected, out var tBC, out var depthContributionBCOnA);
        ClipEdge(flatEdgeStartB, flatEdgeOffsetB, aC, edgeOffsetCAOnA, inverseEdgeOffsetCAOnALengthSquared, cDotNormalOnA, caDotNormalOnA, out var caIntersected, out var tCA, out var depthContributionCAOnA);
        const float minValue = float.MinValue;
        const float maxValue = float.MaxValue;
        var entryAB = ScalarMath.Select(ScalarMath.Mask(abIntersected), tAB, maxValue);
        var entryBC = ScalarMath.Select(ScalarMath.Mask(bcIntersected), tBC, maxValue);
        var entryCA = ScalarMath.Select(ScalarMath.Mask(caIntersected), tCA, maxValue);
        var exitAB = ScalarMath.Select(ScalarMath.Mask(abIntersected), tAB, minValue);
        var exitBC = ScalarMath.Select(ScalarMath.Mask(bcIntersected), tBC, minValue);
        var exitCA = ScalarMath.Select(ScalarMath.Mask(caIntersected), tCA, minValue);
        var entry = MathF.Min(entryAB, MathF.Min(entryBC, entryCA));
        var exit = MathF.Max(exitAB, MathF.Max(exitBC, exitCA));
        //Tie-sensitive Equals chains pick the depth contributions; replicate the wide select order exactly.
        var useABAsEntry = ScalarMath.EqualMask(entry, tAB);
        var useBCAsEntry = ScalarMath.EqualMask(entry, tBC);
        var useABAsExit = ScalarMath.EqualMask(exit, tAB);
        var useBCAsExit = ScalarMath.EqualMask(exit, tBC);
        var depthContributionAAtEntry = ScalarMath.Select(useABAsEntry, depthContributionABOnA, ScalarMath.Select(useBCAsEntry, depthContributionBCOnA, depthContributionCAOnA));
        var depthContributionAAtExit = ScalarMath.Select(useABAsExit, depthContributionABOnA, ScalarMath.Select(useBCAsExit, depthContributionBCOnA, depthContributionCAOnA));
        //If an edge fails to generate any interval, then it's not intersecting the triangle bounds and should not generate contacts.
        //Vector.AndNot(a, b) = a & ~b.
        allowContacts = allowContacts & !(entry == minValue | exit == maxValue);
        entry = MathF.Max(0f, entry);
        exit = MathF.Min(1f, exit);

        var edgeStartBDotNormal = ScalarMath.Dot(edgeStartB, localNormal);
        var edgeOffsetBDotNormal = ScalarMath.Dot(edgeOffsetB, localNormal);
        var depthContributionBAtEntry = edgeStartBDotNormal + entry * edgeOffsetBDotNormal;
        var depthContributionBAtExit = edgeStartBDotNormal + exit * edgeOffsetBDotNormal;

        var offset = edgeStartB - triangleCenterB;
        var offsetX = ScalarMath.Dot(offset, tangentBX);
        var offsetY = ScalarMath.Dot(offset, tangentBY);
        var edgeDirectionX = ScalarMath.Dot(tangentBX, edgeOffsetB);
        var edgeDirectionY = ScalarMath.Dot(tangentBY, edgeOffsetB);

        Unsafe.SkipInit(out CandidateAos candidate);
        //Entry. The capacity test uses the count BEFORE this append; the exit's test sees the updated count, matching the
        //wide sequencing where each AddCandidateWithDepth increments the per-lane count.
        candidate.Depth = depthContributionBAtEntry - depthContributionAAtEntry;
        var exists = (allowContacts & candidate.Depth >= minimumDepth) & ((candidateCount < 6 & exit - entry >= 1e-5f) & (entry < 1f & entry > 0f));
        candidate.X = entry * edgeDirectionX + offsetX;
        candidate.Y = entry * edgeDirectionY + offsetY;
        candidate.FeatureId = entryId;
        AddCandidateWithDepth(candidates, ref candidateCount, candidate, exists);
        //Exit
        candidate.Depth = depthContributionBAtExit - depthContributionAAtExit;
        exists = (allowContacts & candidate.Depth >= minimumDepth) & ((candidateCount < 6 & exit >= entry) & (exit <= 1f & exit >= 0f));
        candidate.X = exit * edgeDirectionX + offsetX;
        candidate.Y = exit * edgeDirectionY + offsetY;
        candidate.FeatureId = entryId + exitIdOffset;
        AddCandidateWithDepth(candidates, ref candidateCount, candidate, exists);
    }

    /// <summary>
    /// Scalar mirror of the wide ManifoldCandidateHelper.ReduceWithoutComputingDepths: identical to BoxTriangleScalarTester's
    /// Reduce copy minus the depth-computation prepass — candidates arrive already carrying Depth (AddCandidateWithDepth path).
    /// The wide bundle-level count squishing only skips candidates that are masked out per lane anyway.
    /// </summary>
    static void ReduceWithoutComputingDepths(Span<CandidateAos> candidates, int candidateCount,
        float epsilonScale, float minimumDepth,
        out CandidateAos contact0, out CandidateAos contact1, out CandidateAos contact2, out CandidateAos contact3,
        out bool contact0Exists, out bool contact1Exists, out bool contact2Exists, out bool contact3Exists)
    {
        ref var candidatesBase = ref MemoryMarshal.GetReference(candidates);
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
        in Triangle a, in Triangle b, float speculativeMargin,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        Unsafe.SkipInit(out manifold);
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 worldRA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 worldRB);
        //A-local space: note (worldRB, worldRA) argument order, reversed relative to the B-local testers.
        ScalarMath.MultiplyByTranspose(worldRB, worldRA, out var rB);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, worldRA);
        Matrix3x3.Transform(b.A, rB, out var bA);
        bA += localOffsetB;
        Matrix3x3.Transform(b.B, rB, out var bB);
        bB += localOffsetB;
        Matrix3x3.Transform(b.C, rB, out var bC);
        bC += localOffsetB;

        var localTriangleCenterB = bA + bB;
        localTriangleCenterB = localTriangleCenterB + bC;
        localTriangleCenterB = localTriangleCenterB * (1f / 3f);

        var abB = bB - bA;
        var bcB = bC - bB;
        var caB = bA - bC;

        var localTriangleCenterA = a.A + a.B;
        localTriangleCenterA = localTriangleCenterA + a.C;
        localTriangleCenterA = localTriangleCenterA * (1f / 3f);

        var abA = a.B - a.A;
        var bcA = a.C - a.B;
        var caA = a.A - a.C;

        //A AB x *
        TestEdgeEdge(abA, abB, a.A, a.B, a.C, bA, bB, bC, out var depth, out var localNormal);
        TestEdgeEdge(abA, bcB, a.A, a.B, a.C, bA, bB, bC, out var depthCandidate, out var localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
        TestEdgeEdge(abA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

        //A BC x *
        TestEdgeEdge(bcA, abB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
        TestEdgeEdge(bcA, bcB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
        TestEdgeEdge(bcA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

        //A CA x *
        TestEdgeEdge(caA, abB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
        TestEdgeEdge(caA, bcB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
        TestEdgeEdge(caA, caB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
        Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

        //Face normals
        var faceNormalA = Vector3.Cross(abA, caA);
        var faceNormalALength = ScalarMath.Length(faceNormalA);
        faceNormalA = faceNormalA * (1f / faceNormalALength);
        depthCandidate = GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalA);
        Select(ref depth, ref localNormal, depthCandidate, faceNormalA);
        var faceNormalB = Vector3.Cross(abB, caB);
        var faceNormalBLength = ScalarMath.Length(faceNormalB);
        faceNormalB = faceNormalB * (1f / faceNormalBLength);
        var faceDepthB = GetDepthForNormal(a.A, a.B, a.C, bA, bB, bC, faceNormalB);
        Select(ref depth, ref localNormal, faceDepthB, faceNormalB);

        var abALengthSquared = ScalarMath.Dot(abA, abA);
        var abBLengthSquared = ScalarMath.Dot(abB, abB);
        var caALengthSquared = ScalarMath.Dot(caA, caA);
        var caBLengthSquared = ScalarMath.Dot(caB, caB);
        //The commented-out vertex-normal block in the wide tester is dead code and is skipped here.

        //Point the normal from B to A by convention.
        var centerAToCenterB = localTriangleCenterB - localTriangleCenterA;
        var calibrationDot = ScalarMath.Dot(localNormal, centerAToCenterB);
        var shouldFlip = ScalarMath.GreaterMask(calibrationDot, 0f);
        localNormal = ScalarMath.Select(shouldFlip, -localNormal, localNormal);

        var minimumDepth = -speculativeMargin;
        var localNormalDotFaceNormalA = ScalarMath.Dot(localNormal, faceNormalA);
        var localNormalDotFaceNormalB = ScalarMath.Dot(localNormal, faceNormalB);
        //TriangleWide.ComputeNondegenerateTriangleMask; sums of squares can't hit the MaxNative/IEEE divergence,
        //but MathF.Max is the exact IEEE mirror regardless.
        var epsilonScaleA = MathF.Sqrt(MathF.Max(abALengthSquared, caALengthSquared));
        var epsilonScaleB = MathF.Sqrt(MathF.Max(abBLengthSquared, caBLengthSquared));
        var nondegenerateMaskA = faceNormalALength > TriangleWide.DegenerateTriangleEpsilon * epsilonScaleA;
        var nondegenerateMaskB = faceNormalBLength > TriangleWide.DegenerateTriangleEpsilon * epsilonScaleB;
        //allowContacts starts true for a tested lane (CreateMaskForCountInBundle).
        var allowContacts = (nondegenerateMaskA & nondegenerateMaskB) &
            (depth >= minimumDepth &
            (localNormalDotFaceNormalA < -TriangleWide.BackfaceNormalDotRejectionThreshold &
            localNormalDotFaceNormalB > TriangleWide.BackfaceNormalDotRejectionThreshold));
        if (!allowContacts)
        {
            //A rejected wide lane generates zero candidates and its exists flags are ANDed with allowContacts, so every
            //exists flag comes out false; nothing else is observable.
            manifold = default;
            return;
        }

        //Flatten both triangles onto a plane with normal equal to the detected local normal to perform clipping.
        BepuPhysics.Helpers.BuildOrthonormalBasis(localNormal, out var flattenX, out var flattenY);
        Vector2 flatVertexAOnA, flatVertexBOnA, flatVertexCOnA;
        Vector2 flatVertexAOnB, flatVertexBOnB, flatVertexCOnB;
        flatVertexAOnA.X = ScalarMath.Dot(a.A, flattenX);
        flatVertexAOnA.Y = ScalarMath.Dot(a.A, flattenY);
        flatVertexBOnA.X = ScalarMath.Dot(a.B, flattenX);
        flatVertexBOnA.Y = ScalarMath.Dot(a.B, flattenY);
        flatVertexCOnA.X = ScalarMath.Dot(a.C, flattenX);
        flatVertexCOnA.Y = ScalarMath.Dot(a.C, flattenY);
        flatVertexAOnB.X = ScalarMath.Dot(bA, flattenX);
        flatVertexAOnB.Y = ScalarMath.Dot(bA, flattenY);
        flatVertexBOnB.X = ScalarMath.Dot(bB, flattenX);
        flatVertexBOnB.Y = ScalarMath.Dot(bB, flattenY);
        flatVertexCOnB.X = ScalarMath.Dot(bC, flattenX);
        flatVertexCOnB.Y = ScalarMath.Dot(bC, flattenY);

        var flatEdgeABOnA = flatVertexBOnA - flatVertexAOnA;
        var flatEdgeBCOnA = flatVertexCOnA - flatVertexBOnA;
        var flatEdgeCAOnA = flatVertexAOnA - flatVertexCOnA;
        var flatEdgeABOnB = flatVertexBOnB - flatVertexAOnB;
        var flatEdgeBCOnB = flatVertexCOnB - flatVertexBOnB;
        var flatEdgeCAOnB = flatVertexAOnB - flatVertexCOnB;

        //Only the face-case-for-B lane matters downstream; the edge/face flags for A and the edge flag for B are unused.
        //allowContacts is true past the early-out, so the wide BitwiseAnd terms drop out.
        const float edgeThreshold = 0.2f;
        var useFaceCaseForB = !(MathF.Abs(localNormalDotFaceNormalB) < edgeThreshold);

        //2d parameterization of the surface of triangle B for contact reduction.
        var tangentBX = abB * (1f / MathF.Sqrt(abBLengthSquared));
        var tangentBY = Vector3.Cross(tangentBX, faceNormalB);

        //At most 6 candidates: 3 A-vertex contacts plus at most 2 per B edge, capacity-gated on the clipping path.
        Span<CandidateAos> candidates = stackalloc CandidateAos[6];
        int candidateCount = 0;

        if (useFaceCaseForB)
        {
            //aVertex-bFace contacts, added first so this pass never needs capacity tests.
            //Division by zero is protected by useFaceCaseForB.
            var inverseContactNormalDotFaceNormalB = 1f / localNormalDotFaceNormalB;
            TryAddTriangleAVertex(a.A, flatVertexAOnA, 0, tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, flatEdgeABOnB, flatEdgeBCOnB, flatEdgeCAOnB, flatVertexAOnB, flatVertexBOnB, useFaceCaseForB, inverseContactNormalDotFaceNormalB, minimumDepth, candidates, ref candidateCount);
            TryAddTriangleAVertex(a.B, flatVertexBOnA, 1, tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, flatEdgeABOnB, flatEdgeBCOnB, flatEdgeCAOnB, flatVertexAOnB, flatVertexBOnB, useFaceCaseForB, inverseContactNormalDotFaceNormalB, minimumDepth, candidates, ref candidateCount);
            TryAddTriangleAVertex(a.C, flatVertexCOnA, 2, tangentBX, tangentBY, localTriangleCenterB, localNormal, faceNormalB, flatEdgeABOnB, flatEdgeBCOnB, flatEdgeCAOnB, flatVertexAOnB, flatVertexBOnB, useFaceCaseForB, inverseContactNormalDotFaceNormalB, minimumDepth, candidates, ref candidateCount);
        }
        //Note that edge cases will also add triangle B vertices that are within triangle A's bounds, so no B vertex case is required.
        //The wide code evaluates this once, before any clipping call, using the post-vertex-pass count.
        var stillCouldUseClippingContacts = candidateCount < 3;
        if (stillCouldUseClippingContacts)
        {
            var flatEdgeOffsetABOnALengthSquared = flatEdgeABOnA.X * flatEdgeABOnA.X + flatEdgeABOnA.Y * flatEdgeABOnA.Y;
            var flatEdgeOffsetBCOnALengthSquared = flatEdgeBCOnA.X * flatEdgeBCOnA.X + flatEdgeBCOnA.Y * flatEdgeBCOnA.Y;
            var flatEdgeOffsetCAOnALengthSquared = flatEdgeCAOnA.X * flatEdgeCAOnA.X + flatEdgeCAOnA.Y * flatEdgeCAOnA.Y;
            var inverseFlatEdgeOffsetABOnALengthSquared = 1f / flatEdgeOffsetABOnALengthSquared;
            var inverseFlatEdgeOffsetBCOnALengthSquared = 1f / flatEdgeOffsetBCOnALengthSquared;
            var inverseFlatEdgeOffsetCAOnALengthSquared = 1f / flatEdgeOffsetCAOnALengthSquared;
            //These clipping routines compute depth directly rather than relying on reduction to compute it.
            var aDotNormalOnA = ScalarMath.Dot(localNormal, a.A);
            var bDotNormalOnA = ScalarMath.Dot(localNormal, a.B);
            var cDotNormalOnA = ScalarMath.Dot(localNormal, a.C);
            var abDotNormalOnA = ScalarMath.Dot(localNormal, abA);
            var bcDotNormalOnA = ScalarMath.Dot(localNormal, bcA);
            var caDotNormalOnA = ScalarMath.Dot(localNormal, caA);
            ClipBEdgeAgainstABounds(flatVertexAOnA, flatVertexBOnA, flatVertexCOnA, flatEdgeABOnA, flatEdgeBCOnA, flatEdgeCAOnA, inverseFlatEdgeOffsetABOnALengthSquared, inverseFlatEdgeOffsetBCOnALengthSquared, inverseFlatEdgeOffsetCAOnALengthSquared, aDotNormalOnA, bDotNormalOnA, cDotNormalOnA, abDotNormalOnA, bcDotNormalOnA, caDotNormalOnA, flatVertexAOnB, flatEdgeABOnB, bA, abB, 3, 3, localTriangleCenterB, tangentBX, tangentBY, localNormal, minimumDepth, stillCouldUseClippingContacts, candidates, ref candidateCount);
            ClipBEdgeAgainstABounds(flatVertexAOnA, flatVertexBOnA, flatVertexCOnA, flatEdgeABOnA, flatEdgeBCOnA, flatEdgeCAOnA, inverseFlatEdgeOffsetABOnALengthSquared, inverseFlatEdgeOffsetBCOnALengthSquared, inverseFlatEdgeOffsetCAOnALengthSquared, aDotNormalOnA, bDotNormalOnA, cDotNormalOnA, abDotNormalOnA, bcDotNormalOnA, caDotNormalOnA, flatVertexBOnB, flatEdgeBCOnB, bB, bcB, 4, 3, localTriangleCenterB, tangentBX, tangentBY, localNormal, minimumDepth, stillCouldUseClippingContacts, candidates, ref candidateCount);
            ClipBEdgeAgainstABounds(flatVertexAOnA, flatVertexBOnA, flatVertexCOnA, flatEdgeABOnA, flatEdgeBCOnA, flatEdgeCAOnA, inverseFlatEdgeOffsetABOnALengthSquared, inverseFlatEdgeOffsetBCOnALengthSquared, inverseFlatEdgeOffsetCAOnALengthSquared, aDotNormalOnA, bDotNormalOnA, cDotNormalOnA, abDotNormalOnA, bcDotNormalOnA, caDotNormalOnA, flatVertexCOnB, flatEdgeCAOnB, bC, caB, 5, 3, localTriangleCenterB, tangentBX, tangentBY, localNormal, minimumDepth, stillCouldUseClippingContacts, candidates, ref candidateCount);
        }

        //Scale-sensitive epsilon based on the size of the involved shapes.
        var epsilonScale = MathF.Min(epsilonScaleA, epsilonScaleB);
        ReduceWithoutComputingDepths(candidates, candidateCount, epsilonScale, minimumDepth,
            out var contact0, out var contact1, out var contact2, out var contact3,
            out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

        //Transform the contacts into the manifold; the basis moves into world rotation so individual contacts don't need transforming.
        Matrix3x3.Transform(tangentBX, worldRA, out var worldTangentBX);
        Matrix3x3.Transform(tangentBY, worldRA, out var worldTangentBY);
        Matrix3x3.Transform(localTriangleCenterB, worldRA, out var worldTriangleCenter);
        Matrix3x3.Transform(localNormal, worldRA, out manifold.Normal);
        //The wide code ANDs the exists flags with allowContacts here; allowContacts is true past the early-out.
        TransformContactToManifold(contact0, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA0, out manifold.Depth0, out manifold.FeatureId0);
        TransformContactToManifold(contact1, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA1, out manifold.Depth1, out manifold.FeatureId1);
        TransformContactToManifold(contact2, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA2, out manifold.Depth2, out manifold.FeatureId2);
        TransformContactToManifold(contact3, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA3, out manifold.Depth3, out manifold.FeatureId3);
        //Note that we privilege triangle B. Boundary smoothing is only performed on one of the two meshes.
        var faceFlag = localNormalDotFaceNormalB >= MeshReduction.MinimumDotForFaceCollision ? MeshReduction.FaceCollisionFlag : 0;
        manifold.FeatureId0 += faceFlag;
    }
}
