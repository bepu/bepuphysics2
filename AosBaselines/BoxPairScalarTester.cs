using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace AosBaselines;

/// <summary>
/// Manifold candidate for the scalar box pair tester. Mirrors the per-lane content of ManifoldCandidate.
/// </summary>
public struct CandidateAos
{
    public float X;
    public float Y;
    public float Depth;
    public int FeatureId;
}

/// <summary>
/// Scalar AoS port of BoxPairTester, bitwise identical per lane to the wide implementation.
/// Structure and comments deliberately track BoxPairTester line by line; see that file for the algorithm's reasoning.
/// </summary>
public static class BoxPairScalarTester
{
    static void TestEdgeEdge(
        float halfWidthA, float halfHeightA, float halfLengthA,
        float halfWidthB, float halfHeightB, float halfLengthB,
        float offsetBX, float offsetBY, float offsetBZ,
        Vector3 rBX, Vector3 rBY, Vector3 rBZ,
        Vector3 edgeBDirection,
        out float depth, out Vector3 localNormal)
    {
        //Tests one axis of B against all three axes of A.
        if (Vector128.IsHardwareAccelerated)
        {
            //The three axis blocks below have identical structure, so they pack into lanes 0..2 of Vector128 values.
            //Per lane, every operation matches the scalar block's operand order exactly, and vector sqrt/divide/multiply/add
            //are lane-wise bit-identical to their scalar counterparts, so this path is bitwise equal to the scalar fallback.
            //Lane layout: lane0 = A.X x edgeB, lane1 = A.Y x edgeB, lane2 = A.Z x edgeB. Lane 3 selectors point at the
            //zero-extended W so p and q carry an exact zero there (after masking the 0 * inf = NaN degenerate case),
            //which lets the candidate normals assemble with shuffle+or merges.
            var e = edgeBDirection.AsVector128();
            var sq = e * e; //(x2, y2, z2, 0)
            //lengthSquared = (y2 + z2, x2 + z2, x2 + y2, 0): first operand (y2, x2, x2), second (z2, z2, y2), preserving each block's add order.
            var lengthSquared = Vector128.Shuffle(sq, Vector128.Create(1, 0, 0, 3)) + Vector128.Shuffle(sq, Vector128.Create(2, 2, 1, 3));
            var length = Vector128.Sqrt(lengthSquared);
            var inverseLength = Vector128.Create(1f) / length;
            var lane3Zero = Vector128.Create(-1, -1, -1, 0).AsSingle();
            //Candidate normal components: p = first nonzero component per block = (eZ, eZ, eY) * il; q = second = (-eY, -eX, -eX) * il.
            var p = (Vector128.Shuffle(e, Vector128.Create(2, 2, 1, 3)) * inverseLength) & lane3Zero;
            var q = ((Vector128.Shuffle(e, Vector128.Create(1, 0, 0, 3)) ^ Vector128.Create(-0.0f)) * inverseLength) & lane3Zero;
            var absMask = Vector128.Create(0x7FFFFFFF).AsSingle();
            var extremeA = ((p & absMask) * Vector128.Create(halfHeightA, halfWidthA, halfWidthA, 0f))
                + ((q & absMask) * Vector128.Create(halfLengthA, halfLengthA, halfHeightA, 0f));
            //nB_c = p * rBc[firstAxis] + q * rBc[secondAxis]; per block the axis pairs are (Y,Z), (X,Z), (X,Y).
            var firstAxes = Vector128.Create(1, 0, 0, 3);
            var secondAxes = Vector128.Create(2, 2, 1, 3);
            var rbxv = rBX.AsVector128();
            var rbyv = rBY.AsVector128();
            var rbzv = rBZ.AsVector128();
            var nBX = (p * Vector128.Shuffle(rbxv, firstAxes)) + (q * Vector128.Shuffle(rbxv, secondAxes));
            var nBY = (p * Vector128.Shuffle(rbyv, firstAxes)) + (q * Vector128.Shuffle(rbyv, secondAxes));
            var nBZ = (p * Vector128.Shuffle(rbzv, firstAxes)) + (q * Vector128.Shuffle(rbzv, secondAxes));
            var extremeB = ((nBX & absMask) * Vector128.Create(halfWidthB))
                + ((nBY & absMask) * Vector128.Create(halfHeightB))
                + ((nBZ & absMask) * Vector128.Create(halfLengthB));
            var offsetDot = (Vector128.Create(offsetBY, offsetBX, offsetBX, 0f) * p)
                + (Vector128.Create(offsetBZ, offsetBZ, offsetBY, 0f) * q);
            var depths = (extremeA + extremeB) - (offsetDot & absMask);
            var degenerate = Vector128.LessThan(length, Vector128.Create(1e-7f));
            depths = Vector128.ConditionalSelect(degenerate, Vector128.Create(float.MaxValue), depths);

            //Candidate normals: n0 = (0, p0, q0), n1 = (p1, 0, q1), n2 = (p2, q2, 0).
            //p and q have exact zeros in lane 3, so each normal is two shuffles or'd together; or-with-zero preserves bits exactly.
            var n0 = Vector128.Shuffle(p, Vector128.Create(3, 0, 3, 3)) | Vector128.Shuffle(q, Vector128.Create(3, 3, 0, 3));
            var n1 = Vector128.Shuffle(p, Vector128.Create(1, 3, 3, 3)) | Vector128.Shuffle(q, Vector128.Create(3, 3, 1, 3));
            var n2 = Vector128.Shuffle(p, Vector128.Create(2, 3, 3, 3)) | Vector128.Shuffle(q, Vector128.Create(3, 2, 3, 3));

            //Sequential min-depth selection across the three lanes, same comparison order as the scalar blocks.
            depth = depths.ToScalar();
            var normal = n0;
            var d1 = depths.GetElement(1);
            var use1 = ScalarMath.LessMask(d1, depth);
            normal = Vector128.ConditionalSelect(use1, n1, normal);
            depth = ScalarMath.Select(use1, d1, depth);
            var d2 = depths.GetElement(2);
            var use2 = ScalarMath.LessMask(d2, depth);
            normal = Vector128.ConditionalSelect(use2, n2, normal);
            depth = ScalarMath.Select(use2, d2, depth);
            localNormal = normal.AsVector3();
            return;
        }
        //Normals are built as whole vectors and blended as whole vectors; every component still sees the exact same operation sequence
        //as the original componentwise version, so results remain bitwise identical while avoiding insert/extract churn on SIMD-promoted locals.
        var x2 = edgeBDirection.X * edgeBDirection.X;
        var y2 = edgeBDirection.Y * edgeBDirection.Y;
        var z2 = edgeBDirection.Z * edgeBDirection.Z;
        {
            //A.X x edgeB
            var length = MathF.Sqrt(y2 + z2);
            var inverseLength = 1f / length;
            var nY = edgeBDirection.Z * inverseLength;
            var nZ = -edgeBDirection.Y * inverseLength;
            localNormal = new Vector3(0f, nY, nZ);
            var extremeA = MathF.Abs(nY) * halfHeightA + MathF.Abs(nZ) * halfLengthA;
            var nBX = nY * rBX.Y + nZ * rBX.Z;
            var nBY = nY * rBY.Y + nZ * rBY.Z;
            var nBZ = nY * rBZ.Y + nZ * rBZ.Z;
            var extremeB = MathF.Abs(nBX) * halfWidthB + MathF.Abs(nBY) * halfHeightB + MathF.Abs(nBZ) * halfLengthB;
            depth = extremeA + extremeB - MathF.Abs(offsetBY * nY + offsetBZ * nZ);
            depth = ScalarMath.Select(ScalarMath.LessMask(length, 1e-7f), float.MaxValue, depth);
        }
        {
            //A.Y x edgeB
            var length = MathF.Sqrt(x2 + z2);
            var inverseLength = 1f / length;
            var nX = edgeBDirection.Z * inverseLength;
            var nZ = -edgeBDirection.X * inverseLength;
            var extremeA = MathF.Abs(nX) * halfWidthA + MathF.Abs(nZ) * halfLengthA;
            var nBX = nX * rBX.X + nZ * rBX.Z;
            var nBY = nX * rBY.X + nZ * rBY.Z;
            var nBZ = nX * rBZ.X + nZ * rBZ.Z;
            var extremeB = MathF.Abs(nBX) * halfWidthB + MathF.Abs(nBY) * halfHeightB + MathF.Abs(nBZ) * halfLengthB;
            var d = extremeA + extremeB - MathF.Abs(offsetBX * nX + offsetBZ * nZ);
            d = ScalarMath.Select(ScalarMath.LessMask(length, 1e-7f), float.MaxValue, d);
            var useY = ScalarMath.LessMask(d, depth);
            depth = ScalarMath.Select(useY, d, depth);
            localNormal = ScalarMath.Select(useY, new Vector3(nX, 0f, nZ), localNormal);
        }
        {
            //A.Z x edgeB
            var length = MathF.Sqrt(x2 + y2);
            var inverseLength = 1f / length;
            var nX = edgeBDirection.Y * inverseLength;
            var nY = -edgeBDirection.X * inverseLength;
            var extremeA = MathF.Abs(nX) * halfWidthA + MathF.Abs(nY) * halfHeightA;
            var nBX = nX * rBX.X + nY * rBX.Y;
            var nBY = nX * rBY.X + nY * rBY.Y;
            var nBZ = nX * rBZ.X + nY * rBZ.Y;
            var extremeB = MathF.Abs(nBX) * halfWidthB + MathF.Abs(nBY) * halfHeightB + MathF.Abs(nBZ) * halfLengthB;
            var d = extremeA + extremeB - MathF.Abs(offsetBX * nX + offsetBY * nY);
            d = ScalarMath.Select(ScalarMath.LessMask(length, 1e-7f), float.MaxValue, d);
            var useZ = ScalarMath.LessMask(d, depth);
            depth = ScalarMath.Select(useZ, d, depth);
            localNormal = ScalarMath.Select(useZ, new Vector3(nX, nY, 0f), localNormal);
        }
    }

    static void Select(ref float depth, ref Vector3 normal, float candidateDepth, Vector3 candidateNormal)
    {
        var useCandidate = ScalarMath.LessMask(candidateDepth, depth);
        depth = ScalarMath.Select(useCandidate, candidateDepth, depth);
        normal = ScalarMath.Select(useCandidate, candidateNormal, normal);
    }

    static void AddBoxAVertex(Vector3 vertex, int featureId, Vector3 faceNormalB, Vector3 contactNormal, float inverseContactNormalDotFaceNormalB,
        Vector3 faceCenterB, Vector3 faceTangentBX, Vector3 faceTangentBY, float halfSpanBX, float halfSpanBY,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        //Cast a ray from the box A vertex up to the box B face along the contact normal.
        var pointOnBToVertex = vertex - faceCenterB;
        var planeDistance = ScalarMath.Dot(faceNormalB, pointOnBToVertex);
        var t = planeDistance * inverseContactNormalDotFaceNormalB;
        var offset = contactNormal * t;
        //Contact normal points from B to A by convention, so we have to subtract.
        var vertexOnBFace = vertex - offset;

        var vertexOffsetOnBFace = vertexOnBFace - faceCenterB;
        Unsafe.SkipInit(out CandidateAos candidate);
        candidate.X = ScalarMath.Dot(vertexOffsetOnBFace, faceTangentBX);
        candidate.Y = ScalarMath.Dot(vertexOffsetOnBFace, faceTangentBY);
        candidate.FeatureId = featureId;

        var contained = MathF.Abs(candidate.X) <= halfSpanBX & MathF.Abs(candidate.Y) <= halfSpanBY;
        //Wide version clamps the candidate count against the buffer capacity explicitly; mirror it.
        if (contained & candidateCount < 8)
        {
            Unsafe.Add(ref MemoryMarshal.GetReference(candidates), candidateCount) = candidate;
            ++candidateCount;
        }
    }

    //Phase methods carry no inlining directives: they are large enough that the JIT keeps them as calls by default, restoring
    //a full inlining budget per phase (the tester in one piece exhausted it, degrading even Vector3 operator intrinsics into
    //real calls), while dynamic PGO remains free to inline a phase where the profile justifies it. Arguments are by value —
    //in/byref parameters take addresses of the caller's SIMD locals and force them addressable whether or not the call inlines.
    static void AddBoxAVertices(Vector3 faceCenterB, Vector3 faceTangentBX, Vector3 faceTangentBY, float halfSpanBX, float halfSpanBY,
        Vector3 faceNormalB, Vector3 contactNormal,
        Vector3 v00, Vector3 v01, Vector3 v10, Vector3 v11,
        int f00, int f01, int f10, int f11,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        var normalDot = ScalarMath.Dot(faceNormalB, contactNormal);
        var inverseContactNormalDotFaceNormalB = MathF.Abs(normalDot) > 1e-10f ? 1f / normalDot : float.MaxValue;

        AddBoxAVertex(v00, f00, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY, candidates, ref candidateCount);
        AddBoxAVertex(v01, f01, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY, candidates, ref candidateCount);
        AddBoxAVertex(v10, f10, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY, candidates, ref candidateCount);
        AddBoxAVertex(v11, f11, faceNormalB, contactNormal, inverseContactNormalDotFaceNormalB, faceCenterB, faceTangentBX, faceTangentBY, halfSpanBX, halfSpanBY, candidates, ref candidateCount);
    }

    static void ClipBoxBEdgeAgainstBoxAFace(Vector3 edgeDirection,
        Vector3 edgeStartB0ToEdgeAnchorA00, Vector3 edgeStartB0ToEdgeAnchorA11,
        Vector3 edgeStartB1ToEdgeAnchorA00, Vector3 edgeStartB1ToEdgeAnchorA11,
        Vector3 boxEdgePlaneNormal,
        out float min0, out float max0,
        out float min1, out float max1)
    {
        var distance00 = ScalarMath.Dot(edgeStartB0ToEdgeAnchorA00, boxEdgePlaneNormal);
        var distance01 = ScalarMath.Dot(edgeStartB0ToEdgeAnchorA11, boxEdgePlaneNormal);
        var distance10 = ScalarMath.Dot(edgeStartB1ToEdgeAnchorA00, boxEdgePlaneNormal);
        var distance11 = ScalarMath.Dot(edgeStartB1ToEdgeAnchorA11, boxEdgePlaneNormal);
        var velocity = ScalarMath.Dot(boxEdgePlaneNormal, edgeDirection);
        var inverseVelocity = 1f / velocity;

        //If the distances to the planes have opposing signs, then the start must be between the two.
        var edgeStartIsInside0 = ScalarMath.LessOrEqualMask(distance00 * distance01, 0f);
        var edgeStartIsInside1 = ScalarMath.LessOrEqualMask(distance10 * distance11, 0f);
        var dontUseFallback = ScalarMath.GreaterMask(MathF.Abs(velocity), 1e-15f);
        var t00 = distance00 * inverseVelocity;
        var t01 = distance01 * inverseVelocity;
        var t10 = distance10 * inverseVelocity;
        var t11 = distance11 * inverseVelocity;
        //If the edge direction and plane surface is parallel, then the interval is defined entirely by whether the edge starts inside or outside.
        const float largeNegative = -float.MaxValue;
        const float largePositive = float.MaxValue;
        min0 = ScalarMath.Select(dontUseFallback, float.MinNative(t00, t01), ScalarMath.Select(edgeStartIsInside0, largeNegative, largePositive));
        max0 = ScalarMath.Select(dontUseFallback, float.MaxNative(t00, t01), ScalarMath.Select(edgeStartIsInside0, largePositive, largeNegative));
        min1 = ScalarMath.Select(dontUseFallback, float.MinNative(t10, t11), ScalarMath.Select(edgeStartIsInside1, largeNegative, largePositive));
        max1 = ScalarMath.Select(dontUseFallback, float.MaxNative(t10, t11), ScalarMath.Select(edgeStartIsInside1, largePositive, largeNegative));
    }

    //The fully packed (transposed four-dot) form of this clip measured as a wash against the form below, so the simpler
    //portable version stays: mask-based selects keep the chains in SIMD registers, and the dots are ordered vector reductions.
    static void ClipBoxBEdgesAgainstBoxAFace(Vector3 edgeStartB0, Vector3 edgeStartB1, Vector3 edgeDirectionB, float halfSpanB,
        Vector3 vertexA00, Vector3 vertexA11, Vector3 edgePlaneNormalAX, Vector3 edgePlaneNormalAY,
        out float min0, out float max0, out float min1, out float max1)
    {
        var edgeStartB0ToVA00 = vertexA00 - edgeStartB0;
        var edgeStartB0ToVA11 = vertexA11 - edgeStartB0;
        var edgeStartB1ToVA00 = vertexA00 - edgeStartB1;
        var edgeStartB1ToVA11 = vertexA11 - edgeStartB1;
        ClipBoxBEdgeAgainstBoxAFace(edgeDirectionB, edgeStartB0ToVA00, edgeStartB0ToVA11, edgeStartB1ToVA00, edgeStartB1ToVA11, edgePlaneNormalAX,
            out var minX0, out var maxX0, out var minX1, out var maxX1);
        ClipBoxBEdgeAgainstBoxAFace(edgeDirectionB, edgeStartB0ToVA00, edgeStartB0ToVA11, edgeStartB1ToVA00, edgeStartB1ToVA11, edgePlaneNormalAY,
            out var minY0, out var maxY0, out var minY1, out var maxY1);
        var negativeHalfSpanB = -halfSpanB;
        //Note that we are computing the intersection of the two intervals; see the wide version for details.
        min0 = float.MaxNative(negativeHalfSpanB, float.MaxNative(minX0, minY0));
        max0 = float.MinNative(halfSpanB, float.MinNative(maxX0, maxY0));
        min1 = float.MaxNative(negativeHalfSpanB, float.MaxNative(minX1, minY1));
        max1 = float.MinNative(halfSpanB, float.MinNative(maxX1, maxY1));
    }

    static void AddContactsForEdge(float min, CandidateAos minCandidate, float max, CandidateAos maxCandidate, float halfSpanB,
        float epsilon, Span<CandidateAos> candidates, ref int candidateCount)
    {
        //If -halfSpan<min<halfSpan && (max-min)>epsilon for an edge, use the min intersection as a contact.
        //If -halfSpan<=max<=halfSpan && max>=min, use the max intersection as a contact.
        //Plain branchy appends: the branchless blind-store variant (trash slot + count advanced by the reinterpreted flag)
        //measured as a wash against this, so the simpler form stays.
        ref var candidatesBase = ref MemoryMarshal.GetReference(candidates);
        var minExists = max - min > epsilon & MathF.Abs(min) < halfSpanB;
        if (minExists)
        {
            Unsafe.Add(ref candidatesBase, candidateCount) = minCandidate;
            ++candidateCount;
        }

        var maxExists = max >= min & MathF.Abs(max) <= halfSpanB;
        if (maxExists)
        {
            Unsafe.Add(ref candidatesBase, candidateCount) = maxCandidate;
            ++candidateCount;
        }
    }

    static void CreateEdgeContacts(
        Vector3 faceCenterB, Vector3 faceTangentBX, Vector3 faceTangentBY, float halfSpanBX, float halfSpanBY,
        Vector3 vertexA00, Vector3 vertexA11, Vector3 faceTangentAX, Vector3 faceTangentAY, Vector3 contactNormal,
        int featureIdX0, int featureIdX1, int featureIdY0, int featureIdY1,
        float epsilonScale, Span<CandidateAos> candidates, ref int candidateCount)
    {
        //Clip on the contact normal plane; see the wide version for commentary.
        var edgePlaneNormalAX = Vector3.Cross(faceTangentAY, contactNormal);
        var edgePlaneNormalAY = Vector3.Cross(faceTangentAX, contactNormal);

        var edgeOffsetBX = faceTangentBY * halfSpanBY;
        var edgeOffsetBY = faceTangentBX * halfSpanBX;
        var edgeStartBX0 = faceCenterB - edgeOffsetBX;
        var edgeStartBX1 = faceCenterB + edgeOffsetBX;
        ClipBoxBEdgesAgainstBoxAFace(edgeStartBX0, edgeStartBX1, faceTangentBX, halfSpanBX, vertexA00, vertexA11, edgePlaneNormalAX, edgePlaneNormalAY,
            out var minX0, out var maxX0, out var unflippedMinX1, out var unflippedMaxX1);
        var edgeStartBY0 = faceCenterB - edgeOffsetBY;
        var edgeStartBY1 = faceCenterB + edgeOffsetBY;
        ClipBoxBEdgesAgainstBoxAFace(edgeStartBY0, edgeStartBY1, faceTangentBY, halfSpanBY, vertexA00, vertexA11, edgePlaneNormalAX, edgePlaneNormalAY,
            out var unflippedMinY0, out var unflippedMaxY0, out var minY1, out var maxY1);

        //Flip the X1 and Y0 intervals to maintain winding; see wide version.
        var minX1 = -unflippedMaxX1;
        var maxX1 = -unflippedMinX1;
        var minY0 = -unflippedMaxY0;
        var maxY0 = -unflippedMinY0;

        //We now have intervals for all four box B edges.
        const int edgeFeatureIdOffset = 64;
        var epsilon = epsilonScale * 1e-5f;
        Unsafe.SkipInit(out CandidateAos min);
        Unsafe.SkipInit(out CandidateAos max);
        //X0
        min.FeatureId = featureIdX0;
        min.X = minX0;
        min.Y = -halfSpanBY;
        max.FeatureId = featureIdX0 + edgeFeatureIdOffset;
        max.X = maxX0;
        max.Y = min.Y;
        AddContactsForEdge(minX0, min, maxX0, max, halfSpanBX, epsilon, candidates, ref candidateCount);

        //Y1
        min.FeatureId = featureIdY1;
        min.X = halfSpanBX;
        min.Y = minY1;
        max.FeatureId = featureIdY1 + edgeFeatureIdOffset;
        max.X = halfSpanBX;
        max.Y = maxY1;
        AddContactsForEdge(minY1, min, maxY1, max, halfSpanBY, epsilon, candidates, ref candidateCount);

        //X1
        min.FeatureId = featureIdX1;
        min.X = unflippedMaxX1;
        min.Y = halfSpanBY;
        max.FeatureId = featureIdX1 + edgeFeatureIdOffset;
        max.X = unflippedMinX1;
        max.Y = halfSpanBY;
        AddContactsForEdge(minX1, min, maxX1, max, halfSpanBX, epsilon, candidates, ref candidateCount);

        //Y0
        min.FeatureId = featureIdY0;
        min.X = -halfSpanBX;
        min.Y = unflippedMaxY0;
        max.FeatureId = featureIdY0 + edgeFeatureIdOffset;
        max.X = min.X;
        max.Y = unflippedMinY0;
        AddContactsForEdge(minY0, min, maxY0, max, halfSpanBY, epsilon, candidates, ref candidateCount);
    }

    /// <summary>
    /// Scalar mirror of the wide ManifoldCandidateHelper.Reduce. The wide version's bundle-level iteration count squishing
    /// (SquishMaximumContactCount / depth-based trimming) only skips candidates that are masked out per lane anyway, so per-lane results are identical.
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
        //candidateCount never exceeds the span length by construction; index through a base ref to skip per-iteration bounds checks.
        ref var candidatesBase = ref MemoryMarshal.GetReference(candidates);
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref Unsafe.Add(ref candidatesBase, i);
            candidate.Depth = candidate.X * xDot + candidate.Y * yDot - negativeBaseDot;
        }

        //The wide version leaves unselected contacts uninitialized; zero init here just keeps mismatch reports stable. Existence flags gate all comparisons.
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

        //The select chains track the winning INDEX (as a float, so the select stays in the SIMD domain; exact for 0..8) instead of
        //blending the whole 16-byte candidate every iteration; the winner is copied once after the loop. The index chain updates on
        //exactly the same mask as the value chain did, so the selected candidate — and therefore every output bit — is unchanged.
        //While depth is the dominant heuristic, extremity is used as a bias to keep initial contact selection a little more consistent in near-equal cases.
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

    static void TestSat(Box a, Box b, Vector3 localOffsetB, in Matrix3x3 rB, float minimumDepth, out float depthResult, out Vector3 localNormalResult)
    {
        //Work in locals; writing intermediate select results through the out byrefs would force a store+reload round trip per select.
        //b.X
        TestEdgeEdge(
            a.HalfWidth, a.HalfHeight, a.HalfLength,
            b.HalfWidth, b.HalfHeight, b.HalfLength,
            localOffsetB.X, localOffsetB.Y, localOffsetB.Z,
            rB.X, rB.Y, rB.Z, rB.X,
            out var depth, out var localNormal);
        //b.Y
        TestEdgeEdge(
            a.HalfWidth, a.HalfHeight, a.HalfLength,
            b.HalfWidth, b.HalfHeight, b.HalfLength,
            localOffsetB.X, localOffsetB.Y, localOffsetB.Z,
            rB.X, rB.Y, rB.Z, rB.Y,
            out var edgeYDepth, out var edgeYNormal);
        Select(ref depth, ref localNormal, edgeYDepth, edgeYNormal);
        //b.Z
        TestEdgeEdge(
            a.HalfWidth, a.HalfHeight, a.HalfLength,
            b.HalfWidth, b.HalfHeight, b.HalfLength,
            localOffsetB.X, localOffsetB.Y, localOffsetB.Z,
            rB.X, rB.Y, rB.Z, rB.Z,
            out var edgeZDepth, out var edgeZNormal);
        Select(ref depth, ref localNormal, edgeZDepth, edgeZNormal);

        //The SAT depth is a running minimum: once it falls below the speculative margin it can never recover, and the caller
        //discards the normal on rejection, so bailing out here produces bit-identical output while skipping the remaining tests.
        //(The wide path cannot do this unless all lanes in the bundle miss — this is the structural per-pair early-out advantage of AoS.)
        if (depth < minimumDepth)
        {
            depthResult = depth;
            localNormalResult = localNormal;
            return;
        }

        //Test face normals of A. Working in local space of A means potential axes are just (1,0,0) etc.
        //All three depths are one Vector3 expression: per lane the operation sequence is
        //(((aHalf + bW * absRBX.c) + bH * absRBY.c) + bL * absRBZ.c) - |localOffsetB.c|, exactly the componentwise order.
        var absRBX = Vector3.Abs(rB.X);
        var absRBY = Vector3.Abs(rB.Y);
        var absRBZ = Vector3.Abs(rB.Z);
        var aHalfExtents = new Vector3(a.HalfWidth, a.HalfHeight, a.HalfLength);
        var faceADepths = aHalfExtents + b.HalfWidth * absRBX + b.HalfHeight * absRBY + b.HalfLength * absRBZ - Vector3.Abs(localOffsetB);
        Select(ref depth, ref localNormal, faceADepths.X, new Vector3(1f, 0f, 0f));
        Select(ref depth, ref localNormal, faceADepths.Y, new Vector3(0f, 1f, 0f));
        Select(ref depth, ref localNormal, faceADepths.Z, new Vector3(0f, 0f, 1f));
        if (depth < minimumDepth)
        {
            depthResult = depth;
            localNormalResult = localNormal;
            return;
        }

        //Test face normals of B. Rows of A->B rotation.
        //One transpose serves both the offset transform (broadcast-row form of the ordered dots — same per-component
        //operand order and associativity, so bitwise identical) and the face B columns (abs commutes with transposition).
        Matrix3x3.Transpose(rB, out var rBT);
        var bLocalOffsetB = localOffsetB.X * rBT.X + localOffsetB.Y * rBT.Y + localOffsetB.Z * rBT.Z;
        var colX = Vector3.Abs(rBT.X);
        var colY = Vector3.Abs(rBT.Y);
        var colZ = Vector3.Abs(rBT.Z);
        //Per lane: (((bHalf + aW * absRBrow.X) + aH * absRBrow.Y) + aL * absRBrow.Z) - |bLocalOffsetB.c|, exactly the componentwise order.
        var bHalfExtents = new Vector3(b.HalfWidth, b.HalfHeight, b.HalfLength);
        var faceBDepths = bHalfExtents + a.HalfWidth * colX + a.HalfHeight * colY + a.HalfLength * colZ - Vector3.Abs(bLocalOffsetB);
        Select(ref depth, ref localNormal, faceBDepths.X, rB.X);
        Select(ref depth, ref localNormal, faceBDepths.Y, rB.Y);
        Select(ref depth, ref localNormal, faceBDepths.Z, rB.Z);
        depthResult = depth;
        localNormalResult = localNormal;
    }

    public static void Test(
        in Box a, in Box b, float speculativeMargin,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        Unsafe.SkipInit(out manifold);
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 worldRA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 worldRB);
        //Share one transpose of worldRA between the world->A rotation product and the offset transform: the broadcast-row sums
        //below have the ordered dot's exact multiplicand order and add associativity per component, so results are bitwise identical.
        Matrix3x3.Transpose(worldRA, out var worldRAT);
        Unsafe.SkipInit(out Matrix3x3 rB);
        rB.X = worldRB.X.X * worldRAT.X + worldRB.X.Y * worldRAT.Y + worldRB.X.Z * worldRAT.Z;
        rB.Y = worldRB.Y.X * worldRAT.X + worldRB.Y.Y * worldRAT.Y + worldRB.Y.Z * worldRAT.Z;
        rB.Z = worldRB.Z.X * worldRAT.X + worldRB.Z.Y * worldRAT.Y + worldRB.Z.Z * worldRAT.Z;
        var localOffsetB = offsetB.X * worldRAT.X + offsetB.Y * worldRAT.Y + offsetB.Z * worldRAT.Z;

        var minimumDepth = -speculativeMargin;
        TestSat(a, b, localOffsetB, rB, minimumDepth, out var depth, out var localNormal);
        var allowContacts = depth >= minimumDepth;
        if (!allowContacts)
        {
            manifold.Contact0Exists = false;
            manifold.Contact1Exists = false;
            manifold.Contact2Exists = false;
            manifold.Contact3Exists = false;
            return;
        }

        //Calibrate the normal to point from B to A, matching convention.
        var normalDotOffsetB = ScalarMath.Dot(localNormal, localOffsetB);
        var shouldNegateNormal = ScalarMath.GreaterMask(normalDotOffsetB, 0f);
        localNormal = ScalarMath.Select(shouldNegateNormal, -localNormal, localNormal);
        Matrix3x3.Transform(localNormal, worldRA, out manifold.Normal);

        //Choose the representative face on each box; see wide version for commentary.
        //The three row dots are one broadcast-row expression against the transpose (same ordered-dot operand order per component).
        var aDots = manifold.Normal.X * worldRAT.X + manifold.Normal.Y * worldRAT.Y + manifold.Normal.Z * worldRAT.Z;
        var absADots = Vector3.Abs(aDots);
        var absAXDot = absADots.X;
        var absAYDot = absADots.Y;
        var absAZDot = absADots.Z;
        var maxADot = float.MaxNative(absAXDot, float.MaxNative(absAYDot, absAZDot));
        var useAXMask = ScalarMath.EqualMask(maxADot, absAXDot);
        //useAY = (maxADot == absAYDot) & !useAX; Vector128.AndNot(left, right) = left & ~right.
        var useAYMask = Vector128.AndNot(ScalarMath.EqualMask(maxADot, absAYDot), useAXMask);
        var normalA = ScalarMath.Select(useAXMask, worldRA.X, worldRA.Z);
        normalA = ScalarMath.Select(useAYMask, worldRA.Y, normalA);
        var tangentAX = ScalarMath.Select(useAXMask, worldRA.Z, worldRA.Y);
        tangentAX = ScalarMath.Select(useAYMask, worldRA.X, tangentAX);
        var tangentAY = ScalarMath.Select(useAXMask, worldRA.Y, worldRA.X);
        tangentAY = ScalarMath.Select(useAYMask, worldRA.Z, tangentAY);
        var halfSpanAX = ScalarMath.Select(useAXMask, a.HalfLength, ScalarMath.Select(useAYMask, a.HalfWidth, a.HalfHeight));
        var halfSpanAY = ScalarMath.Select(useAXMask, a.HalfHeight, ScalarMath.Select(useAYMask, a.HalfLength, a.HalfWidth));
        var halfSpanAZ = ScalarMath.Select(useAXMask, a.HalfWidth, ScalarMath.Select(useAYMask, a.HalfHeight, a.HalfLength));
        var useAX = maxADot == absAXDot;
        var useAY = maxADot == absAYDot & !useAX;
        //We'll construct vertex feature ids from axis ids.
        const int localXId = 1;
        const int localYId = 4;
        const int localZId = 16;
        var axisIdAX = ScalarMath.Select(useAX, localZId, ScalarMath.Select(useAY, localXId, localYId));
        var axisIdAY = ScalarMath.Select(useAX, localYId, ScalarMath.Select(useAY, localZId, localXId));
        var axisIdAZ = ScalarMath.Select(useAX, localXId, ScalarMath.Select(useAY, localYId, localZId));

        var bxDot = ScalarMath.Dot(manifold.Normal, worldRB.X);
        var byDot = ScalarMath.Dot(manifold.Normal, worldRB.Y);
        var bzDot = ScalarMath.Dot(manifold.Normal, worldRB.Z);
        var absBXDot = MathF.Abs(bxDot);
        var absBYDot = MathF.Abs(byDot);
        var absBZDot = MathF.Abs(bzDot);
        var maxBDot = float.MaxNative(absBXDot, float.MaxNative(absBYDot, absBZDot));
        var useBXMask = ScalarMath.EqualMask(maxBDot, absBXDot);
        var useBYMask = Vector128.AndNot(ScalarMath.EqualMask(maxBDot, absBYDot), useBXMask);
        var normalB = ScalarMath.Select(useBXMask, worldRB.X, worldRB.Z);
        normalB = ScalarMath.Select(useBYMask, worldRB.Y, normalB);
        var tangentBX = ScalarMath.Select(useBXMask, worldRB.Z, worldRB.Y);
        tangentBX = ScalarMath.Select(useBYMask, worldRB.X, tangentBX);
        var tangentBY = ScalarMath.Select(useBXMask, worldRB.Y, worldRB.X);
        tangentBY = ScalarMath.Select(useBYMask, worldRB.Z, tangentBY);
        var halfSpanBX = ScalarMath.Select(useBXMask, b.HalfLength, ScalarMath.Select(useBYMask, b.HalfWidth, b.HalfHeight));
        var halfSpanBY = ScalarMath.Select(useBXMask, b.HalfHeight, ScalarMath.Select(useBYMask, b.HalfLength, b.HalfWidth));
        var halfSpanBZ = ScalarMath.Select(useBXMask, b.HalfWidth, ScalarMath.Select(useBYMask, b.HalfHeight, b.HalfLength));
        var useBX = maxBDot == absBXDot;
        var useBY = maxBDot == absBYDot & !useBX;
        var axisIdBX = ScalarMath.Select(useBX, localZId, ScalarMath.Select(useBY, localXId, localYId));
        var axisIdBY = ScalarMath.Select(useBX, localYId, ScalarMath.Select(useBY, localZId, localXId));
        var axisIdBZ = ScalarMath.Select(useBX, localXId, ScalarMath.Select(useBY, localYId, localZId));

        //Calibrate normalB to face toward A, and normalA to face toward B.
        var calibrationDotA = ScalarMath.Dot(normalA, manifold.Normal);
        normalA = ScalarMath.Select(ScalarMath.GreaterMask(calibrationDotA, 0f), -normalA, normalA);
        var calibrationDotB = ScalarMath.Dot(normalB, manifold.Normal);
        normalB = ScalarMath.Select(ScalarMath.LessMask(calibrationDotB, 0f), -normalB, normalB);

        //Note that we only allocate up to 8 candidates. It is not possible for this process to generate more than 8.
        Span<CandidateAos> candidates = stackalloc CandidateAos[8];
        int candidateCount = 0;

        //Face B edges against face A bound planes
        var faceCenterA = normalA * halfSpanAZ;
        var faceCenterB = normalB * halfSpanBZ;
        faceCenterB += offsetB;
        var faceCenterBToFaceCenterA = faceCenterA - faceCenterB;
        var edgeOffsetAX = tangentAY * halfSpanAY;
        var edgeOffsetAY = tangentAX * halfSpanAX;

        var vertexA0 = faceCenterA - edgeOffsetAX;
        var vertexA00 = vertexA0 - edgeOffsetAY;
        var vertexA1 = faceCenterA + edgeOffsetAX;
        var vertexA11 = vertexA1 + edgeOffsetAY;

        var epsilonScale = float.MinNative(
            float.MaxNative(halfSpanAX, float.MaxNative(halfSpanAY, halfSpanAZ)),
            float.MaxNative(halfSpanBX, float.MaxNative(halfSpanBY, halfSpanBZ)));
        var twiceAxisIdBX = axisIdBX * 2;
        var axisZEdgeIdContribution = axisIdBZ * 3;
        var edgeIdBX0 = twiceAxisIdBX + axisIdBY + axisZEdgeIdContribution;
        var edgeIdBX1 = twiceAxisIdBX + axisIdBY * 3 + axisZEdgeIdContribution;
        var twiceAxisIdBY = axisIdBY * 2;
        var edgeIdBY0 = axisIdBX + twiceAxisIdBY + axisZEdgeIdContribution;
        var edgeIdBY1 = axisIdBX * 3 + twiceAxisIdBY + axisZEdgeIdContribution;
        CreateEdgeContacts(faceCenterB, tangentBX, tangentBY, halfSpanBX, halfSpanBY, vertexA00, vertexA11, tangentAX, tangentAY, manifold.Normal,
            edgeIdBX0, edgeIdBX1, edgeIdBY0, edgeIdBY1, epsilonScale, candidates, ref candidateCount);

        //Face A vertices. Note that the feature id is negated to disambiguate from edge contacts.
        var vertexId00 = -axisIdAZ;
        var vertexA01 = vertexA0 + edgeOffsetAY;
        var vertexId01 = -(axisIdAZ + axisIdAY);
        var vertexA10 = vertexA1 - edgeOffsetAY;
        var vertexId10 = -(axisIdAZ + axisIdAX);
        var vertexId11 = -(axisIdAZ + axisIdAX + axisIdAY);
        AddBoxAVertices(faceCenterB, tangentBX, tangentBY, halfSpanBX, halfSpanBY, normalB, manifold.Normal,
            vertexA00, vertexA01, vertexA10, vertexA11, vertexId00, vertexId01, vertexId10, vertexId11, candidates, ref candidateCount);

        Reduce(candidates, candidateCount, normalA, -1f / MathF.Abs(calibrationDotA), faceCenterBToFaceCenterA, tangentBX, tangentBY, epsilonScale, minimumDepth,
            out var contact0, out var contact1, out var contact2, out var contact3,
            out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

        //Transform the contacts into the manifold.
        TransformContactToManifold(contact0, faceCenterB, tangentBX, tangentBY, out manifold.OffsetA0, out manifold.Depth0, out manifold.FeatureId0);
        TransformContactToManifold(contact1, faceCenterB, tangentBX, tangentBY, out manifold.OffsetA1, out manifold.Depth1, out manifold.FeatureId1);
        TransformContactToManifold(contact2, faceCenterB, tangentBX, tangentBY, out manifold.OffsetA2, out manifold.Depth2, out manifold.FeatureId2);
        TransformContactToManifold(contact3, faceCenterB, tangentBX, tangentBY, out manifold.OffsetA3, out manifold.Depth3, out manifold.FeatureId3);
    }
}
