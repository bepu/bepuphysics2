using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace AosBaselines;

/// <summary>
/// FMA variant of BoxPairScalarTester: identical structure, but multiply-add chains use fused operations (single rounding).
/// NOT bitwise equal to the wide implementation — this exists to characterize what explicit FMA placement buys (perf) and
/// changes (ulp drift, tie flips at selection boundaries, asymmetric fused determinants). Fusion sites:
/// ordered dots (scalar fma chains), broadcast-row transforms, the packed SAT accumulations, face depth chains, raycasts,
/// Reduce's depth/extremity/distance computations, and — deliberately, as known-risk sites — the signed-area determinants
/// and cross products, where fusing one product but not the other breaks sign symmetry near zero.
/// </summary>
public static class BoxPairScalarTesterFma
{

    //Fused helpers. All produce single-rounded a*b+c results; placement is explicit and part of this variant's definition.
    static float DotFma(Vector3 a, Vector3 b)
        => MathF.FusedMultiplyAdd(a.Z, b.Z, MathF.FusedMultiplyAdd(a.Y, b.Y, a.X * b.X));

    static Vector128<float> MulAdd(float scalar, Vector3 v, Vector128<float> acc)
        => Vector128.FusedMultiplyAdd(Vector128.Create(scalar), v.AsVector128(), acc);

    static Vector3 TransformRows(float x, float y, float z, in Matrix3x3 rows)
        => MulAdd(z, rows.Z, MulAdd(y, rows.Y, Vector128.Create(x) * rows.X.AsVector128())).AsVector3();

    static Vector3 CrossFma(Vector3 a, Vector3 b)
    {
        //Asymmetric by construction: one product fused, one rounded. Characterization site.
        var av = a.AsVector128();
        var bv = b.AsVector128();
        var yzx = Vector128.Create(1, 2, 0, 3);
        var aYZX = Vector128.Shuffle(av, yzx);
        var bYZX = Vector128.Shuffle(bv, yzx);
        var t = Vector128.FusedMultiplyAdd(av, bYZX, -(aYZX * bv));
        return Vector128.Shuffle(t, yzx).AsVector3();
    }
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
            //lengthSquared lanes = fma(c1, c1, c2 * c2) with (c1, c2) = (y,z), (x,z), (x,y).
            var lsA = Vector128.Shuffle(e, Vector128.Create(1, 0, 0, 3));
            var lsB = Vector128.Shuffle(e, Vector128.Create(2, 2, 1, 3));
            var lengthSquared = Vector128.FusedMultiplyAdd(lsA, lsA, lsB * lsB);
            var length = Vector128.Sqrt(lengthSquared);
            var inverseLength = Vector128.Create(1f) / length;
            var lane3Zero = Vector128.Create(-1, -1, -1, 0).AsSingle();
            //Candidate normal components: p = first nonzero component per block = (eZ, eZ, eY) * il; q = second = (-eY, -eX, -eX) * il.
            var p = (Vector128.Shuffle(e, Vector128.Create(2, 2, 1, 3)) * inverseLength) & lane3Zero;
            var q = ((Vector128.Shuffle(e, Vector128.Create(1, 0, 0, 3)) ^ Vector128.Create(-0.0f)) * inverseLength) & lane3Zero;
            var absMask = Vector128.Create(0x7FFFFFFF).AsSingle();
            var extremeA = Vector128.FusedMultiplyAdd(p & absMask, Vector128.Create(halfHeightA, halfWidthA, halfWidthA, 0f),
                (q & absMask) * Vector128.Create(halfLengthA, halfLengthA, halfHeightA, 0f));
            //nB_c = p * rBc[firstAxis] + q * rBc[secondAxis]; per block the axis pairs are (Y,Z), (X,Z), (X,Y).
            var firstAxes = Vector128.Create(1, 0, 0, 3);
            var secondAxes = Vector128.Create(2, 2, 1, 3);
            var rbxv = rBX.AsVector128();
            var rbyv = rBY.AsVector128();
            var rbzv = rBZ.AsVector128();
            var nBX = Vector128.FusedMultiplyAdd(p, Vector128.Shuffle(rbxv, firstAxes), q * Vector128.Shuffle(rbxv, secondAxes));
            var nBY = Vector128.FusedMultiplyAdd(p, Vector128.Shuffle(rbyv, firstAxes), q * Vector128.Shuffle(rbyv, secondAxes));
            var nBZ = Vector128.FusedMultiplyAdd(p, Vector128.Shuffle(rbzv, firstAxes), q * Vector128.Shuffle(rbzv, secondAxes));
            var extremeB = Vector128.FusedMultiplyAdd(nBZ & absMask, Vector128.Create(halfLengthB),
                Vector128.FusedMultiplyAdd(nBY & absMask, Vector128.Create(halfHeightB),
                (nBX & absMask) * Vector128.Create(halfWidthB)));
            var offsetDot = Vector128.FusedMultiplyAdd(Vector128.Create(offsetBY, offsetBX, offsetBX, 0f), p,
                Vector128.Create(offsetBZ, offsetBZ, offsetBY, 0f) * q);
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
        var planeDistance = DotFma(faceNormalB, pointOnBToVertex);
        var t = planeDistance * inverseContactNormalDotFaceNormalB;
        //Contact normal points from B to A by convention, so we have to subtract; fused as vertex + normal * -t.
        var vertexOnBFace = MulAdd(-t, contactNormal, vertex.AsVector128()).AsVector3();

        var vertexOffsetOnBFace = vertexOnBFace - faceCenterB;
        Unsafe.SkipInit(out CandidateAos candidate);
        candidate.X = DotFma(vertexOffsetOnBFace, faceTangentBX);
        candidate.Y = DotFma(vertexOffsetOnBFace, faceTangentBY);
        candidate.FeatureId = featureId;

        var contained = MathF.Abs(candidate.X) <= halfSpanBX & MathF.Abs(candidate.Y) <= halfSpanBY;
        //Wide version clamps the candidate count against the buffer capacity explicitly; mirror it.
        //Branchless append: store unconditionally (slot 8 is a trash bin), advance count by the flag.
        var append = contained & candidateCount < 8;
        Unsafe.Add(ref MemoryMarshal.GetReference(candidates), candidateCount) = candidate;
        candidateCount += Unsafe.As<bool, byte>(ref append);
    }

    //Phase methods are deliberately not inlined: the tester in one piece exhausts the JIT's inlining budget, at which point
    //even Vector3 operator intrinsics degrade into real calls with struct-return ABI. Splitting restores a full budget per phase.
    [MethodImpl(MethodImplOptions.NoInlining)]
    static void AddBoxAVertices(in Vector3 faceCenterB, in Vector3 faceTangentBX, in Vector3 faceTangentBY, float halfSpanBX, float halfSpanBY,
        in Vector3 faceNormalB, in Vector3 contactNormal,
        in Vector3 v00, in Vector3 v01, in Vector3 v10, in Vector3 v11,
        int f00, int f01, int f10, int f11,
        Span<CandidateAos> candidates, ref int candidateCount)
    {
        var normalDot = DotFma(faceNormalB, contactNormal);
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
        var distance00 = DotFma(edgeStartB0ToEdgeAnchorA00, boxEdgePlaneNormal);
        var distance01 = DotFma(edgeStartB0ToEdgeAnchorA11, boxEdgePlaneNormal);
        var distance10 = DotFma(edgeStartB1ToEdgeAnchorA00, boxEdgePlaneNormal);
        var distance11 = DotFma(edgeStartB1ToEdgeAnchorA11, boxEdgePlaneNormal);
        var velocity = DotFma(boxEdgePlaneNormal, edgeDirection);
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
        //Branchless appends; see the candidate buffer's comment. At most 8 edge contacts can exist, so the blind store
        //index never exceeds the trash slot.
        ref var candidatesBase = ref MemoryMarshal.GetReference(candidates);
        var minExists = max - min > epsilon & MathF.Abs(min) < halfSpanB;
        Unsafe.Add(ref candidatesBase, candidateCount) = minCandidate;
        candidateCount += Unsafe.As<bool, byte>(ref minExists);

        var maxExists = max >= min & MathF.Abs(max) <= halfSpanB;
        Unsafe.Add(ref candidatesBase, candidateCount) = maxCandidate;
        candidateCount += Unsafe.As<bool, byte>(ref maxExists);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static void CreateEdgeContacts(
        in Vector3 faceCenterB, in Vector3 faceTangentBX, in Vector3 faceTangentBY, float halfSpanBX, float halfSpanBY,
        in Vector3 vertexA00, in Vector3 vertexA11, in Vector3 faceTangentAX, in Vector3 faceTangentAY, in Vector3 contactNormal,
        int featureIdX0, int featureIdX1, int featureIdY0, int featureIdY1,
        float epsilonScale, Span<CandidateAos> candidates, ref int candidateCount)
    {
        //Clip on the contact normal plane; see the wide version for commentary.
        var edgePlaneNormalAX = CrossFma(faceTangentAY, contactNormal);
        var edgePlaneNormalAY = CrossFma(faceTangentAX, contactNormal);

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
        in Vector3 faceNormalA, float inverseFaceNormalADotNormal, in Vector3 faceCenterBToFaceCenterA, in Vector3 tangentBX, in Vector3 tangentBY,
        float epsilonScale, float minimumDepth,
        out CandidateAos contact0, out CandidateAos contact1, out CandidateAos contact2, out CandidateAos contact3,
        out bool contact0Exists, out bool contact1Exists, out bool contact2Exists, out bool contact3Exists)
    {
        //ComputeDepthsForReduction: cast a ray from the point on face B toward the plane of face A along the contact normal.
        var dotAxis = faceNormalA * inverseFaceNormalADotNormal;
        var negativeBaseDot = DotFma(faceCenterBToFaceCenterA, dotAxis);
        var xDot = DotFma(tangentBX, dotAxis);
        var yDot = DotFma(tangentBY, dotAxis);
        //candidateCount never exceeds the span length by construction; index through a base ref to skip per-iteration bounds checks.
        ref var candidatesBase = ref MemoryMarshal.GetReference(candidates);
        var negatedBaseDot = -negativeBaseDot;
        for (int i = 0; i < candidateCount; ++i)
        {
            ref var candidate = ref Unsafe.Add(ref candidatesBase, i);
            candidate.Depth = MathF.FusedMultiplyAdd(candidate.X, xDot, MathF.FusedMultiplyAdd(candidate.Y, yDot, negatedBaseDot));
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
            var extremity = MathF.Abs(MathF.FusedMultiplyAdd(candidate.X, 0.7946897654f, candidate.Y * 0.60701579614f));
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
            var distanceSquared = MathF.FusedMultiplyAdd(offsetX, offsetX, offsetY * offsetY);
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
            //Asymmetric fused determinant: characterization site for sign flips near zero.
            var signedArea = MathF.FusedMultiplyAdd(candidateOffsetX, edgeOffsetY, -(candidateOffsetY * edgeOffsetX));
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
        manifoldOffsetA = MulAdd(rawContact.X, tangentBX, MulAdd(rawContact.Y, tangentBY, faceCenterB.AsVector128())).AsVector3();
        manifoldDepth = rawContact.Depth;
        manifoldFeatureId = rawContact.FeatureId;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static void TestSat(in Box a, in Box b, in Vector3 localOffsetB, in Matrix3x3 rB, float minimumDepth, out float depthResult, out Vector3 localNormalResult)
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
        var faceADepths = (MulAdd(b.HalfLength, absRBZ, MulAdd(b.HalfHeight, absRBY, MulAdd(b.HalfWidth, absRBX, aHalfExtents.AsVector128())))
            - Vector3.Abs(localOffsetB).AsVector128()).AsVector3();
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
        var bLocalOffsetB = TransformRows(localOffsetB.X, localOffsetB.Y, localOffsetB.Z, rBT);
        var colX = Vector3.Abs(rBT.X);
        var colY = Vector3.Abs(rBT.Y);
        var colZ = Vector3.Abs(rBT.Z);
        //Per lane: (((bHalf + aW * absRBrow.X) + aH * absRBrow.Y) + aL * absRBrow.Z) - |bLocalOffsetB.c|, exactly the componentwise order.
        var bHalfExtents = new Vector3(b.HalfWidth, b.HalfHeight, b.HalfLength);
        var faceBDepths = (MulAdd(a.HalfLength, colZ, MulAdd(a.HalfHeight, colY, MulAdd(a.HalfWidth, colX, bHalfExtents.AsVector128())))
            - Vector3.Abs(bLocalOffsetB).AsVector128()).AsVector3();
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
        rB.X = TransformRows(worldRB.X.X, worldRB.X.Y, worldRB.X.Z, worldRAT);
        rB.Y = TransformRows(worldRB.Y.X, worldRB.Y.Y, worldRB.Y.Z, worldRAT);
        rB.Z = TransformRows(worldRB.Z.X, worldRB.Z.Y, worldRB.Z.Z, worldRAT);
        var localOffsetB = TransformRows(offsetB.X, offsetB.Y, offsetB.Z, worldRAT);

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
        var normalDotOffsetB = DotFma(localNormal, localOffsetB);
        var shouldNegateNormal = ScalarMath.GreaterMask(normalDotOffsetB, 0f);
        localNormal = ScalarMath.Select(shouldNegateNormal, -localNormal, localNormal);
        Matrix3x3.Transform(localNormal, worldRA, out manifold.Normal);

        //Choose the representative face on each box; see wide version for commentary.
        //The three row dots are one broadcast-row expression against the transpose (same ordered-dot operand order per component).
        var aDots = TransformRows(manifold.Normal.X, manifold.Normal.Y, manifold.Normal.Z, worldRAT);
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

        var bxDot = DotFma(manifold.Normal, worldRB.X);
        var byDot = DotFma(manifold.Normal, worldRB.Y);
        var bzDot = DotFma(manifold.Normal, worldRB.Z);
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
        var calibrationDotA = DotFma(normalA, manifold.Normal);
        normalA = ScalarMath.Select(ScalarMath.GreaterMask(calibrationDotA, 0f), -normalA, normalA);
        var calibrationDotB = DotFma(normalB, manifold.Normal);
        normalB = ScalarMath.Select(ScalarMath.LessMask(calibrationDotB, 0f), -normalB, normalB);

        //Note that at most 8 candidates can be generated. The ninth slot is a trash bin: appends store unconditionally and
        //advance the count by the exists flag, so rejected (or capacity-clamped) candidates land in a slot that is either
        //overwritten by the next append or sits at index 8 and is never read. This keeps the append path branchless —
        //the exists flags are ~coin-flip in contact-heavy workloads, and mispredicted branches cost more than the blind store.
        Span<CandidateAos> candidates = stackalloc CandidateAos[9];
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
