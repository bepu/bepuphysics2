using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace AosBaselines;

/// <summary>
/// Scalar support function mirror for a shape. Mirrors the per-lane body of the corresponding wide ISupportFinder.
/// </summary>
public interface IScalarSupportFinder<TShape>
{
    //Vector3 arguments and results pass by value: byref parameters (in/out) take addresses, which address-exposes the caller's
    //SIMD loop locals whether or not the call inlines — the direct cause of per-iteration store/reload dances in the refiner loop.
    static abstract Vector3 ComputeLocalSupport(in TShape shape, Vector3 direction);

    /// <summary>
    /// Mirrors the wide finders' oriented ComputeSupport. Most shapes rotate into local space, sample, and rotate back;
    /// shapes whose wide finder skips that dance (like spheres, which just return zero) do the same here.
    /// Takes the orientation's transpose alongside it so a loop caller can transpose once: transforming by the transpose in
    /// broadcast-row form is bitwise identical to the ordered horizontal dots (same add association; multiplies commuted).
    /// </summary>
    static abstract Vector3 ComputeSupport(in TShape shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction);

    static virtual bool HasMargin => false;
    static virtual float GetMargin(in TShape shape) => 0f;
}

/// <summary>
/// Mirrors the per-lane body of ConvexHullSupportFinder.ComputeLocalSupport: broadcasts the direction and sweeps the hull's own point bundles.
/// The within-hull vectorization is identical to the wide path's, so per-lane results match bitwise.
/// </summary>
public struct HullSupportScalar : IScalarSupportFinder<ConvexHull>
{
    public static Vector3 ComputeLocalSupport(in ConvexHull hull, Vector3 direction)
    {
        Helpers.FillVectorWithLaneIndices(out var indexOffsets);
        Vector3Wide.Broadcast(direction, out var slotDirection);
        var bestIndices = indexOffsets;
        Vector3Wide.Dot(slotDirection, hull.Points[0], out var dot);
        for (int j = 1; j < hull.Points.Length; ++j)
        {
            ref var candidate = ref hull.Points[j];
            Vector3Wide.Dot(slotDirection, candidate, out var dotCandidate);
            var useCandidate = Vector.GreaterThan(dotCandidate, dot);
            bestIndices = Vector.ConditionalSelect(useCandidate, indexOffsets + new Vector<int>(j << BundleIndexing.VectorShift), bestIndices);
            dot = Vector.ConditionalSelect(useCandidate, dotCandidate, dot);
        }
        var bestSlotIndex = 0;
        var bestSlotDot = dot[0];
        for (int j = 1; j < Vector<float>.Count; ++j)
        {
            var candidate = dot[j];
            if (candidate > bestSlotDot)
            {
                bestSlotDot = candidate;
                bestSlotIndex = j;
            }
        }
        var supportIndex = bestIndices[bestSlotIndex];
        BundleIndexing.GetBundleIndices(supportIndex, out var bundleIndex, out var innerIndex);
        Vector3Wide.ReadSlot(ref hull.Points[bundleIndex], innerIndex, out var support);
        return support;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in ConvexHull shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        Matrix3x3.Transform(direction, orientationTranspose, out var localDirection);
        var localSupport = ComputeLocalSupport(shape, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }
}

/// <summary>
/// Mirrors SphereSupportFinder: a sphere is a zero point with its radius as margin.
/// </summary>
public struct SphereSupportScalar : IScalarSupportFinder<Sphere>
{
    public static bool HasMargin => true;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float GetMargin(in Sphere shape) => shape.Radius;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeLocalSupport(in Sphere shape, Vector3 direction)
    {
        return default;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in Sphere shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        return default;
    }
}

/// <summary>
/// Mirrors the per-lane semantics of CylinderSupportFinder.ComputeLocalSupport.
/// </summary>
public struct CylinderSupportScalar : IScalarSupportFinder<Cylinder>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeLocalSupport(in Cylinder shape, Vector3 direction)
    {
        //Promotion-friendly form: build whole candidate vectors and bit-blend on the masks. Per-component arithmetic is identical
        //to the componentwise ternary original (blends are bitwise selects), so results match bitwise.
        var y = ScalarMath.Select(ScalarMath.GreaterMask(direction.Y, 0f), shape.HalfLength, -shape.HalfLength);
        //Whole-vector square, then extract: the single add sees the same two square values in the same order as the
        //componentwise form, so the result is bit-identical while the multiply runs as one vmulps.
        var squared = direction * direction;
        var horizontalLength = MathF.Sqrt(squared.X + squared.Z);
        var normalizeScale = shape.Radius / horizontalLength;
        var useHorizontal = ScalarMath.GreaterMask(horizontalLength, 1e-8f);
        return ScalarMath.Select(useHorizontal, new Vector3(direction.X * normalizeScale, y, direction.Z * normalizeScale), new Vector3(0f, y, 0f));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in Cylinder shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        Matrix3x3.Transform(direction, orientationTranspose, out var localDirection);
        var localSupport = ComputeLocalSupport(shape, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }
}

/// <summary>
/// Mirrors CapsuleSupportFinder: a segment with the radius as a margin.
/// </summary>
public struct CapsuleSupportScalar : IScalarSupportFinder<Capsule>
{
    public static bool HasMargin => true;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float GetMargin(in Capsule shape) => shape.Radius;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeLocalSupport(in Capsule shape, Vector3 direction)
    {
        return new Vector3(0f, ScalarMath.Select(ScalarMath.LessMask(direction.Y, 0f), -shape.HalfLength, shape.HalfLength), 0f);
    }

    /// <summary>
    /// Mirrors the wide finder, which never rotates into local space: support = ±(orientation.Y * halfLength) by the sign of dot(orientation.Y, direction).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in Capsule shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        var support = orientation.Y * shape.HalfLength;
        var dot = ScalarMath.Dot(orientation.Y, direction);
        return ScalarMath.Select(ScalarMath.LessMask(dot, 0f), -support, support);
    }
}

/// <summary>
/// Mirrors the per-lane semantics of BoxSupportFinder.
/// </summary>
public struct BoxSupportScalar : IScalarSupportFinder<Box>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeLocalSupport(in Box shape, Vector3 direction)
    {
        return new Vector3(
            ScalarMath.Select(ScalarMath.LessMask(direction.X, 0f), -shape.HalfWidth, shape.HalfWidth),
            ScalarMath.Select(ScalarMath.LessMask(direction.Y, 0f), -shape.HalfHeight, shape.HalfHeight),
            ScalarMath.Select(ScalarMath.LessMask(direction.Z, 0f), -shape.HalfLength, shape.HalfLength));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in Box shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        Matrix3x3.Transform(direction, orientationTranspose, out var localDirection);
        var localSupport = ComputeLocalSupport(shape, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }
}

/// <summary>
/// Mirrors PretransformedTriangleSupportFinder: the triangle's vertices already live in the caller's space, so ComputeSupport
/// ignores the orientation entirely and the local variant is unimplemented (matching the wide finder's NotImplementedException).
/// </summary>
public struct PretransformedTriangleSupportScalar : IScalarSupportFinder<Triangle>
{
    public static Vector3 ComputeLocalSupport(in Triangle shape, Vector3 direction) => throw new NotImplementedException();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in Triangle shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        var a = ScalarMath.Dot(shape.A, direction);
        var b = ScalarMath.Dot(shape.B, direction);
        var c = ScalarMath.Dot(shape.C, direction);
        //MathF.Max mirrors Vector.Max's IEEE semantics exactly (dots can be +-0; the equality selects below would mask a zero-sign
        //difference anyway since +0 == -0, but the exact mirror costs nothing here).
        var max = MathF.Max(a, MathF.Max(b, c));
        var support = ScalarMath.Select(ScalarMath.EqualMask(max, a), shape.A, shape.B);
        return ScalarMath.Select(ScalarMath.EqualMask(max, c), shape.C, support);
    }
}

public struct VertexScalar
{
    //16-byte slots: a Vector3 field in a byref-held struct loads as vmovsd+vinsertps and stores as the reverse; a Vector128
    //field is a single vmovups each way. Lane W is padding (garbage); all arithmetic runs on AsVector3 views, so it never
    //participates in results. No arithmetic changes — data movement only.
    public Vector128<float> Support;
    public Vector128<float> SupportOnA;
    public float Weight;
    public bool Exists;
}

public struct SimplexScalar
{
    public VertexScalar A;
    public VertexScalar B;
    public VertexScalar C;
    public float WeightDenominator;
}

/// <summary>
/// Scalar AoS port of DepthRefiner's witness variant for margin-free shapes.
/// The wide implementation freezes per-lane state on termination (required for bundle-independence/determinism),
/// so a scalar transliteration that stops when its own lane terminates produces bitwise identical results.
/// </summary>
public static class ScalarDepthRefiner<TShapeA, TSupportFinderA, TShapeB, TSupportFinderB>
    where TSupportFinderA : IScalarSupportFinder<TShapeA>
    where TSupportFinderB : IScalarSupportFinder<TShapeB>
{
    //Diagnostics: tracks how many refiner invocations and support-sampling iterations have run for this shape pair instantiation.
    //Compiled out by default: the memory increment per loop iteration sits in the timed path and the wide implementation has no
    //counterpart, so it distorts benchmarks. Flip CollectStats to true (and rebuild) to use the stats mode.
    public const bool CollectStats = false;
    public static long TotalCalls;
    public static long TotalIterations;
    //Branch-condition bias counters (per GetNextNormal invocation), for judging which mask selects would be predictable branches.
    public static long StatSimplexFull, StatFillA, StatFillB, StatFillC, StatSubABD, StatSubBCD, StatSubCAD, StatSubFallback;
    public static long StatOutsideEdges, StatDegenerate, StatIsVertex, StatUseEdge, StatEdgeAB, StatEdgeBC, StatEdgeCA;
    public static long StatTStart, StatTEnd, StatTInterior, StatTargetContained, StatCalibrationNegate, StatBestDepthNegative, StatPushCandidate;
    public static long StatTerminated;

    static bool Any(Vector128<float> mask) => mask.AsInt32().ToScalar() != 0;
    //Neither fill helper writes Exists anymore: every GetNextNormal invocation unconditionally overwrites all three Exists flags
    //from relevantFeatures before returning, so the old set-true/or stores were dead. The fill decision reads the previous
    //invocation's (or the initial simplex's) Exists value, which is unchanged by this.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void FillSlot(ref VertexScalar vertex, Vector128<float> support, Vector128<float> supportOnA)
    {
        var fill = ScalarMath.Mask(!vertex.Exists);
        vertex.Support = Vector128.ConditionalSelect(fill, support, vertex.Support);
        vertex.SupportOnA = Vector128.ConditionalSelect(fill, supportOnA, vertex.SupportOnA);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ForceFillSlot(Vector128<float> shouldFill, ref VertexScalar vertex, Vector128<float> support, Vector128<float> supportOnA)
    {
        vertex.Support = Vector128.ConditionalSelect(shouldFill, support, vertex.Support);
        vertex.SupportOnA = Vector128.ConditionalSelect(shouldFill, supportOnA, vertex.SupportOnA);
    }

    //hasNewSupport is gone: the first (no-new-support) invocation is specialized inline in FindMinimumDepth, so this body
    //always runs the new-support path.
    static Vector3 GetNextNormal(ref SimplexScalar simplex, Vector3 support, Vector3 supportOnA, ref bool terminated,
        Vector3 bestNormal, float bestDepth, float convergenceThreshold)
    {
        //The caller only reads the returned normal when the lane has not terminated; default matches the old undefined-out contract.
        Vector3 nextNormal = default;
        //The search target is the closest point to the origin on the so-far-best bounding plane, also known as the tootbird.
        //On .NET 10, Vector.Max/Min carry IEEE maximum/minimum semantics (probe: bitwise identical to MathF.Max/Min for +-0 and
        //NaN payloads), NOT the maxss-style (a OP b) ? a : b of float.MaxNative. bestDepth can be -0 (it's a dot product), and
        //Vector.Max(+0, -0) = +0 while MaxNative(+0, -0) = -0, so the mirror must be MathF.Max. This was caught by capsule-hull
        //fuzzing: capsule supports carry exact +-0 components that flow into zero dots, unlike the shapes fuzzed before.
        var searchTarget = bestNormal * MathF.Max(0f, bestDepth);
        var terminationEpsilon = ScalarMath.Select(ScalarMath.LessMask(bestDepth, 0f), convergenceThreshold - bestDepth, convergenceThreshold);
        var terminationEpsilonSquared = terminationEpsilon * terminationEpsilon;

        var support128 = support.AsVector128Unsafe();
        var supportOnA128 = supportOnA.AsVector128Unsafe();
        {
            var simplexFull = simplex.A.Exists & simplex.B.Exists & simplex.C.Exists;
            if (CollectStats)
            {
                if (simplexFull) ++StatSimplexFull;
                if (!simplex.A.Exists) ++StatFillA;
                if (!simplex.B.Exists) ++StatFillB;
                if (!simplex.C.Exists) ++StatFillC;
            }
            //Fill any empty slots with the new support. Combines partial simplex case with degenerate simplex case.
            //Fills live in the else of the existing simplexFull branch: when the simplex is full every FillSlot is a provable
            //no-op (fill mask false), so skipping them is output-identical and saves six blend+stores in the ~70%-frequent
            //full case (condition stats 2026-08-13) without adding any new branch.
            if (!simplexFull)
            {
                FillSlot(ref simplex.A, support128, supportOnA128);
                FillSlot(ref simplex.B, support128, supportOnA128);
                FillSlot(ref simplex.C, support128, supportOnA128);
            }
            else
            {
                //Choose the subtriangle based on the edge plane tests of AD, BD, and CD, where D is the new support point.
                var supportA = simplex.A.Support.AsVector3();
                var supportB = simplex.B.Support.AsVector3();
                var supportC = simplex.C.Support.AsVector3();
                var abEarly = supportB - supportA;
                var caEarly = supportA - supportC;
                var ad = support - supportA;
                var bd = support - supportB;
                var cd = support - supportC;
                var triangleNormalEarly = Vector3.Cross(abEarly, caEarly);
                var targetToSupport = support - searchTarget;
                var nxOffset = Vector3.Cross(triangleNormalEarly, targetToSupport);
                var adPlaneTest = ScalarMath.Dot(nxOffset, ad);
                var bdPlaneTest = ScalarMath.Dot(nxOffset, bd);
                var cdPlaneTest = ScalarMath.Dot(nxOffset, cd);

                var adPlaneTestNonNegative = ScalarMath.GreaterOrEqualMask(adPlaneTest, 0f);
                var bdPlaneTestNonNegative = ScalarMath.GreaterOrEqualMask(bdPlaneTest, 0f);
                var cdPlaneTestNonNegative = ScalarMath.GreaterOrEqualMask(cdPlaneTest, 0f);
                var useABD = adPlaneTestNonNegative & ScalarMath.LessMask(bdPlaneTest, 0f);
                var useBCD = bdPlaneTestNonNegative & ScalarMath.LessMask(cdPlaneTest, 0f);
                var useCAD = cdPlaneTestNonNegative & ScalarMath.LessMask(adPlaneTest, 0f);

                //Fallback for the rare case where no subtriangle contains the best normal; see the wide implementation's commentary.
                //Branchless mask form of: if none set, use ABD.
                if (CollectStats)
                {
                    if (!Any(useABD | useBCD | useCAD)) ++StatSubFallback;
                    else if (Any(useABD)) ++StatSubABD;
                    else if (Any(useBCD)) ++StatSubBCD;
                    else ++StatSubCAD;
                }
                useABD |= ~(useABD | useBCD | useCAD);

                ForceFillSlot(useBCD, ref simplex.A, support128, supportOnA128);
                ForceFillSlot(useCAD, ref simplex.B, support128, supportOnA128);
                ForceFillSlot(useABD, ref simplex.C, support128, supportOnA128);
                //(masks carry the same conditions as the old bools; the selects are bit-identical.)
            }
        }
        var simplexSupportA = simplex.A.Support.AsVector3();
        var simplexSupportB = simplex.B.Support.AsVector3();
        var simplexSupportC = simplex.C.Support.AsVector3();
        var ab = simplexSupportB - simplexSupportA;
        var ca = simplexSupportA - simplexSupportC;
        var bc = simplexSupportC - simplexSupportB;
        var triangleNormal = Vector3.Cross(ab, ca);
        var triangleNormalLengthSquared = triangleNormal.LengthSquared();

        //Compute the plane sign tests. These are unnormalized barycentric weights; only the signs matter for the containment tests.
        var targetToA = simplexSupportA - searchTarget;
        var targetToC = simplexSupportC - searchTarget;
        var abxta = Vector3.Cross(ab, targetToA);
        var caxtc = Vector3.Cross(ca, targetToC);
        var abPlaneTest = ScalarMath.Dot(abxta, triangleNormal);
        var caPlaneTest = ScalarMath.Dot(caxtc, triangleNormal);
        var bcPlaneTest = triangleNormalLengthSquared - caPlaneTest - abPlaneTest;
        var outsideAB = abPlaneTest < 0f;
        var outsideBC = bcPlaneTest < 0f;
        var outsideCA = caPlaneTest < 0f;

        var abLengthSquared = ab.LengthSquared();
        var bcLengthSquared = bc.LengthSquared();
        var caLengthSquared = ca.LengthSquared();
        //MaxNative is safe here (unlike the +-0-reachable clamps below): sums of squares are never -0 or NaN, and
        //MaxNative only diverges from Vector.Max's IEEE semantics on those inputs.
        var longestEdgeLengthSquared = float.MaxNative(float.MaxNative(abLengthSquared, bcLengthSquared), caLengthSquared);
        var simplexDegenerate = triangleNormalLengthSquared <= longestEdgeLengthSquared * 1e-10f;
        const float degeneracyEpsilon = 1e-14f;
        var simplexIsAVertex = longestEdgeLengthSquared < degeneracyEpsilon;
        var simplexIsAnEdge = simplexDegenerate & !simplexIsAVertex;

        var calibrationDot = ScalarMath.Dot(triangleNormal, bestNormal);
        //Whole-vector negate-select: one xor+blend, per-component bitwise identical to the componentwise selects.
        triangleNormal = ScalarMath.Select(ScalarMath.LessMask(calibrationDot, 0f), -triangleNormal, triangleNormal);

        var targetOutsideTriangleEdges = outsideAB | outsideBC | outsideCA;

        var triangleToTarget = -targetToA;

        var relevantFeatures = 1;
        simplex.A.Weight = 1f;
        simplex.B.Weight = 0f;
        simplex.C.Weight = 0f;
        simplex.WeightDenominator = 1f;

        //If this is a vertex case and the sample is right on top of the target, immediately quit.
        var targetToALengthSquared = targetToA.LengthSquared();
        if (simplexIsAVertex & targetToALengthSquared < terminationEpsilonSquared)
            terminated = true;

        if (CollectStats)
        {
            if (targetOutsideTriangleEdges) ++StatOutsideEdges;
            if (simplexDegenerate) ++StatDegenerate;
            if (simplexIsAVertex) ++StatIsVertex;
            if (Any(ScalarMath.LessMask(calibrationDot, 0f))) ++StatCalibrationNegate;
            if (Any(ScalarMath.LessMask(bestDepth, 0f))) ++StatBestDepthNegative;
        }
        var useEdge = (targetOutsideTriangleEdges | simplexIsAnEdge) & !terminated;
        if (useEdge)
        {
            //Choose the edge that is closest to the search target; see the wide implementation for the derivation.
            var inverseABLengthSquared = 1f / abLengthSquared;
            var inverseBCLengthSquared = 1f / bcLengthSquared;
            var inverseCALengthSquared = 1f / caLengthSquared;
            var targetToB = simplexSupportB - searchTarget;
            var oaDotAB = ScalarMath.Dot(targetToA, ab);
            var obDotBC = ScalarMath.Dot(targetToB, bc);
            var ocDotCA = ScalarMath.Dot(targetToC, ca);
            //MathF.Max/Min mirror Vector.Max/Min's IEEE semantics exactly (see searchTarget note); the negated dots can be -0,
            //and Max(0, -0) must be +0 to match the wide clamp.
            var abScaledT = MathF.Max(0f, MathF.Min(abLengthSquared, -oaDotAB));
            var bcScaledT = MathF.Max(0f, MathF.Min(bcLengthSquared, -obDotBC));
            var caScaledT = MathF.Max(0f, MathF.Min(caLengthSquared, -ocDotCA));
            var abT = abScaledT * inverseABLengthSquared;
            var bcT = bcScaledT * inverseBCLengthSquared;
            var caT = caScaledT * inverseCALengthSquared;
            var abScaledEdgeOffset = ab * abT;
            var bcScaledEdgeOffset = bc * bcT;
            var caScaledEdgeOffset = ca * caT;
            var abClosestOffset = targetToA + abScaledEdgeOffset;
            var bcClosestOffset = targetToB + bcScaledEdgeOffset;
            var caClosestOffset = targetToC + caScaledEdgeOffset;
            var abDistanceSquared = abClosestOffset.LengthSquared();
            var bcDistanceSquared = bcClosestOffset.LengthSquared();
            var caDistanceSquared = caClosestOffset.LengthSquared();

            var bcDegenerate = ScalarMath.EqualMask(bcLengthSquared, 0f);
            var caDegenerate = ScalarMath.EqualMask(caLengthSquared, 0f);
            var abCloserThanBC = bcDegenerate | ScalarMath.LessMask(abDistanceSquared, bcDistanceSquared);
            var abCloserThanCA = caDegenerate | ScalarMath.LessMask(abDistanceSquared, caDistanceSquared);
            var bcCloserThanCA = caDegenerate | ScalarMath.LessMask(bcDistanceSquared, caDistanceSquared);

            var useAB = abCloserThanBC & abCloserThanCA;
            var useBC = bcCloserThanCA & ~useAB;

            var bestDistanceSquared = ScalarMath.Select(useAB, abDistanceSquared, ScalarMath.Select(useBC, bcDistanceSquared, caDistanceSquared));

            //If the search target is on the edge, we can immediately quit.
            if (bestDistanceSquared <= terminationEpsilonSquared)
                terminated = true;
            {
                //Note that this block executes even if the lane just terminated at the distance check above, mirroring the wide version's masking.
                var t = ScalarMath.Select(useAB, abT, ScalarMath.Select(useBC, bcT, caT));
                var edgeOffset = ScalarMath.Select(useAB, ab, ca);
                edgeOffset = ScalarMath.Select(useBC, bc, edgeOffset);
                var edgeStart = ScalarMath.Select(useAB, targetToA, targetToC);
                edgeStart = ScalarMath.Select(useBC, targetToB, edgeStart);

                var scaledOffset = edgeOffset * -t;
                var triangleToTargetCandidate = scaledOffset - edgeStart;

                var originNearestStart = ScalarMath.EqualMask(t, 0f);
                var originNearestEnd = ScalarMath.EqualMask(t, 1f);
                if (CollectStats)
                {
                    ++StatUseEdge;
                    if (Any(useAB)) ++StatEdgeAB;
                    else if (Any(useBC)) ++StatEdgeBC;
                    else ++StatEdgeCA;
                    if (Any(originNearestStart)) ++StatTStart;
                    else if (Any(originNearestEnd)) ++StatTEnd;
                    else ++StatTInterior;
                }
                var featureForAB = ScalarMath.Select(originNearestStart, 1, ScalarMath.Select(originNearestEnd, 2, 1 + 2));
                var featureForBC = ScalarMath.Select(originNearestStart, 2, ScalarMath.Select(originNearestEnd, 4, 2 + 4));
                var featureForCA = ScalarMath.Select(originNearestStart, 4, ScalarMath.Select(originNearestEnd, 1, 4 + 1));
                relevantFeatures = ScalarMath.Select(useAB, featureForAB, ScalarMath.Select(useBC, featureForBC, featureForCA));
                triangleToTarget = triangleToTargetCandidate;
                var weightEdgeStart = 1f - t;
                simplex.A.Weight = ScalarMath.Select(useAB, weightEdgeStart, ScalarMath.Select(useBC, 0f, t));
                simplex.B.Weight = ScalarMath.Select(useAB, t, ScalarMath.Select(useBC, weightEdgeStart, 0f));
                simplex.C.Weight = ScalarMath.Select(useAB, 0f, ScalarMath.Select(useBC, t, weightEdgeStart));
                //Weight denominator is still just one, as it is in the vertex case.
            }
        }

        //We've examined the vertex and edge case, now we need to check the triangle face case.
        var targetContainedInEdgePlanes = !targetOutsideTriangleEdges & !simplexDegenerate & !terminated;
        if (CollectStats && targetContainedInEdgePlanes) ++StatTargetContained;
        if (targetContainedInEdgePlanes)
        {
            var targetToADot = ScalarMath.Dot(targetToA, triangleNormal);
            var targetOnTriangleSurface = targetToADot * targetToADot < terminationEpsilonSquared * triangleNormalLengthSquared;
            if (targetOnTriangleSurface)
                terminated = true;
            triangleToTarget = triangleNormal;
            relevantFeatures = 1 + 2 + 4;
            simplex.A.Weight = bcPlaneTest;
            simplex.B.Weight = caPlaneTest;
            simplex.C.Weight = abPlaneTest;
            simplex.WeightDenominator = triangleNormalLengthSquared;
        }

        simplex.A.Exists = (relevantFeatures & 1) > 0;
        simplex.B.Exists = (relevantFeatures & 2) > 0;
        simplex.C.Exists = (relevantFeatures & 4) > 0;

        if (CollectStats && terminated) ++StatTerminated;
        if (!terminated)
        {
            //Use the offset to tilt the normal rather than using the offset directly; see the wide implementation.
            var pushOffset = triangleToTarget * 4f;
            var pushNormalCandidate = searchTarget + pushOffset;
            if (CollectStats && !Any(ScalarMath.LessOrEqualMask(bestDepth, 0f) | ScalarMath.Mask(targetContainedInEdgePlanes))) ++StatPushCandidate;
            triangleToTarget = ScalarMath.Select(ScalarMath.LessOrEqualMask(bestDepth, 0f) | ScalarMath.Mask(targetContainedInEdgePlanes), triangleToTarget, pushNormalCandidate);

            var lengthSquared = triangleToTarget.LengthSquared();
            nextNormal = triangleToTarget * (1f / MathF.Sqrt(lengthSquared));
        }
        return nextNormal;
    }

    /// <summary>
    /// Scalar mirror of the witness-output FindMinimumDepth for margin-free shapes.
    /// </summary>
    public static void FindMinimumDepth(in TShapeA shapeA, in TShapeB shapeB, in Vector3 localOffsetB, in Matrix3x3 localOrientationB,
        in Vector3 initialNormal, float convergenceThreshold, float minimumDepthThreshold,
        out float refinedDepth, out Vector3 refinedNormal, out Vector3 witnessOnA, int maximumIterations = 25)
    {
        if (CollectStats)
            ++TotalCalls;
        //Local copies defeat byref aliasing (in-params are byrefs, so loop loads through them cannot be hoisted past simplex
        //stores) and the transpose is computed once instead of re-derived as horizontal dots every iteration.
        var orientationB = localOrientationB;
        Matrix3x3.Transpose(orientationB, out var orientationBTranspose);
        var offsetB = localOffsetB;
        //FindSupport (support(N, A) - support(-N, B)) is inlined manually at both sample sites: even an inlined helper with
        //out-params can address-expose the destination locals. Shape naming follows the refiner's convention; the pair testers
        //pass their shape B as refiner shape A and vice versa.
        var initialSupportOnA = TSupportFinderA.ComputeLocalSupport(shapeA, initialNormal);
        var initialExtremeB = TSupportFinderB.ComputeSupport(shapeB, orientationB, orientationBTranspose, -initialNormal);
        initialExtremeB += offsetB;
        var initialSupport = initialSupportOnA - initialExtremeB;
        var initialDepth = ScalarMath.Dot(initialSupport, initialNormal);

        //Create: only slot A is really filled; empty slots duplicate its data.
        Unsafe.SkipInit(out SimplexScalar simplex);
        var initialSupport128 = initialSupport.AsVector128Unsafe();
        var initialSupportOnA128 = initialSupportOnA.AsVector128Unsafe();
        simplex.A.Support = initialSupport128;
        simplex.B.Support = initialSupport128;
        simplex.C.Support = initialSupport128;
        simplex.A.SupportOnA = initialSupportOnA128;
        simplex.B.SupportOnA = initialSupportOnA128;
        simplex.C.SupportOnA = initialSupportOnA128;
        simplex.A.Exists = true;
        simplex.B.Exists = false;
        simplex.C.Exists = false;

        var depthThreshold = minimumDepthThreshold;
        if (TSupportFinderA.HasMargin)
            depthThreshold -= TSupportFinderA.GetMargin(shapeA);
        if (TSupportFinderB.HasMargin)
            depthThreshold -= TSupportFinderB.GetMargin(shapeB);
        refinedNormal = initialNormal;
        refinedDepth = initialDepth;
        if (initialDepth < depthThreshold)
        {
            //The wide version leaves the witness undefined (garbage weights) for lanes terminated before the loop; callers never use it. Use zero for stability.
            //(It also skips the margin re-add on this path only when the whole bundle terminates; such lanes never produce contacts either way.)
            witnessOnA = default;
            return;
        }

        //First-normal computation, specialized: the freshly created simplex is a single vertex (every slot duplicates the
        //initial support), so the general GetNextNormal body provably collapses — all edges are zero, the triangle normal and
        //every plane test are exactly zero, simplexIsAVertex is true, and the edge/face paths are off. The straight-line form
        //below executes exactly the operations the general path would retain, in the same order; everything skipped is
        //discarded by construction, so simplex state, termination, and the next normal are bitwise identical.
        var firstSearchTarget = refinedNormal * MathF.Max(0f, refinedDepth);
        var firstTerminationEpsilon = ScalarMath.Select(ScalarMath.LessMask(refinedDepth, 0f), convergenceThreshold - refinedDepth, convergenceThreshold);
        var firstTerminationEpsilonSquared = firstTerminationEpsilon * firstTerminationEpsilon;
        var firstTargetToA = initialSupport - firstSearchTarget;
        //The general path also writes these weights (vertex case) before its termination check; Exists flags keep their
        //creation values, which match the vertex case's relevantFeatures.
        simplex.A.Weight = 1f;
        simplex.B.Weight = 0f;
        simplex.C.Weight = 0f;
        simplex.WeightDenominator = 1f;
        var terminated = firstTargetToA.LengthSquared() < firstTerminationEpsilonSquared;
        Vector3 normal = default;
        if (!terminated)
        {
            var triangleToTarget = -firstTargetToA;
            var pushOffset = triangleToTarget * 4f;
            var pushNormalCandidate = firstSearchTarget + pushOffset;
            triangleToTarget = ScalarMath.Select(ScalarMath.LessOrEqualMask(refinedDepth, 0f), triangleToTarget, pushNormalCandidate);
            var lengthSquared = triangleToTarget.LengthSquared();
            normal = triangleToTarget * (1f / MathF.Sqrt(lengthSquared));
        }

        for (int i = 0; i < maximumIterations; ++i)
        {
            if (terminated)
                break;
            if (CollectStats)
                ++TotalIterations;
            var supportOnA = TSupportFinderA.ComputeLocalSupport(shapeA, normal);
            var extremeB = TSupportFinderB.ComputeSupport(shapeB, orientationB, orientationBTranspose, -normal);
            extremeB += offsetB;
            var support = supportOnA - extremeB;
            var depth = ScalarMath.Dot(support, normal);

            if (depth < refinedDepth)
            {
                refinedDepth = depth;
                refinedNormal = normal;
            }
            if (refinedDepth <= depthThreshold)
                break;

            normal = GetNextNormal(ref simplex, support, supportOnA, ref terminated, refinedNormal, refinedDepth, convergenceThreshold);
        }
        if (TSupportFinderA.HasMargin)
            refinedDepth += TSupportFinderA.GetMargin(shapeA);
        if (TSupportFinderB.HasMargin)
            refinedDepth += TSupportFinderB.GetMargin(shapeB);
        //For simplexes terminating in a triangle state, the division for converting plane tests to barycentric coordinates was deferred until now.
        var inverseDenominator = 1f / simplex.WeightDenominator;
        var weightedA = simplex.A.SupportOnA.AsVector3() * (simplex.A.Weight * inverseDenominator);
        var weightedB = simplex.B.SupportOnA.AsVector3() * (simplex.B.Weight * inverseDenominator);
        var weightedC = simplex.C.SupportOnA.AsVector3() * (simplex.C.Weight * inverseDenominator);
        witnessOnA = weightedA + weightedB;
        witnessOnA = weightedC + witnessOnA;
        if (TSupportFinderA.HasMargin)
        {
            var witnessOffset = refinedNormal * TSupportFinderA.GetMargin(shapeA);
            witnessOnA = witnessOffset + witnessOnA;
        }
    }
}
