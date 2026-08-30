using BepuPhysics.Collidables;
using BepuUtilities;
using System.Diagnostics;
using System.Numerics;

namespace AosBaselines;

/// <summary>
/// Track 4 followup: quantifies post-refiner axis polish. After the frozen-configuration engine refiner converges on a
/// penetrating pair, its axis is polished by candidates of increasing infrastructure cost and each result is judged
/// against Track 2's exhaustive-SAT oracle:
///   P0  (zero new engine infrastructure): evaluate the exact depths of the two representative-face normals the engine
///        already selects for clipping (frozen PickRepresentativeFace with the refiner witness); adopt the best improvement.
///   P0b (still no topology): evaluate every face inside PickRepresentativeFace's plane-error tolerance band, both hulls.
///   P1  (full topology): one steepest-descent step of Track 4's CSO facet machinery seeded at the refiner axis.
///   Pk  (full topology): same descent iterated to its local minimum.
/// All depth evaluations are exact interval depths (warm adjacency-climb supports, exact by convexity); descent accepts
/// only strict improvements, so polished depth can never be worse than the refiner's (asserted and counted).
/// Relaxed-equality code: no bitwise mirroring, frozen testers untouched.
/// </summary>
public static class PolishOracle
{
    public struct Work
    {
        public int Supports;   //Support evaluations (adjacency climbs here; O(V) scans in a zero-infrastructure engine port).
        public int FaceEvals;  //Candidate face axes whose exact depth was evaluated.
        public int GaussTests; //Edge pairs run through the gauss-arc (Minkowski face) test.
    }

    const int TypeFaceA = 0, TypeFaceB = 1, TypeEdge = 2, TypeExtra = 3;

    public struct DescentState
    {
        public float Depth;
        public Vector3 Normal;
        public int Type;
        public int FaceA, FaceB, EdgeA, EdgeB;
        public int WarmA, WarmB;
    }

    /// <summary>Exact interval depth along a B-local B->A axis: support_B(n) - dot(localOffsetA, n) + support_A(-R^T n).</summary>
    static float AxisDepth(HullTopology topologyA, HullTopology topologyB, in Matrix3x3 bLocalOrientationA, in Vector3 localOffsetA,
        Vector3 n, ref int warmA, ref int warmB, ref Work w)
    {
        w.Supports += 2;
        var nInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
        var negNInA = -nInA;
        var sB = HullSupportScalarHillclimb.Climb(topologyB, warmB, n, out warmB);
        var sA = HullSupportScalarHillclimb.Climb(topologyA, warmA, negNInA, out warmA);
        return Vector3.Dot(sB, n) - Vector3.Dot(localOffsetA, n) + Vector3.Dot(sA, negNInA);
    }

    /// <summary>
    /// One steepest-descent step over the CSO facet neighborhood (Track 4's penetrating-regime machinery, minus the
    /// margin-reject early-outs, which cannot fire on SAT-penetrating pairs): candidate moves are the faces incident to
    /// either hull's support vertex along the incumbent axis, the incumbent face's adjacent faces and loop edges when the
    /// incumbent is a face cell, and the gauss-arc-filtered incident edge pairs. Returns false at a local minimum.
    /// </summary>
    public static bool DescentStep(HullTopology topologyA, HullTopology topologyB, in Matrix3x3 bLocalOrientationA, in Vector3 localOffsetA,
        ref DescentState s, ref Work w)
    {
        var edgesA = topologyA.Edges;
        var edgesB = topologyB.Edges;
        var verticesA = topologyA.Vertices;
        var verticesB = topologyB.Vertices;
        var faceNormalsA = topologyA.FaceNormals;
        var faceNormalsB = topologyB.FaceNormals;

        //Anchors: the support vertices along the incumbent axis.
        var incumbentInA = ScalarMath.TransformByTransposed(s.Normal, bLocalOrientationA);
        HullSupportScalarHillclimb.Climb(topologyA, s.WarmA, -incumbentInA, out s.WarmA);
        HullSupportScalarHillclimb.Climb(topologyB, s.WarmB, s.Normal, out s.WarmB);
        w.Supports += 2;
        var anchorA = s.WarmA;
        var anchorB = s.WarmB;
        var warmA = s.WarmA;
        var warmB = s.WarmB;

        var stepBestDepth = s.Depth;
        var stepBestNormal = s.Normal;
        var stepBestType = s.Type;
        int stepFaceA = s.FaceA, stepFaceB = s.FaceB, stepEdgeA = s.EdgeA, stepEdgeB = s.EdgeB;
        bool improved = false;

        //(faceB, vertexA) cells: pass 0 = B's support-vertex star, pass 1 = faces adjacent to a FaceB incumbent.
        for (int pass = 0; pass < 2; ++pass)
        {
            int[] source;
            int start, end;
            if (pass == 0)
            {
                source = topologyB.VertexFaces;
                start = topologyB.VertexFaceStarts[anchorB];
                end = topologyB.VertexFaceStarts[anchorB + 1];
            }
            else
            {
                if (s.Type != TypeFaceB)
                    break;
                source = topologyB.AdjacentFaces;
                start = topologyB.FaceStarts[s.FaceB];
                end = topologyB.FaceStarts[s.FaceB + 1];
            }
            for (int i = start; i < end; ++i)
            {
                var f = source[i];
                if (s.Type == TypeFaceB && f == s.FaceB)
                    continue;
                var n = faceNormalsB[f];
                var dirInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                var negDirInA = -dirInA;
                var support = HullSupportScalarHillclimb.Climb(topologyA, warmA, negDirInA, out warmA);
                var depth = topologyB.FaceSupportOffsets[f] - Vector3.Dot(localOffsetA, n) + Vector3.Dot(support, negDirInA);
                ++w.FaceEvals;
                ++w.Supports;
                if (depth < stepBestDepth)
                {
                    stepBestDepth = depth;
                    stepBestNormal = n;
                    stepBestType = TypeFaceB;
                    stepFaceB = f;
                    improved = true;
                }
            }
        }
        //(vertexB, faceA) cells: pass 0 = A's support-vertex star, pass 1 = faces adjacent to a FaceA incumbent.
        for (int pass = 0; pass < 2; ++pass)
        {
            int[] source;
            int start, end;
            if (pass == 0)
            {
                source = topologyA.VertexFaces;
                start = topologyA.VertexFaceStarts[anchorA];
                end = topologyA.VertexFaceStarts[anchorA + 1];
            }
            else
            {
                if (s.Type != TypeFaceA)
                    break;
                source = topologyA.AdjacentFaces;
                start = topologyA.FaceStarts[s.FaceA];
                end = topologyA.FaceStarts[s.FaceA + 1];
            }
            for (int i = start; i < end; ++i)
            {
                var g = source[i];
                if (s.Type == TypeFaceA && g == s.FaceA)
                    continue;
                Matrix3x3.Transform(faceNormalsA[g], bLocalOrientationA, out var m);
                var axis = -m;
                var support = HullSupportScalarHillclimb.Climb(topologyB, warmB, axis, out warmB);
                var depth = Vector3.Dot(support, axis) + Vector3.Dot(localOffsetA, m) + topologyA.FaceSupportOffsets[g];
                ++w.FaceEvals;
                ++w.Supports;
                if (depth < stepBestDepth)
                {
                    stepBestDepth = depth;
                    stepBestNormal = axis;
                    stepBestType = TypeFaceA;
                    stepFaceA = g;
                    improved = true;
                }
            }
        }
        //(edgeA, edgeB) cells: incident edges of the two support vertices (plus a face incumbent's loop edges),
        //gauss-arc filtered, sqrt-free improvement pretest, exact climb rescan on improvers.
        for (int passA = 0; passA < 2; ++passA)
        {
            int[] sourceA;
            int aStartSlot, aEndSlot;
            if (passA == 0)
            {
                sourceA = topologyA.VertexEdges;
                aStartSlot = topologyA.VertexAdjacencyStarts[anchorA];
                aEndSlot = topologyA.VertexAdjacencyStarts[anchorA + 1];
            }
            else
            {
                if (s.Type != TypeFaceA)
                    break;
                sourceA = topologyA.FaceEdges;
                aStartSlot = topologyA.FaceStarts[s.FaceA];
                aEndSlot = topologyA.FaceStarts[s.FaceA + 1];
            }
            var bStarStart = topologyB.VertexAdjacencyStarts[anchorB];
            var bStarEnd = topologyB.VertexAdjacencyStarts[anchorB + 1];
            var useFaceEdgesB = s.Type == TypeFaceB;
            var bFaceStart = useFaceEdgesB ? topologyB.FaceStarts[s.FaceB] : 0;
            var bFaceEnd = useFaceEdgesB ? topologyB.FaceStarts[s.FaceB + 1] : 0;
            for (int ia = aStartSlot; ia < aEndSlot; ++ia)
            {
                var eA = sourceA[ia];
                ref var edgeA = ref edgesA[eA];
                var dALocal = verticesA[edgeA.End] - verticesA[edgeA.Start];
                Matrix3x3.Transform(dALocal, bLocalOrientationA, out var dA);
                Matrix3x3.Transform(faceNormalsA[edgeA.Face0], bLocalOrientationA, out var a1);
                Matrix3x3.Transform(faceNormalsA[edgeA.Face1], bLocalOrientationA, out var a2);
                Matrix3x3.Transform(verticesA[edgeA.Start], bLocalOrientationA, out var rvAStart);
                var pAStart = rvAStart + localOffsetA;
                var dALengthSquared = dA.LengthSquared();
                var bTotal = (bStarEnd - bStarStart) + (bFaceEnd - bFaceStart);
                for (int ibCombined = 0; ibCombined < bTotal; ++ibCombined)
                {
                    var inStar = ibCombined < bStarEnd - bStarStart;
                    var eB = inStar
                        ? topologyB.VertexEdges[bStarStart + ibCombined]
                        : topologyB.FaceEdges[bFaceStart + (ibCombined - (bStarEnd - bStarStart))];
                    if (s.Type == TypeEdge && eA == s.EdgeA && eB == s.EdgeB)
                        continue;
                    ref var edgeB = ref edgesB[eB];
                    ++w.GaussTests;
                    var b1 = faceNormalsB[edgeB.Face0];
                    var b2 = faceNormalsB[edgeB.Face1];
                    var t1 = Vector3.Dot(b1, dA);
                    var t2 = Vector3.Dot(b2, dA);
                    if (t1 * t2 >= 0f)
                        continue;
                    var dB = verticesB[edgeB.End] - verticesB[edgeB.Start];
                    var t3 = Vector3.Dot(a1, dB);
                    var t4 = Vector3.Dot(a2, dB);
                    if (t3 * t4 >= 0f || t1 * t4 >= 0f)
                        continue;
                    var axis = Vector3.Cross(dA, dB);
                    var axisLengthSquared = axis.LengthSquared();
                    if (axisLengthSquared < 1e-10f * dALengthSquared * dB.LengthSquared())
                        continue;
                    var calibrationDot = Vector3.Dot(axis, rvAStart);
                    var pBStart = verticesB[edgeB.Start];
                    var raw = Vector3.Dot(pBStart - pAStart, axis);
                    if (calibrationDot > 0)
                        raw = -raw;
                    bool improves = stepBestDepth >= 0f
                        ? raw < 0f || raw * raw < stepBestDepth * stepBestDepth * axisLengthSquared
                        : raw < 0f && raw * raw > stepBestDepth * stepBestDepth * axisLengthSquared;
                    if (improves)
                    {
                        var n = axis * ((calibrationDot > 0 ? -1f : 1f) / MathF.Sqrt(axisLengthSquared));
                        var nInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                        var negNInA = -nInA;
                        var sB = HullSupportScalarHillclimb.Climb(topologyB, warmB, n, out warmB);
                        var sA = HullSupportScalarHillclimb.Climb(topologyA, warmA, negNInA, out warmA);
                        w.Supports += 2;
                        var depth = Vector3.Dot(sB, n) - Vector3.Dot(localOffsetA, n) + Vector3.Dot(sA, negNInA);
                        if (depth >= stepBestDepth)
                            continue;
                        stepBestDepth = depth;
                        stepBestNormal = n;
                        stepBestType = TypeEdge;
                        stepEdgeA = eA;
                        stepEdgeB = eB;
                        improved = true;
                    }
                }
            }
        }
        s.WarmA = warmA;
        s.WarmB = warmB;
        if (!improved)
            return false;
        s.Depth = stepBestDepth;
        s.Normal = stepBestNormal;
        s.Type = stepBestType;
        s.FaceA = stepFaceA;
        s.FaceB = stepFaceB;
        s.EdgeA = stepEdgeA;
        s.EdgeB = stepEdgeB;
        return true;
    }

    /// <summary>
    /// Faces inside PickRepresentativeFace's plane-error tolerance band: |dot(N_f, witness) - engineOffset_f| within
    /// boundingPlaneEpsilon of the minimum error. Same error metric (engine bounding-plane offsets) as the frozen picker;
    /// band membership uses the min error (conservative superset of the picker's sequential band).
    /// </summary>
    static int CollectBandFaces(HullTopology topology, Vector3 closestOnHull, float boundingPlaneEpsilon, int[] bandFaces)
    {
        var faceNormals = topology.FaceNormals;
        var faceOffsets = topology.FaceOffsets;
        var minError = float.MaxValue;
        for (int f = 0; f < topology.FaceCount; ++f)
        {
            var error = MathF.Abs(Vector3.Dot(faceNormals[f], closestOnHull) - faceOffsets[f]);
            if (error < minError)
                minError = error;
        }
        var count = 0;
        var threshold = minError + boundingPlaneEpsilon;
        for (int f = 0; f < topology.FaceCount && count < bandFaces.Length; ++f)
        {
            var error = MathF.Abs(Vector3.Dot(faceNormals[f], closestOnHull) - faceOffsets[f]);
            if (error <= threshold)
                bandFaces[count++] = f;
        }
        return count;
    }

    static float Percentile(List<float> values, double p)
    {
        if (values.Count == 0)
            return 0;
        var sorted = values.ToArray();
        Array.Sort(sorted);
        var index = (int)Math.Ceiling(p * sorted.Length) - 1;
        return sorted[Math.Clamp(index, 0, sorted.Length - 1)];
    }

    static (double mean, double p99) Stats(List<float> values)
    {
        if (values.Count == 0)
            return (0, 0);
        double sum = 0;
        foreach (var v in values)
            sum += v;
        return (sum / values.Count, Percentile(values, 0.99));
    }

    public static void RunCell(HullSet hullSet, HullTopology[] topologies, int size, bool contactHeavy, int seed, int targetPenetrating, int maxFailReports)
    {
        var generator = new HullHullGenerator(seed + size + (contactHeavy ? 1 : 0), hullSet, contactHeavy);
        const int VariantCount = 5; //0 refiner, 1 P0, 2 P0b, 3 P1, 4 Pk
        string[] variantNames = ["refiner", "P0", "P0b", "P1", "Pk"];
        var hits = new long[VariantCount];
        var excess = new List<float>[VariantCount];
        for (int i = 0; i < VariantCount; ++i)
            excess[i] = new List<float>(targetPenetrating);
        //Work distributions for the four polish variants (index 1..4).
        var workSupports = new List<float>[VariantCount];
        var workFaceEvals = new List<float>[VariantCount];
        var workGauss = new List<float>[VariantCount];
        for (int i = 1; i < VariantCount; ++i)
        {
            workSupports[i] = new List<float>(targetPenetrating);
            workFaceEvals[i] = new List<float>(targetPenetrating);
            workGauss[i] = new List<float>(targetPenetrating);
        }
        var pkSteps = new List<float>(targetPenetrating);
        //Depth tiers: 0 = shallow (satDepth <= 0.1*scale), 1 = mid (0.1..0.25), 2 = deep (> 0.25).
        var tierCounts = new long[3];
        var tierHits = new long[3, VariantCount];
        //Refiner-miss anatomy: true axis type from the SAT oracle (0 faceA, 1 faceB, 2 edge, 3 extra) x recovery.
        var missByType = new long[4];
        var missByTier = new long[3];
        var recovered = new long[4, VariantCount]; //[satType, variant]; variant 0 unused.
        var recoveredByTier = new long[3, VariantCount];
        long adoptWorseViolations = 0, belowSatViolations = 0, refinerReportMismatches = 0, refinerRejectsOnPenetrating = 0;
        double maxBelowSatGap = 0, maxRefinerRejectDiscrepancy = 0;
        long totalCases = 0, penetrating = 0;
        var bandFacesA = new int[512];
        var bandFacesB = new int[512];
        var maxCases = 80L * targetPenetrating;
        var watch = Stopwatch.StartNew();

        while (penetrating < targetPenetrating && totalCases < maxCases)
        {
            var pairCase = generator.Next();
            ++totalCases;
            ref var hullA = ref hullSet.Hulls[pairCase.A];
            ref var hullB = ref hullSet.Hulls[pairCase.B];
            var topologyA = topologies[pairCase.A];
            var topologyB = topologies[pairCase.B];
            //Exhaustive SAT oracle (no polish/fallback): full sweep on penetrating pairs.
            SatHullPairTester.Test(topologyA, topologyB, ref hullA, ref hullB,
                pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, usePolish: false, out _);
            if (SatHullPairTester.LastWinnerType < 0)
                continue; //Rejected beyond margin.
            var satDepth = SatHullPairTester.LastDepth;
            if (satDepth < 0f)
                continue; //Separated within margin: not the penetrating regime this study targets.
            var satType = SatHullPairTester.LastWinnerType;
            ++penetrating;
            var scale = MathF.Min(hullSet.MaxRadii[pairCase.A], hullSet.MaxRadii[pairCase.B]);
            var tolerance = MathF.Max(1e-5f * scale, 1e-4f * MathF.Abs(satDepth));
            var tier = satDepth <= 0.1f * scale ? 0 : satDepth <= 0.25f * scale ? 1 : 2;
            ++tierCounts[tier];

            //Frozen-configuration engine refiner (identical setup to the frozen scalar tester).
            Matrix3x3.CreateFromQuaternion(pairCase.OrientationA, out Matrix3x3 rA);
            Matrix3x3.CreateFromQuaternion(pairCase.OrientationB, out Matrix3x3 rB);
            ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);
            var localOffsetA = -ScalarMath.TransformByTransposed(pairCase.OffsetB, rB);
            var centerDistance = ScalarMath.Length(localOffsetA);
            var initialNormal = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
            var epsilonScale = MathF.Min(
                (MathF.Abs(topologyA.Vertices[0].X) + MathF.Abs(topologyA.Vertices[0].Y) + MathF.Abs(topologyA.Vertices[0].Z)) / 3f,
                (MathF.Abs(topologyB.Vertices[0].X) + MathF.Abs(topologyB.Vertices[0].Y) + MathF.Abs(topologyB.Vertices[0].Z)) / 3f);
            ScalarDepthRefiner<ConvexHull, HullSupportScalar, ConvexHull, HullSupportScalar>.FindMinimumDepth(
                hullB, hullA, localOffsetA, bLocalOrientationA, initialNormal,
                1e-5f * epsilonScale, -pairCase.SpeculativeMargin, out var refinerDepth, out var refinerNormal, out var closestOnB);
            if (refinerDepth < -pairCase.SpeculativeMargin)
            {
                //satDepth >= 0 while the refiner's (exact-interval-depth) axis certifies separation beyond the margin:
                //expected only at near-touch knife edges where the two exact evaluators disagree at rounding level.
                ++refinerRejectsOnPenetrating;
                var discrepancy = (satDepth - refinerDepth) / scale;
                if (discrepancy > maxRefinerRejectDiscrepancy)
                    maxRefinerRejectDiscrepancy = discrepancy;
            }

            //Exact depth of the refiner's axis: the baseline every polish must beat (descent-only-accepts-improvements).
            int warmA0 = 0, warmB0 = 0;
            var wBase = default(Work);
            var refExact = AxisDepth(topologyA, topologyB, bLocalOrientationA, localOffsetA, refinerNormal, ref warmA0, ref warmB0, ref wBase);
            if (MathF.Abs(refExact - refinerDepth) > tolerance)
                ++refinerReportMismatches;

            //--- P0: the two representative faces the engine already picks for clipping. ---
            var wP0 = default(Work);
            var localNormalInA = ScalarMath.TransformByTransposed(refinerNormal, bLocalOrientationA);
            var negatedLocalNormalInA = -localNormalInA;
            var closestOnA = closestOnB - refinerNormal * refinerDepth;
            var closestOnAInA = ScalarMath.TransformByTransposed(closestOnA - localOffsetA, bLocalOrientationA);
            var boundingPlaneEpsilon = 1e-3f * epsilonScale;
            ConvexHullPairScalarTester.PickRepresentativeFace(ref hullA, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out _, out var faceA);
            ConvexHullPairScalarTester.PickRepresentativeFace(ref hullB, refinerNormal, closestOnB, boundingPlaneEpsilon, out _, out var faceB);
            int warmA = warmA0, warmB = warmB0;
            var p0Depth = refExact;
            {
                var axisB = topologyB.FaceNormals[faceB];
                var depthB = AxisDepth(topologyA, topologyB, bLocalOrientationA, localOffsetA, axisB, ref warmA, ref warmB, ref wP0);
                ++wP0.FaceEvals;
                Matrix3x3.Transform(topologyA.FaceNormals[faceA], bLocalOrientationA, out var mA);
                var depthA = AxisDepth(topologyA, topologyB, bLocalOrientationA, localOffsetA, -mA, ref warmA, ref warmB, ref wP0);
                ++wP0.FaceEvals;
                if (depthB < p0Depth)
                    p0Depth = depthB;
                if (depthA < p0Depth)
                    p0Depth = depthA;
            }

            //--- P0b: every face in the picker's plane-error tolerance band, both hulls. ---
            var wP0b = default(Work);
            var p0bDepth = refExact;
            {
                warmA = warmA0;
                warmB = warmB0;
                var bandCountB = CollectBandFaces(topologyB, closestOnB, boundingPlaneEpsilon, bandFacesB);
                for (int i = 0; i < bandCountB; ++i)
                {
                    var axis = topologyB.FaceNormals[bandFacesB[i]];
                    var depth = AxisDepth(topologyA, topologyB, bLocalOrientationA, localOffsetA, axis, ref warmA, ref warmB, ref wP0b);
                    ++wP0b.FaceEvals;
                    if (depth < p0bDepth)
                        p0bDepth = depth;
                }
                var bandCountA = CollectBandFaces(topologyA, closestOnAInA, boundingPlaneEpsilon, bandFacesA);
                for (int i = 0; i < bandCountA; ++i)
                {
                    Matrix3x3.Transform(topologyA.FaceNormals[bandFacesA[i]], bLocalOrientationA, out var m);
                    var depth = AxisDepth(topologyA, topologyB, bLocalOrientationA, localOffsetA, -m, ref warmA, ref warmB, ref wP0b);
                    ++wP0b.FaceEvals;
                    if (depth < p0bDepth)
                        p0bDepth = depth;
                }
            }

            //--- P1 / Pk: Track 4 facet descent seeded at the refiner axis. ---
            var descent = new DescentState { Depth = refExact, Normal = refinerNormal, Type = TypeExtra, WarmA = warmA0, WarmB = warmB0 };
            var wP1 = default(Work);
            var improved = DescentStep(topologyA, topologyB, bLocalOrientationA, localOffsetA, ref descent, ref wP1);
            var p1Depth = descent.Depth;
            var wPk = wP1;
            int steps = improved ? 1 : 0;
            while (improved && steps < 64)
            {
                improved = DescentStep(topologyA, topologyB, bLocalOrientationA, localOffsetA, ref descent, ref wPk);
                if (improved)
                    ++steps;
            }
            var pkDepth = descent.Depth;

            //--- Aggregate. ---
            Span<float> depths = [refExact, p0Depth, p0bDepth, p1Depth, pkDepth];
            var refinerHit = refExact - satDepth <= tolerance;
            for (int v = 0; v < VariantCount; ++v)
            {
                var e = depths[v] - satDepth;
                if (e <= tolerance)
                {
                    ++hits[v];
                    ++tierHits[tier, v];
                    if (!refinerHit && v > 0)
                    {
                        ++recovered[satType, v];
                        ++recoveredByTier[tier, v];
                    }
                }
                excess[v].Add(MathF.Max(0f, e) / scale);
                if (v > 0)
                {
                    if (depths[v] > refExact + 1e-6f * scale)
                    {
                        ++adoptWorseViolations;
                        if (adoptWorseViolations <= maxFailReports)
                            Console.WriteLine($"    [polish] VIOLATION adopt-worse case {totalCases}: {variantNames[v]} depth {depths[v]:G6} vs refiner exact {refExact:G6}");
                    }
                    if (e < -tolerance)
                    {
                        ++belowSatViolations;
                        var gap = -e / scale;
                        if (gap > maxBelowSatGap)
                            maxBelowSatGap = gap;
                        if (belowSatViolations <= maxFailReports)
                            Console.WriteLine($"    [polish] below-SAT case {totalCases}: {variantNames[v]} depth {depths[v]:G6} vs sat {satDepth:G6} (tol {tolerance:G3})");
                    }
                }
            }
            if (!refinerHit)
            {
                ++missByType[satType];
                ++missByTier[tier];
            }
            workSupports[1].Add(wP0.Supports); workFaceEvals[1].Add(wP0.FaceEvals); workGauss[1].Add(wP0.GaussTests);
            workSupports[2].Add(wP0b.Supports); workFaceEvals[2].Add(wP0b.FaceEvals); workGauss[2].Add(wP0b.GaussTests);
            workSupports[3].Add(wP1.Supports); workFaceEvals[3].Add(wP1.FaceEvals); workGauss[3].Add(wP1.GaussTests);
            workSupports[4].Add(wPk.Supports); workFaceEvals[4].Add(wPk.FaceEvals); workGauss[4].Add(wPk.GaussTests);
            pkSteps.Add(steps);
        }
        watch.Stop();

        var profile = contactHeavy ? "contact-heavy" : "mixed";
        Console.WriteLine($"polishoracle size {size} {profile} seed {seed}: {totalCases} cases -> {penetrating} penetrating ({watch.Elapsed.TotalSeconds:F1}s)");
        var hitLine = "    global-axis hit rate: ";
        for (int v = 0; v < VariantCount; ++v)
            hitLine += $"{variantNames[v]} {(double)hits[v] / Math.Max(1, penetrating):P2}{(v < VariantCount - 1 ? " -> " : "")}";
        Console.WriteLine(hitLine);
        var excessLine = "    excess/scale mean|p99: ";
        for (int v = 0; v < VariantCount; ++v)
        {
            var (mean, p99) = Stats(excess[v]);
            excessLine += $"{variantNames[v]} {mean:E2}|{p99:E2}{(v < VariantCount - 1 ? ", " : "")}";
        }
        Console.WriteLine(excessLine);
        string[] tierNames = ["shallow<=0.10", "mid 0.10-0.25", "deep >0.25"];
        for (int t = 0; t < 3; ++t)
        {
            if (tierCounts[t] == 0)
            {
                Console.WriteLine($"    tier {tierNames[t]}: 0 cases");
                continue;
            }
            var line = $"    tier {tierNames[t]} ({tierCounts[t]} cases, {(double)tierCounts[t] / Math.Max(1, penetrating):P1}): miss% ";
            for (int v = 0; v < VariantCount; ++v)
                line += $"{variantNames[v]} {(double)(tierCounts[t] - tierHits[t, v]) / tierCounts[t]:P2}{(v < VariantCount - 1 ? " -> " : "")}";
            Console.WriteLine(line);
        }
        var totalMisses = missByType[0] + missByType[1] + missByType[2] + missByType[3];
        Console.WriteLine($"    refiner misses {totalMisses}: true axis type faceA {missByType[0]}, faceB {missByType[1]}, edge {missByType[2]}, extra {missByType[3]}; " +
            $"by tier shallow {missByTier[0]}, mid {missByTier[1]}, deep {missByTier[2]}");
        for (int v = 1; v < VariantCount; ++v)
        {
            long recTotal = 0, recFace = 0, recEdge = 0;
            for (int t = 0; t < 4; ++t)
            {
                recTotal += recovered[t, v];
                if (t == 0 || t == 1)
                    recFace += recovered[t, v];
                if (t == 2)
                    recEdge += recovered[t, v];
            }
            Console.WriteLine($"    miss recovery by {variantNames[v]}: {recTotal}/{totalMisses} ({(double)recTotal / Math.Max(1, totalMisses):P1}) " +
                $"[face-type misses {recFace}/{missByType[0] + missByType[1]}, edge-type {recEdge}/{missByType[2]}]; " +
                $"per tier {recoveredByTier[0, v]}/{missByTier[0]}, {recoveredByTier[1, v]}/{missByTier[1]}, {recoveredByTier[2, v]}/{missByTier[2]}");
        }
        Console.WriteLine($"    adjacency (P1 = one facet step): adjacent misses {recovered[0, 3] + recovered[1, 3] + recovered[2, 3] + recovered[3, 3]}, " +
            $"distant-basin misses {totalMisses - (recovered[0, 3] + recovered[1, 3] + recovered[2, 3] + recovered[3, 3])}; " +
            $"unrecovered even by Pk {totalMisses - (recovered[0, 4] + recovered[1, 4] + recovered[2, 4] + recovered[3, 4])}");
        for (int v = 1; v < VariantCount; ++v)
        {
            var (sMean, sP99) = Stats(workSupports[v]);
            var (fMean, fP99) = Stats(workFaceEvals[v]);
            var (gMean, gP99) = Stats(workGauss[v]);
            Console.WriteLine($"    cost {variantNames[v]}: supports mean {sMean:F1} p99 {sP99:F0}, face evals mean {fMean:F1} p99 {fP99:F0}, gauss tests mean {gMean:F1} p99 {gP99:F0}" +
                (v == 4 ? $", steps mean {Stats(pkSteps).mean:F2} p99 {Stats(pkSteps).p99:F0}" : ""));
        }
        Console.WriteLine($"    sanity: adopt-worse violations {adoptWorseViolations}, below-SAT {belowSatViolations} (max gap/scale {maxBelowSatGap:E2}), " +
            $"refiner reported-vs-exact mismatches {refinerReportMismatches}, refiner rejects on SAT-penetrating {refinerRejectsOnPenetrating} (max discrepancy/scale {maxRefinerRejectDiscrepancy:E2})");
    }

    /// <summary>
    /// Rough interleaved wall-clock check: frozen-configuration refiner alone vs refiner + Pk facet descent on the same
    /// cases (setup + axis search only, no manifold generation). Counts are the primary cost metric; this bounds the
    /// wall-clock delta on one cell.
    /// </summary>
    public static void RunBench(int size, bool contactHeavy, int seed, int pairCount, int trialCount)
    {
        var setupRandom = new Random(seed + size);
        using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, size);
        var topologies = HullTopology.CreateForSet(hullSet, out _);
        var generator = new HullHullGenerator(seed + size + (contactHeavy ? 1 : 0), hullSet, contactHeavy);
        var cases = new HullPairCase[pairCount];
        for (int i = 0; i < pairCount; ++i)
            cases[i] = generator.Next();

        float RunCases(bool polish)
        {
            float sink = 0;
            for (int i = 0; i < cases.Length; ++i)
            {
                ref var pairCase = ref cases[i];
                Matrix3x3.CreateFromQuaternion(pairCase.OrientationA, out Matrix3x3 rA);
                Matrix3x3.CreateFromQuaternion(pairCase.OrientationB, out Matrix3x3 rB);
                ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);
                var localOffsetA = -ScalarMath.TransformByTransposed(pairCase.OffsetB, rB);
                var centerDistance = ScalarMath.Length(localOffsetA);
                var initialNormal = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
                var topologyA = topologies[pairCase.A];
                var topologyB = topologies[pairCase.B];
                var epsilonScale = MathF.Min(
                    (MathF.Abs(topologyA.Vertices[0].X) + MathF.Abs(topologyA.Vertices[0].Y) + MathF.Abs(topologyA.Vertices[0].Z)) / 3f,
                    (MathF.Abs(topologyB.Vertices[0].X) + MathF.Abs(topologyB.Vertices[0].Y) + MathF.Abs(topologyB.Vertices[0].Z)) / 3f);
                ScalarDepthRefiner<ConvexHull, HullSupportScalar, ConvexHull, HullSupportScalar>.FindMinimumDepth(
                    hullSet.Hulls[pairCase.B], hullSet.Hulls[pairCase.A], localOffsetA, bLocalOrientationA, initialNormal,
                    1e-5f * epsilonScale, -pairCase.SpeculativeMargin, out var depth, out var normal, out _);
                sink += depth;
                if (polish && depth >= 0f && depth >= -pairCase.SpeculativeMargin)
                {
                    var w = default(Work);
                    var descent = new DescentState { Depth = depth, Normal = normal, Type = TypeExtra };
                    for (int step = 0; step < 64; ++step)
                    {
                        if (!DescentStep(topologyA, topologyB, bLocalOrientationA, localOffsetA, ref descent, ref w))
                            break;
                    }
                    sink += descent.Depth;
                }
            }
            return sink;
        }

        //Interleaved measurement (MeasurePair pattern): 2s warmup alternating both closures, then alternating trials.
        const double warmupSeconds = 2.0;
        var warmupStart = Stopwatch.GetTimestamp();
        for (int i = 0; i < 10 || Stopwatch.GetTimestamp() - warmupStart < warmupSeconds * Stopwatch.Frequency; ++i)
        {
            ThroughputRunner.Sink += RunCases(false);
            ThroughputRunner.Sink += RunCases(true);
        }
        if (ThroughputRunner.InterTrialCooldownMilliseconds > 0)
            Thread.Sleep(ThroughputRunner.InterTrialCooldownMilliseconds * 4);
        var plain = new double[trialCount];
        var polished = new double[trialCount];
        double Trial(bool polish)
        {
            var start = Stopwatch.GetTimestamp();
            const int reps = 4;
            for (int rep = 0; rep < reps; ++rep)
                ThroughputRunner.Sink += RunCases(polish);
            var end = Stopwatch.GetTimestamp();
            return (end - start) * 1e9 / ((double)Stopwatch.Frequency * reps * pairCount);
        }
        for (int trial = 0; trial < trialCount; ++trial)
        {
            plain[trial] = Trial(false);
            polished[trial] = Trial(true);
            if (ThroughputRunner.InterTrialCooldownMilliseconds > 0)
                Thread.Sleep(ThroughputRunner.InterTrialCooldownMilliseconds);
        }
        Array.Sort(plain);
        Array.Sort(polished);
        Console.WriteLine($"polishbench size {size} {(contactHeavy ? "contact-heavy" : "mixed")} seed {seed} ({pairCount} pairs, axis search only):");
        Console.WriteLine($"    refiner alone: min {plain[0]:F1} ns/pair, median {plain[trialCount / 2]:F1}");
        Console.WriteLine($"    refiner + Pk:  min {polished[0]:F1} ns/pair, median {polished[trialCount / 2]:F1} " +
            $"(+{polished[0] - plain[0]:F1} ns by min, {polished[0] / plain[0]:F3}x)");
    }
}
