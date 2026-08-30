using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace AosBaselines;

/// <summary>
/// Track 2 relaxed candidate: full SAT hull-hull tester (Gregorius-style). Face axes of both hulls are tested with
/// bundled support-interval scans over the precomputed SoA vertex layouts; edge-edge axes are prefiltered with the
/// gauss-arc (Minkowski face) test vectorized over the SoA edge layout (for a fixed A edge, all B edges in
/// Vector&lt;float&gt; bundles). Penetrating depths from SAT are exact for polytopes. For separated-but-within-margin
/// cases the SAT axis set underestimates the true distance (vertex-region closest features are not SAT axes), so this
/// candidate optionally polishes the separated axis with a support-witness iteration. Manifold generation reuses the
/// frozen tester's representative-face pick + clip + Reduce so conventions match the engine's.
/// </summary>
public sealed class SatRelaxedTester : IRelaxedHullPairTester
{
    public string Name => "sat";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        SatHullPairTester.Test(topologyA, topologyB, ref a, ref b, speculativeMargin, offsetB, orientationA, orientationB,
            usePolish: true, out manifold);
    }
}

/// <summary>Same SAT tester without the separated-case witness polish, to quantify that disagreement class.</summary>
public sealed class SatRawRelaxedTester : IRelaxedHullPairTester
{
    public string Name => "satraw";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        SatHullPairTester.Test(topologyA, topologyB, ref a, ref b, speculativeMargin, offsetB, orientationA, orientationB,
            usePolish: false, out manifold);
    }
}

/// <summary>Cheap always-on statistics for the SAT tester (a handful of static adds per pair; negligible vs pair cost).</summary>
public static class SatStats
{
    public static long Pairs, PrepassRejects, FaceBRejects, FaceARejects, EdgeRejects, PolishRejects, Accepted;
    public static long WinnerFaceA, WinnerFaceB, WinnerEdge, WinnerExtra;
    public static long EdgePhaseRuns, EdgePairsTested, EdgeCond1Bundles, EdgeBundles, EdgeSurvivors, PolishRuns, PolishIterations;
    public static void Reset()
    {
        Pairs = PrepassRejects = FaceBRejects = FaceARejects = EdgeRejects = PolishRejects = Accepted = 0;
        WinnerFaceA = WinnerFaceB = WinnerEdge = WinnerExtra = 0;
        EdgePhaseRuns = EdgePairsTested = EdgeCond1Bundles = EdgeBundles = EdgeSurvivors = PolishRuns = PolishIterations = 0;
    }
    public static void Print(string label)
    {
        Console.WriteLine($"{label}: pairs {Pairs}, accepted {Accepted} ({(double)Accepted / Math.Max(1, Pairs):P1}); rejects: prepass {PrepassRejects}, faceB {FaceBRejects}, faceA {FaceARejects}, edge {EdgeRejects}, polish {PolishRejects}");
        Console.WriteLine($"    winners: faceA {WinnerFaceA}, faceB {WinnerFaceB}, edge {WinnerEdge}, extra(prepass/polish) {WinnerExtra}");
        Console.WriteLine($"    edge filter: phase ran on {EdgePhaseRuns} pairs, {EdgePairsTested} edge pairs tested, {EdgeSurvivors} survivors " +
            $"({(double)EdgeSurvivors / Math.Max(1, EdgePairsTested):P3} pass rate); bundles {EdgeBundles}, bundles passing arc-test-1 {EdgeCond1Bundles} ({(double)EdgeCond1Bundles / Math.Max(1, EdgeBundles):P1})");
        Console.WriteLine($"    polish: ran {PolishRuns}, iterations {PolishIterations} ({(double)PolishIterations / Math.Max(1, PolishRuns):F2}/run)");
    }
}

public static class SatHullPairTester
{
    /// <summary>Depth of the decisive axis of the most recent Test call (the rejecting axis on rejects, the SAT minimum on accepts). Triage aid.</summary>
    public static float LastDepth;
    /// <summary>B-local normal of the decisive axis of the most recent Test call (B->A convention). Triage aid.</summary>
    public static Vector3 LastLocalNormal;
    /// <summary>Winner type of the most recent accepted Test call (0 faceA, 1 faceB, 2 edge, 3 extra; -1 = rejected). Triage aid.</summary>
    public static int LastWinnerType;
    /// <summary>Clip candidate count fed to Reduce on the most recent accepted Test call. Triage aid.</summary>
    public static int LastClipCandidateCount;

    struct CachedEdge
    {
        public Vector3 Vertex;
        public Vector3 EdgePlaneNormal;
        public float MaximumContainmentDot;
    }

    enum WinnerType { FaceA, FaceB, Edge, Extra }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float HorizontalMax(Vector<float> v)
    {
        if (Vector<float>.Count == 8)
        {
            var v256 = v.AsVector256();
            var v128 = Vector128.MaxNative(v256.GetLower(), v256.GetUpper());
            v128 = Vector128.MaxNative(v128, Vector128.Shuffle(v128, Vector128.Create(2, 3, 0, 1)));
            v128 = Vector128.MaxNative(v128, Vector128.Shuffle(v128, Vector128.Create(1, 0, 3, 2)));
            return v128.ToScalar();
        }
        var best = v[0];
        for (int i = 1; i < Vector<float>.Count; ++i)
            best = float.MaxNative(best, v[i]);
        return best;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static uint MoveMask(Vector<int> mask)
    {
        if (Vector<int>.Count == 8)
            return mask.AsVector256().AsSingle().ExtractMostSignificantBits();
        if (Vector<int>.Count == 4)
            return mask.AsVector128().AsSingle().ExtractMostSignificantBits();
        uint bits = 0;
        for (int i = 0; i < Vector<int>.Count; ++i)
            if (mask[i] != 0)
                bits |= 1u << i;
        return bits;
    }

    /// <summary>Max over the hull's vertices of dot(vertex, direction), bundled over the padded SoA layout.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float SupportMax(HullTopology topology, Vector3 direction)
    {
        ref var xs = ref MemoryMarshal.GetArrayDataReference(topology.VerticesX);
        ref var ys = ref MemoryMarshal.GetArrayDataReference(topology.VerticesY);
        ref var zs = ref MemoryMarshal.GetArrayDataReference(topology.VerticesZ);
        var dx = new Vector<float>(direction.X);
        var dy = new Vector<float>(direction.Y);
        var dz = new Vector<float>(direction.Z);
        var best = dx * Vector.LoadUnsafe(ref xs, 0) + dy * Vector.LoadUnsafe(ref ys, 0) + dz * Vector.LoadUnsafe(ref zs, 0);
        for (nuint i = (nuint)Vector<float>.Count; i < (nuint)topology.VerticesX.Length; i += (nuint)Vector<float>.Count)
        {
            var dot = dx * Vector.LoadUnsafe(ref xs, i) + dy * Vector.LoadUnsafe(ref ys, i) + dz * Vector.LoadUnsafe(ref zs, i);
            best = Vector.MaxNative(best, dot);
        }
        return HorizontalMax(best);
    }

    /// <summary>Support scan that also reports the winning vertex index (first occurrence on ties).</summary>
    static int SupportArgmax(HullTopology topology, Vector3 direction, out float bestDot)
    {
        ref var xs = ref MemoryMarshal.GetArrayDataReference(topology.VerticesX);
        ref var ys = ref MemoryMarshal.GetArrayDataReference(topology.VerticesY);
        ref var zs = ref MemoryMarshal.GetArrayDataReference(topology.VerticesZ);
        var dx = new Vector<float>(direction.X);
        var dy = new Vector<float>(direction.Y);
        var dz = new Vector<float>(direction.Z);
        Helpers.FillVectorWithLaneIndices(out var laneOffsets);
        var best = dx * Vector.LoadUnsafe(ref xs, 0) + dy * Vector.LoadUnsafe(ref ys, 0) + dz * Vector.LoadUnsafe(ref zs, 0);
        var bestIndices = laneOffsets;
        for (nuint i = (nuint)Vector<float>.Count; i < (nuint)topology.VerticesX.Length; i += (nuint)Vector<float>.Count)
        {
            var dot = dx * Vector.LoadUnsafe(ref xs, i) + dy * Vector.LoadUnsafe(ref ys, i) + dz * Vector.LoadUnsafe(ref zs, i);
            var useCandidate = Vector.GreaterThan(dot, best);
            best = Vector.ConditionalSelect(useCandidate, dot, best);
            bestIndices = Vector.ConditionalSelect(useCandidate, laneOffsets + new Vector<int>((int)i), bestIndices);
        }
        bestDot = best[0];
        var bestIndex = bestIndices[0];
        for (int i = 1; i < Vector<float>.Count; ++i)
        {
            if (best[i] > bestDot)
            {
                bestDot = best[i];
                bestIndex = bestIndices[i];
            }
        }
        //Padding duplicates the last real vertex; a padded lane can only tie it and strict-greater keeps the earlier
        //(real) occurrence, so bestIndex is always a real index. Clamp anyway for belt and suspenders.
        return Math.Min(bestIndex, topology.VertexCount - 1);
    }

    public static unsafe void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        bool usePolish, out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        ++SatStats.Pairs;
        LastWinnerType = -1;
        LastClipCandidateCount = 0;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 rA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 rB);
        //bLocalOrientationA maps A-local into B-local. Everything below happens in B's local space; the SAT normal
        //convention is B -> A (matching the engine manifold convention after rotation into world by rB).
        ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, rB);
        var localOffsetA = -localOffsetB;

        var firstPointA = topologyA.Vertices[0];
        var firstPointB = topologyB.Vertices[0];
        var aEpsilonScale = (MathF.Abs(firstPointA.X) + MathF.Abs(firstPointA.Y) + MathF.Abs(firstPointA.Z)) * (1f / 3f);
        var bEpsilonScale = (MathF.Abs(firstPointB.X) + MathF.Abs(firstPointB.Y) + MathF.Abs(firstPointB.Z)) * (1f / 3f);
        var epsilonScale = float.MinNative(aEpsilonScale, bEpsilonScale);
        var depthThreshold = -speculativeMargin;

        var bestDepth = float.MaxValue;
        var bestNormal = new Vector3(0f, 1f, 0f);
        var bestType = WinnerType.Extra;
        int bestFaceA = 0, bestFaceB = 0, bestEdgeA = 0, bestEdgeB = 0;

        //Depth along a B->A unit direction n (B-local): depth(n) = max_B dot(p, n) - min_A dot(p, n)
        //  = maxScanB(n) - dot(localOffsetA, n) + maxScanA(-R^T n).      (positive = penetration)
        //Any direction's depth is an upper bound on the true minimum, so any direction with depth < -margin certifies
        //rejection; the SAT axes (faces + gauss-filtered edge crosses) realize the true minimum when penetrating.

        //Cheap prepass: the center-to-center direction rejects far misses with two support scans, and for separated
        //pairs it participates in the minimum tracking (it is a valid direction, often closer to the true distance
        //direction than any SAT axis when closest features are vertices).
        var centerDistance = ScalarMath.Length(localOffsetA);
        if (centerDistance > 1e-8f)
        {
            var u = localOffsetA * (1f / centerDistance);
            var uInA = ScalarMath.TransformByTransposed(u, bLocalOrientationA);
            var depth = SupportMax(topologyB, u) - Vector3.Dot(localOffsetA, u) + SupportMax(topologyA, -uInA);
            if (depth < depthThreshold)
            {
                ++SatStats.PrepassRejects;
                LastDepth = depth;
                LastLocalNormal = u;
                return;
            }
            bestDepth = depth;
            bestNormal = u;
            bestType = WinnerType.Extra;
        }

        //Any axis with negative depth PROVES separation (depth(n) upper-bounds the true minimum). For the fallback
        //configuration the separated band's manifold comes from the refiner regardless, so the remaining SAT phases
        //would only ever be spent producing an axis the fallback then discards; skip straight to the refiner. The raw
        //configuration (no fallback) keeps the full SAT sweep since its output normal IS the SAT axis.
        var skipToFallback = usePolish && bestDepth < 0f;

        //Face axes of B: n = face normal (already B-local); max_B along n = the precomputed TRUE support offset
        //(the engine bounding-plane offset can understate the extent on merged near-coplanar faces).
        var faceNormalsB = topologyB.FaceNormals;
        var faceOffsetsB = topologyB.FaceSupportOffsets;
        for (int f = 0; skipToFallback == false && f < topologyB.FaceCount; ++f)
        {
            var n = faceNormalsB[f];
            var dirA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
            var depth = faceOffsetsB[f] - Vector3.Dot(localOffsetA, n) + SupportMax(topologyA, -dirA);
            if (depth < bestDepth)
            {
                if (depth < depthThreshold)
                {
                    ++SatStats.FaceBRejects;
                    LastDepth = depth;
                    LastLocalNormal = n;
                    return;
                }
                bestDepth = depth;
                bestNormal = n;
                bestType = WinnerType.FaceB;
                bestFaceB = f;
                if (usePolish && depth < 0f)
                {
                    skipToFallback = true;
                    break;
                }
            }
        }

        //Face axes of A: candidate direction (B->A) is the NEGATED transformed face normal; max_A along the face
        //normal (in A space) = the face plane offset exactly.
        var faceNormalsA = topologyA.FaceNormals;
        var faceOffsetsA = topologyA.FaceSupportOffsets;
        for (int f = 0; skipToFallback == false && f < topologyA.FaceCount; ++f)
        {
            Matrix3x3.Transform(faceNormalsA[f], bLocalOrientationA, out var m);
            var depth = SupportMax(topologyB, -m) + Vector3.Dot(localOffsetA, m) + faceOffsetsA[f];
            if (depth < bestDepth)
            {
                if (depth < depthThreshold)
                {
                    ++SatStats.FaceARejects;
                    LastDepth = depth;
                    LastLocalNormal = -m;
                    return;
                }
                bestDepth = depth;
                bestNormal = -m;
                bestType = WinnerType.FaceA;
                bestFaceA = f;
                if (usePolish && depth < 0f)
                {
                    skipToFallback = true;
                    break;
                }
            }
        }

        //Edge-edge axes with the gauss-arc (Minkowski face) prefilter. For A edge with direction dA and adjacent
        //face normals a1, a2 (all transformed into B-local) against B edge with direction dB and normals b1, b2:
        //the arcs intersect (so cross(dA, dB) is a support-realizing axis) iff
        //  (b1.dA)(b2.dA) < 0  and  (a1.dB)(a2.dB) < 0  and  (b1.dA)(a2.dB) < 0
        //(the standard IsMinkowskiFace(a1, a2, -b1, -b2) with the cross products replaced by the edge directions,
        //valid because this topology's edge directions satisfy dir = cross(nFace0, nFace1) up to positive scale).
        if (!skipToFallback)
        {
            var edgeCountA = topologyA.EdgeCount;
            var edgeCountB = topologyB.EdgeCount;
            var laneCount = Vector<float>.Count;
            var bundleCountB = topologyB.EdgeDirX.Length / laneCount;
            ref var b1x = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeNormal0X);
            ref var b1y = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeNormal0Y);
            ref var b1z = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeNormal0Z);
            ref var b2x = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeNormal1X);
            ref var b2y = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeNormal1Y);
            ref var b2z = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeNormal1Z);
            ref var dbx = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeDirX);
            ref var dby = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeDirY);
            ref var dbz = ref MemoryMarshal.GetArrayDataReference(topologyB.EdgeDirZ);
            var edgesA = topologyA.Edges;
            var verticesA = topologyA.Vertices;
            var faceNormalsAScalar = topologyA.FaceNormals;
            long testedPairs = 0, survivors = 0, bundlesTotal = 0, bundlesCond1 = 0;
            ++SatStats.EdgePhaseRuns;
            for (int eA = 0; eA < edgeCountA; ++eA)
            {
                ref var edgeA = ref edgesA[eA];
                var dALocal = verticesA[edgeA.End] - verticesA[edgeA.Start];
                Matrix3x3.Transform(dALocal, bLocalOrientationA, out var dA);
                Matrix3x3.Transform(faceNormalsAScalar[edgeA.Face0], bLocalOrientationA, out var a1);
                Matrix3x3.Transform(faceNormalsAScalar[edgeA.Face1], bLocalOrientationA, out var a2);
                Matrix3x3.Transform(verticesA[edgeA.Start], bLocalOrientationA, out var rvAStart);
                var pAStart = rvAStart + localOffsetA;
                var dALengthSquared = dA.LengthSquared();
                var dAx = new Vector<float>(dA.X);
                var dAy = new Vector<float>(dA.Y);
                var dAz = new Vector<float>(dA.Z);
                var a1x = new Vector<float>(a1.X);
                var a1y = new Vector<float>(a1.Y);
                var a1z = new Vector<float>(a1.Z);
                var a2x = new Vector<float>(a2.X);
                var a2y = new Vector<float>(a2.Y);
                var a2z = new Vector<float>(a2.Z);
                testedPairs += edgeCountB;
                bundlesTotal += bundleCountB;
                for (int bundle = 0; bundle < bundleCountB; ++bundle)
                {
                    var offset = (nuint)(bundle * laneCount);
                    //Arc test 1: does B's arc (b1..b2) straddle the plane of A's arc (plane normal = dA)?
                    var t1 = Vector.LoadUnsafe(ref b1x, offset) * dAx + Vector.LoadUnsafe(ref b1y, offset) * dAy + Vector.LoadUnsafe(ref b1z, offset) * dAz;
                    var t2 = Vector.LoadUnsafe(ref b2x, offset) * dAx + Vector.LoadUnsafe(ref b2y, offset) * dAy + Vector.LoadUnsafe(ref b2z, offset) * dAz;
                    var cross1 = Vector.LessThan(t1 * t2, Vector<float>.Zero);
                    var bits1 = MoveMask(cross1);
                    if (bits1 == 0)
                        continue;
                    ++bundlesCond1;
                    //Arc test 2 (A's arc straddles B's arc plane) and the shared-hemisphere test.
                    var dBxv = Vector.LoadUnsafe(ref dbx, offset);
                    var dByv = Vector.LoadUnsafe(ref dby, offset);
                    var dBzv = Vector.LoadUnsafe(ref dbz, offset);
                    var t3 = a1x * dBxv + a1y * dByv + a1z * dBzv;
                    var t4 = a2x * dBxv + a2y * dByv + a2z * dBzv;
                    var mask = Vector.BitwiseAnd(cross1, Vector.BitwiseAnd(
                        Vector.LessThan(t3 * t4, Vector<float>.Zero),
                        Vector.LessThan(t1 * t4, Vector<float>.Zero)));
                    var bits = MoveMask(mask);
                    var remaining = edgeCountB - bundle * laneCount;
                    if (remaining < 32)
                        bits &= (1u << remaining) - 1;
                    while (bits != 0)
                    {
                        var lane = System.Numerics.BitOperations.TrailingZeroCount(bits);
                        bits &= bits - 1;
                        var eB = bundle * laneCount + lane;
                        ++survivors;
                        var dB = new Vector3(topologyB.EdgeDirX[eB], topologyB.EdgeDirY[eB], topologyB.EdgeDirZ[eB]);
                        var axis = Vector3.Cross(dA, dB);
                        var axisLengthSquared = axis.LengthSquared();
                        //Near-parallel edge pairs produce degenerate axes; the face axes cover those directions.
                        if (axisLengthSquared < 1e-10f * dALengthSquared * dB.LengthSquared())
                            continue;
                        //Calibration: the axis must point B -> A, i.e. dot(axis, pAStart - localOffsetA) <= 0 (A's
                        //hull-local origin is its centroid). Rather than normalizing every survivor, carry the
                        //unnormalized 'raw = depth * length' and compare against bestDepth via sign analysis with one
                        //squaring - the sqrt/divide is deferred to actual improvers, which are rare.
                        var calibrationDot = Vector3.Dot(axis, rvAStart);
                        var pBStart = new Vector3(topologyB.EdgeStartX[eB], topologyB.EdgeStartY[eB], topologyB.EdgeStartZ[eB]);
                        var raw = Vector3.Dot(pBStart - pAStart, axis);
                        if (calibrationDot > 0)
                            raw = -raw;
                        //improves <=> raw / length < bestDepth, length > 0.
                        bool improves = bestDepth >= 0f
                            ? raw < 0f || raw * raw < bestDepth * bestDepth * axisLengthSquared
                            : raw < 0f && raw * raw > bestDepth * bestDepth * axisLengthSquared;
                        if (improves)
                        {
                            var n = axis * ((calibrationDot > 0 ? -1f : 1f) / MathF.Sqrt(axisLengthSquared));
                            //The edge-point depth assumes the two edges are exact supports along n. The gauss filter
                            //guarantees that only up to the accuracy of the stored (area-averaged) face normals, so on
                            //merged near-coplanar faces the shortcut can understate the true interval depth. It never
                            //OVERstates it (support max >= any vertex dot), so non-improving axes are safely skipped;
                            //improving axes get an exact rescan before they can steer the winner or reject the pair.
                            var nInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                            var depth = SupportMax(topologyB, n) - Vector3.Dot(localOffsetA, n) + SupportMax(topologyA, -nInA);
                            if (depth >= bestDepth)
                                continue;
                            if (depth < depthThreshold)
                            {
                                SatStats.EdgePairsTested += testedPairs;
                                SatStats.EdgeSurvivors += survivors;
                                SatStats.EdgeBundles += bundlesTotal;
                                SatStats.EdgeCond1Bundles += bundlesCond1;
                                ++SatStats.EdgeRejects;
                                LastDepth = depth;
                                LastLocalNormal = n;
                                return;
                            }
                            bestDepth = depth;
                            bestNormal = n;
                            bestType = WinnerType.Edge;
                            bestEdgeA = eA;
                            bestEdgeB = eB;
                        }
                    }
                }
            }
            SatStats.EdgePairsTested += testedPairs;
            SatStats.EdgeSurvivors += survivors;
            SatStats.EdgeBundles += bundlesTotal;
            SatStats.EdgeCond1Bundles += bundlesCond1;
        }

        //Separated-within-margin fallback: SAT's axis set does not contain the true distance direction when the
        //closest features are vertex-region (SAT underestimates the gap, and worse, a misaligned axis makes the
        //support-vertex face-pick witnesses select faces whose projections don't overlap -> empty manifolds where the
        //reference produces speculative contacts). The engine's refiner handles the distance case robustly, so seed
        //the frozen scalar refiner with the SAT axis and use its normal/depth/witness for manifold generation.
        //Penetrating pairs (bestDepth >= 0, where SAT is exact) never take this path.
        var haveRefinerWitness = false;
        Vector3 refinerClosestOnB = default;
        if (usePolish && bestDepth < 0f)
        {
            ++SatStats.PolishRuns;
            //Seed with the frozen tester's initial normal (center direction), NOT the SAT axis: with the identical
            //seed and the identical support finder the refiner reproduces the frozen scalar trajectory exactly, and
            //the frozen scalar tester is bitwise-equal to the wide reference — so the whole separated band inherits
            //the reference's knife-edge decisions instead of rolling its own (seeding with the SAT axis saved a few
            //iterations but produced ~0.5% existence flips on speculative vertex-region contacts).
            var initialNormal = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
            ScalarDepthRefiner<ConvexHull, HullSupportScalar, ConvexHull, HullSupportScalar>.FindMinimumDepth(
                b, a, localOffsetA, bLocalOrientationA, initialNormal, 1e-5f * epsilonScale, depthThreshold,
                out var refinedDepth, out var refinedNormal, out var refinedClosestOnB);
            if (refinedDepth < depthThreshold)
            {
                ++SatStats.PolishRejects;
                LastDepth = refinedDepth;
                LastLocalNormal = refinedNormal;
                return;
            }
            bestDepth = refinedDepth;
            bestNormal = refinedNormal;
            bestType = WinnerType.Extra;
            haveRefinerWitness = true;
            refinerClosestOnB = refinedClosestOnB;
        }

        ++SatStats.Accepted;
        LastDepth = bestDepth;
        LastLocalNormal = bestNormal;
        switch (bestType)
        {
            case WinnerType.FaceA: ++SatStats.WinnerFaceA; break;
            case WinnerType.FaceB: ++SatStats.WinnerFaceB; break;
            case WinnerType.Edge: ++SatStats.WinnerEdge; break;
            default: ++SatStats.WinnerExtra; break;
        }

        //Manifold generation: identical structure to the frozen tester (representative faces on both hulls, clip B's
        //face edges against A's face, add contained A vertices, Reduce), driven by the SAT normal. Where the SAT
        //winner IS a face, that face is the representative directly (exact alignment, zero plane error); the other
        //hull's face comes from the frozen PickRepresentativeFace with the SAT axis's support vertex as witness.
        var localNormal = bestNormal;
        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, bLocalOrientationA);
        var negatedLocalNormalInA = -localNormalInA;
        var boundingPlaneEpsilon = 1e-3f * epsilonScale;

        Vector3 slotFaceNormalAInA, slotFaceNormalB;
        int bestFaceIndexA, bestFaceIndexB;
        switch (bestType)
        {
            case WinnerType.FaceB:
                {
                    bestFaceIndexB = bestFaceB;
                    slotFaceNormalB = topologyB.FaceNormals[bestFaceB];
                    var closestOnAInA = topologyA.Vertices[SupportArgmax(topologyA, negatedLocalNormalInA, out _)];
                    ConvexHullPairScalarTester.PickRepresentativeFace(ref a, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out slotFaceNormalAInA, out bestFaceIndexA);
                    break;
                }
            case WinnerType.FaceA:
                {
                    bestFaceIndexA = bestFaceA;
                    slotFaceNormalAInA = topologyA.FaceNormals[bestFaceA];
                    var closestOnB = topologyB.Vertices[SupportArgmax(topologyB, localNormal, out _)];
                    ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnB, boundingPlaneEpsilon, out slotFaceNormalB, out bestFaceIndexB);
                    break;
                }
            case WinnerType.Edge:
                {
                    //A point on each winning edge lies on both of its adjacent faces (zero plane error), so the pick
                    //selects the most normal-aligned face touching the edge, i.e. the incident face.
                    var closestOnAInA = topologyA.Vertices[topologyA.Edges[bestEdgeA].Start];
                    var closestOnB = topologyB.Vertices[topologyB.Edges[bestEdgeB].Start];
                    ConvexHullPairScalarTester.PickRepresentativeFace(ref a, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out slotFaceNormalAInA, out bestFaceIndexA);
                    ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnB, boundingPlaneEpsilon, out slotFaceNormalB, out bestFaceIndexB);
                    break;
                }
            default:
                {
                    Vector3 closestOnAInA, closestOnB;
                    if (haveRefinerWitness)
                    {
                        //Frozen-tester-style witnesses from the refiner's closest point (see ConvexHullPairScalarTester.Test).
                        closestOnB = refinerClosestOnB;
                        var closestOnA = closestOnB - localNormal * bestDepth;
                        var aToClosestOnA = closestOnA - localOffsetA;
                        closestOnAInA = ScalarMath.TransformByTransposed(aToClosestOnA, bLocalOrientationA);
                    }
                    else
                    {
                        closestOnAInA = topologyA.Vertices[SupportArgmax(topologyA, negatedLocalNormalInA, out _)];
                        closestOnB = topologyB.Vertices[SupportArgmax(topologyB, localNormal, out _)];
                    }
                    ConvexHullPairScalarTester.PickRepresentativeFace(ref a, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out slotFaceNormalAInA, out bestFaceIndexA);
                    ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, closestOnB, boundingPlaneEpsilon, out slotFaceNormalB, out bestFaceIndexB);
                    break;
                }
        }
        Matrix3x3.Transform(slotFaceNormalAInA, bLocalOrientationA, out var slotFaceNormalA);
        Helpers.BuildOrthonormalBasis(slotFaceNormalB, out var bFaceX, out var bFaceY);

        //Clip + reduce, structurally identical to the frozen tester's per-slot contact generation (vertex reads come
        //from the topology's scalar arrays; linear indices double as feature ids).
        var slotLocalNormal = localNormal;
        var slotLocalOffsetA = localOffsetA;

        ref var faceVerticesA = ref MemoryMarshal.GetArrayDataReference(topologyA.FaceVertices);
        ref var verticesAScalar = ref MemoryMarshal.GetArrayDataReference(topologyA.Vertices);
        ref var faceVerticesB = ref MemoryMarshal.GetArrayDataReference(topologyB.FaceVertices);
        ref var verticesBScalar = ref MemoryMarshal.GetArrayDataReference(topologyB.Vertices);
        var faceStartA = topologyA.FaceStarts[bestFaceIndexA];
        var faceCountA = topologyA.FaceStarts[bestFaceIndexA + 1] - faceStartA;
        var faceStartB = topologyB.FaceStarts[bestFaceIndexB];
        var faceCountB = topologyB.FaceStarts[bestFaceIndexB + 1] - faceStartB;

        var cachedEdges = stackalloc CachedEdge[faceCountA];
        var previousVertexA = Unsafe.Add(ref verticesAScalar, Unsafe.Add(ref faceVerticesA, faceStartA + faceCountA - 1));
        Matrix3x3.Transform(previousVertexA, bLocalOrientationA, out previousVertexA);
        previousVertexA += slotLocalOffsetA;
        for (int i = 0; i < faceCountA; ++i)
        {
            ref var edge = ref cachedEdges[i];
            edge.MaximumContainmentDot = float.MinValue;
            edge.Vertex = Unsafe.Add(ref verticesAScalar, Unsafe.Add(ref faceVerticesA, faceStartA + i));
            Matrix3x3.Transform(edge.Vertex, bLocalOrientationA, out edge.Vertex);
            edge.Vertex += slotLocalOffsetA;
            //Note flipped cross order; local normal points from B to A.
            edge.EdgePlaneNormal = Vector3.Cross(slotLocalNormal, edge.Vertex - previousVertexA);
            previousVertexA = edge.Vertex;
        }
        var maximumCandidateCount = Math.Max(Math.Max(faceCountA, faceCountB), Math.Min(faceCountA * 2, faceCountB * 2));
        var candidates = stackalloc ManifoldCandidateScalar[maximumCandidateCount];
        var candidateCount = 0;
        var previousIndexB = Unsafe.Add(ref faceVerticesB, faceStartB + faceCountB - 1);
        var bFaceOrigin = Unsafe.Add(ref verticesBScalar, previousIndexB);
        var previousVertexB = bFaceOrigin;
        for (int faceVertexIndexB = 0; faceVertexIndexB < faceCountB; ++faceVertexIndexB)
        {
            var indexB = Unsafe.Add(ref faceVerticesB, faceStartB + faceVertexIndexB);
            var vertexB = Unsafe.Add(ref verticesBScalar, indexB);

            var edgeOffsetB = vertexB - previousVertexB;
            var edgePlaneNormalB = Vector3.Cross(edgeOffsetB, slotLocalNormal);

            var latestEntry = float.MinValue;
            var earliestExit = float.MaxValue;
            for (int faceVertexIndexA = 0; faceVertexIndexA < faceCountA; ++faceVertexIndexA)
            {
                ref var edgeA = ref cachedEdges[faceVertexIndexA];

                var edgeBToEdgeA = edgeA.Vertex - previousVertexB;
                var containmentDot = Vector3.Dot(edgeBToEdgeA, edgePlaneNormalB);
                if (edgeA.MaximumContainmentDot < containmentDot)
                    edgeA.MaximumContainmentDot = containmentDot;

                var numerator = Vector3.Dot(edgeBToEdgeA, edgeA.EdgePlaneNormal);
                var denominator = Vector3.Dot(edgeA.EdgePlaneNormal, edgeOffsetB);

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
                    earliestExit = float.MinValue;
                    latestEntry = float.MaxValue;
                }
            }
            if (latestEntry <= earliestExit)
            {
                latestEntry = latestEntry < 0 ? 0 : latestEntry;
                earliestExit = earliestExit > 1 ? 1 : earliestExit;
                var startId = previousIndexB;
                var endId = indexB;
                var baseFeatureId = (startId ^ endId) << 8;
                if (earliestExit >= latestEntry && candidateCount < maximumCandidateCount)
                {
                    var point = edgeOffsetB * earliestExit + previousVertexB - bFaceOrigin;
                    var newContactIndex = candidateCount++;
                    ref var candidate = ref candidates[newContactIndex];
                    candidate.X = Vector3.Dot(point, bFaceX);
                    candidate.Y = Vector3.Dot(point, bFaceY);
                    candidate.FeatureId = baseFeatureId + endId;
                }
                if (latestEntry < earliestExit && latestEntry > 0 && candidateCount < maximumCandidateCount)
                {
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
        var inverseLocalNormalADotFaceNormalB = 1f / Vector3.Dot(slotLocalNormal, slotFaceNormalB);
        for (int i = 0; i < faceCountA && candidateCount < maximumCandidateCount; ++i)
        {
            ref var edge = ref cachedEdges[i];
            if (edge.MaximumContainmentDot <= 0)
            {
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
        LastWinnerType = (int)bestType;
        LastClipCandidateCount = candidateCount;
        ConvexHullPairScalarTester.Reduce(candidates, candidateCount, slotFaceNormalA, 1f / Vector3.Dot(slotFaceNormalA, slotLocalNormal), cachedEdges[0].Vertex, bFaceOrigin, bFaceX, bFaceY,
            epsilonScale, depthThreshold, rB, offsetB, ref manifold);
        if (bestType == WinnerType.Edge && !(manifold.Contact0Exists | manifold.Contact1Exists | manifold.Contact2Exists | manifold.Contact3Exists))
        {
            //Edge-edge winners can leave the face clip empty: with the EXACT SAT axis the two representative faces'
            //projections touch only at the edge crossing point, and sign noise decides whether the degenerate span
            //survives. Classic SAT manifold fallback: a single contact at the closest points of the two edges.
            ref var edgeAWinner = ref topologyA.Edges[bestEdgeA];
            ref var edgeBWinner = ref topologyB.Edges[bestEdgeB];
            Matrix3x3.Transform(topologyA.Vertices[edgeAWinner.Start], bLocalOrientationA, out var pA0);
            pA0 += localOffsetA;
            Matrix3x3.Transform(topologyA.Vertices[edgeAWinner.End] - topologyA.Vertices[edgeAWinner.Start], bLocalOrientationA, out var dA);
            var pB0 = topologyB.Vertices[edgeBWinner.Start];
            var dB = topologyB.Vertices[edgeBWinner.End] - pB0;
            var r = pA0 - pB0;
            var aa = dA.LengthSquared();
            var ee = dB.LengthSquared();
            var bb = Vector3.Dot(dA, dB);
            var cc = Vector3.Dot(dA, r);
            var ff = Vector3.Dot(dB, r);
            var denom = aa * ee - bb * bb;
            var s = denom > 1e-20f ? float.MaxNative(0f, float.MinNative(1f, (bb * ff - cc * ee) / denom)) : 0f;
            var t = ee > 1e-20f ? float.MaxNative(0f, float.MinNative(1f, (bb * s + ff) / ee)) : 0f;
            var contactOnB = pB0 + dB * t;
            Matrix3x3.Transform(contactOnB, rB, out var worldContact);
            manifold.OffsetA0 = worldContact + offsetB;
            manifold.Depth0 = bestDepth;
            manifold.FeatureId0 = ((edgeBWinner.Start ^ edgeBWinner.End) << 8) + edgeBWinner.End;
            manifold.Contact0Exists = true;
        }
        Matrix3x3.Transform(localNormal, rB, out manifold.Normal);
    }
}
