using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

/// <summary>
/// Track 4 relaxed candidate: a direct feature walk on the Minkowski boundary (CSO) — "V-Clip in normal space".
/// The CSO D = B ⊖ A_placed (B-local space; every facet's outward normal IS the B→A manifold axis) has three facet
/// types, the cells of the two hulls' overlaid gauss maps: (faceB, vertexA), (vertexB, faceA), and (edgeB, edgeA with
/// crossing arcs). Facet adjacency is computed on the fly from each hull's own precomputed topology; no pose-dependent
/// precompute exists or is needed.
///
/// PENETRATING regime (origin inside D): steepest descent on exact facet plane offsets (= exact SAT axis depths via
/// warm adjacency-climb supports) over a support-vertex-anchored neighborhood: all faces incident to either hull's
/// current support vertex plus the gauss-filtered incident-edge pairs. Local minima are expected and measured
/// (csooracle). Any facet with offset &lt; -margin is a sound rejection certificate.
///
/// SEPARATED regime (any facet with negative offset certifies origin-outside): the classic point-vs-polytope V-Clip
/// walk on D's features (facet/edge/vertex states with Voronoi-region validity conditions), which is globally
/// convergent for a convex set and natively produces vertex-region distance directions that no SAT axis can represent
/// (Track 2) — so the speculative band runs with NO refiner fallback; the walk's own witnesses feed the frozen
/// banded PickRepresentativeFace (Track 3 proved O(degree) picks unsafe on this band).
/// </summary>
public sealed class CsoWalkRelaxedTester : IRelaxedHullPairTester
{
    public string Name => "csowalk";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        CsoWalkHullPairTester.Test(topologyA, topologyB, ref a, ref b, speculativeMargin, offsetB, orientationA, orientationB,
            out manifold);
    }
}

/// <summary>Cheap always-on statistics for the CSO walk tester (a handful of static adds per pair).</summary>
public static class CsoStats
{
    public static long Pairs, PrepassRejects, PenRejects, SepRejects, Accepted;
    public static long WinnerFaceA, WinnerFaceB, WinnerEdge, WinnerSeparated, WinnerPrepass;
    public static long PenSteps, PenFaceEvals, PenEdgePairsTested, PenGaussSurvivors, PenRescans, PenCapHits;
    public static long SepWalks, SepSteps, SepStalls, SepCapHits, SepDegenerate;
    public static long[] PenStepHistogram = new long[64];
    public static long[] SepStepHistogram = new long[160];
    public static void Reset()
    {
        Pairs = PrepassRejects = PenRejects = SepRejects = Accepted = 0;
        WinnerFaceA = WinnerFaceB = WinnerEdge = WinnerSeparated = WinnerPrepass = 0;
        PenSteps = PenFaceEvals = PenEdgePairsTested = PenGaussSurvivors = PenRescans = PenCapHits = 0;
        SepWalks = SepSteps = SepStalls = SepCapHits = SepDegenerate = 0;
        Array.Clear(PenStepHistogram);
        Array.Clear(SepStepHistogram);
    }
    static (double mean, int p99) HistogramStats(long[] histogram)
    {
        long total = 0;
        foreach (var count in histogram)
            total += count;
        if (total == 0)
            return (0, 0);
        long sum = 0, running = 0;
        int p99 = 0;
        var p99Target = (long)Math.Ceiling(total * 0.99);
        bool p99Found = false;
        for (int i = 0; i < histogram.Length; ++i)
        {
            sum += histogram[i] * (long)i;
            running += histogram[i];
            if (!p99Found && running >= p99Target)
            {
                p99 = i;
                p99Found = true;
            }
        }
        return ((double)sum / total, p99);
    }
    public static void Print(string label)
    {
        Console.WriteLine($"{label}: pairs {Pairs}, accepted {Accepted} ({(double)Accepted / Math.Max(1, Pairs):P1}); rejects: prepass {PrepassRejects}, pen {PenRejects}, sep {SepRejects}");
        Console.WriteLine($"    winners: faceA {WinnerFaceA}, faceB {WinnerFaceB}, edge {WinnerEdge}, separated-walk {WinnerSeparated}, prepass {WinnerPrepass}");
        var penPairs = Math.Max(1, Pairs);
        var (penMean, penP99) = HistogramStats(PenStepHistogram);
        Console.WriteLine($"    pen walk: steps mean {penMean:F2} p99 {penP99} (cap hits {PenCapHits}); {(double)PenFaceEvals / penPairs:F1} face evals/pair, " +
            $"{(double)PenEdgePairsTested / penPairs:F1} edge pairs tested/pair (gauss survivors {PenGaussSurvivors}, {(double)PenGaussSurvivors / Math.Max(1, PenEdgePairsTested):P2}), rescans {PenRescans}");
        var (sepMean, sepP99) = HistogramStats(SepStepHistogram);
        Console.WriteLine($"    sep walk: ran on {SepWalks} ({(double)SepWalks / penPairs:P1} of pairs), steps mean {sepMean:F2} p99 {sepP99}; stalls {SepStalls}, cap hits {SepCapHits}, degenerate {SepDegenerate}");
    }
}

public static class CsoWalkHullPairTester
{
    /// <summary>Depth of the decisive axis of the most recent Test call (the rejecting axis on rejects, the search minimum on accepts). Triage aid.</summary>
    public static float LastDepth;
    /// <summary>B-local normal of the decisive axis of the most recent Test call (B->A convention). Triage aid.</summary>
    public static Vector3 LastLocalNormal;
    /// <summary>Winner type of the most recent accepted Test call (0 faceA, 1 faceB, 2 edge, 3 extra/separated; -1 = rejected). Triage aid.</summary>
    public static int LastWinnerType;
    /// <summary>Clip candidate count fed to Reduce on the most recent accepted Test call. Triage aid.</summary>
    public static int LastClipCandidateCount;
    /// <summary>0 = penetrating walk decided the pair, 1 = the separated distance walk ran. Triage/oracle aid.</summary>
    public static int LastRegime;
    /// <summary>True when the most recent separated walk terminated by stall/cap acceptance rather than a Voronoi-condition terminal. Triage aid.</summary>
    public static bool LastSepStalled;

    struct CachedEdge
    {
        public Vector3 Vertex;
        public Vector3 EdgePlaneNormal;
        public float MaximumContainmentDot;
    }

    enum WinnerType { FaceA, FaceB, Edge, Extra }

    //Separated-walk feature kinds on D = B ⊖ A_placed.
    const int KindFacetB = 0;   //(faceB i0, vertexA i1)
    const int KindFacetA = 1;   //(vertexB i0, faceA i1)
    const int KindFacetEE = 2;  //(edgeA i0, edgeB i1)
    const int KindEdgeB = 3;    //(edgeB i0, vertexA i1)
    const int KindEdgeA = 4;    //(vertexB i0, edgeA i1)
    const int KindVertex = 5;   //(vertexB i0, vertexA i1)

    /// <summary>Exact support dot via warm-started adjacency climb (exact by convexity; Track 1's climbprobe evidence).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float ClimbDot(HullTopology topology, ref int warm, Vector3 direction)
    {
        var support = HullSupportScalarHillclimb.Climb(topology, warm, direction, out warm);
        return Vector3.Dot(support, direction);
    }

    /// <summary>Most-aligned face among those incident to the support vertex (validated for penetrating winners; NOT safe
    /// on the speculative band — Track 3 boundary).</summary>
    static int PickIncidentFace(HullTopology topology, int supportVertex, Vector3 direction, out Vector3 faceNormal)
    {
        ref var starts = ref MemoryMarshal.GetArrayDataReference(topology.VertexFaceStarts);
        ref var faces = ref MemoryMarshal.GetArrayDataReference(topology.VertexFaces);
        ref var normals = ref MemoryMarshal.GetArrayDataReference(topology.FaceNormals);
        var start = Unsafe.Add(ref starts, supportVertex);
        var end = Unsafe.Add(ref starts, supportVertex + 1);
        var bestFace = Unsafe.Add(ref faces, start);
        var bestDot = Vector3.Dot(Unsafe.Add(ref normals, bestFace), direction);
        for (int i = start + 1; i < end; ++i)
        {
            var face = Unsafe.Add(ref faces, i);
            var dot = Vector3.Dot(Unsafe.Add(ref normals, face), direction);
            if (dot > bestDot)
            {
                bestDot = dot;
                bestFace = face;
            }
        }
        faceNormal = Unsafe.Add(ref normals, bestFace);
        return bestFace;
    }

    public static unsafe void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        ++CsoStats.Pairs;
        LastWinnerType = -1;
        LastClipCandidateCount = 0;
        LastRegime = 0;
        LastSepStalled = false;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 rA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 rB);
        //bLocalOrientationA maps A-local into B-local; everything below is in B's local space, normal convention B -> A.
        //CSO D = B ⊖ A_placed where A_placed(a) = bLocalOrientationA * a + localOffsetA; the outward facet normals of D
        //are exactly the B->A axes, and depth(n) = support_D(n).
        ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, rB);
        var localOffsetA = -localOffsetB;

        var firstPointA = topologyA.Vertices[0];
        var firstPointB = topologyB.Vertices[0];
        var aEpsilonScale = (MathF.Abs(firstPointA.X) + MathF.Abs(firstPointA.Y) + MathF.Abs(firstPointA.Z)) * (1f / 3f);
        var bEpsilonScale = (MathF.Abs(firstPointB.X) + MathF.Abs(firstPointB.Y) + MathF.Abs(firstPointB.Z)) * (1f / 3f);
        var epsilonScale = float.MinNative(aEpsilonScale, bEpsilonScale);
        var depthThreshold = -speculativeMargin;

        int warmA = 0, warmB = 0;

        //Prepass: exact depth along the center-to-center axis (upper-bounds the true minimum: sound reject certificate),
        //also seeds the warm climb slots at the center-direction support features.
        var centerDistance = ScalarMath.Length(localOffsetA);
        var u = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
        var uInA = ScalarMath.TransformByTransposed(u, bLocalOrientationA);
        var bestDepth = ClimbDot(topologyB, ref warmB, u) - Vector3.Dot(localOffsetA, u) + ClimbDot(topologyA, ref warmA, -uInA);
        var bestNormal = u;
        var bestType = WinnerType.Extra;
        int bestFaceA = 0, bestFaceB = 0, bestEdgeA = 0, bestEdgeB = 0;
        if (bestDepth < depthThreshold)
        {
            ++CsoStats.PrepassRejects;
            LastDepth = bestDepth;
            LastLocalNormal = u;
            return;
        }
        var separatedMode = bestDepth < 0f;

        //-------------------- Penetrating regime: facet-offset descent over the CSO cell graph --------------------
        long penSteps = 0, penFaceEvals = 0, penEdgePairs = 0, penGauss = 0, penRescans = 0;
        if (!separatedMode)
        {
            const int stepCap = 48;
            var edgesA = topologyA.Edges;
            var edgesB = topologyB.Edges;
            var verticesA = topologyA.Vertices;
            var verticesB = topologyB.Vertices;
            var faceNormalsA = topologyA.FaceNormals;
            var faceNormalsB = topologyB.FaceNormals;
            int step = 0;
            for (; step < stepCap; ++step)
            {
                //Anchor: the support vertices along the incumbent axis. The searched neighborhood per step is the true
                //CSO facet adjacency of the incumbent cell plus the gauss-overlay star around the anchors: for a face
                //incumbent the true neighbors are its adjacent faces (support ties across a face freeze the anchor, so
                //the vertex star alone cannot traverse a hull's face graph) and the EE cells from its own loop edges;
                //the anchor stars supply the cross-hull moves.
                var incumbentInA = ScalarMath.TransformByTransposed(bestNormal, bLocalOrientationA);
                HullSupportScalarHillclimb.Climb(topologyA, warmA, -incumbentInA, out warmA);
                HullSupportScalarHillclimb.Climb(topologyB, warmB, bestNormal, out warmB);
                var anchorA = warmA;
                var anchorB = warmB;

                var stepBestDepth = bestDepth;
                var stepBestNormal = bestNormal;
                var stepBestType = bestType;
                int stepFaceA = bestFaceA, stepFaceB = bestFaceB, stepEdgeA = bestEdgeA, stepEdgeB = bestEdgeB;
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
                        if (bestType != WinnerType.FaceB)
                            break;
                        source = topologyB.AdjacentFaces;
                        start = topologyB.FaceStarts[bestFaceB];
                        end = topologyB.FaceStarts[bestFaceB + 1];
                    }
                    for (int i = start; i < end; ++i)
                    {
                        var f = source[i];
                        if (bestType == WinnerType.FaceB && f == bestFaceB)
                            continue;
                        var n = faceNormalsB[f];
                        var dirInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                        var depth = topologyB.FaceSupportOffsets[f] - Vector3.Dot(localOffsetA, n) + ClimbDot(topologyA, ref warmA, -dirInA);
                        ++penFaceEvals;
                        if (depth < depthThreshold)
                        {
                            ++CsoStats.PenRejects;
                            CommitStats(penSteps, penFaceEvals, penEdgePairs, penGauss, penRescans);
                            LastDepth = depth;
                            LastLocalNormal = n;
                            return;
                        }
                        if (depth < stepBestDepth)
                        {
                            stepBestDepth = depth;
                            stepBestNormal = n;
                            stepBestType = WinnerType.FaceB;
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
                        if (bestType != WinnerType.FaceA)
                            break;
                        source = topologyA.AdjacentFaces;
                        start = topologyA.FaceStarts[bestFaceA];
                        end = topologyA.FaceStarts[bestFaceA + 1];
                    }
                    for (int i = start; i < end; ++i)
                    {
                        var g = source[i];
                        if (bestType == WinnerType.FaceA && g == bestFaceA)
                            continue;
                        Matrix3x3.Transform(faceNormalsA[g], bLocalOrientationA, out var m);
                        var axis = -m;
                        var depth = ClimbDot(topologyB, ref warmB, axis) + Vector3.Dot(localOffsetA, m) + topologyA.FaceSupportOffsets[g];
                        ++penFaceEvals;
                        if (depth < depthThreshold)
                        {
                            ++CsoStats.PenRejects;
                            CommitStats(penSteps, penFaceEvals, penEdgePairs, penGauss, penRescans);
                            LastDepth = depth;
                            LastLocalNormal = axis;
                            return;
                        }
                        if (depth < stepBestDepth)
                        {
                            stepBestDepth = depth;
                            stepBestNormal = axis;
                            stepBestType = WinnerType.FaceA;
                            stepFaceA = g;
                            improved = true;
                        }
                    }
                }
                //(edgeA, edgeB) cells: incident edges of the two support vertices (plus, for a face incumbent, that
                //face's own loop edges — its true EE neighbors), gauss-arc filtered (arc crossing = the cell exists on
                //the overlay), sqrt-free improvement pretest, exact climb rescan on improvers.
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
                        if (bestType != WinnerType.FaceA)
                            break;
                        sourceA = topologyA.FaceEdges;
                        aStartSlot = topologyA.FaceStarts[bestFaceA];
                        aEndSlot = topologyA.FaceStarts[bestFaceA + 1];
                    }
                    var bStarStart = topologyB.VertexAdjacencyStarts[anchorB];
                    var bStarEnd = topologyB.VertexAdjacencyStarts[anchorB + 1];
                    var useFaceEdgesB = bestType == WinnerType.FaceB;
                    var bFaceStart = useFaceEdgesB ? topologyB.FaceStarts[bestFaceB] : 0;
                    var bFaceEnd = useFaceEdgesB ? topologyB.FaceStarts[bestFaceB + 1] : 0;
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
                            if (bestType == WinnerType.Edge && eA == bestEdgeA && eB == bestEdgeB)
                                continue;
                            ref var edgeB = ref edgesB[eB];
                            ++penEdgePairs;
                            //Gauss-arc (Minkowski face) test, cross-free form (topology guarantees dir = +cross(N0, N1)).
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
                            ++penGauss;
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
                                ++penRescans;
                                var candidateNInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                                var depth = ClimbDot(topologyB, ref warmB, n) - Vector3.Dot(localOffsetA, n) + ClimbDot(topologyA, ref warmA, -candidateNInA);
                                if (depth >= stepBestDepth)
                                    continue;
                                if (depth < depthThreshold)
                                {
                                    ++CsoStats.PenRejects;
                                    CommitStats(penSteps, penFaceEvals, penEdgePairs, penGauss, penRescans);
                                    LastDepth = depth;
                                    LastLocalNormal = n;
                                    return;
                                }
                                stepBestDepth = depth;
                                stepBestNormal = n;
                                stepBestType = WinnerType.Edge;
                                stepEdgeA = eA;
                                stepEdgeB = eB;
                                improved = true;
                            }
                        }
                    }
                }
                if (!improved)
                    break;
                bestDepth = stepBestDepth;
                bestNormal = stepBestNormal;
                bestType = stepBestType;
                bestFaceA = stepFaceA;
                bestFaceB = stepFaceB;
                bestEdgeA = stepEdgeA;
                bestEdgeB = stepEdgeB;
                ++penSteps;
                if (bestDepth < 0f)
                {
                    separatedMode = true;
                    break;
                }
            }
            if (step == stepCap)
                ++CsoStats.PenCapHits;
        }
        CommitStats(penSteps, penFaceEvals, penEdgePairs, penGauss, penRescans);

        //-------------------- Separated regime: point-vs-polytope V-Clip walk on D --------------------
        var haveSeparatedWitness = false;
        Vector3 sepClosestOnB = default;
        if (separatedMode)
        {
            LastRegime = 1;
            ++CsoStats.SepWalks;
            //Seed feature from the certifying negative cell (or the prepass supports).
            int kind, i0, i1;
            switch (bestType)
            {
                case WinnerType.FaceB: kind = KindFacetB; i0 = bestFaceB; i1 = warmA; break;
                case WinnerType.FaceA: kind = KindFacetA; i0 = warmB; i1 = bestFaceA; break;
                case WinnerType.Edge: kind = KindFacetEE; i0 = bestEdgeA; i1 = bestEdgeB; break;
                default: kind = KindVertex; i0 = warmB; i1 = warmA; break;
            }
            var accepted = SeparatedWalk(topologyA, topologyB, bLocalOrientationA, localOffsetA, depthThreshold, epsilonScale,
                kind, i0, i1, bestDepth, bestNormal, ref warmA, ref warmB, out bestDepth, out bestNormal, out sepClosestOnB);
            if (!accepted)
            {
                ++CsoStats.SepRejects;
                LastDepth = bestDepth;
                LastLocalNormal = bestNormal;
                return;
            }
            bestType = WinnerType.Extra;
            haveSeparatedWitness = true;
        }

        ++CsoStats.Accepted;
        LastDepth = bestDepth;
        LastLocalNormal = bestNormal;
        switch (bestType)
        {
            case WinnerType.FaceA: ++CsoStats.WinnerFaceA; break;
            case WinnerType.FaceB: ++CsoStats.WinnerFaceB; break;
            case WinnerType.Edge: ++CsoStats.WinnerEdge; break;
            default:
                if (haveSeparatedWitness) ++CsoStats.WinnerSeparated; else ++CsoStats.WinnerPrepass;
                break;
        }

        //-------------------- Manifold generation --------------------
        var localNormal = bestNormal;
        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, bLocalOrientationA);
        var negatedLocalNormalInA = -localNormalInA;

        Vector3 slotFaceNormalAInA, slotFaceNormalB;
        int bestFaceIndexA, bestFaceIndexB;
        switch (bestType)
        {
            case WinnerType.FaceB:
                {
                    //The reference face IS the walk state — no PickRepresentativeFace scan.
                    bestFaceIndexB = bestFaceB;
                    slotFaceNormalB = topologyB.FaceNormals[bestFaceB];
                    HullSupportScalarHillclimb.Climb(topologyA, warmA, negatedLocalNormalInA, out warmA);
                    bestFaceIndexA = PickIncidentFace(topologyA, warmA, negatedLocalNormalInA, out slotFaceNormalAInA);
                    break;
                }
            case WinnerType.FaceA:
                {
                    bestFaceIndexA = bestFaceA;
                    slotFaceNormalAInA = topologyA.FaceNormals[bestFaceA];
                    HullSupportScalarHillclimb.Climb(topologyB, warmB, localNormal, out warmB);
                    bestFaceIndexB = PickIncidentFace(topologyB, warmB, localNormal, out slotFaceNormalB);
                    break;
                }
            case WinnerType.Edge:
                {
                    ref var edgeA = ref topologyA.Edges[bestEdgeA];
                    ref var edgeB = ref topologyB.Edges[bestEdgeB];
                    var na0 = topologyA.FaceNormals[edgeA.Face0];
                    var na1 = topologyA.FaceNormals[edgeA.Face1];
                    bestFaceIndexA = Vector3.Dot(na0, negatedLocalNormalInA) >= Vector3.Dot(na1, negatedLocalNormalInA) ? edgeA.Face0 : edgeA.Face1;
                    slotFaceNormalAInA = topologyA.FaceNormals[bestFaceIndexA];
                    var nb0 = topologyB.FaceNormals[edgeB.Face0];
                    var nb1 = topologyB.FaceNormals[edgeB.Face1];
                    bestFaceIndexB = Vector3.Dot(nb0, localNormal) >= Vector3.Dot(nb1, localNormal) ? edgeB.Face0 : edgeB.Face1;
                    slotFaceNormalB = topologyB.FaceNormals[bestFaceIndexB];
                    break;
                }
            default:
                {
                    if (haveSeparatedWitness)
                    {
                        //Speculative band: the O(degree) incident pick is NOT robust here (Track 3: ~0.17% emptied
                        //clips); use the frozen plane-error-banded picker with the walk's exact witness points.
                        var boundingPlaneEpsilon = 1e-3f * epsilonScale;
                        var closestOnA = sepClosestOnB - localNormal * bestDepth;
                        var aToClosestOnA = closestOnA - localOffsetA;
                        var closestOnAInA = ScalarMath.TransformByTransposed(aToClosestOnA, bLocalOrientationA);
                        ConvexHullPairScalarTester.PickRepresentativeFace(ref a, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out slotFaceNormalAInA, out bestFaceIndexA);
                        ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, sepClosestOnB, boundingPlaneEpsilon, out slotFaceNormalB, out bestFaceIndexB);
                    }
                    else
                    {
                        //Prepass axis won outright (rare, penetrating): support-vertex incident picks.
                        HullSupportScalarHillclimb.Climb(topologyA, warmA, negatedLocalNormalInA, out warmA);
                        bestFaceIndexA = PickIncidentFace(topologyA, warmA, negatedLocalNormalInA, out slotFaceNormalAInA);
                        HullSupportScalarHillclimb.Climb(topologyB, warmB, localNormal, out warmB);
                        bestFaceIndexB = PickIncidentFace(topologyB, warmB, localNormal, out slotFaceNormalB);
                    }
                    break;
                }
        }
        Matrix3x3.Transform(slotFaceNormalAInA, bLocalOrientationA, out var slotFaceNormalA);
        Helpers.BuildOrthonormalBasis(slotFaceNormalB, out var bFaceX, out var bFaceY);

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
            //Edge-edge winners can leave the face clip empty (exact-axis boundary-touch degeneracy); classic SAT
            //fallback: a single contact at the closest points of the two edges.
            ref var edgeAWinner = ref topologyA.Edges[bestEdgeA];
            ref var edgeBWinner = ref topologyB.Edges[bestEdgeB];
            Matrix3x3.Transform(topologyA.Vertices[edgeAWinner.Start], bLocalOrientationA, out var pA0);
            pA0 += localOffsetA;
            Matrix3x3.Transform(topologyA.Vertices[edgeAWinner.End] - topologyA.Vertices[edgeAWinner.Start], bLocalOrientationA, out var dAWinner);
            var pB0 = topologyB.Vertices[edgeBWinner.Start];
            var dBWinner = topologyB.Vertices[edgeBWinner.End] - pB0;
            var r = pA0 - pB0;
            var aa = dAWinner.LengthSquared();
            var ee = dBWinner.LengthSquared();
            var bb = Vector3.Dot(dAWinner, dBWinner);
            var cc = Vector3.Dot(dAWinner, r);
            var ff = Vector3.Dot(dBWinner, r);
            var denom = aa * ee - bb * bb;
            var s = denom > 1e-20f ? float.MaxNative(0f, float.MinNative(1f, (bb * ff - cc * ee) / denom)) : 0f;
            var t = ee > 1e-20f ? float.MaxNative(0f, float.MinNative(1f, (bb * s + ff) / ee)) : 0f;
            var contactOnB = pB0 + dBWinner * t;
            Matrix3x3.Transform(contactOnB, rB, out var worldContact);
            manifold.OffsetA0 = worldContact + offsetB;
            manifold.Depth0 = bestDepth;
            manifold.FeatureId0 = ((edgeBWinner.Start ^ edgeBWinner.End) << 8) + edgeBWinner.End;
            manifold.Contact0Exists = true;
        }
        //NOTE: a single-contact witness fallback for empty clips on the separated band was built and MEASURED OUT: the
        //reference itself returns empty speculative manifolds on ~0.9-1.5% of cases (vertex-region face projections
        //that never overlap), so emitting the witness contact there converts every such reference empty into a
        //candidate-only existence flip (total size-8 FAILs 5.83%->6.19% mixed, 4.31%->4.95% contact-heavy). Matching
        //the reference's empty-manifold semantics costs instead a symmetric ~0.3-0.4% knife-edge class where the
        //walk's exact normal (vs the refiner's converged bits) flips a degenerate clip the other way.
        Matrix3x3.Transform(localNormal, rB, out manifold.Normal);
    }

    static void CommitStats(long penSteps, long penFaceEvals, long penEdgePairs, long penGauss, long penRescans)
    {
        CsoStats.PenSteps += penSteps;
        CsoStats.PenFaceEvals += penFaceEvals;
        CsoStats.PenEdgePairsTested += penEdgePairs;
        CsoStats.PenGaussSurvivors += penGauss;
        CsoStats.PenRescans += penRescans;
        ++CsoStats.PenStepHistogram[Math.Min((int)penSteps, CsoStats.PenStepHistogram.Length - 1)];
    }

    /// <summary>
    /// Point-vs-polytope V-Clip walk: closest point of the implicit CSO D = B ⊖ A_placed to the origin, entered only
    /// with a certified separating axis (some exact facet offset &lt; 0 or the prepass axis negative). Facet states
    /// self-restore gauss-membership validity by re-climbing the partner support vertex; edge/vertex states check the
    /// Voronoi cone conditions and route to the violated neighbor. Termination: a facet with negative offset containing
    /// the origin projection is the certified global closest (D lies in the separating halfspace); edge/vertex Voronoi
    /// satisfaction likewise. Distance is monotone non-increasing with strict decreases on cone-violation moves;
    /// cycling guards = iteration cap + most-negative-axis stall acceptance (counted).
    /// Outputs depth = -distance (B->A normal), the closest point witness on B, and accept/reject vs the margin.
    /// </summary>
    static bool SeparatedWalk(HullTopology topologyA, HullTopology topologyB, Matrix3x3 bLocalOrientationA, Vector3 localOffsetA,
        float depthThreshold, float epsilonScale, int kind, int i0, int i1, float entryDepth, Vector3 entryNormal,
        ref int warmA, ref int warmB, out float depth, out Vector3 normal, out Vector3 closestOnB)
    {
        //A raised (size-scaled, up to 128) cap was tried and did NOT reduce cap hits at 128 verts — the tail is genuine
        //facet<->edge dithering (the facet states' per-visit partner re-climbs shift the polygons), not truncation of
        //legitimate long paths — so the cap stays small and the streak guard below ends dithering early; both land in
        //the certified stall path.
        const int iterationCap = 32;
        const int nonImprovingStreakCap = 6;
        const int validityMoveCap = 4;
        int nonImprovingStreak = 0;
        //Absolute tolerances keyed to hull scale: kill boundary dithering (facet<->edge two-cycles from inconsistent
        //rounding at containment boundaries) and near-touching normal instability (w -> 0).
        var boundaryTolerance = 1e-6f * epsilonScale;
        //Below this distance a w-direction (closest-point-difference) normal is numerically meaningless (fp noise in
        //the feature points is ~1e-6*scale); edge/vertex terminals inside it fall back to the most-negative certified
        //axis, whose certificate closes exactly and whose near-zero depth makes any supporting normal comparator-tie
        //equivalent at touch. Facet terminals keep their (always well-conditioned) face normals.
        var touchTolerance = 1e-4f * epsilonScale;
        //Best feature point seen (a true point of D): the stall answer's witness. Its depth UNDERestimates the true
        //(negative) depth while certified axes OVERestimate it, so the two bracket the truth.
        var bestFeatureDist = float.MaxValue;
        Vector3 bestFeatureNormal = default;
        Vector3 bestFeatureClosestOnB = default;
        int validityMoves = 0;
        var verticesA = topologyA.Vertices;
        var verticesB = topologyB.Vertices;
        var faceNormalsA = topologyA.FaceNormals;
        var faceNormalsB = topologyB.FaceNormals;
        var edgesA = topologyA.Edges;
        var edgesB = topologyB.Edges;
        //Most-negative certified axis seen; the stall-acceptance answer.
        var bkDepth = entryDepth;
        var bkNormal = entryNormal;
        var degenerate = false;
        int iteration = 0;
        for (; iteration < iterationCap; ++iteration)
        {
            switch (kind)
            {
                case KindFacetB:
                    {
                        //Facet (faceB i0, vertexA i1): outward normal = B face normal. Validity restoration: i1 must be
                        //A's support along -R^T n; the climb both restores it and yields the exact plane offset.
                        var face = i0;
                        var n = faceNormalsB[face];
                        var nInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                        var supportDotA = ClimbDot(topologyA, ref warmA, -nInA);
                        var va = warmA;
                        var h = topologyB.FaceSupportOffsets[face] - Vector3.Dot(localOffsetA, n) + supportDotA;
                        if (h < bkDepth)
                        {
                            bkDepth = h;
                            bkNormal = n;
                        }
                        var x = n * h;
                        Matrix3x3.Transform(verticesA[va], bLocalOrientationA, out var pva);
                        pva += localOffsetA;
                        //Containment of x in the CSO polygon (translate of face B's CCW loop by -p(va)).
                        var faceStart = topologyB.FaceStarts[face];
                        var loopCount = topologyB.FaceStarts[face + 1] - faceStart;
                        var previous = verticesB[topologyB.FaceVertices[faceStart + loopCount - 1]] - pva;
                        var worstViolation = boundaryTolerance * boundaryTolerance;
                        var worstSlot = -1;
                        for (int k = 0; k < loopCount; ++k)
                        {
                            var current = verticesB[topologyB.FaceVertices[faceStart + k]] - pva;
                            var inward = Vector3.Cross(n, current - previous);
                            var violation = Vector3.Dot(x - previous, inward);
                            if (violation < 0f)
                            {
                                var scaled = violation * violation / float.MaxNative(inward.LengthSquared(), 1e-30f);
                                if (scaled > worstViolation)
                                {
                                    worstViolation = scaled;
                                    worstSlot = k;
                                }
                            }
                            previous = current;
                        }
                        if (worstSlot < 0)
                        {
                            if (h < 0f)
                            {
                                //Global closest: D lies in the halfspace dot(y, n) <= h < 0, and x achieves that bound.
                                depth = h;
                                normal = n;
                                closestOnB = x + pva;
                                goto terminal;
                            }
                            //Origin projects inside the polygon but the plane doesn't separate: not a terminal (the
                            //certified separating facet is elsewhere); accept the best-known axis instead of cycling.
                            goto stall;
                        }
                        kind = KindEdgeB;
                        i0 = topologyB.FaceEdges[faceStart + worstSlot];
                        i1 = va;
                        break;
                    }
                case KindFacetA:
                    {
                        //Facet (vertexB i0, faceA i1): outward normal = -R * n_faceA. Validity: i0 must be B's support along n.
                        var face = i1;
                        Matrix3x3.Transform(faceNormalsA[face], bLocalOrientationA, out var m);
                        var n = -m;
                        var supportDotB = ClimbDot(topologyB, ref warmB, n);
                        var vb = warmB;
                        var h = supportDotB - Vector3.Dot(localOffsetA, n) + topologyA.FaceSupportOffsets[face];
                        if (h < bkDepth)
                        {
                            bkDepth = h;
                            bkNormal = n;
                        }
                        var x = n * h;
                        var vbPosition = verticesB[vb];
                        //CSO polygon = {vb - p(a_k)}; the A loop reverses orientation under ⊖, so CCW traversal around n
                        //is the REVERSED loop. CSO polygon edge j (from reversed position j-1 to j) corresponds to
                        //FaceEdges slot (loopCount - j) % loopCount.
                        var faceStart = topologyA.FaceStarts[face];
                        var loopCount = topologyA.FaceStarts[face + 1] - faceStart;
                        Vector3 CsoVertex(int reversedPosition)
                        {
                            var loopIndex = loopCount - 1 - reversedPosition;
                            Matrix3x3.Transform(verticesA[topologyA.FaceVertices[faceStart + loopIndex]], bLocalOrientationA, out var pa);
                            return vbPosition - (pa + localOffsetA);
                        }
                        var previous = CsoVertex(loopCount - 1);
                        var worstViolation = boundaryTolerance * boundaryTolerance;
                        var worstReversed = -1;
                        for (int j = 0; j < loopCount; ++j)
                        {
                            var current = CsoVertex(j);
                            var inward = Vector3.Cross(n, current - previous);
                            var violation = Vector3.Dot(x - previous, inward);
                            if (violation < 0f)
                            {
                                var scaled = violation * violation / float.MaxNative(inward.LengthSquared(), 1e-30f);
                                if (scaled > worstViolation)
                                {
                                    worstViolation = scaled;
                                    worstReversed = j;
                                }
                            }
                            previous = current;
                        }
                        if (worstReversed < 0)
                        {
                            if (h < 0f)
                            {
                                depth = h;
                                normal = n;
                                closestOnB = vbPosition;
                                goto terminal;
                            }
                            goto stall;
                        }
                        kind = KindEdgeA;
                        i0 = vb;
                        i1 = topologyA.FaceEdges[faceStart + (loopCount - worstReversed) % loopCount];
                        break;
                    }
                case KindFacetEE:
                    {
                        //Facet (edgeA i0, edgeB i1): parallelogram {b(s) - p(a(t))}, outward normal = calibrated cross.
                        ref var edgeA = ref edgesA[i0];
                        ref var edgeB = ref edgesB[i1];
                        var dALocal = verticesA[edgeA.End] - verticesA[edgeA.Start];
                        Matrix3x3.Transform(dALocal, bLocalOrientationA, out var dAp);
                        var dB = verticesB[edgeB.End] - verticesB[edgeB.Start];
                        var axis = Vector3.Cross(dAp, dB);
                        var axisLengthSquared = axis.LengthSquared();
                        if (axisLengthSquared < 1e-10f * dAp.LengthSquared() * dB.LengthSquared())
                        {
                            degenerate = true;
                            goto stall;
                        }
                        Matrix3x3.Transform(verticesA[edgeA.Start], bLocalOrientationA, out var rvAStart);
                        var calibrationDot = Vector3.Dot(axis, rvAStart);
                        var n = axis * ((calibrationDot > 0 ? -1f : 1f) / MathF.Sqrt(axisLengthSquared));
                        //Exact plane offset via climbs (also keeps the certificate bookkeeping exact).
                        var nInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                        var h = ClimbDot(topologyB, ref warmB, n) - Vector3.Dot(localOffsetA, n) + ClimbDot(topologyA, ref warmA, -nInA);
                        if (h < bkDepth)
                        {
                            bkDepth = h;
                            bkNormal = n;
                        }
                        var x = n * h;
                        var pAStart = rvAStart + localOffsetA;
                        var c00 = verticesB[edgeB.Start] - pAStart;
                        //Solve x - c00 = s * dB + t * (-dAp) in the facet plane (2x2 Gram).
                        var uDir = dB;
                        var vDir = -dAp;
                        var xd = x - c00;
                        var uu = uDir.LengthSquared();
                        var vv = vDir.LengthSquared();
                        var uv = Vector3.Dot(uDir, vDir);
                        var det = uu * vv - uv * uv;
                        if (MathF.Abs(det) < 1e-20f)
                        {
                            degenerate = true;
                            goto stall;
                        }
                        var xu = Vector3.Dot(xd, uDir);
                        var xv = Vector3.Dot(xd, vDir);
                        var invDet = 1f / det;
                        var s = (xu * vv - xv * uv) * invDet;
                        var t = (xv * uu - xu * uv) * invDet;
                        //Route to the worst-violated parallelogram edge (violations scaled by edge length).
                        var sLow = -s;
                        var sHigh = s - 1f;
                        var tLow = -t;
                        var tHigh = t - 1f;
                        var uLen = MathF.Sqrt(uu);
                        var vLen = MathF.Sqrt(vv);
                        var worst = 0f;
                        var route = -1;
                        if (sLow > 0f && sLow * uLen > worst) { worst = sLow * uLen; route = 0; }
                        if (sHigh > 0f && sHigh * uLen > worst) { worst = sHigh * uLen; route = 1; }
                        if (tLow > 0f && tLow * vLen > worst) { worst = tLow * vLen; route = 2; }
                        if (tHigh > 0f && tHigh * vLen > worst) { worst = tHigh * vLen; route = 3; }
                        if (route < 0)
                        {
                            if (h < 0f)
                            {
                                depth = h;
                                normal = n;
                                closestOnB = verticesB[edgeB.Start] + dB * float.MaxNative(0f, float.MinNative(1f, s));
                                goto terminal;
                            }
                            goto stall;
                        }
                        switch (route)
                        {
                            case 0: kind = KindEdgeA; i1 = i0; i0 = edgeB.Start; break;              //s < 0: edge (bStart, eA)
                            case 1: kind = KindEdgeA; i1 = i0; i0 = edgeB.End; break;                //s > 1: edge (bEnd, eA)
                            case 2: kind = KindEdgeB; i0 = i1; i1 = edgeA.Start; break;              //t < 0: edge (eB, aStart)
                            default: kind = KindEdgeB; i0 = i1; i1 = edgeA.End; break;               //t > 1: edge (eB, aEnd)
                        }
                        break;
                    }
                case KindEdgeB:
                    {
                        //Edge (edgeB i0, vertexA i1): segment {b(s) - p(va)}.
                        ref var edgeB = ref edgesB[i0];
                        var va = i1;
                        Matrix3x3.Transform(verticesA[va], bLocalOrientationA, out var pva);
                        pva += localOffsetA;
                        var e0 = verticesB[edgeB.Start] - pva;
                        var d = verticesB[edgeB.End] - verticesB[edgeB.Start];
                        var dLengthSquared = d.LengthSquared();
                        var t = -Vector3.Dot(e0, d) / float.MaxNative(dLengthSquared, 1e-30f);
                        if (t <= 0f)
                        {
                            kind = KindVertex;
                            i0 = edgeB.Start;
                            break;
                        }
                        if (t >= 1f)
                        {
                            kind = KindVertex;
                            i0 = edgeB.End;
                            break;
                        }
                        var x = e0 + d * t;
                        var w = -x;
                        var dist = ScalarMath.Length(w);
                        if (dist < bestFeatureDist)
                        {
                            if (dist > touchTolerance)
                            {
                                bestFeatureDist = dist;
                                bestFeatureNormal = w * (1f / dist);
                                bestFeatureClosestOnB = verticesB[edgeB.Start] + d * t;
                            }
                            nonImprovingStreak = 0;
                        }
                        else if (++nonImprovingStreak > nonImprovingStreakCap)
                            goto stall;
                        var scoreEpsilon = float.MaxNative(1e-6f * dist, boundaryTolerance);
                        var bestScore = scoreEpsilon;
                        int bestKind = -1, best0 = 0, best1 = 0;
                        //Adjacent face-facet candidates: inward in-plane direction = cross(nFace, d_ccw); Face0's CCW
                        //loop walks Start->End (topology contract), so d_ccw = +d for Face0, -d for Face1.
                        {
                            var nF0 = faceNormalsB[edgeB.Face0];
                            var s0 = Vector3.Cross(nF0, d);
                            var score = Vector3.Dot(w, s0) / float.MaxNative(ScalarMath.Length(s0), 1e-30f);
                            if (score > bestScore) { bestScore = score; bestKind = KindFacetB; best0 = edgeB.Face0; best1 = va; }
                            var nF1 = faceNormalsB[edgeB.Face1];
                            var s1 = Vector3.Cross(nF1, -d);
                            score = Vector3.Dot(w, s1) / float.MaxNative(ScalarMath.Length(s1), 1e-30f);
                            if (score > bestScore) { bestScore = score; bestKind = KindFacetB; best0 = edgeB.Face1; best1 = va; }
                        }
                        //Adjacent EE-facet candidates: arcs of va's incident A edges crossing this B edge's arc; inward
                        //direction along the parallelogram = -dAp when va is eA.Start (t grows inward), +dAp when va is eA.End.
                        {
                            var slotStart = topologyA.VertexAdjacencyStarts[va];
                            var slotEnd = topologyA.VertexAdjacencyStarts[va + 1];
                            var b1 = faceNormalsB[edgeB.Face0];
                            var b2 = faceNormalsB[edgeB.Face1];
                            for (int slot = slotStart; slot < slotEnd; ++slot)
                            {
                                var eA = topologyA.VertexEdges[slot];
                                ref var edgeA = ref edgesA[eA];
                                var dALocal = verticesA[edgeA.End] - verticesA[edgeA.Start];
                                Matrix3x3.Transform(dALocal, bLocalOrientationA, out var dAp);
                                var t1 = Vector3.Dot(b1, dAp);
                                var t2 = Vector3.Dot(b2, dAp);
                                if (t1 * t2 >= 0f)
                                    continue;
                                Matrix3x3.Transform(faceNormalsA[edgeA.Face0], bLocalOrientationA, out var a1);
                                Matrix3x3.Transform(faceNormalsA[edgeA.Face1], bLocalOrientationA, out var a2);
                                var t3 = Vector3.Dot(a1, d);
                                var t4 = Vector3.Dot(a2, d);
                                if (t3 * t4 >= 0f || t1 * t4 >= 0f)
                                    continue;
                                var sDir = edgeA.Start == va ? -dAp : dAp;
                                var score = Vector3.Dot(w, sDir) / float.MaxNative(ScalarMath.Length(sDir), 1e-30f);
                                if (score > bestScore) { bestScore = score; bestKind = KindFacetEE; best0 = eA; best1 = i0; }
                            }
                        }
                        if (bestKind < 0)
                        {
                            if (dist <= touchTolerance)
                                goto stall;
                            //Voronoi conditions hold on the B side and across EE crossings. The A-side membership
                            //condition (va must be A's support along the claimed normal — the gauss-map cell test) is
                            //checked explicitly: routing through vertex states can carry a stale partner vertex, and a
                            //terminal on a non-cell is unsound. A violated condition tells us the fixed feature: move va.
                            var n = w * (1f / dist);
                            if (validityMoves < validityMoveCap)
                            {
                                var nInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                                HullSupportScalarHillclimb.Climb(topologyA, va, -nInA, out var vaWinner);
                                if (vaWinner != va)
                                {
                                    ++validityMoves;
                                    i1 = vaWinner;
                                    break;
                                }
                            }
                            depth = -dist;
                            normal = n;
                            closestOnB = verticesB[edgeB.Start] + d * t;
                            goto terminal;
                        }
                        kind = bestKind;
                        i0 = best0;
                        i1 = best1;
                        break;
                    }
                case KindEdgeA:
                    {
                        //Edge (vertexB i0, edgeA i1): segment {vb - p(a(t))}, direction -dAp.
                        var vb = i0;
                        ref var edgeA = ref edgesA[i1];
                        var vbPosition = verticesB[vb];
                        Matrix3x3.Transform(verticesA[edgeA.Start], bLocalOrientationA, out var paStart);
                        paStart += localOffsetA;
                        var dALocal = verticesA[edgeA.End] - verticesA[edgeA.Start];
                        Matrix3x3.Transform(dALocal, bLocalOrientationA, out var dAp);
                        var e0 = vbPosition - paStart;
                        var d = -dAp;
                        var dLengthSquared = d.LengthSquared();
                        var t = -Vector3.Dot(e0, d) / float.MaxNative(dLengthSquared, 1e-30f);
                        if (t <= 0f)
                        {
                            kind = KindVertex;
                            i1 = edgeA.Start;
                            break;
                        }
                        if (t >= 1f)
                        {
                            kind = KindVertex;
                            i1 = edgeA.End;
                            break;
                        }
                        var x = e0 + d * t;
                        var w = -x;
                        var dist = ScalarMath.Length(w);
                        if (dist < bestFeatureDist)
                        {
                            if (dist > touchTolerance)
                            {
                                bestFeatureDist = dist;
                                bestFeatureNormal = w * (1f / dist);
                                bestFeatureClosestOnB = vbPosition;
                            }
                            nonImprovingStreak = 0;
                        }
                        else if (++nonImprovingStreak > nonImprovingStreakCap)
                            goto stall;
                        var scoreEpsilon = float.MaxNative(1e-6f * dist, boundaryTolerance);
                        var bestScore = scoreEpsilon;
                        int bestKind = -1, best0 = 0, best1 = 0;
                        //Adjacent A-face facets: the reversed CSO loop traverses this edge along +dAp for Face0, -dAp
                        //for Face1; inward = cross(n_cso, d_ccw) with n_cso = -R * n_face.
                        {
                            Matrix3x3.Transform(faceNormalsA[edgeA.Face0], bLocalOrientationA, out var m0);
                            var s0 = Vector3.Cross(-m0, dAp);
                            var score = Vector3.Dot(w, s0) / float.MaxNative(ScalarMath.Length(s0), 1e-30f);
                            if (score > bestScore) { bestScore = score; bestKind = KindFacetA; best0 = vb; best1 = edgeA.Face0; }
                            Matrix3x3.Transform(faceNormalsA[edgeA.Face1], bLocalOrientationA, out var m1);
                            var s1 = Vector3.Cross(-m1, -dAp);
                            score = Vector3.Dot(w, s1) / float.MaxNative(ScalarMath.Length(s1), 1e-30f);
                            if (score > bestScore) { bestScore = score; bestKind = KindFacetA; best0 = vb; best1 = edgeA.Face1; }
                        }
                        //Adjacent EE facets: vb's incident B edges whose arcs cross; inward = +dB when vb is eB.Start,
                        //-dB when vb is eB.End (s grows inward from the s=0 edge).
                        {
                            Matrix3x3.Transform(faceNormalsA[edgeA.Face0], bLocalOrientationA, out var a1);
                            Matrix3x3.Transform(faceNormalsA[edgeA.Face1], bLocalOrientationA, out var a2);
                            var slotStart = topologyB.VertexAdjacencyStarts[vb];
                            var slotEnd = topologyB.VertexAdjacencyStarts[vb + 1];
                            for (int slot = slotStart; slot < slotEnd; ++slot)
                            {
                                var eB = topologyB.VertexEdges[slot];
                                ref var edgeB = ref edgesB[eB];
                                var dB = verticesB[edgeB.End] - verticesB[edgeB.Start];
                                var t1 = Vector3.Dot(faceNormalsB[edgeB.Face0], dAp);
                                var t2 = Vector3.Dot(faceNormalsB[edgeB.Face1], dAp);
                                if (t1 * t2 >= 0f)
                                    continue;
                                var t3 = Vector3.Dot(a1, dB);
                                var t4 = Vector3.Dot(a2, dB);
                                if (t3 * t4 >= 0f || t1 * t4 >= 0f)
                                    continue;
                                var sDir = edgeB.Start == vb ? dB : -dB;
                                var score = Vector3.Dot(w, sDir) / float.MaxNative(ScalarMath.Length(sDir), 1e-30f);
                                if (score > bestScore) { bestScore = score; bestKind = KindFacetEE; best0 = i1; best1 = eB; }
                            }
                        }
                        if (bestKind < 0)
                        {
                            if (dist <= touchTolerance)
                                goto stall;
                            //B-side gauss-membership check (see the EdgeB terminal note): vb must be B's support along
                            //the claimed normal; a violation moves vb instead of terminating.
                            var n = w * (1f / dist);
                            if (validityMoves < validityMoveCap)
                            {
                                HullSupportScalarHillclimb.Climb(topologyB, vb, n, out var vbWinner);
                                if (vbWinner != vb)
                                {
                                    ++validityMoves;
                                    i0 = vbWinner;
                                    break;
                                }
                            }
                            depth = -dist;
                            normal = n;
                            closestOnB = vbPosition;
                            goto terminal;
                        }
                        kind = bestKind;
                        i0 = best0;
                        i1 = best1;
                        break;
                    }
                default:
                    {
                        //Vertex (vertexB i0, vertexA i1): c = vb - p(va); improving emanating CSO edges have dot(w, dir) > 0.
                        var vb = i0;
                        var va = i1;
                        Matrix3x3.Transform(verticesA[va], bLocalOrientationA, out var pva);
                        pva += localOffsetA;
                        var c = verticesB[vb] - pva;
                        var w = -c;
                        var dist = ScalarMath.Length(w);
                        if (dist < bestFeatureDist)
                        {
                            if (dist > touchTolerance)
                            {
                                bestFeatureDist = dist;
                                bestFeatureNormal = w * (1f / dist);
                                bestFeatureClosestOnB = verticesB[vb];
                            }
                            nonImprovingStreak = 0;
                        }
                        else if (++nonImprovingStreak > nonImprovingStreakCap)
                            goto stall;
                        var scoreEpsilon = float.MaxNative(1e-6f * dist, boundaryTolerance);
                        var bestScore = scoreEpsilon;
                        int bestKind = -1, best0 = 0, best1 = 0;
                        {
                            var slotStart = topologyB.VertexAdjacencyStarts[vb];
                            var slotEnd = topologyB.VertexAdjacencyStarts[vb + 1];
                            for (int slot = slotStart; slot < slotEnd; ++slot)
                            {
                                var neighbor = topologyB.AdjacentVertices[slot];
                                var dir = verticesB[neighbor] - verticesB[vb];
                                var score = Vector3.Dot(w, dir) / float.MaxNative(ScalarMath.Length(dir), 1e-30f);
                                if (score > bestScore) { bestScore = score; bestKind = KindEdgeB; best0 = topologyB.VertexEdges[slot]; best1 = va; }
                            }
                        }
                        {
                            var slotStart = topologyA.VertexAdjacencyStarts[va];
                            var slotEnd = topologyA.VertexAdjacencyStarts[va + 1];
                            for (int slot = slotStart; slot < slotEnd; ++slot)
                            {
                                var neighbor = topologyA.AdjacentVertices[slot];
                                Matrix3x3.Transform(verticesA[neighbor] - verticesA[va], bLocalOrientationA, out var dAp);
                                var dir = -dAp;
                                var score = Vector3.Dot(w, dir) / float.MaxNative(ScalarMath.Length(dir), 1e-30f);
                                if (score > bestScore) { bestScore = score; bestKind = KindEdgeA; best0 = vb; best1 = topologyA.VertexEdges[slot]; }
                            }
                        }
                        if (bestKind < 0)
                        {
                            if (dist <= touchTolerance)
                                goto stall;
                            //Both gauss-membership conditions checked explicitly (vb support along n, va support along
                            //-R^T n); a violation moves the offending vertex instead of terminating.
                            var n = w * (1f / dist);
                            if (validityMoves < validityMoveCap)
                            {
                                HullSupportScalarHillclimb.Climb(topologyB, vb, n, out var vbWinner);
                                if (vbWinner != vb)
                                {
                                    ++validityMoves;
                                    i0 = vbWinner;
                                    break;
                                }
                                var nInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                                HullSupportScalarHillclimb.Climb(topologyA, va, -nInA, out var vaWinner);
                                if (vaWinner != va)
                                {
                                    ++validityMoves;
                                    i1 = vaWinner;
                                    break;
                                }
                            }
                            depth = -dist;
                            normal = n;
                            closestOnB = verticesB[vb];
                            goto terminal;
                        }
                        kind = bestKind;
                        i0 = best0;
                        i1 = best1;
                        break;
                    }
            }
        }
        ++CsoStats.SepCapHits;
    stall:
        //Accept the best feature point seen (a genuine witness on D) when it is well-conditioned; otherwise the
        //most-negative certified axis with its support witness. Counted; the comparator judges the damage.
        ++CsoStats.SepStalls;
        if (degenerate)
            ++CsoStats.SepDegenerate;
        LastSepStalled = true;
        if (bestFeatureDist < float.MaxValue && bestFeatureDist > touchTolerance)
        {
            //Self-certification: if supportDepth(w-direction) == -dist, the feature IS the global closest (complete
            //separating-plane certificate) and the stall was just the cone tests failing to recognize a terminal.
            //Otherwise the w-direction is not a supporting normal; the certified axis is the honest answer.
            var nInA = ScalarMath.TransformByTransposed(bestFeatureNormal, bLocalOrientationA);
            var certifiedDepth = ClimbDot(topologyB, ref warmB, bestFeatureNormal) - Vector3.Dot(localOffsetA, bestFeatureNormal) + ClimbDot(topologyA, ref warmA, -nInA);
            if (certifiedDepth < bkDepth)
            {
                bkDepth = certifiedDepth;
                bkNormal = bestFeatureNormal;
            }
            if (certifiedDepth + bestFeatureDist <= 1e-4f * epsilonScale)
            {
                depth = -bestFeatureDist;
                normal = bestFeatureNormal;
                closestOnB = bestFeatureClosestOnB;
                goto terminal;
            }
        }
        depth = bkDepth;
        normal = bkNormal;
        closestOnB = HullSupportScalarHillclimb.Climb(topologyB, warmB, bkNormal, out warmB);
    terminal:
        ++CsoStats.SepStepHistogram[Math.Min(iteration, CsoStats.SepStepHistogram.Length - 1)];
        CsoStats.SepSteps += iteration;
        return depth >= depthThreshold;
    }
}
