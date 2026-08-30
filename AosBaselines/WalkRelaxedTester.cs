using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

/// <summary>
/// Track 3 relaxed candidate: local SAT via hillclimbing the axis space. Instead of the DepthRefiner's iteration loop
/// (Track 1) or full SAT's exhaustive face scans + O(Ea*Eb) gauss-filtered edge sweep (Track 2), this walks each hull's
/// face-adjacency graph by steepest descent on exact axis depth (per-axis depth = precomputed own-face support offset +
/// one warm-started adjacency-climb support on the other hull, so each face evaluation is O(vertex degree), not O(V)).
/// Edge-edge axes are only examined in the gauss neighborhood of the incumbent axis: edges of faces incident to the
/// support vertices along the current best axis (ring-expandable), gauss-arc filtered, iterated to a fixed point as the
/// axis moves. Face picks for manifold generation come from the topology (most-aligned face incident to the support
/// vertex) instead of the O(F) PickRepresentativeFace scan. The separated-within-margin band falls back to the scalar
/// DepthRefiner with hillclimb supports (Track 1 configuration), because SAT axes cannot represent vertex-region
/// distance directions (Track 2 finding). Axis-space descent is NOT convex (local minima exist); the walkoracle mode
/// quantifies local-vs-global misses against Track 2's exhaustive SAT.
/// </summary>
public sealed class WalkRelaxedTester : IRelaxedHullPairTester
{
    public string Name => "walk";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        WalkHullPairTester.Test(topologyA, topologyB, ref a, ref b, speculativeMargin, offsetB, orientationA, orientationB,
            ringDepth: 0, useFallback: true, out manifold);
    }
}

/// <summary>Walk candidate with a one-larger vertex ring for the edge-phase restriction (quantifies ring-expansion cost/benefit).</summary>
public sealed class Walk2RelaxedTester : IRelaxedHullPairTester
{
    public string Name => "walk2";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        WalkHullPairTester.Test(topologyA, topologyB, ref a, ref b, speculativeMargin, offsetB, orientationA, orientationB,
            ringDepth: 1, useFallback: true, out manifold);
    }
}

/// <summary>Cheap always-on statistics for the walk tester (static adds; negligible vs pair cost).</summary>
public static class WalkStats
{
    public static long Pairs, PrepassRejects, FaceRejects, EdgeRejects, FallbackRejects, Accepted;
    public static long WinnerFaceA, WinnerFaceB, WinnerEdge, WinnerExtra;
    public static long FaceEvals, WalkSteps, EdgeRounds, EdgeCandidatePairs, EdgeGaussSurvivors, EdgeRescans, EdgeImprovements;
    public static long FallbackRuns, EdgeBufferOverflows;
    public static void Reset()
    {
        Pairs = PrepassRejects = FaceRejects = EdgeRejects = FallbackRejects = Accepted = 0;
        WinnerFaceA = WinnerFaceB = WinnerEdge = WinnerExtra = 0;
        FaceEvals = WalkSteps = EdgeRounds = EdgeCandidatePairs = EdgeGaussSurvivors = EdgeRescans = EdgeImprovements = 0;
        FallbackRuns = EdgeBufferOverflows = 0;
    }
    public static void Print(string label)
    {
        Console.WriteLine($"{label}: pairs {Pairs}, accepted {Accepted} ({(double)Accepted / Math.Max(1, Pairs):P1}); rejects: prepass {PrepassRejects}, face {FaceRejects}, edge {EdgeRejects}, fallback {FallbackRejects}");
        Console.WriteLine($"    winners: faceA {WinnerFaceA}, faceB {WinnerFaceB}, edge {WinnerEdge}, extra(prepass/fallback) {WinnerExtra}");
        Console.WriteLine($"    face walk: {(double)FaceEvals / Math.Max(1, Pairs):F1} evals/pair, {(double)WalkSteps / Math.Max(1, Pairs):F2} moves/pair");
        Console.WriteLine($"    edge phase: {(double)EdgeRounds / Math.Max(1, Pairs):F2} rounds/pair, {(double)EdgeCandidatePairs / Math.Max(1, EdgeRounds):F1} pairs/round, " +
            $"gauss survivors {EdgeGaussSurvivors} ({(double)EdgeGaussSurvivors / Math.Max(1, EdgeCandidatePairs):P2}), rescans {EdgeRescans}, improvements {EdgeImprovements}, buffer overflows {EdgeBufferOverflows}");
        Console.WriteLine($"    fallback (refiner) runs: {FallbackRuns} ({(double)FallbackRuns / Math.Max(1, Pairs):P1} of pairs)");
    }
}

public static class WalkHullPairTester
{
    /// <summary>Depth of the decisive axis of the most recent Test call (the rejecting axis on rejects, the search minimum on accepts). Triage aid.</summary>
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

    /// <summary>Exact support dot via warm-started adjacency climb (exact by convexity; Track 1's climbprobe evidence).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float ClimbDot(HullTopology topology, ref int warm, Vector3 direction)
    {
        var support = HullSupportScalarHillclimb.Climb(topology, warm, direction, out warm);
        return Vector3.Dot(support, direction);
    }

    /// <summary>Most-aligned face among those incident to the support vertex. For a support vertex along the query
    /// direction this matches the frozen PickRepresentativeFace's zero-plane-error preference in O(degree).</summary>
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

    /// <summary>
    /// Collects the candidate edge set around a support vertex: all edges of all faces incident to the vertices within
    /// ringDepth adjacency hops of the support vertex (ringDepth 0 = the support vertex alone). Deduplicated; returns count.
    /// </summary>
    static int CollectCandidateEdges(HullTopology topology, int supportVertex, int ringDepth, Span<int> edgeBuffer)
    {
        Span<int> ringVertices = stackalloc int[64];
        int ringCount = 0;
        ringVertices[ringCount++] = supportVertex;
        int layerStart = 0;
        for (int layer = 0; layer < ringDepth; ++layer)
        {
            int layerEnd = ringCount;
            for (int i = layerStart; i < layerEnd; ++i)
            {
                var v = ringVertices[i];
                var adjacencyStart = topology.VertexAdjacencyStarts[v];
                var adjacencyEnd = topology.VertexAdjacencyStarts[v + 1];
                for (int j = adjacencyStart; j < adjacencyEnd; ++j)
                {
                    var neighbor = topology.AdjacentVertices[j];
                    bool present = false;
                    for (int m = 0; m < ringCount; ++m)
                    {
                        if (ringVertices[m] == neighbor)
                        {
                            present = true;
                            break;
                        }
                    }
                    if (!present)
                    {
                        if (ringCount == ringVertices.Length)
                            goto verticesDone;
                        ringVertices[ringCount++] = neighbor;
                    }
                }
            }
            layerStart = layerEnd;
        }
    verticesDone:
        int count = 0;
        for (int i = 0; i < ringCount; ++i)
        {
            var v = ringVertices[i];
            var facesStart = topology.VertexFaceStarts[v];
            var facesEnd = topology.VertexFaceStarts[v + 1];
            for (int j = facesStart; j < facesEnd; ++j)
            {
                var face = topology.VertexFaces[j];
                var edgesStart = topology.FaceStarts[face];
                var edgesEnd = topology.FaceStarts[face + 1];
                for (int k = edgesStart; k < edgesEnd; ++k)
                {
                    var edge = topology.FaceEdges[k];
                    bool present = false;
                    for (int m = 0; m < count; ++m)
                    {
                        if (edgeBuffer[m] == edge)
                        {
                            present = true;
                            break;
                        }
                    }
                    if (!present)
                    {
                        if (count == edgeBuffer.Length)
                        {
                            ++WalkStats.EdgeBufferOverflows;
                            return count;
                        }
                        edgeBuffer[count++] = edge;
                    }
                }
            }
        }
        return count;
    }

    public static unsafe void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        int ringDepth, bool useFallback, out Convex4ManifoldScalar manifold)
    {
        manifold = default;
        ++WalkStats.Pairs;
        LastWinnerType = -1;
        LastClipCandidateCount = 0;
        Matrix3x3.CreateFromQuaternion(orientationA, out Matrix3x3 rA);
        Matrix3x3.CreateFromQuaternion(orientationB, out Matrix3x3 rB);
        //bLocalOrientationA maps A-local into B-local; everything below is in B's local space, normal convention B -> A.
        ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, rB);
        var localOffsetA = -localOffsetB;

        var firstPointA = topologyA.Vertices[0];
        var firstPointB = topologyB.Vertices[0];
        var aEpsilonScale = (MathF.Abs(firstPointA.X) + MathF.Abs(firstPointA.Y) + MathF.Abs(firstPointA.Z)) * (1f / 3f);
        var bEpsilonScale = (MathF.Abs(firstPointB.X) + MathF.Abs(firstPointB.Y) + MathF.Abs(firstPointB.Z)) * (1f / 3f);
        var epsilonScale = float.MinNative(aEpsilonScale, bEpsilonScale);
        var depthThreshold = -speculativeMargin;

        //Warm-start slots for the adjacency-climb supports; every depth evaluation continues from the previous winner.
        int warmA = 0, warmB = 0;

        //Depth along a B->A unit direction n (B-local): depth(n) = max_B dot(p, n) - dot(localOffsetA, n) + max_A dot(p, -R^T n).
        //Any direction's depth upper-bounds the true minimum: depth < -margin certifies rejection.

        //Prepass: center-to-center axis.
        var centerDistance = ScalarMath.Length(localOffsetA);
        var u = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
        var uInA = ScalarMath.TransformByTransposed(u, bLocalOrientationA);
        var bestDepth = ClimbDot(topologyB, ref warmB, u) - Vector3.Dot(localOffsetA, u) + ClimbDot(topologyA, ref warmA, -uInA);
        var bestNormal = u;
        var bestType = WinnerType.Extra;
        int bestFaceA = 0, bestFaceB = 0, bestEdgeA = 0, bestEdgeB = 0;
        if (bestDepth < depthThreshold)
        {
            ++WalkStats.PrepassRejects;
            LastDepth = bestDepth;
            LastLocalNormal = u;
            return;
        }
        //Any negative-depth axis proves separation; the separated band's manifold comes from the refiner fallback
        //regardless, so further axis search would be discarded work (mirrors the sat candidate's structure).
        var skipToFallback = useFallback && bestDepth < 0f;

        if (!skipToFallback)
        {
            //Outer restart loop (coordinate descent in axis space): when the A walk or the edge phase moves the
            //incumbent axis after B's walk finished, B's neighborhood around the NEW axis hasn't been searched; re-walk
            //both hulls around the incumbent and repeat until a full round brings no improvement. Deep-overlap cases
            //are where restarts matter — the center-to-center start direction is nearly meaningless there.
            const int maxOuterRounds = 3;
            Span<int> edgesABuffer = stackalloc int[128];
            Span<int> edgesBBuffer = stackalloc int[128];
            int previousVA = -1, previousVB = -1;
            for (int outer = 0; outer < maxOuterRounds && !skipToFallback; ++outer)
            {
            var depthAtRoundStart = bestDepth;
            //Face walk on B: steepest descent on exact axis depth over B's face-adjacency graph. Start face:
            //most-aligned face incident to B's support vertex along the start axis (round 0: the center direction,
            //whose B support climb the prepass already did; restarts: the incumbent axis).
            {
                var startAxis = u;
                if (outer > 0)
                {
                    startAxis = bestNormal;
                    HullSupportScalarHillclimb.Climb(topologyB, warmB, startAxis, out warmB);
                }
                var face = PickIncidentFace(topologyB, warmB, startAxis, out _);
                var n = topologyB.FaceNormals[face];
                var dirInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                var currentDepth = topologyB.FaceSupportOffsets[face] - Vector3.Dot(localOffsetA, n) + ClimbDot(topologyA, ref warmA, -dirInA);
                ++WalkStats.FaceEvals;
                if (currentDepth < depthThreshold)
                {
                    ++WalkStats.FaceRejects;
                    LastDepth = currentDepth;
                    LastLocalNormal = n;
                    return;
                }
                if (currentDepth < bestDepth)
                {
                    bestDepth = currentDepth;
                    bestNormal = n;
                    bestType = WinnerType.FaceB;
                    bestFaceB = face;
                    if (useFallback && bestDepth < 0f)
                        skipToFallback = true;
                }
                for (int step = 0; step < topologyB.FaceCount && !skipToFallback; ++step)
                {
                    var neighborsStart = topologyB.FaceStarts[face];
                    var neighborsEnd = topologyB.FaceStarts[face + 1];
                    var bestNeighbor = -1;
                    var bestNeighborDepth = currentDepth;
                    Vector3 bestNeighborNormal = default;
                    for (int i = neighborsStart; i < neighborsEnd; ++i)
                    {
                        var neighbor = topologyB.AdjacentFaces[i];
                        var nn = topologyB.FaceNormals[neighbor];
                        var dInA = ScalarMath.TransformByTransposed(nn, bLocalOrientationA);
                        var depth = topologyB.FaceSupportOffsets[neighbor] - Vector3.Dot(localOffsetA, nn) + ClimbDot(topologyA, ref warmA, -dInA);
                        ++WalkStats.FaceEvals;
                        if (depth < depthThreshold)
                        {
                            ++WalkStats.FaceRejects;
                            LastDepth = depth;
                            LastLocalNormal = nn;
                            return;
                        }
                        if (depth < bestNeighborDepth)
                        {
                            bestNeighborDepth = depth;
                            bestNeighbor = neighbor;
                            bestNeighborNormal = nn;
                        }
                    }
                    if (bestNeighbor < 0)
                        break;
                    face = bestNeighbor;
                    currentDepth = bestNeighborDepth;
                    ++WalkStats.WalkSteps;
                    if (currentDepth < bestDepth)
                    {
                        bestDepth = currentDepth;
                        bestNormal = bestNeighborNormal;
                        bestType = WinnerType.FaceB;
                        bestFaceB = face;
                        if (useFallback && bestDepth < 0f)
                            skipToFallback = true;
                    }
                }
            }

            //Face walk on A: same descent over A's faces; the B->A axis for face f of A is -R * n_f.
            if (!skipToFallback)
            {
                //Start near the incumbent axis: A's support vertex along -R^T bestNormal.
                var incumbentInA = ScalarMath.TransformByTransposed(bestNormal, bLocalOrientationA);
                var startDir = -incumbentInA;
                HullSupportScalarHillclimb.Climb(topologyA, warmA, startDir, out warmA);
                var face = PickIncidentFace(topologyA, warmA, startDir, out _);
                float EvaluateFaceA(int f, ref int warmBSlot, out Vector3 axis)
                {
                    Matrix3x3.Transform(topologyA.FaceNormals[f], bLocalOrientationA, out var m);
                    axis = -m;
                    return ClimbDot(topologyB, ref warmBSlot, axis) + Vector3.Dot(localOffsetA, m) + topologyA.FaceSupportOffsets[f];
                }
                var currentDepth = EvaluateFaceA(face, ref warmB, out var currentAxis);
                ++WalkStats.FaceEvals;
                if (currentDepth < depthThreshold)
                {
                    ++WalkStats.FaceRejects;
                    LastDepth = currentDepth;
                    LastLocalNormal = currentAxis;
                    return;
                }
                if (currentDepth < bestDepth)
                {
                    bestDepth = currentDepth;
                    bestNormal = currentAxis;
                    bestType = WinnerType.FaceA;
                    bestFaceA = face;
                    if (useFallback && bestDepth < 0f)
                        skipToFallback = true;
                }
                for (int step = 0; step < topologyA.FaceCount && !skipToFallback; ++step)
                {
                    var neighborsStart = topologyA.FaceStarts[face];
                    var neighborsEnd = topologyA.FaceStarts[face + 1];
                    var bestNeighbor = -1;
                    var bestNeighborDepth = currentDepth;
                    Vector3 bestNeighborAxis = default;
                    for (int i = neighborsStart; i < neighborsEnd; ++i)
                    {
                        var neighbor = topologyA.AdjacentFaces[i];
                        var depth = EvaluateFaceA(neighbor, ref warmB, out var axis);
                        ++WalkStats.FaceEvals;
                        if (depth < depthThreshold)
                        {
                            ++WalkStats.FaceRejects;
                            LastDepth = depth;
                            LastLocalNormal = axis;
                            return;
                        }
                        if (depth < bestNeighborDepth)
                        {
                            bestNeighborDepth = depth;
                            bestNeighbor = neighbor;
                            bestNeighborAxis = axis;
                        }
                    }
                    if (bestNeighbor < 0)
                        break;
                    face = bestNeighbor;
                    currentDepth = bestNeighborDepth;
                    ++WalkStats.WalkSteps;
                    if (currentDepth < bestDepth)
                    {
                        bestDepth = currentDepth;
                        bestNormal = bestNeighborAxis;
                        bestType = WinnerType.FaceA;
                        bestFaceA = face;
                        if (useFallback && bestDepth < 0f)
                            skipToFallback = true;
                    }
                }
            }

            //Edge-edge phase, restricted to the gauss neighborhood of the incumbent axis: edges of faces incident to the
            //support vertices along the current best axis. Iterated: an improving edge axis moves the support vertices,
            //so re-collect and retest until a fixed point (or the round cap).
            if (!skipToFallback)
            {
                const int maxRounds = 4;
                for (int round = 0; round < maxRounds && !skipToFallback; ++round)
                {
                    var nInA = ScalarMath.TransformByTransposed(bestNormal, bLocalOrientationA);
                    HullSupportScalarHillclimb.Climb(topologyA, warmA, -nInA, out warmA);
                    HullSupportScalarHillclimb.Climb(topologyB, warmB, bestNormal, out warmB);
                    if (warmA == previousVA && warmB == previousVB)
                        break;
                    previousVA = warmA;
                    previousVB = warmB;
                    ++WalkStats.EdgeRounds;
                    var edgeCountA = CollectCandidateEdges(topologyA, warmA, ringDepth, edgesABuffer);
                    var edgeCountB = CollectCandidateEdges(topologyB, warmB, ringDepth, edgesBBuffer);
                    WalkStats.EdgeCandidatePairs += edgeCountA * edgeCountB;
                    bool improved = false;
                    for (int i = 0; i < edgeCountA && !skipToFallback; ++i)
                    {
                        var eA = edgesABuffer[i];
                        ref var edgeA = ref topologyA.Edges[eA];
                        var dALocal = topologyA.Vertices[edgeA.End] - topologyA.Vertices[edgeA.Start];
                        Matrix3x3.Transform(dALocal, bLocalOrientationA, out var dA);
                        Matrix3x3.Transform(topologyA.FaceNormals[edgeA.Face0], bLocalOrientationA, out var a1);
                        Matrix3x3.Transform(topologyA.FaceNormals[edgeA.Face1], bLocalOrientationA, out var a2);
                        Matrix3x3.Transform(topologyA.Vertices[edgeA.Start], bLocalOrientationA, out var rvAStart);
                        var pAStart = rvAStart + localOffsetA;
                        var dALengthSquared = dA.LengthSquared();
                        for (int j = 0; j < edgeCountB; ++j)
                        {
                            var eB = edgesBBuffer[j];
                            ref var edgeB = ref topologyB.Edges[eB];
                            //Gauss-arc (Minkowski face) test, cross-free form (topology guarantees dir = +cross(N0, N1)):
                            //arcs intersect iff (b1.dA)(b2.dA) < 0 and (a1.dB)(a2.dB) < 0 and (b1.dA)(a2.dB) < 0.
                            var b1 = topologyB.FaceNormals[edgeB.Face0];
                            var b2 = topologyB.FaceNormals[edgeB.Face1];
                            var t1 = Vector3.Dot(b1, dA);
                            var t2 = Vector3.Dot(b2, dA);
                            if (t1 * t2 >= 0f)
                                continue;
                            var dB = topologyB.Vertices[edgeB.End] - topologyB.Vertices[edgeB.Start];
                            var t3 = Vector3.Dot(a1, dB);
                            var t4 = Vector3.Dot(a2, dB);
                            if (t3 * t4 >= 0f || t1 * t4 >= 0f)
                                continue;
                            ++WalkStats.EdgeGaussSurvivors;
                            var axis = Vector3.Cross(dA, dB);
                            var axisLengthSquared = axis.LengthSquared();
                            //Near-parallel edge pairs produce degenerate axes; the face axes cover those directions.
                            if (axisLengthSquared < 1e-10f * dALengthSquared * dB.LengthSquared())
                                continue;
                            //Calibrate B->A via A's centroid side; sqrt-free improvement test on the unnormalized
                            //edge-point depth (never overstates the exact interval depth; see the sat candidate notes).
                            var calibrationDot = Vector3.Dot(axis, rvAStart);
                            var pBStart = topologyB.Vertices[edgeB.Start];
                            var raw = Vector3.Dot(pBStart - pAStart, axis);
                            if (calibrationDot > 0)
                                raw = -raw;
                            bool improves = bestDepth >= 0f
                                ? raw < 0f || raw * raw < bestDepth * bestDepth * axisLengthSquared
                                : raw < 0f && raw * raw > bestDepth * bestDepth * axisLengthSquared;
                            if (improves)
                            {
                                var n = axis * ((calibrationDot > 0 ? -1f : 1f) / MathF.Sqrt(axisLengthSquared));
                                //Exact interval rescan (climbs are exact supports) before the axis can steer the winner.
                                ++WalkStats.EdgeRescans;
                                var candidateNInA = ScalarMath.TransformByTransposed(n, bLocalOrientationA);
                                var depth = ClimbDot(topologyB, ref warmB, n) - Vector3.Dot(localOffsetA, n) + ClimbDot(topologyA, ref warmA, -candidateNInA);
                                if (depth >= bestDepth)
                                    continue;
                                if (depth < depthThreshold)
                                {
                                    ++WalkStats.EdgeRejects;
                                    LastDepth = depth;
                                    LastLocalNormal = n;
                                    return;
                                }
                                ++WalkStats.EdgeImprovements;
                                bestDepth = depth;
                                bestNormal = n;
                                bestType = WinnerType.Edge;
                                bestEdgeA = eA;
                                bestEdgeB = eB;
                                improved = true;
                                if (useFallback && bestDepth < 0f)
                                {
                                    skipToFallback = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (!improved)
                        break;
                }
            }
            //Restart decision: a strict improvement this round means the incumbent axis moved after some walk finished;
            //search its neighborhood again. No improvement = fixed point.
            if (bestDepth >= depthAtRoundStart)
                break;
            }
        }

        //Separated-within-margin fallback: SAT-style axis sets cannot represent vertex-region distance directions
        //(Track 2 finding), so the speculative band uses the scalar DepthRefiner with hillclimb supports (Track 1
        //configuration, center-direction seed for reference-like trajectories).
        var haveRefinerWitness = false;
        Vector3 refinerClosestOnB = default;
        if (useFallback && (skipToFallback || bestDepth < 0f))
        {
            ++WalkStats.FallbackRuns;
            var initialNormal = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
            int fallbackWarmA = warmA, fallbackWarmB = warmB;
            var refinerShapeA = new HillclimbHull { Topology = topologyB, WarmStartSlot = &fallbackWarmB };
            var refinerShapeB = new HillclimbHull { Topology = topologyA, WarmStartSlot = &fallbackWarmA };
            ScalarDepthRefiner<HillclimbHull, HullSupportScalarHillclimb, HillclimbHull, HullSupportScalarHillclimb>.FindMinimumDepth(
                refinerShapeA, refinerShapeB, localOffsetA, bLocalOrientationA, initialNormal, 1e-5f * epsilonScale, depthThreshold,
                out var refinedDepth, out var refinedNormal, out refinerClosestOnB);
            if (refinedDepth < depthThreshold)
            {
                ++WalkStats.FallbackRejects;
                LastDepth = refinedDepth;
                LastLocalNormal = refinedNormal;
                return;
            }
            bestDepth = refinedDepth;
            bestNormal = refinedNormal;
            bestType = WinnerType.Extra;
            haveRefinerWitness = true;
            warmA = fallbackWarmA;
            warmB = fallbackWarmB;
        }

        ++WalkStats.Accepted;
        LastDepth = bestDepth;
        LastLocalNormal = bestNormal;
        switch (bestType)
        {
            case WinnerType.FaceA: ++WalkStats.WinnerFaceA; break;
            case WinnerType.FaceB: ++WalkStats.WinnerFaceB; break;
            case WinnerType.Edge: ++WalkStats.WinnerEdge; break;
            default: ++WalkStats.WinnerExtra; break;
        }

        //Manifold generation: representative faces from the topology (O(degree) picks; the walk already knows the
        //winning face where a face axis won), then the sat candidate's clip + Reduce + edge-degenerate fallback.
        var localNormal = bestNormal;
        var localNormalInA = ScalarMath.TransformByTransposed(localNormal, bLocalOrientationA);
        var negatedLocalNormalInA = -localNormalInA;

        Vector3 slotFaceNormalAInA, slotFaceNormalB;
        int bestFaceIndexA, bestFaceIndexB;
        switch (bestType)
        {
            case WinnerType.FaceB:
                {
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
                    //The incident face of the winning edge that better aligns with the axis on each hull.
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
                    if (haveRefinerWitness)
                    {
                        //Refiner-fallback axis (the separated-within-margin band): the O(degree) incident pick along the
                        //normal is NOT robust here — on speculative pairs the support vertex's most-aligned face can
                        //have a projection that misses the other face entirely, emptying the clip (measured: ~0.17% of
                        //cases as candidate-empty flips). Use the frozen plane-error-banded picker with the refiner's
                        //witness point, exactly like the frozen tester and the sat candidate.
                        var boundingPlaneEpsilon = 1e-3f * epsilonScale;
                        var closestOnA = refinerClosestOnB - localNormal * bestDepth;
                        var aToClosestOnA = closestOnA - localOffsetA;
                        var closestOnAInA = ScalarMath.TransformByTransposed(aToClosestOnA, bLocalOrientationA);
                        ConvexHullPairScalarTester.PickRepresentativeFace(ref a, negatedLocalNormalInA, closestOnAInA, boundingPlaneEpsilon, out slotFaceNormalAInA, out bestFaceIndexA);
                        ConvexHullPairScalarTester.PickRepresentativeFace(ref b, localNormal, refinerClosestOnB, boundingPlaneEpsilon, out slotFaceNormalB, out bestFaceIndexB);
                    }
                    else
                    {
                        //Prepass axis won outright (rare): support-vertex incident picks.
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
        Matrix3x3.Transform(localNormal, rB, out manifold.Normal);
    }
}
