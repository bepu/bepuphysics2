using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

/// <summary>
/// Shape adapter handing the hillclimb support finder its per-hull topology plus a per-pair-test warm start slot.
/// The pointer targets a stack local in the tester body (one per hull role in the pair); the refiner's successive support
/// directions are strongly correlated, so the previous query's winner is an excellent start for the next adjacency walk.
/// Relaxed-equality code: NOT bitwise-mirroring anything, judged by the tolerance comparator and speed.
/// </summary>
public unsafe struct HillclimbHull
{
    public HullTopology Topology;
    public int* WarmStartSlot;
}

/// <summary>
/// Warm-started adjacency hillclimb support finder: starts at the cached vertex index and greedily walks the CSR vertex
/// adjacency to any strictly-better neighbor until a local (= global, by convexity) maximum of dot(vertex, direction).
/// O(few) steps per query on correlated direction sequences versus the frozen full scan's O(N).
/// Tie behavior differs from the frozen scan (which takes the smallest encoded index among exact ties): the climb stops at
/// the first tied vertex it reaches, so exact-tie winners can differ — a legitimate-disagreement class for the comparator.
/// </summary>
public unsafe struct HullSupportScalarHillclimb : IScalarSupportFinder<HillclimbHull>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector3 Climb(HullTopology topology, int start, Vector3 direction, out int winner)
    {
        //Indices come from validated topology (closed 2-manifold), so bounds checks are skipped via refs.
        ref var vertices = ref MemoryMarshal.GetArrayDataReference(topology.Vertices);
        ref var adjacencyStarts = ref MemoryMarshal.GetArrayDataReference(topology.VertexAdjacencyStarts);
        ref var adjacency = ref MemoryMarshal.GetArrayDataReference(topology.AdjacentVertices);
        var best = start;
        var bestDot = Vector3.Dot(Unsafe.Add(ref vertices, best), direction);
        //Strict improvement means every accepted move strictly raises the support dot, so a vertex can win at most once and
        //the walk provably terminates within VertexCount rounds; the loop bound is a belt-and-suspenders guard, not a limiter.
        for (int round = 0; round < topology.VertexCount; ++round)
        {
            var neighborsStart = Unsafe.Add(ref adjacencyStarts, best);
            var neighborsEnd = Unsafe.Add(ref adjacencyStarts, best + 1);
            var previousBest = best;
            for (int i = neighborsStart; i < neighborsEnd; ++i)
            {
                var neighbor = Unsafe.Add(ref adjacency, i);
                var dot = Vector3.Dot(Unsafe.Add(ref vertices, neighbor), direction);
                if (dot > bestDot)
                {
                    bestDot = dot;
                    best = neighbor;
                }
            }
            if (best == previousBest)
                break;
        }
        winner = best;
        return Unsafe.Add(ref vertices, best);
    }

    public static Vector3 ComputeLocalSupport(in HillclimbHull shape, Vector3 direction)
    {
        var support = Climb(shape.Topology, *shape.WarmStartSlot, direction, out var winner);
        *shape.WarmStartSlot = winner;
        return support;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in HillclimbHull shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        Matrix3x3.Transform(direction, orientationTranspose, out var localDirection);
        var localSupport = ComputeLocalSupport(shape, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }
}

/// <summary>
/// Cold variant: every query climbs from vertex 0 and never consults or updates the warm start slot.
/// Exists to isolate how much the warm start contributes versus the adjacency walk itself.
/// </summary>
public unsafe struct HullSupportScalarHillclimbCold : IScalarSupportFinder<HillclimbHull>
{
    public static Vector3 ComputeLocalSupport(in HillclimbHull shape, Vector3 direction)
    {
        return HullSupportScalarHillclimb.Climb(shape.Topology, 0, direction, out _);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in HillclimbHull shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        Matrix3x3.Transform(direction, orientationTranspose, out var localDirection);
        var localSupport = ComputeLocalSupport(shape, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }
}
