#nullable enable
using BepuPhysics.Collidables;
using System.Numerics;

namespace AosBaselines;

/// <summary>
/// A relaxed-equality hull-hull tester candidate: judged by the tolerance comparator (RelaxedComparator) and by speed,
/// not by bitwise equality. Receives the harness-precomputed topology for both hulls (adjacency, edges, gauss arcs,
/// SIMD repack) alongside the engine hull structs; topology precompute is amortized per shape and excluded from
/// per-pair timing.
/// </summary>
public interface IRelaxedHullPairTester
{
    string Name { get; }
    void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold);
}

/// <summary>
/// Wraps the frozen bitwise scalar tester as a candidate. Exists to validate the comparator + harness plumbing end to end
/// (it must produce ~100% PASS against the wide reference) and as a convenient A/B anchor inside the candidate machinery.
/// </summary>
public sealed class FrozenScalarRelaxedTester : IRelaxedHullPairTester
{
    public string Name => "frozen";
    public void Test(HullTopology topologyA, HullTopology topologyB, ref ConvexHull a, ref ConvexHull b,
        float speculativeMargin, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
        out Convex4ManifoldScalar manifold)
    {
        ConvexHullPairScalarTester.Test(ref a, ref b, speculativeMargin, offsetB, orientationA, orientationB, out manifold);
    }
}

/// <summary>Registry of relaxed hull-hull candidates addressable by name from the relaxfuzzhull/benchhullscan modes.</summary>
public static class RelaxedCandidateRegistry
{
    public static readonly List<IRelaxedHullPairTester> Candidates = new()
    {
        new FrozenScalarRelaxedTester(),
        new HillclimbRelaxedTester(),
        new HillclimbColdRelaxedTester(),
        new SatRelaxedTester(),
        new SatRawRelaxedTester(),
        new WalkRelaxedTester(),
        new Walk2RelaxedTester(),
        new CsoWalkRelaxedTester(),
    };

    public static IRelaxedHullPairTester? Find(string name) =>
        Candidates.FirstOrDefault(candidate => string.Equals(candidate.Name, name, StringComparison.OrdinalIgnoreCase));

    public static string AvailableNames => string.Join(", ", Candidates.Select(candidate => candidate.Name));
}
