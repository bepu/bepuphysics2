using BenchmarkDotNet.Attributes;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using static DemoBenchmarks.BenchmarkHelper;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all multilane convex collision testers, but grouped together by approximate runtime for quicker top-level benchmarking.
/// To check performance of individual pairs, use the <see cref="ConvexCollisionTesterBenchmarks"/>.
/// </summary>
/// <remarks>
/// Note that all of these collision testers operate across <see cref="Vector{}.Count"/> lanes simultaneously where T is of type <see cref="float"/>.
/// <para>The number of bundles being executed does not change if <see cref="Vector{}.Count"/> changes; if larger bundles are allowed, then more lanes end up getting solved.</para>
/// </remarks>
public class GroupedCollisionTesterBenchmarks
{
    ConvexCollisionTesterBenchmarks benchmarks;

    [GlobalSetup]
    public unsafe void Setup()
    {
        benchmarks = new ConvexCollisionTesterBenchmarks();
        benchmarks.Setup();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        //All outstanding allocations poof when the pool is cleared.
        benchmarks.Cleanup();
    }

    [Benchmark]
    public Vector<float> CheapCollisionBenchmarks()
    {
        return
            benchmarks.SpherePairTester() +
            benchmarks.SphereCapsuleTester() +
            benchmarks.SphereBoxTester() +
            benchmarks.SphereTriangleTester() +
            benchmarks.SphereCylinderTester() +
            benchmarks.CapsulePairTester();
    }

    [Benchmark]
    public Vector<float> ModerateCostCollisionBenchmarks()
    {
        return
            benchmarks.CapsuleBoxTester() +
            benchmarks.CapsuleTriangleTester() +
            benchmarks.CapsuleCylinderTester() +
            benchmarks.BoxPairTester() +
            benchmarks.BoxTriangleTester() +
            benchmarks.TrianglePairTester();
    }

    [Benchmark]
    public Vector<float> ExpensiveCollisionBenchmarks()
    {
        return
            benchmarks.SphereConvexHullTester() +
            benchmarks.CapsuleConvexHullTester() +
            benchmarks.BoxConvexHullTester() +
            benchmarks.TriangleConvexHullTester() +
            benchmarks.CylinderConvexHullTester() +
            benchmarks.ConvexHullPairTester() +
            benchmarks.BoxCylinderTester() +
            benchmarks.TriangleCylinderTester() +
            benchmarks.CylinderPairTester();
    }
}
