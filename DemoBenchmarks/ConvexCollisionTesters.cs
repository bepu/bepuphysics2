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
/// Evaluates performance of all multilane convex collision testers.
/// </summary>
/// <remarks>
/// Note that all of these collision testers operate across <see cref="Vector{}.Count"/> lanes simultaneously where T is of type <see cref="float"/>.
/// <para>The number of bundles being executed does not change if <see cref="Vector{}.Count"/> changes; if larger bundles are allowed, then more lanes end up getting solved.</para>
/// </remarks>
public class ConvexCollisionTesters
{
    const int iterationCount = 1000;
    BufferPool pool;
    Buffer<Vector3Wide> offsetsB;
    Buffer<Vector<float>> speculativeMargins;
    Buffer<QuaternionWide> orientationsA;
    Buffer<QuaternionWide> orientationsB;

    [GlobalSetup]
    public unsafe void Setup()
    {
        pool = new BufferPool();
        pool.Take(iterationCount, out offsetsB);
        pool.Take(iterationCount, out speculativeMargins);
        pool.Take(iterationCount, out orientationsA);
        pool.Take(iterationCount, out orientationsB);

        //Fill random values for test instances.
        BoundingBox bounds = new() { Min = new Vector3(0, 0, 0), Max = new Vector3(2, 2, 2) };
        Random random = new(5);
        Span<float> bundleMargins = stackalloc float[Vector<float>.Count];
        for (int i = 0; i < iterationCount; ++i)
        {
            Vector3Wide offsetB = default;
            QuaternionWide orientationA = default;
            QuaternionWide orientationB = default;
            for (int j = 0; j < Vector<float>.Count; ++j)
            {
                var poseA = CreateRandomPose(random, bounds);
                var poseB = CreateRandomPose(random, bounds);
                Vector3Wide.WriteSlot(poseB.Position - poseA.Position, j, ref offsetB);
                QuaternionWide.WriteSlot(poseA.Orientation, j, ref orientationA);
                QuaternionWide.WriteSlot(poseB.Orientation, j, ref orientationB);
                bundleMargins[j] = random.NextSingle() * 2;

            }
            offsetsB[i] = offsetB;
            orientationsA[i] = orientationA;
            orientationsB[i] = orientationB;
            speculativeMargins[i] = new Vector<float>(bundleMargins);
        }

        //Convex hulls are a little heavier to set up than the other shapes, so precreate one.
        const int pointCount = 50;
        pool.Take<Vector3>(pointCount, out var points);
        for (int i = 0; i < pointCount; ++i)
        {
            points[i] = new Vector3(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
        }
        var narrowHull = new ConvexHull(points, pool, out _);
        pool.Take<ConvexHull>(Vector<float>.Count, out var hullBundle);
        var hull = default(ConvexHullWide);
        hull.Initialize(hullBundle.As<byte>());
        hull.Broadcast(narrowHull);
        Hull = hull;
    }

    public void Cleanup()
    {
        //All outstanding allocations poof when the pool is cleared.
        pool.Clear();
    }

    SphereWide Sphere => new() { Radius = new Vector<float>(1f) };
    CapsuleWide Capsule => new() { Radius = new Vector<float>(0.5f), HalfLength = new Vector<float>(1f) };
    BoxWide Box => new() { HalfWidth = new Vector<float>(1f), HalfHeight = new Vector<float>(1f), HalfLength = new Vector<float>(1f) };
    TriangleWide Triangle => new() { A = Vector3Wide.Broadcast(new Vector3(-1f / 3f, 0, -1f / 3f)), B = Vector3Wide.Broadcast(new Vector3(2 / 3f, 0, -1f / 3f)), C = Vector3Wide.Broadcast(new Vector3(-1f / 3f, 0, 2 / 3f)), };
    CylinderWide Cylinder => new() { Radius = new Vector<float>(1f), HalfLength = new Vector<float>(1f) };
    ConvexHullWide Hull { get; set; }


    Vector<float> TestOrientationless1Contact<TTester, TShapeA, TShapeB>(TShapeA a, TShapeB b) where TTester : unmanaged, IPairTester<TShapeA, TShapeB, Convex1ContactManifoldWide>
    {
        var tester = default(TTester);
        Vector<float> testSum = Vector<float>.Zero;
        for (int i = 0; i < iterationCount; ++i)
        {
            tester.Test(ref a, ref b, ref speculativeMargins[i], ref offsetsB[i], Vector<float>.Count, out var manifold);
            testSum += manifold.Depth + manifold.Normal.X + manifold.Normal.Y + manifold.Normal.Z + manifold.OffsetA.X + manifold.OffsetA.Y + manifold.OffsetA.Z;
        }
        return testSum;
    }
    Vector<float> TestOrientationB1Contact<TTester, TShapeA, TShapeB>(TShapeA a, TShapeB b) where TTester : unmanaged, IPairTester<TShapeA, TShapeB, Convex1ContactManifoldWide>
    {
        var tester = default(TTester);
        Vector<float> testSum = Vector<float>.Zero;
        for (int i = 0; i < iterationCount; ++i)
        {
            tester.Test(ref a, ref b, ref speculativeMargins[i], ref offsetsB[i], ref orientationsB[i], Vector<float>.Count, out var manifold);
            testSum += manifold.Depth + manifold.Normal.X + manifold.Normal.Y + manifold.Normal.Z + manifold.OffsetA.X + manifold.OffsetA.Y + manifold.OffsetA.Z;
        }
        return testSum;
    }
    Vector<float> Test2Contact<TTester, TShapeA, TShapeB>(TShapeA a, TShapeB b) where TTester : unmanaged, IPairTester<TShapeA, TShapeB, Convex2ContactManifoldWide>
    {
        var tester = default(TTester);
        Vector<float> testSum = Vector<float>.Zero;
        for (int i = 0; i < iterationCount; ++i)
        {
            tester.Test(ref a, ref b, ref speculativeMargins[i], ref offsetsB[i], ref orientationsA[i], ref orientationsB[i], Vector<float>.Count, out var manifold);
            testSum += manifold.Normal.X + manifold.Normal.Y + manifold.Normal.Z +
                manifold.OffsetA0.X + manifold.OffsetA0.Y + manifold.OffsetA0.Z + manifold.Depth0 +
                manifold.OffsetA1.X + manifold.OffsetA1.Y + manifold.OffsetA1.Z + manifold.Depth1;
        }
        return testSum;
    }
    Vector<float> Test4Contact<TTester, TShapeA, TShapeB>(TShapeA a, TShapeB b) where TTester : unmanaged, IPairTester<TShapeA, TShapeB, Convex4ContactManifoldWide>
    {
        var tester = default(TTester);
        Vector<float> testSum = Vector<float>.Zero;
        for (int i = 0; i < iterationCount; ++i)
        {
            tester.Test(ref a, ref b, ref speculativeMargins[i], ref offsetsB[i], ref orientationsA[i], ref orientationsB[i], Vector<float>.Count, out var manifold);
            testSum += manifold.Normal.X + manifold.Normal.Y + manifold.Normal.Z +
                manifold.OffsetA0.X + manifold.OffsetA0.Y + manifold.OffsetA0.Z + manifold.Depth0 +
                manifold.OffsetA1.X + manifold.OffsetA1.Y + manifold.OffsetA1.Z + manifold.Depth1 +
                manifold.OffsetA2.X + manifold.OffsetA2.Y + manifold.OffsetA2.Z + manifold.Depth2 +
                manifold.OffsetA3.X + manifold.OffsetA3.Y + manifold.OffsetA3.Z + manifold.Depth3;
        }
        return testSum;
    }

    [Benchmark]
    public Vector<float> SpherePairTester()
    {
        return TestOrientationless1Contact<SpherePairTester, SphereWide, SphereWide>(Sphere, Sphere);
    }
    [Benchmark]
    public Vector<float> SphereCapsuleTester()
    {
        return TestOrientationB1Contact<SphereCapsuleTester, SphereWide, CapsuleWide>(Sphere, Capsule);
    }
    [Benchmark]
    public Vector<float> SphereBoxTester()
    {
        return TestOrientationB1Contact<SphereBoxTester, SphereWide, BoxWide>(Sphere, Box);
    }
    [Benchmark]
    public Vector<float> SphereTriangleTester()
    {
        return TestOrientationB1Contact<SphereTriangleTester, SphereWide, TriangleWide>(Sphere, Triangle);
    }
    [Benchmark]
    public Vector<float> SphereCylinderTester()
    {
        return TestOrientationB1Contact<SphereCylinderTester, SphereWide, CylinderWide>(Sphere, Cylinder);
    }
    [Benchmark]
    public Vector<float> SphereConvexHullTester()
    {
        return TestOrientationB1Contact<SphereConvexHullTester, SphereWide, ConvexHullWide>(Sphere, Hull);
    }
    [Benchmark]
    public Vector<float> CapsulePairTester()
    {
        return Test2Contact<CapsulePairTester, CapsuleWide, CapsuleWide>(Capsule, Capsule);
    }
    [Benchmark]
    public Vector<float> CapsuleBoxTester()
    {
        return Test2Contact<CapsuleBoxTester, CapsuleWide, BoxWide>(Capsule, Box);
    }
    [Benchmark]
    public Vector<float> CapsuleTriangleTester()
    {
        return Test2Contact<CapsuleTriangleTester, CapsuleWide, TriangleWide>(Capsule, Triangle);
    }
    [Benchmark]
    public Vector<float> CapsuleCylinderTester()
    {
        return Test2Contact<CapsuleCylinderTester, CapsuleWide, CylinderWide>(Capsule, Cylinder);
    }
    [Benchmark]
    public Vector<float> CapsuleConvexHullTester()
    {
        return Test2Contact<CapsuleConvexHullTester, CapsuleWide, ConvexHullWide>(Capsule, Hull);
    }
    [Benchmark]
    public Vector<float> BoxPairTester()
    {
        return Test4Contact<BoxPairTester, BoxWide, BoxWide>(Box, Box);
    }
    [Benchmark]
    public Vector<float> BoxTriangleTester()
    {
        return Test4Contact<BoxTriangleTester, BoxWide, TriangleWide>(Box, Triangle);
    }
    [Benchmark]
    public Vector<float> BoxCylinderTester()
    {
        return Test4Contact<BoxCylinderTester, BoxWide, CylinderWide>(Box, Cylinder);
    }
    [Benchmark]
    public Vector<float> BoxConvexHullTester()
    {
        return Test4Contact<BoxConvexHullTester, BoxWide, ConvexHullWide>(Box, Hull);
    }
    [Benchmark]
    public Vector<float> TrianglePairTester()
    {
        return Test4Contact<TrianglePairTester, TriangleWide, TriangleWide>(Triangle, Triangle);
    }
    [Benchmark]
    public Vector<float> TriangleCylinderTester()
    {
        return Test4Contact<TriangleCylinderTester, TriangleWide, CylinderWide>(Triangle, Cylinder);
    }
    [Benchmark]
    public Vector<float> TriangleConvexHullTester()
    {
        return Test4Contact<TriangleConvexHullTester, TriangleWide, ConvexHullWide>(Triangle, Hull);
    }
    [Benchmark]
    public Vector<float> CylinderPairTester()
    {
        return Test4Contact<CylinderPairTester, CylinderWide, CylinderWide>(Cylinder, Cylinder);
    }
    [Benchmark]
    public Vector<float> CylinderConvexHullTester()
    {
        return Test4Contact<CylinderConvexHullTester, CylinderWide, ConvexHullWide>(Cylinder, Hull);
    }
    [Benchmark]
    public Vector<float> ConvexHullPairTester()
    {
        return Test4Contact<ConvexHullPairTester, ConvexHullWide, ConvexHullWide>(Hull, Hull);
    }
}
