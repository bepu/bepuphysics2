using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using static DemoBenchmarks.BenchmarkHelper;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of shape ray tests. Each benchmark covers a different shape type.
/// </summary>
public class ShapeRayBenchmarksDeep
{
    const int iterationCount = 100;
    BufferPool pool;

    struct Iteration
    {
        public RigidPose Pose;
        public RayData Ray;
    }

    Buffer<Iteration> iterations;
    Shapes shapes;


    [GlobalSetup]
    public unsafe void Setup()
    {
        pool = new BufferPool();
        pool.Take(iterationCount, out iterations);
        shapes = new Shapes(pool, 1);

        Random random = new(5);
        CreateShapes(random, pool, shapes);

        //Fill random values for pair tests.
        BoundingBox bounds = new() { Min = new Vector3(0, 0, 0), Max = new Vector3(2, 2, 2) };
        for (int i = 0; i < iterationCount; ++i)
        {
            iterations[i] = new()
            {
                Pose = CreateRandomPose(random, bounds),
                Ray = new RayData { Origin = CreateRandomPosition(random, bounds), Direction = CreateRandomDirection(random), Id = i }
            };
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        //All outstanding allocations poof when the pool is cleared.
        pool.Clear();
    }


    struct HitHandler : IShapeRayHitHandler
    {
        public Vector3 ResultSum;

        public bool AllowTest(int childIndex)
        {
            return true;
        }

        public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, int childIndex)
        {
            ResultSum += new Vector3(t) + normal;
        }
    }

    unsafe Vector3 Test<TShape>() where TShape : unmanaged, IShape
    {
        var hitHandler = new HitHandler();
        for (int i = 0; i < iterationCount; ++i)
        {
            ref var iteration = ref iterations[i];
            float maximumT = float.MaxValue;
            shapes[default(TShape).TypeId].RayTest(0, iteration.Pose, iteration.Ray, ref maximumT, ref hitHandler);
        }
        return hitHandler.ResultSum;
    }

    [Benchmark]
    public unsafe Vector3 RaySphere() => Test<Sphere>();
    [Benchmark]
    public unsafe Vector3 RayCapsule() => Test<Capsule>();
    [Benchmark]
    public unsafe Vector3 RayBox() => Test<Box>();
    [Benchmark]
    public unsafe Vector3 RayTriangle() => Test<Triangle>();
    [Benchmark]
    public unsafe Vector3 RayCylinder() => Test<Cylinder>();
    [Benchmark]
    public unsafe Vector3 RayConvexHull() => Test<ConvexHull>();
    [Benchmark]
    public unsafe Vector3 RayCompound() => Test<Compound>();
    [Benchmark]
    public unsafe Vector3 RayBigCompound() => Test<BigCompound>();
    [Benchmark]
    public unsafe Vector3 RayMesh() => Test<Mesh>();
}
