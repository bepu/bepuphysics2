using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static DemoBenchmarks.BenchmarkHelper;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all sweep tests.
/// </summary>
public class Sweeps
{
    const int iterationCount = 100;
    BufferPool pool;
    struct Pair
    {
        public Vector3 OffsetB;
        public Quaternion OrientationA;
        public Quaternion OrientationB;
        public BodyVelocity VelocityA;
        public BodyVelocity VelocityB;
        public float SpeculativeMargin;
        public TypedIndex A;
        public TypedIndex B;
    }
    Buffer<Pair> pairs;
    SweepTaskRegistry taskRegistry;
    Shapes shapes;


    [GlobalSetup]
    public unsafe void Setup()
    {
        pool = new BufferPool();
        pool.Take(iterationCount, out pairs);
        taskRegistry = DefaultTypes.CreateDefaultSweepTaskRegistry();
        shapes = new Shapes(pool, 1);
        Random random = new(5);
        CreateShapes(random, pool, shapes);

        Span<float> shapeRelativeProbabilities = stackalloc float[9];
        shapeRelativeProbabilities[0] = 1;
        shapeRelativeProbabilities[1] = 1;
        shapeRelativeProbabilities[2] = 1;
        shapeRelativeProbabilities[3] = 1;
        shapeRelativeProbabilities[4] = 1;
        shapeRelativeProbabilities[5] = 1;

        shapeRelativeProbabilities[6] = 0.2f;
        shapeRelativeProbabilities[7] = 0.2f;
        shapeRelativeProbabilities[7] = 0.2f;

        var sum = 0f;
        Span<float> cumulative = stackalloc float[9];
        for (int i = 0; i < shapeRelativeProbabilities.Length; ++i)
        {
            cumulative[i] = sum;
            sum += shapeRelativeProbabilities[i];
        }
        var inverseSum = 1f / sum;
        for (int i = 0; i < shapeRelativeProbabilities.Length; ++i)
            cumulative[i] *= inverseSum;

        TypedIndex GetRandomShapeTypeIndex(Span<float> cumulative)
        {
            var r = random.NextSingle();
            for (int i = 0; i < cumulative.Length; ++i)
            {
                if (r < cumulative[i])
                    return new TypedIndex(i, 0); //there's only one shape per type, sooo.
            }
            Debug.Fail("hey whatnow");
            return default;
        }

        //Fill random values for pair tests.
        BoundingBox bounds = new() { Min = new Vector3(0, 0, 0), Max = new Vector3(2, 2, 2) };
        for (int i = 0; i < iterationCount; ++i)
        {
            var poseA = CreateRandomPose(random, bounds);
            var poseB = CreateRandomPose(random, bounds);
            ref var pair = ref pairs[i];
            pair = new Pair
            {
                OffsetB = poseB.Position - poseA.Position,
                OrientationA = poseA.Orientation,
                OrientationB = poseB.Orientation,
                VelocityA = new BodyVelocity
                {
                    Linear = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One,
                    Angular = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One
                },
                VelocityB = new BodyVelocity
                {
                    Linear = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One,
                    Angular = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One
                },
                SpeculativeMargin = random.NextSingle(),
                A = GetRandomShapeTypeIndex(cumulative),
                B = GetRandomShapeTypeIndex(cumulative)
            };
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        //All outstanding allocations poof when the pool is cleared.
        pool.Clear();
    }

    struct Filter : ISweepFilter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowTest(int childA, int childB)
        {
            return true;
        }
    }

    unsafe Vector3 Test<TA, TB>() where TA : unmanaged, IShape where TB : unmanaged, IShape
    {
        var task = taskRegistry.GetTask<TA, TB>();
        var aType = default(TA).TypeId;
        var bType = default(TB).TypeId;
        shapes[aType].GetShapeData(0, out var aData, out _);
        shapes[bType].GetShapeData(0, out var bData, out _);
        var filter = default(Filter);
        Vector3 resultSum = Vector3.Zero;
        for (int i = 0; i < iterationCount; ++i)
        {
            ref var pair = ref pairs[i];
            var hit = task.Sweep(
                aData, aType, pair.OrientationA, pair.VelocityA,
                bData, bType, pair.OffsetB, pair.OrientationB, pair.VelocityB,
                0.1f, 1e-3f, 1e-3f, 15, ref filter, shapes, taskRegistry, pool, out var t0, out var t1, out var hitLocation, out var hitNormal);
            if (hit)
                resultSum += new Vector3(t0) + new Vector3(t1) + hitLocation + hitNormal;
        }
        return resultSum;
    }

    [Benchmark]
    public void SphereSphere() => Test<Sphere, Sphere>();
    [Benchmark]
    public void SphereCapsule() => Test<Sphere, Capsule>();
    [Benchmark]
    public void SphereBox() => Test<Sphere, Box>();
    [Benchmark]
    public void SphereTriangle() => Test<Sphere, Triangle>();
    [Benchmark]
    public void SphereCylinder() => Test<Sphere, Cylinder>();
    [Benchmark]
    public void SphereConvexHull() => Test<Sphere, ConvexHull>();
    [Benchmark]
    public void SphereCompound() => Test<Sphere, Compound>();
    [Benchmark]
    public void SphereBigCompound() => Test<Sphere, BigCompound>();
    [Benchmark]
    public void SphereMesh() => Test<Sphere, Mesh>();
    [Benchmark]
    public void CapsuleCapsule() => Test<Capsule, Capsule>();
    [Benchmark]
    public void CapsuleBox() => Test<Capsule, Box>();
    [Benchmark]
    public void CapsuleTriangle() => Test<Capsule, Triangle>();
    [Benchmark]
    public void CapsuleCylinder() => Test<Capsule, Cylinder>();
    [Benchmark]
    public void CapsuleConvexHull() => Test<Capsule, ConvexHull>();
    [Benchmark]
    public void CapsuleCompound() => Test<Capsule, Compound>();
    [Benchmark]
    public void CapsuleBigCompound() => Test<Capsule, BigCompound>();
    [Benchmark]
    public void CapsuleMesh() => Test<Capsule, Mesh>();
    [Benchmark]
    public void BoxBox() => Test<Box, Box>();
    [Benchmark]
    public void BoxTriangle() => Test<Box, Triangle>();
    [Benchmark]
    public void BoxCylinder() => Test<Box, Cylinder>();
    [Benchmark]
    public void BoxConvexHull() => Test<Box, ConvexHull>();
    [Benchmark]
    public void BoxCompound() => Test<Box, Compound>();
    [Benchmark]
    public void BoxBigCompound() => Test<Box, BigCompound>();
    [Benchmark]
    public void BoxMesh() => Test<Box, Mesh>();
    [Benchmark]
    public void TriangleTriangle() => Test<Triangle, Triangle>();
    [Benchmark]
    public void TriangleCylinder() => Test<Triangle, Cylinder>();
    [Benchmark]
    public void TriangleConvexHull() => Test<Triangle, ConvexHull>();
    [Benchmark]
    public void TriangleCompound() => Test<Triangle, Compound>();
    [Benchmark]
    public void TriangleBigCompound() => Test<Triangle, BigCompound>();
    [Benchmark]
    public void TriangleMesh() => Test<Triangle, Mesh>();
    [Benchmark]
    public void CylinderCylinder() => Test<Cylinder, Cylinder>();
    [Benchmark]
    public void CylinderConvexHull() => Test<Cylinder, ConvexHull>();
    [Benchmark]
    public void CylinderCompound() => Test<Cylinder, Compound>();
    [Benchmark]
    public void CylinderBigCompound() => Test<Cylinder, BigCompound>();
    [Benchmark]
    public void CylinderMesh() => Test<Cylinder, Mesh>();
    [Benchmark]
    public void ConvexHullConvexHull() => Test<ConvexHull, ConvexHull>();
    [Benchmark]
    public void ConvexHullCompound() => Test<ConvexHull, Compound>();
    [Benchmark]
    public void ConvexHullBigCompound() => Test<ConvexHull, BigCompound>();
    [Benchmark]
    public void ConvexHullMesh() => Test<ConvexHull, Mesh>();
    [Benchmark]
    public void CompoundCompound() => Test<Compound, Compound>();
    [Benchmark]
    public void CompoundBigCompound() => Test<Compound, BigCompound>();
    [Benchmark]
    public void CompoundMesh() => Test<Compound, Mesh>();
    [Benchmark]
    public void BigCompoundBigCompound() => Test<BigCompound, BigCompound>();
    [Benchmark]
    public void BigCompoundMesh() => Test<BigCompound, Mesh>();
    //No mesh-mesh!
}
