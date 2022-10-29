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

public class Sweeper
{
    const int iterationCount = 10;
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

    public Sweeper()
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
                    Angular = 0.1f * (new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One)
                },
                VelocityB = new BodyVelocity
                {
                    Linear = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One,
                    Angular = 0.1f * (new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One)
                },
                SpeculativeMargin = random.NextSingle(),
                A = GetRandomShapeTypeIndex(cumulative),
                B = GetRandomShapeTypeIndex(cumulative)
            };
        }
    }

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

    public unsafe Vector3 Test<TA, TB>() where TA : unmanaged, IShape where TB : unmanaged, IShape
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
}

/// <summary>
/// Evaluates performance of a representative subset of sweep tests.
/// </summary>
public class SweepBenchmarks
{
    Sweeper sweeper;
    [GlobalSetup]
    public unsafe void Setup()
    {
        sweeper = new Sweeper();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        sweeper.Cleanup();
    }


    //Some (commented) benchmarks are punted to SweepBenchmarksDeep:
    //-Convex-compound tests are pretty similar to each other. Box-compound/bigcompound/mesh is preserved, but the rest are commented. Likewise, bigcompound-bigcompound is the only compound-compound pair that's tested.
    //-Multiple sweeps use GJK for the distance tester. While the support functions used vary, those are relatively low value compared to the GJK outer loop. If we really need to, we can cover the support functions separately.
    //Most sphere tests aren't super interesting, so we exclude some of them.

    //[Benchmark]
    //public void SphereSphere() => sweeper.Test<Sphere, Sphere>();
    [Benchmark]
    public void SphereCapsule() => sweeper.Test<Sphere, Capsule>();
    //[Benchmark]
    //public void SphereBox() => sweeper.Test<Sphere, Box>();
    [Benchmark]
    public void SphereTriangle() => sweeper.Test<Sphere, Triangle>();
    //[Benchmark]
    //public void SphereCylinder() => sweeper.Test<Sphere, Cylinder>();
    //[Benchmark]
    //public void SphereConvexHull() => Test<Sphere, ConvexHull>(); //GJK
    //[Benchmark]
    //public void SphereCompound() => Test<Sphere, Compound>();
    //[Benchmark]
    //public void SphereBigCompound() => Test<Sphere, BigCompound>();
    //[Benchmark]
    //public void SphereMesh() => Test<Sphere, Mesh>();
    [Benchmark]
    public void CapsuleCapsule() => sweeper.Test<Capsule, Capsule>();
    [Benchmark]
    public void CapsuleBox() => sweeper.Test<Capsule, Box>();
    //[Benchmark]
    //public void CapsuleTriangle() => Test<Capsule, Triangle>(); //GJK
    [Benchmark]
    public void CapsuleCylinder() => sweeper.Test<Capsule, Cylinder>(); //GJK
    //[Benchmark]
    //public void CapsuleConvexHull() => Test<Capsule, ConvexHull>(); //GJK
    //[Benchmark]
    //public void CapsuleCompound() => Test<Capsule, Compound>();
    //[Benchmark]
    //public void CapsuleBigCompound() => Test<Capsule, BigCompound>();
    //[Benchmark]
    //public void CapsuleMesh() => Test<Capsule, Mesh>();
    //[Benchmark]
    //public void BoxBox() => Test<Box, Box>(); //GJK
    //[Benchmark]
    //public void BoxTriangle() => Test<Box, Triangle>(); //GJK
    //[Benchmark]
    //public void BoxCylinder() => Test<Box, Cylinder>(); //GJK
    //[Benchmark]
    //public void BoxConvexHull() => Test<Box, ConvexHull>(); //GJK
    [Benchmark]
    public void BoxCompound() => sweeper.Test<Box, Compound>();
    //[Benchmark]
    //public void BoxBigCompound() => Test<Box, BigCompound>(); //Well covered by bigcompound-bigcompound.
    [Benchmark]
    public void BoxMesh() => sweeper.Test<Box, Mesh>();
    //[Benchmark]
    //public void TriangleTriangle() => Test<Triangle, Triangle>(); //GJK
    //[Benchmark]
    //public void TriangleCylinder() => Test<Triangle, Cylinder>(); //GJK
    //[Benchmark]
    //public void TriangleConvexHull() => Test<Triangle, ConvexHull>(); //GJK
    //[Benchmark]
    //public void TriangleCompound() => Test<Triangle, Compound>();
    //[Benchmark]
    //public void TriangleBigCompound() => Test<Triangle, BigCompound>();
    //[Benchmark]
    //public void TriangleMesh() => Test<Triangle, Mesh>();
    //[Benchmark]
    //public void CylinderCylinder() => Test<Cylinder, Cylinder>(); //GJK
    //[Benchmark]
    //public void CylinderConvexHull() => Test<Cylinder, ConvexHull>(); //GJK
    //[Benchmark]
    //public void CylinderCompound() => Test<Cylinder, Compound>();
    //[Benchmark]
    //public void CylinderBigCompound() => Test<Cylinder, BigCompound>();
    //[Benchmark]
    //public void CylinderMesh() => Test<Cylinder, Mesh>();
    //[Benchmark]
    //public void ConvexHullConvexHull() => Test<ConvexHull, ConvexHull>(); //GJK
    //[Benchmark]
    //public void ConvexHullCompound() => Test<ConvexHull, Compound>();
    //[Benchmark]
    //public void ConvexHullBigCompound() => Test<ConvexHull, BigCompound>();
    //[Benchmark]
    //public void ConvexHullMesh() => Test<ConvexHull, Mesh>();
    //[Benchmark]
    //public void CompoundCompound() => Test<Compound, Compound>();
    //[Benchmark]
    //public void CompoundBigCompound() => Test<Compound, BigCompound>();
    //[Benchmark]
    //public void CompoundMesh() => Test<Compound, Mesh>();
    [Benchmark]
    public void BigCompoundBigCompound() => sweeper.Test<BigCompound, BigCompound>();
    //[Benchmark]
    //public void BigCompoundMesh() => Test<BigCompound, Mesh>();
    //No mesh-mesh!
}
