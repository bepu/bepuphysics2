using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using static DemoBenchmarks.BenchmarkHelper;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all collision tasks together in a collision batcher.
/// </summary>
public class CollisionBatcherTasks
{
    const int pairCount = 100000;
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
    CollisionTaskRegistry taskRegistry;
    Shapes shapes;

    [GlobalSetup]
    public unsafe void Setup()
    {
        pool = new BufferPool();
        pool.Take(pairCount, out pairs);
        taskRegistry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
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
        for (int i = 0; i < pairCount; ++i)
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


    struct Callbacks : ICollisionCallbacks
    {
        public Vector3 ResultSum;
        public bool AllowCollisionTesting(int pairId, int childA, int childB)
        {
            return true;
        }

        public void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold)
        {
        }

        public void OnPairCompleted<TManifold>(int pairId, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            var count = manifold.Count;
            for (int i = 0; i < count; ++i)
            {
                ResultSum += manifold.GetNormal(ref manifold, i) + manifold.GetOffset(ref manifold, i) + new Vector3(manifold.GetDepth(ref manifold, i));
            }
        }
    }

    [Benchmark]
    public unsafe Vector3 Benchmark()
    {
        CollisionBatcher<Callbacks> batcher = new(pool, shapes, taskRegistry, 1 / 60f, new Callbacks());

        for (int i = 0; i < pairCount; ++i)
        {
            ref var pair = ref pairs[i];
            shapes[pair.A.Type].GetShapeData(pair.A.Index, out var aData, out _);
            shapes[pair.B.Type].GetShapeData(pair.B.Index, out var bData, out _);
            batcher.AddDirectly(pair.A.Type, pair.B.Type, aData, bData, pair.OffsetB, pair.OrientationA, pair.OrientationB, pair.VelocityA, pair.VelocityB, pair.SpeculativeMargin, float.MaxValue, new PairContinuation(i));
        }
        batcher.Flush();
        return batcher.Callbacks.ResultSum;
    }
}
