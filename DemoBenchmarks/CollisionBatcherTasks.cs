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


    public static void CreateDeformedPlane(int width, int height, Func<int, int, Vector3> deformer, Vector3 scaling, BufferPool pool, out Mesh mesh)
    {
        pool.Take<Vector3>(width * height, out var vertices);
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                vertices[width * j + i] = deformer(i, j);
            }
        }

        var quadWidth = width - 1;
        var quadHeight = height - 1;
        var triangleCount = quadWidth * quadHeight * 2;
        pool.Take<Triangle>(triangleCount, out var triangles);

        for (int i = 0; i < quadWidth; ++i)
        {
            for (int j = 0; j < quadHeight; ++j)
            {
                var triangleIndex = (j * quadWidth + i) * 2;
                ref var triangle0 = ref triangles[triangleIndex];
                ref var v00 = ref vertices[width * j + i];
                ref var v01 = ref vertices[width * j + i + 1];
                ref var v10 = ref vertices[width * (j + 1) + i];
                ref var v11 = ref vertices[width * (j + 1) + i + 1];
                triangle0.A = v00;
                triangle0.B = v01;
                triangle0.C = v10;
                ref var triangle1 = ref triangles[triangleIndex + 1];
                triangle1.A = v01;
                triangle1.B = v11;
                triangle1.C = v10;
            }
        }
        pool.Return(ref vertices);
        mesh = new Mesh(triangles, scaling, pool);
    }

    [GlobalSetup]
    public unsafe void Setup()
    {
        pool = new BufferPool();
        pool.Take(pairCount, out pairs);
        taskRegistry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
        shapes = new Shapes(pool, 1);

        var sphere = shapes.Add(new Sphere(1));
        var capsule = shapes.Add(new Capsule(0.5f, 1));
        var box = shapes.Add(new Box(2, 2, 2));
        var triangle = shapes.Add(new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1)));
        var cylinder = shapes.Add(new Cylinder(0.5f, 1));

        Random random = new(5);
        const int pointCount = 50;
        pool.Take<Vector3>(pointCount, out var points);
        for (int i = 0; i < pointCount; ++i)
        {
            points[i] = new Vector3(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
        }
        var hullShape = new ConvexHull(points, pool, out _);
        var hull = shapes.Add(hullShape);

        CompoundBuilder builder = new(pool, shapes, 64);
        BoundingBox compoundBounds = new() { Min = new Vector3(0, 0, 0), Max = new Vector3(4, 4, 4) };
        builder.AddForKinematic(sphere, CreateRandomPose(random, compoundBounds), 1);
        builder.AddForKinematic(capsule, CreateRandomPose(random, compoundBounds), 1);
        builder.AddForKinematic(box, CreateRandomPose(random, compoundBounds), 1);
        builder.AddForKinematic(triangle, CreateRandomPose(random, compoundBounds), 1);
        builder.AddForKinematic(cylinder, CreateRandomPose(random, compoundBounds), 1);
        builder.AddForKinematic(hull, CreateRandomPose(random, compoundBounds), 1);
        builder.BuildKinematicCompound(out var children, out _);
        var compound = shapes.Add(new Compound(children));
        builder.Reset();

        Span<TypedIndex> shapeIndices = stackalloc TypedIndex[9];
        shapeIndices[0] = sphere;
        shapeIndices[1] = capsule;
        shapeIndices[2] = box;
        shapeIndices[3] = triangle;
        shapeIndices[4] = cylinder;
        shapeIndices[5] = hull;

        BoundingBox bigCompoundBounds = new() { Min = new Vector3(0, 0, 0), Max = new Vector3(16, 16, 16) };
        for (int i = 0; i < 64; ++i)
        {
            builder.AddForKinematic(shapeIndices[random.Next(6)], CreateRandomPose(random, bigCompoundBounds), 1);
        }
        builder.BuildKinematicCompound(out var bigChildren, out _);
        var bigCompound = shapes.Add(new BigCompound(bigChildren, shapes, pool));

        CreateDeformedPlane(16, 16, (x, y) => { return new Vector3(x * 2 - 8, 3 * MathF.Sin(x) * MathF.Sin(y), y * 2 - 8); }, Vector3.One, pool, out var meshShape);
        var mesh = shapes.Add(meshShape);

        shapeIndices[6] = compound;
        shapeIndices[7] = bigCompound;
        shapeIndices[8] = mesh;

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
