using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;

namespace DemoBenchmarks;

public static class BenchmarkHelper
{
    public static Vector3 CreateRandomPosition(Random random, BoundingBox positionBounds)
    {
        var span = positionBounds.Max - positionBounds.Min;
        return positionBounds.Min + span * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
    }
    public static Vector3 CreateRandomDirection(Random random)
    {
        //This is a biased sampling, but that doesn't matter.
        var axis = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * 2 - Vector3.One;
        var length = axis.Length();
        if (length > 1e-10f)
            axis /= length;
        else
            axis = new Vector3(0, 1, 0);
        return axis;
    }
    public static RigidPose CreateRandomPose(Random random, BoundingBox positionBounds)
    {
        RigidPose pose;
        pose.Position = CreateRandomPosition(random, positionBounds);
        pose.Orientation = QuaternionEx.CreateFromAxisAngle(CreateRandomDirection(random), 1203f * random.NextSingle());
        return pose;
    }

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

    public static void CreateShapes(Random random, BufferPool pool, Shapes shapes)
    {
        var sphere = shapes.Add(new Sphere(1));
        var capsule = shapes.Add(new Capsule(0.5f, 1));
        var box = shapes.Add(new Box(2, 2, 2));
        var triangle = shapes.Add(new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1)));
        var cylinder = shapes.Add(new Cylinder(0.5f, 1));

        const int hullPointCount = 50;
        pool.Take<Vector3>(hullPointCount, out var points);
        for (int i = 0; i < hullPointCount; ++i)
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

        BoundingBox bigCompoundBounds = new() { Min = new Vector3(0, 0, 0), Max = new Vector3(16, 16, 16) };
        for (int i = 0; i < 64; ++i)
        {
            builder.AddForKinematic(new TypedIndex(random.Next(6), 0), CreateRandomPose(random, bigCompoundBounds), 1);
        }
        builder.BuildKinematicCompound(out var bigChildren, out _);
        var bigCompound = shapes.Add(new BigCompound(bigChildren, shapes, pool));

        CreateDeformedPlane(16, 16, (x, y) => { return new Vector3(x * 2 - 8, 3 * MathF.Sin(x) * MathF.Sin(y), y * 2 - 8); }, Vector3.One, pool, out var meshShape);
        var mesh = shapes.Add(meshShape);
    }

}
