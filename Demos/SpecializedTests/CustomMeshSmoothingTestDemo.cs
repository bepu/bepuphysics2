using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuPhysics.Constraints;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;

namespace Demos.SpecializedTests;

/// <summary>
/// Pure forwarding wrapper around <see cref="Mesh"/>. Has its own TypeId so the narrow phase treats it as a distinct shape,
/// which lets us verify that <see cref="MeshReduction"/>'s boundary smoothing works for any <see cref="IHomogeneousCompoundShape{Triangle, TriangleWide}">, not just the built-in Mesh type.
/// </summary>
public struct WrappedMesh : IHomogeneousCompoundShape<Triangle, TriangleWide>
{
    public Mesh Inner;

    public WrappedMesh(Mesh inner)
    {
        Inner = inner;
    }

    public const int Id = 13;
    public static int TypeId => Id;

    public readonly int ChildCount => Inner.ChildCount;

    public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
    {
        return new HomogeneousCompoundShapeBatch<WrappedMesh, Triangle, TriangleWide>(pool, initialCapacity);
    }

    public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
    {
        Inner.ComputeBounds(orientation, out min, out max);
    }

    public readonly void GetLocalChild(int childIndex, out Triangle target)
    {
        Inner.GetLocalChild(childIndex, out target);
    }

    public readonly void GetPosedLocalChild(int childIndex, out Triangle target, out RigidPose childPose)
    {
        Inner.GetPosedLocalChild(childIndex, out target, out childPose);
    }

    public readonly void GetLocalChild(int childIndex, ref TriangleWide target)
    {
        Inner.GetLocalChild(childIndex, ref target);
    }

    public readonly void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, BufferPool pool, ref TRayHitHandler hitHandler)
        where TRayHitHandler : struct, IShapeRayHitHandler
    {
        Inner.RayTest(pose, ray, ref maximumT, pool, ref hitHandler);
    }

    public readonly void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, BufferPool pool, ref TRayHitHandler hitHandler)
        where TRayHitHandler : struct, IShapeRayHitHandler
    {
        Inner.RayTest(pose, ref rays, pool, ref hitHandler);
    }

    public readonly unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
        where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
        where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
    {
        //Can't forward directly: the Mesh implementation reinterprets each pair.Container as Mesh*, but here the containers point to WrappedMesh instances.
        //Replicate the loop and forward each pair's AABB to the inner mesh's single-AABB overload instead.
        ShapeTreeOverlapEnumerator<TSubpairOverlaps> enumerator;
        enumerator.Pool = pool;
        for (int i = 0; i < pairs.Length; ++i)
        {
            ref var pair = ref pairs[i];
            ref var wrapped = ref Unsafe.AsRef<WrappedMesh>(pair.Container);
            enumerator.Overlaps = Unsafe.AsPointer(ref overlaps.GetOverlapsForPair(i));
            wrapped.Inner.FindLocalOverlaps(pair.Min, pair.Max, pool, shapes, ref enumerator);
        }
    }

    public readonly unsafe void FindLocalOverlaps<TOverlaps>(Vector3 min, Vector3 max, Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps)
        where TOverlaps : ICollisionTaskSubpairOverlaps
    {
        Inner.FindLocalOverlaps<TOverlaps>(min, max, sweep, maximumT, pool, shapes, overlaps);
    }

    public readonly void FindLocalOverlaps<TEnumerator>(Vector3 min, Vector3 max, BufferPool pool, Shapes shapes, ref TEnumerator enumerator)
        where TEnumerator : IBreakableForEach<int>
    {
        Inner.FindLocalOverlaps(min, max, pool, shapes, ref enumerator);
    }

    public void Dispose(BufferPool pool)
    {
        Inner.Dispose(pool);
    }
}

/// <summary>
/// Drops convex shapes onto two WrappedMesh heightfields side by side. The fine mesh (many small triangles) forces MeshReduction into its
/// dictionary-based high-subpair-count path; the coarse mesh (few large triangles) keeps subpair counts under the brute-force threshold.
/// Between them the demo exercises every branch of <see cref="MeshReduction.ReduceManifolds"/> for a non-<see cref="Mesh"/>
/// IHomogeneousCompoundShape so boundary smoothing can be validated on the type-erased path.
/// </summary>
public class CustomMeshSmoothingTestDemo : Demo
{
    (StaticHandle Handle, Mesh InnerMesh)[] wrappedMeshes;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 20, 60);
        camera.Yaw = 0;
        camera.Pitch = -0.3f;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

        //Register collision tasks for every convex shape we're going to drop against the WrappedMesh.
        //These are the same tasks DefaultTypes registers for Mesh, just closed over WrappedMesh so MeshReductionThunks<WrappedMesh> is used instead of MeshReductionThunks<Mesh>.
        var collisionTasks = Simulation.NarrowPhase.CollisionTaskRegistry;
        collisionTasks.Register(new ConvexCompoundCollisionTask<Sphere, WrappedMesh, ConvexCompoundOverlapFinder<Sphere, SphereWide, WrappedMesh>, ConvexMeshContinuations<WrappedMesh>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Capsule, WrappedMesh, ConvexCompoundOverlapFinder<Capsule, CapsuleWide, WrappedMesh>, ConvexMeshContinuations<WrappedMesh>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Box, WrappedMesh, ConvexCompoundOverlapFinder<Box, BoxWide, WrappedMesh>, ConvexMeshContinuations<WrappedMesh>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Triangle, WrappedMesh, ConvexCompoundOverlapFinder<Triangle, TriangleWide, WrappedMesh>, ConvexMeshContinuations<WrappedMesh>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<Cylinder, WrappedMesh, ConvexCompoundOverlapFinder<Cylinder, CylinderWide, WrappedMesh>, ConvexMeshContinuations<WrappedMesh>, MeshReduction>());
        collisionTasks.Register(new ConvexCompoundCollisionTask<ConvexHull, WrappedMesh, ConvexCompoundOverlapFinder<ConvexHull, ConvexHullWide, WrappedMesh>, ConvexMeshContinuations<WrappedMesh>, MeshReduction>());

        //Compound-vs-WrappedMesh uses a separate continuation type (CompoundMeshReduction), but it plugs into MeshReductionThunks<WrappedMesh> the same way.
        collisionTasks.Register(new CompoundPairCollisionTask<Compound, WrappedMesh, CompoundPairOverlapFinder<Compound, WrappedMesh>, CompoundMeshContinuations<Compound, WrappedMesh>, CompoundMeshReduction>());

        //Sweep tasks matching the convex set, so swept queries keep working too.
        var sweepTasks = Simulation.NarrowPhase.SweepTaskRegistry;
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Sphere, SphereWide, WrappedMesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Sphere, WrappedMesh>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Capsule, CapsuleWide, WrappedMesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Capsule, WrappedMesh>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Box, BoxWide, WrappedMesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Box, WrappedMesh>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Triangle, TriangleWide, WrappedMesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Triangle, WrappedMesh>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<Cylinder, CylinderWide, WrappedMesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<Cylinder, WrappedMesh>>());
        sweepTasks.Register(new ConvexHomogeneousCompoundSweepTask<ConvexHull, ConvexHullWide, WrappedMesh, Triangle, TriangleWide, ConvexCompoundSweepOverlapFinder<ConvexHull, WrappedMesh>>());
        sweepTasks.Register(new CompoundHomogeneousCompoundSweepTask<Compound, WrappedMesh, Triangle, TriangleWide, CompoundPairSweepOverlapFinder<Compound, WrappedMesh>>());

        //Two meshes that share the same world-space terrain shape and footprint, but with wildly different tessellation density.
        //The fine mesh pushes subpair counts into the dictionary path; the coarse mesh keeps them in the brute-force path.
        wrappedMeshes = new (StaticHandle, Mesh)[2];
        var fineOrigin = Vector3.Zero;
        var coarseOrigin = new Vector3(0, 0, 160);
        AddWrappedTerrain(fineOrigin, planeWidth: 513, xzScale: 0.3f, out wrappedMeshes[0].Handle, out wrappedMeshes[0].InnerMesh);
        AddShapesAt(fineOrigin);
        AddWrappedTerrain(coarseOrigin, planeWidth: 33, xzScale: 4.8f, out wrappedMeshes[1].Handle, out wrappedMeshes[1].InnerMesh);
        AddShapesAt(coarseOrigin);
    }

    void AddWrappedTerrain(Vector3 staticPosition, int planeWidth, float xzScale, out StaticHandle handle, out Mesh innerMesh)
    {
        //The noise is evaluated in mesh-local world space so both meshes end up with the same apparent terrain — only triangle density differs.
        Vector2 terrainOffset = new Vector2(1 - planeWidth, 1 - planeWidth) * 0.5f;
        var scale = new Vector3(xzScale, 0.1f, xzScale);
        innerMesh = DemoMeshHelper.CreateDeformedPlane(planeWidth, planeWidth,
            (int vX, int vY) =>
            {
                //vX and vY are vertex indices; multiply by scale after adding the centering offset to get a local-space position in world units.
                var localX = (vX + terrainOffset.X) * xzScale;
                var localZ = (vY + terrainOffset.Y) * xzScale;
                var octave0 = (MathF.Sin((localX + 5f) * 0.133f) + MathF.Sin((localZ + 11) * 0.133f)) * 0.9f;
                var octave1 = (MathF.Sin((localX + 17) * 0.367f) + MathF.Sin((localZ + 19) * 0.367f)) * 0.35f;
                var octave2 = (MathF.Sin((localX + 37) * 0.767f) + MathF.Sin((localZ + 93) * 0.767f)) * 0.15f;
                var terrainHeight = octave0 + octave1 + octave2;
                return new Vector3(vX + terrainOffset.X, terrainHeight, vY + terrainOffset.Y);
            }, scale, BufferPool);
        var wrapped = new WrappedMesh(innerMesh);
        handle = Simulation.Statics.Add(new StaticDescription(staticPosition, QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2), Simulation.Shapes.Add(wrapped)));
    }

    void AddShapesAt(Vector3 center)
    {
        //Wide, shallow shapes maximize the number of triangle AABBs intersecting the convex AABB on the fine mesh; on the coarse mesh the same shapes
        //keep subpair counts well below MeshReduction's bruteForceThreshold of 128.

        //1) Small box: fewer than 128 subpairs on either mesh.
        {
            var box = new Box(1.2f, 1.2f, 1.2f);
            var shape = Simulation.Shapes.Add(box);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(-12, 4, 0), box.ComputeInertia(1), shape, 0.01f));
        }

        //2) Medium box: ~300-500 subpairs on the fine mesh (dictionary path), a handful on the coarse mesh.
        {
            var box = new Box(5f, 0.6f, 5f);
            var shape = Simulation.Shapes.Add(box);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(-4, 4, 0), box.ComputeInertia(1), shape, 0.01f));
        }

        //3) Large box: ~800-1000 subpairs on the fine mesh, still close to the skip threshold.
        {
            var box = new Box(8f, 0.6f, 8f);
            var shape = Simulation.Shapes.Add(box);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(6, 4, 0), box.ComputeInertia(1), shape, 0.01f));
        }

        //4) Oversized box: intentionally exceeds the 1024-subpair skip threshold on the fine mesh to confirm the fall-through doesn't crash.
        {
            var box = new Box(14f, 0.6f, 14f);
            var shape = Simulation.Shapes.Add(box);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(18, 4, 0), box.ComputeInertia(1), shape, 0.01f));
        }

        //5) A few rounded shapes rolling across the bumpy surface. Boundary smoothing matters most when contacts straddle edges, so rollers are a good stress test.
        {
            var sphere = new Sphere(1.5f);
            var shape = Simulation.Shapes.Add(sphere);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(-12, 6, 6), sphere.ComputeInertia(1), shape, 0.01f));

            var cylinder = new Cylinder(2.5f, 1.5f);
            var cylinderShape = Simulation.Shapes.Add(cylinder);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(-4, 6, 6), cylinder.ComputeInertia(1), cylinderShape, 0.01f));

            var capsule = new Capsule(0.8f, 4f);
            var capsuleShape = Simulation.Shapes.Add(capsule);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(6, 6, 6), capsule.ComputeInertia(1), capsuleShape, 0.01f));
        }

        //6) A Compound of a few boxes. This routes through CompoundMeshContinuations / CompoundMeshReduction instead of the convex-only MeshReduction path,
        //   but it still feeds MeshReductionThunks<WrappedMesh>, so it's the complementary check that compound-vs-wrapped-mesh boundary smoothing works too.
        {
            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 3);
            builder.Add(new Box(3f, 0.5f, 3f), RigidPose.Identity, 1);
            builder.Add(new Box(1.5f, 1.5f, 1.5f), new RigidPose(new Vector3(0, 1f, 0)), 1);
            builder.Add(new Box(0.75f, 0.75f, 4f), new RigidPose(new Vector3(1.5f, 0.5f, 0)), 1);
            builder.BuildDynamicCompound(out var children, out var compoundInertia);
            builder.Dispose();
            var compound = new Compound(children);
            var shape = Simulation.Shapes.Add(compound);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(14, 8, -6), compoundInertia, shape, 0.01f));
        }

        //7) A wide, low convex hull. Hulls exercise a different convex-triangle tester than boxes, so including one catches regressions specific to hull-triangle manifolds.
        {
            const int hullPoints = 32;
            var points = new QuickList<Vector3>(hullPoints, BufferPool);
            var random = new Random(5);
            for (int i = 0; i < hullPoints; ++i)
            {
                var xz = new Vector2(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1);
                //Flatten the hull so it covers a lot of ground when resting.
                points.AllocateUnsafely() = new Vector3(xz.X * 3f, (random.NextSingle() * 2 - 1) * 0.35f, xz.Y * 3f);
            }
            var hull = new ConvexHull(points.Span.Slice(points.Count), BufferPool, out _);
            var shape = Simulation.Shapes.Add(hull);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(center + new Vector3(-4, 8, -6), hull.ComputeInertia(1), shape, 0.01f));
        }
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        //The renderer's shape extractor switch doesn't know about WrappedMesh, so add each inner Mesh directly at its static's pose.
        //Using AddShape<Mesh> (rather than AddShape<WrappedMesh>) makes AddShape see Mesh.Id and routes to the existing mesh path.
        foreach (var (handle, innerMesh) in wrappedMeshes)
        {
            ref var pose = ref Simulation.Statics[handle].Pose;
            renderer.Shapes.AddShape(innerMesh, Simulation.Shapes, pose, new Vector3(0.7f, 0.7f, 0.75f));
        }

        var resolution = renderer.Surface.Resolution;
        renderer.TextBatcher.Write(text.Clear().Append("Two WrappedMesh terrains: fine (near) and coarse (far, +Z). Identical shapes are dropped on each."), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Fine mesh pushes MeshReduction into its dictionary path; coarse mesh keeps everything in the brute-force path."), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Note: the largest box on the fine mesh overlaps more than 1024 triangles, so MeshReduction.ReduceManifolds early-outs"), new Vector2(16, resolution.Y - 40), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("and no boundary smoothing is applied to it. Expect visible bumps there; the coarse-mesh counterpart still smooths."), new Vector2(16, resolution.Y - 24), 16, Vector3.One, font);
        base.Render(renderer, camera, input, text, font);
    }
}
