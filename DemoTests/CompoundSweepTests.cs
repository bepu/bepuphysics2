using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using Demos;
using System;
using System.Numerics;
using Xunit;

namespace DemoTests
{
    public static class CompoundSweepTests
    {
        struct NearestHitHandler : ISweepHitHandler
        {
            public float T;
            public bool AllowTest(CollidableReference collidable) => true;
            public bool AllowTest(CollidableReference collidable, int childIndex) => true;
            public void OnHit(ref float maximumT, float t, Vector3 hitLocation, Vector3 hitNormal, CollidableReference collidable)
            {
                T = MathF.Min(T, t);
                maximumT = MathF.Min(maximumT, t);
            }
            public void OnHitAtZeroT(ref float maximumT, CollidableReference collidable)
            {
                T = 0;
                maximumT = 0;
            }
        }

        struct AllowAllFilter : ISweepFilter
        {
            public bool AllowTest(int childA, int childB) => true;
        }

        //A wall whose near face is 0.75 units in front of the swept sphere, built as each kind of compound.
        const float WallZ = -0.9f;
        const float ExpectedDistance = 0.75f;
        static readonly string[] wallNames = ["Compound", "BigCompound", "Mesh"];
        static TypedIndex[] CreateWalls(Simulation simulation, BufferPool pool)
        {
            var box = simulation.Shapes.Add(new Box(1, 1, 0.1f));
            pool.Take(3, out Buffer<CompoundChild> compoundChildren);
            pool.Take(3, out Buffer<CompoundChild> bigCompoundChildren);
            for (int i = 0; i < 3; ++i)
            {
                compoundChildren[i] = new CompoundChild(new RigidPose(new Vector3(i - 1, 0, 0)), box);
                bigCompoundChildren[i] = compoundChildren[i];
            }
            pool.Take(2, out Buffer<Triangle> triangles);
            var a = new Vector3(-1.5f, -0.5f, 0.05f);
            var b = new Vector3(1.5f, -0.5f, 0.05f);
            var c = new Vector3(1.5f, 0.5f, 0.05f);
            var d = new Vector3(-1.5f, 0.5f, 0.05f);
            triangles[0] = new Triangle(a, c, b);
            triangles[1] = new Triangle(a, d, c);
            return
            [
                simulation.Shapes.Add(new Compound(compoundChildren)),
                simulation.Shapes.Add(new BigCompound(bigCompoundChildren, simulation.Shapes, pool)),
                simulation.Shapes.Add(new Mesh(triangles, Vector3.One, pool)),
            ];
        }

        //The same displacement split into velocity and maximumT in different ways; the compound and mesh overlap finders used to only look velocity * maximumT * maximumT ahead.
        static readonly (float Speed, float MaximumT)[] sweeps = [(1, 0.8f), (0.8f, 1), (0.4f, 2), (60, 1f / 60f)];

        [Fact]
        public static void ConvexSweepFindsCompoundChildrenForAnyMaximumT()
        {
            var pool = new BufferPool();
            var simulation = Simulation.Create(pool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(), new SolveDescription(1, 1));
            var walls = CreateWalls(simulation, pool);
            for (int i = 0; i < walls.Length; ++i)
                simulation.Statics.Add(new StaticDescription(new Vector3(10 * i, 0, WallZ), walls[i]));
            var sphere = new Sphere(0.1f);
            for (int i = 0; i < walls.Length; ++i)
            {
                foreach (var (speed, maximumT) in sweeps)
                {
                    var handler = new NearestHitHandler { T = float.MaxValue };
                    simulation.Sweep(sphere, new RigidPose(new Vector3(10 * i, 0, 0)), new BodyVelocity(new Vector3(0, 0, -speed)), maximumT, pool, ref handler);
                    Assert.True(handler.T < float.MaxValue, $"Sphere sweep at {wallNames[i]} wall missed with speed {speed}, maximumT {maximumT}.");
                    Assert.Equal(ExpectedDistance, handler.T * speed, 2);
                }
            }
            simulation.Dispose();
            pool.Clear();
        }

        [Fact]
        public static unsafe void CompoundSweepFindsCompoundChildrenForAnyMaximumT()
        {
            var pool = new BufferPool();
            var simulation = Simulation.Create(pool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(), new SolveDescription(1, 1));
            var walls = CreateWalls(simulation, pool);
            pool.Take(1, out Buffer<CompoundChild> sphereChildren);
            sphereChildren[0] = new CompoundChild(RigidPose.Identity, simulation.Shapes.Add(new Sphere(0.1f)));
            var swept = simulation.Shapes.Add(new Compound(sphereChildren));
            simulation.Shapes[swept.Type].GetShapeData(swept.Index, out var sweptData, out _);
            var registry = simulation.NarrowPhase.SweepTaskRegistry;
            for (int i = 0; i < walls.Length; ++i)
            {
                simulation.Shapes[walls[i].Type].GetShapeData(walls[i].Index, out var wallData, out _);
                var task = registry.GetTask(swept.Type, walls[i].Type);
                foreach (var (speed, maximumT) in sweeps)
                {
                    var filter = new AllowAllFilter();
                    var hit = task.Sweep(
                        sweptData, swept.Type, Quaternion.Identity, new BodyVelocity(new Vector3(0, 0, -speed)),
                        wallData, walls[i].Type, new Vector3(0, 0, WallZ), Quaternion.Identity, default,
                        maximumT, 1e-5f * maximumT, 1e-5f * maximumT, 25, ref filter, simulation.Shapes, registry, pool, out _, out var t1, out _, out _);
                    Assert.True(hit, $"Compound sweep at {wallNames[i]} wall missed with speed {speed}, maximumT {maximumT}.");
                    Assert.Equal(ExpectedDistance, t1 * speed, 2);
                }
            }
            simulation.Dispose();
            pool.Clear();
        }
    }
}
