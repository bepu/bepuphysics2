using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using DemoContentLoader;
using BepuPhysics.CollisionDetection;

namespace Demos.SpecializedTests
{
    public class BroadPhaseStressTestDemo : Demo
    {

        public struct NoNarrowphaseTestingCallbacks : INarrowPhaseCallbacks
        {

            public NoNarrowphaseTestingCallbacks()
            {
            }

            public void Initialize(Simulation simulation)
            {
            }

            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            {
                return false;
            }

            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return false;
            }

            public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                pairMaterial = default;
                return false;
            }

            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                return false;
            }

            public void Dispose()
            {
            }
        }

        Vector3[] startingLocations;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-20f, 13, -20f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new NoNarrowphaseTestingCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)), new SolveDescription(1, 1));

            var shape = new Sphere(0.5f);
            var sphereInertia = shape.ComputeInertia(1);
            var shapeIndex = Simulation.Shapes.Add(shape);
            const int width = 128;
            const int height = 128;
            const int length = 128;
            var spacing = new Vector3(1.01f);
            float randomization = 0.9f;
            var randomizationSpan = (spacing - new Vector3(1)) * randomization;
            var randomizationBase = randomizationSpan * -0.5f;
            var random = new Random(5);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var r = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
                        //var location = spacing * (new Vector3(i, j, k) + new Vector3(-width, 1, -length)) + randomizationBase + r * randomizationSpan;
                        var location = (r - new Vector3(0.5f)) * spacing * new Vector3(width, height, length);
                        //var hash = HashHelper.Rehash(HashHelper.Rehash(HashHelper.Rehash(i) + HashHelper.Rehash(j)) + HashHelper.Rehash(k));
                        var hash = i + j + k;
                        if (hash % 2 == 0)
                        {
                            Simulation.Bodies.Add(BodyDescription.CreateDynamic(location, sphereInertia, shapeIndex, -1));
                        }
                        else
                        {
                            Simulation.Statics.Add(new StaticDescription(location, shapeIndex));
                        }
                    }
                }
            }
            Console.WriteLine($"Body count: {Simulation.Bodies.ActiveSet.Count}");
            Console.WriteLine($"Static count: {Simulation.Statics.Count}");
            startingLocations = new Vector3[Simulation.Bodies.ActiveSet.Count];
            for (int i = 0; i < startingLocations.Length; ++i)
            {
                startingLocations[i] = Simulation.Bodies.ActiveSet.DynamicsState[i].Motion.Pose.Position;
            }
            refineTimes = new TimingsRingBuffer(sampleCount, BufferPool);
            testTimes = new TimingsRingBuffer(sampleCount, BufferPool);
        }

        const int sampleCount = 128;
        TimingsRingBuffer refineTimes;
        TimingsRingBuffer testTimes;
        long frameCount;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var rotationAngle = frameCount * 1e-3f;
            var rotation = Matrix3x3.CreateFromAxisAngle(Vector3.UnitY, rotationAngle);
            for (int i = 0; i < Simulation.Bodies.ActiveSet.Count / 2; ++i)
            {
                //For every body, set the velocity such that body moves toward some body-specific goal state that evolves over time.
                Matrix3x3.Transform(startingLocations[i], rotation, out var targetLocation);
                ref var motion = ref Simulation.Bodies.ActiveSet.DynamicsState[i].Motion;
                var offset = targetLocation - motion.Pose.Position;
                motion.Velocity.Linear = offset;
            }

            //Simulation.BroadPhase.ActiveTree.CacheOptimize(0);
            //Simulation.BroadPhase.StaticTree.CacheOptimize(0);
            base.Update(window, camera, input, dt);
            refineTimes.Add(Simulation.Profiler[Simulation.BroadPhase]);
            testTimes.Add(Simulation.Profiler[Simulation.BroadPhaseOverlapFinder]);


            if (frameCount++ % sampleCount == 0)
            {
                var refineStats = refineTimes.ComputeStats();
                var testStats = testTimes.ComputeStats();
                Console.WriteLine($"Refine: {refineStats.Average * 1000} ms average, {refineStats.StdDev * 1000} stddev");
                Console.WriteLine($"Test:   {testStats.Average * 1000} ms average, {testStats.StdDev * 1000} stddev");
                Console.WriteLine($"Active Cost: {Simulation.BroadPhase.ActiveTree.MeasureCostMetric()}");
                Console.WriteLine($"Static Cost: {Simulation.BroadPhase.StaticTree.MeasureCostMetric()}");
                Console.WriteLine($"Active CQ:   {Simulation.BroadPhase.ActiveTree.MeasureCacheQuality()}");
                Console.WriteLine($"Static CQ:   {Simulation.BroadPhase.StaticTree.MeasureCacheQuality()}");

                var a = Stopwatch.GetTimestamp();
                Simulation.BroadPhase.ActiveTree.Refit2();
                var b = Stopwatch.GetTimestamp();
                Simulation.BroadPhase.StaticTree.Refit2();
                var c = Stopwatch.GetTimestamp();


                Console.WriteLine($"Manual ST refit active: {1e3 * (b - a) / Stopwatch.Frequency} ms");
                Console.WriteLine($"Manual ST refit static: {1e3 * (c - b) / Stopwatch.Frequency} ms");

            }
        }

    }
}
