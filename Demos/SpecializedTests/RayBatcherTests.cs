using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Runtime.CompilerServices;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Trees;

namespace Demos.SpecializedTests
{
    public class RayBatcherTests : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-20f, 13, -20f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            //Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var shape = new Sphere(0.5f);
            shape.ComputeInertia(1, out var sphereInertia);
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 32;
            const int height = 32;
            const int length = 32;
            var spacing = new Vector3(1.01f);
            var halfSpacing = spacing / 2;
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
                        var r = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                        //var location = spacing * (new Vector3(i, j, k)) + randomizationBase + r * randomizationSpan;
                        var location = spacing * (new Vector3(i, j, k) + new Vector3(-width, 1, -length) * 0.5f) + randomizationBase + r * randomizationSpan;
                        //var location = new Vector3();
                        if ((i + j + k) % 2 == 1)
                        {
                            var bodyDescription = new BodyDescription
                            {
                                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = -0.1f },
                                Pose = new RigidPose
                                {
                                    Orientation = BepuUtilities.Quaternion.Identity,
                                    Position = location
                                },
                                Collidable = new CollidableDescription
                                {
                                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                    SpeculativeMargin = 0.1f,
                                    Shape = shapeIndex
                                },
                                LocalInertia = sphereInertia
                            };
                            Simulation.Bodies.Add(ref bodyDescription);
                        }
                        else
                        {
                            var staticDescription = new StaticDescription
                            {
                                Pose = new RigidPose
                                {
                                    Orientation = BepuUtilities.Quaternion.Identity,
                                    Position = location
                                },
                                Collidable = new CollidableDescription
                                {
                                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                    SpeculativeMargin = 0.1f,
                                    Shape = shapeIndex
                                }
                            };
                            Simulation.Statics.Add(ref staticDescription);
                        }

                    }
                }
            }

            int rayCount = 8192;
            QuickList<TestRay, Buffer<TestRay>>.Create(BufferPool.SpecializeFor<TestRay>(), rayCount, out testRays);

            for (int i = 0; i < rayCount; ++i)
            {
                var direction = GetDirection(random);
                testRays.AllocateUnsafely() = new TestRay
                {
                    Origin = -direction * 300,
                    Direction = direction,
                    MaximumT = 1000// 50 + (float)random.NextDouble() * 300
                    //Origin = new Vector3(-100, 0, 0),
                    //Direction = new Vector3(1, 0, 0),
                    //MaximumT = 1000
                };
            }
        }
        static Vector3 GetDirection(Random random)
        {
            Vector3 direction;
            float length;
            do
            {
                direction = 2 * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - Vector3.One;
                length = direction.Length();
            }
            while (length < 1e-7f);
            direction /= length;
            return direction;
        }

        struct TestRay
        {
            public Vector3 Origin;
            public float MaximumT;
            public Vector3 Direction;
        }

        QuickList<TestRay, Buffer<TestRay>> testRays;

        unsafe struct RayTester : IBroadPhaseRayTester
        {
            public int* IntersectionCount;
            public void RayTest(CollidableReference collidable, ref RaySource rays)
            {
                *IntersectionCount += rays.RayCount;
            }
        }

        const int sampleCount = 128;
        TimingsRingBuffer times = new TimingsRingBuffer(sampleCount);
        long frameCount;
        public unsafe override void Update(Input input, float dt)
        {
            base.Update(input, dt);

            int leafIntersectionCount = 0;
            var rayTester = new RayTester { IntersectionCount = &leafIntersectionCount };
            var batcher = new BroadPhaseRayBatcher<RayTester>(BufferPool, Simulation.BroadPhase, rayTester);

            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < testRays.Count; ++i)
            {
                ref var ray = ref testRays[i];
                batcher.Add(ref ray.Origin, ref ray.Direction, ray.MaximumT, i);
            }
            batcher.Flush();
            var stop = Stopwatch.GetTimestamp();
            times.Add((stop - start) / (double)Stopwatch.Frequency);

            batcher.Dispose();
            if (frameCount++ % sampleCount == 0)
            {
                var stats = times.ComputeStats();
                Console.WriteLine($"Test times: {stats.Average * 1000} ms average, {stats.StdDev * 1000} stddev, {leafIntersectionCount} intersectionCount");
                Console.WriteLine($"Time per ray: {1e9 * stats.Average / testRays.Count} ns");
                Console.WriteLine($"Time per intersection: {1e9 * stats.Average / leafIntersectionCount} ns");
                Console.WriteLine($"Intersections per ray: {leafIntersectionCount / (double)testRays.Count}");
            }
        }

    }
}
