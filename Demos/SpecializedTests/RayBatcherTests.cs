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
using DemoRenderer.UI;
using DemoRenderer.Constraints;

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
            const int width = 16;
            const int height = 16;
            const int length = 16;
            var spacing = new Vector3(2.01f);
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
                        var location = spacing * (new Vector3(i, j, k) + new Vector3(-width, -height, -length) * 0.5f) + randomizationBase + r * randomizationSpan;
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

            int rayCount = 1 << 10;
            QuickList<TestRay, Buffer<TestRay>>.Create(BufferPool.SpecializeFor<TestRay>(), rayCount, out testRays);
            BufferPool.Take(rayCount, out batchedResults);
            BufferPool.Take(rayCount, out unbatchedResults);

            for (int i = 0; i < rayCount; ++i)
            {
                var direction = GetDirection(random);
                testRays.AllocateUnsafely() = new TestRay
                {
                    Origin = GetDirection(random) * width * spacing * 0.25f,
                    Direction = GetDirection(random),
                    MaximumT = 50// 50 + (float)random.NextDouble() * 300
                    //Origin = new Vector3(-500, 0, 0),
                    //Direction = new Vector3(1, 0, 0),
                    //MaximumT = 50
                    //Origin = new Vector3(-500),
                    //Direction = new Vector3(1),
                    //MaximumT = 50
                    //    Direction = new Vector3(0.5346225f, -0.7184405f, -0.4449964f),
                    //MaximumT = 50,
                    //Origin = new Vector3(0.8212769f, -0.8531729f, 0.9328218f)
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

        struct RayHit
        {
            public Vector3 Normal;
            public float T;
            public CollidableReference Collidable;
            public bool Hit;
        }
        Buffer<RayHit> batchedResults;
        Buffer<RayHit> unbatchedResults;

        unsafe struct RayTester : IBroadPhaseBatchedRayTester
        {
            public int* IntersectionCount;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void RayTest(CollidableReference collidable, ref RaySource rays)
            {
                *IntersectionCount += rays.RayCount;
            }

            public void RayTest(CollidableReference collidable, RayData* rayData, float* maximumT)
            {
                ++*IntersectionCount;
            }
        }


        unsafe struct HitHandler : IRayHitHandler
        {
            public Buffer<RayHit> Hits;
            public int* IntersectionCount;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowTest(CollidableReference collidable)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, CollidableReference collidable)
            {
                maximumT = t;
                ref var hit = ref Hits[ray.Id];
                if (t < hit.T)
                {
                    if (hit.T == float.MaxValue)
                        ++*IntersectionCount;
                    hit.Normal = normal;
                    hit.T = t;
                    hit.Collidable = collidable;
                    hit.Hit = true;
                }
            }
        }


        const int sampleCount = 1;
        TimingsRingBuffer broadPhaseQueryTimes = new TimingsRingBuffer(sampleCount);
        TimingsRingBuffer simulationQueryTimes = new TimingsRingBuffer(sampleCount);
        long frameCount;
        public unsafe override void Update(Input input, float dt)
        {
            base.Update(input, dt);

            //{
            //    int leafIntersectionCount = 0;
            //    var rayTester = new RayTester { IntersectionCount = &leafIntersectionCount };
            //    var batcher = new BroadPhaseRayBatcher<RayTester>(BufferPool, Simulation.BroadPhase, rayTester);

            //    var start = Stopwatch.GetTimestamp();
            //    for (int i = 0; i < testRays.Count; ++i)
            //    {
            //        ref var ray = ref testRays[i];
            //        batcher.Add(ref ray.Origin, ref ray.Direction, ray.MaximumT, i);
            //    }
            //    batcher.Flush();
            //    var stop = Stopwatch.GetTimestamp();
            //    broadPhaseQueryTimes.Add((stop - start) / (double)Stopwatch.Frequency);

            //    batcher.Dispose();
            //    if (frameCount % sampleCount == 0)
            //    {
            //        var stats = broadPhaseQueryTimes.ComputeStats();
            //        Console.WriteLine($"BroadPhase Query times: {stats.Average * 1000} ms average, {stats.StdDev * 1000} stddev, {leafIntersectionCount} intersectionCount");
            //        Console.WriteLine($"Time per ray: {1e9 * stats.Average / testRays.Count} ns");
            //        Console.WriteLine($"Time per intersection: {1e9 * stats.Average / leafIntersectionCount} ns");
            //        Console.WriteLine($"Intersections per ray: {leafIntersectionCount / (double)testRays.Count}");
            //    }
            //}

            CacheBlaster.Blast();
            double batchedTime;
            {
                int intersectionCount = 0;
                for (int i = 0; i < testRays.Count; ++i)
                {
                    batchedResults[i].T = float.MaxValue;
                }
                var hitHandler = new HitHandler { Hits = batchedResults, IntersectionCount = &intersectionCount };
                var batcher = new SimulationRayBatcher<HitHandler>(BufferPool, Simulation, hitHandler, 4096);
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < testRays.Count; ++i)
                {
                    ref var ray = ref testRays[i];
                    batcher.Add(ref ray.Origin, ref ray.Direction, ray.MaximumT, i);
                }
                batcher.Flush();
                var stop = Stopwatch.GetTimestamp();
                simulationQueryTimes.Add(batchedTime = (stop - start) / (double)Stopwatch.Frequency);

                batcher.Dispose();
                if (frameCount % sampleCount == 0)
                {
                    var stats = simulationQueryTimes.ComputeStats();
                    Console.WriteLine($"Batched times: {stats.Average * 1000} ms average, {stats.StdDev * 1000} stddev, {intersectionCount} intersectionCount");
                    Console.WriteLine($"Time per ray: {1e9 * stats.Average / testRays.Count} ns");
                    //Console.WriteLine($"Time per intersection: {1e9 * stats.Average / intersectionCount} ns");
                    Console.WriteLine($"Intersections per ray: {intersectionCount / (double)testRays.Count}");
                }
            }
            CacheBlaster.Blast();
            double unbatchedTime;
            {
                int intersectionCount = 0;
                for (int i = 0; i < testRays.Count; ++i)
                {
                    unbatchedResults[i].T = float.MaxValue;
                }
                var hitHandler = new HitHandler { Hits = unbatchedResults, IntersectionCount = &intersectionCount };
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < testRays.Count; ++i)
                {
                    ref var ray = ref testRays[i];
                    Simulation.RayCast(ref ray.Origin, ref ray.Direction, ray.MaximumT, ref hitHandler, i);
                }
                var stop = Stopwatch.GetTimestamp();
                simulationQueryTimes.Add(unbatchedTime = (stop - start) / (double)Stopwatch.Frequency);

                if (frameCount % sampleCount == 0)
                {
                    var stats = simulationQueryTimes.ComputeStats();
                    Console.WriteLine($"Unbatched times: {stats.Average * 1000} ms average, {stats.StdDev * 1000} stddev, {intersectionCount} intersectionCount");
                    Console.WriteLine($"Time per ray: {1e9 * stats.Average / testRays.Count} ns");
                    //Console.WriteLine($"Time per intersection: {1e9 * stats.Average / intersectionCount} ns");
                    Console.WriteLine($"Intersections per ray: {intersectionCount / (double)testRays.Count}");
                }
            }

            if (frameCount % sampleCount == 0)
            {
                Console.ForegroundColor = ConsoleColor.Magenta;
                Console.WriteLine($"Batched Speedup: {unbatchedTime / batchedTime}");
                Console.ResetColor();
            }
            ++frameCount;

        }

        void DrawRays(ref Buffer<RayHit> results, Renderer renderer, Vector3 foregroundMissColor, Vector3 foregroundHitColor, Vector3 foregroundNormalColor, Vector3 backgroundColor)
        {
            var packedForegroundMiss = Helpers.PackColor(foregroundMissColor);
            var packedForegroundHit = Helpers.PackColor(foregroundHitColor);
            var packedForegroundNormal = Helpers.PackColor(foregroundNormalColor);
            var packedBackground = Helpers.PackColor(backgroundColor);
            for (int i = 0; i < testRays.Count; ++i)
            {
                ref var result = ref results[i];
                ref var ray = ref testRays[i];
                if (result.Hit)
                {
                    var end = ray.Origin + ray.Direction * result.T;
                    renderer.Lines.Allocate() = new LineInstance(ray.Origin, end, packedForegroundHit, packedBackground);
                    renderer.Lines.Allocate() = new LineInstance(end, end + result.Normal, packedForegroundNormal, packedBackground);
                }
                else
                {
                    var end = ray.Origin + ray.Direction * ray.MaximumT;
                    renderer.Lines.Allocate() = new LineInstance(ray.Origin, end, packedForegroundMiss, packedBackground);
                }
            }
        }

        public override void Render(Renderer renderer, TextBuilder text, Font font)
        {
            var batchedPackedColor = Helpers.PackColor(new Vector3(0.75f, 0.75f, 0));
            var batchedPackedNormalColor = Helpers.PackColor(new Vector3(1f, 1f, 0));
            var batchedPackedBackgroundColor = Helpers.PackColor(new Vector3());

            DrawRays(ref batchedResults, renderer, new Vector3(0.5f, 0.5f, 0), new Vector3(1, 1, 0), new Vector3(0, 1, 0), new Vector3());
            //DrawRays(ref unbatchedResults, renderer, new Vector3(0.5f, 0, 0.5f), new Vector3(0.75f, 0, 0.75f), new Vector3());
            base.Render(renderer, text, font);
        }

    }
}
