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
        public unsafe struct NoCollisionCallbacks : INarrowPhaseCallbacks
        {
            public void Initialize(Simulation simulation)
            {
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
            {
                return false;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return false;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            void ConfigureMaterial(out PairMaterialProperties pairMaterial)
            {
                pairMaterial = new PairMaterialProperties();
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
            {
                pairMaterial = new PairMaterialProperties();
                return false;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
            {
                pairMaterial = new PairMaterialProperties();
                return false;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
            {
                return false;
            }

            public void Dispose()
            {
            }
        }
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-20f, 13, -20f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new NoCollisionCallbacks());
            //Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var box = new Box(0.5f, 1.5f, 1f);
            var capsule = new Capsule(0.5f, 0.5f);
            var sphere = new Sphere(.5f);
            var boxIndex = Simulation.Shapes.Add(ref box);
            var capsuleIndex = Simulation.Shapes.Add(ref capsule);
            var sphereIndex = Simulation.Shapes.Add(ref sphere);
            const int width = 16;
            const int height = 16;
            const int length = 16;
            var spacing = new Vector3(2.01f);
            var halfSpacing = spacing / 2;
            float randomizationSubset = 0.9f;
            var randomizationSpan = (spacing - new Vector3(1)) * randomizationSubset;
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
        
                        BepuUtilities.Quaternion orientation;
                        orientation.X = -1 + 2 * (float)random.NextDouble();
                        orientation.Y = -1 + 2 * (float)random.NextDouble();
                        orientation.Z = -1 + 2 * (float)random.NextDouble();
                        orientation.W = 0.01f + (float)random.NextDouble();
                        orientation.Normalize();

                        TypedIndex shapeIndex;
                        switch ((i + j + k) % 3)
                        {
                            case 0:
                                shapeIndex = boxIndex;
                                break;
                            case 1:
                                shapeIndex = capsuleIndex;
                                break;
                            default:
                                shapeIndex = sphereIndex;
                                break;
                        }

                        if ((i + j + k) % 2 == 1)
                        {
                            var bodyDescription = new BodyDescription
                            {
                                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = -0.1f },
                                Pose = new RigidPose
                                {
                                    Orientation = orientation,
                                    Position = location
                                },
                                Collidable = new CollidableDescription
                                {
                                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                    SpeculativeMargin = 0.1f,
                                    Shape = shapeIndex
                                }
                            };
                            Simulation.Bodies.Add(ref bodyDescription);
                        }
                        else
                        {
                            var staticDescription = new StaticDescription
                            {
                                Pose = new RigidPose
                                {
                                    Orientation = orientation,
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

            //Spew rays all over the place, starting inside the shape cube.
            int randomRayCount = 1 << 12;
            QuickList<TestRay, Buffer<TestRay>>.Create(BufferPool.SpecializeFor<TestRay>(), randomRayCount, out randomRays);
            for (int i = 0; i < randomRayCount; ++i)
            {
                var direction = GetDirection(random);
                randomRays.AllocateUnsafely() = new TestRay
                {
                    Origin = GetDirection(random) * width * spacing * 0.25f,
                    Direction = GetDirection(random),
                    MaximumT = 50
                };
            }

            //Send rays out matching a planar projection.
            int frustumRayWidth = 40;
            int frustumRayHeight = 20;
            float aspectRatio = 1.6f;
            float verticalFOV = MathHelper.Pi * 0.2f;
            var unitZScreenHeight = 2 * MathF.Tan(verticalFOV / 2);
            var unitZScreenWidth = unitZScreenHeight * aspectRatio;
            var unitZSpacing = new Vector2(unitZScreenWidth / frustumRayWidth, unitZScreenHeight / frustumRayHeight);
            var unitZBase = (unitZSpacing - new Vector2(unitZScreenWidth, unitZScreenHeight)) * 0.5f;
            QuickList<TestRay, Buffer<TestRay>>.Create(BufferPool.SpecializeFor<TestRay>(), frustumRayWidth * frustumRayHeight, out frustumRays);
            var frustumOrigin = new Vector3(0, 0, -50);
            for (int i = 0; i < frustumRayWidth; ++i)
            {
                for (int j = 0; j < frustumRayHeight; ++j)
                {
                    frustumRays.AllocateUnsafely() = new TestRay
                    {
                        Origin = frustumOrigin,
                        Direction = new Vector3(unitZBase + new Vector2(i, j) * unitZSpacing, 1),
                        MaximumT = 100
                    };
                }
            }

            //Send a wall of rays. Matches an orthographic projection.
            int wallWidth = 32;
            int wallHeight = 32;
            var wallOrigin = new Vector3(0, 0, -50);
            var wallSpacing = new Vector2(0.25f);
            var wallBase = 0.5f * (wallSpacing - wallSpacing * new Vector2(wallWidth, wallHeight));
            QuickList<TestRay, Buffer<TestRay>>.Create(BufferPool.SpecializeFor<TestRay>(), wallWidth * wallHeight, out wallRays);
            for (int i = 0; i < wallWidth; ++i)
            {
                for (int j = 0; j < wallHeight; ++j)
                {
                    wallRays.AllocateUnsafely() = new TestRay
                    {
                        Origin = wallOrigin + new Vector3(wallBase + wallSpacing * new Vector2(i, j), 0),
                        Direction = new Vector3(0, 0, 1),
                        MaximumT = 100
                    };
                }
            }
            var maxRayCount = Math.Max(randomRays.Count, Math.Max(frustumRays.Count, wallRays.Count));
            QuickList<TestRay, Buffer<TestRay>>.Create(BufferPool.SpecializeFor<TestRay>(), maxRayCount, out testRays);
            BufferPool.Take(maxRayCount, out batchedResults);
            BufferPool.Take(maxRayCount, out unbatchedResults);
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
        QuickList<TestRay, Buffer<TestRay>> randomRays;
        QuickList<TestRay, Buffer<TestRay>> frustumRays;
        QuickList<TestRay, Buffer<TestRay>> wallRays;

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


        const int sampleCount = 16;
        TimingsRingBuffer batchedQueryTimes = new TimingsRingBuffer(sampleCount);
        TimingsRingBuffer unbatchedQueryTimes = new TimingsRingBuffer(sampleCount);
        int frameCount;
        float rotation;
        void CopyAndRotate(ref QuickList<TestRay, Buffer<TestRay>> source)
        {
            testRays.Count = source.Count;
            var transform = Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), rotation);

            for (int i = 0; i < source.Count; ++i)
            {
                ref var targetRay = ref testRays[i];
                ref var sourceRay = ref source[i];
                Matrix3x3.Transform(ref sourceRay.Origin, ref transform, out targetRay.Origin);
                Matrix3x3.Transform(ref sourceRay.Direction, ref transform, out targetRay.Direction);
                targetRay.MaximumT = sourceRay.MaximumT;
            }
        }
        public unsafe override void Update(Input input, float dt)
        {
            base.Update(input, dt);

            ++frameCount;
            if (frameCount > 1 << 20)
                frameCount = 0;
            rotation += (MathF.PI * 1e-2f * (1 / 60f)) % (2 * MathF.PI);


            switch ((frameCount / 256) % 3)
            {
                case 0:
                    CopyAndRotate(ref randomRays);
                    break;
                case 1:
                    CopyAndRotate(ref frustumRays);
                    break;
                default:
                    CopyAndRotate(ref wallRays);
                    break;
            }



            CacheBlaster.Blast();
            double batchedTime;
            {
                int intersectionCount = 0;
                for (int i = 0; i < testRays.Count; ++i)
                {
                    batchedResults[i].T = float.MaxValue;
                    batchedResults[i].Hit = false;
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
                batchedQueryTimes.Add(batchedTime = (stop - start) / (double)Stopwatch.Frequency);

                batcher.Dispose();
            }
            CacheBlaster.Blast();
            double unbatchedTime;
            {
                int intersectionCount = 0;
                for (int i = 0; i < testRays.Count; ++i)
                {
                    unbatchedResults[i].T = float.MaxValue;
                    unbatchedResults[i].Hit = false;
                }
                var hitHandler = new HitHandler { Hits = unbatchedResults, IntersectionCount = &intersectionCount };
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < testRays.Count; ++i)
                {
                    ref var ray = ref testRays[i];
                    Simulation.RayCast(ref ray.Origin, ref ray.Direction, ray.MaximumT, ref hitHandler, i);
                }
                var stop = Stopwatch.GetTimestamp();
                unbatchedQueryTimes.Add(unbatchedTime = (stop - start) / (double)Stopwatch.Frequency);

            }

            for (int i = 0; i < testRays.Count; ++i)
            {
                Debug.Assert(unbatchedResults[i].Hit == batchedResults[i].Hit && (!batchedResults[i].Hit || Math.Abs(batchedResults[i].T - unbatchedResults[i].T) < 1e-6f));
            }

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

        void WriteResults(string name, double time, double baseline, float y, TextBatcher batcher, TextBuilder text, Font font)
        {
            batcher.Write(
                text.Clear().Append(name).Append(":"),
                new Vector2(32, y), 16, new Vector3(1), font);
            batcher.Write(
                text.Clear().Append(time * 1e6, 2),
                new Vector2(128, y), 16, new Vector3(1), font);
            batcher.Write(
                text.Clear().Append(testRays.Count / time, 0),
                new Vector2(224, y), 16, new Vector3(1), font);
            batcher.Write(
                text.Clear().Append(baseline / time, 2),
                new Vector2(350, y), 16, new Vector3(1), font);
        }

        public override void Render(Renderer renderer, TextBuilder text, Font font)
        {
            var batchedPackedColor = Helpers.PackColor(new Vector3(0.75f, 0.75f, 0));
            var batchedPackedNormalColor = Helpers.PackColor(new Vector3(1f, 1f, 0));
            var batchedPackedBackgroundColor = Helpers.PackColor(new Vector3());

            DrawRays(ref batchedResults, renderer, new Vector3(0.25f, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 1, 0), new Vector3());

            renderer.TextBatcher.Write(text.Clear().Append("Ray count: ").Append(testRays.Count), new Vector2(32, renderer.Surface.Resolution.Y - 80), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Time (us):"), new Vector2(128, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Rays per second:"), new Vector2(224, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Relative speed:"), new Vector2(350, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);

            var batchedStats = batchedQueryTimes.ComputeStats();
            var unbatchedStats = unbatchedQueryTimes.ComputeStats();
            WriteResults("Unbatched", unbatchedStats.Average, unbatchedStats.Average, renderer.Surface.Resolution.Y - 48, renderer.TextBatcher, text, font);
            WriteResults("Batched", batchedStats.Average, unbatchedStats.Average, renderer.Surface.Resolution.Y - 32, renderer.TextBatcher, text, font);

            base.Render(renderer, text, font);
        }

    }
}
