﻿using BepuUtilities;
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
using System.Threading;
using Demos.SpecializedTests;
using DemoContentLoader;
using Helpers = DemoRenderer.Helpers;

namespace Demos
{
    public class RayCastingDemo : Demo
    {
        public unsafe struct NoCollisionCallbacks : INarrowPhaseCallbacks
        {
            public void Initialize(Simulation simulation)
            {
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            {
                return false;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return false;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                pairMaterial = new PairMaterialProperties();
                return false;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                return false;
            }

            public void Dispose()
            {
            }
        }
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-20f, 13, -20f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new NoCollisionCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));



            var sphere = new Sphere(0.5f);
            var capsule = new Capsule(0, 0.5f);
            var box = new Box(0.5f, 1.5f, 1f);
            var cylinder = new Cylinder(0.5f, 1);
            const int pointCount = 16;
            var points = new QuickList<Vector3>(pointCount, BufferPool);
            var random = new Random(5);
            for (int i = 0; i < pointCount; ++i)
            {
                points.AllocateUnsafely() = new Vector3(1 * random.NextSingle(), 1 * random.NextSingle(), 1 * random.NextSingle());
            }
            var hullShape = new ConvexHull(points, BufferPool, out _);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var boxIndex = Simulation.Shapes.Add(box);
            var cylinderIndex = Simulation.Shapes.Add(cylinder);
            var hullIndex = Simulation.Shapes.Add(hullShape);
            const int width = 16;
            const int height = 16;
            const int length = 16;
            var spacing = new Vector3(2.01f);
            var halfSpacing = spacing / 2;
            float randomizationSubset = 0.9f;
            var randomizationSpan = (spacing - new Vector3(1)) * randomizationSubset;
            var randomizationBase = randomizationSpan * -0.5f;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var r = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
                        var location = spacing * (new Vector3(i, j, k) + new Vector3(-width, -height, -length) * 0.5f) + randomizationBase + r * randomizationSpan;

                        Quaternion orientation;
                        orientation.X = -1 + 2 * random.NextSingle();
                        orientation.Y = -1 + 2 * random.NextSingle();
                        orientation.Z = -1 + 2 * random.NextSingle();
                        orientation.W = 0.01f + random.NextSingle();
                        QuaternionEx.Normalize(ref orientation);
                        var shapeIndex = ((i + j + k) % 5) switch
                        {
                            0 => boxIndex,
                            1 => capsuleIndex,
                            2 => sphereIndex,
                            3 => cylinderIndex,
                            _ => hullIndex,
                        };
                        if ((i + j + k) % 2 == 1)
                        {
                            Simulation.Bodies.Add(BodyDescription.CreateKinematic((location, orientation), shapeIndex, -0.1f));
                        }
                        else
                        {
                            Simulation.Statics.Add(new StaticDescription(location, orientation, shapeIndex));
                        }
                    }
                }
            }

            const int planeWidth = 128;
            const int planeHeight = 128;
            var planeMesh = DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    return new Vector3(x - planeWidth / 2, 1 * MathF.Cos(x / 4f) * MathF.Sin(y / 4f), y - planeHeight / 2);
                }, new Vector3(1, 3, 1), BufferPool);
            Simulation.Statics.Add(new StaticDescription(
                new Vector3(0, -10, 0), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 4),
                Simulation.Shapes.Add(planeMesh)));

            int raySourceCount = 3;
            raySources = new QuickList<QuickList<TestRay>>(raySourceCount, BufferPool);
            raySources.Count = raySourceCount;

            //Spew rays all over the place, starting inside the shape cube.
            int randomRayCount = 1 << 14;
            ref var randomRays = ref raySources[0];
            randomRays = new QuickList<TestRay>(randomRayCount, BufferPool);
            for (int i = 0; i < randomRayCount; ++i)
            {
                var direction = GetDirection(random);
                var originScale = (float)Math.Sqrt(random.NextDouble());
                randomRays.AllocateUnsafely() = new TestRay
                {
                    Origin = originScale * GetDirection(random) * width * spacing * 0.25f,
                    Direction = GetDirection(random),
                    MaximumT = 50
                };
            }

            //Send rays out matching a planar projection.
            int frustumRayWidth = 128;
            int frustumRayHeight = 128;
            float aspectRatio = 1.6f;
            float verticalFOV = MathHelper.Pi * 0.16f;
            var unitZScreenHeight = 2 * MathF.Tan(verticalFOV / 2);
            var unitZScreenWidth = unitZScreenHeight * aspectRatio;
            var unitZSpacing = new Vector2(unitZScreenWidth / frustumRayWidth, unitZScreenHeight / frustumRayHeight);
            var unitZBase = (unitZSpacing - new Vector2(unitZScreenWidth, unitZScreenHeight)) * 0.5f;
            ref var frustumRays = ref raySources[1];
            frustumRays = new QuickList<TestRay>(frustumRayWidth * frustumRayHeight, BufferPool);
            var frustumOrigin = new Vector3(0, 0, -50);
            for (int i = 0; i < frustumRayWidth; ++i)
            {
                for (int j = 0; j < frustumRayHeight; ++j)
                {
                    ref var ray = ref frustumRays.AllocateUnsafely();
                    ray.Direction = new Vector3(unitZBase + new Vector2(i, j) * unitZSpacing, 1);
                    ray.Origin = frustumOrigin + ray.Direction * 10;
                    ray.MaximumT = 100;
                }
            }

            //Send a wall of rays. Matches an orthographic projection.
            int wallWidth = 128;
            int wallHeight = 128;
            var wallOrigin = new Vector3(0, 0, -50);
            var wallSpacing = new Vector2(0.1f);
            var wallBase = 0.5f * (wallSpacing - wallSpacing * new Vector2(wallWidth, wallHeight));
            ref var wallRays = ref raySources[2];
            wallRays = new QuickList<TestRay>(wallWidth * wallHeight, BufferPool);
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
            testRays = new QuickList<TestRay>(maxRayCount, BufferPool);
            var timeSampleCount = 16;
            algorithms = new IntersectionAlgorithm[2];
            algorithms[0] = new IntersectionAlgorithm("Unbatched", UnbatchedWorker, BufferPool, maxRayCount, timeSampleCount);
            algorithms[1] = new IntersectionAlgorithm("Batched", BatchedWorker, BufferPool, maxRayCount, timeSampleCount);

            BufferPool.Take(Environment.ProcessorCount * 2, out jobs);
        }

        static Vector3 GetDirection(Random random)
        {
            Vector3 direction;
            float length;
            do
            {
                direction = 2 * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) - Vector3.One;
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
        QuickList<QuickList<TestRay>> raySources;
        QuickList<TestRay> testRays;


        struct RayHit
        {
            public Vector3 Normal;
            public float T;
            public CollidableReference Collidable;
            public bool Hit;
        }
        unsafe class IntersectionAlgorithm
        {
            public string Name;
            public int IntersectionCount;
            public Buffer<RayHit> Results;
            public TimingsRingBuffer Timings;

            Func<int, IntersectionAlgorithm, int> worker;
            Action<int> internalWorker;
            public int JobIndex;

            public IntersectionAlgorithm(string name, Func<int, IntersectionAlgorithm, int> worker,
                BufferPool pool, int largestRayCount, int timingSampleCount = 16)
            {
                Name = name;
                Timings = new TimingsRingBuffer(timingSampleCount, pool);
                this.worker = worker;
                internalWorker = ExecuteWorker;
                pool.Take(largestRayCount, out Results);
            }

            unsafe void ExecuteWorker(int workerIndex)
            {
                var intersectionCount = worker(workerIndex, this);
                Interlocked.Add(ref IntersectionCount, intersectionCount);
            }

            public void Execute(ref QuickList<TestRay> rays, IThreadDispatcher dispatcher)
            {
                CacheBlaster.Blast();
                for (int i = 0; i < rays.Count; ++i)
                {
                    Results[i].T = float.MaxValue;
                    Results[i].Hit = false;
                }
                JobIndex = -1;
                IntersectionCount = 0;
                var start = Stopwatch.GetTimestamp();
                if (dispatcher != null)
                {
                    dispatcher.DispatchWorkers(internalWorker);
                }
                else
                {
                    internalWorker(0);
                }
                var stop = Stopwatch.GetTimestamp();
                Timings.Add((stop - start) / (double)Stopwatch.Frequency);
            }
        }



        unsafe int BatchedWorker(int workerIndex, IntersectionAlgorithm algorithm)
        {
            int intersectionCount = 0;
            var hitHandler = new HitHandler { Hits = algorithm.Results, IntersectionCount = &intersectionCount };
            var batcher = new SimulationRayBatcher<HitHandler>(ThreadDispatcher.WorkerPools[workerIndex], Simulation, hitHandler, 2048);
            int claimedIndex;
            while ((claimedIndex = Interlocked.Increment(ref algorithm.JobIndex)) < jobs.Length)
            {
                ref var job = ref jobs[claimedIndex];
                for (int i = job.Start; i < job.End; ++i)
                {
                    ref var ray = ref testRays[i];
                    batcher.Add(ref ray.Origin, ref ray.Direction, ray.MaximumT, i);
                }
            }
            batcher.Flush();
            batcher.Dispose();
            return intersectionCount;
        }

        unsafe int UnbatchedWorker(int workerIndex, IntersectionAlgorithm algorithm)
        {
            int intersectionCount = 0;
            var hitHandler = new HitHandler { Hits = algorithm.Results, IntersectionCount = &intersectionCount };
            int claimedIndex;
            while ((claimedIndex = Interlocked.Increment(ref algorithm.JobIndex)) < jobs.Length)
            {
                ref var job = ref jobs[claimedIndex];
                for (int i = job.Start; i < job.End; ++i)
                {
                    ref var ray = ref testRays[i];
                    Simulation.RayCast(ray.Origin, ray.Direction, ray.MaximumT, ref hitHandler, i);
                }
            }
            return intersectionCount;
        }

        IntersectionAlgorithm[] algorithms;

        struct RayJob
        {
            public int Start;
            public int End;
        }
        Buffer<RayJob> jobs;


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
            public bool AllowTest(CollidableReference collidable, int childIndex)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnRayHit(in RayData ray, ref float maximumT, float t, Vector3 normal, CollidableReference collidable, int childIndex)
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

        bool shouldCycle = true;
        bool shouldRotate = true;
        bool shouldUseMultithreading = true;
        int raySourceIndex;
        int frameCount;
        float rotation;
        void CopyAndRotate(ref QuickList<TestRay> source)
        {
            testRays.Count = source.Count;
            var transform = Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), rotation);

            for (int i = 0; i < source.Count; ++i)
            {
                ref var targetRay = ref testRays[i];
                ref var sourceRay = ref source[i];
                Matrix3x3.Transform(sourceRay.Origin, transform, out targetRay.Origin);
                Matrix3x3.Transform(sourceRay.Direction, transform, out targetRay.Direction);
                targetRay.MaximumT = sourceRay.MaximumT;
            }
        }

        public unsafe override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);

            char one = '1';
            for (int i = 0; i < raySources.Count; ++i)
            {
                if (input.TypedCharacters.Contains((char)(one + i)))
                {
                    shouldCycle = false;
                    raySourceIndex = i + 1;
                }
            }
            if (input.WasPushed(OpenTK.Input.Key.C))
            {
                frameCount = 0;
                shouldCycle = !shouldCycle;
            }
            if (input.WasPushed(OpenTK.Input.Key.R))
            {
                shouldRotate = !shouldRotate;
            }
            if (input.WasPushed(OpenTK.Input.Key.F))
            {
                rotation = 0;
            }
            if (input.WasPushed(OpenTK.Input.Key.T))
            {
                shouldUseMultithreading = !shouldUseMultithreading;
            }

            ++frameCount;
            if (frameCount > 1 << 20)
                frameCount = 0;
            if (shouldRotate)
            {
                rotation += (MathF.PI * 1e-2f * (TimestepDuration)) % (2 * MathF.PI);
            }
            if (shouldCycle)
            {
                raySourceIndex = 1 + (frameCount / 256) % raySources.Count;
            }
            CopyAndRotate(ref raySources[raySourceIndex - 1]);

            var raysPerJobBase = testRays.Count / jobs.Length;
            var remainder = testRays.Count - raysPerJobBase * jobs.Length;
            var previousJobEnd = 0;
            for (int i = 0; i < jobs.Length; ++i)
            {
                int raysInJob = i < remainder ? raysPerJobBase + 1 : raysPerJobBase;
                ref var job = ref jobs[i];
                job.Start = previousJobEnd;
                job.End = previousJobEnd = previousJobEnd + raysInJob;
            }


            for (int i = 0; i < algorithms.Length; ++i)
            {
                algorithms[i].Execute(ref testRays, shouldUseMultithreading ? ThreadDispatcher : null);
            }
            for (int i = 1; i < algorithms.Length; ++i)
            {
                Debug.Assert(algorithms[i].IntersectionCount == algorithms[0].IntersectionCount);
                var current = algorithms[i];
                var earlier = algorithms[0];
                for (int j = 0; j < testRays.Count; ++j)
                {
                    ref var currentResult = ref current.Results[j];
                    ref var earlierResult = ref earlier.Results[j];
                    Debug.Assert(currentResult.Hit == earlierResult.Hit && (!earlierResult.Hit || Math.Abs(earlierResult.T - currentResult.T) < 1e-6f));
                }
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
                    var diffuseLight = Vector3.Dot(result.Normal, new Vector3(0.57735f));
                    if (diffuseLight < 0)
                    {
                        diffuseLight = -0.5f * diffuseLight;
                    }
                    renderer.Lines.Allocate() = new LineInstance(ray.Origin, end, Helpers.PackColor(foregroundHitColor * (0.2f + 0.8f * diffuseLight)), packedBackground);
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

        void WriteControl(string name, TextBuilder control, float y, TextBatcher batcher, Font font)
        {
            batcher.Write(control,
                new Vector2(176, y), 16, new Vector3(1), font);
            batcher.Write(control.Clear().Append(name).Append(":"),
                new Vector2(32, y), 16, new Vector3(1), font);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var batchedPackedColor = Helpers.PackColor(new Vector3(0.75f, 0.75f, 0));
            var batchedPackedNormalColor = Helpers.PackColor(new Vector3(1f, 1f, 0));
            var batchedPackedBackgroundColor = Helpers.PackColor(new Vector3());

            DrawRays(ref algorithms[0].Results, renderer, new Vector3(0.25f, 0, 0), new Vector3(0, 1, 0), new Vector3(1, 1, 0), new Vector3());

            text.Clear().Append("Active ray source index: ").Append(raySourceIndex);
            if (shouldCycle)
                text.Append(" (cycling)");
            if (shouldRotate)
                text.Append(" (rotating)");
            if (shouldUseMultithreading)
                text.Append(" (multithreaded)");
            renderer.TextBatcher.Write(text, new Vector2(32, renderer.Surface.Resolution.Y - 192), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Demo specific controls:"), new Vector2(32, renderer.Surface.Resolution.Y - 176), 16, new Vector3(1), font);
            WriteControl("Change ray source", text.Clear().Append("1 through ").Append(raySources.Count), renderer.Surface.Resolution.Y - 160, renderer.TextBatcher, font);
            WriteControl("Toggle cycling", text.Clear().Append("C"), renderer.Surface.Resolution.Y - 144, renderer.TextBatcher, font);
            WriteControl("Toggle rotation", text.Clear().Append("R"), renderer.Surface.Resolution.Y - 128, renderer.TextBatcher, font);
            WriteControl("Reset rotation", text.Clear().Append("F"), renderer.Surface.Resolution.Y - 112, renderer.TextBatcher, font);
            WriteControl("Toggle threading", text.Clear().Append("T"), renderer.Surface.Resolution.Y - 96, renderer.TextBatcher, font);

            renderer.TextBatcher.Write(text.Clear().Append("Ray count: ").Append(testRays.Count), new Vector2(32, renderer.Surface.Resolution.Y - 80), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Time (us):"), new Vector2(128, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Rays per second:"), new Vector2(224, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Relative speed:"), new Vector2(350, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);

            var baseStats = algorithms[0].Timings.ComputeStats();
            var baseHeight = 48;
            for (int i = 0; i < algorithms.Length; ++i)
            {
                var stats = algorithms[i].Timings.ComputeStats();
                WriteResults(algorithms[i].Name, stats.Average, baseStats.Average, renderer.Surface.Resolution.Y - (baseHeight - 16 * i), renderer.TextBatcher, text, font);
            }

            base.Render(renderer, camera, input, text, font);
        }

    }
}
