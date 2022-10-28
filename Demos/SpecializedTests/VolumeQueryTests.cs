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
using System.Threading;
using Demos.SpecializedTests;
using DemoContentLoader;

namespace Demos.SpecializedTests
{
    public class VolumeQueryTests : Demo
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
            Simulation = Simulation.Create(BufferPool, new NoCollisionCallbacks(), new DemoPoseIntegratorCallbacks(), new SolveDescription(8, 1));

            var sphere = new Sphere(0.5f);
            var shapeIndex = Simulation.Shapes.Add(sphere);
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
                        var r = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
                        var location = spacing * (new Vector3(i, j, k) + new Vector3(-width, -height, -length) * 0.5f) + randomizationBase + r * randomizationSpan;

                        Quaternion orientation;
                        orientation.X = -1 + 2 * random.NextSingle();
                        orientation.Y = -1 + 2 * random.NextSingle();
                        orientation.Z = -1 + 2 * random.NextSingle();
                        orientation.W = 0.01f + random.NextSingle();
                        QuaternionEx.Normalize(ref orientation);

                        if ((i + j + k) % 2 == 1)
                        {
                            var bodyDescription = BodyDescription.CreateKinematic((location, orientation), shapeIndex, -1);
                            Simulation.Bodies.Add(bodyDescription);
                        }
                        else
                        {
                            var staticDescription = new StaticDescription(location, orientation, shapeIndex);
                            Simulation.Statics.Add(staticDescription);
                        }

                    }
                }
            }


            int boxCount = 16384;
            var randomMin = new Vector3(width, height, length) * spacing * -0.5f;
            var randomSpan = randomMin * -2;
            queryBoxes = new QuickList<BoundingBox>(boxCount, BufferPool);
            for (int i = 0; i < boxCount; ++i)
            {
                ref var box = ref queryBoxes.AllocateUnsafely();
                var r = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
                var boxOrigin = randomMin + r * randomSpan;
                var boxHalfSize = new Vector3(0.25f + 0.75f * random.NextSingle());
                box.Min = boxOrigin - boxHalfSize;
                box.Max = boxOrigin + boxHalfSize;
            }

            algorithms = new BoxQueryAlgorithm[1];
            algorithms[0] = new BoxQueryAlgorithm("1", BufferPool, Worker1);

            BufferPool.Take(Environment.ProcessorCount * 2, out jobs);
        }


        QuickList<BoundingBox> queryBoxes;


        unsafe class BoxQueryAlgorithm
        {
            public string Name;
            public int IntersectionCount;
            public TimingsRingBuffer Timings;

            Func<int, BoxQueryAlgorithm, int> worker;
            ThreadDispatcherWorker internalWorker;
            public int JobIndex;

            public BoxQueryAlgorithm(string name, BufferPool pool, Func<int, BoxQueryAlgorithm, int> worker, int timingSampleCount = 16)
            {
                Name = name;
                Timings = new TimingsRingBuffer(timingSampleCount, pool);
                this.worker = worker;
                internalWorker = ExecuteWorker;
            }

            unsafe void ExecuteWorker(int workerIndex, void* context)
            {
                var intersectionCount = worker(workerIndex, this);
                Interlocked.Add(ref IntersectionCount, intersectionCount);
            }

            public void Execute(ref QuickList<BoundingBox> boxes, IThreadDispatcher dispatcher)
            {
                CacheBlaster.Blast();
                JobIndex = -1;
                IntersectionCount = 0;
                var start = Stopwatch.GetTimestamp();
                if (dispatcher != null)
                {
                    dispatcher.DispatchWorkers(internalWorker);
                }
                else
                {
                    internalWorker(0, null);
                }
                var stop = Stopwatch.GetTimestamp();
                Timings.Add((stop - start) / (double)Stopwatch.Frequency);
            }
        }



        unsafe int Worker1(int workerIndex, BoxQueryAlgorithm algorithm)
        {
            int intersectionCount = 0;
            var hitHandler = new HitHandler { IntersectionCount = &intersectionCount };
            int claimedIndex;
            while ((claimedIndex = Interlocked.Increment(ref algorithm.JobIndex)) < jobs.Length)
            {
                ref var job = ref jobs[claimedIndex];
                for (int i = job.Start; i < job.End; ++i)
                {
                    ref var box = ref queryBoxes[i];
                    Simulation.BroadPhase.GetOverlaps(box, ref hitHandler);
                }
            }
            return intersectionCount;
        }


        BoxQueryAlgorithm[] algorithms;

        struct QueryJob
        {
            public int Start;
            public int End;
        }
        Buffer<QueryJob> jobs;


        unsafe struct HitHandler : IBreakableForEach<CollidableReference>
        {
            public int* IntersectionCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool LoopBody(CollidableReference collidable)
            {
                ++*IntersectionCount;
                return true;
            }
        }

        bool shouldUseMultithreading = true;

        public unsafe override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);

            if (input.WasPushed(OpenTK.Input.Key.T))
            {
                shouldUseMultithreading = !shouldUseMultithreading;
            }

            var raysPerJobBase = queryBoxes.Count / jobs.Length;
            var remainder = queryBoxes.Count - raysPerJobBase * jobs.Length;
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
                algorithms[i].Execute(ref queryBoxes, shouldUseMultithreading ? ThreadDispatcher : null);
            }
            for (int i = 1; i < algorithms.Length; ++i)
            {
                Debug.Assert(algorithms[i].IntersectionCount == algorithms[0].IntersectionCount);
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
                text.Clear().Append(queryBoxes.Count / time, 0),
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
            text.Clear().Append("Multithreading: ").Append(shouldUseMultithreading ? "On" : "Off");
            renderer.TextBatcher.Write(text, new Vector2(32, renderer.Surface.Resolution.Y - 128), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Demo specific controls:"), new Vector2(32, renderer.Surface.Resolution.Y - 112), 16, new Vector3(1), font);
            WriteControl("Toggle threading", text.Clear().Append("T"), renderer.Surface.Resolution.Y - 96, renderer.TextBatcher, font);

            renderer.TextBatcher.Write(text.Clear().Append("Box count: ").Append(queryBoxes.Count), new Vector2(32, renderer.Surface.Resolution.Y - 80), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Time (us):"), new Vector2(128, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Boxes per second:"), new Vector2(224, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);
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
