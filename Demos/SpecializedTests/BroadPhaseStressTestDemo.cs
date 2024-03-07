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
using BepuPhysics.Trees;
using static Demos.SpecializedTests.TreeFiddlingTestDemo;

namespace Demos.SpecializedTests;

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
        const int width = 2048;
        const int height = 2;
        const int length = 2048;
        var spacing = new Vector3(16.01f);
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
                    var location = (r - new Vector3(0.5f)) * (r - new Vector3(0.5f)) * spacing * new Vector3(width, height, length);
                    //var location = (r - new Vector3(0.5f)) * spacing * new Vector3(width, height, length);
                    //var hash = HashHelper.Rehash(HashHelper.Rehash(HashHelper.Rehash(i) + HashHelper.Rehash(j)) + HashHelper.Rehash(k));
                    var hash = i + j + k;
                    if (hash % 64 == 0)
                    {
                        if (i == 7 && j == 1 && k == 0)
                            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(100, 0, 100), sphereInertia, Simulation.Shapes.Add(new Sphere(100)), -1));
                        else
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
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10, 0), Simulation.Shapes.Add(new Box(5000, 1, 5000))));
        startingLocations = new Vector3[Simulation.Bodies.ActiveSet.Count];
        for (int i = 0; i < startingLocations.Length; ++i)
        {
            startingLocations[i] = Simulation.Bodies.ActiveSet.DynamicsState[i].Motion.Pose.Position;
        }
        updateTimes = new TimingsRingBuffer(sampleCount, BufferPool);
        testTimes = new TimingsRingBuffer(sampleCount, BufferPool);
        test2Times = new TimingsRingBuffer(sampleCount, BufferPool);
        intertreeTest2Times = new TimingsRingBuffer(sampleCount, BufferPool);
    }

    const int sampleCount = 128;
    TimingsRingBuffer updateTimes;
    TimingsRingBuffer testTimes;
    TimingsRingBuffer test2Times;
    TimingsRingBuffer intertreeTest2Times;
    long frameCount;
    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        var rotationAngle = frameCount * 1e-3f;
        var rotation = Matrix3x3.CreateFromAxisAngle(Vector3.UnitY, rotationAngle);
        //for (int i = 0; i < Simulation.Bodies.ActiveSet.Count / 2; ++i)
        //{
        //    //For every body, set the velocity such that body moves toward some body-specific goal state that evolves over time.
        //    Matrix3x3.Transform(startingLocations[i], rotation, out var targetLocation);
        //    ref var motion = ref Simulation.Bodies.ActiveSet.DynamicsState[i].Motion;
        //    var offset = targetLocation - motion.Pose.Position;
        //    motion.Velocity.Linear = offset;
        //}

        //Simulation.BroadPhase.ActiveTree.CacheOptimize(0);
        //Simulation.BroadPhase.StaticTree.CacheOptimize(0);
        base.Update(window, camera, input, dt);
        updateTimes.Add(Simulation.Profiler[Simulation.BroadPhase]);
        testTimes.Add(Simulation.Profiler[Simulation.BroadPhaseOverlapFinder]);

        //var overlaps = new OverlapHandler();
        //Simulation.BroadPhase.ActiveTree.GetSelfOverlaps2(ref overlaps);
        //var a = Stopwatch.GetTimestamp();
        //var threadedOverlaps = new TreeFiddlingTestDemo.ThreadedOverlapHandler(BufferPool, ThreadDispatcher.ThreadCount);
        //Simulation.BroadPhase.ActiveTree.GetSelfOverlaps2(ref threadedOverlaps, BufferPool, ThreadDispatcher);
        //var (selfOverlapCount, _) = threadedOverlaps.SumResults();
        //threadedOverlaps.Reset();
        //var b = Stopwatch.GetTimestamp();
        //Simulation.BroadPhase.ActiveTree.GetOverlaps2(ref Simulation.BroadPhase.ActiveTree, ref threadedOverlaps, BufferPool, ThreadDispatcher);
        //var c = Stopwatch.GetTimestamp();
        //var interOverlaps = new OverlapHandler();
        ////Simulation.BroadPhase.ActiveTree.GetOverlaps(ref Simulation.BroadPhase.ActiveTree, ref interOverlaps);
        //var (interOverlapCount, _) = threadedOverlaps.SumResults();
        //test2Times.Add((b - a) / (double)Stopwatch.Frequency);
        //intertreeTest2Times.Add((c - b) / (double)Stopwatch.Frequency);


        if (frameCount++ % sampleCount == 0)
        {
            var updateStats = updateTimes.ComputeStats();
            var testStats = testTimes.ComputeStats();
            var test2Stats = test2Times.ComputeStats();
            var intertreeTest2Stats = intertreeTest2Times.ComputeStats();
            Console.WriteLine($"Update: {updateStats.Average * 1000} ms average, {updateStats.StdDev * 1000} stddev");
            Console.WriteLine($"Test:   {testStats.Average * 1000} ms average, {testStats.StdDev * 1000} stddev");
            //Console.WriteLine($"Test2:  {test2Stats.Average * 1000} ms average, {test2Stats.StdDev * 1000} stddev");
            //Console.WriteLine($"Inter2: {intertreeTest2Stats.Average * 1000} ms average, {intertreeTest2Stats.StdDev * 1000} stddev");
            Console.WriteLine($"Active Cost: {Simulation.BroadPhase.ActiveTree.MeasureCostMetric()}");
            Console.WriteLine($"Static Cost: {Simulation.BroadPhase.StaticTree.MeasureCostMetric()}");
            //Console.WriteLine($"Active CQ:   {Simulation.BroadPhase.ActiveTree.MeasureCacheQuality()}");
            //Console.WriteLine($"Static CQ:   {Simulation.BroadPhase.StaticTree.MeasureCacheQuality()}");

            //var min = int.MaxValue;
            //var max = 0;
            //for (int i = 0; i < threadedOverlaps.Workers.Length; ++i)
            //{
            //    var count = threadedOverlaps.Workers[i].OverlapCount;
            //    //Console.Write($"{count}, ");
            //    min = int.Min(count, min);
            //    max = int.Max(count, max);
            //}
            //Console.WriteLine($"min, max: {min}, {max}");

        }
        //threadedOverlaps.Dispose(BufferPool);
    }

}
