using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Trees;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.Constraints;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;
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
                    //var location = new Vector3(15, 15, 15);
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
        groundStatic = Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10, 0), Simulation.Shapes.Add(new Box(5000, 1, 5000))));
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
    StaticHandle groundStatic;

    const int sampleCount = 128;
    TimingsRingBuffer updateTimes;
    TimingsRingBuffer testTimes;
    TimingsRingBuffer test2Times;
    TimingsRingBuffer intertreeTest2Times;
    long frameCount;

    void PrintPathToRoot(StaticHandle handle)
    {
        var index = Simulation.Statics[handle].Static.BroadPhaseIndex;
        var leaf = Simulation.BroadPhase.StaticTree.Leaves[index];
        int depth = 0;
        var nodeIndex = leaf.NodeIndex;
        Console.Write($"Starting from {leaf.NodeIndex}:{leaf.ChildIndex}, path: ");
        while (true)
        {
            ref var node = ref Simulation.BroadPhase.StaticTree.Metanodes[nodeIndex];
            Console.Write($"{nodeIndex}, ");
            if (node.Parent >= 0)
            {
                nodeIndex = node.Parent;
                depth++;
            }
            else
                break;
        }
        Console.WriteLine($"; depth {depth}.");

    }

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

        //if (frameCount % 32 == 0)
        //PrintPathToRoot(groundStatic);

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
            Console.WriteLine($"Active Depth {frameCount}: {Simulation.BroadPhase.ActiveTree.ComputeMaximumDepth()}");
            Console.WriteLine($"Static Depth {frameCount}: {Simulation.BroadPhase.StaticTree.ComputeMaximumDepth()}");
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

    // Claude did an okay job of this visualization, I'd say.
    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        base.Render(renderer, camera, input, text, font);

        //VisualizeTreeTopology(renderer, camera, text, font, ref Simulation.BroadPhase.StaticTree);
    }

    void VisualizeTreeTopology(Renderer renderer, Camera camera, TextBuilder text, Font font, ref Tree tree)
    {
        if (tree.LeafCount > 0)
        {
            const float levelHeight = 3f;
            const float nodeSpacing = 20f;

            // Start visualization from the root (node 0) at origin
            RenderNodeRecursive(renderer, camera, text, font, ref tree, 0, 0, 0f, levelHeight, nodeSpacing);
        }
    }

    void RenderNodeRecursive(Renderer renderer, Camera camera, TextBuilder text, Font font, ref Tree tree, int nodeIndex, int depth, float horizontalOffset, float levelHeight, float nodeSpacing)
    {
        var nodePosition = new Vector3(horizontalOffset, depth * levelHeight, 0);

        var nodeColor = new Vector3(0.8f, 0.2f, 0.2f);
        renderer.Shapes.AddShape(new Sphere(0.3f), null, nodePosition, nodeColor);

        if (DemoRenderer.Helpers.GetScreenLocation(nodePosition, camera.ViewProjection, renderer.Surface.Resolution, out var location))
        {
            renderer.TextBatcher.Write(text.Clear().Append(nodeIndex), location, 10, new Vector3(1), font);
        }

        ref var node = ref tree.Nodes[nodeIndex];

        // Calculate child spacing (gets tighter with depth)
        float childSpacing = nodeSpacing / float.Pow(1.7f, depth);
        float leftOffset = horizontalOffset - childSpacing;
        float rightOffset = horizontalOffset + childSpacing;

        // Process child A (left)
        var childAPosition = new Vector3(leftOffset, (depth + 1) * levelHeight, 0);
        if (node.A.Index >= 0)
        {
            // Internal node - recurse
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childAPosition, new Vector3(0.8f, 0.8f, 0.8f), default);
            RenderNodeRecursive(renderer, camera, text, font, ref tree, node.A.Index, depth + 1, leftOffset, levelHeight, nodeSpacing);
        }
        else
        {
            // Leaf node
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childAPosition, new Vector3(0.2f, 0.8f, 0.2f), default);
            renderer.Shapes.AddShape(new Sphere(0.15f), null, childAPosition, new Vector3(0.2f, 1f, 0.2f));
            if (DemoRenderer.Helpers.GetScreenLocation(childAPosition, camera.ViewProjection, renderer.Surface.Resolution, out var childLocation))
                renderer.TextBatcher.Write(text.Clear().Append(Tree.Encode(node.A.Index)), childLocation, 10, new Vector3(1), font);
        }

        // Process child B (right)
        var childBPosition = new Vector3(rightOffset, (depth + 1) * levelHeight, 0);
        if (node.B.Index >= 0)
        {
            // Internal node - recurse
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childBPosition, new Vector3(0.8f, 0.8f, 0.8f), default);
            RenderNodeRecursive(renderer, camera, text, font, ref tree, node.B.Index, depth + 1, rightOffset, levelHeight, nodeSpacing);
        }
        else
        {
            // Leaf node
            renderer.Lines.Allocate() = new LineInstance(nodePosition, childBPosition, new Vector3(0.2f, 0.8f, 0.2f), default);
            renderer.Shapes.AddShape(new Sphere(0.15f), null, childBPosition, new Vector3(0.2f, 1f, 0.2f));
            if (DemoRenderer.Helpers.GetScreenLocation(childBPosition, camera.ViewProjection, renderer.Surface.Resolution, out var childLocation))
                renderer.TextBatcher.Write(text.Clear().Append(Tree.Encode(node.B.Index)), childLocation, 10, new Vector3(1), font);
        }
    }

}
