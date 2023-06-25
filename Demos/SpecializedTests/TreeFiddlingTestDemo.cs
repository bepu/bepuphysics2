using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Trees;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Collidables;
using BepuUtilities.TaskScheduling;

namespace Demos.SpecializedTests;

public unsafe class TreeFiddlingTestDemo : Demo
{
    struct Pair : IEquatable<Pair>
    {
        public int A;
        public int B;

        public Pair(int a, int b)
        {
            A = a;
            B = b;
        }

        public bool Equals(Pair other)
        {
            return (A == other.A && B == other.B) || (A == other.B && B == other.A);
        }

        public override bool Equals(object obj)
        {
            return obj is Pair pair && Equals(pair);
        }
        public override int GetHashCode()
        {
            return A.GetHashCode() + B.GetHashCode();
        }
        public override string ToString()
        {
            return $"{A}, {B}";
        }
    }
    struct OverlapHandler : IOverlapHandler
    {
        public int OverlapCount;
        public int OverlapSum;
        public int OverlapHash;
        public int TreeLeafCount;

        //public HashSet<Pair> Set;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Handle(int indexA, int indexB)
        {
            //Debug.Assert(indexA >= 0 && indexB >= 0 && indexA < TreeLeafCount && indexB < TreeLeafCount && Set.Add(new Pair(indexA, indexB)));
            ++OverlapCount;
            OverlapSum += indexA + indexB;
            OverlapHash += (indexA + (indexB * OverlapCount)) * OverlapCount;
        }
    }

    Buffer<Triangle> CreateDeformedPlaneTriangles(int width, int height, Vector3 scale)
    {
        Vector3 Deform(int x, int y) => new Vector3(x - width * scale.X * 0.5f, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - height * scale.Y * 0.5f);
        BufferPool.Take<Vector3>(width * height, out var vertices);
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                vertices[width * j + i] = Deform(i, j);
            }
        }

        var quadWidth = width - 1;
        var quadHeight = height - 1;
        var triangleCount = quadWidth * quadHeight * 2;
        BufferPool.Take<Triangle>(triangleCount, out var triangles);

        for (int i = 0; i < quadWidth; ++i)
        {
            for (int j = 0; j < quadHeight; ++j)
            {
                var triangleIndex = (j * quadWidth + i) * 2;
                ref var triangle0 = ref triangles[triangleIndex];
                ref var v00 = ref vertices[width * j + i];
                ref var v01 = ref vertices[width * j + i + 1];
                ref var v10 = ref vertices[width * (j + 1) + i];
                ref var v11 = ref vertices[width * (j + 1) + i + 1];
                triangle0.A = v00;
                triangle0.B = v01;
                triangle0.C = v10;
                ref var triangle1 = ref triangles[triangleIndex + 1];
                triangle1.A = v01;
                triangle1.B = v11;
                triangle1.C = v10;
            }
        }
        BufferPool.Return(ref vertices);
        //Scramble the heck out of its triangles.
        var random = new Random(5);
        for (int index = 0; index < triangles.Length - 1; ++index)
        {
            ref var a = ref triangles[index];
            ref var b = ref triangles[random.Next(index + 1, triangles.Length)];
            BepuPhysics.Helpers.Swap(ref a, ref b);
        }
        return triangles;
    }


    Buffer<Triangle> CreateRandomSoupTriangles(BoundingBox bounds, int triangleCount, float minimumSize, float maximumSize)
    {
        Random random = new Random(5);
        BufferPool.Take<Triangle>(triangleCount, out var triangles);
        for (int i = 0; i < triangleCount; ++i)
        {
            var startPoint = new Vector3(random.NextSingle() * random.NextSingle(), random.NextSingle(), random.NextSingle() * random.NextSingle()) * (bounds.Max - bounds.Min) + bounds.Min;
            var size = new Vector3(MathF.Pow(random.NextSingle(), 200), MathF.Pow(random.NextSingle(), 200), MathF.Pow(random.NextSingle(), 200)) * (maximumSize - minimumSize) + new Vector3(minimumSize);

            ref var triangle = ref triangles[i];
            triangle.A = (2 * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) - Vector3.One) * size;
            triangle.B = (2 * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) - Vector3.One) * size;
            triangle.C = (2 * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) - Vector3.One) * size;

            if (random.NextSingle() < 0.75f)
            {
                var rotation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.0001f) + new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle())), random.NextSingle() * MathF.PI * 2);
                triangle.A = Vector3.Transform(triangle.A, rotation);
                triangle.B = Vector3.Transform(triangle.B, rotation);
                triangle.C = Vector3.Transform(triangle.C, rotation);
            }
            triangle.A += startPoint;
            triangle.B += startPoint;
            triangle.C += startPoint;
        }
        return triangles;
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-10, 3, -10);
        camera.Yaw = MathHelper.Pi * 3f / 4;
        camera.Pitch = 0;
        Tree.Times = new Tree.NodeTimes[1 << 22];
        for (int i = 0; i < 2; ++i)
        {
            BufferPool.Clear();
            ThreadDispatcher.WorkerPools.Clear();
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));

            //Create a mesh.
            var width = 1024;
            var height = 1024;
            var scale = new Vector3(1, 1, 1);
            //DemoMeshHelper.CreateDeformedPlane(width, height, (x, y) => new Vector3(x - width * scale.X * 0.5f, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - height * scale.Y * 0.5f), scale, BufferPool, out var mesh);
            //DemoMeshHelper.CreateDeformedPlane(width, height, (x, y) => new Vector3(x - width * scale.X * 0.5f, 0, y - height * scale.Y * 0.5f), scale, BufferPool, out var mesh);

            //var triangles = CreateDeformedPlaneTriangles(width, height, scale);
            var triangles = CreateRandomSoupTriangles(new BoundingBox(new(width / -2f, scale.Y * -2, height / -2f), new(width / 2f, scale.Y * 2, height / 2f)), (width - 1) * (height - 1) * 2, 0.5f, 100f);
            //var mesh = new Mesh(triangles, Vector3.One, BufferPool);
            var mesh = DemoMeshHelper.CreateGiantMeshFast(triangles, Vector3.One, BufferPool);

            int refinementState = 0;
            long sum = 0;
            int cacheOptimizationStart = 0;
            var taskStack = new TaskStack(BufferPool, ThreadDispatcher, ThreadDispatcher.ThreadCount);
            for (int refinementIndex = 0; refinementIndex < 16384; ++refinementIndex)
            {
                //mesh.Tree.CacheOptimizeLimitedSubtree(0, 4096);
                //mesh.Tree.CacheOptimize(0);
                //var optimizedCount = mesh.Tree.CacheOptimizeRegion(0, int.MaxValue);
                //int localOptimizationCount = 0;
                //const int targetOptimizationCount = 8192;
                //while (localOptimizationCount < targetOptimizationCount)
                //{
                //    var optimizedCount = mesh.Tree.CacheOptimizeRegion(cacheOptimizationStart, targetOptimizationCount);
                //    localOptimizationCount += optimizedCount;
                //    cacheOptimizationStart += optimizedCount;
                //    if (cacheOptimizationStart >= mesh.Tree.NodeCount)
                //        cacheOptimizationStart -= mesh.Tree.NodeCount;
                //}

                var start = Stopwatch.GetTimestamp();
                //mesh.Tree.Refine2(8192, ref refinementState, 0, 8192, BufferPool);
                //mesh.Tree.Refine2(8192, ref refinementState, 32, 1024, BufferPool);
                mesh.Tree.Refine2(8192, ref refinementState, 32, 4096, BufferPool, ThreadDispatcher);
                var end = Stopwatch.GetTimestamp();
                sum += end - start;
                if ((refinementIndex + 1) % 128 == 0)
                {
                    var cacheQuality = mesh.Tree.MeasureCacheQuality();
                    var costMetric = mesh.Tree.MeasureCostMetric();
                    Console.WriteLine($"cost, cache for {refinementIndex}: {costMetric}, {cacheQuality}");
                    Console.WriteLine($"Time (average) (ms): {(end - start) * 1e3 / Stopwatch.Frequency}, {sum * 1e3 / ((refinementIndex + 1) * Stopwatch.Frequency)}");
                }
            }
            taskStack.Dispose(BufferPool, ThreadDispatcher);
            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));

            Console.WriteLine($"node count: {mesh.Tree.NodeCount}");
            Console.WriteLine($"initial SAH: {mesh.Tree.MeasureCostMetric()}, cache quality: {mesh.Tree.MeasureCacheQuality()}");
            Console.WriteLine($"initial bounds: A ({mesh.Tree.Nodes[0].A.Min}, {mesh.Tree.Nodes[0].B.Max}), B ({mesh.Tree.Nodes[0].B.Min}, {mesh.Tree.Nodes[0].B.Max})");

            //BufferPool.Take<NodeChild>(mesh.Triangles.Length, out var subtrees);

            //Action setup = () =>
            //{
            //    for (int i = 0; i < mesh.Triangles.Length; ++i)
            //    {
            //        ref var t = ref mesh.Triangles[i];
            //        ref var subtree = ref subtrees[i];
            //        subtree.Min = Vector3.Min(t.A, Vector3.Min(t.B, t.C));
            //        subtree.Max = Vector3.Max(t.A, Vector3.Max(t.B, t.C));
            //        subtree.Index = Tree.Encode(i);
            //        subtree.LeafCount = 1;
            //    }
            //};

            //BinnedTest(setup, () =>
            //{
            //    mesh.Tree.BinnedBuild(subtrees, ThreadDispatcher, pool: BufferPool);
            //}, "Revamp Single Axis MT", ref mesh.Tree);

            //BufferPool.Take<BoundingBox>(mesh.Triangles.Length, out var leafBounds);
            //BufferPool.Take<int>(mesh.Triangles.Length, out var leafIndices);

            //Action yeOldeSetup = () =>
            //{
            //    for (int i = 0; i < mesh.Triangles.Length; ++i)
            //    {
            //        ref var t = ref mesh.Triangles[i];
            //        ref var bounds = ref leafBounds[i];
            //        bounds.Min = Vector3.Min(t.A, Vector3.Min(t.B, t.C));
            //        bounds.Max = Vector3.Max(t.A, Vector3.Max(t.B, t.C));
            //        leafIndices[i] = Tree.Encode(i);
            //    }
            //};
            //BinnedTest(yeOldeSetup, () =>
            //{
            //    Tree.BinnedBuilder(leafIndices, leafBounds, mesh.Tree.Nodes, mesh.Tree.Metanodes, mesh.Tree.Leaves);
            //}, "Revamp Single Axis ST", ref mesh.Tree);



            //Mesh mesh2 = default;
            //Mesh* mesh2Pointer = &mesh2;

            //QuickList<int> subtreeReferences = new(triangles.Length, BufferPool);
            //QuickList<int> treeletInternalNodes = new(triangles.Length, BufferPool);
            //Tree.CreateBinnedResources(BufferPool, triangles.Length, out var binnedResourcesBuffer, out var binnedResources);
            //BinnedTest(() =>
            //{
            //    if (mesh2Pointer->Tree.Leaves.Allocated)
            //        mesh2Pointer->Tree.Dispose(BufferPool);
            //    *mesh2Pointer = DemoMeshHelper.CreateGiantMeshFast(triangles, Vector3.One, BufferPool);
            //}, () =>
            //{
            //    subtreeReferences.Count = 0;
            //    treeletInternalNodes.Count = 0;
            //    mesh2Pointer->Tree.BinnedRefine(0, ref subtreeReferences, mesh2Pointer->Tree.LeafCount, ref treeletInternalNodes, ref binnedResources, BufferPool);
            //}, "Original", ref mesh2Pointer->Tree);

            //RefitTest(() => mesh.Tree.Refit(), "Refit", ref mesh.Tree);

            //SelfTest((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlapsContiguousPrepass(ref handler, BufferPool), mesh.Tree.LeafCount, "Prepass");
            //SelfTest((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlaps(ref handler), mesh.Tree.LeafCount, "Original");

            Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 10, 0), 1, Simulation.Shapes, new Sphere(0.5f)));
        }
    }

    delegate void TestFunction(ref OverlapHandler handler);

    static void SelfTest(TestFunction function, int leafCount, string name)
    {
        var overlapHandler = new OverlapHandler();
        overlapHandler.TreeLeafCount = leafCount;
        //overlapHandler.Set = new HashSet<Pair>();
        long accumulatedTime = 0;
        const int testCount = 16;
        for (int i = 0; i < testCount; ++i)
        {
            var startTime = Stopwatch.GetTimestamp();
            function(ref overlapHandler);
            var endTime = Stopwatch.GetTimestamp();
            accumulatedTime += endTime - startTime;
            //overlapHandler.Set.Clear();
            CacheBlaster.Blast();
        }
        Console.WriteLine($"{name} time per execution (ms): {(accumulatedTime) * 1e3 / (testCount * Stopwatch.Frequency)}");
        Console.WriteLine($"{name} count: {overlapHandler.OverlapCount}, sum {overlapHandler.OverlapSum}, hash {overlapHandler.OverlapHash}");
    }


    static void RefitTest(Action function, string name, ref Tree tree)
    {
        long accumulatedTime = 0;
        const int testCount = 16;
        for (int i = 0; i < testCount; ++i)
        {
            var startTime = Stopwatch.GetTimestamp();
            function();
            var endTime = Stopwatch.GetTimestamp();
            accumulatedTime += endTime - startTime;
            //overlapHandler.Set.Clear();
            CacheBlaster.Blast();
        }
        Console.WriteLine($"{name} time per execution (ms): {(accumulatedTime) * 1e3 / (testCount * Stopwatch.Frequency)}");

        var sum = tree.Nodes[0].A.Min * 5 + tree.Nodes[0].A.Max * 7 + tree.Nodes[0].B.Min * 13 + tree.Nodes[0].B.Max * 17;
        var hash = Unsafe.As<float, int>(ref sum.X) * 31 + Unsafe.As<float, int>(ref sum.Y) * 37 + Unsafe.As<float, int>(ref sum.Z) * 41;
        Console.WriteLine($"{name} bounds 0 hash: {hash}, A ({tree.Nodes[0].A.Min}, {tree.Nodes[0].B.Max}), B ({tree.Nodes[0].B.Min}, {tree.Nodes[0].B.Max})");
    }

    static void BinnedTest(Action setup, Action function, string name, ref Tree tree)
    {
        long accumulatedTime = 0;
        const int testCount = 64;
        for (int i = 0; i < testCount; ++i)
        {
            setup?.Invoke();
            var startTime = Stopwatch.GetTimestamp();
            function();
            var endTime = Stopwatch.GetTimestamp();
            accumulatedTime += endTime - startTime;
            //overlapHandler.Set.Clear();
            CacheBlaster.Blast();
        }
        Console.WriteLine($"{name} time per execution (ms): {(accumulatedTime) * 1e3 / (testCount * Stopwatch.Frequency)}");

        ulong accumulator = 0;
        for (int i = 0; i < 1000; ++i)
        {
            var index = (int)(((ulong)i * 941083987 + accumulator * 797003413) % (ulong)tree.NodeCount);
            var localSum = tree.Nodes[index].A.Min * 5 + tree.Nodes[index].A.Max * 7 + tree.Nodes[index].B.Min * 13 + tree.Nodes[index].B.Max * 17;
            var hash = Unsafe.As<float, int>(ref localSum.X) * 31 + Unsafe.As<float, int>(ref localSum.Y) * 37 + Unsafe.As<float, int>(ref localSum.Z) * 41;
            accumulator = ((accumulator << 7) | (accumulator >> (64 - 7))) + (ulong)hash;
        }
        Console.WriteLine($"{name} bounds hash: {accumulator}, A ({tree.Nodes[0].A.Min}, {tree.Nodes[0].B.Max}), B ({tree.Nodes[0].B.Min}, {tree.Nodes[0].B.Max})");
        Console.WriteLine($"SAH: {tree.MeasureCostMetric()}, cache quality: {tree.MeasureCacheQuality()}");
    }
}
