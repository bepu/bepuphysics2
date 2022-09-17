using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using BepuPhysics.Trees;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Collidables;
using static System.Formats.Asn1.AsnWriter;
using static OpenTK.Graphics.OpenGL.GL;

namespace Demos.SpecializedTests
{
    public class TreeFiddlingTestDemo : Demo
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
            for (int index = 0; index < triangles.Length; ++index)
            {
                ref var a = ref triangles[index];
                ref var b = ref triangles[random.Next(index + 1, triangles.Length)];
                BepuPhysics.Helpers.Swap(ref a, ref b);
            }
            return triangles;
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 3, -10);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));
            var width = 768;
            var height = 768;
            var scale = new Vector3(1, 1, 1);

            //DemoMeshHelper.CreateDeformedPlane(width, height, (x, y) => new Vector3(x - width * scale.X * 0.5f, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - height * scale.Y * 0.5f), scale, BufferPool, out var mesh);
            //DemoMeshHelper.CreateDeformedPlane(width, height, (x, y) => new Vector3(x - width * scale.X * 0.5f, 0, y - height * scale.Y * 0.5f), scale, BufferPool, out var mesh);

            //Create a wiggly mesh.
            var triangles = CreateDeformedPlaneTriangles(width, height, scale);
            var mesh = new Mesh(triangles, Vector3.One, BufferPool);
            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));


            Console.WriteLine($"node count: {mesh.Tree.NodeCount}");

            BufferPool.Take<BoundingBox>(mesh.Triangles.Length, out var leafBounds);
            BufferPool.Take<int>(mesh.Triangles.Length, out var leafIndices);
            for (int i = 0; i < mesh.Triangles.Length; ++i)
            {
                ref var t = ref mesh.Triangles[i];
                ref var bounds = ref leafBounds[i];
                bounds.Min = Vector3.Min(t.A, Vector3.Min(t.B, t.C));
                bounds.Max = Vector3.Max(t.A, Vector3.Max(t.B, t.C));
                leafIndices[i] = i;
            }
            BinnedTest(() =>
            {
                Tree.BinnedBuilder(leafIndices, leafBounds, mesh.Tree.Nodes, BufferPool);
            }, "Revamp", ref mesh.Tree);


            var mesh2 = new Mesh(triangles, Vector3.One, BufferPool);

            QuickList<int> subtreeReferences = new(mesh2.Tree.LeafCount, BufferPool);
            QuickList<int> treeletInternalNodes = new(mesh2.Tree.LeafCount, BufferPool);
            Tree.CreateBinnedResources(BufferPool, mesh2.Tree.LeafCount, out var binnedResourcesBuffer, out var binnedResources);
            BinnedTest(() =>
            {
                subtreeReferences.Count = 0;
                treeletInternalNodes.Count = 0;
                mesh2.Tree.BinnedRefine(0, ref subtreeReferences, mesh2.Tree.LeafCount, ref treeletInternalNodes, ref binnedResources, BufferPool);
            }, "Original", ref mesh.Tree);

            //RefitTest(() => mesh.Tree.Refit2(), "refit2", ref mesh.Tree);
            //RefitTest(() => mesh.Tree.Refit(), "Original", ref mesh.Tree);

            //SelfTest((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlapsContiguousPrepass(ref handler, BufferPool), mesh.Tree.LeafCount, "Prepass");
            //SelfTest((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlaps(ref handler), mesh.Tree.LeafCount, "Original");
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

        static void BinnedTest(Action function, string name, ref Tree tree)
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

            ulong accumulator = 0;
            for (int i = 0; i < 1000; ++i)
            {
                var index = (int)(((ulong)i * 941083987 + accumulator * 797003413) % (ulong)tree.NodeCount);
                var localSum = tree.Nodes[index].A.Min * 5 + tree.Nodes[index].A.Max * 7 + tree.Nodes[index].B.Min * 13 + tree.Nodes[index].B.Max * 17;
                var hash = Unsafe.As<float, int>(ref localSum.X) * 31 + Unsafe.As<float, int>(ref localSum.Y) * 37 + Unsafe.As<float, int>(ref localSum.Z) * 41;
                accumulator = ((accumulator << 7) | (accumulator >> (64 - 7))) + (ulong)hash;
            }
            Console.WriteLine($"{name} bounds hash: {accumulator}, A ({tree.Nodes[0].A.Min}, {tree.Nodes[0].B.Max}), B ({tree.Nodes[0].B.Min}, {tree.Nodes[0].B.Max})");
            var sah = tree.MeasureCostMetric();
            Console.WriteLine($"SAH: {sah}");
        }
    }
}
