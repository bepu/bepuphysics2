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

namespace Demos.SpecializedTests
{
    public class TreeFiddlingTestDemo : Demo
    {
        struct OverlapHandler : IOverlapHandler
        {
            public int OverlapCount;
            public int OverlapSum;
            public int OverlapHash;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Handle(int indexA, int indexB)
            {
                ++OverlapCount;
                OverlapSum += indexA + indexB;
                OverlapHash += (indexA + (indexB * OverlapCount)) * OverlapCount;
            }
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
            DemoMeshHelper.CreateDeformedPlane(width, height, (x, y) => new Vector3(x - width * scale.X * 0.5f, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - height * scale.Y * 0.5f), scale, BufferPool, out var mesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));

            Test((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlapsPrepassWithRecursion(ref handler, BufferPool), "LRecurse");
            //Test((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlapsContiguousPrepass(ref handler, BufferPool), "Prepass");
            //Test((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlaps2(ref handler, BufferPool), "Revamp 2");
            //Test((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlaps3(ref handler, BufferPool), "Revamp 3");
            Test((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlaps4(ref handler), "Revamp 4");
            //Test((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlaps5(ref handler, BufferPool), "Revamp 5");
            Test((ref OverlapHandler handler) => mesh.Tree.GetSelfOverlaps(ref handler), "Original");
        }

        delegate void TestFunction(ref OverlapHandler handler);

        static void Test(TestFunction function, string name)
        {
            var overlapHandler = new OverlapHandler();
            long accumulatedTime = 0;
            const int testCount = 16;
            for (int i = 0; i < testCount; ++i)
            {
                var startTime = Stopwatch.GetTimestamp(); 
                function(ref overlapHandler);
                var endTime = Stopwatch.GetTimestamp();
                accumulatedTime += endTime - startTime;
                CacheBlaster.Blast();
            }
            Console.WriteLine($"{name} time per execution (ms): {(accumulatedTime) * 1e3 / (testCount * Stopwatch.Frequency)}");
            Console.WriteLine($"{name} count: {overlapHandler.OverlapCount}, sum {overlapHandler.OverlapSum}, hash {overlapHandler.OverlapHash}");
        }
    }
}
