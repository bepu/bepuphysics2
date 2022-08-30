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
                OverlapHash = indexA * OverlapCount ^ indexB * OverlapCount * OverlapCount;
            }
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 3, -10);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));
            var width = 128;
            var height = 128;
            var scale = new Vector3(1, 1, 1);
            DemoMeshHelper.CreateDeformedPlane(width, height, (x, y) => new Vector3(x - width * scale.X * 0.5f, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - height * scale.Y * 0.5f), scale, BufferPool, out var mesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));

            int testCount = 1000;
            var overlapHandler0 = new OverlapHandler();
            var startTime0 = Stopwatch.GetTimestamp();
            for (int i = 0; i < testCount; ++i)
                mesh.Tree.GetSelfOverlaps(ref overlapHandler0);
            var endTime0 = Stopwatch.GetTimestamp();
            var overlapHandler1 = new OverlapHandler();
            var startTime1 = Stopwatch.GetTimestamp();
            for (int i = 0; i < testCount; ++i)
                mesh.Tree.GetSelfOverlaps2(ref overlapHandler1, BufferPool);
            var endTime1 = Stopwatch.GetTimestamp();

            Console.WriteLine($"Original time per execution (ms): {(endTime0 - startTime0) * 1e3 / Stopwatch.Frequency}");
            Console.WriteLine($"Revamped time per execution (ms): {(endTime1 - startTime1) * 1e3 / Stopwatch.Frequency}");

            Console.WriteLine($"Original count: {overlapHandler0.OverlapCount}, sum {overlapHandler0.OverlapSum}, hash {overlapHandler0.OverlapHash}");
            Console.WriteLine($"Revamped count: {overlapHandler1.OverlapCount}, sum {overlapHandler1.OverlapSum}, hash {overlapHandler1.OverlapHash}");
        }


    }
}
