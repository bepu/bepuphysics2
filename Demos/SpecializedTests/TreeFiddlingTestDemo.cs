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
                OverlapHash += (indexA + (indexB * OverlapCount)) * OverlapCount;
            }
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 3, -10);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));
            var width = 512;
            var height = 512;
            var scale = new Vector3(1, 1, 1);
            DemoMeshHelper.CreateDeformedPlane(width, height, (x, y) => new Vector3(x - width * scale.X * 0.5f, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - height * scale.Y * 0.5f), scale, BufferPool, out var mesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));

            int testCount = 100;
            //var overlapHandlerLoopWithRecursion = new OverlapHandler();
            //var startTimeLoopRecursion = Stopwatch.GetTimestamp();
            //for (int i = 0; i < testCount; ++i)
            //    mesh.Tree.GetSelfOverlapsPrepassWithRecursion(ref overlapHandlerLoopWithRecursion, BufferPool);
            //var endTimeLoopRecursion = Stopwatch.GetTimestamp();
            //Console.WriteLine($"LRecurse time per execution (ms): {(endTimeLoopRecursion - startTimeLoopRecursion) * 1e3 / (testCount * Stopwatch.Frequency)}");
            //Console.WriteLine($"LRecurse count: {overlapHandlerLoopWithRecursion.OverlapCount}, sum {overlapHandlerLoopWithRecursion.OverlapSum}, hash {overlapHandlerLoopWithRecursion.OverlapHash}");

            //var overlapHandlerPre = new OverlapHandler();
            //var startTimePre = Stopwatch.GetTimestamp();
            //for (int i = 0; i < testCount; ++i)
            //    mesh.Tree.GetSelfOverlapsContiguousPrepass(ref overlapHandlerPre, BufferPool);
            //var endTimePre = Stopwatch.GetTimestamp();
            //Console.WriteLine($"CPrepass time per execution (ms): {(endTimePre - startTimePre) * 1e3 / (testCount * Stopwatch.Frequency)}");
            //Console.WriteLine($"CPrepass count: {overlapHandlerPre.OverlapCount}, sum {overlapHandlerPre.OverlapSum}, hash {overlapHandlerPre.OverlapHash}");

            //var overlapHandler2 = new OverlapHandler();
            //var startTime2 = Stopwatch.GetTimestamp();
            //for (int i = 0; i < testCount; ++i)
            //    mesh.Tree.GetSelfOverlaps2(ref overlapHandler2, BufferPool);
            //var endTime2 = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Revamp 2 time per execution (ms): {(endTime2 - startTime2) * 1e3 / (testCount * Stopwatch.Frequency)}");
            //Console.WriteLine($"Revamp 2 count: {overlapHandler2.OverlapCount}, sum {overlapHandler2.OverlapSum}, hash {overlapHandler2.OverlapHash}");

            //var overlapHandler3 = new OverlapHandler();
            //var startTime3 = Stopwatch.GetTimestamp();
            //for (int i = 0; i < testCount; ++i)
            //    mesh.Tree.GetSelfOverlaps3(ref overlapHandler3, BufferPool);
            //var endTime3 = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Revamp 3 time per execution (ms): {(endTime3 - startTime3) * 1e3 / (testCount * Stopwatch.Frequency)}");
            //Console.WriteLine($"Revamp 3 count: {overlapHandler3.OverlapCount}, sum {overlapHandler3.OverlapSum}, hash {overlapHandler3.OverlapHash}");

            var overlapHandler4 = new OverlapHandler();
            var startTime4 = Stopwatch.GetTimestamp();
            for (int i = 0; i < testCount; ++i)
                mesh.Tree.GetSelfOverlaps4(ref overlapHandler4);
            var endTime4 = Stopwatch.GetTimestamp();
            Console.WriteLine($"Revamp 4 time per execution (ms): {(endTime4 - startTime4) * 1e3 / (testCount * Stopwatch.Frequency)}");
            Console.WriteLine($"Revamp 4 count: {overlapHandler4.OverlapCount}, sum {overlapHandler4.OverlapSum}, hash {overlapHandler4.OverlapHash}");

            var overlapHandler5 = new OverlapHandler();
            var startTime5 = Stopwatch.GetTimestamp();
            for (int i = 0; i < testCount; ++i)
                mesh.Tree.GetSelfOverlaps5(ref overlapHandler5, BufferPool);
            var endTime5 = Stopwatch.GetTimestamp();
            Console.WriteLine($"Revamp 5 time per execution (ms): {(endTime5 - startTime5) * 1e3 / (testCount * Stopwatch.Frequency)}");
            Console.WriteLine($"Revamp 5 count: {overlapHandler5.OverlapCount}, sum {overlapHandler5.OverlapSum}, hash {overlapHandler5.OverlapHash}");

            var overlapHandlerOld = new OverlapHandler();
            var startTimeOld = Stopwatch.GetTimestamp();
            for (int i = 0; i < testCount; ++i)
                mesh.Tree.GetSelfOverlaps(ref overlapHandlerOld);
            var endTimeOld = Stopwatch.GetTimestamp();
            Console.WriteLine($"Original time per execution (ms): {(endTimeOld - startTimeOld) * 1e3 / (testCount * Stopwatch.Frequency)}");
            Console.WriteLine($"Original count: {overlapHandlerOld.OverlapCount}, sum {overlapHandlerOld.OverlapSum}, hash {overlapHandlerOld.OverlapHash}");



        }


    }
}
