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
using System.Xml.Linq;
using System.Threading;

namespace Demos.SpecializedTests
{
    public unsafe class TaskQueueTestDemo : Demo
    {
        static void Test(int taskId, void* context, int workerIndex)
        {

        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 3, -10);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));

            var taskQueue = new TaskQueue(BufferPool);

            taskQueue.For(&Test, null, 0, 100, 0);

            taskQueue.Dispose(BufferPool);

        }

        delegate int TestFunction();

        static void Test(TestFunction function, string name)
        {
            long accumulatedTime = 0;
            const int testCount = 16;
            int accumulator = 0;
            for (int i = 0; i < testCount; ++i)
            {
                var startTime = Stopwatch.GetTimestamp();
                accumulator += function();
                var endTime = Stopwatch.GetTimestamp();
                accumulatedTime += endTime - startTime;
                //overlapHandler.Set.Clear();
                CacheBlaster.Blast();
            }
            Console.WriteLine($"{name} time per execution (ms): {(accumulatedTime) * 1e3 / (testCount * Stopwatch.Frequency)}, acc: {accumulator}");
        }


    }
}
