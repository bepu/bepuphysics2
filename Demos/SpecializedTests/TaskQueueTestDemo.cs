using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Threading;

namespace Demos.SpecializedTests;

public unsafe class TaskQueueTestDemo : Demo
{
    static void Test(int taskId, void* context, int workerIndex)
    {
        int sum = 0;
        for (int i = 0; i < 100000; ++i)
        {
            sum = (sum ^ i) * i;
        }
        var typedContext = (Context*)context;
        Interlocked.Add(ref typedContext->Sum, sum);
    }

    static void DispatcherBody(int workerIndex, void* context)
    {
        var taskQueue = (TaskQueue*)context;
        while (taskQueue->DequeueAndRun(workerIndex)) ;
    }

    struct Context
    {
        public int Sum;
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-10, 3, -10);
        camera.Yaw = MathHelper.Pi * 3f / 4;
        camera.Pitch = 0;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));




        int iterationCount = 10;
        int tasksPerIteration = 16;
        Test(() =>
        {
            var context = new Context { };
            var taskQueue = new TaskQueue(BufferPool);
            for (int i = 0; i < iterationCount; ++i)
                taskQueue.EnqueueFor(&Test, &context, 0, tasksPerIteration);
            taskQueue.EnqueueStop();
            ThreadDispatcher.DispatchWorkers(&DispatcherBody, &taskQueue);

            taskQueue.Dispose(BufferPool);
            return context.Sum;
        }, "MT");


        Test(() =>
        {
            var testContext = new Context { };
            for (int i = 0; i < iterationCount * tasksPerIteration; ++i)
            {
                Test(0, &testContext, 0);
            }
            return testContext.Sum;
        }, "ST");

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
