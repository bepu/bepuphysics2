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
        //if (sum == int.MaxValue)
        Interlocked.Add(ref typedContext->Sum, sum);
    }

    static void DispatcherBody(int workerIndex, void* context)
    {
        var taskQueue = (TaskQueue*)context;
        while (taskQueue->TryDequeueAndRun(workerIndex) != DequeueTaskResult.Stop) ;
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




        int iterationCount = 128;
        int tasksPerIteration = 4;
        var taskQueue = new TaskQueue(BufferPool);
        var taskQueuePointer = &taskQueue;
        Test(() =>
        {
            var context = new Context { };
            for (int i = 0; i < iterationCount; ++i)
                taskQueuePointer->TryEnqueueForUnsafely(&Test, &context, 0, tasksPerIteration);
            taskQueuePointer->TryEnqueueStopUnsafely();
            ThreadDispatcher.DispatchWorkers(&DispatcherBody, taskQueuePointer);
            return context.Sum;
        }, "MT", () => taskQueuePointer->Reset());

        taskQueue.Dispose(BufferPool);

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

    static void Test(TestFunction function, string name, Action reset = null)
    {
        long accumulatedTime = 0;
        const int testCount = 128;
        int accumulator = 0;
        for (int i = 0; i < testCount; ++i)
        {
            var startTime = Stopwatch.GetTimestamp();
            accumulator += function();
            var endTime = Stopwatch.GetTimestamp();
            reset?.Invoke();
            accumulatedTime += endTime - startTime;
            //overlapHandler.Set.Clear();
            //CacheBlaster.Blast();
        }
        Console.WriteLine($"{name} time per execution (ms): {(accumulatedTime) * 1e3 / (testCount * Stopwatch.Frequency)}, acc: {accumulator}");
    }


}
