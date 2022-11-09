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
    static int DoSomeWork(int iterations, int sum)
    {
        for (int i = 0; i < iterations; ++i)
        {
            sum = (sum ^ i) * i;
        }
        return sum;
    }
    static int counter = 0;
    static void DynamicallyEnqueuedTest(long taskId, void* context, int workerIndex)
    {
        var sum = DoSomeWork(10000, 0);
        //Console.WriteLine($"Dynamically enqueued {Interlocked.Increment(ref counter)}");
        Interlocked.Add(ref ((Context*)context)->Sum, sum);
    }
    static void Test(long taskId, void* context, int workerIndex)
    {
        var sum = DoSomeWork(100000, 0);
        var typedContext = (Context*)context;
        if ((taskId & 7) == 0)
        {
            const int subtaskCount = 8;
            typedContext->Queue->For(&DynamicallyEnqueuedTest, context, 0, subtaskCount, workerIndex);
        }
        Interlocked.Add(ref typedContext->Sum, sum);
    }
    static void STTest(long taskId, void* context, int workerIndex)
    {
        var sum = DoSomeWork(100000, 0);
        var typedContext = (Context*)context;
        if ((taskId & 7) == 0)
        {
            const int subtaskCount = 8;
            for (int i = 0; i < subtaskCount; ++i)
            {
                DynamicallyEnqueuedTest(i, context, workerIndex);
            }
        }
        Interlocked.Add(ref typedContext->Sum, sum);
    }

    static void DispatcherBody(int workerIndex, void* context)
    {
        var taskQueue = (TaskQueue*)context;
        while (taskQueue->TryDequeueAndRun(workerIndex) != DequeueTaskResult.Stop) ;
    }

    struct Context
    {
        public TaskQueue* Queue;
        public int Sum;
    }

    static void IssueStop(long id, void* context, int workerIndex)
    {
        var typedContext = (Context*)context;
        typedContext->Queue->EnqueueStop(workerIndex);
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-10, 3, -10);
        camera.Yaw = MathHelper.Pi * 3f / 4;
        camera.Pitch = 0;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));




        int iterationCount = 4;
        int tasksPerIteration = 64;
        var taskQueue = new TaskQueue(BufferPool);
        var taskQueuePointer = &taskQueue;
        Test(() =>
        {
            var context = new Context { Queue = taskQueuePointer };
            var continuation = taskQueuePointer->AllocateContinuation(iterationCount * tasksPerIteration, 0, new Task(&IssueStop, &context));
            for (int i = 0; i < iterationCount; ++i)
                taskQueuePointer->TryEnqueueForUnsafely(&Test, &context, i * tasksPerIteration, tasksPerIteration, continuation);
            //taskQueuePointer->TryEnqueueStopUnsafely();
            //taskQueuePointer->EnqueueTasks()
            ThreadDispatcher.DispatchWorkers(&DispatcherBody, taskQueuePointer);
            return context.Sum;
        }, "MT", () => taskQueuePointer->Reset());

        taskQueue.Dispose(BufferPool);

        Test(() =>
        {
            var testContext = new Context { };
            for (int i = 0; i < iterationCount; ++i)
            {
                for (int j = 0; j < tasksPerIteration; ++j)
                {
                    STTest(i * tasksPerIteration + j, &testContext, 0);
                }
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
