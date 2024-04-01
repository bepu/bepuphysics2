using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Threading;
using System.Runtime.InteropServices;
using BepuUtilities.TaskScheduling;

namespace Demos.SpecializedTests;


public unsafe class TaskStackTestDemo : Demo
{
    static int DoSomeWork(int iterations, int sum)
    {
        for (int i = 0; i < iterations; ++i)
        {
            sum = (sum ^ i) * i;
        }
        return sum;
    }

    //Try different context layouts to make sure the task queue isn't mixing and matching tasks somehow.
    [StructLayout(LayoutKind.Explicit)]
    struct DynamicContext1
    {
        [FieldOffset(0)]
        public long Pad;
        [FieldOffset(8)]
        public Context* Context;
    }

    static void DynamicallyEnqueuedTest1(long taskId, void* context, int workerIndex, IThreadDispatcher dispatcher)
    {
        var sum = DoSomeWork(100, 0);
        Interlocked.Add(ref ((DynamicContext1*)context)->Context->Sum, sum);
    }

    [StructLayout(LayoutKind.Explicit)]
    struct DynamicContext2
    {
        [FieldOffset(0)]
        public long Pad1;
        [FieldOffset(8)]
        public long Pad2;
        [FieldOffset(16)]
        public Context* Context;
    }

    static void DynamicallyEnqueuedTest2(long taskId, void* context, int workerIndex, IThreadDispatcher dispatcher)
    {
        var sum = DoSomeWork(100, 0);
        Interlocked.Add(ref ((DynamicContext2*)context)->Context->Sum, sum);
    }
    static void Test<T>(long taskId, void* context, int workerIndex, IThreadDispatcher dispatcher) where T : unmanaged
    {
        var sum = DoSomeWork(100, 0);
        var typedContext = (Context*)context;
        //if ((taskId & 7) == 0)
        {
            const int subtaskCount = 8;
            var context1 = new DynamicContext1 { Context = typedContext };
            var context2 = new DynamicContext2 { Context = typedContext };

            var stack = (TaskStack*)typedContext->TaskPile;
            stack->For(&DynamicallyEnqueuedTest1, &context1, 0, subtaskCount, workerIndex, dispatcher);
            stack->For(&DynamicallyEnqueuedTest2, &context2, 0, subtaskCount, workerIndex, dispatcher);
        }
        Interlocked.Add(ref typedContext->Sum, sum);
    }
    static void STTest(long taskId, void* context, int workerIndex, IThreadDispatcher dispatcher)
    {
        var sum = DoSomeWork(100, 0);
        var typedContext = (Context*)context;
        //if ((taskId & 7) == 0)
        {
            const int subtaskCount = 8;
            var context1 = new DynamicContext1 { Context = typedContext };
            for (int i = 0; i < subtaskCount; ++i)
            {
                DynamicallyEnqueuedTest1(i, &context1, workerIndex, dispatcher);
            }
            var context2 = new DynamicContext2 { Context = typedContext };
            for (int i = 0; i < subtaskCount; ++i)
            {
                DynamicallyEnqueuedTest2(i, &context2, workerIndex, dispatcher);
            }
        }
        Interlocked.Add(ref typedContext->Sum, sum);
    }

    static void DispatcherBody<T>(int workerIndex, IThreadDispatcher dispatcher) where T : unmanaged
    {
        //if (workerIndex > 1)
        //    return;
        var taskStack = (TaskStack*)dispatcher.UnmanagedContext;
        while (taskStack->TryPopAndRun(workerIndex, dispatcher) != PopTaskResult.Stop) ;

    }

    struct Context
    {
        public void* TaskPile;
        public int Sum;
    }

    static void IssueStop<T>(long id, void* context, int workerIndex, IThreadDispatcher dispatcher) where T : unmanaged
    {
        var typedContext = (Context*)context;
        ((TaskStack*)typedContext->TaskPile)->RequestStop();

    }

    static void EmptyDispatch(int workerIndex, IThreadDispatcher dispatcher)
    {

    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-10, 3, -10);
        camera.Yaw = MathHelper.Pi * 3f / 4;
        camera.Pitch = 0;

        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));

        //Console.WriteLine($"Task size: {Unsafe.SizeOf<TaskQ>()}, {Unsafe.SizeOf<TaskPS>()}");

        int iterationCount = 4;
        int tasksPerIteration = 64;

        //Test(() =>
        //{
        //    for (int i = 0; i < 1024; ++i)
        //        ThreadDispatcher.DispatchWorkers(&EmptyDispatch);
        //    return 0;
        //}, "Dispatch");

        //LINKED TASK STACK

        for (int i = 0; i < 10; ++i)
        {
            var linkedTaskStack = new TaskStack(BufferPool, ThreadDispatcher, ThreadDispatcher.ThreadCount);
            var linkedTaskStackPointer = &linkedTaskStack;
            Test(() =>
            {
                var context = new Context { TaskPile = linkedTaskStackPointer };
                var continuation = linkedTaskStackPointer->AllocateContinuation(iterationCount * tasksPerIteration, 0, ThreadDispatcher, new Task(&IssueStop<TaskStack>, &context));
                for (int i = 0; i < iterationCount; ++i)
                {
                    linkedTaskStackPointer->PushForUnsafely(&Test<TaskStack>, &context, i * tasksPerIteration, tasksPerIteration, 0, ThreadDispatcher, continuation: continuation);
                }
                //taskQueuePointer->TryEnqueueStopUnsafely();
                //taskQueuePointer->EnqueueTasks()
                ThreadDispatcher.DispatchWorkers(&DispatcherBody<TaskStack>, unmanagedContext: linkedTaskStackPointer);
                return context.Sum;
            }, "MT Linked Stack", () => linkedTaskStackPointer->Reset(ThreadDispatcher));
            linkedTaskStack.Dispose(BufferPool, ThreadDispatcher);
        }



        //Test(() =>
        //{
        //    var testContext = new Context { };
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        for (int j = 0; j < tasksPerIteration; ++j)
        //        {
        //            STTest(i * tasksPerIteration + j, &testContext, 0, ThreadDispatcher);
        //        }
        //    }
        //    return testContext.Sum;
        //}, "ST");

    }

    delegate int TestFunction();

    static void Test(TestFunction function, string name, Action reset = null)
    {
        long accumulatedTime = 0;
        const int testCount = 1024;
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
