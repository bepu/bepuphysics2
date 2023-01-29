using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Threading;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Reflection.Metadata;
using BepuUtilities.TestQueue;
using BepuUtilities.TestParallelStack;
using TaskQ = BepuUtilities.TestQueue.Task;
using BepuUtilities.TestStack;
using BepuUtilities.TestLinkedTaskStack;

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
        var sum = DoSomeWork(10000, 0);
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
        var sum = DoSomeWork(10000, 0);
        Interlocked.Add(ref ((DynamicContext2*)context)->Context->Sum, sum);
    }
    static void Test<T>(long taskId, void* context, int workerIndex, IThreadDispatcher dispatcher) where T : unmanaged
    {
        var sum = DoSomeWork(100000, 0);
        var typedContext = (Context*)context;
        //if ((taskId & 7) == 0)
        {
            const int subtaskCount = 8;
            var context1 = new DynamicContext1 { Context = typedContext };
            var context2 = new DynamicContext2 { Context = typedContext };
            if (typeof(T) == typeof(TaskQueue))
            {
                var queue = (TaskQueue*)typedContext->TaskPile;
                queue->For(&DynamicallyEnqueuedTest1, &context1, 0, subtaskCount, workerIndex, dispatcher);
                queue->For(&DynamicallyEnqueuedTest2, &context2, 0, subtaskCount, workerIndex, dispatcher);
            }
            else if (typeof(T) == typeof(ParallelTaskStack))
            {
                var stack = (ParallelTaskStack*)typedContext->TaskPile;
                stack->For(&DynamicallyEnqueuedTest1, &context1, 0, subtaskCount, workerIndex, dispatcher);
                stack->For(&DynamicallyEnqueuedTest2, &context2, 0, subtaskCount, workerIndex, dispatcher);
            }
            else if (typeof(T) == typeof(TaskStack))
            {
                var stack = (TaskStack*)typedContext->TaskPile;
                stack->For(&DynamicallyEnqueuedTest1, &context1, 0, subtaskCount, workerIndex, dispatcher);
                stack->For(&DynamicallyEnqueuedTest2, &context2, 0, subtaskCount, workerIndex, dispatcher);
            }
            else if (typeof(T) == typeof(LinkedTaskStack))
            {
                var stack = (LinkedTaskStack*)typedContext->TaskPile;
                stack->For(&DynamicallyEnqueuedTest1, &context1, 0, subtaskCount, workerIndex, dispatcher);
                stack->For(&DynamicallyEnqueuedTest2, &context2, 0, subtaskCount, workerIndex, dispatcher);
            }
        }
        Interlocked.Add(ref typedContext->Sum, sum);
    }
    static void STTest(long taskId, void* context, int workerIndex, IThreadDispatcher dispatcher)
    {
        var sum = DoSomeWork(100000, 0);
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
        if (typeof(T) == typeof(TaskQueue))
        {
            var taskQueue = (TaskQueue*)dispatcher.UnmanagedContext;
            while (taskQueue->TryDequeueAndRun(workerIndex, dispatcher) != DequeueTaskResult.Stop) ;
        }
        else if (typeof(T) == typeof(ParallelTaskStack))
        {
            var taskStack = (ParallelTaskStack*)dispatcher.UnmanagedContext;
            while (taskStack->TryPopAndRun(workerIndex, dispatcher) != BepuUtilities.TestParallelStack.PopTaskResult.Stop) ;
        }
        else if (typeof(T) == typeof(TaskStack))
        {
            var taskStack = (TaskStack*)dispatcher.UnmanagedContext;
            while (taskStack->TryPopAndRun(workerIndex, dispatcher) != BepuUtilities.TestStack.PopTaskResult.Stop) ;
        }
        else if (typeof(T) == typeof(LinkedTaskStack))
        {
            var taskStack = (LinkedTaskStack*)dispatcher.UnmanagedContext;
            while (taskStack->TryPopAndRun(workerIndex, dispatcher) != BepuUtilities.TestLinkedTaskStack.PopTaskResult.Stop) ;
        }
    }

    struct Context
    {
        public void* TaskPile;
        public int Sum;
    }

    static void IssueStop<T>(long id, void* context, int workerIndex, IThreadDispatcher dispatcher) where T : unmanaged
    {
        var typedContext = (Context*)context;
        if (typeof(T) == typeof(TaskQueue))
            ((TaskQueue*)typedContext->TaskPile)->EnqueueStop(workerIndex, dispatcher);
        else if (typeof(T) == typeof(ParallelTaskStack))
            ((ParallelTaskStack*)typedContext->TaskPile)->RequestStop();
        else if (typeof(T) == typeof(TaskStack))
            ((TaskStack*)typedContext->TaskPile)->RequestStop();
        else if (typeof(T) == typeof(LinkedTaskStack))
            ((LinkedTaskStack*)typedContext->TaskPile)->RequestStop();

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

        for (int i = 0; i < 1000; ++i)
        {
            var linkedTaskStack = new LinkedTaskStack(BufferPool, ThreadDispatcher, ThreadDispatcher.ThreadCount);
            var linkedTaskStackPointer = &linkedTaskStack;
            Test(() =>
            {
                var context = new Context { TaskPile = linkedTaskStackPointer };
                var continuation = linkedTaskStackPointer->AllocateContinuation(iterationCount * tasksPerIteration, 0, ThreadDispatcher, new BepuUtilities.TestLinkedTaskStack.Task(&IssueStop<LinkedTaskStack>, &context));
                for (int i = 0; i < iterationCount; ++i)
                {
                    linkedTaskStackPointer->PushForUnsafely(&Test<LinkedTaskStack>, &context, i * tasksPerIteration, tasksPerIteration, 0, ThreadDispatcher, continuation);
                }
                //taskQueuePointer->TryEnqueueStopUnsafely();
                //taskQueuePointer->EnqueueTasks()
                ThreadDispatcher.DispatchWorkers(&DispatcherBody<LinkedTaskStack>, unmanagedContext: linkedTaskStackPointer);
                return context.Sum;
            }, "MT Linked Stack", () => linkedTaskStackPointer->Reset(ThreadDispatcher));
            linkedTaskStack.Dispose(BufferPool, ThreadDispatcher);
        }




        //TASK STACK
        var taskStack = new TaskStack(BufferPool, ThreadDispatcher);
        var taskStackPointer = &taskStack;

        Test(() =>
        {
            var context = new Context { TaskPile = taskStackPointer };
            var continuation = taskStackPointer->AllocateContinuation(iterationCount * tasksPerIteration, 0, ThreadDispatcher, new BepuUtilities.TestStack.Task(&IssueStop<TaskStack>, &context));
            for (int i = 0; i < iterationCount; ++i)
            {
                taskStackPointer->TryPushForUnsafely(&Test<TaskStack>, &context, i * tasksPerIteration, tasksPerIteration, continuation);
            }
            //taskQueuePointer->TryEnqueueStopUnsafely();
            //taskQueuePointer->EnqueueTasks()
            ThreadDispatcher.DispatchWorkers(&DispatcherBody<TaskStack>, unmanagedContext: taskStackPointer);
            return context.Sum;
        }, "MT Stack", () => taskStackPointer->Reset());


        taskStack.Dispose(BufferPool);


        //PARALLEL TASK STACK
        var parallelTaskStack = new ParallelTaskStack(BufferPool, ThreadDispatcher, ThreadDispatcher.ThreadCount);
        var parallelTaskStackPointer = &parallelTaskStack;

        Test(() =>
        {
            var context = new Context { TaskPile = parallelTaskStackPointer };
            var continuation = parallelTaskStackPointer->AllocateContinuation(iterationCount * tasksPerIteration, 0, ThreadDispatcher, new BepuUtilities.TestParallelStack.Task(&IssueStop<ParallelTaskStack>, &context));
            for (int i = 0; i < iterationCount; ++i)
            {
                parallelTaskStackPointer->PushForUnsafely(&Test<ParallelTaskStack>, &context, i * tasksPerIteration, tasksPerIteration, i % ThreadDispatcher.ThreadCount, ThreadDispatcher, continuation);
            }
            //taskQueuePointer->TryEnqueueStopUnsafely();
            //taskQueuePointer->EnqueueTasks()
            ThreadDispatcher.DispatchWorkers(&DispatcherBody<ParallelTaskStack>, unmanagedContext: parallelTaskStackPointer);
            return context.Sum;
        }, "MT Parallel Stack", () => parallelTaskStackPointer->Reset());

        var noSteal = ParallelTaskStack.NoStealRequired;
        var stealRequired = ParallelTaskStack.StealRequired;
        var stealFail = ParallelTaskStack.PopFailed;
        var steal = stealRequired - stealFail;

        Console.WriteLine($"Nosteal, steal succeed, steal fail: {noSteal}, {steal}, {stealFail} ({steal / (double)(stealRequired + noSteal)})");

        parallelTaskStack.Dispose(BufferPool);


        //TASK QUEUE
        var taskQueue = new TaskQueue(BufferPool, maximumTaskCapacity: 1 << 19, maximumContinuationCapacity: 1 << 19);
        var taskQueuePointer = &taskQueue;

        Test(() =>
        {
            var context = new Context { TaskPile = taskQueuePointer };
            var continuation = taskQueuePointer->AllocateContinuation(iterationCount * tasksPerIteration, 0, ThreadDispatcher, new TaskQ(&IssueStop<TaskQueue>, &context));
            for (int i = 0; i < iterationCount; ++i)
            {
                taskQueuePointer->TryEnqueueForUnsafely(&Test<TaskQueue>, &context, i * tasksPerIteration, tasksPerIteration, continuation);
            }
            //taskQueuePointer->TryEnqueueStopUnsafely();
            //taskQueuePointer->EnqueueTasks()
            ThreadDispatcher.DispatchWorkers(&DispatcherBody<TaskQueue>, unmanagedContext: taskQueuePointer);
            return context.Sum;
        }, "MT", () => taskQueuePointer->Reset());

        taskQueue.Dispose(BufferPool);

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
