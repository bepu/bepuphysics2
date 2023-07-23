using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuUtilities.TaskScheduling;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Threading;

namespace BepuPhysics.Trees;

partial struct Tree
{
    readonly unsafe void Refit2(ref NodeChild childInParent)
    {
        Debug.Assert(LeafCount >= 2);
        ref var node = ref Nodes[childInParent.Index];
        ref var a = ref node.A;
        if (a.Index >= 0)
        {
            Refit2(ref a);
        }
        ref var b = ref node.B;
        if (b.Index >= 0)
        {
            Refit2(ref b);
        }
        BoundingBox.CreateMergedUnsafeWithPreservation(a, b, out childInParent);
    }
    /// <summary>
    /// Updates the bounding boxes of all internal nodes in the tree.
    /// </summary>
    public unsafe readonly void Refit2()
    {
        //No point in refitting a tree with no internal nodes!
        if (LeafCount <= 2)
            return;
        NodeChild stub = default;
        Refit2(ref stub);
    }

    unsafe struct RefitContext
    {
        public Tree* Tree;
        public TaskStack* TaskStack;
        public int LeafCountPerTask;
    }
    unsafe readonly void Refit2WithTaskSpawning(ref NodeChild childInParent, RefitContext* context, int workerIndex, IThreadDispatcher dispatcher)
    {
        Debug.Assert(LeafCount >= 2);
        ref var node = ref Nodes[childInParent.Index];
        ref var a = ref node.A;
        ref var b = ref node.B;
        Debug.Assert(context->LeafCountPerTask > 1);
        if (a.LeafCount >= context->LeafCountPerTask && b.LeafCount >= context->LeafCountPerTask)
        {
            //Both children are big enough to warrant a task. Spawn one task for B and recurse on A.
            //(We always punt B because, if any cache optimizer-ish stuff is going on, A will be more likely to be contiguous in memory.)
            var task = new Task(&Refit2Task, context, childInParent.Index);
            var continuation = context->TaskStack->AllocateContinuationAndPush(new Span<Task>(ref task), workerIndex, dispatcher);
            Debug.Assert(a.Index >= 0);
            Refit2WithTaskSpawning(ref a, context, workerIndex, dispatcher);
            //Wait until B is fully done before continuing.
            context->TaskStack->WaitForCompletion(continuation, workerIndex, dispatcher);
        }
        else
        {
            //At least one child is too small to warrant a new task. Just recurse on both.
            if (a.Index >= 0)
            {
                Refit2(ref a);
            }
            if (b.Index >= 0)
            {
                Refit2(ref b);
            }
        }
        BoundingBox.CreateMergedUnsafeWithPreservation(a, b, out childInParent);
    }

    static unsafe void Refit2Task(long parentNodeIndex, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        var context = (RefitContext*)untypedContext;
        ref var node = ref context->Tree->Nodes[(int)parentNodeIndex];
        context->Tree->Refit2WithTaskSpawning(ref node.B, context, workerIndex, dispatcher);
    }
    static unsafe void RefitRootEntryTask(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        var context = (RefitContext*)untypedContext;
        NodeChild stub = default;
        context->Tree->Refit2WithTaskSpawning(ref stub, context, workerIndex, dispatcher);
        context->TaskStack->RequestStop();
    }

    unsafe readonly void Refit2(BufferPool pool, IThreadDispatcher dispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount, bool internallyDispatch)
    {
        //No point in refitting a tree with no internal nodes!
        if (LeafCount <= 2)
            return;
        var tree = this;
        if (targetTaskCount < 0)
            targetTaskCount = dispatcher.ThreadCount;
        var leafCountPerTask = (int)float.Ceiling(LeafCount / (float)targetTaskCount);
        var refitContext = new RefitContext { LeafCountPerTask = leafCountPerTask, TaskStack = taskStack, Tree = &tree };
        if (internallyDispatch)
        {
            taskStack->PushUnsafely(new Task(&RefitRootEntryTask, &refitContext), workerIndex, dispatcher);
            TaskStack.DispatchWorkers(dispatcher, taskStack, int.Min(dispatcher.ThreadCount, targetTaskCount));
        }
        else
        {
            NodeChild stub = default;
            Refit2WithTaskSpawning(ref stub, &refitContext, workerIndex, dispatcher);
        }
    }

    /// <summary>
    /// Refits all bounding boxes in the tree using multiple threads.
    /// </summary>
    /// <param name="pool">Pool used for main thread temporary allocations during execution.</param>
    /// <param name="dispatcher">Dispatcher used during execution.</param>
    public unsafe readonly void Refit2(BufferPool pool, IThreadDispatcher dispatcher)
    {
        var taskStack = new TaskStack(pool, dispatcher, dispatcher.ThreadCount);
        Refit2(pool, dispatcher, &taskStack, 0, -1, internallyDispatch: true);
        taskStack.Dispose(pool, dispatcher);
    }

    /// <summary>
    /// Refits all bounding boxes in the tree using multiple threads.<para/>Pushes tasks into the provided <see cref="TaskStack"/>. Does not dispatch threads internally; this is intended to be used as a part of a caller-managed dispatch.
    /// </summary>
    /// <param name="pool">Pool used for allocations during execution.</param>
    /// <param name="dispatcher">Dispatcher used during execution.</param>
    /// <param name="taskStack"><see cref="TaskStack"/> that the refit operation will push tasks onto as needed.</param>
    /// <param name="workerIndex">Index of the worker calling the function.</param>
    /// <param name="targetTaskCount">Number of tasks the refit should try to create during execution.</param>
    public unsafe readonly void Refit2(BufferPool pool, IThreadDispatcher dispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount = -1)
    {
        Refit2(pool, dispatcher, taskStack, workerIndex, targetTaskCount, internallyDispatch: false);
    }

}
