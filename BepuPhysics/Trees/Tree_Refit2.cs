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
        public Tree Tree;
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
            //At least one child is too small to warrant a new task. The larger one may still be worth spawning subtasks within, though.
            if (a.Index >= 0)
            {
                if (a.LeafCount >= context->LeafCountPerTask)
                    Refit2WithTaskSpawning(ref a, context, workerIndex, dispatcher);
                else
                    Refit2(ref a);
            }
            if (b.Index >= 0)
            {
                if (b.LeafCount >= context->LeafCountPerTask)
                    Refit2WithTaskSpawning(ref b, context, workerIndex, dispatcher);
                else
                    Refit2(ref b);
            }
        }
        BoundingBox.CreateMergedUnsafeWithPreservation(a, b, out childInParent);
    }

    static unsafe void Refit2Task(long parentNodeIndex, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        var context = (RefitContext*)untypedContext;
        ref var node = ref context->Tree.Nodes[(int)parentNodeIndex];
        context->Tree.Refit2WithTaskSpawning(ref node.B, context, workerIndex, dispatcher);
    }
    static unsafe void RefitRootEntryTask(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        var context = (RefitContext*)untypedContext;
        NodeChild stub = default;
        context->Tree.Refit2WithTaskSpawning(ref stub, context, workerIndex, dispatcher);
        context->TaskStack->RequestStop();
    }

    unsafe readonly void Refit2(BufferPool pool, IThreadDispatcher dispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount, bool internallyDispatch)
    {
        //No point in refitting a tree with no internal nodes!
        if (LeafCount <= 2)
            return;
        if (targetTaskCount < 0)
            targetTaskCount = dispatcher.ThreadCount;
        const int minimumTaskSize = 32;
        var leafCountPerTask = int.Max(minimumTaskSize, (int)float.Ceiling(LeafCount / (float)targetTaskCount));
        var refitContext = new RefitContext { LeafCountPerTask = leafCountPerTask, TaskStack = taskStack, Tree = this };
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

    unsafe struct RefitWithCacheOptimizationContext
    {
        public Buffer<Node> SourceNodes;
        public Tree Tree;
        //These aren't used in the ST path, but, shrug! it's a few bytes.
        public int LeafCountPerTask;
        public TaskStack* TaskStack;
    }

    static unsafe void Refit2WithCacheOptimization(int sourceNodeIndex, int parentIndex, int childIndexInParent, ref NodeChild childInParent, ref RefitWithCacheOptimizationContext context)
    {
        Debug.Assert(context.Tree.LeafCount >= 2);

        ref var sourceNode = ref context.SourceNodes[sourceNodeIndex];
        var targetNodeIndex = childInParent.Index;
        ref var targetNode = ref context.Tree.Nodes[targetNodeIndex];
        ref var targetMetanode = ref context.Tree.Metanodes[targetNodeIndex];
        targetMetanode.Parent = parentIndex;
        targetMetanode.IndexInParent = childIndexInParent;
        ref var sourceA = ref sourceNode.A;
        ref var sourceB = ref sourceNode.B;
        var targetIndexA = targetNodeIndex + 1;
        var targetIndexB = targetNodeIndex + sourceA.LeafCount;
        ref var targetA = ref targetNode.A;
        ref var targetB = ref targetNode.B;
        if (sourceA.Index >= 0)
        {
            targetA.Index = targetIndexA;
            targetA.LeafCount = sourceA.LeafCount;
            Refit2WithCacheOptimization(sourceA.Index, targetNodeIndex, 0, ref targetA, ref context);
        }
        else
        {
            //It's a leaf; copy over the source verbatim.
            targetA = sourceA;
            context.Tree.Leaves[Encode(sourceA.Index)] = new Leaf(targetNodeIndex, 0);
        }
        if (sourceB.Index >= 0)
        {
            targetB.Index = targetIndexB;
            targetB.LeafCount = sourceB.LeafCount;
            Refit2WithCacheOptimization(sourceB.Index, targetNodeIndex, 1, ref targetB, ref context);
        }
        else
        {
            targetB = sourceB;
            context.Tree.Leaves[Encode(sourceB.Index)] = new Leaf(targetNodeIndex, 1);
        }
        BoundingBox.CreateMergedUnsafeWithPreservation(targetA, targetB, out childInParent);
    }


    /// <summary>
    /// Updates the bounding boxes of all internal nodes in the tree. The refit is based on the provided sourceNodes, and 
    /// the results are written into the tree's current <see cref="Nodes"/>, <see cref="Metanodes"/>, and <see cref="Leaves"/> buffers.
    /// The nodes and metanodes will be in depth traversal order.
    /// The input source buffer is not modified.
    /// </summary>
    /// <param name="sourceNodes">Nodes to base the refit on.</param>
    public unsafe void Refit2WithCacheOptimization(Buffer<Node> sourceNodes)
    {
        //No point in refitting a tree with no internal nodes!
        if (LeafCount <= 2)
            return;
        NodeChild stub = default;
        var context = new RefitWithCacheOptimizationContext
        {
            SourceNodes = sourceNodes,
            Tree = this,
        };
        Refit2WithCacheOptimization(0, -1, -1, ref stub, ref context);

    }

    /// <summary>
    /// Updates the bounding boxes of all internal nodes in the tree. Reallocates the nodes and metanodes and writes the refit tree into them in depth first traversal order.
    /// The tree instance is modified to point to the new nodes and metanodes.
    /// </summary>
    /// <param name="pool">Pool to allocate from. If disposeOriginals is true, this must be the same pool from which the <see cref="Nodes"/> buffer was allocated from.</param>
    /// <param name="disposeOriginalNodes">Whether to dispose of the original nodes buffer. If false, it's up to the caller to dispose of it appropriately.</param>
    public unsafe void Refit2WithCacheOptimization(BufferPool pool, bool disposeOriginalNodes = true)
    {
        //No point in refitting a tree with no internal nodes!
        if (LeafCount <= 2)
            return;
        var oldNodes = Nodes;
        Nodes = new Buffer<Node>(oldNodes.Length, pool);
        Refit2WithCacheOptimization(oldNodes);
        if (disposeOriginalNodes)
            oldNodes.Dispose(pool);
    }

    static unsafe void Refit2WithCacheOptimizationAndTaskSpawning(
        int sourceNodeIndex, int parentIndex, int childIndexInParent, ref NodeChild childInParent, RefitWithCacheOptimizationContext* context, int workerIndex, IThreadDispatcher dispatcher)
    {
        Debug.Assert(context->Tree.LeafCount >= 2);
        ref var sourceNode = ref context->SourceNodes[sourceNodeIndex];
        var targetNodeIndex = childInParent.Index;
        ref var targetNode = ref context->Tree.Nodes[targetNodeIndex];
        ref var targetMetanode = ref context->Tree.Metanodes[targetNodeIndex];
        targetMetanode.Parent = parentIndex;
        targetMetanode.IndexInParent = childIndexInParent;
        ref var sourceA = ref sourceNode.A;
        ref var sourceB = ref sourceNode.B;
        var targetIndexA = targetNodeIndex + 1;
        var targetIndexB = targetNodeIndex + sourceA.LeafCount;
        ref var targetA = ref targetNode.A;
        ref var targetB = ref targetNode.B;
        Debug.Assert(context->LeafCountPerTask > 1);
        if (sourceA.LeafCount >= context->LeafCountPerTask && sourceB.LeafCount >= context->LeafCountPerTask)
        {
            //Both children are big enough to warrant a task. Spawn one task for B and recurse on A.
            //(We always punt B because, if any cache optimizer-ish stuff is going on (like this process we're doing now!), A will be more likely to be contiguous in memory.)
            //Note that we encode both the target AND source parent indices into the task id.
            targetA.Index = targetIndexA;
            targetA.LeafCount = sourceA.LeafCount;
            targetB.Index = targetIndexB;
            targetB.LeafCount = sourceB.LeafCount;
            var task = new Task(&Refit2WithCacheOptimizationTask, context, (long)childInParent.Index | ((long)sourceNodeIndex << 32));
            var continuation = context->TaskStack->AllocateContinuationAndPush(new Span<Task>(ref task), workerIndex, dispatcher);
            Debug.Assert(sourceA.Index >= 0);
            Refit2WithCacheOptimizationAndTaskSpawning(sourceA.Index, targetNodeIndex, 0, ref targetA, context, workerIndex, dispatcher);
            //Wait until B is fully done before continuing.
            context->TaskStack->WaitForCompletion(continuation, workerIndex, dispatcher);
        }
        else
        {
            //At least one child is too small to warrant a new task. The larger one may still be worth spawning subtasks within, though.
            if (sourceA.Index >= 0)
            {
                targetA.Index = targetIndexA;
                targetA.LeafCount = sourceA.LeafCount;
                if (sourceA.LeafCount >= context->LeafCountPerTask)
                    Refit2WithCacheOptimizationAndTaskSpawning(sourceA.Index, targetNodeIndex, 0, ref targetA, context, workerIndex, dispatcher);
                else
                    Refit2WithCacheOptimization(sourceA.Index, targetNodeIndex, 0, ref targetA, ref *context);
            }
            else
            {
                //It's a leaf; copy over the source verbatim.
                targetA = sourceA;
                context->Tree.Leaves[Encode(sourceA.Index)] = new Leaf(targetNodeIndex, 0);
            }
            if (sourceB.Index >= 0)
            {
                targetB.Index = targetIndexB;
                targetB.LeafCount = sourceB.LeafCount;
                if (sourceB.LeafCount >= context->LeafCountPerTask)
                    Refit2WithCacheOptimizationAndTaskSpawning(sourceB.Index, targetNodeIndex, 1, ref targetB, context, workerIndex, dispatcher);
                else
                    Refit2WithCacheOptimization(sourceB.Index, targetNodeIndex, 1, ref targetB, ref *context);
            }
            else
            {
                targetB = sourceB;
                context->Tree.Leaves[Encode(sourceB.Index)] = new Leaf(targetNodeIndex, 1);
            }
        }
        BoundingBox.CreateMergedUnsafeWithPreservation(targetA, targetB, out childInParent);
    }

    static unsafe void Refit2WithCacheOptimizationTask(long parentNodeIndices, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        var context = (RefitWithCacheOptimizationContext*)untypedContext;
        var sourceParentIndex = (int)(parentNodeIndices >> 32);
        var targetParentIndex = (int)parentNodeIndices;
        ref var sourceParentNode = ref context->SourceNodes[sourceParentIndex];
        ref var targetParentNode = ref context->Tree.Nodes[targetParentIndex];
        Refit2WithCacheOptimizationAndTaskSpawning(sourceParentNode.B.Index, targetParentIndex, 1, ref targetParentNode.B, context, workerIndex, dispatcher);
    }
    static unsafe void RefitWithCacheOptimizationRootEntryTask(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        var context = (RefitWithCacheOptimizationContext*)untypedContext;
        NodeChild stub = default;
        Refit2WithCacheOptimizationAndTaskSpawning(0, -1, -1, ref stub, context, workerIndex, dispatcher);
        context->TaskStack->RequestStop();
    }
    unsafe void Refit2WithCacheOptimization(BufferPool pool, IThreadDispatcher dispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount, bool internallyDispatch, Buffer<Node> sourceNodes)
    {
        //No point in refitting a tree with no internal nodes!
        if (LeafCount <= 2)
            return;
        if (targetTaskCount < 0)
            targetTaskCount = dispatcher.ThreadCount;

        const int minimumTaskSize = 32;
        var leafCountPerTask = int.Max(minimumTaskSize, (int)float.Ceiling(LeafCount / (float)targetTaskCount));
        var refitContext = new RefitWithCacheOptimizationContext
        {
            SourceNodes = sourceNodes,
            Tree = this,
            LeafCountPerTask = leafCountPerTask,
            TaskStack = taskStack,
        };
        if (internallyDispatch)
        {
            taskStack->PushUnsafely(new Task(&RefitWithCacheOptimizationRootEntryTask, &refitContext), workerIndex, dispatcher);
            TaskStack.DispatchWorkers(dispatcher, taskStack, int.Min(dispatcher.ThreadCount, targetTaskCount));
        }
        else
        {
            NodeChild stub = default;
            Refit2WithCacheOptimizationAndTaskSpawning(0, -1, -1, ref stub, &refitContext, workerIndex, dispatcher);
        }
    }


    /// <summary>
    /// Refits all bounding boxes in the tree using multiple threads. Reallocates the nodes and metanodes and writes the refit tree into them in depth first traversal order.
    /// The tree instance is modified to point to the new nodes and metanodes.
    /// </summary>
    /// <param name="pool">Pool used for main thread temporary allocations during execution.</param>
    /// <param name="dispatcher">Dispatcher used during execution.</param>
    /// <param name="disposeOriginalNodes">Whether to dispose of the original nodes buffer. If false, it's up to the caller to dispose of it appropriately.</param>
    public unsafe void Refit2WithCacheOptimization(BufferPool pool, IThreadDispatcher dispatcher, bool disposeOriginalNodes = true)
    {
        if (LeafCount <= 2)
            return;
        var taskStack = new TaskStack(pool, dispatcher, dispatcher.ThreadCount);
        var oldNodes = Nodes;
        Nodes = new Buffer<Node>(oldNodes.Length, pool);
        Refit2WithCacheOptimization(pool, dispatcher, &taskStack, 0, -1, internallyDispatch: true, oldNodes);
        taskStack.Dispose(pool, dispatcher);
        if (disposeOriginalNodes)
            oldNodes.Dispose(pool);
    }

    /// <summary>
    /// Refits all bounding boxes in the tree using multiple threads. Reallocates the nodes and writes the refit tree into them in depth first traversal order.
    /// The tree instance is modified to point to the new nodes.
    /// <para/>Pushes tasks into the provided <see cref="TaskStack"/>. Does not dispatch threads internally; this is intended to be used as a part of a caller-managed dispatch.
    /// </summary>
    /// <param name="pool">Pool used for allocations during execution.</param>
    /// <param name="dispatcher">Dispatcher used during execution.</param>
    /// <param name="taskStack"><see cref="TaskStack"/> that the refit operation will push tasks onto as needed.</param>
    /// <param name="workerIndex">Index of the worker calling the function.</param>
    /// <param name="targetTaskCount">Number of tasks the refit should try to create during execution. If negative, uses <see cref="IThreadDispatcher.ThreadCount"/>.</param>
    /// <param name="disposeOriginalNodes">Whether to dispose of the original nodes buffer. If false, it's up to the caller to dispose of it appropriately.</param>
    public unsafe void Refit2WithCacheOptimization(BufferPool pool, IThreadDispatcher dispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount = -1, bool disposeOriginalNodes = true)
    {
        if (LeafCount <= 2)
            return;
        var oldNodes = Nodes;
        Nodes = new Buffer<Node>(oldNodes.Length, pool);
        Refit2WithCacheOptimization(pool, dispatcher, taskStack, workerIndex, targetTaskCount, internallyDispatch: false, oldNodes);
        if (disposeOriginalNodes)
            oldNodes.Dispose(pool);
    }

    /// <summary>
    /// Updates the bounding boxes of all internal nodes in the tree using multiple threads. The refit is based on the provided sourceNodes, and 
    /// the results are written into the tree's current <see cref="Nodes"/>, <see cref="Metanodes"/>, and <see cref="Leaves"/> buffers.
    /// The nodes and metanodes will be in depth traversal order.
    /// The input source buffer is not modified.
    /// <para/>Pushes tasks into the provided <see cref="TaskStack"/>. Does not dispatch threads internally; this is intended to be used as a part of a caller-managed dispatch.
    /// </summary>
    /// <param name="sourceNodes">Nodes to base the refit on.</param>
    /// <param name="pool">Pool used for allocations during execution.</param>
    /// <param name="dispatcher">Dispatcher used during execution.</param>
    /// <param name="taskStack"><see cref="TaskStack"/> that the refit operation will push tasks onto as needed.</param>
    /// <param name="workerIndex">Index of the worker calling the function.</param>
    /// <param name="targetTaskCount">Number of tasks the refit should try to create during execution. If negative, uses <see cref="IThreadDispatcher.ThreadCount"/>.</param>
    public unsafe void Refit2WithCacheOptimization(Buffer<Node> sourceNodes, BufferPool pool, IThreadDispatcher dispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount = -1)
    {
        Refit2WithCacheOptimization(pool, dispatcher, taskStack, workerIndex, targetTaskCount, internallyDispatch: false, sourceNodes);
    }
}
