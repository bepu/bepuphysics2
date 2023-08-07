using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuUtilities.TaskScheduling;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Reflection.Metadata;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics.Trees;

partial struct Tree
{
    private unsafe void GetOverlapsBetweenDifferentNodes2<TOverlapHandler>(ref Node a, ref Node b, ref Tree treeB, ref IntertreeContext<TOverlapHandler> context, int workerIndex, ThreadDispatcher dispatcher)
        where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {

    }

    unsafe struct IntertreeContext<TOverlapHandler> where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {
        public Tree TreeA;
        public Tree TreeB;
        public TaskStack* Stack;
        public int LeafThreshold;
        public TOverlapHandler* Results;
    }


    public static int TasksSpawned;
    static unsafe void IntertreeTask<TOverlapHandler>(long encodedIndices, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {
        var indexA = (int)encodedIndices;
        var indexB = (int)(encodedIndices >> 32);
        ref var context = ref *(IntertreeContext<TOverlapHandler>*)untypedContext;
        if (indexA >= 0 && indexB >= 0)
        {
            //Both nodes.
            ref var a = ref context.TreeA.Nodes[indexA];
            ref var b = ref context.TreeB.Nodes[indexB];
            ref var aa = ref a.A;
            ref var ab = ref a.B;
            ref var ba = ref b.A;
            ref var bb = ref b.B;
            var aaIntersects = BoundingBox.IntersectsUnsafe(aa, ba);
            var abIntersects = BoundingBox.IntersectsUnsafe(aa, bb);
            var baIntersects = BoundingBox.IntersectsUnsafe(ab, ba);
            var bbIntersects = BoundingBox.IntersectsUnsafe(ab, bb);

            //Push all pushable work *before* doing any work to ensure that other threads have something to grab.
            var pushAA = aaIntersects && int.Max(aa.LeafCount, ba.LeafCount) >= context.LeafThreshold;
            var pushAB = abIntersects && int.Max(aa.LeafCount, bb.LeafCount) >= context.LeafThreshold;
            var pushBA = baIntersects && int.Max(ab.LeafCount, ba.LeafCount) >= context.LeafThreshold;
            var pushBB = bbIntersects && int.Max(ab.LeafCount, bb.LeafCount) >= context.LeafThreshold;
            var pushCount = (pushAA ? 1 : 0) + (pushAB ? 1 : 0) + (pushBA ? 1 : 0) + (pushBB ? 1 : 0);
            ContinuationHandle handle = default;
            if (pushCount > 0)
            {
                Span<Task> tasks = stackalloc Task[pushCount];
                pushCount = 0;
                if (pushAA) tasks[pushCount++] = new Task(&IntertreeTask<TOverlapHandler>, untypedContext, ((uint)a.A.Index | ((long)b.A.Index << 32)));
                if (pushAB) tasks[pushCount++] = new Task(&IntertreeTask<TOverlapHandler>, untypedContext, ((uint)a.A.Index | ((long)b.B.Index << 32)));
                if (pushBA) tasks[pushCount++] = new Task(&IntertreeTask<TOverlapHandler>, untypedContext, ((uint)a.B.Index | ((long)b.A.Index << 32)));
                if (pushBB) tasks[pushCount++] = new Task(&IntertreeTask<TOverlapHandler>, untypedContext, ((uint)a.B.Index | ((long)b.B.Index << 32)));
                handle = context.Stack->AllocateContinuationAndPush(tasks, workerIndex, dispatcher);
                //Interlocked.Add(ref TasksSpawned, pushCount);
            }

            var wrapped = new WrappedOverlapHandler<TOverlapHandler> { Inner = context.Results, WorkerIndex = workerIndex };
            if (aaIntersects && !pushAA)
            {
                context.TreeA.DispatchTestForNodes(ref aa, ref ba, ref context.TreeB, ref wrapped);
            }
            if (abIntersects && !pushAB)
            {
                context.TreeA.DispatchTestForNodes(ref aa, ref bb, ref context.TreeB, ref wrapped);
            }
            if (baIntersects && !pushBA)
            {
                context.TreeA.DispatchTestForNodes(ref ab, ref ba, ref context.TreeB, ref wrapped);
            }
            if (bbIntersects && !pushBB)
            {
                context.TreeA.DispatchTestForNodes(ref ab, ref bb, ref context.TreeB, ref wrapped);
            }
            if (pushCount > 0)
            {
                context.Stack->WaitForCompletion(handle, workerIndex, dispatcher);
            }
        }
        else
        {
            //One of the two indices points at a leaf.
            //In reasonably balanced circumstances, there's hardly any reason to bother spawning tasks for node-leaf, but we may be in a pathological case
            //where one tree is much smaller in *leaf count*, but not in size, and so the node might actually have a ton of leaves beneath it.
            Debug.Assert(indexA >= 0 || indexB >= 0, "A task should not have been spawned for a leaf-leaf test. Leaf threshold is likely messed up.");
            var nodeBelongsToTreeA = indexA >= 0;
            ref var node = ref nodeBelongsToTreeA ? ref context.TreeA.Nodes[indexA] : ref context.TreeB.Nodes[indexB];
            //Go looking for the leaf child by using the leaf buffer indirection since we didn't store the bounding box anywhere else.
            //This hurts a bit- a whole extra bit of memory being touched- but this codepath *should* be pretty rare as a fraction of test time; the whole point is for it to hit a sequential path quickly.
            ref var leafTree = ref nodeBelongsToTreeA ? ref context.TreeB : ref context.TreeA;
            var leafIndex = nodeBelongsToTreeA ? Encode(indexB) : Encode(indexA);
            var leaf = leafTree.Leaves[leafIndex];
            ref var leafChild = ref Unsafe.Add(ref leafTree.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
            var aIntersects = BoundingBox.IntersectsUnsafe(leafChild, node.A);
            var bIntersects = BoundingBox.IntersectsUnsafe(leafChild, node.B);
            var pushA = aIntersects && node.A.LeafCount >= context.LeafThreshold;
            var pushB = bIntersects && node.B.LeafCount >= context.LeafThreshold;
            var pushCount = (pushA ? 1 : 0) + (pushB ? 1 : 0);
            ContinuationHandle handle = default;
            if (pushCount > 0)
            {
                Span<Task> tasks = stackalloc Task[pushCount];
                if (pushA) tasks[pushCount++] = new Task(&IntertreeTask<TOverlapHandler>, untypedContext, (uint)(nodeBelongsToTreeA ? node.A.Index : indexA) | ((long)(nodeBelongsToTreeA ? indexB : node.A.Index) << 32));
                if (pushB) tasks[pushCount++] = new Task(&IntertreeTask<TOverlapHandler>, untypedContext, (uint)(nodeBelongsToTreeA ? node.B.Index : indexA) | ((long)(nodeBelongsToTreeA ? indexB : node.B.Index) << 32));
                handle = context.Stack->AllocateContinuationAndPush(tasks, workerIndex, dispatcher);
            }
            var wrapped = new WrappedOverlapHandler<TOverlapHandler> { Inner = context.Results, WorkerIndex = workerIndex };
            if (aIntersects && !pushA)
            {
                context.TreeA.DispatchTestForNodes(ref nodeBelongsToTreeA ? ref node.A : ref leafChild, ref nodeBelongsToTreeA ? ref leafChild : ref node.A, ref context.TreeB, ref wrapped);
            }
            if (bIntersects && !pushB)
            {
                context.TreeA.DispatchTestForNodes(ref nodeBelongsToTreeA ? ref node.B : ref leafChild, ref nodeBelongsToTreeA ? ref leafChild : ref node.B, ref context.TreeB, ref wrapped);
            }
            if (pushCount > 0)
            {
                context.Stack->WaitForCompletion(handle, workerIndex, dispatcher);
            }
        }

    }

    public unsafe void GetOverlaps2<TOverlapHandler>(ref Tree treeB, ref TOverlapHandler results,
        BufferPool pool, int workerIndex, TaskStack* taskStack, IThreadDispatcher threadDispatcher, bool internallyDispatch, int workerCount, int targetTaskBudget) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {
        var resultsCopy = results;
        var wrapped = new WrappedOverlapHandler<TOverlapHandler> { Inner = &resultsCopy, WorkerIndex = 0 };
        if (!TryGetOverlapsForDegenerateTrees(ref treeB, ref wrapped))
        {
            //Both trees have complete nodes; we can use a general case.
            const int minimumLeafThreshold = 256;
            if (LeafCount < minimumLeafThreshold && treeB.LeafCount < minimumLeafThreshold)
            {
                //No point in spawning a bunch of tasks for tiny trees.
                GetOverlapsBetweenDifferentNodes(ref Nodes[0], ref treeB.Nodes[0], ref treeB, ref wrapped);
            }
            else
            {
                if (targetTaskBudget < 0)
                    targetTaskBudget = workerCount;
                var leafThreshold = int.Max(minimumLeafThreshold, (LeafCount + treeB.LeafCount) / (8 * targetTaskBudget));
                var context = new IntertreeContext<TOverlapHandler>
                {
                    TreeA = this,
                    TreeB = treeB,
                    Stack = taskStack,
                    LeafThreshold = leafThreshold,
                    Results = &resultsCopy
                };
                TasksSpawned = 0;
                var rootTask = new Task(&IntertreeTask<TOverlapHandler>, &context, 0);
                if (internallyDispatch)
                {
                    taskStack->AllocateContinuationAndPush(rootTask, workerIndex, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(taskStack));
                    TaskStack.DispatchWorkers(threadDispatcher, taskStack, int.Min(threadDispatcher.ThreadCount, targetTaskBudget));
                }
                else
                {
                    taskStack->Push(rootTask, workerIndex, threadDispatcher);
                }
                //Console.WriteLine($"Tasks: {TasksSpawned}");
            }
        }
        //Copy back potential changes.
        results = resultsCopy;
    }

    public unsafe void GetOverlaps2<TOverlapHandler>(ref Tree treeB, ref TOverlapHandler results, BufferPool pool, IThreadDispatcher dispatcher)
        where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {
        var taskStack = new TaskStack(pool, dispatcher, dispatcher.ThreadCount);
        GetOverlaps2(ref treeB, ref results, pool, 0, &taskStack, dispatcher, true, dispatcher.ThreadCount, -1);
        taskStack.Dispose(pool, dispatcher);
    }

    public unsafe void GetOverlaps2<TOverlapHandler>(ref Tree treeB, ref TOverlapHandler results, BufferPool pool, IThreadDispatcher dispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount = -1)
        where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {
        GetOverlaps2(ref treeB, ref results, pool, workerIndex, taskStack, dispatcher, false, dispatcher.ThreadCount, targetTaskCount);
    }
}
