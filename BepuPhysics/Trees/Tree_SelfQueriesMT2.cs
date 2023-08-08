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
    static unsafe void SelfTestTask<TOverlapHandler>(long encodedIndices, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {
        var indexA = (int)encodedIndices;
        var indexB = (int)(encodedIndices >> 32);
        ref var context = ref *(SelfTestContext<TOverlapHandler>*)untypedContext;
        if (indexA >= 0 && indexB >= 0)
        {
            //Both nodes.
            ref var a = ref context.Tree.Nodes[indexA];
            ref var b = ref context.Tree.Nodes[indexB];
            ref var aa = ref a.A;
            ref var ab = ref a.B;
            ref var ba = ref b.A;
            ref var bb = ref b.B;
            var aaIntersects = BoundingBox.IntersectsUnsafe(aa, ba);
            var abIntersects = BoundingBox.IntersectsUnsafe(aa, bb);
            var baIntersects = BoundingBox.IntersectsUnsafe(ab, ba);
            var bbIntersects = BoundingBox.IntersectsUnsafe(ab, bb);

            //Push all pushable work *before* doing any work to ensure that other threads have something to grab.
            var pushAA = aaIntersects && int.Max(aa.LeafCount, ba.LeafCount) >= context.LeafThresholdForTask;
            var pushAB = abIntersects && int.Max(aa.LeafCount, bb.LeafCount) >= context.LeafThresholdForTask;
            var pushBA = baIntersects && int.Max(ab.LeafCount, ba.LeafCount) >= context.LeafThresholdForTask;
            var pushBB = bbIntersects && int.Max(ab.LeafCount, bb.LeafCount) >= context.LeafThresholdForTask;
            var pushCount = (pushAA ? 1 : 0) + (pushAB ? 1 : 0) + (pushBA ? 1 : 0) + (pushBB ? 1 : 0);
            ContinuationHandle handle = default;
            if (pushCount > 0)
            {
                Span<Task> tasks = stackalloc Task[pushCount];
                pushCount = 0;
                if (pushAA) tasks[pushCount++] = new Task(&SelfTestTask<TOverlapHandler>, untypedContext, ((uint)a.A.Index | ((long)b.A.Index << 32)));
                if (pushAB) tasks[pushCount++] = new Task(&SelfTestTask<TOverlapHandler>, untypedContext, ((uint)a.A.Index | ((long)b.B.Index << 32)));
                if (pushBA) tasks[pushCount++] = new Task(&SelfTestTask<TOverlapHandler>, untypedContext, ((uint)a.B.Index | ((long)b.A.Index << 32)));
                if (pushBB) tasks[pushCount++] = new Task(&SelfTestTask<TOverlapHandler>, untypedContext, ((uint)a.B.Index | ((long)b.B.Index << 32)));
                handle = context.TaskStack->AllocateContinuationAndPush(tasks, workerIndex, dispatcher);
            }

            var wrapped = new WrappedOverlapHandler<TOverlapHandler> { Inner = context.Results, WorkerIndex = workerIndex };
            if (aaIntersects && !pushAA)
            {
                context.Tree.DispatchTestForNodes(ref aa, ref ba, ref wrapped);
            }
            if (abIntersects && !pushAB)
            {
                context.Tree.DispatchTestForNodes(ref aa, ref bb, ref wrapped);
            }
            if (baIntersects && !pushBA)
            {
                context.Tree.DispatchTestForNodes(ref ab, ref ba, ref wrapped);
            }
            if (bbIntersects && !pushBB)
            {
                context.Tree.DispatchTestForNodes(ref ab, ref bb, ref wrapped);
            }
            if (pushCount > 0)
            {
                context.TaskStack->WaitForCompletion(handle, workerIndex, dispatcher);
            }
        }
        else
        {
            //One of the two indices points at a leaf.
            //In reasonably balanced circumstances, there's hardly any reason to bother spawning tasks for node-leaf, but we may be in a pathological case
            //where one tree is much smaller in *leaf count*, but not in size, and so the node might actually have a ton of leaves beneath it.
            Debug.Assert(indexA >= 0 || indexB >= 0, "A task should not have been spawned for a leaf-leaf test. Leaf threshold is likely messed up.");
            var aIsNode = indexA >= 0;
            ref var node = ref aIsNode ? ref context.Tree.Nodes[indexA] : ref context.Tree.Nodes[indexB];
            //Go looking for the leaf child by using the leaf buffer indirection since we didn't store the bounding box anywhere else.
            //This hurts a bit- a whole extra bit of memory being touched- but this codepath *should* be pretty rare as a fraction of test time; the whole point is for it to hit a sequential path quickly.
            var leafIndex = aIsNode ? Encode(indexB) : Encode(indexA);
            var leaf = context.Tree.Leaves[leafIndex];
            ref var leafChild = ref Unsafe.Add(ref context.Tree.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
            var aIntersects = BoundingBox.IntersectsUnsafe(leafChild, node.A);
            var bIntersects = BoundingBox.IntersectsUnsafe(leafChild, node.B);
            var pushA = aIntersects && node.A.LeafCount >= context.LeafThresholdForTask;
            var pushB = bIntersects && node.B.LeafCount >= context.LeafThresholdForTask;
            var pushCount = (pushA ? 1 : 0) + (pushB ? 1 : 0);
            ContinuationHandle handle = default;
            if (pushCount > 0)
            {
                Span<Task> tasks = stackalloc Task[pushCount];
                pushCount = 0;
                if (pushA) tasks[pushCount++] = new Task(&SelfTestTask<TOverlapHandler>, untypedContext, (uint)(aIsNode ? node.A.Index : indexA) | ((long)(aIsNode ? indexB : node.A.Index) << 32));
                if (pushB) tasks[pushCount++] = new Task(&SelfTestTask<TOverlapHandler>, untypedContext, (uint)(aIsNode ? node.B.Index : indexA) | ((long)(aIsNode ? indexB : node.B.Index) << 32));
                handle = context.TaskStack->AllocateContinuationAndPush(tasks, workerIndex, dispatcher);
            }
            var wrapped = new WrappedOverlapHandler<TOverlapHandler> { Inner = context.Results, WorkerIndex = workerIndex };
            if (aIntersects && !pushA)
            {
                context.Tree.DispatchTestForNodes(ref aIsNode ? ref node.A : ref leafChild, ref aIsNode ? ref leafChild : ref node.A, ref wrapped);
            }
            if (bIntersects && !pushB)
            {
                context.Tree.DispatchTestForNodes(ref aIsNode ? ref node.B : ref leafChild, ref aIsNode ? ref leafChild : ref node.B, ref wrapped);
            }
            if (pushCount > 0)
            {
                context.TaskStack->WaitForCompletion(handle, workerIndex, dispatcher);
            }
        }

    }

    unsafe static void LoopEntryTaskWithSubtasks4<TOverlapHandler>(long taskStartAndEnd, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher) where TOverlapHandler : unmanaged, IThreadedOverlapHandler
    {
        var taskStart = (int)taskStartAndEnd;
        var taskEnd = (int)(taskStartAndEnd >> 32);
        ref var context = ref *(SelfTestContext<TOverlapHandler>*)untypedContext;
        var wrapped = new WrappedOverlapHandler<TOverlapHandler> { Inner = context.Results, WorkerIndex = workerIndex };
        var nodes = context.Tree.Nodes;
        var pool = dispatcher.WorkerPools[workerIndex];
        var continuationCountEstimate = int.Max(32, (taskEnd - taskStart) / 64);
        var subtaskContinuations = new QuickList<ContinuationHandle>(continuationCountEstimate, pool);
        //While reverse execution tends to be faster for access pattern reasons, the largest nodes tend to come early. We'd like to submit tasks for them early.
        //TODO: You may want to try tiled execution- do a forward pass for tasks up to N sequential nodes, then flip and do all single threaded dispatches in reverse.
        //for (int i = taskEnd - 1; i >= taskStart; --i)
        for (int i = taskStart; i < taskEnd; ++i)
        {
            ref var node = ref nodes[i];
            ref var a = ref node.A;
            ref var b = ref node.B;
            var ab = BoundingBox.IntersectsUnsafe(a, b);
            if (ab)
            {
                if (int.Max(a.LeafCount, b.LeafCount) >= context.LeafThresholdForTask)
                {
                    //The number of potential overlaps is high; push this pair to a subtask.
                    //Note that the leaf threshold guarantees at least one of the children is an internal node.
                    Debug.Assert(context.LeafThresholdForTask > 1);
                    subtaskContinuations.Allocate(pool) = context.TaskStack->AllocateContinuationAndPush(new Task(&SelfTestTask<TOverlapHandler>, untypedContext, (uint)a.Index | ((long)b.Index << 32)), workerIndex, dispatcher);
                }
                else
                {
                    context.Tree.DispatchTestForNodes(ref a, ref b, ref wrapped);
                }
            }
        }
        //if(subtaskContinuations.Count > 0)
        //{
        //    Console.WriteLine($"Start {taskStart}: {subtaskContinuations.Count}");
        //}
        for (int i = 0; i < subtaskContinuations.Count; ++i)
        {
            context.TaskStack->WaitForCompletion(subtaskContinuations[i], workerIndex, dispatcher);
        }
        subtaskContinuations.Dispose(pool);

    }

}
