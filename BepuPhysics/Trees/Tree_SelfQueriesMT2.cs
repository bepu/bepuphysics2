using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuUtilities.TaskScheduling;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics.Trees;

partial struct Tree
{
    public unsafe class MultithreadedSelfTest2<TOverlapHandler> where TOverlapHandler : struct, IOverlapHandler
    {
        public BufferPool Pool;
        public TaskStack* Stack;
        public delegate*<long, void*, int, IThreadDispatcher, void> TaskFunction;
        int leafThreshold;
        public Tree Tree;
        public TOverlapHandler[] OverlapHandlers;

        /// <summary>
        /// Prepares the jobs associated with a self test. Must be called before a dispatch over PairTest.
        /// </summary>
        /// <param name="tree">Tree to test against itself.</param>
        /// <param name="overlapHandlers">Callbacks used to handle individual overlaps detected by the self test.</param>
        /// <param name="threadCount">Number of threads to prepare jobs for.</param>
        /// <param name="workerIndex">Index of the worker executing the preparation job.</param>
        /// <param name="pool">Pool to allocate from.</param>
        public void PushRootTasks(ref Tree tree, TOverlapHandler[] overlapHandlers, int threadCount, int workerIndex, IThreadDispatcher dispatcher, BufferPool pool,
              TaskStack* taskStack, delegate*<long, void*, int, IThreadDispatcher, void> taskFunction)
        {
            Pool = pool;
            Stack = taskStack;
            TaskFunction = taskFunction;
            //If there are not multiple children, there's no need to recurse.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (tree.LeafCount < 2)
            {
                return;
            }
            Debug.Assert(overlapHandlers.Length >= threadCount);
            const float jobMultiplier = 8f;
            var targetJobCount = Math.Max(1, jobMultiplier * threadCount);
            leafThreshold = (int)(tree.LeafCount / targetJobCount);
            this.OverlapHandlers = overlapHandlers;
            this.Tree = tree;
            //Collect jobs.
            CollectJobsInNode(0, tree.LeafCount, workerIndex, dispatcher, ref OverlapHandlers[workerIndex]);
        }

        /// <summary>
        /// Cleans up after a multithreaded self test.
        /// </summary>
        public void CompleteTest()
        {
            Pool = null;
            Stack = null;
            TaskFunction = null;
            OverlapHandlers = null;
            Tree = default;
        }

        public void ExecuteJob(long encodedJobId, int workerIndex)
        {
            var (a, b) = DecodeJobId(encodedJobId);
            if (a >= 0)
            {
                if (a == b)
                {
                    //Same node.
                    Tree.GetOverlapsInNode(ref Tree.Nodes[a], ref OverlapHandlers[workerIndex]);
                }
                else if (b >= 0)
                {
                    //Different nodes.
                    Tree.GetOverlapsBetweenDifferentNodes(ref Tree.Nodes[a], ref Tree.Nodes[b], ref OverlapHandlers[workerIndex]);
                }
                else
                {
                    //A is an internal node, B is a leaf.
                    var leafIndex = Encode(b);
                    ref var leaf = ref Tree.Leaves[leafIndex];
                    ref var childOwningLeaf = ref Unsafe.Add(ref Tree.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                    Tree.TestLeafAgainstNode(leafIndex, ref childOwningLeaf, a, ref OverlapHandlers[workerIndex]);
                }
            }
            else
            {
                //A is a leaf, B is internal.
                var leafIndex = Encode(a);
                ref var leaf = ref Tree.Leaves[leafIndex];
                ref var childOwningLeaf = ref Unsafe.Add(ref Tree.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                Tree.TestLeafAgainstNode(leafIndex, ref childOwningLeaf, b, ref OverlapHandlers[workerIndex]);

                //NOTE THAT WE DO NOT HANDLE THE CASE THAT BOTH A AND B ARE LEAVES HERE.
                //The collection routine should take care of that, since it has more convenient access to bounding boxes and because a single test isn't worth an atomic increment.
            }
        }

        void DispatchTestForLeaf(int leafIndex, ref NodeChild leafChild, int nodeIndex, int nodeLeafCount, int workerIndex, IThreadDispatcher dispatcher, ref TOverlapHandler results)
        {
            if (nodeIndex < 0)
            {
                results.Handle(leafIndex, Encode(nodeIndex));
            }
            else
            {
                if (nodeLeafCount <= leafThreshold)
                    Stack->Push(new Task(TaskFunction, null, EncodeJobId(Encode(leafIndex), nodeIndex)), workerIndex, dispatcher);
                else
                    TestLeafAgainstNode(leafIndex, ref leafChild, nodeIndex, workerIndex, dispatcher, ref results);
            }
        }

        void TestLeafAgainstNode(int leafIndex, ref NodeChild leafChild, int nodeIndex, int workerIndex, IThreadDispatcher dispatcher, ref TOverlapHandler results)
        {
            ref var node = ref Tree.Nodes[nodeIndex];
            ref var a = ref node.A;
            ref var b = ref node.B;
            //Despite recursion, leafBounds should remain in L1- it'll be used all the way down the recursion from here.
            //However, while we likely loaded child B when we loaded child A, there's no guarantee that it will stick around.
            //Reloading that in the event of eviction would require more work than keeping the derived data on the stack.
            //TODO: this is some pretty questionable microtuning. It's not often that the post-leaf-found recursion will be long enough to evict L1. Definitely test it.
            var bIndex = b.Index;
            var bLeafCount = b.LeafCount;
            var aIntersects = BoundingBox.IntersectsUnsafe(leafChild, a);
            var bIntersects = BoundingBox.IntersectsUnsafe(leafChild, b);
            if (aIntersects)
            {
                DispatchTestForLeaf(leafIndex, ref leafChild, a.Index, a.LeafCount, workerIndex, dispatcher, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForLeaf(leafIndex, ref leafChild, bIndex, bLeafCount, workerIndex, dispatcher, ref results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void DispatchTestForNodes(ref NodeChild a, ref NodeChild b, int workerIndex, IThreadDispatcher dispatcher, ref TOverlapHandler results)
        {
            if (a.Index >= 0)
            {
                if (b.Index >= 0)
                {
                    if (a.LeafCount + b.LeafCount <= leafThreshold)
                        Stack->Push(new Task(TaskFunction, null, EncodeJobId(a.Index, b.Index)), workerIndex, dispatcher);
                    else
                        GetJobsBetweenDifferentNodes(ref Tree.Nodes[a.Index], ref Tree.Nodes[b.Index], workerIndex, dispatcher, ref results);

                }
                else
                {
                    //leaf B versus node A.
                    TestLeafAgainstNode(Encode(b.Index), ref b, a.Index, workerIndex, dispatcher, ref results);
                }
            }
            else if (b.Index >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode(Encode(a.Index), ref a, b.Index, workerIndex, dispatcher, ref results);
            }
            else
            {
                //Two leaves.
                results.Handle(Encode(a.Index), Encode(b.Index));
            }
        }

        void GetJobsBetweenDifferentNodes(ref Node a, ref Node b, int workerIndex, IThreadDispatcher dispatcher, ref TOverlapHandler results)
        {
            //There are no shared children, so test them all.

            ref var aa = ref a.A;
            ref var ab = ref a.B;
            ref var ba = ref b.A;
            ref var bb = ref b.B;
            var aaIntersects = BoundingBox.IntersectsUnsafe(aa, ba);
            var abIntersects = BoundingBox.IntersectsUnsafe(aa, bb);
            var baIntersects = BoundingBox.IntersectsUnsafe(ab, ba);
            var bbIntersects = BoundingBox.IntersectsUnsafe(ab, bb);

            if (aaIntersects)
            {
                DispatchTestForNodes(ref aa, ref ba, workerIndex, dispatcher, ref results);
            }
            if (abIntersects)
            {
                DispatchTestForNodes(ref aa, ref bb, workerIndex, dispatcher, ref results);
            }
            if (baIntersects)
            {
                DispatchTestForNodes(ref ab, ref ba, workerIndex, dispatcher, ref results);
            }
            if (bbIntersects)
            {
                DispatchTestForNodes(ref ab, ref bb, workerIndex, dispatcher, ref results);
            }

        }

        void CollectJobsInNode(int nodeIndex, int leafCount, int workerIndex, IThreadDispatcher dispatcher, ref TOverlapHandler results)
        {
            if (leafCount <= leafThreshold)
            {
                Stack->Push(new Task(TaskFunction, null, EncodeJobId(nodeIndex, nodeIndex)), workerIndex, dispatcher);
                return;
            }

            ref var node = ref Tree.Nodes[nodeIndex];
            ref var a = ref node.A;
            ref var b = ref node.B;

            var ab = BoundingBox.IntersectsUnsafe(a, b);

            if (a.Index >= 0)
                CollectJobsInNode(a.Index, a.LeafCount, workerIndex, dispatcher, ref results);
            if (b.Index >= 0)
                CollectJobsInNode(b.Index, b.LeafCount, workerIndex, dispatcher, ref results);

            if (ab)
            {
                DispatchTestForNodes(ref a, ref b, workerIndex, dispatcher, ref results);
            }

        }
    }
}
