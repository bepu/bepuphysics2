using BepuPhysics.CollisionDetection;
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
    static long EncodeJobId(int a, int b) => (long)a << 32 | (uint)b;
    static (int a, int b) DecodeJobId(long id) => ((int)(id >> 32), (int)id);
    public unsafe class MultithreadedIntertreeTest2<TOverlapHandler> where TOverlapHandler : struct, IOverlapHandler
    {
        public BufferPool Pool;
        public TaskStack* Stack;
        public delegate*<long, void*, int, IThreadDispatcher, void> TaskFunction;
        int leafThreshold;
        public Tree TreeA;
        public Tree TreeB;
        public TOverlapHandler[] OverlapHandlers;

        /// <summary>
        /// Prepares the jobs associated with a self test. Must be called before a dispatch over PairTest.
        /// </summary>
        /// <param name="overlapHandlers">Callbacks used to handle individual overlaps detected by the self test.</param>
        /// <param name="threadCount">Number of threads to prepare jobs for.</param>
        /// <param name="workerIndex">Index of the worker executing the preparation job.</param>
        /// <param name="pool">Pool to allocate from.</param>
        /// <param name="taskStack">Task stack to push jobs into.</param>
        /// <param name="taskFunction">Function to be invoked by tasks pushed onto the stack.</param>
        public void PushRootTasks(ref Tree treeA, ref Tree treeB, TOverlapHandler[] overlapHandlers, int threadCount, int workerIndex, IThreadDispatcher dispatcher, BufferPool pool,
            TaskStack* taskStack, delegate*<long, void*, int, IThreadDispatcher, void> taskFunction)
        {
            Pool = pool;
            Stack = taskStack;
            TaskFunction = taskFunction;
            if (treeA.LeafCount == 0 || treeB.LeafCount == 0)
            {
                //If either tree has zero leaves, no intertree test is required.
                //Since this context has a count property for scheduling purposes that reads the jobs list, clear it to ensure no spurious jobs are executed. 
                return;
            }
            Debug.Assert(overlapHandlers.Length >= threadCount);
            const float jobMultiplier = 8f;
            var targetJobCount = Math.Max(1, jobMultiplier * threadCount);
            //TODO: Not a lot of thought was put into this leaf threshold for intertree. Probably better options.
            leafThreshold = (int)((treeA.LeafCount + treeB.LeafCount) / targetJobCount);
            this.OverlapHandlers = overlapHandlers;
            this.TreeA = treeA;
            this.TreeB = treeB;
            //Collect jobs.
            ref var handler = ref OverlapHandlers[workerIndex];
            if (treeA.LeafCount >= 2 && treeB.LeafCount >= 2)
            {
                //Both trees have complete nodes; we can use a general case.
                GetJobsBetweenDifferentNodes(ref treeA.Nodes[0], ref treeB.Nodes[0], workerIndex, dispatcher, ref handler);
            }
            else if (treeA.LeafCount == 1 && treeB.LeafCount >= 2)
            {
                //Tree A is degenerate; needs a special case.
                ref var a = ref treeA.Nodes[0];
                ref var b = ref treeB.Nodes[0];
                var aaIntersects = BoundingBox.IntersectsUnsafe(a.A, b.A);
                var abIntersects = BoundingBox.IntersectsUnsafe(a.A, b.B);
                if (aaIntersects)
                {
                    DispatchTestForNodes(ref a.A, ref b.A, workerIndex, dispatcher, ref handler);
                }
                if (abIntersects)
                {
                    DispatchTestForNodes(ref a.A, ref b.B, workerIndex, dispatcher, ref handler);
                }
            }
            else if (treeA.LeafCount >= 2 && treeB.LeafCount == 1)
            {
                //Tree B is degenerate; needs a special case.
                ref var a = ref treeA.Nodes[0];
                ref var b = ref treeB.Nodes[0];
                var aaIntersects = BoundingBox.IntersectsUnsafe(a.A, b.A);
                var baIntersects = BoundingBox.IntersectsUnsafe(a.B, b.A);
                if (aaIntersects)
                {
                    DispatchTestForNodes(ref a.A, ref b.A, workerIndex, dispatcher, ref handler);
                }
                if (baIntersects)
                {
                    DispatchTestForNodes(ref a.B, ref b.A, workerIndex, dispatcher, ref handler);
                }
            }
            else
            {
                Debug.Assert(treeA.LeafCount == 1 && treeB.LeafCount == 1);
                if (BoundingBox.IntersectsUnsafe(treeA.Nodes[0].A, treeB.Nodes[0].A))
                {
                    DispatchTestForNodes(ref treeA.Nodes[0].A, ref treeB.Nodes[0].A, workerIndex, dispatcher, ref handler);
                }
            }

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
            TreeA = default;
            TreeB = default;
        }

        /// <summary>
        /// Executes a an intertree test job given by an encoded job id.
        /// </summary>
        /// <param name="encodedJobId">Encoded job id.</param>
        /// <param name="workerIndex">Index of the worker executing the job.</param>
        public void ExecuteJob(long encodedJobId, int workerIndex)
        {
            var (a, b) = DecodeJobId(encodedJobId);
            if (a >= 0)
            {
                if (b >= 0)
                {
                    //Different internal nodes.
                    TreeA.GetOverlapsBetweenDifferentNodes(ref TreeA.Nodes[a], ref TreeB.Nodes[b], ref TreeB, ref OverlapHandlers[workerIndex]);
                }
                else
                {
                    //A is an internal node, B is a leaf.
                    var leafIndex = Encode(b);
                    ref var leaf = ref TreeB.Leaves[leafIndex];
                    ref var childOwningLeaf = ref Unsafe.Add(ref TreeB.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                    TreeA.TestNodeAgainstLeaf(a, leafIndex, ref childOwningLeaf, ref OverlapHandlers[workerIndex]);
                }
            }
            else
            {
                //A is a leaf, B is internal.
                var leafIndex = Encode(a);
                ref var leaf = ref TreeA.Leaves[leafIndex];
                ref var childOwningLeaf = ref Unsafe.Add(ref TreeA.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                TreeA.TestLeafAgainstNode(leafIndex, ref childOwningLeaf, b, ref TreeB, ref OverlapHandlers[workerIndex]);

                //NOTE THAT WE DO NOT HANDLE THE CASE THAT BOTH A AND B ARE LEAVES HERE.
                //The collection routine should take care of that, since it has more convenient access to bounding boxes and because a single test isn't worth an atomic increment.
            }
        }

        void DispatchTestForLeaf(ref Tree nodeOwner, int leafIndex, ref NodeChild leafChild, int nodeIndex, int nodeLeafCount, int workerIndex, IThreadDispatcher dispatcher, ref TOverlapHandler results)
        {
            if (nodeIndex < 0)
            {
                //Maintain the order of trees. Leaves from tree A should always be the first parameter.
                if (Tree.Equals(nodeOwner, TreeA))
                    results.Handle(Encode(nodeIndex), leafIndex);
                else
                    results.Handle(leafIndex, Encode(nodeIndex));
            }
            else
            {
                if (nodeLeafCount <= leafThreshold)
                {
                    //Maintain the order of trees. Leaves from tree A should always be the first parameter.
                    var jobId = Tree.Equals(nodeOwner, TreeA) ? EncodeJobId(nodeIndex, Encode(leafIndex)) : EncodeJobId(Encode(leafIndex), nodeIndex);
                    if (jobId == -1)
                        Console.WriteLine("hm");
                    Stack->Push(new Task(TaskFunction, null, jobId), workerIndex, dispatcher);
                }
                else
                    TestLeafAgainstNode(ref nodeOwner, leafIndex, ref leafChild, nodeIndex, workerIndex, dispatcher, ref results);
            }
        }

        void TestLeafAgainstNode(ref Tree nodeOwner, int leafIndex, ref NodeChild leafChild, int nodeIndex, int workerIndex, IThreadDispatcher dispatcher, ref TOverlapHandler results)
        {
            ref var node = ref nodeOwner.Nodes[nodeIndex];
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
                DispatchTestForLeaf(ref nodeOwner, leafIndex, ref leafChild, a.Index, a.LeafCount, workerIndex, dispatcher, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForLeaf(ref nodeOwner, leafIndex, ref leafChild, bIndex, bLeafCount, workerIndex, dispatcher, ref results);
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
                    {
                        if (EncodeJobId(a.Index, b.Index) == -1)
                            Console.WriteLine("hm");
                        Stack->Push(new Task(TaskFunction, null, EncodeJobId(a.Index, b.Index)), workerIndex, dispatcher);
                    }
                    else
                        GetJobsBetweenDifferentNodes(ref TreeA.Nodes[a.Index], ref TreeB.Nodes[b.Index], workerIndex, dispatcher, ref results);

                }
                else
                {
                    //leaf B versus node A.
                    TestLeafAgainstNode(ref TreeA, Encode(b.Index), ref b, a.Index, workerIndex, dispatcher, ref results);
                }
            }
            else if (b.Index >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode(ref TreeB, Encode(a.Index), ref a, b.Index, workerIndex, dispatcher, ref results);
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
    }

}
