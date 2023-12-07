using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        public class MultithreadedIntertreeTest<TOverlapHandler> where TOverlapHandler : struct, IOverlapHandler
        {
            struct Job
            {
                public int A;
                public int B;
            }

            public BufferPool Pool;

            int NextNodePair;
            int leafThreshold;
            private QuickList<Job> jobs;
            public int JobCount => jobs.Count;
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
            public void PrepareJobs(ref Tree treeA, ref Tree treeB, TOverlapHandler[] overlapHandlers, int threadCount, int workerIndex, BufferPool pool)
            {
                Pool = pool;
                if (treeA.LeafCount == 0 || treeB.LeafCount == 0)
                {
                    //If either tree has zero leaves, no intertree test is required.
                    //Since this context has a count property for scheduling purposes that reads the jobs list, clear it to ensure no spurious jobs are executed. 
                    jobs = new QuickList<Job>();
                    return;
                }
                Debug.Assert(overlapHandlers.Length >= threadCount);
                const float jobMultiplier = 8f;
                var targetJobCount = Math.Max(1, jobMultiplier * threadCount);
                //TODO: Not a lot of thought was put into this leaf threshold for intertree. Probably better options.
                leafThreshold = (int)((treeA.LeafCount + treeB.LeafCount) / targetJobCount);
                jobs = new QuickList<Job>((int)(targetJobCount * 2), Pool);
                NextNodePair = -1;
                this.OverlapHandlers = overlapHandlers;
                this.TreeA = treeA;
                this.TreeB = treeB;
                //Collect jobs.
                ref var handler = ref OverlapHandlers[workerIndex];
                if (treeA.LeafCount >= 2 && treeB.LeafCount >= 2)
                {
                    //Both trees have complete nodes; we can use a general case.
                    GetJobsBetweenDifferentNodes(ref treeA.Nodes[0], ref treeB.Nodes[0], ref handler);
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
                        DispatchTestForNodes(ref a.A, ref b.A, ref handler);
                    }
                    if (abIntersects)
                    {
                        DispatchTestForNodes(ref a.A, ref b.B, ref handler);
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
                        DispatchTestForNodes(ref a.A, ref b.A, ref handler);
                    }
                    if (baIntersects)
                    {
                        DispatchTestForNodes(ref a.B, ref b.A, ref handler);
                    }
                }
                else
                {
                    Debug.Assert(treeA.LeafCount == 1 && treeB.LeafCount == 1);
                    if (BoundingBox.IntersectsUnsafe(treeA.Nodes[0].A, treeB.Nodes[0].A))
                    {
                        DispatchTestForNodes(ref treeA.Nodes[0].A, ref treeB.Nodes[0].A, ref handler);
                    }
                }

            }

            /// <summary>
            /// Cleans up after a multithreaded self test. Returns resources to the pool used by <see cref="PrepareJobs"/>.
            /// </summary>
            public void CompleteTest()
            {
                //Note that we don't allocate a job list if there aren't any jobs.
                if (jobs.Span.Allocated)
                    jobs.Dispose(Pool);
                Pool = null;
            }

            public void ExecuteJob(int jobIndex, int workerIndex)
            {
                ref var overlap = ref jobs[jobIndex];
                if (overlap.A >= 0)
                {
                    if (overlap.B >= 0)
                    {
                        //Different internal nodes.
                        TreeA.GetOverlapsBetweenDifferentNodes(ref TreeA.Nodes[overlap.A], ref TreeB.Nodes[overlap.B], ref TreeB, ref OverlapHandlers[workerIndex]);
                    }
                    else
                    {
                        //A is an internal node, B is a leaf.
                        var leafIndex = Encode(overlap.B);
                        ref var leaf = ref TreeB.Leaves[leafIndex];
                        ref var childOwningLeaf = ref Unsafe.Add(ref TreeB.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                        TreeA.TestNodeAgainstLeaf(overlap.A, leafIndex, ref childOwningLeaf, ref OverlapHandlers[workerIndex]);
                    }
                }
                else
                {
                    //A is a leaf, B is internal.
                    var leafIndex = Encode(overlap.A);
                    ref var leaf = ref TreeA.Leaves[leafIndex];
                    ref var childOwningLeaf = ref Unsafe.Add(ref TreeA.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                    TreeA.TestLeafAgainstNode(leafIndex, ref childOwningLeaf, overlap.B, ref TreeB, ref OverlapHandlers[workerIndex]);

                    //NOTE THAT WE DO NOT HANDLE THE CASE THAT BOTH A AND B ARE LEAVES HERE.
                    //The collection routine should take care of that, since it has more convenient access to bounding boxes and because a single test isn't worth an atomic increment.
                }
            }
            /// <summary>
            /// Executes a single worker of the multithreaded self test.
            /// </summary>
            /// <param name="workerIndex">Index of the worker executing this set of tests.</param>
            public void PairTest(int workerIndex)
            {
                Debug.Assert(workerIndex >= 0 && workerIndex < OverlapHandlers.Length);
                int nextNodePairIndex;
                //To minimize the number of worker overlap lists, perform direct load balancing by manually grabbing the next indices.
                while ((nextNodePairIndex = Interlocked.Increment(ref NextNodePair)) < jobs.Count)
                {
                    ExecuteJob(nextNodePairIndex, workerIndex);
                }
            }

            void DispatchTestForLeaf(ref Tree nodeOwner, int leafIndex, ref NodeChild leafChild, int nodeIndex, int nodeLeafCount, ref TOverlapHandler results)
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
                        if (Tree.Equals(nodeOwner, TreeA))
                            jobs.Add(new Job { B = Encode(leafIndex), A = nodeIndex }, Pool);
                        else
                            jobs.Add(new Job { A = Encode(leafIndex), B = nodeIndex }, Pool);
                    }
                    else
                        TestLeafAgainstNode(ref nodeOwner, leafIndex, ref leafChild, nodeIndex, ref results);
                }
            }

            void TestLeafAgainstNode(ref Tree nodeOwner, int leafIndex, ref NodeChild leafChild, int nodeIndex, ref TOverlapHandler results)
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
                    DispatchTestForLeaf(ref nodeOwner, leafIndex, ref leafChild, a.Index, a.LeafCount, ref results);
                }
                if (bIntersects)
                {
                    DispatchTestForLeaf(ref nodeOwner, leafIndex, ref leafChild, bIndex, bLeafCount, ref results);
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            void DispatchTestForNodes(ref NodeChild a, ref NodeChild b, ref TOverlapHandler results)
            {
                if (a.Index >= 0)
                {
                    if (b.Index >= 0)
                    {
                        if (a.LeafCount + b.LeafCount <= leafThreshold)
                            jobs.Add(new Job { A = a.Index, B = b.Index }, Pool);
                        else
                            GetJobsBetweenDifferentNodes(ref TreeA.Nodes[a.Index], ref TreeB.Nodes[b.Index], ref results);

                    }
                    else
                    {
                        //leaf B versus node A.
                        TestLeafAgainstNode(ref TreeA, Encode(b.Index), ref b, a.Index, ref results);
                    }
                }
                else if (b.Index >= 0)
                {
                    //leaf A versus node B.
                    TestLeafAgainstNode(ref TreeB, Encode(a.Index), ref a, b.Index, ref results);
                }
                else
                {
                    //Two leaves.
                    results.Handle(Encode(a.Index), Encode(b.Index));
                }
            }

            void GetJobsBetweenDifferentNodes(ref Node a, ref Node b, ref TOverlapHandler results)
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
                    DispatchTestForNodes(ref aa, ref ba, ref results);
                }
                if (abIntersects)
                {
                    DispatchTestForNodes(ref aa, ref bb, ref results);
                }
                if (baIntersects)
                {
                    DispatchTestForNodes(ref ab, ref ba, ref results);
                }
                if (bbIntersects)
                {
                    DispatchTestForNodes(ref ab, ref bb, ref results);
                }

            }
        }

    }
}
