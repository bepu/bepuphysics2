using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics.Trees;

partial struct Tree
{
    public struct NodePair
    {
        public int A;
        public int B;
    }
    static float ComputePriorityHeuristic(ref NodeChild a, ref NodeChild b)
    {
        //Use an overlapped volume heuristic to guess how many leaves might be in the overlapped region.
        //Note that the 'volume' we use is sum of span axes; this plays more nicely with thin bounds.
        var min = Vector3.Max(a.Min, b.Min);
        var max = Vector3.Min(a.Max, b.Max);
        var intersection = Vector3.Max(Vector3.Zero, max - min);
        var intersectionSpace = Vector3.Dot(Vector3.One, intersection);
        var aSpace = Vector3.Dot(Vector3.One, a.Max - a.Min);
        var bSpace = Vector3.Dot(Vector3.One, b.Max - b.Min);
        if (aSpace == 0 || bSpace == 0)
            return a.LeafCount + b.LeafCount;
        return intersectionSpace * (a.LeafCount / aSpace + b.LeafCount / bSpace);

    }
    static float ComputePriorityHeuristicForSelfTest(int leafCountA, int leafCountB)
    {
        return leafCountA + leafCountB;
    }
    /// <summary>
    /// Prepares jobs for a multithreaded tree self overlap test.
    /// </summary>
    /// <typeparam name="TOverlapHandler">Type of the overlap handler to consume any overlapping leaf node pairs identified during the job preparation traversal.</typeparam>
    /// <param name="pool"><see cref="BufferPool"/> used for any temporary alloations.</param>
    /// <param name="targetJobCount">Number of jobs requested.</param>
    /// <param name="jobNodePairs">Initially empty binary heap that will be used to hold the jobs.</param>
    /// <param name="overlapHandler">Overlap handler to consume any overlapping leaf node pairs identified during the job preparation traversal.</param>
    public readonly void GetSelfOverlapJobs<TOverlapHandler>(BufferPool pool, int targetJobCount, ref BinaryHeap<NodePair> jobNodePairs, ref TOverlapHandler overlapHandler)
        where TOverlapHandler : struct, IOverlapHandler
    {
        if (LeafCount <= 1)
            return;
        Debug.Assert(jobNodePairs.Entries.Length >= targetJobCount + jobNodePairs.Count, "List should be able to hold all additional jobs identified during the traversal.");
        var pair = new NodePair { A = 0, B = 0 };
        while (true)
        {
            ref var a = ref Nodes[pair.A];
            ref var b = ref Nodes[pair.B];
            if (pair.A == pair.B)
            {
                //Same node; each child is guaranteed to be pushed, and the overlap between the children is conditionally pushed.
                jobNodePairs.Insert(new NodePair { A = a.A.Index, B = a.A.Index }, );
            }
            else
            {
            }

            if (jobNodePairs.Count == 0)
                break;
        }

        nodePairsToProcess.Dispose(pool);
    }

    public class MultithreadedSelfTest4<TOverlapHandler> where TOverlapHandler : struct, IOverlapHandler
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
        public void PrepareJobs(ref Tree tree, TOverlapHandler[] overlapHandlers, int threadCount, int workerIndex, BufferPool pool)
        {
            Pool = pool;
            //If there are not multiple children, there's no need to recurse.
            //This provides a guarantee that there are at least 2 children in each internal node considered by GetOverlapsInNode.
            if (tree.LeafCount < 2)
            {
                //We clear it out to avoid keeping any old job counts. The count property is used for scheduling, so incorrect values could break the job scheduler.
                jobs = new QuickList<Job>();
                return;
            }
            Debug.Assert(overlapHandlers.Length >= threadCount);
            const float jobMultiplier = 4f;
            var targetJobCount = Math.Max(1, jobMultiplier * threadCount);
            leafThreshold = (int)(tree.LeafCount / targetJobCount);
            jobs = new QuickList<Job>((int)(targetJobCount * 2), Pool);
            NextNodePair = -1;
            this.OverlapHandlers = overlapHandlers;
            this.Tree = tree;
            //Collect jobs.
            CollectJobsInNode(0, tree.LeafCount, ref OverlapHandlers[workerIndex]);
        }

        /// <summary>
        /// Cleans up after a multithreaded self test. Returns resources to the pool used by <see cref="PrepareJobs"/>.
        /// </summary>
        public void CompleteTest()
        {
            //Note that a tree with 0 or 1 entries won't have any jobs.
            if (jobs.Span.Allocated)
                jobs.Dispose(Pool);
            Pool = null;
        }

        public void ExecuteJob(int jobIndex, int workerIndex)
        {
            ref var overlap = ref jobs[jobIndex];
            if (overlap.A >= 0)
            {
                if (overlap.A == overlap.B)
                {
                    //Same node.
                    Tree.GetOverlapsInNode(ref Tree.Nodes[overlap.A], ref OverlapHandlers[workerIndex]);
                }
                else if (overlap.B >= 0)
                {
                    //Different nodes.
                    Tree.GetOverlapsBetweenDifferentNodes(ref Tree.Nodes[overlap.A], ref Tree.Nodes[overlap.B], ref OverlapHandlers[workerIndex]);
                }
                else
                {
                    //A is an internal node, B is a leaf.
                    var leafIndex = Encode(overlap.B);
                    ref var leaf = ref Tree.Leaves[leafIndex];
                    ref var childOwningLeaf = ref Unsafe.Add(ref Tree.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                    Tree.TestLeafAgainstNode(leafIndex, ref childOwningLeaf, overlap.A, ref OverlapHandlers[workerIndex]);
                }
            }
            else
            {
                //A is a leaf, B is internal.
                var leafIndex = Encode(overlap.A);
                ref var leaf = ref Tree.Leaves[leafIndex];
                ref var childOwningLeaf = ref Unsafe.Add(ref Tree.Nodes[leaf.NodeIndex].A, leaf.ChildIndex);
                Tree.TestLeafAgainstNode(leafIndex, ref childOwningLeaf, overlap.B, ref OverlapHandlers[workerIndex]);

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

        void DispatchTestForLeaf(int leafIndex, ref NodeChild leafChild, int nodeIndex, int nodeLeafCount, ref TOverlapHandler results)
        {
            if (nodeIndex < 0)
            {
                results.Handle(leafIndex, Encode(nodeIndex));
            }
            else
            {
                if (nodeLeafCount <= leafThreshold)
                    jobs.Add(new Job { A = Encode(leafIndex), B = nodeIndex }, Pool);
                else
                    TestLeafAgainstNode(leafIndex, ref leafChild, nodeIndex, ref results);
            }
        }

        void TestLeafAgainstNode(int leafIndex, ref NodeChild leafChild, int nodeIndex, ref TOverlapHandler results)
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
                DispatchTestForLeaf(leafIndex, ref leafChild, a.Index, a.LeafCount, ref results);
            }
            if (bIntersects)
            {
                DispatchTestForLeaf(leafIndex, ref leafChild, bIndex, bLeafCount, ref results);
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
                        GetJobsBetweenDifferentNodes(ref Tree.Nodes[a.Index], ref Tree.Nodes[b.Index], ref results);

                }
                else
                {
                    //leaf B versus node A.
                    TestLeafAgainstNode(Encode(b.Index), ref b, a.Index, ref results);
                }
            }
            else if (b.Index >= 0)
            {
                //leaf A versus node B.
                TestLeafAgainstNode(Encode(a.Index), ref a, b.Index, ref results);
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

        void CollectJobsInNode(int nodeIndex, int leafCount, ref TOverlapHandler results)
        {
            if (leafCount <= leafThreshold)
            {
                jobs.Add(new Job { A = nodeIndex, B = nodeIndex }, Pool);
                return;
            }

            ref var node = ref Tree.Nodes[nodeIndex];
            ref var a = ref node.A;
            ref var b = ref node.B;

            var ab = BoundingBox.IntersectsUnsafe(a, b);

            if (a.Index >= 0)
                CollectJobsInNode(a.Index, a.LeafCount, ref results);
            if (b.Index >= 0)
                CollectJobsInNode(b.Index, b.LeafCount, ref results);

            if (ab)
            {
                DispatchTestForNodes(ref a, ref b, ref results);
            }

        }
    }
}
