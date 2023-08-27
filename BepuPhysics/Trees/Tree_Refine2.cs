using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuUtilities.TaskScheduling;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Task = BepuUtilities.TaskScheduling.Task;

namespace BepuPhysics.Trees;

public partial struct Tree
{
    const int flagForRootRefinementSubtree = 1 << 30;

    readonly void ReifyRootRefinementNodeChild(ref int index, ref QuickList<int> refinementNodeIndices, int realNodeIndex, int childIndexInParent)
    {
        //Root refinements mark internal subtrees with a flag in the second to last index.
        if (index < 0)
        {
            //The child is a leaf.
            Leaves[Encode(index)] = new Leaf(realNodeIndex, childIndexInParent);
        }
        else
        {
            //The child is an internal node.
            if ((uint)index < flagForRootRefinementSubtree)
            {
                //The child is an internal node that is part of the refinement; remap its index to point at the real memory location.
                index = refinementNodeIndices[index];
            }
            else
            {
                //The child is an internal node that is *not* part of the refinement: it's a subtree endpoint.
                //No remapping is required, but we do need to strip off the 'this is a subtree endpoint of the refinement' flag.
                index &= ~flagForRootRefinementSubtree;
            }
            //Just as leaves need to be updated to point at the new node state, parent pointers for internal nodes need be updated too.
            //Note that this touches memory associated with nodes that weren't included in the refinement.
            //This is only safe if the subtree refinement either occurs sequentially with root refinement, or the subtree refinement doesn't touch the subtree refinement root's metanode.
            //NOTE: This means the binned builder *should not touch the metanodes*.
            ref var childMetanode = ref Metanodes[index];
            childMetanode.Parent = realNodeIndex;
            childMetanode.IndexInParent = childIndexInParent;
        }
    }

    static unsafe void ReifyRootRefinement(int startIndex, int endIndex, QuickList<int> nodeIndices, Buffer<Node> refinementNodes, Tree tree)
    {
        for (int i = startIndex; i < endIndex; ++i)
        {
            //refinementNodeIndices maps "refinement index space" to "real index space"; we can use it to update child pointers to the real locations.
            var realNodeIndex = nodeIndices[i];
            ref var refinedNode = ref refinementNodes[i];
            //Map child indices, and update leaf references.
            tree.ReifyRootRefinementNodeChild(ref refinedNode.A.Index, ref nodeIndices, realNodeIndex, 0);
            tree.ReifyRootRefinementNodeChild(ref refinedNode.B.Index, ref nodeIndices, realNodeIndex, 1);
            tree.Nodes[realNodeIndex] = refinedNode;
        }
    }

    readonly void ReifyRootRefinementST(QuickList<int> refinementNodeIndices, Buffer<Node> refinementNodes)
    {
        ReifyRootRefinement(0, refinementNodeIndices.Count, refinementNodeIndices, refinementNodes, this);
    }

    unsafe struct ReifyRefinementContext
    {
        public QuickList<int>* RefinementNodeIndices;
        public Buffer<Node>* RefinementNodes;
        public int StartIndex;
        public int EndIndex;
        public Tree* Tree;
    }

    static unsafe void ReifyRootRefinementTask(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ref var context = ref *(ReifyRefinementContext*)untypedContext;
        ReifyRootRefinement(context.StartIndex, context.EndIndex, *context.RefinementNodeIndices, *context.RefinementNodes, *context.Tree);
    }

    readonly unsafe void ReifyRootRefinementMT(QuickList<int>* refinementNodeIndices, Buffer<Node>* refinementNodes, int targetTaskCount, int workerIndex, TaskStack* taskStack, IThreadDispatcher dispatcher)
    {
        var nodesPerTask = refinementNodeIndices->Count / targetTaskCount;
        var remainder = refinementNodeIndices->Count - targetTaskCount * nodesPerTask;
        Debug.Assert(targetTaskCount < 1024, "We used a stackalloc for these task allocations under the assumption that there would be *very* few required, and that's clearly wrong here! What's going on?");
        Span<Task> tasks = stackalloc Task[targetTaskCount];
        ReifyRefinementContext* contexts = stackalloc ReifyRefinementContext[targetTaskCount];
        var tree = this;

        var previousEnd = 0;
        for (int i = 0; i < tasks.Length; ++i)
        {
            var count = i < remainder ? nodesPerTask + 1 : nodesPerTask;
            ref var context = ref contexts[i];
            context.RefinementNodeIndices = refinementNodeIndices;
            context.RefinementNodes = refinementNodes;
            context.Tree = &tree;
            context.StartIndex = previousEnd;
            previousEnd += count;
            context.EndIndex = previousEnd;
            tasks[i] = new Task(&ReifyRootRefinementTask, contexts + i, i);
        }

        taskStack->RunTasks(tasks, workerIndex, dispatcher);
    }


    readonly void ReifySubtreeRefinementNodeChild(ref int index, ref QuickList<int> refinementNodeIndices, int realNodeIndex, int childIndexInParent)
    {
        if (index < 0)
        {
            //The child is a leaf.
            Leaves[Encode(index)] = new Leaf(realNodeIndex, childIndexInParent);
        }
        else
        {
            //The child is an internal node that is part of the refinement; remap its index to point at the real memory location.
            index = refinementNodeIndices[index];
            //Just as leaves need to be updated to point at the new node state, parent pointers for internal nodes need be updated too.
            //Note that this touches memory associated with nodes that weren't included in the refinement.
            //This is only safe if the subtree refinement either occurs sequentially with root refinement, or the subtree refinement doesn't touch the subtree refinement root's metanode.
            //NOTE: This means the binned builder *should not touch the metanodes*.
            ref var childMetanode = ref Metanodes[index];
            childMetanode.Parent = realNodeIndex;
            childMetanode.IndexInParent = childIndexInParent;
        }
    }

    static unsafe void ReifySubtreeRefinement(int startIndex, int endIndex, QuickList<int> nodeIndices, Buffer<Node> refinementNodes, Tree tree)
    {
        for (int i = startIndex; i < endIndex; ++i)
        {
            //refinementNodeIndices maps "refinement index space" to "real index space"; we can use it to update child pointers to the real locations.
            var realNodeIndex = nodeIndices[i];
            ref var refinedNode = ref refinementNodes[i];
            //Map child indices, and update leaf references.
            //Root refinements mark internal subtrees with a flag in the second to last index.
            tree.ReifySubtreeRefinementNodeChild(ref refinedNode.A.Index, ref nodeIndices, realNodeIndex, 0);
            tree.ReifySubtreeRefinementNodeChild(ref refinedNode.B.Index, ref nodeIndices, realNodeIndex, 1);
            tree.Nodes[realNodeIndex] = refinedNode;
            //Debug.Assert(Metanodes[realNodeIndex].Parent < 0 || Unsafe.Add(ref Nodes[Metanodes[realNodeIndex].Parent].A, Metanodes[realNodeIndex].IndexInParent).Index == realNodeIndex);
        }
    }


    void ReifySubtreeRefinementST(QuickList<int> refinementNodeIndices, Buffer<Node> refinementNodes)
    {
        ReifySubtreeRefinement(0, refinementNodeIndices.Count, refinementNodeIndices, refinementNodes, this);
    }

    static unsafe void ReifySubtreeRefinementTask(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ref var context = ref *(ReifyRefinementContext*)untypedContext;
        ReifySubtreeRefinement(context.StartIndex, context.EndIndex, *context.RefinementNodeIndices, *context.RefinementNodes, *context.Tree);
    }

    readonly unsafe void ReifySubtreeRefinementMT(QuickList<int>* refinementNodeIndices, Buffer<Node>* refinementNodes, int targetTaskCount, int workerIndex, TaskStack* taskStack, IThreadDispatcher dispatcher)
    {
        var nodesPerTask = refinementNodeIndices->Count / targetTaskCount;
        var remainder = refinementNodeIndices->Count - targetTaskCount * nodesPerTask;
        Debug.Assert(targetTaskCount < 1024, "We used a stackalloc for these task allocations under the assumption that there would be *very* few required, and that's clearly wrong here! What's going on?");
        Span<Task> tasks = stackalloc Task[targetTaskCount];
        ReifyRefinementContext* contexts = stackalloc ReifyRefinementContext[targetTaskCount];
        var tree = this;

        var previousEnd = 0;
        for (int i = 0; i < tasks.Length; ++i)
        {
            var count = i < remainder ? nodesPerTask + 1 : nodesPerTask;
            ref var context = ref contexts[i];
            context.RefinementNodeIndices = refinementNodeIndices;
            context.RefinementNodes = refinementNodes;
            context.Tree = &tree;
            context.StartIndex = previousEnd;
            previousEnd += count;
            context.EndIndex = previousEnd;
            tasks[i] = new Task(&ReifySubtreeRefinementTask, contexts + i, i);
        }

        taskStack->RunTasks(tasks, workerIndex, dispatcher);
    }


    void FindSubtreeRefinementTargets(int nodeIndex, int leftLeafCount, int subtreeRefinementSize, int targetSubtreeRefinementCount, ref int startIndex, int endIndex, ref QuickList<int> refinementTargets)
    {
        //If we've used up the target region already, just quit.
        if (startIndex >= endIndex || refinementTargets.Count == targetSubtreeRefinementCount)
            return;

        ref var node = ref Nodes[nodeIndex];
        var midpoint = leftLeafCount + node.A.LeafCount;
        if (startIndex < midpoint)
        {
            //Go left!
            if (node.A.LeafCount <= subtreeRefinementSize)
            {
                //This is a candidate!
                if (node.A.LeafCount > 2) //Only include subtrees if they could be meaningfully refined.
                    refinementTargets.AllocateUnsafely() = node.A.Index;
                startIndex += node.A.LeafCount;
            }
            else
            {
                //Too big to be a candidate; traverse further.
                FindSubtreeRefinementTargets(node.A.Index, leftLeafCount, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, endIndex, ref refinementTargets);
            }
        }

        //If we've used up the target region already, just quit.
        if (startIndex >= endIndex || refinementTargets.Count == targetSubtreeRefinementCount)
            return;

        //Note that A's traversal may have modified startIndex such that B should now be traversed.
        if (startIndex >= midpoint)
        {
            //Go right!
            if (node.B.LeafCount <= subtreeRefinementSize)
            {
                //This is a candidate!
                if (node.B.LeafCount > 2) //Only include subtrees if they could be meaningfully refined.
                    refinementTargets.AllocateUnsafely() = node.B.Index;
                startIndex += node.B.LeafCount;
            }
            else
            {
                //Too big to be a candidate; traverse further.
                FindSubtreeRefinementTargets(node.B.Index, leftLeafCount + node.A.LeafCount, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, endIndex, ref refinementTargets);
            }
        }
    }
    void FindSubtreeRefinementTargets(int subtreeRefinementSize, int targetSubtreeRefinementCount, ref int startIndex, ref QuickList<int> refinementTargets)
    {
        //It's not impossible for the tree to have changed state such that the start index is invalid (or the user might have given invalid input). Just wrap back to 0 if that happens.
        if (startIndex >= LeafCount || startIndex < 0)
            startIndex = 0;
        var initialStart = startIndex;
        FindSubtreeRefinementTargets(0, 0, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, LeafCount, ref refinementTargets);
        if (startIndex >= LeafCount && refinementTargets.Count < targetSubtreeRefinementCount)
        {
            //Hit the end of the tree. Reset.
            startIndex = 0;
            var remainingLeaves = initialStart; //We walk through all leaves once, so if we started at X, we can go as far as X (after one wrap).
            FindSubtreeRefinementTargets(0, 0, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, remainingLeaves, ref refinementTargets);
        }
    }



    static bool IsNodeChildSubtreeRefinementTarget(Buffer<Vector<int>> subtreeRefinementBundles, in NodeChild child, int parentTotalLeafCount, int subtreeRefinementSize)
    {
        //First check if it *could* be one by checking the leaf count threshold.
        if (child.LeafCount <= subtreeRefinementSize && parentTotalLeafCount > subtreeRefinementSize)
        {
            //It may be a subtree refinement. Do a deeper test!
            var search = new Vector<int>(child.Index);
            for (int i = 0; i < subtreeRefinementBundles.Length; ++i)
            {
                if (Vector.EqualsAny(search, subtreeRefinementBundles[i]))
                    return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Checks if a child should be a subtree in the root refinement. If so, it's added to the list. Otherwise, it's pushed onto the stack.
    /// </summary>
    private static void TryPushChildForRootRefinement(
         int subtreeRefinementSize, Buffer<Vector<int>> subtreeRefinementRootBundles, int nodeTotalLeafCount, int subtreeBudget, in NodeChild child, ref QuickList<(int nodeIndex, int subtreeBudget)> stack, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        //We automatically accept any child as a subtree for the refinement process if:
        //1. It's a leaf node, or
        //2. This traversal path has used up its node budget.  
        Debug.Assert(subtreeBudget >= 0);
        if (subtreeBudget == 1)
        {
            //rootRefinementSubtrees.AllocateUnsafely() = child;
            ref var allocatedChild = ref rootRefinementSubtrees.AllocateUnsafely();
            allocatedChild = child;
            allocatedChild.Index |= flagForRootRefinementSubtree;
        }
        else
        {
            //Internal node; is it a subtree refinement?
            if (IsNodeChildSubtreeRefinementTarget(subtreeRefinementRootBundles, child, nodeTotalLeafCount, subtreeRefinementSize))
            {
                //Yup!
                ref var allocatedChild = ref rootRefinementSubtrees.AllocateUnsafely();
                allocatedChild = child;
                //Internal nodes used as subtrees by the root refinement are flagged so that the reification process knows to stop.
                Debug.Assert(allocatedChild.Index < flagForRootRefinementSubtree, "The use of an upper index bit as flag means the binned refiner cannot handle trees with billions of children.");
                allocatedChild.Index |= flagForRootRefinementSubtree;
            }
            else
            {
                //Not a subtree refinement, and we know we have budget remaining.
                stack.AllocateUnsafely() = (child.Index, subtreeBudget);
            }
        }
    }

    unsafe void CollectSubtreesForRootRefinement(int rootRefinementSize, int subtreeRefinementSize, BufferPool pool, in QuickList<int> subtreeRefinementTargets, ref QuickList<int> rootRefinementNodeIndices, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        var rootStack = new QuickList<(int nodeIndex, int subtreeBudget)>(rootRefinementSize, pool);
        rootStack.AllocateUnsafely() = (0, rootRefinementSize);
        var subtreeRefinementTargetBundles = new Buffer<Vector<int>>(subtreeRefinementTargets.Span.Memory, BundleIndexing.GetBundleCount(subtreeRefinementTargets.Count));
        while (rootStack.TryPop(out var nodeToVisit))
        {
            rootRefinementNodeIndices.AllocateUnsafely() = nodeToVisit.nodeIndex;
            ref var node = ref Nodes[nodeToVisit.nodeIndex];
            var nodeTotalLeafCount = node.A.LeafCount + node.B.LeafCount;
            Debug.Assert(nodeToVisit.subtreeBudget <= nodeTotalLeafCount);
            var lowerSubtreeBudget = int.Min((nodeToVisit.subtreeBudget + 1) / 2, int.Min(node.A.LeafCount, node.B.LeafCount));
            var higherSubtreeBudget = nodeToVisit.subtreeBudget - lowerSubtreeBudget;
            var useSmallerForA = lowerSubtreeBudget == node.A.LeafCount;
            var aSubtreeBudget = useSmallerForA ? lowerSubtreeBudget : higherSubtreeBudget;
            var bSubtreeBudget = useSmallerForA ? higherSubtreeBudget : lowerSubtreeBudget;

            TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargetBundles, nodeTotalLeafCount, bSubtreeBudget, node.B, ref rootStack, ref rootRefinementSubtrees);
            TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargetBundles, nodeTotalLeafCount, aSubtreeBudget, node.A, ref rootStack, ref rootRefinementSubtrees);
        }
        rootStack.Dispose(pool);
    }

    internal struct HeapEntry
    {
        public int Index;
        public float Cost;
    }
    unsafe internal struct BinaryHeap
    {
        public Buffer<HeapEntry> Entries;
        public int Count;

        public BinaryHeap(Buffer<HeapEntry> entries)
        {
            Entries = entries;
            Count = 0;
        }

        public BinaryHeap(int capacity, BufferPool pool) : this(new Buffer<HeapEntry>(capacity, pool)) { }

        public void Dispose(BufferPool pool)
        {
            pool.Return(ref Entries);
        }

        public unsafe void Insert(int indexToInsert, float cost)
        {
            int index = Count;
            ++Count;
            //Sift up.
            while (index > 0)
            {
                var parentIndex = (index - 1) >> 1;
                var parent = Entries[parentIndex];
                if (parent.Cost < cost)
                {
                    //Pull the parent down.
                    Entries[index] = parent;
                    index = parentIndex;
                }
                else
                {
                    //Found the insertion spot.
                    break;
                }
            }
            ref var entry = ref Entries[index];
            entry.Index = indexToInsert;
            entry.Cost = cost;
        }


        public HeapEntry Pop()
        {
            var entry = Entries[0];
            --Count;
            var cost = Entries[Count].Cost;

            //Pull the elements up to fill in the gap.
            int index = 0;
            while (true)
            {
                var childIndexA = (index << 1) + 1;
                var childIndexB = (index << 1) + 2;
                if (childIndexB < Count)
                {
                    //Both children are available.
                    //Try swapping with the largest one.
                    var childA = Entries[childIndexA];
                    var childB = Entries[childIndexB];
                    if (childA.Cost > childB.Cost)
                    {
                        if (cost > childA.Cost)
                        {
                            break;
                        }
                        Entries[index] = Entries[childIndexA];
                        index = childIndexA;
                    }
                    else
                    {
                        if (cost > childB.Cost)
                        {
                            break;
                        }
                        Entries[index] = Entries[childIndexB];
                        index = childIndexB;
                    }
                }
                else if (childIndexA < Count)
                {
                    //Only one child was available.
                    ref var childA = ref Entries[childIndexA];
                    if (cost > childA.Cost)
                    {
                        break;
                    }
                    Entries[index] = Entries[childIndexA];
                    index = childIndexA;
                }
                else
                {
                    //The children were beyond the heap.
                    break;
                }
            }
            //Move the last entry into position.
            Entries[index] = Entries[Count];
            return entry;
        }

    }

    /// <summary>
    /// Checks if a child should be a subtree in the root refinement. If so, it's added to the list. Otherwise, it's pushed onto the stack.
    /// </summary>
    private void TryPushChildForRootRefinement2(
         int subtreeRefinementSize, int nodeTotalLeafCount, Buffer<Vector<int>> subtreeRefinementRootBundles, ref NodeChild child, ref BinaryHeap heap, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        if (child.Index < 0)
        {
            //It's a leaf node; directly accept it.
            rootRefinementSubtrees.AllocateUnsafely() = child;
        }
        else
        {
            //Internal node; is it a subtree refinement?
            if (IsNodeChildSubtreeRefinementTarget(subtreeRefinementRootBundles, child, nodeTotalLeafCount, subtreeRefinementSize))
            {
                //Yup!
                ref var allocatedChild = ref rootRefinementSubtrees.AllocateUnsafely();
                allocatedChild = child;
                //Internal nodes used as subtrees by the root refinement are flagged so that the reification process knows to stop.
                Debug.Assert(allocatedChild.Index < flagForRootRefinementSubtree, "The use of an upper index bit as flag means the binned refiner cannot handle trees with billions of children.");
                allocatedChild.Index |= flagForRootRefinementSubtree;
            }
            else
            {
                //A regular internal node; push it.
                //TODO: The heuristic for cost is pretty simple: the bounds metric of the child in isolation.
                //The root collection will always visit the *biggest* nodes. That's often a good idea, but not always.
                //A couple of potential improvements to consider:
                //1. Use boundsMetric * leafCount. This will tend to push nodes with lots of leaves to the top of the heap.
                //2. Dive one step deeper and compute the ratio of the bounds metric of the children to the parent. Nodes that do a poor job of splitting the space will tend to have a higher ratio.
                //#1 is trivial. #2 requires touching a little more memory. Worth investigating.
                //(Just doing this for now to match the old implementation.)
                //var childBoundsMetric = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref child));
                //ref var childNode = ref Nodes[child.Index];
                //var grandchildABoundsMetric = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref childNode.A));
                //var grandchildBBoundsMetric = ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref childNode.B));
                //var cost = (grandchildABoundsMetric * childNode.A.LeafCount + grandchildBBoundsMetric * childNode.B.LeafCount);
                //heap.Insert(child.Index, cost);
                heap.Insert(child.Index, ComputeBoundsMetric(Unsafe.As<NodeChild, BoundingBox4>(ref child)));
            }
        }
    }
    unsafe void CollectSubtreesForRootRefinementWithPriorityQueue(int rootRefinementSize, int subtreeRefinementSize, BufferPool pool, in QuickList<int> subtreeRefinementTargets, ref QuickList<int> rootRefinementNodeIndices, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        //Instead of using a breadth first search, we greedily expand the root refinement by looking for the next node with the highest cost.
        //This will tend to force the root refinement to find pathologically bad subtrees rapidly.
        var heap = new BinaryHeap(new Buffer<HeapEntry>(rootRefinementSize, pool));
        heap.Insert(0, 0); //no need to actually calculate the cost for the root; it's gonna get popped.
        var subtreeRefinementTargetBundles = new Buffer<Vector<int>>(subtreeRefinementTargets.Span.Memory, BundleIndexing.GetBundleCount(subtreeRefinementTargets.Count));
        while (heap.Count > 0 && heap.Count + rootRefinementSubtrees.Count < rootRefinementSize)
        {
            var entry = heap.Pop();
            rootRefinementNodeIndices.AllocateUnsafely() = entry.Index;
            ref var node = ref Nodes[entry.Index];
            var nodeTotalLeafCount = node.A.LeafCount + node.B.LeafCount;

            TryPushChildForRootRefinement2(subtreeRefinementSize, nodeTotalLeafCount, subtreeRefinementTargetBundles, ref node.B, ref heap, ref rootRefinementSubtrees);
            TryPushChildForRootRefinement2(subtreeRefinementSize, nodeTotalLeafCount, subtreeRefinementTargetBundles, ref node.A, ref heap, ref rootRefinementSubtrees);
        }
        //The traversal has added any subtrees that represent either 1. a leaf or 2. a subtree refinement target.
        //The heap contains all the other subtrees that need to be added in index form; add them all now.
        for (int i = 0; i < heap.Count; ++i)
        {
            var entry = heap.Entries[i];
            var metanode = Metanodes[entry.Index];
            Debug.Assert(metanode.Parent >= 0, "The root should never show up in the heap post traversal! Something weird has happened.");
            ref var parent = ref Nodes[metanode.Parent];
            ref var childInParent = ref Unsafe.Add(ref parent.A, metanode.IndexInParent);
            ref var allocated = ref rootRefinementSubtrees.AllocateUnsafely();
            Debug.Assert(childInParent.Index >= 0, "Anything in the heap should be an internal node.");
            allocated = childInParent;
            allocated.Index |= flagForRootRefinementSubtree;
        }
        heap.Entries.Dispose(pool);
    }

    void CollectSubtreesForSubtreeRefinement(int refinementTargetRootNodeIndex, Buffer<int> subtreeStackBuffer, ref QuickList<int> subtreeRefinementNodeIndices, ref QuickList<NodeChild> subtreeRefinementLeaves)
    {
        Debug.Assert(subtreeRefinementLeaves.Count == 0 && subtreeRefinementNodeIndices.Count == 0);
        var subtreeStack = new QuickList<int>(subtreeStackBuffer);
        subtreeStack.AllocateUnsafely() = refinementTargetRootNodeIndex;
        while (subtreeStack.TryPop(out var nodeToVisit))
        {
            ref var node = ref Nodes[nodeToVisit];
            subtreeRefinementNodeIndices.AllocateUnsafely() = nodeToVisit;
            if (node.B.Index >= 0)
                subtreeStack.AllocateUnsafely() = node.B.Index;
            else
                subtreeRefinementLeaves.AllocateUnsafely() = node.B;
            if (node.A.Index >= 0)
                subtreeStack.AllocateUnsafely() = node.A.Index;
            else
                subtreeRefinementLeaves.AllocateUnsafely() = node.A;
        }
    }

    /// <summary>
    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// </summary>
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root. Nonpositive values will cause the root refinement to be skipped.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="subtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param> 
    /// <param name="usePriorityQueue">True if the root refinement should use a priority queue during subtree collection to find larger nodes, false if it should try to collect a more balanced tree.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    public unsafe void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool, bool usePriorityQueue = true)
    {
        //No point refining anything with two leaves. This condition also avoids having to special case for an incomplete root node.
        if (LeafCount <= 2)
            return;
        //Clamp refinement sizes to avoid pointless overallocations when the user supplies odd inputs.
        rootRefinementSize = int.Min(rootRefinementSize, LeafCount);
        subtreeRefinementSize = int.Min(subtreeRefinementSize, LeafCount);
        //We used a vectorized containment test later, so make sure to pad out the refinement target list.
        var subtreeRefinementCapacity = BundleIndexing.GetBundleCount(subtreeRefinementCount) * Vector<int>.Count;
        var subtreeRefinementTargets = new QuickList<int>(subtreeRefinementCapacity, pool);
        FindSubtreeRefinementTargets(subtreeRefinementSize, subtreeRefinementCount, ref subtreeRefinementStartIndex, ref subtreeRefinementTargets);
        //Fill the trailing slots in the list with -1 to avoid matches.
        ((Span<int>)subtreeRefinementTargets.Span)[subtreeRefinementTargets.Count..].Fill(-1);

        var refinementNodesAllocation = new Buffer<Node>(int.Max(rootRefinementSize, subtreeRefinementSize), pool);
        if (rootRefinementSize > 0) //Skip root refinement if it's zero or negative size.
        {
            var rootRefinementSubtrees = new QuickList<NodeChild>(rootRefinementSize, pool);
            var rootRefinementNodeIndices = new QuickList<int>(rootRefinementSize, pool);
            if (usePriorityQueue)
                CollectSubtreesForRootRefinementWithPriorityQueue(rootRefinementSize, subtreeRefinementSize, pool, subtreeRefinementTargets, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);
            else
                CollectSubtreesForRootRefinement(rootRefinementSize, subtreeRefinementSize, pool, subtreeRefinementTargets, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);

            //Now that we have a list of nodes to refine, we can run the root refinement.
            Debug.Assert(rootRefinementNodeIndices.Count == rootRefinementSubtrees.Count - 1);

            var rootRefinementNodes = refinementNodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
            //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
            BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, default, default, pool);
            ReifyRootRefinementST(rootRefinementNodeIndices, rootRefinementNodes);
            rootRefinementSubtrees.Dispose(pool);
            rootRefinementNodeIndices.Dispose(pool);
        }


        var subtreeRefinementNodeIndices = new QuickList<int>(subtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(subtreeRefinementSize, pool);
        var subtreeStackBuffer = new Buffer<int>(subtreeRefinementSize, pool);
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            //Accumulate nodes and leaves with a prepass.
            CollectSubtreesForSubtreeRefinement(subtreeRefinementTargets[i], subtreeStackBuffer, ref subtreeRefinementNodeIndices, ref subtreeRefinementLeaves);

            var refinementNodes = refinementNodesAllocation.Slice(0, subtreeRefinementNodeIndices.Count);
            //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
            BinnedBuild(subtreeRefinementLeaves, refinementNodes, default, default, pool);
            ReifySubtreeRefinementST(subtreeRefinementNodeIndices, refinementNodes);

            subtreeRefinementNodeIndices.Count = 0;
            subtreeRefinementLeaves.Count = 0;
        }

        subtreeRefinementNodeIndices.Dispose(pool);
        subtreeRefinementLeaves.Dispose(pool);
        subtreeRefinementTargets.Dispose(pool);
        subtreeStackBuffer.Dispose(pool);
        refinementNodesAllocation.Dispose(pool);
    }


    unsafe struct RefinementContext
    {
        public int RootRefinementSize;
        public int SubtreeRefinementSize;
        public int TotalLeafCountInSubtrees;
        public int TargetTaskBudget;
        public int WorkerCount;
        public QuickList<int> SubtreeRefinementTargets;
        public TaskStack* TaskStack;
        public bool Deterministic;
        public bool UsePriorityQueue;
        /// <remarks>
        /// This is a *copy* of the original tree that spawned this refinement. Refinements do not modify the memory stored at the level of the Tree, only memory *pointed* to by the tree.
        /// </remarks>
        public Tree Tree;
    }

    unsafe static void ExecuteRootRefinementTask(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ref var context = ref *(RefinementContext*)untypedContext;
        var pool = dispatcher.WorkerPools[workerIndex];

        var taskCount = (int)float.Ceiling(context.TargetTaskBudget * (float)context.RootRefinementSize / (context.RootRefinementSize + context.TotalLeafCountInSubtrees));

        //We now know which nodes are the roots of subtree refinements; the root refinement can avoid traversing through them.
        var rootRefinementSubtrees = new QuickList<NodeChild>(context.RootRefinementSize, pool);
        var rootRefinementNodeIndices = new QuickList<int>(context.RootRefinementSize, pool);

        //We now know which nodes are the roots of subtree refinements; the root refinement can avoid traversing through them.
        //TODO: A multithreaded version of the collection phase *could* help a little, but there's a few problems to overcome:
        // 1. The cost of the collection phase is pretty cheap. Around the cost of a refit. Multithreading a collection phase of a thousand nodes is probably going to be net slower.
        // 2. It's difficult to do it deterministically without having to do a postpass to copy things into position in the contiguous buffer, and touching all that memory is a big hit.
        // 3. The main use case for refinements is in the broad phase. This will usually be run next to a dynamic refit refine, so we'll already have decent utilization.
        // 4. Doing it for the priority queue variant is even harder.
        // 5. I don't wanna.
        //So, punting this for later. A nondeterministic implementation wouldn't be too bad. Could always just fall back to ST when deterministic flag is set. 
        if (context.UsePriorityQueue)
            context.Tree.CollectSubtreesForRootRefinementWithPriorityQueue(context.RootRefinementSize, context.SubtreeRefinementSize, pool, context.SubtreeRefinementTargets, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);
        else
            context.Tree.CollectSubtreesForRootRefinement(context.RootRefinementSize, context.SubtreeRefinementSize, pool, context.SubtreeRefinementTargets, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);

        //Now that we have a list of nodes to refine, we can run the root refinement.
        Debug.Assert(rootRefinementNodeIndices.Count == rootRefinementSubtrees.Count - 1);
        var rootRefinementNodes = new Buffer<Node>(rootRefinementNodeIndices.Count, pool);
        //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
        if (taskCount > 1)
        {
            BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, default, default, pool, dispatcher, context.TaskStack, workerIndex, context.WorkerCount, taskCount, deterministic: context.Deterministic);
            context.Tree.ReifyRootRefinementMT(&rootRefinementNodeIndices, &rootRefinementNodes, taskCount, workerIndex, context.TaskStack, dispatcher);
        }
        else
        {
            BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, default, default, pool, workerIndex: workerIndex);
            context.Tree.ReifyRootRefinementST(rootRefinementNodeIndices, rootRefinementNodes);
        }

        rootRefinementSubtrees.Dispose(pool);
        rootRefinementNodeIndices.Dispose(pool);
        rootRefinementNodes.Dispose(pool);
    }
    unsafe static void ExecuteSubtreeRefinementTask(long subtreeRefinementTarget, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ref var context = ref *(RefinementContext*)untypedContext;
        var pool = dispatcher.WorkerPools[workerIndex];

        ref var refinementRootNode = ref context.Tree.Nodes[(int)subtreeRefinementTarget];
        var refinementLeafCount = refinementRootNode.A.LeafCount + refinementRootNode.B.LeafCount;
        var taskCount = (int)float.Ceiling(context.TargetTaskBudget * (float)refinementLeafCount / (context.RootRefinementSize + context.TotalLeafCountInSubtrees));

        var subtreeRefinementNodeIndices = new QuickList<int>(context.SubtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(context.SubtreeRefinementSize, pool);
        var subtreeStackBuffer = new Buffer<int>(context.SubtreeRefinementSize, pool);

        //Accumulate nodes and leaves with a prepass.
        context.Tree.CollectSubtreesForSubtreeRefinement((int)subtreeRefinementTarget, subtreeStackBuffer, ref subtreeRefinementNodeIndices, ref subtreeRefinementLeaves);
        var refinementNodes = new Buffer<Node>(subtreeRefinementNodeIndices.Count, pool);
        //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.

        if (taskCount > 1)
        {
            BinnedBuild(subtreeRefinementLeaves, refinementNodes, default, default, pool, dispatcher, context.TaskStack,
                workerIndex: workerIndex, workerCount: context.WorkerCount, targetTaskCount: taskCount, deterministic: context.Deterministic);
            context.Tree.ReifySubtreeRefinementMT(&subtreeRefinementNodeIndices, &refinementNodes, taskCount, workerIndex, context.TaskStack, dispatcher);
        }
        else
        {
            BinnedBuild(subtreeRefinementLeaves, refinementNodes, default, default, pool, workerIndex: workerIndex);
            context.Tree.ReifySubtreeRefinementST(subtreeRefinementNodeIndices, refinementNodes);
        }

        refinementNodes.Dispose(pool);
        subtreeRefinementNodeIndices.Dispose(pool);
        subtreeRefinementLeaves.Dispose(pool);
        subtreeStackBuffer.Dispose(pool);

    }

    private unsafe void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool, int workerIndex, TaskStack* taskStack, IThreadDispatcher threadDispatcher, bool internallyDispatch, int workerCount, int targetTaskBudget, bool deterministic, bool usePriorityQueue)
    {
        //No point refining anything with two leaves. This condition also avoids having to special case for an incomplete root node.
        if (LeafCount <= 2)
            return;
        //Just early out of a fake refine attempt!
        if (rootRefinementSize <= 0 && (subtreeRefinementCount <= 0 || subtreeRefinementSize <= 0))
            return;
        //Setting root refinement size to 0 or negative values disables root refinement. People might intuitively try the same for subtree sizes.
        if (subtreeRefinementSize <= 0)
            subtreeRefinementCount = 0;
        if (targetTaskBudget < 0)
            targetTaskBudget = threadDispatcher.ThreadCount;

        //Clamp refinement sizes to avoid pointless overallocations when the user supplies odd inputs.
        rootRefinementSize = int.Min(rootRefinementSize, LeafCount);
        subtreeRefinementSize = int.Min(subtreeRefinementSize, LeafCount);
        //We used a vectorized containment test later, so make sure to pad out the refinement target list.
        var subtreeRefinementCapacity = BundleIndexing.GetBundleCount(subtreeRefinementCount) * Vector<int>.Count;
        var subtreeRefinementTargets = new QuickList<int>(subtreeRefinementCapacity, pool);
        var preStartIndex = subtreeRefinementStartIndex;
        subtreeRefinementStartIndex = preStartIndex;
        FindSubtreeRefinementTargets(subtreeRefinementSize, subtreeRefinementCount, ref subtreeRefinementStartIndex, ref subtreeRefinementTargets);
        //Fill the trailing slots in the list with -1 to avoid matches.
        ((Span<int>)subtreeRefinementTargets.Span)[subtreeRefinementTargets.Count..].Fill(-1);

        //Zero or negative root refine sizes means skip it.
        var rootRefinementCount = rootRefinementSize > 0 ? 1 : 0;
        var tasks = new Buffer<Task>(rootRefinementCount + subtreeRefinementTargets.Count, pool);
        var totalLeafCountInSubtrees = 0;
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            ref var node = ref Nodes[subtreeRefinementTargets[i]];
            totalLeafCountInSubtrees += node.A.LeafCount + node.B.LeafCount;
        }
        var context = new RefinementContext
        {
            RootRefinementSize = rootRefinementSize,
            SubtreeRefinementSize = subtreeRefinementSize,
            TotalLeafCountInSubtrees = totalLeafCountInSubtrees,
            TargetTaskBudget = targetTaskBudget,
            SubtreeRefinementTargets = subtreeRefinementTargets,
            TaskStack = taskStack,
            WorkerCount = workerCount,
            Deterministic = deterministic,
            UsePriorityQueue = usePriorityQueue,
            Tree = this
        };
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            tasks[i] = new Task(&ExecuteSubtreeRefinementTask, &context, subtreeRefinementTargets[i]);
        }
        if (rootRefinementSize > 0)
            tasks[^1] = new Task(&ExecuteRootRefinementTask, &context);
        if (internallyDispatch)
        {
            //There isn't an active dispatch, so we need to do it.
            taskStack->AllocateContinuationAndPush(tasks, workerIndex, threadDispatcher, onComplete: TaskStack.GetRequestStopTask(taskStack));
            TaskStack.DispatchWorkers(threadDispatcher, taskStack, workerCount);
        }
        else
        {
            //We're executing from within a multithreaded dispatch already, so we can simply run the tasks and trust that other threads are ready to steal.
            taskStack->RunTasks(tasks, workerIndex, threadDispatcher);
        }
        tasks.Dispose(pool);

        subtreeRefinementTargets.Dispose(pool);
    }

    /// <summary>
    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// </summary>
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root. Nonpositive values will cause the root refinement to be skipped.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="subtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param>
    /// <param name="threadDispatcher">Thread dispatcher used during the refinement.</param>
    /// <param name="deterministic">Whether to force determinism at a slightly higher cost when using internally multithreaded execution for an individual refinement operation.<para/>
    /// If the refine is single threaded, it is already deterministic and this flag has no effect.</param>
    /// <param name="usePriorityQueue">True if the root refinement should use a priority queue during subtree collection to find larger nodes, false if it should try to collect a more balanced tree.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    public unsafe void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool, IThreadDispatcher threadDispatcher, bool deterministic = false, bool usePriorityQueue = true)
    {
        var taskStack = new TaskStack(pool, threadDispatcher, threadDispatcher.ThreadCount);
        Refine2(rootRefinementSize, ref subtreeRefinementStartIndex, subtreeRefinementCount, subtreeRefinementSize, pool, 0, &taskStack, threadDispatcher, true, threadDispatcher.ThreadCount, threadDispatcher.ThreadCount, deterministic, usePriorityQueue);
        taskStack.Dispose(pool, threadDispatcher);
    }


    /// <summary>
    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// <para/>Pushes tasks into the provided <see cref="TaskStack"/>. Does not dispatch threads internally; this is intended to be used as a part of a caller-managed dispatch.
    /// </summary>
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root. Nonpositive values will cause the root refinement to be skipped.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="subtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param>
    /// <param name="threadDispatcher">Thread dispatcher used during the refinement.</param>
    /// <param name="deterministic">Whether to force determinism at a slightly higher cost when using internally multithreaded execution for an individual refinement operation.<para/>
    /// If the refine is single threaded, it is already deterministic and this flag has no effect.</param>
    /// <param name="taskStack"><see cref="TaskStack"/> that the refine operation will push tasks onto as needed.</param>
    /// <param name="workerIndex">Index of the worker calling the function.</param>
    /// <param name="targetTaskCount">Number of tasks the refinement should try to create during execution. If negative, uses <see cref="IThreadDispatcher.ThreadCount"/>.</param>
    /// <param name="usePriorityQueue">True if the root refinement should use a priority queue during subtree collection to find larger nodes, false if it should try to collect a more balanced tree.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    public unsafe void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize,
        BufferPool pool, IThreadDispatcher threadDispatcher, TaskStack* taskStack, int workerIndex, int targetTaskCount = -1, bool deterministic = false, bool usePriorityQueue = true)
    {
        Refine2(rootRefinementSize, ref subtreeRefinementStartIndex, subtreeRefinementCount, subtreeRefinementSize, pool, workerIndex, taskStack, threadDispatcher, false, threadDispatcher.ThreadCount, targetTaskCount, deterministic, usePriorityQueue);
    }
}
