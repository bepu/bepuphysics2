using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuUtilities.TaskScheduling;
using System;
using System.ComponentModel.Design;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;
using Task = BepuUtilities.TaskScheduling.Task;

namespace BepuPhysics.Trees;

public partial struct Tree
{
    const int flagForRootRefinementSubtree = 1 << 30;

    void ReifyRootRefinementNodeChild(ref int index, ref QuickList<int> refinementNodeIndices, int realNodeIndex, int childIndexInParent)
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
            ref var childMetanode = ref Metanodes[index];
            childMetanode.Parent = realNodeIndex;
            childMetanode.IndexInParent = childIndexInParent;
        }
    }

    void ReifyRootRefinement(QuickList<int> refinementNodeIndices, Buffer<Node> refinementNodes)
    {
        for (int i = 0; i < refinementNodeIndices.Count; ++i)
        {
            //refinementNodeIndices maps "refinement index space" to "real index space"; we can use it to update child pointers to the real locations.
            var realNodeIndex = refinementNodeIndices[i];
            ref var refinedNode = ref refinementNodes[i];
            //Map child indices, and update leaf references.
            ReifyRootRefinementNodeChild(ref refinedNode.A.Index, ref refinementNodeIndices, realNodeIndex, 0);
            ReifyRootRefinementNodeChild(ref refinedNode.B.Index, ref refinementNodeIndices, realNodeIndex, 1);
            Nodes[realNodeIndex] = refinedNode;
            Debug.Assert(Metanodes[realNodeIndex].Parent < 0 || Unsafe.Add(ref Nodes[Metanodes[realNodeIndex].Parent].A, Metanodes[realNodeIndex].IndexInParent).LeafCount == refinedNode.A.LeafCount + refinedNode.B.LeafCount);
            Debug.Assert(Metanodes[realNodeIndex].Parent < 0 || Unsafe.Add(ref Nodes[Metanodes[realNodeIndex].Parent].A, Metanodes[realNodeIndex].IndexInParent).Index == realNodeIndex);
        }
    }

    void ReifyRefinementNodeChild(ref int index, ref QuickList<int> refinementNodeIndices, int realNodeIndex, int childIndexInParent)
    {
        //Root refinements mark internal subtrees with a flag in the second to last index.
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
            ref var childMetanode = ref Metanodes[index];
            childMetanode.Parent = realNodeIndex;
            childMetanode.IndexInParent = childIndexInParent;
        }
    }

    void ReifyRefinement(QuickList<int> refinementNodeIndices, Buffer<Node> refinementNodes)
    {
        for (int i = 0; i < refinementNodeIndices.Count; ++i)
        {
            //refinementNodeIndices maps "refinement index space" to "real index space"; we can use it to update child pointers to the real locations.
            var realNodeIndex = refinementNodeIndices[i];
            ref var refinedNode = ref refinementNodes[i];
            //Map child indices, and update leaf references.
            //Root refinements mark internal subtrees with a flag in the second to last index.
            ReifyRefinementNodeChild(ref refinedNode.A.Index, ref refinementNodeIndices, realNodeIndex, 0);
            ReifyRefinementNodeChild(ref refinedNode.B.Index, ref refinementNodeIndices, realNodeIndex, 1);
            Nodes[realNodeIndex] = refinedNode;
            //Debug.Assert(Metanodes[realNodeIndex].Parent < 0 || Unsafe.Add(ref Nodes[Metanodes[realNodeIndex].Parent].A, Metanodes[realNodeIndex].IndexInParent).Index == realNodeIndex);
        }
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
        var initialStart = startIndex;
        FindSubtreeRefinementTargets(0, 0, subtreeRefinementSize, targetSubtreeRefinementCount, ref startIndex, LeafCount, ref refinementTargets);
        if (startIndex >= LeafCount && refinementTargets.Count < targetSubtreeRefinementCount)
        {
            //Hit the end of the tree. Reset.
            startIndex = 0;
            var remainingLeaves = LeafCount - initialStart;
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


    static unsafe void CollectSubtreesForRootRefinement(ref QuickList<(int nodeIndex, int subtreeBudget)> stack, int subtreeRefinementSize, BufferPool pool, Buffer<Vector<int>> subtreeRefinementTargetBundles, ref Tree tree, ref QuickList<int> rootRefinementNodeIndices, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        while (stack.TryPop(out var nodeToVisit))
        {
            rootRefinementNodeIndices.AllocateUnsafely() = nodeToVisit.nodeIndex;
            ref var node = ref tree.Nodes[nodeToVisit.nodeIndex];
            var nodeTotalLeafCount = node.A.LeafCount + node.B.LeafCount;
            Debug.Assert(nodeToVisit.subtreeBudget <= nodeTotalLeafCount);
            var lowerSubtreeBudget = int.Min((nodeToVisit.subtreeBudget + 1) / 2, int.Min(node.A.LeafCount, node.B.LeafCount));
            var higherSubtreeBudget = nodeToVisit.subtreeBudget - lowerSubtreeBudget;
            var useSmallerForA = lowerSubtreeBudget == node.A.LeafCount;
            var aSubtreeBudget = useSmallerForA ? lowerSubtreeBudget : higherSubtreeBudget;
            var bSubtreeBudget = useSmallerForA ? higherSubtreeBudget : lowerSubtreeBudget;

            TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargetBundles, nodeTotalLeafCount, bSubtreeBudget, node.B, ref stack, ref rootRefinementSubtrees);
            TryPushChildForRootRefinement(subtreeRefinementSize, subtreeRefinementTargetBundles, nodeTotalLeafCount, aSubtreeBudget, node.A, ref stack, ref rootRefinementSubtrees);
        }
    }

    unsafe void CollectSubtreesForRootRefinementST(int rootRefinementSize, int subtreeRefinementSize, BufferPool pool, in QuickList<int> subtreeRefinementTargets, ref QuickList<int> rootRefinementNodeIndices, ref QuickList<NodeChild> rootRefinementSubtrees)
    {
        var rootStack = new QuickList<(int nodeIndex, int subtreeBudget)>(rootRefinementSize, pool);
        rootStack.AllocateUnsafely() = (0, rootRefinementSize);
        var subtreeRefinementTargetBundles = new Buffer<Vector<int>>(subtreeRefinementTargets.Span.Memory, BundleIndexing.GetBundleCount(subtreeRefinementTargets.Count));
        CollectSubtreesForRootRefinement(ref rootStack, subtreeRefinementSize, pool, subtreeRefinementTargetBundles, ref this, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);
        rootStack.Dispose(pool);
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
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="subtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    public unsafe void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool)
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

        //We now know which nodes are the roots of subtree refinements; the root refinement can avoid traversing through them.
        //TODO: A multithreaded version of the collection phase *could* help a little, but there's a few problems to overcome:
        // 1. The cost of the collection phase is pretty cheap. Around the cost of a refit. Multithreading a collection phase of a thousand nodes is probably going to be net slower.
        // 2. It's difficult to do it deterministically without having to do a postpass to copy things into position in the contiguous buffer, and touching all that memory is a big hit.
        // 3. The main use case for refinements is in the broad phase. This will usually be run next to a dynamic refit refine, so we'll already have decent utilization.
        // 4. I don't wanna.
        //So, punting this for later. A nondeterministic implementation wouldn't be too bad. Could always just fall back to ST when deterministic flag is set. 
        var rootRefinementSubtrees = new QuickList<NodeChild>(rootRefinementSize, pool);
        var rootRefinementNodeIndices = new QuickList<int>(rootRefinementSize, pool);
        CollectSubtreesForRootRefinementST(rootRefinementSize, subtreeRefinementSize, pool, subtreeRefinementTargets, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);

        //Now that we have a list of nodes to refine, we can run the root refinement.
        Debug.Assert(rootRefinementNodeIndices.Count == rootRefinementSubtrees.Count - 1);
        var refinementNodesAllocation = new Buffer<Node>(int.Max(rootRefinementNodeIndices.Count, subtreeRefinementSize), pool);
        var refinementMetanodesAllocation = new Buffer<Metanode>(refinementNodesAllocation.Length, pool);

        var rootRefinementNodes = refinementNodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        var rootRefinementMetanodes = refinementMetanodesAllocation.Slice(0, rootRefinementNodeIndices.Count);
        //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
        BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, rootRefinementMetanodes, default, pool);
        ReifyRootRefinement(rootRefinementNodeIndices, rootRefinementNodes);
        rootRefinementSubtrees.Dispose(pool);
        rootRefinementNodeIndices.Dispose(pool);


        var subtreeRefinementNodeIndices = new QuickList<int>(subtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(subtreeRefinementSize, pool);
        var subtreeStackBuffer = new Buffer<int>(subtreeRefinementSize, pool);
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            //Accumulate nodes and leaves with a prepass.
            CollectSubtreesForSubtreeRefinement(subtreeRefinementTargets[i], subtreeStackBuffer, ref subtreeRefinementNodeIndices, ref subtreeRefinementLeaves);

            var refinementNodes = refinementNodesAllocation.Slice(0, subtreeRefinementNodeIndices.Count);
            var refinementMetanodes = refinementMetanodesAllocation.Slice(0, subtreeRefinementNodeIndices.Count);
            //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
            BinnedBuild(subtreeRefinementLeaves, refinementNodes, refinementMetanodes, default, pool);
            ReifyRefinement(subtreeRefinementNodeIndices, refinementNodes);

            subtreeRefinementNodeIndices.Count = 0;
            subtreeRefinementLeaves.Count = 0;
        }

        subtreeRefinementNodeIndices.Dispose(pool);
        subtreeRefinementLeaves.Dispose(pool);
        subtreeRefinementTargets.Dispose(pool);
        subtreeStackBuffer.Dispose(pool);
        refinementNodesAllocation.Dispose(pool);
        refinementMetanodesAllocation.Dispose(pool);
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
        /// <remarks>
        /// This is a *copy* of the original tree that spawned this refinement. Refinements do not modify the memory stored at the level of the Tree, only memory *pointed* to by the tree.
        /// </remarks>
        public Tree Tree;
    }
    //static int debugAccumulatorForSubtreeIndices = 0;
    //static int debugAccumulatorForNodeTopology = 0;
    //static int debugIndex = 0;
    unsafe static void ExecuteRootRefinementTask(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ref var context = ref *(RefinementContext*)untypedContext;
        var pool = dispatcher.WorkerPools[workerIndex];

        var taskCount = (int)float.Ceiling(context.TargetTaskBudget * (float)context.RootRefinementSize / (context.RootRefinementSize + context.TotalLeafCountInSubtrees));

        //We now know which nodes are the roots of subtree refinements; the root refinement can avoid traversing through them.
        var rootRefinementSubtrees = new QuickList<NodeChild>(context.RootRefinementSize, pool);
        var rootRefinementNodeIndices = new QuickList<int>(context.RootRefinementSize, pool);

        context.Tree.CollectSubtreesForRootRefinementST(context.RootRefinementSize, context.SubtreeRefinementSize, pool, context.SubtreeRefinementTargets, ref rootRefinementNodeIndices, ref rootRefinementSubtrees);

        //var localSubtreeHash = 0;
        //for (int i = 0; i < rootRefinementSubtrees.Count; ++i)
        //{
        //    localSubtreeHash = HashHelper.Rehash(localSubtreeHash) + rootRefinementSubtrees[i].Index;
        //    //localSubtreeHash += rootRefinementSubtrees[i].Index;
        //}
        //debugAccumulatorForSubtreeIndices = HashHelper.Rehash(debugAccumulatorForSubtreeIndices) + localSubtreeHash;
        //Now that we have a list of nodes to refine, we can run the root refinement.
        Debug.Assert(rootRefinementNodeIndices.Count == rootRefinementSubtrees.Count - 1);
        var rootRefinementNodes = new Buffer<Node>(rootRefinementNodeIndices.Count, pool);
        var rootRefinementMetanodes = new Buffer<Metanode>(rootRefinementNodeIndices.Count, pool);
        //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
        if (taskCount > 1)
        {
            BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, rootRefinementMetanodes, default, pool, dispatcher, context.TaskStack, workerIndex, context.WorkerCount, taskCount, deterministic: context.Deterministic);
        }
        else
        {
            BinnedBuild(rootRefinementSubtrees, rootRefinementNodes, rootRefinementMetanodes, default, pool, workerIndex: workerIndex);
        }
        //var localNodeTopologyHash = 0;
        //for (int i = 0; i < rootRefinementNodes.Length; ++i)
        //{
        //    localNodeTopologyHash = HashHelper.Rehash(HashHelper.Rehash(localNodeTopologyHash) + rootRefinementNodes[i].A.Index) + rootRefinementNodes[i].B.Index;
        //    //localNodeTopologyHash += rootRefinementNodes[i].A.Index + rootRefinementNodes[i].B.Index;
        //}


        //debugAccumulatorForNodeTopology = HashHelper.Rehash(debugAccumulatorForNodeTopology) + localNodeTopologyHash;
        ////for (int i = 0; i < context.SubtreeRefinementTargets.Count; ++i)
        ////{
        ////    debugAccumulator = HashHelper.Rehash(debugAccumulator) + context.SubtreeRefinementTargets[i];
        ////}
        //Console.WriteLine($"Refinement targets count {debugIndex}: {context.SubtreeRefinementTargets.Count}, node topology hash: {debugAccumulatorForNodeTopology}, subtree indices hash: {debugAccumulatorForSubtreeIndices}");
        //++debugIndex;
        context.Tree.ReifyRootRefinement(rootRefinementNodeIndices, rootRefinementNodes);
        rootRefinementSubtrees.Dispose(pool);
        rootRefinementNodeIndices.Dispose(pool);
        rootRefinementNodes.Dispose(pool);
        rootRefinementMetanodes.Dispose(pool);
    }
    unsafe static void ExecuteSubtreeRefinementTask(long subtreeRefinementTarget, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ref var context = ref *(RefinementContext*)untypedContext;
        var pool = dispatcher.WorkerPools[workerIndex];

        var subtreeRefinementNodeIndices = new QuickList<int>(context.SubtreeRefinementSize, pool);
        var subtreeRefinementLeaves = new QuickList<NodeChild>(context.SubtreeRefinementSize, pool);
        var subtreeStackBuffer = new Buffer<int>(context.SubtreeRefinementSize, pool);

        //Accumulate nodes and leaves with a prepass.
        context.Tree.CollectSubtreesForSubtreeRefinement((int)subtreeRefinementTarget, subtreeStackBuffer, ref subtreeRefinementNodeIndices, ref subtreeRefinementLeaves);

        var refinementNodes = new Buffer<Node>(subtreeRefinementNodeIndices.Count, pool);
        var refinementMetanodes = new Buffer<Metanode>(subtreeRefinementNodeIndices.Count, pool);
        //Passing 'default' for the leaves tells the binned builder to not worry about updating leaves.
        BinnedBuild(subtreeRefinementLeaves, refinementNodes, refinementMetanodes, default, pool);
        context.Tree.ReifyRefinement(subtreeRefinementNodeIndices, refinementNodes);

        refinementNodes.Dispose(pool);
        refinementMetanodes.Dispose(pool);
        subtreeRefinementNodeIndices.Dispose(pool);
        subtreeRefinementLeaves.Dispose(pool);
        subtreeStackBuffer.Dispose(pool);

    }
    static unsafe void ExecuteWorker(int workerIndex, IThreadDispatcher dispatcher)
    {
        var taskStack = (TaskStack*)dispatcher.UnmanagedContext;
        PopTaskResult popTaskResult;
        var waiter = new SpinWait();
        while ((popTaskResult = taskStack->TryPopAndRun(workerIndex, dispatcher)) != PopTaskResult.Stop)
        {
            waiter.SpinOnce(-1);
        }
    }

    static unsafe void StopStackOnCompletion(long id, void* untypedContext, int workerIndex, IThreadDispatcher dispatcher)
    {
        ((TaskStack*)untypedContext)->RequestStop();
    }


    /// <summary>
    /// Incrementally refines a subset of the tree by running a binned builder over subtrees.
    /// </summary>
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="subtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param>
    /// <param name="deterministic">Whether to force determinism at a slightly higher cost when using internally multithreaded execution for an individual refinement operation.<para/>
    /// If the refine is single threaded, it is already deterministic and this flag has no effect.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    private unsafe void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool, int workerIndex, TaskStack* taskStack, IThreadDispatcher threadDispatcher, bool internallyDispatch, int workerCount, int targetTaskBudget, bool deterministic)
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

        var tasks = new Buffer<Task>(1 + subtreeRefinementTargets.Count, pool);
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
            Tree = this
        };
        for (int i = 0; i < subtreeRefinementTargets.Count; ++i)
        {
            tasks[i] = new Task(&ExecuteSubtreeRefinementTask, &context, subtreeRefinementTargets[i]);
        }
        tasks[^1] = new Task(&ExecuteRootRefinementTask, &context);
        if (internallyDispatch)
        {
            //There isn't an active dispatch, so we need to do it.
            taskStack->AllocateContinuationAndPush(tasks, workerIndex, threadDispatcher, onComplete: new Task(&StopStackOnCompletion, taskStack));
            threadDispatcher.DispatchWorkers(&ExecuteWorker, unmanagedContext: taskStack, maximumWorkerCount: workerCount);
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
    /// <param name="rootRefinementSize">Size of the refinement run on nodes near the root.</param>
    /// <param name="subtreeRefinementStartIndex">Index used to distribute subtree refinements over multiple executions.</param>
    /// <param name="subtreeRefinementCount">Number of subtree refinements to execute.</param>
    /// <param name="subtreeRefinementSize">Target size of subtree refinements. The actual size of refinement will usually be larger or smaller.</param>
    /// <param name="pool">Pool used for ephemeral allocations during the refinement.</param>
    /// <param name="deterministic">Whether to force determinism at a slightly higher cost when using internally multithreaded execution for an individual refinement operation.<para/>
    /// If the refine is single threaded, it is already deterministic and this flag has no effect.</param>
    /// <remarks>Nodes will not be refit.</remarks>
    public unsafe void Refine2(int rootRefinementSize, ref int subtreeRefinementStartIndex, int subtreeRefinementCount, int subtreeRefinementSize, BufferPool pool, IThreadDispatcher threadDispatcher, bool deterministic = false)
    {
        //No point refining anything with two leaves. This condition also avoids having to special case for an incomplete root node.
        if (LeafCount <= 2)
            return;
        var taskStack = new TaskStack(pool, threadDispatcher, threadDispatcher.ThreadCount);
        Refine2(rootRefinementSize, ref subtreeRefinementStartIndex, subtreeRefinementCount, subtreeRefinementSize, pool, 0, &taskStack, threadDispatcher, true, threadDispatcher.ThreadCount, threadDispatcher.ThreadCount, deterministic);
        taskStack.Dispose(pool, threadDispatcher);
    }
}
