using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    public struct SubtreeHeapEntry
    {
        public int Index;
        public float Cost;
    }
    unsafe internal struct SubtreeBinaryHeap
    {
        public SubtreeHeapEntry* Entries;
        public int Count;

        public SubtreeBinaryHeap(SubtreeHeapEntry* entries)
        {
            Entries = entries;
            Count = 0;
        }


        public unsafe void Insert(ref Node node, ref QuickList<int> subtrees)
        {
            ref var children = ref node.A;
            for (int childIndex = 0; childIndex < 2; ++childIndex)
            {
                ref var child = ref Unsafe.Add(ref children, childIndex);
                if (child.Index >= 0)
                {
                    int index = Count;
                    var cost = Tree.ComputeBoundsMetric(ref child.Min, ref child.Max);// - node->PreviousMetric;
                    ++Count;

                    //Sift up.
                    while (index > 0)
                    {
                        var parentIndex = (index - 1) >> 1;
                        var parent = Entries + parentIndex;
                        if (parent->Cost < cost)
                        {
                            //Pull the parent down.
                            Entries[index] = *parent;
                            index = parentIndex;
                        }
                        else
                        {
                            //Found the insertion spot.
                            break;
                        }
                    }
                    var entry = Entries + index;
                    entry->Index = child.Index;
                    entry->Cost = cost;

                }
                else
                {
                    //Immediately add leaf nodes.
                    subtrees.AddUnsafely(child.Index);
                }
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Pop(out SubtreeHeapEntry entry)
        {
            entry = Entries[0];
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
                    var childA = Entries + childIndexA;
                    var childB = Entries + childIndexB;
                    if (childA->Cost > childB->Cost)
                    {
                        if (cost > childA->Cost)
                        {
                            break;
                        }
                        Entries[index] = Entries[childIndexA];
                        index = childIndexA;
                    }
                    else
                    {
                        if (cost > childB->Cost)
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
                    var childA = Entries + childIndexA;
                    if (cost > childA->Cost)
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

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryPop(ref Buffer<Metanode> metanodes, ref int remainingSubtreeSpace, ref QuickList<int> subtrees, out int index, out float cost)
        {
            while (Count > 0)
            {
                //Repeatedly pop minimum until you find one that can fit.
                //Given the unique access nature, the fact that you're destroying the heap when there's not much space left doesn't matter.
                //In the event that you consume all the nodes, that just means there aren't any entries which would fit in the subtree set anymore.
                Pop(out SubtreeHeapEntry entry);
                //Choose to expand this node, or not.
                //Only choose to expand if its children will fit.
                //Any time a node is expanded, the existing node is removed from the set of potential subtrees stored in the priorityQueue.
                //So, the change in remainingSubtreeSpace = maximumSubtreesCount - (priorityQueue.Count + subtrees.Count) is childCount - 1.
                //This is ALWAYS the case.
                if (remainingSubtreeSpace >= 1 && metanodes[entry.Index].RefineFlag == 0)
                {
                    //Debug.Fail("don't forget to reenable the refine flag condition");
                    //This node's children can be included successfully in the remaining space.
                    index = entry.Index;
                    cost = entry.Cost;
                    remainingSubtreeSpace -= 1;
                    return true;
                }
                else
                {
                    //Either this node's children did not fit, or it was a refinement target. Refinement targets cannot be expanded.
                    //Since we won't be able to find this later, it needs to be added now.
                    //We popped the previous entry off the queue, so the remainingSubtreeSpace does not change by re-adding it.
                    //(remainingSubtreeSpace = maximumSubtreesCount - (priorityQueue.Count + subtrees.Count))
                    subtrees.AddUnsafely(entry.Index);
                }
            }
            index = -1;
            cost = -1;
            return false;
        }
    }

    partial struct Tree
    {

        public unsafe void CollectSubtrees(int nodeIndex, int maximumSubtrees, SubtreeHeapEntry* entries,
            ref QuickList<int> subtrees, ref QuickList<int> internalNodes, out float treeletCost)
        {

            //Collect subtrees iteratively by choosing the highest cost subtree repeatedly.
            //This collects every child of a given node at once- the set of subtrees must not include only SOME of the children of a node.

            //(You could lift this restriction and only take some nodes, but it would complicate things. You could not simply remove
            //the parent and add its children to go deeper; it would require doing some post-fixup on the results of the construction
            //or perhaps constraining the generation process to leave room for the unaffected nodes.)


            ref var node = ref Nodes[nodeIndex];
            Debug.Assert(maximumSubtrees >= 2, "Can't only consider some of a node's children, but specified maximumSubtrees precludes the treelet root's children.");
            //All of treelet root's children are included immediately. (Follows from above requirement.)

            var priorityQueue = new SubtreeBinaryHeap(entries);

            priorityQueue.Insert(ref node, ref subtrees);

            //Note that the treelet root is NOT added to the internal nodes list.

            //Note that the treelet root's cost is excluded from the treeletCost.
            //That's because the treelet root cannot change.
            treeletCost = 0;
            int remainingSubtreeSpace = maximumSubtrees - priorityQueue.Count - subtrees.Count;
            while (priorityQueue.TryPop(ref Metanodes, ref remainingSubtreeSpace, ref subtrees, out int highestIndex, out float highestCost))
            {
                treeletCost += highestCost;
                internalNodes.AddUnsafely(highestIndex);

                //Add all the children to the set of subtrees.
                //This is safe because we pre-validated the number of children in the node.
                ref var expandedNode = ref Nodes[highestIndex];
                priorityQueue.Insert(ref expandedNode, ref subtrees);
            }

            for (int i = 0; i < priorityQueue.Count; ++i)
            {
                subtrees.AddUnsafely(priorityQueue.Entries[i].Index);
            }

            //Sort the internal nodes so that the depth first builder will tend to produce less cache-scrambled results.
            //TODO: Note that the range of values in node indices is VERY often constrained to values below 65535. In other words, a radix sort would be a significant win.
            //Even better, use some form of implicit cache optimization that eliminates the possibility of noncontiguous internal nodes, so that this entire collection process
            //boils down to computing a range of elements based on the root index and leaf count. Dealing with the root node is a little more complex.
            //I suspect this entire function is going to go away later. For example:
            //An internally multithreaded root builder that always runs, treating a set of subtrees as leaves, followed by externally multithreaded subtree refines.
            //The root builder would write out its nodes into a new block of memory rather than working in place.
            //If the root builder terminates with a set of subtrees of known leaf counts and known positions, then multithreaded refines will execute on contiguous regions.
            //In other words, at no point is a sort of target nodes required, because they're all computed analytically and they are known to be in cache optimal locations.
            if (internalNodes.Count > 0) //It's possible for there to be no internal nodes if both children of the target node were leaves.
            {
                var comparer = new PrimitiveComparer<int>();
                QuickSort.Sort(ref internalNodes[0], 0, internalNodes.Count - 1, ref comparer);
            }
        }


        unsafe void ValidateStaging(Node* stagingNodes, ref QuickList<int> subtreeNodePointers, int treeletParent, int treeletIndexInParent, BufferPool pool)
        {
            var collectedSubtreeReferences = new QuickList<int>(subtreeNodePointers.Count, pool);
            var internalReferences = new QuickList<int>(subtreeNodePointers.Count, pool);
            internalReferences.Add(0, pool);
            ValidateStaging(stagingNodes, 0, ref subtreeNodePointers, ref collectedSubtreeReferences, ref internalReferences, pool, out int foundSubtrees, out int foundLeafCount);
            if (treeletParent < -1 || treeletParent >= nodeCount)
                throw new Exception("Bad treelet parent.");
            if (treeletIndexInParent < -1 || (treeletParent >= 0 && treeletIndexInParent >= 2))
                throw new Exception("Bad treelet index in parent.");
            if (treeletParent >= 0 && Unsafe.Add(ref Nodes[treeletParent].A, treeletIndexInParent).LeafCount != foundLeafCount)
            {
                throw new Exception("Bad leaf count.");
            }
            if (subtreeNodePointers.Count != foundSubtrees)
            {
                throw new Exception("Bad subtree found count.");
            }
            for (int i = 0; i < collectedSubtreeReferences.Count; ++i)
            {
                if (!subtreeNodePointers.Contains(collectedSubtreeReferences[i]) || !collectedSubtreeReferences.Contains(subtreeNodePointers[i]))
                    throw new Exception("Bad subtree reference.");
            }
            collectedSubtreeReferences.Dispose(pool);
            internalReferences.Dispose(pool);
        }
        unsafe void ValidateStaging(Node* stagingNodes, int stagingNodeIndex,
            ref QuickList<int> subtreeNodePointers, ref QuickList<int> collectedSubtreeReferences,
            ref QuickList<int> internalReferences, BufferPool pool, out int foundSubtrees, out int foundLeafCount)
        {
            var stagingNode = stagingNodes + stagingNodeIndex;
            var children = &stagingNode->A;
            foundSubtrees = foundLeafCount = 0;
            for (int i = 0; i < 2; ++i)
            {
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    if (internalReferences.Contains(child.Index))
                        throw new Exception("A child points to an internal node that was visited. Possible loop, or just general invalid.");
                    internalReferences.Add(child.Index, pool);
                    ValidateStaging(stagingNodes, child.Index, ref subtreeNodePointers, ref collectedSubtreeReferences, ref internalReferences, pool, out int childFoundSubtrees, out int childFoundLeafCount);

                    if (childFoundLeafCount != child.LeafCount)
                        throw new Exception("Bad leaf count.");
                    foundSubtrees += childFoundSubtrees;
                    foundLeafCount += childFoundLeafCount;
                }
                else
                {
                    var subtreeNodePointerIndex = Encode(child.Index);
                    var subtreeNodePointer = subtreeNodePointers[subtreeNodePointerIndex];
                    //Rather than looking up the shuffled SweepSubtree for information, just go back to the source.
                    if (subtreeNodePointer >= 0)
                    {
                        ref var node = ref Nodes[subtreeNodePointer];
                        var totalLeafCount = 0;
                        for (int childIndex = 0; childIndex < 2; ++childIndex)
                        {
                            totalLeafCount += Unsafe.Add(ref node.A, childIndex).LeafCount;
                        }

                        if (child.LeafCount != totalLeafCount)
                            throw new Exception("bad leaf count.");
                        foundLeafCount += totalLeafCount;
                    }
                    else
                    {
                        var leafIndex = Encode(subtreeNodePointer);
                        if (child.LeafCount != 1)
                            throw new Exception("bad leaf count.");
                        foundLeafCount += 1;
                    }
                    ++foundSubtrees;
                    collectedSubtreeReferences.Add(subtreeNodePointer, pool);
                }
            }

        }

    }
}
