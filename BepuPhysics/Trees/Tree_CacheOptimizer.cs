using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        unsafe void SwapNodes(int indexA, int indexB)
        {
            ref var a = ref Nodes[indexA];
            ref var b = ref Nodes[indexB];
            ref var metaA = ref Metanodes[indexA];
            ref var metaB = ref Metanodes[indexB];

            Helpers.Swap(ref a, ref b);
            Helpers.Swap(ref metaA, ref metaB);

            if (metaA.Parent == indexA)
            {
                //The original B's parent was A.
                //That parent has moved.
                metaA.Parent = indexB;
            }
            else if (metaB.Parent == indexB)
            {
                //The original A's parent was B.
                //That parent has moved.
                metaB.Parent = indexA;
            }
            Unsafe.Add(ref Nodes[metaA.Parent].A, metaA.IndexInParent).Index = indexA;
            Unsafe.Add(ref Nodes[metaB.Parent].A, metaB.IndexInParent).Index = indexB;


            //Update the parent pointers of the children.
            ref var children = ref a.A;
            for (int i = 0; i < 2; ++i)
            {
                ref var child = ref Unsafe.Add(ref children, i);
                if (child.Index >= 0)
                {
                    Metanodes[child.Index].Parent = indexA;
                }
                else
                {
                    var leafIndex = Encode(child.Index);
                    Leaves[leafIndex] = new Leaf(indexA, i);
                }
            }
            children = ref b.A;
            for (int i = 0; i < 2; ++i)
            {
                ref var child = ref Unsafe.Add(ref children, i);
                if (child.Index >= 0)
                {
                    Metanodes[child.Index].Parent = indexB;
                }
                else
                {
                    var leafIndex = Encode(child.Index);
                    Leaves[leafIndex] = new Leaf(indexB, i);
                }
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe bool TryLock(ref int nodeIndex)
        {
            //Node index may change during the execution of this function.
            int lockedIndex;
            while (true)
            {
                lockedIndex = nodeIndex;
                if (0 != Interlocked.CompareExchange(ref Metanodes[lockedIndex].RefineFlag, 1, 0))
                {
                    //Abort.
                    return false;
                }
                if (lockedIndex != nodeIndex) //Compare exchange inserts memory barrier.
                {
                    //Locked the wrong node, let go.
                    Metanodes[lockedIndex].RefineFlag = 0;
                }
                else
                {
                    // If lockedIndex == nodeIndex and we have the lock on lockedIndex, nodeIndex can't move and we're done.
                    return true;
                }
            }
        }

        unsafe bool TrySwapNodeWithTargetThreadSafe(int swapperIndex, int swapperParentIndex, int swapTargetIndex)
        {
            Debug.Assert(Metanodes[swapperIndex].RefineFlag == 1, "The swapper should be locked.");
            Debug.Assert(Metanodes[swapperParentIndex].RefineFlag == 1, "The swapper parent should be locked.");
            Debug.Assert(swapTargetIndex != swapperIndex, "If the swapper is already at the swap target, this should not be called."); //safe to compare since if equal, it's locked.
            //We must make sure that the node, its parent, and its children are locked.
            //But watch out for parent or grandparent relationships between the nodes. Those lower the number of locks required.

            //The possible cases are as follows:
            //0) The swap target is the swapper's grandparent. Don't lock the swap target's child that == swapper's parent.
            //1) The swap target is the swapper's parent. Don't lock the swap target, AND don't lock the swap target's child == swapper.
            //2) The swap target is the swapper's position (do nothing, you're done).
            //3) The swap target is one of the swapper's children. Don't lock the swap target, AND don't lock swap target's parent.
            //4) The swap target is one of the swapper's grandchildren. Don't lock the swap target's parent.
            //5) The swap target is unrelated to the swapper.

            //Note that we don't have to worry about reordering/deadlocks because these are not blocking locks. If one fails, all locks immediately abort.
            //This means that we won't always end up with an optimal cache layout, but it doesn't affect correctness at all.
            //Eventually, the node will be revisited and it will probably get fixed.

            //Note the use of an iterating TryLock. It accepts the fact that the reference memory could be changed at any time before a lock is acquired.
            //It explicitly checks to ensure that it actually grabs a lock on the correct node.

            //Don't lock swapTarget if:
            //1) swapTargetIndex == swapperParentIndex, because swapperParentIndex is already locked
            //2) nodes[swapTargetIndex].Parent == swapperIndex, because swapper's children are already locked 

            //Note that the above comparison between a potentially unlocked nodes[swapTargetIndex].Parent and swapperIndex is safe because
            //if it evaluates true, then it was actually locked. In the event that it evaluates to false, they aren't the same node- which random node it might be doesn't matter.
            //Similar logic applies to the similar lock elisions below.

            bool success = false;
            var needSwapTargetLock = swapTargetIndex != swapperParentIndex && Metanodes[swapTargetIndex].Parent != swapperIndex;
            if (!needSwapTargetLock || TryLock(ref swapTargetIndex))
            {
                ref var swapTarget = ref Metanodes[swapTargetIndex];

                //Don't lock swapTarget->Parent if:
                //1) swapTarget->Parent == swapperIndex, because swapper is already locked.
                //2) nodes[swapTarget->Parent].Parent == swapperIndex, because swapper's children are already locked.

                var needSwapTargetParentLock = swapTarget.Parent != swapperIndex && Metanodes[swapTarget.Parent].Parent != swapperIndex;
                if (!needSwapTargetParentLock || TryLock(ref swapTarget.Parent))
                {

                    int childrenLockedCount = 2;
                    ref var children = ref Nodes[swapTargetIndex].A;
                    for (int i = 0; i < 2; ++i)
                    {
                        ref var child = ref Unsafe.Add(ref children, i);
                        //Don't lock children[i] if:
                        //1) children[i] == swapperIndex, because the swapper is already locked 
                        //2) children[i] == swapperParentIndex, because the swapperParent is already locked
                        if (child.Index >= 0 && child.Index != swapperIndex && child.Index != swapperParentIndex && !TryLock(ref child.Index))
                        {
                            //Failed to acquire lock on all children.
                            childrenLockedCount = i;
                            break;
                        }
                    }

                    if (childrenLockedCount == 2)
                    {
                        //Nodes locked successfully.
                        SwapNodes(swapperIndex, swapTargetIndex);
                        success = true;

                        //Unlock children of the original swap target, *which now lives in the swapperIndex*.
                        children = ref Nodes[swapperIndex].A;
                        for (int i = childrenLockedCount - 1; i >= 0; --i)
                        {
                            ref var child = ref Unsafe.Add(ref children, i);
                            //Again, note use of swapTargetIndex instead of swapperIndex.
                            if (child.Index >= 0 && child.Index != swapTargetIndex && child.Index != swapperParentIndex) //Avoid unlocking children already locked by the caller.
                                Metanodes[child.Index].RefineFlag = 0;
                        }
                    }
                    else
                    {
                        //No swap occurred. Can still use the swapTarget->ChildA pointer.
                        for (int i = childrenLockedCount - 1; i >= 0; --i)
                        {
                            ref var child = ref Unsafe.Add(ref children, i);
                            if (child.Index >= 0 && child.Index != swapperIndex && child.Index != swapperParentIndex) //Avoid unlocking children already locked by the caller.
                                Metanodes[child.Index].RefineFlag = 0;
                        }
                    }


                    if (needSwapTargetParentLock)
                    {
                        if (success)
                        {
                            //Note that swapTarget pointer is no longer used, since the node was swapped.
                            //The old swap target is now in the swapper index slot!
                            Metanodes[Metanodes[swapperIndex].Parent].RefineFlag = 0;
                        }
                        else
                        {
                            //No swap occurred, 
                            Metanodes[swapTarget.Parent].RefineFlag = 0;
                        }
                    }
                }

                if (needSwapTargetLock)
                {
                    if (success)
                    {
                        //Once again, the original swapTarget now lives in swapperIndex.
                        Metanodes[swapperIndex].RefineFlag = 0;
                    }
                    else
                    {
                        swapTarget.RefineFlag = 0;
                    }
                }
            }
            return success;
        }


        public unsafe bool TryLockSwapTargetThreadSafe(ref int swapTargetIndex, int swapperIndex, int swapperParentIndex)
        {
            Debug.Assert(Metanodes[swapperIndex].RefineFlag == 1, "The swapper should be locked.");
            Debug.Assert(Metanodes[swapperParentIndex].RefineFlag == 1, "The swapper parent should be locked.");
            Debug.Assert(swapTargetIndex != swapperIndex, "If the swapper is already at the swap target, this should not be called."); //safe to compare since if equal, it's locked.
            //We must make sure that the node, its parent, and its children are locked.
            //But watch out for parent or grandparent relationships between the nodes. Those lower the number of locks required.

            //The possible cases are as follows:
            //0) The swap target is the swapper's grandparent. Don't lock the swap target's child that == swapper's parent.
            //1) The swap target is the swapper's parent. Don't lock the swap target, AND don't lock the swap target's child == swapper.
            //2) The swap target is the swapper's position (do nothing, you're done).
            //3) The swap target is one of the swapper's children. Don't lock the swap target, AND don't lock swap target's parent.
            //4) The swap target is one of the swapper's grandchildren. Don't lock the swap target's parent.
            //5) The swap target is unrelated to the swapper.

            //Note that we don't have to worry about reordering/deadlocks because these are not blocking locks. If one fails, all locks immediately abort.
            //This means that we won't always end up with an optimal cache layout, but it doesn't affect correctness at all.
            //Eventually, the node will be revisited and it will probably get fixed.

            //Note the use of an iterating TryLock. It accepts the fact that the reference memory could be changed at any time before a lock is acquired.
            //It explicitly checks to ensure that it actually grabs a lock on the correct node.

            //Don't lock swapTarget if:
            //1) swapTargetIndex == swapperParentIndex, because swapperParentIndex is already locked
            //2) nodes[swapTargetIndex].Parent == swapperIndex, because swapper's children are already locked 

            //Note that the above comparison between a potentially unlocked nodes[swapTargetIndex].Parent and swapperIndex is safe because
            //if it evaluates true, then it was actually locked. In the event that it evaluates to false, they aren't the same node- which random node it might be doesn't matter.
            //Similar logic applies to the similar lock elisions below.

            bool success = false;
            var needSwapTargetLock = swapTargetIndex != swapperParentIndex && Metanodes[swapTargetIndex].Parent != swapperIndex;
            if (!needSwapTargetLock || TryLock(ref swapTargetIndex))
            {
                ref var swapTarget = ref Metanodes[swapTargetIndex];

                //Don't lock swapTarget->Parent if:
                //1) swapTarget->Parent == swapperIndex, because swapper is already locked.
                //2) nodes[swapTarget->Parent].Parent == swapperIndex, because swapper's children are already locked.

                var needSwapTargetParentLock = swapTarget.Parent != swapperIndex && Metanodes[swapTarget.Parent].Parent != swapperIndex;
                if (!needSwapTargetParentLock || TryLock(ref swapTarget.Parent))
                {

                    int childrenLockedCount = 2;
                    ref var children = ref Nodes[swapTargetIndex].A;
                    for (int i = 0; i < 2; ++i)
                    {
                        ref var child = ref Unsafe.Add(ref children, i);
                        //Don't lock children[i] if:
                        //1) children[i] == swapperIndex, because the swapper is already locked 
                        //2) children[i] == swapperParentIndex, because the swapperParent is already locked
                        if (child.Index != swapperIndex && child.Index != swapperParentIndex && !TryLock(ref child.Index))
                        {
                            //Failed to acquire lock on all children.
                            childrenLockedCount = i;
                            break;
                        }
                    }

                    if (childrenLockedCount == 2)
                    {
                        //Nodes locked successfully.
                        success = true;
                    }
                    //TODO: should not unlock here because this is a LOCK function!
                    for (int i = childrenLockedCount - 1; i >= 0; --i)
                    {
                        ref var child = ref Unsafe.Add(ref children, i);
                        if (child.Index != swapperIndex && child.Index != swapperParentIndex) //Avoid unlocking children already locked by the caller.
                            Metanodes[child.Index].RefineFlag = 0;
                    }

                    if (needSwapTargetParentLock)
                        Metanodes[swapTarget.Parent].RefineFlag = 0;
                }
                if (needSwapTargetLock)
                    swapTarget.RefineFlag = 0;
            }
            return success;
        }

        /// <summary>
        /// Attempts to swap two nodes. Aborts without changing memory if the swap is contested by another thread.
        /// </summary>
        /// <remarks>Uses Node.RefineFlag as a lock-keeping mechanism. All refine flags should be cleared to 0 before a multithreaded processing stage that performs swaps.</remarks>
        /// <param name="aIndex">First node of the swap pair.</param>
        /// <param name="bIndex">Second node of the swap pair.</param>
        /// <returns>True if the nodes were swapped, false if the swap was contested.</returns>
        public unsafe bool TrySwapNodesThreadSafe(ref int aIndex, ref int bIndex)
        {
            Debug.Assert(aIndex != bIndex, "Can't swap a node with itself.");

            //We must lock:
            //a
            //b
            //a->Parent
            //b->Parent
            //a->{Children}
            //b->{Children}
            //But watch out for parent or grandparent relationships between the nodes. Those lower the number of locks required.

            //Note that we don't have to worry about reordering/deadlocks because these are not blocking locks. If one fails, all locks immediately abort.
            //This means that we won't always end up with an optimal cache layout, but it doesn't affect correctness at all.
            //Eventually, the node will be revisited and it will probably get fixed.

            //Note the use of an iterating TryLock. It accepts the fact that the reference memory could be changed at any time before a lock is acquired.
            //It explicitly checks to ensure that it actually grabs a lock on the correct node.

            bool success = false;
            if (TryLock(ref aIndex))
            {
                ref var a = ref Metanodes[aIndex];
                if (TryLock(ref bIndex))
                {
                    //Now, we know that aIndex and bIndex will not change.
                    ref var b = ref Metanodes[bIndex];

                    var aParentAvoidedLock = a.Parent == bIndex;
                    if (aParentAvoidedLock || TryLock(ref a.Parent))
                    {
                        var bParentAvoidedLock = b.Parent == aIndex;
                        if (bParentAvoidedLock || TryLock(ref b.Parent))
                        {

                            int aChildrenLockedCount = 2;
                            ref var aChildren = ref Nodes[aIndex].A;
                            for (int i = 0; i < 2; ++i)
                            {
                                ref var child = ref Unsafe.Add(ref aChildren, i);
                                if (child.Index != bIndex && child.Index != b.Parent && !TryLock(ref child.Index))
                                {
                                    //Failed to acquire lock on all children.
                                    aChildrenLockedCount = i;
                                    break;
                                }
                            }

                            if (aChildrenLockedCount == 2)
                            {
                                int bChildrenLockedCount = 2;
                                ref var bChildren = ref Nodes[bIndex].A;
                                for (int i = 0; i < 2; ++i)
                                {
                                    ref var child = ref Unsafe.Add(ref bChildren, i);
                                    if (child.Index != aIndex && child.Index != a.Parent && !TryLock(ref child.Index))
                                    {
                                        //Failed to acquire lock on all children.
                                        bChildrenLockedCount = i;
                                        break;
                                    }
                                }

                                if (bChildrenLockedCount == 2)
                                {
                                    //ALL nodes locked successfully.
                                    SwapNodes(aIndex, bIndex);
                                    success = true;
                                }

                                for (int i = bChildrenLockedCount - 1; i >= 0; --i)
                                {
                                    ref var child = ref Unsafe.Add(ref bChildren, i);
                                    if (child.Index != aIndex && child.Index != a.Parent) //Do not yet unlock a or its parent.
                                        Metanodes[child.Index].RefineFlag = 0;
                                }
                            }
                            for (int i = aChildrenLockedCount - 1; i >= 0; --i)
                            {
                                ref var child = ref Unsafe.Add(ref aChildren, i);
                                if (child.Index != bIndex && child.Index != b.Parent) //Do not yet unlock b or its parent.
                                    Metanodes[child.Index].RefineFlag = 0;
                            }
                            if (!bParentAvoidedLock)
                                Metanodes[b.Parent].RefineFlag = 0;
                        }
                        if (!aParentAvoidedLock)
                            Metanodes[a.Parent].RefineFlag = 0;
                    }
                    b.RefineFlag = 0;
                }
                a.RefineFlag = 0;
            }
            return success;
        }


        /// <summary>
        /// Moves the children if the specified node into the correct relative position in memory.
        /// Takes care to avoid contested moves in multithreaded contexts. May not successfully
        /// complete all desired moves if contested.
        /// </summary>
        /// <param name="nodeIndex">Node whose children should be optimized.</param>
        /// <returns>True if no other threads contested the optimization or if the node is already optimized, otherwise false.
        /// Will return true even if not all nodes are optimized if the reason was a target index outside of the node list bounds.</returns>
        public unsafe bool IncrementalCacheOptimizeThreadSafe(int nodeIndex)
        {
            Debug.Assert(leafCount >= 2,
                "Should only use cache optimization when there are at least two leaves. Every node has to have 2 children, and optimizing a 0 or 1 leaf tree is silly anyway.");
            //Multithreaded cache optimization attempts to acquire a lock on every involved node.
            //If any lock fails, it just abandons the entire attempt.
            //That's acceptable- the incremental optimization only cares about eventual success.

            //TODO: if you know the tree in question has a ton of coherence, could attempt to compare child pointers without locks ahead of time.
            //Unsafe, but acceptable as an optimization prepass. Would avoid some interlocks. Doesn't seem to help for trees undergoing any significant motion.
            ref var node = ref Metanodes[nodeIndex];
            bool success = true;

            if (0 == Interlocked.CompareExchange(ref node.RefineFlag, 1, 0))
            {
                ref var children = ref Nodes[nodeIndex].A;
                var targetIndex = nodeIndex + 1;



                //Note that we pull all children up to their final positions relative to the current node index.
                //This helps ensure that more nodes can converge to their final positions- if we didn't do this,
                //a full top-down cache optimization could end up leaving some nodes near the bottom of the tree and without any room for their children.
                //TODO: N-ary tree support. Tricky without subtree count and without fixed numbers of children per node, but it may be possible
                //to stil choose something which converged.

                for (int i = 0; i < 2; ++i)
                {
                    ref var child = ref Unsafe.Add(ref children, i);
                    if (targetIndex >= nodeCount)
                    {
                        //This attempted swap would reach beyond the allocated nodes.
                        //That means the current node is quite a bit a lower than it should be.
                        //Later refinement attempts should fix this, but for now, do nothing.
                        //Other options:
                        //We could aggressively swap this node upward. More complicated.
                        break;
                    }
                    //It is very possible that this child pointer could swap between now and the compare exchange read. 
                    //However, a child pointer will not turn from an internal node (positive) to a leaf node (negative), and that's all that matters.
                    if (child.Index >= 0)
                    {
                        //Lock before comparing the children to stop the children from changing.
                        if (TryLock(ref child.Index))
                        {
                            //While we checked if children[i] != targetIndex earlier as an early-out, it must be done post-lock for correctness because children[i] could have changed.
                            //Attempting a swap between an index and itself is invalid.
                            if (child.Index != targetIndex)
                            {
                                //Now lock all of this child's children.
                                ref var childNode = ref Nodes[child.Index];
                                ref var grandchildren = ref childNode.A;
                                int lockedChildrenCount = 2;
                                for (int grandchildIndex = 0; grandchildIndex < 2; ++grandchildIndex)
                                {
                                    ref var grandchild = ref Unsafe.Add(ref grandchildren, grandchildIndex);
                                    //It is very possible that this grandchild pointer could swap between now and the compare exchange read. 
                                    //However, a child pointer will not turn from an internal node (positive) to a leaf node (negative), and that's all that matters.
                                    if (grandchild.Index >= 0 && !TryLock(ref grandchild.Index))
                                    {
                                        lockedChildrenCount = grandchildIndex;
                                        break;
                                    }
                                }
                                if (lockedChildrenCount == 2)
                                {
                                    Debug.Assert(node.RefineFlag == 1);
                                    if (!TrySwapNodeWithTargetThreadSafe(child.Index, nodeIndex, targetIndex))
                                    {
                                        //Failed target lock.
                                        success = false;
                                    }
                                    Debug.Assert(node.RefineFlag == 1);

                                }
                                else
                                {
                                    //Failed grandchild lock.
                                    success = false;
                                }

                                //Unlock all grandchildren.
                                //Note that we can't use the old grandchildren pointer. If the swap went through, it's pointing to the *target's* children.
                                //So update the pointer.
                                grandchildren = ref Nodes[child.Index].A;
                                for (int grandchildIndex = lockedChildrenCount - 1; grandchildIndex >= 0; --grandchildIndex)
                                {
                                    ref var grandchild = ref Unsafe.Add(ref grandchildren, grandchildIndex);
                                    if (grandchild.Index >= 0)
                                        Metanodes[grandchild.Index].RefineFlag = 0;
                                }

                            }
                            //Unlock. children[i] is either the targetIndex, if a swap went through, or it's the original child index if it didn't.
                            //Those are the proper targets.
                            Metanodes[child.Index].RefineFlag = 0;
                        }
                        else
                        {
                            //Failed child lock.
                            success = false;
                        }
                        //Leafcounts cannot change due to other threads.
                        targetIndex += child.LeafCount - 1; //Only works on 2-ary trees.
                    }
                }
                //Unlock the parent.
                node.RefineFlag = 0;
            }
            else
            {
                //Failed parent lock.
                success = false;
            }
            return success;
        }

        public unsafe void IncrementalCacheOptimize(int nodeIndex)
        {
            if (leafCount <= 2)
            {
                //Don't bother cache optimizing if there are only two leaves. There's no work to be done, and it supplies a guarantee to the rest of the optimization logic
                //so that we don't have to check per-node child counts.
                return;
            }

            ref var node = ref Nodes[nodeIndex];
            ref var children = ref node.A;
            var targetIndex = nodeIndex + 1;

            //Note that we pull all children up to their final positions relative to the current node index.
            //This helps ensure that more nodes can converge to their final positions- if we didn't do this,
            //a full top-down cache optimization could end up leaving some nodes near the bottom of the tree and without any room for their children.
            //TODO: N-ary tree support. Tricky without subtree count and without fixed numbers of children per node, but it may be possible
            //to stil choose something which converged.

            for (int i = 0; i < 2; ++i)
            {
                if (targetIndex >= nodeCount)
                {
                    //This attempted swap would reach beyond the allocated nodes.
                    //That means the current node is quite a bit a lower than it should be.
                    //Later refinement attempts should fix this, but for now, do nothing.
                    //Other options:
                    //We could aggressively swap this node upward. More complicated.
                    break;
                }
                ref var child = ref Unsafe.Add(ref children, i);
                if (child.Index >= 0)
                {
                    if (child.Index != targetIndex)
                    {
                        SwapNodes(child.Index, targetIndex);
                    }
                    //break;
                    targetIndex += child.LeafCount - 1;
                }
            }
        }



        unsafe void CacheOptimize(int nodeIndex, ref int nextIndex)
        {
            ref var node = ref Nodes[nodeIndex];
            ref var children = ref node.A;
            for (int i = 0; i < 2; ++i)
            {
                ref var child = ref Unsafe.Add(ref children, i);
                if (child.Index >= 0)
                {
                    Debug.Assert(nextIndex >= 0 && nextIndex < nodeCount,
                        "Swap target should be within the node set. If it's not, the initial node was probably not in global optimum position.");
                    if (child.Index != nextIndex)
                        SwapNodes(child.Index, nextIndex);
                    Debug.Assert(child.Index != nextIndex);
                    ++nextIndex;
                    CacheOptimize(child.Index, ref nextIndex);
                }
            }
        }


        /// <summary>
        /// Begins a cache optimization at the given node and proceeds all the way to the bottom of the tree.
        /// Requires that the targeted node is already at the global optimum position.
        /// </summary>
        /// <param name="nodeIndex">Node to begin the optimization process at.</param>
        public unsafe void CacheOptimize(int nodeIndex)
        {
            if (leafCount <= 2)
            {
                //Don't bother cache optimizing if there are only two leaves. There's no work to be done, and it supplies a guarantee to the rest of the optimization logic
                //so that we don't have to check per-node child counts.
                return;
            }
            var targetIndex = nodeIndex + 1;

            CacheOptimize(nodeIndex, ref targetIndex);
        }
    }

}
