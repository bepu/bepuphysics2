using BepuUtilities;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        unsafe void RemoveNodeAt(int nodeIndex)
        {
            //Note that this function is a cache scrambling influence. That's okay- the cache optimization routines will take care of it later.
            Debug.Assert(nodeIndex < nodeCount && nodeIndex >= 0);
            //We make no guarantees here about maintaining the tree's coherency after a remove.
            //That's the responsibility of whoever called RemoveAt.
            --nodeCount;
            //If the node wasn't the last node in the list, it will be replaced by the last node.
            if (nodeIndex < nodeCount)
            {
                //Swap last node for removed node.
                ref var node = ref Nodes[nodeIndex];
                node = Nodes[nodeCount];
                ref var metanode = ref Metanodes[nodeIndex];
                metanode = Metanodes[nodeCount];

                //Update the moved node's pointers:
                //its parent's child pointer should change, and...
                Unsafe.Add(ref Nodes[metanode.Parent].A, metanode.IndexInParent).Index = nodeIndex;
                //its children's parent pointers should change.
                ref var nodeChildren = ref node.A;
                for (int i = 0; i < 2; ++i)
                {
                    ref var child = ref Unsafe.Add(ref nodeChildren, i);
                    if (child.Index >= 0)
                    {
                        Metanodes[child.Index].Parent = nodeIndex;
                    }
                    else
                    {
                        //It's a leaf node. It needs to have its pointers updated.
                        Leaves[Encode(child.Index)] = new Leaf(nodeIndex, i);
                    }
                }

            }


        }


        unsafe void RefitForRemoval(int nodeIndex)
        {
            //Note that no attempt is made to refit the root node. Note that the root node is the only node that can have a number of children less than 2.
            ref var node = ref Nodes[nodeIndex];
            ref var metanode = ref Metanodes[nodeIndex];
            while (metanode.Parent >= 0)
            {
                //Compute the new bounding box for this node.
                ref var parent = ref Nodes[metanode.Parent];
                ref var childInParent = ref Unsafe.Add(ref parent.A, metanode.IndexInParent);
                BoundingBox.CreateMerged(node.A.Min, node.A.Max, node.B.Min, node.B.Max, out childInParent.Min, out childInParent.Max);
                --childInParent.LeafCount;
                node = ref parent;
                metanode = ref Metanodes[metanode.Parent];
            }
        }

        /// <summary>
        /// Removes a leaf at an index. If the index is not at the end of the leaf list, the last leaf is swapped into the removed location.
        /// </summary>
        /// <param name="leafIndex">Index of the leaf to remove.</param>
        /// <returns>Former index of the leaf that was moved into the removed leaf's slot, if any.
        /// If leafIndex pointed at the last slot in the list, then this returns -1 since no leaf was moved.</returns>
        public unsafe int RemoveAt(int leafIndex)
        {
            if (leafIndex < 0 || leafIndex >= leafCount)
                throw new ArgumentOutOfRangeException("Leaf index must be a valid index in the tree's leaf array.");

            //Cache the leaf being removed.
            var leaf = Leaves[leafIndex];
            //Delete the leaf from the leaves array.
            --leafCount;
            if (leafIndex < leafCount)
            {
                //The removed leaf was not the last leaf, so we should move the last leaf into its slot.
                //This can result in a form of cache scrambling, but these leaves do not need to be referenced during high performance stages.
                //It does somewhat reduce the performance of AABB updating, but we shouldn't bother with any form of cache optimization for this unless it becomes a proven issue.
                ref var lastLeaf = ref Leaves[leafCount];
                Leaves[leafIndex] = lastLeaf;
                Unsafe.Add(ref Nodes[lastLeaf.NodeIndex].A, lastLeaf.ChildIndex).Index = Encode(leafIndex);
            }

            ref var node = ref Nodes[leaf.NodeIndex];
            ref var metanode = ref Metanodes[leaf.NodeIndex];
            ref var nodeChildren = ref node.A;

            //Remove the leaf from this node.
            //Note that the children must remain contiguous. Requires taking the last child of the node and moving into the slot
            //if the removed child was not the last child.
            //Further, if a child is moved and if it is a leaf, that leaf's ChildIndex must be updated.
            //If a child is moved and it is an internal node, all immediate children of that node must have their parent nodes updated.

            var survivingChildIndexInNode = leaf.ChildIndex ^ 1;
            ref var survivingChild = ref Unsafe.Add(ref nodeChildren, survivingChildIndexInNode);

            //Check to see if this node should collapse.
            if (metanode.Parent >= 0)
            {
                //This is a non-root internal node.
                //Since there are only two children in the node, then the node containing the removed leaf will collapse.

                //Move the other node into the slot that used to point to the collapsing internal node.
                ref var childInParent = ref Unsafe.Add(ref Nodes[metanode.Parent].A, metanode.IndexInParent);
                childInParent.Min = survivingChild.Min;
                childInParent.Max = survivingChild.Max;
                childInParent.Index = survivingChild.Index;
                childInParent.LeafCount = survivingChild.LeafCount;

                if (survivingChild.Index < 0)
                {
                    //It's a leaf. Update the leaf's reference in the leaves array.
                    var otherLeafIndex = Encode(survivingChild.Index);
                    Leaves[otherLeafIndex] = new Leaf(metanode.Parent, metanode.IndexInParent);
                }
                else
                {
                    //It's an internal node. Update its parent node.
                    ref var survivingMeta = ref Metanodes[survivingChild.Index];
                    survivingMeta.Parent = metanode.Parent;
                    survivingMeta.IndexInParent = metanode.IndexInParent;
                }

                //Work up the chain of parent pointers, refitting bounding boxes and decrementing leaf counts.
                //Note that this starts at the parent; we've already done the refit for the current level via collapse.
                RefitForRemoval(metanode.Parent);

                //Remove the now dead node.
                RemoveNodeAt(leaf.NodeIndex);


            }
            else
            {
                //This is the root. It cannot collapse, but if the other child is an internal node, then it will overwrite the root node.
                //This maintains the guarantee that any tree with at least 2 leaf nodes has every single child slot filled with a node or leaf.
                Debug.Assert(leaf.NodeIndex == 0, "Only the root should have a negative parent, so only the root should show up here.");
                if (leafCount > 0)
                {
                    //The post-removal leafCount is still positive, so there must be at least one child in the root node.
                    //If it is an internal node, then it will be promoted into the root node's slot.
                    if (survivingChild.Index >= 0)
                    {
                        //The surviving child is an internal node and it should replace the root node.
                        var pulledNodeIndex = survivingChild.Index;
                        //TODO: This node movement logic could be unified with other instances of node moving. Nothing too special about the fact that it's the root.
                        Nodes[0] = Nodes[pulledNodeIndex]; //Note that this overwrites the memory pointed to by the otherChild reference.
                        Metanodes[0].Parent = -1;
                        Metanodes[0].IndexInParent = -1;
                        //Update the parent pointers of the children of the moved internal node.
                        for (int i = 0; i < 2; ++i)
                        {
                            ref var child = ref Unsafe.Add(ref Nodes[0].A, i);
                            if (child.Index >= 0)
                            {
                                //Child is an internal node. Note that the index in child doesn't change; we copied the children directly.
                                Metanodes[child.Index].Parent = 0;
                            }
                            else
                            {
                                //Child is a leaf node.
                                Leaves[Encode(child.Index)] = new Leaf(0, i);
                            }
                        }
                        RemoveNodeAt(pulledNodeIndex);
                    }
                    else
                    {
                        //The surviving child is a leaf node.
                        if (survivingChildIndexInNode > 0)
                        {
                            //It needs to be moved to keep the lowest slot filled.
                            Nodes[0].A = Nodes[0].B;
                            //Update the leaf pointer to reflect the change.
                            Leaves[Encode(survivingChild.Index)] = new Leaf(0, 0);
                        }
                    }
                }
                //No need to perform a RefitForRemoval here; it's the root. There is no higher bounding box.
            }
            return leafIndex < leafCount ? leafCount : -1;
        }
    }
}
