using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{


    partial struct Tree
    {
        unsafe void CollectSubtreesForNodeDirect(int nodeIndex, int remainingDepth,
            ref QuickList<int> subtrees, ref QuickQueue<int> internalNodes, out float treeletCost)
        {
            internalNodes.EnqueueUnsafely(nodeIndex);

            treeletCost = 0;
            ref var node = ref Nodes[nodeIndex];
            ref var children = ref node.A;

            --remainingDepth;
            if (remainingDepth >= 0)
            {
                for (int i = 0; i < 2; ++i)
                {
                    ref var child = ref Unsafe.Add(ref children, i);
                    if (child.Index >= 0)
                    {
                        treeletCost += ComputeBoundsMetric(ref child.Min, ref child.Max);
                        float childCost;
                        CollectSubtreesForNodeDirect(child.Index, remainingDepth, ref subtrees, ref internalNodes, out childCost);
                        treeletCost += childCost;
                    }
                    else
                    {
                        //It's a leaf, immediately add it to subtrees.
                        subtrees.AddUnsafely(child.Index);
                    }
                }
            }
            else
            {
                //Recursion has bottomed out. Add every child.
                //Once again, note that the treelet costs of these nodes are not considered, even if they are internal.
                //That's because the subtree internal nodes cannot change size due to the refinement.
                for (int i = 0; i < 2; ++i)
                {
                    subtrees.AddUnsafely(Unsafe.Add(ref children, i).Index);
                }
            }
        }

        public unsafe void CollectSubtreesDirect(int nodeIndex, int maximumSubtrees,
            ref QuickList<int> subtrees, ref QuickQueue<int> internalNodes, out float treeletCost)
        {
            var maximumDepth = SpanHelper.GetContainingPowerOf2(maximumSubtrees) - 1;
            Debug.Assert(maximumDepth > 0);
            //Cost excludes the treelet root, since refinement can't change the treelet root's size. So don't bother including it in treeletCost.

            CollectSubtreesForNodeDirect(nodeIndex, maximumDepth, ref subtrees, ref internalNodes, out treeletCost);


        }




    }
}
