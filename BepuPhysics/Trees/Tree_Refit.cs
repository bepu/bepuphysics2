using BepuUtilities;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BepuPhysics.Trees
{
    partial struct Tree
    {
        /// <summary>
        /// Refits the bounding box of every parent of the node recursively to the root.
        /// </summary>
        /// <param name="nodeIndex">Node to propagate a node change for.</param>
        public unsafe readonly void RefitForNodeBoundsChange(int nodeIndex)
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
                node = ref parent;
                metanode = ref Metanodes[metanode.Parent];
            }
        }

        readonly unsafe void Refit(ref NodeChild childInParent)
        {
            Debug.Assert(LeafCount >= 2);
            ref var node = ref Nodes[childInParent.Index];
            ref var a = ref node.A;
            if (node.A.Index >= 0)
            {
                Refit(ref a);
            }
            ref var b = ref node.B;
            if (b.Index >= 0)
            {
                Refit(ref b);
            }
            BoundingBox.CreateMergedUnsafeWithPreservation(a, b, out childInParent);
        }
        /// <summary>
        /// Updates the bounding boxes of all internal nodes in the tree.
        /// </summary>
        public unsafe readonly void Refit()
        {
            //No point in refitting a tree with no internal nodes!
            if (LeafCount <= 2)
                return;
            NodeChild stub = default;
            Refit(ref stub);
        }

    }
}
