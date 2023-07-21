using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Threading;

namespace BepuPhysics.Trees;

partial struct Tree
{    readonly unsafe void Refit2(ref NodeChild childInParent)
    {
        Debug.Assert(LeafCount >= 2);
        ref var node = ref Nodes[childInParent.Index];
        ref var a = ref node.A;
        if (node.A.Index >= 0)
        {
            Refit2(ref a);
        }
        ref var b = ref node.B;
        if (b.Index >= 0)
        {
            Refit2(ref b);
        }
        BoundingBox.CreateMergedUnsafeWithPreservation(a, b, out childInParent);
    }
    /// <summary>
    /// Updates the bounding boxes of all internal nodes in the tree.
    /// </summary>
    public unsafe readonly void Refit2()
    {
        //No point in refitting a tree with no internal nodes!
        if (LeafCount <= 2)
            return;
        NodeChild stub = default;
        Refit2(ref stub);
    }

    public void RefitBottomUp(BufferPool pool)
    {
        var refitFlags = new Buffer<ulong>((NodeCount + 63) / 64, pool);
        refitFlags.Clear(0, refitFlags.Length);
        //Refit doesn't calculate new bounding boxes for leaves; they are assumed to have already been supplied to the leaf-owning nodes.
        for (int i = 0; i < LeafCount; ++i)
        {
            var leaf = Leaves[i];
            var nodeIndex = leaf.NodeIndex;
            //There's nowhere to go from the root, so if the traversal gets there, it's done.
            //(If the leaf is owned by the root, that's fine; the direct owner of a leaf is assumed to have been updated by an external process before the refit.)
            while (nodeIndex > 0)
            {
                var bundleIndex = nodeIndex / 64;
                var indexInBundle = nodeIndex - bundleIndex * 64;
                var mask = 1ul << indexInBundle;
                ref var bundle = ref refitFlags[bundleIndex];
                if ((bundle & mask) == 0)
                {
                    //This node is incomplete. Mark it and terminate the local refit 'thread'.
                    bundle |= mask;
                    break;
                }
                //This node is complete; both children have been refit. Merge into the parent.
                ref var metanode = ref Metanodes[nodeIndex];
                nodeIndex = metanode.Parent;
                ref var node = ref Nodes[nodeIndex];
                ref var child = ref Unsafe.Add(ref node.A, metanode.IndexInParent);
                BoundingBox.CreateMergedUnsafeWithPreservation(node.A, node.B, out child);
            }
        }
        refitFlags.Dispose(pool);
    }
}
