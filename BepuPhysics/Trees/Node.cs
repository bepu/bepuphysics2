using System.Numerics;
using System.Runtime.InteropServices;

namespace BepuPhysics.Trees
{
    [StructLayout(LayoutKind.Explicit)]
    public struct NodeChild
    {
        [FieldOffset(0)]
        public Vector3 Min;
        [FieldOffset(12)]
        public int Index;
        [FieldOffset(16)]
        public Vector3 Max;
        [FieldOffset(28)]
        public int LeafCount;
    }

    //Note that the format of this node implies that we don't explicitly test against the root bounding box during normal execution.
    //For almost all broad phase use cases, queries will be inside the root bounding box anyway. For non-broad phase uses, the outer bounding box will likely be stored
    //elsewhere- for example, in the broad phase.

    /// <summary>
    /// 2-wide tree node.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public unsafe struct Node
    {
        [FieldOffset(0)]
        public NodeChild A;
        [FieldOffset(32)]
        public NodeChild B;
    }

    //Node metadata isn't required or used during collision testing, so it is stored separately.
    //This helps avoid splitting Nodes across cache lines and decreases memory bandwidth requirements during testing.
    /// <summary>
    /// Metadata associated with a 2-child tree node.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public unsafe struct Metanode
    {
        [FieldOffset(0)]
        public int Parent;
        [FieldOffset(4)]
        public int IndexInParent;
        [FieldOffset(8)]
        public int RefineFlag;
        /// <summary>
        /// Cached change in cost of the tree starting at this node since the previous frame.
        /// The local cost change is unioned with the refine flags. They're never used simultaneously.
        /// This will be overwritten right after use, so don't expect anything meaningful here outside of refinement scheduling's scope.
        /// </summary>
        [FieldOffset(8)]
        public float LocalCostChange;

    }
}
