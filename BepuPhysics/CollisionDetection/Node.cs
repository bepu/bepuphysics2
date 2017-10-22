using System.Numerics;
using System.Runtime.InteropServices;

namespace BepuPhysics.CollisionDetection
{
    //There are a lot of improvements that can be made here, some noted. We're not going to mess with them yet.
    //There's a good chance that this entire layout will change later on, so it's not wise to spend a bunch of time trying to optimize it.
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

        //TODO: The only time that a per-node count should ever be anything but 2 is in the root node. We can make use of a tree-wide count or other approaches to deal with that.
        //This is only truly meaningful if we can achieve some more packing elsewhere.
        //TODO: A lot of this data is not accessed during all execution paths. If you split the node into two parallel arrays, you could save nontrivial amounts of memory bandwidth.
        //For example, in volume queries, sweep queries, ray queries, tree-tree intersections, and (most importantly for the broad phase) self tests, none of the following are used.
        [FieldOffset(64)]
        public int ChildCount; 
        [FieldOffset(68)]
        public int Parent;
        [FieldOffset(72)]
        public int IndexInParent;
        [FieldOffset(76)]
        public int RefineFlag;
        /// <summary>
        /// Cached change in cost of the tree starting at this node since the previous frame.
        /// The local cost change is unioned with the refine flags. They're never used simultaneously.
        /// This will be overwritten right after use, so don't expect anything meaningful here outside of refinement scheduling's scope.
        /// </summary>
        [FieldOffset(76)]
        public float LocalCostChange;

    }
}
