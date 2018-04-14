using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Trees
{
    /// <summary>
    /// Pointer to a leaf's tree location.
    /// </summary>
    /// <remarks>The identity of a leaf is implicit in its position within the leaf array.</remarks>
    public struct Leaf
    {
        /// <summary>
        /// Gets the index of the node that the leaf is directly held by.
        /// </summary>
        public int NodeIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x7FFFFFFF); }
        }
        /// <summary>
        /// Gets which child within the owning node the leaf is in.
        /// </summary>
        public int ChildIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)((packed & 0x80000000) >> 31); }
        }

        uint packed;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Leaf(int nodeIndex, int childIndex)
        {
            Debug.Assert((childIndex & ~1) == 0, "Binary trees can't have children in slots other than 0 and 1!");
            packed = ((uint)nodeIndex & 0x7FFF_FFFF) | ((uint)childIndex << 31);
        }
    }

}
