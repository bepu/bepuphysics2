using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public struct ContinuationIndex
    {
        uint packed;

        //From least to most significant: 11 bits inner index, 13 bits continuation index, 7 bits type, 1 bit 'exists' flag.


        /// <summary>
        /// Gets the inner index of the continuation.
        /// </summary>
        public int InnerIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x7FF); }
        }


        /// <summary>
        /// Gets the index of the continuation.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)((packed >> 11) & 0x1FFF); }
        }


        /// <summary>
        /// Gets the type index of the continuation.
        /// </summary>
        public int Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)((packed >> 24) & 0x7F); }
        }

        /// <summary>
        /// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
        /// </summary>
        public bool Exists
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (packed & (1 << 31)) > 0; }
        }

        public ContinuationIndex(int type, int index, int innerIndex)
        {
            Debug.Assert(type >= 0 && type < 128, "Do you really have that many type indices, or is the index corrupt?");
            Debug.Assert(index >= 0 && index < (1 << 13), "Do you really have that many instances, or is the index corrupt?");
            Debug.Assert(innerIndex >= 0 && innerIndex < (1 << 11), "Do you really have that many inner slots, or is the index corrupt?");
            //Note the inclusion of a set bit in the most significant slot.
            //This encodes that the index was explicitly constructed, so it is a 'real' reference.
            //A default constructed TypeIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
            packed = (uint)((type << 24) | (index << 11) | (innerIndex) | (1u << 31));
        }

        public override string ToString()
        {
            return $"<{Type}, {Index}, {InnerIndex}>";
        }

    }
}
