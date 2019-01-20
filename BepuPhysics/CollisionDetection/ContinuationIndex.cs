using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public struct CCDContinuationIndex
    {
        public uint Packed;

        //From least to most significant: 30 bits index, 1 bit type, 1 bit 'exists' flag.

        /// <summary>
        /// Gets the index of the continuation.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(Packed & 0x3FFFFFFF); }
        }
        
        /// <summary>
        /// Gets the type index of the continuation.
        /// </summary>
        public int Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)((Packed >> 30) & 1); }
        }

        /// <summary>
        /// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
        /// </summary>
        public bool Exists
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (Packed & (1 << 31)) > 0; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CCDContinuationIndex(int type, int index)
        {
            Debug.Assert(type >= 0 && type < 2, "Do you really have that many type indices, or is the index corrupt?");
            Debug.Assert(index >= 0 && index < (1 << 30), "Do you really have that many instances, or is the index corrupt?");
            //Note the inclusion of a set bit in the most significant slot.
            //This encodes that the index was explicitly constructed, so it is a 'real' reference.
            //A default constructed TypeIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
            Packed = (uint)((type << 30) | index | (1u << 31));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CCDContinuationIndex(int packed)
        {
            this.Packed = (uint)packed;
        }

        public override string ToString()
        {
            return $"<{Type}, {Index}>";
        }

    }
}
