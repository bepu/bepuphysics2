using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Packed indirection to data associated with a pair cache entry.
    /// </summary>
    public struct PairCacheIndex
    {
        ulong packed;

        /// <summary>
        /// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
        /// </summary>
        public bool Exists
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (packed & (1UL << 63)) > 0; }
        }

        /// <summary>
        /// Gets the worker index that created the entry.
        /// </summary>
        public int Worker
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed >> 48) & 0x7FFF; } //15 bits
        }

        /// <summary>
        /// Gets the type index of the object.
        /// </summary>
        public int Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed >> 40) & 0xFF; } //8 bits
        }

        /// <summary>
        /// Gets the index of the object.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0xFF_FFFF_FFFF); } //40 bits
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairCacheIndex(int worker, int type, int index)
        {
            Debug.Assert(worker >= 0 && worker < (1 << 15), "Do you really have that many threads, or is the index corrupt?");
            Debug.Assert(type >= 0 && type < (1 << 8), "Do you really have that many type indices, or is the index corrupt?");
            //Note the inclusion of a set bit in the most significant slot.
            //This encodes that the index was explicitly constructed, so it is a 'real' reference.
            //A default constructed PairCacheIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
            packed = (ulong)((1L << 63) | ((long)worker << 48) | ((long)type << 40) | (long)index);
        }

    }
}
