using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Packed indirection to data associated with a pair cache entry.
    /// </summary>
    public struct PairCacheIndex
    {
        internal ulong packed;

        /// <summary>
        /// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
        /// </summary>
        public bool Exists
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (packed & (1UL << 63)) > 0; }
        }

        /// <summary>
        /// Gets whether this index refers to an active cache entry. If false, the entry exists in an inactive set.
        /// </summary>
        public bool Active
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (packed & (1UL << 62)) > 0; }
        }


        /// <summary>
        /// Gets the index of the cache that owns the entry.
        /// </summary>
        public int Cache
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed >> 38) & 0x3FFFFF; } //24 bits
        }

        /// <summary>
        /// Gets the type index of the object.
        /// </summary>
        public int Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed >> 30) & 0xFF; } //8 bits
        }

        /// <summary>
        /// Gets the index of the object within the type specific list.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x3FFF_FFFF); } //30 bits
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairCacheIndex(int cache, int type, int index)
        {
            Debug.Assert(cache >= 0 && cache < (1 << 24), "Do you really have that many threads, or is the index corrupt?");
            Debug.Assert(type >= 0 && type < (1 << 8), "Do you really have that many type indices, or is the index corrupt?");
            //Note the inclusion of a set bit in the most significant 2 bits.
            //The MSB encodes that the index was explicitly constructed, so it is a 'real' reference.
            //A default constructed PairCacheIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
            //The second most significant bit sets the active flag. This constructor is used only by active references.
            packed = (ulong)((3L << 62) | ((long)cache << 38) | ((long)type << 30) | (long)index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PairCacheIndex CreateInactiveReference(int cache, int type, int index)
        {
            Debug.Assert(cache >= 0 && cache < (1 << 24), "Do you really have that many sets, or is the index corrupt?");
            Debug.Assert(type >= 0 && type < (1 << 8), "Do you really have that many type indices, or is the index corrupt?");
            //Note the inclusion of a set bit in the most significant 2 bits.
            //The MSB encodes that the index was explicitly constructed, so it is a 'real' reference.
            //A default constructed PairCacheIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
            //The second most significant bit is left unset. This function creates only inactive references..
            PairCacheIndex toReturn;
            toReturn.packed = (ulong)((1L << 63) | ((long)cache << 38) | ((long)type << 30) | (long)index);
            return toReturn;
        }

        public override string ToString()
        {
            return $"{{{Cache}, {Type}, {Index}}}";
        }

    }
}
