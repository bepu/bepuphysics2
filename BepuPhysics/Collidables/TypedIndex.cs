using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    public struct TypedIndex
    {
        uint packed;

        /// <summary>
        /// Gets the type index of the object.
        /// </summary>
        public int Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x7F000000) >> 24; }
        }

        /// <summary>
        /// Gets the index of the object.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x00FFFFFF); }
        }

        /// <summary>
        /// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
        /// </summary>
        public bool Exists
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (packed & (1 << 31)) > 0; }
        }
        
        public TypedIndex(int type, int index)
        {
            Debug.Assert(type >= 0 && type < 128, "Do you really have that many type indices, or is the index corrupt?");
            Debug.Assert(index >= 0 && index < (1 << 24), "Do you really have that many instances, or is the index corrupt?");
            //Note the inclusion of a set bit in the most significant slot.
            //This encodes that the index was explicitly constructed, so it is a 'real' reference.
            //A default constructed TypeIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
            packed = (uint)((type << 24) | index | (1u << 31));
        }

        public override string ToString()
        {
            return $"<{Type}, {Index}>";
        }

    }
}
