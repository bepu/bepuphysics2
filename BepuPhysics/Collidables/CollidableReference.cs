using BepuUtilities.Collections;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    public enum CollidableMobility
    {
        /// <summary>
        /// Marks a collidable as owned by a dynamic body.
        /// </summary>
        Dynamic = 0,
        /// <summary>
        /// Marks a collidable as owned by a kinematic body.
        /// </summary>
        Kinematic = 1,
        /// <summary>
        /// Marks the collidable as an independent immobile collidable.
        /// </summary>
        Static = 2
    }

    public struct CollidableReference
    {
        public uint Packed;

        /// <summary>
        /// Gets or sets whether this reference points to a static collidable. If false, the reference points to a body.
        /// </summary>
        public CollidableMobility Mobility
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (CollidableMobility)(Packed >> 30); }
        }

        /// <summary>
        /// Gets or sets the handle of the collidable referred to by this instance.
        /// </summary>
        public int Handle
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(Packed & 0x3FFFFFFF); }
        }

        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CollidableReference(CollidableMobility mobility, int collidableHandle)
        {
            Debug.Assert((int)mobility >= 0 && (int)mobility <= 2, "Hey you, that mobility type doesn't exist. Or we changed something and this needs to be updated.");
            Debug.Assert(collidableHandle >= 0 && collidableHandle < 1 << 30, "Do you actually have more than 2^30 collidables? That seems unlikely.");
            Packed = ((uint)mobility << 30) | (uint)collidableHandle;
        }

        public override string ToString()
        {
            return $"{Mobility}: {Handle}";
        }
    }

    public struct CollidableReferenceComparer : IEqualityComparerRef<CollidableReference>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref CollidableReference a, ref CollidableReference b)
        {
            return a.Packed == b.Packed;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref CollidableReference item)
        {
            return (int)item.Packed;
        }
    }

}
