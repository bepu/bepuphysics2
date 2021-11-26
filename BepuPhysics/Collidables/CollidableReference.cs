using BepuUtilities.Collections;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Represents how a collidable can interact and move.
    /// </summary>
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

    /// <summary>
    /// Uses a bitpacked representation to refer to a body or static collidable.
    /// </summary>
    public struct CollidableReference : IEquatable<CollidableReference>
    {
        /// <summary>
        /// Bitpacked representation of the collidable reference.
        /// </summary>
        public uint Packed;

        /// <summary>
        /// Gets the mobility state of the owner of this collidable.
        /// </summary>
        public CollidableMobility Mobility
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (CollidableMobility)(Packed >> 30); }
        }

        /// <summary>
        /// Gets the body handle of the owner of the collidable referred to by this instance.
        /// </summary>
        public BodyHandle BodyHandle
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(Mobility == CollidableMobility.Dynamic || Mobility == CollidableMobility.Kinematic, "Extracting a body handle from a collidable reference requires that the collidable is owned by a body.");
                return new BodyHandle(RawHandleValue);
            }
        }


        /// <summary>
        /// Gets the static handle of the owner of the collidable referred to by this instance.
        /// </summary>
        public StaticHandle StaticHandle
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(Mobility == CollidableMobility.Static, "Extracting a static handle from a collidable reference requires that the collidable is owned by a static.");
                return new StaticHandle(RawHandleValue);
            }
        }

        /// <summary>
        /// Gets the integer value of the handle of the owner of the collidable referred to by this instance.
        /// </summary>
        public int RawHandleValue
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return (int)(Packed & 0x3FFFFFFF);
            }
        }


        /// <summary>
        /// Creates a collidable reference.
        /// </summary>
        /// <param name="mobility">Mobility type of the owner of the collidable.</param>
        /// <param name="handle">Handle of the owner of the collidable.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal CollidableReference(CollidableMobility mobility, int handle)
        {
            Debug.Assert((int)mobility >= 0 && (int)mobility <= 2, "Hey you, that mobility type doesn't exist. Or we changed something and this needs to be updated.");
            Debug.Assert(handle >= 0 && handle < 1 << 30, "Do you actually have more than 2^30 collidables? That seems unlikely.");
            Packed = ((uint)mobility << 30) | (uint)handle;
        }

        /// <summary>
        /// Creates a collidable reference for a body.
        /// </summary>
        /// <param name="mobility">Mobility type of the owner of the collidable.</param>
        /// <param name="handle">Handle of the owner of the collidable.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CollidableReference(CollidableMobility mobility, BodyHandle handle)
            : this(mobility, handle.Value)
        {
            Debug.Assert(mobility == CollidableMobility.Dynamic || mobility == CollidableMobility.Kinematic, "Creating a collidable reference associated with a body requires a body-related mobility.");
        }

        /// <summary>
        /// Creates a collidable reference for a static.
        /// </summary>
        /// <param name="handle">Handle of the owner of the collidable.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CollidableReference(StaticHandle handle)
            : this(CollidableMobility.Static, handle.Value)
        {
        }

        public override string ToString()
        {
            var handle = (Mobility == CollidableMobility.Static) ? StaticHandle.Value : BodyHandle.Value;
            return $"{Mobility}[{handle}]";
        }

        public bool Equals(CollidableReference other) => Packed == other.Packed;

        public override bool Equals(object other) => other is CollidableReference otherReference && Equals(otherReference);

        public static bool operator ==(CollidableReference x, CollidableReference y) => x.Packed == y.Packed;

        public static bool operator !=(CollidableReference x, CollidableReference y) => x.Packed != y.Packed;

        public override int GetHashCode() => (int)Packed;
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
