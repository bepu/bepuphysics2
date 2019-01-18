using BepuUtilities.Collections;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// A set of 3 integers, useful for spatial hashing.
    /// </summary>
    public struct Int3 : IEquatable<Int3>, IEqualityComparerRef<Int3>
    {
        public int X;
        public int Y;
        public int Z;

        public unsafe override int GetHashCode()
        {
            const ulong p1 = 961748927UL;
            const ulong p2 = 899809343UL;
            const ulong p3 = 715225741UL;
            var hash64 = (ulong)X * unchecked(p1 * p2 * p3) + (ulong)Y * (p2 * p3) + (ulong)Z * p3;
            return (int)(hash64 ^ (hash64 >> 32));
        }

        public override bool Equals(object obj)
        {
            return this.Equals((Int3)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Int3 other)
        {
            return other.X == X && other.Y == Y && other.Z == Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Int3 lhs, Int3 rhs)
        {
            return lhs.X == rhs.X && lhs.Y == rhs.Y && lhs.Z == rhs.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Int3 lhs, Int3 rhs)
        {
            return lhs.X != rhs.X || lhs.Y != rhs.Y || lhs.Z != rhs.Z;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"{{{X}, {Y}, {Z}}}";
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref Int3 item)
        {
            return item.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref Int3 a, ref Int3 b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z;
        }
    }

}
