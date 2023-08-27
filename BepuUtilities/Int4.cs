using BepuUtilities.Collections;
using System;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// A set of 4 integers, useful for spatial hashing.
    /// </summary>
    public struct Int4 : IEquatable<Int4>, IEqualityComparerRef<Int4>
    {
        public int X;
        public int Y;
        public int Z;
        public int W;

        public unsafe override int GetHashCode()
        {
            const ulong p1 = 961748927UL;
            const ulong p2 = 899809343UL;
            const ulong p3 = 715225741UL;
            const ulong p4 = 472882027UL;
            var hash64 = (ulong)X * unchecked(p1 * p2 * p3) + (ulong)Y * (p2 * p3) + (ulong)Z * p3 + (ulong)W * p4;
            return (int)(hash64 ^ (hash64 >> 32));
        }

        public override bool Equals(object obj)
        {
            return Equals((Int4)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Int4 other)
        {
            return other.X == X && other.Y == Y && other.Z == Z && other.W == W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Int4 lhs, Int4 rhs)
        {
            return lhs.X == rhs.X && lhs.Y == rhs.Y && lhs.Z == rhs.Z && lhs.W == rhs.W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Int4 lhs, Int4 rhs)
        {
            return lhs.X != rhs.X || lhs.Y != rhs.Y || lhs.Z != rhs.Z || lhs.W != rhs.W;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"{{{X}, {Y}, {Z}, {W}}}";
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref Int4 item)
        {
            return item.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref Int4 a, ref Int4 b)
        {
            return a.X == b.X && a.Y == b.Y && a.Z == b.Z && a.W == b.W;
        }
    }

}
