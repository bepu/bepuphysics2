using BepuUtilities.Collections;
using System;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Provides simple 2d cell hashing.
    /// </summary>
    public struct Int2 : IEquatable<Int2>, IEqualityComparerRef<Int2>
    {
        public int X;
        public int Y;

        public Int2(int x, int y)
        {
            X = x;
            Y = y;
        }

        public override bool Equals(object obj)
        {
            return Equals((Int2)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Int2 other)
        {
            return X == other.X && Y == other.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Int2 lhs, Int2 rhs)
        {
            return lhs.X == rhs.X && lhs.Y == rhs.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Int2 lhs, Int2 rhs)
        {
            return lhs.X != rhs.X || lhs.Y != rhs.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            const ulong p1 = 961748927UL;
            const ulong p2 = 899809343UL;
            var hash64 = (ulong)X * (p1 * p2) + (ulong)Y * (p2);
            return (int)(hash64 ^ (hash64 >> 32));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return $"{{{X}, {Y}}}";
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref Int2 item)
        {
            return item.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref Int2 a, ref Int2 b)
        {
            return a.X == b.X && a.Y == b.Y;
        }
    }
}
