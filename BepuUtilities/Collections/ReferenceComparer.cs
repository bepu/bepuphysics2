using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Provides equality comparison and hashing for referenced types.
    /// </summary>
    /// <typeparam name="T">Type to compare for equality and hash.</typeparam>
    public struct ReferenceComparer<T> : IEqualityComparerRef<T> where T : class
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref T a, ref T b)
        {
            return ReferenceEquals(a, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref T item)
        {
            return item.GetHashCode();
        }
    }
}
