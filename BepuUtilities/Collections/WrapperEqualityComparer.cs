using BepuUtilities.Memory;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// IEqualityComparerRef wrapper around an EqualityComparer.
    /// </summary>
    /// <typeparam name="T">Type of the objects to compare and hash.</typeparam>
    public struct WrapperEqualityComparer<T> : IEqualityComparerRef<T>
    {
        public EqualityComparer<T> Comparer;
        /// <summary>
        /// Creates a default comparer for the given type.
        /// </summary>
        /// <param name="predicate">Predicate to test against other items using the default comparer for this type.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateDefault(out WrapperEqualityComparer<T> predicate)
        {
            predicate.Comparer = EqualityComparer<T>.Default;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref T a, ref T b)
        {
            return Comparer.Equals(a, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref T item)
        {
            return item.GetHashCode();
        }
    }
}
