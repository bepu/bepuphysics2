using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// IPredicate wrapper around an EqualityComparer and an object to compare against.
    /// </summary>
    /// <typeparam name="T">Type of the objects to compare.</typeparam>
    public struct WrapperPredicate<T> : IPredicate<T>
    {
        public T Item;
        public EqualityComparer<T> Comparer;
        /// <summary>
        /// Creates a default comparer for the given type.
        /// </summary>
        /// <param name="item">Item to compare against other items.</param>
        /// <param name="predicate">Predicate to test against other items using the default comparer for this type.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateDefault(ref T item, out WrapperPredicate<T> predicate)
        {
            predicate.Item = item;
            predicate.Comparer = EqualityComparer<T>.Default;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Matches(ref T otherItem)
        {
            return Comparer.Equals(Item, otherItem);
        }
    }


}
