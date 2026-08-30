namespace BepuUtilities.Collections
{
    /// <summary>
    /// Defines a type capable of performing the hashing and equality comparisons necessary for hash based collections.
    /// </summary>
    /// <typeparam name="T">Type of the elements to be hashed and compared.</typeparam>
    public interface IEqualityComparerRef<T>
    {
        int Hash(ref T item);
        bool Equals(ref T a, ref T b);
    }
}
