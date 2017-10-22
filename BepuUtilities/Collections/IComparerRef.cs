namespace BepuUtilities.Collections
{
    /// <summary>
    /// Defines a type capable of comparing two objects passed by reference.
    /// </summary>
    /// <typeparam name="T">Type of the objects to compare.</typeparam>
    public interface IComparerRef<T>
    {
        int Compare(ref T a, ref T b);
    }
}
