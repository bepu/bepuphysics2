namespace BepuUtilities.Memory
{
    /// <summary>
    /// Defines a type that is capable of pooling blocks of unmanaged memory.
    /// </summary>
    public interface IUnmanagedMemoryPool
    {
        //TODO: In the future, when the unmanaged gets generalized for constructed types (https://github.com/dotnet/csharplang/issues/1937), we can tighten these generic constraints for safety.
        void TakeAtLeast<T>(int count, out Buffer<T> span) where T : struct;
        void Return<T>(ref Buffer<T> span) where T : struct;
        /// <summary>
        /// Gets the capacity of a buffer that would be returned by the pool if a given element count was requested.
        /// </summary>
        /// <typeparam name="T">Type of the elements being requested.</typeparam>
        /// <param name="count">Number of elements to request.</param>
        /// <returns>Capacity of a buffer that would be returned if the given element count was requested.</returns>
        int GetCapacityForCount<T>(int count) where T : struct;
    }
}