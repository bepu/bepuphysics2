namespace BepuUtilities.Memory
{
    /// <summary>
    /// Defines a type that is capable of rapidly serving requests for allocation and deallocation of unmanaged memory.
    /// </summary>
    public interface IUnmanagedMemoryPool
    {
        /// <summary>
        /// Takes a buffer large enough to contain a number of elements of a given type. Capacity may be larger than requested.
        /// </summary>
        /// <typeparam name="T">Type of the elements in the buffer.</typeparam>
        /// <param name="count">Desired minimum capacity of the buffer in typed elements.</param>
        /// <param name="buffer">Buffer large enough to contain the requested number of elements.</param>
        void TakeAtLeast<T>(int count, out Buffer<T> buffer) where T : unmanaged;
        /// <summary>
        /// Takes a typed buffer of the requested size from the pool.
        /// </summary>
        /// <typeparam name="T">Type of the instances in the buffer.</typeparam>
        /// <param name="count">Desired capacity of the buffer in typed elements.</param>
        /// <param name="buffer">Typed buffer of the requested size.</param>
        void Take<T>(int count, out Buffer<T> buffer) where T : unmanaged;
        /// <summary>
        /// Returns a buffer to the pool.
        /// </summary>
        /// <typeparam name="T">Type of the buffer's elements.</typeparam>
        /// <param name="buffer">Buffer to return to the pool.</param>
        void Return<T>(ref Buffer<T> buffer) where T : unmanaged;
        /// <summary>
        /// Gets the capacity of a buffer that would be returned by the pool if a given element count was requested from TakeAtLeast.
        /// </summary>
        /// <typeparam name="T">Type of the elements being requested.</typeparam>
        /// <param name="count">Number of elements to request.</param>
        /// <returns>Capacity of a buffer that would be returned if the given element count was requested.</returns>
        int GetCapacityForCount<T>(int count) where T : unmanaged;
    }
}