using System;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Defines a type that is capable of rapidly serving requests for allocation and deallocation of unmanaged memory.
    /// </summary>
    public interface IUnmanagedMemoryPool : IDisposable
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
        /// <param name="buffer">Buffer to return to the pool. The reference will be cleared.</param>
        void Return<T>(ref Buffer<T> buffer) where T : unmanaged;
        /// <summary>
        /// Gets the capacity of a buffer that would be returned by the pool if a given element count was requested from TakeAtLeast.
        /// </summary>
        /// <typeparam name="T">Type of the elements being requested.</typeparam>
        /// <param name="count">Number of elements to request.</param>
        /// <returns>Capacity of a buffer that would be returned if the given element count was requested.</returns>
        int GetCapacityForCount<T>(int count) where T : unmanaged;

        /// <summary>
        /// Returns a buffer to the pool by id.
        /// </summary>
        /// <param name="id">Id of the buffer to return to the pool.</param>
        /// <remarks>Pools zero out the passed-in buffer by convention. This costs very little and avoids a wide variety of bugs (either directly or by forcing fast failure).
        /// This "Unsafe" overload should be used only in cases where there's a reason to bypass the clear; the naming is intended to dissuade casual use.</remarks>
        void ReturnUnsafely(int id);

        /// <summary>
        /// Resizes a typed buffer to the smallest size available in the pool which contains the target size. Copies a subset of elements into the new buffer. 
        /// Final buffer size is at least as large as the target size and may be larger.
        /// </summary>
        /// <typeparam name="T">Type of the buffer to resize.</typeparam>
        /// <param name="buffer">Buffer reference to resize.</param>
        /// <param name="targetSize">Number of elements to resize the buffer for.</param>
        /// <param name="copyCount">Number of elements to copy into the new buffer from the old buffer. Contents of slots outside the copied range in the resized buffer are undefined.</param>
        void ResizeToAtLeast<T>(ref Buffer<T> buffer, int targetSize, int copyCount) where T : unmanaged;

        /// <summary>
        /// Resizes a buffer to the target size. Copies a subset of elements into the new buffer.
        /// </summary>
        /// <typeparam name="T">Type of the buffer to resize.</typeparam>
        /// <param name="buffer">Buffer reference to resize.</param>
        /// <param name="targetSize">Number of elements to resize the buffer for.</param>
        /// <param name="copyCount">Number of elements to copy into the new buffer from the old buffer.</param>
        void Resize<T>(ref Buffer<T> buffer, int targetSize, int copyCount) where T : unmanaged;

        /// <summary>
        /// Returns all allocations in the pool to sources. Any outstanding buffers will be invalidated silently.
        /// The pool will remain in a usable state after clearing.
        /// </summary>
        void Clear();

        /// <summary>
        /// Computes the total number of bytes allocated from the memory source in this pool.
        /// Includes allocated memory regardless of whether it currently has outstanding references.
        /// </summary>
        /// <returns>Total number of bytes allocated from the backing memory source in this pool.</returns>
        ulong GetTotalAllocatedByteCount();
    }
}