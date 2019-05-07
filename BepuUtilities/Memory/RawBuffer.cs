using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Raw byte buffer with some helpers for interoperating with typed spans.
    /// </summary>
    public unsafe struct RawBuffer : IEquatable<RawBuffer>
    {
        public byte* Memory;
        public int Length;
        //We're primarily interested in x64, so memory + length is 12 bytes. This struct would/should get padded to 16 bytes for alignment reasons anyway, 
        //so making use of the last 4 bytes to speed up the case where the raw buffer is taken from a pool (which is basically always) is a good option.

        /// <summary>
        /// Implementation specific identifier of the raw buffer set by its source. If taken from a BufferPool, Id represents the packed power and internal power pool index from which it was taken.
        /// </summary>
        public int Id;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RawBuffer(void* memory, int length, int id = -1)
        {
            Memory = (byte*)memory;
            Length = length;
            Id = id;
        }

        public bool Allocated
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Memory != null;
            }
        }

        public ref byte this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref *(Memory + index);
            }
        }

        /// <summary>
        /// Interprets the bytes at the memory location as a given type.
        /// </summary>
        /// <typeparam name="T">Type to interpret the memory as.</typeparam>
        /// <param name="byteIndex">Memory location to interpret.</param>
        /// <returns>Reference to the memory as a given type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref T Interpret<T>(int byteIndex)
        {
            ValidateRegion(byteIndex, Unsafe.SizeOf<T>());
            return ref Unsafe.As<byte, T>(ref *(Memory + byteIndex));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RawBuffer Slice(int start, int count)
        {
            ValidateRegion(start, count);
            return new RawBuffer(Memory + start, count);
        }

        /// <summary>
        /// Takes a region of the raw buffer as a typed buffer.
        /// </summary>
        /// <typeparam name="T">Type to interpret the region as.</typeparam>
        /// <param name="start">Start of the region in terms of the type's size.</param>
        /// <param name="count">Number of elements in the region in terms of the type.</param>
        /// <returns>A typed buffer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Buffer<T> Slice<T>(int start, int count) where T : struct
        {
            ValidateRegion<T>(start, count);
            return new Buffer<T>(Memory + start * Unsafe.SizeOf<T>(), count * Unsafe.SizeOf<T>());
        }

        /// <summary>
        /// Creates a typed region from the raw buffer with the largest capacity that can fit within the allocated bytes.
        /// </summary>
        /// <typeparam name="T">Type of the buffer.</typeparam>
        /// <returns>Typed buffer of maximum extent within the current raw buffer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Buffer<T> As<T>() where T : struct
        {
            var count = Length / Unsafe.SizeOf<T>();
            return new Buffer<T>(Memory, count, Id);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear(int start, int count)
        {
            Unsafe.InitBlockUnaligned(Memory + start, 0, (uint)count);
        }

        //TODO: Some copies could be helpful, but let's wait until we actually need them.

        [Conditional("DEBUG")]
        void ValidateRegion<T>(int startInElements, int countInElements)
        {
            Debug.Assert(startInElements * Unsafe.SizeOf<T>() >= 0, "The start of a region must be within the buffer's extent.");
            Debug.Assert((startInElements + countInElements) * Unsafe.SizeOf<T>() <= Length, "The end of a region must be within the buffer's extent.");
        }

        [Conditional("DEBUG")]
        void ValidateRegion(int start, int count)
        {
            Debug.Assert(start >= 0, "The start of a region must be within the buffer's extent.");
            Debug.Assert(start + count <= Length, "The end of a region must be within the buffer's extent.");
        }

        //These are primarily used for debug purposes in the buffer pool.
        public override unsafe int GetHashCode()
        {
            if (IntPtr.Size == 4)
            {
                var temp = Memory;
                return *((int*)&temp);
            }
            else
            {
                var temp = Memory;
                return (*((long*)&temp)).GetHashCode();
            }
        }

        public bool Equals(RawBuffer other)
        {
            return other.Memory == Memory && other.Length == Length;
        }

        public override bool Equals(object obj)
        {
            if (obj is RawBuffer buffer)
                return Equals(buffer);
            return false;
        }

    }
}