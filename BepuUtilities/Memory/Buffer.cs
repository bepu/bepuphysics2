using BepuUtilities.Collections;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Span over an unmanaged memory region.
    /// </summary>
    /// <typeparam name="T">Type of the memory exposed by the span.</typeparam>
    public unsafe struct Buffer<T> where T : struct
    {
        //TODO: Once blittable exists, replace this.
        public byte* Memory;
        internal int length;
        //We're primarily interested in x64, so memory + length is 12 bytes. This struct would/should get padded to 16 bytes for alignment reasons anyway, 
        //so making use of the last 4 bytes to speed up the case where the raw buffer is taken from a pool (which is basically always) is a good option.

        /// <summary>
        /// Implementation specific identifier of the raw buffer set by its source. If taken from a BufferPool, Id represents the index in the power pool from which it was taken.
        /// </summary>
        public int Id;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Buffer(void* memory, int length, int id = -1)
        {
            Memory = (byte*)memory;
            this.length = length;
            Id = id;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T Get(byte* memory, int index)
        {
            return ref Unsafe.Add(ref Unsafe.As<byte, T>(ref *memory), index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T Get(ref RawBuffer buffer, int index)
        {
            Debug.Assert(index >= 0 && index * Unsafe.SizeOf<T>() < buffer.Length, "Index out of range.");
            return ref Get(buffer.Memory, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T Get(ref Buffer<T> buffer, int index)
        {
            Debug.Assert(index >= 0 && index < buffer.length, "Index out of range.");
            return ref Get(buffer.Memory, index);
        }

        /// <summary>
        /// Gets a reference to the element at the given index.
        /// </summary>
        /// <param name="index">Index of the element to grab a reference of.</param>
        /// <returns>Reference to the element at the given index.</returns>
        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                //TODO: As of this writing, the codegen for this isn't perfect. It's marginally better than doing bounds checks on a regular array.
                //Still not quite as fast as a raw Unsafe.Add on a properly typed ref, or a pure pointer index.
                //Hopefully, coreclr's Span<T> will result in some improvement here. 
                //No guarantee, though- they used to have a similar issue earlier in development but swapped to using internal intrinsics. 
                //Specifically, they're using:
                //return ref Unsafe.Add(ref _pointer.Value, index);
                //where _pointer is a ByReference<T>, which we cannot use.
                Debug.Assert(index >= 0 && index < length, "Index out of range.");
                return ref Get(Memory, index);
            }
        }

        /// <summary>
        /// Creates a view of a subset of the buffer's memory.
        /// </summary>
        /// <param name="start">Index at which to start the sliced buffer.</param>
        /// <param name="count">Number of elements to include in the sliced buffer.</param>
        /// <returns>Buffer spanning the specified subset of the original buffer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Buffer<T> Slice(int start, int count)
        {
            ValidateRegion(start, count);
            return new Buffer<T>(Memory + Unsafe.SizeOf<T>() * start, count, Id);
        }

        /// <summary>
        /// Creates a view of a subset of the buffer's memory, starting from the first index.
        /// </summary>
        /// <param name="count">Number of elements to include in the sliced buffer.</param>
        /// <returns>Buffer spanning the specified subset of the original buffer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Buffer<T> Slice(int count)
        {
            ValidateRegion(0, count);
            return new Buffer<T>(Memory, count, Id);
        }


        /// <summary>
        /// Creates a view of a subset of the buffer's memory.
        /// </summary>
        /// <param name="start">Index at which to start the sliced buffer.</param>
        /// <param name="count">Number of elements to include in the sliced buffer.</param>
        /// <param name="sliced">Buffer spanning the specified subset of the original buffer.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Slice(int start, int count, out Buffer<T> sliced)
        {
            ValidateRegion(start, count);
            sliced = new Buffer<T>(Memory + Unsafe.SizeOf<T>() * start, count, Id);
        }

        /// <summary>
        /// Creates a view of a subset of the buffer's memory, starting from the first index.
        /// </summary>
        /// <param name="count">Number of elements to include in the sliced buffer.</param>
        /// <param name="sliced">Buffer spanning the specified subset of the original buffer.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Slice(int count, out Buffer<T> sliced)
        {
            ValidateRegion(0, count);
            sliced = new Buffer<T>(Memory, count, Id);
        }

        /// <summary>
        /// Gets the length of the buffer in typed elements.
        /// </summary>
        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return length; }
        }

        /// <summary>
        /// Gets whether the buffer references non-null memory.
        /// </summary>
        public bool Allocated
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Memory != null;
            }
        }

        /// <summary>
        /// Zeroes out the buffer's memory.
        /// </summary>
        /// <param name="start">Start location in the buffer.</param>
        /// <param name="count">Number of elements to clear beyond the start index.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear(int start, int count)
        {
            ValidateRegion(start, count);
            Unsafe.InitBlockUnaligned(Memory + Unsafe.SizeOf<T>() * start, 0, (uint)(count * Unsafe.SizeOf<T>()));
        }

        /// <summary>
        /// Copies buffer data into another buffer.
        /// </summary>
        /// <param name="sourceStart">Start index in the source buffer.</param>
        /// <param name="target">Target buffer to copy into.</param>
        /// <param name="targetStart">Start index in the target buffer.</param>
        /// <param name="count">Number of elements to copy from the source buffer into the target buffer.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo(int sourceStart, ref Buffer<T> target, int targetStart, int count)
        {
            SpanHelper.Copy(ref this, sourceStart, ref target, targetStart, count);
        }

        /// <summary>
        /// Gets the index of an element in the buffer using the type's default comparer.
        /// </summary>
        /// <param name="element">Element to look for in the buffer.</param>
        /// <param name="start">Start index at which to begin the search.</param>
        /// <param name="count">Number of elements to scan beyond the start index.</param>
        /// <returns>Index of the element in the buffer if found, -1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref T element, int start, int count)
        {
            ValidateRegion(start, count);
            var end = start + count;
            if (SpanHelper.IsPrimitive<T>())
            {
                var defaultComparer = default(PrimitiveComparer<T>);
                for (int i = start; i < end; ++i)
                    if (defaultComparer.Equals(ref this[i], ref element))
                        return i;
                return -1;
            }
            else
            {
                WrapperEqualityComparer<T>.CreateDefault(out var defaultComparer);
                for (int i = start; i < end; ++i)
                    if (defaultComparer.Equals(ref this[i], ref element))
                        return i;
                return -1;
            }
        }

        /// <summary>
        /// Gets the index of an element in the buffer using the type's default comparer.
        /// </summary>
        /// <param name="element">Element to look for in the buffer.</param>
        /// <param name="start">Start index at which to begin the search.</param>
        /// <param name="count">Number of elements to scan beyond the start index.</param>
        /// <returns>Index of the element in the buffer if found, -1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element, int start, int count)
        {
            return IndexOf(ref element, start, count);
        }

        /// <summary>
        /// Gets the index of the first element that matches a provided predicate.
        /// </summary>
        /// <param name="predicate">Predicate to test each element with.</param>
        /// <param name="start">Start index at which to begin the search.</param>
        /// <param name="count">Number of elements to scan beyond the start index.</param>
        /// <returns>Index of the first matching element in the buffer if any, -1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf<TPredicate>(ref TPredicate predicate, int start, int count) where TPredicate : IPredicate<T>
        {
            ValidateRegion(start, count);
            var end = start + count;
            for (int i = start; i < end; ++i)
                if (predicate.Matches(ref this[i]))
                    return i;
            return -1;
        }

        /// <summary>
        /// Creates an untyped buffer containing the same data as the Buffer<typeparamref name="T"/>.
        /// </summary>
        /// <returns>Untyped buffer containing the same data as the source buffer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RawBuffer AsRaw()
        {
            RawBuffer buffer;
            buffer.Memory = Memory;
            buffer.Length = length * Unsafe.SizeOf<T>();
            buffer.Id = Id;
            return buffer;
        }

        [Conditional("DEBUG")]
        void ValidateRegion(int start, int count)
        {
            Debug.Assert(start >= 0, "The start of a region must be within the buffer's extent.");
            Debug.Assert(start + count <= length, "The end of a region must be within the buffer's extent.");
        }
    }

}