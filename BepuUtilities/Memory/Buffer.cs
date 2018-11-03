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
    public unsafe struct Buffer<T> : ISpan<T> 
    {
        //TODO: Once blittable exists, replace this.
        public byte* Memory;
        int length;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Buffer<T> Slice(int start, int count)
        {
            ValidateRegion(start, count);
            return new Buffer<T>(Memory + Unsafe.SizeOf<T>() * start, count, Id);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Slice(int start, int count, out Buffer<T> sliced)
        {
            ValidateRegion(start, count);
            sliced = new Buffer<T>(Memory + Unsafe.SizeOf<T>() * start, count, Id);
        }

        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return length; }
        }

        public bool Allocated
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Memory != null;
            }
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear(int start, int count)
        {
            ValidateRegion(start, count);
            Unsafe.InitBlockUnaligned(Memory + Unsafe.SizeOf<T>() * start, 0, (uint)(count * Unsafe.SizeOf<T>()));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearManagedReferences(int start, int count)
        {
            //Pointer spans cannot validly hold managed references. nop!
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo<TOtherSpan>(int sourceStart, ref TOtherSpan targetSpan, int targetStart, int count) where TOtherSpan : ISpan<T>
        {
            if (typeof(TOtherSpan) == typeof(Buffer<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, Buffer<T>>(ref targetSpan), targetStart, count);
            }
            else if (typeof(TOtherSpan) == typeof(Array<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, Array<T>>(ref targetSpan), targetStart, count);
            }
            else
            {
                SpanHelper.CopyFallback<T, Buffer<T>, TOtherSpan>(ref this, sourceStart, ref targetSpan, targetStart, count);
            }
        }

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element, int start, int count)
        {
            return IndexOf(ref element, start, count);
        }

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryPin(out GCHandle handle)
        {
            //We assume that the pointer span is already pinned. This isn't actually guaranteed, but... in the situations where the backing memory isn't pinned, there's a bug.
            handle = new GCHandle();
            return false;
        }

        [Conditional("DEBUG")]
        void ValidateRegion(int start, int count)
        {
            Debug.Assert(start >= 0, "The start of a region must be within the buffer's extent.");
            Debug.Assert(start + count <= length, "The end of a region must be within the buffer's extent.");
        }
    }

}