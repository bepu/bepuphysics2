using BepuUtilities.Collections;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Span over a managed memory region.
    /// </summary>
    /// <typeparam name="T">Type of the memory exposed by the span.</typeparam>
    public struct Array<T> : ISpan<T>
    {
        public readonly T[] Memory;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Array(T[] memory)
        {
            this.Memory = memory;
        }

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                //Spans are assumed to be unsafe anyway, so we can abandon bounds checking.
                //By using a regular array index, we force the jit to test bounds on potentially dynamically generated indices. If we instead take the reference of index 0
                //and offset from it, the jit can elide the bounds check in almost all repeated usages. There are also ways to completely eliminate the bounds check,
                //but it gets into a bit of brittle black magic...
                Debug.Assert(index >= 0 && index < Memory.Length);
                return ref Unsafe.Add(ref Memory[0], index);
                //return ref Memory[index];
            }
        }
        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Memory.Length;
            }
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
            System.Array.Clear(Memory, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearManagedReferences(int start, int count)
        {
            //Can't easily check to see if it contains *any* references recursively at compile time, but we can check for primitives with a compile time branch.
            //TODO: Unfortunately, the jit doesn't type specialize the type checks when T is a reference type- which was somewhat expected since all reference
            //types share the same implementation. However, no reference type is a primitive, so it seems like this is something that could be improved pretty easily...
            //In the interim, it's still valuable to have the primitive test for the common case where the type is in fact primitive.
            //(RuntimeHelpers.IsReferenceOrContainsReferences could solve this if it becomes available.)
            if (!SpanHelper.IsPrimitive<T>())
                System.Array.Clear(Memory, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo<TOtherSpan>(int sourceStart, ref TOtherSpan targetSpan, int targetStart, int count) where TOtherSpan : ISpan<T>
        {
            if (typeof(TOtherSpan) == typeof(Array<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, Array<T>>(ref targetSpan), targetStart, count);
            }
            else if (typeof(TOtherSpan) == typeof(Buffer<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, Buffer<T>>(ref targetSpan), targetStart, count);
            }
            else
            {
                SpanHelper.CopyFallback<T, Array<T>, TOtherSpan>(ref this, sourceStart, ref targetSpan, targetStart, count);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element, int start, int count)
        {
            return System.Array.IndexOf(Memory, element, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref T element, int start, int count)
        {
            return System.Array.IndexOf(Memory, element, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf<TPredicate>(ref TPredicate predicate, int start, int count) where TPredicate : IPredicate<T>
        {
            var end = start + count;
            for (int i = start; i < end; ++i)
            {
                if (predicate.Matches(ref Memory[i]))
                    return i;
            }
            return -1;
        }

        public bool TryPin(out GCHandle handle)
        {
            handle = GCHandle.Alloc(Memory, GCHandleType.Pinned);
            return true;
        }
    }



}