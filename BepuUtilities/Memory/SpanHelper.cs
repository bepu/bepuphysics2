using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuUtilities.Memory
{
    public static class SpanHelper
    {
        /// <summary>
        /// The highest size span exponent. The largest span is 2^MaximumSpanSizePower. This avoids overflow.
        /// </summary>
        public const int MaximumSpanSizePower = 30;

        [Conditional("DEBUG")]
        internal static void ValidatePower(int power)
        {
            Debug.Assert(power >= 0 && power <= MaximumSpanSizePower, $"Power must be from 0 to {MaximumSpanSizePower}, inclusive.");
        }
        /// <summary>
        /// Computes the largest integer N such that 2^N is less than or equal to i.
        /// </summary>
        /// <param name="i">Integer to compute the power of.</param>
        /// <returns>Loweset integer N such that 2^N is less than or equal to i.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetPowerOf2(int i)
        {
            int log = 0;
            if ((i & 0xFFFF0000) > 0)
            {
                i >>= 16;
                log |= 16;
            }
            if ((i & 0xFF00) > 0)
            {
                i >>= 8;
                log |= 8;
            }
            if ((i & 0xF0) > 0)
            {
                i >>= 4;
                log |= 4;
            }
            if ((i & 0xC) > 0)
            {
                i >>= 2;
                log |= 2;
            }
            if ((i & 0x2) > 0)
            {
                log |= 1;
            }
            return log;
        }
        /// <summary>
        /// Computes the lowest integer N such that 2^N >= i.
        /// </summary>
        /// <param name="i">Integer to compute the power of.</param>
        /// <returns>Loweset integer N such that 2^N >= i.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetContainingPowerOf2(int i)
        {
            Debug.Assert(i >= 0 && i <= (1 << MaximumSpanSizePower), "i must be from 0 to " + ((1 << MaximumSpanSizePower) - 1) + ", inclusive.");
            //We want the buffer which would fully contain the count, so it should be effectively Ceiling(Log(i)).
            //Doubling the value (and subtracting one, to avoid the already-a-power-of-two case) takes care of this.
            i = ((i > 0 ? i : 1) << 1) - 1;
            return GetPowerOf2(i);
        }

        /// <summary>
        /// Tests if a generic parameter is primitive. Fast path; specialized compilation.
        /// </summary>
        /// <typeparam name="T">Type to check for primitiveness.</typeparam>
        /// <returns>True if the type is one of the primitive types, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPrimitive<T>()
        {
            //The jit is able to specialize this, so the whole function just becomes a constant.
            return
                typeof(T) == typeof(bool) ||
                typeof(T) == typeof(byte) ||
                typeof(T) == typeof(sbyte) ||
                typeof(T) == typeof(ushort) ||
                typeof(T) == typeof(short) ||
                typeof(T) == typeof(uint) ||
                typeof(T) == typeof(int) ||
                typeof(T) == typeof(ulong) ||
                typeof(T) == typeof(long) ||
                typeof(T) == typeof(IntPtr) ||
                typeof(T) == typeof(UIntPtr) ||
                typeof(T) == typeof(char) ||
                typeof(T) == typeof(double) ||
                typeof(T) == typeof(float);
        }

        /// <summary>
        /// Tests if a type is primitive. Slow path; unspecialized compilation.
        /// </summary>
        /// <param name="type">Type to check for primitiveness.</typeparam>
        /// <returns>True if the type is one of the primitive types, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPrimitive(Type type)
        {
            //The jit CANNOT specialize this! Without a value type generic parameter, the jit doesn't generate different versions. 
            return
                type == typeof(bool) ||
                type == typeof(byte) ||
                type == typeof(sbyte) ||
                type == typeof(ushort) ||
                type == typeof(short) ||
                type == typeof(uint) ||
                type == typeof(int) ||
                type == typeof(ulong) ||
                type == typeof(long) ||
                type == typeof(IntPtr) ||
                type == typeof(UIntPtr) ||
                type == typeof(char) ||
                type == typeof(double) ||
                type == typeof(float);
        }

        [Conditional("DEBUG")]
        private static void Validate<T, TSourceSpan, TTargetSpan>(ref TSourceSpan source, int sourceIndex, ref TTargetSpan target, int targetIndex, int count)
            where TSourceSpan : ISpan<T>
            where TTargetSpan : ISpan<T>
        {
            Debug.Assert(targetIndex >= 0 && targetIndex + count <= target.Length, "Can't perform a copy that extends beyond the target span.");
            Debug.Assert(sourceIndex >= 0 && sourceIndex + count <= source.Length, "Can't perform a copy that extends beyond the source span.");
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref Buffer<T> source, int sourceIndex, ref Buffer<T> target, int targetIndex, int count)
        {
            Validate<T, Buffer<T>, Buffer<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                source.Memory + sourceIndex * Unsafe.SizeOf<T>(),
                target.Memory + targetIndex * Unsafe.SizeOf<T>(),
                byteCount, byteCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref Array<T> source, int sourceIndex, ref Array<T> target, int targetIndex, int count)
        {
            Validate<T, Array<T>, Array<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var byteCount = count * Unsafe.SizeOf<T>();
            Array.Copy(source.Memory, sourceIndex, target.Memory, targetIndex, count);
        }

        //Copies between spans of different types should be extremely rare in practice.
        //Due to that rareness, and the complexity involved in the overlap-isn't-a-problem guarantees of the function, just bite the bullet and pin.
        //This will have slightly worse performance, but it doesn't matter much.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref Buffer<T> source, int sourceIndex, ref Array<T> target, int targetIndex, int count)
        {
            Validate<T, Buffer<T>, Array<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var arrayHandle = GCHandle.Alloc(target.Memory, GCHandleType.Pinned);
            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                source.Memory + sourceIndex * Unsafe.SizeOf<T>(),
                Unsafe.AsPointer(ref target[targetIndex]),
                byteCount, byteCount);
            arrayHandle.Free();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref Array<T> source, int sourceIndex, ref Buffer<T> target, int targetIndex, int count)
        {
            Validate<T, Array<T>, Buffer<T>>(ref source, sourceIndex, ref target, targetIndex, count);
            var arrayHandle = GCHandle.Alloc(source.Memory, GCHandleType.Pinned);
            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                Unsafe.AsPointer(ref source[sourceIndex]),
                target.Memory + targetIndex * Unsafe.SizeOf<T>(),
                byteCount, byteCount);
            arrayHandle.Free();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void CopyFallback<T, TSourceSpan, TTargetSpan>(ref TSourceSpan source, int sourceIndex, ref TTargetSpan target, int targetIndex, int count)
            where TSourceSpan : ISpan<T>
            where TTargetSpan : ISpan<T>
        {
            Validate<T, TSourceSpan, TTargetSpan>(ref source, sourceIndex, ref target, targetIndex, count);
            var sourcePinned = source.TryPin(out var sourceHandle);
            var targetPinned = target.TryPin(out var targetHandle);

            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                Unsafe.AsPointer(ref source[sourceIndex]),
                Unsafe.AsPointer(ref target[targetIndex]),
                byteCount, byteCount);

            if (sourcePinned)
                sourceHandle.Free();
            if (targetPinned)
                targetHandle.Free();
        }

        /// <summary>
        /// Checks if the memory backing an instance is fully zeroed.
        /// </summary>
        /// <typeparam name="T">Type of the memory to check.</typeparam>
        /// <param name="memory">Memory to check.</param>
        /// <returns>True if all bytes backing the memory are zero, false otherwise.</returns>
        public static bool IsZeroed<T>(ref T memory) where T : struct
        {
            //This could be made much much faster, but at the moment this is only used in debug-related things. If you actually end up needing it elsewhere, consider
            //optimizing it to use wider types.
            var byteCount = Unsafe.SizeOf<T>();
            ref var byteStart = ref Unsafe.As<T, byte>(ref memory);
            for (int i = 0; i < byteCount; ++i)
            {
                if (Unsafe.Add(ref byteStart, i) != 0)
                    return false;
            }
            return true;
        }

        /// <summary>
        /// Checks if the memory backing a span region is fully zeroed.
        /// </summary>
        /// <typeparam name="T">Type of the instances in the span.</typeparam>
        /// <typeparam name="TSpan">Type of the span.</typeparam>
        /// <param name="span">Span to check for zeroing.</param>
        /// <param name="start">Inclusive start index of the region to check.</param>
        /// <param name="end">Exclusive end index of the region to check.</param>
        /// <returns>True if all bytes backing the memory are zero, false otherwise.</returns>
        public static bool IsZeroed<T, TSpan>(ref TSpan span, int start, int end) where TSpan : ISpan<T> where T : struct
        {
            //Again, this is way, way slower than it needs to be- it just doesn't matter right now.
            for (int i = start; i < end; ++i)
            {
                if (!IsZeroed(ref span[i]))
                    return false;
            }
            return true;
        }

    }
}
