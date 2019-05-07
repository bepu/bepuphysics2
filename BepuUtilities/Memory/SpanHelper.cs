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
        private static void Validate<T>(ref Buffer<T> source, int sourceIndex, ref Buffer<T> target, int targetIndex, int count) where T : struct
        {
            Debug.Assert(targetIndex >= 0 && targetIndex + count <= target.Length, "Can't perform a copy that extends beyond the target span.");
            Debug.Assert(sourceIndex >= 0 && sourceIndex + count <= source.Length, "Can't perform a copy that extends beyond the source span.");
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref Buffer<T> source, int sourceIndex, ref Buffer<T> target, int targetIndex, int count) where T : struct
        {
            Validate(ref source, sourceIndex, ref target, targetIndex, count);
            var byteCount = count * Unsafe.SizeOf<T>();
            Buffer.MemoryCopy(
                source.Memory + sourceIndex * Unsafe.SizeOf<T>(),
                target.Memory + targetIndex * Unsafe.SizeOf<T>(),
                byteCount, byteCount);
        }

        //Copies between buffers and arrays of different types should be extremely rare in practice. Using a simple fallback is fine.
        //Note that we assume that the source is not aliased with the target. That's pretty safe.

        /// <summary>
        /// Copies data from a buffer to an array. Assumes the buffer does not overlap the array's memory.
        /// </summary>
        /// <typeparam name="T">Type of element being copied.</typeparam>
        /// <param name="source">Source buffer to pull elements from.</param>
        /// <param name="sourceIndex">Index in the buffer to start pulling elements from.</param>
        /// <param name="target">Target array to set values.</param>
        /// <param name="targetIndex">Index in the array to start putting elements into.</param>
        /// <param name="count">Number of elements to copy.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref Buffer<T> source, int sourceIndex, T[] target, int targetIndex, int count) where T : struct
        {
            for (int i = 0; i < count; ++i)
            {
                target[targetIndex + i] = source[sourceIndex + i];
            }
        }
        /// <summary>
        /// Copies data from an array to a buffer. Assumes the buffer does not overlap the array's memory.
        /// </summary>
        /// <typeparam name="T">Type of element being copied.</typeparam>
        /// <param name="source">Source array to pull elements from.</param>
        /// <param name="sourceIndex">Index in the array to start pulling elements from.</param>
        /// <param name="target">Target buffer to set values into.</param>
        /// <param name="targetIndex">Index in the buffer to start putting elements into.</param>
        /// <param name="count">Number of elements to copy.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(ref T[] source, int sourceIndex, ref Buffer<T> target, int targetIndex, int count) where T : struct
        {
            for (int i = 0; i < count; ++i)
            {
                target[targetIndex + i] = source[sourceIndex + i];
            }
        }
    }
}
