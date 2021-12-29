using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

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
        /// Computes the lowest integer N such that 2^N >= i.
        /// </summary>
        /// <param name="i">Integer to compute the power of.</param>
        /// <returns>Lowest integer N such that 2^N >= i.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetContainingPowerOf2(int i)
        {
            var unsigned = i == 0 ? 1u : (uint)i;
            return 32 - BitOperations.LeadingZeroCount(unsigned - 1);
            
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


        /// <summary>
        /// Copies data from one buffer to another.
        /// </summary>
        /// <typeparam name="T">Type of element being copied.</typeparam>
        /// <param name="source">Source buffer to pull elements from.</param>
        /// <param name="sourceIndex">Index in the buffer to start pulling elements from.</param>
        /// <param name="target">Target buffer to set values.</param>
        /// <param name="targetIndex">Index in the buffer to start putting elements into.</param>
        /// <param name="count">Number of elements to copy.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(in Buffer<T> source, int sourceIndex, in Buffer<T> target, int targetIndex, int count) where T : unmanaged
        {
            Debug.Assert(targetIndex >= 0 && targetIndex + count <= target.Length, "Can't perform a copy that extends beyond the target span.");
            Debug.Assert(sourceIndex >= 0 && sourceIndex + count <= source.Length, "Can't perform a copy that extends beyond the source span.");
            Unsafe.CopyBlockUnaligned(target.Memory + targetIndex, source.Memory + sourceIndex, (uint)(Unsafe.SizeOf<T>() * count));
        }

        /// <summary>
        /// Copies data from a buffer to a span.
        /// </summary>
        /// <typeparam name="T">Type of element being copied.</typeparam>
        /// <param name="source">Source buffer to pull elements from.</param>
        /// <param name="sourceIndex">Index in the buffer to start pulling elements from.</param>
        /// <param name="target">Target span to set values.</param>
        /// <param name="targetIndex">Index in the span to start putting elements into.</param>
        /// <param name="count">Number of elements to copy.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(in Buffer<T> source, int sourceIndex, Span<T> target, int targetIndex, int count) where T : unmanaged
        {
            Debug.Assert(targetIndex >= 0 && targetIndex + count <= target.Length, "Can't perform a copy that extends beyond the target span.");
            Debug.Assert(sourceIndex >= 0 && sourceIndex + count <= source.Length, "Can't perform a copy that extends beyond the source span.");
            Unsafe.CopyBlockUnaligned(ref Unsafe.As<T, byte>(ref target[targetIndex]), ref Unsafe.AsRef<byte>(source.Memory + sourceIndex), (uint)(Unsafe.SizeOf<T>() * count));
        }

        /// <summary>
        /// Copies data from a span to a buffer.
        /// </summary>
        /// <typeparam name="T">Type of element being copied.</typeparam>
        /// <param name="source">Source span to pull elements from.</param>
        /// <param name="sourceIndex">Index in the span to start pulling elements from.</param>
        /// <param name="target">Target buffer to set values into.</param>
        /// <param name="targetIndex">Index in the buffer to start putting elements into.</param>
        /// <param name="count">Number of elements to copy.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(in Span<T> source, int sourceIndex, in Buffer<T> target, int targetIndex, int count) where T : unmanaged
        {
            Debug.Assert(targetIndex >= 0 && targetIndex + count <= target.Length, "Can't perform a copy that extends beyond the target span.");
            Debug.Assert(sourceIndex >= 0 && sourceIndex + count <= source.Length, "Can't perform a copy that extends beyond the source span.");
            Unsafe.CopyBlockUnaligned(ref Unsafe.AsRef<byte>(target.Memory + targetIndex), ref Unsafe.As<T, byte>(ref source[sourceIndex]), (uint)(Unsafe.SizeOf<T>() * count));
        }

        /// <summary>
        /// Copies data from a span to a buffer.
        /// </summary>
        /// <typeparam name="T">Type of element being copied.</typeparam>
        /// <param name="source">Source span to pull elements from.</param>
        /// <param name="sourceIndex">Index in the span to start pulling elements from.</param>
        /// <param name="target">Target buffer to set values into.</param>
        /// <param name="targetIndex">Index in the buffer to start putting elements into.</param>
        /// <param name="count">Number of elements to copy.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Copy<T>(in ReadOnlySpan<T> source, int sourceIndex, in Buffer<T> target, int targetIndex, int count) where T : unmanaged
        {
            Debug.Assert(targetIndex >= 0 && targetIndex + count <= target.Length, "Can't perform a copy that extends beyond the target span.");
            Debug.Assert(sourceIndex >= 0 && sourceIndex + count <= source.Length, "Can't perform a copy that extends beyond the source span.");
            source.Slice(sourceIndex, count).CopyTo(new Span<T>(target.Memory + targetIndex, count));
        }
    }
}
