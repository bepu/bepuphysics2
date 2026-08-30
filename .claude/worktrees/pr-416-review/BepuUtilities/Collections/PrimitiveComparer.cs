using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Provides optimized equality testing, comparison, and hashing for primitive types.
    /// </summary>
    /// <typeparam name="T">Type to compare and hash.</typeparam>
    public struct PrimitiveComparer<T> : IEqualityComparerRef<T>, IComparerRef<T>
    {
        //using T4 templates? pfah

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Compare(ref T a, ref T b)
        {
            if (typeof(T) == typeof(bool) || typeof(T) == typeof(byte))
            {
                var aTemp = Unsafe.As<T, byte>(ref a);
                var bTemp = Unsafe.As<T, byte>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(sbyte))
            {
                var aTemp = Unsafe.As<T, sbyte>(ref a);
                var bTemp = Unsafe.As<T, sbyte>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(short))
            {
                var aTemp = Unsafe.As<T, short>(ref a);
                var bTemp = Unsafe.As<T, short>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(ushort))
            {
                var aTemp = Unsafe.As<T, ushort>(ref a);
                var bTemp = Unsafe.As<T, ushort>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(int))
            {
                var aTemp = Unsafe.As<T, int>(ref a);
                var bTemp = Unsafe.As<T, int>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(uint))
            {
                var aTemp = Unsafe.As<T, uint>(ref a);
                var bTemp = Unsafe.As<T, uint>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(long))
            {
                var aTemp = Unsafe.As<T, long>(ref a);
                var bTemp = Unsafe.As<T, long>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(ulong))
            {
                var aTemp = Unsafe.As<T, ulong>(ref a);
                var bTemp = Unsafe.As<T, ulong>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(IntPtr))
            {
                unsafe
                {
                    var aTemp = Unsafe.As<T, IntPtr>(ref a).ToPointer();
                    var bTemp = Unsafe.As<T, IntPtr>(ref b).ToPointer();
                    return aTemp < bTemp ? -1 : aTemp > bTemp ? -1 : 0;
                }
            }
            if (typeof(T) == typeof(UIntPtr))
            {
                unsafe
                {
                    var aTemp = Unsafe.As<T, UIntPtr>(ref a).ToPointer();
                    var bTemp = Unsafe.As<T, UIntPtr>(ref b).ToPointer();
                    return aTemp < bTemp ? -1 : aTemp > bTemp ? -1 : 0;
                }
            }
            if (typeof(T) == typeof(char))
            {
                var aTemp = Unsafe.As<T, char>(ref a);
                var bTemp = Unsafe.As<T, char>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(double))
            {
                var aTemp = Unsafe.As<T, double>(ref a);
                var bTemp = Unsafe.As<T, double>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            if (typeof(T) == typeof(float))
            {
                var aTemp = Unsafe.As<T, float>(ref a);
                var bTemp = Unsafe.As<T, float>(ref b);
                return aTemp > bTemp ? 1 : aTemp < bTemp ? -1 : 0;
            }
            Debug.Assert(false, "Should only use the supported primitive types with the primitive comparer.");
            return 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref T a, ref T b)
        {
            if (typeof(T) == typeof(bool))
            {
                return Unsafe.As<T, bool>(ref a) == Unsafe.As<T, bool>(ref b);
            }
            if (typeof(T) == typeof(byte))
            {
                return Unsafe.As<T, byte>(ref a) == Unsafe.As<T, byte>(ref b);
            }
            if (typeof(T) == typeof(sbyte))
            {
                return Unsafe.As<T, sbyte>(ref a) == Unsafe.As<T, sbyte>(ref b);
            }
            if (typeof(T) == typeof(short))
            {
                return Unsafe.As<T, short>(ref a) == Unsafe.As<T, short>(ref b);
            }
            if (typeof(T) == typeof(ushort))
            {
                return Unsafe.As<T, ushort>(ref a) == Unsafe.As<T, ushort>(ref b);
            }
            if (typeof(T) == typeof(int))
            {
                return Unsafe.As<T, int>(ref a) == Unsafe.As<T, int>(ref b);
            }
            if (typeof(T) == typeof(uint))
            {
                return Unsafe.As<T, uint>(ref a) == Unsafe.As<T, uint>(ref b);
            }
            if (typeof(T) == typeof(long))
            {
                return Unsafe.As<T, long>(ref a) == Unsafe.As<T, long>(ref b);
            }
            if (typeof(T) == typeof(ulong))
            {
                return Unsafe.As<T, ulong>(ref a) == Unsafe.As<T, ulong>(ref b);
            }
            if (typeof(T) == typeof(IntPtr))
            {
                return Unsafe.As<T, IntPtr>(ref a) == Unsafe.As<T, IntPtr>(ref b);
            }
            if (typeof(T) == typeof(UIntPtr))
            {
                return Unsafe.As<T, UIntPtr>(ref a) == Unsafe.As<T, UIntPtr>(ref b);
            }
            if (typeof(T) == typeof(char))
            {
                return Unsafe.As<T, char>(ref a) == Unsafe.As<T, char>(ref b);
            }
            if (typeof(T) == typeof(double))
            {
                return Unsafe.As<T, double>(ref a) == Unsafe.As<T, double>(ref b);
            }
            if (typeof(T) == typeof(float))
            {
                return Unsafe.As<T, float>(ref a) == Unsafe.As<T, float>(ref b);
            }
            Debug.Assert(false, "Should only use the supported primitive types with the primitive comparer.");
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref T item)
        {
            //Note: the jit is able to inline the GetHashCodes; no need for custom implementations.
            if (typeof(T) == typeof(bool))
            {
                return Unsafe.As<T, bool>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(byte))
            {
                return Unsafe.As<T, byte>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(sbyte))
            {
                return Unsafe.As<T, sbyte>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(short))
            {
                return Unsafe.As<T, short>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(ushort))
            {
                return Unsafe.As<T, ushort>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(int))
            {
                return Unsafe.As<T, int>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(uint))
            {
                return Unsafe.As<T, uint>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(long))
            {
                return Unsafe.As<T, long>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(ulong))
            {
                return Unsafe.As<T, ulong>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(IntPtr))
            {
                return Unsafe.As<T, IntPtr>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(UIntPtr))
            {
                return Unsafe.As<T, UIntPtr>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(char))
            {
                return Unsafe.As<T, char>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(double))
            {
                return Unsafe.As<T, double>(ref item).GetHashCode();
            }
            if (typeof(T) == typeof(float))
            {
                return Unsafe.As<T, float>(ref item).GetHashCode();
            }
            Debug.Assert(false, "Should only use the supported primitive types with the primitive comparer.");
            return 0;
        }
    }
}
