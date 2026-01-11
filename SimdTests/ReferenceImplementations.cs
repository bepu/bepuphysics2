using System.Numerics;
using System.Runtime.CompilerServices;

namespace SimdTests
{
    /// <summary>
    /// Software reference implementations for SIMD operations.
    /// These are scalar implementations used to verify correctness of SIMD code.
    /// </summary>
    public static class ReferenceImplementations
    {
        /// <summary>
        /// Reference implementation of FastReciprocal - computes 1/x for each element.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static Vector<float> FastReciprocal_Reference(Vector<float> v)
        {
            var result = new float[Vector<float>.Count];
            for (int i = 0; i < Vector<float>.Count; i++)
            {
                result[i] = 1.0f / v[i];
            }
            return new Vector<float>(result);
        }

        /// <summary>
        /// Reference implementation of FastReciprocalSquareRoot - computes 1/sqrt(x) for each element.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static Vector<float> FastReciprocalSquareRoot_Reference(Vector<float> v)
        {
            var result = new float[Vector<float>.Count];
            for (int i = 0; i < Vector<float>.Count; i++)
            {
                result[i] = 1.0f / MathF.Sqrt(v[i]);
            }
            return new Vector<float>(result);
        }

        /// <summary>
        /// Reference implementation of CreateTrailingMaskForCountInBundle.
        /// Creates a mask where elements at index >= countInBundle are set to -1, others to 0.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static Vector<int> CreateTrailingMaskForCountInBundle_Reference(int countInBundle)
        {
            var result = new int[Vector<int>.Count];
            for (int i = 0; i < Vector<int>.Count; i++)
            {
                result[i] = countInBundle <= i ? -1 : 0;
            }
            return new Vector<int>(result);
        }

        /// <summary>
        /// Reference implementation of CreateMaskForCountInBundle.
        /// Creates a mask where elements at index < countInBundle are set to -1, others to 0.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static Vector<int> CreateMaskForCountInBundle_Reference(int countInBundle)
        {
            var result = new int[Vector<int>.Count];
            for (int i = 0; i < Vector<int>.Count; i++)
            {
                result[i] = i < countInBundle ? -1 : 0;
            }
            return new Vector<int>(result);
        }

        /// <summary>
        /// Reference implementation of GetFirstSetLaneIndex.
        /// Returns the index of the first lane with a non-zero value, or -1 if all are zero.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static int GetFirstSetLaneIndex_Reference(Vector<int> mask)
        {
            for (int i = 0; i < Vector<int>.Count; i++)
            {
                if (mask[i] != 0)
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// Reference implementation of GetLastSetLaneCount.
        /// Returns the number of consecutive set lanes from the end.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static int GetLastSetLaneCount_Reference(Vector<int> mask)
        {
            int count = 0;
            for (int i = Vector<int>.Count - 1; i >= 0; i--)
            {
                if (mask[i] != 0)
                    count++;
                else
                    break;
            }
            return count;
        }
    }
}
