using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BepuUtilities
{
    /// <summary>
    /// Some helpers for indexing into vector bundles.
    /// </summary>
    public static class BundleIndexing
    {
        /// <summary>
        /// <![CDATA[Gets the mask value such that x & VectorMask computes x % Vector<float>.Count.]]>
        /// </summary>
        /// <remarks>The JIT recognizes that this value is constant!</remarks>
        public static int VectorMask
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Vector<float>.Count - 1;
            }
        }
        /// <summary>
        /// <![CDATA[Gets the shift value such that x >> VectorShift divides x by Vector<float>.Count.]]>
        /// </summary>
        /// <remarks>The JIT recognizes that this value is constant!</remarks>
        public static int VectorShift
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                switch (Vector<float>.Count)
                {
                    case 4:
                        return 2;
                    case 8:
                        return 3;
                    case 16:
                        return 4;
                    default:
                        return 0;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetBundleIndices(int linearIndex, out int bundleIndex, out int indexInBundle)
        {
            bundleIndex = linearIndex >> VectorShift;
            indexInBundle = linearIndex & VectorMask;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetBundleCount(int elementCount)
        {
            return (elementCount + VectorMask) >> VectorShift;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe Vector<int> CreateMaskForCountInBundle(int countInBundle)
        {
            if (Avx.IsSupported && Vector<int>.Count == 8)
            {
                return Avx.CompareGreaterThan(Vector256.Create((float)countInBundle), Vector256.Create(0f, 1f, 2f, 3f, 4f, 5f, 6f, 7f)).AsInt32().AsVector();
            }
            else if (Sse.IsSupported && Vector<int>.Count == 4)
            {
                return Sse.CompareGreaterThan(Vector128.Create((float)countInBundle), Vector128.Create(0f, 1f, 2f, 3f)).AsInt32().AsVector();
            }
            else
            {
                Vector<int> mask;
                var toReturnPointer = (int*)&mask;
                for (int i = 0; i < Vector<int>.Count; ++i)
                {
                    toReturnPointer[i] = i < countInBundle ? -1 : 0;
                }
                return mask;
            }
        }
    }
}
