using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

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
    }
}
