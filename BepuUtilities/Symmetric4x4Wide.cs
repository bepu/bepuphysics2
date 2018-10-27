using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuUtilities
{
    public struct Symmetric4x4Wide
    {
        public Vector<float> XX;
        public Vector<float> YX;
        public Vector<float> YY;
        public Vector<float> ZX;
        public Vector<float> ZY;
        public Vector<float> ZZ;
        public Vector<float> WX;
        public Vector<float> WY;
        public Vector<float> WZ;
        public Vector<float> WW;

        /// <summary>
        /// Returns a reference to the upper left 3x3 block of the matrix.
        /// </summary>
        /// <param name="m">Matrix to pull a block from.</param>
        /// <returns>Reference to the requested block.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Symmetric3x3Wide GetUpperLeft3x3Block(ref Symmetric4x4Wide m)
        {
            return ref Unsafe.As<Vector<float>, Symmetric3x3Wide>(ref m.XX);
        }
        /// <summary>
        /// Returns a reference to the upper right 3x1 (or lower left 1x3) block of the matrix. 
        /// </summary>
        /// <param name="m">Matrix to pull a block from.</param>
        /// <returns>Reference to the requested block.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide GetUpperRight3x1Block(ref Symmetric4x4Wide m)
        {
            return ref Unsafe.As<Vector<float>, Vector3Wide>(ref m.WX);
        }

        /// <summary>
        /// Scales each component of m by the given scale.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scale to apply to the components of m.</param>
        /// <param name="result">Result of scaling each component of m by scale.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Symmetric4x4Wide m, in Vector<float> scale, out Symmetric4x4Wide result)
        {
            result.XX = m.XX * scale;
            result.YX = m.YX * scale;
            result.YY = m.YY * scale;
            result.ZX = m.ZX * scale;
            result.ZY = m.ZY * scale;
            result.ZZ = m.ZZ * scale;
            result.WX = m.WX * scale;
            result.WY = m.WY * scale;
            result.WZ = m.WZ * scale;
            result.WW = m.WW * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InvertWithoutOverlap(in Symmetric4x4Wide m, out Symmetric4x4Wide result)
        {
            var s0 = m.XX * m.YY - m.YX * m.YX;
            var s1 = m.XX * m.ZY - m.YX * m.ZX;
            var s2 = m.XX * m.WY - m.YX * m.WX;
            var s3 = m.YX * m.ZY - m.YY * m.ZX;
            var s4 = m.YX * m.WY - m.YY * m.WX;
            var s5 = m.ZX * m.WY - m.ZY * m.WX;
            var c5 = m.ZZ * m.WW - m.WZ * m.WZ;
            var c4 = m.ZY * m.WW - m.WY * m.WZ;
            var c3 = m.ZY * m.WZ - m.WY * m.ZZ;
            var c2 = m.ZX * m.WW - m.WX * m.WZ;
            var c1 = m.ZX * m.WZ - m.WX * m.ZZ;
            //c0 = s5

            var inverseDeterminant = Vector<float>.One / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * s5);

            result.XX = (m.YY * c5 - m.ZY * c4 + m.WY * c3) * inverseDeterminant;

            result.YX = (-m.YX * c5 + m.ZY * c2 - m.WY * c1) * inverseDeterminant;
            result.YY = (m.XX * c5 - m.ZX * c2 + m.WX * c1) * inverseDeterminant;

            result.ZX = (m.YX * c4 - m.YY * c2 + m.WY * s5) * inverseDeterminant;
            result.ZY = (-m.XX * c4 + m.YX * c2 - m.WX * s5) * inverseDeterminant;
            result.ZZ = (m.WX * s4 - m.WY * s2 + m.WW * s0) * inverseDeterminant;

            result.WX = (-m.YX * c3 + m.YY * c1 - m.ZY * s5) * inverseDeterminant;
            result.WY = (m.XX * c3 - m.YX * c1 + m.ZX * s5) * inverseDeterminant;
            result.WZ = (-m.WX * s3 + m.WY * s1 - m.WZ * s0) * inverseDeterminant;
            result.WW = (m.ZX * s3 - m.ZY * s1 + m.ZZ * s0) * inverseDeterminant;
        }
        
        /// <summary>
        /// Computes result = v * m.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="m">Matrix to transform with.</param>
        /// <param name="result">Result of the transform.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector4Wide v, Symmetric4x4Wide m, out Vector4Wide result)
        {
            result.X = v.X * m.XX + v.Y * m.YX + v.Z * m.ZX + v.W * m.WX;
            result.Y = v.X * m.YX + v.Y * m.YY + v.Z * m.ZY + v.W * m.WY;
            result.Z = v.X * m.ZX + v.Y * m.ZY + v.Z * m.ZZ + v.W * m.WZ;
            result.W = v.X * m.WX + v.Y * m.WY + v.Z * m.WZ + v.W * m.WW;
        }
    }
}
