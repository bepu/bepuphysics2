using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Stores the lower left triangle (including diagonal) of a 2x2 matrix. Useful for triangular forms and (anti)symmetric matrices.
    /// </summary>
    public struct Triangular2x2Wide
    {
        public Vector<float> M11;
        public Vector<float> M21;
        public Vector<float> M22;

        /// <summary>
        /// Computes m * scale * mT. 
        /// </summary>
        /// <param name="m">Matrix to sandwich the scale with.</param>
        /// <param name="scale">Scale to be sandwiched.</param>
        /// <param name="result">Result of m * scale * mT.</param>
        /// <remarks>This is a peculiar operation, but it's useful for computing linear effective mass contributions in 2DOF constraints.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SandwichScale(ref Matrix2x3Wide m, ref Vector<float> scale, out Triangular2x2Wide result)
        {
            result.M11 = scale * (m.X.X * m.X.X + m.X.Y * m.X.Y + m.X.Z * m.X.Z);
            result.M21 = scale * (m.Y.X * m.X.X + m.Y.Y * m.X.Y + m.Y.Z * m.X.Z);
            result.M22 = scale * (m.Y.X * m.Y.X + m.Y.Y * m.Y.Y + m.Y.Z * m.Y.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Triangular2x2Wide t, ref Vector<float> scale, out Triangular2x2Wide result)
        {
            result.M11 = t.M11 * scale;
            result.M21 = t.M21 * scale;
            result.M22 = t.M22 * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Triangular2x2Wide a, ref Triangular2x2Wide b, out Triangular2x2Wide result)
        {
            result.M11 = a.M11 + b.M11;
            result.M21 = a.M21 + b.M21;
            result.M22 = a.M22 + b.M22;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Triangular2x2Wide a, ref Triangular2x2Wide b, out Triangular2x2Wide result)
        {
            result.M11 = a.M11 - b.M11;
            result.M21 = a.M21 - b.M21;
            result.M22 = a.M22 - b.M22;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SymmetricInvertWithoutOverlap(ref Triangular2x2Wide m, out Triangular2x2Wide inverse)
        {
            var denom = Vector<float>.One / (m.M21 * m.M21 - m.M11 * m.M22);
            inverse.M11 = -m.M22 * denom;
            inverse.M21 = m.M21 * denom;
            inverse.M22 = -m.M11 * denom;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformBySymmetricWithoutOverlap(ref Vector2Wide v, ref Triangular2x2Wide m, out Vector2Wide result)
        {
            result.X = v.X * m.M11 + v.Y * m.M21;
            result.Y = v.X * m.M21 + v.Y * m.M22;
        }

        /// <summary>
        /// Computes result = transpose(transpose(a) * b), assuming b is symmetric.
        /// </summary>
        /// <param name="a">Matrix to be transposed and multiplied.</param>
        /// <param name="b">Symmetric matrix to multiply.</param>
        /// <param name="result">Result of transpose(transpose(a) * b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyTransposedBySymmetric(ref Matrix2x3Wide a, ref Triangular2x2Wide b, out Matrix2x3Wide result)
        {
            result.X.X = a.X.X * b.M11 + a.Y.X * b.M21;
            result.X.Y = a.X.X * b.M21 + a.Y.X * b.M22;
            result.X.Z = a.X.Y * b.M11 + a.Y.Y * b.M21;
            result.Y.X = a.X.Y * b.M21 + a.Y.Y * b.M22;
            result.Y.Y = a.X.Z * b.M11 + a.Y.Z * b.M21;
            result.Y.Z = a.X.Z * b.M21 + a.Y.Z * b.M22;
        }

        /// <summary>
        /// Computes a * transpose(b), assuming a = b * M. This is conceptually the second half of Triangular3x3Wide.MatrixSandwich.
        /// </summary>
        /// <param name="a">First matrix to multiply. Must be of the form a = b * M.</param>
        /// <param name="b">Matrix to be transaposed and multiplied with a..</param>
        /// <param name="result">Symmetric result of a * transpose(b), assuming a = b * M.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CompleteMatrixSandwich(ref Matrix2x3Wide a, ref Matrix2x3Wide b, out Triangular2x2Wide result)
        {
            result.M11 = a.X.X * b.X.X + a.X.Y * b.X.Y + a.X.Z * b.X.Z;
            result.M21 = a.Y.X * b.X.X + a.Y.Y * b.X.Y + a.Y.Z * b.X.Z;
            result.M22 = a.Y.X * b.Y.X + a.Y.Y * b.Y.Y + a.Y.Z * b.Y.Z;
        }

    }
}