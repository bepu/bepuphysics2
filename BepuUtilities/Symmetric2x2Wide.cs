using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Stores the lower left triangle (including diagonal) of a 2x2 matrix.
    /// </summary>
    public struct Symmetric2x2Wide
    {
        public Vector<float> XX;
        public Vector<float> YX;
        public Vector<float> YY;

        /// <summary>
        /// Computes m * scale * mT. 
        /// </summary>
        /// <param name="m">Matrix to sandwich the scale with.</param>
        /// <param name="scale">Scale to be sandwiched.</param>
        /// <param name="result">Result of m * scale * mT.</param>
        /// <remarks>This is a peculiar operation, but it's useful for computing linear effective mass contributions in 2DOF constraints.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SandwichScale(in Matrix2x3Wide m, in Vector<float> scale, out Symmetric2x2Wide result)
        {
            result.XX = scale * (m.X.X * m.X.X + m.X.Y * m.X.Y + m.X.Z * m.X.Z);
            result.YX = scale * (m.Y.X * m.X.X + m.Y.Y * m.X.Y + m.Y.Z * m.X.Z);
            result.YY = scale * (m.Y.X * m.Y.X + m.Y.Y * m.Y.Y + m.Y.Z * m.Y.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Symmetric2x2Wide t, in Vector<float> scale, out Symmetric2x2Wide result)
        {
            result.XX = t.XX * scale;
            result.YX = t.YX * scale;
            result.YY = t.YY * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Symmetric2x2Wide a, in Symmetric2x2Wide b, out Symmetric2x2Wide result)
        {
            result.XX = a.XX + b.XX;
            result.YX = a.YX + b.YX;
            result.YY = a.YY + b.YY;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Symmetric2x2Wide a, in Symmetric2x2Wide b, out Symmetric2x2Wide result)
        {
            result.XX = a.XX - b.XX;
            result.YX = a.YX - b.YX;
            result.YY = a.YY - b.YY;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InvertWithoutOverlap(in Symmetric2x2Wide m, out Symmetric2x2Wide inverse)
        {
            var denom = Vector<float>.One / (m.YX * m.YX - m.XX * m.YY);
            inverse.XX = -m.YY * denom;
            inverse.YX = m.YX * denom;
            inverse.YY = -m.XX * denom;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector2Wide v, in Symmetric2x2Wide m, out Vector2Wide result)
        {
            result.X = v.X * m.XX + v.Y * m.YX;
            result.Y = v.X * m.YX + v.Y * m.YY;
        }

        /// <summary>
        /// Computes result = transpose(transpose(a) * b), assuming b is symmetric.
        /// </summary>
        /// <param name="a">Matrix to be transposed and multiplied.</param>
        /// <param name="b">Symmetric matrix to multiply.</param>
        /// <param name="result">Result of transpose(transpose(a) * b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyTransposed(in Matrix2x3Wide a, in Symmetric2x2Wide b, out Matrix2x3Wide result)
        {
            result.X.X = a.X.X * b.XX + a.Y.X * b.YX;
            result.X.Y = a.X.Y * b.XX + a.Y.Y * b.YX;
            result.X.Z = a.X.Z * b.XX + a.Y.Z * b.YX;
            result.Y.X = a.X.X * b.YX + a.Y.X * b.YY;
            result.Y.Y = a.X.Y * b.YX + a.Y.Y * b.YY;
            result.Y.Z = a.X.Z * b.YX + a.Y.Z * b.YY;
        }

        /// <summary>
        /// Computes a * transpose(b), assuming a = b * M for some symmetric matrix M. This is conceptually the second half of Triangular3x3Wide.MatrixSandwich.
        /// </summary>
        /// <param name="a">First matrix to multiply. Must be of the form a = b * M for some symmetric matrix M.</param>
        /// <param name="b">Matrix to be transaposed and multiplied with a..</param>
        /// <param name="result">Symmetric result of a * transpose(b), assuming a = b * M.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CompleteMatrixSandwich(in Matrix2x3Wide a, in Matrix2x3Wide b, out Symmetric2x2Wide result)
        {
            result.XX = a.X.X * b.X.X + a.X.Y * b.X.Y + a.X.Z * b.X.Z;
            result.YX = a.Y.X * b.X.X + a.Y.Y * b.X.Y + a.Y.Z * b.X.Z;
            result.YY = a.Y.X * b.Y.X + a.Y.Y * b.Y.Y + a.Y.Z * b.Y.Z;
        }

    }
}