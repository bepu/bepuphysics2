using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuUtilities
{
    
    public struct Matrix2x3Wide
    {
        /// <summary>
        /// First row of the matrix.
        /// </summary>
        public Vector3Wide X;
        /// <summary>
        /// Second row of the matrix.
        /// </summary>
        public Vector3Wide Y;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(in Matrix2x3Wide a, in Matrix3x3Wide b, out Matrix2x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.Y.X + a.X.Z * b.Z.X;
            result.X.Y = a.X.X * b.X.Y + a.X.Y * b.Y.Y + a.X.Z * b.Z.Y;
            result.X.Z = a.X.X * b.X.Z + a.X.Y * b.Y.Z + a.X.Z * b.Z.Z;
            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.Y.X + a.Y.Z * b.Z.X;
            result.Y.Y = a.Y.X * b.X.Y + a.Y.Y * b.Y.Y + a.Y.Z * b.Z.Y;
            result.Y.Z = a.Y.X * b.X.Z + a.Y.Y * b.Y.Z + a.Y.Z * b.Z.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(in Matrix2x2Wide a, in Matrix2x3Wide b, out Matrix2x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.Y.X;
            result.X.Y = a.X.X * b.X.Y + a.X.Y * b.Y.Y;
            result.X.Z = a.X.X * b.X.Z + a.X.Y * b.Y.Z;
            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.Y.X;
            result.Y.Y = a.Y.X * b.X.Y + a.Y.Y * b.Y.Y;
            result.Y.Z = a.Y.X * b.X.Z + a.Y.Y * b.Y.Z;
        }

        /// <summary>
        /// Multiplies a matrix by another matrix, where the first matrix is sampled as if it were transposed: result = transpose(a) * b.
        /// </summary>
        /// <param name="a">Matrix to be sampled as if it were transposed when multiplied with the second matrix.</param>
        /// <param name="b">Second matrix in the pair.</param>
        /// <param name="result">Result of the multiplication transpose(a) * b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyTransposedWithoutOverlap(in Matrix2x2Wide a, in Matrix2x3Wide b, out Matrix2x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.Y.X * b.Y.X;
            result.X.Y = a.X.X * b.X.Y + a.Y.X * b.Y.Y;
            result.X.Z = a.X.X * b.X.Z + a.Y.X * b.Y.Z;
            result.Y.X = a.X.Y * b.X.X + a.Y.Y * b.Y.X;
            result.Y.Y = a.X.Y * b.X.Y + a.Y.Y * b.Y.Y;
            result.Y.Z = a.X.Y * b.X.Z + a.Y.Y * b.Y.Z;

        }

        /// <summary>
        /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
        /// </summary>
        /// <param name="a">First matrix in the pair.</param>
        /// <param name="b">Matrix to be sampled as if it were transposed when multiplied with the first matrix.</param>
        /// <param name="result">Result of the multiplication a * transpose(b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyByTransposeWithoutOverlap(in Matrix2x3Wide a, in Matrix2x3Wide b, out Matrix2x2Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.X.Y + a.X.Z * b.X.Z;
            result.X.Y = a.X.X * b.Y.X + a.X.Y * b.Y.Y + a.X.Z * b.Y.Z;
            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.X.Y + a.Y.Z * b.X.Z;
            result.Y.Y = a.Y.X * b.Y.X + a.Y.Y * b.Y.Y + a.Y.Z * b.Y.Z;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformByTransposeWithoutOverlap(in Vector3Wide v, in Matrix2x3Wide m, out Vector2Wide result)
        {
            result.X = v.X * m.X.X + v.Y * m.X.Y + v.Z * m.X.Z;
            result.Y = v.X * m.Y.X + v.Y * m.Y.Y + v.Z * m.Y.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in Matrix2x3Wide m, out Matrix2x3Wide result)
        {
            Vector3Wide.Negate(m.X, out result.X);
            Vector3Wide.Negate(m.Y, out result.Y);
        }

        /// <summary>
        /// Multiplies every component in the matrix by the given scalar value.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scaling value to apply to the matrix's components.</param>
        /// <param name="result">Resulting matrix with scaled components.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Matrix2x3Wide m, in Vector<float> scale, out Matrix2x3Wide result)
        {
            result.X.X = m.X.X * scale;
            result.X.Y = m.X.Y * scale;
            result.X.Z = m.X.Z * scale;
            result.Y.X = m.Y.X * scale;
            result.Y.Y = m.Y.Y * scale;
            result.Y.Z = m.Y.Z * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in Vector2Wide v, in Matrix2x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.X.X + v.Y * m.Y.X;
            result.Y = v.X * m.X.Y + v.Y * m.Y.Y;
            result.Z = v.X * m.X.Z + v.Y * m.Y.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Matrix2x3Wide a, in Matrix2x3Wide b, out Matrix2x3Wide result)
        {
            Vector3Wide.Add(a.X, b.X, out result.X);
            Vector3Wide.Add(a.Y, b.Y, out result.Y);
        }
    }
}
