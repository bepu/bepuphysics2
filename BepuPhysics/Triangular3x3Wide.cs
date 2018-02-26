using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Stores the lower left triangle (including diagonal) of a 3x3 matrix. Useful for triangular forms and (anti)symmetric matrices.
    /// </summary>
    public struct Triangular3x3Wide
    {
        /// <summary>
        /// First row, first column of the matrix.
        /// </summary>
        public Vector<float> M11;
        /// <summary>
        /// Second row, first column of the matrix.
        /// </summary>
        public Vector<float> M21;
        /// <summary>
        /// Second row, second column of the matrix.
        /// </summary>
        public Vector<float> M22;
        /// <summary>
        /// Third row, first column of the matrix.
        /// </summary>
        public Vector<float> M31;
        /// <summary>
        /// Third row, second column of the matrix.
        /// </summary>
        public Vector<float> M32;
        /// <summary>
        /// Third row, third column of the matrix.
        /// </summary>
        public Vector<float> M33;

        /// <summary>
        /// Inverts the matrix as if it is a symmetric matrix where M32 == M23, M13 == M31, and M21 == M12.
        /// </summary>
        /// <param name="m">Symmetric matrix to invert.</param>
        /// <param name="inverse">Inverse of the symmetric matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SymmetricInvert(ref Triangular3x3Wide m, out Triangular3x3Wide inverse)
        {
            var m11 = m.M22 * m.M33 - m.M32 * m.M32;
            var m21 = m.M32 * m.M31 - m.M33 * m.M21;
            var m31 = m.M21 * m.M32 - m.M31 * m.M22;
            var determinantInverse = Vector<float>.One / (m11 * m.M11 + m21 * m.M21 + m31 * m.M31);

            var m22 = m.M33 * m.M11 - m.M31 * m.M31;
            var m32 = m.M31 * m.M21 - m.M11 * m.M32;

            var m33 = m.M11 * m.M22 - m.M21 * m.M21;

            inverse.M11 = m11 * determinantInverse;
            inverse.M21 = m21 * determinantInverse;
            inverse.M31 = m31 * determinantInverse;
            inverse.M22 = m22 * determinantInverse;
            inverse.M32 = m32 * determinantInverse;
            inverse.M33 = m33 * determinantInverse;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Triangular3x3Wide a, ref Triangular3x3Wide b, out Triangular3x3Wide sum)
        {
            sum.M11 = a.M11 + b.M11;
            sum.M21 = a.M21 + b.M21;
            sum.M22 = a.M22 + b.M22;
            sum.M31 = a.M31 + b.M31;
            sum.M32 = a.M32 + b.M32;
            sum.M33 = a.M33 + b.M33;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Triangular3x3Wide m, ref Vector<float> scale, out Triangular3x3Wide result)
        {
            result.M11 = m.M11 * scale;
            result.M21 = m.M21 * scale;
            result.M22 = m.M22 * scale;
            result.M31 = m.M31 * scale;
            result.M32 = m.M32 * scale;
            result.M33 = m.M33 * scale;
        }

        //If you ever need a triangular invert, a couple of options:
        //For matrices of the form:
        //[ 1   0   0 ]
        //[ M21 1   0 ]
        //[ M31 M32 1 ]
        //The inverse is simply:
        //       [ 1                0     0 ]
        //M^-1 = [ -M21             1     0 ]
        //       [ M21 * M32 - M31  -M32  1 ]

        //For a matrix with an arbitrary diagonal (that's still invertible):
        //       [ 1/M11                               0               0     ]
        //M^-1 = [ -M21/(M11*M22)                      1/M22           0     ]
        //       [ -(M22*M31 - M21*M32)/(M11*M22*M33)  -M32/(M22*M33)  1/M33 ]
        //And with some refiddling, you could make all the denominators the same to avoid repeated divisions.

        /// <summary>
        /// Computes skewSymmetric(v) * m * transpose(skewSymmetric(v)) for a symmetric matrix m. Assumes that the input and output matrices do not overlap.
        /// </summary>
        /// <param name="m">Symmetric matrix.</param>
        /// <param name="v">Vector to create the skew symmetric matrix from to act as the sandwich bread.</param>
        /// <param name="sandwich">Result of skewSymmetric(v) * m * transpose(skewSymmetric(v)).</param>
        /// <remarks>This operation might have a formal name that isn't skew sandwich. But that's okay, its real name is skew sandwich.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SkewSandwichWithoutOverlap(ref Vector3Wide v, ref Triangular3x3Wide m, out Triangular3x3Wide sandwich)
        {
            //27 muls, 15 adds.
            var x32 = v.X * m.M32;
            var y31 = v.Y * m.M31;
            var z21 = v.Z * m.M21;
            var i11 = y31 - z21;
            var i12 = v.Y * m.M32 - v.Z * m.M22;
            var i13 = v.Y * m.M33 - v.Z * m.M32;
            var i21 = v.Z * m.M11 - v.X * m.M31;
            var i22 = z21 - x32;
            var i23 = v.Z * m.M31 - v.X * m.M33;
            var i31 = v.X * m.M21 - v.Y * m.M11;
            var i32 = v.X * m.M22 - v.Y * m.M21;
            var i33 = x32 - y31;

            sandwich.M11 = v.Y * i13 - v.Z * i12;
            sandwich.M21 = v.Y * i23 - v.Z * i22;
            sandwich.M22 = v.Z * i21 - v.X * i23;
            sandwich.M31 = v.Y * i33 - v.Z * i32;
            sandwich.M32 = v.Z * i31 - v.X * i33;
            sandwich.M33 = v.X * i32 - v.Y * i31;
        }

        /// <summary>
        /// Computes v * m * transpose(v) for a symmetric matrix m. Assumes that the input and output do not overlap.
        /// </summary>
        /// <param name="v">Vector acting as the sandwich bread.</param>
        /// <param name="m">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of v * m * transpose(v) for a symmetric matrix m.</param>
        /// <remarks>Since I called the other one a skew sandwich, I really don't have a choice in the naming convention anymore.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void VectorSandwich(ref Vector3Wide v, ref Triangular3x3Wide m, out Vector<float> sandwich)
        {
            //This isn't actually fewer flops than the equivalent explicit operation, but it does avoid some struct locals and it's a pretty common operation.
            //(And at the moment, avoiding struct locals is unfortunately helpful for codegen reasons.)
            var x = v.X * m.M11 + v.Y * m.M21 + v.Z * m.M31;
            var y = v.X * m.M21 + v.Y * m.M22 + v.Z * m.M32;
            var z = v.X * m.M31 + v.Y * m.M32 + v.Z * m.M33;
            sandwich = x * v.X + y * v.Y + z * v.Z;
        }
        /// <summary>
        /// Computes rT * m * r for a symmetric matrix m and a rotation matrix R.
        /// </summary>
        /// <param name="r">Rotation matrix to use as the sandwich bread.</param>
        /// <param name="m">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of v * m * transpose(v) for a symmetric matrix m.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotationSandwich(ref Matrix3x3Wide r, ref Triangular3x3Wide m, out Triangular3x3Wide sandwich)
        {
            var i11 = r.X.X * m.M11 + r.Y.X * m.M21 + r.Z.X * m.M31;
            var i12 = r.X.X * m.M21 + r.Y.X * m.M22 + r.Z.X * m.M32;
            var i13 = r.X.X * m.M31 + r.Y.X * m.M32 + r.Z.X * m.M33;

            var i21 = r.X.Y * m.M11 + r.Y.Y * m.M21 + r.Z.Y * m.M31;
            var i22 = r.X.Y * m.M21 + r.Y.Y * m.M22 + r.Z.Y * m.M32;
            var i23 = r.X.Y * m.M31 + r.Y.Y * m.M32 + r.Z.Y * m.M33;

            var i31 = r.X.Z * m.M11 + r.Y.Z * m.M21 + r.Z.Z * m.M31;
            var i32 = r.X.Z * m.M21 + r.Y.Z * m.M22 + r.Z.Z * m.M32;
            var i33 = r.X.Z * m.M31 + r.Y.Z * m.M32 + r.Z.Z * m.M33;

            sandwich.M11 = i11 * r.X.X + i12 * r.Y.X + i13 * r.Z.X;
            sandwich.M21 = i21 * r.X.X + i22 * r.Y.X + i23 * r.Z.X;
            sandwich.M22 = i21 * r.X.Y + i22 * r.Y.Y + i23 * r.Z.Y;
            sandwich.M31 = i31 * r.X.X + i32 * r.Y.X + i33 * r.Z.X;
            sandwich.M32 = i31 * r.X.Y + i32 * r.Y.Y + i33 * r.Z.Y;
            sandwich.M33 = i31 * r.X.Z + i32 * r.Y.Z + i33 * r.Z.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyBySymmetricWithoutOverlap(ref Matrix2x3Wide a, ref Triangular3x3Wide t, out Matrix2x3Wide result)
        {
            result.X = 
        }

        /// <summary>
        /// Computes m * t * mT for a symmetric matrix t and a matrix m.
        /// </summary>
        /// <param name="m">Matrix to use as the sandwich bread.</param>
        /// <param name="t">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of m * t * mT for a symmetric matrix t.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MatrixSandwich(ref Matrix2x3Wide m, ref Triangular3x3Wide t, out Triangular2x2Wide result)
        {
            var i11 = m.X.X * t.M11 + m.X.Y * t.M21 + m.X.Z * t.M31;
            var i12 = m.X.X * t.M21 + m.X.Y * t.M22 + m.X.Z * t.M32;
            var i13 = m.X.X * t.M31 + m.X.Y * t.M32 + m.X.Z * t.M33;
            var i21 = m.Y.X * t.M11 + m.Y.Y * t.M21 + m.Y.Z * t.M31;
            var i22 = m.Y.X * t.M21 + m.Y.Y * t.M22 + m.Y.Z * t.M32;
            var i23 = m.Y.X * t.M31 + m.Y.Y * t.M32 + m.Y.Z * t.M33;
            result.M11 = i11 * m.X.X + i12 * m.X.Y + i13 * m.X.Z;
            result.M21 = i21 * m.X.X + i22 * m.X.Y + i23 * m.X.Z;
            result.M22 = i21 * m.Y.X + i22 * m.Y.Y + i23 * m.Y.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformBySymmetricWithoutOverlap(ref Vector3Wide v, ref Triangular3x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.M11 + v.Y * m.M21 + v.Z * m.M31;
            result.Y = v.X * m.M21 + v.Y * m.M22 + v.Z * m.M32;
            result.Z = v.X * m.M31 + v.Y * m.M32 + v.Z * m.M33;
        }



    }
}
