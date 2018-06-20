using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuUtilities
{
    /// <summary>
    /// Stores the lower left triangle (including diagonal) of a 3x3 matrix. Useful for symmetric matrices (and sometimes antisymmetric matrices).
    /// </summary>
    public struct Symmetric3x3Wide
    {
        /// <summary>
        /// First row, first column of the matrix.
        /// </summary>
        public Vector<float> XX;
        /// <summary>
        /// Second row, first column of the matrix.
        /// </summary>
        public Vector<float> YX;
        /// <summary>
        /// Second row, second column of the matrix.
        /// </summary>
        public Vector<float> YY;
        /// <summary>
        /// Third row, first column of the matrix.
        /// </summary>
        public Vector<float> ZX;
        /// <summary>
        /// Third row, second column of the matrix.
        /// </summary>
        public Vector<float> ZY;
        /// <summary>
        /// Third row, third column of the matrix.
        /// </summary>
        public Vector<float> ZZ;

        /// <summary>
        /// Inverts the matrix as if it is a symmetric matrix where M32 == M23, M13 == M31, and M21 == M12.
        /// </summary>
        /// <param name="m">Symmetric matrix to invert.</param>
        /// <param name="inverse">Inverse of the symmetric matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in Symmetric3x3Wide m, out Symmetric3x3Wide inverse)
        {
            var xx = m.YY * m.ZZ - m.ZY * m.ZY;
            var yx = m.ZY * m.ZX - m.ZZ * m.YX;
            var zx = m.YX * m.ZY - m.ZX * m.YY;
            var determinantInverse = Vector<float>.One / (xx * m.XX + yx * m.YX + zx * m.ZX);

            var yy = m.ZZ * m.XX - m.ZX * m.ZX;
            var zy = m.ZX * m.YX - m.XX * m.ZY;

            var zz = m.XX * m.YY - m.YX * m.YX;

            inverse.XX = xx * determinantInverse;
            inverse.YX = yx * determinantInverse;
            inverse.ZX = zx * determinantInverse;
            inverse.YY = yy * determinantInverse;
            inverse.ZY = zy * determinantInverse;
            inverse.ZZ = zz * determinantInverse;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Symmetric3x3Wide a, in Symmetric3x3Wide b, out Symmetric3x3Wide sum)
        {
            sum.XX = a.XX + b.XX;
            sum.YX = a.YX + b.YX;
            sum.YY = a.YY + b.YY;
            sum.ZX = a.ZX + b.ZX;
            sum.ZY = a.ZY + b.ZY;
            sum.ZZ = a.ZZ + b.ZZ;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Symmetric3x3Wide m, in Vector<float> scale, out Symmetric3x3Wide result)
        {
            result.XX = m.XX * scale;
            result.YX = m.YX * scale;
            result.YY = m.YY * scale;
            result.ZX = m.ZX * scale;
            result.ZY = m.ZY * scale;
            result.ZZ = m.ZZ * scale;
        }

        //If you ever need a triangular invert, a couple of options:
        //For matrices of the form:
        //[ 1  0  0 ]
        //[ YX 1  0 ]
        //[ ZX ZY 1 ]
        //The inverse is simply:
        //       [ 1               0   0 ]
        //M^-1 = [ -YX             1   0 ]
        //       [ YX * ZY - ZX   -ZY  1 ]

        //For a matrix with an arbitrary diagonal (that's still invertible):
        //       [ 1/XX                         0               0    ]
        //M^-1 = [ -YX/(XX*YY)                  1/M22           0    ]
        //       [ -(YY*ZX - YX*ZY)/(XX*YY*ZZ)  -ZY/(YY*ZZ)     1/ZZ ]
        //And with some refiddling, you could make all the denominators the same to avoid repeated divisions.

        /// <summary>
        /// Computes skewSymmetric(v) * m * transpose(skewSymmetric(v)) for a symmetric matrix m. Assumes that the input and output matrices do not overlap.
        /// </summary>
        /// <param name="m">Symmetric matrix.</param>
        /// <param name="v">Vector to create the skew symmetric matrix from to act as the sandwich bread.</param>
        /// <param name="sandwich">Result of skewSymmetric(v) * m * transpose(skewSymmetric(v)).</param>
        /// <remarks>This operation might have a formal name that isn't skew sandwich. But that's okay, its real name is skew sandwich.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SkewSandwichWithoutOverlap(in Vector3Wide v, in Symmetric3x3Wide m, out Symmetric3x3Wide sandwich)
        {
            //27 muls, 15 adds.
            var xzy = v.X * m.ZY;
            var yzx = v.Y * m.ZX;
            var zyx = v.Z * m.YX;
            var ixx = yzx - zyx;
            var ixy = v.Y * m.ZY - v.Z * m.YY;
            var ixz = v.Y * m.ZZ - v.Z * m.ZY;
            var iyx = v.Z * m.XX - v.X * m.ZX;
            var iyy = zyx - xzy;
            var iyz = v.Z * m.ZX - v.X * m.ZZ;
            var izx = v.X * m.YX - v.Y * m.XX;
            var izy = v.X * m.YY - v.Y * m.YX;
            var izz = xzy - yzx;

            sandwich.XX = v.Y * ixz - v.Z * ixy;
            sandwich.YX = v.Y * iyz - v.Z * iyy;
            sandwich.YY = v.Z * iyx - v.X * iyz;
            sandwich.ZX = v.Y * izz - v.Z * izy;
            sandwich.ZY = v.Z * izx - v.X * izz;
            sandwich.ZZ = v.X * izy - v.Y * izx;
        }

        /// <summary>
        /// Computes v * m * transpose(v) for a symmetric matrix m. Assumes that the input and output do not overlap.
        /// </summary>
        /// <param name="v">Vector acting as the sandwich bread.</param>
        /// <param name="m">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of v * m * transpose(v) for a symmetric matrix m.</param>
        /// <remarks>Since I called the other one a skew sandwich, I really don't have a choice in the naming convention anymore.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void VectorSandwich(in Vector3Wide v, in Symmetric3x3Wide m, out Vector<float> sandwich)
        {
            //This isn't actually fewer flops than the equivalent explicit operation, but it does avoid some struct locals and it's a pretty common operation.
            //(And at the moment, avoiding struct locals is unfortunately helpful for codegen reasons.)
            var x = v.X * m.XX + v.Y * m.YX + v.Z * m.ZX;
            var y = v.X * m.YX + v.Y * m.YY + v.Z * m.ZY;
            var z = v.X * m.ZX + v.Y * m.ZY + v.Z * m.ZZ;
            sandwich = x * v.X + y * v.Y + z * v.Z;
        }
        /// <summary>
        /// Computes rT * m * r for a symmetric matrix m and a rotation matrix R.
        /// </summary>
        /// <param name="r">Rotation matrix to use as the sandwich bread.</param>
        /// <param name="m">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of v * m * transpose(v) for a symmetric matrix m.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotationSandwich(in Matrix3x3Wide r, in Symmetric3x3Wide m, out Symmetric3x3Wide sandwich)
        {
            var ixx = r.X.X * m.XX + r.Y.X * m.YX + r.Z.X * m.ZX;
            var ixy = r.X.X * m.YX + r.Y.X * m.YY + r.Z.X * m.ZY;
            var ixz = r.X.X * m.ZX + r.Y.X * m.ZY + r.Z.X * m.ZZ;

            var iyx = r.X.Y * m.XX + r.Y.Y * m.YX + r.Z.Y * m.ZX;
            var iyy = r.X.Y * m.YX + r.Y.Y * m.YY + r.Z.Y * m.ZY;
            var iyz = r.X.Y * m.ZX + r.Y.Y * m.ZY + r.Z.Y * m.ZZ;

            var izx = r.X.Z * m.XX + r.Y.Z * m.YX + r.Z.Z * m.ZX;
            var izy = r.X.Z * m.YX + r.Y.Z * m.YY + r.Z.Z * m.ZY;
            var izz = r.X.Z * m.ZX + r.Y.Z * m.ZY + r.Z.Z * m.ZZ;

            sandwich.XX = ixx * r.X.X + ixy * r.Y.X + ixz * r.Z.X;
            sandwich.YX = iyx * r.X.X + iyy * r.Y.X + iyz * r.Z.X;
            sandwich.YY = iyx * r.X.Y + iyy * r.Y.Y + iyz * r.Z.Y;
            sandwich.ZX = izx * r.X.X + izy * r.Y.X + izz * r.Z.X;
            sandwich.ZY = izx * r.X.Y + izy * r.Y.Y + izz * r.Z.Y;
            sandwich.ZZ = izx * r.X.Z + izy * r.Y.Z + izz * r.Z.Z;
        }

        /// <summary>
        /// Computes result = a * b, assuming that b represents a symmetric 3x3 matrix. Assumes that input parameters and output result do not overlap.
        /// </summary>
        /// <param name="a">First matrix of the pair to multiply.</param>
        /// <param name="b">Matrix to be reinterpreted as symmetric for the multiply.</param>
        /// <param name="result">Result of multiplying a * b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(in Matrix2x3Wide a, in Symmetric3x3Wide b, out Matrix2x3Wide result)
        {
            result.X.X = a.X.X * b.XX + a.X.Y * b.YX + a.X.Z * b.ZX;
            result.X.Y = a.X.X * b.YX + a.X.Y * b.YY + a.X.Z * b.ZY;
            result.X.Z = a.X.X * b.ZX + a.X.Y * b.ZY + a.X.Z * b.ZZ;
            result.Y.X = a.Y.X * b.XX + a.Y.Y * b.YX + a.Y.Z * b.ZX;
            result.Y.Y = a.Y.X * b.YX + a.Y.Y * b.YY + a.Y.Z * b.ZY;
            result.Y.Z = a.Y.X * b.ZX + a.Y.Y * b.ZY + a.Y.Z * b.ZZ;
        }

        /// <summary>
        /// Computes m * t * mT for a symmetric matrix t and a matrix m.
        /// </summary>
        /// <param name="m">Matrix to use as the sandwich bread.</param>
        /// <param name="t">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of m * t * mT for a symmetric matrix t.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MatrixSandwich(in Matrix2x3Wide m, in Symmetric3x3Wide t, out Symmetric2x2Wide result)
        {
            var ixx = m.X.X * t.XX + m.X.Y * t.YX + m.X.Z * t.ZX;
            var ixy = m.X.X * t.YX + m.X.Y * t.YY + m.X.Z * t.ZY;
            var ixz = m.X.X * t.ZX + m.X.Y * t.ZY + m.X.Z * t.ZZ;
            var iyx = m.Y.X * t.XX + m.Y.Y * t.YX + m.Y.Z * t.ZX;
            var iyy = m.Y.X * t.YX + m.Y.Y * t.YY + m.Y.Z * t.ZY;
            var iyz = m.Y.X * t.ZX + m.Y.Y * t.ZY + m.Y.Z * t.ZZ;
            result.XX = ixx * m.X.X + ixy * m.X.Y + ixz * m.X.Z;
            result.YX = iyx * m.X.X + iyy * m.X.Y + iyz * m.X.Z;
            result.YY = iyx * m.Y.X + iyy * m.Y.Y + iyz * m.Y.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector3Wide v, in Symmetric3x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.XX + v.Y * m.YX + v.Z * m.ZX;
            result.Y = v.X * m.YX + v.Y * m.YY + v.Z * m.ZY;
            result.Z = v.X * m.ZX + v.Y * m.ZY + v.Z * m.ZZ;
        }



    }
}
