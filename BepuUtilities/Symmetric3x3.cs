using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuUtilities
{
    /// <summary>
    /// Lower left triangle (including diagonal) of a symmetric 3x3 matrix.
    /// </summary>
    public struct Symmetric3x3
    {
        /// <summary>
        /// First row, first column of the matrix.
        /// </summary>
        public float XX;
        /// <summary>
        /// Second row, first column of the matrix.
        /// </summary>
        public float YX;
        /// <summary>
        /// Second row, second column of the matrix.
        /// </summary>
        public float YY;
        /// <summary>
        /// Third row, first column of the matrix.
        /// </summary>
        public float ZX;
        /// <summary>
        /// Third row, second column of the matrix.
        /// </summary>
        public float ZY;
        /// <summary>
        /// Third row, third column of the matrix.
        /// </summary>
        public float ZZ;

        /// <summary>
        /// Computes rT * m * r for a symmetric matrix m and a rotation matrix r.
        /// </summary>
        /// <param name="r">Rotation matrix to use as the sandwich bread.</param>
        /// <param name="m">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of transpose(r) * m * r.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotationSandwich(in Matrix3x3 r, in Symmetric3x3 m, out Symmetric3x3 sandwich)
        {
            //TODO: We just copied this from the wide implementation. There are a lot of ways to improve this, should it be necessary.
            //(There's virtually no chance that optimizing this to a serious degree would be worth it- at the time of writing, it's only called by the pose integrator, which is 
            //horribly memory bound anyway.)
            var i11 = r.X.X * m.XX + r.Y.X * m.YX + r.Z.X * m.ZX;
            var i12 = r.X.X * m.YX + r.Y.X * m.YY + r.Z.X * m.ZY;
            var i13 = r.X.X * m.ZX + r.Y.X * m.ZY + r.Z.X * m.ZZ;

            var i21 = r.X.Y * m.XX + r.Y.Y * m.YX + r.Z.Y * m.ZX;
            var i22 = r.X.Y * m.YX + r.Y.Y * m.YY + r.Z.Y * m.ZY;
            var i23 = r.X.Y * m.ZX + r.Y.Y * m.ZY + r.Z.Y * m.ZZ;

            var i31 = r.X.Z * m.XX + r.Y.Z * m.YX + r.Z.Z * m.ZX;
            var i32 = r.X.Z * m.YX + r.Y.Z * m.YY + r.Z.Z * m.ZY;
            var i33 = r.X.Z * m.ZX + r.Y.Z * m.ZY + r.Z.Z * m.ZZ;

            sandwich.XX = i11 * r.X.X + i12 * r.Y.X + i13 * r.Z.X;
            sandwich.YX = i21 * r.X.X + i22 * r.Y.X + i23 * r.Z.X;
            sandwich.YY = i21 * r.X.Y + i22 * r.Y.Y + i23 * r.Z.Y;
            sandwich.ZX = i31 * r.X.X + i32 * r.Y.X + i33 * r.Z.X;
            sandwich.ZY = i31 * r.X.Y + i32 * r.Y.Y + i33 * r.Z.Y;
            sandwich.ZZ = i31 * r.X.Z + i32 * r.Y.Z + i33 * r.Z.Z;
        }

        /// <summary>
        /// Computes the determinant of a symmetric matrix.
        /// </summary>
        /// <param name="m">Matrix to intepret as symmetric.</param>
        /// <returns>Determinant of the matrix interpreted as symmetric.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Determinant(in Symmetric3x3 m)
        {
            var m11 = m.YY * m.ZZ - m.ZY * m.ZY;
            var m21 = m.ZY * m.ZX - m.ZZ * m.YX;
            var m31 = m.YX * m.ZY - m.ZX * m.YY;
            return m11 * m.XX + m21 * m.YX + m31 * m.ZX;
        }

        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="m">Matrix to be inverted.</param>
        /// <param name="inverse">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Invert(in Symmetric3x3 m, out Symmetric3x3 inverse)
        {
            var m11 = m.YY * m.ZZ - m.ZY * m.ZY;
            var m21 = m.ZY * m.ZX - m.ZZ * m.YX;
            var m31 = m.YX * m.ZY - m.ZX * m.YY;
            var determinantInverse = 1f / (m11 * m.XX + m21 * m.YX + m31 * m.ZX);

            var m22 = m.ZZ * m.XX - m.ZX * m.ZX;
            var m32 = m.ZX * m.YX - m.XX * m.ZY;

            var m33 = m.XX * m.YY - m.YX * m.YX;

            inverse.XX = m11 * determinantInverse;
            inverse.YX = m21 * determinantInverse;
            inverse.ZX = m31 * determinantInverse;
            inverse.YY = m22 * determinantInverse;
            inverse.ZY = m32 * determinantInverse;
            inverse.ZZ = m33 * determinantInverse;
        }

        /// <summary>
        /// Adds the components of two matrices together.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Matrix with components equal to the components of the two input matrices added together.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Symmetric3x3 a, in Symmetric3x3 b, out Symmetric3x3 result)
        {
            result.XX = a.XX + b.XX;
            result.YX = a.YX + b.YX;
            result.YY = a.YY + b.YY;
            result.ZX = a.ZX + b.ZX;
            result.ZY = a.ZY + b.ZY;
            result.ZZ = a.ZZ + b.ZZ;
        }

        /// <summary>
        /// Subtracts the components of b from a.
        /// </summary>
        /// <param name="a">Matrix to be subtracted from.</param>
        /// <param name="b">Matrix to subtract from the first matrix..</param>
        /// <param name="result">Matrix with subtracted components.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Symmetric3x3 a, in Symmetric3x3 b, out Symmetric3x3 result)
        {
            result.XX = a.XX - b.XX;
            result.YX = a.YX - b.YX;
            result.YY = a.YY - b.YY;
            result.ZX = a.ZX - b.ZX;
            result.ZY = a.ZY - b.ZY;
            result.ZZ = a.ZZ - b.ZZ;
        }

        /// <summary>
        /// Adds the components of two matrices together.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Matrix with components equal to the components of the two input matrices added together.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Matrix3x3 a, in Symmetric3x3 b, out Matrix3x3 result)
        {
            var bX = new Vector3(b.XX, b.YX, b.ZX);
            var bY = new Vector3(b.YX, b.YY, b.ZY);
            var bZ = new Vector3(b.ZX, b.ZY, b.ZZ);
            result.X = a.X + bX;
            result.Y = a.Y + bY;
            result.Z = a.Z + bZ;
        }

        /// <summary>
        /// Subtracts the components of one matrix from another.
        /// </summary>
        /// <param name="a">Matrix to be subtracted from.</param>
        /// <param name="b">Matrix to subtract from the first matrix.</param>
        /// <param name="result">Matrix with components equal to the difference of the two input matrices.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Matrix3x3 a, in Symmetric3x3 b, out Matrix3x3 result)
        {
            var bX = new Vector3(b.XX, b.YX, b.ZX);
            var bY = new Vector3(b.YX, b.YY, b.ZY);
            var bZ = new Vector3(b.ZX, b.ZY, b.ZZ);
            result.X = a.X - bX;
            result.Y = a.Y - bY;
            result.Z = a.Z - bZ;
        }

        /// <summary>
        /// Multiplies every component in the matrix by the given scale.
        /// </summary>
        /// <param name="m">Matrix to be scaled.</param>
        /// <param name="scale">Scale to apply to every component of the original matrix.</param>
        /// <param name="scaled">Scaled result.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Symmetric3x3 m, float scale, out Symmetric3x3 scaled)
        {
            scaled.XX = m.XX * scale;
            scaled.YX = m.YX * scale;
            scaled.YY = m.YY * scale;
            scaled.ZX = m.ZX * scale;
            scaled.ZY = m.ZY * scale;
            scaled.ZZ = m.ZZ * scale;
        }

        /// <summary>
        /// Multiplies the two matrices as if they were symmetric.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(in Symmetric3x3 a, in Symmetric3x3 b, out Symmetric3x3 result)
        {
            var ayxbyx = a.YX * b.YX;
            var azxbzx = a.ZX * b.ZX;
            var azybzy = a.ZY * b.ZY;
            result.XX = a.XX * b.XX + ayxbyx + azxbzx;

            result.YX = a.YX * b.XX + a.YY * b.YX + a.ZY * b.ZX;
            result.YY = ayxbyx + a.YY * b.YY + azybzy;

            result.ZX = a.ZX * b.XX + a.ZY * b.YX + a.ZZ * b.ZX;
            result.ZY = a.ZX * b.YX + a.ZY * b.YY + a.ZZ * b.ZY;
            result.ZZ = azxbzx + azybzy + a.ZZ * b.ZZ;
        }


        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(in Matrix3x3 a, in Symmetric3x3 b, out Matrix3x3 result)
        {
            var bX = new Vector3(b.XX, b.YX, b.ZX);
            var bY = new Vector3(b.YX, b.YY, b.ZY);
            var bZ = new Vector3(b.ZX, b.ZY, b.ZZ);
            {
                var x = new Vector3(a.X.X);
                var y = new Vector3(a.X.Y);
                var z = new Vector3(a.X.Z);
                result.X = x * bX + y * bY + z * bZ;
            }

            {
                var x = new Vector3(a.Y.X);
                var y = new Vector3(a.Y.Y);
                var z = new Vector3(a.Y.Z);
                result.Y = x * bX + y * bY + z * bZ;
            }

            {
                var x = new Vector3(a.Z.X);
                var y = new Vector3(a.Z.Y);
                var z = new Vector3(a.Z.Z);
                result.Z = x * bX + y * bY + z * bZ;
            }
        }

        /// <summary>
        /// Transforms a vector by a symmetric matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="m">Matrix to interpret as symmetric transform.</param>
        /// <param name="result">Result of transforming the vector by the given symmetric matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector3 v, in Symmetric3x3 m, out Vector3 result)
        {
            result.X = v.X * m.XX + v.Y * m.YX + v.Z * m.ZX;
            result.Y = v.X * m.YX + v.Y * m.YY + v.Z * m.ZY;
            result.Z = v.X * m.ZX + v.Y * m.ZY + v.Z * m.ZZ;
        }

        public override string ToString()
        {
            return $"x: {XX}, y: {YX}, {YY}, z: {ZX}, {ZY}, {ZZ}";
        }
    }
}
