using BepuUtilities;
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
    public struct Triangular3x3
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
        /// Computes rT * m * r for a symmetric matrix m and a rotation matrix R.
        /// </summary>
        /// <param name="r">Rotation matrix to use as the sandwich bread.</param>
        /// <param name="m">Succulent interior symmetric matrix.</param>
        /// <param name="sandwich">Result of v * m * transpose(v) for a symmetric matrix m.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotationSandwich(ref Matrix3x3 r, ref Triangular3x3 m, out Triangular3x3 sandwich)
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
        /// Inverts the given matix.
        /// </summary>
        /// <param name="m">Matrix to be inverted.</param>
        /// <param name="inverse">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void SymmetricInvert(ref Triangular3x3 m, out Triangular3x3 inverse)
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
        public static void Add(ref Triangular3x3 a, ref Triangular3x3 b, out Triangular3x3 result)
        {
            result.XX = a.XX + b.XX;
            result.YX = a.YX + b.YX;
            result.YY = a.YY + b.YY;
            result.ZX = a.ZX + b.ZX;
            result.ZY = a.ZY + b.ZY;
            result.ZZ = a.ZZ + b.ZZ;
        }

        /// <summary>
        /// Multiplies every component in the matrix by the given scale.
        /// </summary>
        /// <param name="m">Matrix to be scaled.</param>
        /// <param name="scale">Scale to apply to every component of the original matrix.</param>
        /// <param name="scaled">Scaled result.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Triangular3x3 m, float scale, out Triangular3x3 scaled)
        {
            scaled.XX = m.XX * scale;
            scaled.YX = m.YX * scale;
            scaled.YY = m.YY * scale;
            scaled.ZX = m.ZX * scale;
            scaled.ZY = m.ZY * scale;
            scaled.ZZ = m.ZZ * scale;
        }
    }
}
