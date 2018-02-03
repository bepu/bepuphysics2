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
        public float M11;
        /// <summary>
        /// Second row, first column of the matrix.
        /// </summary>
        public float M21;
        /// <summary>
        /// Second row, second column of the matrix.
        /// </summary>
        public float M22;
        /// <summary>
        /// Third row, first column of the matrix.
        /// </summary>
        public float M31;
        /// <summary>
        /// Third row, second column of the matrix.
        /// </summary>
        public float M32;
        /// <summary>
        /// Third row, third column of the matrix.
        /// </summary>
        public float M33;

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
        
        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="m">Matrix to be inverted.</param>
        /// <param name="inverse">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void SymmetricInvert(ref Triangular3x3 m, out Triangular3x3 inverse)
        {
            var m11 = m.M22 * m.M33 - m.M32 * m.M32;
            var m21 = m.M32 * m.M31 - m.M33 * m.M21;
            var m31 = m.M21 * m.M32 - m.M31 * m.M22;
            var determinantInverse = 1f / (m11 * m.M11 + m21 * m.M21 + m31 * m.M31);

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

        /// <summary>
        /// Multiplies every component in the matrix by the given scale.
        /// </summary>
        /// <param name="m">Matrix to be scaled.</param>
        /// <param name="scale">Scale to apply to every component of the original matrix.</param>
        /// <param name="scaled">Scaled result.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Triangular3x3 m, float scale, out Triangular3x3 scaled)
        {
            scaled.M11 = m.M11 * scale;
            scaled.M21 = m.M21 * scale;
            scaled.M22 = m.M22 * scale;
            scaled.M31 = m.M31 * scale;
            scaled.M32 = m.M32 * scale;
            scaled.M33 = m.M33 * scale;
        }
    }
}
