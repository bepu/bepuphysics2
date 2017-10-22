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
    }
}
