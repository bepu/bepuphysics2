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
        public static void InvertWithoutOverlap(ref Triangular2x2Wide m, out Triangular2x2Wide inverse)
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
    }
}