using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuUtilities
{
    public struct Symmetric6x6Wide
    {
        public Symmetric3x3Wide A;
        public Matrix3x3Wide B;
        public Symmetric3x3Wide D;

        /// <summary>
        /// Scales each component of m by the given scale.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scale to apply to the components of m.</param>
        /// <param name="result">Result of scaling each component of m by scale.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Symmetric6x6Wide m, in Vector<float> scale, out Symmetric6x6Wide result)
        {
            Symmetric3x3Wide.Scale(m.A, scale, out result.A);
            Matrix3x3Wide.Scale(m.B, scale, out result.B);
            Symmetric3x3Wide.Scale(m.D, scale, out result.D);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in Symmetric3x3Wide a, in Matrix3x3Wide b, in Symmetric3x3Wide d, out Symmetric6x6Wide result)
        {
            // [ A  B ]^-1 = [ (A - B * D^-1 * BT)^-1, -(A - B * D^-1 * BT)^-1 * B * D^-1                   ]
            // [ BT D ]      [ symmetric               D^-1 + D^-1 * BT * (A - B * D^-1 * BT)^-1 * B * D^-1 ]
            //Note that B * D^-1 * BT produces a symmetric result. Likewise for the more complex result:
            //N = D^-1 * BT => 
            //D^-1 * BT * M * B * D^-1 = N * M * NT, because D is symmetric.
            Symmetric3x3Wide.Invert(d, out var invD);
            Symmetric3x3Wide.MultiplyWithoutOverlap(b, invD, out var bInvD);
            Symmetric3x3Wide.CompleteMatrixSandwichByTranspose(bInvD, b, out var bInvDBT);
            Symmetric3x3Wide.Subtract(a, bInvDBT, out var resultAInverse);
            Symmetric3x3Wide.Invert(resultAInverse, out result.A);

            Symmetric3x3Wide.Multiply(result.A, bInvD, out var negatedResultB);
            Matrix3x3Wide.Negate(negatedResultB, out result.B);
            Symmetric3x3Wide.CompleteMatrixSandwichTranspose(bInvD, negatedResultB, out result.D);
            Symmetric3x3Wide.Add(result.D, invD, out result.D);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in Symmetric6x6Wide m, out Symmetric6x6Wide result)
        {
            Invert(m.A, m.B, m.D, out result);
        }

        /// <summary>
        /// Computes result = v * m, where v and result are 1x6 vectors which are split into two 1x3 values.
        /// </summary>
        /// <param name="v0">First half of the a vector.</param>
        /// <param name="v1">Second half of the a vector.</param>
        /// <param name="m">Matrix to transform with.</param>
        /// <param name="result0">First half of the result.</param>
        /// <param name="result1">Second half of the result.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector3Wide v0, in Vector3Wide v1, in Symmetric6x6Wide m, out Vector3Wide result0, out Vector3Wide result1)
        {
            result0.X = v0.X * m.A.XX + v0.Y * m.A.YX + v0.Z * m.A.ZX + v1.X * m.B.X.X + v1.Y * m.B.X.Y + v1.Z * m.B.X.Z;
            result0.Y = v0.X * m.A.YX + v0.Y * m.A.YY + v0.Z * m.A.ZY + v1.X * m.B.Y.X + v1.Y * m.B.Y.Y + v1.Z * m.B.Y.Z;
            result0.Z = v0.X * m.A.ZX + v0.Y * m.A.ZY + v0.Z * m.A.ZZ + v1.X * m.B.Z.X + v1.Y * m.B.Z.Y + v1.Z * m.B.Z.Z;

            result1.X = v0.X * m.B.X.X + v0.Y * m.B.Y.X + v0.Z * m.B.Z.X + v1.X * m.D.XX + v1.Y * m.D.YX + v1.Z * m.D.ZX;
            result1.Y = v0.X * m.B.X.Y + v0.Y * m.B.Y.Y + v0.Z * m.B.Z.Y + v1.X * m.D.YX + v1.Y * m.D.YY + v1.Z * m.D.ZY;
            result1.Z = v0.X * m.B.X.Z + v0.Y * m.B.Y.Z + v0.Z * m.B.Z.Z + v1.X * m.D.ZX + v1.Y * m.D.ZY + v1.Z * m.D.ZZ;
        }
    }
}
