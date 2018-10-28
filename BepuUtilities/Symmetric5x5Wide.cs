using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuUtilities
{
    public struct Symmetric5x5Wide
    {
        /// <summary>
        /// Upper left 3x3 block of the matrix.
        /// </summary>
        public Symmetric3x3Wide A;
        /// <summary>
        /// Lower left 2x3 block of the matrix.
        /// </summary>
        public Matrix2x3Wide B;
        /// <summary>
        /// Lower right 2x2 block of the matrix.
        /// </summary>
        public Symmetric2x2Wide D;

        /// <summary>
        /// Scales each component of m by the given scale.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scale to apply to the components of m.</param>
        /// <param name="result">Result of scaling each component of m by scale.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Symmetric5x5Wide m, in Vector<float> scale, out Symmetric5x5Wide result)
        {
            Symmetric3x3Wide.Scale(m.A, scale, out result.A);
            Matrix2x3Wide.Scale(m.B, scale, out result.B);
            Symmetric2x2Wide.Scale(m.D, scale, out result.D);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in Symmetric3x3Wide a, in Matrix2x3Wide b, in Symmetric2x2Wide d, out Symmetric5x5Wide result)
        {
            // [ A  BT ]^-1 = [ (A - BT * D^-1 * B)^-1, -(A - BT * D^-1 * B)^-1 * BT * D^-1                   ]
            // [ B  D  ]      [ symmetric               D^-1 + D^-1 * B * (A - BT * D^-1 * B)^-1 * BT * D^-1 ]
            Symmetric2x2Wide.InvertWithoutOverlap(d, out var invD);
            Symmetric2x2Wide.MultiplyTransposed(b, invD, out var bTInvD);
            Symmetric3x3Wide.CompleteMatrixSandwich(bTInvD, b, out var bTInvDB);
            Symmetric3x3Wide.Subtract(a, bTInvDB, out var resultAInverse);
            Symmetric3x3Wide.Invert(resultAInverse, out result.A);

            Symmetric3x3Wide.MultiplyByTransposed(result.A, bTInvD, out var negatedResultBT);
            Matrix2x3Wide.Negate(negatedResultBT, out result.B);
            Symmetric2x2Wide.CompleteMatrixSandwich(bTInvD, negatedResultBT, out result.D);
            Symmetric2x2Wide.Add(result.D, invD, out result.D);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InvertWithoutOverlap(in Symmetric5x5Wide m, out Symmetric5x5Wide result)
        {
            Invert(m.A, m.B, m.D, out result);
        }

        /// <summary>
        /// Computes result = v * m, where v and result are 1x5 vectors which are split into two subvectors.
        /// </summary>
        /// <param name="v0">First half of the a vector.</param>
        /// <param name="v1">Second half of the a vector.</param>
        /// <param name="m">Matrix to transform with.</param>
        /// <param name="result0">First half of the result.</param>
        /// <param name="result1">Second half of the result.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector3Wide v0, in Vector2Wide v1, in Symmetric5x5Wide m, out Vector3Wide result0, out Vector2Wide result1)
        {
            // [ v0x v0y v0z v1x v1y ] * [ m.A.XX m.A.YX m.A.ZX b.X.X b.Y.X ]
            //                           [ m.A.YX m.A.YY m.A.ZY b.X.Y b.Y.Y ]
            //                           [ m.A.ZX m.A.ZY m.A.ZZ b.X.Z b.Y.Z ]
            //                           [ b.X.X  b.X.Y  b.X.Z  d.XX  d.YX  ]
            //                           [ b.Y.X  b.Y.Y  b.Y.Z  d.YX  d.YY  ]
            result0.X = v0.X * m.A.XX + v0.Y * m.A.YX + v0.Z * m.A.ZX + v1.X * m.B.X.X + v1.Y * m.B.Y.X;
            result0.Y = v0.X * m.A.YX + v0.Y * m.A.YY + v0.Z * m.A.ZY + v1.X * m.B.X.Y + v1.Y * m.B.Y.Y;
            result0.Z = v0.X * m.A.ZX + v0.Y * m.A.ZY + v0.Z * m.A.ZZ + v1.X * m.B.X.Z + v1.Y * m.B.Y.Z;

            result1.X = v0.X * m.B.X.X + v0.Y * m.B.X.Y + v0.Z * m.B.X.Z + v1.X * m.D.XX + v1.Y * m.D.YX;
            result1.Y = v0.X * m.B.Y.X + v0.Y * m.B.Y.Y + v0.Z * m.B.Y.Z + v1.X * m.D.YX + v1.Y * m.D.YY;
        }
    }
}
