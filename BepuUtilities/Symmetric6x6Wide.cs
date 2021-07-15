using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
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

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        /// <summary>
        /// Solves [vLower, vUpper] = [resultLower, resultUpper] * [[a, b], [bT, d]] for [resultLower, resultUpper] using LDLT decomposition.
        /// [[a, b], [bT, d]] should be positive semidefinite.
        /// </summary>
        /// <param name="v0">First 3 values of the 6 component input vector.</param>
        /// <param name="v1">Second 3 values of the 6 component input vector.</param>
        /// <param name="a">Upper left 3x3 region of the matrix.</param>
        /// <param name="b">Upper right 3x3 region of the matrix. Also the lower left 3x3 region of the matrix, transposed.</param>
        /// <param name="d">Lower right 3x3 region of the matrix.</param>
        /// <param name="result0">First 3 values of the result vector.</param>
        /// <param name="result1">Second 3 values of the result vector.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LDLTSolve(
            in Vector3Wide v0, in Vector3Wide v1, in Symmetric3x3Wide a, in Matrix3x3Wide b, in Symmetric3x3Wide d, out Vector3Wide result0, out Vector3Wide result1)
        {
            var d1 = a.XX;
            var inverseD1 = Vector<float>.One / d1;
            var l21 = inverseD1 * a.YX;
            var l31 = inverseD1 * a.ZX;
            var l41 = inverseD1 * b.X.X;
            var l51 = inverseD1 * b.X.Y;
            var l61 = inverseD1 * b.X.Z;
            var d2 = a.YY - l21 * l21 * d1;
            var inverseD2 = Vector<float>.One / d2;
            var l32 = inverseD2 * (a.ZY - l31 * l21 * d1);
            var l42 = inverseD2 * (b.Y.X - l41 * l21 * d1);
            var l52 = inverseD2 * (b.Y.Y - l51 * l21 * d1);
            var l62 = inverseD2 * (b.Y.Z - l61 * l21 * d1);
            var d3 = a.ZZ - l31 * l31 * d1 - l32 * l32 * d2;
            var inverseD3 = Vector<float>.One / d3;
            var l43 = inverseD3 * (b.Z.X - l41 * l31 * d1 - l42 * l32 * d2);
            var l53 = inverseD3 * (b.Z.Y - l51 * l31 * d1 - l52 * l32 * d2);
            var l63 = inverseD3 * (b.Z.Z - l61 * l31 * d1 - l62 * l32 * d2);
            var d4 = d.XX - l41 * l41 * d1 - l42 * l42 * d2 - l43 * l43 * d3;
            var inverseD4 = Vector<float>.One / d4;
            var l54 = inverseD4 * (d.YX - l51 * l41 * d1 - l52 * l42 * d2 - l53 * l43 * d3);
            var l64 = inverseD4 * (d.ZX - l61 * l41 * d1 - l62 * l42 * d2 - l63 * l43 * d3);
            var d5 = d.YY - l51 * l51 * d1 - l52 * l52 * d2 - l53 * l53 * d3 - l54 * l54 * d4;
            var inverseD5 = Vector<float>.One / d5;
            var l65 = inverseD5 * (d.ZY - l61 * l51 * d1 - l62 * l52 * d2 - l63 * l53 * d3 - l64 * l54 * d4);
            var d6 = d.ZZ - l61 * l61 * d1 - l62 * l62 * d2 - l63 * l63 * d3 - l64 * l64 * d4 - l65 * l65 * d5;
            var inverseD6 = Vector<float>.One / d6;

            //We now have the components of L and D, so substitute.
            result0.X = v0.X;
            result0.Y = v0.Y - l21 * result0.X;
            result0.Z = v0.Z - l31 * result0.X - l32 * result0.Y;
            result1.X = v1.X - l41 * result0.X - l42 * result0.Y - l43 * result0.Z;
            result1.Y = v1.Y - l51 * result0.X - l52 * result0.Y - l53 * result0.Z - l54 * result1.X;
            result1.Z = v1.Z - l61 * result0.X - l62 * result0.Y - l63 * result0.Z - l64 * result1.X - l65 * result1.Y;

            result1.Z = result1.Z * inverseD6;
            result1.Y = result1.Y * inverseD5 - l65 * result1.Z;
            result1.X = result1.X * inverseD4 - l64 * result1.Z - l54 * result1.Y;
            result0.Z = result0.Z * inverseD3 - l63 * result1.Z - l53 * result1.Y - l43 * result1.X;
            result0.Y = result0.Y * inverseD2 - l62 * result1.Z - l52 * result1.Y - l42 * result1.X - l32 * result0.Z;
            result0.X = result0.X * inverseD1 - l61 * result1.Z - l51 * result1.Y - l41 * result1.X - l31 * result0.Z - l21 * result0.Y;
        }
    }
}
