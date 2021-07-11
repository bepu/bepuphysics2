using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    public struct Matrix3x3Wide
    {
        /// <summary>
        /// First row of the matrix.
        /// </summary>
        public Vector3Wide X;
        /// <summary>
        /// Second row of the matrix.
        /// </summary>
        public Vector3Wide Y;
        /// <summary>
        /// Third row of the matrix.
        /// </summary>
        public Vector3Wide Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in Matrix3x3 source, out Matrix3x3Wide broadcasted)
        {
            broadcasted.X.X = new Vector<float>(source.X.X);
            broadcasted.X.Y = new Vector<float>(source.X.Y);
            broadcasted.X.Z = new Vector<float>(source.X.Z);
            broadcasted.Y.X = new Vector<float>(source.Y.X);
            broadcasted.Y.Y = new Vector<float>(source.Y.Y);
            broadcasted.Y.Z = new Vector<float>(source.Y.Z);
            broadcasted.Z.X = new Vector<float>(source.Z.X);
            broadcasted.Z.Y = new Vector<float>(source.Z.Y);
            broadcasted.Z.Z = new Vector<float>(source.Z.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateIdentity(out Matrix3x3Wide identity)
        {
            identity.X.X = Vector<float>.One;
            identity.X.Y = Vector<float>.Zero;
            identity.X.Z = Vector<float>.Zero;
            identity.Y.X = Vector<float>.Zero;
            identity.Y.Y = Vector<float>.One;
            identity.Y.Z = Vector<float>.Zero;
            identity.Z.X = Vector<float>.Zero;
            identity.Z.Y = Vector<float>.Zero;
            identity.Z.Z = Vector<float>.One;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(in Matrix3x3Wide a, in Matrix3x3Wide b, out Matrix3x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.Y.X + a.X.Z * b.Z.X;
            result.X.Y = a.X.X * b.X.Y + a.X.Y * b.Y.Y + a.X.Z * b.Z.Y;
            result.X.Z = a.X.X * b.X.Z + a.X.Y * b.Y.Z + a.X.Z * b.Z.Z;

            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.Y.X + a.Y.Z * b.Z.X;
            result.Y.Y = a.Y.X * b.X.Y + a.Y.Y * b.Y.Y + a.Y.Z * b.Z.Y;
            result.Y.Z = a.Y.X * b.X.Z + a.Y.Y * b.Y.Z + a.Y.Z * b.Z.Z;

            result.Z.X = a.Z.X * b.X.X + a.Z.Y * b.Y.X + a.Z.Z * b.Z.X;
            result.Z.Y = a.Z.X * b.X.Y + a.Z.Y * b.Y.Y + a.Z.Z * b.Z.Y;
            result.Z.Z = a.Z.X * b.X.Z + a.Z.Y * b.Y.Z + a.Z.Z * b.Z.Z;
        }
        /// <summary>
        /// Multiplies a matrix by another matrix, where the first matrix is sampled as if it were transposed: result = transpose(a) * b.
        /// </summary>
        /// <param name="a">Matrix to be sampled as if it were transposed when multiplied with the second matrix.</param>
        /// <param name="b">Second matrix in the pair.</param>
        /// <param name="result">Result of the multiplication transpose(a) * b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyTransposedWithoutOverlap(in Matrix3x3Wide a, in Matrix3x3Wide b, out Matrix3x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.Y.X * b.Y.X + a.Z.X * b.Z.X;
            result.X.Y = a.X.X * b.X.Y + a.Y.X * b.Y.Y + a.Z.X * b.Z.Y;
            result.X.Z = a.X.X * b.X.Z + a.Y.X * b.Y.Z + a.Z.X * b.Z.Z;

            result.Y.X = a.X.Y * b.X.X + a.Y.Y * b.Y.X + a.Z.Y * b.Z.X;
            result.Y.Y = a.X.Y * b.X.Y + a.Y.Y * b.Y.Y + a.Z.Y * b.Z.Y;
            result.Y.Z = a.X.Y * b.X.Z + a.Y.Y * b.Y.Z + a.Z.Y * b.Z.Z;

            result.Z.X = a.X.Z * b.X.X + a.Y.Z * b.Y.X + a.Z.Z * b.Z.X;
            result.Z.Y = a.X.Z * b.X.Y + a.Y.Z * b.Y.Y + a.Z.Z * b.Z.Y;
            result.Z.Z = a.X.Z * b.X.Z + a.Y.Z * b.Y.Z + a.Z.Z * b.Z.Z;
        }

        /// <summary>
        /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
        /// </summary>
        /// <param name="a">First matrix in the pair.</param>
        /// <param name="b">Matrix to be sampled as if it were transposed when multiplied with the first matrix.</param>
        /// <param name="result">Result of the multiplication a * transpose(b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyByTransposeWithoutOverlap(in Matrix3x3Wide a, in Matrix3x3Wide b, out Matrix3x3Wide result)
        {
            result.X.X = a.X.X * b.X.X + a.X.Y * b.X.Y + a.X.Z * b.X.Z;
            result.X.Y = a.X.X * b.Y.X + a.X.Y * b.Y.Y + a.X.Z * b.Y.Z;
            result.X.Z = a.X.X * b.Z.X + a.X.Y * b.Z.Y + a.X.Z * b.Z.Z;

            result.Y.X = a.Y.X * b.X.X + a.Y.Y * b.X.Y + a.Y.Z * b.X.Z;
            result.Y.Y = a.Y.X * b.Y.X + a.Y.Y * b.Y.Y + a.Y.Z * b.Y.Z;
            result.Y.Z = a.Y.X * b.Z.X + a.Y.Y * b.Z.Y + a.Y.Z * b.Z.Z;

            result.Z.X = a.Z.X * b.X.X + a.Z.Y * b.X.Y + a.Z.Z * b.X.Z;
            result.Z.Y = a.Z.X * b.Y.X + a.Z.Y * b.Y.Y + a.Z.Z * b.Y.Z;
            result.Z.Z = a.Z.X * b.Z.X + a.Z.Y * b.Z.Y + a.Z.Z * b.Z.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector3Wide v, in Matrix3x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.X.X + v.Y * m.Y.X + v.Z * m.Z.X;
            result.Y = v.X * m.X.Y + v.Y * m.Y.Y + v.Z * m.Z.Y;
            result.Z = v.X * m.X.Z + v.Y * m.Y.Z + v.Z * m.Z.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator *(Vector3Wide v, Matrix3x3Wide m)
        {
            Vector3Wide result;
            result.X = v.X * m.X.X + v.Y * m.Y.X + v.Z * m.Z.X;
            result.Y = v.X * m.X.Y + v.Y * m.Y.Y + v.Z * m.Z.Y;
            result.Z = v.X * m.X.Z + v.Y * m.Y.Z + v.Z * m.Z.Z;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformByTransposedWithoutOverlap(in Vector3Wide v, in Matrix3x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.X.X + v.Y * m.X.Y + v.Z * m.X.Z;
            result.Y = v.X * m.Y.X + v.Y * m.Y.Y + v.Z * m.Y.Z;
            result.Z = v.X * m.Z.X + v.Y * m.Z.Y + v.Z * m.Z.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in Vector3Wide v, in Matrix3x3Wide m, out Vector3Wide result)
        {
            TransformWithoutOverlap(v, m, out var temp);
            result = temp;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in Matrix3x3Wide m, out Matrix3x3Wide inverse)
        {
            var m11 = m.Y.Y * m.Z.Z - m.Z.Y * m.Y.Z;
            var m21 = m.Y.Z * m.Z.X - m.Z.Z * m.Y.X;
            var m31 = m.Y.X * m.Z.Y - m.Z.X * m.Y.Y;
            var determinantInverse = Vector<float>.One / (m11 * m.X.X + m21 * m.X.Y + m31 * m.X.Z);

            var m12 = m.Z.Y * m.X.Z - m.X.Y * m.Z.Z;
            var m22 = m.Z.Z * m.X.X - m.X.Z * m.Z.X;
            var m32 = m.Z.X * m.X.Y - m.X.X * m.Z.Y;

            var m13 = m.X.Y * m.Y.Z - m.Y.Y * m.X.Z;
            var m23 = m.X.Z * m.Y.X - m.Y.Z * m.X.X;
            var m33 = m.X.X * m.Y.Y - m.Y.X * m.X.Y;

            inverse.X.X = m11 * determinantInverse;
            inverse.Y.X = m21 * determinantInverse;
            inverse.Z.X = m31 * determinantInverse;
            inverse.X.Y = m12 * determinantInverse;
            inverse.Y.Y = m22 * determinantInverse;
            inverse.Z.Y = m32 * determinantInverse;
            inverse.X.Z = m13 * determinantInverse;
            inverse.Y.Z = m23 * determinantInverse;
            inverse.Z.Z = m33 * determinantInverse;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateCrossProduct(in Vector3Wide v, out Matrix3x3Wide skew)
        {
            skew.X.X = Vector<float>.Zero;
            skew.X.Y = -v.Z;
            skew.X.Z = v.Y;
            skew.Y.X = v.Z;
            skew.Y.Y = Vector<float>.Zero;
            skew.Y.Z = -v.X;
            skew.Z.X = -v.Y;
            skew.Z.Y = v.X;
            skew.Z.Z = Vector<float>.Zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3x3Wide CreateCrossProduct(in Vector3Wide v) //TODO: this has some weird codegen on .NET 6 preview 5.
        {
            Matrix3x3Wide skew;
            skew.X.X = Vector<float>.Zero;
            skew.X.Y = -v.Z;
            skew.X.Z = v.Y;
            skew.Y.X = v.Z;
            skew.Y.Y = Vector<float>.Zero;
            skew.Y.Z = -v.X;
            skew.Z.X = -v.Y;
            skew.Z.Y = v.X;
            skew.Z.Z = Vector<float>.Zero;
            return skew;
        }

        /// <summary>
        /// Negates the components of a matrix.
        /// </summary>
        /// <param name="m">Matrix to negate.</param>
        /// <param name="result">Negated result matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in Matrix3x3Wide m, out Matrix3x3Wide result)
        {
            result.X.X = -m.X.X;
            result.X.Y = -m.X.Y;
            result.X.Z = -m.X.Z;
            result.Y.X = -m.Y.X;
            result.Y.Y = -m.Y.Y;
            result.Y.Z = -m.Y.Z;
            result.Z.X = -m.Z.X;
            result.Z.Y = -m.Z.Y;
            result.Z.Z = -m.Z.Z;
        }

        /// <summary>
        /// Multiplies every component in the matrix by the given scalar value.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scaling value to apply to the matrix's components.</param>
        /// <param name="result">Resulting matrix with scaled components.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Matrix3x3Wide m, in Vector<float> scale, out Matrix3x3Wide result)
        {
            result.X.X = m.X.X * scale;
            result.X.Y = m.X.Y * scale;
            result.X.Z = m.X.Z * scale;
            result.Y.X = m.Y.X * scale;
            result.Y.Y = m.Y.Y * scale;
            result.Y.Z = m.Y.Z * scale;
            result.Z.X = m.Z.X * scale;
            result.Z.Y = m.Z.Y * scale;
            result.Z.Z = m.Z.Z * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFromQuaternion(in QuaternionWide quaternion, out Matrix3x3Wide result)
        {
            var qX2 = quaternion.X + quaternion.X;
            var qY2 = quaternion.Y + quaternion.Y;
            var qZ2 = quaternion.Z + quaternion.Z;

            var YY = qY2 * quaternion.Y;
            var ZZ = qZ2 * quaternion.Z;
            result.X.X = Vector<float>.One - YY - ZZ;
            var XY = qX2 * quaternion.Y;
            var ZW = qZ2 * quaternion.W;
            result.X.Y = XY + ZW;
            var XZ = qX2 * quaternion.Z;
            var YW = qY2 * quaternion.W;
            result.X.Z = XZ - YW;

            var XX = qX2 * quaternion.X;
            result.Y.X = XY - ZW;
            result.Y.Y = Vector<float>.One - XX - ZZ;
            var XW = qX2 * quaternion.W;
            var YZ = qY2 * quaternion.Z;
            result.Y.Z = YZ + XW;

            result.Z.X = XZ + YW;
            result.Z.Y = YZ - XW;
            result.Z.Z = Vector<float>.One - XX - YY;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Matrix3x3Wide a, in Matrix3x3Wide b, out Matrix3x3Wide result)
        {
            result.X.X = a.X.X - b.X.X;
            result.X.Y = a.X.Y - b.X.Y;
            result.X.Z = a.X.Z - b.X.Z;
            result.Y.X = a.Y.X - b.Y.X;
            result.Y.Y = a.Y.Y - b.Y.Y;
            result.Y.Z = a.Y.Z - b.Y.Z;
            result.Z.X = a.Z.X - b.Z.X;
            result.Z.Y = a.Z.Y - b.Z.Y;
            result.Z.Z = a.Z.Z - b.Z.Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3x3Wide operator -(in Matrix3x3Wide a, in Matrix3x3Wide b)
        {
            Matrix3x3Wide result;
            result.X.X = a.X.X - b.X.X;
            result.X.Y = a.X.Y - b.X.Y;
            result.X.Z = a.X.Z - b.X.Z;
            result.Y.X = a.Y.X - b.Y.X;
            result.Y.Y = a.Y.Y - b.Y.Y;
            result.Y.Z = a.Y.Z - b.Y.Z;
            result.Z.X = a.Z.X - b.Z.X;
            result.Z.Y = a.Z.Y - b.Z.Y;
            result.Z.Z = a.Z.Z - b.Z.Z;
            return result;
        }

        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="source">Source of the lane.</param>
        /// <param name="target">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in Matrix3x3Wide source, out Matrix3x3 target)
        {
            Vector3Wide.ReadFirst(source.X, out target.X);
            Vector3Wide.ReadFirst(source.Y, out target.Y);
            Vector3Wide.ReadFirst(source.Z, out target.Z);
        }

        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="wide">Source of the lane.</param>
        /// <param name="slotIndex">Index of the lane within the wide representation to read.</param>
        /// <param name="narrow">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadSlot(ref Matrix3x3Wide wide, int slotIndex, out Matrix3x3 narrow)
        {
            ref var offset = ref GatherScatter.GetOffsetInstance(ref wide, slotIndex);
            ReadFirst(offset, out narrow);
        }
    }
}
