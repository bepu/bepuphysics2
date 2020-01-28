using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// 3 row, 3 column matrix.
    /// </summary>
    public struct Matrix3x3
    {
        public Vector3 X;
        public Vector3 Y;
        public Vector3 Z;


        /// <summary>
        /// Gets the 3x3 identity matrix.
        /// </summary>
        public static Matrix3x3 Identity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Matrix3x3 toReturn;
                toReturn.X = new Vector3(1, 0, 0);
                toReturn.Y = new Vector3(0, 1, 0);
                toReturn.Z = new Vector3(0, 0, 1);
                return toReturn;
            }
        }

        /// <summary>
        /// Adds the components of two matrices together.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two input matrices.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Matrix3x3 a, in Matrix3x3 b, out Matrix3x3 result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }

        /// <summary>
        /// Scales the components of a matrix by a scalar.
        /// </summary>
        /// <param name="matrix">Matrix to scale.</param>
        /// <param name="scale">Scale to apply to the matrix's components.</param>
        /// <param name="result">Scaled matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(Matrix3x3 matrix, float scale, out Matrix3x3 result)
        {
            result.X = matrix.X * scale;
            result.Y = matrix.Y * scale;
            result.Z = matrix.Z * scale;
        }

        /// <summary>
        /// Subtracts the components of one matrix from another.
        /// </summary>
        /// <param name="a">Matrix to be subtracted from.</param>
        /// <param name="b">Matrix to subtract from a.</param>
        /// <param name="result">Difference of the two input matrices.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Matrix3x3 a, in Matrix3x3 b, out Matrix3x3 result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void Transpose(M* m, M* transposed)
        {
            //A weird function! Why?
            //1) Missing some helpful instructions for actual SIMD accelerated transposition.
            //2) Difficult to get SIMD types to generate competitive codegen due to lots of componentwise access.

            float m12 = m->M12;
            float m13 = m->M13;
            float m23 = m->M23;
            transposed->M11 = m->M11;
            transposed->M12 = m->M21;
            transposed->M13 = m->M31;

            transposed->M21 = m12;
            transposed->M22 = m->M22;
            transposed->M23 = m->M32;

            transposed->M31 = m13;
            transposed->M32 = m23;
            transposed->M33 = m->M33;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Transpose(Matrix3x3* m, Matrix3x3* transposed)
        {
            Transpose((M*)m, (M*)transposed);
        }

        /// <summary>                                                                                                
        /// Computes the transposed matrix of a matrix.                                                              
        /// </summary>                                                                                               
        /// <param name="m">Matrix to transpose.</param>                                                             
        /// <param name="transposed">Transposed matrix.</param>                                                      
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transpose(in Matrix3x3 m, out Matrix3x3 transposed)
        {
            var xy = m.X.Y;
            var xz = m.X.Z;
            var yz = m.Y.Z;
            transposed.X = new Vector3(m.X.X, m.Y.X, m.Z.X);
            transposed.Y = new Vector3(xy, m.Y.Y, m.Z.Y);
            transposed.Z = new Vector3(xz, yz, m.Z.Z);
        }

        /// <summary>
        /// Calculates the determinant of the matrix.
        /// </summary>
        /// <returns>The matrix's determinant.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float Determinant()
        {
            return Vector3.Dot(X, Vector3.Cross(Y, Z));
        }

        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="m">Matrix to be inverted.</param>
        /// <param name="inverse">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in Matrix3x3 m, out Matrix3x3 inverse)
        {
            //Current implementation of cross far from optimal without shuffles, and even then this has some room for improvement.
            //Inverts should be really rare, so it's not too concerning. Use the scalar version when possible until ryujit improves (and we improve this implementation).
            var yz = Vector3.Cross(m.Y, m.Z);
            var zx = Vector3.Cross(m.Z, m.X);
            var xy = Vector3.Cross(m.X, m.Y);
            var inverseDeterminant = 1f / Vector3.Dot(m.X, yz);
            inverse.X = yz * inverseDeterminant;
            inverse.Y = zx * inverseDeterminant;
            inverse.Z = xy * inverseDeterminant;
            Transpose(inverse, out inverse);
        }

        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="m">Matrix to be inverted.</param>
        /// <param name="inverse">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Invert(Matrix3x3* m, Matrix3x3* inverse)
        {
            var mScalar = (M*)m;
            var inverseScalar = (M*)inverse;

            var m11 = mScalar->M22 * mScalar->M33 - mScalar->M32 * mScalar->M23;
            var m21 = mScalar->M23 * mScalar->M31 - mScalar->M33 * mScalar->M21;
            var m31 = mScalar->M21 * mScalar->M32 - mScalar->M31 * mScalar->M22;
            var determinantInverse = 1f / (m11 * mScalar->M11 + m21 * mScalar->M12 + m31 * mScalar->M13);

            var m12 = mScalar->M32 * mScalar->M13 - mScalar->M12 * mScalar->M33;
            var m22 = mScalar->M33 * mScalar->M11 - mScalar->M13 * mScalar->M31;
            var m32 = mScalar->M31 * mScalar->M12 - mScalar->M11 * mScalar->M32;

            var m13 = mScalar->M12 * mScalar->M23 - mScalar->M22 * mScalar->M13;
            var m23 = mScalar->M13 * mScalar->M21 - mScalar->M23 * mScalar->M11;
            var m33 = mScalar->M11 * mScalar->M22 - mScalar->M21 * mScalar->M12;

            inverseScalar->M11 = m11 * determinantInverse;
            inverseScalar->M21 = m21 * determinantInverse;
            inverseScalar->M31 = m31 * determinantInverse;

            inverseScalar->M12 = m12 * determinantInverse;
            inverseScalar->M22 = m22 * determinantInverse;
            inverseScalar->M32 = m32 * determinantInverse;

            inverseScalar->M13 = m13 * determinantInverse;
            inverseScalar->M23 = m23 * determinantInverse;
            inverseScalar->M33 = m33 * determinantInverse;
        }






        /// <summary>
        /// Transforms the vector by the matrix.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="m">Matrix to use as the transformation.</param>
        /// <param name="result">Product of the transformation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in Vector3 v, in Matrix3x3 m, out Vector3 result)
        {
            var x = new Vector3(v.X);
            var y = new Vector3(v.Y);
            var z = new Vector3(v.Z);
            result = m.X * x + m.Y * y + m.Z * z;
        }

        /// <summary>
        /// Transforms the vector by the matrix's transpose.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="m">Matrix to use as the transformation transpose.</param>
        /// <param name="result">Product of the transformation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformTranspose(in Vector3 v, in Matrix3x3 m, out Vector3 result)
        {
            result = new Vector3(
                Vector3.Dot(v, m.X),
                Vector3.Dot(v, m.Y),
                Vector3.Dot(v, m.Z));
        }

        struct M
        {
            public float M11, M12, M13;
            public float M21, M22, M23;
            public float M31, M32, M33;
        }


        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(in Matrix3x3 a, in Matrix3x3 b, out Matrix3x3 result)
        {
            var bX = b.X;
            var bY = b.Y;
            {
                var x = new Vector3(a.X.X);
                var y = new Vector3(a.X.Y);
                var z = new Vector3(a.X.Z);
                result.X = x * bX + y * bY + z * b.Z;
            }

            {
                var x = new Vector3(a.Y.X);
                var y = new Vector3(a.Y.Y);
                var z = new Vector3(a.Y.Z);
                result.Y = x * bX + y * bY + z * b.Z;
            }

            {
                var x = new Vector3(a.Z.X);
                var y = new Vector3(a.Z.Y);
                var z = new Vector3(a.Z.Z);
                result.Z = x * bX + y * bY + z * b.Z;
            }
        }

        /// <summary>
        /// Multiplies the two matrices, where a is treated as transposed: result = transpose(a) * b
        /// </summary>
        /// <param name="a">First matrix to multiply that will be transposed.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyTransposed(in Matrix3x3 a, in Matrix3x3 b, out Matrix3x3 result)
        {
            var bX = b.X;
            var bY = b.Y;
            {
                var x = new Vector3(a.X.X);
                var y = new Vector3(a.Y.X);
                var z = new Vector3(a.Z.X);
                result.X = x * bX + y * bY + z * b.Z;
            }

            {
                var x = new Vector3(a.X.Y);
                var y = new Vector3(a.Y.Y);
                var z = new Vector3(a.Z.Y);
                result.Y = x * bX + y * bY + z * b.Z;
            }

            {
                var x = new Vector3(a.X.Z);
                var y = new Vector3(a.Y.Z);
                var z = new Vector3(a.Z.Z);
                result.Z = x * bX + y * bY + z * b.Z;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFromMatrix(in Matrix matrix, out Matrix3x3 matrix3x3)
        {
            matrix3x3.X = new Vector3(matrix.X.X, matrix.X.Y, matrix.X.Z);
            matrix3x3.Y = new Vector3(matrix.Y.X, matrix.Y.Y, matrix.Y.Z);
            matrix3x3.Z = new Vector3(matrix.Z.X, matrix.Z.Y, matrix.Z.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFromQuaternion(in Quaternion quaternion, out Matrix3x3 result)
        {
            float qX2 = quaternion.X + quaternion.X;
            float qY2 = quaternion.Y + quaternion.Y;
            float qZ2 = quaternion.Z + quaternion.Z;
            float XX = qX2 * quaternion.X;
            float YY = qY2 * quaternion.Y;
            float ZZ = qZ2 * quaternion.Z;
            float XY = qX2 * quaternion.Y;
            float XZ = qX2 * quaternion.Z;
            float XW = qX2 * quaternion.W;
            float YZ = qY2 * quaternion.Z;
            float YW = qY2 * quaternion.W;
            float ZW = qZ2 * quaternion.W;

            result.X = new Vector3(
                1 - YY - ZZ,
                XY + ZW,
                XZ - YW);

            result.Y = new Vector3(
                XY - ZW,
                1 - XX - ZZ,
                YZ + XW);

            result.Z = new Vector3(
                XZ + YW,
                YZ - XW,
                1 - XX - YY);


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3x3 CreateFromQuaternion(in Quaternion quaternion)
        {
            CreateFromQuaternion(quaternion, out var toReturn);
            return toReturn;
        }


        /// <summary>
        /// Creates a 3x3 matrix representing the given scale along its local axes.
        /// </summary>
        /// <param name="scale">Scale to represent.</param>
        /// <param name="linearTransform">Matrix representing a scale.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateScale(in Vector3 scale, out Matrix3x3 linearTransform)
        {
            linearTransform.X = new Vector3(scale.X, 0, 0);
            linearTransform.Y = new Vector3(0, scale.Y, 0);
            linearTransform.Z = new Vector3(0, 0, scale.Z);
        }

        /// <summary>
        /// Creates a matrix representing a rotation derived from an axis and angle.
        /// </summary>
        /// <param name="axis">Axis of the rotation.</param>
        /// <param name="angle">Angle of the rotation.</param>
        /// <param name="result">Resulting rotation matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFromAxisAngle(in Vector3 axis, float angle, out Matrix3x3 result)
        {
            //TODO: Could be better simdified.
            float xx = axis.X * axis.X;
            float yy = axis.Y * axis.Y;
            float zz = axis.Z * axis.Z;
            float xy = axis.X * axis.Y;
            float xz = axis.X * axis.Z;
            float yz = axis.Y * axis.Z;

            float sinAngle = (float)Math.Sin(angle);
            float oneMinusCosAngle = 1 - (float)Math.Cos(angle);

            result.X = new Vector3(
                1 + oneMinusCosAngle * (xx - 1),
                axis.Z * sinAngle + oneMinusCosAngle * xy,
                -axis.Y * sinAngle + oneMinusCosAngle * xz);

            result.Y = new Vector3(
                -axis.Z * sinAngle + oneMinusCosAngle * xy,
                1 + oneMinusCosAngle * (yy - 1),
                axis.X * sinAngle + oneMinusCosAngle * yz);

            result.Z = new Vector3(
                axis.Y * sinAngle + oneMinusCosAngle * xz,
                -axis.X * sinAngle + oneMinusCosAngle * yz,
                1 + oneMinusCosAngle * (zz - 1));

        }

        /// <summary>
        /// Creates a matrix representing a rotation derived from an axis and angle.
        /// </summary>
        /// <param name="axis">Axis of the rotation.</param>
        /// <param name="angle">Angle of the rotation.</param>
        /// <returns>Resulting rotation matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3x3 CreateFromAxisAngle(in Vector3 axis, float angle)
        {
            CreateFromAxisAngle(axis, angle, out var result);
            return result;

        }

        /// <summary>
        /// Creates a matrix such that a x v = a * result.
        /// </summary>
        /// <param name="v">Vector to build the skew symmetric matrix from.</param>
        /// <param name="result">Skew symmetric matrix representing the cross product.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateCrossProduct(in Vector3 v, out Matrix3x3 result)
        {
            result.X.X = 0f;
            result.X.Y = -v.Z;
            result.X.Z = v.Y;
            result.Y.X = v.Z;
            result.Y.Y = 0f;
            result.Y.Z = -v.X;
            result.Z.X = -v.Y;
            result.Z.Y = v.X;
            result.Z.Z = 0f;
        }

        /// <summary>
        /// Concatenates two matrices.
        /// </summary>
        /// <param name="m1">First input matrix.</param>
        /// <param name="m2">Second input matrix.</param>
        /// <returns>Concatenated transformation of the form m1 * m2.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3x3 operator *(in Matrix3x3 m1, in Matrix3x3 m2)
        {
            Multiply(m1, m2, out var toReturn);
            return toReturn;
        }
    }
}