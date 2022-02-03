using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    public struct QuaternionWide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;
        public Vector<float> W;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in Quaternion source, out QuaternionWide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X);
            broadcasted.Y = new Vector<float>(source.Y);
            broadcasted.Z = new Vector<float>(source.Z);
            broadcasted.W = new Vector<float>(source.W);
        }

        /// <summary>
        /// Takes a slot from the source quaternion and broadcasts it into all slots of the target quaternion.
        /// </summary>
        /// <param name="source">Quaternion to pull values from.</param>
        /// <param name="slotIndex">Slot in the source vectors to pull values from.</param>
        /// <param name="broadcasted">Target quaternion to be filled with the selected data.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Rebroadcast(in QuaternionWide source, int slotIndex, out QuaternionWide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X[slotIndex]);
            broadcasted.Y = new Vector<float>(source.Y[slotIndex]);
            broadcasted.Z = new Vector<float>(source.Z[slotIndex]);
            broadcasted.W = new Vector<float>(source.W[slotIndex]);
        }

        /// <summary>
        /// Constructs a quaternion from a rotation matrix.
        /// </summary>
        /// <param name="r">Rotation matrix to create the quaternion from.</param>
        /// <param name="q">Quaternion based on the rotation matrix.</param>
        public static void CreateFromRotationMatrix(in Matrix3x3Wide r, out QuaternionWide q)
        {
            //Since we can't branch, we're going to end up calculating the possible states of all branches.
            //This requires doing more ALU work than the branching implementation, but there are a lot of common terms across the branches, and (random-ish) branches aren't free.
            //Overall, this turns out to be about 2x-2.5x more expensive per call than the scalar version, but it handles multiple lanes, so it's a net win.
            var oneAddX = Vector<float>.One + r.X.X;
            var oneSubX = Vector<float>.One - r.X.X;
            var yAddZ = r.Y.Y + r.Z.Z;
            var ySubZ = r.Y.Y - r.Z.Z;
            var tX = oneAddX - yAddZ;
            var tY = oneSubX + ySubZ;
            var tZ = oneSubX - ySubZ;
            var tW = oneAddX + yAddZ;

            //There are two layers of conditions- inner, and outer. We have to first select each of the two inner halves- upper, and lower-
            //and then we will select which of the two inners to use for the outer.
            var useUpper = Vector.LessThan(r.Z.Z, Vector<float>.Zero);
            var useUpperUpper = Vector.GreaterThan(r.X.X, r.Y.Y);
            var useLowerUpper = Vector.LessThan(r.X.X, -r.Y.Y);
            var t = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, tX, tY),
                    Vector.ConditionalSelect(useLowerUpper, tZ, tW));
            var xyAddYx = r.X.Y + r.Y.X;
            var yzSubZy = r.Y.Z - r.Z.Y;
            var zxAddXz = r.Z.X + r.X.Z;
            q.X = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, tX, xyAddYx),
                    Vector.ConditionalSelect(useLowerUpper, zxAddXz, yzSubZy));
            var yzAddZy = r.Y.Z + r.Z.Y;
            var zxSubXz = r.Z.X - r.X.Z;
            q.Y = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, xyAddYx, tY),
                    Vector.ConditionalSelect(useLowerUpper, yzAddZy, zxSubXz));
            var xySubYx = r.X.Y - r.Y.X;
            q.Z = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, zxAddXz, yzAddZy),
                    Vector.ConditionalSelect(useLowerUpper, tZ, xySubYx));
            q.W = Vector.ConditionalSelect(useUpper,
                    Vector.ConditionalSelect(useUpperUpper, yzSubZy, zxSubXz),
                    Vector.ConditionalSelect(useLowerUpper, xySubYx, tW));

            var scale = new Vector<float>(0.5f) / Vector.SquareRoot(t);
            Scale(q, scale, out q);
        }

        /// <summary>
        /// Adds the components of two quaternions together.
        /// </summary>
        /// <param name="a">First quaternion to add.</param>
        /// <param name="b">Second quaternion to add.</param>
        /// <param name="result">Sum of the two input quaternions.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in QuaternionWide a, in QuaternionWide b, out QuaternionWide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            result.W = a.W + b.W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in QuaternionWide q, in Vector<float> scale, out QuaternionWide result)
        {
            result.X = q.X * scale;
            result.Y = q.Y * scale;
            result.Z = q.Z * scale;
            result.W = q.W * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector<float> LengthSquared()
        {
            return X * X + Y * Y + Z * Z + W * W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector<float> Length()
        {
            return Vector.SquareRoot(X * X + Y * Y + Z * Z + W * W);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionWide Normalize(QuaternionWide q)
        {
            //TODO: fast path is possible with intrinsics.
            var inverseNorm = Vector<float>.One / Vector.SquareRoot(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
            QuaternionWide normalized;
            normalized.X = q.X * inverseNorm;
            normalized.Y = q.Y * inverseNorm;
            normalized.Z = q.Z * inverseNorm;
            normalized.W = q.W * inverseNorm;
            return normalized;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in QuaternionWide q, out QuaternionWide negated)
        {
            negated.X = -q.X;
            negated.Y = -q.Y;
            negated.Z = -q.Z;
            negated.W = -q.W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionWide operator -(QuaternionWide q)
        {
            QuaternionWide negated;
            negated.X = -q.X;
            negated.Y = -q.Y;
            negated.Z = -q.Z;
            negated.W = -q.W;
            return negated;
        }

        /// <summary>
        /// Computes the quaternion rotation between two normalized vectors.
        /// </summary>
        /// <param name="v1">First unit-length vector.</param>
        /// <param name="v2">Second unit-length vector.</param>
        /// <param name="q">Quaternion representing the rotation from v1 to v2.</param>
        public static void GetQuaternionBetweenNormalizedVectors(in Vector3Wide v1, in Vector3Wide v2, out QuaternionWide q)
        {
            Vector3Wide.Dot(v1, v2, out var dot);
            //For non-normal vectors, the multiplying the axes length squared would be necessary:
            //float w = dot + Sqrt(v1.LengthSquared() * v2.LengthSquared());


            //There exists an ambiguity at dot == -1. If the directions point away from each other, there are an infinite number of shortest paths.
            //One must be chosen arbitrarily. Here, we choose one by projecting onto the plane whose normal is associated with the smallest magnitude.
            //Since this is a SIMD operation, the special case is always executed and its result is conditionally selected.

            Vector3Wide.CrossWithoutOverlap(v1, v2, out var cross);
            var useNormalCase = Vector.GreaterThan(dot, new Vector<float>(-0.999999f));
            var absX = Vector.Abs(v1.X);
            var absY = Vector.Abs(v1.Y);
            var absZ = Vector.Abs(v1.Z);
            var xIsSmallest = Vector.BitwiseAnd(Vector.LessThan(absX, absY), Vector.LessThan(absX, absZ));
            var yIsSmaller = Vector.LessThan(absY, absZ);
            q.X = Vector.ConditionalSelect(useNormalCase, cross.X, Vector.ConditionalSelect(xIsSmallest, Vector<float>.Zero, Vector.ConditionalSelect(yIsSmaller, -v1.Z, -v1.Y)));
            q.Y = Vector.ConditionalSelect(useNormalCase, cross.Y, Vector.ConditionalSelect(xIsSmallest, -v1.Z, Vector.ConditionalSelect(yIsSmaller, Vector<float>.Zero, v1.X)));
            q.Z = Vector.ConditionalSelect(useNormalCase, cross.Z, Vector.ConditionalSelect(xIsSmallest, v1.Y, Vector.ConditionalSelect(yIsSmaller, v1.X, Vector<float>.Zero)));
            q.W = Vector.ConditionalSelect(useNormalCase, dot + Vector<float>.One, Vector<float>.Zero);

            q = Normalize(q);
        }

        /// <summary>
        /// Computes the quaternion rotation between two normalized vectors.
        /// </summary>
        /// <param name="v1">First unit-length vector.</param>
        /// <param name="v2">Second unit-length vector.</param>
        /// <returns>Quaternion representing the rotation from v1 to v2.</returns>
        public static QuaternionWide GetQuaternionBetweenNormalizedVectors(Vector3Wide v1, Vector3Wide v2)
        {
            Vector3Wide.Dot(v1, v2, out var dot);
            //For non-normal vectors, the multiplying the axes length squared would be necessary:
            //float w = dot + Sqrt(v1.LengthSquared() * v2.LengthSquared());


            //There exists an ambiguity at dot == -1. If the directions point away from each other, there are an infinite number of shortest paths.
            //One must be chosen arbitrarily. Here, we choose one by projecting onto the plane whose normal is associated with the smallest magnitude.
            //Since this is a SIMD operation, the special case is always executed and its result is conditionally selected.

            var cross = Vector3Wide.Cross(v1, v2);
            var useNormalCase = Vector.GreaterThan(dot, new Vector<float>(-0.999999f));
            var absX = Vector.Abs(v1.X);
            var absY = Vector.Abs(v1.Y);
            var absZ = Vector.Abs(v1.Z);
            var xIsSmallest = Vector.BitwiseAnd(Vector.LessThan(absX, absY), Vector.LessThan(absX, absZ));
            var yIsSmaller = Vector.LessThan(absY, absZ);
            QuaternionWide q;
            q.X = Vector.ConditionalSelect(useNormalCase, cross.X, Vector.ConditionalSelect(xIsSmallest, Vector<float>.Zero, Vector.ConditionalSelect(yIsSmaller, -v1.Z, -v1.Y)));
            q.Y = Vector.ConditionalSelect(useNormalCase, cross.Y, Vector.ConditionalSelect(xIsSmallest, -v1.Z, Vector.ConditionalSelect(yIsSmaller, Vector<float>.Zero, v1.X)));
            q.Z = Vector.ConditionalSelect(useNormalCase, cross.Z, Vector.ConditionalSelect(xIsSmallest, v1.Y, Vector.ConditionalSelect(yIsSmaller, v1.X, Vector<float>.Zero)));
            q.W = Vector.ConditionalSelect(useNormalCase, dot + Vector<float>.One, Vector<float>.Zero);
            return Normalize(q);
        }

        /// <summary>
        /// Gets an axis and angle representation of the rotation stored in a quaternion.
        /// </summary>
        /// <param name="q">Quaternion to extract an axis-angle representation from.</param>
        /// <param name="axis">Axis of rotation extracted from the quaternion.</param>
        /// <param name="angle">Angle of rotation extracted from the quaternion.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetAxisAngleFromQuaternion(in QuaternionWide q, out Vector3Wide axis, out Vector<float> angle)
        {
            var shouldNegate = Vector.LessThan(q.W, Vector<float>.Zero);
            axis.X = Vector.ConditionalSelect(shouldNegate, -q.X, q.X);
            axis.Y = Vector.ConditionalSelect(shouldNegate, -q.Y, q.Y);
            axis.Z = Vector.ConditionalSelect(shouldNegate, -q.Z, q.Z);
            var qw = Vector.ConditionalSelect(shouldNegate, -q.W, q.W);

            Vector3Wide.Length(axis, out var axisLength);
            Vector3Wide.Scale(axis, Vector<float>.One / axisLength, out axis);
            var useFallback = Vector.LessThan(axisLength, new Vector<float>(1e-14f));
            axis.X = Vector.ConditionalSelect(useFallback, Vector<float>.One, axis.X);
            axis.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, axis.Y);
            axis.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, axis.Z);
            var halfAngle = MathHelper.Acos(qw);
            angle = new Vector<float>(2) * halfAngle;
        }

        /// <summary>
        /// Transforms the vector using a quaternion. Assumes that the memory backing the input and output do not overlap.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(in Vector3Wide v, in QuaternionWide rotation, out Vector3Wide result)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            var x2 = rotation.X + rotation.X;
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xx2 = rotation.X * x2;
            var xy2 = rotation.X * y2;
            var xz2 = rotation.X * z2;
            var yy2 = rotation.Y * y2;
            var yz2 = rotation.Y * z2;
            var zz2 = rotation.Z * z2;
            var wx2 = rotation.W * x2;
            var wy2 = rotation.W * y2;
            var wz2 = rotation.W * z2;
            result.X = v.X * (Vector<float>.One - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
            result.Y = v.X * (xy2 + wz2) + v.Y * (Vector<float>.One - xx2 - zz2) + v.Z * (yz2 - wx2);
            result.Z = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (Vector<float>.One - xx2 - yy2);

        }

        /// <summary>
        /// Transforms the vector using a quaternion.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in Vector3Wide v, in QuaternionWide rotation, out Vector3Wide result)
        {
            TransformWithoutOverlap(v, rotation, out var temp);
            result = temp;
        }

        /// <summary>
        /// Transforms the vector using a quaternion.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Transform(Vector3Wide v, QuaternionWide rotation)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            var x2 = rotation.X + rotation.X;
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xx2 = rotation.X * x2;
            var xy2 = rotation.X * y2;
            var xz2 = rotation.X * z2;
            var yy2 = rotation.Y * y2;
            var yz2 = rotation.Y * z2;
            var zz2 = rotation.Z * z2;
            var wx2 = rotation.W * x2;
            var wy2 = rotation.W * y2;
            var wz2 = rotation.W * z2;
            Vector3Wide result;
            result.X = v.X * (Vector<float>.One - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
            result.Y = v.X * (xy2 + wz2) + v.Y * (Vector<float>.One - xx2 - zz2) + v.Z * (yz2 - wx2);
            result.Z = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (Vector<float>.One - xx2 - yy2);
            return result;
        }

        /// <summary>
        /// Transforms the vector using a quaternion.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide TransformByConjugate(Vector3Wide v, QuaternionWide rotation)
        {
            //This operation is an optimized-down version of v' = q * v * q^-1.
            //The expanded form would be to treat v as an 'axis only' quaternion
            //and perform standard quaternion multiplication.  Assuming q is normalized,
            //q^-1 can be replaced by a conjugation.
            var x2 = rotation.X + rotation.X;
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xx2 = rotation.X * x2;
            var xy2 = rotation.X * y2;
            var xz2 = rotation.X * z2;
            var yy2 = rotation.Y * y2;
            var yz2 = rotation.Y * z2;
            var zz2 = rotation.Z * z2;
            var nW = -rotation.W;
            var wx2 = nW * x2;
            var wy2 = nW * y2;
            var wz2 = nW * z2;
            Vector3Wide result;
            result.X = v.X * (Vector<float>.One - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
            result.Y = v.X * (xy2 + wz2) + v.Y * (Vector<float>.One - xx2 - zz2) + v.Z * (yz2 - wx2);
            result.Z = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (Vector<float>.One - xx2 - yy2);
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator *(Vector3Wide v, QuaternionWide rotation)
        {
            return Transform(v, rotation);
        }

        /// <summary>
        /// Transforms the unit X direction using a quaternion.
        /// </summary>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide TransformUnitX(QuaternionWide rotation)
        {
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xy2 = rotation.X * y2;
            var xz2 = rotation.X * z2;
            var yy2 = rotation.Y * y2;
            var zz2 = rotation.Z * z2;
            var wy2 = rotation.W * y2;
            var wz2 = rotation.W * z2;
            Vector3Wide result;
            result.X = Vector<float>.One - yy2 - zz2;
            result.Y = xy2 + wz2;
            result.Z = xz2 - wy2;
            return result;
        }

        /// <summary>
        /// Transforms the unit Y vector using a quaternion.
        /// </summary>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide TransformUnitY(QuaternionWide rotation)
        {
            var x2 = rotation.X + rotation.X;
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xx2 = rotation.X * x2;
            var xy2 = rotation.X * y2;
            var yz2 = rotation.Y * z2;
            var zz2 = rotation.Z * z2;
            var wx2 = rotation.W * x2;
            var wz2 = rotation.W * z2;
            Vector3Wide result;
            result.X = xy2 - wz2;
            result.Y = Vector<float>.One - xx2 - zz2;
            result.Z = yz2 + wx2;
            return result;
        }

        /// <summary>
        /// Transforms the unit Z vector using a quaternion.
        /// </summary>
        /// <param name="rotation">Rotation to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide TransformUnitZ(QuaternionWide rotation)
        {
            var x2 = rotation.X + rotation.X;
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xx2 = rotation.X * x2;
            var xz2 = rotation.X * z2;
            var yy2 = rotation.Y * y2;
            var yz2 = rotation.Y * z2;
            var wx2 = rotation.W * x2;
            var wy2 = rotation.W * y2;
            Vector3Wide result;
            result.X = xz2 + wy2;
            result.Y = yz2 - wx2;
            result.Z = Vector<float>.One - xx2 - yy2;
            return result;
        }

        /// <summary>
        /// Transforms the unit X and unit Y direction using a quaternion.
        /// </summary>
        /// <param name="rotation">Rotation to apply to the vectors.</param>
        /// <param name="x">Transformed unit X vector.</param>
        /// <param name="y">Transformed unit Y vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformUnitXY(in QuaternionWide rotation, out Vector3Wide x, out Vector3Wide y)
        {
            var x2 = rotation.X + rotation.X;
            var y2 = rotation.Y + rotation.Y;
            var z2 = rotation.Z + rotation.Z;
            var xx2 = rotation.X * x2;
            var xy2 = rotation.X * y2;
            var xz2 = rotation.X * z2;
            var yy2 = rotation.Y * y2;
            var yz2 = rotation.Y * z2;
            var zz2 = rotation.Z * z2;
            var wx2 = rotation.W * x2;
            var wy2 = rotation.W * y2;
            var wz2 = rotation.W * z2;
            x.X = Vector<float>.One - yy2 - zz2;
            x.Y = xy2 + wz2;
            x.Z = xz2 - wy2;
            y.X = xy2 - wz2;
            y.Y = Vector<float>.One - xx2 - zz2;
            y.Z = yz2 + wx2;
        }

        /// <summary>
        /// Transforms the unit X and unit Z direction using a quaternion.
        /// </summary>
        /// <param name="rotation">Rotation to apply to the vectors.</param>
        /// <param name="x">Transformed unit X vector.</param>
        /// <param name="z">Transformed unit Z vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformUnitXZ(in QuaternionWide rotation, out Vector3Wide x, out Vector3Wide z)
        {
            var qX2 = rotation.X + rotation.X;
            var qY2 = rotation.Y + rotation.Y;
            var qZ2 = rotation.Z + rotation.Z;

            var YY = qY2 * rotation.Y;
            var ZZ = qZ2 * rotation.Z;
            x.X = Vector<float>.One - YY - ZZ;
            var XY = qX2 * rotation.Y;
            var ZW = qZ2 * rotation.W;
            x.Y = XY + ZW;
            var XZ = qX2 * rotation.Z;
            var YW = qY2 * rotation.W;
            x.Z = XZ - YW;

            var XX = qX2 * rotation.X;
            var XW = qX2 * rotation.W;
            var YZ = qY2 * rotation.Z;
            z.X = XZ + YW;
            z.Y = YZ - XW;
            z.Z = Vector<float>.One - XX - YY;
        }


        /// <summary>
        /// Concatenates the transforms of two quaternions together such that the resulting quaternion, applied as an orientation to a vector v, is equivalent to
        /// transformed = (v * a) * b. Assumes that the memory backing the input and output do not overlap.
        /// </summary>
        /// <param name="a">First quaternion to concatenate.</param>
        /// <param name="b">Second quaternion to concatenate.</param>
        /// <param name="result">Product of the concatenation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConcatenateWithoutOverlap(in QuaternionWide a, in QuaternionWide b, out QuaternionWide result)
        {
            result.X = a.W * b.X + a.X * b.W + a.Z * b.Y - a.Y * b.Z;
            result.Y = a.W * b.Y + a.Y * b.W + a.X * b.Z - a.Z * b.X;
            result.Z = a.W * b.Z + a.Z * b.W + a.Y * b.X - a.X * b.Y;
            result.W = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z;
        }

        /// <summary>
        /// Concatenates the transforms of two quaternions together such that the resulting quaternion, applied as an orientation to a vector v, is equivalent to
        /// transformed = (v * a) * b.
        /// </summary>
        /// <param name="a">First quaternion to concatenate.</param>
        /// <param name="b">Second quaternion to concatenate.</param>
        /// <param name="result">Product of the concatenation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Concatenate(in QuaternionWide a, in QuaternionWide b, out QuaternionWide result)
        {
            ConcatenateWithoutOverlap(a, b, out var tempResult);
            result = tempResult;
        }

        /// <summary>
        /// Concatenates the transforms of two quaternions together such that the resulting quaternion, applied as an orientation to a vector v, is equivalent to
        /// transformed = (v * a) * b.
        /// </summary>
        /// <param name="a">First quaternion to concatenate.</param>
        /// <param name="b">Second quaternion to concatenate.</param>
        /// <returns>Product of the concatenation.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionWide operator *(QuaternionWide a, QuaternionWide b)
        {
            QuaternionWide result;
            result.X = a.W * b.X + a.X * b.W + a.Z * b.Y - a.Y * b.Z;
            result.Y = a.W * b.Y + a.Y * b.W + a.X * b.Z - a.Z * b.X;
            result.Z = a.W * b.Z + a.Z * b.W + a.Y * b.X - a.X * b.Y;
            result.W = a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z;
            return result;
        }

        /// <summary>
        /// Computes the conjugate of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to conjugate.</param>
        /// <param name="result">Conjugated quaternion.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Conjugate(in QuaternionWide quaternion, out QuaternionWide result)
        {
            result.X = quaternion.X;
            result.Y = quaternion.Y;
            result.Z = quaternion.Z;
            result.W = -quaternion.W;
        }

        /// <summary>
        /// Computes the conjugate of the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to conjugate.</param>
        /// <returns>Conjugated quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionWide Conjugate(in QuaternionWide quaternion)
        {
            QuaternionWide result;
            result.X = quaternion.X;
            result.Y = quaternion.Y;
            result.Z = quaternion.Z;
            result.W = -quaternion.W;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(in Vector<int> condition, in QuaternionWide left, in QuaternionWide right, out QuaternionWide result)
        {
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
            result.W = Vector.ConditionalSelect(condition, left.W, right.W);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionWide ConditionalSelect(Vector<int> condition, QuaternionWide left, QuaternionWide right)
        {
            QuaternionWide result;
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
            result.W = Vector.ConditionalSelect(condition, left.W, right.W);
            return result;
        }

        /// <summary>
        /// Gathers values from the first slot of a wide quaternion and puts them into a narrow representation.
        /// </summary>
        /// <param name="source">Wide quaternion to copy values from.</param>
        /// <param name="target">Narrow quaternion to place values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in QuaternionWide source, out Quaternion target)
        {
            //TODO: Check in parameter codegen on indexer.
            target.X = source.X[0];
            target.Y = source.Y[0];
            target.Z = source.Z[0];
            target.W = source.W[0];
        }

        /// <summary>
        /// Gathers values from a quaternion and places them into the first indices of the target wide quaternion.
        /// </summary>
        /// <param name="source">Quaternion to copy values from.</param>
        /// <param name="targetSlot">Wide quaternion to place values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in Quaternion source, ref QuaternionWide targetSlot)
        {
            GatherScatter.GetFirst(ref targetSlot.X) = source.X;
            GatherScatter.GetFirst(ref targetSlot.Y) = source.Y;
            GatherScatter.GetFirst(ref targetSlot.Z) = source.Z;
            GatherScatter.GetFirst(ref targetSlot.W) = source.W;
        }       

        /// <summary>
        /// Writes a value into a slot of the target bundle.
        /// </summary>
        /// <param name="source">Source of the value to write.</param>
        /// <param name="slotIndex">Index of the slot to write into.</param>
        /// <param name="target">Bundle to write the value into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteSlot(in Quaternion source, int slotIndex, ref QuaternionWide target)
        {
            WriteFirst(source, ref GatherScatter.GetOffsetInstance(ref target, slotIndex));
        }

        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="wide">Source of the lane.</param>
        /// <param name="slotIndex">Index of the lane within the wide representation to read.</param>
        /// <param name="narrow">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadSlot(ref QuaternionWide wide, int slotIndex, out Quaternion narrow)
        {
            ref var offset = ref GatherScatter.GetOffsetInstance(ref wide, slotIndex);
            ReadFirst(offset, out narrow);
        }
    }
}