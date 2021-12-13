using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Three dimensional vector with <see cref="Vector{T}.Count"/> (with generic type argument of <see cref="float"/>) SIMD lanes.
    /// </summary>
    public struct Vector3Wide
    {
        /// <summary>
        /// First component of the vector.
        /// </summary>
        public Vector<float> X;
        /// <summary>
        /// Second component of the vector.
        /// </summary>
        public Vector<float> Y;
        /// <summary>
        /// Third component of the vector.
        /// </summary>
        public Vector<float> Z;

        /// <summary>
        /// Creates a vector by populating each component with the given scalar.
        /// </summary>
        /// <param name="s">Scalar to copy into all lanes of the vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3Wide(ref Vector<float> s)
        {
            X = s;
            Y = s;
            Z = s;
        }

        /// <summary>
        /// Creates a vector by populating each component with the given scalar.
        /// </summary>
        /// <param name="s">Scalar to copy into all lanes of the vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3Wide(Vector<float> s)
        {
            X = s;
            Y = s;
            Z = s;
        }

        /// <summary>
        /// Performs a componentwise add between two vectors.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <param name="result">Sum of a and b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <param name="result">Vector with components equal to the input vector added to the input scalar.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Vector3Wide v, in Vector<float> s, out Vector3Wide result)
        {
            result.X = v.X + s;
            result.Y = v.Y + s;
            result.Z = v.Z + s;
        }
        /// <summary>
        /// Performs a componentwise add between two vectors.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <returns>Sum of a and b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator +(Vector3Wide a, Vector3Wide b)
        {
            Vector3Wide result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            return result;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <returns>Vector with components equal to the input vector added to the input scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator +(Vector3Wide v, Vector<float> s)
        {
            Vector3Wide result;
            result.X = v.X + s;
            result.Y = v.Y + s;
            result.Z = v.Z + s;
            return result;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <returns>Vector with components equal to the input vector added to the input scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator +(Vector<float> s, Vector3Wide v)
        {
            Vector3Wide result;
            result.X = v.X + s;
            result.Y = v.Y + s;
            result.Z = v.Z + s;
            return result;
        }

        /// <summary>
        /// Subtracts one vector from another.
        /// </summary>
        /// <param name="a">Vector to subtract from.</param>
        /// <param name="b">Vector to subtract from the first vector.</param>
        /// <param name="result">Vector with components equal the input scalar subtracted from the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
        }

        /// <summary>
        /// Finds the result of subtracting a scalar from every component of a vector.
        /// </summary>
        /// <param name="v">Vector to subtract from.</param>
        /// <param name="s">Scalar to subtract from every component of the vector.</param>
        /// <param name="result">Vector with components equal the input scalar subtracted from the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Vector3Wide v, in Vector<float> s, out Vector3Wide result)
        {
            result.X = v.X - s;
            result.Y = v.Y - s;
            result.Z = v.Z - s;
        }

        /// <summary>
        /// Subtracts one vector from another.
        /// </summary>
        /// <param name="a">Vector to subtract from.</param>
        /// <param name="b">Vector to subtract from the first vector.</param>
        /// <returns>Vector with components equal the input scalar subtracted from the input vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator -(Vector3Wide a, Vector3Wide b)
        {
            Vector3Wide result;
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
            return result;
        }

        /// <summary>
        /// Finds the result of subtracting the components of a vector from a scalar.
        /// </summary>
        /// <param name="v">Vector to subtract from the scalar.</param>
        /// <param name="s">Scalar to subtract from.</param>
        /// <param name="result">Vector with components equal the input vector subtracted from the input scalar.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Vector<float> s, in Vector3Wide v, out Vector3Wide result)
        {
            result.X = s - v.X;
            result.Y = s - v.Y;
            result.Z = s - v.Z;
        }

        /// <summary>
        /// Finds the result of subtracting the components of a vector from a scalar.
        /// </summary>
        /// <param name="v">Vector to subtract from the scalar.</param>
        /// <param name="s">Scalar to subtract from.</param>
        /// <returns>Vector with components equal the input vector subtracted from the input scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator -(Vector3Wide a, Vector<float> b)
        {
            Vector3Wide result;
            result.X = a.X - b;
            result.Y = a.Y - b;
            result.Z = a.Z - b;
            return result;
        }

        /// <summary>
        /// Computes the inner product between two vectors.
        /// </summary>
        /// <param name="a">First vector to dot.</param>
        /// <param name="b">Second vector to dot.</param>
        /// <param name="result">Dot product of a and b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(in Vector3Wide a, in Vector3Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// <summary>
        /// Computes the inner product between two vectors.
        /// </summary>
        /// <param name="a">First vector to dot.</param>
        /// <param name="b">Second vector to dot.</param>
        /// <returns>Dot product of a and b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<float> Dot(Vector3Wide a, Vector3Wide b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// <summary>
        /// Computes the per-component minimum between a scalar value and the components of a vector.
        /// </summary>
        /// <param name="s">Scalar to compare to each vector component.</param>
        /// <param name="v">Vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the smaller of the scalar value and the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(in Vector<float> s, in Vector3Wide v, out Vector3Wide result)
        {
            result.X = Vector.Min(s, v.X);
            result.Y = Vector.Min(s, v.Y);
            result.Z = Vector.Min(s, v.Z);
        }
        /// <summary>
        /// Computes the per-component minimum of two vectors.
        /// </summary>
        /// <param name="a">First vector whose components will be compared.</param>
        /// <param name="b">Second vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the smaller of the two input vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            result.X = Vector.Min(a.X, b.X);
            result.Y = Vector.Min(a.Y, b.Y);
            result.Z = Vector.Min(a.Z, b.Z);
        }

        /// <summary>
        /// Computes the per-component minimum between a scalar value and the components of a vector.
        /// </summary>
        /// <param name="s">Scalar to compare to each vector component.</param>
        /// <param name="v">Vector whose components will be compared.</param>
        /// <returns>Vector with components matching the smaller of the scalar value and the input vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Min(Vector<float> s, Vector3Wide v)
        {
            Vector3Wide result;
            result.X = Vector.Min(s, v.X);
            result.Y = Vector.Min(s, v.Y);
            result.Z = Vector.Min(s, v.Z);
            return result;
        }
        /// <summary>
        /// Computes the per-component minimum of two vectors.
        /// </summary>
        /// <param name="a">First vector whose components will be compared.</param>
        /// <param name="b">Second vector whose components will be compared.</param>
        /// <returns>Vector with components matching the smaller of the two input vectors.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Min(Vector3Wide a, Vector3Wide b)
        {
            Vector3Wide result;
            result.X = Vector.Min(a.X, b.X);
            result.Y = Vector.Min(a.Y, b.Y);
            result.Z = Vector.Min(a.Z, b.Z);
            return result;
        }

        /// <summary>
        /// Computes the per-component maximum between a scalar value and the components of a vector.
        /// </summary>
        /// <param name="s">Scalar to compare to each vector component.</param>
        /// <param name="v">Vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the larger of the scalar value and the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(in Vector<float> s, in Vector3Wide v, out Vector3Wide result)
        {
            result.X = Vector.Max(s, v.X);
            result.Y = Vector.Max(s, v.Y);
            result.Z = Vector.Max(s, v.Z);
        }

        /// <summary>
        /// Computes the per-component maximum of two vectors.
        /// </summary>
        /// <param name="a">First vector whose components will be compared.</param>
        /// <param name="b">Second vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the larger of the two input vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            result.X = Vector.Max(a.X, b.X);
            result.Y = Vector.Max(a.Y, b.Y);
            result.Z = Vector.Max(a.Z, b.Z);
        }

        /// <summary>
        /// Computes the per-component maximum between a scalar value and the components of a vector.
        /// </summary>
        /// <param name="s">Scalar to compare to each vector component.</param>
        /// <param name="v">Vector whose components will be compared.</param>
        /// <returns>Vector with components matching the larger of the scalar value and the input vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Max(Vector<float> s, Vector3Wide v)
        {
            Vector3Wide result;
            result.X = Vector.Max(s, v.X);
            result.Y = Vector.Max(s, v.Y);
            result.Z = Vector.Max(s, v.Z);
            return result;
        }
        /// <summary>
        /// Computes the per-component maximum of two vectors.
        /// </summary>
        /// <param name="a">First vector whose components will be compared.</param>
        /// <param name="b">Second vector whose components will be compared.</param>
        /// <returns>Vector with components matching the larger of the two input vectors.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Max(Vector3Wide a, Vector3Wide b)
        {
            Vector3Wide result;
            result.X = Vector.Max(a.X, b.X);
            result.Y = Vector.Max(a.Y, b.Y);
            result.Z = Vector.Max(a.Z, b.Z);
            return result;
        }


        /// <summary>
        /// Scales a vector by a scalar.
        /// </summary>
        /// <param name="vector">Vector to scale.</param>
        /// <param name="scalar">Scalar to apply to the vector.</param>
        /// <param name="result">Scaled result vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Vector3Wide vector, in Vector<float> scalar, out Vector3Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
        }

        /// <summary>
        /// Divides each component of the vector by the scalar.
        /// </summary>
        /// <param name="vector">Vector to divide.</param>
        /// <param name="scalar">Scalar to divide the vector by.</param>
        /// <returns>Value of the vector divided by the scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator /(Vector3Wide vector, Vector<float> scalar)
        {
            Vector3Wide result;
            var inverse = Vector<float>.One / scalar;
            result.X = vector.X * inverse;
            result.Y = vector.Y * inverse;
            result.Z = vector.Z * inverse;
            return result;
        }

        /// <summary>
        /// Scales a vector by a scalar.
        /// </summary>
        /// <param name="vector">Vector to scale.</param>
        /// <param name="scalar">Scalar to apply to the vector.</param>
        /// <returns>Scaled result vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator *(Vector3Wide vector, Vector<float> scalar)
        {
            Vector3Wide result;
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
            return result;
        }

        /// <summary>
        /// Scales a vector by a scalar.
        /// </summary>
        /// <param name="vector">Vector to scale.</param>
        /// <param name="scalar">Scalar to apply to the vector.</param>
        /// <returns>Scaled result vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator *(Vector<float> scalar, Vector3Wide vector)
        {
            Vector3Wide result;
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
            return result;
        }

        /// <summary>
        /// Computes the absolute value of a vector.
        /// </summary>
        /// <param name="vector">Vector to take the absolute value of.</param>
        /// <param name="result">Absolute value of the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Abs(in Vector3Wide vector, out Vector3Wide result)
        {
            result.X = Vector.Abs(vector.X);
            result.Y = Vector.Abs(vector.Y);
            result.Z = Vector.Abs(vector.Z);
        }

        /// <summary>
        /// Computes the absolute value of a vector.
        /// </summary>
        /// <param name="vector">Vector to take the absolute value of.</param>
        /// <returns>Absolute value of the input vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Abs(Vector3Wide vector)
        {
            Vector3Wide result;
            result.X = Vector.Abs(vector.X);
            result.Y = Vector.Abs(vector.Y);
            result.Z = Vector.Abs(vector.Z);
            return result;
        }

        /// <summary>
        /// Negates a vector.
        /// </summary>
        /// <param name="v">Vector to negate.</param>
        /// <param name="result">Negated vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in Vector3Wide v, out Vector3Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
        }

        /// <summary>
        /// Negates a vector in place and returns a reference to it.
        /// </summary>
        /// <param name="v">Vector to negate.</param>
        /// <returns>Reference to the input parameter, mutated.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide Negate(ref Vector3Wide v)
        {
            v.X = -v.X;
            v.Y = -v.Y;
            v.Z = -v.Z;
            return ref v;
        }

        /// <summary>
        /// Negates a vector.
        /// </summary>
        /// <param name="v">Vector to negate.</param>
        /// <returns>Negated vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator -(Vector3Wide v)
        {
            Vector3Wide result;
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
            return result;
        }

        /// <summary>
        /// Conditionally negates lanes of the vector.
        /// </summary>
        /// <param name="shouldNegate">Mask indicating which lanes should be negated.</param>
        /// <param name="v">Reference to the vector to be conditionally negated.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionallyNegate(in Vector<int> shouldNegate, ref Vector3Wide v)
        {
            v.X = Vector.ConditionalSelect(shouldNegate, -v.X, v.X);
            v.Y = Vector.ConditionalSelect(shouldNegate, -v.Y, v.Y);
            v.Z = Vector.ConditionalSelect(shouldNegate, -v.Z, v.Z);
        }

        /// <summary>
        /// Conditionally negates lanes of the vector.
        /// </summary>
        /// <param name="shouldNegate">Mask indicating which lanes should be negated.</param>
        /// <param name="v">Vector to be conditionally negated.</param>
        /// <param name="negated">Conditionally negated result.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionallyNegate(in Vector<int> shouldNegate, in Vector3Wide v, out Vector3Wide negated)
        {
            negated.X = Vector.ConditionalSelect(shouldNegate, -v.X, v.X);
            negated.Y = Vector.ConditionalSelect(shouldNegate, -v.Y, v.Y);
            negated.Z = Vector.ConditionalSelect(shouldNegate, -v.Z, v.Z);
        }

        /// <summary>
        /// Conditionally negates lanes of the vector.
        /// </summary>
        /// <param name="shouldNegate">Mask indicating which lanes should be negated.</param>
        /// <param name="v">Vector to be conditionally negated.</param>
        /// <returns>Conditionally negated vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide ConditionallyNegate(Vector<int> shouldNegate, Vector3Wide v)
        {
            Vector3Wide negated;
            negated.X = Vector.ConditionalSelect(shouldNegate, -v.X, v.X);
            negated.Y = Vector.ConditionalSelect(shouldNegate, -v.Y, v.Y);
            negated.Z = Vector.ConditionalSelect(shouldNegate, -v.Z, v.Z);
            return negated;
        }

        /// <summary>
        /// Computes the cross product between two vectors, assuming that the vector references are not aliased.
        /// </summary>
        /// <param name="a">First vector to cross.</param>
        /// <param name="b">Second vector to cross.</param>
        /// <param name="result">Result of the cross product.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CrossWithoutOverlap(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            //This will fail if the result reference is actually a or b! 
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Computes the cross product between two vectors.
        /// </summary>
        /// <param name="a">First vector to cross.</param>
        /// <param name="b">Second vector to cross.</param>
        /// <param name="result">Result of the cross product.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            CrossWithoutOverlap(a, b, out var temp);
            result = temp;
        }

        /// <summary>
        /// Computes the cross product between two vectors.
        /// </summary>
        /// <param name="a">First vector to cross.</param>
        /// <param name="b">Second vector to cross.</param>
        /// <returns>Result of the cross product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Cross(Vector3Wide a, Vector3Wide b)
        {
            Vector3Wide result;
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
            return result;
        }

        /// <summary>
        /// Computes the squared length of a vector.
        /// </summary>
        /// <param name="v">Vector to compute the squared length of.</param>
        /// <param name="lengthSquared">Squared length of the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LengthSquared(in Vector3Wide v, out Vector<float> lengthSquared)
        {
            lengthSquared = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        }

        /// <summary>
        /// Computes the length of a vector.
        /// </summary>
        /// <param name="v">Vector to compute the length of.</param>
        /// <param name="length">Length of the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(in Vector3Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }

        /// <summary>
        /// Computes the squared length of a vector.
        /// </summary>
        /// <param name="v">Vector to compute the squared length of.</param>
        /// <returns>Squared length of the input vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<float> LengthSquared(Vector3Wide v)
        {
            return v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        }

        /// <summary>
        /// Computes the length of a vector.
        /// </summary>
        /// <param name="v">Vector to compute the length of.</param>
        /// <returns>Length of the input vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<float> Length(Vector3Wide v)
        {
            return Vector.SquareRoot(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }

        /// <summary>
        /// Computes the squared length of the vector.
        /// </summary>
        /// <returns>Squared length of this vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector<float> LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }

        /// <summary>
        /// Computes the length of the vector.
        /// </summary>
        /// <returns>Length of this vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector<float> Length()
        {
            return Vector.SquareRoot(X * X + Y * Y + Z * Z);
        }

        /// <summary>
        /// Computes the distance between two vectors.
        /// </summary>
        /// <param name="a">First vector in the pair.</param>
        /// <param name="b">Second vector in the pair.</param>
        /// <param name="distance">Distance between a and b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Distance(in Vector3Wide a, in Vector3Wide b, out Vector<float> distance)
        {
            var x = b.X - a.X;
            var y = b.Y - a.Y;
            var z = b.Z - a.Z;
            distance = Vector.SquareRoot(x * x + y * y + z * z);
        }

        /// <summary>
        /// Computes the squared distance between two vectors.
        /// </summary>
        /// <param name="a">First vector in the pair.</param>
        /// <param name="b">Second vector in the pair.</param>
        /// <param name="distanceSquared">Squared distance between a and b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DistanceSquared(in Vector3Wide a, in Vector3Wide b, out Vector<float> distanceSquared)
        {
            var x = b.X - a.X;
            var y = b.Y - a.Y;
            var z = b.Z - a.Z;
            distanceSquared = x * x + y * y + z * z;
        }

        /// <summary>
        /// Computes the distance between two vectors.
        /// </summary>
        /// <param name="a">First vector in the pair.</param>
        /// <param name="b">Second vector in the pair.</param>
        /// <returns>Distance between a and b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<float> Distance(Vector3Wide a, Vector3Wide b)
        {
            var x = b.X - a.X;
            var y = b.Y - a.Y;
            var z = b.Z - a.Z;
            return Vector.SquareRoot(x * x + y * y + z * z);
        }

        /// <summary>
        /// Computes the squared distance between two vectors.
        /// </summary>
        /// <param name="a">First vector in the pair.</param>
        /// <param name="b">Second vector in the pair.</param>
        /// <returns>Squared distance between a and b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<float> DistanceSquared(Vector3Wide a, Vector3Wide b)
        {
            var x = b.X - a.X;
            var y = b.Y - a.Y;
            var z = b.Z - a.Z;
            return x * x + y * y + z * z;
        }

        //TODO: We have better intrinsics options here for a fast rsqrt path.

        /// <summary>
        /// Computes a unit length vector pointing in the same direction as the input.
        /// </summary>
        /// <param name="v">Vector to normalize.</param>
        /// <param name="result">Vector pointing in the same direction as the input, but with unit length.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Normalize(in Vector3Wide v, out Vector3Wide result)
        {
            Length(v, out var length);
            var scale = Vector<float>.One / length;
            Scale(v, scale, out result);
        }

        /// <summary>
        /// Computes a unit length vector pointing in the same direction as the input.
        /// </summary>
        /// <param name="v">Vector to normalize.</param>
        /// <returns>Vector pointing in the same direction as the input, but with unit length.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Normalize(Vector3Wide v)
        {
            Length(v, out var length);
            var scale = Vector<float>.One / length;
            return v * scale;
        }

        /// <summary>
        /// Selects the left or right input for each lane depending on a mask.
        /// </summary>
        /// <param name="condition">Mask to use to decide between the left and right value for each lane..</param>
        /// <param name="left">Value to choose if the condition mask is set.</param>
        /// <param name="right">Value to choose if the condition mask is unset.</param>
        /// <param name="result">Blended result.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(in Vector<int> condition, in Vector3Wide left, in Vector3Wide right, out Vector3Wide result)
        {
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
        }

        /// <summary>
        /// Selects the left or right input for each lane depending on a mask.
        /// </summary>
        /// <param name="condition">Mask to use to decide between the left and right value for each lane..</param>
        /// <param name="left">Value to choose if the condition mask is set.</param>
        /// <param name="right">Value to choose if the condition mask is unset.</param>
        /// <returns>Blended result.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide ConditionalSelect(Vector<int> condition, Vector3Wide left, Vector3Wide right)
        {
            Vector3Wide result;
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
            return result;
        }

        /// <summary>
        /// Multiplies the components of one vector with another.
        /// </summary>
        /// <param name="a">First vector to multiply.</param>
        /// <param name="b">Second vector to multiply.</param>
        /// <param name="result">Result of the multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X * b.X;
            result.Y = a.Y * b.Y;
            result.Z = a.Z * b.Z;
        }

        /// <summary>
        /// Multiplies the components of one vector with another.
        /// </summary>
        /// <param name="a">First vector to multiply.</param>
        /// <param name="b">Second vector to multiply.</param>
        /// <returns>Result of the multiplication.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator *(Vector3Wide a, Vector3Wide b)
        {
            Vector3Wide result;
            result.X = a.X * b.X;
            result.Y = a.Y * b.Y;
            result.Z = a.Z * b.Z;
            return result;
        }

        //TODO: most of these gatherscattery functions are fallbacks in AOS->SOA conversions and should often be replaced by explicit vectorized transposes.
        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="wide">Source of the lane.</param>
        /// <param name="slotIndex">Index of the lane within the wide representation to read.</param>
        /// <param name="narrow">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadSlot(ref Vector3Wide wide, int slotIndex, out Vector3 narrow)
        {
            ref var offset = ref GatherScatter.GetOffsetInstance(ref wide, slotIndex);
            ReadFirst(offset, out narrow);
        }

        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="source">Source of the lane.</param>
        /// <param name="target">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in Vector3Wide source, out Vector3 target)
        {
            target.X = source.X[0];
            target.Y = source.Y[0];
            target.Z = source.Z[0];
        }

        /// <summary>
        /// Gathers values from a vector and places them into the first indices of the target vector.
        /// </summary>
        /// <param name="source">Vector to copy values from.</param>
        /// <param name="targetSlot">Wide vectorto place values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in Vector3 source, ref Vector3Wide targetSlot)
        {
            GatherScatter.GetFirst(ref targetSlot.X) = source.X;
            GatherScatter.GetFirst(ref targetSlot.Y) = source.Y;
            GatherScatter.GetFirst(ref targetSlot.Z) = source.Z;
        }

        /// <summary>
        /// Writes a value into a slot of the target bundle.
        /// </summary>
        /// <param name="source">Source of the value to write.</param>
        /// <param name="slotIndex">Index of the slot to write into.</param>
        /// <param name="target">Bundle to write the value into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteSlot(in Vector3 source, int slotIndex, ref Vector3Wide target)
        {
            WriteFirst(source, ref GatherScatter.GetOffsetInstance(ref target, slotIndex));
        }

        /// <summary>
        /// Expands each scalar value to every slot of the bundle.
        /// </summary>
        /// <param name="source">Source value to write to every bundle slot.</param>
        /// <param name="broadcasted">Bundle containing the source's components in every slot.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in Vector3 source, out Vector3Wide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X);
            broadcasted.Y = new Vector<float>(source.Y);
            broadcasted.Z = new Vector<float>(source.Z);
        }

        /// <summary>
        /// Expands each scalar value to every slot of the bundle.
        /// </summary>
        /// <param name="source">Source value to write to every bundle slot.</param>
        /// <returns>Bundle containing the source's components in every slot.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Broadcast(Vector3 source)
        {
            Vector3Wide broadcasted;
            broadcasted.X = new Vector<float>(source.X);
            broadcasted.Y = new Vector<float>(source.Y);
            broadcasted.Z = new Vector<float>(source.Z);
            return broadcasted;
        }

        /// <summary>
        /// Takes a slot from the source vector and broadcasts it into all slots of the target vector.
        /// </summary>
        /// <param name="source">Vector to pull values from.</param>
        /// <param name="slotIndex">Slot in the source vectors to pull values from.</param>
        /// <param name="broadcasted">Target vector to be filled with the selected data.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Rebroadcast(in Vector3Wide source, int slotIndex, out Vector3Wide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X[slotIndex]);
            broadcasted.Y = new Vector<float>(source.Y[slotIndex]);
            broadcasted.Z = new Vector<float>(source.Z[slotIndex]);
        }


        /// <summary>
        /// Takes a slot from the source vector and broadcasts it into all slots of the target vector.
        /// </summary>
        /// <param name="source">Vector to pull values from.</param>
        /// <param name="slotIndex">Slot in the source vectors to pull values from.</param>
        /// <returns>Target vector to be filled with the selected data.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide Rebroadcast(Vector3Wide source, int slotIndex)
        {
            Vector3Wide broadcasted;
            broadcasted.X = new Vector<float>(source.X[slotIndex]);
            broadcasted.Y = new Vector<float>(source.Y[slotIndex]);
            broadcasted.Z = new Vector<float>(source.Z[slotIndex]);
            return broadcasted;
        }

        /// <summary>
        /// Takes a slot from the source vector and places it into a slot of the target.
        /// </summary>
        /// <param name="source">Vector to pull values from.</param>
        /// <param name="sourceSlotIndex">Slot in the source vectors to pull values from.</param>
        /// <param name="target">Target vector whose slot will be filled with the selected data.</param>
        /// <param name="targetSlotIndex">Slot in the target vectors to write values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CopySlot(ref Vector3Wide source, int sourceSlotIndex, ref Vector3Wide target, int targetSlotIndex)
        {
            ref var sourceSlot = ref GatherScatter.GetOffsetInstance(ref source, sourceSlotIndex);
            ref var targetSlot = ref GatherScatter.GetOffsetInstance(ref target, targetSlotIndex);
            GatherScatter.GetFirst(ref targetSlot.X) = sourceSlot.X[0];
            GatherScatter.GetFirst(ref targetSlot.Y) = sourceSlot.Y[0];
            GatherScatter.GetFirst(ref targetSlot.Z) = sourceSlot.Z[0];
        }

        public override string ToString()
        {
            return $"<{X}, {Y}, {Z}>";
        }

    }
}
