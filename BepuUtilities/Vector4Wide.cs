using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Four dimensional vector with <see cref="Vector{T}.Count"/> (with generic type argument of <see cref="float"/>) SIMD lanes.
    /// </summary>
    public struct Vector4Wide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;
        public Vector<float> W;
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in Vector4 source, out Vector4Wide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X);
            broadcasted.Y = new Vector<float>(source.Y);
            broadcasted.Z = new Vector<float>(source.Z);
            broadcasted.W = new Vector<float>(source.W);
        }


        /// <summary>
        /// Performs a componentwise add between two vectors.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <param name="result">Sum of a and b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(Vector4Wide a, Vector4Wide b, out Vector4Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            result.W = a.W + b.W;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <param name="result">Vector with components equal to the input vector added to the input scalar.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(Vector4Wide v, Vector<float> s, out Vector4Wide result)
        {
            result.X = v.X + s;
            result.Y = v.Y + s;
            result.Z = v.Z + s;
            result.W = v.W + s;
        }
        /// <summary>
        /// Performs a componentwise add between two vectors.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <returns>Sum of a and b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4Wide operator +(Vector4Wide a, Vector4Wide b)
        {
            Vector4Wide result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            result.W = a.W + b.W;
            return result;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <returns>Vector with components equal to the input vector added to the input scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4Wide operator +(Vector4Wide v, Vector<float> s)
        {
            Vector4Wide result;
            result.X = v.X + s;
            result.Y = v.Y + s;
            result.Z = v.Z + s;
            result.W = v.W + s;
            return result;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <returns>Vector with components equal to the input vector added to the input scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector4Wide operator +(Vector<float> s, Vector4Wide v)
        {
            Vector4Wide result;
            result.X = v.X + s;
            result.Y = v.Y + s;
            result.Z = v.Z + s;
            result.W = v.W + s;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Vector4Wide a, in Vector4Wide b, out Vector4Wide result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
            result.W = a.W - b.W;
        }
  

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(in Vector4Wide a, in Vector4Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
        }

        /// <summary>
        /// Computes the per-component minimum of two vectors.
        /// </summary>
        /// <param name="a">First vector whose components will be compared.</param>
        /// <param name="b">Second vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the smaller of the two input vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(in Vector4Wide a, in Vector4Wide b, out Vector4Wide result)
        {
            result.X = Vector.Min(a.X, b.X);
            result.Y = Vector.Min(a.Y, b.Y);
            result.Z = Vector.Min(a.Z, b.Z);
            result.W = Vector.Min(a.W, b.W);
        }
        /// <summary>
        /// Computes the per-component maximum of two vectors.
        /// </summary>
        /// <param name="a">First vector whose components will be compared.</param>
        /// <param name="b">Second vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the larger of the two input vectors.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(in Vector4Wide a, in Vector4Wide b, out Vector4Wide result)
        {
            result.X = Vector.Max(a.X, b.X);
            result.Y = Vector.Max(a.Y, b.Y);
            result.Z = Vector.Max(a.Z, b.Z);
            result.W = Vector.Max(a.W, b.W);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Vector4Wide vector, in Vector<float> scalar, out Vector4Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
            result.W = vector.W * scalar;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Abs(in Vector4Wide vector, out Vector4Wide result)
        {
            result.X = Vector.Abs(vector.X);
            result.Y = Vector.Abs(vector.Y);
            result.Z = Vector.Abs(vector.Z);
            result.W = Vector.Abs(vector.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in Vector4Wide v, out Vector4Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
            result.W = -v.W;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector4Wide Negate(ref Vector4Wide v)
        {
            v.X = -v.X;
            v.Y = -v.Y;
            v.Z = -v.Z;
            v.Z = -v.W;
            return ref v;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LengthSquared(in Vector4Wide v, out Vector<float> lengthSquared)
        {
            lengthSquared = v.X * v.X + v.Y * v.Y + v.Z * v.Z + v.W * v.W;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(in Vector4Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y + v.Z * v.Z + v.W * v.W);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Distance(in Vector4Wide a, in Vector4Wide b, out Vector<float> distance)
        {
            Subtract(b, a, out var offset);
            Length(offset, out distance);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Normalize(in Vector4Wide v, out Vector4Wide result)
        {
            Length(v, out var length);
            var scale = Vector<float>.One / length;
            Scale(v, scale, out result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(in Vector<int> condition, in Vector4Wide left, in Vector4Wide right, out Vector4Wide result)
        {
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
            result.W = Vector.ConditionalSelect(condition, left.W, right.W);
        }
        
        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="wide">Source of the lane.</param>
        /// <param name="slotIndex">Index of the lane within the wide representation to read.</param>
        /// <param name="narrow">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadSlot(ref Vector4Wide wide, int slotIndex, out Vector4 narrow)
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
        public static void ReadFirst(in Vector4Wide source, out Vector4 target)
        {
            target.X = source.X[0];
            target.Y = source.Y[0];
            target.Z = source.Z[0];
            target.W = source.W[0];
        }

        /// <summary>
        /// Gathers values from a vector and places them into the first indices of the target vector.
        /// </summary>
        /// <param name="source">Vector to copy values from.</param>
        /// <param name="targetSlot">Wide vectorto place values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in Vector4 source, ref Vector4Wide targetSlot)
        {
            GatherScatter.GetFirst(ref targetSlot.X) = source.X;
            GatherScatter.GetFirst(ref targetSlot.Y) = source.Y;
            GatherScatter.GetFirst(ref targetSlot.Z) = source.Z;
            GatherScatter.GetFirst(ref targetSlot.W) = source.W;
        }

        public override string ToString()
        {
            return $"<{X}, {Y}, {Z}, {W}>";
        }
    }
}
