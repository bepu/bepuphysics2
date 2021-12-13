using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Two dimensional vector with <see cref="Vector{T}.Count"/> (with generic type argument of <see cref="float"/>) SIMD lanes.
    /// </summary>
    public struct Vector2Wide
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
        /// Performs a componentwise add between two vectors.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <param name="result">Sum of a and b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Vector2Wide a, in Vector2Wide b, out Vector2Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
        }
        /// <summary>
        /// Performs a componentwise add between two vectors.
        /// </summary>
        /// <param name="a">First vector to add.</param>
        /// <param name="b">Second vector to add.</param>
        /// <returns>Sum of a and b.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2Wide operator +(Vector2Wide a, Vector2Wide b)
        {
            Vector2Wide result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            return result;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <returns>Vector with components equal to the input vector added to the input scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2Wide operator +(Vector2Wide v, Vector<float> s)
        {
            Vector2Wide result;
            result.X = v.X + s;
            result.Y = v.Y + s;
            return result;
        }
        /// <summary>
        /// Finds the result of adding a scalar to every component of a vector.
        /// </summary>
        /// <param name="v">Vector to add to.</param>
        /// <param name="s">Scalar to add to every component of the vector.</param>
        /// <returns>Vector with components equal to the input vector added to the input scalar.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2Wide operator +(Vector<float> s, Vector2Wide v)
        {
            Vector2Wide result;
            result.X = v.X + s;
            result.Y = v.Y + s;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Vector2Wide a, in Vector2Wide b, out Vector2Wide result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(in Vector2Wide a, in Vector2Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Vector2Wide vector, in Vector<float> scalar, out Vector2Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2Wide operator *(Vector2Wide vector, Vector<float> scalar)
        {
            Vector2Wide result;
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            return result;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2Wide operator *(Vector<float> scalar, Vector2Wide vector)
        {
            Vector2Wide result;
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in Vector2Wide v, out Vector2Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionallyNegate(in Vector<int> shouldNegate, ref Vector2Wide v)
        {
            v.X = Vector.ConditionalSelect(shouldNegate, -v.X, v.X);
            v.Y = Vector.ConditionalSelect(shouldNegate, -v.Y, v.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionallyNegate(in Vector<int> shouldNegate, in Vector2Wide v, out Vector2Wide negated)
        {
            negated.X = Vector.ConditionalSelect(shouldNegate, -v.X, v.X);
            negated.Y = Vector.ConditionalSelect(shouldNegate, -v.Y, v.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(in Vector<int> condition, in Vector2Wide left, in Vector2Wide right, out Vector2Wide result)
        {
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LengthSquared(in Vector2Wide v, out Vector<float> lengthSquared)
        {
            lengthSquared = v.X * v.X + v.Y * v.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(in Vector2Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void PerpDot(in Vector2Wide a, in Vector2Wide b, out Vector<float> result)
        {
            result = a.Y * b.X - a.X * b.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in Vector2 source, out Vector2Wide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X);
            broadcasted.Y = new Vector<float>(source.Y);
        }

        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="source">Source of the lane.</param>
        /// <param name="target">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in Vector2Wide source, out Vector2 target)
        {
            target.X = source.X[0];
            target.Y = source.Y[0];
        }

        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="wide">Source of the lane.</param>
        /// <param name="slotIndex">Index of the lane within the wide representation to read.</param>
        /// <param name="narrow">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadSlot(ref Vector2Wide wide, int slotIndex, out Vector2 narrow)
        {
            ref var offset = ref GatherScatter.GetOffsetInstance(ref wide, slotIndex);
            ReadFirst(offset, out narrow);
        }

        /// <summary>
        /// Gathers values from a vector and places them into the first indices of the target vector.
        /// </summary>
        /// <param name="source">Vector to copy values from.</param>
        /// <param name="targetSlot">Wide vectorto place values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in Vector2 source, ref Vector2Wide targetSlot)
        {
            GatherScatter.GetFirst(ref targetSlot.X) = source.X;
            GatherScatter.GetFirst(ref targetSlot.Y) = source.Y;
        }

        /// <summary>
        /// Writes a value into a slot of the target bundle.
        /// </summary>
        /// <param name="source">Source of the value to write.</param>
        /// <param name="slotIndex">Index of the slot to write into.</param>
        /// <param name="target">Bundle to write the value into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteSlot(in Vector2 source, int slotIndex, ref Vector2Wide target)
        {
            WriteFirst(source, ref GatherScatter.GetOffsetInstance(ref target, slotIndex));
        }

        public override string ToString()
        {
            return $"<{X}, {Y}>";
        }

    }
}
