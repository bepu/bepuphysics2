using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    public struct Vector2Wide
    {
        public Vector<float> X;
        public Vector<float> Y;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Vector2Wide a, in Vector2Wide b, out Vector2Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
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
        public static void Negate(in Vector2Wide v, out Vector2Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
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

        public override string ToString()
        {
            return $"<{X}, {Y}>";
        }
  
    }
}
