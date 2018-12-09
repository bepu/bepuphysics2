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

        
        public override string ToString()
        {
            return $"<{X}, {Y}>";
        }


    }
}
