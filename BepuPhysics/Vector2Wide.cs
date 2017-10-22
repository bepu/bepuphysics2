using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    public struct Vector2Wide
    {
        public Vector<float> X;
        public Vector<float> Y;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Vector2Wide a, ref Vector2Wide b, out Vector2Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Vector2Wide a, ref Vector2Wide b, out Vector2Wide result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
        }                  

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(ref Vector2Wide a, ref Vector2Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Vector2Wide vector, ref Vector<float> scalar, out Vector2Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref Vector2Wide v, out Vector2Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(ref Vector2Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y);
        }
    }
}
