using System.Numerics;

namespace BepuPhysics
{
    internal static class VectorExtensions
    {
        extension(Vector<int>)
        {
            public static Vector<int> operator !(Vector<int> value)
            {
                return ~value;
            }

            public static Vector<int> operator <(Vector<int> left, Vector<int> right)
            {
                return Vector.LessThan(left, right);
            }

            public static Vector<int> operator >(Vector<int> left, Vector<int> right)
            {
                return Vector.GreaterThan(left, right);
            }

            public static Vector<int> operator <=(Vector<int> left, Vector<int> right)
            {
                return Vector.LessThanOrEqual(left, right);
            }

            public static Vector<int> operator >=(Vector<int> left, Vector<int> right)
            {
                return Vector.GreaterThanOrEqual(left, right);
            }
        }
        extension(Vector<float>)
        {
            public static Vector<int> operator <(Vector<float> left, Vector<float> right)
            {
                return Vector.LessThan(left, right);
            }

            public static Vector<int> operator >(Vector<float> left, Vector<float> right)
            {
                return Vector.GreaterThan(left, right);
            }

            public static Vector<int> operator <=(Vector<float> left, Vector<float> right)
            {
                return Vector.LessThanOrEqual(left, right);
            }

            public static Vector<int> operator >=(Vector<float> left, Vector<float> right)
            {
                return Vector.GreaterThanOrEqual(left, right);
            }

            // Can be removed in .NET 11
            public static Vector<float> NegativeOne => new(-1f);
        }
        extension(Vector)
        {
            public static Vector<T> Min<T>(Vector<T> a, Vector<T> b, Vector<T> c)
            {
                return Vector.Min(Vector.Min(a, b), c);
            }
            public static Vector<T> Max<T>(Vector<T> a, Vector<T> b, Vector<T> c)
            {
                return Vector.Max(Vector.Max(a, b), c);
            }
            public static Vector<T> ClampZeroOne<T>(Vector<T> value)
            {
                return Vector.Clamp(value, Vector<T>.Zero, Vector<T>.One);
            }
        }
    }
}
