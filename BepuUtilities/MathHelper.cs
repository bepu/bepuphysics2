using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Contains helper math methods.
    /// </summary>
    public static class MathHelper
    {
        /// <summary>
        /// Approximate value of Pi.
        /// </summary>
        public const float Pi = 3.141592653589793239f;

        /// <summary>
        /// Approximate value of Pi multiplied by two.
        /// </summary>
        public const float TwoPi = 6.283185307179586477f;

        /// <summary>
        /// Approximate value of Pi divided by two.
        /// </summary>
        public const float PiOver2 = 1.570796326794896619f;

        /// <summary>
        /// Approximate value of Pi divided by four.
        /// </summary>
        public const float PiOver4 = 0.785398163397448310f;

        /// <summary>
        /// Clamps a value between a minimum and maximum value.
        /// </summary>
        /// <param name="value">Value to clamp.</param>
        /// <param name="min">Minimum value.  If the value is less than this, the minimum is returned instead.</param>
        /// <param name="max">Maximum value.  If the value is more than this, the maximum is returned instead.</param>
        /// <returns>Clamped value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float value, float min, float max)
        {
            if (value < min)
                return min;
            else if (value > max)
                return max;
            return value;
        }


        /// <summary>
        /// Returns the higher value of the two parameters.
        /// </summary>
        /// <param name="a">First value.</param>
        /// <param name="b">Second value.</param>
        /// <returns>Higher value of the two parameters.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Max(float a, float b)
        {
            return a > b ? a : b;
        }

        /// <summary>
        /// Returns the lower value of the two parameters.
        /// </summary>
        /// <param name="a">First value.</param>
        /// <param name="b">Second value.</param>
        /// <returns>Lower value of the two parameters.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Min(float a, float b)
        {
            return a < b ? a : b;
        }

        /// <summary>
        /// Clamps a value between a minimum and maximum value.
        /// </summary>
        /// <param name="value">Value to clamp.</param>
        /// <param name="min">Minimum value.  If the value is less than this, the minimum is returned instead.</param>
        /// <param name="max">Maximum value.  If the value is more than this, the maximum is returned instead.</param>
        /// <returns>Clamped value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Clamp(int value, int min, int max)
        {
            if (value < min)
                return min;
            else if (value > max)
                return max;
            return value;
        }


        /// <summary>
        /// Returns the higher value of the two parameters.
        /// </summary>
        /// <param name="a">First value.</param>
        /// <param name="b">Second value.</param>
        /// <returns>Higher value of the two parameters.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Max(int a, int b)
        {
            return a > b ? a : b;
        }

        /// <summary>
        /// Returns the lower value of the two parameters.
        /// </summary>
        /// <param name="a">First value.</param>
        /// <param name="b">Second value.</param>
        /// <returns>Lower value of the two parameters.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Min(int a, int b)
        {
            return a < b ? a : b;
        }

        /// <summary>
        /// Clamps a value between a minimum and maximum value.
        /// </summary>
        /// <param name="value">Value to clamp.</param>
        /// <param name="min">Minimum value.  If the value is less than this, the minimum is returned instead.</param>
        /// <param name="max">Maximum value.  If the value is more than this, the maximum is returned instead.</param>
        /// <returns>Clamped value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long Clamp(long value, long min, long max)
        {
            if (value < min)
                return min;
            else if (value > max)
                return max;
            return value;
        }


        /// <summary>
        /// Returns the higher value of the two parameters.
        /// </summary>
        /// <param name="a">First value.</param>
        /// <param name="b">Second value.</param>
        /// <returns>Higher value of the two parameters.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long Max(long a, long b)
        {
            return a > b ? a : b;
        }

        /// <summary>
        /// Returns the lower value of the two parameters.
        /// </summary>
        /// <param name="a">First value.</param>
        /// <param name="b">Second value.</param>
        /// <returns>Lower value of the two parameters.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long Min(long a, long b)
        {
            return a < b ? a : b;
        }

        /// <summary>
        /// Converts degrees to radians.
        /// </summary>
        /// <param name="degrees">Degrees to convert.</param>
        /// <returns>Radians equivalent to the input degrees.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToRadians(float degrees)
        {
            return degrees * (Pi / 180f);
        }

        /// <summary>
        /// Converts radians to degrees.
        /// </summary>
        /// <param name="radians">Radians to convert.</param>
        /// <returns>Degrees equivalent to the input radians.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToDegrees(float radians)
        {
            return radians * (180f / Pi);
        }


        /// <summary>
        /// Returns -1 if the value is negative and 1 otherwise.
        /// </summary>
        /// <param name="x">Value to compute the sign of.</param>
        /// <returns>-1 if the input is negative, and 1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float BinarySign(float x)
        {
            return x < 0 ? -1 : 1;
        }

        //Note that these cos/sin implementations are not here for performance, but rather to:
        //1) Provide a SIMD accelerated version for wide processing, and
        //2) Provide a scalar implementation that is consistent with the SIMD version for systems which need to match its behavior.
        //The main motivating use case is the pose integrator (which is scalar) and the sweep tests (which are widely vectorized).

        /// <summary>
        /// Computes an approximation of cosine. Maximum error a little above 3e-6.
        /// </summary>
        /// <param name="x">Value to take the cosine of.</param>
        /// <returns>Approximate cosine of the input value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Cos(float x)
        {
            //This exists primarily for consistency between the PoseIntegrator and sweeps, not necessarily for raw performance relative to Math.Cos.
            if (x < 0)
                x = -x;
            var intervalIndex = x * (1f / TwoPi);
            x -= (int)intervalIndex * TwoPi;

            //[0, pi/2] = f(x)
            //(pi/2, pi] = -f(Pi - x)
            //(pi, 3 * pi / 2] = -f(x - Pi)
            //(3*pi/2, 2*pi] = f(2 * Pi - x)
            //This could be done more cleverly.
            bool negate;
            if (x < Pi)
            {
                if (x < PiOver2)
                {
                    negate = false;
                }
                else
                {
                    x = Pi - x;
                    negate = true;
                }
            }
            else
            {
                if (x < 3 * PiOver2)
                {
                    x = x - Pi;
                    negate = true;
                }
                else
                {
                    x = TwoPi - x;
                    negate = false;
                }
            }

            //The expression is a rational interpolation from 0 to Pi/2. Maximum error is a little more than 3e-6.
            var x2 = x * x;
            var x3 = x2 * x;
            //TODO: This could be reorganized into two streams of FMAs if that was available.
            var numerator = 1 - 0.24f * x - 0.4266f * x2 + 0.110838f * x3;
            var denominator = 1 - 0.240082f * x + 0.0741637f * x2 - 0.0118786f * x3;
            var result = numerator / denominator;
            return negate ? -result : result;

        }
        /// <summary>
        /// Computes an approximation of sine. Maximum error a little above 3e-6.
        /// </summary>
        /// <param name="x">Value to take the sine of.</param>
        /// <returns>Approximate sine of the input value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Sin(float x)
        {
            return Cos(x - PiOver2);
        }

        /// <summary>
        /// Computes an approximation of cosine. Maximum error a little above 3e-6.
        /// </summary>
        /// <param name="x">Values to take the cosine of.</param>
        /// <returns>Approximate cosine of the input values.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cos(in Vector<float> x, out Vector<float> result)
        {
            //This exists primarily for consistency between the PoseIntegrator and sweeps, not necessarily for raw performance relative to Math.Cos.
            var periodX = Vector.Abs(x);
            //TODO: No floor or truncate available... may want to revisit later.
            periodX = periodX - TwoPi * Vector.ConvertToSingle(Vector.ConvertToInt32(periodX * (1f / TwoPi)));

            //[0, pi/2] = f(x)
            //(pi/2, pi] = -f(Pi - x)
            //(pi, 3 * pi / 2] = -f(x - Pi)
            //(3*pi/2, 2*pi] = f(2 * Pi - x)
            //This could be done more cleverly.
            Vector<float> y;
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, new Vector<float>(PiOver2)), new Vector<float>(Pi) - periodX, periodX);
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, new Vector<float>(Pi)), new Vector<float>(-Pi) + periodX, y);
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, new Vector<float>(3 * PiOver2)), new Vector<float>(TwoPi) - periodX, y);

            //The expression is a rational interpolation from 0 to Pi/2. Maximum error is a little more than 3e-6.
            var y2 = y * y;
            var y3 = y2 * y;
            //TODO: This could be reorganized into two streams of FMAs if that was available.
            var numerator = Vector<float>.One - 0.24f * y - 0.4266f * y2 + 0.110838f * y3;
            var denominator = Vector<float>.One - 0.240082f * y + 0.0741637f * y2 - 0.0118786f * y3;
            result = numerator / denominator;
            result = Vector.ConditionalSelect(
                Vector.BitwiseAnd(
                    Vector.GreaterThan(periodX, new Vector<float>(PiOver2)),
                    Vector.LessThan(periodX, new Vector<float>(3 * PiOver2))), -result, result);
        }
        /// <summary>
        /// Computes an approximation of sine. Maximum error a little above 3e-6.
        /// </summary>
        /// <param name="x">Value to take the sine of.</param>
        /// <returns>Approximate sine of the input value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Sin(in Vector<float> x, out Vector<float> result)
        {
            Cos(x - new Vector<float>(PiOver2), out result);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApproximateAcos(in Vector<float> x, out Vector<float> acos)
        {
            //TODO: Could probably do better than this. Definitely don't need to use a square root.
            //acos(x) ~= (pi / (2 * sqrt(2))) * sqrt(2 - 2 * x), for 0<=x<=1
            //acos(x) ~= pi - (pi / (2 * sqrt(2))) * sqrt(2 + 2 * x), for -1<=x<=0
            var two = new Vector<float>(2f);
            acos = new Vector<float>(1.11072073454f) * Vector.SquareRoot(Vector.Max(Vector<float>.Zero, two - two * Vector.Abs(x)));
            acos = Vector.ConditionalSelect(Vector.LessThan(x, Vector<float>.Zero), new Vector<float>(Pi) - acos, acos);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Floor(in Vector<float> x, out Vector<float> result)
        {
            //This is far from ideal. You could probably do better- especially with platform intrinsics.
            var intX = Vector.ConvertToInt32(x);
            result = Vector.ConvertToSingle(Vector.ConditionalSelect(Vector.LessThan(x, Vector<float>.Zero), intX - Vector<int>.One, intX));
        }

        /// <summary>
        /// Gets the change in angle from a to b as a signed value from -pi to pi.
        /// </summary>
        /// <param name="a">Source angle.</param>
        /// <param name="b">Target angle.</param>
        /// <param name="difference">Difference between a and b, expressed as a value from -pi to pi.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetSignedAngleDifference(in Vector<float> a, in Vector<float> b, out Vector<float> difference)
        {
            var pi = new Vector<float>(Pi);
            var half = new Vector<float>(0.5f);
            var x = (b - a) * new Vector<float>(1f / TwoPi) + half;
            Floor(x, out var flooredX);
            difference = (x - flooredX - half) * TwoPi;
        }
    }
}
