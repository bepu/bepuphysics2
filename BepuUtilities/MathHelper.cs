using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

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

        /// <summary>
        /// Computes an approximation of cosine. Maximum error a little below 8e-7 for the interval -2 * Pi to 2 * Pi. Values further from the interval near zero have gracefully degrading error.
        /// </summary>
        /// <param name="x">Value to take the cosine of.</param>
        /// <returns>Approximate cosine of the input value.</returns>
        public static float Cos(float x)
        {
            //Rational approximation over [0, pi/2], use symmetry for the rest.
            var periodCount = x * (float)(0.5 / Math.PI);
            var periodFraction = periodCount - MathF.Floor(periodCount); //This is a source of error as you get away from 0.
            var periodX = periodFraction * TwoPi;

            //[0, pi/2] = f(x)
            //(pi/2, pi] = -f(Pi - x)
            //(pi, 3 * pi / 2] = -f(x - Pi)
            //(3*pi/2, 2*pi] = f(2 * Pi - x)
            float y = periodX > 3 * PiOver2 ? TwoPi - periodX : periodX > Pi ? periodX - Pi : periodX > PiOver2 ? Pi - periodX : periodX;

            //Using a fifth degree numerator and denominator.
            //This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
            //TODO: FMA could help here, primarily in terms of precision.
            var numerator = ((((-0.003436308368583229f * y + 0.021317031205957775f) * y + 0.06955843390178032f) * y - 0.4578088075324152f) * y - 0.15082367674208508f) * y + 1f;
            var denominator = ((((-0.00007650398834677185f * y + 0.0007451378206294365f) * y - 0.00585321045829395f) * y + 0.04219116713777847f) * y - 0.15082367538305258f) * y + 1f;
            var result = numerator / denominator;
            return periodX > PiOver2 && periodX < 3 * PiOver2 ? -result : result;

        }
        /// <summary>
        /// Computes an approximation of sine. Maximum error a little below 5e-7 for the interval -2 * Pi to 2 * Pi. Values further from the interval near zero have gracefully degrading error.
        /// </summary>
        /// <param name="x">Value to take the sine of.</param>
        /// <returns>Approximate sine of the input value.</returns>
        public static float Sin(float x)
        {
            //Similar to cos, use a rational approximation for the region of sin from [0, pi/2]. Use symmetry to cover the rest.
            //This has its own implementation rather than just calling into Cos because we want maximum fidelity near 0.
            var periodCount = x * (float)(0.5 / Math.PI);
            var periodFraction = periodCount - MathF.Floor(periodCount); //This is a source of error as you get away from 0.
            var periodX = periodFraction * TwoPi;

            //[0, pi/2] = f(x)
            //(pi/2, pi] = f(pi - x)
            //(pi, 3/2 * pi] = -f(x - pi)
            //(3/2 * pi, 2*pi] = -f(2 * pi - x)
            float y = periodX > 3 * PiOver2 ? TwoPi - periodX : periodX > Pi ? periodX - Pi : periodX > PiOver2 ? Pi - periodX : periodX;

            //Using a fifth degree numerator and denominator.
            //This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
            //TODO: FMA could help here, primarily in terms of precision.
            var numerator = ((((0.0040507708755727605f * y - 0.006685815219853882f) * y - 0.13993701695343166f) * y + 0.06174562337697123f) * y + 1.00000000151466040f) * y;
            var denominator = ((((0.00009018370615921334f * y + 0.0001700784176413186f) * y + 0.003606014457152456f) * y + 0.02672943625500751f) * y + 0.061745651499203795f) * y + 1f;
            var result = numerator / denominator;
            return periodX > Pi ? -result : result;
        }

        /// <summary>
        /// Computes an approximation of arccos. Inputs outside of [-1, 1] are clamped. Maximum error less than 5.17e-07.
        /// </summary>
        /// <param name="x">Input value to the arccos function.</param>
        /// <returns>Result of the arccos function.</returns>
        public static float Acos(float x)
        {
            var negativeInput = x < 0;
            x = MathF.Min(1f, MathF.Abs(x));
            //Rational approximation (scaling sqrt(1-x)) over [0, 1], use symmetry for the rest. TODO: FMA would help with precision.
            var numerator = MathF.Sqrt(1f - x) * (62.95741097600742f + x * (69.6550664543659f + x * (17.54512349463405f + x * 0.6022076120669532f)));
            var denominator = 40.07993264439811f + x * (49.81949855726789f + x * (15.703851745284796f + x));
            var result = numerator / denominator;
            return negativeInput ? Pi - result : result;
        }

        /// <summary>
        /// Computes an approximation of cosine. Maximum error a little below 8e-7 for the interval -2 * Pi to 2 * Pi. Values further from the interval near zero have gracefully degrading error.
        /// </summary>
        /// <param name="x">Values to take the cosine of.</param>
        /// <returns>Approximate cosine of the input values.</returns>
        public static Vector<float> Cos(Vector<float> x)
        {
            //Rational approximation over [0, pi/2], use symmetry for the rest.
            var periodCount = x * (float)(0.5 / Math.PI);
            var periodFraction = periodCount - Vector.Floor(periodCount); //This is a source of error as you get away from 0.
            var twoPi = new Vector<float>(TwoPi);
            var periodX = periodFraction * twoPi;

            //[0, pi/2] = f(x)
            //(pi/2, pi] = -f(Pi - x)
            //(pi, 3 * pi / 2] = -f(x - Pi)
            //(3*pi/2, 2*pi] = f(2 * Pi - x)
            Vector<float> y;
            var piOver2 = new Vector<float>(PiOver2);
            var pi = new Vector<float>(Pi);
            var pi3Over2 = new Vector<float>(3 * PiOver2);
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, piOver2), pi - periodX, periodX);
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, pi), periodX - pi, y);
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, pi3Over2), new Vector<float>(TwoPi) - periodX, y);

            //Using a fifth degree numerator and denominator.
            //This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
            //TODO: FMA could help here, primarily in terms of precision.
            //var y2 = y * y;
            //var y3 = y2 * y;
            //var y4 = y2 * y2;
            //var y5 = y3 * y2;
            //var numerator = Vector<float>.One - 0.15082367674208508f * y - 0.4578088075324152f * y2 + 0.06955843390178032f * y3 + 0.021317031205957775f * y4 - 0.003436308368583229f * y5;
            //var denominator = Vector<float>.One - 0.15082367538305258f * y + 0.04219116713777847f * y2 - 0.00585321045829395f * y3 + 0.0007451378206294365f * y4 - 0.00007650398834677185f * y5;
            var numerator = ((((new Vector<float>(-0.003436308368583229f) * y + new Vector<float>(0.021317031205957775f)) * y + new Vector<float>(0.06955843390178032f)) * y - new Vector<float>(0.4578088075324152f)) * y - new Vector<float>(0.15082367674208508f)) * y + Vector<float>.One;
            var denominator = ((((new Vector<float>(-0.00007650398834677185f) * y + new Vector<float>(0.0007451378206294365f)) * y - new Vector<float>(0.00585321045829395f)) * y + new Vector<float>(0.04219116713777847f)) * y - new Vector<float>(0.15082367538305258f)) * y + Vector<float>.One;
            var result = numerator / denominator;
            return Vector.ConditionalSelect(Vector.BitwiseAnd(Vector.GreaterThan(periodX, piOver2), Vector.LessThan(periodX, pi3Over2)), -result, result);
        }
        /// <summary>
        /// Computes an approximation of sine. Maximum error a little below 5e-7 for the interval -2 * Pi to 2 * Pi. Values further from the interval near zero have gracefully degrading error.
        /// </summary>
        /// <param name="x">Value to take the sine of.</param>
        /// <returns>Approximate sine of the input value.</returns>
        public static Vector<float> Sin(Vector<float> x)
        {
            //Similar to cos, use a rational approximation for the region of sin from [0, pi/2]. Use symmetry to cover the rest.
            //This has its own implementation rather than just calling into Cos because we want maximum fidelity near 0.
            var periodCount = x * (float)(0.5 / Math.PI);
            var periodFraction = periodCount - Vector.Floor(periodCount); //This is a source of error as you get away from 0.
            var twoPi = new Vector<float>(TwoPi);
            var periodX = periodFraction * twoPi;
            //[0, pi/2] = f(x)
            //(pi/2, pi] = f(pi - x)
            //(pi, 3/2 * pi] = -f(x - pi)
            //(3/2 * pi, 2*pi] = -f(2 * pi - x)
            Vector<float> y;
            var pi = new Vector<float>(Pi);
            var piOver2 = new Vector<float>(PiOver2);
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, piOver2), pi - periodX, periodX);
            var inSecondHalf = Vector.GreaterThan(periodX, pi);
            y = Vector.ConditionalSelect(inSecondHalf, periodX - pi, y);
            y = Vector.ConditionalSelect(Vector.GreaterThan(periodX, new Vector<float>(3 * PiOver2)), twoPi - periodX, y);

            //Using a fifth degree numerator and denominator.
            //This will be precise beyond a single's useful representation most of the time, but we're not *that* worried about performance here.
            //TODO: FMA could help here, primarily in terms of precision.
            //var y2 = y * y;
            //var y3 = y2 * y;
            //var y4 = y2 * y2;
            //var y5 = y3 * y2;
            //var numerator = 1.0000000015146604f * y + 0.06174562337697123f * y2 - 0.13993701695343166f * y3 - 0.006685815219853882f * y4 + 0.0040507708755727605f * y5;
            //var denominator = Vector<float>.One + 0.061745651499203795f * y + 0.02672943625500751f * y2 + 0.003606014457152456f * y3 + 0.0001700784176413186f * y4 + 0.00009018370615921334f * y5;
            var numerator = ((((0.0040507708755727605f * y - new Vector<float>(0.006685815219853882f)) * y - new Vector<float>(0.13993701695343166f)) * y + new Vector<float>(0.06174562337697123f)) * y + new Vector<float>(1.00000000151466040f)) * y;
            var denominator = ((((new Vector<float>(0.00009018370615921334f) * y + new Vector<float>(0.0001700784176413186f)) * y + new Vector<float>(0.003606014457152456f)) * y + new Vector<float>(0.02672943625500751f)) * y + new Vector<float>(0.061745651499203795f)) * y + Vector<float>.One;
            var result = numerator / denominator;
            return Vector.ConditionalSelect(inSecondHalf, -result, result);
        }

        /// <summary>
        /// Computes an approximation of arccos. Inputs outside of [-1, 1] are clamped. Maximum error less than 5.17e-07.
        /// </summary>
        /// <param name="x">Input value to the arccos function.</param>
        /// <returns>Result of the arccos function.</returns>
        public static Vector<float> Acos(Vector<float> x)
        {
            var negativeInput = Vector.LessThan(x, Vector<float>.Zero);
            x = Vector.Min(Vector<float>.One, Vector.Abs(x));
            //Rational approximation (scaling sqrt(1-x)) over [0, 1], use symmetry for the rest. TODO: FMA would help with precision.
            var numerator = Vector.SquareRoot(Vector<float>.One - x) * (new Vector<float>(62.95741097600742f) + x * (new Vector<float>(69.6550664543659f) + x * (new Vector<float>(17.54512349463405f) + x * 0.6022076120669532f)));
            var denominator = new Vector<float>(40.07993264439811f) + x * (new Vector<float>(49.81949855726789f) + x * (new Vector<float>(15.703851745284796f) + x));
            var result = numerator / denominator;
            return Vector.ConditionalSelect(negativeInput, new Vector<float>(Pi) - result, result);
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
            var half = new Vector<float>(0.5f);
            var x = (b - a) * new Vector<float>(1f / TwoPi) + half;
            difference = (x - Vector.Floor(x) - half) * new Vector<float>(TwoPi);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<float> FastReciprocal(Vector<float> v)
        {
            if (Avx.IsSupported && Vector<float>.Count == 8)
            {
                return Avx.Reciprocal(v.AsVector256()).AsVector();
            }
            else if (Sse.IsSupported && Vector<float>.Count == 4)
            {
                return Sse.Reciprocal(v.AsVector128()).AsVector();
            }
            else
            {
                return Vector<float>.One / v;
            }
            //TODO: Arm!
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector<float> FastReciprocalSquareRoot(Vector<float> v)
        {
            if (Avx.IsSupported && Vector<float>.Count == 8)
            {
                return Avx.ReciprocalSqrt(v.AsVector256()).AsVector();
            }
            else if (Sse.IsSupported && Vector<float>.Count == 4)
            {
                return Sse.ReciprocalSqrt(v.AsVector128()).AsVector();
            }
            else
            {
                return Vector<float>.One / Vector.SquareRoot(v);
            }
            //TODO: Arm!
        }
    }
}
