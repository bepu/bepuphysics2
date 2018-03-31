using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    public struct Vector3Wide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3Wide(ref Vector<float> s)
        {
            X = s;
            Y = s;
            Z = s;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFrom(ref Vector3 source, out Vector3Wide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X);
            broadcasted.Y = new Vector<float>(source.Y);
            broadcasted.Z = new Vector<float>(source.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
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
        public static void Add(ref Vector3Wide v, ref Vector<float> s, out Vector3Wide result)
        {
            result.X = v.X + s;
            result.Y = v.Y + s;
            result.Z = v.Z + s;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
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
        public static void Subtract(ref Vector3Wide v, ref Vector<float> s, out Vector3Wide result)
        {
            result.X = v.X - s;
            result.Y = v.Y - s;
            result.Z = v.Z - s;
        }
        /// <summary>
        /// Finds the result of subtracting the components of a vector from a scalar.
        /// </summary>
        /// <param name="v">Vector to subtract from the scalar.</param>
        /// <param name="s">Scalar to subtract from.</param>
        /// <param name="result">Vector with components equal the input vector subtracted from the input scalar.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Vector<float> s, ref Vector3Wide v, out Vector3Wide result)
        {
            result.X = s - v.X;
            result.Y = s - v.Y;
            result.Z = s - v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(ref Vector3Wide a, ref Vector3Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// <summary>
        /// Computes the per-component minimum between a scalar value and the components of a vector.
        /// </summary>
        /// <param name="s">Scalar to compare to each vector component.</param>
        /// <param name="v">Vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the smaller of the scalar value and the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Min(ref Vector<float> s, ref Vector3Wide v, out Vector3Wide result)
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
        public static void Min(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = Vector.Min(a.X, b.X);
            result.Y = Vector.Min(a.Y, b.Y);
            result.Z = Vector.Min(a.Z, b.Z);
        }
        /// <summary>
        /// Computes the per-component maximum between a scalar value and the components of a vector.
        /// </summary>
        /// <param name="s">Scalar to compare to each vector component.</param>
        /// <param name="v">Vector whose components will be compared.</param>
        /// <param name="result">Vector with components matching the larger of the scalar value and the input vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Max(ref Vector<float> s, ref Vector3Wide v, out Vector3Wide result)
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
        public static void Max(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = Vector.Max(a.X, b.X);
            result.Y = Vector.Max(a.Y, b.Y);
            result.Z = Vector.Max(a.Z, b.Z);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Vector3Wide vector, ref Vector<float> scalar, out Vector3Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Abs(ref Vector3Wide vector, out Vector3Wide result)
        {
            result.X = Vector.Abs(vector.X);
            result.Y = Vector.Abs(vector.Y);
            result.Z = Vector.Abs(vector.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref Vector3Wide v, out Vector3Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide Negate(ref Vector3Wide v)
        {
            v.X = -v.X;
            v.Y = -v.Y;
            v.Z = -v.Z;
            return ref v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CrossWithoutOverlap(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            //This will fail if the result reference is actually a or b! 
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            CrossWithoutOverlap(ref a, ref b, out var temp);
            result = temp;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LengthSquared(ref Vector3Wide v, out Vector<float> lengthSquared)
        {
            lengthSquared = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(ref Vector3Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Distance(ref Vector3Wide a, ref Vector3Wide b, out Vector<float> distance)
        {
            Subtract(ref b, ref a, out var offset);
            Length(ref offset, out distance);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Normalize(ref Vector3Wide v, out Vector3Wide result)
        {
            Length(ref v, out var length);
            var scale = Vector<float>.One / length;
            Scale(ref v, ref scale, out result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(ref Vector<int> condition, ref Vector3Wide left, ref Vector3Wide right, out Vector3Wide result)
        {
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
        }

        /// <summary>
        /// Pulls one lane out of the wide representation.
        /// </summary>
        /// <param name="wide">Source of the lane.</param>
        /// <param name="laneIndex">Index of the lane within the wide representation to read.</param>
        /// <param name="narrow">Non-SIMD type to store the lane in.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLane(ref Vector3Wide wide, int laneIndex, out Vector3 narrow)
        {
            ref var start = ref Unsafe.Add(ref Unsafe.As<Vector<float>, float>(ref wide.X), laneIndex);
            narrow.X = start;
            narrow.Y = Unsafe.Add(ref start, Vector<float>.Count);
            narrow.Z = Unsafe.Add(ref start, 2 * Vector<float>.Count);
        }
        
        /// <summary>
        /// Gathers values from a vector and places them into the first indices of the target vector.
        /// </summary>
        /// <param name="source">Vector to copy values from.</param>
        /// <param name="targetSlot">Wide vectorto place values into.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GatherSlot(ref Vector3 source, ref Vector3Wide targetSlot)
        {
            GatherScatter.GetFirst(ref targetSlot.X) = source.X;
            GatherScatter.GetFirst(ref targetSlot.Y) = source.Y;
            GatherScatter.GetFirst(ref targetSlot.Z) = source.Z;
        }

        public override string ToString()
        {
            return $"<{X}, {Y}, {Z}>";
        }
    }
}
