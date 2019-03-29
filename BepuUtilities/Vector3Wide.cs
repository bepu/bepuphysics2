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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(in Vector3Wide a, in Vector3Wide b, out Vector<float> result)
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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Vector3Wide vector, in Vector<float> scalar, out Vector3Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Abs(in Vector3Wide vector, out Vector3Wide result)
        {
            result.X = Vector.Abs(vector.X);
            result.Y = Vector.Abs(vector.Y);
            result.Z = Vector.Abs(vector.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(in Vector3Wide v, out Vector3Wide result)
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
        public static void ConditionallyNegate(in Vector<int> shouldNegate, ref Vector3Wide v)
        {
            v.X = Vector.ConditionalSelect(shouldNegate, -v.X, v.X);
            v.Y = Vector.ConditionalSelect(shouldNegate, -v.Y, v.Y);
            v.Z = Vector.ConditionalSelect(shouldNegate, -v.Z, v.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionallyNegate(in Vector<int> shouldNegate, in Vector3Wide v, out Vector3Wide negated)
        {
            negated.X = Vector.ConditionalSelect(shouldNegate, -v.X, v.X);
            negated.Y = Vector.ConditionalSelect(shouldNegate, -v.Y, v.Y);
            negated.Z = Vector.ConditionalSelect(shouldNegate, -v.Z, v.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CrossWithoutOverlap(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            //This will fail if the result reference is actually a or b! 
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            CrossWithoutOverlap(a, b, out var temp);
            result = temp;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LengthSquared(in Vector3Wide v, out Vector<float> lengthSquared)
        {
            lengthSquared = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(in Vector3Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Distance(in Vector3Wide a, in Vector3Wide b, out Vector<float> distance)
        {
            var x = b.X - a.X;
            var y = b.Y - a.Y;
            var z = b.Z - a.Z;
            distance = Vector.SquareRoot(x * x + y * y + z * z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DistanceSquared(in Vector3Wide a, in Vector3Wide b, out Vector<float> distanceSquared)
        {
            var x = b.X - a.X;
            var y = b.Y - a.Y;
            var z = b.Z - a.Z;
            distanceSquared = x * x + y * y + z * z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Normalize(in Vector3Wide v, out Vector3Wide result)
        {
            Length(v, out var length);
            var scale = Vector<float>.One / length;
            Scale(v, scale, out result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(in Vector<int> condition, in Vector3Wide left, in Vector3Wide right, out Vector3Wide result)
        {
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
        }

        /// <summary>
        /// Multiplies the components of one vector with another.
        /// </summary>
        /// <param name="a">First vector to multiply.</param>
        /// <param name="b">Second vector to multiply.</param>
        /// <param name="result">Result of the multiplication.</param>
        public static void Multiply(in Vector3Wide a, in Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X * b.X;
            result.Y = a.Y * b.Y;
            result.Z = a.Z * b.Z;
        }

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
