using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Shared miscellaneous helper functions.
    /// </summary>
    public static class Helpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Swap<T>(ref T a, ref T b)
        {
            var temp = a;
            a = b;
            b = temp;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildOrthonormalBasis(in Vector3Wide normal, out Vector3Wide t1, out Vector3Wide t2)
        {
            //This could probably be improved.
            var sign = Vector.ConditionalSelect(Vector.LessThan(normal.Z, Vector<float>.Zero), -Vector<float>.One, Vector<float>.One);

            //This has a discontinuity at z==0. Raw frisvad has only one discontinuity, though that region is more unpredictable than the revised version.
            var scale = -Vector<float>.One / (sign + normal.Z);
            t1.X = normal.X * normal.Y * scale;
            t1.Y = sign + normal.Y * normal.Y * scale;
            t1.Z = -normal.Y;

            t2.X = Vector<float>.One + sign * normal.X * normal.X * scale;
            t2.Y = sign * t1.X;
            t2.Z = -sign * normal.X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindPerpendicular(in Vector3Wide normal, out Vector3Wide perpendicular)
        {
            var sign = Vector.ConditionalSelect(Vector.LessThan(normal.Z, Vector<float>.Zero), -Vector<float>.One, Vector<float>.One);

            var scale = -Vector<float>.One / (sign + normal.Z);
            perpendicular.X = normal.X * normal.Y * scale;
            perpendicular.Y = sign + normal.Y * normal.Y * scale;
            perpendicular.Z = -normal.Y;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildOrthonormalBasis(in Vector3 normal, out Vector3 t1, out Vector3 t2)
        {
            var sign = normal.Z < 0 ? -1f : 1f;

            var scale = -1f / (sign + normal.Z);
            t1.X = normal.X * normal.Y * scale;
            t1.Y = sign + normal.Y * normal.Y * scale;
            t1.Z = -normal.Y;

            t2.X = 1f + sign * normal.X * normal.X * scale;
            t2.Y = sign * t1.X;
            t2.Z = -sign * normal.X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FillVectorWithLaneIndices(out Vector<int> indices)
        {
            Unsafe.SkipInit(out indices);
            ref var start = ref Unsafe.As<Vector<int>, int>(ref indices);
            start = 0;
            for (int i = 1; i < Vector<int>.Count; ++i)
            {
                Unsafe.Add(ref start, i) = i;
            }
        }
    }
}
