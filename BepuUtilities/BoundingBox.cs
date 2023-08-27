using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BepuUtilities
{
    /// <summary>
    /// Represents a bounding box as two <see cref="Vector4"/> values to to avoid complexity associated with a <see cref="Vector3"/>'s empty SIMD lane.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct BoundingBox4
    {
        /// <summary>
        /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box. W lane is undefined.
        /// </summary>
        [FieldOffset(0)]
        public Vector4 Min;

        /// <summary>
        /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box. W lane is undefined.
        /// </summary>
        [FieldOffset(16)]
        public Vector4 Max;


        /// <summary>
        /// Creates a string representation of the bounding box.
        /// </summary>
        /// <returns>String representation of the bounding box.</returns>
        public override string ToString()
        {
            return $"({Unsafe.As<Vector4, Vector3>(ref Min)}, {Unsafe.As<Vector4, Vector3>(ref Max)})";
        }

    }

    /// <summary>
    /// Provides simple axis-aligned bounding box functionality.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct BoundingBox
    {
        /// <summary>
        /// Location with the lowest X, Y, and Z coordinates in the axis-aligned bounding box.
        /// </summary>
        [FieldOffset(0)]
        public Vector3 Min;

        /// <summary>
        /// Location with the highest X, Y, and Z coordinates in the axis-aligned bounding box.
        /// </summary>
        [FieldOffset(16)]
        public Vector3 Max;

        /// <summary>
        /// Constructs a bounding box from the specified minimum and maximum.
        /// </summary>
        /// <param name="min">Location with the lowest X, Y, and Z coordinates contained by the axis-aligned bounding box.</param>
        /// <param name="max">Location with the highest X, Y, and Z coordinates contained by the axis-aligned bounding box.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingBox(Vector3 min, Vector3 max)
        {
            this.Min = min;
            this.Max = max;
        }

        /// <summary>
        /// Checks if two structures with memory layouts equivalent to the <see cref="BoundingBox"/> intersect.
        /// The referenced values must not be in unpinned managed memory.
        /// </summary>
        /// <param name="boundingBoxA">First bounding box to compare.</param>
        /// <param name="boundingBoxB">Second bounding box to compare.</param>
        /// <returns>True if the children overlap, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool IntersectsUnsafe<TA, TB>(in TA boundingBoxA, in TB boundingBoxB) where TA : unmanaged where TB : unmanaged
        {
            //This is a weird function. We're directly interpreting the memory of an incoming type as a vector where we assume the min/max layout matches the BoundingBox.
            //Happens to be convenient!
            Debug.Assert(Unsafe.SizeOf<TA>() == 32 && Unsafe.SizeOf<TB>() == 32);
            //AVX codepath is not helpful in my tests.
            //if (Vector256.IsHardwareAccelerated && Avx.IsSupported)
            //{
            //    var a = Vector256.LoadUnsafe(ref Unsafe.As<TA, float>(ref Unsafe.AsRef(childA)));
            //    var b = Vector256.LoadUnsafe(ref Unsafe.As<TB, float>(ref Unsafe.AsRef(childB)));
            //    var min = Avx.Permute2x128(a, b, (0) | (2 << 4)); //(aMin, aMax) (bMin, bMax) -> (aMin, bMin)
            //    var max = Avx.Permute2x128(a, b, (3) | (1 << 4)); //(aMin, aMax) (bMin, bMax) -> (bMax, aMax)
            //    var noIntersection = Vector256.LessThan(max, min);
            //    return (Vector256.ExtractMostSignificantBits(noIntersection) & 0b1110111) == 0;
            //}
            //else
            if (Vector128.IsHardwareAccelerated)
            {
                //THIS IS A POTENTIAL GC HOLE IF CHILDREN ARE PASSED FROM UNPINNED MANAGED MEMORY
                ref var a = ref Unsafe.As<TA, float>(ref Unsafe.AsRef(boundingBoxA));
                ref var b = ref Unsafe.As<TB, float>(ref Unsafe.AsRef(boundingBoxB));
                var aMin = Vector128.LoadUnsafe(ref a);
                var aMax = Vector128.LoadUnsafe(ref Unsafe.Add(ref a, 4));
                var bMin = Vector128.LoadUnsafe(ref b);
                var bMax = Vector128.LoadUnsafe(ref Unsafe.Add(ref b, 4));
                var noIntersectionOnAxes = Vector128.LessThan(aMax, bMin) | Vector128.LessThan(bMax, aMin);
                return (Vector128.ExtractMostSignificantBits(noIntersectionOnAxes) & 0b111) == 0;
            }
            else
            {
                var a = (float*)Unsafe.AsPointer(ref Unsafe.AsRef(boundingBoxA));
                var b = (float*)Unsafe.AsPointer(ref Unsafe.AsRef(boundingBoxB));
                return a[4] >= b[0] & a[5] >= b[1] & a[6] >= b[2] &
                       b[4] >= a[0] & b[5] >= a[1] & b[6] >= a[2];
            }
        }

        /// <summary>
        /// Determines if a bounding box intersects another bounding box.
        /// </summary>
        /// <param name="a">First bounding box to test.</param>
        /// <param name="b">Second bounding box to test.</param>
        /// <returns>Whether the bounding boxes intersected.</returns>
        /// <remarks>When possible, prefer using the <see cref="IntersectsUnsafe{TA, TB}(in TA, in TB)"/> variant for slightly better performance.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool Intersects(BoundingBox a, BoundingBox b)
        {
            return IntersectsUnsafe(a, b);
        }

        /// <summary>
        /// Determines if a bounding box intersects another bounding box.
        /// </summary>
        /// <param name="minA">Minimum bounds of bounding box A.</param>
        /// <param name="maxA">Maximum bounds of bounding box A.</param>
        /// <param name="minB">Minimum bounds of bounding box B.</param>
        /// <param name="maxB">Maximum bounds of bounding box B.</param>
        /// <returns>Whether the bounding boxes intersected.</returns>
        /// <remarks>When possible, prefer using the <see cref="IntersectsUnsafe{TA, TB}(in TA, in TB)"/> variant for slightly better performance.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Intersects(Vector3 minA, Vector3 maxA, Vector3 minB, Vector3 maxB)
        {
            if (Vector128.IsHardwareAccelerated)
            {
                var noIntersectionOnAxes = Vector128.LessThan(maxA.AsVector128(), minB.AsVector128()) | Vector128.LessThan(maxB.AsVector128(), minA.AsVector128());
                return (Vector128.ExtractMostSignificantBits(noIntersectionOnAxes) & 0b111) == 0;
            }
            return maxA.X >= minB.X & maxA.Y >= minB.Y & maxA.Z >= minB.Z &
                   maxB.X >= minA.X & maxB.Y >= minA.Y & maxB.Z >= minA.Z;
        }

        /// <summary>
        /// Computes the volume of the bounding box.
        /// </summary>
        /// <param name="box">Bounding box to measure.</param>
        /// <returns>Volume of the bounding box.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe float ComputeVolume(ref BoundingBox box)
        {
            var diagonal = (box.Max - box.Min);
            return diagonal.X * diagonal.Y * diagonal.Z;
        }


        /// <summary>
        /// Computes a bounding box which contains two other bounding boxes.
        /// </summary>
        /// <param name="minA">Minimum of the first bounding box to merge.</param>
        /// <param name="maxA">Maximum of the first bounding box to merge.</param>
        /// <param name="minB">Minimum of the second bounding box to merge.</param>
        /// <param name="maxB">Maximum of the second bounding box to merge.</param>
        /// <param name="min">Minimum of the merged bounding box.</param>
        /// <param name="max">Maximum of the merged bounding box.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateMerged(Vector3 minA, Vector3 maxA, Vector3 minB, Vector3 maxB, out Vector3 min, out Vector3 max)
        {
            min = Vector3.Min(minA, minB);
            max = Vector3.Max(maxA, maxB);
        }

        /// <summary>
        /// Computes a bounding box which contains two other bounding boxes.
        /// </summary>
        /// <param name="a">First bounding box to contain.</param>
        /// <param name="b">Second bounding box to contain.</param>
        /// <param name="merged">Bounding box to contain both input boxes.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateMerged(in BoundingBox a, in BoundingBox b, out BoundingBox merged)
        {
            CreateMerged(a.Min, a.Max, b.Min, b.Max, out merged.Min, out merged.Max);
        }

        /// <summary>
        /// Merges two structures with memory layouts equivalent to the <see cref="BoundingBox"/>.
        /// The referenced values must not be in unpinned managed memory.
        /// Any data in the empty slots is preserved.
        /// </summary>
        /// <param name="boundingBoxA">First bounding box to compare.</param>
        /// <param name="boundingBoxB">Second bounding box to compare.</param>
        /// <param name="merged">Merged bounding box.</param>
        /// <typeparam name="TA">Type of the first bounding box-like parameter.</typeparam>
        /// <typeparam name="TB">Type of the second bounding box-like parameter.</typeparam>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void CreateMergedUnsafeWithPreservation<TA, TB>(in TA boundingBoxA, in TB boundingBoxB, out TA merged) where TA : unmanaged where TB : unmanaged
        {
            if (Vector128.IsHardwareAccelerated)
            {
                Unsafe.SkipInit(out merged);
                ref var resultMin = ref Unsafe.As<TA, Vector128<float>>(ref merged);
                ref var resultMax = ref Unsafe.Add(ref Unsafe.As<TA, Vector128<float>>(ref merged), 1);
                var min = Vector128.Min(
                    Unsafe.As<TA, Vector128<float>>(ref Unsafe.AsRef(boundingBoxA)),
                    Unsafe.As<TB, Vector128<float>>(ref Unsafe.AsRef(boundingBoxB)));
                var max = Vector128.Max(
                    Unsafe.Add(ref Unsafe.As<TA, Vector128<float>>(ref Unsafe.AsRef(boundingBoxA)), 1),
                    Unsafe.Add(ref Unsafe.As<TB, Vector128<float>>(ref Unsafe.AsRef(boundingBoxB)), 1));
                if (Sse41.IsSupported)
                {
                    resultMin = Sse41.Blend(min, resultMin, 0b1000);
                    resultMax = Sse41.Blend(max, resultMax, 0b1000);
                }
                else
                {
                    var mask = Vector128.Create(-1, -1, -1, 0).As<int, float>();
                    resultMin = Vector128.ConditionalSelect(mask, min, resultMin);
                    resultMax = Vector128.ConditionalSelect(mask, max, resultMax);
                }
            }
            else
            {
                ref var a = ref Unsafe.As<TA, BoundingBox>(ref Unsafe.AsRef(boundingBoxA));
                ref var b = ref Unsafe.As<TB, BoundingBox>(ref Unsafe.AsRef(boundingBoxB));
                Unsafe.SkipInit(out merged);
                ref var result = ref Unsafe.As<TA, BoundingBox>(ref Unsafe.AsRef(merged));
                result.Min = Vector3.Min(a.Min, b.Min);
                result.Max = Vector3.Max(a.Max, b.Max);
            }
        }
        /// <summary>
        /// Merges two structures with memory layouts equivalent to the <see cref="BoundingBox"/>.
        /// The referenced values must not be in unpinned managed memory.
        /// Any data in the empty slots is not preserved.
        /// </summary>
        /// <param name="boundingBoxA">First bounding box to compare.</param>
        /// <param name="boundingBoxB">Second bounding box to compare.</param>
        /// <param name="merged">Merged bounding box.</param>
        /// <typeparam name="TA">Type of the first bounding box-like parameter.</typeparam>
        /// <typeparam name="TB">Type of the second bounding box-like parameter.</typeparam>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void CreateMergedUnsafe<TA, TB>(in TA boundingBoxA, in TB boundingBoxB, out TA merged) where TA : unmanaged where TB : unmanaged
        {
            if (Vector128.IsHardwareAccelerated)
            {
                Unsafe.SkipInit(out merged);
                ref var resultMin = ref Unsafe.As<TA, Vector128<float>>(ref merged);
                ref var resultMax = ref Unsafe.Add(ref Unsafe.As<TA, Vector128<float>>(ref merged), 1);
                resultMin = Vector128.Min(
                    Unsafe.As<TA, Vector128<float>>(ref Unsafe.AsRef(boundingBoxA)),
                    Unsafe.As<TB, Vector128<float>>(ref Unsafe.AsRef(boundingBoxB)));
                resultMax = Vector128.Max(
                    Unsafe.Add(ref Unsafe.As<TA, Vector128<float>>(ref Unsafe.AsRef(boundingBoxA)), 1),
                    Unsafe.Add(ref Unsafe.As<TB, Vector128<float>>(ref Unsafe.AsRef(boundingBoxB)), 1));
            }
            else
            {
                ref var a = ref Unsafe.As<TA, BoundingBox>(ref Unsafe.AsRef(boundingBoxA));
                ref var b = ref Unsafe.As<TB, BoundingBox>(ref Unsafe.AsRef(boundingBoxB));
                Unsafe.SkipInit(out merged);
                ref var result = ref Unsafe.As<TA, BoundingBox>(ref Unsafe.AsRef(merged));
                result.Min = Vector3.Min(a.Min, b.Min);
                result.Max = Vector3.Max(a.Max, b.Max);
            }
        }

        /// <summary>
        /// Determines if a bounding box intersects a bounding sphere.
        /// </summary>
        /// <param name="boundingSphere">Sphere to test for intersection.</param>
        /// <returns>Whether the bounding shapes intersect.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(ref BoundingSphere boundingSphere)
        {
            var offset = boundingSphere.Center - Vector3.Min(Vector3.Max(boundingSphere.Center, Min), Max);
            return Vector3.Dot(offset, offset) <= boundingSphere.Radius * boundingSphere.Radius;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ContainmentType Contains(ref BoundingBox boundingBox)
        {
            if (Max.X < boundingBox.Min.X || Min.X > boundingBox.Max.X ||
                Max.Y < boundingBox.Min.Y || Min.Y > boundingBox.Max.Y ||
                Max.Z < boundingBox.Min.Z || Min.Z > boundingBox.Max.Z)
                return ContainmentType.Disjoint;
            //It is known to be at least intersecting. Is it contained?
            if (Min.X <= boundingBox.Min.X && Max.X >= boundingBox.Max.X &&
                Min.Y <= boundingBox.Min.Y && Max.Y >= boundingBox.Max.Y &&
                Min.Z <= boundingBox.Min.Z && Max.Z >= boundingBox.Max.Z)
                return ContainmentType.Contains;
            return ContainmentType.Intersects;
        }



        /// <summary>
        /// Creates the smallest possible bounding box that contains a list of points.
        /// </summary>
        /// <param name="points">Points to enclose with a bounding box.</param>
        /// <returns>Bounding box which contains the list of points.</returns>
        public static BoundingBox CreateFromPoints(ReadOnlySpan<Vector3> points)
        {
            BoundingBox aabb;
            if (points.Length == 0)
                throw new Exception("Cannot construct a bounding box from an empty list.");
            aabb.Min = points[0];
            aabb.Max = aabb.Min;
            for (int i = points.Length - 1; i >= 1; i--)
            {
                aabb.Min = Vector3.Min(points[i], aabb.Min);
                aabb.Max = Vector3.Max(points[i], aabb.Max);
            }
            return aabb;
        }




        /// <summary>
        /// Creates a bounding box from a bounding sphere.
        /// </summary>
        /// <param name="boundingSphere">Bounding sphere to be used to create the bounding box.</param>
        /// <param name="boundingBox">Bounding box created from the bounding sphere.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFromSphere(ref BoundingSphere boundingSphere, out BoundingBox boundingBox)
        {
            var radius = new Vector3(boundingSphere.Radius);
            boundingBox.Min = boundingSphere.Center - radius;
            boundingBox.Max = boundingSphere.Center + radius;
        }

        /// <summary>
        /// Creates a string representation of the bounding box.
        /// </summary>
        /// <returns>String representation of the bounding box.</returns>
        public override string ToString()
        {
            return $"({Min}, {Max})";
        }

    }
}
