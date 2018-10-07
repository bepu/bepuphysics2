using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace BepuUtilities
{
    /// <summary>
    /// Provides XNA-like axis-aligned bounding box functionality.
    /// </summary>
    //NOTE: The explicit size avoids https://github.com/dotnet/coreclr/issues/12950
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
        /// Determines if a bounding box intersects another bounding box.
        /// </summary>
        /// <param name="a">First bounding box to test.</param>
        /// <param name="b">Second bounding box to test.</param>
        /// <returns>Whether the bounding boxes intersected.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Intersects(in BoundingBox a, in BoundingBox b)
        {
            return Intersects(a.Min, a.Max, b.Min, b.Max);
        }
        //TODO: At some point in the past, intersection was found to be faster with non-short circuiting operators.
        //While that does make some sense (the branches aren't really valuable relative to their cost), it's still questionable enough that it should be reevaluated on a modern compiler. 
        /// <summary>
        /// Determines if a bounding box intersects another bounding box.
        /// </summary>
        /// <param name="a">First bounding box to test.</param>
        /// <param name="b">Second bounding box to test.</param>
        /// <returns>Whether the bounding boxes intersected.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Intersects(in Vector3 minA, in Vector3 maxA, in Vector3 minB, in Vector3 maxB)
        {
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
        public static void CreateMerged(in Vector3 minA, in Vector3 maxA, in Vector3 minB, in Vector3 maxB, out Vector3 min, out Vector3 max)
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
        public static BoundingBox CreateFromPoints(IList<Vector3> points)
        {
            BoundingBox aabb;
            if (points.Count == 0)
                throw new Exception("Cannot construct a bounding box from an empty list.");
            aabb.Min = points[0];
            aabb.Max = aabb.Min;
            for (int i = points.Count - 1; i >= 1; i--)
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
            return $"({Min.ToString()}, {Max.ToString()})";
        }

    }
}
