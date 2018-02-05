using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuUtilities.Memory;
using System.Diagnostics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;

namespace BepuPhysics.Collidables
{
    public struct BoxWide
    {
        public Vector<float> HalfWidth;
        public Vector<float> HalfHeight;
        public Vector<float> HalfLength;
    }

    /// <summary>
    /// Collision shape representing a solid cuboid.
    /// </summary>
    public struct Box : IShape
    {
        public float HalfWidth;
        public float HalfHeight;
        public float HalfLength;

        /// <summary>
        /// Gets or sets the width of the box.
        /// </summary>
        public float Width { get { return HalfWidth * 2; } set { HalfWidth = value * 0.5f; } }
        /// <summary>
        /// Gets or sets the height of the box.
        /// </summary>
        public float Height { get { return HalfHeight * 2; } set { HalfHeight = value * 0.5f; } }
        /// <summary>
        /// Gets or sets the length of the box.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        public Box(float width, float height, float length)
        {
            HalfWidth = width * 0.5f;
            HalfHeight = height * 0.5f;
            HalfLength = length * 0.5f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Gather(ref Buffer<Box> shapes, ref Vector<int> shapeIndices, int count, out BoxWide capsules)
        {
            ref var halfWidthBase = ref Unsafe.As<Vector<float>, float>(ref capsules.HalfWidth);
            ref var halfHeightBase = ref Unsafe.As<Vector<float>, float>(ref capsules.HalfHeight);
            ref var halfLengthBase = ref Unsafe.As<Vector<float>, float>(ref capsules.HalfLength);
            ref var shapeIndicesBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            Debug.Assert(count <= Vector<float>.Count);
            for (int i = 0; i < count; ++i)
            {
                ref var shape = ref shapes[Unsafe.Add(ref shapeIndicesBase, i)];
                Unsafe.Add(ref halfWidthBase, i) = shape.HalfWidth;
                Unsafe.Add(ref halfHeightBase, i) = shape.HalfHeight;
                Unsafe.Add(ref halfLengthBase, i) = shape.HalfLength;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref QuaternionWide orientations,
            out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
            where TShape : struct, IShape
        {
            Gather(ref Unsafe.As<Buffer<TShape>, Buffer<Box>>(ref shapes), ref shapeIndices, count, out var boxes);
            Matrix3x3Wide.CreateFromQuaternion(ref orientations, out var basis);
            //Compute the extreme point along each axis in local space. We use the transposed basis as a source of local axes- the world axes tranformed into local space.
            Vector3Wide localX, localY, localZ;
            //TODO: There is likely some significantly better alternative to this in a variety of platform intrinsics. Not a big deal, though.
            localX.X = Vector.ConditionalSelect(Vector.LessThan(basis.X.X, Vector<float>.Zero), -boxes.HalfWidth, boxes.HalfWidth);
            localX.Y = Vector.ConditionalSelect(Vector.LessThan(basis.Y.X, Vector<float>.Zero), -boxes.HalfHeight, boxes.HalfHeight);
            localX.Z = Vector.ConditionalSelect(Vector.LessThan(basis.Z.X, Vector<float>.Zero), -boxes.HalfLength, boxes.HalfLength);
            localY.X = Vector.ConditionalSelect(Vector.LessThan(basis.X.Y, Vector<float>.Zero), -boxes.HalfWidth, boxes.HalfWidth);
            localY.Y = Vector.ConditionalSelect(Vector.LessThan(basis.Y.Y, Vector<float>.Zero), -boxes.HalfHeight, boxes.HalfHeight);
            localY.Z = Vector.ConditionalSelect(Vector.LessThan(basis.Z.Y, Vector<float>.Zero), -boxes.HalfLength, boxes.HalfLength);
            localZ.X = Vector.ConditionalSelect(Vector.LessThan(basis.X.Z, Vector<float>.Zero), -boxes.HalfWidth, boxes.HalfWidth);
            localZ.Y = Vector.ConditionalSelect(Vector.LessThan(basis.Y.Z, Vector<float>.Zero), -boxes.HalfHeight, boxes.HalfHeight);
            localZ.Z = Vector.ConditionalSelect(Vector.LessThan(basis.Z.Z, Vector<float>.Zero), -boxes.HalfLength, boxes.HalfLength);

            //Now move those extreme points into world space. Note that we're only interested in the extent for a given point along the axis it was created for.
            max.X = Vector.Abs(localX.X * basis.X.X + localX.Y * basis.Y.X + localX.Z * basis.Z.X);
            max.Y = Vector.Abs(localY.X * basis.X.Y + localY.Y * basis.Y.Y + localY.Z * basis.Z.Y);
            max.Z = Vector.Abs(localZ.X * basis.X.Z + localZ.Y * basis.Y.Z + localZ.Z * basis.Z.Z);

            Vector3Wide.Negate(ref max, out min);

            maximumRadius = Vector.SquareRoot(boxes.HalfWidth * boxes.HalfWidth + boxes.HalfHeight * boxes.HalfHeight + boxes.HalfLength * boxes.HalfLength);
            maximumAngularExpansion = maximumRadius - Vector.Min(boxes.HalfLength, Vector.Min(boxes.HalfHeight, boxes.HalfLength));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(ref orientation, out var basis);
            //Compute the extreme point along each axis in local space. We use the transposed basis as a source of local axes- the world axes tranformed into local space.
            Vector3 localX, localY, localZ;
            //TODO: There is likely some significantly better alternative to this in a variety of platform intrinsics. Not a big deal, though.
            localX.X = basis.X.X < 0 ? -HalfWidth : HalfWidth;
            localX.Y = basis.Y.X < 0 ? -HalfHeight : HalfHeight;
            localX.Z = basis.Z.X < 0 ? -HalfLength : HalfLength;
            localY.X = basis.X.Y < 0 ? -HalfWidth : HalfWidth;
            localY.Y = basis.Y.Y < 0 ? -HalfHeight : HalfHeight;
            localY.Z = basis.Z.Y < 0 ? -HalfLength : HalfLength;
            localZ.X = basis.X.Z < 0 ? -HalfWidth : HalfWidth;
            localZ.Y = basis.Y.Z < 0 ? -HalfHeight : HalfHeight;
            localZ.Z = basis.Z.Z < 0 ? -HalfLength : HalfLength;

            //Now move those extreme points into world space. Note that we're only interested in the extent for a given point along the axis it was created for.
            max.X = localX.X * basis.X.X + localX.Y * basis.Y.X + localX.Z * basis.Z.X;
            max.Y = localY.X * basis.X.Y + localY.Y * basis.Y.Y + localY.Z * basis.Z.Y;
            max.Z = localZ.X * basis.X.Z + localZ.Y * basis.Y.Z + localZ.Z * basis.Z.Z;
            max = Vector3.Abs(max);
            min = -max;
        }

        public bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, out float t, out Vector3 normal)
        {
            var offset = origin - pose.Position;
            Matrix3x3.CreateFromQuaternion(ref pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(ref offset, ref orientation, out var localOffset);
            Matrix3x3.TransformTranspose(ref direction, ref orientation, out var localDirection);  
            //Note that this division has two odd properties:
            //1) If the local direction has a near zero component, it is clamped to a nonzero but extremely small value. This is a hack, but it works reasonably well.
            //The idea is that any interval computed using such an inverse would be enormous. Those values will not be exactly accurate, but they will never appear as a result
            //because a parallel ray will never actually intersect the surface. The resulting intervals are practical approximations of the 'true' infinite intervals.
            //2) To compensate for the clamp and abs, we reintroduce the sign in the numerator. Note that it has the reverse sign since it will be applied to the offset to get the T value.
            var offsetToTScale = 
                new Vector3(localDirection.X < 0 ? 1 : -1, localDirection.Y < 0 ? 1 : -1, localDirection.Z < 0 ? 1 : -1) / Vector3.Max(new Vector3(1e-15f), Vector3.Abs(localDirection));

            //Compute impact times for each pair of planes in local space.
            var halfExtent = new Vector3(HalfWidth, HalfHeight, HalfLength);
            var negativeT = (localOffset - halfExtent) * offsetToTScale;
            var positiveT = (localOffset + halfExtent) * offsetToTScale;
            var entryT = Vector3.Min(negativeT, positiveT);
            var exitT = Vector3.Max(negativeT, positiveT);

            //In order for an impact to occur, the ray must enter all three slabs formed by the axis planes before exiting any of them.
            //In other words, the first exit must occur after the last entry.
            var earliestExit = exitT.X < exitT.Y ? exitT.X : exitT.Y;
            if (exitT.Z < earliestExit)
                earliestExit = exitT.Z;
            //The interval of ray-box intersection goes from latestEntry to earliestExit. If earliestExit is negative, then the ray is pointing away from the box.
            if (earliestExit < 0)
            {
                t = 0;
                normal = new Vector3();
                return false;
            }
            float latestEntry;
            if (entryT.X > entryT.Y)
            {
                if (entryT.X > entryT.Z)
                {
                    latestEntry = entryT.X;
                    normal = orientation.X;
                }
                else
                {
                    latestEntry = entryT.Z;
                    normal = orientation.Z;
                }
            }
            else
            {
                if (entryT.Y > entryT.Z)
                {
                    latestEntry = entryT.Y;
                    normal = orientation.Y;
                }
                else
                {
                    latestEntry = entryT.Z;
                    normal = orientation.Z;
                }
            }

            if (earliestExit < latestEntry)
            {
                //At no point is the ray in all three slabs at once.
                t = 0;
                normal = new Vector3();
                return false;
            }
            t = latestEntry < 0 ? 0 : latestEntry;
            //The normal should point away from the center of the box.
            if (Vector3.Dot(normal, localOffset) < 0)
            {
                normal = -normal;
            }
            return true;
        }

        public void ComputeLocalInverseInertia(float inverseMass, out Triangular3x3 localInverseInertia)
        {
            var x2 = HalfWidth * HalfWidth;
            var y2 = HalfHeight * HalfHeight;
            var z2 = HalfLength * HalfLength;
            localInverseInertia.M11 = inverseMass * 3 / (y2 + z2);
            localInverseInertia.M21 = 0;
            localInverseInertia.M22 = inverseMass * 3 / (x2 + z2);
            localInverseInertia.M31 = 0;
            localInverseInertia.M32 = 0;
            localInverseInertia.M33 = inverseMass * 3 / (x2 + y2);
        }

        /// <summary>
        /// Type id of box shapes.
        /// </summary>
        public const int Id = 2;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
