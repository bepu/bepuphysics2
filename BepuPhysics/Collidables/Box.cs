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
    /// <summary>
    /// Collision shape representing a solid cuboid.
    /// </summary>
    public struct Box : IConvexShape
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
        public void ComputeBounds(in Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var basis);
            var x = HalfWidth * basis.X;
            var y = HalfHeight * basis.Y;
            var z = HalfLength * basis.Z;
            max = Vector3.Abs(x) + Vector3.Abs(y) + Vector3.Abs(z);
            min = -max;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            maximumRadius = (float)Math.Sqrt(HalfWidth * HalfWidth + HalfHeight * HalfHeight + HalfLength * HalfLength);
            maximumAngularExpansion = maximumRadius - Vector4.Min(new Vector4(HalfLength), Vector4.Min(new Vector4(HalfHeight), new Vector4(HalfLength))).X;
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
        {
            var offset = origin - pose.Position;
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(offset, orientation, out var localOffset);
            Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);
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
            if (Vector3.Dot(normal, offset) < 0)
            {
                normal = -normal;
            }
            return true;
        }

        public void ComputeInertia(float mass, out BodyInertia inertia)
        {
            inertia.InverseMass = 1f / mass;
            var x2 = HalfWidth * HalfWidth;
            var y2 = HalfHeight * HalfHeight;
            var z2 = HalfLength * HalfLength;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass * 3 / (y2 + z2);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseMass * 3 / (x2 + z2);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseMass * 3 / (x2 + y2);
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<Box, BoxWide>(pool, initialCapacity);
        }

        /// <summary>
        /// Type id of box shapes.
        /// </summary>
        public const int Id = 2;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }


    public struct BoxWide : IShapeWide<Box>
    {
        public Vector<float> HalfWidth;
        public Vector<float> HalfHeight;
        public Vector<float> HalfLength;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Broadcast(ref Box shape)
        {
            HalfWidth = new Vector<float>(shape.HalfWidth);
            HalfHeight = new Vector<float>(shape.HalfHeight);
            HalfLength = new Vector<float>(shape.HalfLength);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref Box source)
        {
            Unsafe.As<Vector<float>, float>(ref HalfWidth) = source.HalfWidth;
            Unsafe.As<Vector<float>, float>(ref HalfHeight) = source.HalfHeight;
            Unsafe.As<Vector<float>, float>(ref HalfLength) = source.HalfLength;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            Matrix3x3Wide.CreateFromQuaternion(ref orientations, out var basis);
            max.X = Vector.Abs(HalfWidth * basis.X.X) + Vector.Abs(HalfHeight * basis.Y.X) + Vector.Abs(HalfLength * basis.Z.X);
            max.Y = Vector.Abs(HalfWidth * basis.X.Y) + Vector.Abs(HalfHeight * basis.Y.Y) + Vector.Abs(HalfLength * basis.Z.Y);
            max.Z = Vector.Abs(HalfWidth * basis.X.Z) + Vector.Abs(HalfHeight * basis.Y.Z) + Vector.Abs(HalfLength * basis.Z.Z);

            Vector3Wide.Negate(ref max, out min);

            maximumRadius = Vector.SquareRoot(HalfWidth * HalfWidth + HalfHeight * HalfHeight + HalfLength * HalfLength);
            maximumAngularExpansion = maximumRadius - Vector.Min(HalfLength, Vector.Min(HalfHeight, HalfLength));
        }
       
        public int MinimumWideRayCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return 2;
            }
        }

        public void RayTest(ref RigidPoses pose, ref RayWide ray, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            Vector3Wide.Subtract(ref ray.Origin, ref pose.Position, out var offset);
            Matrix3x3Wide.CreateFromQuaternion(ref pose.Orientation, out var orientation);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offset, ref orientation, out var localOffset);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref ray.Direction, ref orientation, out var localDirection);
            //Note that this division has two odd properties:
            //1) If the local direction has a near zero component, it is clamped to a nonzero but extremely small value. This is a hack, but it works reasonably well.
            //The idea is that any interval computed using such an inverse would be enormous. Those values will not be exactly accurate, but they will never appear as a result
            //because a parallel ray will never actually intersect the surface. The resulting intervals are practical approximations of the 'true' infinite intervals.
            //2) To compensate for the clamp and abs, we reintroduce the sign in the numerator. Note that it has the reverse sign since it will be applied to the offset to get the T value.
            var negativeOne = -Vector<float>.One;
            var epsilon = new Vector<float>(1e-15f);
            Vector3Wide offsetToTScale;
            offsetToTScale.X = Vector.ConditionalSelect(Vector.GreaterThan(localDirection.X, Vector<float>.Zero), negativeOne, Vector<float>.One) / Vector.Max(epsilon, Vector.Abs(localDirection.X));
            offsetToTScale.Y = Vector.ConditionalSelect(Vector.GreaterThan(localDirection.Y, Vector<float>.Zero), negativeOne, Vector<float>.One) / Vector.Max(epsilon, Vector.Abs(localDirection.Y));
            offsetToTScale.Z = Vector.ConditionalSelect(Vector.GreaterThan(localDirection.Z, Vector<float>.Zero), negativeOne, Vector<float>.One) / Vector.Max(epsilon, Vector.Abs(localDirection.Z));

            //Compute impact times for each pair of planes in local space.
            Vector3Wide negativeT, positiveT;
            negativeT.X = (localOffset.X - HalfWidth) * offsetToTScale.X;
            negativeT.Y = (localOffset.Y - HalfHeight) * offsetToTScale.Y;
            negativeT.Z = (localOffset.Z - HalfLength) * offsetToTScale.Z;
            positiveT.X = (localOffset.X + HalfWidth) * offsetToTScale.X;
            positiveT.Y = (localOffset.Y + HalfHeight) * offsetToTScale.Y;
            positiveT.Z = (localOffset.Z + HalfLength) * offsetToTScale.Z;
            Vector3Wide entryT, exitT;
            entryT.X = Vector.Min(negativeT.X, positiveT.X);
            entryT.Y = Vector.Min(negativeT.Y, positiveT.Y);
            entryT.Z = Vector.Min(negativeT.Z, positiveT.Z);
            exitT.X = Vector.Max(negativeT.X, positiveT.X);
            exitT.Y = Vector.Max(negativeT.Y, positiveT.Y);
            exitT.Z = Vector.Max(negativeT.Z, positiveT.Z);
            //In order for an impact to occur, the ray must enter all three slabs formed by the axis planes before exiting any of them.
            //In other words, the first exit must occur after the last entry.
            var earliestExit = Vector.Min(Vector.Min(exitT.X, exitT.Y), exitT.Z);
            var earliestEntry = Vector.Max(Vector.Max(entryT.X, entryT.Y), entryT.Z);
            t = Vector.Max(Vector<float>.Zero, earliestEntry);
            intersected = Vector.LessThanOrEqual(t, earliestExit);

            var useX = Vector.Equals(earliestEntry, entryT.X);
            var useY = Vector.AndNot(Vector.Equals(earliestEntry, entryT.Y), useX);
            normal.X = Vector.ConditionalSelect(useX, orientation.X.X, Vector.ConditionalSelect(useY, orientation.Y.X, orientation.Z.X));
            normal.Y = Vector.ConditionalSelect(useX, orientation.X.Y, Vector.ConditionalSelect(useY, orientation.Y.Y, orientation.Z.Y));
            normal.Z = Vector.ConditionalSelect(useX, orientation.X.Z, Vector.ConditionalSelect(useY, orientation.Y.Z, orientation.Z.Z));
            Vector3Wide.Dot(ref normal, ref offset, out var dot);
            var shouldNegate = Vector.LessThan(dot, Vector<float>.Zero);
            normal.X = Vector.ConditionalSelect(shouldNegate, -normal.X, normal.X);
            normal.Y = Vector.ConditionalSelect(shouldNegate, -normal.Y, normal.Y);
            normal.Z = Vector.ConditionalSelect(shouldNegate, -normal.Z, normal.Z);
        }
    }
}
