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
        public void GetBounds(ref Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(ref orientation, out var basis);
            var x = HalfWidth * basis.X;
            var y = HalfHeight * basis.Y;
            var z = HalfLength * basis.Z;
            max = Vector3.Abs(x) + Vector3.Abs(y) + Vector3.Abs(z);
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
    }
}
