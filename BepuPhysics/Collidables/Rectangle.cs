using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Collision shape representing an individual rectangle. Rectangle collisions and ray tests are one-sided; only tests which see the rectangle from the side of its normal will report hits.
    /// </summary>
    public struct Rectangle : IConvexShape
    {
        /// <summary>
        /// Half of the rectangle's width along its local X axis.
        /// </summary>
        public float HalfWidth;
        /// <summary>
        /// Half of the rectangle's length along its local Z axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the width of the rectangle along its local X axis.
        /// </summary>
        public float Width { readonly get { return HalfWidth * 2; } set { HalfWidth = value * 0.5f; } }
        /// <summary>
        /// Gets or sets the length of the rectangle along its local Z axis.
        /// </summary>
        public float Length { readonly get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        /// <inheritdoc/>
        public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var basis);
            var x = HalfWidth * basis.X;
            var z = HalfLength * basis.Z;
            max = Vector3.Abs(x) + Vector3.Abs(z);
            min = -max;
        }

        /// <inheritdoc/>
        public readonly void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            maximumRadius = (float)Math.Sqrt(HalfWidth * HalfWidth + HalfLength * HalfLength);
            maximumAngularExpansion = maximumRadius;
        }

        /// <inheritdoc/>
        public readonly bool RayTest(in RigidPose pose, Vector3 origin, Vector3 direction, out float t, out Vector3 normal)
        {
            var offset = origin - pose.Position;
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(offset, orientation, out var localOffset);
            Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);

            // Rectangle lies in the local XZ plane (Y = 0). Its local normal points along +Y.

            var velocityAlongNormal = localDirection.Y;
            if (velocityAlongNormal >= 0)
            {
                // One sided: require the ray to approach from the side of the normal.
                t = 0;
                normal = new Vector3();
                return false;
            }

            // Distance from origin to the plane along the normal (unnormalized math is fine since normal is unit).
            var distanceAlongNormal = localOffset.Y;
            if (distanceAlongNormal < 0)
            {
                // Impact would be behind the ray origin.
                t = 0;
                normal = new Vector3();
                return false;
            }

            t = distanceAlongNormal / -velocityAlongNormal;

            // Compute local hit location and test bounds against rectangle extents.
            var hitX = localOffset.X + localDirection.X * t;
            var hitZ = localOffset.Z + localDirection.Z * t;
            if (float.Abs(hitX) > HalfWidth || float.Abs(hitZ) > HalfLength)
            {
                t = 0;
                normal = new Vector3();
                return false;
            }

            // Transform normal back to world space. Local normal is the local Y axis.
            normal = orientation.Y;

            // Ensure normal points away from the rectangle center relative to the ray origin.
            if (Vector3.Dot(normal, offset) < 0)
            {
                normal = -normal;
            }
            return true;
        }

        /// <inheritdoc/>
        public readonly BodyInertia ComputeInertia(float mass)
        {
            BodyInertia inertia;
            inertia.InverseMass = 1f / mass;
            var x2 = HalfWidth * HalfWidth;
            const float y2 = 0f;
            var z2 = HalfLength * HalfLength;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass * 3 / (y2 + z2);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseMass * 3 / (x2 + z2);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseMass * 3 / (x2 + y2);
            return inertia;
        }

        /// <inheritdoc/>
        public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<Rectangle, RectangleWide>(pool, initialCapacity);
        }

        public const int Id = 12;
        /// <inheritdoc/>
        public static int TypeId => Id;
    }


    public struct RectangleWide : IShapeWide<Rectangle>
    {
        public Vector<float> HalfWidth;
        public Vector<float> HalfLength;

        public void Broadcast(in Rectangle shape)
        {
            HalfWidth = new Vector<float>(shape.HalfWidth);
            HalfLength = new Vector<float>(shape.HalfLength);
        }

        public void WriteFirst(in Rectangle source)
        {
            Unsafe.As<Vector<float>, float>(ref HalfWidth) = source.HalfWidth;
            Unsafe.As<Vector<float>, float>(ref HalfLength) = source.HalfLength;
        }

        public readonly bool AllowOffsetMemoryAccess => true;
        public readonly int InternalAllocationSize => 0;

        public readonly void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientations, out var basis);
            max.X = Vector.Abs(HalfWidth * basis.X.X) + Vector.Abs(HalfLength * basis.Z.X);
            max.Y = Vector.Abs(HalfWidth * basis.X.Y) + Vector.Abs(HalfLength * basis.Z.Y);
            max.Z = Vector.Abs(HalfWidth * basis.X.Z) + Vector.Abs(HalfLength * basis.Z.Z);

            Vector3Wide.Negate(max, out min);

            maximumRadius = Vector.SquareRoot(HalfWidth * HalfWidth + HalfLength * HalfLength);
            maximumAngularExpansion = maximumRadius - Vector.Min(HalfWidth, HalfLength);
        }

        public readonly void Initialize(in Buffer<byte> memory)
        {
        }

        public void WriteSlot(int index, in Rectangle source)
        {
            GatherScatter.GetOffsetInstance(ref this, index).WriteFirst(source);
        }

        public static int MinimumWideRayCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return 3;
            }
        }

        public readonly void RayTest(ref RigidPoseWide poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            const float Epsilon = 1e-8f;

            Vector3Wide.Subtract(rayWide.Origin, poses.Position, out var offset);
            Matrix3x3Wide.CreateFromQuaternion(poses.Orientation, out var orientation);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offset, orientation, out var localOffset);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(rayWide.Direction, orientation, out var localDirection);

            //The rectangle lies in the local XZ plane and is one sided, with its normal pointing along +Y.
            var zero = Vector<float>.Zero;
            var approaching = Vector.LessThan(localDirection.Y, zero);
            var inFront = Vector.GreaterThanOrEqual(localOffset.Y, zero);

            //Avoid division by zero in lanes which cannot intersect. The value used in those lanes is discarded below.
            var distanceAlongNormal = Vector.Max(new Vector<float>(Epsilon), -localDirection.Y);
            var candidateT = Vector.Max(zero, localOffset.Y / distanceAlongNormal);

            var hitX = localOffset.X + localDirection.X * candidateT;
            var hitZ = localOffset.Z + localDirection.Z * candidateT;
            var withinBounds = (Vector.Abs(hitX) <= HalfWidth) & (Vector.Abs(hitZ) <= HalfLength);
            intersected = approaching & inFront & withinBounds;
            t = Vector.ConditionalSelect(intersected, candidateT, zero);

            normal.X = orientation.Y.X;
            normal.Y = orientation.Y.Y;
            normal.Z = orientation.Y.Z;
            Vector3Wide.Dot(normal, offset, out var dot);
            var shouldNegate = dot < zero;
            normal.X = Vector.ConditionalSelect(shouldNegate, -normal.X, normal.X);
            normal.Y = Vector.ConditionalSelect(shouldNegate, -normal.Y, normal.Y);
            normal.Z = Vector.ConditionalSelect(shouldNegate, -normal.Z, normal.Z);
            normal.X = Vector.ConditionalSelect(intersected, normal.X, zero);
            normal.Y = Vector.ConditionalSelect(intersected, normal.Y, zero);
            normal.Z = Vector.ConditionalSelect(intersected, normal.Z, zero);
        }
    }


    public readonly struct RectangleSupportFinder : ISupportFinder<Rectangle, RectangleWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in RectangleWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in RectangleWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, terminatedLanes, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeLocalSupport(in RectangleWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            support.X = Vector.ConditionalSelect(direction.X < Vector<float>.Zero, -shape.HalfWidth, shape.HalfWidth);
            support.Y = Vector<float>.Zero;
            support.Z = Vector.ConditionalSelect(direction.Z < Vector<float>.Zero, -shape.HalfLength, shape.HalfLength);
        }
    }
}
