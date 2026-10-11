using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
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
    public struct Rectangle : IConvexShape, IHomogeneousCompoundShape<Triangle, TriangleWide>
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

        /// <summary>
        /// Gets the local space position of the rectangle's corner in the first quadrant (positive X, positive Z).
        /// </summary>
        public readonly Vector3 Quadrant1 => new(HalfWidth, 0, HalfLength);
        /// <summary>
        /// Gets the local space position of the rectangle's corner in the second quadrant (negative X, positive Z).
        /// </summary>
        public readonly Vector3 Quadrant2 => new(-HalfWidth, 0, HalfLength);
        /// <summary>
        /// Gets the local space position of the rectangle's corner in the third quadrant (negative X, negative Z).
        /// </summary>
        public readonly Vector3 Quadrant3 => new(-HalfWidth, 0, -HalfLength);
        /// <summary>
        /// Gets the local space position of the rectangle's corner in the fourth quadrant (positive X, negative Z).
        /// </summary>
        public readonly Vector3 Quadrant4 => new(HalfWidth, 0, -HalfLength);

        readonly int IHomogeneousCompoundShape<Triangle, TriangleWide>.ChildCount => 2;

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

        /// <inheritdoc/>
        public readonly void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, BufferPool pool, ref TRayHitHandler hitHandler)
            where TRayHitHandler : struct, IShapeRayHitHandler
        {
            for (int i = 0; i < 2; ++i)
            {
                GetPosedLocalChild(i, out var triangle, out var childPose);
                RigidPose.MultiplyWithoutOverlap(pose, childPose, out var finalPose);
                if (triangle.RayTest(finalPose, ray.Origin, ray.Direction, out var t, out var normal) && t < maximumT)
                {
                    maximumT = t;
                    hitHandler.OnRayHit(in ray, ref maximumT, t, normal, i);
                }
            }
        }

        /// <inheritdoc/>
        public readonly void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, BufferPool pool, ref TRayHitHandler hitHandler)
            where TRayHitHandler : struct, IShapeRayHitHandler
        {
            for (int i = 0; i < 2; ++i)
            {
                GetPosedLocalChild(i, out var triangle, out var childPose);
                RigidPose.MultiplyWithoutOverlap(pose, childPose, out var finalPose);
                WideRayTester.Test<RaySource, Triangle, TriangleWide, TRayHitHandler>(ref triangle, finalPose, ref rays, ref hitHandler);
            }
        }

        /// <inheritdoc/>
        public readonly void GetLocalChild(int triangleIndex, out Triangle triangleData)
        {
            triangleData = triangleIndex == 0 ? new Triangle
            {
                A = Quadrant3,
                B = Quadrant4,
                C = Quadrant1
            } : new Triangle
            {
                A = Quadrant3,
                B = Quadrant1,
                C = Quadrant2
            };
        }

        /// <inheritdoc/>
        public readonly void GetPosedLocalChild(int triangleIndex, out Triangle triangleData, out RigidPose childPose)
        {
            GetLocalChild(triangleIndex, out triangleData);
            childPose = (triangleData.A + triangleData.B + triangleData.C) * (1f / 3f);
            triangleData.A -= childPose.Position;
            triangleData.B -= childPose.Position;
            triangleData.C -= childPose.Position;
        }

        /// <inheritdoc/>
        public readonly void GetLocalChild(int triangleIndex, ref TriangleWide triangleData)
        {
            GetLocalChild(triangleIndex, out var triangle);
            triangleData.WriteFirst(triangle);
        }

        readonly void IDisposableShape.Dispose(BufferPool pool)
        {
        }

        readonly unsafe void IBoundsQueryableCompound.FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
        {
            for (int pairIndex = 0; pairIndex < pairs.Length; ++pairIndex)
            {
                ref var pair = ref pairs[pairIndex];
                ref var rectangle = ref Unsafe.AsRef<Rectangle>(pair.Container);
                if (BoundingBox.Intersects(rectangle.Quadrant3, rectangle.Quadrant1, pair.Min, pair.Max))
                {
                    ref var overlapsForPair = ref overlaps.GetOverlapsForPair(pairIndex);
                    overlapsForPair.Allocate(pool) = 0;
                    overlapsForPair.Allocate(pool) = 1;
                }
            }
        }

        readonly unsafe void IBoundsQueryableCompound.FindLocalOverlaps<TOverlaps>(Vector3 min, Vector3 max, Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps)
        {
            Tree.ConvertBoxToCentroidWithExtent(min, max, out var sweepOrigin, out var expansion);
            TreeRay.CreateFrom(sweepOrigin, sweep, maximumT, out var ray);
            ref var overlapsCollection = ref Unsafe.AsRef<TOverlaps>(overlaps);
            var childMin = Quadrant3 - expansion;
            var childMax = Quadrant1 + expansion;
            if (Tree.Intersects(childMin, childMax, &ray, out _))
            {
                //Both triangles are guaranteed to overlap because they have the same bounds as the rectangle itself.
                overlapsCollection.Allocate(pool) = 0;
                overlapsCollection.Allocate(pool) = 1;
            }
        }

        readonly void IBoundsQueryableCompound.FindLocalOverlaps<TEnumerator>(Vector3 min, Vector3 max, BufferPool pool, Shapes shapes, ref TEnumerator enumerator)
        {
            // The rectangle is a homogeneous compound with two triangles.
            // Both triangles are guaranteed to have the same bounds as the rectangle itself, so we can just check the rectangle's bounds against the query.

            if (BoundingBox.Intersects(Quadrant3, Quadrant1, min, max))
            {
                //Both triangles are guaranteed to overlap.
                if (!enumerator.LoopBody(0))
                    return;
                enumerator.LoopBody(1);
            }
        }

        public const int Id = 9;

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
            var approaching = Vector.LessThan(localDirection.Y, Vector<float>.Zero);
            var inFront = Vector.GreaterThanOrEqual(localOffset.Y, Vector<float>.Zero);

            //Avoid division by zero in lanes which cannot intersect. The value used in those lanes is discarded below.
            var distanceAlongNormal = Vector.Max(new Vector<float>(Epsilon), -localDirection.Y);
            var candidateT = Vector.Max(Vector<float>.Zero, localOffset.Y / distanceAlongNormal);

            var hitX = localOffset.X + localDirection.X * candidateT;
            var hitZ = localOffset.Z + localDirection.Z * candidateT;
            var withinBounds = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(Vector.Abs(hitX), HalfWidth),
                Vector.LessThanOrEqual(Vector.Abs(hitZ), HalfLength));
            intersected = Vector.BitwiseAnd(Vector.BitwiseAnd(approaching, inFront), withinBounds);
            t = Vector.ConditionalSelect(intersected, candidateT, Vector<float>.Zero);

            normal.X = orientation.Y.X;
            normal.Y = orientation.Y.Y;
            normal.Z = orientation.Y.Z;
            Vector3Wide.Dot(normal, offset, out var dot);
            var shouldNegate = Vector.LessThan(dot, Vector<float>.Zero);
            normal.X = Vector.ConditionalSelect(shouldNegate, -normal.X, normal.X);
            normal.Y = Vector.ConditionalSelect(shouldNegate, -normal.Y, normal.Y);
            normal.Z = Vector.ConditionalSelect(shouldNegate, -normal.Z, normal.Z);
            normal.X = Vector.ConditionalSelect(intersected, normal.X, Vector<float>.Zero);
            normal.Y = Vector.ConditionalSelect(intersected, normal.Y, Vector<float>.Zero);
            normal.Z = Vector.ConditionalSelect(intersected, normal.Z, Vector<float>.Zero);
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
            support.X = Vector.ConditionalSelect(Vector.LessThan(direction.X, Vector<float>.Zero), -shape.HalfWidth, shape.HalfWidth);
            support.Y = Vector<float>.Zero;
            support.Z = Vector.ConditionalSelect(Vector.LessThan(direction.Z, Vector<float>.Zero), -shape.HalfLength, shape.HalfLength);
        }
    }
}
