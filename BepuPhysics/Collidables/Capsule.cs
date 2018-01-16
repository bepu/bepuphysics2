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
    public struct CapsuleWide
    {
        public Vector<float> Radius;
        public Vector<float> HalfLength;
    }

    /// <summary>
    /// Collision shape representing a sphere-expanded line segment.
    /// </summary>
    public struct Capsule : IShape
    {
        /// <summary>
        /// Spherical expansion applied to the internal line segment.
        /// </summary>
        public float Radius;
        /// <summary>
        /// Half of the length of the internal line segment. Oriented along the local Y axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the length of the capsule.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        public Capsule(float radius, float length)
        {
            Radius = radius;
            HalfLength = length * 0.5f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Gather(ref Buffer<Capsule> shapes, ref Vector<int> shapeIndices, int count, out CapsuleWide capsules)
        {
            ref var radiusBase = ref Unsafe.As<Vector<float>, float>(ref capsules.Radius);
            ref var halfLengthBase = ref Unsafe.As<Vector<float>, float>(ref capsules.HalfLength);
            ref var shapeIndicesBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            Debug.Assert(count <= Vector<float>.Count);
            for (int i = 0; i < count; ++i)
            {
                ref var shape = ref shapes[Unsafe.Add(ref shapeIndicesBase, i)];
                Unsafe.Add(ref radiusBase, i) = shape.Radius;
                Unsafe.Add(ref halfLengthBase, i) = shape.HalfLength;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref QuaternionWide orientations,
            out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
            where TShape : struct, IShape
        {
            Gather(ref Unsafe.As<Buffer<TShape>, Buffer<Capsule>>(ref shapes), ref shapeIndices, count, out var capsules);
            QuaternionWide.TransformUnitY(ref orientations, out var segmentOffset);
            Vector3Wide.Scale(ref segmentOffset, ref capsules.HalfLength, out segmentOffset);
            Vector3Wide.Abs(ref segmentOffset, out segmentOffset);

            //The half length extends symmetrically along positive local Y and negative local Y.
            Vector3Wide.Add(ref segmentOffset, ref capsules.Radius, out max);
            Vector3Wide.Negate(ref max, out min);

            maximumRadius = capsules.HalfLength + capsules.Radius;
            //The minimum radius is capsules.Radius, so the maximum offset is simply the half length.
            maximumAngularExpansion = capsules.HalfLength;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            BepuUtilities.Quaternion.TransformUnitY(ref orientation, out var segmentOffset);
            max = Vector3.Abs(HalfLength * segmentOffset) + new Vector3(Radius);
            min = -max;
        }


        public bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, out float t, out Vector3 normal)
        {
            //It's convenient to work in local space, so pull the ray into the capsule's local space.
            Matrix3x3.CreateFromQuaternion(ref pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(ref direction, ref orientation, out var d);
            var o = origin - pose.Position;
            Matrix3x3.TransformTranspose(ref o, ref orientation, out o);

            //The goal is to solve:
            //||o + d * t - (0, clamp((o + d * t).y, -halfLength, halfLength), 0) || = radius
            //for t.
            //This is a a quadratic equation, but it is made more complicated by the clamps.
            //We will handle it piecewise. First, calculate the time of impact on the infinite cylinder, solving the following for t:
            //||o + d * t - (0, (o + d * t).y, 0) || = radius
            //t == (-dx*ox - dz*oz - sqrt(-dz^2*ox^2 + 2*dx*dz*ox*oz - dx^2*oz^2 + (dx^2 + dz^2)*r^2))/(dx^2 + dz^2)
            var dx2 = d.X * d.X;
            var dz2 = d.Z * d.Z;
            var dxox = d.X * o.X;
            var dzoz = d.Z * o.Z;
            var denom = dx2 + dz2;
            var discriminant = 2 * dxox * dzoz - dz2 * o.X * o.X - dx2 * o.Z * o.Z + denom * Radius * Radius;
            if (discriminant < 0)
            {
                //The infinite cylinder isn't hit, so the capsule can't be hit.
                t = 0;
                normal = new Vector3();
                return false;
            }
            var b = dxox + dzoz;

            //There exists a solution to the cylinder intersection.
            var offset = (float)Math.Sqrt(discriminant);
            //If -b + offset < 0, then the second impact occurs before the ray starts and there is no intersection.
            if (offset - b < 0)
            {
                t = 0;
                normal = new Vector3();
                return false;
            }
            t = (-b - offset) / denom;
            //We've already ensured that the second solution occurs at some point after t = 0. Still don't want to generate negative t values, so clamp:
            if (t < 0)
                t = 0;
            var cylinderHitLocation = o + d * t;
            //var normalOffset = o + d * t;
            //normalOffset.Y -= tb;
            //var lengthSquared = normalOffset.LengthSquared();
            //if (lengthSquared > 1e-9f)
            //{
            //    normal = normalOffset / (float)Math.Sqrt(lengthSquared);
            //}
            //else
            //{
            //    normal = new Vector3(0, 1, 0);
            //}
            //Matrix3x3.Transform(ref normal, ref orientation, out normal);
            t = 0;
            normal = new Vector3();
            return true;


        }

        /// <summary>
        /// Type id of capsule shapes.
        /// </summary>
        public const int Id = 1;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
