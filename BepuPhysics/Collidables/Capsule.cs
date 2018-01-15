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


        public bool RayTest(ref Vector3 origin, ref Vector3 direction, ref Capsule capsule, ref RigidPose pose, out float t, out Vector3 normal)
        {
            //It's convenient to work in local space, so pull the ray into the capsule's local space.
            Matrix3x3.CreateFromQuaternion(ref pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(ref direction, ref orientation, out var d);
            var o = origin - pose.Position;
            Matrix3x3.TransformTranspose(ref o, ref orientation, out o);

            //The goal is to solve:
            //||o + d * t - closestPointOnSegmentToRay|| = radius
            //for t.
            //We'll first compute closestPointOnSegment.
            //That requires minimizing the distance between two lines:
            //distance == ((oa + da * ta) - (ob + db * tb)).norm()
            //Taking the derivative with respect to ta and tb and solving for tb (where the 'b' line is the capsule's axis) gives:
            var horizontalPlaneDot = d.X * d.X + d.Z * d.Z;
            var inverseHorizontalPlaneDot = 1f / horizontalPlaneDot;
            float tb;
            if ((d.Y * d.Y) * inverseHorizontalPlaneDot > 1e7f)
            {
                //The ray is parallel with the capsule's local axis. Infinite solutions, so just project the ray origin to the axis.
                tb = o.Y;
            }
            else
            {
                //The simplicity is largely due to ob being (0,0,0) and db being (0,1,0), since we're working in local space.
                tb = -(d.X * d.Y * o.X + d.Y * d.Z * o.Z - horizontalPlaneDot * o.Y) * inverseHorizontalPlaneDot;
            }
            //Note that the true closest point on the segment cannot extend beyond the half length from the local origin.
            if (tb > HalfLength)
                tb = HalfLength;
            var negativeHalfLength = -HalfLength;
            if (tb < negativeHalfLength)
                tb = negativeHalfLength;

            //Now we can try to solve our original equation:
            //|| o + d * t - (0, tb, 0) || = radius
            //|| o + d * t - (0, tb, 0) ||^2 = radius^2
            //(d * d) * t * t + 2 * (d * o - d.y * tb) * t - 2 * o.y * tb + tb * tb + (o * o) - radius * radius = 0
            //This is now a quadratic equation with coefficients:
            //a: d * d
            //b: 2 * (d * o - d.y * tb)
            //c: 2 * o.y * tb + tb * tb + (o * o) - radius * radius
            var a = Vector3.Dot(d, d);
            var b = 2 * (Vector3.Dot(d, o) - d.Y * tb);
            var c = 2 * o.Y * tb + tb * tb + Vector3.Dot(o, o) - capsule.Radius * capsule.Radius;

            var discriminant = b * b - 4 * a * c;
            if (discriminant >= 0)
            {
                //There exists a solution.
                var offset = (float)Math.Sqrt(discriminant);
                //If -b + offset < 0, then the second impact occurs before the ray starts and there is no intersection.
                if(offset - b < 0)
                {
                    t = 0;
                    normal = new Vector3();
                    return false;
                }
                t = (-b - offset) / (2 * a);
                //We've already ensured that the second solution occurs at some point after t = 0. Still don't want to generate negative t values, so clamp:
                if (t < 0)
                    t = 0;
                var normalOffset = o + d * t;
                normalOffset.Y -= tb;
                var lengthSquared = normalOffset.LengthSquared();
                if(lengthSquared > 1e-9f)
                {
                    normal = normalOffset / (float)Math.Sqrt(lengthSquared);
                }
                else
                {
                    normal = new Vector3(0, 1, 0);
                }
                Matrix3x3.Transform(ref normal, ref orientation, out normal);
                return true;
            }
            t = 0;
            normal = new Vector3();
            return false;

        }

        /// <summary>
        /// Type id of capsule shapes.
        /// </summary>
        public const int Id = 1;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
