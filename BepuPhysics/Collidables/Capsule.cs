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
            var o = origin - pose.Position;
            Matrix3x3.TransformTranspose(ref o, ref orientation, out o);
            var radiusSquared = Radius * Radius;
            //Check up front whether the ray starts within the capsule.
            if ((o - new Vector3(0, MathHelper.Clamp(o.Y, -HalfLength, HalfLength), 0)).LengthSquared() <= radiusSquared)
            {
                t = 0;
                //There is no obvious choice for a normal for rays which start inside the capsule's volume.
                //Just choose the reverse of the ray direction.
                //(One potentially useful alternative is the direction of minimum penetration, but that is not trivial to compute for all shapes, and it's not like this function
                //exposes penetration depth. Probably best to leave that logic to a dedicated point sample function.)
                normal = -direction;
                return true;
            }
            Matrix3x3.TransformTranspose(ref direction, ref orientation, out var d);

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
            var discriminant = 2 * dxox * dzoz - dz2 * o.X * o.X - dx2 * o.Z * o.Z + denom * radiusSquared;
            if (discriminant < 0)
            {
                //The infinite cylinder isn't hit, so the capsule can't be hit.
                t = 0;
                normal = new Vector3();
                return false;
            }
            var b = dxox + dzoz;

            //There exists a solution to the cylinder intersection.
            var unscaledTOffset = (float)Math.Sqrt(discriminant);
            //If -b + offset < 0, then the second impact on the cylinder occurs before the ray starts and there can be no intersection.
            if (unscaledTOffset - b < 0)
            {
                t = 0;
                normal = new Vector3();
                return false;
            }
            var inverseDenom = 1f / denom;
            float spherePosition;
            if (d.Y * d.Y * inverseDenom < 1e10)
            {
                t = (-b - unscaledTOffset) / denom;
                //We've already ensured that the second solution occurs at some point after t = 0. Still don't want to generate negative t values, so clamp:
                if (t < 0)
                    t = 0;
                var cylinderHitLocation = o + d * t;
                if (cylinderHitLocation.Y < -HalfLength)
                {
                    spherePosition = -HalfLength;
                }
                else if (cylinderHitLocation.Y > HalfLength)
                {
                    spherePosition = HalfLength;
                }
                else
                {
                    //The hit is on the cylindrical portion of the capsule.
                    normal = new Vector3(cylinderHitLocation.X, 0, cylinderHitLocation.Z) / Radius;
                    Matrix3x3.Transform(ref normal, ref orientation, out normal);
                    return true;
                }
            }
            else
            {
                //The ray is parallel to the axis; the impact is on a spherical cap.
                //We know the ray start isn't in the capsule, so we only need to test 
                spherePosition = d.Y > 0 ? -HalfLength : HalfLength;
            }

            var capA = Vector3.Dot(d, d);
            var capB = -2 * (spherePosition * d.Y - Vector3.Dot(d, o));
            var capC = spherePosition * spherePosition - 2 * spherePosition * o.Y + Vector3.Dot(o, o) - radiusSquared;
            var capDiscriminant = capB * capB - 4 * capA * capC;
            if (capDiscriminant < 0)
            {
                t = 0;
                normal = new Vector3();
                return false;
            }
            var unscaledCapTOffset = (float)Math.Sqrt(capDiscriminant);
            if (unscaledCapTOffset - capB < 0)
            {
                t = 0;
                normal = new Vector3();
                return false;
            }
            t = (-capB - unscaledCapTOffset) / (2 * capA);
            normal = (o + d * t - new Vector3(0, spherePosition, 0)) / Radius;
            Matrix3x3.Transform(ref normal, ref orientation, out normal);
            return true;

        }

        /// <summary>
        /// Type id of capsule shapes.
        /// </summary>
        public const int Id = 1;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
