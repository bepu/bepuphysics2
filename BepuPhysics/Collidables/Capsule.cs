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
        public void GetBounds(ref Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Quaternion.TransformUnitY(ref orientation, out var segmentOffset);
            max = Vector3.Abs(HalfLength * segmentOffset) + new Vector3(Radius);
            min = -max;
        }

        public bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, out float t, out Vector3 normal)
        {
            //It's convenient to work in local space, so pull the ray into the capsule's local space.
            Matrix3x3.CreateFromQuaternion(ref pose.Orientation, out var orientation);
            var o = origin - pose.Position;
            Matrix3x3.TransformTranspose(ref o, ref orientation, out o);
            Matrix3x3.TransformTranspose(ref direction, ref orientation, out var d);

            //Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
            var inverseDLength = 1f / d.Length();
            d *= inverseDLength;

            //Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
            var tOffset = -Vector3.Dot(o, d) - (HalfLength + Radius);
            if (tOffset < 0)
                tOffset = 0;
            o += d * tOffset;
            var oh = new Vector3(o.X, 0, o.Z);
            var dh = new Vector3(d.X, 0, d.Z);
            var a = Vector3.Dot(dh, dh);
            var b = Vector3.Dot(oh, dh);
            var radiusSquared = Radius * Radius;
            var c = Vector3.Dot(oh, oh) - radiusSquared;
            if (b > 0 && c > 0)
            {
                //Ray is outside and pointing away, no hit.
                t = 0;
                normal = new Vector3();
                return false;
            }

            float sphereY;
            if (a > 1e-8f)
            {
                var discriminant = b * b - a * c;
                if (discriminant < 0)
                {
                    //The infinite cylinder isn't hit, so the capsule can't be hit.
                    t = 0;
                    normal = new Vector3();
                    return false;
                }
                t = (-b - (float)Math.Sqrt(discriminant)) / a;
                if (t < -tOffset)
                    t = -tOffset;
                var cylinderHitLocation = o + d * t;
                if (cylinderHitLocation.Y < -HalfLength)
                {
                    sphereY = -HalfLength;
                }
                else if (cylinderHitLocation.Y > HalfLength)
                {
                    sphereY = HalfLength;
                }
                else
                {
                    //The hit is on the cylindrical portion of the capsule.
                    normal = new Vector3(cylinderHitLocation.X, 0, cylinderHitLocation.Z) / Radius;
                    Matrix3x3.Transform(ref normal, ref orientation, out normal);
                    t = (t + tOffset) * inverseDLength;
                    return true;
                }
            }
            else
            {
                //The ray is parallel to the axis; the impact is on a spherical cap or nothing.
                sphereY = d.Y > 0 ? -HalfLength : HalfLength;
            }

            var os = o - new Vector3(0, sphereY, 0);
            var capB = Vector3.Dot(os, d);
            var capC = Vector3.Dot(os, os) - radiusSquared;

            if (capB > 0 && capC > 0)
            {
                //Ray is outside and pointing away, no hit.
                t = 0;
                normal = new Vector3();
                return false;
            }

            var capDiscriminant = capB * capB - capC;
            if (capDiscriminant < 0)
            {
                //Ray misses, no hit.
                t = 0;
                normal = new Vector3();
                return false;
            }
            t = -capB - (float)Math.Sqrt(capDiscriminant);
            if (t < -tOffset)
                t = -tOffset;
            normal = (os + d * t) / Radius;
            t = (t + tOffset) * inverseDLength;
            Matrix3x3.Transform(ref normal, ref orientation, out normal);
            return true;

        }

        public void ComputeLocalInverseInertia(float inverseMass, out Triangular3x3 localInverseInertia)
        {

            var r2 = Radius * Radius;
            var h2 = HalfLength * HalfLength;
            var cylinderVolume = 2 * HalfLength * r2 * MathHelper.Pi;
            var sphereVolume = (4f / 3f) * r2 * Radius * MathHelper.Pi;
            var inverseTotal = 1f / (cylinderVolume + sphereVolume);
            //Volume is in units of the capsule's whole volume.
            cylinderVolume *= inverseTotal;
            sphereVolume *= inverseTotal;
            localInverseInertia.M11 = inverseMass / (
                cylinderVolume * ((3f / 12f) * r2 + (4f / 12f) * h2) +
                sphereVolume * ((2f / 5f) * r2 + (6f / 8f) * Radius * HalfLength + h2));
            localInverseInertia.M21 = 0;
            localInverseInertia.M22 = inverseMass / (cylinderVolume * (1f / 2f) * r2 + sphereVolume * (2f / 5f) * r2);
            localInverseInertia.M31 = 0;
            localInverseInertia.M32 = 0;
            localInverseInertia.M33 = localInverseInertia.M11;            
        }

        /// <summary>
        /// Type id of capsule shapes.
        /// </summary>
        public const int Id = 1;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
