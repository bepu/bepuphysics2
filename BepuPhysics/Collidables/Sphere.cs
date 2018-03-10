using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuUtilities.Memory;
using System.Diagnostics;

namespace BepuPhysics.Collidables
{
    public struct Sphere : IConvexShape
    {
        public float Radius;

        public Sphere(float radius)
        {
            Radius = radius;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            min = new Vector3(-Radius);
            max = new Vector3(Radius);
        }

        public bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, out float t, out Vector3 normal)
        {
            //Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
            var inverseDLength = 1f / direction.Length();
            var d = direction * inverseDLength;

            //Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
            var o = origin - pose.Position;
            var tOffset = -Vector3.Dot(o, d) - Radius;
            if (tOffset < 0)
                tOffset = 0;
            o += d * tOffset;
            var b = Vector3.Dot(o, d);
            var c = Vector3.Dot(o, o) - Radius * Radius;

            if (b > 0 && c > 0)
            {
                //Ray is outside and pointing away, no hit.
                t = 0;
                normal = new Vector3();
                return false;
            }

            var discriminant = b * b - c;
            if (discriminant < 0)
            {
                //Ray misses, no hit.
                t = 0;
                normal = new Vector3();
                return false;
            }
            t = -b - (float)Math.Sqrt(discriminant);
            if (t < -tOffset)
                t = -tOffset;
            normal = (o + d * t) / Radius;
            t = (t + tOffset) * inverseDLength;
            return true;
        }

        public void ComputeLocalInverseInertia(float inverseMass, out Triangular3x3 localInverseInertia)
        {
            localInverseInertia.XX = inverseMass / ((2f / 5f) * Radius * Radius);
            localInverseInertia.YX = 0;
            localInverseInertia.YY = localInverseInertia.XX;
            localInverseInertia.ZX = 0;
            localInverseInertia.ZY = 0;
            localInverseInertia.ZZ = localInverseInertia.XX;
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapes)
        {
            return new ConvexShapeBatch<Sphere, SphereWide>(pool, initialCapacity);
        }

        /// <summary>
        /// Type id of sphere shapes.
        /// </summary>
        public const int Id = 0;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }

    public struct SphereWide : IShapeWide<Sphere>
    {
        public Vector<float> Radius;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref Sphere source)
        {
            Unsafe.As<Vector<float>, float>(ref Radius) = source.Radius;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            //Spheres have perfect symmetry, so there is no need for angular expansion.
            maximumRadius = new Vector<float>();
            maximumAngularExpansion = new Vector<float>();

            //It's technically true that spheres (and only spheres) do not require orientation to be loaded and could be special cased to reduce memory traffic, but just heck no.
            //It's very likely that the orientation loaded for the sphere was already in L1 anyway due to the online batching performed during the pose integrator.
            var negatedRadius = -Radius;
            max = new Vector3Wide(ref Radius);
            min = new Vector3Wide(ref negatedRadius);
        }
    }

}
