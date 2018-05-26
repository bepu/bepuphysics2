using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuUtilities.Memory;
using System.Diagnostics;
using BepuUtilities;
using BepuPhysics.Trees;

namespace BepuPhysics.Collidables
{
    public struct RayWide
    {
        public Vector3Wide Origin;
        public Vector3Wide Direction;

        public void Gather(in RayData ray)
        {
            GatherScatter.GetFirst(ref Origin.X) = ray.Origin.X;
            GatherScatter.GetFirst(ref Origin.Y) = ray.Origin.Y;
            GatherScatter.GetFirst(ref Origin.Z) = ray.Origin.Z;
            GatherScatter.GetFirst(ref Direction.X) = ray.Direction.X;
            GatherScatter.GetFirst(ref Direction.Y) = ray.Direction.Y;
            GatherScatter.GetFirst(ref Direction.Z) = ray.Direction.Z;
        }
    }

    public struct Sphere : IConvexShape
    {
        public float Radius;

        public Sphere(float radius)
        {
            Radius = radius;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            maximumRadius = Radius;
            maximumAngularExpansion = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeBounds(in BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            min = new Vector3(-Radius);
            max = new Vector3(Radius);
        }
        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
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

        public void ComputeInertia(float mass, out BodyInertia inertia)
        {
            inertia.InverseMass = 1f / mass;
            inertia.InverseInertiaTensor.XX = inertia.InverseMass / ((2f / 5f) * Radius * Radius);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inertia.InverseInertiaTensor.XX;
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inertia.InverseInertiaTensor.XX;
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
        public void Broadcast(in Sphere shape)
        {
            Radius = new Vector<float>(shape.Radius);
        }

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


        public int MinimumWideRayCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return 2;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RayTest(ref RigidPoses pose, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            //Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
            Vector3Wide.Length(ref rayWide.Direction, out var inverseDLength);
            inverseDLength = Vector<float>.One / inverseDLength;
            Vector3Wide.Scale(ref rayWide.Direction, ref inverseDLength, out var d);

            //Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
            Vector3Wide.Subtract(ref rayWide.Origin, ref pose.Position, out var o);
            Vector3Wide.Dot(ref o, ref d, out var dot);
            var tOffset = Vector.Max(Vector<float>.Zero, -dot - Radius);
            Vector3Wide.Scale(ref d, ref tOffset, out var oOffset);
            Vector3Wide.Add(ref oOffset, ref o, out o);
            Vector3Wide.Dot(ref o, ref d, out var b);
            Vector3Wide.Dot(ref o, ref o, out var c);
            c -= Radius * Radius;

            //If b > 0 && c > 0, ray is outside and pointing away, no hit.
            //If discriminant < 0, the ray misses.
            var discriminant = b * b - c;
            intersected = Vector.BitwiseAnd(
                Vector.BitwiseOr(
                    Vector.LessThanOrEqual(b, Vector<float>.Zero),
                    Vector.LessThanOrEqual(c, Vector<float>.Zero)),
                Vector.GreaterThanOrEqual(discriminant, Vector<float>.Zero));


            t = Vector.Max(-tOffset, -b - Vector.SquareRoot(discriminant));
            Vector3Wide.Scale(ref d, ref t, out oOffset);
            Vector3Wide.Add(ref o, ref oOffset, out normal);
            var inverseRadius = Vector<float>.One / Radius;
            Vector3Wide.Scale(ref normal, ref inverseRadius, out normal);
            t = (t + tOffset) * inverseDLength;
        }
    }

}
