using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuUtilities.Memory;
using System.Diagnostics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Trees;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Collision shape representing an individual triangle. Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound clockwise will generate contacts.
    /// </summary>
    public struct Triangle : IConvexShape
    {
        /// <summary>
        /// First vertex of the triangle in local space.
        /// </summary>
        public Vector3 A;
        /// <summary>
        /// Second vertex of the triangle in local space.
        /// </summary>
        public Vector3 B;
        /// <summary>
        /// Third vertex of the triangle in local space.
        /// </summary>
        public Vector3 C;

        /// <summary>
        /// Creates a triangle shape.
        /// </summary>
        /// <param name="a">First vertex of the triangle in local space.</param>
        /// <param name="b">Second vertex of the triangle in local space.</param>
        /// <param name="c">Third vertex of the triangle in local space.</param>
        public Triangle(in Vector3 a, in Vector3 b, in Vector3 c)
        {
            A = a;
            B = b;
            C = c;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeBounds(in Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var basis);
            Matrix3x3.Transform(A, basis, out var worldA);
            Matrix3x3.Transform(B, basis, out var worldB);
            Matrix3x3.Transform(C, basis, out var worldC);
            min = Vector3.Min(worldA, Vector3.Min(worldB, worldC));
            max = Vector3.Max(worldA, Vector3.Max(worldB, worldC));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            maximumRadius = (float)Math.Sqrt(MathHelper.Max(A.LengthSquared(), MathHelper.Max(B.LengthSquared(), C.LengthSquared())));
            maximumAngularExpansion = maximumRadius;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool RayTest(in Vector3 a, in Vector3 b, in Vector3 c, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
        {
            //Note that this assumes clockwise winding. Rays coming from the opposite direction pass through; triangles are one sided.
            var ab = b - a;
            var ac = c - a;
            normal = Vector3.Cross(ac, ab);
            var dn = -Vector3.Dot(direction, normal);
            if (dn <= 0)
            {
                t = 0;
                normal = new Vector3();
                return false;
            }
            var ao = origin - a;
            t = Vector3.Dot(ao, normal);
            if (t < 0)
            {
                //Impact occurred before the start of the ray.
                return false;
            }
            var aoxd = Vector3.Cross(ao, direction);
            var v = -Vector3.Dot(ac, aoxd);
            if (v < 0 || v > dn)
            {
                //Invalid barycentric coordinate for b.
                return false;
            }
            var w = Vector3.Dot(ab, aoxd);
            if (w < 0 || v + w > dn)
            {
                //Invalid barycentric coordinate for b and/or c.
                return false;
            }
            t /= dn;
            normal /= (float)Math.Sqrt(Vector3.Dot(normal, normal));
            return true;
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
        {
            var offset = origin - pose.Position;
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(offset, orientation, out var localOffset);
            Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);
            if (RayTest(A, B, C, localOffset, localDirection, out t, out normal))
            {
                Matrix3x3.Transform(normal, orientation, out normal);
                return true;
            }
            return false;
        }

        public void ComputeInertia(float mass, out BodyInertia inertia)
        {
            MeshInertiaHelper.ComputeTriangleContribution(A, B, C, mass, out var inertiaTensor);
            Symmetric3x3.Invert(inertiaTensor, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / mass;
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<Triangle, TriangleWide>(pool, initialCapacity);
        }

        /// <summary>
        /// Type id of triangle shapes.
        /// </summary>
        public const int Id = 3;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }


    public struct TriangleWide : IShapeWide<Triangle>
    {
        public Vector3Wide A;
        public Vector3Wide B;
        public Vector3Wide C;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Broadcast(in Triangle shape)
        {
            Vector3Wide.Broadcast(shape.A, out A);
            Vector3Wide.Broadcast(shape.B, out B);
            Vector3Wide.Broadcast(shape.C, out C);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteFirst(in Triangle source)
        {
            Vector3Wide.WriteFirst(source.A, ref A);
            Vector3Wide.WriteFirst(source.B, ref B);
            Vector3Wide.WriteFirst(source.C, ref C);
        }

        public bool AllowOffsetMemoryAccess => true;
        public int InternalAllocationSize => 0;
        public void Initialize(in RawBuffer memory) { }

        /// <summary>
        /// Provides an estimate of the scale of a shape. 
        /// </summary>
        /// <param name="epsilonScale">Approximate scale of the shape for use in epsilons.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EstimateEpsilonScale(out Vector<float> epsilonScale)
        {
            var minX = Vector.Min(A.X, Vector.Min(B.X, C.X));
            var maxX = Vector.Max(A.X, Vector.Max(B.X, C.X));
            var minY = Vector.Min(A.Y, Vector.Min(B.Y, C.Y));
            var maxY = Vector.Max(A.Y, Vector.Max(B.Y, C.Y));
            var minZ = Vector.Min(A.Z, Vector.Min(B.Z, C.Z));
            var maxZ = Vector.Max(A.Z, Vector.Max(B.Z, C.Z));
            epsilonScale = Vector.Max(maxX - minX, Vector.Max(maxY - minY, maxZ - minZ));
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteSlot(int index, in Triangle source)
        {
            GatherScatter.GetOffsetInstance(ref this, index).WriteFirst(source);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientations, out var basis);
            Matrix3x3Wide.TransformWithoutOverlap(A, basis, out var worldA);
            Matrix3x3Wide.TransformWithoutOverlap(B, basis, out var worldB);
            Matrix3x3Wide.TransformWithoutOverlap(C, basis, out var worldC);
            min.X = Vector.Min(worldA.X, Vector.Min(worldB.X, worldC.X));
            min.Y = Vector.Min(worldA.Y, Vector.Min(worldB.Y, worldC.Y));
            min.Z = Vector.Min(worldA.Z, Vector.Min(worldB.Z, worldC.Z));
            max.X = Vector.Max(worldA.X, Vector.Max(worldB.X, worldC.X));
            max.Y = Vector.Max(worldA.Y, Vector.Max(worldB.Y, worldC.Y));
            max.Z = Vector.Max(worldA.Z, Vector.Max(worldB.Z, worldC.Z));

            Vector3Wide.LengthSquared(A, out var aLengthSquared);
            Vector3Wide.LengthSquared(B, out var bLengthSquared);
            Vector3Wide.LengthSquared(C, out var cLengthSquared);
            maximumRadius = Vector.SquareRoot(Vector.Max(aLengthSquared, Vector.Max(bLengthSquared, cLengthSquared)));
            maximumAngularExpansion = maximumRadius;
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
        public static void RayTest(ref Vector3Wide a, ref Vector3Wide b, ref Vector3Wide c, ref Vector3Wide origin, ref Vector3Wide direction, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            //Note that this assumes clockwise winding. Rays coming from the opposite direction pass through; triangles are one sided.
            Vector3Wide.Subtract(b, a, out var ab);
            Vector3Wide.Subtract(c, a, out var ac);
            Vector3Wide.CrossWithoutOverlap(ac, ab, out normal);
            Vector3Wide.Dot(direction, normal, out var dn);
            dn = -dn;
            Vector3Wide.Subtract(origin, a, out var ao);
            Vector3Wide.Dot(ao, normal, out t);
            t /= dn;
            Vector3Wide.CrossWithoutOverlap(ao, direction, out var aoxd);
            Vector3Wide.Dot(ac, aoxd, out var v);
            v = -v;
            Vector3Wide.Dot(ab, aoxd, out var w);
            Vector3Wide.Normalize(normal, out normal);
            intersected = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.GreaterThan(dn, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(t, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.BitwiseAnd(
                        Vector.GreaterThanOrEqual(v, Vector<float>.Zero),
                        Vector.GreaterThanOrEqual(w, Vector<float>.Zero)),
                    Vector.LessThanOrEqual(v + w, dn)));
        }
        public void RayTest(ref RigidPoses pose, ref RayWide ray, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            Vector3Wide.Subtract(ray.Origin, pose.Position, out var offset);
            Matrix3x3Wide.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offset, orientation, out var localOffset);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ray.Direction, orientation, out var localDirection);
            RayTest(ref A, ref B, ref C, ref localOffset, ref localDirection, out intersected, out t, out var localNormal);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientation, out normal);
        }
    }

    public struct TriangleSupportFinder : ISupportFinder<Triangle, TriangleWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in TriangleWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in TriangleWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, terminatedLanes, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeLocalSupport(in TriangleWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Vector3Wide.Dot(shape.A, direction, out var a);
            Vector3Wide.Dot(shape.B, direction, out var b);
            Vector3Wide.Dot(shape.C, direction, out var c);
            var max = Vector.Max(a, Vector.Max(b, c));
            Vector3Wide.ConditionalSelect(Vector.Equals(max, a), shape.A, shape.B, out support);
            Vector3Wide.ConditionalSelect(Vector.Equals(max, c), shape.C, support, out support);
        }
    }
}
