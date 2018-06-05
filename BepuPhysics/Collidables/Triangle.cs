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
    /// Collision shape representing an individual triangle.
    /// </summary>
    public struct Triangle : IConvexShape
    {
        public Vector3 A;
        public Vector3 B;
        public Vector3 C;

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
            Vector3x.Cross(ac, ab, out normal);
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
            Vector3x.Cross(ao, direction, out var aoxd);
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
            //This follows the same logic as the tetrahedral inertia tensor calculation, but the transform is different.
            //There are only two dimensions of interest, but if we wanted to express it as a 3x3 linear transform:
            // [ B - A ]
            // [   N   ]
            // [ C - A ]
            //where N = (ab x ac) / ||ab x ac||.
            //In other words, this transform maintains the plane normal such that you can compute the scaled triangle area using (ab x ac) * N.
            //In practice, that normal won't actually appear in our calculations because we were given the mass explicitly rather than integrating it from density across the area.
            //So, putting that together and assuming the scaling term is pulled out, here's a chunk of code you can plop into wolfram cloud and whatnot to recreate the results:
            //f[{x_, y_, z_}] := {{y^2 + z^2, -x * y, -x * z}, {-x * y, x^2 + z^2, -y * z}, {-x * z, -y * z, x^2 + y^2}}
            //a = { ax, ay, az };
            //b = { bx, by, bz };
            //c = { cx, cy, cz };
            //ab = b - a;
            //ac = c - a;
            //n = Cross[ab, ac] / Length[Cross[ab, ac]];
            //A = { ab, n, ac };
            //result = Integrate[Integrate[f[{ i, 0, k}.A + a], {k, 0, 1-i}], {i, 0, 1}];
            inertia.InverseMass = 1f / mass;
            var ab = B - A;
            var ac = C - A;
            //Revisiting the determinant, note that:
            //density * abs(determinant) = density * volume * 2 = mass * 2
            //So there's no need to actually compute the determinant/area since we were given the mass directly.
            var diagonalScaling = mass * (2f / 12f);
            Triangular3x3 inertiaTensor;
            inertiaTensor.XX = diagonalScaling * (
                A.Y * A.Y + A.Z * A.Z + B.Y * B.Y + B.Z * B.Z + C.Y * C.Y + C.Z * C.Z +
                A.Y * B.Y + A.Z * B.Z + A.Y * C.Y + B.Y * C.Y + A.Z * C.Z + B.Z * C.Z);
            inertiaTensor.YY = diagonalScaling * (
                A.X * A.X + A.Z * A.Z + B.X * B.X + B.Z * B.Z + C.X * C.X + C.Z * C.Z +
                A.X * B.X + A.Z * B.Z + A.X * C.X + B.X * C.X + A.Z * C.Z + B.Z * C.Z);
            inertiaTensor.ZZ = diagonalScaling * (
                A.X * A.X + A.Y * A.Y + B.X * B.X + B.Y * B.Y + C.X * C.X + C.Y * C.Y +
                A.X * B.X + A.Y * B.Y + A.X * C.X + B.X * C.X + A.Y * C.Y + B.Y * C.Y);
            var offScaling = mass * (2f / 24f);
            inertiaTensor.YX = offScaling * (-A.Y * (B.X + C.X) - B.Y * (2 * B.X + C.X) - (B.X + 2 * C.X) * C.Y - A.X * (2 * A.Y + B.Y + C.Y));
            inertiaTensor.ZX = offScaling * (-A.Z * (B.X + C.X) - B.Z * (2 * B.X + C.X) - (B.X + 2 * C.X) * C.Z - A.X * (2 * A.Z + B.Z + C.Z));
            inertiaTensor.ZY = offScaling * (-A.Z * (B.Y + C.Y) - B.Z * (2 * B.Y + C.Y) - (B.Y + 2 * C.Y) * C.Z - A.Y * (2 * A.Z + B.Z + C.Z));
            //TODO: Note that the above implementation isn't exactly optimal. Assuming for now that the performance isn't going to be relevant.
            //That could change given certain convex hull use cases, but in that situation you should probably just jump to vectorizing over multiple tetrahedra at a time.
            //(Plus some basic term caching.)
            Triangular3x3.SymmetricInvert(inertiaTensor, out inertia.InverseInertiaTensor);
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<Triangle, TriangleWide>(pool, initialCapacity);
        }

        /// <summary>
        /// Type id of triangle shapes.
        /// </summary>
        public const int Id = 5;
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
        public void Gather(ref Triangle source)
        {
            Vector3Wide.WriteFirst(ref source.A, ref A);
            Vector3Wide.WriteFirst(ref source.B, ref B);
            Vector3Wide.WriteFirst(ref source.C, ref C);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            Matrix3x3Wide.CreateFromQuaternion(ref orientations, out var basis);
            Matrix3x3Wide.TransformWithoutOverlap(ref A, ref basis, out var worldA);
            Matrix3x3Wide.TransformWithoutOverlap(ref B, ref basis, out var worldB);
            Matrix3x3Wide.TransformWithoutOverlap(ref C, ref basis, out var worldC);
            min.X = Vector.Min(worldA.X, Vector.Min(worldB.X, worldC.X));
            min.Y = Vector.Min(worldA.Y, Vector.Min(worldB.Y, worldC.Y));
            min.Z = Vector.Min(worldA.Z, Vector.Min(worldB.Z, worldC.Z));
            max.X = Vector.Max(worldA.X, Vector.Max(worldB.X, worldC.X));
            max.Y = Vector.Max(worldA.Y, Vector.Max(worldB.Y, worldC.Y));
            max.Z = Vector.Max(worldA.Z, Vector.Max(worldB.Z, worldC.Z));

            Vector3Wide.LengthSquared(ref A, out var aLengthSquared);
            Vector3Wide.LengthSquared(ref B, out var bLengthSquared);
            Vector3Wide.LengthSquared(ref C, out var cLengthSquared);
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
            Vector3Wide.Subtract(ref b, ref a, out var ab);
            Vector3Wide.Subtract(ref c, ref a, out var ac);
            Vector3Wide.CrossWithoutOverlap(ref ac, ref ab, out normal);
            Vector3Wide.Dot(ref direction, ref normal, out var dn);
            dn = -dn;
            Vector3Wide.Subtract(ref origin, ref a, out var ao);
            Vector3Wide.Dot(ref ao, ref normal, out t);
            t /= dn;
            Vector3Wide.CrossWithoutOverlap(ref ao, ref direction, out var aoxd);
            Vector3Wide.Dot(ref ac, ref aoxd, out var v);
            v = -v;
            Vector3Wide.Dot(ref ab, ref aoxd, out var w);
            Vector3Wide.Normalize(ref normal, out normal);
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
            Vector3Wide.Subtract(ref ray.Origin, ref pose.Position, out var offset);
            Matrix3x3Wide.CreateFromQuaternion(ref pose.Orientation, out var orientation);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offset, ref orientation, out var localOffset);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref ray.Direction, ref orientation, out var localDirection);
            RayTest(ref A, ref B, ref C, ref localOffset, ref localDirection, out intersected, out t, out var localNormal);
            Matrix3x3Wide.TransformWithoutOverlap(ref localNormal, ref orientation, out normal);
        }
    }
}
