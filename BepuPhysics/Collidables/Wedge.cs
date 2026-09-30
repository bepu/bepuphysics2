using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Collision shape representing a wedge, created by extruding a right triangle in the XY plane along the Z axis.
    /// </summary>
    /// <remarks>
    /// Its centroid is positioned at the local origin.
    /// This means that the shape has local extents of (-Width/3, -Height/3, -HalfLength) to (2*Width/3, 2*Height/3, HalfLength).
    /// </remarks>
    public struct Wedge : IConvexShape
    {
        /// <summary>
        /// The width of the triangle along the X axis.
        /// </summary>
        public float Width;
        /// <summary>
        /// The height of the triangle along the Y axis.
        /// </summary>
        public float Height;
        /// <summary>
        /// Half of the length of the prism along the Z axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// The length of the prism along the Z axis.
        /// </summary>
        public float Length { readonly get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        private const float OneThird = 1f / 3f;
        private const float TwoThirds = 2f / 3f;

        public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var basis);
            Matrix3x3.Transform(new Vector3(-OneThird * Width, -OneThird * Height, 0), basis, out var worldA);
            Matrix3x3.Transform(new Vector3(TwoThirds * Width, -OneThird * Height, 0), basis, out var worldB);
            Matrix3x3.Transform(new Vector3(-OneThird * Width, TwoThirds * Height, 0), basis, out var worldC);

            var extrusion = Vector3.Abs(HalfLength * basis.Z);
            min = Vector3.Min(worldA, Vector3.Min(worldB, worldC)) - extrusion;
            max = Vector3.Max(worldA, Vector3.Max(worldB, worldC)) + extrusion;
        }

        public readonly void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            // The farthest point is one of the triangle's vertices at either end of the extrusion.
            var widthSquared = Width * Width;
            var heightSquared = Height * Height;
            var baseRadiusSquared = HalfLength * HalfLength + OneThird * OneThird * float.Max(4 * widthSquared + heightSquared, widthSquared + 4 * heightSquared);
            maximumRadius = float.Sqrt(baseRadiusSquared);

            // The closest surface is either an end cap or the triangular prism's hypotenuse side.
            var endCapDistance = HalfLength;
            var hypotenuseDistance = OneThird * Width * Height / float.Sqrt(widthSquared + heightSquared);
            var minimumRadius = float.Min(endCapDistance, hypotenuseDistance);
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        public readonly BodyInertia ComputeInertia(float mass)
        {
            var widthSquared = Width * Width;
            var heightSquared = Height * Height;
            var halfLengthSquared = HalfLength * HalfLength;

            // For a uniform right triangle, the centroidal second moments of
            // area are the classic bh^3/36 results divided by area, giving:
            //   E[x^2] = Width^2  / 18
            //   E[y^2] = Height^2 / 18
            //   E[xy]  = -Width * Height / 36
            // (E[xy] is nonzero because a right triangle isn't symmetric about
            // either centroidal axis - unlike, say, an isosceles triangle.)
            //
            // Extruding along Z adds a uniform-rod contribution to E[z^2]:
            // for a rod of length L centered at the origin, E[z^2] = L^2 / 12.
            // Since HalfLength = L / 2, that's halfLengthSquared / 3.
            //
            // The inertia tensor's diagonal entries are mass * (E[other-axes^2]),
            // and off-diagonal entries are -mass * E[xy] (standard inertia-tensor
            // sign convention), which is why YX below comes out positive despite
            // E[xy] itself being negative. ZX and ZY are zero because the prism
            // is symmetric front-to-back along Z.
            Symmetric3x3 inertiaTensor;
            inertiaTensor.XX = mass * (heightSquared / 18f + halfLengthSquared / 3f);
            inertiaTensor.YX = mass * Width * Height / 36f;
            inertiaTensor.YY = mass * (widthSquared / 18f + halfLengthSquared / 3f);
            inertiaTensor.ZX = 0;
            inertiaTensor.ZY = 0;
            inertiaTensor.ZZ = mass * (widthSquared + heightSquared) / 18f;

            BodyInertia inertia;
            Symmetric3x3.Invert(inertiaTensor, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / mass;
            return inertia;
        }

        public readonly bool RayTest(in RigidPose pose, Vector3 origin, Vector3 direction, out float t, out Vector3 normal)
        {
            var offset = origin - pose.Position;
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(offset, orientation, out var localOrigin);
            Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);

            // The prism is the intersection of the five half spaces bounded by
            // its two triangular sides and its two extrusion caps.
            var latestEntry = -float.MaxValue;
            var earliestExit = float.MaxValue;
            var latestEntryNormal = default(Vector3);

            ProcessPlane(new Vector3(-1, 0, 0), Width * OneThird);
            ProcessPlane(new Vector3(0, -1, 0), Height * OneThird);
            ProcessPlane(new Vector3(Height, Width, 0), Width * Height * OneThird);
            ProcessPlane(new Vector3(0, 0, -1), HalfLength);
            ProcessPlane(new Vector3(0, 0, 1), HalfLength);

            if (earliestExit < 0 || latestEntry > earliestExit)
            {
                t = 0;
                normal = default;
                return false;
            }

            t = latestEntry >= 0 ? latestEntry : 0;
            Matrix3x3.Transform(latestEntryNormal, orientation, out normal);
            return true;

            void ProcessPlane(Vector3 planeNormal, float planeOffset)
            {
                const float Epsilon = 1e-8f;
                var denominator = Vector3.Dot(planeNormal, localDirection);
                var numerator = planeOffset - Vector3.Dot(planeNormal, localOrigin);
                if (float.Abs(denominator) < Epsilon)
                {
                    denominator = denominator < 0 ? -Epsilon : Epsilon;
                }

                var planeT = numerator / denominator;
                if (denominator > 0)
                {
                    if (planeT < earliestExit)
                        earliestExit = planeT;
                }
                else if (planeT > latestEntry)
                {
                    latestEntry = planeT;
                    latestEntryNormal = Vector3.Normalize(planeNormal);
                }
            }
        }

        public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<Wedge, WedgeWide>(pool, initialCapacity);
        }

        public const int Id = 11;
        public static int TypeId => Id;
    }


    public struct WedgeWide : IShapeWide<Wedge>
    {
        public Vector<float> Width;
        public Vector<float> Height;
        public Vector<float> HalfLength;

        public readonly bool AllowOffsetMemoryAccess => true;
        public readonly int InternalAllocationSize => 0;
        public static int MinimumWideRayCount => 3;

        public void Broadcast(in Wedge shape)
        {
            Width = new Vector<float>(shape.Width);
            Height = new Vector<float>(shape.Height);
            HalfLength = new Vector<float>(shape.HalfLength);
        }

        public void WriteFirst(in Wedge source)
        {
            Unsafe.As<Vector<float>, float>(ref Width) = source.Width;
            Unsafe.As<Vector<float>, float>(ref Height) = source.Height;
            Unsafe.As<Vector<float>, float>(ref HalfLength) = source.HalfLength;
        }

        public void WriteSlot(int index, in Wedge source)
        {
            GatherScatter.GetOffsetInstance(ref this, index).WriteFirst(source);
        }

        public readonly void Initialize(in Buffer<byte> memory)
        {
        }

        public readonly void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientations, out var basis);
            var oneThird = new Vector<float>(1f / 3f);
            var twoThirds = new Vector<float>(2f / 3f);
            var negativeOneThird = new Vector<float>(-1f / 3f);

            Vector3Wide localA;
            localA.X = negativeOneThird * Width;
            localA.Y = negativeOneThird * Height;
            localA.Z = Vector<float>.Zero;
            Matrix3x3Wide.TransformWithoutOverlap(localA, basis, out var worldA);

            Vector3Wide localB;
            localB.X = twoThirds * Width;
            localB.Y = negativeOneThird * Height;
            localB.Z = Vector<float>.Zero;
            Matrix3x3Wide.TransformWithoutOverlap(localB, basis, out var worldB);

            Vector3Wide localC;
            localC.X = negativeOneThird * Width;
            localC.Y = twoThirds * Height;
            localC.Z = Vector<float>.Zero;
            Matrix3x3Wide.TransformWithoutOverlap(localC, basis, out var worldC);

            var extrusionX = Vector.Abs(HalfLength * basis.Z.X);
            var extrusionY = Vector.Abs(HalfLength * basis.Z.Y);
            var extrusionZ = Vector.Abs(HalfLength * basis.Z.Z);
            min.X = Vector.Min(worldA.X, Vector.Min(worldB.X, worldC.X)) - extrusionX;
            min.Y = Vector.Min(worldA.Y, Vector.Min(worldB.Y, worldC.Y)) - extrusionY;
            min.Z = Vector.Min(worldA.Z, Vector.Min(worldB.Z, worldC.Z)) - extrusionZ;
            max.X = Vector.Max(worldA.X, Vector.Max(worldB.X, worldC.X)) + extrusionX;
            max.Y = Vector.Max(worldA.Y, Vector.Max(worldB.Y, worldC.Y)) + extrusionY;
            max.Z = Vector.Max(worldA.Z, Vector.Max(worldB.Z, worldC.Z)) + extrusionZ;

            var widthSquared = Width * Width;
            var heightSquared = Height * Height;
            var radiusSquared = HalfLength * HalfLength + oneThird * oneThird * Vector.Max(4 * widthSquared + heightSquared, widthSquared + 4 * heightSquared);
            maximumRadius = Vector.SquareRoot(radiusSquared);

            var hypotenuseDistance = oneThird * Width * Height / Vector.SquareRoot(widthSquared + heightSquared);
            var minimumRadius = Vector.Min(HalfLength, hypotenuseDistance);
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        public void RayTest(ref RigidPoseWide poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            const float Epsilon = 1e-8f;

            Vector3Wide.Subtract(rayWide.Origin, poses.Position, out var offset);
            Matrix3x3Wide.CreateFromQuaternion(poses.Orientation, out var orientation);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offset, orientation, out var localOrigin);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(rayWide.Direction, orientation, out var localDirection);

            var oneThird = new Vector<float>(1f / 3f);
            var latestEntry = new Vector<float>(-float.MaxValue);
            var earliestExit = new Vector<float>(float.MaxValue);
            var latestEntryNormal = default(Vector3Wide);

            ProcessPlane(
                new Vector3Wide { X = -Vector<float>.One, Y = Vector<float>.Zero, Z = Vector<float>.Zero },
                Width * oneThird);
            ProcessPlane(
                new Vector3Wide { X = Vector<float>.Zero, Y = -Vector<float>.One, Z = Vector<float>.Zero },
                Height * oneThird);
            ProcessPlane(
                new Vector3Wide { X = Height, Y = Width, Z = Vector<float>.Zero },
                Width * Height * oneThird);
            ProcessPlane(
                new Vector3Wide { X = Vector<float>.Zero, Y = Vector<float>.Zero, Z = -Vector<float>.One },
                HalfLength);
            ProcessPlane(
                new Vector3Wide { X = Vector<float>.Zero, Y = Vector<float>.Zero, Z = Vector<float>.One },
                HalfLength);

            intersected = (earliestExit >= Vector<float>.Zero) & (latestEntry <= earliestExit);
            t = Vector.ConditionalSelect(intersected, Vector.Max(latestEntry, Vector<float>.Zero), Vector<float>.Zero);

            Vector3Wide.Length(latestEntryNormal, out var normalLength);
            Vector3Wide.Scale(latestEntryNormal, Vector<float>.One / Vector.Max(normalLength, new Vector<float>(Epsilon)), out var localNormal);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientation, out normal);
            normal.X = Vector.ConditionalSelect(intersected, normal.X, Vector<float>.Zero);
            normal.Y = Vector.ConditionalSelect(intersected, normal.Y, Vector<float>.Zero);
            normal.Z = Vector.ConditionalSelect(intersected, normal.Z, Vector<float>.Zero);

            void ProcessPlane(Vector3Wide planeNormal, Vector<float> planeOffset)
            {
                Vector3Wide.Dot(localOrigin, planeNormal, out var normalDotOrigin);
                var numerator = planeOffset - normalDotOrigin;
                Vector3Wide.Dot(localDirection, planeNormal, out var denominator);
                var nearParallel = Vector.Abs(denominator) < new Vector<float>(Epsilon);
                denominator = Vector.ConditionalSelect(
                    nearParallel,
                    Vector.ConditionalSelect(denominator < Vector<float>.Zero, -new Vector<float>(Epsilon), new Vector<float>(Epsilon)),
                    denominator);
                var planeT = numerator / denominator;
                var exits = denominator > Vector<float>.Zero;
                earliestExit = Vector.ConditionalSelect(exits, Vector.Min(earliestExit, planeT), earliestExit);

                var enters = planeT > latestEntry & !exits;
                latestEntry = Vector.ConditionalSelect(enters, planeT, latestEntry);
                latestEntryNormal.X = Vector.ConditionalSelect(enters, planeNormal.X, latestEntryNormal.X);
                latestEntryNormal.Y = Vector.ConditionalSelect(enters, planeNormal.Y, latestEntryNormal.Y);
                latestEntryNormal.Z = Vector.ConditionalSelect(enters, planeNormal.Z, latestEntryNormal.Z);
            }
        }
    }


    public readonly struct WedgeSupportFinder : ISupportFinder<Wedge, WedgeWide>
    {
        public bool HasMargin => false;

        public void GetMargin(in WedgeWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        public void ComputeSupport(in WedgeWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, terminatedLanes, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }

        public void ComputeLocalSupport(in WedgeWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            var ax = shape.Width * (-1f / 3f);
            var ay = shape.Height * (-1f / 3f);
            var bx = shape.Width * (2f / 3f);
            var by = shape.Height * (-1f / 3f);
            var cx = shape.Width * (-1f / 3f);
            var cy = shape.Height * (2f / 3f);

            var a = ax * direction.X + ay * direction.Y;
            var b = bx * direction.X + by * direction.Y;
            var c = cx * direction.X + cy * direction.Y;
            var useB = b > a;
            var useC = c > Vector.Max(a, b);

            support.X = Vector.ConditionalSelect(useC, cx, Vector.ConditionalSelect(useB, bx, ax));
            support.Y = Vector.ConditionalSelect(useC, cy, Vector.ConditionalSelect(useB, by, ay));
            support.Z = Vector.ConditionalSelect(direction.Z < Vector<float>.Zero, -shape.HalfLength, shape.HalfLength);
        }
    }
}
