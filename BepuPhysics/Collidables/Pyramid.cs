using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Collision shape representing a rectangular pyramid.
    /// </summary>
    /// <remarks>
    /// This is positioned with its center of mass at the origin.
    /// </remarks>
    public struct Pyramid : IConvexShape
    {
        /// <summary>
        /// Half of the pyramid's width along its local X axis.
        /// </summary>
        public float HalfWidth;
        /// <summary>
        /// Half of the pyramid's length along its local Z axis.
        /// </summary>
        public float HalfLength;
        /// <summary>
        /// The height of the pyramid along its local Y axis.
        /// </summary>
        public float Height;

        public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var basis);
            Matrix3x3.Transform(new Vector3(0, Height * -0.25f, 0), basis, out var baseCenter);
            Matrix3x3.Transform(new Vector3(0, Height * 0.75f, 0), basis, out var apex);

            var baseExtents = Vector3.Abs(HalfWidth * basis.X) + Vector3.Abs(HalfLength * basis.Z);

            min = Vector3.Min(baseCenter - baseExtents, apex);
            max = Vector3.Max(baseCenter + baseExtents, apex);
        }

        public readonly void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            // The farthest point is either a base corner or the apex.
            var baseHeight = Height * 0.25f;
            var baseRadiusSquared = HalfWidth * HalfWidth + HalfLength * HalfLength + baseHeight * baseHeight;
            var apexRadius = Height * 0.75f;
            maximumRadius = float.Sqrt(float.Max(baseRadiusSquared, apexRadius * apexRadius));

            // The closest surface is either the base or one of the two distinct side-plane types.
            // Each side distance is the altitude of a right triangle from the origin to its sloped face.
            var baseDistance = baseHeight;
            var widthSideDistance = apexRadius * HalfWidth / float.Sqrt(HalfWidth * HalfWidth + Height * Height);
            var lengthSideDistance = apexRadius * HalfLength / float.Sqrt(HalfLength * HalfLength + Height * Height);
            var minimumRadius = float.Min(baseDistance, float.Min(widthSideDistance, lengthSideDistance));
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        public readonly BodyInertia ComputeInertia(float mass)
        {
            var inverseMass = 1f / mass;
            var heightSquared = Height * Height;
            var widthSquared = HalfWidth * HalfWidth;
            var lengthSquared = HalfLength * HalfLength;
            var axialHeightContribution = 3f * heightSquared / 80f;

            // For a pyramid centered at its center of mass, the diagonal moments are:
            // Ixx = m * (HalfLength^2 / 5 + 3 * Height^2 / 80)
            // Iyy = m * ((HalfWidth^2 + HalfLength^2) / 5)
            // Izz = m * (HalfWidth^2 / 5 + 3 * Height^2 / 80)
            BodyInertia inertia;
            inertia.InverseMass = inverseMass;
            inertia.InverseInertiaTensor.XX = inverseMass / (lengthSquared / 5f + axialHeightContribution);
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inverseMass / ((widthSquared + lengthSquared) / 5f);
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inverseMass / (widthSquared / 5f + axialHeightContribution);
            return inertia;
        }

        public readonly bool RayTest(in RigidPose pose, Vector3 origin, Vector3 direction, out float t, out Vector3 normal)
        {
            var offset = origin - pose.Position;
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(offset, orientation, out var localOrigin);
            Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);

            // The pyramid is the intersection of the five half spaces bounded by its base and side faces.
            // The face normals point outward and the interior satisfies dot(normal, point) <= offset.
            var baseHeight = Height * 0.25f;
            var sideHeight = Height * 0.75f;
            var latestEntry = -float.MaxValue;
            var earliestExit = float.MaxValue;
            var latestEntryNormal = default(Vector3);

            ProcessPlane(new Vector3(0, -1, 0), baseHeight);
            ProcessPlane(new Vector3(Height, HalfWidth, 0), sideHeight * HalfWidth);
            ProcessPlane(new Vector3(-Height, HalfWidth, 0), sideHeight * HalfWidth);
            ProcessPlane(new Vector3(0, HalfLength, Height), sideHeight * HalfLength);
            ProcessPlane(new Vector3(0, HalfLength, -Height), sideHeight * HalfLength);

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
            return new ConvexShapeBatch<Pyramid, PyramidWide>(pool, initialCapacity);
        }

        public const int Id = 10;
        public static int TypeId => Id;
    }


    public struct PyramidWide : IShapeWide<Pyramid>
    {
        public Vector<float> HalfWidth;
        public Vector<float> HalfLength;
        public Vector<float> Height;

        public readonly bool AllowOffsetMemoryAccess => true;
        public readonly int InternalAllocationSize => 0;

        public static int MinimumWideRayCount => 3;

        public void Broadcast(in Pyramid shape)
        {
            HalfWidth = new Vector<float>(shape.HalfWidth);
            HalfLength = new Vector<float>(shape.HalfLength);
            Height = new Vector<float>(shape.Height);
        }
        public void WriteFirst(in Pyramid source)
        {
            Unsafe.As<Vector<float>, float>(ref HalfWidth) = source.HalfWidth;
            Unsafe.As<Vector<float>, float>(ref HalfLength) = source.HalfLength;
            Unsafe.As<Vector<float>, float>(ref Height) = source.Height;
        }
        public readonly void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientations, out var basis);

            // The rotated base is a rectangle. Its world-space half extents are the sums
            // of the absolute contributions from the local X and Z axes.
            var baseCenterHeight = -Height * new Vector<float>(0.25f);
            var apexHeight = Height * new Vector<float>(0.75f);
            var baseExtentX = Vector.Abs(HalfWidth * basis.X.X) + Vector.Abs(HalfLength * basis.Z.X);
            var baseExtentY = Vector.Abs(HalfWidth * basis.X.Y) + Vector.Abs(HalfLength * basis.Z.Y);
            var baseExtentZ = Vector.Abs(HalfWidth * basis.X.Z) + Vector.Abs(HalfLength * basis.Z.Z);
            var baseCenterX = baseCenterHeight * basis.Y.X;
            var baseCenterY = baseCenterHeight * basis.Y.Y;
            var baseCenterZ = baseCenterHeight * basis.Y.Z;
            var apexX = apexHeight * basis.Y.X;
            var apexY = apexHeight * basis.Y.Y;
            var apexZ = apexHeight * basis.Y.Z;

            min.X = Vector.Min(baseCenterX - baseExtentX, apexX);
            min.Y = Vector.Min(baseCenterY - baseExtentY, apexY);
            min.Z = Vector.Min(baseCenterZ - baseExtentZ, apexZ);
            max.X = Vector.Max(baseCenterX + baseExtentX, apexX);
            max.Y = Vector.Max(baseCenterY + baseExtentY, apexY);
            max.Z = Vector.Max(baseCenterZ + baseExtentZ, apexZ);

            var baseRadiusSquared = HalfWidth * HalfWidth + HalfLength * HalfLength + baseCenterHeight * baseCenterHeight;
            var apexRadiusSquared = apexHeight * apexHeight;
            maximumRadius = Vector.SquareRoot(Vector.Max(baseRadiusSquared, apexRadiusSquared));

            var baseHeight = Height * new Vector<float>(0.25f);
            var widthSideDistance = apexHeight * HalfWidth / Vector.SquareRoot(HalfWidth * HalfWidth + Height * Height);
            var lengthSideDistance = apexHeight * HalfLength / Vector.SquareRoot(HalfLength * HalfLength + Height * Height);
            var minimumRadius = Vector.Min(baseHeight, widthSideDistance, lengthSideDistance);
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        public readonly void Initialize(in Buffer<byte> memory)
        {
        }

        public void WriteSlot(int index, in Pyramid source)
        {
            GatherScatter.GetOffsetInstance(ref this, index).WriteFirst(source);
        }

        public void RayTest(ref RigidPoseWide poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            const float Epsilon = 1e-8f;

            Vector3Wide.Subtract(rayWide.Origin, poses.Position, out var offset);
            Matrix3x3Wide.CreateFromQuaternion(poses.Orientation, out var orientation);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offset, orientation, out var localOrigin);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(rayWide.Direction, orientation, out var localDirection);

            var baseHeight = Height * new Vector<float>(0.25f);
            var sideHeight = Height * new Vector<float>(0.75f);
            var latestEntry = new Vector<float>(-float.MaxValue);
            var earliestExit = new Vector<float>(float.MaxValue);
            var latestEntryNormal = default(Vector3Wide);

            ProcessPlane(
                new Vector3Wide { X = Vector<float>.Zero, Y = -Vector<float>.One, Z = Vector<float>.Zero },
                baseHeight);
            ProcessPlane(
                new Vector3Wide { X = Height, Y = HalfWidth, Z = Vector<float>.Zero },
                sideHeight * HalfWidth);
            ProcessPlane(
                new Vector3Wide { X = -Height, Y = HalfWidth, Z = Vector<float>.Zero },
                sideHeight * HalfWidth);
            ProcessPlane(
                new Vector3Wide { X = Vector<float>.Zero, Y = HalfLength, Z = Height },
                sideHeight * HalfLength);
            ProcessPlane(
                new Vector3Wide { X = Vector<float>.Zero, Y = HalfLength, Z = -Height },
                sideHeight * HalfLength);

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
                    Vector.ConditionalSelect(denominator < Vector<float>.Zero, new Vector<float>(-Epsilon), new Vector<float>(Epsilon)),
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


    public readonly struct PyramidSupportFinder : ISupportFinder<Pyramid, PyramidWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in PyramidWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        public void ComputeSupport(in PyramidWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, terminatedLanes, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }

        public void ComputeLocalSupport(in PyramidWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            var baseY = shape.Height * new Vector<float>(-0.25f);
            var apexY = shape.Height * new Vector<float>(0.75f);

            // The best base corner is selected independently along X and Z.
            var baseX = Vector.ConditionalSelect(direction.X < Vector<float>.Zero, -shape.HalfWidth, shape.HalfWidth);
            var baseZ = Vector.ConditionalSelect(direction.Z < Vector<float>.Zero, -shape.HalfLength, shape.HalfLength);
            var baseDot = baseX * direction.X + baseY * direction.Y + baseZ * direction.Z;
            var apexDot = apexY * direction.Y;
            var useApex = apexDot >= baseDot;

            support.X = Vector.ConditionalSelect(useApex, Vector<float>.Zero, baseX);
            support.Y = Vector.ConditionalSelect(useApex, apexY, baseY);
            support.Z = Vector.ConditionalSelect(useApex, Vector<float>.Zero, baseZ);
        }
    }
}
