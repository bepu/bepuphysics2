using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Collision shape representing a cone.
    /// </summary>
    /// <remarks>
    /// This is positioned with its center of mass at the origin.
    /// </remarks>
    public struct Cone : IConvexShape
    {
        public float Radius;
        public float Height;

        public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            QuaternionEx.TransformUnitY(orientation, out var axis);
            var baseCenter = Height * -0.25f * axis;
            var apex = Height * 0.75f * axis;
            var baseExtents = Vector3.SquareRoot(Vector3.Max(Vector3.Zero, Vector3.One - axis * axis)) * Radius;
            min = Vector3.Min(baseCenter - baseExtents, apex);
            max = Vector3.Max(baseCenter + baseExtents, apex);
        }

        public readonly void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            // The farthest point is either the apex or a point on the base rim.
            var baseHeight = Height * 0.25f;
            var baseRadius = float.Sqrt(Radius * Radius + baseHeight * baseHeight);
            var apexRadius = Height * 0.75f;
            maximumRadius = float.Max(baseRadius, apexRadius);

            // The closest surface is either the base or the lateral surface.
            var baseDistance = baseHeight;
            var sideDistance = apexRadius * Radius / float.Sqrt(Radius * Radius + Height * Height);
            var minimumRadius = float.Min(baseDistance, sideDistance);
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        public readonly BodyInertia ComputeInertia(float mass)
        {
            var inverseMass = 1f / mass;
            var radiusSquared = Radius * Radius;
            var heightSquared = Height * Height;
            var transverseInertia = 3f * radiusSquared / 20f + 3f * heightSquared / 80f;
            var axialInertia = 3f * radiusSquared / 10f;

            // For a cone centered at its center of mass, the diagonal moments are:
            // Ixx = m * (3 * Radius^2 / 20 + 3 * Height^2 / 80)
            // Iyy = m * 3 * Radius^2 / 10
            // Izz = m * (3 * Radius^2 / 20 + 3 * Height^2 / 80)
            BodyInertia inertia;
            inertia.InverseMass = inverseMass;
            inertia.InverseInertiaTensor.XX = inverseMass / transverseInertia;
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = inverseMass / axialInertia;
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = inverseMass / transverseInertia;
            return inertia;
        }

        public readonly bool RayTest(in RigidPose pose, Vector3 origin, Vector3 direction, out float t, out Vector3 normal)
        {
            const float Epsilon = 1e-8f;

            // Work in the cone's local space, where the axis is Y and the base is at -Height / 4.
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            var offset = origin - pose.Position;
            Matrix3x3.TransformTranspose(offset, orientation, out var localOffset);
            Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);

            // Normalize the direction to improve the numerical stability of the intersection tests.
            var directionLength = localDirection.Length();
            if (directionLength <= Epsilon)
            {
                t = 0;
                normal = Vector3.Zero;
                return false;
            }
            var inverseDirectionLength = 1f / directionLength;
            localDirection *= inverseDirectionLength;

            var baseY = Height * -0.25f;
            var apexY = Height * 0.75f;
            var slopeSquared = Radius * Radius / (Height * Height);
            var hitT = float.MaxValue;
            var hitNormal = Vector3.Zero;
            var hit = false;

            void ConsiderSideHit(float candidateT)
            {
                if (candidateT < 0 || candidateT >= hitT)
                    return;

                var location = localOffset + localDirection * candidateT;
                if (location.Y < baseY || location.Y > apexY)
                    return;

                var sideNormal = new Vector3(location.X, slopeSquared * (apexY - location.Y), location.Z);
                if (sideNormal.LengthSquared() <= 1e-16f)
                    sideNormal = Vector3.UnitY;
                else
                    sideNormal = Vector3.Normalize(sideNormal);

                hitT = candidateT;
                hitNormal = sideNormal;
                hit = true;
            }

            // The lateral surface is x^2 + z^2 = (Radius / Height)^2 * (apexY - y)^2.
            // Substituting the ray equation produces this quadratic in the ray parameter.
            var a = localDirection.X * localDirection.X + localDirection.Z * localDirection.Z - slopeSquared * localDirection.Y * localDirection.Y;
            var q = apexY - localOffset.Y;
            var b = 2f * (localOffset.X * localDirection.X + localOffset.Z * localDirection.Z + slopeSquared * q * localDirection.Y);
            var c = localOffset.X * localOffset.X + localOffset.Z * localOffset.Z - slopeSquared * q * q;

            if (float.Abs(a) > Epsilon)
            {
                // Handle the regular quadratic case. The finite-height check is performed
                // after solving because the equation describes the infinite cone.
                var discriminant = b * b - 4f * a * c;
                if (discriminant >= 0)
                {
                    var root = float.Sqrt(discriminant);
                    var t0 = (-b - root) / (2f * a);
                    var t1 = (-b + root) / (2f * a);
                    if (t1 < t0)
                    {
                        (t1, t0) = (t0, t1);
                    }

                    if (t0 < 0 && t1 >= 0 && c <= 0)
                        t0 = 0;
                    ConsiderSideHit(t0);
                    ConsiderSideHit(t1);
                }
            }
            else if (float.Abs(b) > Epsilon)
            {
                // A ray tangent to the cone's quadratic form reduces to a linear equation.
                ConsiderSideHit(-c / b);
            }

            // Check the base cap.
            if (float.Abs(localDirection.Y) > Epsilon)
            {
                var capT = (baseY - localOffset.Y) / localDirection.Y;
                if (capT >= 0 && capT < hitT)
                {
                    var capLocation = localOffset + localDirection * capT;
                    if (capLocation.X * capLocation.X + capLocation.Z * capLocation.Z <= Radius * Radius)
                    {
                        hitT = capT;
                        hitNormal = -Vector3.UnitY;
                        hit = true;
                    }
                }
            }

            if (!hit)
            {
                t = 0;
                normal = Vector3.Zero;
                return false;
            }

            t = hitT * inverseDirectionLength;
            Matrix3x3.Transform(hitNormal, orientation, out normal);
            return true;
        }

        public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<Cone, ConeWide>(pool, initialCapacity);
        }

        public const int Id = 9;
        public static int TypeId => Id;
    }

    public struct ConeWide : IShapeWide<Cone>
    {
        public Vector<float> Radius;
        public Vector<float> Height;
        public static int MinimumWideRayCount => 3;
        public void Broadcast(in Cone shape)
        {
            Radius = new Vector<float>(shape.Radius);
            Height = new Vector<float>(shape.Height);
        }
        public void WriteFirst(in Cone source)
        {
            Unsafe.As<Vector<float>, float>(ref Radius) = source.Radius;
            Unsafe.As<Vector<float>, float>(ref Height) = source.Height;
        }

        public readonly void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            var axis = QuaternionWide.TransformUnitY(orientations);
            Vector3Wide.Multiply(axis, axis, out var axisSquared);
            Vector3Wide.Subtract(Vector<float>.One, axisSquared, out var squaredBaseExtents);

            var negativeBaseHeight = Height * new Vector<float>(-0.25f);
            var apexHeight = Height * new Vector<float>(0.75f);
            var baseCenterX = negativeBaseHeight * axis.X;
            var baseCenterY = negativeBaseHeight * axis.Y;
            var baseCenterZ = negativeBaseHeight * axis.Z;
            var apexX = apexHeight * axis.X;
            var apexY = apexHeight * axis.Y;
            var apexZ = apexHeight * axis.Z;
            var baseExtentX = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, squaredBaseExtents.X)) * Radius;
            var baseExtentY = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, squaredBaseExtents.Y)) * Radius;
            var baseExtentZ = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, squaredBaseExtents.Z)) * Radius;

            min.X = Vector.Min(baseCenterX - baseExtentX, apexX);
            min.Y = Vector.Min(baseCenterY - baseExtentY, apexY);
            min.Z = Vector.Min(baseCenterZ - baseExtentZ, apexZ);
            max.X = Vector.Max(baseCenterX + baseExtentX, apexX);
            max.Y = Vector.Max(baseCenterY + baseExtentY, apexY);
            max.Z = Vector.Max(baseCenterZ + baseExtentZ, apexZ);

            var baseHeight = Height * new Vector<float>(0.25f);
            var baseRadius = Vector.SquareRoot(Radius * Radius + baseHeight * baseHeight);
            maximumRadius = Vector.Max(baseRadius, apexHeight);
            var sideDistance = apexHeight * Radius / Vector.SquareRoot(Radius * Radius + Height * Height);
            var minimumRadius = Vector.Min(baseHeight, sideDistance);
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        public readonly bool AllowOffsetMemoryAccess => true;
        public readonly int InternalAllocationSize => 0;
        public readonly void Initialize(in Buffer<byte> memory) { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteSlot(int index, in Cone source)
        {
            GatherScatter.GetOffsetInstance(ref this, index).WriteFirst(source);
        }

        public readonly void RayTest(ref RigidPoseWide poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            const float Epsilon = 1e-8f;
            var epsilonWide = new Vector<float>(Epsilon);

            // Transform each ray into the cone's local space, where the axis is Y.
            Matrix3x3Wide.CreateFromQuaternion(poses.Orientation, out var orientation);
            Vector3Wide.Subtract(rayWide.Origin, poses.Position, out var worldOffset);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(worldOffset, orientation, out var localOffset);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(rayWide.Direction, orientation, out var localDirection);

            Vector3Wide.Length(localDirection, out var directionLength);
            var validDirection = directionLength > epsilonWide;
            var safeDirectionLength = Vector.Max(directionLength, epsilonWide);
            var inverseDirectionLength = Vector<float>.One / safeDirectionLength;
            Vector3Wide.Scale(localDirection, inverseDirectionLength, out localDirection);

            var baseY = Height * new Vector<float>(-0.25f);
            var apexY = Height * new Vector<float>(0.75f);
            var slopeSquared = Radius * Radius / (Height * Height);

            // Substituting the ray into x^2 + z^2 = slopeSquared * (apexY - y)^2
            // produces a quadratic for the infinite cone. The Y-range checks below
            // restrict its solutions to the finite lateral surface.
            var q = apexY - localOffset.Y;
            var a = localDirection.X * localDirection.X + localDirection.Z * localDirection.Z - slopeSquared * localDirection.Y * localDirection.Y;
            var b = 2f * (localOffset.X * localDirection.X + localOffset.Z * localDirection.Z + slopeSquared * q * localDirection.Y);
            var c = localOffset.X * localOffset.X + localOffset.Z * localOffset.Z - slopeSquared * q * q;

            var quadratic = Vector.Abs(a) > epsilonWide;
            var discriminant = b * b - 4f * a * c;
            var hasDiscriminant = discriminant >= Vector<float>.Zero;
            var squareRoot = Vector.SquareRoot(Vector.Max(discriminant, Vector<float>.Zero));
            var quadraticDenominator = 2f * a;
            var safeQuadraticDenominator = Vector.ConditionalSelect(quadratic, quadraticDenominator, Vector<float>.One);
            var quadraticT0 = (-b - squareRoot) / safeQuadraticDenominator;
            var quadraticT1 = (-b + squareRoot) / safeQuadraticDenominator;

            var linear = Vector.Abs(b) > epsilonWide;
            var safeLinearDenominator = Vector.ConditionalSelect(linear, b, Vector<float>.One);
            var linearT = -c / safeLinearDenominator;
            var t0 = Vector.ConditionalSelect(quadratic, Vector.Min(quadraticT0, quadraticT1), linearT);
            var t1 = Vector.ConditionalSelect(quadratic, Vector.Max(quadraticT0, quadraticT1), linearT);
            var validSurfaceEquation = (quadratic & hasDiscriminant) | (!quadratic & linear);

            // A ray beginning inside the infinite cone uses an immediate lateral hit.
            // Invalid lanes are masked out before selecting the earliest candidate.
            var insideSurface = (t0 < Vector<float>.Zero) & (t1 >= Vector<float>.Zero) & (c <= Vector<float>.Zero);
            t0 = Vector.ConditionalSelect(insideSurface, Vector<float>.Zero, t0);

            Vector3Wide.Scale(localDirection, t0, out var sideLocation0Offset);
            Vector3Wide.Add(localOffset, sideLocation0Offset, out var sideLocation0);
            Vector3Wide.Scale(localDirection, t1, out var sideLocation1Offset);
            Vector3Wide.Add(localOffset, sideLocation1Offset, out var sideLocation1);
            var side0InBounds = (sideLocation0.Y >= baseY) & (sideLocation0.Y <= apexY);
            var side1InBounds = (sideLocation1.Y >= baseY) & (sideLocation1.Y <= apexY);
            var side0Valid = validDirection & validSurfaceEquation & (t0 >= Vector<float>.Zero) & side0InBounds;
            var side1Valid = validDirection & validSurfaceEquation & (t1 >= Vector<float>.Zero) & side1InBounds;
            var sideUse0 = t0 <= t1;
            var sideT = Vector.ConditionalSelect(sideUse0, t0, t1);
            var sideValid = side0Valid | side1Valid;
            sideT = Vector.ConditionalSelect(side0Valid & sideUse0, t0, sideT);
            sideT = Vector.ConditionalSelect(side1Valid & !side0Valid, t1, sideT);
            sideT = Vector.ConditionalSelect(sideValid, sideT, new Vector<float>(float.PositiveInfinity));

            var sideLocationT = Vector.ConditionalSelect(sideValid, sideT, Vector<float>.Zero);
            Vector3Wide.Scale(localDirection, sideLocationT, out var sideLocationOffset);
            Vector3Wide.Add(localOffset, sideLocationOffset, out var sideLocation);
            sideLocation.Y = apexY - sideLocation.Y;
            var sideNormal = sideLocation;
            sideNormal.Y *= slopeSquared;
            Vector3Wide.Length(sideNormal, out var sideNormalLength);
            Vector3Wide.Scale(sideNormal, Vector<float>.One / Vector.Max(sideNormalLength, epsilonWide), out sideNormal);
            sideNormal.X = Vector.ConditionalSelect(sideNormalLength < epsilonWide, Vector<float>.Zero, sideNormal.X);
            sideNormal.Y = Vector.ConditionalSelect(sideNormalLength < epsilonWide, Vector<float>.One, sideNormal.Y);
            sideNormal.Z = Vector.ConditionalSelect(sideNormalLength < epsilonWide, Vector<float>.Zero, sideNormal.Z);

            // Test the circular base cap and select it only when it is closer than
            // the valid lateral-surface candidate.
            var safeDirectionY = Vector.ConditionalSelect(Vector.Abs(localDirection.Y) > epsilonWide, localDirection.Y, Vector<float>.One);
            var capT = (baseY - localOffset.Y) / safeDirectionY;
            Vector3Wide.Scale(localDirection, capT, out var capLocationOffset);
            Vector3Wide.Add(localOffset, capLocationOffset, out var capLocation);
            var capValid = validDirection & (Vector.Abs(localDirection.Y) > epsilonWide) & (capT >= Vector<float>.Zero) & (capLocation.X * capLocation.X + capLocation.Z * capLocation.Z <= Radius * Radius);
            var useCap = capValid & (capT < sideT);
            var hit = sideValid | capValid;
            var hitT = Vector.ConditionalSelect(useCap, capT, sideT);
            t = Vector.ConditionalSelect(hit, hitT * inverseDirectionLength, Vector<float>.Zero);
            intersected = hit;

            normal.X = Vector.ConditionalSelect(useCap, Vector<float>.Zero, sideNormal.X);
            normal.Y = Vector.ConditionalSelect(useCap, -Vector<float>.One, sideNormal.Y);
            normal.Z = Vector.ConditionalSelect(useCap, Vector<float>.Zero, sideNormal.Z);
            Matrix3x3Wide.TransformWithoutOverlap(normal, orientation, out normal);
            normal.X = Vector.ConditionalSelect(intersected, normal.X, Vector<float>.Zero);
            normal.Y = Vector.ConditionalSelect(intersected, normal.Y, Vector<float>.Zero);
            normal.Z = Vector.ConditionalSelect(intersected, normal.Z, Vector<float>.Zero);
        }
    }


    public readonly struct ConeSupportFinder : ISupportFinder<Cone, ConeWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in ConeWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        public void ComputeSupport(in ConeWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, terminatedLanes, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }

        public void ComputeLocalSupport(in ConeWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            const float Epsilon = 1e-8f;
            var baseY = shape.Height * new Vector<float>(-0.25f);
            var apexY = shape.Height * new Vector<float>(0.75f);
            var horizontalLength = Vector.SquareRoot(direction.X * direction.X + direction.Z * direction.Z);
            var horizontalScale = shape.Radius / Vector.Max(horizontalLength, new Vector<float>(Epsilon));
            var useHorizontal = horizontalLength > new Vector<float>(Epsilon);
            var baseX = Vector.ConditionalSelect(useHorizontal, direction.X * horizontalScale, Vector<float>.Zero);
            var baseZ = Vector.ConditionalSelect(useHorizontal, direction.Z * horizontalScale, Vector<float>.Zero);
            var baseDot = baseX * direction.X + baseY * direction.Y + baseZ * direction.Z;
            var apexDot = apexY * direction.Y;
            var useApex = apexDot >= baseDot;

            support.X = Vector.ConditionalSelect(useApex, Vector<float>.Zero, baseX);
            support.Y = Vector.ConditionalSelect(useApex, apexY, baseY);
            support.Z = Vector.ConditionalSelect(useApex, Vector<float>.Zero, baseZ);
        }
    }
}
