using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;
namespace Demos.SpecializedTests
{
    public interface IRayTester<T> where T : IShape
    {
        void GetRandomShape(Random random, out T shape);
        void GetPointInVolume(Random random, float innerMargin, ref T shape, out Vector3 localPointInCapsule);
        void GetSurface(Random random, ref T shape, out Vector3 localPointOnCapsule, out Vector3 localNormal);
        bool PointIsOnSurface(ref T shape, ref Vector3 localPoint);
    }
    public struct SphereRayTester : IRayTester<Sphere>
    {
        public void GetRandomShape(Random random, out Sphere shape)
        {
            const float sizeMin = 0.1f;
            const float sizeSpan = 200;
            shape = new Sphere(sizeMin + sizeSpan * (float)random.NextDouble());
        }
        public void GetPointInVolume(Random random, float innerMargin, ref Sphere shape, out Vector3 localPoint)
        {
            float effectiveRadius = Math.Max(0, shape.Radius - innerMargin);
            float radiusSquared = effectiveRadius * effectiveRadius;
            var min = new Vector3(effectiveRadius, effectiveRadius, effectiveRadius);
            var span = min * 2;
            min = -min;
            do
            {
                localPoint = min + span * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());

            } while (localPoint.LengthSquared() > radiusSquared);
        }

        public void GetSurface(Random random, ref Sphere sphere, out Vector3 localPoint, out Vector3 localNormal)
        {
            RayTesting.GetUnitDirection(random, out localNormal);
            localPoint = localNormal * sphere.Radius;
        }

        public bool PointIsOnSurface(ref Sphere shape, ref Vector3 localPoint)
        {
            var surfaceDistance = localPoint.Length() - shape.Radius;
            if (surfaceDistance < 0)
                surfaceDistance = -surfaceDistance;
            return surfaceDistance < shape.Radius * 1e-3f;
        }
    }

    public struct CapsuleRayTester : IRayTester<Capsule>
    {
        public void GetRandomShape(Random random, out Capsule shape)
        {
            const float sizeMin = 0.1f;
            const float sizeSpan = 200;
            shape = new Capsule(sizeMin + sizeSpan * (float)random.NextDouble(), sizeMin * sizeSpan * (float)random.NextDouble());
        }
        public void GetPointInVolume(Random random, float innerMargin, ref Capsule capsule, out Vector3 localPointInCapsule)
        {
            float distanceSquared;
            float effectiveRadius = Math.Max(0, capsule.Radius - innerMargin);
            float radiusSquared = effectiveRadius * effectiveRadius;
            var min = new Vector3(effectiveRadius, effectiveRadius + capsule.HalfLength, effectiveRadius);
            var span = min * 2;
            min = -min;
            do
            {
                localPointInCapsule = min + span * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                var projectedCandidate = new Vector3(0, Math.Max(-capsule.HalfLength, Math.Min(capsule.HalfLength, localPointInCapsule.Y)), 0);
                distanceSquared = Vector3.DistanceSquared(projectedCandidate, localPointInCapsule);

            } while (distanceSquared > radiusSquared);
        }

        public void GetSurface(Random random, ref Capsule capsule, out Vector3 localPointOnCapsule, out Vector3 localNormal)
        {
            float distanceSquared;
            float radiusSquared = capsule.Radius * capsule.Radius;
            var min = new Vector3(capsule.Radius, capsule.Radius + capsule.HalfLength, capsule.Radius);
            var span = min * 2;
            Vector3 offset, projectedCandidate;
            min = -min;
            do
            {
                localPointOnCapsule = min + span * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                projectedCandidate = new Vector3(0, Math.Max(-capsule.HalfLength, Math.Min(capsule.HalfLength, localPointOnCapsule.Y)), 0);
                offset = localPointOnCapsule - projectedCandidate;
                distanceSquared = offset.LengthSquared();

            } while (distanceSquared < 1e-7f);
            localNormal = offset / (float)Math.Sqrt(distanceSquared);
            localPointOnCapsule = projectedCandidate + localNormal * capsule.Radius;
        }

        public bool PointIsOnSurface(ref Capsule capsule, ref Vector3 localPoint)
        {
            var projected = MathHelper.Clamp(localPoint.Y, -capsule.HalfLength, capsule.HalfLength);
            var surfaceDistance = Vector3.Distance(localPoint, new Vector3(0, projected, 0)) - capsule.Radius;
            if (surfaceDistance < 0)
                surfaceDistance = -surfaceDistance;
            return surfaceDistance < capsule.Radius * 1e-3f;
        }
    }

    public struct CylinderRayTester : IRayTester<Cylinder>
    {
        public void GetRandomShape(Random random, out Cylinder shape)
        {
            const float sizeMin = 0.1f;
            const float sizeSpan = 200;
            shape = new Cylinder(sizeMin + sizeSpan * (float)random.NextDouble(), sizeMin * sizeSpan * (float)random.NextDouble());
        }
        public void GetPointInVolume(Random random, float innerMargin, ref Cylinder cylinder, out Vector3 localPointInCylinder)
        {
            float distanceSquared;
            float effectiveRadius = Math.Max(0, cylinder.Radius - innerMargin);
            float effectiveHalfLength = Math.Max(0, cylinder.HalfLength - innerMargin);
            float radiusSquared = effectiveRadius * effectiveRadius;
            var min = new Vector2(effectiveRadius);
            var span = min * 2;
            min = -min;
            Vector2 randomHorizontal;
            do
            {
                randomHorizontal = min + span * new Vector2((float)random.NextDouble(), (float)random.NextDouble());
                distanceSquared = randomHorizontal.LengthSquared();

            } while (distanceSquared > radiusSquared);
            localPointInCylinder = new Vector3(randomHorizontal.X, -effectiveHalfLength + 2 * effectiveHalfLength * (float)random.NextDouble(), randomHorizontal.Y);
        }

        public void GetSurface(Random random, ref Cylinder cylinder, out Vector3 localPointOnCylinder, out Vector3 localNormal)
        {
            float distanceSquared;
            var min = new Vector2(cylinder.Radius);
            var span = min * 2;
            min = -min;

            var sideArea = 4 * MathF.PI * cylinder.Radius * cylinder.HalfLength;
            var capArea = MathF.PI * cylinder.Radius * cylinder.Radius;
            var totalArea = capArea * 2 + sideArea;
            var faceSelection = random.NextDouble();
            if (faceSelection * totalArea < sideArea)
            {
                //Side.
                Vector2 randomHorizontal;
                do
                {
                    randomHorizontal = min + span * new Vector2((float)random.NextDouble(), (float)random.NextDouble());
                    distanceSquared = randomHorizontal.LengthSquared();

                } while (distanceSquared < 1e-7f);
                var horizontalNormal = randomHorizontal / (float)Math.Sqrt(distanceSquared);
                localNormal = new Vector3(horizontalNormal.X, 0, horizontalNormal.Y);
                var horizontalOffset = horizontalNormal * cylinder.Radius;
                localPointOnCylinder = new Vector3(horizontalOffset.X, -cylinder.HalfLength + 2 * cylinder.HalfLength * (float)random.NextDouble(), horizontalOffset.Y);
            }
            else
            {
                //One of the two caps.
                var upperCap = faceSelection * totalArea < totalArea - capArea;
                localNormal = new Vector3(0, upperCap ? 1 : -1, 0);
                Vector2 randomHorizontal;
                do
                {
                    randomHorizontal = min + span * new Vector2((float)random.NextDouble(), (float)random.NextDouble());
                    distanceSquared = randomHorizontal.LengthSquared();

                } while (distanceSquared < cylinder.Radius * cylinder.Radius);
                localPointOnCylinder = new Vector3(randomHorizontal.X, upperCap ? cylinder.HalfLength : -cylinder.HalfLength, randomHorizontal.Y);
            }
        }

        public bool PointIsOnSurface(ref Cylinder cylinder, ref Vector3 localPoint)
        {
            var epsilon = MathF.Max(cylinder.HalfLength, cylinder.Radius) * 1e-3f;
            if (MathF.Abs(localPoint.Y) > cylinder.HalfLength + epsilon)
            {
                //Too far up or down.
                return false;
            }
            var horizontalDistanceSquared = localPoint.X * localPoint.X + localPoint.Z * localPoint.Z;
            var radiusPlusEpsilon = cylinder.Radius + epsilon;
            if (horizontalDistanceSquared > radiusPlusEpsilon * radiusPlusEpsilon)
            {
                //Too far out.
                return false;
            }
            if (MathF.Abs(localPoint.Y) > cylinder.HalfLength - epsilon)
            {
                //It's on one of the caps. Already confirmed that the point isn't outside of the radius.
                return true;
            }
            //It's not on a cap. If it's not too deep, then it's on the surface of the side.
            var radiusMinusEpsilon = cylinder.Radius - epsilon;
            return horizontalDistanceSquared > radiusMinusEpsilon * radiusMinusEpsilon;
        }
    }

    public struct BoxRayTester : IRayTester<Box>
    {
        public void GetRandomShape(Random random, out Box shape)
        {
            const float sizeMin = 0.1f;
            const float sizeSpan = 200;
            shape = new Box(sizeMin + sizeSpan * (float)random.NextDouble(), sizeMin * sizeSpan * (float)random.NextDouble(), sizeMin * sizeSpan * (float)random.NextDouble());
        }
        public void GetPointInVolume(Random random, float innerMargin, ref Box box, out Vector3 localPoint)
        {
            var min = new Vector3(box.HalfWidth - innerMargin, box.HalfHeight - innerMargin, box.HalfLength - innerMargin);
            var span = min * 2;
            min = -min;
            localPoint = min + span * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
        }

        public void GetSurface(Random random, ref Box box, out Vector3 localPoint, out Vector3 localNormal)
        {
            var a = (float)random.NextDouble();
            var b = (float)random.NextDouble();
            var axisSign = (float)(random.Next(2) * 2 - 1);
            Vector3 x, y, z;
            switch (random.Next(3))
            {
                case 0:
                    x = new Vector3(box.HalfWidth, 0, 0);
                    y = new Vector3(0, box.HalfHeight, 0);
                    localNormal = new Vector3(0, 0, axisSign);
                    z = localNormal * box.HalfLength;
                    break;
                case 1:
                    x = new Vector3(0, box.HalfHeight, 0);
                    y = new Vector3(0, 0, box.HalfLength);
                    localNormal = new Vector3(axisSign, 0, 0);
                    z = localNormal * box.HalfWidth;
                    break;
                default:
                    x = new Vector3(0, 0, box.HalfLength);
                    y = new Vector3(box.HalfWidth, 0, 0);
                    localNormal = new Vector3(0, axisSign, 0);
                    z = localNormal * box.HalfHeight;
                    break;
            }
            localPoint = (2 * a - 1) * x + (2 * b - 1) * y + z;
        }

        public bool PointIsOnSurface(ref Box box, ref Vector3 localPoint)
        {
            //Cast a ray against the box's bounding planes from the local origin using the local point as the direction.
            //In effect, all we're doing here is making sure that the closest plane impact has an offset similar to its box extent.
            var halfExtents = new Vector3(box.HalfWidth, box.HalfHeight, box.HalfLength);
            var t = (halfExtents * halfExtents) / Vector3.Max(new Vector3(1e-15f), Vector3.Abs(localPoint)) - halfExtents;
            var minT = t.X < t.Y ? t.X : t.Y;
            if (t.Z < minT)
                minT = t.Z;
            return Math.Abs(minT) < 1e-3f * Math.Max(box.HalfWidth, Math.Max(box.HalfHeight, box.HalfLength));
        }
    }

    public static class RayTesting
    {
        internal static void GetUnitDirection(Random random, out Vector3 direction)
        {
            //Not much cleverness involved here. This does not produce a uniform distribution over the the unit sphere.
            float length;
            do
            {
                direction = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * new Vector3(2) - new Vector3(1);
                length = direction.Length();
            } while (length < 1e-7f);
            direction /= length;
        }
        static void GetUnitQuaternion(Random random, out Quaternion orientation)
        {
            //Not much cleverness involved here. This does not produce a uniform distribution over the the unit sphere.
            float length;
            do
            {
                orientation = new Quaternion(
                    (float)random.NextDouble() * 2 - 1,
                    (float)random.NextDouble() * 2 - 1,
                    (float)random.NextDouble() * 2 - 1,
                    (float)random.NextDouble() * 2 - 1);
                length = orientation.Length();
            } while (length < 1e-7f);
            Unsafe.As<Quaternion, Vector4>(ref orientation) /= length;
        }
        static void GetPointOnPlane(Random random, float centralExclusion, float span, ref Vector3 anchor, ref Vector3 normal, out Vector3 point)
        {

            Vector2 localPoint;
            var exclusionSquared = centralExclusion * centralExclusion;
            do
            {
                localPoint = span * (new Vector2((float)random.NextDouble(), (float)random.NextDouble()) - new Vector2(0.5f));
            } while (localPoint.LengthSquared() < exclusionSquared);

            Vector3 basisX;
            float basisXLengthSquared;
            do
            {
                GetUnitDirection(random, out var randomDirection);
                basisX = Vector3.Cross(normal, randomDirection);
                basisXLengthSquared = basisX.LengthSquared();
            } while (basisXLengthSquared < 1e-7f);
            var basisZ = Vector3.Cross(normal, basisX);
            point = anchor + basisX * localPoint.X + basisZ * localPoint.Y;
        }

        static void CheckWide<TShape, TShapeWide>(ref RigidPoses poses, ref TShapeWide shapeWide, ref Vector3 origin, ref Vector3 direction, bool intersected, float t, ref Vector3 normal)
                            where TShape : IConvexShape where TShapeWide : IShapeWide<TShape>
        {
            RayWide rayWide;
            Vector3Wide.Broadcast(origin, out rayWide.Origin);
            Vector3Wide.Broadcast(direction, out rayWide.Direction);

            shapeWide.RayTest(ref poses, ref rayWide, out var intersectedWide, out var tWide, out var normalWide);
            if (intersectedWide[0] < 0 != intersected)
            {
                Console.WriteLine($"Wide ray boolean result disagrees with scalar ray.");
            }
            if (intersected && intersectedWide[0] < 0)
            {
                if (Math.Abs(tWide[0] - t) > 1e-7f)
                {
                    Console.WriteLine("Wide ray t disagrees with scalar ray.");
                }
                if (Math.Abs(normalWide.X[0] - normal.X) > 1e-6f ||
                    Math.Abs(normalWide.Y[0] - normal.Y) > 1e-6f ||
                    Math.Abs(normalWide.Z[0] - normal.Z) > 1e-6f)
                {
                    Console.WriteLine("Wide ray normal disagrees with scalar ray.");
                }
            }
        }

        static void Test<TShape, TShapeWide, TTester>() where TShape : IConvexShape where TTester : IRayTester<TShape> where TShapeWide : IShapeWide<TShape>
        {
            const int shapeIterations = 1000;
            const int transformIterations = 100;
            const int outsideToInsideRays = 100;
            const int insideRays = 10;
            const int outsideRays = 100;
            const int outwardPointingRays = 100;

            const float volumeInnerMargin = 1e-4f;

            const float positionBoundsSpan = 100;
            const float positionMin = positionBoundsSpan * -0.5f;

            const float outsideMinimumDistance = 0.02f;
            const float outsideDistanceSpan = 1000;

            const float tangentMinimumDistance = 0.02f;
            const float tangentDistanceSpan = 10;
            const float tangentCentralExclusionMin = 0.01f;
            const float tangentCentralExclusionSpan = 10;
            const float tangentSourceSpanMin = 0.01f;
            const float tangentSourceSpanSpan = 1000f;

            const float outwardPointingSpan = 1000f;

            var tester = default(TTester);
            Random random = new Random(5);
            TShapeWide shapeWide = default;
            for (int shapeIteration = 0; shapeIteration < shapeIterations; ++shapeIteration)
            {
                tester.GetRandomShape(random, out var shape);
                shapeWide.Broadcast(shape);
                for (int transformIteration = 0; transformIteration < transformIterations; ++transformIteration)
                {
                    RigidPose pose;
                    pose.Position = new Vector3(positionMin) + positionBoundsSpan * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                    GetUnitQuaternion(random, out pose.Orientation);
                    Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
                    RigidPoses poses;
                    Vector3Wide.Broadcast(pose.Position, out poses.Position);
                    QuaternionWide.Broadcast(pose.Orientation, out poses.Orientation);
                    for (int rayIndex = 0; rayIndex < outsideToInsideRays; ++rayIndex)
                    {
                        tester.GetSurface(random, ref shape, out var pointOnSurface, out var normal);
                        var localSourcePoint = pointOnSurface + normal * (outsideMinimumDistance + (float)random.NextDouble() * outsideDistanceSpan);
                        tester.GetPointInVolume(random, volumeInnerMargin, ref shape, out var localTargetPoint);

                        Matrix3x3.Transform(localSourcePoint, orientation, out var sourcePoint);
                        sourcePoint += pose.Position;
                        var directionScale = (0.01f + 2 * (float)random.NextDouble());
                        var localDirection = (localTargetPoint - localSourcePoint) * directionScale;
                        Matrix3x3.Transform(localDirection, orientation, out var direction);

                        bool intersected;
                        if (intersected = shape.RayTest(pose, sourcePoint, direction, out var t, out var rayTestedNormal))
                        {
                            //If the ray start is outside the shape and the target point is inside, then the ray impact should exist on the surface of the shape.
                            var hitLocation = sourcePoint + t * direction;
                            var localHitLocation = hitLocation - pose.Position;
                            Matrix3x3.TransformTranspose(localHitLocation, orientation, out localHitLocation);
                            if (!tester.PointIsOnSurface(ref shape, ref localHitLocation))
                            {
                                Console.WriteLine("Outside->inside ray detected non-surface impact.");
                            }
                        }
                        else
                        {
                            Console.WriteLine($"Outside->inside ray detected no hit.");
                        }
                        CheckWide<TShape, TShapeWide>(ref poses, ref shapeWide, ref sourcePoint, ref direction, intersected, t, ref rayTestedNormal);
                    }
                    for (int rayIndex = 0; rayIndex < insideRays; ++rayIndex)
                    {
                        tester.GetPointInVolume(random, volumeInnerMargin, ref shape, out var localSourcePoint);
                        Matrix3x3.Transform(localSourcePoint, orientation, out var sourcePoint);
                        sourcePoint += pose.Position;

                        var directionScale = (0.01f + 100 * (float)random.NextDouble());
                        GetUnitDirection(random, out var direction);
                        direction *= directionScale;

                        //If the ray start is inside the shape, then the impact t should be 0.
                        bool intersected;
                        if (intersected = shape.RayTest(pose, sourcePoint, direction, out var t, out var rayTestedNormal))
                        {
                            if (t > 0)
                            {
                                Console.WriteLine($"Inside ray detected nonzero t value.");
                            }
                        }
                        else
                        {
                            Console.WriteLine($"Inside ray detected no impact.");
                        }
                        CheckWide<TShape, TShapeWide>(ref poses, ref shapeWide, ref sourcePoint, ref direction, intersected, t, ref rayTestedNormal);
                    }
                    for (int rayIndex = 0; rayIndex < outsideRays; ++rayIndex)
                    {
                        //Create a ray that lies on one of the shape's tangent planes, offset from the surface some amount to avoid numerical limitations.
                        tester.GetSurface(random, ref shape, out var pointOnSurface, out var localNormal);
                        var localTargetPoint = pointOnSurface + localNormal * (tangentMinimumDistance + (float)random.NextDouble() * tangentDistanceSpan);
                        var exclusion = tangentCentralExclusionMin + (float)random.NextDouble() * tangentCentralExclusionSpan;
                        var span = 2 * exclusion + tangentSourceSpanMin + tangentSourceSpanSpan * (float)random.NextDouble();
                        GetPointOnPlane(random, exclusion, span, ref localTargetPoint, ref localNormal, out var localSourcePoint);
                        var directionScale = (0.01f + 2 * (float)random.NextDouble());
                        var localDirection = (localTargetPoint - localSourcePoint) * directionScale;
                        Matrix3x3.Transform(localSourcePoint, orientation, out var sourcePoint);
                        sourcePoint += pose.Position;
                        Matrix3x3.Transform(localDirection, orientation, out var direction);
                        bool intersected;
                        if (intersected = shape.RayTest(pose, sourcePoint, direction, out var t, out var rayTestedNormal))
                        {
                            Console.WriteLine($"Outside ray incorrectly detected an impact.");
                        }
                        CheckWide<TShape, TShapeWide>(ref poses, ref shapeWide, ref sourcePoint, ref direction, intersected, t, ref rayTestedNormal);
                    }
                    for (int rayIndex = 0; rayIndex < outwardPointingRays; ++rayIndex)
                    {
                        tester.GetSurface(random, ref shape, out var pointOnSurface, out var localNormal);
                        var localSourcePoint = pointOnSurface + localNormal * (tangentMinimumDistance + (float)random.NextDouble() * tangentDistanceSpan);
                        Vector3 localTargetPoint;
                        do
                        {
                            localTargetPoint = localSourcePoint + new Vector3(-0.5f * outwardPointingSpan) + new Vector3(outwardPointingSpan) * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                        } while (Vector3.Dot(localTargetPoint - localSourcePoint, localNormal) < 0);
                        var directionScale = (0.01f + 2 * (float)random.NextDouble());
                        var localDirection = (localTargetPoint - localSourcePoint) * directionScale;
                        Matrix3x3.Transform(localSourcePoint, orientation, out var sourcePoint);
                        sourcePoint += pose.Position;
                        Matrix3x3.Transform(localDirection, orientation, out var direction);
                        bool intersected;
                        if (intersected = shape.RayTest(pose, sourcePoint, direction, out var t, out var rayTestedNormal))
                        {
                            Console.WriteLine($"Outward ray incorrectly detected an impact.");
                        }
                        CheckWide<TShape, TShapeWide>(ref poses, ref shapeWide, ref sourcePoint, ref direction, intersected, t, ref rayTestedNormal);
                    }
                }
            }
        }

        public static void Test()
        {
            Test<Sphere, SphereWide, SphereRayTester>();
            Test<Capsule, CapsuleWide, CapsuleRayTester>();
            Test<Cylinder, CylinderWide, CylinderRayTester>();
            Test<Box, BoxWide, BoxRayTester>();
        }
    }
}
