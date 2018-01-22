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
    public static class CapsuleRayTesting
    {
        static void GetRandomShape(Random random, out Capsule shape)
        {
            const float sizeMin = 0.1f;
            const float sizeSpan = 200;
            shape = new Capsule(sizeMin + sizeSpan * (float)random.NextDouble(), sizeMin * sizeSpan * (float)random.NextDouble());
        }
        static void GetPointInVolume(Random random, float innerMargin, ref Capsule capsule, out Vector3 localPointInCapsule)
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

        static void GetSurface(Random random, ref Capsule capsule, out Vector3 localPointOnCapsule, out Vector3 localNormal)
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

        static bool PointIsOnSurface(ref Capsule capsule, ref Vector3 localPoint)
        {
            var projected = MathHelper.Clamp(localPoint.Y, -capsule.HalfLength, capsule.HalfLength);
            var surfaceDistance = Vector3.Distance(localPoint, new Vector3(0, projected, 0)) - capsule.Radius;
            if (surfaceDistance < 0)
                surfaceDistance = -surfaceDistance;
            return surfaceDistance < capsule.Radius * 1e-3f;
        }

        static float GetDistance(ref Capsule capsule, ref Vector3 localPoint)
        {
            return Vector3.Distance(localPoint, new Vector3(0, MathHelper.Clamp(localPoint.Y, -capsule.HalfLength, capsule.HalfLength), 0)) - capsule.Radius;
        }

        static void GetUnitDirection(Random random, out Vector3 direction)
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
                Vector3x.Cross(ref normal, ref randomDirection, out basisX);
                basisXLengthSquared = basisX.LengthSquared();
            } while (basisXLengthSquared < 1e-7f);
            Vector3x.Cross(ref normal, ref basisX, out var basisZ);
            point = anchor + basisX * localPoint.X + basisZ * localPoint.Y;
        }


        public static void Test()
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

            Random random = new Random(5);
            for (int shapeIteration = 0; shapeIteration < shapeIterations; ++shapeIteration)
            {
                GetRandomShape(random, out var shape);
                for (int transformIteration = 0; transformIteration < transformIterations; ++transformIteration)
                {
                    RigidPose pose;
                    pose.Position = new Vector3(positionMin) + positionBoundsSpan * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                    GetUnitQuaternion(random, out pose.Orientation);
                    Matrix3x3.CreateFromQuaternion(ref pose.Orientation, out var orientation);
                    for (int rayIndex = 0; rayIndex < outsideToInsideRays; ++rayIndex)
                    {
                        GetSurface(random, ref shape, out var pointOnSurface, out var normal);
                        var localSourcePoint = pointOnSurface + normal * (outsideMinimumDistance + (float)random.NextDouble() * outsideDistanceSpan);
                        GetPointInVolume(random, volumeInnerMargin, ref shape, out var localTargetPoint);

                        Matrix3x3.Transform(ref localSourcePoint, ref orientation, out var sourcePoint);
                        sourcePoint += pose.Position;
                        var directionScale = (0.01f + 2 * (float)random.NextDouble());
                        var localDirection = (localTargetPoint - localSourcePoint) * directionScale;
                        Matrix3x3.Transform(ref localDirection, ref orientation, out var direction);
                        
                        if (shape.RayTest(ref pose, ref sourcePoint, ref direction, out var t, out var rayTestedNormal))
                        {
                            //If the ray start is outside the shape and the target point is inside, then the ray impact should exist on the surface of the shape.
                            var hitLocation = sourcePoint + t * direction;
                            var localHitLocation = hitLocation - pose.Position;
                            Matrix3x3.TransformTranspose(ref localHitLocation, ref orientation, out localHitLocation);
                            if (!PointIsOnSurface(ref shape, ref localHitLocation))
                            {
                                Console.WriteLine("Outside->inside ray detected non-surface impact.");
                            }
                        }
                        else
                        {
                            Console.WriteLine($"Outside->inside ray detected no hit.");
                        }
                    }
                    for (int rayIndex = 0; rayIndex < insideRays; ++rayIndex)
                    {
                        GetPointInVolume(random, volumeInnerMargin, ref shape, out var localSourcePoint);
                        Matrix3x3.Transform(ref localSourcePoint, ref orientation, out var sourcePoint);
                        sourcePoint += pose.Position;

                        var directionScale = (0.01f + 100 * (float)random.NextDouble());
                        GetUnitDirection(random, out var direction);
                        direction *= directionScale;

                        var distance = GetDistance(ref shape, ref localSourcePoint);

                        //If the ray start is inside the shape, then the impact t should be 0.
                        if (shape.RayTest(ref pose, ref sourcePoint, ref direction, out var t, out var rayTestedNormal))
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
                    }
                    for (int rayIndex = 0; rayIndex < outsideRays; ++rayIndex)
                    {
                        //Create a ray that lies on one of the shape's tangent planes, offset from the surface some amount to avoid numerical limitations.
                        GetSurface(random, ref shape, out var pointOnSurface, out var localNormal);
                        var localTargetPoint = pointOnSurface + localNormal * (tangentMinimumDistance + (float)random.NextDouble() * tangentDistanceSpan);
                        var exclusion = tangentCentralExclusionMin + (float)random.NextDouble() * tangentCentralExclusionSpan;
                        var span = 2 * exclusion + tangentSourceSpanMin + tangentSourceSpanSpan * (float)random.NextDouble();
                        GetPointOnPlane(random, exclusion, span, ref localTargetPoint, ref localNormal, out var localSourcePoint);
                        var directionScale = (0.01f + 2 * (float)random.NextDouble());
                        var localDirection = (localTargetPoint - localSourcePoint) * directionScale;
                        Matrix3x3.Transform(ref localSourcePoint, ref orientation, out var sourcePoint);
                        sourcePoint += pose.Position;
                        Matrix3x3.Transform(ref localDirection, ref orientation, out var direction);

                        if (shape.RayTest(ref pose, ref sourcePoint, ref direction, out var t, out var rayTestedNormal))
                        {
                            Console.WriteLine($"Outside ray incorrectly detected an impact.");
                        }
                    }
                    for (int rayIndex = 0; rayIndex < outwardPointingRays; ++rayIndex)
                    {
                        GetSurface(random, ref shape, out var pointOnSurface, out var localNormal);
                        var localSourcePoint = pointOnSurface + localNormal * (tangentMinimumDistance + (float)random.NextDouble() * tangentDistanceSpan);
                        Vector3 localTargetPoint;
                        do
                        {
                            localTargetPoint = localSourcePoint + new Vector3(-0.5f * outwardPointingSpan) + new Vector3(outwardPointingSpan) * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                        } while (Vector3.Dot(localTargetPoint - localSourcePoint, localNormal) < 0);
                        var directionScale = (0.01f + 2 * (float)random.NextDouble());
                        var localDirection = (localTargetPoint - localSourcePoint) * directionScale;
                        Matrix3x3.Transform(ref localSourcePoint, ref orientation, out var sourcePoint);
                        sourcePoint += pose.Position;
                        Matrix3x3.Transform(ref localDirection, ref orientation, out var direction);
                        if (shape.RayTest(ref pose, ref sourcePoint, ref direction, out var t, out var rayTestedNormal))
                        {
                            Console.WriteLine($"Outward ray incorrectly detected an impact.");
                        }
                    }
                }
            }
        }
    }
}
