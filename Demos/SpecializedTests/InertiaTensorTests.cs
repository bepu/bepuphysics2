using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    public interface IInertiaTester<TShape> where TShape : IShape
    {
        bool PointIsContained(ref TShape shape, ref Vector3 point);
    }

    //TODO: We might later actually bundle point depth/normal queries into the shape types themselves, but for now the demos will provide it.
    public struct SphereInertiaTester : IInertiaTester<Sphere>
    {
        public bool PointIsContained(ref Sphere shape, ref Vector3 point)
        {
            return point.LengthSquared() <= shape.Radius * shape.Radius;
        }
    }

    public struct CapsuleInertiaTester : IInertiaTester<Capsule>
    {
        public bool PointIsContained(ref Capsule shape, ref Vector3 point)
        {
            var projectedPoint = new Vector3(0, Math.Max(-shape.HalfLength, Math.Min(shape.HalfLength, point.Y)), 0);
            return (point - projectedPoint).LengthSquared() <= shape.Radius * shape.Radius;
        }
    }
    public struct BoxInertiaTester : IInertiaTester<Box>
    {
        public bool PointIsContained(ref Box shape, ref Vector3 point)
        {
            ref var extent = ref Unsafe.As<float, Vector3>(ref shape.HalfWidth);
            var clampedPoint = Vector3.Min(extent, Vector3.Max(-extent, point));
            return point == clampedPoint;
        }
    }

    public static class InertiaTensorTests
    {
        static void CheckInertia<TShape, TInertiaTester>(ref TShape shape) where TShape : IConvexShape where TInertiaTester : IInertiaTester<TShape>
        {
            var orientation = BepuUtilities.Quaternion.Identity;
            shape.ComputeBounds(orientation, out var min, out var max);
            var span = max - min;
            const int axisSampleCount = 64;
            var sampleSpacing = span / axisSampleCount;
            var sampleMin = min + sampleSpacing * 0.5f;
            //This constant value isn't meaningful- it's just here to capture mass scaling bugs in implementations.
            var mass = (float)Math.Sqrt(11f / MathHelper.Pi);
            var numericalLocalInertia = new Triangular3x3();
            var tester = default(TInertiaTester);
            int containedSampleCount = 0;

            for (int i = 0; i < axisSampleCount; ++i)
            {
                for (int j = 0; j < axisSampleCount; ++j)
                {
                    for (int k = 0; k < axisSampleCount; ++k)
                    {
                        var sampleLocation = sampleMin + new Vector3(i, j, k) * sampleSpacing;
                        if (tester.PointIsContained(ref shape, ref sampleLocation))
                        {
                            var dd = Vector3.Dot(sampleLocation, sampleLocation);
                            numericalLocalInertia.XX += dd - sampleLocation.X * sampleLocation.X;
                            numericalLocalInertia.YX += -sampleLocation.X * sampleLocation.Y;
                            numericalLocalInertia.YY += dd - sampleLocation.Y * sampleLocation.Y;
                            numericalLocalInertia.ZX += -sampleLocation.X * sampleLocation.Z;
                            numericalLocalInertia.ZY += -sampleLocation.Y * sampleLocation.Z;
                            numericalLocalInertia.ZZ += dd - sampleLocation.Z * sampleLocation.Z;
                            ++containedSampleCount;
                        }
                    }
                }
            }

            Triangular3x3.Scale(ref numericalLocalInertia, mass / containedSampleCount, out numericalLocalInertia);
            Triangular3x3.SymmetricInvert(numericalLocalInertia, out var numericalLocalInverseInertia);
            shape.ComputeInertia(mass, out var analyticInertia);
            if (!ValuesAreSimilar(analyticInertia.InverseInertiaTensor.XX, numericalLocalInverseInertia.XX) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.YX, numericalLocalInverseInertia.YX) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.YY, numericalLocalInverseInertia.YY) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.ZX, numericalLocalInverseInertia.ZX) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.ZY, numericalLocalInverseInertia.ZY) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.ZZ, numericalLocalInverseInertia.ZZ))
            {
                Console.WriteLine("Excessive error in numerical vs analytic inertia tensor.");
            }
        }

        static bool ValuesAreSimilar(float a, float b)
        {
            if (Math.Abs(a) > 1e-5f)
            {
                var ratio = a / b;
                const float threshold = 0.01f;
                return ratio < (1 + threshold) && ratio > 1f / (1 + threshold);
            }
            return Math.Abs(b) <= 1e-5;
        }

        public static void Test()
        {
            var random = new Random(5);
            const int shapeTrials = 32;
            for (int i = 0; i < shapeTrials; ++i)
            {
                var sphere = new Sphere(0.01f + (float)random.NextDouble() * 10);
                CheckInertia<Sphere, SphereInertiaTester>(ref sphere);
            }
            for (int i = 0; i < shapeTrials; ++i)
            {
                var capsule = new Capsule(0.01f + (float)random.NextDouble() * 10, 0.01f + (float)random.NextDouble() * 10);
                CheckInertia<Capsule, CapsuleInertiaTester>(ref capsule);
            }
            for (int i = 0; i < shapeTrials; ++i)
            {
                var box = new Box(0.01f + 10 * (float)random.NextDouble(), 0.01f + 10 * (float)random.NextDouble(), 0.01f + 10 * (float)random.NextDouble());
                CheckInertia<Box, BoxInertiaTester>(ref box);

            }
        }
    }
}
