using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
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
            return point.Length() <= shape.Radius;
        }
    }

    public struct CapsuleInertiaTester : IInertiaTester<Capsule>
    {
        public bool PointIsContained(ref Capsule shape, ref Vector3 point)
        {
            var projectedPoint = new Vector3(0, Math.Max(-shape.HalfLength, Math.Min(shape.HalfLength, point.Y)), 0);
            return (point - projectedPoint).Length() <= shape.Radius;
        }
    }

    public static class InertiaTensorTests
    {
        static void CheckInertia<TShape, TInertiaTester>(ref TShape shape) where TShape : IShape where TInertiaTester : IInertiaTester<TShape>
        {
            var orientation = BepuUtilities.Quaternion.Identity;
            shape.GetBounds(ref orientation, out var min, out var max);
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
                            numericalLocalInertia.M11 += dd - sampleLocation.X * sampleLocation.X;
                            numericalLocalInertia.M21 += -sampleLocation.X * sampleLocation.Y;
                            numericalLocalInertia.M22 += dd - sampleLocation.Y * sampleLocation.Y;
                            numericalLocalInertia.M31 += -sampleLocation.X * sampleLocation.Z;
                            numericalLocalInertia.M32 += -sampleLocation.Y * sampleLocation.Z;
                            numericalLocalInertia.M33 += dd - sampleLocation.Z * sampleLocation.Z;
                            ++containedSampleCount;
                        }
                    }
                }
            }

            Triangular3x3.Scale(ref numericalLocalInertia, mass / containedSampleCount, out numericalLocalInertia);
            Triangular3x3.SymmetricInvert(ref numericalLocalInertia, out var numericalLocalInverseInertia);
            shape.ComputeLocalInverseInertia(1f / mass, out var analyticLocalInverseInertia);
            if (!ValuesAreSimilar(analyticLocalInverseInertia.M11, numericalLocalInverseInertia.M11) ||
                !ValuesAreSimilar(analyticLocalInverseInertia.M21, numericalLocalInverseInertia.M21) ||
                !ValuesAreSimilar(analyticLocalInverseInertia.M22, numericalLocalInverseInertia.M22) ||
                !ValuesAreSimilar(analyticLocalInverseInertia.M31, numericalLocalInverseInertia.M31) ||
                !ValuesAreSimilar(analyticLocalInverseInertia.M32, numericalLocalInverseInertia.M32) ||
                !ValuesAreSimilar(analyticLocalInverseInertia.M33, numericalLocalInverseInertia.M33))
            {
                Console.WriteLine("Excessive error in numerical vs analytic inertia tensor.");
            }
        }

        static bool ValuesAreSimilar(float a, float b)
        {
            if (Math.Abs(a) > 1e-5f)
            {
                var ratio = a / b;
                return ratio < 1.1f && ratio > 1f / 1.1f;
            }
            return Math.Abs(b) <= 1e-5;
        }

        public static void Test()
        {
            var random = new Random(5);
            const int shapeTrials = 32;
            //for (int i = 0; i < shapeTrials; ++i)
            //{
            //    var sphere = new Sphere(0.01f + (float)random.NextDouble() * 10);
            //    CheckInertia<Sphere, SphereInertiaTester>(ref sphere);
            //}
            for (int i = 0; i < shapeTrials; ++i)
            {
                var capsule = new Capsule(0.01f + (float)random.NextDouble() * 10, 0.01f + (float)random.NextDouble() * 10);
                capsule = new Capsule(5f, 10);
                CheckInertia<Capsule, CapsuleInertiaTester>(ref capsule);
            }
        }
    }
}
