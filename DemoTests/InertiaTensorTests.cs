﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Xunit;

namespace DemoTests
{
    public interface IInertiaTester
    {
        void ComputeBounds(out Vector3 min, out Vector3 max);
        void ComputeAnalyticInertia(float mass, out BodyInertia inverseInertia);
        bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point);
    }

    public struct SphereInertiaTester : IInertiaTester
    {
        public Sphere Sphere;

        public void ComputeAnalyticInertia(float mass, out BodyInertia inertia)
        {
            inertia = Sphere.ComputeInertia(mass);
        }

        public void ComputeBounds(out Vector3 min, out Vector3 max)
        {
            Sphere.ComputeBounds(Quaternion.Identity, out min, out max);
        }

        public bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point)
        {
            return point.LengthSquared() <= Sphere.Radius * Sphere.Radius;
        }
    }

    public struct CapsuleInertiaTester : IInertiaTester
    {
        public Capsule Capsule;

        public void ComputeAnalyticInertia(float mass, out BodyInertia inertia)
        {
            inertia = Capsule.ComputeInertia(mass);
        }
        public void ComputeBounds(out Vector3 min, out Vector3 max)
        {
            Capsule.ComputeBounds(Quaternion.Identity, out min, out max);
        }

        public bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point)
        {
            var projectedPoint = new Vector3(0, Math.Max(-Capsule.HalfLength, Math.Min(Capsule.HalfLength, point.Y)), 0);
            return (point - projectedPoint).LengthSquared() <= Capsule.Radius * Capsule.Radius;
        }
    }

    public struct CylinderInertiaTester : IInertiaTester
    {
        public Cylinder Cylinder;

        public void ComputeAnalyticInertia(float mass, out BodyInertia inertia)
        {
            inertia = Cylinder.ComputeInertia(mass);
        }
        public void ComputeBounds(out Vector3 min, out Vector3 max)
        {
            Cylinder.ComputeBounds(Quaternion.Identity, out min, out max);
        }

        public bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point)
        {
            var horizontalDistanceSquared = point.X * point.X + point.Z * point.Z;
            return MathF.Abs(point.Y) < Cylinder.HalfLength && horizontalDistanceSquared < Cylinder.Radius * Cylinder.Radius;
        }
    }

    public struct BoxInertiaTester : IInertiaTester
    {
        public Box Box;
        public void ComputeAnalyticInertia(float mass, out BodyInertia inertia)
        {
            inertia = Box.ComputeInertia(mass);
        }
        public void ComputeBounds(out Vector3 min, out Vector3 max)
        {
            Box.ComputeBounds(Quaternion.Identity, out min, out max);
        }
        public bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point)
        {
            ref var extent = ref Unsafe.As<float, Vector3>(ref Box.HalfWidth);
            var clampedPoint = Vector3.Min(extent, Vector3.Max(-extent, point));
            return point == clampedPoint;
        }
    }

    public struct TriangleInertiaTester : IInertiaTester
    {
        public Triangle Triangle;
        public void ComputeAnalyticInertia(float mass, out BodyInertia inertia)
        {
            inertia = Triangle.ComputeInertia(mass);
        }
        public void ComputeBounds(out Vector3 min, out Vector3 max)
        {
            Triangle.ComputeBounds(Quaternion.Identity, out min, out max);
        }
        public bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point)
        {
            //This is quite distant from an efficient implementation. That's fine.
            var ap = point - Triangle.A;
            var bp = point - Triangle.B;
            var ab = Triangle.B - Triangle.A;
            var ac = Triangle.C - Triangle.A;
            var bc = Triangle.C - Triangle.B;
            var nn = Vector3.Dot(Vector3.Cross(ab, ac), Vector3.Cross(ab, ac));
            var wc = Vector3.Dot(Vector3.Cross(ab, ap), Vector3.Cross(ab, ac)) / nn;
            var wb = Vector3.Dot(Vector3.Cross(ac, ap), Vector3.Cross(ac, ab)) / nn;
            if (wb >= 0 && wc >= 0 && wb + wc <= 1)
            {
                var wa = 1 - wb - wc;
                var pointOnTriangle = Triangle.A * wa + Triangle.B * wb + Triangle.C * wc;
                var offset = pointOnTriangle - point;
                //We treat a point sample as 'contained' by the triangle if it just happens to be close.
                return Vector3.Dot(offset, offset) < Vector3.Dot(sampleSpacing, sampleSpacing);
            }
            return false;
        }
    }
    public struct ConvexHullInertiaTester : IInertiaTester
    {
        public ConvexHull Hull;
        public void ComputeAnalyticInertia(float mass, out BodyInertia inertia)
        {
            inertia = Hull.ComputeInertia(mass);
        }
        public void ComputeBounds(out Vector3 min, out Vector3 max)
        {
            Hull.ComputeBounds(Quaternion.Identity, out min, out max);
        }
        public bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point)
        {
            Vector3Wide.Broadcast(point, out var pointBundle);
            var containmentEpsilon = new Vector<float>(sampleSpacing.Length() * 0.5f);
            for (int i = 0; i < Hull.BoundingPlanes.Length; ++i)
            {
                ref var plane = ref Hull.BoundingPlanes[i];
                Vector3Wide.Dot(plane.Normal, pointBundle, out var dot);
                var contained = Vector.LessThan(dot - plane.Offset, containmentEpsilon);
                var count = Math.Min(Hull.FaceToVertexIndicesStart.Length - i * Vector<float>.Count, Vector<float>.Count);
                for (int j = 0; j < count; ++j)
                {
                    if (contained[j] == 0)
                        return false;
                }
            }
            return true;
        }
    }


    public static class InertiaTensorTests
    {
        static bool ValuesAreSimilar(float a, float b)
        {
            var ratio = a / b;
            const float ratioThreshold = 0.15f;
            return MathF.Abs(a - b) < 3e-2f || (ratio < (1 + ratioThreshold) && ratio > 1f / (1 + ratioThreshold));
        }

        private static void CheckInertiaError(Symmetric3x3 numericalLocalInverseInertia, BodyInertia analyticInertia)
        {
            if (!ValuesAreSimilar(analyticInertia.InverseInertiaTensor.XX, numericalLocalInverseInertia.XX) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.YX, numericalLocalInverseInertia.YX) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.YY, numericalLocalInverseInertia.YY) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.ZX, numericalLocalInverseInertia.ZX) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.ZY, numericalLocalInverseInertia.ZY) ||
                !ValuesAreSimilar(analyticInertia.InverseInertiaTensor.ZZ, numericalLocalInverseInertia.ZZ))
            {
                Assert.True(false, "Excessive error in numerical vs analytic inertia tensor.");
                Console.WriteLine($"ANALYTIC INERTIA:   {analyticInertia.InverseInertiaTensor} vs ");
                Console.WriteLine($"NUMERICAL INERTIA:  {numericalLocalInverseInertia}");
                Symmetric3x3.Subtract(analyticInertia.InverseInertiaTensor, numericalLocalInverseInertia, out var difference);
                Console.WriteLine($"DIFFERENCE: {difference}");
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void AccumulateSampleIntoInertia(Vector3 sampleLocation, ref Symmetric3x3 numericalLocalInertia)
        {
            var dd = Vector3.Dot(sampleLocation, sampleLocation);
            numericalLocalInertia.XX += dd - sampleLocation.X * sampleLocation.X;
            numericalLocalInertia.YX += -sampleLocation.X * sampleLocation.Y;
            numericalLocalInertia.YY += dd - sampleLocation.Y * sampleLocation.Y;
            numericalLocalInertia.ZX += -sampleLocation.X * sampleLocation.Z;
            numericalLocalInertia.ZY += -sampleLocation.Y * sampleLocation.Z;
            numericalLocalInertia.ZZ += dd - sampleLocation.Z * sampleLocation.Z;
        }

        static void CheckInertia<TInertiaTester>(ref TInertiaTester tester) where TInertiaTester : IInertiaTester
        {
            tester.ComputeBounds(out var min, out var max);
            var span = max - min;
            const int axisSampleCount = 128;
            var sampleSpacing = span / axisSampleCount;
            var sampleMin = min + sampleSpacing * 0.5f;
            //This constant value isn't meaningful- it's just here to capture mass scaling bugs in implementations.
            var mass = (float)Math.Sqrt(11f / MathHelper.Pi);
            var numericalLocalInertia = new Symmetric3x3();
            int containedSampleCount = 0;

            for (int i = 0; i < axisSampleCount; ++i)
            {
                for (int j = 0; j < axisSampleCount; ++j)
                {
                    for (int k = 0; k < axisSampleCount; ++k)
                    {
                        var sampleLocation = sampleMin + new Vector3(i, j, k) * sampleSpacing;
                        if (tester.PointIsContained(ref sampleSpacing, ref sampleLocation))
                        {
                            AccumulateSampleIntoInertia(sampleLocation, ref numericalLocalInertia);
                            ++containedSampleCount;
                        }
                    }
                }
            }

            Symmetric3x3.Scale(numericalLocalInertia, mass / containedSampleCount, out numericalLocalInertia);
            Symmetric3x3.Invert(numericalLocalInertia, out var numericalLocalInverseInertia);
            tester.ComputeAnalyticInertia(mass, out var analyticInertia);
            CheckInertiaError(numericalLocalInverseInertia, analyticInertia);
        }

        unsafe struct HitCounter : IShapeRayHitHandler
        {
            public int Counter;

            public bool AllowTest(int childIndex)
            {
                return true;
            }

            public void OnRayHit(in RayData ray, ref float maximumT, float t, Vector3 normal, int childIndex)
            {
                ++Counter;
            }
        }


        private static void TestCompound(Random random, BufferPool pool)
        {
            var shapes = new Shapes(pool, 8);
            var treeCompoundBoxShape = new Box(0.5f, 1.5f, 1f);
            var treeCompoundBoxShapeIndex = shapes.Add(treeCompoundBoxShape);
            using var compoundBuilder = new CompoundBuilder(pool, shapes, 128);

            //This constant value isn't meaningful- it's just here to capture mass scaling bugs in implementations.
            var mass = (float)Math.Sqrt(11f / MathHelper.Pi);
            const int childCount = 4;
            var massPerChild = mass / childCount;
            var childInertia = treeCompoundBoxShape.ComputeInertia(massPerChild);
            for (int i = 0; i < childCount; ++i)
            {
                RigidPose localPose;
                localPose.Position = new Vector3(2, 4, 2) * (0.5f * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) - Vector3.One);
                float orientationLengthSquared;
                do
                {
                    localPose.Orientation = new Quaternion(random.NextSingle(), random.NextSingle(), random.NextSingle(), random.NextSingle());
                    orientationLengthSquared = QuaternionEx.LengthSquared(ref localPose.Orientation);
                }
                while (orientationLengthSquared < 1e-9f);
                QuaternionEx.Scale(localPose.Orientation, 1f / MathF.Sqrt(orientationLengthSquared), out localPose.Orientation);
                //localPose.Orientation = Quaternion.Identity;
                //Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI, out localPose.Orientation);

                compoundBuilder.Add(treeCompoundBoxShapeIndex, localPose, childInertia.InverseInertiaTensor, massPerChild);
            }
            compoundBuilder.BuildDynamicCompound(out var children, out var analyticInertia, out var center);
            var compound = new Compound(children);
            if (random.NextDouble() < 0.5)
            {
                //Bit hacky. Technically two codepaths, want to cover both.
                Span<float> childMasses = stackalloc float[childCount];
                for (int i = 0; i < childMasses.Length; ++i)
                    childMasses[i] = massPerChild;
                analyticInertia = compound.ComputeInertia(childMasses, shapes, out var doubleCheckedCenter);
                Assert.True(doubleCheckedCenter.Length() < 1e-4f, "The compound builder should have already recentered the children. Is there a disagreement in the center of mass calculation somewhere?");
            }

            compound.ComputeBounds(Quaternion.Identity, shapes, out var min, out var max);
            var span = max - min;
            const int axisSampleCount = 128;
            var sampleSpacing = span / axisSampleCount;
            var sampleMin = min + sampleSpacing * 0.5f;
            var numericalLocalInertia = new Symmetric3x3();

            var pose = RigidPose.Identity;
            var hitCounter = new HitCounter();
            float maximumT = 0.000001f;


            for (int i = 0; i < axisSampleCount; ++i)
            {
                for (int j = 0; j < axisSampleCount; ++j)
                {
                    for (int k = 0; k < axisSampleCount; ++k)
                    {
                        var sampleLocation = sampleMin + new Vector3(i, j, k) * sampleSpacing;
                        var previousCount = hitCounter.Counter;
                        compound.RayTest(pose, new RayData { Origin = sampleLocation, Direction = Vector3.UnitY }, ref maximumT, shapes, ref hitCounter);
                        //If the ray hit more than one shape, then we count them all.
                        //This matches how the analytic inertia is calculated- every shape provides its own tensor, and they're summed.
                        //(Notably, if you wanted non-overlapping inertia, this is counterproductive!)
                        for (int p = previousCount; p < hitCounter.Counter; ++p)
                        {
                            AccumulateSampleIntoInertia(sampleLocation, ref numericalLocalInertia);
                        }
                    }
                }
            }

            Symmetric3x3.Scale(numericalLocalInertia, mass / hitCounter.Counter, out numericalLocalInertia);
            Symmetric3x3.Invert(numericalLocalInertia, out var numericalLocalInverseInertia);
            CheckInertiaError(numericalLocalInverseInertia, analyticInertia);

            compound.Dispose(pool);
            shapes.Dispose();

        }

        [Fact]
        public static void Test()
        {
            var random = new Random(5);
            const int shapeTrials = 64;
            for (int i = 0; i < shapeTrials; ++i)
            {
                var tester = new SphereInertiaTester { Sphere = new Sphere(0.01f + random.NextSingle() * 10) };
                CheckInertia(ref tester);
            }
            for (int i = 0; i < shapeTrials; ++i)
            {
                var tester = new CapsuleInertiaTester { Capsule = new Capsule(0.01f + random.NextSingle() * 10, 0.01f + random.NextSingle() * 10) };
                CheckInertia(ref tester);
            }
            for (int i = 0; i < shapeTrials; ++i)
            {
                var tester = new CylinderInertiaTester { Cylinder = new Cylinder(0.01f + random.NextSingle() * 10, 0.01f + random.NextSingle() * 10) };
                CheckInertia(ref tester);
            }
            for (int i = 0; i < shapeTrials; ++i)
            {
                var tester = new BoxInertiaTester { Box = new Box(0.01f + 10 * random.NextSingle(), 0.01f + 10 * random.NextSingle(), 0.01f + 10 * random.NextSingle()) };
                CheckInertia(ref tester);
            }
            for (int i = 0; i < shapeTrials; ++i)
            {
                var tester = new TriangleInertiaTester
                {
                    Triangle = new Triangle(
                        new Vector3(-2 + 4 * random.NextSingle(), -2 + 4 * random.NextSingle(), -2 + 4 * random.NextSingle()),
                        new Vector3(-2 + 4 * random.NextSingle(), -2 + 4 * random.NextSingle(), -2 + 4 * random.NextSingle()),
                        new Vector3(-2 + 4 * random.NextSingle(), -2 + 4 * random.NextSingle(), -2 + 4 * random.NextSingle())),
                };
                CheckInertia(ref tester);
            }
            var pool = new BufferPool();
            for (int i = 0; i < shapeTrials; ++i)
            {
                const int pointCount = 32;
                var pointSet = new QuickList<Vector3>(pointCount, pool);
                for (int j = 0; j < pointCount; ++j)
                {
                    pointSet.AllocateUnsafely() = new Vector3(-1 + 2 * random.NextSingle(), -1 + 2 * random.NextSingle(), -1 + 2 * random.NextSingle());
                }
                for (int j = 0; j < pointSet.Count; ++j)
                {
                    pointSet[j] -= new Vector3(0, 1, 0);
                }
                var pointsBuffer = pointSet.Span.Slice(pointSet.Count);
                ConvexHullInertiaTester tester;
                ConvexHullHelper.CreateShape(pointsBuffer, pool, out _, out tester.Hull);

                CheckInertia(ref tester);
                tester.Hull.Dispose(pool);
            }

            for (int i = 0; i < shapeTrials; ++i)
            {
                TestCompound(random, pool);
            }
            pool.Clear();
        }
    }
}
