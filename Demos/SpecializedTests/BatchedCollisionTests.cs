using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities.Collections;

namespace Demos.SpecializedTests
{
    public static class BatchedCollisionTests
    {
        unsafe struct TestCollisionCallbacks : ICollisionCallbacks
        {
            public int* Count;

            public unsafe void OnPairCompleted(int pairId, ConvexContactManifold* manifold)
            {
                ref var contact = ref manifold->Contact0;
                var extra = 1e-16 * (contact.Depth + contact.Offset.X + manifold->Normal.X);
                *Count += 1 + (int)extra;
            }
            public unsafe void OnPairCompleted(int pairId, NonconvexContactManifold* manifold)
            {
                ref var contact = ref manifold->Contact0;
                var extra = 1e-16 * (contact.Depth + contact.Offset.X + contact.Normal.X);
                *Count += 1 + (int)extra;
            }
            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ConvexContactManifold* manifold)
            {
            }

            public bool AllowCollisionTesting(int pairId, int childA, int childB)
            {
                return true;
            }

        }


        static void TestPair<TA, TB>(ref TA a, ref TB b, ref Buffer<RigidPose> posesA, ref Buffer<RigidPose> posesB,
            ref TestCollisionCallbacks callbacks, BufferPool pool, Shapes shapes, CollisionTaskRegistry registry, int iterationCount)
            where TA : struct, IShape where TB : struct, IShape
        {
            var batcher = new CollisionBatcher<TestCollisionCallbacks>(pool, shapes, registry, 1 / 60f, callbacks);
            for (int i = 0; i < iterationCount; ++i)
            {
                ref var poseA = ref posesA[i];
                ref var poseB = ref posesB[i];
                batcher.Add(a, b, poseB.Position - poseA.Position, poseA.Orientation, poseB.Orientation, 0.1f, 0);
            }
            batcher.Flush();
        }

        unsafe static void Test<TA, TB>(ref TA a, ref TB b, ref Buffer<RigidPose> posesA, ref Buffer<RigidPose> posesB,
            BufferPool pool, Shapes shapes, CollisionTaskRegistry registry, int iterationCount)
                        where TA : struct, IShape where TB : struct, IShape
        {
            int count = 0;
            var callbacks = new TestCollisionCallbacks { Count = &count };
            TestPair(ref a, ref b, ref posesA, ref posesB, ref callbacks, pool, shapes, registry, 64);
            count = 0;
            var start = Stopwatch.GetTimestamp();
            TestPair(ref a, ref b, ref posesA, ref posesB, ref callbacks, pool, shapes, registry, iterationCount);
            var end = Stopwatch.GetTimestamp();
            var time = (end - start) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Completed {count} {typeof(TA).Name}-{typeof(TB).Name} pairs, time (ms): {1e3 * time}, time per pair (ns): {1e9 * time / *callbacks.Count}");
            //Console.WriteLine($"{typeof(TA).Name}-{typeof(TB).Name} {1e9 * time / *callbacks.Count}");
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static float TestPair<TA, TAWide, TB, TBWide, TDistanceTester>(ref TAWide a, ref TBWide b, ref TDistanceTester tester,
            ref Buffer<RigidPose> posesA, ref Buffer<RigidPose> posesB, int iterationCount)
            where TA : IShape where TB : IShape
            where TAWide : struct, IShapeWide<TA> where TBWide : struct, IShapeWide<TB>
            where TDistanceTester : IPairDistanceTester<TAWide, TBWide>
        {
            var distanceSum = Vector<float>.Zero;
            for (int i = 0; i < iterationCount; ++i)
            {
                ref var poseA = ref posesA[i];
                ref var poseB = ref posesB[i];
                Vector3Wide.Broadcast(poseB.Position - poseA.Position, out var offsetB);
                QuaternionWide.Broadcast(poseA.Orientation, out var orientationA);
                QuaternionWide.Broadcast(poseB.Orientation, out var orientationB);
                tester.Test(a, b, offsetB, orientationA, orientationB, Vector<int>.Zero, out var intersected, out var distance, out var closestA, out var normal);
                distanceSum += distance;
            }
            return distanceSum[0];
        }

        unsafe static void Test<TA, TAWide, TB, TBWide, TDistanceTester>(in TA a, in TB b,
            ref Buffer<RigidPose> posesA, ref Buffer<RigidPose> posesB, int iterationCount)
            where TA : IShape where TB : IShape
            where TAWide : struct, IShapeWide<TA> where TBWide : struct, IShapeWide<TB>
            where TDistanceTester : IPairDistanceTester<TAWide, TBWide>
        {
            TAWide aWide = default;
            aWide.Broadcast(a);
            TBWide bWide = default;
            bWide.Broadcast(b);
            var tester = default(TDistanceTester);
            TestPair<TA, TAWide, TB, TBWide, TDistanceTester>(ref aWide, ref bWide, ref tester, ref posesA, ref posesB, 64);
            var start = Stopwatch.GetTimestamp();
            TestPair<TA, TAWide, TB, TBWide, TDistanceTester>(ref aWide, ref bWide, ref tester, ref posesA, ref posesB, iterationCount);
            var end = Stopwatch.GetTimestamp();
            var time = (end - start) / (double)Stopwatch.Frequency;
            var instanceCount = Vector<float>.Count * iterationCount;
            Console.WriteLine($"Completed {instanceCount} {typeof(TA).Name}-{typeof(TB).Name} distance test instances using {typeof(TDistanceTester).Name}, time (ms): {1e3 * time}, time per instance (ns): {1e9 * time / instanceCount}");
        }

        static void GetRandomPose(Random random, out RigidPose pose)
        {
            pose.Position = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());

            float orientationLengthSquared;
            do
            {
                pose.Orientation = new BepuUtilities.Quaternion
                {
                    X = 2 * (float)random.NextDouble() - 1,
                    Y = 2 * (float)random.NextDouble() - 1,
                    Z = 2 * (float)random.NextDouble() - 1,
                    W = 2 * (float)random.NextDouble() - 1
                };
            }
            while ((orientationLengthSquared = pose.Orientation.LengthSquared()) < 1e-5f);
            var inverseLength = 1f / MathF.Sqrt(orientationLengthSquared);
            pose.Orientation.X *= inverseLength;
            pose.Orientation.Y *= inverseLength;
            pose.Orientation.Z *= inverseLength;
            pose.Orientation.W *= inverseLength;
        }

        public static void Test()
        {
            var pool = new BufferPool();
            var random = new Random(5);
            var registry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
            var sphere = new Sphere(1);
            var capsule = new Capsule(0.5f, 1f);
            var box = new Box(1f, 1f, 1f);
            var triangle = new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1));
            var cylinder = new Cylinder(0.5f, 1f);

            const int pointCount = 8192;
            var points = new QuickList<Vector3>(pointCount, pool);
            //points.Allocate(pool) = new Vector3(0, 0, 0);
            //points.Allocate(pool) = new Vector3(0, 0, 1);
            //points.Allocate(pool) = new Vector3(0, 1, 0);
            //points.Allocate(pool) = new Vector3(0, 1, 1);
            //points.Allocate(pool) = new Vector3(1, 0, 0);
            //points.Allocate(pool) = new Vector3(1, 0, 1);
            //points.Allocate(pool) = new Vector3(1, 1, 0);
            //points.Allocate(pool) = new Vector3(1, 1, 1);
            for (int i = 0; i < pointCount; ++i)
            {
                points.AllocateUnsafely() = new Vector3((float)random.NextDouble(), 1 * (float)random.NextDouble(), (float)random.NextDouble());
                //points.AllocateUnsafely() = new Vector3(0, 1, 0) + Vector3.Normalize(new Vector3((float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1)) * (float)random.NextDouble();
            }
            
            var pointsBuffer = points.Span.Slice(points.Count);
            ConvexHullHelper.CreateShape(pointsBuffer, pool, out _, out var convexHull);

            var poseA = new RigidPose { Position = new Vector3(0, 0, 0), Orientation = BepuUtilities.Quaternion.Identity };
            var poseB = new RigidPose { Position = new Vector3(0, 1, 0), Orientation = BepuUtilities.Quaternion.Identity };
            Shapes shapes = new Shapes(pool, 32);

            int iterationCount = 1 << 22;
            pool.Take<RigidPose>(iterationCount, out var posesA);
            pool.Take<RigidPose>(iterationCount, out var posesB);
            for (int i = 0; i < iterationCount; ++i)
            {
                GetRandomPose(random, out posesA[i]);
                GetRandomPose(random, out posesB[i]);
            }


            //Test(ref sphere, ref sphere, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref sphere, ref capsule, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref sphere, ref box, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref sphere, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref sphere, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref sphere, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref capsule, ref capsule, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref capsule, ref box, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref capsule, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref capsule, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref capsule, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref box, ref box, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref box, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref box, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref box, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref triangle, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref triangle, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref triangle, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref cylinder, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            //Test(ref cylinder, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref convexHull, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);


            //Test<Sphere, SphereWide, Sphere, SphereWide, SpherePairDistanceTester>(sphere, sphere, ref posesA, ref posesB, iterationCount);
            //Test<Sphere, SphereWide, Capsule, CapsuleWide, SphereCapsuleDistanceTester>(sphere, capsule, ref posesA, ref posesB, iterationCount);
            //Test<Sphere, SphereWide, Box, BoxWide, SphereBoxDistanceTester>(sphere, box, ref posesA, ref posesB, iterationCount);
            //Test<Sphere, SphereWide, Triangle, TriangleWide, SphereTriangleDistanceTester>(sphere, triangle, ref posesA, ref posesB, iterationCount);
            //Test<Sphere, SphereWide, Cylinder, CylinderWide, SphereCylinderDistanceTester>(sphere, cylinder, ref posesA, ref posesB, iterationCount);
            //Test<Capsule, CapsuleWide, Capsule, CapsuleWide, CapsulePairDistanceTester>(capsule, capsule, ref posesA, ref posesB, iterationCount);
            //Test<Capsule, CapsuleWide, Box, BoxWide, CapsuleBoxDistanceTester>(capsule, box, ref posesA, ref posesB, iterationCount);
            //Test<Capsule, CapsuleWide, Triangle, TriangleWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(capsule, triangle, ref posesA, ref posesB, iterationCount);
            //Test<Capsule, CapsuleWide, Cylinder, CylinderWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>(capsule, cylinder, ref posesA, ref posesB, iterationCount);
            //Test<Cylinder, CylinderWide, Box, BoxWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Box, BoxWide, BoxSupportFinder>>(cylinder, box, ref posesA, ref posesB, iterationCount);
            //Test<Cylinder, CylinderWide, Triangle, TriangleWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(cylinder, triangle, ref posesA, ref posesB, iterationCount);
            //Test<Cylinder, CylinderWide, Cylinder, CylinderWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>(cylinder, cylinder, ref posesA, ref posesB, iterationCount);
            //Test<Box, BoxWide, Box, BoxWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>>(box, box, ref posesA, ref posesB, iterationCount);
            //Test<Box, BoxWide, Triangle, TriangleWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(box, triangle, ref posesA, ref posesB, iterationCount);
            //Test<Triangle, TriangleWide, Triangle, TriangleWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(triangle, triangle, ref posesA, ref posesB, iterationCount);
            Console.WriteLine($"Done. Hit enter to exit.");
            Console.ReadLine();
        }
    }
}
