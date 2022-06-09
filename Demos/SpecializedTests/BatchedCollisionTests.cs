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

            public unsafe void OnPairCompleted<TManifold>(int pairId, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                if (manifold.Count > 0)
                {
                    manifold.GetContact(0, out var offset, out var normal, out var depth, out var featureId);
                    var extra = 1e-16 * (depth + offset.X + normal.X);
                    *Count += 1 + (int)extra;
                }
                else
                {
                    ++*Count;
                }
            }

            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold)
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
            var batcher = new CollisionBatcher<TestCollisionCallbacks>(pool, shapes, registry, Demo.TimestepDuration, callbacks);
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
            TestPair(ref a, ref b, ref posesA, ref posesB, ref callbacks, pool, shapes, registry, 256);
            count = 0;
            var start = Stopwatch.GetTimestamp();
            TestPair(ref a, ref b, ref posesA, ref posesB, ref callbacks, pool, shapes, registry, iterationCount);
            var end = Stopwatch.GetTimestamp();
            var time = (end - start) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Completed {count} {typeof(TA).Name}-{typeof(TB).Name} pairs, time (ms): {1e3 * time}, time per pair (ns): {1e9 * time / *callbacks.Count}");
            //Console.WriteLine($"{typeof(TA).Name}-{typeof(TB).Name}, {1e9 * time / *callbacks.Count}");
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
            where TA : unmanaged, IShape where TB : unmanaged, IShape
            where TAWide : unmanaged, IShapeWide<TA> where TBWide : unmanaged, IShapeWide<TB>
            where TDistanceTester : struct, IPairDistanceTester<TAWide, TBWide>
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
            pose.Position = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());

            float orientationLengthSquared;
            do
            {
                pose.Orientation = new Quaternion
                {
                    X = 2 * random.NextSingle() - 1,
                    Y = 2 * random.NextSingle() - 1,
                    Z = 2 * random.NextSingle() - 1,
                    W = 2 * random.NextSingle() - 1
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

            const int pointCount = 64;
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
                points.AllocateUnsafely() = new Vector3(random.NextSingle(), 1 * random.NextSingle(), random.NextSingle());
                //points.AllocateUnsafely() = new Vector3(0, 1, 0) + Vector3.Normalize(new Vector3(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1)) * random.NextSingle();
            }

            Shapes shapes = new Shapes(pool, 32);
            var pointsBuffer = points.Span.Slice(points.Count);
            ConvexHullHelper.CreateShape(pointsBuffer, pool, out _, out var convexHull);

            using var compoundBuilder = new CompoundBuilder(pool, shapes, 64);
            //COMPOUND
            var legShape = new Box(0.2f, 1, 0.2f);
            var legInverseInertia = legShape.ComputeInertia(1f);
            var legShapeIndex = shapes.Add(legShape);
            var legPose0 = new RigidPose { Position = new Vector3(-1.5f, 0, -1.5f), Orientation = Quaternion.Identity };
            var legPose1 = new RigidPose { Position = new Vector3(-1.5f, 0, 1.5f), Orientation = Quaternion.Identity };
            var legPose2 = new RigidPose { Position = new Vector3(1.5f, 0, -1.5f), Orientation = Quaternion.Identity };
            var legPose3 = new RigidPose { Position = new Vector3(1.5f, 0, 1.5f), Orientation = Quaternion.Identity };
            compoundBuilder.Add(legShapeIndex, legPose0, legInverseInertia.InverseInertiaTensor, 1);
            compoundBuilder.Add(legShapeIndex, legPose1, legInverseInertia.InverseInertiaTensor, 1);
            compoundBuilder.Add(legShapeIndex, legPose2, legInverseInertia.InverseInertiaTensor, 1);
            compoundBuilder.Add(legShapeIndex, legPose3, legInverseInertia.InverseInertiaTensor, 1);
            var tableTopPose = new RigidPose { Position = new Vector3(0, 0.6f, 0), Orientation = Quaternion.Identity };
            var tableTopShape = new Box(3.2f, 0.2f, 3.2f);
            compoundBuilder.Add(tableTopShape, tableTopPose, 3);

            compoundBuilder.BuildDynamicCompound(out var tableChildren, out var tableInertia, out var tableCenter);
            compoundBuilder.Reset();
            var compound = new Compound(tableChildren);

            //BIGCOMPOUND
            var treeCompoundBoxShape = new Box(0.5f, 1.5f, 1f);
            var treeCompoundBoxShapeIndex = shapes.Add(treeCompoundBoxShape);
            var childInertia = treeCompoundBoxShape.ComputeInertia(1);
            for (int i = 0; i < 64; ++i)
            {
                RigidPose localPose;
                localPose.Position = new Vector3(12, 12, 12) * (0.5f * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) - Vector3.One);
                float orientationLengthSquared;
                do
                {
                    localPose.Orientation = new Quaternion(random.NextSingle(), random.NextSingle(), random.NextSingle(), random.NextSingle());
                    orientationLengthSquared = QuaternionEx.LengthSquared(ref localPose.Orientation);
                }
                while (orientationLengthSquared < 1e-9f);
                QuaternionEx.Scale(localPose.Orientation, 1f / MathF.Sqrt(orientationLengthSquared), out localPose.Orientation);
                //Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI, out localPose.Orientation);

                compoundBuilder.Add(treeCompoundBoxShapeIndex, localPose, childInertia.InverseInertiaTensor, 1);
            }
            compoundBuilder.BuildDynamicCompound(out var children, out var inertia, out var center);
            compoundBuilder.Reset();
            var bigCompound = new BigCompound(children, shapes, pool);

            //MESH
            DemoMeshHelper.CreateDeformedPlane(8, 8, (x, y) => { return new Vector3(x * 2 - 8, 3 * MathF.Sin(x) * MathF.Sin(y), y * 2 - 8); }, Vector3.One, pool, out var mesh);

            int iterationCount = 1 << 20;
            pool.Take<RigidPose>(iterationCount, out var posesA);
            pool.Take<RigidPose>(iterationCount, out var posesB);
            for (int i = 0; i < iterationCount; ++i)
            {
                GetRandomPose(random, out posesA[i]);
                GetRandomPose(random, out posesB[i]);
            }


            Test(ref sphere, ref sphere, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref capsule, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref box, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref compound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref sphere, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref capsule, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref box, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref compound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref capsule, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref box, ref box, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref box, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref box, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref box, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref box, ref compound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref box, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref box, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref triangle, ref triangle, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref triangle, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref triangle, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref triangle, ref compound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref triangle, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref triangle, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref cylinder, ref cylinder, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref cylinder, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref cylinder, ref compound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref cylinder, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref cylinder, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref convexHull, ref convexHull, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref convexHull, ref compound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref convexHull, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref convexHull, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref compound, ref compound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref compound, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref compound, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref bigCompound, ref bigCompound, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref bigCompound, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);
            Test(ref mesh, ref mesh, ref posesA, ref posesB, pool, shapes, registry, iterationCount);


            Test<Sphere, SphereWide, Sphere, SphereWide, SpherePairDistanceTester>(sphere, sphere, ref posesA, ref posesB, iterationCount);
            Test<Sphere, SphereWide, Capsule, CapsuleWide, SphereCapsuleDistanceTester>(sphere, capsule, ref posesA, ref posesB, iterationCount);
            Test<Sphere, SphereWide, Box, BoxWide, SphereBoxDistanceTester>(sphere, box, ref posesA, ref posesB, iterationCount);
            Test<Sphere, SphereWide, Triangle, TriangleWide, SphereTriangleDistanceTester>(sphere, triangle, ref posesA, ref posesB, iterationCount);
            Test<Sphere, SphereWide, Cylinder, CylinderWide, SphereCylinderDistanceTester>(sphere, cylinder, ref posesA, ref posesB, iterationCount);
            Test<Capsule, CapsuleWide, Capsule, CapsuleWide, CapsulePairDistanceTester>(capsule, capsule, ref posesA, ref posesB, iterationCount);
            Test<Capsule, CapsuleWide, Box, BoxWide, CapsuleBoxDistanceTester>(capsule, box, ref posesA, ref posesB, iterationCount);
            Test<Capsule, CapsuleWide, Triangle, TriangleWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(capsule, triangle, ref posesA, ref posesB, iterationCount);
            Test<Capsule, CapsuleWide, Cylinder, CylinderWide, GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>(capsule, cylinder, ref posesA, ref posesB, iterationCount);
            Test<Cylinder, CylinderWide, Box, BoxWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Box, BoxWide, BoxSupportFinder>>(cylinder, box, ref posesA, ref posesB, iterationCount);
            Test<Cylinder, CylinderWide, Triangle, TriangleWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(cylinder, triangle, ref posesA, ref posesB, iterationCount);
            Test<Cylinder, CylinderWide, Cylinder, CylinderWide, GJKDistanceTester<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>>(cylinder, cylinder, ref posesA, ref posesB, iterationCount);
            Test<Box, BoxWide, Box, BoxWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>>(box, box, ref posesA, ref posesB, iterationCount);
            Test<Box, BoxWide, Triangle, TriangleWide, GJKDistanceTester<Box, BoxWide, BoxSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(box, triangle, ref posesA, ref posesB, iterationCount);
            Test<Triangle, TriangleWide, Triangle, TriangleWide, GJKDistanceTester<Triangle, TriangleWide, TriangleSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>>(triangle, triangle, ref posesA, ref posesB, iterationCount);
            Console.WriteLine($"Done. Hit enter to exit.");
            Console.ReadLine();
        }
    }
}
