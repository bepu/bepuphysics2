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

namespace Demos.SpecializedTests
{
    public static class BatchedCollisionTests
    {
        struct ContinuationsTest : IContinuations
        {
            public int Count;
            public unsafe void Notify(ContinuationIndex continuationId, ContactManifold* manifold)
            {
                //Console.WriteLine($"Completed {continuationId}:");
                //var normals = &manifold->Normal0;
                //var offsets = &manifold->Offset0;
                //var depths = &manifold->Depth0;
                //if (manifold->Convex)
                //{
                //    for (int i = 0; i < manifold->ContactCount; ++i)
                //    {
                //        Console.WriteLine($"{i}: P: {offsets[i]}, N: {manifold->ConvexNormal}, D: {depths[i]}");
                //    }
                //}
                //else
                //{
                //    for (int i = 0; i < manifold->ContactCount; ++i)
                //    {
                //        Console.WriteLine($"{i}: P: {offsets[i]}, N: {normals[i]}, D: {depths[i]}");
                //    }
                //}
                var extra = 1e-16 * (manifold->Depth0 + manifold->Offset0.X + manifold->Normal0.X);
                Count += 1 + (int)extra;
            }
        }
        struct SubtaskFiltersTest : ICollisionSubtaskFilters
        {
            public bool AllowCollisionTesting(CollidablePair parent, int childA, int childB)
            {
                return true;
            }

            public unsafe void Configure(CollidablePair parent, int childA, int childB, ContactManifold* manifold)
            {
            }
        }

        static void TestPair<TA, TB>(ref TA a, ref TB b, ref RigidPose poseA, ref RigidPose poseB,
            ref ContinuationsTest continuations, ref SubtaskFiltersTest filters, BufferPool pool, CollisionTaskRegistry registry, int iterationCount)
            where TA : struct, IShape where TB : struct, IShape
        {
            var batcher = new StreamingBatcher(pool, registry);
            for (int i = 0; i < iterationCount; ++i)
            {
                batcher.Add(ref a, ref b, ref poseA, ref poseB, new ContinuationIndex(0, 0, 0), ref continuations, ref filters);
                batcher.Add(ref a, ref b, ref poseA, ref poseB, new ContinuationIndex(0, 0, 0), ref continuations, ref filters);
                batcher.Add(ref a, ref b, ref poseA, ref poseB, new ContinuationIndex(0, 0, 0), ref continuations, ref filters);
                batcher.Add(ref a, ref b, ref poseA, ref poseB, new ContinuationIndex(0, 0, 0), ref continuations, ref filters);
            }
            batcher.Flush(ref continuations, ref filters);
        }

        static void Test<TA, TB>(ref TA a, ref TB b, ref RigidPose poseA, ref RigidPose poseB,
            BufferPool pool, CollisionTaskRegistry registry, int iterationCount)
                        where TA : struct, IShape where TB : struct, IShape
        {
            var continuations = new ContinuationsTest();
            var filters = new SubtaskFiltersTest();
            TestPair(ref a, ref b, ref poseA, ref poseB, ref continuations, ref filters, pool, registry, 64);
            var start = Stopwatch.GetTimestamp();
            TestPair(ref a, ref b, ref poseA, ref poseB, ref continuations, ref filters, pool, registry, iterationCount);
            var end = Stopwatch.GetTimestamp();
            var time = (end - start) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Completed {continuations.Count} {typeof(TA).Name}-{typeof(TB).Name} pairs, time (ms): {1e3 * time}, time per pair (ns): {1e9 * time / continuations.Count}");
        }



        public static void Test()
        {
            var pool = new BufferPool();
            var registry = new CollisionTaskRegistry();
            registry.Register(new SpherePairCollisionTask());
            registry.Register(new SphereCapsuleCollisionTask());
            registry.Register(new SphereBoxCollisionTask());
            registry.Register(new CapsulePairCollisionTask());
            registry.Register(new CapsuleBoxCollisionTask());
            registry.Register(new BoxPairCollisionTask());
            var continuations = new ContinuationsTest();
            var filters = new SubtaskFiltersTest();
            var sphere = new Sphere(1);
            var capsule = new Capsule(0.5f, 1f);
            var box = new Box(1f, 1f, 1f);
            var poseA = new RigidPose { Position = new Vector3(0, 0, 0), Orientation = BepuUtilities.Quaternion.Identity };
            var poseB = new RigidPose { Position = new Vector3(0, 1, 0), Orientation = BepuUtilities.Quaternion.Identity };

            Test(ref sphere, ref sphere, ref poseA, ref poseB, pool, registry, 1 << 25);
            Test(ref sphere, ref capsule, ref poseA, ref poseB, pool, registry, 1 << 25);
            Test(ref sphere, ref box, ref poseA, ref poseB, pool, registry, 1 << 25);
            Test(ref capsule, ref capsule, ref poseA, ref poseB, pool, registry, 1 << 25);
            Test(ref capsule, ref box, ref poseA, ref poseB, pool, registry, 1 << 25);
            Test(ref box, ref box, ref poseA, ref poseB, pool, registry, 1 << 25);
            Console.ReadKey();
        }
    }
}
