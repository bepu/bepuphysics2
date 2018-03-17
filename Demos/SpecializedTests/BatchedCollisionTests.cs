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
        struct TestCollisionCallbacks : ICollisionCallbacks
        {
            public int Count;

            public unsafe void OnPairCompleted(int pairId, ConvexContactManifold* manifold)
            {
                ref var contact = ref manifold->Contact0;
                var extra = 1e-16 * (contact.Depth + contact.Offset.X + manifold->Normal.X);
                Count += 1 + (int)extra;
            }
            public unsafe void OnPairCompleted(int pairId, NonconvexContactManifold* manifold)
            {
                ref var contact = ref manifold->Contact0;
                var extra = 1e-16 * (contact.Depth + contact.Offset.X + contact.Normal.X);
                Count += 1 + (int)extra;
            }
            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ConvexContactManifold* manifold)
            {
            }

            public bool AllowCollisionTesting(int pairId, int childA, int childB)
            {
                return true;
            }

        }


        static void TestPair<TA, TB>(ref TA a, ref TB b, ref RigidPose poseA, ref RigidPose poseB,
            ref TestCollisionCallbacks callbacks, BufferPool pool, Shapes shapes, CollisionTaskRegistry registry, int iterationCount)
            where TA : struct, IShape where TB : struct, IShape
        {
            var batcher = new CollisionBatcher<TestCollisionCallbacks>(pool, shapes, registry, callbacks);
            for (int i = 0; i < iterationCount; ++i)
            {
                batcher.Add(a, b, ref poseA, ref poseB, 0);
                batcher.Add(a, b, ref poseA, ref poseB, 0);
                batcher.Add(a, b, ref poseA, ref poseB, 0);
                batcher.Add(a, b, ref poseA, ref poseB, 0);
            }
            batcher.Flush();
        }

        static void Test<TA, TB>(ref TA a, ref TB b, ref RigidPose poseA, ref RigidPose poseB,
            BufferPool pool, Shapes shapes, CollisionTaskRegistry registry, int iterationCount)
                        where TA : struct, IShape where TB : struct, IShape
        {
            var callbacks = new TestCollisionCallbacks();
            TestPair(ref a, ref b, ref poseA, ref poseB, ref callbacks, pool, shapes, registry, 64);
            var start = Stopwatch.GetTimestamp();
            TestPair(ref a, ref b, ref poseA, ref poseB, ref callbacks, pool, shapes, registry, iterationCount);
            var end = Stopwatch.GetTimestamp();
            var time = (end - start) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Completed {callbacks.Count} {typeof(TA).Name}-{typeof(TB).Name} pairs, time (ms): {1e3 * time}, time per pair (ns): {1e9 * time / callbacks.Count}");
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
            var sphere = new Sphere(1);
            var capsule = new Capsule(0.5f, 1f);
            var box = new Box(1f, 1f, 1f);
            var poseA = new RigidPose { Position = new Vector3(0, 0, 0), Orientation = BepuUtilities.Quaternion.Identity };
            var poseB = new RigidPose { Position = new Vector3(0, 1, 0), Orientation = BepuUtilities.Quaternion.Identity };
            Shapes shapes = new Shapes(pool, 32);

            Test(ref sphere, ref sphere, ref poseA, ref poseB, pool, shapes, registry, 1 << 25);
            Test(ref sphere, ref capsule, ref poseA, ref poseB, pool, shapes, registry, 1 << 25);
            Test(ref sphere, ref box, ref poseA, ref poseB, pool, shapes, registry, 1 << 25);
            Test(ref capsule, ref capsule, ref poseA, ref poseB, pool, shapes, registry, 1 << 25);
            Test(ref capsule, ref box, ref poseA, ref poseB, pool, shapes, registry, 1 << 25);
            Test(ref box, ref box, ref poseA, ref poseB, pool, shapes, registry, 1 << 25);
            Console.ReadKey();
        }
    }
}
