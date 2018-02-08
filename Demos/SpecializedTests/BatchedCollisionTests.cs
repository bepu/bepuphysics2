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

        static double Test(Action<int> action, int iterationCount)
        {
            action(64);

            var start = Stopwatch.GetTimestamp();
            action(iterationCount);
            var end = Stopwatch.GetTimestamp();
            return (end - start) / (double)Stopwatch.Frequency;
        }



        public static void Test()
        {
            var pool = new BufferPool();
            var registry = new CollisionTaskRegistry();
            var task = new SpherePairCollisionTask();
            registry.Register(task);
            var continuations = new ContinuationsTest();
            var filters = new SubtaskFiltersTest();
            var sphere = new Sphere(1);
            var poseA = new RigidPose { Position = new Vector3(0, 0, 0), Orientation = BepuUtilities.Quaternion.Identity };
            var poseB = new RigidPose { Position = new Vector3(0, 1, 0), Orientation = BepuUtilities.Quaternion.Identity };
            void action(int iterationCount)
            {
                var batcher = new StreamingBatcher(pool, registry);
                for (int i = 0; i < iterationCount; ++i)
                {
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new ContinuationIndex(), ref continuations, ref filters);
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new ContinuationIndex(), ref continuations, ref filters);
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new ContinuationIndex(), ref continuations, ref filters);
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new ContinuationIndex(), ref continuations, ref filters);
                }
                batcher.Flush(ref continuations, ref filters);
            }
            var time0 = Test(action, 1<<25);
            Console.WriteLine($"Completed count: {continuations.Count}, time (ms): {1e3 * time0}");
            Console.ReadKey();
        }
    }
}
