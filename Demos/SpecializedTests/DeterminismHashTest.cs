using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using DemoContentLoader;
using System.Runtime.CompilerServices;

namespace Demos.SpecializedTests
{
    public static class DeterminismHashTest<T> where T : Demo, new()
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long ComputeHash(ref float v, long constant)
        {
            return Unsafe.As<float, int>(ref v) * constant;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long ComputeHash(ref Vector3 v, long constant)
        {
            return constant * (ComputeHash(ref v.X, 13) + ComputeHash(ref v.Y, 31) + ComputeHash(ref v.Z, 53));
        }

        static long ExecuteSimulation(ContentArchive content, int frameCount)
        {
            var demo = new T();
            demo.Initialize(content, new DemoRenderer.Camera(1, 1, 1, 1));
            Console.Write("Completed frames: ");
            for (int i = 0; i < frameCount; ++i)
            {
                demo.Update(null, null, null, 1 / 60f);
                if ((i + 1) % 32 == 0)
                    Console.Write($"{i + 1}, ");
            }
            long hash = 0;

            for (int setIndex = 0; setIndex < demo.Simulation.Bodies.Sets.Length; ++setIndex)
            {
                ref var set = ref demo.Simulation.Bodies.Sets[setIndex];
                if (set.Allocated)
                {
                    for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                    {
                        ref var pose = ref set.Poses[bodyIndex];
                        ref var velocity = ref set.Velocities[bodyIndex];
                        var poseHash = ComputeHash(ref pose.Position, 89) + ComputeHash(ref pose.Orientation.X, 107) + ComputeHash(ref pose.Orientation.Y, 113) + ComputeHash(ref pose.Orientation.Z, 131) + ComputeHash(ref pose.Orientation.W, 149);
                        var velocityHash = ComputeHash(ref velocity.Linear, 211) + ComputeHash(ref velocity.Angular, 397);
                        hash += set.IndexToHandle[bodyIndex] * (poseHash + velocityHash);
                    }
                }
            }
            demo.Dispose();
            Console.WriteLine();
            Console.WriteLine($"Simulation hash after {frameCount} frames: {hash}");
            return hash;
        }

        public static void Test(ContentArchive archive, int runCount, int frameCount)
        {
            var originalHash = ExecuteSimulation(archive, frameCount);
            Console.WriteLine($"Completed initial run.");
            for (int i = 0; i < runCount; ++i)
            {
                var hash = ExecuteSimulation(archive, frameCount);
                if (originalHash != hash)
                {
                    Console.WriteLine($"Local determinism failure.");
                }
            }
            Console.WriteLine($"All runs complete.");
        }
    }
}
