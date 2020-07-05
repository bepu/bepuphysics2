using System.Numerics;
using DemoContentLoader;
using System.Runtime.CompilerServices;
using Xunit;
using Demos;

namespace DemoTests
{
    public static class TestUtilities
    {
        public static ContentArchive GetDemosContentArchive()
        {
            using (var stream = typeof(Demos.Demos.FountainStressTestDemo).Assembly.GetManifestResourceStream("Demos.Demos.contentarchive"))
            {
                return ContentArchive.Load(stream);
            }
        }

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

        static long ExecuteSimulation<T>(ContentArchive content, int frameCount) where T : Demo, new()
        {
            var demo = new T();
            demo.Initialize(content, new DemoRenderer.Camera(1, 1, 1, 1));
            for (int i = 0; i < frameCount; ++i)
            {
                demo.Update(null, null, null, 1 / 60f);
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
                        hash += set.IndexToHandle[bodyIndex].Value * (poseHash + velocityHash);
                    }
                }
            }
            demo.Dispose();
            return hash;
        }

        public static void TestDeterminism<T>(int runCount, int frameCount) where T : Demo, new()
        {
            var archive = GetDemosContentArchive();
            var originalHash = ExecuteSimulation<T>(archive, frameCount);
            for (int i = 0; i < runCount; ++i)
            {
                var hash = ExecuteSimulation<T>(archive, frameCount);
                Assert.True(originalHash == hash, $"Local determinism failure for {typeof(T).Name}.");
            }
        }
    }
}
