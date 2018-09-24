using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using DemoContentLoader;

namespace Demos.SpecializedTests
{
    public static class DeterminismTest<T> where T : Demo, new()
    {
        static Dictionary<int, RigidPose> ExecuteSimulation(ContentArchive content, int frameCount)
        {
            var demo = new T();
            demo.Initialize(content, new DemoRenderer.Camera(1, 1, 1, 1));
            Console.Write("Completed frames: ");
            for (int i = 0; i < frameCount; ++i)
            {
                demo.Update(null, 1 / 60f);
                if (i % 32 == 0)
                    Console.Write($"{i}, ");
            }
            var bodyPoses = new Dictionary<int, RigidPose>();
            for (int setIndex = 0; setIndex < demo.Simulation.Bodies.Sets.Length; ++setIndex)
            {
                ref var set = ref demo.Simulation.Bodies.Sets[setIndex];
                if (set.Allocated)
                {
                    for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                    {
                        bodyPoses.Add(set.IndexToHandle[bodyIndex], set.Poses[bodyIndex]);
                    }
                }
            }
            Console.WriteLine();
            Console.WriteLine($"Completed run.");
            demo.Dispose();

            return bodyPoses;
        }

        public static void Test(ContentArchive archive, int runCount, int frameCount)
        {
            var initialPoses = ExecuteSimulation(archive, frameCount);
            for (int i = 0; i < runCount; ++i)
            {
                var poses = ExecuteSimulation(archive, frameCount);
                Console.WriteLine($"Completed iteration {i}; checking...");
                if (poses.Count != initialPoses.Count)
                    Console.WriteLine("DETERMINISM FAILURE: Differing body count.");
                foreach (var bodyPose in poses)
                {
                    if (!initialPoses.TryGetValue(bodyPose.Key, out var initialPose))
                        Console.WriteLine($"DETERMINISM FAILURE: Body {bodyPose.Key} does not exist in first run results.");
                    if (bodyPose.Value.Position != initialPose.Position)
                        Console.WriteLine($"DETERMINISM FAILURE: body position is not the same. Current position: {bodyPose.Value.Position}, original position: {initialPose.Position}");
                    if (bodyPose.Value.Orientation != initialPose.Orientation)
                        Console.WriteLine($"DETERMINISM FAILURE: body orientation is not the same. Current orientation: {bodyPose.Value.Orientation}, original orientation: {initialPose.Orientation}");


                }
                Console.WriteLine($"Test {i} complete.");
            }
            Console.WriteLine($"All runs complete.");
        }
    }
}
