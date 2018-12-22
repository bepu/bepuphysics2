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
        struct MotionState
        {
            public RigidPose Pose;
            public BodyVelocity Velocity;
        }
        static Dictionary<int, MotionState> ExecuteSimulation(ContentArchive content, int frameCount)
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
            var motionStates = new Dictionary<int, MotionState>();
            for (int setIndex = 0; setIndex < demo.Simulation.Bodies.Sets.Length; ++setIndex)
            {
                ref var set = ref demo.Simulation.Bodies.Sets[setIndex];
                if (set.Allocated)
                {
                    for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                    {
                        motionStates.Add(set.IndexToHandle[bodyIndex], new MotionState { Pose = set.Poses[bodyIndex], Velocity = set.Velocities[bodyIndex] });
                    }
                }
            }
            demo.Dispose();
            Console.WriteLine();
            return motionStates;
        }

        public static void Test(ContentArchive archive, int runCount, int frameCount)
        {
            var initialStates = ExecuteSimulation(archive, frameCount);
            Console.WriteLine($"Completed initial run.");
            for (int i = 0; i < runCount; ++i)
            {
                var states = ExecuteSimulation(archive, frameCount);
                Console.Write($"Completed iteration {i}; checking... ");
                if (states.Count != initialStates.Count)
                    Console.WriteLine("DETERMINISM FAILURE: Differing body count.");
                foreach (var state in states)
                {
                    if (!initialStates.TryGetValue(state.Key, out var initialState))
                        Console.WriteLine($"FAILURE: Body {state.Key} does not exist in first run results.");
                    else
                    {
                        if (state.Value.Pose.Position != initialState.Pose.Position)
                            Console.WriteLine($"FAILURE: Position, current: {state.Value.Pose.Position}, original: {initialState.Pose.Position}");
                        if (state.Value.Pose.Orientation != initialState.Pose.Orientation)
                            Console.WriteLine($"FAILURE: Orientation, current: {state.Value.Pose.Orientation}, original: {initialState.Pose.Orientation}");
                        if (state.Value.Velocity.Linear != initialState.Velocity.Linear)
                            Console.WriteLine($"FAILURE: Linear velocity, current: {state.Value.Velocity.Linear}, original: {initialState.Velocity.Linear}");
                        if (state.Value.Velocity.Angular != initialState.Velocity.Angular)
                            Console.WriteLine($"FAILURE: Angular velocity, current: {state.Value.Velocity.Angular}, original: {initialState.Velocity.Angular}");
                    }
                }
                Console.WriteLine($"Test {i} complete.");
            }
            Console.WriteLine($"All runs complete.");
        }
    }
}
