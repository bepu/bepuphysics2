using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class DeterminismTest
    {
        public static RigidPose[] ExecuteSimulation(int frameCount, BufferPool bufferPool, IThreadDispatcher threadDispatcher)
        {
            var simulation = Simulation.Create(bufferPool, new TestCallbacks());
            var shape = new Sphere(0.5f);
            var shapeIndex = simulation.Shapes.Add(ref shape);
            const int width = 8;
            const int height = 8;
            const int length = 8;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1.2f, 1.05f, 1.2f), new Vector3(1, 1, 1), 1f / (shape.Radius * shape.Radius * 2 / 3), shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, simulation, out var bodyHandles, out var constraintHandles);
            simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            simulation.Deterministic = true;

            ref var velocity = ref simulation.Bodies.Velocities[simulation.Bodies.HandleToIndex[bodyHandles[width]]];
            velocity.Linear = new Vector3(.1f, 0, 0.1f);
            velocity.Angular = new Vector3();


            for (int i = 0; i < frameCount; ++i)
            {
                simulation.Timestep(1 / 60f, threadDispatcher);
                //TODO: Probably should do add/remove on bodies alone, and possibly a variant that includes SOME constraints.
                //(not enough to limit the amount of potential nondeterminism from collisions)
                //Then we can add/remove those. Do need to control the seed too, though.
                //SimulationScrambling.AddRemoveChurn(simulation, 100, bodyHandles, constraintHandles);
            }

            var poses = new RigidPose[simulation.Bodies.Count];
            for (int i = 0; i < simulation.Bodies.Count; ++i)
            {
                poses[i] = simulation.Bodies.Poses[simulation.Bodies.HandleToIndex[bodyHandles[i]]];
            }
            simulation.Dispose();
            return poses;
        }

        public static void Test()
        {
            const int frameCount = 10000;
            var bufferPool = new BufferPool();
            SimpleThreadDispatcher dispatcher = new SimpleThreadDispatcher(Environment.ProcessorCount);
            var initialPoses = ExecuteSimulation(frameCount, bufferPool, dispatcher);
            Console.WriteLine($"Completed initial test.");
            const int testIterations = 100;
            for (int i = 0; i < testIterations; ++i)
            {
                var poses = ExecuteSimulation(frameCount, bufferPool, dispatcher);
                Console.WriteLine($"Completed iteration {i}; checking...");
                for (int j = 0; j < poses.Length; ++j)
                {
                    if (initialPoses[j].Position != poses[j].Position || initialPoses[j].Orientation != poses[j].Orientation)
                    {
                        Console.WriteLine($"DETERMINISM FAILURE, test {i}, body {j}. Expected <{initialPoses[j].Position}, {initialPoses[j].Orientation}>, got <{poses[j].Position}, {poses[j].Orientation}>");
                    }
                }
                Console.WriteLine($"Test complete.");
            }
        }
    }
}
