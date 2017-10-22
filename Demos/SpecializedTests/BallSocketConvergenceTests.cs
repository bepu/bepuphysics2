using System;
using System.Diagnostics;
using System.Numerics;

namespace Demos.SpecializedTests
{
    static class BallSocketConvergenceTests
    {
        public static void Test()
        {
            const int width = 32;
            const int height = 32;
            const int length = 32;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3()),
                new BallSocketConstraintBuilder(),
                width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);

            var threadDispatcher = new SimpleThreadDispatcher(8);

            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            const int frameCount = 128;
            simulation.Solver.IterationCount = iterationCount;

            simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var samples = new SimulationTimeSamples(frameCount);
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = simulation.Bodies.GetBodyEnergyHeuristic();

                //simulation.Timestep(dt);
                simulation.Timestep(dt, threadDispatcher);

                samples.RecordFrame(simulation);

                var energyAfter = simulation.Bodies.GetBodyEnergyHeuristic();
                int sampledBodyIndex = width;
                var samplePose = simulation.Bodies.Poses[sampledBodyIndex];
                var sampleVelocity = simulation.Bodies.Velocities[sampledBodyIndex];
                //for (int i =0; i < simulation.Bodies.BodyCount; ++i)
                //{
                //    simulation.Bodies.GetPose(simulation.Bodies.IndexToHandle[i], out var pose);
                //    simulation.Bodies.GetVelocity(simulation.Bodies.IndexToHandle[i], out var velocity);
                //    Console.WriteLine($"Sample {i} position: {pose.Position}, velocity: {velocity.Linear}");
                //}
                Console.WriteLine($"Sample {sampledBodyIndex} position: {samplePose.Position}, velocity: {sampleVelocity.Linear}");

                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");
            }
            
            var multiplier = 1e3 / frameCount;
            Console.WriteLine($"Simulation time (ms):       {multiplier * samples.Simulation.ComputeStats().Total}");
            Console.WriteLine($"Body opt time (ms):         {multiplier * samples.BodyOptimizer.ComputeStats().Total}");
            Console.WriteLine($"Constraint opt time (ms):   {multiplier * samples.ConstraintOptimizer.ComputeStats().Total}");
            Console.WriteLine($"Batch compress time (ms):   {multiplier * samples.BatchCompressor.ComputeStats().Total}");
            Console.WriteLine($"Pose integrate time (ms):   {multiplier * samples.PoseIntegrator.ComputeStats().Total}");
            Console.WriteLine($"Solve time (ms):            {multiplier * samples.Solver.ComputeStats().Total}");


            threadDispatcher.Dispose();
            simulation.BufferPool.Clear();

        }



    }
}

