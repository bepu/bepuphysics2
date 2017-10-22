using BepuUtilities;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;

namespace Demos.SpecializedTests
{
    public static class AutoTester
    {
        public static SimulationTimeSamples Solve<TBodyBuilder, TConstraintBuilder, TConstraint>(TBodyBuilder bodyBuilder, TConstraintBuilder constraintBuilder,
            int width, int height, int length, int frameCount, int threadCount, IThreadDispatcher initializationThreadPool, IThreadDispatcher threadDispatcher)
            where TBodyBuilder : IBodyBuilder where TConstraintBuilder : IConstraintBuilder where TConstraint : IConstraintDescription<TConstraint>
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);
            GC.Collect(3, GCCollectionMode.Forced, true);
            SimulationSetup.BuildLattice(
                bodyBuilder, constraintBuilder,
                width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);

            SimulationScrambling.ScrambleBodies(simulation);
            SimulationScrambling.ScrambleConstraints(simulation.Solver);
            SimulationScrambling.ScrambleBodyConstraintLists(simulation);
            SimulationScrambling.AddRemoveChurn<TConstraint>(simulation, bodyHandles.Length * 2, bodyHandles, constraintHandles);

            const int batchCompressionIterations = 1000;
            simulation.SolverBatchCompressor.TargetCandidateFraction = .005f;
            simulation.SolverBatchCompressor.MaximumCompressionFraction = 0.0005f;
            for (int i = 0; i < batchCompressionIterations; ++i)
            {
                simulation.SolverBatchCompressor.Compress(simulation.BufferPool, initializationThreadPool);
            }

            //Attempt cache optimization.
            int bodyOptimizationIterations = bodyHandles.Length / 4;
            simulation.BodyLayoutOptimizer.OptimizationFraction = 0.005f;
            for (int i = 0; i < bodyOptimizationIterations; ++i)
            {
                simulation.BodyLayoutOptimizer.IncrementalOptimize(simulation.BufferPool, initializationThreadPool);
            }

            simulation.ConstraintLayoutOptimizer.OptimizationFraction = 0.044f;
            int constraintOptimizationIterations = 1024;
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                simulation.ConstraintLayoutOptimizer.Update(simulation.BufferPool, initializationThreadPool);
            }

            var simulationTimeSamples = new SimulationTimeSamples(frameCount);

            const float dt = 1 / 60f;
            const int iterationCount = 8;
            simulation.Solver.IterationCount = iterationCount;

            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                CacheBlaster.Blast();
                simulation.Timestep(dt, threadDispatcher);
                simulationTimeSamples.RecordFrame(simulation);
            }

            simulation.Dispose();
            simulation.BufferPool.Clear();


            return simulationTimeSamples;
        }

        static void WriteLine(StreamWriter writer, string text)
        {
            writer.WriteLine(text);
            Console.WriteLine(text);
        }
        static void Subtest<TBodyBuilder, TConstraintBuilder, TConstraint>(TBodyBuilder bodyBuilder, TConstraintBuilder constraintBuilder,
            int width, int height, int length, int frameCount, IThreadDispatcher initializationThreadPool, StreamWriter writer)
            where TBodyBuilder : IBodyBuilder where TConstraintBuilder : IConstraintBuilder where TConstraint : IConstraintDescription<TConstraint>
        {
            const int testsPerVariant = 8;
            WriteLine(writer, $"{width}x{height}x{length} lattice, {frameCount} frames:");
            var timings = new SimulationTimeSamples[Environment.ProcessorCount];


            //for (int threadCount = 1; threadCount <= 1; ++threadCount)
            for (int threadCount = 1; threadCount <= Environment.ProcessorCount; ++threadCount)
            {
                var threadPool = new SimpleThreadDispatcher(threadCount);
                SimulationTimeSamples bestTimings = null;
                double bestTime = double.MaxValue;
                for (int i = 0; i < testsPerVariant; ++i)
                {
                    var candidateTimings = Solve<TBodyBuilder, TConstraintBuilder, TConstraint>(bodyBuilder, constraintBuilder, width, height, length,
                        frameCount, threadCount, initializationThreadPool, threadPool);
                    var totalStats = candidateTimings.Simulation.ComputeStats();
                    WriteLine(writer,
                        $"{i} AVE: {Math.Round(1e3 * totalStats.Average, 2)}, " +
                        $"MIN: {Math.Round(1e3 * totalStats.Min, 2)}, " +
                        $"MAX: {Math.Round(1e3 * totalStats.Max, 2)}, " +
                        $"STD DEV: {Math.Round(1e3 * totalStats.StdDev, 3)}, ");
                    if (totalStats.Total < bestTime)
                    {
                        bestTime = totalStats.Total;
                        bestTimings = candidateTimings;
                    }
                }

                WriteLine(writer, $"{threadCount}T: " +
                    $"{Math.Round(bestTime * 1e3, 2)}, " +
                    $"BOPT: {Math.Round(bestTimings.BodyOptimizer.ComputeStats().Total * 1e3, 2)}, " +
                    $"COPT: {Math.Round(bestTimings.ConstraintOptimizer.ComputeStats().Total * 1e3, 2)}, " +
                    $"BCMP: {Math.Round(bestTimings.BatchCompressor.ComputeStats().Total * 1e3, 2)}, " +
                    $"POIN: {Math.Round(bestTimings.PoseIntegrator.ComputeStats().Total * 1e3, 2)}, " +
                    $"SOLV: {Math.Round(bestTimings.Solver.ComputeStats().Total * 1e3, 2)}");
                timings[threadCount - 1] = bestTimings;
                threadPool.Dispose();
            }
            int fastestIndex = 0;
            for (int i = 1; i < timings.Length; ++i)
            {
                if (timings[i].Simulation.ComputeStats().Total < timings[fastestIndex].Simulation.ComputeStats().Total)
                    fastestIndex = i;
            }
            WriteLine(writer, $"Scaling: {timings[0].Simulation.ComputeStats().Total / timings[fastestIndex].Simulation.ComputeStats().Total}");
        }

        public static void Test()
        {
            var memoryStream = new MemoryStream();
            var writer = new StreamWriter(memoryStream);
            var initializationThreadPool = new SimpleThreadDispatcher(Environment.ProcessorCount);


            var bodyBuilder = new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3());
            var constraintBuilder = new ContactManifoldConstraintBuilder();
            //Subtest<RegularGridWithKinematicBaseBuilder, ContactManifoldConstraintBuilder, ContactManifold4Constraint>(bodyBuilder, constraintBuilder, 32, 32, 32, 8, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, ContactManifoldConstraintBuilder, ContactManifold4Constraint>(bodyBuilder, constraintBuilder, 26, 26, 26, 12, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, ContactManifoldConstraintBuilder, ContactManifold4Constraint>(bodyBuilder, constraintBuilder, 20, 20, 20, 20, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, ContactManifoldConstraintBuilder, ContactManifold4Constraint>(bodyBuilder, constraintBuilder, 16, 16, 16, 30, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, ContactManifoldConstraintBuilder, ContactManifold4Constraint>(bodyBuilder, constraintBuilder, 13, 13, 13, 45, initializationThreadPool, writer);
            Subtest<RegularGridWithKinematicBaseBuilder, ContactManifoldConstraintBuilder, Contact4>(bodyBuilder, constraintBuilder, 10, 10, 10, 700, initializationThreadPool, writer);

            //var bodyBuilder = new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3());
            //var constraintBuilder = new BallSocketConstraintBuilder();
            //Subtest<RegularGridWithKinematicBaseBuilder, BallSocketConstraintBuilder, BallSocket>(bodyBuilder, constraintBuilder, 32, 32, 32, 8, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, BallSocketConstraintBuilder, BallSocket>(bodyBuilder, constraintBuilder, 26, 26, 26, 12, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, BallSocketConstraintBuilder, BallSocket>(bodyBuilder, constraintBuilder, 20, 20, 20, 20, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, BallSocketConstraintBuilder, BallSocket>(bodyBuilder, constraintBuilder, 16, 16, 16, 30, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, BallSocketConstraintBuilder, BallSocket>(bodyBuilder, constraintBuilder, 13, 13, 13, 45, initializationThreadPool, writer);
            //Subtest<RegularGridWithKinematicBaseBuilder, BallSocketConstraintBuilder, BallSocket>(bodyBuilder, constraintBuilder, 10, 10, 10, 70, initializationThreadPool, writer);
            initializationThreadPool.Dispose();
            writer.Flush();
            var path = "log.txt";
            using (var stream = File.OpenWrite(path))
            {
                Console.WriteLine($"Writing results to path: {Path.GetFullPath(path)}");
                var bytes = memoryStream.ToArray();
                stream.Write(bytes, 0, bytes.Length);
            }
            Console.ReadKey();
        }
    }
}
