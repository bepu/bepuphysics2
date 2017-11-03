using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using static BepuPhysics.Solver;
using Quaternion = BepuUtilities.Quaternion;
using BepuPhysics.Collidables;

namespace Demos.SpecializedTests
{
    static class MemoryResizeTests
    {
        static void Resize(Simulation simulation, Random random, int[] bodyHandles, int[] constraintHandles)
        {
            var scale = 4 * random.NextDouble();
            var sizes = new SimulationAllocationSizes
            {
                Bodies = (int)(bodyHandles.Length * scale),
                Statics = (int)(bodyHandles.Length * 4 * scale),
                ShapesPerType = (int)(16 * scale),
                Constraints = (int)(constraintHandles.Length * scale),
                ConstraintCountPerBodyEstimate = (int)(8 * scale),
                ConstraintsPerTypeBatch = (int)(128 * scale),
            };
            //None of these should ever shrink the size below the current sim size.
            if (random.NextDouble() < 0.5)
            {
                simulation.EnsureCapacity(sizes);
            }
            else
            {
                simulation.Resize(sizes);
            }

        }
        public static void Test()
        {
            var simulation = Simulation.Create(new BufferPool(), new TestCallbacks());
            var sphere = new Sphere(0.5f);
            var shapeIndex = simulation.Shapes.Add(ref sphere);

            var bodyBuilder = new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3(), 1, shapeIndex);
            var constraintBuilder = new BallSocketConstraintBuilder();
            const int width = 8;
            const int height = 8;
            const int length = 8;
            SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out var bodyHandles, out var constraintHandles);

            var random = new Random(5);
            for (int i = 0; i < 30; ++i)
            {
                var sample = random.NextDouble();
                if (sample < 0.1)
                {
                    //Clear and recreate.
                    simulation.Clear();
                    SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out bodyHandles, out constraintHandles);
                }
                else
                {
                    //Try to change size.
                    Resize(simulation, random, bodyHandles, constraintHandles);

                }
            }

            //SimulationScrambling.ScrambleBodies(simulation);
            //SimulationScrambling.ScrambleConstraints(simulation.Solver);
            //SimulationScrambling.ScrambleBodyConstraintLists(simulation);
            //SimulationScrambling.AddRemoveChurn<Contact4>(simulation, 1000, bodyHandles, constraintHandles);

            var threadDispatcher = new SimpleThreadDispatcher(8);

            const int iterations = 10;
            //const int internalCompressionIterations = 10;
            //for (int i = 0; i < iterations; ++i)
            //{
            //    SimulationScrambling.AddRemoveChurn<Contact4>(simulation, 10, bodyHandles, constraintHandles);
            //    GC.Collect(3, GCCollectionMode.Forced, true);
            //    var start = Stopwatch.GetTimestamp();
            //    for (int j = 0; j < internalCompressionIterations; ++j)
            //    {
            //        simulation.SolverBatchCompressor.Compress(simulation.BufferPool, threadDispatcher);
            //    }
            //}

            ////Attempt cache optimization.
            //int bodyOptimizationIterations = bodyHandles.Length * 1;
            //simulation.BodyLayoutOptimizer.OptimizationFraction = 0.004f;
            //for (int i = 0; i < bodyOptimizationIterations; ++i)
            //{
            //    simulation.BodyLayoutOptimizer.IncrementalOptimize(simulation.BufferPool, threadDispatcher);
            //}
            ////Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
            //simulation.ConstraintLayoutOptimizer.OptimizationFraction = 0.044f;
            //int constraintOptimizationIterations = 8192;
            //for (int i = 0; i < constraintOptimizationIterations; ++i)
            //{
            //    simulation.ConstraintLayoutOptimizer.Update(simulation.BufferPool, threadDispatcher);

            //}

            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            const int frameCount = 256;
            simulation.Solver.IterationCount = iterationCount;

            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = simulation.Bodies.GetBodyEnergyHeuristic();

                simulation.Timestep(dt);

                var energyAfter = simulation.Bodies.GetBodyEnergyHeuristic();
                //var velocityChange = solver.GetVelocityChangeHeuristic();
                //Console.WriteLine($"Constraint velocity change after frame {frameIndex}: {velocityChange}");
                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");

                for (int resizeIndex = 0; resizeIndex < 100; ++resizeIndex)
                {
                    Resize(simulation, random, bodyHandles, constraintHandles);
                }
            }



            threadDispatcher.Dispose();
            simulation.BufferPool.Clear();

        }



    }
}

