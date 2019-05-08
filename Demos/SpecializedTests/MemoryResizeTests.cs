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
using BepuUtilities.Collections;

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

        static void FillTrashBuffers(Simulation simulation, Random random)
        {
            var pool = simulation.BufferPool;
            const int bufferCount = 50;
            var bufferList = new QuickList<Buffer<int>>(bufferCount, pool);
            for (int trashBufferIndex = 0; trashBufferIndex < bufferCount; ++trashBufferIndex)
            {
                //Pull a buffer from the pool, fill it with trash data, and return it. 
                ref var buffer = ref bufferList.AllocateUnsafely();
                pool.TakeAtLeast(1 << random.Next(18), out buffer);
                for (int k = 0; k < buffer.Length; ++k)
                    buffer[k] = random.Next(int.MinValue, int.MaxValue);
            }
            for (int i = 0; i < bufferCount; ++i)
            {
                pool.Return(ref bufferList[i]);
            }
            bufferList.Dispose(pool);
        }
        public static void Test()
        {
            var simulation = Simulation.Create(new BufferPool(), new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            var sphere = new Sphere(0.5f);
            var shapeIndex = simulation.Shapes.Add(sphere);

            var bodyBuilder = new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3(), 1, shapeIndex);
            var constraintBuilder = new BallSocketConstraintBuilder();
            const int width = 8;
            const int height = 8;
            const int length = 8;
            SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out var bodyHandles, out var constraintHandles);

            var random = new Random(5);
            for (int i = 0; i < 1000; ++i)
            {
                var sample = random.NextDouble();
                if (sample < 0.1)
                {
                    //Clear and recreate.
                    simulation.Clear();
                    shapeIndex = simulation.Shapes.Add(sphere);
                    SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out bodyHandles, out constraintHandles);
                }
                else
                {
                    //Try to change size.
                    Resize(simulation, random, bodyHandles, constraintHandles);
                }
                FillTrashBuffers(simulation, random);

                if (i % 100 == 0)
                    Console.WriteLine($"Iteration {i} completed...");
            }

            SimulationScrambling.ScrambleBodies(simulation);
            SimulationScrambling.ScrambleConstraints(simulation.Solver);
            SimulationScrambling.ScrambleBodyConstraintLists(simulation);
            SimulationScrambling.AddRemoveChurn<BallSocket>(simulation, 1000, bodyHandles, constraintHandles);

            
            ref var location = ref simulation.Bodies.HandleToLocation[bodyHandles[width]];
            Debug.Assert(location.SetIndex == 0, "Nothing above should result in inactive objects.");
            simulation.Bodies.ActiveSet.Velocities[location.Index].Linear = new Vector3(0.1f, 0, 0.1f);

            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            const int frameCount = 256;
            simulation.Solver.IterationCount = iterationCount;

            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = TestHelpers.GetBodyEnergyHeuristic(simulation.Bodies);

                simulation.Timestep(dt);

                var energyAfter = TestHelpers.GetBodyEnergyHeuristic(simulation.Bodies);
                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");

                for (int resizeIndex = 0; resizeIndex < 100; ++resizeIndex)
                {
                    Resize(simulation, random, bodyHandles, constraintHandles);
                }
                FillTrashBuffers(simulation, random);
            }


            
            simulation.BufferPool.Clear();

        }



    }
}

