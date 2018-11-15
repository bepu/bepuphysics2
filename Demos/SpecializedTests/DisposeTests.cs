using BepuUtilities.Memory;
using BepuPhysics;
using System;
using System.Numerics;
using BepuPhysics.Collidables;

namespace Demos.SpecializedTests
{
    static class DisposeTests
    {
        static void CreateAndRunSimulation(BufferPool bufferPool)
        {
            //TODO: As more features get added, you'll probably want to revisit this and lengthen the per-execution duration.
            var simulation = Simulation.Create(bufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            var sphere = new Sphere(0.5f);
            var shapeIndex = simulation.Shapes.Add(sphere);

            var bodyBuilder = new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3(), 1, shapeIndex);
            var constraintBuilder = new BallSocketConstraintBuilder();
            const int width = 3;
            const int height = 3;
            const int length = 3;
            SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out var bodyHandles, out var constraintHandles);
            
            simulation.Bodies.ActiveSet.Velocities[width].Linear = new Vector3(0.1f, 0, 0.1f);

            for (int i = 0; i < 16; ++i)
            {
                simulation.Timestep(1 / 60f);
            }

            simulation.Dispose();
        }
        public static void Test()
        {

            var pool = new BufferPool();
            for (int i = 0; i < 131072; ++i)
            {
                CreateAndRunSimulation(pool);
                GC.Collect(3, GCCollectionMode.Forced, true);
                pool.AssertEmpty();
                if (i % 32 == 0)
                    Console.WriteLine($"Completed execution {i}...");
            }
            pool.Clear();

        }



    }
}

