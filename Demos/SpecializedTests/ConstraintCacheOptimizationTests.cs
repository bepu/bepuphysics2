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

namespace Demos.SpecializedTests
{
    static class ConstraintCacheOptimizationTests
    {
        public static void Test()
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);

            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3()), 
                new ContactManifoldConstraintBuilder(),
                32, 32, 32, out var simulation, out var bodyHandles, out var constraintHandles);

            SimulationScrambling.ScrambleBodies(simulation);
            SimulationScrambling.ScrambleConstraints(simulation.Solver);
            SimulationScrambling.ScrambleBodyConstraintLists(simulation);
            SimulationScrambling.AddRemoveChurn<Contact4>(simulation, 100000, bodyHandles, constraintHandles);

            var threadDispatcher = new SimpleThreadDispatcher(8);
            //var threadDispatcher = new NotQuiteAThreadDispatcher(8);

            simulation.ConstraintLayoutOptimizer.OptimizationFraction = 0.01f;
            int constraintOptimizationIterations = 8192;

            simulation.ConstraintLayoutOptimizer.Update(simulation.BufferPool, threadDispatcher);//prejit
            var timer = Stopwatch.StartNew();
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                simulation.ConstraintLayoutOptimizer.Update(simulation.BufferPool, threadDispatcher);
            }
            timer.Stop();
            Console.WriteLine($"Finished constraint optimizations, time (ms): {timer.Elapsed.TotalMilliseconds}" +
                $", per iteration (us): {timer.Elapsed.TotalSeconds * 1e6 / constraintOptimizationIterations}");
        
            //threadDispatcher.Dispose();
            simulation.BufferPool.Clear();

        }



    }
}

