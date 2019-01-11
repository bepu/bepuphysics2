using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using BepuUtilities;

namespace BepuPhysics
{
    /// <summary>
    /// Updates the simulation in the order of: sleeper -> integrate velocities and update body bounding boxes -> collision detection -> solver -> integrate body poses -> data structure optimization.
    /// </summary>
    public class PositionLastTimestepper : ITimestepper
    {
        /// <summary>
        /// Fires after the sleeper completes and before bodies are integrated.
        /// </summary>
        public event TimestepperStageHandler Slept;
        /// <summary>
        /// Fires after bodies have had their velocities and bounding boxes updated, but before collision detection begins.
        /// </summary>
        public event TimestepperStageHandler BeforeCollisionDetection;
        /// <summary>
        /// Fires after all collisions have been identified, but before constraints are solved.
        /// </summary>
        public event TimestepperStageHandler CollisionsDetected;
        /// <summary>
        /// Fires after the solver executes and before body poses are integrated.
        /// </summary>
        public event TimestepperStageHandler ConstraintsSolved;
        /// <summary>
        /// Fires after bodies have their poses integrated and before data structures are incrementally optimized.
        /// </summary>
        public event TimestepperStageHandler PosesIntegrated;

        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            simulation.Sleep(threadDispatcher);
            Slept?.Invoke(dt, threadDispatcher);

            simulation.IntegrateVelocitiesBoundsAndInertias(dt, threadDispatcher);
            BeforeCollisionDetection?.Invoke(dt, threadDispatcher);

            simulation.CollisionDetection(dt, threadDispatcher);
            CollisionsDetected?.Invoke(dt, threadDispatcher);

            simulation.Solve(dt, threadDispatcher);
            ConstraintsSolved?.Invoke(dt, threadDispatcher);

            simulation.IntegratePoses(dt, threadDispatcher);
            PosesIntegrated?.Invoke(dt, threadDispatcher);

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}
