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
        public event Action Slept;
        /// <summary>
        /// Fires after bodies have had their velocities and bounding boxes updated, but before collision detection begins.
        /// </summary>
        public event Action VelocitiesAndBoundsUpdated;
        /// <summary>
        /// Fires after all collisions have been identified, but before constraints are solved.
        /// </summary>
        public event Action CollisionsDetected;
        /// <summary>
        /// Fires after the solver executes and before body poses are integrated.
        /// </summary>
        public event Action ConstraintsSolved;
        /// <summary>
        /// Fires after bodies have their poses integrated and before data structures are incrementally optimized.
        /// </summary>
        public event Action PosesIntegrated;

        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            simulation.Sleep(threadDispatcher);
            Slept?.Invoke();

            simulation.IntegrateVelocitiesBoundsAndInertias(dt, threadDispatcher);
            VelocitiesAndBoundsUpdated?.Invoke();

            simulation.CollisionDetection(dt, threadDispatcher);
            CollisionsDetected?.Invoke();

            simulation.Solve(dt, threadDispatcher);
            ConstraintsSolved?.Invoke();

            simulation.IntegratePoses(dt, threadDispatcher);
            PosesIntegrated?.Invoke();

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}
