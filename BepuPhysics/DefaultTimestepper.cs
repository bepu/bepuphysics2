using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using BepuUtilities;

namespace BepuPhysics
{
    /// <summary>
    /// Updates the simulation in the order of: sleeper -> predict body bounding boxes -> collision detection -> substepping solve -> data structure optimization.
    /// Each substep of the solve simulates and integrates a sub-timestep of length dt/substepCount.
    /// </summary>
    public class DefaultTimestepper : ITimestepper
    {
        /// <summary>
        /// Fires after the sleeper completes and before bodies are integrated.
        /// </summary>
        public event TimestepperStageHandler Slept;
        /// <summary>
        /// Fires after bodies have their bounding boxes updated for the frame's predicted motion and before collision detection.
        /// </summary>
        public event TimestepperStageHandler BeforeCollisionDetection;
        /// <summary>
        /// Fires after all collisions have been identified, but before the substep loop begins.
        /// </summary>
        public event TimestepperStageHandler CollisionsDetected;
        /// <summary>
        /// Fires after the solver executes and before the final integration step.
        /// </summary>
        public event TimestepperStageHandler ConstraintsSolved;

        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            simulation.Sleep(threadDispatcher);
            Slept?.Invoke(dt, threadDispatcher);

            simulation.PredictBoundingBoxes(dt, threadDispatcher);
            BeforeCollisionDetection?.Invoke(dt, threadDispatcher);

            simulation.CollisionDetection(dt, threadDispatcher);
            CollisionsDetected?.Invoke(dt, threadDispatcher);

            simulation.Solve(dt, threadDispatcher);
            ConstraintsSolved?.Invoke(dt, threadDispatcher);

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}
