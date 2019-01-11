using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using BepuUtilities;

namespace BepuPhysics
{
    /// <summary>
    /// Updates the simulation in the order of: sleeper -> predict body bounding boxes -> collision detection -> LOOP { contact data update (if on iteration > 0) -> integrate body velocities -> solver -> integrate body poses } -> data structure optimization.
    /// Each inner loop execution simulates a sub-timestep of length dt/substepCount.
    /// Useful for simulations with difficult to solve constraint systems that need shorter timestep durations but which don't require high frequency collision detection.
    /// </summary>
    public class SubsteppingTimestepper : ITimestepper
    {
        /// <summary>
        /// Gets or sets the number of substeps to execute during each timestep.
        /// </summary>
        public int SubstepCount { get; set; }

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
        /// Fires at the beginning of a substep.
        /// </summary>
        public event TimestepperSubstepStageHandler SubstepStarted;
        /// <summary>
        /// Fires after contact constraints are incrementally updated at the beginning of substeps after the first and before velocities are integrated.
        /// </summary>
        public event TimestepperSubstepStageHandler ContactConstraintsUpdatedForSubstep;
        /// <summary>
        /// Fires after bodies have their velocities integrated and before the solver executes.
        /// </summary>
        public event TimestepperSubstepStageHandler VelocitiesIntegrated;
        /// <summary>
        /// Fires after the solver executes and before body poses are integrated.
        /// </summary>
        public event TimestepperSubstepStageHandler ConstraintsSolved;
        /// <summary>
        /// Fires after bodies have their poses integrated and before the substep ends.
        /// </summary>
        public event TimestepperSubstepStageHandler PosesIntegrated;
        /// <summary>
        /// Fires at the end of a substep.
        /// </summary>
        public event TimestepperSubstepStageHandler SubstepEnded;
        /// <summary>
        /// Fires after all substeps are finished executing and before data structures are incrementally optimized.
        /// </summary>
        public event TimestepperStageHandler SubstepsComplete;

        public SubsteppingTimestepper(int substepCount)
        {
            SubstepCount = substepCount;
        }

        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            simulation.Sleep(threadDispatcher);
            Slept?.Invoke(dt, threadDispatcher);

            simulation.PredictBoundingBoxes(dt, threadDispatcher);
            BeforeCollisionDetection?.Invoke(dt, threadDispatcher);

            simulation.CollisionDetection(dt, threadDispatcher);
            CollisionsDetected?.Invoke(dt, threadDispatcher);

            Debug.Assert(SubstepCount >= 0, "Substep count should be positive.");
            var substepDt = dt / SubstepCount;

            for (int substepIndex = 0; substepIndex < SubstepCount; ++substepIndex)
            {
                SubstepStarted?.Invoke(substepIndex, dt, threadDispatcher);
                if (substepIndex > 0)
                {
                    //This takes the place of collision detection for the substeps. It uses the current velocity to update penetration depths.
                    //It's definitely an approximation, but it's important for avoiding some obviously weird behavior.
                    //Note that we do not run this on the first iteration- the actual collision detection above takes care of it.
                    simulation.IncrementallyUpdateContactConstraints(substepDt, threadDispatcher);
                    ContactConstraintsUpdatedForSubstep?.Invoke(substepIndex, dt, threadDispatcher);
                }
                simulation.IntegrateVelocitiesAndUpdateInertias(substepDt, threadDispatcher);
                VelocitiesIntegrated?.Invoke(substepIndex, dt, threadDispatcher);

                simulation.Solve(substepDt, threadDispatcher);
                ConstraintsSolved?.Invoke(substepIndex, dt, threadDispatcher);

                simulation.IntegratePoses(substepDt, threadDispatcher);
                PosesIntegrated?.Invoke(substepIndex, dt, threadDispatcher);
                SubstepEnded?.Invoke(substepIndex, dt, threadDispatcher);
            }
            SubstepsComplete?.Invoke(dt, threadDispatcher);

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}
