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
        public event Action Slept;
        /// <summary>
        /// Fires after bodies have their bounding boxes updated for the frame's predicted motion.
        /// </summary>
        public event Action BoundsPredicted;
        /// <summary>
        /// Fires after all collisions have been identified, but before the substep loop begins.
        /// </summary>
        public event Action CollisionsDetected;
        /// <summary>
        /// Fires at the beginning of a substep.
        /// </summary>
        public event Action<int> SubstepStarted;
        /// <summary>
        /// Fires after contact constraints are incrementally updated at the beginning of substeps after the first and before velocities are integrated.
        /// </summary>
        public event Action<int> ContactConstraintsUpdatedForSubstep;
        /// <summary>
        /// Fires after bodies have their velocities integrated and before the solver executes.
        /// </summary>
        public event Action<int> VelocitiesIntegrated;
        /// <summary>
        /// Fires after the solver executes and before body poses are integrated.
        /// </summary>
        public event Action<int> ConstraintsSolved;
        /// <summary>
        /// Fires after bodies have their poses integrated and before the substep ends.
        /// </summary>
        public event Action<int> PosesIntegrated;
        /// <summary>
        /// Fires at the end of a substep.
        /// </summary>
        public event Action<int> SubstepEnded;
        /// <summary>
        /// Fires after all substeps are finished executing and before data structures are incrementally optimized.
        /// </summary>
        public event Action SubstepsComplete;

        public SubsteppingTimestepper(int substepCount)
        {
            SubstepCount = substepCount;
        }

        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            simulation.Sleep(threadDispatcher);
            Slept?.Invoke();

            simulation.PredictBoundingBoxes(dt, threadDispatcher);
            BoundsPredicted?.Invoke();

            simulation.CollisionDetection(dt, threadDispatcher);
            CollisionsDetected?.Invoke();

            Debug.Assert(SubstepCount >= 0, "Substep count should be positive.");
            var substepDt = dt / SubstepCount;

            for (int substepIndex = 0; substepIndex < SubstepCount; ++substepIndex)
            {
                SubstepStarted?.Invoke(substepIndex);
                if (substepIndex > 0)
                {
                    //This takes the place of collision detection for the substeps. It uses the current velocity to update penetration depths.
                    //It's definitely an approximation, but it's important for avoiding some obviously weird behavior.
                    //Note that we do not run this on the first iteration- the actual collision detection above takes care of it.
                    simulation.IncrementallyUpdateContactConstraints(substepDt, threadDispatcher);
                    ContactConstraintsUpdatedForSubstep?.Invoke(substepIndex);
                }
                simulation.IntegrateVelocitiesAndUpdateInertias(substepDt, threadDispatcher);
                VelocitiesIntegrated?.Invoke(substepIndex);

                simulation.Solve(substepDt, threadDispatcher);
                ConstraintsSolved?.Invoke(substepIndex);

                simulation.IntegratePoses(substepDt, threadDispatcher);
                PosesIntegrated?.Invoke(substepIndex);
                SubstepEnded?.Invoke(substepIndex);
            }
            SubstepsComplete?.Invoke();

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}
