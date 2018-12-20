using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using BepuUtilities;

namespace BepuPhysics
{
    /// <summary>
    /// Updates the simulation in the order of: sleeper -> body bounding boxes -> collision detection -> LOOP { contact data update (if on iteration > 0) -> integrate body position and velocity -> solver } -> data structure optimization.
    /// Each inner loop execution simulates a sub-timestep of length dt/substepCount.
    /// Useful for simulations with difficult to solve constraint systems that need shorter timestep durations but which don't require high frequency collision detection.
    /// </summary>
    public class PositionFirstSubsteppingTimestepper : ITimestepper
    {
        /// <summary>
        /// Gets or sets the number of substeps to execute during each timestep.
        /// </summary>
        public int SubstepCount { get; set; }
        
        public PositionFirstSubsteppingTimestepper(int substepCount)
        {
            SubstepCount = substepCount;
        }

        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            simulation.Sleep(threadDispatcher);

            simulation.PredictBoundingBoxes(dt, threadDispatcher);

            simulation.CollisionDetection(dt, threadDispatcher);

            Debug.Assert(SubstepCount >= 0, "Substep count should be positive.");
            var substepDt = dt / SubstepCount;

            for (int i = 0; i < SubstepCount; ++i)
            {
                if (i > 0)
                {
                    //This takes the place of collision detection for the substeps. It uses the current velocity to update pentration depths.
                    //It's definitely an approximation, but it's important for avoiding some obviously weird behavior.
                    //Note that we do not run this on the first iteration- the actual collision detection above takes care of it.
                    simulation.IncrementallyUpdateContactConstraints(substepDt, threadDispatcher);
                }
                simulation.IntegrateBodies(substepDt, threadDispatcher);
                simulation.Solve(substepDt, threadDispatcher);

            }

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}
