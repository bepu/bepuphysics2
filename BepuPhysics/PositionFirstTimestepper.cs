using System;
using System.Collections.Generic;
using System.Text;
using BepuUtilities;

namespace BepuPhysics
{
    /// <summary>
    /// Updates the simulation in the order of: sleeper -> integrate body poses, velocity and bounding boxes -> collision detection -> solver -> data structure optimization.
    /// </summary>
    public class PositionFirstTimestepper : ITimestepper
    {
        /// <summary>
        /// Fires after the sleeper completes and before bodies are integrated.
        /// </summary>
        public event TimestepperStageHandler Slept;
        /// <summary>
        /// Fires after bodies have had their position, velocity, and bounding boxes updated, but before collision detection begins.
        /// </summary>
        public event TimestepperStageHandler BeforeCollisionDetection;
        /// <summary>
        /// Fires after all collisions have been identified, but before constraints are solved.
        /// </summary>
        public event TimestepperStageHandler CollisionsDetected;
        /// <summary>
        /// Fires after the solver executes and before data structures are incrementally optimized.
        /// </summary>
        public event TimestepperStageHandler ConstraintsSolved;

        public void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null)
        {
            //Note that there is a reason to put the sleep *after* velocity integration. That sounds a little weird, but there's a good reason:
            //When the narrow phase activates a bunch of objects in a pile, their accumulated impulses will represent all forces acting on them at the time of sleep.
            //That includes gravity. If we sleep objects *before* gravity is applied in a given frame, then when those bodies are awakened, the accumulated impulses
            //will be less accurate because they assume that gravity has already been applied. This can cause a small bump.
            //So, velocity integration (and deactivation candidacy management) could come before sleep.

            //Sleep at the start, on the other hand, stops some forms of unintuitive behavior when using direct awakenings. Just a matter of preference.
            simulation.Sleep(threadDispatcher);
            Slept?.Invoke(dt, threadDispatcher);

            //Note that pose integrator comes before collision detection and solving. This is a shift from v1, where collision detection went first.
            //This is a tradeoff:
            //1) Any externally set velocities will be integrated without input from the solver. The v1-style external velocity control won't work as well-
            //the user would instead have to change velocities after the pose integrator runs. This isn't perfect either, since the pose integrator is also responsible
            //for updating the bounding boxes used for collision detection.
            //2) By bundling bounding box calculation with pose integration, you avoid redundant pose and velocity memory accesses.
            //3) Generated contact positions are in sync with the integrated poses. 
            //That's often helpful for gameplay purposes- you don't have to reinterpret contact data when creating graphical effects or positioning sound sources.
            
            //#1 is a difficult problem, though. There is no fully 'correct' place to change velocities. We might just have to bite the bullet and create a
            //inertia tensor/bounding box update separate from pose integration. If the cache gets evicted in between (virtually guaranteed unless no stages run),
            //this basically means an extra 100-200 microseconds per frame on a processor with ~20GBps bandwidth simulating 32768 bodies.

            //Note that the reason why the pose integrator comes first instead of, say, the solver, is that the solver relies on world space inertias calculated by the pose integration.
            //If the pose integrator doesn't run first, we either need 
            //1) complicated on demand updates of world inertia when objects are added or local inertias are changed or 
            //2) local->world inertia calculation before the solver.
            simulation.IntegrateBodiesAndUpdateBoundingBoxes(dt, threadDispatcher);
            BeforeCollisionDetection?.Invoke(dt, threadDispatcher);

            simulation.CollisionDetection(dt, threadDispatcher);
            CollisionsDetected?.Invoke(dt, threadDispatcher);

            simulation.Solve(dt, threadDispatcher);
            ConstraintsSolved?.Invoke(dt, threadDispatcher);

            simulation.IncrementallyOptimizeDataStructures(threadDispatcher);
        }
    }
}
