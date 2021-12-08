using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Delegate used by ITimesteppers for their stage callbacks.
    /// </summary>
    /// <param name="dt">Time step duration.</param>
    /// <param name="threadDispatcher">Thread dispatcher used for this timestep.</param>
    public delegate void TimestepperStageHandler(float dt, IThreadDispatcher threadDispatcher);

    /// <summary>
    /// Defines a type capable of updating the simulation state for a given elapsed time.
    /// </summary>
    public interface ITimestepper
    {
        /// <summary>
        /// Callbacks to execute immediately before collision detection executes.
        /// </summary>
        event TimestepperStageHandler BeforeCollisionDetection;
        
        /// <summary>
        /// Callbacks to execute after collision detection completes.
        /// </summary>
        event TimestepperStageHandler CollisionsDetected;

        /// <summary>
        /// Performs one timestep of the given length.
        /// </summary>
        /// <param name="simulation">Simulation to be stepped forward in time.</param>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null);
    }
}
