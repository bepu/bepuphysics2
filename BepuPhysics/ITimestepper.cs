using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Defines a type capable of updating the simulation state for a given elapsed time.
    /// </summary>
    public interface ITimestepper
    {
        /// <summary>
        /// Performs one timestep of the given length.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        void Timestep(Simulation simulation, float dt, IThreadDispatcher threadDispatcher = null);
    }
}
