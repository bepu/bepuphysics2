using BepuPhysics;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class TestHelpers
    {
        /// <summary>
        /// Gets a value roughly representing the amount of energy in the simulation. This is occasionally handy for debug purposes.
        /// </summary>
        public static float GetBodyEnergyHeuristic(Bodies bodies)
        {
            float accumulated = 0;
            for (int index = 0; index < bodies.ActiveSet.Count; ++index)
            {
                accumulated += Vector3.Dot(bodies.ActiveSet.Velocities[index].Linear, bodies.ActiveSet.Velocities[index].Linear);
                accumulated += Vector3.Dot(bodies.ActiveSet.Velocities[index].Angular, bodies.ActiveSet.Velocities[index].Angular);
            }
            return accumulated;
        }
    }
}
