using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BepuPhysics
{
    public struct SpringSettingsWide
    {
        //Be careful when fiddling with the memory layout. It's aligned with execution order.
        public Vector<float> NaturalFrequency;
        public Vector<float> DampingRatio;
    }

    public static class Springiness
    {
        /// <summary>
        /// Computes springiness values for a set of constraints.
        /// </summary>
        /// <param name="settings">Spring settings associated with the constraints.</param>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="positionErrorToVelocity">The multiplier applied to error to get bias velocity.</param>
        /// <param name="effectiveMassCFMScale">Scaling factor to apply to the effective mass to get the softened effective mass.</param>
        /// <param name="softnessImpulseScale">Scaling factor to apply to the accumulated impulse during the solve to soften the target velocity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeSpringiness(ref SpringSettingsWide settings, float dt,
            out Vector<float> positionErrorToVelocity, out Vector<float> effectiveMassCFMScale, out Vector<float> softnessImpulseScale)
        {
            //For more information behind these values, check the Inequality1DOF constraint comments.
            //softenedEffectiveMass = effectiveMass * (1 + (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1)^-1

            //CFM/dt * softenedEffectiveMass:
            //(naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1 * (1 + (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1)^-1

            //ERP = (naturalFrequency * dt) * (naturalFrequency * dt + 2 * dampingRatio)^-1
            //"ERP" is the error reduction per frame. Note that it can never exceed 1 given physically valid input.
            //Since it is a *per frame* term, note that the position error is additionally scaled by inverseDt to get the target velocity
            //needed to accomplish the desired error reduction in one frame.
            var frequencyDt = settings.NaturalFrequency * new Vector<float>(dt);
            var twiceDampingRatio = settings.DampingRatio * new Vector<float>(2); //Could precompute.
            positionErrorToVelocity = settings.NaturalFrequency / (frequencyDt + twiceDampingRatio);
            var extra = Vector<float>.One / (frequencyDt * (frequencyDt + twiceDampingRatio));
            effectiveMassCFMScale = Vector<float>.One / (Vector<float>.One + extra);
            softnessImpulseScale = extra * effectiveMassCFMScale;
        }
    }
}
