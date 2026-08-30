using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints
{
    public struct SpringSettingsWide
    {
        //Be careful when fiddling with the memory layout. It's aligned with execution order.
        public Vector<float> AngularFrequency;
        public Vector<float> TwiceDampingRatio;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in SpringSettings source, ref SpringSettingsWide target)
        {
            GatherScatter.GetFirst(ref target.AngularFrequency) = source.AngularFrequency;
            GatherScatter.GetFirst(ref target.TwiceDampingRatio) = source.TwiceDampingRatio;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in SpringSettingsWide source, out SpringSettings target)
        {
            target.AngularFrequency = source.AngularFrequency[0];
            target.TwiceDampingRatio = source.TwiceDampingRatio[0];
        }

        /// <summary>
        /// Computes springiness values for a set of constraints.
        /// </summary>
        /// <param name="settings">Spring settings associated with the constraints.</param>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="positionErrorToVelocity">The multiplier applied to error to get bias velocity.</param>
        /// <param name="effectiveMassCFMScale">Scaling factor to apply to the effective mass to get the softened effective mass.</param>
        /// <param name="softnessImpulseScale">Scaling factor to apply to the accumulated impulse during the solve to soften the target velocity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeSpringiness(in SpringSettingsWide settings, float dt,
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
            var angularFrequencyDt = settings.AngularFrequency * new Vector<float>(dt);
            positionErrorToVelocity = settings.AngularFrequency / (angularFrequencyDt + settings.TwiceDampingRatio);
            var extra = Vector<float>.One / (angularFrequencyDt * (angularFrequencyDt + settings.TwiceDampingRatio));
            effectiveMassCFMScale = Vector<float>.One / (Vector<float>.One + extra);
            softnessImpulseScale = extra * effectiveMassCFMScale;
        }
    }

    public struct SpringSettings
    {
        /// <summary>
        /// Target number of undamped oscillations per unit of time, scaled by 2 * PI.
        /// </summary>
        public float AngularFrequency;
        /// <summary>
        /// Twice the ratio of the spring's actual damping to its critical damping.
        /// </summary>
        public float TwiceDampingRatio;

        /// <summary>
        /// Gets or sets the target number of undamped oscillations per unit of time.
        /// </summary>
        public float Frequency { get { return AngularFrequency / MathHelper.TwoPi; } set { AngularFrequency = value * MathHelper.TwoPi; } }

        /// <summary>
        /// Gets or sets the ratio of the spring's actual damping to its critical damping. 0 is undamped, 1 is critically damped, and higher values are overdamped.
        /// </summary>
        public float DampingRatio { get { return TwiceDampingRatio / 2f; } set { TwiceDampingRatio = value * 2; } }
        
        /// <summary>
        /// Checks if a spring settings instance contains valid values.
        /// </summary>
        /// <param name="springSettings">Settings to check.</param>
        /// <returns>True if the spring settings are valid, false otherwise.</returns>
        public static bool Validate(in SpringSettings springSettings)
        {
            return ConstraintChecker.IsPositiveNumber(springSettings.AngularFrequency) && ConstraintChecker.IsNonnegativeNumber(springSettings.TwiceDampingRatio);
        }

        /// <summary>
        /// Constructs a new spring settings instance.
        /// </summary>
        /// <param name="frequency">Target number of undamped oscillations per unit of time.</param>
        /// <param name="dampingRatio">Ratio of the spring's actual damping to its critical damping. 0 is undamped, 1 is critically damped, and higher values are overdamped.</param>
        public SpringSettings(float frequency, float dampingRatio)
        {
            AngularFrequency = frequency * MathHelper.TwoPi;
            TwiceDampingRatio = dampingRatio * 2;
            Debug.Assert(Validate(this), "Spring settings must have positive frequency and nonnegative damping ratio.");
        }

       
    }
}
