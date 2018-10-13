using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Constraints
{
    public struct ServoSettings
    {
        public float MaximumSpeed;
        public float BaseSpeed;
        public float MaximumForce;

        /// <summary>
        /// Gets settings representing a servo with unlimited force, speed, and no base speed.
        /// </summary>
        public static ServoSettings Default { get { return new ServoSettings(float.MaxValue, 0, float.MaxValue); } }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ServoSettings(float maximumSpeed, float baseSpeed, float maximumForce)
        {
            MaximumSpeed = maximumSpeed;
            BaseSpeed = baseSpeed;
            MaximumForce = maximumForce;
        }
    }
    public struct ServoSettingsWide
    {
        public Vector<float> MaximumSpeed;
        public Vector<float> BaseSpeed;
        public Vector<float> MaximumForce;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClampBiasVelocity(in Vector<float> biasVelocity, in Vector<float> error, in ServoSettingsWide servoSettings, float inverseDt, out Vector<float> clampedBiasVelocity)
        {
            //Can't request speed that would cause an overshoot.
            var baseSpeed = Vector.Min(servoSettings.BaseSpeed, Vector.Abs(error) * inverseDt);
            clampedBiasVelocity = Vector.ConditionalSelect(Vector.LessThan(biasVelocity, Vector<float>.Zero),
                Vector.Max(-servoSettings.MaximumSpeed, Vector.Min(-baseSpeed, biasVelocity)),
                Vector.Min(servoSettings.MaximumSpeed, Vector.Max(baseSpeed, biasVelocity)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeClampedBiasVelocity(in Vector3Wide errorAxis, in Vector<float> errorLength, in Vector<float> positionErrorToBiasVelocity, in ServoSettingsWide servoSettings, 
            float inverseDt, out Vector3Wide clampedBiasVelocity)
        {
            //Can't request speed that would cause an overshoot.
            var baseSpeed = Vector.Min(servoSettings.BaseSpeed, errorLength * inverseDt);
            var unclampedBiasSpeed = errorLength * positionErrorToBiasVelocity;
            var scale = Vector.Min(Vector<float>.One, servoSettings.MaximumSpeed / Vector.Max(baseSpeed, unclampedBiasSpeed));
            Vector3Wide.Scale(errorAxis, scale * unclampedBiasSpeed, out clampedBiasVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClampImpulse(in Vector<float> maximumImpulse, ref Vector<float> accumulatedImpulse, ref Vector<float> csi)
        {
            var previousImpulse = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(-maximumImpulse, Vector.Min(maximumImpulse, accumulatedImpulse + csi));
            csi = accumulatedImpulse - previousImpulse;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in ServoSettings source, ref ServoSettingsWide target)
        {
            GatherScatter.GetFirst(ref target.MaximumSpeed) = source.MaximumSpeed;
            GatherScatter.GetFirst(ref target.BaseSpeed) = source.BaseSpeed;
            GatherScatter.GetFirst(ref target.MaximumForce) = source.MaximumForce;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in ServoSettingsWide source, out ServoSettings target)
        {
            target.MaximumSpeed = source.MaximumSpeed[0];
            target.BaseSpeed = source.BaseSpeed[0];
            target.MaximumForce = source.MaximumForce[0];
        }

    }
}
