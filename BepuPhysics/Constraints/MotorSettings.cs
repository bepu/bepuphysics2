using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Constraints
{
    public struct MotorSettings
    {
        //Seems a little awkward to have the target speed hidden inside a MotorSettings property... also awkward to a shared field repeated over and over.
        public float TargetSpeed;
        public float MaximumForce;
        public float Softness;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public MotorSettings(float targetSpeed, float maximumForce, float softness)
        {
            TargetSpeed = targetSpeed;
            MaximumForce = maximumForce;
            Softness = softness;
        }
    }
    public struct MotorSettingsWide
    {
        public Vector<float> TargetSpeed;
        public Vector<float> MaximumForce;
        public Vector<float> Softness;
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in MotorSettings source, ref MotorSettingsWide target)
        {
            GatherScatter.GetFirst(ref target.TargetSpeed) = source.TargetSpeed;
            GatherScatter.GetFirst(ref target.MaximumForce) = source.MaximumForce;
            GatherScatter.GetFirst(ref target.Softness) = source.Softness;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in MotorSettingsWide source, out MotorSettings target)
        {
            target.TargetSpeed = source.TargetSpeed[0];
            target.MaximumForce = source.MaximumForce[0];
            target.Softness = source.Softness[0];
        }
    }
}
