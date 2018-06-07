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
        public float MaximumSpeed;
        public float BaseSpeed;
        public float MaximumForce;
    }
    public struct MotorSettingsWide
    {
        public Vector<float> MaximumSpeed;
        public Vector<float> BaseSpeed;
        public Vector<float> MaximumForce;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in MotorSettings source, ref MotorSettingsWide target)
        {
            GatherScatter.GetFirst(ref target.MaximumSpeed) = source.MaximumSpeed;
            GatherScatter.GetFirst(ref target.BaseSpeed) = source.BaseSpeed;
            GatherScatter.GetFirst(ref target.MaximumForce) = source.MaximumForce;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in MotorSettingsWide source, out MotorSettings target)
        {
            target.MaximumSpeed = source.MaximumSpeed[0];
            target.BaseSpeed = source.BaseSpeed[0];
            target.MaximumForce = source.MaximumForce[0];
        }
    }
}
