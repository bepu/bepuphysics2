using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;

namespace BepuPhysics.Constraints
{
    public static class ConstraintChecker
    {
        [Conditional("DEBUG")]
        public static void AssertUnitLength(in Vector3 v, string typeName, string propertyName)
        {
            var lengthSquared = v.LengthSquared();
            if (lengthSquared > 1 + 1e-5f || lengthSquared < 1 - 1e-5f)
            {
                Debug.Fail($"{typeName}.{propertyName} must be unit length.");
            }
        }
        [Conditional("DEBUG")]
        public static void AssertUnitLength(in BepuUtilities.Quaternion q, string typeName, string propertyName)
        {
            var lengthSquared = q.LengthSquared();
            if (lengthSquared > 1 + 1e-5f || lengthSquared < 1 - 1e-5f)
            {
                Debug.Fail($"{typeName}.{propertyName} must be unit length.");
            }
        }

        [Conditional("DEBUG")]
        public static void AssertValid(in SpringSettings settings, string typeName)
        {
            if(!SpringSettings.Validate(settings))
            {
                Debug.Fail($"{typeName}.SpringSettings must have positive frequency and nonnegative damping ratio.");
            }
        }

        [Conditional("DEBUG")]
        public static void AssertValid(in MotorSettings settings, string typeName)
        {
            if (!MotorSettings.Validate(settings))
            {
                Debug.Fail($"{typeName}.MotorSettings must have nonnegative maximum force and damping.");
            }
        }

        [Conditional("DEBUG")]
        public static void AssertValid(in ServoSettings settings, string typeName)
        {
            if (!ServoSettings.Validate(settings))
            {
                Debug.Fail($"{typeName}.ServoSettings must have nonnegative maximum speed, base speed, and maximum force.");
            }
        }

        [Conditional("DEBUG")]
        public static void AssertValid(in ServoSettings servoSettings, in SpringSettings springSettings, string typeName)
        {
            AssertValid(servoSettings, typeName);
            AssertValid(springSettings, typeName);
        }
    }
}
