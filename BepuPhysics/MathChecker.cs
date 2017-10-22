using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    //TODO: should probably bundle this into the bepuutilities version.
    public static class MathChecker
    {
        [Conditional("CHECKMATH")]
        public static void Validate(this Vector<float> f, int laneCount = -1)
        {
            if (laneCount < -1 || laneCount > Vector<float>.Count)
                throw new ArgumentException("Invalid lane count.");
            if (laneCount == -1)
                laneCount = Vector<float>.Count;
            ref var casted = ref Unsafe.As<Vector<float>, float>(ref f);
            for (int i = 0; i < laneCount; ++i)
            {
                var value = Unsafe.Add(ref casted, i);
                if (float.IsNaN(value) || float.IsInfinity(value))
                {
                    throw new InvalidOperationException($"Invalid floating point value: {value}.");
                }
            }

        }
        [Conditional("CHECKMATH")]
        public static void Validate(this Vector2Wide v, int laneCount = -1)
        {
            v.X.Validate(laneCount);
            v.Y.Validate(laneCount);
        }
        [Conditional("CHECKMATH")]
        public static void Validate(this Vector3Wide v, int laneCount = -1)
        {
            v.X.Validate(laneCount);
            v.Y.Validate(laneCount);
            v.Z.Validate(laneCount);
        }
        [Conditional("CHECKMATH")]
        public static void Validate(this Matrix2x2Wide m, int laneCount = -1)
        {
            m.X.Validate(laneCount);
            m.Y.Validate(laneCount);
        }
        [Conditional("CHECKMATH")]
        public static void Validate(this Matrix2x3Wide m, int laneCount = -1)
        {
            m.X.Validate(laneCount);
            m.Y.Validate(laneCount);
        }
        [Conditional("CHECKMATH")]
        public static void Validate(this Matrix3x3Wide m, int laneCount = -1)
        {
            m.X.Validate(laneCount);
            m.Y.Validate(laneCount);
            m.Z.Validate(laneCount);
        }
        [Conditional("CHECKMATH")]
        public static void Validate(this QuaternionWide m, int laneCount = -1)
        {
            m.X.Validate(laneCount);
            m.Y.Validate(laneCount);
            m.Z.Validate(laneCount);
            m.W.Validate(laneCount);
        }
    }
}
