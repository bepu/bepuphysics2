using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Constraints
{
    public static class InequalityHelpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClampPositive(ref Vector<float> accumulatedImpulse, ref Vector<float> impulse)
        {
            var previous = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse + impulse);
            impulse = accumulatedImpulse - previous;
        }
    }
}
