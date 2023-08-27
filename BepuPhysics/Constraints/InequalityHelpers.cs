using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints
{
    public static class InequalityHelpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeBiasVelocity(Vector<float> error, in Vector<float> positionErrorToVelocity, float inverseDt, out Vector<float> biasVelocity)
        {
            biasVelocity = Vector.Min(error * inverseDt, error * positionErrorToVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClampPositive(ref Vector<float> accumulatedImpulse, ref Vector<float> impulse)
        {
            var previous = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse + impulse);
            impulse = accumulatedImpulse - previous;
        }
    }
}
