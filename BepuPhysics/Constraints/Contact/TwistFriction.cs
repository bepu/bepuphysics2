using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{
    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TwistFriction
    {
        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3Wide angularJacobianA, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector<float> correctiveImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Vector3Wide.Scale(angularJacobianA, correctiveImpulse, out var worldCorrectiveImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(worldCorrectiveImpulseA, inertiaA.InverseInertiaTensor, out var worldCorrectiveVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(worldCorrectiveImpulseA, inertiaB.InverseInertiaTensor, out var worldCorrectiveVelocityB);
            Vector3Wide.Add(wsvA.Angular, worldCorrectiveVelocityA, out wsvA.Angular);
            Vector3Wide.Subtract(wsvB.Angular, worldCorrectiveVelocityB, out wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(in Vector3Wide angularJacobianA, in Vector<float> effectiveMass,
            in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, in Vector<float> maximumImpulse,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            Vector3Wide.Dot(wsvA.Angular, angularJacobianA, out var csvA);
            Vector3Wide.Dot(wsvB.Angular, angularJacobianA, out var negatedCSVB);
            var negatedCSI = (csvA - negatedCSVB) * effectiveMass; //Since there is no bias or softness to give us the negative, we just do it when we apply to the accumulated impulse.

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Min(maximumImpulse, Vector.Max(-maximumImpulse, accumulatedImpulse - negatedCSI));

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(in Vector3Wide angularJacobianA, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ApplyImpulse(angularJacobianA, inertiaA, inertiaB, accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(in Vector3Wide angularJacobianA, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector<float> maximumImpulse, ref Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //Compute effective mass matrix contributions. No linear contributions for the twist constraint.
            //Note that we use the angularJacobianA (that is, the normal) for both, despite angularJacobianB = -angularJacobianA. That's fine- J * M * JT is going to be positive regardless.
            Symmetric3x3Wide.VectorSandwich(angularJacobianA, inertiaA.InverseInertiaTensor, out var angularA);
            Symmetric3x3Wide.VectorSandwich(angularJacobianA, inertiaB.InverseInertiaTensor, out var angularB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            //Note that we have to guard against two bodies with infinite inertias. This is a valid state! 
            //(We do not have to do such guarding on constraints with linear jacobians; dynamic bodies cannot have zero *mass*.)
            //(Also note that there's no need for epsilons here... users shouldn't be setting their inertias to the absurd values it would take to cause a problem.
            //Invalid conditions can't arise dynamically.)
            var inverseEffectiveMass = angularA + angularB;
            var inverseIsZero = Vector.Equals(Vector<float>.Zero, inverseEffectiveMass);
            var effectiveMass = Vector.ConditionalSelect(inverseIsZero, Vector<float>.Zero, Vector<float>.One / inverseEffectiveMass);

            //Note that friction constraints have no bias velocity. They target zero velocity.
            ComputeCorrectiveImpulse(angularJacobianA, effectiveMass, wsvA, wsvB, maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(angularJacobianA, inertiaA, inertiaB, correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
