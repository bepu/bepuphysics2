using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{
    //For in depth explanations of constraints, check the Inequality1DOF.cs implementation.
    //The details are omitted for brevity in other implementations.

    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TwistFrictionOneBody
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref Vector3Wide angularJacobianA,
            out TwistFrictionProjection projection)
        {
            //Compute effective mass matrix contributions. No linear contributions for the twist constraint.
            Symmetric3x3Wide.VectorSandwich(angularJacobianA, inertiaA.InverseInertiaTensor, out var inverseEffectiveMass);
    
            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            //Note that we have to guard against two bodies with infinite inertias. This is a valid state! 
            //(We do not have to do such guarding on constraints with linear jacobians; dynamic bodies cannot have zero *mass*.)
            //(Also note that there's no need for epsilons here... users shouldn't be setting their inertias to the absurd values it would take to cause a problem.
            //Invalid conditions can't arise dynamically.)
            var inverseIsZero = Vector.Equals(Vector<float>.Zero, inverseEffectiveMass);
            projection.EffectiveMass = Vector.ConditionalSelect(inverseIsZero, Vector<float>.Zero, Vector<float>.One / inverseEffectiveMass);

            //Note that friction constraints have no bias velocity. They target zero velocity.


        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularJacobianA, ref BodyInertias inertiaA,
            ref Vector<float> correctiveImpulse, ref BodyVelocities wsvA)
        {
            Vector3Wide.Scale(angularJacobianA, correctiveImpulse, out var worldCorrectiveImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(worldCorrectiveImpulseA, inertiaA.InverseInertiaTensor, out var worldCorrectiveVelocityA);
            Vector3Wide.Add(wsvA.Angular, worldCorrectiveVelocityA, out wsvA.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref Vector3Wide angularJacobianA, ref BodyInertias inertiaA,
            ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA)
        {
            ApplyImpulse(ref angularJacobianA, ref inertiaA, ref accumulatedImpulse, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref Vector3Wide angularJacobianA, ref TwistFrictionProjection projection, 
            ref BodyVelocities wsvA, ref Vector<float> maximumImpulse,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            Vector3Wide.Dot(wsvA.Angular, angularJacobianA, out var csvA);
            var negativeCSI = csvA * projection.EffectiveMass; //Since there is no bias or softness to give us the negative, we just do it when we apply to the accumulated impulse.

            var previousAccumulated = accumulatedImpulse;
            //The maximum force of friction depends upon the normal impulse.
            accumulatedImpulse = Vector.Min(maximumImpulse, Vector.Max(-maximumImpulse, accumulatedImpulse - negativeCSI));

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref Vector3Wide angularJacobianA, ref BodyInertias inertiaA, ref TwistFrictionProjection projection, 
            ref Vector<float> maximumImpulse, ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA)
        {
            ComputeCorrectiveImpulse(ref angularJacobianA, ref projection, ref wsvA, ref maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref angularJacobianA, ref inertiaA, ref correctiveCSI, ref wsvA);

        }

    }
}
