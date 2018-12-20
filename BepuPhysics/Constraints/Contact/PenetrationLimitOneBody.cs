using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Constraints.Contact
{
    public struct PenetrationLimitOneBodyProjection
    {
        //Note that these are just the raw jacobians, no precomputation with the JT*EffectiveMass.
        public Vector3Wide AngularA;
        public Vector<float> EffectiveMass;
        public Vector<float> BiasVelocity;
    }

    public static class PenetrationLimitOneBody
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(in BodyInertias inertiaA,
            in Vector3Wide contactOffsetA, in Vector3Wide normal, in Vector<float> depth,
            in Vector<float> positionErrorToVelocity, in Vector<float> effectiveMassCFMScale, in Vector<float> maximumRecoveryVelocity,
            float inverseDt, out PenetrationLimitOneBodyProjection projection)
        {
            //See PenetrationLimit.cs for a derivation.
            Vector3Wide.CrossWithoutOverlap(contactOffsetA, normal, out projection.AngularA);

            //effective mass
            Symmetric3x3Wide.VectorSandwich(projection.AngularA, inertiaA.InverseInertiaTensor, out var angularA0);

            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.EffectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + angularA0);

            //If depth is negative, the bias velocity will permit motion up until the depth hits zero. This works because positionErrorToVelocity * dt will always be <=1.
            projection.BiasVelocity = Vector.Min(
                depth * new Vector<float>(inverseDt),
                Vector.Min(depth * positionErrorToVelocity, maximumRecoveryVelocity));
        }


        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in PenetrationLimitOneBodyProjection projection, in BodyInertias inertiaA, in Vector3Wide normal,
            in Vector<float> correctiveImpulse, ref BodyVelocities wsvA)
        {
            var linearVelocityChangeA = correctiveImpulse * inertiaA.InverseMass;
            Vector3Wide.Scale(normal, linearVelocityChangeA, out var correctiveVelocityALinearVelocity);
            Vector3Wide.Scale(projection.AngularA, correctiveImpulse, out var correctiveAngularImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(correctiveAngularImpulseA, inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);

            Vector3Wide.Add(wsvA.Linear, correctiveVelocityALinearVelocity, out wsvA.Linear);
            Vector3Wide.Add(wsvA.Angular, correctiveVelocityAAngularVelocity, out wsvA.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            in PenetrationLimitOneBodyProjection projection, in BodyInertias inertiaA, in Vector3Wide normal,
            in Vector<float> accumulatedImpulse, ref BodyVelocities wsvA)
        {
            ApplyImpulse(projection, inertiaA, normal, accumulatedImpulse, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(in BodyVelocities wsvA,
            in PenetrationLimitOneBodyProjection projection,
            in Vector3Wide normal, in Vector<float> softnessImpulseScale,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            //Note that we do NOT use pretransformed jacobians here; the linear jacobian sharing (normal) meant that we had the effective mass anyway.
            Vector3Wide.Dot(wsvA.Linear, normal, out var csvaLinear);
            Vector3Wide.Dot(wsvA.Angular, projection.AngularA, out var csvaAngular);
            //Compute negated version to avoid the need for an explicit negate.
            var negatedCSI = accumulatedImpulse * softnessImpulseScale + (csvaLinear + csvaAngular - projection.BiasVelocity) * projection.EffectiveMass;

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse - negatedCSI);

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(in PenetrationLimitOneBodyProjection projection, in BodyInertias inertiaA, in Vector3Wide normal,
            in Vector<float> softnessImpulseScale, ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA)
        {
            ComputeCorrectiveImpulse(wsvA, projection, normal, softnessImpulseScale, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(projection, inertiaA, normal, correctiveCSI, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdatePenetrationDepth(in Vector<float> dt, in Vector3Wide contactOffset, in Vector3Wide normal, in BodyVelocities velocity, ref Vector<float> penetrationDepth)
        {
            //The normal is calibrated to point from B to A. Any movement of A along N results in a decrease in depth. Any movement of B along N results in an increase in depth. 
            //But one body constraints have no B.
            //estimatedPenetrationDepthChange = dot(normal, velocityDtA.Linear + velocityDtA.Angular x contactOffsetA)
            Vector3Wide.CrossWithoutOverlap(velocity.Angular, contactOffset, out var wxr);
            Vector3Wide.Add(wxr, velocity.Linear, out var contactVelocity);
            Vector3Wide.Dot(normal, contactVelocity, out var estimatedDepthChange);
            penetrationDepth -= estimatedDepthChange * dt;
        }

    }
}
