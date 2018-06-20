using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{
    /// <summary>
    /// Four convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public static class PenetrationLimit3OneBody
    {
        /// <summary>
        /// Data required to project world space velocities into a constraint impulse.
        /// </summary>
        public struct Projection
        {
            //Note that the data is interleaved to match the access order. We solve each constraint one at a time internally.
            //Also, the normal and inertias are shared across all constraints.
            public PenetrationLimitOneBodyProjection Penetration0;
            public Vector<float> SoftnessImpulseScale;
            public PenetrationLimitOneBodyProjection Penetration1;
            public PenetrationLimitOneBodyProjection Penetration2;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref Vector3Wide normal, ref Contact3OneBodyPrestepData prestep, float dt, float inverseDt,
            out Projection projection)
        {
            Vector3Wide.CrossWithoutOverlap(prestep.OffsetA0, normal, out projection.Penetration0.AngularA);
            Vector3Wide.CrossWithoutOverlap(prestep.OffsetA1, normal, out projection.Penetration1.AngularA);
            Vector3Wide.CrossWithoutOverlap(prestep.OffsetA2, normal, out projection.Penetration2.AngularA);

            //effective mass
            Symmetric3x3Wide.VectorSandwich(projection.Penetration0.AngularA, inertiaA.InverseInertiaTensor, out var angularA0);
            Symmetric3x3Wide.VectorSandwich(projection.Penetration1.AngularA, inertiaA.InverseInertiaTensor, out var angularA1);
            Symmetric3x3Wide.VectorSandwich(projection.Penetration2.AngularA, inertiaA.InverseInertiaTensor, out var angularA2);

            //Linear effective mass contribution notes:
            //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
            //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
            SpringSettingsWide.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.Penetration0.EffectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + angularA0);
            projection.Penetration1.EffectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + angularA1);
            projection.Penetration2.EffectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + angularA2);

            //If depth is negative, the bias velocity will permit motion up until the depth hits zero. This works because positionErrorToVelocity * dt will always be <=1.
            var inverseDtVector = new Vector<float>(inverseDt);
            projection.Penetration0.BiasVelocity = Vector.Min(prestep.PenetrationDepth0 * inverseDtVector, Vector.Min(prestep.PenetrationDepth0 * positionErrorToVelocity, prestep.MaximumRecoveryVelocity));
            projection.Penetration1.BiasVelocity = Vector.Min(prestep.PenetrationDepth1 * inverseDtVector, Vector.Min(prestep.PenetrationDepth1 * positionErrorToVelocity, prestep.MaximumRecoveryVelocity));
            projection.Penetration2.BiasVelocity = Vector.Min(prestep.PenetrationDepth2 * inverseDtVector, Vector.Min(prestep.PenetrationDepth2 * positionErrorToVelocity, prestep.MaximumRecoveryVelocity));
        }


        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref PenetrationLimitOneBodyProjection projection, ref BodyInertias inertiaA, ref Vector3Wide normal,
            ref Vector<float> correctiveImpulse,
            ref BodyVelocities wsvA)
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
            ref Projection projection, ref BodyInertias inertiaA, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2, ref BodyVelocities wsvA)
        {
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref normal, ref accumulatedImpulse0, ref wsvA);
            ApplyImpulse(ref projection.Penetration1, ref inertiaA, ref normal, ref accumulatedImpulse1, ref wsvA);
            ApplyImpulse(ref projection.Penetration2, ref inertiaA, ref normal, ref accumulatedImpulse2, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA,
            ref PenetrationLimitOneBodyProjection projection,
            ref Vector3Wide normal, ref Vector<float> softnessImpulseScale,
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
        public static void Solve(ref Projection projection, ref BodyInertias inertiaA, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2, ref BodyVelocities wsvA)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref projection.Penetration0, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse0, out var correctiveCSI0);
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref normal, ref correctiveCSI0, ref wsvA);
            ComputeCorrectiveImpulse(ref wsvA, ref projection.Penetration1, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse1, out var correctiveCSI1);
            ApplyImpulse(ref projection.Penetration1, ref inertiaA, ref normal, ref correctiveCSI1, ref wsvA);
            ComputeCorrectiveImpulse(ref wsvA, ref projection.Penetration2, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse2, out var correctiveCSI2);
            ApplyImpulse(ref projection.Penetration2, ref inertiaA, ref normal, ref correctiveCSI2, ref wsvA);
        }

    }
}
