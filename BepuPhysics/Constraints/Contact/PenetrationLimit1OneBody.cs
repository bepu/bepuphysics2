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


    /// <summary>
    /// Four convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public static class PenetrationLimit1OneBody
    {
        public struct Projection
        {
            //Note that the data is interleaved to match the access order. We solve each constraint one at a time internally.
            //Also, the normal and inertias are shared across all constraints.
            public PenetrationLimitOneBodyProjection Penetration0;
            public Vector<float> SoftnessImpulseScale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref Vector3Wide normal, ref Contact1OneBodyPrestepData prestep, float dt, float inverseDt,
            out Projection projection)
        {
            Prestep(ref inertiaA, ref normal, ref prestep, dt, inverseDt, out projection);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA,
            ref Vector3Wide contactOffsetA, ref Vector3Wide normal, ref Vector<float> depth, ref SpringSettingsWide springSettings, ref Vector<float> maximumRecoveryVelocity,
            float dt, float inverseDt, out Projection projection)
        {
            Vector3Wide.CrossWithoutOverlap(ref contactOffsetA, ref normal, out projection.Penetration0.AngularA);

            //effective mass
            Triangular3x3Wide.VectorSandwich(ref projection.Penetration0.AngularA, ref inertiaA.InverseInertiaTensor, out var angularA0);

            //Linear effective mass contribution notes:
            //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
            //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
            Springiness.ComputeSpringiness(ref springSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.Penetration0.EffectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + angularA0);

            //If depth is negative, the bias velocity will permit motion up until the depth hits zero. This works because positionErrorToVelocity * dt will always be <=1.
            projection.Penetration0.BiasVelocity = Vector.Min(
                depth * new Vector<float>(inverseDt),
                Vector.Min(depth * positionErrorToVelocity, maximumRecoveryVelocity));
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
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeA, out var correctiveVelocityALinearVelocity);
            Vector3Wide.Scale(ref projection.AngularA, ref correctiveImpulse, out var correctiveAngularImpulseA);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref correctiveAngularImpulseA, ref inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);

            Vector3Wide.Add(ref wsvA.Linear, ref correctiveVelocityALinearVelocity, out wsvA.Linear);
            Vector3Wide.Add(ref wsvA.Angular, ref correctiveVelocityAAngularVelocity, out wsvA.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            ref Projection projection, ref BodyInertias inertiaA, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0, ref BodyVelocities wsvA)
        {
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref normal, ref accumulatedImpulse0, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA,
            ref PenetrationLimitOneBodyProjection projection,
            ref Vector3Wide normal, ref Vector<float> softnessImpulseScale,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            //Note that we do NOT use pretransformed jacobians here; the linear jacobian sharing (normal) meant that we had the effective mass anyway.
            Vector3Wide.Dot(ref wsvA.Linear, ref normal, out var csvaLinear);
            Vector3Wide.Dot(ref wsvA.Angular, ref projection.AngularA, out var csvaAngular);
            //Compute negated version to avoid the need for an explicit negate.
            var negatedCSI = accumulatedImpulse * softnessImpulseScale + (csvaLinear + csvaAngular - projection.BiasVelocity) * projection.EffectiveMass;

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse - negatedCSI);

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref Projection projection, ref BodyInertias inertiaA, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0, ref BodyVelocities wsvA)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref projection.Penetration0, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse0, out var correctiveCSI0);
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref normal, ref correctiveCSI0, ref wsvA);
        }

    }
}
