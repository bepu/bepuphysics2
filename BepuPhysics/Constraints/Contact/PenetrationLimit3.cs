using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Constraints.Contact
{
    
    /// <summary>
    /// Three convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public static class PenetrationLimit3
    {
        /// <summary>
        /// Data required to project world space velocities into a constraint impulse.
        /// </summary>
        public struct Projection
        {
            //Note that the data is interleaved to match the access order. We solve each constraint one at a time internally.
            //Also, the normal and inertias are shared across all constraints.
            public PenetrationLimitProjection Penetration0;
            public Vector<float> SoftnessImpulseScale;
            public PenetrationLimitProjection Penetration1;
            public PenetrationLimitProjection Penetration2;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal, ref Contact3PrestepData prestep, float dt, float inverseDt,
            out Projection projection)
        {
            Vector3Wide.CrossWithoutOverlap(prestep.OffsetA0, normal, out projection.Penetration0.AngularA);
            Vector3Wide.Subtract(prestep.OffsetA0, prestep.OffsetB, out var offsetB0);
            Vector3Wide.CrossWithoutOverlap(normal, offsetB0, out projection.Penetration0.AngularB);
            Vector3Wide.CrossWithoutOverlap(prestep.OffsetA1, normal, out projection.Penetration1.AngularA);
            Vector3Wide.Subtract(prestep.OffsetA1, prestep.OffsetB, out var offsetB1);
            Vector3Wide.CrossWithoutOverlap(normal, offsetB1, out projection.Penetration1.AngularB);
            Vector3Wide.CrossWithoutOverlap(prestep.OffsetA2, normal, out projection.Penetration2.AngularA);
            Vector3Wide.Subtract(prestep.OffsetA2, prestep.OffsetB, out var offsetB2);
            Vector3Wide.CrossWithoutOverlap(normal, offsetB2, out projection.Penetration2.AngularB);

            //effective mass
            Triangular3x3Wide.VectorSandwich(projection.Penetration0.AngularA, inertiaA.InverseInertiaTensor, out var angularA0);
            Triangular3x3Wide.VectorSandwich(projection.Penetration0.AngularB, inertiaB.InverseInertiaTensor, out var angularB0);
            Triangular3x3Wide.VectorSandwich(projection.Penetration1.AngularA, inertiaA.InverseInertiaTensor, out var angularA1);
            Triangular3x3Wide.VectorSandwich(projection.Penetration1.AngularB, inertiaB.InverseInertiaTensor, out var angularB1);
            Triangular3x3Wide.VectorSandwich(projection.Penetration2.AngularA, inertiaA.InverseInertiaTensor, out var angularA2);
            Triangular3x3Wide.VectorSandwich(projection.Penetration2.AngularB, inertiaB.InverseInertiaTensor, out var angularB2);

            //Linear effective mass contribution notes:
            //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
            //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
            SpringSettingsWide.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var linear = inertiaA.InverseMass + inertiaB.InverseMass;
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.Penetration0.EffectiveMass = effectiveMassCFMScale / (linear + angularA0 + angularB0);
            projection.Penetration1.EffectiveMass = effectiveMassCFMScale / (linear + angularA1 + angularB1);
            projection.Penetration2.EffectiveMass = effectiveMassCFMScale / (linear + angularA2 + angularB2);

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
        public static void ApplyImpulse(ref PenetrationLimitProjection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> correctiveImpulse,
            ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            var linearVelocityChangeA = correctiveImpulse * inertiaA.InverseMass;
            Vector3Wide.Scale(normal, linearVelocityChangeA, out var correctiveVelocityALinearVelocity);
            Vector3Wide.Scale(projection.AngularA, correctiveImpulse, out var correctiveAngularImpulseA);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(correctiveAngularImpulseA, inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);

            var linearVelocityChangeB = correctiveImpulse * inertiaB.InverseMass;
            Vector3Wide.Scale(normal, linearVelocityChangeB, out var correctiveVelocityBLinearVelocity);
            Vector3Wide.Scale(projection.AngularB, correctiveImpulse, out var correctiveAngularImpulseB);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(correctiveAngularImpulseB, inertiaB.InverseInertiaTensor, out var correctiveVelocityBAngularVelocity);

            Vector3Wide.Add(wsvA.Linear, correctiveVelocityALinearVelocity, out wsvA.Linear);
            Vector3Wide.Add(wsvA.Angular, correctiveVelocityAAngularVelocity, out wsvA.Angular);
            Vector3Wide.Subtract(wsvB.Linear, correctiveVelocityBLinearVelocity, out wsvB.Linear); //Note subtract; normal = -jacobianLinearB
            Vector3Wide.Add(wsvB.Angular, correctiveVelocityBAngularVelocity, out wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            ref Projection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse0, ref wsvA, ref wsvB);
            ApplyImpulse(ref projection.Penetration1, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse1, ref wsvA, ref wsvB);
            ApplyImpulse(ref projection.Penetration2, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse2, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB,
            ref PenetrationLimitProjection projection,
            ref Vector3Wide normal, ref Vector<float> softnessImpulseScale,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            //Note that we do NOT use pretransformed jacobians here; the linear jacobian sharing (normal) meant that we had the effective mass anyway.
            Vector3Wide.Dot(wsvA.Linear, normal, out var csvaLinear);
            Vector3Wide.Dot(wsvA.Angular, projection.AngularA, out var csvaAngular);
            Vector3Wide.Dot(wsvB.Linear, normal, out var negatedCSVBLinear);
            Vector3Wide.Dot(wsvB.Angular, projection.AngularB, out var csvbAngular);
            //Compute negated version to avoid the need for an explicit negate.
            var negatedCSI = accumulatedImpulse * softnessImpulseScale + (csvaLinear - negatedCSVBLinear + csvaAngular + csvbAngular - projection.BiasVelocity) * projection.EffectiveMass;

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse - negatedCSI);

            correctiveCSI = accumulatedImpulse - previousAccumulated;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref Projection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration0, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse0, out var correctiveCSI0);
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI0, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration1, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse1, out var correctiveCSI1);
            ApplyImpulse(ref projection.Penetration1, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI1, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration2, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse2, out var correctiveCSI2);
            ApplyImpulse(ref projection.Penetration2, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI2, ref wsvA, ref wsvB);
        }

    }
}
