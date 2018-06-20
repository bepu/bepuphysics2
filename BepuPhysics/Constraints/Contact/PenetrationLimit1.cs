using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Constraints.Contact
{


    /// <summary>
    /// Four convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public static class PenetrationLimit1
    {
        public struct Projection
        {
            public PenetrationLimitProjection Penetration0;
            public Vector<float> SoftnessImpulseScale;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Contact1PrestepData prestep, float dt, float inverseDt,
            out Projection projection)
        {
            Vector3Wide.Subtract(prestep.OffsetA0, prestep.OffsetB, out var contactOffsetB);
            Prestep(ref inertiaA, ref inertiaB, 
                ref prestep.OffsetA0, ref contactOffsetB, ref prestep.Normal, ref prestep.PenetrationDepth0, ref prestep.SpringSettings, ref prestep.MaximumRecoveryVelocity, 
                dt, inverseDt, out projection);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, 
            ref Vector3Wide contactOffsetA, ref Vector3Wide contactOffsetB, ref Vector3Wide normal, ref Vector<float> depth, ref SpringSettingsWide springSettings, ref Vector<float> maximumRecoveryVelocity,
            float dt, float inverseDt,
            out Projection projection)
        {
            //We directly take the prestep data here since the jacobians and error don't undergo any processing.

            //The contact penetration constraint takes the form:
            //dot(positionA + offsetA, N) >= dot(positionB + offsetB, N)
            //Or:
            //dot(positionA + offsetA, N) - dot(positionB + offsetB, N) >= 0
            //dot(positionA + offsetA - positionB - offsetB, N) >= 0
            //where positionA and positionB are the center of mass positions of the bodies offsetA and offsetB are world space offsets from the center of mass to the contact,
            //and N is a unit length vector calibrated to point from B to A. (The normal pointing direction is important; it changes the sign.)
            //In practice, we'll use the collision detection system's penetration depth instead of trying to recompute the error here.

            //So, treating the normal as constant, the velocity constraint is:
            //dot(d/dt(positionA + offsetA - positionB - offsetB), N) >= 0
            //dot(linearVelocityA + d/dt(offsetA) - linearVelocityB - d/dt(offsetB)), N) >= 0
            //The velocity of the offsets are defined by the angular velocity.
            //dot(linearVelocityA + angularVelocityA x offsetA - linearVelocityB - angularVelocityB x offsetB), N) >= 0
            //dot(linearVelocityA, N) + dot(angularVelocityA x offsetA, N) - dot(linearVelocityB, N) - dot(angularVelocityB x offsetB), N) >= 0
            //Use the properties of the scalar triple product:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) - dot(linearVelocityB, N) - dot(offsetB x N, angularVelocityB) >= 0
            //Bake in the negations:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(-offsetB x N, angularVelocityB) >= 0
            //A x B = -B x A:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(N x offsetB, angularVelocityB) >= 0
            //And there you go, the jacobians!
            //linearA: N
            //angularA: offsetA x N
            //linearB: -N
            //angularB: N x offsetB
            //Note that we leave the penetration depth as is, even when it's negative. Speculative contacts!
            Vector3Wide.CrossWithoutOverlap(contactOffsetA, normal, out projection.Penetration0.AngularA);
            Vector3Wide.CrossWithoutOverlap(normal, contactOffsetB, out projection.Penetration0.AngularB);

            //effective mass
            Symmetric3x3Wide.VectorSandwich(projection.Penetration0.AngularA, inertiaA.InverseInertiaTensor, out var angularA0);
            Symmetric3x3Wide.VectorSandwich(projection.Penetration0.AngularB, inertiaB.InverseInertiaTensor, out var angularB0);

            //Linear effective mass contribution notes:
            //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
            //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
            SpringSettingsWide.ComputeSpringiness(ref springSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var linear = inertiaA.InverseMass + inertiaB.InverseMass;
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.Penetration0.EffectiveMass = effectiveMassCFMScale / (linear + angularA0 + angularB0);

            //If depth is negative, the bias velocity will permit motion up until the depth hits zero. This works because positionErrorToVelocity * dt will always be <=1.
            projection.Penetration0.BiasVelocity = Vector.Min(
                depth * new Vector<float>(inverseDt),
                Vector.Min(depth * positionErrorToVelocity, maximumRecoveryVelocity));
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
            Symmetric3x3Wide.TransformWithoutOverlap(correctiveAngularImpulseA, inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);

            var linearVelocityChangeB = correctiveImpulse * inertiaB.InverseMass;
            Vector3Wide.Scale(normal, linearVelocityChangeB, out var correctiveVelocityBLinearVelocity);
            Vector3Wide.Scale(projection.AngularB, correctiveImpulse, out var correctiveAngularImpulseB);
            Symmetric3x3Wide.TransformWithoutOverlap(correctiveAngularImpulseB, inertiaB.InverseInertiaTensor, out var correctiveVelocityBAngularVelocity);

            Vector3Wide.Add(wsvA.Linear, correctiveVelocityALinearVelocity, out wsvA.Linear);
            Vector3Wide.Add(wsvA.Angular, correctiveVelocityAAngularVelocity, out wsvA.Angular);
            Vector3Wide.Subtract(wsvB.Linear, correctiveVelocityBLinearVelocity, out wsvB.Linear); //Note subtract; normal = -jacobianLinearB
            Vector3Wide.Add(wsvB.Angular, correctiveVelocityBAngularVelocity, out wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            ref Projection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse0, ref wsvA, ref wsvB);
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
            ref Vector<float> accumulatedImpulse0, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration0, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse0, out var correctiveCSI0);
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI0, ref wsvA, ref wsvB);
        }

    }
}
