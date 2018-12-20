using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Constraints.Contact
{
    public struct PenetrationLimitProjection
    {
        //Note that these are just the raw jacobians, no precomputation with the JT*EffectiveMass.
        public Vector3Wide AngularA;
        public Vector3Wide AngularB;
        public Vector<float> EffectiveMass;
        public Vector<float> BiasVelocity;
    }

    public static class PenetrationLimit
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(in BodyInertias inertiaA, in BodyInertias inertiaB,
            in Vector3Wide contactOffsetA, in Vector3Wide contactOffsetB, in Vector3Wide normal, in Vector<float> depth,
            in Vector<float> positionErrorToVelocity, in Vector<float> effectiveMassCFMScale, in Vector<float> maximumRecoveryVelocity,
            float inverseDt, out PenetrationLimitProjection projection)
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
            Vector3Wide.CrossWithoutOverlap(contactOffsetA, normal, out projection.AngularA);
            Vector3Wide.CrossWithoutOverlap(normal, contactOffsetB, out projection.AngularB);

            //effective mass
            Symmetric3x3Wide.VectorSandwich(projection.AngularA, inertiaA.InverseInertiaTensor, out var angularA0);
            Symmetric3x3Wide.VectorSandwich(projection.AngularB, inertiaB.InverseInertiaTensor, out var angularB0);

            //Linear effective mass contribution notes:
            //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
            //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
            var linear = inertiaA.InverseMass + inertiaB.InverseMass;
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.EffectiveMass = effectiveMassCFMScale / (linear + angularA0 + angularB0);

            //If depth is negative, the bias velocity will permit motion up until the depth hits zero. This works because positionErrorToVelocity * dt will always be <=1.
            projection.BiasVelocity = Vector.Min(
                depth * new Vector<float>(inverseDt),
                Vector.Min(depth * positionErrorToVelocity, maximumRecoveryVelocity));
        }


        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in PenetrationLimitProjection projection, in BodyInertias inertiaA, in BodyInertias inertiaB, in Vector3Wide normal,
            in Vector<float> correctiveImpulse,
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
            in PenetrationLimitProjection projection, in BodyInertias inertiaA, in BodyInertias inertiaB, in Vector3Wide normal,
            in Vector<float> accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ApplyImpulse(projection, inertiaA, inertiaB, normal, accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(in BodyVelocities wsvA, in BodyVelocities wsvB,
            in PenetrationLimitProjection projection,
            in Vector3Wide normal, in Vector<float> softnessImpulseScale,
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
        public static void Solve(in PenetrationLimitProjection projection, in BodyInertias inertiaA, in BodyInertias inertiaB, in Vector3Wide normal,
            in Vector<float> softnessImpulseScale, ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(wsvA, wsvB, projection, normal, softnessImpulseScale, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(projection, inertiaA, inertiaB, normal, correctiveCSI, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdatePenetrationDepth(in Vector<float> dt, in Vector3Wide contactOffsetA, in Vector3Wide offsetB, in Vector3Wide normal, in BodyVelocities velocityA, in BodyVelocities velocityB, ref Vector<float> penetrationDepth)
        {
            //The normal is calibrated to point from B to A. Any movement of A along N results in a decrease in depth. Any movement of B along N results in an increase in depth. 
            //estimatedPenetrationDepthChange = dot(normal, velocityDtA.Linear + velocityDtA.Angular x contactOffsetA) - dot(normal, velocityDtB.Linear + velocityDtB.Angular x contactOffsetB)
            Vector3Wide.CrossWithoutOverlap(velocityA.Angular, contactOffsetA, out var wxra);
            Vector3Wide.Add(wxra, velocityA.Linear, out var contactVelocityA);

            Vector3Wide.Subtract(contactOffsetA, offsetB, out var contactOffsetB);
            Vector3Wide.CrossWithoutOverlap(velocityB.Angular, contactOffsetB, out var wxrb);
            Vector3Wide.Add(wxrb, velocityB.Linear, out var contactVelocityB);

            Vector3Wide.Subtract(contactVelocityA, contactVelocityB, out var contactVelocityDifference);
            Vector3Wide.Dot(normal, contactVelocityDifference, out var estimatedDepthChangeVelocity);
            penetrationDepth -= estimatedDepthChangeVelocity * dt;
        }
    }
}
