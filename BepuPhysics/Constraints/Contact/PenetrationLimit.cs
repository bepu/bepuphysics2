using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Constraints.Contact
{
    public static class PenetrationLimit
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(in BodyVelocityWide wsvA, in BodyVelocityWide wsvB,
            in Vector3Wide normal, in Vector3Wide angularA, in Vector3Wide angularB, in Vector<float> biasVelocity, in Vector<float> softnessImpulseScale, in Vector<float> effectiveMass,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            //Note that we do NOT use pretransformed jacobians here; the linear jacobian sharing (normal) meant that we had the effective mass anyway.
            Vector3Wide.Dot(wsvA.Linear, normal, out var csvaLinear);
            Vector3Wide.Dot(wsvA.Angular, angularA, out var csvaAngular);
            Vector3Wide.Dot(wsvB.Linear, normal, out var negatedCSVBLinear);
            Vector3Wide.Dot(wsvB.Angular, angularB, out var csvbAngular);
            //Compute negated version to avoid the need for an explicit negate.
            var negatedCSI = accumulatedImpulse * softnessImpulseScale + (csvaLinear - negatedCSVBLinear + csvaAngular + csvbAngular - biasVelocity) * effectiveMass;

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse - negatedCSI);

            correctiveCSI = accumulatedImpulse - previousAccumulated;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdatePenetrationDepth(in Vector<float> dt, in Vector3Wide contactOffsetA, in Vector3Wide offsetB, in Vector3Wide normal, in BodyVelocityWide velocityA, in BodyVelocityWide velocityB, ref Vector<float> penetrationDepth)
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, in Vector3Wide normal, in Vector3Wide angularA, in Vector3Wide angularB,
            in Vector<float> correctiveImpulse,
            ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            var linearVelocityChangeA = correctiveImpulse * inertiaA.InverseMass;
            Vector3Wide.Scale(normal, linearVelocityChangeA, out var correctiveVelocityALinearVelocity);
            Vector3Wide.Scale(angularA, correctiveImpulse, out var correctiveAngularImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(correctiveAngularImpulseA, inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);

            var linearVelocityChangeB = correctiveImpulse * inertiaB.InverseMass;
            Vector3Wide.Scale(normal, linearVelocityChangeB, out var correctiveVelocityBLinearVelocity);
            Vector3Wide.Scale(angularB, correctiveImpulse, out var correctiveAngularImpulseB);
            Symmetric3x3Wide.TransformWithoutOverlap(correctiveAngularImpulseB, inertiaB.InverseInertiaTensor, out var correctiveVelocityBAngularVelocity);

            Vector3Wide.Add(wsvA.Linear, correctiveVelocityALinearVelocity, out wsvA.Linear);
            Vector3Wide.Add(wsvA.Angular, correctiveVelocityAAngularVelocity, out wsvA.Angular);
            Vector3Wide.Subtract(wsvB.Linear, correctiveVelocityBLinearVelocity, out wsvB.Linear); //Note subtract; normal = -jacobianLinearB
            Vector3Wide.Add(wsvB.Angular, correctiveVelocityBAngularVelocity, out wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, in Vector3Wide normal, in Vector3Wide contactOffsetA, in Vector3Wide contactOffsetB,
            in Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Vector3Wide.CrossWithoutOverlap(contactOffsetA, normal, out var angularA);
            Vector3Wide.CrossWithoutOverlap(normal, contactOffsetB, out var angularB);
            ApplyImpulse(inertiaA, inertiaB, normal, angularA, angularB, accumulatedImpulse, ref wsvA, ref wsvB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(
            in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, in Vector3Wide normal, in Vector3Wide contactOffsetA, in Vector3Wide contactOffsetB,
            in Vector<float> depth, in Vector<float> positionErrorToVelocity, in Vector<float> effectiveMassCFMScale, in Vector<float> maximumRecoveryVelocity, in Vector<float> inverseDt, in Vector<float> softnessImpulseScale,
            ref Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
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
            Vector3Wide.CrossWithoutOverlap(contactOffsetA, normal, out var angularA);
            Vector3Wide.CrossWithoutOverlap(normal, contactOffsetB, out var angularB);

            //effective mass
            Symmetric3x3Wide.VectorSandwich(angularA, inertiaA.InverseInertiaTensor, out var angularA0);
            Symmetric3x3Wide.VectorSandwich(angularB, inertiaB.InverseInertiaTensor, out var angularB0);

            //Linear effective mass contribution notes:
            //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
            //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
            var linear = inertiaA.InverseMass + inertiaB.InverseMass;
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            var effectiveMass = effectiveMassCFMScale / (linear + angularA0 + angularB0);

            //If depth is negative, the bias velocity will permit motion up until the depth hits zero. This works because positionErrorToVelocity * dt will always be <=1.
            var biasVelocity = Vector.Min(
                depth * inverseDt,
                Vector.Min(depth * positionErrorToVelocity, maximumRecoveryVelocity));

            ComputeCorrectiveImpulse(wsvA, wsvB, normal, angularA, angularB, biasVelocity, softnessImpulseScale, effectiveMass, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(inertiaA, inertiaB, normal, angularA, angularB, correctiveCSI, ref wsvA, ref wsvB);
        }
    }
}
