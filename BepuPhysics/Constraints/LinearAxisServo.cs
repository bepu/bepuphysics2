using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;
namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Constrains points on two bodies to be on a plane defined in the local space of one of the bodies.
    /// </summary>
    public struct LinearAxisServo : ITwoBodyConstraintDescription<LinearAxisServo>
    {
        /// <summary>
        /// Local offset from the center of body A to its attachment point.
        /// </summary>
        public Vector3 LocalOffsetA;
        /// <summary>
        /// Local offset from the center of body B to its attachment point.
        /// </summary>
        public Vector3 LocalOffsetB;
        /// <summary>
        /// Direction of the plane normal in the local space of body A.
        /// </summary>
        public Vector3 LocalPlaneNormal;
        /// <summary>
        /// Target offset from A's plane anchor to B's anchor along the plane normal.
        /// </summary>
        public float TargetOffset;
        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return LinearAxisServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(LinearAxisServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalPlaneNormal, nameof(LinearAxisServo), nameof(LocalPlaneNormal));
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(LinearAxisServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<LinearAxisServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(LocalPlaneNormal, ref target.LocalPlaneNormal);
            GatherScatter.GetFirst(ref target.TargetOffset) = TargetOffset;
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out LinearAxisServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<LinearAxisServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            Vector3Wide.ReadFirst(source.LocalPlaneNormal, out description.LocalPlaneNormal);
            description.TargetOffset = GatherScatter.GetFirst(ref source.TargetOffset);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct LinearAxisServoPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public Vector3Wide LocalPlaneNormal;
        public Vector<float> TargetOffset;
        public ServoSettingsWide ServoSettings;
        public SpringSettingsWide SpringSettings;
    }

    public struct LinearAxisServoFunctions : ITwoBodyConstraintFunctions<LinearAxisServoPrestepData, Vector<float>>
    {
        public interface IJacobianModifier
        {
            void Modify(in Vector3Wide anchorA, in Vector3Wide anchorB, ref Vector3Wide normal);
        }

        public struct NoChangeModifier : IJacobianModifier
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Modify(in Vector3Wide anchorA, in Vector3Wide anchorB, ref Vector3Wide normal)
            {
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeTransforms<TJacobianModifier>(ref TJacobianModifier jacobianModifier,
            in Vector3Wide localOffsetA, in Vector3Wide localOffsetB, in Vector3Wide localPlaneNormal,
            in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            in Vector<float> effectiveMassCFMScale,
            out Vector3Wide anchorA, out Vector3Wide anchorB, out Vector3Wide normal, out Vector<float> effectiveMass,
            out Vector3Wide linearVelocityToImpulseA, out Vector3Wide angularVelocityToImpulseA, out Vector3Wide angularVelocityToImpulseB,
            out Vector3Wide linearImpulseToVelocityA, out Vector3Wide angularImpulseToVelocityA, out Vector3Wide negatedLinearImpulseToVelocityB, out Vector3Wide angularImpulseToVelocityB)
            where TJacobianModifier : IJacobianModifier
        {
            //This is similar to the point on line joint in that we pick the closest point on the plane as the A's offset.
            //From there, the jacobians are very similar to penetration constraints.
            //dot(closestPointOnPlaneToAnchorB - anchorB, planeNormal) = goal
            //Treating planeNormal as constant, the velocity constraint is:
            //dot(d/dt(closestPointOnPlaneToAnchorB - anchorB), planeNormal) = 0
            //dot(linearA + angularA x offsetA - linearB - angularB x offsetB), planeNormal) = 0
            //dot(linearA - linearB, planeNormal) + dot(angularA x offsetA, planeNormal) + dot(offsetB x angularB, planeNormal) = 0
            //dot(linearA - linearB, planeNormal) + dot(offsetA x planeNormal, angularA) + dot(planeNormal x offsetB, angularB) = 0
            //We'll just use the offset from a to anchorB as the 'offsetA' above.
            //(Note that there's no mathy reason why TargetOffset exists over just the LocalOffsetA alone; it's a usability thing.)
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA, orientationMatrixA, out anchorA);
            Matrix3x3Wide.TransformWithoutOverlap(localPlaneNormal, orientationMatrixA, out normal);
            QuaternionWide.TransformWithoutOverlap(localOffsetB, orientationB, out var offsetB);
            Vector3Wide.Add(ab, offsetB, out anchorB);
            jacobianModifier.Modify(anchorA, anchorB, ref normal);

            //This is a 1DOF constraint, so premultiplication is the best option. Store out JT * Me and J * I^-1. Can avoid storing out JT * Me for linearB since it's just linearA negated.

            Vector3Wide.CrossWithoutOverlap(anchorB, normal, out var angularA);
            Vector3Wide.CrossWithoutOverlap(normal, offsetB, out var angularB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularA, inertiaA.InverseInertiaTensor, out angularImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularB, inertiaB.InverseInertiaTensor, out angularImpulseToVelocityB);
            Vector3Wide.Dot(angularA, angularImpulseToVelocityA, out var angularContributionA);
            Vector3Wide.Dot(angularB, angularImpulseToVelocityB, out var angularContributionB);
            effectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + inertiaB.InverseMass + angularContributionA + angularContributionB);

            Vector3Wide.Scale(normal, inertiaA.InverseMass, out linearImpulseToVelocityA);
            Vector3Wide.Scale(normal, inertiaB.InverseMass, out negatedLinearImpulseToVelocityB); //can save one scalar here by storing inverse masses but.. ehhh...

            Vector3Wide.Scale(normal, effectiveMass, out linearVelocityToImpulseA);
            Vector3Wide.Scale(angularA, effectiveMass, out angularVelocityToImpulseA);
            Vector3Wide.Scale(angularB, effectiveMass, out angularVelocityToImpulseB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB,
            in Vector3Wide linearImpulseToVelocityA, in Vector3Wide angularImpulseToVelocityA, in Vector3Wide negatedLinearImpulseToVelocityB, in Vector3Wide angularImpulseToVelocityB, ref Vector<float> csi)
        {
            Vector3Wide.Scale(linearImpulseToVelocityA, csi, out var linearChangeA);
            Vector3Wide.Scale(angularImpulseToVelocityA, csi, out var angularChangeA);
            Vector3Wide.Scale(negatedLinearImpulseToVelocityB, csi, out var negatedLinearChangeB);
            Vector3Wide.Scale(angularImpulseToVelocityB, csi, out var angularChangeB);

            Vector3Wide.Add(linearChangeA, velocityA.Linear, out velocityA.Linear);
            Vector3Wide.Add(angularChangeA, velocityA.Angular, out velocityA.Angular);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearChangeB, out velocityB.Linear);
            Vector3Wide.Add(angularChangeB, velocityB.Angular, out velocityB.Angular);
        }

        public static void ComputeCorrectiveImpulse(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB,
            in Vector3Wide linearVelocityToImpulseA, in Vector3Wide angularVelocityToImpulseA, in Vector3Wide angularVelocityToImpulseB,
            in Vector<float> biasImpulse, in Vector<float> softnessImpulseScale, in Vector<float> accumulatedImpulse, out Vector<float> csi)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(velocityA.Linear, linearVelocityToImpulseA, out var linearA);
            Vector3Wide.Dot(velocityB.Linear, linearVelocityToImpulseA, out var negatedLinearB);
            Vector3Wide.Dot(velocityA.Angular, angularVelocityToImpulseA, out var angularA);
            Vector3Wide.Dot(velocityB.Angular, angularVelocityToImpulseB, out var angularB);

            csi = biasImpulse - accumulatedImpulse * softnessImpulseScale - (linearA + angularA - negatedLinearB + angularB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3Wide linearJA, in Vector3Wide angularImpulseToVelocityA, in Vector3Wide angularImpulseToVelocityB, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector<float> csi, ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB)
        {
            velocityA.Linear += linearJA * (csi * inertiaA.InverseMass);
            velocityB.Linear -= linearJA * (csi * inertiaB.InverseMass);
            velocityA.Angular += angularImpulseToVelocityA * csi;
            velocityB.Angular += angularImpulseToVelocityB * csi;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(in Vector3Wide ab, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector3Wide localPlaneNormalA, in Vector3Wide localOffsetA, in Vector3Wide localOffsetB,
            out Vector<float> planeNormalDot, out Vector3Wide normal, out Vector3Wide angularJA, out Vector3Wide angularJB)
        {
            //Linear jacobians are just normal and -normal. Angular jacobians are offsetA x normal and offsetB x normal.
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(localPlaneNormalA, orientationMatrixA, out normal);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA, orientationMatrixA, out var anchorA);
            QuaternionWide.TransformWithoutOverlap(localOffsetB, orientationB, out var offsetB);
            //Note that the angular jacobian for A uses the offset from A to the attachment point on B. 
            var anchorB = ab + offsetB;
            Vector3Wide.Dot(anchorB - anchorA, normal, out planeNormalDot);
            var offsetFromAToClosetPointOnPlaneToB = anchorB - planeNormalDot * normal;
            Vector3Wide.CrossWithoutOverlap(offsetFromAToClosetPointOnPlaneToB, normal, out angularJA);
            Vector3Wide.CrossWithoutOverlap(normal, offsetB, out angularJB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeEffectiveMass(in Vector3Wide angularJA, in Vector3Wide angularJB,
            in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector<float> effectiveMassCFMScale, out Vector3Wide angularImpulseToVelocityA, out Vector3Wide angularImpulseToVelocityB, out Vector<float> effectiveMass)
        {
            Symmetric3x3Wide.TransformWithoutOverlap(angularJA, inertiaA.InverseInertiaTensor, out angularImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJB, inertiaB.InverseInertiaTensor, out angularImpulseToVelocityB);
            Vector3Wide.Dot(angularJA, angularImpulseToVelocityA, out var angularContributionA);
            Vector3Wide.Dot(angularJB, angularImpulseToVelocityB, out var angularContributionB);
            effectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + inertiaB.InverseMass + angularContributionA + angularContributionB);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref LinearAxisServoPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalPlaneNormal, prestep.LocalOffsetA, prestep.LocalOffsetB, out _, out var normal, out var angularJA, out var angularJB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJA, inertiaA.InverseInertiaTensor, out var angularImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJB, inertiaB.InverseInertiaTensor, out var angularImpulseToVelocityB);
            ApplyImpulse(normal, angularImpulseToVelocityA, angularImpulseToVelocityB, inertiaA, inertiaB, accumulatedImpulses, ref wsvA, ref wsvB);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref LinearAxisServoPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalPlaneNormal, prestep.LocalOffsetA, prestep.LocalOffsetB, out var planeNormalDot, out var normal, out var angularJA, out var angularJB);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            ComputeEffectiveMass(angularJA, angularJB, inertiaA, inertiaB, effectiveMassCFMScale, out var angularImpulseToVelocityA, out var angularImpulseToVelocityB, out var effectiveMass);


            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            ServoSettingsWide.ComputeClampedBiasVelocity(planeNormalDot - prestep.TargetOffset, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var biasVelocity, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csv = Vector3Wide.Dot(wsvA.Linear - wsvB.Linear, normal) + Vector3Wide.Dot(wsvA.Angular, angularJA) + Vector3Wide.Dot(wsvB.Angular, angularJB);

            var csi = effectiveMass * (biasVelocity - csv) - accumulatedImpulses * softnessImpulseScale;

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            ApplyImpulse(normal, angularImpulseToVelocityA, angularImpulseToVelocityB, inertiaA, inertiaB, csi, ref wsvA, ref wsvB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref LinearAxisServoPrestepData prestepData) { }
    }

    public class LinearAxisServoTypeProcessor : TwoBodyTypeProcessor<LinearAxisServoPrestepData, Vector<float>, LinearAxisServoFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 38;
    }
}
