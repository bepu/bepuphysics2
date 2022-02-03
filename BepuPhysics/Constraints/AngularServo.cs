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
    /// Constrains two bodies to have a target relative rotation.
    /// </summary>
    public struct AngularServo : ITwoBodyConstraintDescription<AngularServo>
    {
        /// <summary>
        /// The target relative rotation from body A to body B in body A's local space. The constraint tries to maintain OrientationB = TargetRelativeRotationLocalA * OrientationA.
        /// </summary>
        public Quaternion TargetRelativeRotationLocalA;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;
        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(AngularServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(TargetRelativeRotationLocalA, nameof(AngularServo), nameof(TargetRelativeRotationLocalA));
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(AngularServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(TargetRelativeRotationLocalA, ref target.TargetRelativeRotationLocalA);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.ReadFirst(source.TargetRelativeRotationLocalA, out description.TargetRelativeRotationLocalA);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct AngularServoPrestepData
    {
        public QuaternionWide TargetRelativeRotationLocalA;
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct AngularServoFunctions : ITwoBodyConstraintFunctions<AngularServoPrestepData, Vector3Wide>
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, in Symmetric3x3Wide impulseToVelocityA, in Symmetric3x3Wide negatedImpulseToVelocityB, in Vector3Wide csi)
        {
            Symmetric3x3Wide.TransformWithoutOverlap(csi, impulseToVelocityA, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(csi, negatedImpulseToVelocityB, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB,
        //    in Symmetric3x3Wide effectiveMass, in Vector<float> softnessImpulseScale, in Vector3Wide biasImpulse, in Vector<float> maximumImpulse,
        //    in Symmetric3x3Wide impulseToVelocityA, in Symmetric3x3Wide negatedImpulseToVelocityB, ref Vector3Wide accumulatedImpulse)
        //{
        //    //Jacobians are just I and -I.
        //    Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var csv);
        //    Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out var csiVelocityComponent);
        //    //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
        //    Vector3Wide.Scale(accumulatedImpulse, softnessImpulseScale, out var softnessComponent);
        //    Vector3Wide.Subtract(biasImpulse, softnessComponent, out var csi);
        //    Vector3Wide.Subtract(csi, csiVelocityComponent, out csi);

        //    ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulse, ref csi);

        //    ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, impulseToVelocityA, negatedImpulseToVelocityB, csi);
        //}


        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref AngularServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ApplyImpulse(ref wsvA.Angular, ref wsvB.Angular, inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref AngularServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //Jacobians are just I and -I.
            QuaternionWide.ConcatenateWithoutOverlap(prestep.TargetRelativeRotationLocalA, orientationA, out var targetOrientationB);
            QuaternionWide.Conjugate(targetOrientationB, out var inverseTarget);
            QuaternionWide.ConcatenateWithoutOverlap(inverseTarget, orientationB, out var errorRotation);

            QuaternionWide.GetAxisAngleFromQuaternion(errorRotation, out var errorAxis, out var errorLength);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            Symmetric3x3Wide.Add(inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, out var unsoftenedInverseEffectiveMass);
            Symmetric3x3Wide.Invert(unsoftenedInverseEffectiveMass, out var unsoftenedEffectiveMass);
            //Note effective mass is not directly scaled by CFM scale; instead scale the CSI.

            ServoSettingsWide.ComputeClampedBiasVelocity(errorAxis, errorLength, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var clampedBiasVelocity, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Subtract(wsvA.Angular, wsvB.Angular, out var csv);
            Vector3Wide.Subtract(clampedBiasVelocity, csv, out csv);
            Symmetric3x3Wide.TransformWithoutOverlap(csv, unsoftenedEffectiveMass, out var csi);
            csi *= effectiveMassCFMScale;
            Vector3Wide.Scale(accumulatedImpulses, softnessImpulseScale, out var softnessComponent);
            Vector3Wide.Subtract(csi, softnessComponent, out csi);

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);

            ApplyImpulse(ref wsvA.Angular, ref wsvB.Angular, inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, csi);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref AngularServoPrestepData prestepData) { }
    }

    public class AngularServoTypeProcessor : TwoBodyTypeProcessor<AngularServoPrestepData, Vector3Wide, AngularServoFunctions, AccessOnlyAngularWithoutPose, AccessOnlyAngularWithoutPose, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 29;
    }
}

