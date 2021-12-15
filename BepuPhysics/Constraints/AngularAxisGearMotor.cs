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
    /// Constrains body B's angular velocity around an axis anchored to body A to equal body A's velocity around that axis with a scaling factor applied.
    /// </summary>
    public struct AngularAxisGearMotor : ITwoBodyConstraintDescription<AngularAxisGearMotor>
    {
        /// <summary>
        /// Axis of rotation in body A's local space.
        /// </summary>
        public Vector3 LocalAxisA;
        /// <summary>
        /// <para>Scale to apply to body A's velocity around the axis to get body B's target velocity.</para>
        /// <para>In other words, a VelocityScale of 2 means that body A could have a velocity of 3 while body B has a velocity of 6.</para>
        /// </summary>
        public float VelocityScale;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularAxisGearMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(AngularAxisGearMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalAxisA, nameof(AngularAxisGearMotor), nameof(LocalAxisA));
            ConstraintChecker.AssertValid(Settings, nameof(AngularAxisGearMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularAxisGearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalAxisA, ref target.LocalAxisA);
            GetFirst(ref target.VelocityScale) = VelocityScale;
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularAxisGearMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularAxisGearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalAxisA, out description.LocalAxisA);
            description.VelocityScale = GetFirst(ref source.VelocityScale);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct AngularAxisGearMotorPrestepData
    {
        public Vector3Wide LocalAxisA;
        public Vector<float> VelocityScale;
        public MotorSettingsWide Settings;
    }

    public struct AngularAxisGearMotorFunctions : ITwoBodyConstraintFunctions<AngularAxisGearMotorPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3Wide impulseToVelocityA, in Vector3Wide negatedImpulseToVelocityB, in Vector<float> csi, ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB)
        {
            angularVelocityA += impulseToVelocityA * csi;
            angularVelocityB -= negatedImpulseToVelocityB * csi;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref AngularAxisGearMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalAxisA, orientationA, out var axis);
            Vector3Wide.Scale(axis, prestep.VelocityScale, out var jA);
            Symmetric3x3Wide.TransformWithoutOverlap(jA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(axis, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, accumulatedImpulses, ref wsvA.Angular, ref wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref AngularAxisGearMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //This is mildly more complex than the AngularAxisMotor:
            //dot(wa, axis) * velocityScale - dot(wb, axis) = 0, so jacobianA is actually axis * velocityScale, not just -axis.
            QuaternionWide.TransformWithoutOverlap(prestep.LocalAxisA, orientationA, out var axis);
            Vector3Wide.Scale(axis, prestep.VelocityScale, out var jA);
            Symmetric3x3Wide.TransformWithoutOverlap(jA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            Vector3Wide.Dot(jA, impulseToVelocityA, out var contributionA);
            Symmetric3x3Wide.TransformWithoutOverlap(axis, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            Vector3Wide.Dot(axis, negatedImpulseToVelocityB, out var contributionB);
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out var softnessImpulseScale, out var maximumImpulse);
            var effectiveMass = effectiveMassCFMScale / (contributionA + contributionB);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(wsvA.Angular, jA, out var unscaledCSVA);
            Vector3Wide.Dot(wsvB.Angular, axis, out var negatedCSVB);
            var csi = (negatedCSVB - unscaledCSVA) * effectiveMass - accumulatedImpulses * softnessImpulseScale;
            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, accumulatedImpulses, ref wsvA.Angular, ref wsvB.Angular);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref AngularAxisGearMotorPrestepData prestepData) { }
    }

    public class AngularAxisGearMotorTypeProcessor : TwoBodyTypeProcessor<AngularAxisGearMotorPrestepData, Vector<float>, AngularAxisGearMotorFunctions, AccessOnlyAngular, AccessOnlyAngularWithoutPose, AccessOnlyAngular, AccessOnlyAngularWithoutPose>
    {
        public const int BatchTypeId = 54;
    }
}

