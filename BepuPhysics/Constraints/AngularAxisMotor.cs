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
    /// Constrains the relative angular velocity of two bodies around a local axis attached to body A to a target velocity.
    /// </summary>
    public struct AngularAxisMotor : ITwoBodyConstraintDescription<AngularAxisMotor>
    {
        /// <summary>
        /// Axis of rotation in body A's local space.
        /// </summary>
        public Vector3 LocalAxisA;
        /// <summary>
        /// Target relative angular velocity around the axis.
        /// </summary>
        public float TargetVelocity;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularAxisMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(AngularAxisMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalAxisA, nameof(AngularAxisMotor), nameof(LocalAxisA));
            ConstraintChecker.AssertValid(Settings, nameof(AngularAxisMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularAxisMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalAxisA, ref target.LocalAxisA);
            GatherScatter.GetFirst(ref target.TargetVelocity) = TargetVelocity;
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularAxisMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularAxisMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalAxisA, out description.LocalAxisA);
            description.TargetVelocity = GatherScatter.GetFirst(ref source.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct AngularAxisMotorPrestepData
    {
        public Vector3Wide LocalAxisA;
        public Vector<float> TargetVelocity;
        public MotorSettingsWide Settings;
    }

    public struct AngularAxisMotorFunctions : ITwoBodyConstraintFunctions<AngularAxisMotorPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3Wide impulseToVelocityA, in Vector3Wide negatedImpulseToVelocityB, in Vector<float> csi, ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB)
        {
            angularVelocityA += impulseToVelocityA * csi;
            angularVelocityB -= negatedImpulseToVelocityB * csi;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref AngularAxisMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalAxisA, orientationA, out var axis);
            Symmetric3x3Wide.TransformWithoutOverlap(axis, inertiaA.InverseInertiaTensor, out var jIA);
            Symmetric3x3Wide.TransformWithoutOverlap(axis, inertiaB.InverseInertiaTensor, out var jIB);
            ApplyImpulse(jIA, jIB, accumulatedImpulses, ref wsvA.Angular, ref wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref AngularAxisMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalAxisA, orientationA, out var jA);
            Symmetric3x3Wide.TransformWithoutOverlap(jA, inertiaA.InverseInertiaTensor, out var jIA);
            Vector3Wide.Dot(jA, jIA, out var contributionA);
            Symmetric3x3Wide.TransformWithoutOverlap(jA, inertiaB.InverseInertiaTensor, out var jIB);
            Vector3Wide.Dot(jA, jIB, out var contributionB);
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out var softnessImpulseScale, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * softnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = (prestep.TargetVelocity + Vector3Wide.Dot(wsvB.Angular, jA) - Vector3Wide.Dot(wsvA.Angular, jA)) * effectiveMassCFMScale / (contributionA + contributionB) - accumulatedImpulses * softnessImpulseScale;
            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            ApplyImpulse(jIA, jIB, csi, ref wsvA.Angular, ref wsvB.Angular);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref AngularAxisMotorPrestepData prestepData) { }
    }

    public class AngularAxisMotorTypeProcessor : TwoBodyTypeProcessor<AngularAxisMotorPrestepData, Vector<float>, AngularAxisMotorFunctions, AccessOnlyAngular, AccessOnlyAngularWithoutPose, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 41;
    }
}

