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
    /// Constrains the angular velocity of one body to the target.
    /// </summary>
    public struct OneBodyAngularMotor : IOneBodyConstraintDescription<OneBodyAngularMotor>
    {
        /// <summary>
        /// Target angular velocity.
        /// </summary>
        public Vector3 TargetVelocity;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return OneBodyAngularMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(OneBodyAngularMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(OneBodyAngularMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyAngularMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(TargetVelocity, ref target.TargetVelocity);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyAngularMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<OneBodyAngularMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.TargetVelocity, out description.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct OneBodyAngularMotorPrestepData
    {
        public Vector3Wide TargetVelocity;
        public MotorSettingsWide Settings;
    }

    public struct OneBodyAngularMotorFunctions : IOneBodyConstraintFunctions<OneBodyAngularMotorPrestepData, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularVelocity, in Symmetric3x3Wide impulseToVelocity, in Vector3Wide csi)
        {
            Symmetric3x3Wide.TransformWithoutOverlap(csi, impulseToVelocity, out var velocityChange);
            Vector3Wide.Add(angularVelocity, velocityChange, out angularVelocity);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref OneBodyAngularMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            ApplyImpulse(ref wsvA.Angular, inertiaA.InverseInertiaTensor, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref OneBodyAngularMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            //Jacobians are just the identity matrix.
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out var softnessImpulseScale, out var maximumImpulse);
            Symmetric3x3Wide.Invert(inertiaA.InverseInertiaTensor, out var unsoftenedEffectiveMass);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiaAngular;
            Symmetric3x3Wide.TransformWithoutOverlap(prestep.TargetVelocity - wsvA.Angular, unsoftenedEffectiveMass, out var csi);
            csi = csi * effectiveMassCFMScale - accumulatedImpulses * softnessImpulseScale;

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);

            ApplyImpulse(ref wsvA.Angular, inertiaA.InverseInertiaTensor, csi);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, ref OneBodyAngularMotorPrestepData prestepData) { }
    }

    public class OneBodyAngularMotorTypeProcessor : OneBodyTypeProcessor<OneBodyAngularMotorPrestepData, Vector3Wide, OneBodyAngularMotorFunctions, AccessOnlyAngularWithoutPose, AccessOnlyAngular>
    {
        public const int BatchTypeId = 43;
    }
}

