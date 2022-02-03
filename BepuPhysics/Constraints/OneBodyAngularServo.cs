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
    /// Constrains a single body to a target orientation.
    /// </summary>
    public struct OneBodyAngularServo : IOneBodyConstraintDescription<OneBodyAngularServo>
    {
        /// <summary>
        /// Target orientation of the constraint.
        /// </summary>
        public Quaternion TargetOrientation;
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
                return OneBodyAngularServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(OneBodyAngularServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(TargetOrientation, nameof(OneBodyAngularServo), nameof(TargetOrientation));
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(OneBodyAngularServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyAngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(TargetOrientation, ref target.TargetOrientation);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyAngularServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<OneBodyAngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.ReadFirst(source.TargetOrientation, out description.TargetOrientation);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct OneBodyAngularServoPrestepData
    {
        public QuaternionWide TargetOrientation;
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct OneBodyAngularServoFunctions : IOneBodyConstraintFunctions<OneBodyAngularServoPrestepData, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Symmetric3x3Wide inverseInertia, in Vector3Wide csi, ref Vector3Wide angularVelocity)
        {
            Symmetric3x3Wide.TransformWithoutOverlap(csi, inverseInertia, out var velocityChange);
            Vector3Wide.Add(angularVelocity, velocityChange, out angularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref OneBodyAngularServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            ApplyImpulse(inertiaA.InverseInertiaTensor, accumulatedImpulses, ref wsvA.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref OneBodyAngularServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            //Jacobians are just the identity matrix.
            QuaternionWide.Conjugate(orientationA, out var inverseOrientation);
            QuaternionWide.ConcatenateWithoutOverlap(inverseOrientation, prestep.TargetOrientation, out var errorRotation);
            QuaternionWide.GetAxisAngleFromQuaternion(errorRotation, out var errorAxis, out var errorLength);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            Symmetric3x3Wide.Invert(inertiaA.InverseInertiaTensor, out var effectiveMass);

            ServoSettingsWide.ComputeClampedBiasVelocity(errorAxis, errorLength, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var clampedBiasVelocity, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiaAngular;
            var csv = clampedBiasVelocity - wsvA.Angular;
            Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
            csi = csi * effectiveMassCFMScale - accumulatedImpulses * softnessImpulseScale;

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            ApplyImpulse(inertiaA.InverseInertiaTensor, csi, ref wsvA.Angular);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, ref OneBodyAngularServoPrestepData prestepData) { }
    }

    public class OneBodyAngularServoTypeProcessor : OneBodyTypeProcessor<OneBodyAngularServoPrestepData, Vector3Wide, OneBodyAngularServoFunctions, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 42;
    }
}

