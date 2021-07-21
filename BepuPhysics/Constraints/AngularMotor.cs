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
    /// Constrains the relative angular velocity between two bodies to a target.
    /// </summary>
    public struct AngularMotor : ITwoBodyConstraintDescription<AngularMotor>
    {
        /// <summary>
        /// Target relative angular velocity between A and B, stored in A's local space. Target world space angular velocity of B is AngularVelocityA + TargetVelocityLocalA * OrientationA.
        /// </summary>
        public Vector3 TargetVelocityLocalA;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(AngularMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(AngularMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(TargetVelocityLocalA, ref target.TargetVelocityLocalA);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.TargetVelocityLocalA, out description.TargetVelocityLocalA);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct AngularMotorPrestepData
    {
        public Vector3Wide TargetVelocityLocalA;
        public MotorSettingsWide Settings;
    }

    public struct AngularMotorProjection
    {
        public Symmetric3x3Wide EffectiveMass;
        public Vector3Wide BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public Symmetric3x3Wide ImpulseToVelocityA;
        public Symmetric3x3Wide NegatedImpulseToVelocityB;
    }


    public struct AngularMotorFunctions : ITwoBodyConstraintFunctions<AngularMotorPrestepData, AngularMotorProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            float dt, float inverseDt, ref AngularMotorPrestepData prestep, out AngularMotorProjection projection)
        {
            projection.ImpulseToVelocityA = inertiaA.InverseInertiaTensor;
            projection.NegatedImpulseToVelocityB = inertiaB.InverseInertiaTensor;

            //Jacobians are just the identity matrix.
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);

            Symmetric3x3Wide.Add(projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, out var unsoftenedInverseEffectiveMass);
            Symmetric3x3Wide.Invert(unsoftenedInverseEffectiveMass, out var unsoftenedEffectiveMass);
            Symmetric3x3Wide.Scale(unsoftenedEffectiveMass, effectiveMassCFMScale, out projection.EffectiveMass);

            QuaternionWide.TransformWithoutOverlap(prestep.TargetVelocityLocalA, orientationA, out var biasVelocity);
            Symmetric3x3Wide.TransformWithoutOverlap(biasVelocity, projection.EffectiveMass, out projection.BiasImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref AngularMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            AngularServoFunctions.ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref AngularMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            AngularServoFunctions.Solve(ref velocityA, ref velocityB, projection.EffectiveMass, projection.SoftnessImpulseScale, projection.BiasImpulse,
                projection.MaximumImpulse, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, ref accumulatedImpulse);
        }

        public void WarmStart2(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in AngularMotorPrestepData prestep, in Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }

        public void Solve2(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, in AngularMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }
    }

    public class AngularMotorTypeProcessor : TwoBodyTypeProcessor<AngularMotorPrestepData, AngularMotorProjection, Vector3Wide, AngularMotorFunctions, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 30;
    }
}

