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
    /// Constrains a point on a body to have a target linear velocity.
    /// </summary>
    public struct OneBodyLinearMotor : IOneBodyConstraintDescription<OneBodyLinearMotor>
    {
        /// <summary>
        /// Offset to the attachment point in the local space of the body.
        /// </summary>
        public Vector3 LocalOffset;
        /// <summary>
        /// Target velocity of the attachment point.
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
                return OneBodyLinearMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(OneBodyLinearMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(OneBodyLinearMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyLinearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffset, ref target.LocalOffset);
            Vector3Wide.WriteFirst(TargetVelocity, ref target.TargetVelocity);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyLinearMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<OneBodyLinearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffset, out description.LocalOffset);
            Vector3Wide.ReadFirst(source.TargetVelocity, out description.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct OneBodyLinearMotorPrestepData
    {
        public Vector3Wide LocalOffset;
        public Vector3Wide TargetVelocity;
        public MotorSettingsWide Settings;
    }

    public struct OneBodyLinearMotorFunctions : IOneBodyConstraintFunctions<OneBodyLinearMotorPrestepData, OneBodyLinearServoProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA,
            float dt, float inverseDt, ref OneBodyLinearMotorPrestepData prestep, out OneBodyLinearServoProjection projection)
        {
            //TODO: Note that this grabs a world position. That poses a problem for different position representations.
            projection.Inertia = inertiaA;

            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);

            OneBodyLinearServoFunctions.ComputeTransforms(prestep.LocalOffset, orientationA, effectiveMassCFMScale, inertiaA, out projection.Offset, out projection.EffectiveMass);
            projection.BiasVelocity = prestep.TargetVelocity;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocityWide velocityA, ref OneBodyLinearServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            OneBodyLinearServoFunctions.ApplyImpulse(ref velocityA, projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocityWide velocityA, ref OneBodyLinearServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            OneBodyLinearServoFunctions.SharedSolve(ref velocityA, projection, ref accumulatedImpulse);
        }

        public void WarmStart2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in OneBodyLinearMotorPrestepData prestep, in Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            throw new NotImplementedException();
        }

        public void Solve2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, in OneBodyLinearMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA,
            in Vector<float> dt, in Vector3Wide accumulatedImpulses, ref OneBodyLinearMotorPrestepData prestep)
        {
        }
    }

    public class OneBodyLinearMotorTypeProcessor : OneBodyTypeProcessor<OneBodyLinearMotorPrestepData, OneBodyLinearServoProjection, Vector3Wide, OneBodyLinearMotorFunctions, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 45;
    }
}
