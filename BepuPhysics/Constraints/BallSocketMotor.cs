using BepuPhysics;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
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
    /// Controls the relative linear velocity from the center of body A to an attachment point on body B.
    /// </summary>
    public struct BallSocketMotor : ITwoBodyConstraintDescription<BallSocketMotor>
    {
        /// <summary>
        /// Offset from body B to its attachment in B's local space.
        /// </summary>
        public Vector3 LocalOffsetB;
        /// <summary>
        /// Target relative linear velocity between A and B, stored in A's local space. Target world space linear velocity of B is LinearVelocityA + TargetVelocityLocalA * OrientationA.
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
                return BallSocketMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(BallSocketMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(BallSocketMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<BallSocketMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(TargetVelocityLocalA, ref target.TargetVelocityLocalA);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out BallSocketMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<BallSocketMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            Vector3Wide.ReadFirst(source.TargetVelocityLocalA, out description.TargetVelocityLocalA);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct BallSocketMotorPrestepData
    {
        public Vector3Wide LocalOffsetB;
        public Vector3Wide TargetVelocityLocalA;
        public MotorSettingsWide Settings;
    }

    public struct BallSocketMotorFunctions : ITwoBodyConstraintFunctions<BallSocketMotorPrestepData, Vector3Wide>
    {
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref BallSocketMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out var targetOffsetB);
            BallSocketShared.ApplyImpulse(ref wsvA, ref wsvB, (positionB - positionA) + targetOffsetB, targetOffsetB, inertiaA, inertiaB, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref BallSocketMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out var targetOffsetB);
            var offsetA = (positionB - positionA) + targetOffsetB;

            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out var softnessImpulseScale, out var maximumImpulse);
            BallSocketShared.ComputeEffectiveMass(inertiaA, inertiaB, offsetA, targetOffsetB, effectiveMassCFMScale, out var effectiveMass);

            QuaternionWide.Transform(prestep.TargetVelocityLocalA, orientationA, out var biasVelocity);
            Vector3Wide.Negate(biasVelocity, out biasVelocity);

            BallSocketShared.Solve(ref wsvA, ref wsvB, offsetA, targetOffsetB, biasVelocity, effectiveMass, softnessImpulseScale, maximumImpulse, ref accumulatedImpulses, inertiaA, inertiaB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref BallSocketMotorPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket motor constraints.
    /// </summary>
    public class BallSocketMotorTypeProcessor : TwoBodyTypeProcessor<BallSocketMotorPrestepData, Vector3Wide, BallSocketMotorFunctions, AccessNoOrientation, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 52;
    }
}
