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
    /// Constrains a point on one body to a point on another body.
    /// Provides speed and force configuration that the BallSocket joint does not.
    /// </summary>
    public struct BallSocketServo : ITwoBodyConstraintDescription<BallSocketServo>
    {
        /// <summary>
        /// Offset from the center of body A to its attachment in A's local space.
        /// </summary>
        public Vector3 LocalOffsetA;
        /// <summary>
        /// Offset from the center of body B to its attachment in B's local space.
        /// </summary>
        public Vector3 LocalOffsetB;
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
                return BallSocketServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(BallSocketServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(BallSocketServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<BallSocketServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out BallSocketServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<BallSocketServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct BallSocketServoPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct BallSocketServoFunctions : ITwoBodyConstraintFunctions<BallSocketServoPrestepData, Vector3Wide>
    {
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref BallSocketServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationA, out var offsetA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out var offsetB);
            BallSocketShared.ApplyImpulse(ref wsvA, ref wsvB, offsetA, offsetB, inertiaA, inertiaB, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref BallSocketServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationA, out var offsetA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out var offsetB);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            BallSocketShared.ComputeEffectiveMass(inertiaA, inertiaB, offsetA, offsetB, effectiveMassCFMScale, out var effectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            var ab = positionB - positionA;
            Vector3Wide.Add(ab, offsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, offsetA, out var error);
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var biasVelocity, out var maximumImpulse);

            BallSocketShared.Solve(ref wsvA, ref wsvB, offsetA, offsetB, biasVelocity, effectiveMass, softnessImpulseScale, maximumImpulse, ref accumulatedImpulses, inertiaA, inertiaB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref BallSocketServoPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket servo constraints.
    /// </summary>
    public class BallSocketServoTypeProcessor : TwoBodyTypeProcessor<BallSocketServoPrestepData, Vector3Wide, BallSocketServoFunctions, AccessNoPosition, AccessNoPosition, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 53;
    }
}
