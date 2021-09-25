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
        /// Local offset from the center of body A to its attachment point.
        /// </summary>
        public Vector3 LocalOffsetA;
        /// <summary>
        /// Local offset from the center of body B to its attachment point.
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

    public struct BallSocketServoProjection
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector3Wide BiasVelocity;
        public Symmetric3x3Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public BodyInertiaWide InertiaA;
        public BodyInertiaWide InertiaB;
    }

    public struct BallSocketServoFunctions : ITwoBodyConstraintFunctions<BallSocketServoPrestepData, BallSocketServoProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            float dt, float inverseDt, ref BallSocketServoPrestepData prestep, out BallSocketServoProjection projection)
        {
            projection.InertiaA = inertiaA;
            projection.InertiaB = inertiaB;

            //Note that we must reconstruct the world offsets from the body orientations since we do not store world offsets.
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationA, out projection.OffsetA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out projection.OffsetB);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            BallSocketShared.ComputeEffectiveMass(inertiaA, inertiaB, ref projection.OffsetA, ref projection.OffsetB, ref effectiveMassCFMScale, out projection.EffectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(ab, projection.OffsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, projection.OffsetA, out var error);
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out projection.BiasVelocity, out projection.MaximumImpulse);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref BallSocketServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            BallSocketShared.ApplyImpulse(ref velocityA, ref velocityB, projection.OffsetA, projection.OffsetB, projection.InertiaA, projection.InertiaB, accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref BallSocketServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            BallSocketShared.Solve(ref velocityA, ref velocityB, ref projection.OffsetA, ref projection.OffsetB, ref projection.BiasVelocity, ref projection.EffectiveMass, ref projection.SoftnessImpulseScale, ref projection.MaximumImpulse, ref accumulatedImpulse, ref projection.InertiaA, ref projection.InertiaB);
        }

        public void WarmStart2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref BallSocketServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }

        public void Solve2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref BallSocketServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA, 
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in BodyVelocityWide wsvB,
            in Vector<float> dt, in Vector3Wide accumulatedImpulses, ref BallSocketServoPrestepData prestep)
        {
        }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket servo constraints.
    /// </summary>
    public class BallSocketServoTypeProcessor : TwoBodyTypeProcessor<BallSocketServoPrestepData, BallSocketServoProjection, Vector3Wide, BallSocketServoFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 53;
    }
}
