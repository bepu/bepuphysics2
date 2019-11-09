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
    /// Constrains the relative linear velocity between two bodies to a target.
    /// Conceptually, controls the relative velocity by a virtual lever arm attached to the center of A and leading to the anchor of B.
    /// </summary>
    public struct BallSocketMotor : ITwoBodyConstraintDescription<BallSocketMotor>
    {
        /// <summary>
        /// Offset from body B to its anchor.
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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return BallSocketMotorTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(BallSocketMotorTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(BallSocketMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<BallSocketMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(TargetVelocityLocalA, ref target.TargetVelocityLocalA);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out BallSocketMotor description)
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

    public struct BallSocketMotorProjection
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector3Wide BiasVelocity;
        public Symmetric3x3Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
    }

    public struct BallSocketMotorFunctions : IConstraintFunctions<BallSocketMotorPrestepData, BallSocketMotorProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref BallSocketMotorPrestepData prestep, out BallSocketMotorProjection projection)
        {
            bodies.GatherPose(ref bodyReferences, count, out var offsetFromACenterToBCenter, out var orientationA, out var orientationB);
            projection.InertiaA = inertiaA;
            projection.InertiaB = inertiaB;

            //The offset for A just goes directly to B's anchor.
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out projection.OffsetB);
            Vector3Wide.Add(offsetFromACenterToBCenter, projection.OffsetB, out projection.OffsetA);
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);
            BallSocketShared.ComputeEffectiveMass(ref inertiaA, ref inertiaB, ref projection.OffsetA, ref projection.OffsetB, ref effectiveMassCFMScale, out projection.EffectiveMass);

            QuaternionWide.Transform(prestep.TargetVelocityLocalA, orientationA, out projection.BiasVelocity);
            Vector3Wide.Negate(projection.BiasVelocity, out projection.BiasVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BallSocketMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            BallSocketShared.ApplyImpulse(ref velocityA, ref velocityB, ref projection.OffsetA, ref projection.OffsetB, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BallSocketMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            BallSocketShared.Solve(ref velocityA, ref velocityB, ref projection.OffsetA, ref projection.OffsetB, ref projection.BiasVelocity, ref projection.EffectiveMass, ref projection.SoftnessImpulseScale, ref projection.MaximumImpulse, ref accumulatedImpulse, ref projection.InertiaA, ref projection.InertiaB);
        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket motor constraints.
    /// </summary>
    public class BallSocketMotorTypeProcessor : TwoBodyTypeProcessor<BallSocketMotorPrestepData, BallSocketMotorProjection, Vector3Wide, BallSocketMotorFunctions>
    {
        public const int BatchTypeId = 52;
    }
}
