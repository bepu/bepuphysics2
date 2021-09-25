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
    /// Constrains points on two bodies to move relative to each other along a direction.
    /// </summary>
    public struct LinearAxisMotor : ITwoBodyConstraintDescription<LinearAxisMotor>
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
        /// Direction of the motorized axis in the local space of body A.
        /// </summary>
        public Vector3 LocalAxis;
        /// <summary>
        /// Target relative velocity along the world axis between A and B's anchor points.
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
                return LinearAxisMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(LinearAxisMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalAxis, nameof(LinearAxisMotor), nameof(LocalAxis));
            ConstraintChecker.AssertValid(Settings, nameof(LinearAxisMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<LinearAxisMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(LocalAxis, ref target.LocalPlaneNormal);
            GatherScatter.GetFirst(ref target.TargetVelocity) = TargetVelocity;
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out LinearAxisMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<LinearAxisMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            Vector3Wide.ReadFirst(source.LocalPlaneNormal, out description.LocalAxis);
            description.TargetVelocity = GatherScatter.GetFirst(ref source.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct LinearAxisMotorPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public Vector3Wide LocalPlaneNormal;
        public Vector<float> TargetVelocity;
        public MotorSettingsWide Settings;
    }
    
    public struct LinearAxisMotorFunctions : ITwoBodyConstraintFunctions<LinearAxisMotorPrestepData, LinearAxisServoProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            float dt, float inverseDt, ref LinearAxisMotorPrestepData prestep, out LinearAxisServoProjection projection)
        {
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);
            var modifier = new LinearAxisServoFunctions.NoChangeModifier();
            LinearAxisServoFunctions.ComputeTransforms(ref modifier, prestep.LocalOffsetA, prestep.LocalOffsetB, prestep.LocalPlaneNormal, orientationA, inertiaA, ab, orientationB, inertiaB, effectiveMassCFMScale,
                out _, out _, out _, out var effectiveMass,
                out projection.LinearVelocityToImpulseA, out projection.AngularVelocityToImpulseA, out projection.AngularVelocityToImpulseB,
                out projection.LinearImpulseToVelocityA, out projection.AngularImpulseToVelocityA, out projection.NegatedLinearImpulseToVelocityB, out projection.AngularImpulseToVelocityB);
                        
            projection.BiasImpulse = -prestep.TargetVelocity * effectiveMass;
        }
                       
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref LinearAxisServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            LinearAxisServoFunctions.ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.NegatedLinearImpulseToVelocityB, projection.AngularImpulseToVelocityB,
                ref accumulatedImpulse);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref LinearAxisServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            LinearAxisServoFunctions.ComputeCorrectiveImpulse(ref velocityA, ref velocityB, projection.LinearVelocityToImpulseA, projection.AngularVelocityToImpulseA, projection.AngularVelocityToImpulseB,
                projection.BiasImpulse, projection.SoftnessImpulseScale, accumulatedImpulse, out var csi);
            ServoSettingsWide.ClampImpulse(projection.MaximumImpulse, ref accumulatedImpulse, ref csi);
            LinearAxisServoFunctions.ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.NegatedLinearImpulseToVelocityB, projection.AngularImpulseToVelocityB,
                ref csi);
        }

        public void WarmStart2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref LinearAxisMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }

        public void Solve2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref LinearAxisMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA, 
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in BodyVelocityWide wsvB,
            in Vector<float> dt, in Vector<float> accumulatedImpulses, ref LinearAxisMotorPrestepData prestep)
        {
        }
    }

    public class LinearAxisMotorTypeProcessor : TwoBodyTypeProcessor<LinearAxisMotorPrestepData, LinearAxisServoProjection, Vector<float>, LinearAxisMotorFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 39;
    }
}
