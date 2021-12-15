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

    public struct LinearAxisMotorFunctions : ITwoBodyConstraintFunctions<LinearAxisMotorPrestepData, Vector<float>>
    {
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref LinearAxisMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            LinearAxisServoFunctions.ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalPlaneNormal, prestep.LocalOffsetA, prestep.LocalOffsetB, out _, out var normal, out var angularJA, out var angularJB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJA, inertiaA.InverseInertiaTensor, out var angularImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJB, inertiaB.InverseInertiaTensor, out var angularImpulseToVelocityB);
            LinearAxisServoFunctions.ApplyImpulse(normal, angularImpulseToVelocityA, angularImpulseToVelocityB, inertiaA, inertiaB, accumulatedImpulses, ref wsvA, ref wsvB);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref LinearAxisMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            LinearAxisServoFunctions.ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalPlaneNormal, prestep.LocalOffsetA, prestep.LocalOffsetB, out _, out var normal, out var angularJA, out var angularJB);
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out var softnessImpulseScale, out var maximumImpulse);
            LinearAxisServoFunctions.ComputeEffectiveMass(angularJA, angularJB, inertiaA, inertiaB, effectiveMassCFMScale, out var angularImpulseToVelocityA, out var angularImpulseToVelocityB, out var effectiveMass);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csv = Vector3Wide.Dot(wsvA.Linear - wsvB.Linear, normal) + Vector3Wide.Dot(wsvA.Angular, angularJA) + Vector3Wide.Dot(wsvB.Angular, angularJB);

            var csi = effectiveMass * (-prestep.TargetVelocity - csv) - accumulatedImpulses * softnessImpulseScale;

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            LinearAxisServoFunctions.ApplyImpulse(normal, angularImpulseToVelocityA, angularImpulseToVelocityB, inertiaA, inertiaB, csi, ref wsvA, ref wsvB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref LinearAxisMotorPrestepData prestepData) { }
    }

    public class LinearAxisMotorTypeProcessor : TwoBodyTypeProcessor<LinearAxisMotorPrestepData, Vector<float>, LinearAxisMotorFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 39;
    }
}
