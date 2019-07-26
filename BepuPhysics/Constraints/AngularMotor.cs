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
        /// Target relative angular velocity between A and B, stored in A's local space. Target world space angular velocity of B is TargetVelocityLocalA * OrientationA.
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
                return AngularMotorTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(AngularMotorTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(AngularMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(TargetVelocityLocalA, ref target.TargetVelocityLocalA);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularMotor description)
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


    public struct AngularMotorFunctions : IConstraintFunctions<AngularMotorPrestepData, AngularMotorProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref AngularMotorPrestepData prestep, out AngularMotorProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA, out var orientationB);
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
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            AngularServoFunctions.ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            AngularServoFunctions.Solve(ref velocityA, ref velocityB, projection.EffectiveMass, projection.SoftnessImpulseScale, projection.BiasImpulse,
                projection.MaximumImpulse, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, ref accumulatedImpulse);
        }

    }

    public class AngularMotorTypeProcessor : TwoBodyTypeProcessor<AngularMotorPrestepData, AngularMotorProjection, Vector3Wide, AngularMotorFunctions>
    {
        public const int BatchTypeId = 30;
    }
}

