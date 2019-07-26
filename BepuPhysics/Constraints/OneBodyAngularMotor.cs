using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Constrains the angular velocity of one body to the target.
    /// </summary>
    public struct OneBodyAngularMotor : IOneBodyConstraintDescription<OneBodyAngularMotor>
    {
        /// <summary>
        /// Target angular velocity.
        /// </summary>
        public Vector3 TargetVelocity;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return OneBodyAngularMotorTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(OneBodyAngularMotorTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(OneBodyAngularMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyAngularMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(TargetVelocity, ref target.TargetVelocity);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyAngularMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<OneBodyAngularMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.TargetVelocity, out description.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct OneBodyAngularMotorPrestepData
    {
        public Vector3Wide TargetVelocity;
        public MotorSettingsWide Settings;
    }

    public struct OneBodyAngularMotorFunctions : IOneBodyConstraintFunctions<OneBodyAngularMotorPrestepData, OneBodyAngularServoProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref Vector<int> bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA,
            ref OneBodyAngularMotorPrestepData prestep, out OneBodyAngularServoProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA);
            projection.ImpulseToVelocity = inertiaA.InverseInertiaTensor;

            //Jacobians are just the identity matrix.
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);
            Symmetric3x3Wide.Invert(inertiaA.InverseInertiaTensor, out projection.VelocityToImpulse);
            Symmetric3x3Wide.Scale(projection.VelocityToImpulse, effectiveMassCFMScale, out projection.VelocityToImpulse);

            Symmetric3x3Wide.TransformWithoutOverlap(prestep.TargetVelocity, projection.VelocityToImpulse, out projection.BiasImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularVelocity, in Symmetric3x3Wide impulseToVelocity, in Vector3Wide csi)
        {
            Symmetric3x3Wide.TransformWithoutOverlap(csi, impulseToVelocity, out var velocityChange);
            Vector3Wide.Add(angularVelocity, velocityChange, out angularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref OneBodyAngularServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA.Angular, projection.ImpulseToVelocity, accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref BodyVelocities velocityA,
            in Symmetric3x3Wide effectiveMass, in Vector<float> softnessImpulseScale, in Vector3Wide biasImpulse, in Vector<float> maximumImpulse,
            in Symmetric3x3Wide impulseToVelocityA, ref Vector3Wide accumulatedImpulse)
        {
            //Jacobians are just I.
            Symmetric3x3Wide.TransformWithoutOverlap(velocityA.Angular, effectiveMass, out var csiVelocityComponent);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiaAngular;
            Vector3Wide.Scale(accumulatedImpulse, softnessImpulseScale, out var softnessComponent);
            Vector3Wide.Subtract(biasImpulse, softnessComponent, out var csi);
            Vector3Wide.Subtract(csi, csiVelocityComponent, out csi);

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulse, ref csi);            

            ApplyImpulse(ref velocityA.Angular, impulseToVelocityA, csi);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref OneBodyAngularServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            Solve(ref velocityA, projection.VelocityToImpulse, projection.SoftnessImpulseScale, projection.BiasImpulse,
                projection.MaximumImpulse, projection.ImpulseToVelocity, ref accumulatedImpulse);
        }
    }

    public class OneBodyAngularMotorTypeProcessor : OneBodyTypeProcessor<OneBodyAngularMotorPrestepData, OneBodyAngularServoProjection, Vector3Wide, OneBodyAngularMotorFunctions>
    {
        public const int BatchTypeId = 43;
    }
}

