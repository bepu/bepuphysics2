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
    /// Constrains a single body to a target orientation.
    /// </summary>
    public struct OneBodyAngularServo : IOneBodyConstraintDescription<OneBodyAngularServo>
    {
        /// <summary>
        /// Target orientation of the constraint.
        /// </summary>
        public Quaternion TargetOrientation;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;
        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return OneBodyAngularServoTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(OneBodyAngularServoTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(TargetOrientation, nameof(OneBodyAngularServo), nameof(TargetOrientation));
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(OneBodyAngularServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyAngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(TargetOrientation, ref target.TargetOrientation);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyAngularServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<OneBodyAngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.ReadFirst(source.TargetOrientation, out description.TargetOrientation);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct OneBodyAngularServoPrestepData
    {
        public QuaternionWide TargetOrientation;
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct OneBodyAngularServoProjection
    {
        public Symmetric3x3Wide VelocityToImpulse;
        public Vector3Wide BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public Symmetric3x3Wide ImpulseToVelocity;
    }

    public struct OneBodyAngularServoFunctions : IOneBodyConstraintFunctions<OneBodyAngularServoPrestepData, OneBodyAngularServoProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref Vector<int> bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA,
            ref OneBodyAngularServoPrestepData prestep, out OneBodyAngularServoProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA);
            projection.ImpulseToVelocity = inertiaA.InverseInertiaTensor;

            //Jacobians are just the identity matrix.

            QuaternionWide.Conjugate(orientationA, out var inverseOrientation);
            QuaternionWide.ConcatenateWithoutOverlap(inverseOrientation, prestep.TargetOrientation, out var errorRotation);

            QuaternionWide.GetApproximateAxisAngleFromQuaternion(errorRotation, out var errorAxis, out var errorLength);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Symmetric3x3Wide.Invert(inertiaA.InverseInertiaTensor, out projection.VelocityToImpulse);
            Symmetric3x3Wide.Scale(projection.VelocityToImpulse, effectiveMassCFMScale, out projection.VelocityToImpulse);

            ServoSettingsWide.ComputeClampedBiasVelocity(errorAxis, errorLength, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var clampedBiasVelocity, out projection.MaximumImpulse);
            Symmetric3x3Wide.TransformWithoutOverlap(clampedBiasVelocity, projection.VelocityToImpulse, out projection.BiasImpulse);
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

    public class OneBodyAngularServoTypeProcessor : OneBodyTypeProcessor<OneBodyAngularServoPrestepData, OneBodyAngularServoProjection, Vector3Wide, OneBodyAngularServoFunctions>
    {
        public const int BatchTypeId = 42;
    }
}

