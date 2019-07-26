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
    /// Constrains two bodies to have a target relative rotation.
    /// </summary>
    public struct AngularServo : ITwoBodyConstraintDescription<AngularServo>
    {
        /// <summary>
        /// The target relative rotation from body A to body B in body A's local space. The constraint tries to maintain OrientationB = TargetRelativeRotationLocalA * OrientationA.
        /// </summary>
        public Quaternion TargetRelativeRotationLocalA;
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
                return AngularServoTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(AngularServoTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(TargetRelativeRotationLocalA, nameof(AngularServo), nameof(TargetRelativeRotationLocalA));
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(AngularServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(TargetRelativeRotationLocalA, ref target.TargetRelativeRotationLocalA);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.ReadFirst(source.TargetRelativeRotationLocalA, out description.TargetRelativeRotationLocalA);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct AngularServoPrestepData
    {
        public QuaternionWide TargetRelativeRotationLocalA;
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct AngularServoProjection
    {
        public Symmetric3x3Wide EffectiveMass;
        public Vector3Wide BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public Symmetric3x3Wide ImpulseToVelocityA;
        public Symmetric3x3Wide NegatedImpulseToVelocityB;
    }


    public struct AngularServoFunctions : IConstraintFunctions<AngularServoPrestepData, AngularServoProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref AngularServoPrestepData prestep, out AngularServoProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA, out var orientationB);
            projection.ImpulseToVelocityA = inertiaA.InverseInertiaTensor;
            projection.NegatedImpulseToVelocityB = inertiaB.InverseInertiaTensor;

            //Jacobians are just the identity matrix.

            QuaternionWide.ConcatenateWithoutOverlap(prestep.TargetRelativeRotationLocalA, orientationA, out var targetOrientationB);
            QuaternionWide.Conjugate(targetOrientationB, out var inverseTarget);
            QuaternionWide.ConcatenateWithoutOverlap(inverseTarget, orientationB, out var errorRotation);

            QuaternionWide.GetApproximateAxisAngleFromQuaternion(errorRotation, out var errorAxis, out var errorLength);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Symmetric3x3Wide.Add(projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, out var unsoftenedInverseEffectiveMass);
            Symmetric3x3Wide.Invert(unsoftenedInverseEffectiveMass, out var unsoftenedEffectiveMass);
            Symmetric3x3Wide.Scale(unsoftenedEffectiveMass, effectiveMassCFMScale, out projection.EffectiveMass);

            ServoSettingsWide.ComputeClampedBiasVelocity(errorAxis, errorLength, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var clampedBiasVelocity, out projection.MaximumImpulse);
            Symmetric3x3Wide.TransformWithoutOverlap(clampedBiasVelocity, projection.EffectiveMass, out projection.BiasImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, in Symmetric3x3Wide impulseToVelocityA, in Symmetric3x3Wide negatedImpulseToVelocityB, in Vector3Wide csi)
        {
            Symmetric3x3Wide.TransformWithoutOverlap(csi, impulseToVelocityA, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(csi, negatedImpulseToVelocityB, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB,
            in Symmetric3x3Wide effectiveMass, in Vector<float> softnessImpulseScale, in Vector3Wide biasImpulse, in Vector<float> maximumImpulse,
            in Symmetric3x3Wide impulseToVelocityA, in Symmetric3x3Wide negatedImpulseToVelocityB, ref Vector3Wide accumulatedImpulse)
        {
            //Jacobians are just I and -I.
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var csv);
            Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out var csiVelocityComponent);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Scale(accumulatedImpulse, softnessImpulseScale, out var softnessComponent);
            Vector3Wide.Subtract(biasImpulse, softnessComponent, out var csi);
            Vector3Wide.Subtract(csi, csiVelocityComponent, out csi);

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulse, ref csi);

            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, impulseToVelocityA, negatedImpulseToVelocityB, csi);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            Solve(ref velocityA, ref velocityB, projection.EffectiveMass, projection.SoftnessImpulseScale, projection.BiasImpulse,
                projection.MaximumImpulse, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, ref accumulatedImpulse);
        }

    }

    public class AngularServoTypeProcessor : TwoBodyTypeProcessor<AngularServoPrestepData, AngularServoProjection, Vector3Wide, AngularServoFunctions>
    {
        public const int BatchTypeId = 29;
    }
}

