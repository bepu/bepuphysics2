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
    /// Constrains the twist velocity between two bodies to a target.
    /// </summary>
    public struct TwistMotor : ITwoBodyConstraintDescription<TwistMotor>
    {
        /// <summary>
        /// Local twist axis attached to body A.
        /// </summary>
        public Vector3 LocalAxisA;
        /// <summary>
        /// Local twist axis attached to body B.
        /// </summary>
        public Vector3 LocalAxisB;
        /// <summary>
        /// Goal relative twist velocity around the body axes.
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
                return TwistMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(TwistMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalAxisA, nameof(TwistMotor), nameof(LocalAxisA));
            ConstraintChecker.AssertUnitLength(LocalAxisB, nameof(TwistMotor), nameof(LocalAxisB));
            ConstraintChecker.AssertValid(Settings, nameof(TwistMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TwistMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalAxisA, ref target.LocalAxisA);
            Vector3Wide.WriteFirst(LocalAxisB, ref target.LocalAxisB);
            GetFirst(ref target.TargetVelocity) = TargetVelocity;
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out TwistMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<TwistMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalAxisA, out description.LocalAxisA);
            Vector3Wide.ReadFirst(source.LocalAxisB, out description.LocalAxisB);
            description.TargetVelocity = GetFirst(ref source.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct TwistMotorPrestepData
    {
        public Vector3Wide LocalAxisA;
        public Vector3Wide LocalAxisB;
        public Vector<float> TargetVelocity;
        public MotorSettingsWide Settings;
    }

    public struct TwistMotorFunctions : ITwoBodyConstraintFunctions<TwistMotorPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobian(in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector3Wide localAxisA, in Vector3Wide localAxisB, out Vector3Wide jacobianA)
        {
            //We don't need any measurement basis in a velocity motor, so the prestep data needs only the axes.
            QuaternionWide.TransformWithoutOverlap(localAxisA, orientationA, out var axisA);
            QuaternionWide.TransformWithoutOverlap(localAxisB, orientationB, out var axisB);
            Vector3Wide.Add(axisA, axisB, out jacobianA);
            Vector3Wide.Length(jacobianA, out var length);
            Vector3Wide.Scale(jacobianA, Vector<float>.One / length, out jacobianA);
            Vector3Wide.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-10f)), axisA, jacobianA, out jacobianA);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref TwistMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(orientationA, orientationB, prestep.LocalAxisA, prestep.LocalAxisB, out var jacobianA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            TwistServoFunctions.ApplyImpulse(ref wsvA.Angular, ref wsvB.Angular, impulseToVelocityA, negatedImpulseToVelocityB, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref TwistMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(orientationA, orientationB, prestep.LocalAxisA, prestep.LocalAxisB, out var jacobianA);

            TwistServoFunctions.ComputeEffectiveMassContributions(inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, jacobianA,
                out var impulseToVelocityA, out var negatedImpulseToVelocityB, out var unsoftenedInverseEffectiveMass);

            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out var softnessImpulseScale, out var maximumImpulse);
            var effectiveMass = effectiveMassCFMScale / unsoftenedInverseEffectiveMass;
            Vector3Wide.Scale(jacobianA, effectiveMass, out var velocityToImpulseA);

            var biasImpulse = prestep.TargetVelocity * effectiveMass;

            Vector3Wide.Subtract(wsvA.Angular, wsvB.Angular, out var netVelocity);
            Vector3Wide.Dot(netVelocity, velocityToImpulseA, out var csiVelocityComponent);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = biasImpulse - accumulatedImpulses * softnessImpulseScale - csiVelocityComponent;
            var previousAccumulatedImpulse = accumulatedImpulses;
            accumulatedImpulses = Vector.Max(Vector.Min(accumulatedImpulses + csi, maximumImpulse), -maximumImpulse);
            csi = accumulatedImpulses - previousAccumulatedImpulse;

            TwistServoFunctions.ApplyImpulse(ref wsvA.Angular, ref wsvB.Angular, impulseToVelocityA, negatedImpulseToVelocityB, csi);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref TwistMotorPrestepData prestepData) { }
    }

    public class TwistMotorTypeProcessor : TwoBodyTypeProcessor<TwistMotorPrestepData, Vector<float>, TwistMotorFunctions, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 28;
    }
}

