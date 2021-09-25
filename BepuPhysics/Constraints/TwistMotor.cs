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

    public struct TwistMotorProjection
    {
        public Vector3Wide VelocityToImpulseA;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> BiasImpulse;
        public Vector<float> MaximumImpulse;
        public Vector3Wide ImpulseToVelocityA;
        public Vector3Wide NegatedImpulseToVelocityB;
    }


    public struct TwistMotorFunctions : ITwoBodyConstraintFunctions<TwistMotorPrestepData, TwistMotorProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            float dt, float inverseDt, ref TwistMotorPrestepData prestep, out TwistMotorProjection projection)
        {
            Unsafe.SkipInit(out projection);
            //We don't need any measurement basis in a velocity motor, so the prestep data needs only the axes.
            QuaternionWide.TransformWithoutOverlap(prestep.LocalAxisA, orientationA, out var axisA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalAxisB, orientationB, out var axisB);
            Vector3Wide.Add(axisA, axisB, out var jacobianA);
            Vector3Wide.Length(jacobianA, out var length);
            Vector3Wide.Scale(jacobianA, Vector<float>.One / length, out jacobianA);
            Vector3Wide.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-10f)), axisA, jacobianA, out jacobianA);

            TwistServoFunctions.ComputeEffectiveMassContributions(inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, jacobianA,
                ref projection.ImpulseToVelocityA, ref projection.NegatedImpulseToVelocityB, out var unsoftenedInverseEffectiveMass);

            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);
            var effectiveMass = effectiveMassCFMScale / unsoftenedInverseEffectiveMass;
            Vector3Wide.Scale(jacobianA, effectiveMass, out projection.VelocityToImpulseA);
            
            projection.BiasImpulse = prestep.TargetVelocity * effectiveMass;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref TwistMotorProjection projection, ref Vector<float> accumulatedImpulse)
        {
            TwistServoFunctions.ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref TwistMotorProjection projection, ref Vector<float> accumulatedImpulse)
        {
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var netVelocity);
            Vector3Wide.Dot(netVelocity, projection.VelocityToImpulseA, out var csiVelocityComponent);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiVelocityComponent;
            var previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector.Min(accumulatedImpulse + csi, projection.MaximumImpulse), -projection.MaximumImpulse);
            csi = accumulatedImpulse - previousAccumulatedImpulse;

            TwistServoFunctions.ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, csi);
        }

        public void WarmStart2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref TwistMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }

        public void Solve2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref TwistMotorPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA, 
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in BodyVelocityWide wsvB,
            in Vector<float> dt, in Vector<float> accumulatedImpulses, ref TwistMotorPrestepData prestep)
        {
        }
    }

    public class TwistMotorTypeProcessor : TwoBodyTypeProcessor<TwistMotorPrestepData, TwistMotorProjection, Vector<float>, TwistMotorFunctions, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 28;
    }
}

