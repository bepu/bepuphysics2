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
    /// Constrains a point on a body to have a target linear velocity.
    /// </summary>
    public struct OneBodyLinearMotor : IOneBodyConstraintDescription<OneBodyLinearMotor>
    {
        /// <summary>
        /// Offset to the attachment point in the local space of the body.
        /// </summary>
        public Vector3 LocalOffset;
        /// <summary>
        /// Target velocity of the attachment point.
        /// </summary>
        public Vector3 TargetVelocity;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return OneBodyLinearMotorTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(OneBodyLinearMotorTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(OneBodyLinearMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyLinearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffset, ref target.LocalOffset);
            Vector3Wide.WriteFirst(TargetVelocity, ref target.TargetVelocity);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyLinearMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<OneBodyLinearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffset, out description.LocalOffset);
            Vector3Wide.ReadFirst(source.TargetVelocity, out description.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct OneBodyLinearMotorPrestepData
    {
        public Vector3Wide LocalOffset;
        public Vector3Wide TargetVelocity;
        public MotorSettingsWide Settings;
    }

    public struct OneBodyLinearMotorFunctions : IOneBodyConstraintFunctions<OneBodyLinearMotorPrestepData, Vector3Wide>
    {
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref OneBodyLinearMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffset, orientationA, out var offset);
            OneBodyLinearServoFunctions.ApplyImpulse(offset, inertiaA, ref wsvA, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref OneBodyLinearMotorPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffset, orientationA, out var offset);
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out var softnessImpulseScale, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular);
            var csv = prestep.TargetVelocity - Vector3Wide.Cross(wsvA.Angular, offset) - wsvA.Linear;

            //The grabber is roughly equivalent to a ball socket joint with a nonzero goal (and only one body).
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offset, inertiaA.InverseInertiaTensor, out var inverseEffectiveMass);

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            inverseEffectiveMass.XX += inertiaA.InverseMass;
            inverseEffectiveMass.YY += inertiaA.InverseMass;
            inverseEffectiveMass.ZZ += inertiaA.InverseMass;
            Symmetric3x3Wide.Invert(inverseEffectiveMass, out var effectiveMass);
            Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
            csi = csi * effectiveMassCFMScale - accumulatedImpulses * softnessImpulseScale;

            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            OneBodyLinearServoFunctions.ApplyImpulse(offset, inertiaA, ref wsvA, csi);
        }
        
        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, ref OneBodyLinearMotorPrestepData prestepData) { }
    }
    public class OneBodyLinearMotorTypeProcessor : OneBodyTypeProcessor<OneBodyLinearMotorPrestepData, Vector3Wide, OneBodyLinearMotorFunctions, AccessNoPosition, AccessNoPosition>
    {
        public const int BatchTypeId = 45;
    }
}
