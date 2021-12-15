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
    /// Constrains a point on a body to a target location.
    /// </summary>
    public struct OneBodyLinearServo : IOneBodyConstraintDescription<OneBodyLinearServo>
    {
        /// <summary>
        /// Offset to the attachment point in the local space of the body.
        /// </summary>
        public Vector3 LocalOffset;
        /// <summary>
        /// Target position.
        /// </summary>
        public Vector3 Target;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;
        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return OneBodyLinearServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(OneBodyLinearServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(OneBodyLinearServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyLinearServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffset, ref target.LocalOffset);
            Vector3Wide.WriteFirst(Target, ref target.Target);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyLinearServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<OneBodyLinearServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffset, out description.LocalOffset);
            Vector3Wide.ReadFirst(source.Target, out description.Target);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct OneBodyLinearServoPrestepData
    {
        public Vector3Wide LocalOffset;
        //TODO: This depends upon position being represented as a 32 bit floating point number.
        //That's a little problematic in the long run. You could avoid this dependency by representing the target as a relative target, but that is quite a bit more difficult to use-
        //you would have to update the target location every single time step, or else it would just continually accelerate.
        public Vector3Wide Target;
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct OneBodyLinearServoFunctions : IOneBodyConstraintFunctions<OneBodyLinearServoPrestepData, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeTransforms(in Vector3Wide localOffset, in QuaternionWide orientation, in Vector<float> effectiveMassCFMScale,
            in BodyInertiaWide inertia, out Vector3Wide offset, out Symmetric3x3Wide effectiveMass)
        {
            //The grabber is roughly equivalent to a ball socket joint with a nonzero goal (and only one body).
            QuaternionWide.TransformWithoutOverlap(localOffset, orientation, out offset);
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offset, inertia.InverseInertiaTensor, out var inverseEffectiveMass);

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            inverseEffectiveMass.XX += inertia.InverseMass;
            inverseEffectiveMass.YY += inertia.InverseMass;
            inverseEffectiveMass.ZZ += inertia.InverseMass;
            Symmetric3x3Wide.Invert(inverseEffectiveMass, out effectiveMass);
            Symmetric3x3Wide.Scale(effectiveMass, effectiveMassCFMScale, out effectiveMass);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3Wide offset, in BodyInertiaWide inertia, ref BodyVelocityWide velocityA, in Vector3Wide csi)
        {
            Vector3Wide.CrossWithoutOverlap(offset, csi, out var wsi);
            Symmetric3x3Wide.TransformWithoutOverlap(wsi, inertia.InverseInertiaTensor, out var change);
            Vector3Wide.Add(velocityA.Angular, change, out velocityA.Angular);

            Vector3Wide.Scale(csi, inertia.InverseMass, out change);
            Vector3Wide.Add(velocityA.Linear, change, out velocityA.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref OneBodyLinearServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffset, orientationA, out var offset);
            ApplyImpulse(offset, inertiaA, ref wsvA, accumulatedImpulses);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref OneBodyLinearServoPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffset, orientationA, out var offset);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(offset, positionA, out var worldGrabPoint);
            Vector3Wide.Subtract(prestep.Target, worldGrabPoint, out var error);
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var biasVelocity, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular);
            var csv = biasVelocity - Vector3Wide.Cross(wsvA.Angular, offset) - wsvA.Linear;

            //The grabber is roughly equivalent to a ball socket joint with a nonzero goal (and only one body).
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offset, inertiaA.InverseInertiaTensor, out var inverseEffectiveMass);

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            inverseEffectiveMass.XX += inertiaA.InverseMass;
            inverseEffectiveMass.YY += inertiaA.InverseMass;
            inverseEffectiveMass.ZZ += inertiaA.InverseMass;
            Symmetric3x3Wide.Invert(inverseEffectiveMass, out var effectiveMass);
            Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
            csi = csi * effectiveMassCFMScale - accumulatedImpulses * softnessImpulseScale;

            //The motor has a limited maximum force, so clamp the accumulated impulse. Watch out for division by zero.
            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            ApplyImpulse(offset, inertiaA, ref wsvA, csi);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, ref OneBodyLinearServoPrestepData prestepData) { }
    }

    public class OneBodyLinearServoTypeProcessor : OneBodyTypeProcessor<OneBodyLinearServoPrestepData, Vector3Wide, OneBodyLinearServoFunctions, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 44;
    }
}
