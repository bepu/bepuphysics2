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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return OneBodyLinearServoTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(OneBodyLinearServoTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(OneBodyLinearServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<OneBodyLinearServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffset, ref target.LocalOffset);
            Vector3Wide.WriteFirst(Target, ref target.Target);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out OneBodyLinearServo description)
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

    public struct OneBodyLinearServoProjection
    {
        public Vector3Wide Offset;
        public Vector3Wide BiasVelocity;
        public Symmetric3x3Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public BodyInertias Inertia;
    }

    public struct OneBodyLinearServoFunctions : IOneBodyConstraintFunctions<OneBodyLinearServoPrestepData, OneBodyLinearServoProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeTransforms(in Vector3Wide localOffset, in QuaternionWide orientation, in Vector<float> effectiveMassCFMScale,
            in BodyInertias inertia, out Vector3Wide offset, out Symmetric3x3Wide effectiveMass)
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
        public void Prestep(Bodies bodies, ref Vector<int> bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertia, ref OneBodyLinearServoPrestepData prestep,
            out OneBodyLinearServoProjection projection)
        {
            //TODO: Note that this grabs a world position. That poses a problem for different position representations.
            bodies.GatherPose(ref bodyReferences, count, out var position, out var orientation);
            projection.Inertia = inertia;

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);

            ComputeTransforms(prestep.LocalOffset, orientation, effectiveMassCFMScale, inertia, out projection.Offset, out projection.EffectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(projection.Offset, position, out var worldGrabPoint);
            Vector3Wide.Subtract(prestep.Target, worldGrabPoint, out var error);
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out projection.BiasVelocity, out projection.MaximumImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref BodyVelocities velocityA, in OneBodyLinearServoProjection projection, ref Vector3Wide csi)
        {
            Vector3Wide.CrossWithoutOverlap(projection.Offset, csi, out var wsi);
            Symmetric3x3Wide.TransformWithoutOverlap(wsi, projection.Inertia.InverseInertiaTensor, out var change);
            Vector3Wide.Add(velocityA.Angular, change, out velocityA.Angular);

            Vector3Wide.Scale(csi, projection.Inertia.InverseMass, out change);
            Vector3Wide.Add(velocityA.Linear, change, out velocityA.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref OneBodyLinearServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SharedSolve(ref BodyVelocities velocities, in OneBodyLinearServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular);
            Vector3Wide.CrossWithoutOverlap(velocities.Angular, projection.Offset, out var angularCSV);
            Vector3Wide.Add(velocities.Linear, angularCSV, out var csv);
            Vector3Wide.Subtract(projection.BiasVelocity, csv, out csv);

            Symmetric3x3Wide.TransformWithoutOverlap(csv, projection.EffectiveMass, out var csi);
            Vector3Wide.Scale(accumulatedImpulse, projection.SoftnessImpulseScale, out var softness);
            Vector3Wide.Subtract(csi, softness, out csi);

            //The motor has a limited maximum force, so clamp the accumulated impulse. Watch out for division by zero.
            ServoSettingsWide.ClampImpulse(projection.MaximumImpulse, ref accumulatedImpulse, ref csi);
            var previous = accumulatedImpulse;
            Vector3Wide.Add(accumulatedImpulse, csi, out accumulatedImpulse);
            Vector3Wide.Length(accumulatedImpulse, out var impulseMagnitude);
            var newMagnitude = Vector.Min(impulseMagnitude, projection.MaximumImpulse);
            var scale = newMagnitude / impulseMagnitude;
            Vector3Wide.Scale(accumulatedImpulse, scale, out accumulatedImpulse);
            Vector3Wide.ConditionalSelect(Vector.GreaterThan(impulseMagnitude, Vector<float>.Zero), accumulatedImpulse, previous, out accumulatedImpulse);
            Vector3Wide.Subtract(accumulatedImpulse, previous, out csi);

            ApplyImpulse(ref velocities, projection, ref csi);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref OneBodyLinearServoProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            SharedSolve(ref velocityA, projection, ref accumulatedImpulse);
        }

    }

    public class OneBodyLinearServoTypeProcessor : OneBodyTypeProcessor<OneBodyLinearServoPrestepData, OneBodyLinearServoProjection, Vector3Wide, OneBodyLinearServoFunctions>
    {
        public const int BatchTypeId = 44;
    }
}
