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
    /// Constrains points on two bodies to be separated by a goal distance.
    /// </summary>
    public struct DistanceServo : ITwoBodyConstraintDescription<DistanceServo>
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
        /// Distance that the constraint will try to reach between the attachment points.
        /// </summary>
        public float TargetDistance;
        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        /// <summary>
        /// Creates a distance servo description.
        /// </summary>
        /// <param name="localOffsetA">Local offset from the center of body A to its attachment point.</param>
        /// <param name="localOffsetB">Local offset from the center of body B to its attachment point.</param>
        /// <param name="targetDistance">Distance that the constraint will try to reach between the attachment points.</param>
        /// <param name="springSettings">Spring frequency and damping parameters.</param>
        /// <param name="servoSettings">Servo control parameters.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public DistanceServo(in Vector3 localOffsetA, in Vector3 localOffsetB, float targetDistance, in SpringSettings springSettings, in ServoSettings servoSettings)
        {
            LocalOffsetA = localOffsetA;
            LocalOffsetB = localOffsetB;
            TargetDistance = targetDistance;
            SpringSettings = springSettings;
            ServoSettings = servoSettings;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public DistanceServo(in Vector3 localOffsetA, in Vector3 localOffsetB, float targetDistance, in SpringSettings springSettings)
            : this(localOffsetA, localOffsetB, targetDistance, springSettings, ServoSettings.Default)
        {
        }

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return DistanceServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(DistanceServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(TargetDistance >= 0, "DistanceServo.TargetDistance must be nonnegative.");
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(DistanceServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<DistanceServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            GatherScatter.GetFirst(ref target.TargetDistance) = TargetDistance;
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out DistanceServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<DistanceServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            description.TargetDistance = GatherScatter.GetFirst(ref source.TargetDistance);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct DistanceServoPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public Vector<float> TargetDistance;
        public ServoSettingsWide ServoSettings;
        public SpringSettingsWide SpringSettings;
    }

    public struct DistanceServoFunctions : ITwoBodyConstraintFunctions<DistanceServoPrestepData, Vector<float>>
    {
        public static void GetDistance(in QuaternionWide orientationA, in Vector3Wide ab, in QuaternionWide orientationB, in Vector3Wide localOffsetA, in Vector3Wide localOffsetB,
            out Vector3Wide anchorOffsetA, out Vector3Wide anchorOffsetB, out Vector3Wide anchorOffset, out Vector<float> distance)
        {
            QuaternionWide.TransformWithoutOverlap(localOffsetA, orientationA, out anchorOffsetA);
            QuaternionWide.TransformWithoutOverlap(localOffsetB, orientationB, out anchorOffsetB);
            Vector3Wide.Add(anchorOffsetB, ab, out var anchorB);
            Vector3Wide.Subtract(anchorB, anchorOffsetA, out anchorOffset);

            Vector3Wide.Length(anchorOffset, out distance);
        }

        public static void ComputeJacobian(in Vector<float> distance, in Vector3Wide anchorOffsetA, in Vector3Wide anchorOffsetB, ref Vector3Wide direction, out Vector3Wide angularJA, out Vector3Wide angularJB)
        {
            //If the distance is zero, there is no valid offset direction. Pick one arbitrarily.
            var needFallback = Vector.LessThan(distance, new Vector<float>(1e-9f));
            direction.X = Vector.ConditionalSelect(needFallback, Vector<float>.One, direction.X);
            direction.Y = Vector.ConditionalSelect(needFallback, Vector<float>.Zero, direction.Y);
            direction.Z = Vector.ConditionalSelect(needFallback, Vector<float>.Zero, direction.Z);

            Vector3Wide.CrossWithoutOverlap(anchorOffsetA, direction, out angularJA);
            Vector3Wide.CrossWithoutOverlap(direction, anchorOffsetB, out angularJB); //Note flip negation.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeTransforms(
            in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, in Vector3Wide anchorOffsetA, in Vector3Wide anchorOffsetB, in Vector<float> distance, ref Vector3Wide direction,
            float dt, in SpringSettingsWide springSettings,
            out Vector<float> positionErrorToVelocity, out Vector<float> softnessImpulseScale, out Vector<float> effectiveMass,
            out Vector3Wide angularJA, out Vector3Wide angularJB, out Vector3Wide angularImpulseToVelocityA, out Vector3Wide angularImpulseToVelocityB)
        {
            //Position constraint:
            //||positionA + localOffsetA * orientationA - positionB - localOffsetB * orientationB|| = distance
            //Skipping a bunch of algebra, the velocity constraint applies to the change in velocity along the separating axis.
            //dot(linearA + angularA x (localOffsetA * orientationA) - linearB - angularA x (localOffsetB * orientationB), normalize(positionA + localOffsetA * orientationA - positionB - localOffsetB * orientationB)) = 0
            //dot(linearA, direction) + dot(angularA x offsetA, direction) + dot(linearB, -direction) + dot(angularB x offsetB, -direction) = 0
            //dot(linearA, direction) + dot(offsetA x direction, angularA) + dot(linearB, -direction) + dot(offsetB x -direction, angularB) = 0
            //dot(linearA, direction) + dot(offsetA x direction, angularA) - dot(linearB, direction) + dot(direction x offsetB, angularB) = 0
            //Jacobians are direction, -direction, offsetA x direction, and direction x offsetB.
            //That's 9 unique scalars.
            //We can either store those 9 plus 14 for the inverse masses, or we can premultiply.
            //V * JT * Me * J * I^-1
            //If you premultiply JT * Me and J * I^-1, you get 9 scalars for velocity->impulse and 12 for impulse->velocity.
            //If you don't premultiply, it takes 9 for jacobians, 14 for inverse inertia, and then 1 for effective mass.
            //That's 21 versus 24. On top of that, premultiplying saves some ALU work.

            //Note that we're working with the distance instead of distance squared. That makes it easier to use and reason about at the cost of a square root in the prestep.
            //That really, really doesn't matter.
            ComputeJacobian(distance, anchorOffsetA, anchorOffsetB, ref direction, out angularJA, out angularJB);

            //The linear jacobian contributions are just a scalar multiplication by 1 since it's a unit length vector.
            Symmetric3x3Wide.TransformWithoutOverlap(angularJA, inertiaA.InverseInertiaTensor, out angularImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJB, inertiaB.InverseInertiaTensor, out angularImpulseToVelocityB);
            Vector3Wide.Dot(angularJA, angularImpulseToVelocityA, out var angularContributionA);
            Vector3Wide.Dot(angularJB, angularImpulseToVelocityB, out var angularContributionB);
            var inverseEffectiveMass = inertiaA.InverseMass + inertiaB.InverseMass + angularContributionA + angularContributionB;

            SpringSettingsWide.ComputeSpringiness(springSettings, dt, out positionErrorToVelocity, out var effectiveMassCFMScale, out softnessImpulseScale);
            effectiveMass = effectiveMassCFMScale / inverseEffectiveMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(
            in Vector<float> inverseMassA, in Vector<float> inverseMassB, in Vector3Wide direction, in Vector3Wide angularImpulseToVelocityA, in Vector3Wide angularImpulseToVelocityB,
            in Vector<float> csi, ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB)
        {
            Vector3Wide.Scale(direction, csi * inverseMassA, out var linearVelocityChangeA);
            Vector3Wide.Scale(angularImpulseToVelocityA, csi, out var angularVelocityChangeA);
            Vector3Wide.Add(linearVelocityChangeA, velocityA.Linear, out velocityA.Linear);
            Vector3Wide.Add(angularVelocityChangeA, velocityA.Angular, out velocityA.Angular);
            Vector3Wide.Scale(direction, csi * inverseMassB, out var negatedLinearVelocityChangeB);
            Vector3Wide.Scale(angularImpulseToVelocityB, csi, out var angularVelocityChangeB);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearVelocityChangeB, out velocityB.Linear);
            Vector3Wide.Add(angularVelocityChangeB, velocityB.Angular, out velocityB.Angular);
        }


        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref DistanceServoPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            GetDistance(orientationA, positionB - positionA, orientationB, prestep.LocalOffsetA, prestep.LocalOffsetB, out var anchorOffsetA, out var anchorOffsetB, out var anchorOffset, out var distance);
            Vector3Wide.Scale(anchorOffset, Vector<float>.One / distance, out var direction);
            ComputeJacobian(distance, anchorOffsetA, anchorOffsetB, ref direction, out var angularJA, out var angularJB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJA, inertiaA.InverseInertiaTensor, out var angularImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJB, inertiaB.InverseInertiaTensor, out var angularImpulseToVelocityB);
            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, direction, angularImpulseToVelocityA, angularImpulseToVelocityB, accumulatedImpulses, ref wsvA, ref wsvB);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref DistanceServoPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            GetDistance(orientationA, positionB - positionA, orientationB, prestep.LocalOffsetA, prestep.LocalOffsetB, out var anchorOffsetA, out var anchorOffsetB, out var anchorOffset, out var distance);

            Vector3Wide.Scale(anchorOffset, Vector<float>.One / distance, out var direction);

            ComputeTransforms(inertiaA, inertiaB, anchorOffsetA, anchorOffsetB, distance, ref direction, dt, prestep.SpringSettings,
                out var positionErrorToVelocity, out var softnessImpulseScale, out var effectiveMass, out var angularJA, out var angularJB, out var angularImpulseToVelocityA, out var angularImpulseToVelocityB);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            var error = distance - prestep.TargetDistance;
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var clampedBiasVelocity, out var maximumImpulse);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(wsvA.Linear, direction, out var linearCSVA);
            Vector3Wide.Dot(wsvB.Linear, direction, out var negatedLinearCSVB);
            Vector3Wide.Dot(wsvA.Angular, angularJA, out var angularCSVA);
            Vector3Wide.Dot(wsvB.Angular, angularJB, out var angularCSVB);
            var csi = (clampedBiasVelocity - linearCSVA - angularCSVA + negatedLinearCSVB - angularCSVB) * effectiveMass - accumulatedImpulses * softnessImpulseScale;
            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);

            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, direction, angularImpulseToVelocityA, angularImpulseToVelocityB, csi, ref wsvA, ref wsvB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref DistanceServoPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class DistanceServoTypeProcessor : TwoBodyTypeProcessor<DistanceServoPrestepData, Vector<float>, DistanceServoFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 33;
    }
}
