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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return DistanceServoTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(DistanceServoTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out DistanceServo description)
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

    public struct DistanceServoProjection
    {
        public Vector3Wide LinearVelocityToImpulseA;
        public Vector3Wide AngularVelocityToImpulseA;
        public Vector3Wide AngularVelocityToImpulseB;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public Vector3Wide LinearImpulseToVelocityA;
        public Vector3Wide AngularImpulseToVelocityA;
        public Vector3Wide LinearImpulseToVelocityB;
        public Vector3Wide AngularImpulseToVelocityB;
    }

    public struct DistanceServoFunctions : IConstraintFunctions<DistanceServoPrestepData, DistanceServoProjection, Vector<float>>
    {
        public static void GetDistance(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, in Vector3Wide localOffsetA, in Vector3Wide localOffsetB,
            out Vector3Wide anchorOffsetA, out Vector3Wide anchorOffsetB, out Vector3Wide anchorOffset, out Vector<float> distance)
        {
            bodies.GatherPose(ref bodyReferences, count, out var offsetB, out var orientationA, out var orientationB);

            QuaternionWide.TransformWithoutOverlap(localOffsetA, orientationA, out anchorOffsetA);
            QuaternionWide.TransformWithoutOverlap(localOffsetB, orientationB, out anchorOffsetB);
            Vector3Wide.Add(anchorOffsetB, offsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, anchorOffsetA, out anchorOffset);

            Vector3Wide.Length(anchorOffset, out distance);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeTransforms(
            in BodyInertias inertiaA, in BodyInertias inertiaB, in Vector3Wide anchorOffsetA, in Vector3Wide anchorOffsetB, in Vector<float> distance, ref Vector3Wide direction,
            float dt, in SpringSettingsWide springSettings,
            out Vector<float> positionErrorToVelocity, out Vector<float> softnessImpulseScale, out Vector<float> effectiveMass,
            out Vector3Wide linearVelocityToImpulseA, out Vector3Wide angularVelocityToImpulseA, out Vector3Wide angularVelocityToImpulseB,
            out Vector3Wide linearImpulseToVelocityA, out Vector3Wide angularImpulseToVelocityA, out Vector3Wide linearImpulseToVelocityB, out Vector3Wide angularImpulseToVelocityB)
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

            //If the distance is zero, there is no valid offset direction. Pick one arbitrarily.
            var needFallback = Vector.LessThan(distance, new Vector<float>(1e-9f));
            Vector3Wide.Broadcast(new Vector3(1, 0, 0), out var fallback);
            direction.X = Vector.ConditionalSelect(needFallback, Vector<float>.One, direction.X);
            direction.Y = Vector.ConditionalSelect(needFallback, Vector<float>.Zero, direction.Y);
            direction.Z = Vector.ConditionalSelect(needFallback, Vector<float>.Zero, direction.Z);

            Vector3Wide.CrossWithoutOverlap(anchorOffsetA, direction, out var angularJA);
            Vector3Wide.CrossWithoutOverlap(direction, anchorOffsetB, out var angularJB); //Note flip negation.

            //The linear jacobian contributions are just a scalar multiplication by 1 since it's a unit length vector.
            Symmetric3x3Wide.VectorSandwich(angularJA, inertiaA.InverseInertiaTensor, out var angularContributionA);
            Symmetric3x3Wide.VectorSandwich(angularJB, inertiaB.InverseInertiaTensor, out var angularContributionB);
            var inverseEffectiveMass = inertiaA.InverseMass + inertiaB.InverseMass + angularContributionA + angularContributionB;

            SpringSettingsWide.ComputeSpringiness(springSettings, dt, out positionErrorToVelocity, out var effectiveMassCFMScale, out softnessImpulseScale);
            effectiveMass = effectiveMassCFMScale / inverseEffectiveMass;
            Vector3Wide.Scale(direction, effectiveMass, out linearVelocityToImpulseA);
            Vector3Wide.Scale(angularJA, effectiveMass, out angularVelocityToImpulseA);
            Vector3Wide.Scale(angularJB, effectiveMass, out angularVelocityToImpulseB);

            Vector3Wide.Scale(direction, inertiaA.InverseMass, out linearImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJA, inertiaA.InverseInertiaTensor, out angularImpulseToVelocityA);
            Vector3Wide.Scale(direction, -inertiaB.InverseMass, out linearImpulseToVelocityB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJB, inertiaB.InverseInertiaTensor, out angularImpulseToVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref DistanceServoPrestepData prestep, out DistanceServoProjection projection)
        {
            GetDistance(bodies, ref bodyReferences, count, prestep.LocalOffsetA, prestep.LocalOffsetB, out var anchorOffsetA, out var anchorOffsetB, out var anchorOffset, out var distance);

            Vector3Wide.Scale(anchorOffset, Vector<float>.One / distance, out var direction);

            ComputeTransforms(inertiaA, inertiaB, anchorOffsetA, anchorOffsetB, distance, ref direction, dt, prestep.SpringSettings,
                out var positionErrorToVelocity, out projection.SoftnessImpulseScale, out var effectiveMass,
                out projection.LinearVelocityToImpulseA, out projection.AngularVelocityToImpulseA, out projection.AngularVelocityToImpulseB,
                out projection.LinearImpulseToVelocityA, out projection.AngularImpulseToVelocityA, out projection.LinearImpulseToVelocityB, out projection.AngularImpulseToVelocityB);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            var error = distance - prestep.TargetDistance;
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var clampedBiasVelocity, out projection.MaximumImpulse);
            projection.BiasImpulse = clampedBiasVelocity * effectiveMass;

        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB,
            in Vector3Wide linearImpulseToVelocityA, in Vector3Wide angularImpulseToVelocityA, in Vector3Wide linearImpulseToVelocityB, in Vector3Wide angularImpulseToVelocityB,
            ref Vector<float> csi)
        {
            Vector3Wide.Scale(linearImpulseToVelocityA, csi, out var linearVelocityChangeA);
            Vector3Wide.Scale(angularImpulseToVelocityA, csi, out var angularVelocityChangeA);
            Vector3Wide.Add(linearVelocityChangeA, velocityA.Linear, out velocityA.Linear);
            Vector3Wide.Add(angularVelocityChangeA, velocityA.Angular, out velocityA.Angular);
            Vector3Wide.Scale(linearImpulseToVelocityB, csi, out var linearVelocityChangeB);
            Vector3Wide.Scale(angularImpulseToVelocityB, csi, out var angularVelocityChangeB);
            Vector3Wide.Add(linearVelocityChangeB, velocityB.Linear, out velocityB.Linear);
            Vector3Wide.Add(angularVelocityChangeB, velocityB.Angular, out velocityB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref DistanceServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.LinearImpulseToVelocityB, projection.AngularImpulseToVelocityB, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref DistanceServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(velocityA.Linear, projection.LinearVelocityToImpulseA, out var linearCSIA);
            Vector3Wide.Dot(velocityB.Linear, projection.LinearVelocityToImpulseA, out var negatedLinearCSIB);
            Vector3Wide.Dot(velocityA.Angular, projection.AngularVelocityToImpulseA, out var angularCSIA);
            Vector3Wide.Dot(velocityB.Angular, projection.AngularVelocityToImpulseB, out var angularCSIB);
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (linearCSIA + angularCSIA - negatedLinearCSIB + angularCSIB);
            ServoSettingsWide.ClampImpulse(projection.MaximumImpulse, ref accumulatedImpulse, ref csi);

            ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.LinearImpulseToVelocityB, projection.AngularImpulseToVelocityB, ref csi);
        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class DistanceServoTypeProcessor : TwoBodyTypeProcessor<DistanceServoPrestepData, DistanceServoProjection, Vector<float>, DistanceServoFunctions>
    {
        public const int BatchTypeId = 33;
    }
}
