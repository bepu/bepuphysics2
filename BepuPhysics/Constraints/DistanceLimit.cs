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
    /// Constrains points on two bodies to be separated by a distance within a range.
    /// </summary>
    public struct DistanceLimit : ITwoBodyConstraintDescription<DistanceLimit>
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
        /// Minimum distance permitted between the point on A and the point on B.
        /// </summary>
        public float MinimumDistance;
        /// <summary>
        /// Maximum distance permitted between the point on A and the point on B.
        /// </summary>
        public float MaximumDistance;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        /// <summary>
        /// Creates a distance limit description.
        /// </summary>
        /// <param name="localOffsetA">Local offset from the center of body A to its attachment point.</param>
        /// <param name="localOffsetB">Local offset from the center of body B to its attachment point.</param>
        /// <param name="minimumDistance">Minimum distance permitted between the point on A and the point on B.</param>
        /// <param name="maximumDistance">Maximum distance permitted between the point on A and the point on B.</param>
        /// <param name="springSettings">Spring frequency and damping parameters.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public DistanceLimit(in Vector3 localOffsetA, in Vector3 localOffsetB, float minimumDistance, float maximumDistance, in SpringSettings springSettings)
        {
            LocalOffsetA = localOffsetA;
            LocalOffsetB = localOffsetB;
            MinimumDistance = minimumDistance;
            MaximumDistance = maximumDistance;
            SpringSettings = springSettings;
        }

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return DistanceLimitTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(DistanceLimitTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(MinimumDistance >= 0, "DistanceLimit.MinimumDistance must be nonnegative.");
            Debug.Assert(MaximumDistance >= 0, "DistanceLimit.MaximumDistance must be nonnegative.");
            Debug.Assert(MaximumDistance >= MinimumDistance, "DistanceLimit.MaximumDistance must be greater than or equal to DistanceLimit.MinimumDistance.");
            ConstraintChecker.AssertValid(SpringSettings, nameof(DistanceLimit));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<DistanceLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            GatherScatter.GetFirst(ref target.MinimumDistance) = MinimumDistance;
            GatherScatter.GetFirst(ref target.MaximumDistance) = MaximumDistance;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out DistanceLimit description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<DistanceLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            description.MinimumDistance = GatherScatter.GetFirst(ref source.MinimumDistance);
            description.MaximumDistance = GatherScatter.GetFirst(ref source.MaximumDistance);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct DistanceLimitPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public Vector<float> MinimumDistance;
        public Vector<float> MaximumDistance;
        public SpringSettingsWide SpringSettings;
    }

    public struct DistanceLimitFunctions : ITwoBodyConstraintFunctions<DistanceLimitPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3Wide linearJacobianA, in Vector3Wide angularJacobianA, in Vector3Wide angularJacobianB, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector<float> csi, ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB)
        {
            //TODO: Examine codegen quality for operators before generalizing.
            var impulseScaledLinearJacobian = linearJacobianA * csi;
            velocityA.Linear += impulseScaledLinearJacobian * inertiaA.InverseMass;
            velocityB.Linear -= impulseScaledLinearJacobian * inertiaB.InverseMass;
            velocityA.Angular += (angularJacobianA * csi) * inertiaA.InverseInertiaTensor;
            velocityB.Angular += (angularJacobianB * csi) * inertiaB.InverseInertiaTensor;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeJacobians(
            in Vector3Wide localOffsetA, in Vector3Wide positionA, in QuaternionWide orientationA, in Vector3Wide localOffsetB, in Vector3Wide positionB, in QuaternionWide orientationB,
            in Vector<float> minimumDistance, in Vector<float> maximumDistance, out Vector<int> useMinimum, out Vector<float> distance, out Vector3Wide direction, out Vector3Wide angularJA, out Vector3Wide angularJB)
        {
            QuaternionWide.TransformWithoutOverlap(localOffsetA, orientationA, out var offsetA);
            QuaternionWide.TransformWithoutOverlap(localOffsetB, orientationB, out var offsetB);
            var anchorOffset = (offsetB - offsetA) + (positionB - positionA);
            Vector3Wide.Length(anchorOffset, out distance);
            //If the current distance is closer to the minimum, calibrate for the minimum. Otherwise, calibrate for the maximum.
            useMinimum = Vector.LessThan(Vector.Abs(distance - minimumDistance), Vector.Abs(distance - maximumDistance));
            var sign = Vector.ConditionalSelect(useMinimum, new Vector<float>(-1f), Vector<float>.One);
            Vector3Wide.Scale(anchorOffset, sign / distance, out direction);
            //If the distance is too short to extract a direction, use an arbitrary fallback.
            var needFallback = Vector.LessThan(distance, new Vector<float>(1e-9f));
            direction.X = Vector.ConditionalSelect(needFallback, Vector<float>.One, direction.X);
            direction.Y = Vector.ConditionalSelect(needFallback, Vector<float>.Zero, direction.Y);
            direction.Z = Vector.ConditionalSelect(needFallback, Vector<float>.Zero, direction.Z);

            Vector3Wide.CrossWithoutOverlap(offsetA, direction, out angularJA);
            Vector3Wide.CrossWithoutOverlap(direction, offsetB, out angularJB); //Note flip negation.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref DistanceLimitPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(prestep.LocalOffsetA, positionA, orientationA, prestep.LocalOffsetB, positionB, orientationB, prestep.MinimumDistance, prestep.MaximumDistance, out _, out _, out var direction, out var angularJA, out var angularJB);
            ApplyImpulse(direction, angularJA, angularJB, inertiaA, inertiaB, accumulatedImpulses, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref DistanceLimitPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(prestep.LocalOffsetA, positionA, orientationA, prestep.LocalOffsetB, positionB, orientationB, prestep.MinimumDistance, prestep.MaximumDistance, out var useMinimum, out var distance, out var direction, out var angularJA, out var angularJB);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(wsvA.Linear, direction, out var linearCSVA);
            Vector3Wide.Dot(wsvB.Linear, direction, out var negatedLinearCSVB);
            Vector3Wide.Dot(wsvA.Angular, angularJA, out var angularCSVA);
            Vector3Wide.Dot(wsvB.Angular, angularJB, out var angularCSVB);
            var csv = linearCSVA - negatedLinearCSVB + angularCSVA + angularCSVB;

            //The linear jacobian contributions are just a scalar multiplication by 1 since it's a unit length vector.
            Symmetric3x3Wide.VectorSandwich(angularJA, inertiaA.InverseInertiaTensor, out var angularContributionA);
            Symmetric3x3Wide.VectorSandwich(angularJB, inertiaB.InverseInertiaTensor, out var angularContributionB);
            var inverseEffectiveMass = inertiaA.InverseMass + inertiaB.InverseMass + angularContributionA + angularContributionB;

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var effectiveMass = effectiveMassCFMScale / inverseEffectiveMass;
            var error = Vector.ConditionalSelect(useMinimum, prestep.MinimumDistance - distance, distance - prestep.MaximumDistance);
            InequalityHelpers.ComputeBiasVelocity(error, positionErrorToVelocity, inverseDt, out var biasVelocity);
            var csi = -accumulatedImpulses * softnessImpulseScale - effectiveMass * (csv - biasVelocity);
            InequalityHelpers.ClampPositive(ref accumulatedImpulses, ref csi);

            ApplyImpulse(direction, angularJA, angularJB, inertiaA, inertiaB, csi, ref wsvA, ref wsvB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref DistanceLimitPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class DistanceLimitTypeProcessor : TwoBodyTypeProcessor<DistanceLimitPrestepData, Vector<float>, DistanceLimitFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 34;
    }
}
