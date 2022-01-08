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
    /// Constrains the center of two bodies to be separated by a distance within a range.
    /// </summary>
    public struct CenterDistanceLimit : ITwoBodyConstraintDescription<CenterDistanceLimit>
    {
        /// <summary>
        /// Minimum distance between the body centers.
        /// </summary>
        public float MinimumDistance;
        /// <summary>
        /// Maximum distance between the body centers.
        /// </summary>
        public float MaximumDistance;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CenterDistanceLimit(float minimumDistance, float maximumDistance, in SpringSettings springSettings)
        {
            MinimumDistance = minimumDistance;
            MaximumDistance = maximumDistance;
            SpringSettings = springSettings;
        }

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return CenterDistanceLimitTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(CenterDistanceLimitTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(MinimumDistance >= 0, "CenterDistanceLimit.MinimumDistance must be nonnegative.");
            Debug.Assert(MaximumDistance >= 0, "CenterDistanceLimit.MaximumDistance must be nonnegative.");
            ConstraintChecker.AssertValid(SpringSettings, nameof(CenterDistanceLimit));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<CenterDistanceLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GatherScatter.GetFirst(ref target.MinimumDistance) = MinimumDistance;
            GatherScatter.GetFirst(ref target.MaximumDistance) = MaximumDistance;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out CenterDistanceLimit description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<CenterDistanceLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.MinimumDistance = GatherScatter.GetFirst(ref source.MinimumDistance);
            description.MaximumDistance = GatherScatter.GetFirst(ref source.MaximumDistance);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct CenterDistanceLimitPrestepData
    {
        public Vector<float> MinimumDistance;
        public Vector<float> MaximumDistance;
        public SpringSettingsWide SpringSettings;
    }

    public struct CenterDistanceLimitFunctions : ITwoBodyConstraintFunctions<CenterDistanceLimitPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeJacobian(Vector<float> minimumDistance, Vector<float> maximumDistance, in Vector3Wide positionA, in Vector3Wide positionB, out Vector3Wide jacobianA, out Vector<float> distance, out Vector<int> useMinimum)
        {
            //Note that we need the actual length in both warmstart and solve in the limit version of the constraint; since the min/max bound determines jacobian sign.
            var ab = positionB - positionA;
            distance = ab.Length();
            var inverseDistance = MathHelper.FastReciprocal(distance);
            var useFallback = Vector.LessThan(distance, new Vector<float>(1e-5f));
            Vector3Wide.Scale(ab, inverseDistance, out jacobianA);
            jacobianA.X = Vector.ConditionalSelect(useFallback, Vector<float>.One, jacobianA.X);
            jacobianA.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, jacobianA.Y);
            jacobianA.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, jacobianA.Z);

            //If the current distance is closer to the minimum, calibrate for the minimum. Otherwise, calibrate for the maximum.
            useMinimum = Vector.LessThan(Vector.Abs(distance - minimumDistance), Vector.Abs(distance - maximumDistance));
            jacobianA = Vector3Wide.ConditionalSelect(useMinimum, -jacobianA, jacobianA);
        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            ref CenterDistanceLimitPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(prestep.MinimumDistance, prestep.MaximumDistance, positionA, positionB, out var jacobianA, out _, out _);
            CenterDistanceConstraintFunctions.ApplyImpulse(jacobianA, inertiaA.InverseMass, inertiaB.InverseMass, accumulatedImpulses, ref wsvA, ref wsvB);
        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt,
            ref CenterDistanceLimitPrestepData prestep, ref Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(prestep.MinimumDistance, prestep.MaximumDistance, positionA, positionB, out var jacobianA, out var distance, out var useMinimum);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            //Jacobian is just the unit length direction, so the effective mass is simple:
            var effectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + inertiaB.InverseMass);

            var error = Vector.ConditionalSelect(useMinimum, prestep.MinimumDistance - distance, distance - prestep.MaximumDistance);
            InequalityHelpers.ComputeBiasVelocity(error, positionErrorToVelocity, inverseDt, out var biasVelocity);
            var csv = Vector3Wide.Dot(wsvA.Linear, jacobianA) - Vector3Wide.Dot(wsvB.Linear, jacobianA);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = -accumulatedImpulse * softnessImpulseScale - effectiveMass * (csv - biasVelocity);
            InequalityHelpers.ClampPositive(ref accumulatedImpulse, ref csi);

            CenterDistanceConstraintFunctions.ApplyImpulse(jacobianA, inertiaA.InverseMass, inertiaB.InverseMass, csi, ref wsvA, ref wsvB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref CenterDistanceLimitPrestepData prestepData) { }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class CenterDistanceLimitTypeProcessor : TwoBodyTypeProcessor<CenterDistanceLimitPrestepData, Vector<float>, CenterDistanceLimitFunctions, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear>
    {
        public const int BatchTypeId = 55;
    }
}
