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
        
        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return DistanceLimitTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(DistanceLimitTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out DistanceLimit description)
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

    public struct DistanceLimitProjection
    {
        public Vector3Wide LinearVelocityToImpulseA;
        public Vector3Wide AngularVelocityToImpulseA;
        public Vector3Wide AngularVelocityToImpulseB;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector3Wide LinearImpulseToVelocityA;
        public Vector3Wide AngularImpulseToVelocityA;
        public Vector3Wide LinearImpulseToVelocityB;
        public Vector3Wide AngularImpulseToVelocityB;
    }

    public struct DistanceLimitFunctions : IConstraintFunctions<DistanceLimitPrestepData, DistanceLimitProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref DistanceLimitPrestepData prestep, out DistanceLimitProjection projection)
        {
            DistanceServoFunctions.GetDistance(bodies, ref bodyReferences, count, prestep.LocalOffsetA, prestep.LocalOffsetB,
                out var anchorOffsetA, out var anchorOffsetB, out var anchorOffset, out var distance);
            //If the current distance is closer to the minimum, calibrate for the minimum. Otherwise, calibrate for the maximum.
            var useMinimum = Vector.LessThan(Vector.Abs(distance - prestep.MinimumDistance), Vector.Abs(distance - prestep.MaximumDistance));
            var sign = Vector.ConditionalSelect(useMinimum, new Vector<float>(-1f), Vector<float>.One);
            Vector3Wide.Scale(anchorOffset, sign / distance, out var direction);
            DistanceServoFunctions.ComputeTransforms(inertiaA, inertiaB, anchorOffsetA, anchorOffsetB, distance, ref direction, dt,
                prestep.SpringSettings, out var positionErrorToVelocity, out projection.SoftnessImpulseScale, out var effectiveMass,
                out projection.LinearVelocityToImpulseA, out projection.AngularVelocityToImpulseA, out projection.AngularVelocityToImpulseB,
                out projection.LinearImpulseToVelocityA, out projection.AngularImpulseToVelocityA, out projection.LinearImpulseToVelocityB, out projection.AngularImpulseToVelocityB);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            var error = Vector.ConditionalSelect(useMinimum, prestep.MinimumDistance - distance, distance - prestep.MaximumDistance);
            InequalityHelpers.ComputeBiasVelocity(error, positionErrorToVelocity, inverseDt, out var biasVelocity);
            projection.BiasImpulse = biasVelocity * effectiveMass;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref DistanceLimitProjection projection, ref Vector<float> accumulatedImpulse)
        {
            DistanceServoFunctions.ApplyImpulse(ref velocityA, ref velocityB, 
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.LinearImpulseToVelocityB, projection.AngularImpulseToVelocityB, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref DistanceLimitProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(velocityA.Linear, projection.LinearVelocityToImpulseA, out var linearCSIA);
            Vector3Wide.Dot(velocityB.Linear, projection.LinearVelocityToImpulseA, out var negatedLinearCSIB);
            Vector3Wide.Dot(velocityA.Angular, projection.AngularVelocityToImpulseA, out var angularCSIA);
            Vector3Wide.Dot(velocityB.Angular, projection.AngularVelocityToImpulseB, out var angularCSIB);
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (linearCSIA + angularCSIA - negatedLinearCSIB + angularCSIB);
            InequalityHelpers.ClampPositive(ref accumulatedImpulse, ref csi);
            DistanceServoFunctions.ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.LinearImpulseToVelocityB, projection.AngularImpulseToVelocityB, ref csi);

        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class DistanceLimitTypeProcessor : TwoBodyTypeProcessor<DistanceLimitPrestepData, DistanceLimitProjection, Vector<float>, DistanceLimitFunctions>
    {
        public const int BatchTypeId = 34;
    }
}
