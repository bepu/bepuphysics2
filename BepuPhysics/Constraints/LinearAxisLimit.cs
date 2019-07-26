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
    /// Constrains points on two bodies to a range of offsets from each other along a direction.
    /// </summary>
    public struct LinearAxisLimit : ITwoBodyConstraintDescription<LinearAxisLimit>
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
        /// Direction of the motorized axis in the local space of body A.
        /// </summary>
        public Vector3 LocalAxis;
        /// <summary>
        /// Minimum offset along the world axis between A and B's anchor points.
        /// </summary>
        public float MinimumOffset;
        /// <summary>
        /// Maximum offset along the world axis between A and B's anchor points.
        /// </summary>
        public float MaximumOffset;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return LinearAxisLimitTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(LinearAxisLimitTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(MaximumOffset >= MinimumOffset, "LinearAxisLimit.MaximumOffset must be greater than or equal to LinearAxisLimit.MinimumOffset.");
            ConstraintChecker.AssertUnitLength(LocalAxis, nameof(LinearAxisLimit), nameof(LocalAxis));
            ConstraintChecker.AssertValid(SpringSettings, nameof(LinearAxisLimit));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<LinearAxisLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(LocalAxis, ref target.LocalPlaneNormal);
            GatherScatter.GetFirst(ref target.MinimumOffset) = MinimumOffset;
            GatherScatter.GetFirst(ref target.MaximumOffset) = MaximumOffset;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out LinearAxisLimit description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<LinearAxisLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            Vector3Wide.ReadFirst(source.LocalPlaneNormal, out description.LocalAxis);
            description.MinimumOffset = GatherScatter.GetFirst(ref source.MinimumOffset);
            description.MaximumOffset = GatherScatter.GetFirst(ref source.MaximumOffset);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct LinearAxisLimitPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public Vector3Wide LocalPlaneNormal;
        public Vector<float> MinimumOffset;
        public Vector<float> MaximumOffset;
        public SpringSettingsWide SpringSettings;
    }

    public struct LinearAxisLimitFunctions : IConstraintFunctions<LinearAxisLimitPrestepData, LinearAxisServoProjection, Vector<float>>
    {
        public struct LimitJacobianModifier : LinearAxisServoFunctions.IJacobianModifier
        {
            public Vector<float> MinimumOffset;
            public Vector<float> MaximumOffset;
            public Vector<float> Error;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Modify(in Vector3Wide anchorA, in Vector3Wide anchorB, ref Vector3Wide normal)
            {
                Vector3Wide.Subtract(anchorB, anchorA, out var anchorOffset);
                Vector3Wide.Dot(anchorOffset, normal, out var planeNormalDot);

                var minimumError = MinimumOffset - planeNormalDot;
                var maximumError = planeNormalDot - MaximumOffset;
                var useMin = Vector.LessThan(Vector.Abs(minimumError), Vector.Abs(maximumError));

                Error = Vector.ConditionalSelect(useMin, minimumError, maximumError);
                normal.X = Vector.ConditionalSelect(useMin, -normal.X, normal.X);
                normal.Y = Vector.ConditionalSelect(useMin, -normal.Y, normal.Y);
                normal.Z = Vector.ConditionalSelect(useMin, -normal.Z, normal.Z);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref LinearAxisLimitPrestepData prestep, out LinearAxisServoProjection projection)
        {
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            LimitJacobianModifier modifier;
            modifier.MinimumOffset = prestep.MinimumOffset;
            modifier.MaximumOffset = prestep.MaximumOffset;
            modifier.Error = default;
            LinearAxisServoFunctions.ComputeTransforms(ref modifier, bodies, ref bodyReferences, count, prestep.LocalOffsetA, prestep.LocalOffsetB, prestep.LocalPlaneNormal, inertiaA, inertiaB, effectiveMassCFMScale,
                out var anchorA, out var anchorB, out var normal, out var effectiveMass,
                out projection.LinearVelocityToImpulseA, out projection.AngularVelocityToImpulseA, out projection.AngularVelocityToImpulseB,
                out projection.LinearImpulseToVelocityA, out projection.AngularImpulseToVelocityA, out projection.NegatedLinearImpulseToVelocityB, out projection.AngularImpulseToVelocityB);

            InequalityHelpers.ComputeBiasVelocity(modifier.Error, positionErrorToVelocity, inverseDt, out var biasVelocity);
            projection.BiasImpulse = biasVelocity * effectiveMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref LinearAxisServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            LinearAxisServoFunctions.ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.NegatedLinearImpulseToVelocityB, projection.AngularImpulseToVelocityB,
                ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref LinearAxisServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            LinearAxisServoFunctions.ComputeCorrectiveImpulse(ref velocityA, ref velocityB, projection.LinearVelocityToImpulseA, projection.AngularVelocityToImpulseA, projection.AngularVelocityToImpulseB,
                projection.BiasImpulse, projection.SoftnessImpulseScale, accumulatedImpulse, out var csi);
            InequalityHelpers.ClampPositive(ref accumulatedImpulse, ref csi);
            LinearAxisServoFunctions.ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.NegatedLinearImpulseToVelocityB, projection.AngularImpulseToVelocityB,
                ref csi);
        }

    }

    public class LinearAxisLimitTypeProcessor : TwoBodyTypeProcessor<LinearAxisLimitPrestepData, LinearAxisServoProjection, Vector<float>, LinearAxisLimitFunctions>
    {
        public const int BatchTypeId = 40;
    }
}
