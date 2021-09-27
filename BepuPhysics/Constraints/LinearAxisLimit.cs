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
    /// Constrains points on two bodies to a range of offsets from each other along a direction anchored to body A.
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

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return LinearAxisLimitTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(LinearAxisLimitTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out LinearAxisLimit description)
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

    public struct LinearAxisLimitFunctions : ITwoBodyConstraintFunctions<LinearAxisLimitPrestepData, LinearAxisServoProjection, Vector<float>>
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
        public void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            float dt, float inverseDt, ref LinearAxisLimitPrestepData prestep, out LinearAxisServoProjection projection)
        {
            Unsafe.SkipInit(out projection);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            LimitJacobianModifier modifier;
            modifier.MinimumOffset = prestep.MinimumOffset;
            modifier.MaximumOffset = prestep.MaximumOffset;
            modifier.Error = default;
            LinearAxisServoFunctions.ComputeTransforms(ref modifier, prestep.LocalOffsetA, prestep.LocalOffsetB, prestep.LocalPlaneNormal, orientationA, inertiaA, ab, orientationB, inertiaB, effectiveMassCFMScale,
                out var anchorA, out var anchorB, out var normal, out var effectiveMass,
                out projection.LinearVelocityToImpulseA, out projection.AngularVelocityToImpulseA, out projection.AngularVelocityToImpulseB,
                out projection.LinearImpulseToVelocityA, out projection.AngularImpulseToVelocityA, out projection.NegatedLinearImpulseToVelocityB, out projection.AngularImpulseToVelocityB);

            InequalityHelpers.ComputeBiasVelocity(modifier.Error, positionErrorToVelocity, inverseDt, out var biasVelocity);
            projection.BiasImpulse = biasVelocity * effectiveMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref LinearAxisServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            LinearAxisServoFunctions.ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.NegatedLinearImpulseToVelocityB, projection.AngularImpulseToVelocityB,
                ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref LinearAxisServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            LinearAxisServoFunctions.ComputeCorrectiveImpulse(ref velocityA, ref velocityB, projection.LinearVelocityToImpulseA, projection.AngularVelocityToImpulseA, projection.AngularVelocityToImpulseB,
                projection.BiasImpulse, projection.SoftnessImpulseScale, accumulatedImpulse, out var csi);
            InequalityHelpers.ClampPositive(ref accumulatedImpulse, ref csi);
            LinearAxisServoFunctions.ApplyImpulse(ref velocityA, ref velocityB,
                projection.LinearImpulseToVelocityA, projection.AngularImpulseToVelocityA, projection.NegatedLinearImpulseToVelocityB, projection.AngularImpulseToVelocityB,
                ref csi);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeJacobians(
            in Vector3Wide ab, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector3Wide localPlaneNormal, in Vector3Wide localOffsetA, in Vector3Wide localOffsetB, in Vector<float> minimumOffset, in Vector<float> maximumOffset,
            out Vector<float> error, out Vector3Wide normal, out Vector3Wide angularJA, out Vector3Wide angularJB)
        {
            //Linear jacobians are just normal and -normal. Angular jacobians are offsetA x normal and offsetB x normal.
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(localPlaneNormal, orientationMatrixA, out normal);
            QuaternionWide.TransformWithoutOverlap(localOffsetB, orientationB, out var offsetB);

            //The limit chooses the normal's sign depending on which limit is closer.
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA, orientationMatrixA, out var anchorA);
            var anchorB = ab + offsetB;
            Vector3Wide.Subtract(anchorB, anchorA, out var anchorOffset);
            Vector3Wide.Dot(anchorOffset, normal, out var planeNormalDot);
            var minimumError = minimumOffset - planeNormalDot;
            var maximumError = planeNormalDot - maximumOffset;
            var useMin = Vector.LessThan(Vector.Abs(minimumError), Vector.Abs(maximumError));
            error = Vector.ConditionalSelect(useMin, minimumError, maximumError);
            normal.X = Vector.ConditionalSelect(useMin, -normal.X, normal.X);
            normal.Y = Vector.ConditionalSelect(useMin, -normal.Y, normal.Y);
            normal.Z = Vector.ConditionalSelect(useMin, -normal.Z, normal.Z);

            //Note that the angular jacobian for A uses the offset from A to the attachment point on B. 
            Vector3Wide.CrossWithoutOverlap(anchorB, normal, out angularJA);
            Vector3Wide.CrossWithoutOverlap(normal, offsetB, out angularJB);
        }

        public void WarmStart2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref LinearAxisLimitPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalPlaneNormal, prestep.LocalOffsetA, prestep.LocalOffsetB, prestep.MinimumOffset, prestep.MaximumOffset, out _, out var normal, out var angularJA, out var angularJB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJA, inertiaA.InverseInertiaTensor, out var angularImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularJB, inertiaB.InverseInertiaTensor, out var angularImpulseToVelocityB);
            LinearAxisServoFunctions.ApplyImpulse(normal, angularImpulseToVelocityA, angularImpulseToVelocityB, inertiaA, inertiaB, accumulatedImpulses, ref wsvA, ref wsvB);
        }

        public void Solve2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref LinearAxisLimitPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalPlaneNormal, prestep.LocalOffsetA, prestep.LocalOffsetB, prestep.MinimumOffset, prestep.MaximumOffset, out var error, out var normal, out var angularJA, out var angularJB);

            LinearAxisServoFunctions.ComputeEffectiveMass(angularJA, angularJB, inertiaA, inertiaB, dt, prestep.SpringSettings,
                out var positionErrorToVelocity, out var softnessImpulseScale,
                out var angularImpulseToVelocityA, out var angularImpulseToVelocityB, out var effectiveMass);

            InequalityHelpers.ComputeBiasVelocity(error, positionErrorToVelocity, inverseDt, out var biasVelocity);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csv = Vector3Wide.Dot(wsvA.Linear - wsvB.Linear, normal) + Vector3Wide.Dot(wsvA.Angular, angularJA) + Vector3Wide.Dot(wsvB.Angular, angularJB);

            var csi = effectiveMass * (biasVelocity - csv) - accumulatedImpulses * softnessImpulseScale;

            InequalityHelpers.ClampPositive(ref accumulatedImpulses, ref csi);
            LinearAxisServoFunctions.ApplyImpulse(normal, angularImpulseToVelocityA, angularImpulseToVelocityB, inertiaA, inertiaB, csi, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA,
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in BodyVelocityWide wsvB,
            in Vector<float> dt, in Vector<float> accumulatedImpulses, ref LinearAxisLimitPrestepData prestep)
        {
        }
    }

    public class LinearAxisLimitTypeProcessor : TwoBodyTypeProcessor<LinearAxisLimitPrestepData, LinearAxisServoProjection, Vector<float>, LinearAxisLimitFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 40;
    }
}
