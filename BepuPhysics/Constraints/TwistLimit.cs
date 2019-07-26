using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Constrains two bodies' rotations around attached twist axes to a range of permitted twist angles.
    /// </summary>
    public struct TwistLimit : ITwoBodyConstraintDescription<TwistLimit>
    {
        /// <summary>
        /// Local space basis attached to body A against which to measure body B's transformed axis. Expressed as a 3x3 rotation matrix, the X axis corresponds with 0 degrees, 
        /// the Y axis corresponds to 90 degrees, and the Z axis is the twist axis.
        /// </summary>
        public Quaternion LocalBasisA;
        /// <summary>
        /// Local space basis attached to body B that will be measured against body A's basis.
        /// Expressed as a 3x3 rotation matrix, the transformed X axis will be measured against A's X and Y axes. The Z axis is the twist axis.
        /// </summary>
        public Quaternion LocalBasisB;
        /// <summary>
        /// Minimum angle between B's axis to measure and A's measurement axis. 
        /// </summary>
        public float MinimumAngle;
        /// <summary>
        /// Maximum angle between B's axis to measure and A's measurement axis. 
        /// </summary>
        public float MaximumAngle;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return TwistLimitTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(TwistLimitTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalBasisA, nameof(TwistLimit), nameof(LocalBasisA));
            ConstraintChecker.AssertUnitLength(LocalBasisB, nameof(TwistLimit), nameof(LocalBasisB));
            ConstraintChecker.AssertValid(SpringSettings, nameof(TwistLimit));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TwistLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(LocalBasisA, ref target.LocalBasisA);
            QuaternionWide.WriteFirst(LocalBasisB, ref target.LocalBasisB);
            GetFirst(ref target.MinimumAngle) = MinimumAngle;
            GetFirst(ref target.MaximumAngle) = MaximumAngle;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out TwistLimit description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<TwistLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.ReadFirst(source.LocalBasisA, out description.LocalBasisA);
            QuaternionWide.ReadFirst(source.LocalBasisB, out description.LocalBasisB);
            description.MinimumAngle = GetFirst(ref source.MinimumAngle);
            description.MaximumAngle = GetFirst(ref source.MaximumAngle);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct TwistLimitPrestepData
    {
        public QuaternionWide LocalBasisA;
        public QuaternionWide LocalBasisB;
        public Vector<float> MinimumAngle;
        public Vector<float> MaximumAngle;
        public SpringSettingsWide SpringSettings;
    }

    public struct TwistLimitProjection
    {
        public Vector3Wide VelocityToImpulseA;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector3Wide ImpulseToVelocityA;
        public Vector3Wide NegatedImpulseToVelocityB;
    }


    public struct TwistLimitFunctions : IConstraintFunctions<TwistLimitPrestepData, TwistLimitProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref TwistLimitPrestepData prestep, out TwistLimitProjection projection)
        {
            TwistServoFunctions.ComputeJacobian(bodies, bodyReferences, count, prestep.LocalBasisA, prestep.LocalBasisB,
                out var basisBX, out var basisBZ, out var basisA, out var jacobianA);

            TwistServoFunctions.ComputeCurrentAngle(basisBX, basisBZ, basisA, out var angle);

            //For simplicity, the solve iterations can only apply a positive impulse. So, the jacobians get flipped when necessary to make that consistent.
            //To figure out which way to flip, take the angular distance from minimum to current angle, and maximum to current angle.
            MathHelper.GetSignedAngleDifference(prestep.MinimumAngle, angle, out var minError);
            MathHelper.GetSignedAngleDifference(prestep.MaximumAngle, angle, out var maxError);
            var useMin = Vector.LessThan(Vector.Abs(minError), Vector.Abs(maxError));

            //If we use the maximum bound, flip the jacobian.
            var error = Vector.ConditionalSelect(useMin, -minError, maxError);
            Vector3Wide.Negate(jacobianA, out var negatedJacobianA);
            Vector3Wide.ConditionalSelect(useMin, negatedJacobianA, jacobianA, out jacobianA);

            TwistServoFunctions.ComputeEffectiveMass(dt, prestep.SpringSettings, inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, jacobianA,
                ref projection.ImpulseToVelocityA, ref projection.NegatedImpulseToVelocityB,
                out var positionErrorToVelocity, out projection.SoftnessImpulseScale, out var effectiveMass, out projection.VelocityToImpulseA);

            //In the speculative case, allow the limit to be approached.
            var biasVelocity = Vector.ConditionalSelect(Vector.LessThan(error, Vector<float>.Zero), error * inverseDt, error * positionErrorToVelocity);
            projection.BiasImpulse = biasVelocity * effectiveMass;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref TwistLimitProjection projection, ref Vector<float> accumulatedImpulse)
        {
            TwistServoFunctions.ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref TwistLimitProjection projection, ref Vector<float> accumulatedImpulse)
        {
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var netVelocity);
            Vector3Wide.Dot(netVelocity, projection.VelocityToImpulseA, out var csiVelocityComponent);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiVelocityComponent;
            var previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse + csi);
            csi = accumulatedImpulse - previousAccumulatedImpulse;

            TwistServoFunctions.ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection.ImpulseToVelocityA, projection.NegatedImpulseToVelocityB, csi);
        }

    }

    public class TwistLimitTypeProcessor : TwoBodyTypeProcessor<TwistLimitPrestepData, TwistLimitProjection, Vector<float>, TwistLimitFunctions>
    {
        public const int BatchTypeId = 27;
    }
}

