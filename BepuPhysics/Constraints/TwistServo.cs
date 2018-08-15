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
    /// Description of a constraint which tries to maintain a target twist angle around an axis attached to each connected body.
    /// </summary>
    public struct TwistServo : IConstraintDescription<TwistServo>
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
        /// Target angle between B's axis to measure (X) and A's measurement axis (X). 
        /// </summary>
        public float TargetAngle;

        public SpringSettings SpringSettings;
        public ServoSettings ServoSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return TwistServoTypeProcessor.BatchTypeId;
            }
        }

        public Type BatchType => typeof(TwistServoTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TwistServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(LocalBasisA, ref target.LocalBasisA);
            QuaternionWide.WriteFirst(LocalBasisB, ref target.LocalBasisB);
            GetFirst(ref target.TargetAngle) = TargetAngle;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out TwistServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<TwistServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.ReadFirst(source.LocalBasisA, out description.LocalBasisA);
            QuaternionWide.ReadFirst(source.LocalBasisB, out description.LocalBasisB);
            description.TargetAngle = GetFirst(ref source.TargetAngle);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
        }
    }

    public struct TwistServoPrestepData
    {
        public QuaternionWide LocalBasisA;
        public QuaternionWide LocalBasisB;
        public Vector<float> TargetAngle;
        public SpringSettingsWide SpringSettings;
        public ServoSettingsWide ServoSettings;
    }

    public struct TwistServoProjection
    {
        public Vector3Wide VelocityToImpulseA;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public Vector3Wide ImpulseToVelocityA;
        public Vector3Wide NegatedImpulseToVelocityB;
    }


    public struct TwistServoFunctions : IConstraintFunctions<TwistServoPrestepData, TwistServoProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref TwistServoPrestepData prestep,
            out TwistServoProjection projection)
        {
            bodies.GatherInertiaAndPose(ref bodyReferences, count,
                out var orientationA, out var orientationB,
                out var inertiaA, out var inertiaB);

            //Twist joints attempt to match rotation around each body's local axis.
            //We'll use a basis attached to each of the two bodies.
            //B's basis will be transformed into alignment with A's basis for measurement.
            //Conceptually, we'll use atan to perform that measurement:
            //C = atan(dot(alignedBasisB.X, basisA.X), dot(alignedBasisB.X, basisA.Y))
            //where alignedBasisB = basisB * ShortestRotationBetweenUnitVectors(basisB.Z, basisA.Z)
            //The full derivation is omitted; check the AngularHinge for a similar derivation.
            //After a lot of manipulation, everything drops down to angular jacobians equal to the twist axes (basisA.Z + basisB.Z).
            //TODO: Would be nice to actually have the derivation here. Secretly, I just handwaved this and referred to the v1 implementation without working it all the way through again.

            //Note that we build the tangents in local space first to avoid inconsistencies.

            QuaternionWide.ConcatenateWithoutOverlap(prestep.LocalBasisA, orientationA, out var basisQuaternionA);
            QuaternionWide.ConcatenateWithoutOverlap(prestep.LocalBasisB, orientationB, out var basisQuaternionB);

            QuaternionWide.TransformUnitXZ(basisQuaternionB, out var basisBX, out var basisBZ);
            Matrix3x3Wide.CreateFromQuaternion(basisQuaternionA, out var basisA);
            //Protect against singularity when the axes point at each other.
            Vector3Wide.Add(basisA.Z, basisBZ, out var jacobianA);
            Vector3Wide.Length(jacobianA, out var length);
            Vector3Wide.Scale(jacobianA, Vector<float>.One / length, out jacobianA);
            Vector3Wide.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-10f)), basisA.Z, jacobianA, out jacobianA);

            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.
            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Vector3Wide.Dot(projection.ImpulseToVelocityA, jacobianA, out var angularA);
            Vector3Wide.Dot(projection.NegatedImpulseToVelocityB, jacobianA, out var angularB);

            SpringSettingsWide.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var effectiveMass = effectiveMassCFMScale / (angularA + angularB);
            Vector3Wide.Scale(jacobianA, effectiveMass, out projection.VelocityToImpulseA);

            //Compute the position error and bias velocities.
            //Now we just have the slight annoyance that our error function contains inverse trigonometry.
            //We'll just use:
            //atan(dot(alignedBasisBX, basisAX), dot(alignedBasisBX, basisAY)) = 
            //sign(dot(alignedBasisBX, basisAY)) * acos(dot(alignedBasisBX, basisAX))
            QuaternionWide.GetQuaternionBetweenNormalizedVectors(basisBZ, basisA.Z, out var aligningRotation);
            QuaternionWide.TransformWithoutOverlap(basisBZ, aligningRotation, out var shouldBeBasisAZ);
            QuaternionWide.TransformWithoutOverlap(basisBX, aligningRotation, out var alignedBasisBX);
            Vector3Wide.Dot(alignedBasisBX, basisA.X, out var x);
            Vector3Wide.Dot(alignedBasisBX, basisA.Y, out var y);
            MathHelper.ApproximateAcos(x, out var absAngle);
            var angle = Vector.ConditionalSelect(Vector.LessThan(y, Vector<float>.Zero), -absAngle, absAngle);

            MathHelper.GetSignedAngleDifference(prestep.TargetAngle, angle, out var error);
            
            ServoSettingsWide.ClampBiasVelocity(error * positionErrorToVelocity, error, prestep.ServoSettings, inverseDt, out var clampedBiasVelocity);
            projection.BiasImpulse = clampedBiasVelocity * effectiveMass;
            projection.MaximumImpulse = prestep.ServoSettings.MaximumForce * dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, ref TwistServoProjection projection, ref Vector<float> csi)
        {
            Vector3Wide.Scale(projection.ImpulseToVelocityA, csi, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Vector3Wide.Scale(projection.NegatedImpulseToVelocityB, csi, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref TwistServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref TwistServoProjection projection, ref Vector<float> accumulatedImpulse)
        {
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var netVelocity);
            Vector3Wide.Dot(netVelocity, projection.VelocityToImpulseA, out var csiVelocityComponent);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiVelocityComponent;
            var previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = Vector.Min(Vector.Max(accumulatedImpulse + csi, -projection.MaximumImpulse), projection.MaximumImpulse);
            csi = accumulatedImpulse - previousAccumulatedImpulse;

            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref csi);
        }

    }

    public class TwistServoTypeProcessor : TwoBodyTypeProcessor<TwistServoPrestepData, TwistServoProjection, Vector<float>, TwistServoFunctions>
    {
        public const int BatchTypeId = 26;
    }
}

