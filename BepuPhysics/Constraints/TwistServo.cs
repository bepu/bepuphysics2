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
    /// Constrains two bodies to maintain a target twist angle around body-attached axes.
    /// </summary>
    public struct TwistServo : ITwoBodyConstraintDescription<TwistServo>
    {
        /// <summary>
        /// Local space basis attached to body A against which to measure body B's transformed axis. Expressed as a 3x3 rotation matrix, the X axis corresponds with 0 degrees, 
        /// the Y axis corresponds to 90 degrees, and the -Z axis is the twist axis. When viewed along the twist axis, positive change in angle causes counterclockwise rotation in right handed coordinates.
        /// </summary>
        public Quaternion LocalBasisA;
        /// <summary>
        /// Local space basis attached to body B that will be measured against body A's basis.
        /// Expressed as a 3x3 rotation matrix, the transformed X axis will be measured against A's X and Y axes. The Z axis is the twist axis.
        /// </summary>
        public Quaternion LocalBasisB;
        /// <summary>
        /// Target angle between B's axis to measure and A's measurement axis. 
        /// </summary>
        public float TargetAngle;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;
        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return TwistServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(TwistServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalBasisA, nameof(TwistServo), nameof(LocalBasisA));
            ConstraintChecker.AssertUnitLength(LocalBasisB, nameof(TwistServo), nameof(LocalBasisB));
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(TwistServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<TwistServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            QuaternionWide.WriteFirst(LocalBasisA, ref target.LocalBasisA);
            QuaternionWide.WriteFirst(LocalBasisB, ref target.LocalBasisB);
            GetFirst(ref target.TargetAngle) = TargetAngle;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out TwistServo description)
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

    public struct TwistServoFunctions : ITwoBodyConstraintFunctions<TwistServoPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobian(in QuaternionWide orientationA, in QuaternionWide orientationB, in QuaternionWide localBasisA, in QuaternionWide localBasisB,
            out Vector3Wide basisBX, out Vector3Wide basisBZ, out Matrix3x3Wide basisA, out Vector3Wide jacobianA)
        {
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

            QuaternionWide.ConcatenateWithoutOverlap(localBasisA, orientationA, out var basisQuaternionA);
            QuaternionWide.ConcatenateWithoutOverlap(localBasisB, orientationB, out var basisQuaternionB);

            QuaternionWide.TransformUnitXZ(basisQuaternionB, out basisBX, out basisBZ);
            Matrix3x3Wide.CreateFromQuaternion(basisQuaternionA, out basisA);
            //Protect against singularity when the axes point at each other.
            Vector3Wide.Add(basisA.Z, basisBZ, out jacobianA);
            Vector3Wide.Length(jacobianA, out var length);
            Vector3Wide.Scale(jacobianA, Vector<float>.One / length, out jacobianA);
            Vector3Wide.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-10f)), basisA.Z, jacobianA, out jacobianA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCurrentAngle(in Vector3Wide basisBX, in Vector3Wide basisBZ, in Matrix3x3Wide basisA, out Vector<float> angle)
        {
            //Compute the position error and bias velocities.
            //Now we just have the slight annoyance that our error function contains inverse trigonometry.
            //We'll just use:
            //atan(dot(alignedBasisBX, basisAX), dot(alignedBasisBX, basisAY)) = 
            //sign(dot(alignedBasisBX, basisAY)) * acos(dot(alignedBasisBX, basisAX))
            QuaternionWide.GetQuaternionBetweenNormalizedVectors(basisBZ, basisA.Z, out var aligningRotation);
            QuaternionWide.TransformWithoutOverlap(basisBX, aligningRotation, out var alignedBasisBX);
            Vector3Wide.Dot(alignedBasisBX, basisA.X, out var x);
            Vector3Wide.Dot(alignedBasisBX, basisA.Y, out var y);
            var absAngle = MathHelper.Acos(x);
            angle = Vector.ConditionalSelect(Vector.LessThan(y, Vector<float>.Zero), -absAngle, absAngle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeEffectiveMassContributions(
           in Symmetric3x3Wide inverseInertiaA, in Symmetric3x3Wide inverseInertiaB, in Vector3Wide jacobianA,
           out Vector3Wide impulseToVelocityA, out Vector3Wide negatedImpulseToVelocityB, out Vector<float> unsoftenedInverseEffectiveMass)
        {
            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.
            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inverseInertiaA, out impulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inverseInertiaB, out negatedImpulseToVelocityB);
            Vector3Wide.Dot(impulseToVelocityA, jacobianA, out var angularA);
            Vector3Wide.Dot(negatedImpulseToVelocityB, jacobianA, out var angularB);
            unsoftenedInverseEffectiveMass = angularA + angularB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeEffectiveMass(float dt, in SpringSettingsWide springSettings,
            in Symmetric3x3Wide inverseInertiaA, in Symmetric3x3Wide inverseInertiaB, in Vector3Wide jacobianA,
            out Vector3Wide impulseToVelocityA, out Vector3Wide negatedImpulseToVelocityB, out Vector<float> positionErrorToVelocity, out Vector<float> softnessImpulseScale,
            out Vector<float> effectiveMass, out Vector3Wide velocityToImpulseA)
        {
            ComputeEffectiveMassContributions(inverseInertiaA, inverseInertiaB, jacobianA, out impulseToVelocityA, out negatedImpulseToVelocityB, out var unsoftenedInverseEffectiveMass);

            SpringSettingsWide.ComputeSpringiness(springSettings, dt, out positionErrorToVelocity, out var effectiveMassCFMScale, out softnessImpulseScale);
            effectiveMass = effectiveMassCFMScale / unsoftenedInverseEffectiveMass;
            Vector3Wide.Scale(jacobianA, effectiveMass, out velocityToImpulseA);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, in Vector3Wide impulseToVelocityA, in Vector3Wide negatedImpulseToVelocityB, in Vector<float> csi)
        {
            Vector3Wide.Scale(impulseToVelocityA, csi, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Vector3Wide.Scale(negatedImpulseToVelocityB, csi, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobian(in QuaternionWide orientationA, in QuaternionWide orientationB, in QuaternionWide localBasisA, in QuaternionWide localBasisB, out Vector3Wide jacobianA)
        {
            QuaternionWide.ConcatenateWithoutOverlap(localBasisA, orientationA, out var basisQuaternionA);
            QuaternionWide.ConcatenateWithoutOverlap(localBasisB, orientationB, out var basisQuaternionB);

            var basisAZ = QuaternionWide.TransformUnitZ(basisQuaternionA);
            var basisBZ = QuaternionWide.TransformUnitZ(basisQuaternionB);
            //Protect against singularity when the axes point at each other.
            Vector3Wide.Add(basisAZ, basisBZ, out jacobianA);
            Vector3Wide.Length(jacobianA, out var length);
            Vector3Wide.Scale(jacobianA, Vector<float>.One / length, out jacobianA);
            Vector3Wide.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-10f)), basisAZ, jacobianA, out jacobianA);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref TwistServoPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(orientationA, orientationB, prestep.LocalBasisA, prestep.LocalBasisB, out var jacobianA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            ApplyImpulse(ref wsvA.Angular, ref wsvB.Angular, impulseToVelocityA, negatedImpulseToVelocityB, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref TwistServoPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(orientationA, orientationB, prestep.LocalBasisA, prestep.LocalBasisB,
                out var basisBX, out var basisBZ, out var basisA, out var jacobianA);

            ComputeEffectiveMass(dt, prestep.SpringSettings, inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, jacobianA,
                out var impulseToVelocityA, out var negatedImpulseToVelocityB,
                out var positionErrorToVelocity, out var softnessImpulseScale, out var effectiveMass, out var velocityToImpulseA);

            ComputeCurrentAngle(basisBX, basisBZ, basisA, out var angle);

            MathHelper.GetSignedAngleDifference(prestep.TargetAngle, angle, out var error);

            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var clampedBiasVelocity, out var maximumImpulse);
            var biasImpulse = clampedBiasVelocity * effectiveMass;

            Vector3Wide.Subtract(wsvA.Angular, wsvB.Angular, out var netVelocity);
            Vector3Wide.Dot(netVelocity, velocityToImpulseA, out var csiVelocityComponent);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = biasImpulse - accumulatedImpulses * softnessImpulseScale - csiVelocityComponent;
            var previousAccumulatedImpulse = accumulatedImpulses;
            accumulatedImpulses = Vector.Min(Vector.Max(accumulatedImpulses + csi, -maximumImpulse), maximumImpulse);
            csi = accumulatedImpulses - previousAccumulatedImpulse;

            ApplyImpulse(ref wsvA.Angular, ref wsvB.Angular, impulseToVelocityA, negatedImpulseToVelocityB, csi);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref TwistServoPrestepData prestepData) { }
    }

    public class TwistServoTypeProcessor : TwoBodyTypeProcessor<TwistServoPrestepData, Vector<float>, TwistServoFunctions, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 26;
    }
}

