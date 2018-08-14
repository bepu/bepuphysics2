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
        /// Target angle between B's axis to measure (X) and A's measurement axis (X). Psoit 
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
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
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
        public Vector3Wide NegatedVelocityToImpulseB;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
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
            //After a lot of manipulation, everything drops down to angular jacobians equal to the twist axes (basisA.Z and -basisB.Z).

            //One key note is that we treat the aligning transform ShortestRotationBetweenUnitVectors(basisB.Z, basisA.Z) as constant.
            //It technically isn't, and that can cause some missed velocity when the objects are moving around non-twist axes. 
            //But v1 used this formulation for years and no one seemed to complain. Position correction will still work as expected, so the errors will be corrected reasonably quickly.
            //As a bonus, it's very cheap.

            //Note that we build the tangents in local space first to avoid inconsistencies.

            QuaternionWide.ConcatenateWithoutOverlap(prestep.LocalBasisA, orientationA, out var basisQuaternionA);
            QuaternionWide.ConcatenateWithoutOverlap(prestep.LocalBasisB, orientationB, out var basisQuaternionB);

            QuaternionWide.TransformUnitXZ(basisQuaternionB, out var basisBX, out var negatedAngularJacobianB);
            Matrix3x3Wide.CreateFromQuaternion(basisQuaternionA, out var basisA);
            
            //Note that JB = -basisB.Z, but for the purposes of calculating the effective mass the sign is irrelevant.
            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Symmetric3x3Wide.TransformWithoutOverlap(basisA.Z, inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(negatedAngularJacobianB, inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Vector3Wide.Dot(projection.ImpulseToVelocityA, basisA.Z, out var angularA);
            Vector3Wide.Dot(projection.NegatedImpulseToVelocityB, negatedAngularJacobianB, out var angularB);

            SpringSettingsWide.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var effectiveMass = effectiveMassCFMScale / (angularA + angularB);
            Vector3Wide.Scale(basisA.Z, effectiveMass, out projection.VelocityToImpulseA);
            Vector3Wide.Scale(negatedAngularJacobianB, effectiveMass, out projection.NegatedVelocityToImpulseB);

            //Compute the position error and bias velocities.
            //Now we just have the slight annoyance that our error function contains inverse trigonometry.
            //We'll just use:
            //atan(dot(alignedBasisBX, basisAX), dot(alignedBasisBX, basisAY)) = 
            //sign(dot(alignedBasisBX, basisAY)) * acos(dot(alignedBasisBX, basisAX))
            QuaternionWide.GetQuaternionBetweenNormalizedVectors(basisA.Z, negatedAngularJacobianB, out var aligningRotation);
            QuaternionWide.TransformWithoutOverlap(basisBX, aligningRotation, out var alignedBasisBX);
            Vector3Wide.Dot(alignedBasisBX, basisA.X, out var x);
            Vector3Wide.Dot(alignedBasisBX, basisA.Y, out var y);
            MathHelper.ApproximateAcos(x, out var absAngle);
            var angle = Vector.ConditionalSelect(Vector.LessThan(y, Vector<float>.Zero), -absAngle, absAngle);

            MathHelper.GetSignedAngleDifference(angle, prestep.TargetAngle, out var error); 
            
            projection.BiasImpulse = error* positionErrorToVelocity * effectiveMass;

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
            Vector3Wide.Dot(velocityA.Angular, projection.VelocityToImpulseA, out var csiA);
            Vector3Wide.Dot(velocityB.Angular, projection.NegatedVelocityToImpulseB, out var negatedCSIB);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csiA + negatedCSIB;
            accumulatedImpulse += csi;
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref csi);
        }

    }

    public class TwistServoTypeProcessor : TwoBodyTypeProcessor<TwistServoPrestepData, TwistServoProjection, Vector<float>, TwistServoFunctions>
    {
        public const int BatchTypeId = 25;
    }
}

