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
    /// Constrains a point on body B to be on a line attached to body A.
    /// </summary>
    public struct PointOnLineServo : ITwoBodyConstraintDescription<PointOnLineServo>
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
        /// Direction of the line in the local space of body A.
        /// </summary>
        public Vector3 LocalDirection;
        /// <summary>
        /// Servo control parameters.
        /// </summary>
        public ServoSettings ServoSettings;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return PointOnLineServoTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(PointOnLineServoTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalDirection, nameof(PointOnLineServo), nameof(LocalDirection));
            ConstraintChecker.AssertValid(ServoSettings, SpringSettings, nameof(PointOnLineServo));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<PointOnLineServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(LocalDirection, ref target.LocalDirection);
            ServoSettingsWide.WriteFirst(ServoSettings, ref target.ServoSettings);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out PointOnLineServo description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<PointOnLineServoPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            Vector3Wide.ReadFirst(source.LocalDirection, out description.LocalDirection);
            ServoSettingsWide.ReadFirst(source.ServoSettings, out description.ServoSettings);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct PointOnLineServoPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public Vector3Wide LocalDirection;
        public ServoSettingsWide ServoSettings;
        public SpringSettingsWide SpringSettings;
    }

    public struct PointOnLineServoFunctions : ITwoBodyConstraintFunctions<PointOnLineServoPrestepData, Vector2Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB,
            in Matrix2x3Wide linearJacobian, in Matrix2x3Wide angularJacobianA, in Matrix2x3Wide angularJacobianB, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, ref Vector2Wide csi)
        {
            Matrix2x3Wide.Transform(csi, linearJacobian, out var linearImpulseA);
            Matrix2x3Wide.Transform(csi, angularJacobianA, out var angularImpulseA);
            Matrix2x3Wide.Transform(csi, angularJacobianB, out var angularImpulseB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, inertiaA.InverseInertiaTensor, out var angularChangeA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseB, inertiaB.InverseInertiaTensor, out var angularChangeB);
            Vector3Wide.Scale(linearImpulseA, inertiaA.InverseMass, out var linearChangeA);
            Vector3Wide.Scale(linearImpulseA, inertiaB.InverseMass, out var negatedLinearChangeB);

            Vector3Wide.Add(linearChangeA, velocityA.Linear, out velocityA.Linear);
            Vector3Wide.Add(angularChangeA, velocityA.Angular, out velocityA.Angular);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearChangeB, out velocityB.Linear);
            Vector3Wide.Add(angularChangeB, velocityB.Angular, out velocityB.Angular);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(in Vector3Wide ab, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector3Wide localDirection, in Vector3Wide localOffsetA, in Vector3Wide localOffsetB,
            out Vector3Wide anchorOffset, out Matrix2x3Wide linearJacobian, out Matrix2x3Wide angularJA, out Matrix2x3Wide angularJB)
        {
            Helpers.BuildOrthonormalBasis(localDirection, out var localTangentX, out var localTangentY);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA, orientationMatrixA, out var anchorA);
            QuaternionWide.TransformWithoutOverlap(localOffsetB, orientationB, out var offsetB);

            //Find offsetA by computing the closest point on the line to anchorB.
            Matrix3x3Wide.TransformWithoutOverlap(localDirection, orientationMatrixA, out var direction);
            Vector3Wide.Add(offsetB, ab, out var anchorB);
            Vector3Wide.Subtract(anchorB, anchorA, out anchorOffset);
            Vector3Wide.Dot(anchorOffset, direction, out var d);
            Vector3Wide.Scale(direction, d, out var lineStartToClosestPointOnLine);
            Vector3Wide.Add(lineStartToClosestPointOnLine, anchorA, out var offsetA);

            Matrix3x3Wide.TransformWithoutOverlap(localTangentX, orientationMatrixA, out linearJacobian.X);
            Matrix3x3Wide.TransformWithoutOverlap(localTangentY, orientationMatrixA, out linearJacobian.Y);

            Vector3Wide.CrossWithoutOverlap(offsetA, linearJacobian.X, out angularJA.X);
            Vector3Wide.CrossWithoutOverlap(offsetA, linearJacobian.Y, out angularJA.Y);
            Vector3Wide.CrossWithoutOverlap(linearJacobian.X, offsetB, out angularJB.X);
            Vector3Wide.CrossWithoutOverlap(linearJacobian.Y, offsetB, out angularJB.Y);
        }
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref PointOnLineServoPrestepData prestep, ref Vector2Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalDirection, prestep.LocalOffsetA, prestep.LocalOffsetB, out _, out var linearJacobian, out var angularJA, out var angularJB);
            ApplyImpulse(ref wsvA, ref wsvB, linearJacobian, angularJA, angularJB, inertiaA, inertiaB, ref accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref PointOnLineServoPrestepData prestep, ref Vector2Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //This constrains a point on B to a line attached to A. It works on two degrees of freedom at the same time; those are the tangent axes to the line direction.
            //The error is measured as closest offset from the line. In other words:
            //dot(closestPointOnLineToAnchorB - anchorB, t1) = 0
            //dot(closestPointOnLineToAnchorB - anchorB, t2) = 0
            //where closestPointOnLineToAnchorB = dot(anchorB - anchorA, lineDirection) * lineDirection + anchorA
            //For the purposes of this derivation, we'll treat t1, t2, and lineDirection as constant with respect to time.
            //In the following, offsetA from the center of A to the closestPointOnLineToAnchorB, and offsetB refers to the LocalOffsetB * orientationB.
            //dot(positionA + offsetA - (positionB + offsetB), t1) = 0
            //dot(positionA + offsetA - (positionB + offsetB), t2) = 0
            //Velocity constraint for t1:
            //dot(d/dt(positionA + offsetA - (positionB + offsetB), t1) + dot(positionA + offsetA - (positionB + offsetB), d/dt(t1)) = 0
            //Treat d/dt(t1) as constant:
            //dot(d/dt(positionA + offsetA - (positionB + offsetB), t1) = 0
            //dot(linearA + angularA x offsetA - linearB - angularB x offsetB, t1) = 0
            //dot(linearA, t1) + dot(angularA x offsetA, t1) + dot(linearB, -t1) + dot(offsetB x angularB, t1) = 0
            //dot(linearA, t1) + dot(offsetA x t1, angularA) + dot(linearB, -t1) + dot(t1 x offsetB, angularB) = 0
            //Following the same pattern for the second degree of freedom, the jacobians are:
            //linearA: t1, t2
            //angularA: offsetA x t1, offsetA x t2
            //linearB: -t1, -t2
            //angularB: t1 x offsetB, t2 x offsetB
            ComputeJacobians(positionB - positionA, orientationA, orientationB, prestep.LocalDirection, prestep.LocalOffsetA, prestep.LocalOffsetB, out var anchorOffset, out var linearJacobian, out var angularJA, out var angularJB);
            Symmetric2x2Wide.SandwichScale(linearJacobian, inertiaA.InverseMass + inertiaB.InverseMass, out var linearContribution);
            Symmetric3x3Wide.MatrixSandwich(angularJA, inertiaA.InverseInertiaTensor, out var angularContributionA);
            Symmetric3x3Wide.MatrixSandwich(angularJB, inertiaB.InverseInertiaTensor, out var angularContributionB);
            Symmetric2x2Wide.Add(angularContributionA, angularContributionB, out var inverseEffectiveMass);
            Symmetric2x2Wide.Add(inverseEffectiveMass, linearContribution, out inverseEffectiveMass);

            Symmetric2x2Wide.InvertWithoutOverlap(inverseEffectiveMass, out var effectiveMass);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            Symmetric2x2Wide.Scale(effectiveMass, effectiveMassCFMScale, out effectiveMass);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Linear, linearJacobian, out var linearCSVA);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvB.Linear, linearJacobian, out var negatedLinearCSVB);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Angular, angularJA, out var angularCSVA);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvB.Angular, angularJB, out var angularCSVB);
            Vector2Wide.Subtract(linearCSVA, negatedLinearCSVB, out var linearCSV);
            Vector2Wide.Add(angularCSVA, angularCSVB, out var angularCSV);
            Vector2Wide.Add(linearCSV, angularCSV, out var csv);
            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector2Wide error;
            Vector3Wide.Dot(anchorOffset, linearJacobian.X, out error.X);
            Vector3Wide.Dot(anchorOffset, linearJacobian.Y, out error.Y);
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out var biasVelocity, out var maximumImpulse);
            Vector2Wide.Subtract(biasVelocity, csv, out csv);
            Symmetric2x2Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
            Vector2Wide.Scale(accumulatedImpulses, softnessImpulseScale, out var softnessContribution);
            Vector2Wide.Subtract(csi, softnessContribution, out csi);
            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulses, ref csi);
            ApplyImpulse(ref wsvA, ref wsvB, linearJacobian, angularJA, angularJB, inertiaA, inertiaB, ref csi);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref PointOnLineServoPrestepData prestepData) { }
    }

    public class PointOnLineServoTypeProcessor : TwoBodyTypeProcessor<PointOnLineServoPrestepData, Vector2Wide, PointOnLineServoFunctions, AccessAll, AccessAll, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 37;
    }
}
