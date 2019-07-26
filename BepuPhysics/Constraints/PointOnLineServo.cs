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
    /// Constrains points on two bodies to be on a line defined in the local space of one of the bodies.
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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return PointOnLineServoTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(PointOnLineServoTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out PointOnLineServo description)
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

    public struct PointOnLineServoProjection
    {
        public Matrix2x3Wide LinearJacobian;
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector2Wide BiasVelocity;
        public Symmetric2x2Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
    }

    public struct PointOnLineServoFunctions : IConstraintFunctions<PointOnLineServoPrestepData, PointOnLineServoProjection, Vector2Wide>
    {
        static void GetAngularJacobians(in Matrix2x3Wide linearJacobians, in Vector3Wide offsetA, in Vector3Wide offsetB, out Matrix2x3Wide angularJacobianA, out Matrix2x3Wide angularJacobianB)
        {
            Vector3Wide.CrossWithoutOverlap(offsetA, linearJacobians.X, out angularJacobianA.X);
            Vector3Wide.CrossWithoutOverlap(offsetA, linearJacobians.Y, out angularJacobianA.Y);
            Vector3Wide.CrossWithoutOverlap(linearJacobians.X, offsetB, out angularJacobianB.X);
            Vector3Wide.CrossWithoutOverlap(linearJacobians.Y, offsetB, out angularJacobianB.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref PointOnLineServoPrestepData prestep, out PointOnLineServoProjection projection)
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

            //Options for storage:
            //1) Reconstruct from direction and the two offsets. Still stores effective mass and inertia tensors (3 + 14 scalars), but requires only 9 scalars for all jacobians.
            //2) Store t1, t2, and the two offsets. Saves reconstruction ALU cost but increases storage cost relative to #1 by 3 scalars.
            //3) Store JT * Me and J * I^-1. Requires 3 * 6 scalars for JT * Me and 3 * 8 scalars for J * I^-1.
            //Memory bandwidth is the primary target, so #1 is attractive. However, recomputing the tangent basis based on the world direction would be unreliable unless
            //additional information was stored because there is no unique basis for a single direction. This is less of a concern when building the basis off the local direction
            //because it isn't expected to change frequently.
            //Instead, we'll go with #2.
            //#3 would be the fastest on a single core by virtue of requiring significantly less ALU work, but it requires more memory bandwidth.

            bodies.GatherPose(ref bodyReferences, count, out var ab, out var orientationA, out var orientationB);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationMatrixA, out var anchorA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out projection.OffsetB);

            //Find offsetA by computing the closest point on the line to anchorB.
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalDirection, orientationMatrixA, out var direction);
            Vector3Wide.Add(projection.OffsetB, ab, out var anchorB);
            Vector3Wide.Subtract(anchorB, anchorA, out var anchorOffset);
            Vector3Wide.Dot(anchorOffset, direction, out var d);
            Vector3Wide.Scale(direction, d, out var lineStartToClosestPointOnLine);
            Vector3Wide.Add(lineStartToClosestPointOnLine, anchorA, out projection.OffsetA);

            //Note again that the basis is created in local space to avoid rapidly changing jacobians.
            Helpers.BuildOrthnormalBasis(prestep.LocalDirection, out var localTangentX, out var localTangentY);
            Matrix3x3Wide.TransformWithoutOverlap(localTangentX, orientationMatrixA, out projection.LinearJacobian.X);
            Matrix3x3Wide.TransformWithoutOverlap(localTangentY, orientationMatrixA, out projection.LinearJacobian.Y);
            GetAngularJacobians(projection.LinearJacobian, projection.OffsetA, projection.OffsetB, out var angularJacobianA, out var angularJacobianB);
            Symmetric2x2Wide.SandwichScale(projection.LinearJacobian, inertiaA.InverseMass + inertiaB.InverseMass, out var linearContribution);
            Symmetric3x3Wide.MatrixSandwich(angularJacobianA, inertiaA.InverseInertiaTensor, out var angularContributionA);
            Symmetric3x3Wide.MatrixSandwich(angularJacobianB, inertiaB.InverseInertiaTensor, out var angularContributionB);
            Symmetric2x2Wide.Add(angularContributionA, angularContributionB, out var inverseEffectiveMass);
            Symmetric2x2Wide.Add(inverseEffectiveMass, linearContribution, out inverseEffectiveMass);

            Symmetric2x2Wide.InvertWithoutOverlap(inverseEffectiveMass, out projection.EffectiveMass);
            projection.InertiaA = inertiaA;
            projection.InertiaB = inertiaB;

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Symmetric2x2Wide.Scale(projection.EffectiveMass, effectiveMassCFMScale, out projection.EffectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector2Wide error;
            Vector3Wide.Dot(anchorOffset, projection.LinearJacobian.X, out error.X);
            Vector3Wide.Dot(anchorOffset, projection.LinearJacobian.Y, out error.Y);
            ServoSettingsWide.ComputeClampedBiasVelocity(error, positionErrorToVelocity, prestep.ServoSettings, dt, inverseDt, out projection.BiasVelocity, out projection.MaximumImpulse);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, 
            in Matrix2x3Wide linearJacobian, in Matrix2x3Wide angularJacobianA, in Matrix2x3Wide angularJacobianB, in BodyInertias inertiaA, in BodyInertias inertiaB, ref Vector2Wide csi)
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
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref PointOnLineServoProjection projection, ref Vector2Wide accumulatedImpulse)
        {
            GetAngularJacobians(projection.LinearJacobian, projection.OffsetA, projection.OffsetB, out var angularA, out var angularB);
            ApplyImpulse(ref velocityA, ref velocityB, projection.LinearJacobian, angularA, angularB, projection.InertiaA, projection.InertiaB, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref PointOnLineServoProjection projection, ref Vector2Wide accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            GetAngularJacobians(projection.LinearJacobian, projection.OffsetA, projection.OffsetB, out var angularA, out var angularB);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(velocityA.Linear, projection.LinearJacobian, out var linearCSVA);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(velocityB.Linear, projection.LinearJacobian, out var negatedLinearCSVB);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(velocityA.Angular, angularA, out var angularCSVA);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(velocityB.Angular, angularB, out var angularCSVB);
            Vector2Wide.Subtract(linearCSVA, negatedLinearCSVB, out var linearCSV);
            Vector2Wide.Add(angularCSVA, angularCSVB, out var angularCSV);
            Vector2Wide.Add(linearCSV, angularCSV, out var csv);
            Vector2Wide.Subtract(projection.BiasVelocity, csv, out csv);
            Symmetric2x2Wide.TransformWithoutOverlap(csv, projection.EffectiveMass, out var csi);
            Vector2Wide.Scale(accumulatedImpulse, projection.SoftnessImpulseScale, out var softnessContribution);
            Vector2Wide.Subtract(csi, softnessContribution, out csi);
            ServoSettingsWide.ClampImpulse(projection.MaximumImpulse, ref accumulatedImpulse, ref csi);
            ApplyImpulse(ref velocityA, ref velocityB, projection.LinearJacobian, angularA, angularB, projection.InertiaA, projection.InertiaB, ref csi);
        }

    }

    public class PointOnLineServoTypeProcessor : TwoBodyTypeProcessor<PointOnLineServoPrestepData, PointOnLineServoProjection, Vector2Wide, PointOnLineServoFunctions>
    {
        public const int BatchTypeId = 37;
    }
}
