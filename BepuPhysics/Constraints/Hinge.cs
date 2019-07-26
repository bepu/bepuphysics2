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
    /// Constrains two bodies with a hinge. Equivalent to a BallSocket constraint and an AngularHinge constraint solved together.
    /// </summary>
    public struct Hinge : ITwoBodyConstraintDescription<Hinge>
    {
        /// <summary>
        /// Local offset from the center of body A to its attachment point.
        /// </summary>
        public Vector3 LocalOffsetA;
        /// <summary>
        /// Hinge axis in the local space of A.
        /// </summary>
        public Vector3 LocalHingeAxisA;
        /// <summary>
        /// Local offset from the center of body B to its attachment point.
        /// </summary>
        public Vector3 LocalOffsetB;
        /// <summary>
        /// Hinge axis in the local space of B.
        /// </summary>
        public Vector3 LocalHingeAxisB;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return HingeTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(HingeTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalHingeAxisA, nameof(Hinge), nameof(LocalHingeAxisA));
            ConstraintChecker.AssertUnitLength(LocalHingeAxisB, nameof(Hinge), nameof(LocalHingeAxisB));
            ConstraintChecker.AssertValid(SpringSettings, nameof(Hinge));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<HingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalHingeAxisA, ref target.LocalHingeAxisA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(LocalHingeAxisB, ref target.LocalHingeAxisB);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Hinge description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<HingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalHingeAxisA, out description.LocalHingeAxisA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            Vector3Wide.ReadFirst(source.LocalHingeAxisB, out description.LocalHingeAxisB);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct HingePrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalHingeAxisA;
        public Vector3Wide LocalOffsetB;
        public Vector3Wide LocalHingeAxisB;
        public SpringSettingsWide SpringSettings;
    }

    public struct HingeProjection
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Matrix2x3Wide HingeJacobian;
        public Vector3Wide BallSocketBiasVelocity;
        public Vector2Wide HingeBiasVelocity;
        public Symmetric5x5Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
    }

    public struct HingeAccumulatedImpulses
    {
        public Vector3Wide BallSocket;
        public Vector2Wide Hinge;
    }

    public struct HingeFunctions : IConstraintFunctions<HingePrestepData, HingeProjection, HingeAccumulatedImpulses>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref HingePrestepData prestep, out HingeProjection projection)
        {
            bodies.GatherPose(ref bodyReferences, count, out var ab, out var orientationA, out var orientationB);
            projection.InertiaA = inertiaA;
            projection.InertiaB = inertiaB;

            //5x12 jacobians, from BallSocket and AngularHinge:
            //[ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //[ 0, constraintAxisAX, 0, -constraintAxisAX ]
            //[ 0, constraintAxisAY, 0, -constraintAxisAY ]

            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationMatrixA, out projection.OffsetA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalHingeAxisA, orientationMatrixA, out var hingeAxisA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationMatrixB, out projection.OffsetB);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalHingeAxisB, orientationMatrixB, out var hingeAxisB);
            Helpers.BuildOrthnormalBasis(prestep.LocalHingeAxisA, out var localAX, out var localAY);
            Matrix3x3Wide.TransformWithoutOverlap(localAX, orientationMatrixA, out projection.HingeJacobian.X);
            Matrix3x3Wide.TransformWithoutOverlap(localAY, orientationMatrixA, out projection.HingeJacobian.Y);

            //The upper left 3x3 block is just the ball socket.
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(projection.OffsetA, inertiaA.InverseInertiaTensor, out var ballSocketContributionAngularA);
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(projection.OffsetB, inertiaB.InverseInertiaTensor, out var ballSocketContributionAngularB);
            Symmetric5x5Wide inverseEffectiveMass;
            Symmetric3x3Wide.Add(ballSocketContributionAngularA, ballSocketContributionAngularB, out inverseEffectiveMass.A);
            var linearContribution = projection.InertiaA.InverseMass + projection.InertiaB.InverseMass;
            inverseEffectiveMass.A.XX += linearContribution;
            inverseEffectiveMass.A.YY += linearContribution;
            inverseEffectiveMass.A.ZZ += linearContribution;

            //The lower right 2x2 block is the AngularHinge.
            Symmetric3x3Wide.MultiplyWithoutOverlap(projection.HingeJacobian, inertiaA.InverseInertiaTensor, out var hingeInertiaA);
            Symmetric3x3Wide.MultiplyWithoutOverlap(projection.HingeJacobian, inertiaB.InverseInertiaTensor, out var hingeInertiaB);
            Symmetric2x2Wide.CompleteMatrixSandwich(hingeInertiaA, projection.HingeJacobian, out var hingeContributionAngularA);
            Symmetric2x2Wide.CompleteMatrixSandwich(hingeInertiaB, projection.HingeJacobian, out var hingeContributionAngularB);
            Symmetric2x2Wide.Add(hingeContributionAngularA, hingeContributionAngularB, out inverseEffectiveMass.D);

            //The remaining off-diagonal region is skew(offsetA) * Ia^-1 * hingeJacobianV + skew(offsetB) * Ib^-1 * hingeJacobianA
            //skew(offsetA) * (Ia^-1 * hingeJacobianA) = [ (Ia^-1 * hingeJacobianA.X) x offsetA ]
            //                                           [ (Ia^-1 * hingeJacobianA.Y) x offsetA ]
            //Careful with cross order/signs!
            Vector3Wide.CrossWithoutOverlap(hingeInertiaA.X, projection.OffsetA, out var offDiagonalContributionAX);
            Vector3Wide.CrossWithoutOverlap(hingeInertiaA.Y, projection.OffsetA, out var offDiagonalContributionAY);
            Vector3Wide.CrossWithoutOverlap(hingeInertiaB.X, projection.OffsetB, out var offDiagonalContributionBX);
            Vector3Wide.CrossWithoutOverlap(hingeInertiaB.Y, projection.OffsetB, out var offDiagonalContributionBY);
            Vector3Wide.Add(offDiagonalContributionAX, offDiagonalContributionBX, out inverseEffectiveMass.B.X);
            Vector3Wide.Add(offDiagonalContributionAY, offDiagonalContributionBY, out inverseEffectiveMass.B.Y);

            Symmetric5x5Wide.InvertWithoutOverlap(inverseEffectiveMass, out projection.EffectiveMass);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Symmetric5x5Wide.Scale(projection.EffectiveMass, effectiveMassCFMScale, out projection.EffectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(ab, projection.OffsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, projection.OffsetA, out var ballSocketError);
            Vector3Wide.Scale(ballSocketError, positionErrorToVelocity, out projection.BallSocketBiasVelocity);

            AngularHingeFunctions.GetErrorAngles(hingeAxisA, hingeAxisB, projection.HingeJacobian, out var errorAngles);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            Vector2Wide.Scale(errorAngles, -positionErrorToVelocity, out projection.HingeBiasVelocity);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref HingeProjection projection, ref HingeAccumulatedImpulses csi)
        {
            //[ csi ] * [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //          [ 0, constraintAxisAX, 0, -constraintAxisAX ]
            //          [ 0, constraintAxisAY, 0, -constraintAxisAY ]
            Vector3Wide.Scale(csi.BallSocket, projection.InertiaA.InverseMass, out var linearChangeA);
            Vector3Wide.Add(velocityA.Linear, linearChangeA, out velocityA.Linear);

            Vector3Wide.CrossWithoutOverlap(projection.OffsetA, csi.BallSocket, out var ballSocketAngularImpulseA);
            Matrix2x3Wide.Transform(csi.Hinge, projection.HingeJacobian, out var hingeAngularImpulseA);
            Vector3Wide.Add(ballSocketAngularImpulseA, hingeAngularImpulseA, out var angularImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, projection.InertiaA.InverseInertiaTensor, out var angularChangeA);
            Vector3Wide.Add(velocityA.Angular, angularChangeA, out velocityA.Angular);

            //Note cross order flip for negation.
            Vector3Wide.Scale(csi.BallSocket, projection.InertiaB.InverseMass, out var negatedLinearChangeB);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearChangeB, out velocityB.Linear);
            Vector3Wide.CrossWithoutOverlap(csi.BallSocket, projection.OffsetB, out var ballSocketAngularImpulseB);
            Vector3Wide.Subtract(ballSocketAngularImpulseB, hingeAngularImpulseA, out var angularImpulseB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseB, projection.InertiaB.InverseInertiaTensor, out var angularChangeB);
            Vector3Wide.Add(velocityB.Angular, angularChangeB, out velocityB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref HingeProjection projection, ref HingeAccumulatedImpulses accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref HingeProjection projection, ref HingeAccumulatedImpulses accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //    [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //J = [ 0, constraintAxisAX, 0, -constraintAxisAX ]
            //    [ 0, constraintAxisAY, 0, -constraintAxisAY ]
            Vector3Wide.CrossWithoutOverlap(velocityA.Angular, projection.OffsetA, out var ballSocketAngularCSVA);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(velocityA.Angular, projection.HingeJacobian, out var hingeCSVA);
            Vector3Wide.CrossWithoutOverlap(projection.OffsetB, velocityB.Angular, out var ballSocketAngularCSVB);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(velocityB.Angular, projection.HingeJacobian, out var negatedHingeCSVB);

            Vector3Wide.Add(ballSocketAngularCSVA, ballSocketAngularCSVB, out var ballSocketAngularCSV);
            Vector3Wide.Subtract(velocityA.Linear, velocityB.Linear, out var ballSocketLinearCSV);
            Vector3Wide.Add(ballSocketAngularCSV, ballSocketLinearCSV, out var ballSocketCSV);
            Vector3Wide.Subtract(projection.BallSocketBiasVelocity, ballSocketCSV, out ballSocketCSV);
            Vector2Wide.Subtract(hingeCSVA, negatedHingeCSVB, out var hingeCSV);
            Vector2Wide.Subtract(projection.HingeBiasVelocity, hingeCSV, out hingeCSV);

            HingeAccumulatedImpulses csi;
            Symmetric5x5Wide.TransformWithoutOverlap(ballSocketCSV, hingeCSV, projection.EffectiveMass, out csi.BallSocket, out csi.Hinge);
            Vector3Wide.Scale(accumulatedImpulse.BallSocket, projection.SoftnessImpulseScale, out var ballSocketSoftnessContribution);
            Vector3Wide.Subtract(csi.BallSocket, ballSocketSoftnessContribution, out csi.BallSocket);
            Vector2Wide.Scale(accumulatedImpulse.Hinge, projection.SoftnessImpulseScale, out var hingeSoftnessContribution);
            Vector2Wide.Subtract(csi.Hinge, hingeSoftnessContribution, out csi.Hinge);

            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref csi);
        }

    }

    public class HingeTypeProcessor : TwoBodyTypeProcessor<HingePrestepData, HingeProjection, HingeAccumulatedImpulses, HingeFunctions>
    {
        public const int BatchTypeId = 47;
    }
}
