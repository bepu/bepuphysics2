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

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return HingeTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(HingeTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Hinge description)
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

    public struct HingeAccumulatedImpulses
    {
        public Vector3Wide BallSocket;
        public Vector2Wide Hinge;
    }

    public struct HingeFunctions : ITwoBodyConstraintFunctions<HingePrestepData, HingeAccumulatedImpulses>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(in Vector3Wide offsetA, in Vector3Wide offsetB, in Matrix2x3Wide hingeJacobian, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, in HingeAccumulatedImpulses csi,
            ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB)
        {
            //[ csi ] * [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //          [ 0, constraintAxisAX, 0, -constraintAxisAX ]
            //          [ 0, constraintAxisAY, 0, -constraintAxisAY ]
            Vector3Wide.Scale(csi.BallSocket, inertiaA.InverseMass, out var linearChangeA);
            Vector3Wide.Add(velocityA.Linear, linearChangeA, out velocityA.Linear);

            Vector3Wide.CrossWithoutOverlap(offsetA, csi.BallSocket, out var ballSocketAngularImpulseA);
            Matrix2x3Wide.Transform(csi.Hinge, hingeJacobian, out var hingeAngularImpulseA);
            Vector3Wide.Add(ballSocketAngularImpulseA, hingeAngularImpulseA, out var angularImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, inertiaA.InverseInertiaTensor, out var angularChangeA);
            Vector3Wide.Add(velocityA.Angular, angularChangeA, out velocityA.Angular);

            //Note cross order flip for negation.
            Vector3Wide.Scale(csi.BallSocket, inertiaB.InverseMass, out var negatedLinearChangeB);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearChangeB, out velocityB.Linear);
            Vector3Wide.CrossWithoutOverlap(csi.BallSocket, offsetB, out var ballSocketAngularImpulseB);
            Vector3Wide.Subtract(ballSocketAngularImpulseB, hingeAngularImpulseA, out var angularImpulseB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseB, inertiaB.InverseInertiaTensor, out var angularChangeB);
            Vector3Wide.Add(velocityB.Angular, angularChangeB, out velocityB.Angular);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref HingePrestepData prestep, ref HingeAccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationMatrixA, out var offsetA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out var offsetB);
            Helpers.BuildOrthonormalBasis(prestep.LocalHingeAxisA, out var localAX, out var localAY);
            Matrix2x3Wide hingeJacobian;
            Matrix3x3Wide.TransformWithoutOverlap(localAX, orientationMatrixA, out hingeJacobian.X);
            Matrix3x3Wide.TransformWithoutOverlap(localAY, orientationMatrixA, out hingeJacobian.Y);
            ApplyImpulse(offsetA, offsetB, hingeJacobian, inertiaA, inertiaB, accumulatedImpulses, ref wsvA, ref wsvB);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref HingePrestepData prestep, ref HingeAccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //5x12 jacobians, from BallSocket and AngularHinge:
            //[ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //[ 0, constraintAxisAX, 0, -constraintAxisAX ]
            //[ 0, constraintAxisAY, 0, -constraintAxisAY ]

            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationMatrixA, out var offsetA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalHingeAxisA, orientationMatrixA, out var hingeAxisA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationMatrixB, out var offsetB);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalHingeAxisB, orientationMatrixB, out var hingeAxisB);
            Helpers.BuildOrthonormalBasis(prestep.LocalHingeAxisA, out var localAX, out var localAY);
            Matrix2x3Wide hingeJacobian;
            Matrix3x3Wide.TransformWithoutOverlap(localAX, orientationMatrixA, out hingeJacobian.X);
            Matrix3x3Wide.TransformWithoutOverlap(localAY, orientationMatrixA, out hingeJacobian.Y);

            //The upper left 3x3 block is just the ball socket.
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offsetA, inertiaA.InverseInertiaTensor, out var ballSocketContributionAngularA);
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offsetB, inertiaB.InverseInertiaTensor, out var ballSocketContributionAngularB);
            Symmetric5x5Wide inverseEffectiveMass;
            Symmetric3x3Wide.Add(ballSocketContributionAngularA, ballSocketContributionAngularB, out inverseEffectiveMass.A);
            var linearContribution = inertiaA.InverseMass + inertiaB.InverseMass;
            inverseEffectiveMass.A.XX += linearContribution;
            inverseEffectiveMass.A.YY += linearContribution;
            inverseEffectiveMass.A.ZZ += linearContribution;

            //The lower right 2x2 block is the AngularHinge.
            Symmetric3x3Wide.MultiplyWithoutOverlap(hingeJacobian, inertiaA.InverseInertiaTensor, out var hingeInertiaA);
            Symmetric3x3Wide.MultiplyWithoutOverlap(hingeJacobian, inertiaB.InverseInertiaTensor, out var hingeInertiaB);
            Symmetric2x2Wide.CompleteMatrixSandwich(hingeInertiaA, hingeJacobian, out var hingeContributionAngularA);
            Symmetric2x2Wide.CompleteMatrixSandwich(hingeInertiaB, hingeJacobian, out var hingeContributionAngularB);
            Symmetric2x2Wide.Add(hingeContributionAngularA, hingeContributionAngularB, out inverseEffectiveMass.D);

            //The remaining off-diagonal region is skew(offsetA) * Ia^-1 * hingeJacobianV + skew(offsetB) * Ib^-1 * hingeJacobianA
            //skew(offsetA) * (Ia^-1 * hingeJacobianA) = [ (Ia^-1 * hingeJacobianA.X) x offsetA ]
            //                                           [ (Ia^-1 * hingeJacobianA.Y) x offsetA ]
            //Careful with cross order/signs!
            Vector3Wide.CrossWithoutOverlap(hingeInertiaA.X, offsetA, out var offDiagonalContributionAX);
            Vector3Wide.CrossWithoutOverlap(hingeInertiaA.Y, offsetA, out var offDiagonalContributionAY);
            Vector3Wide.CrossWithoutOverlap(hingeInertiaB.X, offsetB, out var offDiagonalContributionBX);
            Vector3Wide.CrossWithoutOverlap(hingeInertiaB.Y, offsetB, out var offDiagonalContributionBY);
            Vector3Wide.Add(offDiagonalContributionAX, offDiagonalContributionBX, out inverseEffectiveMass.B.X);
            Vector3Wide.Add(offDiagonalContributionAY, offDiagonalContributionBY, out inverseEffectiveMass.B.Y);

            //TODO: Could consider an LDLT solve here. Helped a little bit in Weld; probably would still be worth it for a 5x5.
            Symmetric5x5Wide.InvertWithoutOverlap(inverseEffectiveMass, out var effectiveMass);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            //Note that the effective mass is *not* scaled by the effectiveMassCFMScale here; instead, we scale the impulse later.

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(positionB - positionA, offsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, offsetA, out var ballSocketError);
            Vector3Wide.Scale(ballSocketError, positionErrorToVelocity, out var ballSocketBiasVelocity);

            AngularHingeFunctions.GetErrorAngles(hingeAxisA, hingeAxisB, hingeJacobian, out var errorAngles);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            Vector2Wide.Scale(errorAngles, -positionErrorToVelocity, out var hingeBiasVelocity);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //    [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //J = [ 0, constraintAxisAX, 0, -constraintAxisAX ]
            //    [ 0, constraintAxisAY, 0, -constraintAxisAY ]
            Vector3Wide.CrossWithoutOverlap(wsvA.Angular, offsetA, out var ballSocketAngularCSVA);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Angular, hingeJacobian, out var hingeCSVA);
            Vector3Wide.CrossWithoutOverlap(offsetB, wsvB.Angular, out var ballSocketAngularCSVB);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvB.Angular, hingeJacobian, out var negatedHingeCSVB);

            Vector3Wide.Add(ballSocketAngularCSVA, ballSocketAngularCSVB, out var ballSocketAngularCSV);
            Vector3Wide.Subtract(wsvA.Linear, wsvB.Linear, out var ballSocketLinearCSV);
            Vector3Wide.Add(ballSocketAngularCSV, ballSocketLinearCSV, out var ballSocketCSV);
            Vector3Wide.Subtract(ballSocketBiasVelocity, ballSocketCSV, out ballSocketCSV);
            Vector2Wide.Subtract(hingeCSVA, negatedHingeCSVB, out var hingeCSV);
            Vector2Wide.Subtract(hingeBiasVelocity, hingeCSV, out hingeCSV);

            HingeAccumulatedImpulses csi;
            Symmetric5x5Wide.TransformWithoutOverlap(ballSocketCSV, hingeCSV, effectiveMass, out csi.BallSocket, out csi.Hinge);
            csi.BallSocket *= effectiveMassCFMScale;
            csi.Hinge *= effectiveMassCFMScale;
            Vector3Wide.Scale(accumulatedImpulses.BallSocket, softnessImpulseScale, out var ballSocketSoftnessContribution);
            Vector3Wide.Subtract(csi.BallSocket, ballSocketSoftnessContribution, out csi.BallSocket);
            Vector2Wide.Scale(accumulatedImpulses.Hinge, softnessImpulseScale, out var hingeSoftnessContribution);
            Vector2Wide.Subtract(csi.Hinge, hingeSoftnessContribution, out csi.Hinge);

            accumulatedImpulses.BallSocket += csi.BallSocket;
            accumulatedImpulses.Hinge += csi.Hinge;

            ApplyImpulse(offsetA, offsetB, hingeJacobian, inertiaA, inertiaB, csi, ref wsvA, ref wsvB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref HingePrestepData prestepData) { }
    }

    public class HingeTypeProcessor : TwoBodyTypeProcessor<HingePrestepData, HingeAccumulatedImpulses, HingeFunctions, AccessNoPosition, AccessNoPosition, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 47;
    }
}
