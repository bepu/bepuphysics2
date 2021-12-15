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
    /// Constrains two bodies with a swivel hinge that allows rotation around two axes, like a laptop monitor hinge that allows flipping the screen. Equivalent to a BallSocket constraint and an AngularSwivelHinge constraint solved together.
    /// </summary>
    public struct SwivelHinge : ITwoBodyConstraintDescription<SwivelHinge>
    {
        /// <summary>
        /// Local offset from the center of body A to its attachment point.
        /// </summary>
        public Vector3 LocalOffsetA;
        /// <summary>
        /// Swivel axis in the local space of body A.
        /// </summary>
        public Vector3 LocalSwivelAxisA;
        /// <summary>
        /// Local offset from the center of body B to its attachment point.
        /// </summary>
        public Vector3 LocalOffsetB;
        /// <summary>
        /// Hinge axis in the local space of body B.
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
                return SwivelHingeTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(SwivelHingeTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalSwivelAxisA, nameof(SwivelHinge), nameof(LocalSwivelAxisA));
            ConstraintChecker.AssertUnitLength(LocalHingeAxisB, nameof(SwivelHinge), nameof(LocalHingeAxisB));
            ConstraintChecker.AssertValid(SpringSettings, nameof(SwivelHinge));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<SwivelHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalSwivelAxisA, ref target.LocalSwivelAxisA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            Vector3Wide.WriteFirst(LocalHingeAxisB, ref target.LocalHingeAxisB);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out SwivelHinge description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<SwivelHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalSwivelAxisA, out description.LocalSwivelAxisA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            Vector3Wide.ReadFirst(source.LocalHingeAxisB, out description.LocalHingeAxisB);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct SwivelHingePrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalSwivelAxisA;
        public Vector3Wide LocalOffsetB;
        public Vector3Wide LocalHingeAxisB;
        public SpringSettingsWide SpringSettings;
    }

    public struct SwivelHingeFunctions : ITwoBodyConstraintFunctions<SwivelHingePrestepData, Vector4Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(in Vector3Wide offsetA, in Vector3Wide offsetB, in Vector3Wide swivelHingeJacobian, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, ref Vector4Wide csi, ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB)
        {
            //[ csi ] * [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //          [ 0, swivelA x hingeB, 0, -swivelA x hingeB ]
            ref var ballSocketCSI = ref Unsafe.As<Vector<float>, Vector3Wide>(ref csi.X);
            Vector3Wide.Scale(ballSocketCSI, inertiaA.InverseMass, out var linearChangeA);
            Vector3Wide.Add(velocityA.Linear, linearChangeA, out velocityA.Linear);

            Vector3Wide.CrossWithoutOverlap(offsetA, ballSocketCSI, out var ballSocketAngularImpulseA);
            Vector3Wide.Scale(swivelHingeJacobian, csi.W, out var swivelHingeAngularImpulseA);
            Vector3Wide.Add(ballSocketAngularImpulseA, swivelHingeAngularImpulseA, out var angularImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, inertiaA.InverseInertiaTensor, out var angularChangeA);
            Vector3Wide.Add(velocityA.Angular, angularChangeA, out velocityA.Angular);

            //Note cross order flip for negation.
            Vector3Wide.Scale(ballSocketCSI, inertiaB.InverseMass, out var negatedLinearChangeB);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearChangeB, out velocityB.Linear);
            Vector3Wide.CrossWithoutOverlap(ballSocketCSI, offsetB, out var ballSocketAngularImpulseB);
            Vector3Wide.Subtract(ballSocketAngularImpulseB, swivelHingeAngularImpulseA, out var angularImpulseB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseB, inertiaB.InverseInertiaTensor, out var angularChangeB);
            Vector3Wide.Add(velocityB.Angular, angularChangeB, out velocityB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ComputeJacobian(in Vector3Wide localOffsetA, in Vector3Wide localSwivelAxisA, in Vector3Wide localOffsetB, in Vector3Wide localHingeAxisB, in QuaternionWide orientationA, in QuaternionWide orientationB,
            out Vector3Wide swivelAxis, out Vector3Wide hingeAxis, out Vector3Wide offsetA, out Vector3Wide offsetB, out Vector3Wide swivelHingeJacobian)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetA, orientationMatrixA, out offsetA);
            Matrix3x3Wide.TransformWithoutOverlap(localSwivelAxisA, orientationMatrixA, out swivelAxis);
            Matrix3x3Wide.TransformWithoutOverlap(localOffsetB, orientationMatrixB, out offsetB);
            Matrix3x3Wide.TransformWithoutOverlap(localHingeAxisB, orientationMatrixB, out hingeAxis);
            Vector3Wide.CrossWithoutOverlap(swivelAxis, hingeAxis, out swivelHingeJacobian);
            //If the axes are aligned, then it'll be zero length and the effective mass can get NaNsploded.
            var lengthSquared = swivelHingeJacobian.LengthSquared();
            var useFallbackJacobian = Vector.LessThan(lengthSquared, new Vector<float>(1e-3f));
            //This causes a discontinuity, but a discontinuity is better than a NaNsplode.
            swivelHingeJacobian = Vector3Wide.ConditionalSelect(useFallbackJacobian, hingeAxis, swivelHingeJacobian);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref SwivelHingePrestepData prestep, ref Vector4Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(prestep.LocalOffsetA, prestep.LocalSwivelAxisA, prestep.LocalOffsetB, prestep.LocalHingeAxisB, orientationA, orientationB,
                out _, out _, out var offsetA, out var offsetB, out var swivelHingeJacobian);
            ApplyImpulse(offsetA, offsetB, swivelHingeJacobian, inertiaA, inertiaB, ref accumulatedImpulses, ref wsvA, ref wsvB);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref SwivelHingePrestepData prestep, ref Vector4Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //4x12 jacobians, from BallSocket and AngularSwivelHinge:
            //[ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //[ 0, swivelA x hingeB, 0, -swivelA x hingeB ]

            ComputeJacobian(prestep.LocalOffsetA, prestep.LocalSwivelAxisA, prestep.LocalOffsetB, prestep.LocalHingeAxisB, orientationA, orientationB,
                out var swivelAxis, out var hingeAxis, out var offsetA, out var offsetB, out var swivelHingeJacobian);

            //The upper left 3x3 block is just the ball socket.
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offsetA, inertiaA.InverseInertiaTensor, out var ballSocketContributionAngularA);
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offsetB, inertiaB.InverseInertiaTensor, out var ballSocketContributionAngularB);
            Unsafe.SkipInit(out Symmetric4x4Wide inverseEffectiveMass);
            ref var upperLeft = ref Symmetric4x4Wide.GetUpperLeft3x3Block(ref inverseEffectiveMass);
            Symmetric3x3Wide.Add(ballSocketContributionAngularA, ballSocketContributionAngularB, out upperLeft);
            var linearContribution = inertiaA.InverseMass + inertiaB.InverseMass;
            upperLeft.XX += linearContribution;
            upperLeft.YY += linearContribution;
            upperLeft.ZZ += linearContribution;

            //The lower right 1x1 block is the AngularSwivelHinge.
            Symmetric3x3Wide.TransformWithoutOverlap(swivelHingeJacobian, inertiaA.InverseInertiaTensor, out var swivelHingeInertiaA);
            Symmetric3x3Wide.TransformWithoutOverlap(swivelHingeJacobian, inertiaB.InverseInertiaTensor, out var swivelHingeInertiaB);
            Vector3Wide.Dot(swivelHingeInertiaA, swivelHingeJacobian, out var swivelHingeContributionAngularA);
            Vector3Wide.Dot(swivelHingeInertiaB, swivelHingeJacobian, out var swivelHingeContributionAngularB);
            inverseEffectiveMass.WW = swivelHingeContributionAngularA + swivelHingeContributionAngularB;

            //The remaining off-diagonal region is skew(offsetA) * Ia^-1 * (swivelAxis x hingeAxis) + skew(offsetB) * Ib^-1 * (swivelAxis x hingeAxis)
            //skew(offsetA) * (Ia^-1 * (swivelAxis x hingeAxis) = (Ia^-1 * (swivelAxis x hingeAxis)) x offsetA
            //Careful with cross order/signs!
            Vector3Wide.CrossWithoutOverlap(swivelHingeInertiaA, offsetA, out var offDiagonalContributionA);
            Vector3Wide.CrossWithoutOverlap(swivelHingeInertiaB, offsetB, out var offDiagonalContributionB);
            Vector3Wide.Add(offDiagonalContributionA, offDiagonalContributionB, out Symmetric4x4Wide.GetUpperRight3x1Block(ref inverseEffectiveMass));

            //TODO: May benefit from LDLT. Weld (6x6) does.
            Symmetric4x4Wide.InvertWithoutOverlap(inverseEffectiveMass, out var effectiveMass);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            //Note that we do not directly scale the effective mass; instead we just scale the CSI; it's smaller.

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(positionB - positionA, offsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, offsetA, out var ballSocketError);
            Vector4Wide biasVelocity;
            biasVelocity.X = ballSocketError.X * positionErrorToVelocity;
            biasVelocity.Y = ballSocketError.Y * positionErrorToVelocity;
            biasVelocity.Z = ballSocketError.Z * positionErrorToVelocity;

            Vector3Wide.Dot(hingeAxis, swivelAxis, out var error);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            biasVelocity.W = positionErrorToVelocity * -error;
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //[ csi ] * [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //          [ 0, swivelA x hingeB, 0, -swivelA x hingeB ]
            Vector3Wide.CrossWithoutOverlap(wsvA.Angular, offsetA, out var ballSocketAngularCSVA);
            Vector3Wide.Dot(swivelHingeJacobian, wsvA.Angular, out var swivelHingeCSVA);
            Vector3Wide.CrossWithoutOverlap(offsetB, wsvB.Angular, out var ballSocketAngularCSVB);
            Vector3Wide.Dot(swivelHingeJacobian, wsvB.Angular, out var negatedSwivelHingeCSVB);

            Vector3Wide.Add(ballSocketAngularCSVA, ballSocketAngularCSVB, out var ballSocketAngularCSV);
            Vector3Wide.Subtract(wsvA.Linear, wsvB.Linear, out var ballSocketLinearCSV);
            Vector4Wide csv;
            csv.X = ballSocketAngularCSV.X + ballSocketLinearCSV.X;
            csv.Y = ballSocketAngularCSV.Y + ballSocketLinearCSV.Y;
            csv.Z = ballSocketAngularCSV.Z + ballSocketLinearCSV.Z;
            csv.W = swivelHingeCSVA - negatedSwivelHingeCSVB;
            Vector4Wide.Subtract(biasVelocity, csv, out csv);

            Symmetric4x4Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
            Vector4Wide.Scale(csi, effectiveMassCFMScale, out csi);
            Vector4Wide.Scale(accumulatedImpulses, softnessImpulseScale, out var softnessContribution);
            Vector4Wide.Subtract(csi, softnessContribution, out csi);

            accumulatedImpulses += csi;

            ApplyImpulse(offsetA, offsetB, swivelHingeJacobian, inertiaA, inertiaB, ref csi, ref wsvA, ref wsvB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref SwivelHingePrestepData prestepData) { }
    }

    public class SwivelHingeTypeProcessor : TwoBodyTypeProcessor<SwivelHingePrestepData, Vector4Wide, SwivelHingeFunctions, AccessNoPosition, AccessNoPosition, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 46;
    }
}
