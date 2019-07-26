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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return SwivelHingeTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(SwivelHingeTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out SwivelHinge description)
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

    public struct SwivelHingeProjection
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector3Wide SwivelHingeJacobian;
        public Vector4Wide BiasVelocity;
        public Symmetric4x4Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
    }

    public struct SwivelHingeFunctions : IConstraintFunctions<SwivelHingePrestepData, SwivelHingeProjection, Vector4Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref SwivelHingePrestepData prestep, out SwivelHingeProjection projection)
        {
            bodies.GatherPose(ref bodyReferences, count, out var ab, out var orientationA, out var orientationB);
            projection.InertiaA = inertiaA;
            projection.InertiaB = inertiaB;

            //4x12 jacobians, from BallSocket and AngularSwivelHinge:
            //[ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //[ 0, swivelA x hingeB, 0, -swivelA x hingeB ]

            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationMatrixA, out projection.OffsetA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalSwivelAxisA, orientationMatrixA, out var swivelAxis);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationMatrixB, out projection.OffsetB);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalHingeAxisB, orientationMatrixB, out var hingeAxis);
            Vector3Wide.CrossWithoutOverlap(swivelAxis, hingeAxis, out projection.SwivelHingeJacobian);

            //The upper left 3x3 block is just the ball socket.
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(projection.OffsetA, inertiaA.InverseInertiaTensor, out var ballSocketContributionAngularA);
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(projection.OffsetB, inertiaB.InverseInertiaTensor, out var ballSocketContributionAngularB);
            Symmetric4x4Wide inverseEffectiveMass;
            ref var upperLeft = ref Symmetric4x4Wide.GetUpperLeft3x3Block(ref inverseEffectiveMass);
            Symmetric3x3Wide.Add(ballSocketContributionAngularA, ballSocketContributionAngularB, out upperLeft);
            var linearContribution = projection.InertiaA.InverseMass + projection.InertiaB.InverseMass;
            upperLeft.XX += linearContribution;
            upperLeft.YY += linearContribution;
            upperLeft.ZZ += linearContribution;

            //The lower right 1x1 block is the AngularSwivelHinge.
            Symmetric3x3Wide.TransformWithoutOverlap(projection.SwivelHingeJacobian, inertiaA.InverseInertiaTensor, out var swivelHingeInertiaA);
            Symmetric3x3Wide.TransformWithoutOverlap(projection.SwivelHingeJacobian, inertiaB.InverseInertiaTensor, out var swivelHingeInertiaB);
            Vector3Wide.Dot(swivelHingeInertiaA, projection.SwivelHingeJacobian, out var swivelHingeContributionAngularA);
            Vector3Wide.Dot(swivelHingeInertiaB, projection.SwivelHingeJacobian, out var swivelHingeContributionAngularB);
            inverseEffectiveMass.WW = swivelHingeContributionAngularA + swivelHingeContributionAngularB;

            //The remaining off-diagonal region is skew(offsetA) * Ia^-1 * (swivelAxis x hingeAxis) + skew(offsetB) * Ib^-1 * (swivelAxis x hingeAxis)
            //skew(offsetA) * (Ia^-1 * (swivelAxis x hingeAxis) = (Ia^-1 * (swivelAxis x hingeAxis)) x offsetA
            //Careful with cross order/signs!
            Vector3Wide.CrossWithoutOverlap(swivelHingeInertiaA, projection.OffsetA, out var offDiagonalContributionA);
            Vector3Wide.CrossWithoutOverlap(swivelHingeInertiaB, projection.OffsetB, out var offDiagonalContributionB);
            Vector3Wide.Add(offDiagonalContributionA, offDiagonalContributionB, out Symmetric4x4Wide.GetUpperRight3x1Block(ref inverseEffectiveMass));

            Symmetric4x4Wide.InvertWithoutOverlap(inverseEffectiveMass, out projection.EffectiveMass);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Symmetric4x4Wide.Scale(projection.EffectiveMass, effectiveMassCFMScale, out projection.EffectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(ab, projection.OffsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, projection.OffsetA, out var ballSocketError);
            projection.BiasVelocity.X = ballSocketError.X * positionErrorToVelocity;
            projection.BiasVelocity.Y = ballSocketError.Y * positionErrorToVelocity;
            projection.BiasVelocity.Z = ballSocketError.Z * positionErrorToVelocity;

            Vector3Wide.Dot(hingeAxis, swivelAxis, out var error);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            projection.BiasVelocity.W = positionErrorToVelocity * -error;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref SwivelHingeProjection projection, ref Vector4Wide csi)
        {
            //[ csi ] * [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //          [ 0, swivelA x hingeB, 0, -swivelA x hingeB ]
            ref var ballSocketCSI = ref Unsafe.As<Vector<float>, Vector3Wide>(ref csi.X);
            Vector3Wide.Scale(ballSocketCSI, projection.InertiaA.InverseMass, out var linearChangeA);
            Vector3Wide.Add(velocityA.Linear, linearChangeA, out velocityA.Linear);

            Vector3Wide.CrossWithoutOverlap(projection.OffsetA, ballSocketCSI, out var ballSocketAngularImpulseA);
            Vector3Wide.Scale(projection.SwivelHingeJacobian, csi.W, out var swivelHingeAngularImpulseA);
            Vector3Wide.Add(ballSocketAngularImpulseA, swivelHingeAngularImpulseA, out var angularImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, projection.InertiaA.InverseInertiaTensor, out var angularChangeA);
            Vector3Wide.Add(velocityA.Angular, angularChangeA, out velocityA.Angular);

            //Note cross order flip for negation.
            Vector3Wide.Scale(ballSocketCSI, projection.InertiaB.InverseMass, out var negatedLinearChangeB);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearChangeB, out velocityB.Linear);
            Vector3Wide.CrossWithoutOverlap(ballSocketCSI, projection.OffsetB, out var ballSocketAngularImpulseB);
            Vector3Wide.Subtract(ballSocketAngularImpulseB, swivelHingeAngularImpulseA, out var angularImpulseB);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseB, projection.InertiaB.InverseInertiaTensor, out var angularChangeB);
            Vector3Wide.Add(velocityB.Angular, angularChangeB, out velocityB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref SwivelHingeProjection projection, ref Vector4Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref SwivelHingeProjection projection, ref Vector4Wide accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //[ csi ] * [ I, skew(offsetA),   -I, -skew(offsetB)    ]
            //          [ 0, swivelA x hingeB, 0, -swivelA x hingeB ]
            Vector3Wide.CrossWithoutOverlap(velocityA.Angular, projection.OffsetA, out var ballSocketAngularCSVA);
            Vector3Wide.Dot(projection.SwivelHingeJacobian, velocityA.Angular, out var swivelHingeCSVA);
            Vector3Wide.CrossWithoutOverlap(projection.OffsetB, velocityB.Angular, out var ballSocketAngularCSVB);
            Vector3Wide.Dot(projection.SwivelHingeJacobian, velocityB.Angular, out var negatedSwivelHingeCSVB);

            Vector3Wide.Add(ballSocketAngularCSVA, ballSocketAngularCSVB, out var ballSocketAngularCSV);
            Vector3Wide.Subtract(velocityA.Linear, velocityB.Linear, out var ballSocketLinearCSV);
            Vector4Wide csv;
            csv.X = ballSocketAngularCSV.X + ballSocketLinearCSV.X;
            csv.Y = ballSocketAngularCSV.Y + ballSocketLinearCSV.Y;
            csv.Z = ballSocketAngularCSV.Z + ballSocketLinearCSV.Z;
            csv.W = swivelHingeCSVA - negatedSwivelHingeCSVB;
            Vector4Wide.Subtract(projection.BiasVelocity, csv, out csv);

            Symmetric4x4Wide.TransformWithoutOverlap(csv, projection.EffectiveMass, out var csi);
            Vector4Wide.Scale(accumulatedImpulse, projection.SoftnessImpulseScale, out var softnessContribution);
            Vector4Wide.Subtract(csi, softnessContribution, out csi);

            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref csi);
        }

    }

    public class SwivelHingeTypeProcessor : TwoBodyTypeProcessor<SwivelHingePrestepData, SwivelHingeProjection, Vector4Wide, SwivelHingeFunctions>
    {
        public const int BatchTypeId = 46;
    }
}
