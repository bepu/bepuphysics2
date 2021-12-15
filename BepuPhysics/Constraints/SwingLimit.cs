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
    /// Restricts axes attached to two bodies to fall within a maximum swing angle.
    /// </summary>
    public struct SwingLimit : ITwoBodyConstraintDescription<SwingLimit>
    {
        /// <summary>
        /// Axis attached to body A in its local space.
        /// </summary>
        public Vector3 AxisLocalA;
        /// <summary>
        /// Axis attached to body B in its local space.
        /// </summary>
        public Vector3 AxisLocalB;
        /// <summary>
        /// Minimum dot product between the world space A and B axes that the constraint attempts to maintain.
        /// </summary>
        public float MinimumDot;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        /// <summary>
        /// Gets or sets the maximum swing angle that the constraint allows between world axis A and B. Based on the MinimumDot field.
        /// </summary>
        public float MaximumSwingAngle { readonly get { return (float)Math.Acos(MinimumDot); } set { MinimumDot = (float)Math.Cos(value); } }

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return SwingLimitTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(SwingLimitTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(AxisLocalA, nameof(SwingLimit), nameof(AxisLocalA));
            ConstraintChecker.AssertUnitLength(AxisLocalB, nameof(SwingLimit), nameof(AxisLocalB));
            Debug.Assert(MinimumDot >= -1f && MinimumDot <= 1f, "SwingLimit.MinimumDot must be from -1 to 1 inclusive.");
            ConstraintChecker.AssertValid(SpringSettings, nameof(SwingLimit));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<SwingLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GetFirst(ref target.AxisLocalA.X) = AxisLocalA.X;
            GetFirst(ref target.AxisLocalA.Y) = AxisLocalA.Y;
            GetFirst(ref target.AxisLocalA.Z) = AxisLocalA.Z;
            GetFirst(ref target.AxisLocalB.X) = AxisLocalB.X;
            GetFirst(ref target.AxisLocalB.Y) = AxisLocalB.Y;
            GetFirst(ref target.AxisLocalB.Z) = AxisLocalB.Z;
            GetFirst(ref target.MinimumDot) = MinimumDot;
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out SwingLimit description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<SwingLimitPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.AxisLocalA.X = GetFirst(ref source.AxisLocalA.X);
            description.AxisLocalA.Y = GetFirst(ref source.AxisLocalA.Y);
            description.AxisLocalA.Z = GetFirst(ref source.AxisLocalA.Z);
            description.AxisLocalB.X = GetFirst(ref source.AxisLocalB.X);
            description.AxisLocalB.Y = GetFirst(ref source.AxisLocalB.Y);
            description.AxisLocalB.Z = GetFirst(ref source.AxisLocalB.Z);
            description.MinimumDot = GetFirst(ref source.MinimumDot);
            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
        }
    }

    public struct SwingLimitPrestepData
    {
        public Vector3Wide AxisLocalA;
        public Vector3Wide AxisLocalB;
        public Vector<float> MinimumDot;
        public SpringSettingsWide SpringSettings;
    }

    public struct SwingLimitFunctions : ITwoBodyConstraintFunctions<SwingLimitPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(in Vector3Wide impulseToVelocityA, in Vector3Wide negatedImpulseToVelocityB, in Vector<float> csi, ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB)
        {
            Vector3Wide.Scale(impulseToVelocityA, csi, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Vector3Wide.Scale(negatedImpulseToVelocityB, csi, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeJacobian(in Vector3Wide axisLocalA, in Vector3Wide axisLocalB, in QuaternionWide orientationA, in QuaternionWide orientationB, out Vector3Wide axisA, out Vector3Wide axisB, out Vector3Wide jacobianA)
        {
            QuaternionWide.TransformWithoutOverlap(axisLocalA, orientationA, out axisA);
            QuaternionWide.TransformWithoutOverlap(axisLocalB, orientationB, out axisB);
            Vector3Wide.CrossWithoutOverlap(axisA, axisB, out jacobianA);
            //In the event that the axes are parallel, there is no unique jacobian. Arbitrarily pick one.
            //Note that this causes a discontinuity in jacobian length at the poles. We just don't worry about it.
            Helpers.FindPerpendicular(axisA, out var fallbackJacobian);
            Vector3Wide.Dot(jacobianA, jacobianA, out var jacobianLengthSquared);
            var useFallback = Vector.LessThan(jacobianLengthSquared, new Vector<float>(1e-7f));
            Vector3Wide.ConditionalSelect(useFallback, fallbackJacobian, jacobianA, out jacobianA);
        }
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref SwingLimitPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(prestep.AxisLocalA, prestep.AxisLocalB, orientationA, orientationB, out _, out _, out var jacobianA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, accumulatedImpulses, ref wsvA.Angular, ref wsvB.Angular);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref SwingLimitPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //The swing limit attempts to keep an axis on body A within from an axis on body B. In other words, this is the same as a hinge joint, but with one fewer DOF.
            //(Note that the jacobians are extremely similar to the AngularSwivelHinge; the difference is that this is a speculative inequality constraint.)
            //C = dot(axisA, axisB) >= MinimumDot
            //C' = dot(d/dt(axisA), axisB) + dot(axisA, d/dt(axisB)) >= 0
            //C' = dot(angularVelocityA x axisA, axisB) + dot(axisA, angularVelocityB x axisB) >= 0
            //C' = dot(axisA x axisB, angularVelocityA) + dot(angularVelocityB, axisB x axisA) >= 0
            //Providing jacobians of:
            //JA = axisA x axisB
            //JB = axisB x axisA
            //a x b == -b x a, so JB == -JA.

            ComputeJacobian(prestep.AxisLocalA, prestep.AxisLocalB, orientationA, orientationB, out var axisA, out var axisB, out var jacobianA);

            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.

            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            Vector3Wide.Dot(impulseToVelocityA, jacobianA, out var angularContributionA);
            Vector3Wide.Dot(negatedImpulseToVelocityB, jacobianA, out var angularContributionB);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var effectiveMass = effectiveMassCFMScale / (angularContributionA + angularContributionB);

            Vector3Wide.Dot(axisA, axisB, out var axisDot);
            var error = axisDot - prestep.MinimumDot;
            //Note the negation: we want to oppose the separation.
            var biasVelocity = -Vector.Min(error * new Vector<float>(inverseDt), error * positionErrorToVelocity);

            //JB = -JA. This is (angularVelocityA * JA + angularVelocityB * JB) * effectiveMass => (angularVelocityA - angularVelocityB) * (JA * effectiveMass)
            Vector3Wide.Subtract(wsvA.Angular, wsvB.Angular, out var difference);
            Vector3Wide.Dot(difference, jacobianA, out var csv);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = effectiveMass * (biasVelocity - csv) - accumulatedImpulses * softnessImpulseScale;

            InequalityHelpers.ClampPositive(ref accumulatedImpulses, ref csi);
            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, csi, ref wsvA.Angular, ref wsvB.Angular);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref SwingLimitPrestepData prestepData) { }
    }

    public class SwingLimitTypeProcessor : TwoBodyTypeProcessor<SwingLimitPrestepData, Vector<float>, SwingLimitFunctions, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 25;
    }
}

