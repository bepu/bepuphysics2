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
    /// Constrains two bodies with the angular component of a swivel hinge that allows rotation around two axes, like a laptop monitor hinge that allows flipping the screen.
    /// </summary>
    public struct AngularSwivelHinge : ITwoBodyConstraintDescription<AngularSwivelHinge>
    {
        /// <summary>
        /// Swivel axis in the local space of body A.
        /// </summary>
        public Vector3 LocalSwivelAxisA;
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
                return AngularSwivelHingeTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(AngularSwivelHingeTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalSwivelAxisA, nameof(AngularSwivelHinge), nameof(LocalSwivelAxisA));
            ConstraintChecker.AssertUnitLength(LocalHingeAxisB, nameof(AngularSwivelHinge), nameof(LocalHingeAxisB));
            ConstraintChecker.AssertValid(SpringSettings, nameof(AngularSwivelHinge));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularSwivelHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalSwivelAxisA, ref target.LocalSwivelAxisA);
            Vector3Wide.WriteFirst(LocalHingeAxisB, ref target.LocalHingeAxisB);
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularSwivelHinge description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularSwivelHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalSwivelAxisA, out description.LocalSwivelAxisA);
            Vector3Wide.ReadFirst(source.LocalHingeAxisB, out description.LocalHingeAxisB);
            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
        }
    }

    public struct AngularSwivelHingePrestepData
    {
        public Vector3Wide LocalSwivelAxisA;
        public Vector3Wide LocalHingeAxisB;
        public SpringSettingsWide SpringSettings;
    }

    public struct AngularSwivelHingeFunctions : ITwoBodyConstraintFunctions<AngularSwivelHingePrestepData, Vector<float>>
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
        static void ComputeJacobian(in Vector3Wide localSwivelAxisA, in Vector3Wide localHingeAxisB, in QuaternionWide orientationA, in QuaternionWide orientationB, out Vector3Wide swivelAxis, out Vector3Wide hingeAxis, out Vector3Wide jacobianA)
        {
            QuaternionWide.TransformWithoutOverlap(localSwivelAxisA, orientationA, out swivelAxis);
            QuaternionWide.TransformWithoutOverlap(localHingeAxisB, orientationB, out hingeAxis);
            Vector3Wide.CrossWithoutOverlap(swivelAxis, hingeAxis, out jacobianA);
            //In the event that the axes are parallel, there is no unique jacobian. Arbitrarily pick one.
            //Note that this causes a discontinuity in jacobian length at the poles. We just don't worry about it.
            Helpers.FindPerpendicular(swivelAxis, out var fallbackJacobian);
            Vector3Wide.Dot(jacobianA, jacobianA, out var jacobianLengthSquared);
            var useFallback = Vector.LessThan(jacobianLengthSquared, new Vector<float>(1e-3f));
            Vector3Wide.ConditionalSelect(useFallback, fallbackJacobian, jacobianA, out jacobianA);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref AngularSwivelHingePrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobian(prestep.LocalSwivelAxisA, prestep.LocalHingeAxisB, orientationA, orientationB, out _, out _, out var jacobianA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, accumulatedImpulses, ref wsvA.Angular, ref wsvB.Angular);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref AngularSwivelHingePrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //The swivel hinge attempts to keep an axis on body A separated 90 degrees from an axis on body B. In other words, this is the same as a hinge joint, but with one fewer DOF.
            //C = dot(swivelA, hingeB) = 0
            //C' = dot(d/dt(swivelA), hingeB) + dot(swivelA, d/dt(hingeB)) = 0
            //C' = dot(angularVelocityB x hingeB, swivelA) + dot(hingeB, angularVelocityA x swivelA) = 0
            //C' = dot(hingeB x swivelA, angularVelocityB) + dot(angularVelocityA, swivelA x hingeB) = 0
            //Providing jacobians of:
            //JA = swivelA x hingeB
            //JB = hingeB x swivelA
            //a x b == -b x a, so JB == -JA.

            ComputeJacobian(prestep.LocalSwivelAxisA, prestep.LocalHingeAxisB, orientationA, orientationB, out var swivelAxis, out var hingeAxis, out var jacobianA);

            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.

            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            Vector3Wide.Dot(impulseToVelocityA, jacobianA, out var angularA);
            Vector3Wide.Dot(negatedImpulseToVelocityB, jacobianA, out var angularB);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var effectiveMass = effectiveMassCFMScale / (angularA + angularB);

            Vector3Wide.Dot(hingeAxis, swivelAxis, out var error);
            //Note the negation: we want to oppose the separation.
            var biasVelocity = -(positionErrorToVelocity * error);

            //JB = -JA. This is (angularVelocityA * JA + angularVelocityB * JB) * effectiveMass => (angularVelocityA - angularVelocityB) * (JA * effectiveMass)
            Vector3Wide.Subtract(wsvA.Angular, wsvB.Angular, out var difference);
            Vector3Wide.Dot(difference, jacobianA, out var csv);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            var csi = effectiveMass * (biasVelocity - csv) - accumulatedImpulses * softnessImpulseScale;

            accumulatedImpulses += csi;
            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, csi, ref wsvA.Angular, ref wsvB.Angular);

        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref AngularSwivelHingePrestepData prestepData) { }
    }

    public class AngularSwivelHingeTypeProcessor : TwoBodyTypeProcessor<AngularSwivelHingePrestepData, Vector<float>, AngularSwivelHingeFunctions, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 24;
    }
}

