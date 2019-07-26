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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularSwivelHingeTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(AngularSwivelHingeTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularSwivelHinge description)
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

    public struct AngularSwivelHingeProjection
    {
        //JacobianB = -JacobianA, so no need to store it explicitly.
        public Vector3Wide VelocityToImpulseA;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector3Wide ImpulseToVelocityA;
        public Vector3Wide NegatedImpulseToVelocityB;
    }

    public struct AngularSwivelHingeFunctions : IConstraintFunctions<AngularSwivelHingePrestepData, AngularSwivelHingeProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref AngularSwivelHingePrestepData prestep, out AngularSwivelHingeProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA, out var orientationB);

            //The swivel hinge attempts to keep an axis on body A separated 90 degrees from an axis on body B. In other words, this is the same as a hinge joint, but with one fewer DOF.
            //C = dot(swivelA, hingeB) = 0
            //C' = dot(d/dt(swivelA), hingeB) + dot(swivelA, d/dt(hingeB)) = 0
            //C' = dot(angularVelocityB x hingeB, swivelA) + dot(hingeB, angularVelocityA x swivelA) = 0
            //C' = dot(hingeB x swivelA, angularVelocityB) + dot(angularVelocityA, swivelA x hingeB) = 0
            //Providing jacobians of:
            //JA = swivelA x hingeB
            //JB = hingeB x swivelA
            //a x b == -b x a, so JB == -JA.

            //Now, we choose the storage representation. The default approach would be to store JA, the effective mass, and both inverse inertias, requiring 6 + 1 + 6 + 6 scalars.  
            //The alternative is to store JAT * effectiveMass, and then also JA * inverseInertiaTensor(A/B), requiring only 3 + 3 + 3 scalars.
            //So, overall, prebaking saves us 10 scalars and a bit of iteration-time ALU.
            QuaternionWide.TransformWithoutOverlap(prestep.LocalSwivelAxisA, orientationA, out var swivelAxis);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalHingeAxisB, orientationB, out var hingeAxis);
            Vector3Wide.CrossWithoutOverlap(swivelAxis, hingeAxis, out var jacobianA);
            //In the event that the axes are parallel, there is no unique jacobian. Arbitrarily pick one.
            //Note that this causes a discontinuity in jacobian length at the poles. We just don't worry about it.
            Helpers.FindPerpendicular(swivelAxis, out var fallbackJacobian);
            Vector3Wide.Dot(jacobianA, jacobianA, out var jacobianLengthSquared);
            var useFallback = Vector.LessThan(jacobianLengthSquared, new Vector<float>(1e-7f));
            Vector3Wide.ConditionalSelect(useFallback, fallbackJacobian, jacobianA, out jacobianA);

            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.

            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Vector3Wide.Dot(projection.ImpulseToVelocityA, jacobianA, out var angularA);
            Vector3Wide.Dot(projection.NegatedImpulseToVelocityB, jacobianA, out var angularB);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var effectiveMass = effectiveMassCFMScale / (angularA + angularB);
            Vector3Wide.Scale(jacobianA, effectiveMass, out projection.VelocityToImpulseA);

            Vector3Wide.Dot(hingeAxis, swivelAxis, out var error);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            projection.BiasImpulse = -effectiveMass * positionErrorToVelocity * error;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, ref AngularSwivelHingeProjection projection, ref Vector<float> csi)
        {
            Vector3Wide.Scale(projection.ImpulseToVelocityA, csi, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Vector3Wide.Scale(projection.NegatedImpulseToVelocityB, csi, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularSwivelHingeProjection projection, ref Vector<float> accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularSwivelHingeProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //JB = -JA. This is (angularVelocityA * JA + angularVelocityB * JB) * effectiveMass => (angularVelocityA - angularVelocityB) * (JA * effectiveMass)
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var difference);
            Vector3Wide.Dot(difference, projection.VelocityToImpulseA, out var csi);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csi;

            accumulatedImpulse += csi;
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref csi);
        }

    }

    public class AngularSwivelHingeTypeProcessor : TwoBodyTypeProcessor<AngularSwivelHingePrestepData, AngularSwivelHingeProjection, Vector<float>, AngularSwivelHingeFunctions>
    {
        public const int BatchTypeId = 24;
    }
}

