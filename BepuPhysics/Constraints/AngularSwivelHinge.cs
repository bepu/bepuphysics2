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
    public struct AngularSwivelHinge : IConstraintDescription<AngularSwivelHinge>
    {
        public Vector3 SwivelAxisLocalA;
        public Vector3 HingeAxisLocalB;
        public SpringSettings SpringSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularSwivelHingeTypeProcessor.BatchTypeId;
            }
        }

        public Type BatchType => typeof(AngularSwivelHingeTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularSwivelHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GetFirst(ref target.SwivelAxisLocalA.X) = SwivelAxisLocalA.X;
            GetFirst(ref target.SwivelAxisLocalA.Y) = SwivelAxisLocalA.Y;
            GetFirst(ref target.SwivelAxisLocalA.Z) = SwivelAxisLocalA.Z;
            GetFirst(ref target.HingeAxisLocalB.X) = HingeAxisLocalB.X;
            GetFirst(ref target.HingeAxisLocalB.Y) = HingeAxisLocalB.Y;
            GetFirst(ref target.HingeAxisLocalB.Z) = HingeAxisLocalB.Z;
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularSwivelHinge description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularSwivelHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.SwivelAxisLocalA.X = GetFirst(ref source.SwivelAxisLocalA.X);
            description.SwivelAxisLocalA.Y = GetFirst(ref source.SwivelAxisLocalA.Y);
            description.SwivelAxisLocalA.Z = GetFirst(ref source.SwivelAxisLocalA.Z);
            description.HingeAxisLocalB.X = GetFirst(ref source.HingeAxisLocalB.X);
            description.HingeAxisLocalB.Y = GetFirst(ref source.HingeAxisLocalB.Y);
            description.HingeAxisLocalB.Z = GetFirst(ref source.HingeAxisLocalB.Z);
            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
        }
    }

    public struct AngularSwivelHingePrestepData
    {
        public Vector3Wide SwivelAxisLocalA;
        public Vector3Wide HingeAxisLocalB;
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
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref AngularSwivelHingePrestepData prestep,
            out AngularSwivelHingeProjection projection)
        {
            bodies.GatherInertiaAndPose(ref bodyReferences, count,
                out var localPositionB, out var orientationA, out var orientationB,
                out var inertiaA, out var inertiaB);

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
            QuaternionWide.TransformWithoutOverlap(prestep.SwivelAxisLocalA, orientationA, out var swivelAxis);
            QuaternionWide.TransformWithoutOverlap(prestep.HingeAxisLocalB, orientationB, out var hingeAxis);
            Vector3Wide jacobianA;
            Vector3Wide.CrossWithoutOverlap(ref swivelAxis, ref hingeAxis, out jacobianA);
            //In the event that the axes are parallel, there is no unique jacobian. Arbitrarily pick one.
            //Note that this causes a discontinuity in jacobian length at the poles. We just don't worry about it.
            Helpers.FindPerpendicular(ref swivelAxis, out var fallbackJacobian);
            Vector3Wide.Dot(ref jacobianA, ref jacobianA, out var jacobianLengthSquared);
            var useFallback = Vector.LessThan(jacobianLengthSquared, new Vector<float>(1e-7f));
            Vector3Wide.ConditionalSelect(useFallback, fallbackJacobian, jacobianA, out jacobianA);

            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.

            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref jacobianA, ref inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref jacobianA, ref inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Vector3Wide.Dot(ref projection.ImpulseToVelocityA, ref jacobianA, out var angularA);
            Vector3Wide.Dot(ref projection.NegatedImpulseToVelocityB, ref jacobianA, out var angularB);

            SpringSettingsWide.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var effectiveMass = effectiveMassCFMScale / (angularA + angularB);
            Vector3Wide.Scale(ref jacobianA, ref effectiveMass, out projection.VelocityToImpulseA);
            
            Vector3Wide.Dot(ref hingeAxis, ref swivelAxis, out var error);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            projection.BiasImpulse = -effectiveMass * positionErrorToVelocity * error;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, ref AngularSwivelHingeProjection projection, ref Vector<float> csi)
        {
            Vector3Wide.Scale(ref projection.ImpulseToVelocityA, ref csi, out var velocityChangeA);
            Vector3Wide.Add(ref angularVelocityA, ref velocityChangeA, out angularVelocityA);
            Vector3Wide.Scale(ref projection.NegatedImpulseToVelocityB, ref csi, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(ref angularVelocityB, ref negatedVelocityChangeB, out angularVelocityB);
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
            Vector3Wide.Subtract(ref velocityA.Angular, ref velocityB.Angular, out var difference);
            Vector3Wide.Dot(ref difference, ref projection.VelocityToImpulseA, out var csi);
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

