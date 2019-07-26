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
        public float MaximumSwingAngle { get { return (float)Math.Acos(MinimumDot); } set { MinimumDot = (float)Math.Cos(value); } }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return SwingLimitTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(SwingLimitTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out SwingLimit description)
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

    public struct SwingLimitProjection
    {
        //JacobianB = -JacobianA, so no need to store it explicitly.
        public Vector3Wide VelocityToImpulseA;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector3Wide ImpulseToVelocityA;
        public Vector3Wide NegatedImpulseToVelocityB;
    }

    public struct SwingLimitFunctions : IConstraintFunctions<SwingLimitPrestepData, SwingLimitProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref SwingLimitPrestepData prestep, out SwingLimitProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA, out var orientationB);

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

            //Now, we choose the storage representation. The default approach would be to store JA, the effective mass, and both inverse inertias, requiring 6 + 1 + 6 + 6 scalars.  
            //The alternative is to store JAT * effectiveMass, and then also JA * inverseInertiaTensor(A/B), requiring only 3 + 3 + 3 scalars.
            //So, overall, prebaking saves us 10 scalars and a bit of iteration-time ALU.
            QuaternionWide.TransformWithoutOverlap(prestep.AxisLocalA, orientationA, out var axisA);
            QuaternionWide.TransformWithoutOverlap(prestep.AxisLocalB, orientationB, out var axisB);
            Vector3Wide.CrossWithoutOverlap(axisA, axisB, out var jacobianA);
            //In the event that the axes are parallel, there is no unique jacobian. Arbitrarily pick one.
            //Note that this causes a discontinuity in jacobian length at the poles. We just don't worry about it.
            Helpers.FindPerpendicular(axisA, out var fallbackJacobian);
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

            Vector3Wide.Dot(axisA, axisB, out var axisDot);
            var error = axisDot - prestep.MinimumDot;
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            var biasVelocity = -Vector.Min(error * new Vector<float>(inverseDt), error * positionErrorToVelocity);
            projection.BiasImpulse = effectiveMass * biasVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, ref SwingLimitProjection projection, ref Vector<float> csi)
        {
            Vector3Wide.Scale(projection.ImpulseToVelocityA, csi, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Vector3Wide.Scale(projection.NegatedImpulseToVelocityB, csi, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref SwingLimitProjection projection, ref Vector<float> accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref SwingLimitProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //JB = -JA. This is (angularVelocityA * JA + angularVelocityB * JB) * effectiveMass => (angularVelocityA - angularVelocityB) * (JA * effectiveMass)
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var difference);
            Vector3Wide.Dot(difference, projection.VelocityToImpulseA, out var csi);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csi;

            var previousAccumulatedImpulse = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse + csi);
            csi = accumulatedImpulse - previousAccumulatedImpulse;
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref csi);
        }

    }

    public class SwingLimitTypeProcessor : TwoBodyTypeProcessor<SwingLimitPrestepData, SwingLimitProjection, Vector<float>, SwingLimitFunctions>
    {
        public const int BatchTypeId = 25;
    }
}

