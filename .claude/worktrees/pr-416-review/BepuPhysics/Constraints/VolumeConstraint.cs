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
    /// Constrains the volume of a tetrahedron connecting the centers of four bodies to match a goal volume.
    /// Scaled volume computed from (ab x ac) * ad; the volume may be negative depending on the winding of the tetrahedron.
    /// </summary>
    public struct VolumeConstraint : IFourBodyConstraintDescription<VolumeConstraint>
    {
        /// <summary>
        /// 6 times the target volume of the tetrahedra. Computed from (ab x ac) * ad; this may be negative depending on the winding of the tetrahedron.
        /// </summary>
        public float TargetScaledVolume;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        /// <summary>
        /// Creates a new volume constraint, initializing the target volume using a set of initial positions.
        /// </summary>
        /// <param name="a">Initial position of the first body.</param>
        /// <param name="b">Initial position of the second body.</param>
        /// <param name="c">Initial position of the third body.</param>
        /// <param name="d">Initial position of the fourth body.</param>
        /// <param name="springSettings">Spring settings to apply to the volume constraint.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public VolumeConstraint(Vector3 a, Vector3 b, Vector3 c, Vector3 d, SpringSettings springSettings)
        {
            TargetScaledVolume = Vector3.Dot(Vector3.Cross(b - a, c - a), d - a);
            SpringSettings = springSettings;
        }

        public static int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return VolumeConstraintTypeProcessor.BatchTypeId;
            }
        }

        public static Type TypeProcessorType => typeof(VolumeConstraintTypeProcessor);
        public static TypeProcessor CreateTypeProcessor() => new VolumeConstraintTypeProcessor();

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(SpringSettings, nameof(VolumeConstraint));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<VolumeConstraintPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Unsafe.As<Vector<float>, float>(ref target.TargetScaledVolume) = TargetScaledVolume;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public static void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out VolumeConstraint description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<VolumeConstraintPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.TargetScaledVolume = Unsafe.As<Vector<float>, float>(ref source.TargetScaledVolume);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct VolumeConstraintPrestepData
    {
        public Vector<float> TargetScaledVolume;
        public SpringSettingsWide SpringSettings;
    }

    public struct VolumeConstraintFunctions : IFourBodyConstraintFunctions<VolumeConstraintPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(
            in Vector<float> inverseMassA, in Vector<float> inverseMassB, in Vector<float> inverseMassC, in Vector<float> inverseMassD,
            in Vector3Wide negatedJacobianA, in Vector3Wide jacobianB, in Vector3Wide jacobianC, in Vector3Wide jacobianD, in Vector<float> impulse,
            ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref BodyVelocityWide velocityC, ref BodyVelocityWide velocityD)
        {
            Vector3Wide.Scale(negatedJacobianA, inverseMassA * impulse, out var negativeVelocityChangeA);
            Vector3Wide.Scale(jacobianB, inverseMassB * impulse, out var velocityChangeB);
            Vector3Wide.Scale(jacobianC, inverseMassC * impulse, out var velocityChangeC);
            Vector3Wide.Scale(jacobianD, inverseMassD * impulse, out var velocityChangeD);
            Vector3Wide.Subtract(velocityA.Linear, negativeVelocityChangeA, out velocityA.Linear);
            Vector3Wide.Add(velocityB.Linear, velocityChangeB, out velocityB.Linear);
            Vector3Wide.Add(velocityC.Linear, velocityChangeC, out velocityC.Linear);
            Vector3Wide.Add(velocityD.Linear, velocityChangeD, out velocityD.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeJacobian(in Vector3Wide positionA, in Vector3Wide positionB, in Vector3Wide positionC, in Vector3Wide positionD,
            out Vector3Wide ad,
            out Vector3Wide negatedJA, out Vector3Wide jacobianB, out Vector3Wide jacobianC, out Vector3Wide jacobianD,
            out Vector<float> contributionA, out Vector<float> contributionB, out Vector<float> contributionC, out Vector<float> contributionD,
            out Vector<float> inverseJacobianLength)
        {
            var ab = positionB - positionA;
            var ac = positionC - positionA;
            ad = positionD - positionA;
            Vector3Wide.CrossWithoutOverlap(ac, ad, out jacobianB);
            Vector3Wide.CrossWithoutOverlap(ad, ab, out jacobianC);
            Vector3Wide.CrossWithoutOverlap(ab, ac, out jacobianD);
            Vector3Wide.Add(jacobianB, jacobianC, out negatedJA);
            Vector3Wide.Add(jacobianD, negatedJA, out negatedJA);
            //Normalize the jacobian to unit length. The raw jacobians are cross products of edges (face area vectors) with magnitude ~L².
            //Normalizing gives a unit-length effective jacobian J_eff = inverseJacobianLength * J_raw where inverseJacobianLength = 1/|J_raw|.
            //This keeps the inverse effective mass bounded (it becomes a weighted average of inverse masses),
            //which bounds the accumulated impulse and makes warm starting stable regardless of configuration.
            //The physical impulse (inverseJacobianLength cancels in the solve) is identical to the raw volume formulation.
            Vector3Wide.Dot(negatedJA, negatedJA, out contributionA);
            Vector3Wide.Dot(jacobianB, jacobianB, out contributionB);
            Vector3Wide.Dot(jacobianC, jacobianC, out contributionC);
            Vector3Wide.Dot(jacobianD, jacobianD, out contributionD);
            var jacobianLengthSquared = contributionA + contributionB + contributionC + contributionD;
            //Guard against the collinear degeneracy (all cross products vanish). This is far more extreme than coplanar;
            //for generic coplanar configurations the cross products remain nonzero.
            jacobianLengthSquared = Vector.Max(new Vector<float>(1e-14f), jacobianLengthSquared);
            inverseJacobianLength = MathHelper.FastReciprocalSquareRoot(jacobianLengthSquared);
        }

        public static void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC, in Vector3Wide positionD, in QuaternionWide orientationD, in BodyInertiaWide inertiaD, ref VolumeConstraintPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC, ref BodyVelocityWide wsvD)
        {
            ComputeJacobian(positionA, positionB, positionC, positionD, out _, out var negatedJA, out var jacobianB, out var jacobianC, out var jacobianD, out _, out _, out _, out _, out var inverseJacobianLength);
            //The accumulated impulse is in unit-jacobian space. Replay through J_eff = inverseJacobianLength * J_raw.
            //Since |J_eff| = 1, the warm start magnitude is bounded by |accumulated| * max(invMass), same as a distance constraint.
            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, inertiaC.InverseMass, inertiaD.InverseMass, negatedJA, jacobianB, jacobianC, jacobianD, inverseJacobianLength * accumulatedImpulses, ref wsvA, ref wsvB, ref wsvC, ref wsvD);
        }

        public static void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC, in Vector3Wide positionD, in QuaternionWide orientationD, in BodyInertiaWide inertiaD, float dt, float inverseDt, ref VolumeConstraintPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC, ref BodyVelocityWide wsvD)
        {
            //Volume of parallelepiped with vertices a, b, c, d is V = (ab x ac) * ad.
            //The raw volume jacobians (dV/dq) are cross products of edges:
            //JA_raw: -(ac x ad) - (ad x ab) - (ab x ac)
            //JB_raw: ac x ad
            //JC_raw: ad x ab
            //JD_raw: ab x ac
            //
            //These have magnitude ~L² (face areas), which varies with configuration and causes warm start instability.
            //We normalize to a unit-length effective jacobian: J_eff = J_raw * inverseJacobianLength, where inverseJacobianLength = 1/|J_raw|.
            //The inverse effective mass becomes a weighted average of inverse masses (always bounded),
            //keeping the accumulated impulse well-scaled across substeps.
            //
            //The position error is the linearized signed distance to the constraint surface V = target:
            //  error = (target_V - V) / |J_raw| = (target_V - V) * inverseJacobianLength
            //The physical impulse (inverseJacobianLength * csi applied through J_raw) is identical to the raw volume formulation
            //because the inverseJacobianLength factors cancel.
            ComputeJacobian(positionA, positionB, positionC, positionD, out var ad, out var negatedJA, out var jacobianB, out var jacobianC, out var jacobianD, out var contributionA, out var contributionB, out var contributionC, out var contributionD, out var inverseJacobianLength);
            var inverseJacobianLengthSquared = inverseJacobianLength * inverseJacobianLength;

            //With the unit-length jacobian, the inverse effective mass is sum(fraction_i * invMass_i) — a weighted average of inverse masses, always bounded.
            //Guard against degenerate configurations (e.g. all points collinear) where all jacobian contributions are zero,
            //which would cause a division by zero when computing the effective mass.
            var inverseEffectiveMass = Vector.Max(new Vector<float>(1e-14f),
                inverseJacobianLengthSquared * (contributionA * inertiaA.InverseMass + contributionB * inertiaB.InverseMass + contributionC * inertiaC.InverseMass + contributionD * inertiaD.InverseMass));

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);

            var effectiveMass = effectiveMassCFMScale / inverseEffectiveMass;
            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Dot(jacobianD, ad, out var volume);
            var biasVelocity = (prestep.TargetScaledVolume - volume) * inverseJacobianLength * positionErrorToVelocity;

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(negatedJA, wsvA.Linear, out var negatedVelocityContributionA);
            Vector3Wide.Dot(jacobianB, wsvB.Linear, out var velocityContributionB);
            Vector3Wide.Dot(jacobianC, wsvC.Linear, out var velocityContributionC);
            Vector3Wide.Dot(jacobianD, wsvD.Linear, out var velocityContributionD);
            var csv = inverseJacobianLength * (velocityContributionB + velocityContributionC + velocityContributionD - negatedVelocityContributionA);
            var csi = (biasVelocity - csv) * effectiveMass - accumulatedImpulses * softnessImpulseScale;
            accumulatedImpulses += csi;

            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, inertiaC.InverseMass, inertiaD.InverseMass, negatedJA, jacobianB, jacobianC, jacobianD, inverseJacobianLength * csi, ref wsvA, ref wsvB, ref wsvC, ref wsvD);
        }

        public static bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, in BodyVelocityWide wsvC, in BodyVelocityWide wsvD, ref VolumeConstraintPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of volume constraints.
    /// </summary>
    public class VolumeConstraintTypeProcessor : FourBodyTypeProcessor<VolumeConstraintPrestepData, Vector<float>, VolumeConstraintFunctions, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear>
    {
        public const int BatchTypeId = 32;
    }
}
