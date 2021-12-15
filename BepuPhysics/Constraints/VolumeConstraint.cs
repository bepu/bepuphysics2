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
        public VolumeConstraint(in Vector3 a, in Vector3 b, in Vector3 c, in Vector3 d, SpringSettings springSettings)
        {
            TargetScaledVolume = Vector3.Dot(Vector3.Cross(b - a, c - a), d - a);
            SpringSettings = springSettings;
        }

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return VolumeConstraintTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(VolumeConstraintTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(SpringSettings, nameof(VolumeConstraint));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<VolumeConstraintPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Unsafe.As<Vector<float>, float>(ref target.TargetScaledVolume) = TargetScaledVolume;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out VolumeConstraint description)
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
            out Vector3Wide negatedJA, out Vector3Wide jacobianB, out Vector3Wide jacobianC, out Vector3Wide jacobianD)
        {
            var ab = positionB - positionA;
            var ac = positionC - positionA;
            ad = positionD - positionA;
            Vector3Wide.CrossWithoutOverlap(ac, ad, out jacobianB);
            Vector3Wide.CrossWithoutOverlap(ad, ab, out jacobianC);
            Vector3Wide.CrossWithoutOverlap(ab, ac, out jacobianD);
            Vector3Wide.Add(jacobianB, jacobianC, out negatedJA);
            Vector3Wide.Add(jacobianD, negatedJA, out negatedJA);
        }
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC, in Vector3Wide positionD, in QuaternionWide orientationD, in BodyInertiaWide inertiaD, ref VolumeConstraintPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC, ref BodyVelocityWide wsvD)
        {
            ComputeJacobian(positionA, positionB, positionC, positionD, out var ad, out var negatedJA, out var jacobianB, out var jacobianC, out var jacobianD);
            //Vector3Wide.Dot(jacobianD, ad, out var unscaledVolume);
            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, inertiaC.InverseMass, inertiaD.InverseMass, negatedJA, jacobianB, jacobianC, jacobianD, accumulatedImpulses, ref wsvA, ref wsvB, ref wsvC, ref wsvD);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC, in Vector3Wide positionD, in QuaternionWide orientationD, in BodyInertiaWide inertiaD, float dt, float inverseDt, ref VolumeConstraintPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC, ref BodyVelocityWide wsvD)
        {
            //Volume of parallelepiped with vertices a, b, c, d is:
            //(ab x ac) * ad
            //A tetrahedron with the same edges will have one sixth of this volume. As a constant factor, it's not relevant. So the constraint is just:
            //OriginalVolume * 6 = (ab x ac) * ad
            //Taking the derivative to get the velocity constraint:
            //0 = d/dt(ab x ac) * ad + (ab x ac) * d/dt(ad)
            //0 = d/dt(ab x ac) * ad + (ab x ac) * (d/dt(d) - d/dt(a))
            //0 = (d/dt(ab) x ac + ab x d/dt(ac)) * ad + (ab x ac) * (d/dt(d) - d/dt(a))
            //0 = ((d/dt(ab) x ac) * ad + (ab x d/dt(ac)) * ad + (ab x ac) * (d/dt(d) - d/dt(a))
            //0 = (ac x ad) * (d/dt(b) - d/dt(a)) + (ad x ab) * (d/dt(c) - d/dt(a)) + (ab x ac) * (d/dt(d) - d/dt(a))
            //Giving the linear jacobians:
            //JA: -ac x ad - ad x ab - ab x ac == bd x bc
            //JB: ac x ad
            //JC: ad x ab
            //JD: ab x ac
            //We're not blending the jacobians into the effective mass or inverse mass either- even though that would save ALU time, the goal here is to minimize memory bandwidth since that
            //tends to be the bottleneck for any multithreaded simulation. (Despite being a 1DOF constraint, this doesn't need to output inverse inertia tensors, so premultiplying isn't a win.)
            ComputeJacobian(positionA, positionB, positionC, positionD, out var ad, out var negatedJA, out var jacobianB, out var jacobianC, out var jacobianD);

            Vector3Wide.Dot(negatedJA, negatedJA, out var contributionA);
            Vector3Wide.Dot(jacobianB, jacobianB, out var contributionB);
            Vector3Wide.Dot(jacobianC, jacobianC, out var contributionC);
            Vector3Wide.Dot(jacobianD, jacobianD, out var contributionD);

            //Protect against singularity by padding the jacobian contributions. This is very much a hack, but it's a pretty simple hack.
            //Less sensitive to tuning than attempting to guard the inverseEffectiveMass itself, since that is sensitive to both scale AND mass.

            //Choose an epsilon based on the target volume. Note that volume ~= width^3, whereas our jacobian contributions are things like (ac x ad) * (ac x ad), which is proportional
            //to the area of the triangle acd squared. In other words, the contribution is ~ width^4. 
            //Scaling the volume by a constant factor will not match the growth rate of the jacobian contributions.
            //We're going to ignore this until proven to be a noticeable problem because Vector<T> does not expose exp or pow and this is cheap. 
            //Could still implement it, but it's not super high value.
            var epsilon = 5e-4f * prestep.TargetScaledVolume;
            contributionA = Vector.Max(epsilon, contributionA);
            contributionB = Vector.Max(epsilon, contributionB);
            contributionC = Vector.Max(epsilon, contributionC);
            contributionD = Vector.Max(epsilon, contributionD);
            var inverseEffectiveMass = contributionA * inertiaA.InverseMass + contributionB * inertiaB.InverseMass + contributionC * inertiaC.InverseMass + contributionD * inertiaD.InverseMass;

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);

            var effectiveMass = effectiveMassCFMScale / inverseEffectiveMass;
            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Dot(jacobianD, ad, out var unscaledVolume);
            var biasVelocity = (prestep.TargetScaledVolume - unscaledVolume) * positionErrorToVelocity;

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(negatedJA, wsvA.Linear, out var negatedVelocityContributionA);
            Vector3Wide.Dot(jacobianB, wsvB.Linear, out var velocityContributionB);
            Vector3Wide.Dot(jacobianC, wsvC.Linear, out var velocityContributionC);
            Vector3Wide.Dot(jacobianD, wsvD.Linear, out var velocityContributionD);
            var csv = velocityContributionB + velocityContributionC + velocityContributionD - negatedVelocityContributionA;
            var csi = (biasVelocity - csv) * effectiveMass - accumulatedImpulses * softnessImpulseScale;
            accumulatedImpulses += csi;

            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, inertiaC.InverseMass, inertiaD.InverseMass, negatedJA, jacobianB, jacobianC, jacobianD, csi, ref wsvA, ref wsvB, ref wsvC, ref wsvD);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, in BodyVelocityWide wsvC, in BodyVelocityWide wsvD, ref VolumeConstraintPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of volume constraints.
    /// </summary>
    public class VolumeConstraintTypeProcessor : FourBodyTypeProcessor<VolumeConstraintPrestepData, Vector<float>, VolumeConstraintFunctions, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear>
    {
        public const int BatchTypeId = 32;
    }
}
