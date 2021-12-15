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
    /// Constrains the area of a triangle connecting the centers of three bodies to match a goal area. 
    /// Scaled volume computed from ||ab x ac||.
    /// </summary>
    public struct AreaConstraint : IThreeBodyConstraintDescription<AreaConstraint>
    {
        /// <summary>
        /// 2 times the target area of the triangle. Computed from ||ab x ac||.
        /// </summary>
        public float TargetScaledArea;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        /// <summary>
        /// Creates a new area constraint, initializing the target area using a set of initial positions.
        /// </summary>
        /// <param name="a">Initial position of the first body.</param>
        /// <param name="b">Initial position of the second body.</param>
        /// <param name="c">Initial position of the third body.</param>
        /// <param name="springSettings">Spring settings to apply to the volume constraint.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public AreaConstraint(in Vector3 a, in Vector3 b, in Vector3 c, SpringSettings springSettings)
        {
            TargetScaledArea = Vector3.Cross(b - a, c - a).Length();
            SpringSettings = springSettings;
        }

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AreaConstraintTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(AreaConstraintTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(TargetScaledArea >= 0, "AreaConstraint.TargetScaledArea must be nonnegative.");
            ConstraintChecker.AssertValid(SpringSettings, nameof(AreaConstraint));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AreaConstraintPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Unsafe.As<Vector<float>, float>(ref target.TargetScaledArea) = TargetScaledArea;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AreaConstraint description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AreaConstraintPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.TargetScaledArea = Unsafe.As<Vector<float>, float>(ref source.TargetScaledArea);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct AreaConstraintPrestepData
    {
        public Vector<float> TargetScaledArea;
        public SpringSettingsWide SpringSettings;
    }

    public struct AreaConstraintFunctions : IThreeBodyConstraintFunctions<AreaConstraintPrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(in Vector<float> inverseMassA, in Vector<float> inverseMassB, in Vector<float> inverseMassC,
            in Vector3Wide negatedJacobianA, in Vector3Wide jacobianB, in Vector3Wide jacobianC, in Vector<float> impulse,
            ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref BodyVelocityWide velocityC)
        {
            Vector3Wide.Scale(negatedJacobianA, inverseMassA * impulse, out var negativeVelocityChangeA);
            Vector3Wide.Scale(jacobianB, inverseMassB * impulse, out var velocityChangeB);
            Vector3Wide.Scale(jacobianC, inverseMassC * impulse, out var velocityChangeC);
            Vector3Wide.Subtract(velocityA.Linear, negativeVelocityChangeA, out velocityA.Linear);
            Vector3Wide.Add(velocityB.Linear, velocityChangeB, out velocityB.Linear);
            Vector3Wide.Add(velocityC.Linear, velocityChangeC, out velocityC.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeJacobian(in Vector3Wide positionA, in Vector3Wide positionB, in Vector3Wide positionC, out Vector<float> normalLength, out Vector3Wide negatedJacobianA, out Vector3Wide jacobianB, out Vector3Wide jacobianC)
        {
            //Area of a triangle with vertices a, b, and c is:
            //||ab x ac|| * 0.5
            //So the constraint is:
            //OriginalArea * 2 = ||ab x ac||
            //Leading to a velocity constraint:
            //d/dt(OriginalArea * 2) = d/dt(||ab x ac||)
            //0 = d/dt(dot(ab x ac, ab x ac)^0.5)
            //0 = d/dt(dot(ab x ac, ab x ac)) * 0.5 * dot(ab x ac, ab x ac)^-0.5)
            //0 = (dot(d/dt(ab x ac), ab x ac) + dot(ab x ac, d/dt(ab x ac))) * 0.5 * dot(ab x ac, ab x ac)^-0.5)
            //0 = (2 * dot(d/dt(ab x ac), ab x ac)) * 0.5 * dot(ab x ac, ab x ac)^-0.5)
            //0 = (2 * dot(d/dt(ab) x ac + ab x d/dt(ac), ab x ac)) * 0.5 * dot(ab x ac, ab x ac)^-0.5)
            //0 = dot(d/dt(ab) x ac + ab x d/dt(ac), ab x ac) * dot(ab x ac, ab x ac)^-0.5)
            //0 = dot(d/dt(ab) x ac + ab x d/dt(ac), ab x ac) / ||ab x ac||
            //0 = (dot(d/dt(ab) x ac, ab x ac) + dot(ab x d/dt(ac), ab x ac)) / ||ab x ac||
            //0 = (dot(ac x (ab x ac), d/dt(ab)) + dot((ab x ac) x ab, d/dt(ac))) / ||ab x ac||
            //0 = dot(ac x ((ab x ac) / ||ab x ac||), d/dt(ab)) + dot(((ab x ac) / ||ab x ac||) x ab, d/dt(ac))
            var ab = positionB - positionA;
            var ac = positionC - positionA;
            Vector3Wide.CrossWithoutOverlap(ab, ac, out var abxac);
            Vector3Wide.Length(abxac, out normalLength);
            //The triangle normal length can be zero if the edges are parallel or antiparallel. Protect against the potential division by zero.
            Vector3Wide.Scale(abxac, Vector.ConditionalSelect(Vector.GreaterThan(normalLength, new Vector<float>(1e-10f)), Vector<float>.One / normalLength, Vector<float>.Zero), out var normal);

            Vector3Wide.CrossWithoutOverlap(ac, normal, out jacobianB);
            Vector3Wide.CrossWithoutOverlap(normal, ab, out jacobianC);
            //Similar to the volume constraint, we could create a similar expression for jacobianA, but it's cheap to just do a couple of adds.
            Vector3Wide.Add(jacobianB, jacobianC, out negatedJacobianA);
        }

        public void WarmStart(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA,
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, 
            in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC, 
            ref AreaConstraintPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC)
        {
            ComputeJacobian(positionA, positionB, positionC, out _, out var negatedJacobianA, out var jacobianB, out var jacobianC);
            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, inertiaC.InverseMass, negatedJacobianA, jacobianB, jacobianC, accumulatedImpulses, ref wsvA, ref wsvB, ref wsvC);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in Vector3Wide positionC, in QuaternionWide orientationC, in BodyInertiaWide inertiaC, float dt, float inverseDt, ref AreaConstraintPrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref BodyVelocityWide wsvC)
        {
            ComputeJacobian(positionA, positionB, positionC, out var normalLength, out var negatedJacobianA, out var jacobianB, out var jacobianC);

            Vector3Wide.Dot(negatedJacobianA, negatedJacobianA, out var contributionA);
            Vector3Wide.Dot(jacobianB, jacobianB, out var contributionB);
            Vector3Wide.Dot(jacobianC, jacobianC, out var contributionC);

            //Protect against singularity by padding the jacobian contributions. This is very much a hack, but it's a pretty simple hack.
            //Less sensitive to tuning than attempting to guard the inverseEffectiveMass itself, since that is sensitive to both scale AND mass.

            //Choose an epsilon based on the target area. Note that area ~= width^2 and our jacobian contributions are things like (ac x N) * (ac x N).
            //Given that N is perpendicular to AC, ||(ac x N)|| == ||ac||, so the contribution is just ||ac||^2. Given the square, it's proportional to area and the area is a decent epsilon source.
            var epsilon = 5e-4f * prestep.TargetScaledArea;
            contributionA = Vector.Max(epsilon, contributionA);
            contributionB = Vector.Max(epsilon, contributionB);
            contributionC = Vector.Max(epsilon, contributionC);
            var inverseEffectiveMass = contributionA * inertiaA.InverseMass + contributionB * inertiaB.InverseMass + contributionC * inertiaC.InverseMass;

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);

            var effectiveMass = effectiveMassCFMScale / inverseEffectiveMass;
            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            var biasVelocity = (prestep.TargetScaledArea - normalLength) * positionErrorToVelocity;

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(negatedJacobianA, wsvA.Linear, out var negatedVelocityContributionA);
            Vector3Wide.Dot(jacobianB, wsvB.Linear, out var velocityContributionB);
            Vector3Wide.Dot(jacobianC, wsvC.Linear, out var velocityContributionC);
            var csv = velocityContributionB + velocityContributionC - negatedVelocityContributionA;
            var csi = (biasVelocity - csv) * effectiveMass - accumulatedImpulses * softnessImpulseScale;
            accumulatedImpulses += csi;

            ApplyImpulse(inertiaA.InverseMass, inertiaB.InverseMass, inertiaC.InverseMass, negatedJacobianA, jacobianB, jacobianC, csi, ref wsvA, ref wsvB, ref wsvC);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, in BodyVelocityWide wsvC, ref AreaConstraintPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of area constraints.
    /// </summary>
    public class AreaConstraintTypeProcessor : ThreeBodyTypeProcessor<AreaConstraintPrestepData, Vector<float>, AreaConstraintFunctions, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear>
    {
        public const int BatchTypeId = 36;
    }
}
