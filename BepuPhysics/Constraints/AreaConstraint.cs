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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AreaConstraintTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(AreaConstraintTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(TargetScaledArea >= 0, "AreaConstraint.TargetScaledArea must be nonnegative.");
            ConstraintChecker.AssertValid(SpringSettings, nameof(AreaConstraint));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AreaConstraintPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Unsafe.As<Vector<float>, float>(ref target.TargetScaledArea) = TargetScaledArea;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AreaConstraint description)
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

    public struct AreaConstraintProjection
    {
        public Vector3Wide JacobianB;
        public Vector3Wide JacobianC;
        public Vector<float> EffectiveMass;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> InverseMassA;
        public Vector<float> InverseMassB;
        public Vector<float> InverseMassC;
    }

    public struct AreaConstraintFunctions : IThreeBodyConstraintFunctions<AreaConstraintPrestepData, AreaConstraintProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref ThreeBodyReferences bodyReferences, int count, float dt, float inverseDt,
            ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref BodyInertias inertiaC,
            ref AreaConstraintPrestepData prestep, out AreaConstraintProjection projection)
        {
            bodies.GatherOffsets(ref bodyReferences, count, out var ab, out var ac);

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
            Vector3Wide.CrossWithoutOverlap(ab, ac, out var abxac);
            Vector3Wide.Length(abxac, out var normalLength);
            //The triangle normal length can be zero if the edges are parallel or antiparallel. Protect against the potential division by zero.
            Vector3Wide.Scale(abxac, Vector.ConditionalSelect(Vector.GreaterThan(normalLength, new Vector<float>(1e-10f)), Vector<float>.One / normalLength, Vector<float>.Zero), out var normal);

            Vector3Wide.CrossWithoutOverlap(ac, normal, out projection.JacobianB);
            Vector3Wide.CrossWithoutOverlap(normal, ab, out projection.JacobianC);
            //Similar to the volume constraint, we could create a similar expression for jacobianA, but it's cheap to just do a couple of adds.
            Vector3Wide.Add(projection.JacobianB, projection.JacobianC, out var negatedJacobianA);

            //We can store:
            //Jacobians (2 * 3)
            //Effective mass (1)
            //Inverse inertia (1 * 3 since we don't need angular inertia)
            //Since we don't need the inertia tensor, this is better than the premultiplied variant.

            Vector3Wide.Dot(negatedJacobianA, negatedJacobianA, out var contributionA);
            Vector3Wide.Dot(projection.JacobianB, projection.JacobianB, out var contributionB);
            Vector3Wide.Dot(projection.JacobianC, projection.JacobianC, out var contributionC);

            //Protect against singularity by padding the jacobian contributions. This is very much a hack, but it's a pretty simple hack.
            //Less sensitive to tuning than attempting to guard the inverseEffectiveMass itself, since that is sensitive to both scale AND mass.

            //Choose an epsilon based on the target area. Note that area ~= width^2 and our jacobian contributions are things like (ac x N) * (ac x N).
            //Given that N is perpendicular to AC, ||(ac x N)|| == ||ac||, so the contribution is just ||ac||^2. Given the square, it's proportional to area and the area is a decent epsilon source.
            var epsilon = 5e-4f * prestep.TargetScaledArea;
            contributionA = Vector.Max(epsilon, contributionA);
            contributionB = Vector.Max(epsilon, contributionB);
            contributionC = Vector.Max(epsilon, contributionC);
            var inverseEffectiveMass = contributionA * inertiaA.InverseMass + contributionB * inertiaB.InverseMass + contributionC * inertiaC.InverseMass;
            projection.InverseMassA = inertiaA.InverseMass;
            projection.InverseMassB = inertiaB.InverseMass;
            projection.InverseMassC = inertiaC.InverseMass;

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);

            projection.EffectiveMass = effectiveMassCFMScale / inverseEffectiveMass;
            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            projection.BiasImpulse = (prestep.TargetScaledArea - normalLength) * (1f / 2f) * positionErrorToVelocity * projection.EffectiveMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BodyVelocities velocityC,
            ref AreaConstraintProjection projection, ref Vector3Wide negatedJacobianA, ref Vector<float> impulse)
        {
            Vector3Wide.Scale(negatedJacobianA, projection.InverseMassA * impulse, out var negativeVelocityChangeA);
            Vector3Wide.Scale(projection.JacobianB, projection.InverseMassB * impulse, out var velocityChangeB);
            Vector3Wide.Scale(projection.JacobianC, projection.InverseMassC * impulse, out var velocityChangeC);
            Vector3Wide.Subtract(velocityA.Linear, negativeVelocityChangeA, out velocityA.Linear);
            Vector3Wide.Add(velocityB.Linear, velocityChangeB, out velocityB.Linear);
            Vector3Wide.Add(velocityC.Linear, velocityChangeC, out velocityC.Linear);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BodyVelocities velocityC, ref AreaConstraintProjection projection, ref Vector<float> accumulatedImpulse)
        {
            Vector3Wide.Add(projection.JacobianB, projection.JacobianC, out var negatedJacobianA);
            ApplyImpulse(ref velocityA, ref velocityB, ref velocityC, ref projection, ref negatedJacobianA, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref BodyVelocities velocityC, ref AreaConstraintProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Add(projection.JacobianB, projection.JacobianC, out var negatedJacobianA);
            Vector3Wide.Dot(negatedJacobianA, velocityA.Linear, out var negatedContributionA);
            Vector3Wide.Dot(projection.JacobianB, velocityB.Linear, out var contributionB);
            Vector3Wide.Dot(projection.JacobianC, velocityC.Linear, out var contributionC);
            var csv = contributionB + contributionC - negatedContributionA;
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - csv * projection.EffectiveMass;
            accumulatedImpulse += csi;

            ApplyImpulse(ref velocityA, ref velocityB, ref velocityC, ref projection, ref negatedJacobianA, ref csi);
        }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket constraints.
    /// </summary>
    public class AreaConstraintTypeProcessor : ThreeBodyTypeProcessor<AreaConstraintPrestepData, AreaConstraintProjection, Vector<float>, AreaConstraintFunctions>
    {
        public const int BatchTypeId = 36;
    }
}
