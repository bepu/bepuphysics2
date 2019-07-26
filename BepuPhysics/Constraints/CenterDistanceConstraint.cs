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
    /// Constrains the center of two bodies to be separated by a goal distance.
    /// </summary>
    public struct CenterDistanceConstraint : ITwoBodyConstraintDescription<CenterDistanceConstraint>
    {
        /// <summary>
        /// Target distance between the body centers.
        /// </summary>
        public float TargetDistance;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CenterDistanceConstraint(float targetDistance, in SpringSettings springSettings)
        {
            TargetDistance = targetDistance;
            SpringSettings = springSettings;
        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return CenterDistanceTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(CenterDistanceTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(TargetDistance >= 0, "CenterDistanceConstraint.TargetDistance must be nonnegative.");
            ConstraintChecker.AssertValid(SpringSettings, nameof(CenterDistanceConstraint));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<CenterDistancePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GatherScatter.GetFirst(ref target.TargetDistance) = TargetDistance;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out CenterDistanceConstraint description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<CenterDistancePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.TargetDistance = GatherScatter.GetFirst(ref source.TargetDistance);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct CenterDistancePrestepData
    {
        public Vector<float> TargetDistance;
        public SpringSettingsWide SpringSettings;
    }

    public struct CenterDistanceProjection
    {
        public Vector3Wide JacobianA;
        public Vector<float> BiasVelocity;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> EffectiveMass;
        public Vector<float> InverseMassA;
        public Vector<float> InverseMassB;
    }

    public struct CenterDistanceConstraintFunctions : IConstraintFunctions<CenterDistancePrestepData, CenterDistanceProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref CenterDistancePrestepData prestep, out CenterDistanceProjection projection)
        {
            bodies.GatherOffsets(ref bodyReferences, count, out var ab);

            Vector3Wide.Length(ab, out var distance);
            Vector3Wide.Scale(ab, Vector<float>.One / distance, out projection.JacobianA);

            var useFallback = Vector.LessThan(distance, new Vector<float>(1e-10f));
            projection.JacobianA.X = Vector.ConditionalSelect(useFallback, Vector<float>.One, projection.JacobianA.X);
            projection.JacobianA.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, projection.JacobianA.Y);
            projection.JacobianA.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, projection.JacobianA.Z);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            //Jacobian is just the unit length direction, so the effective mass is simple:
            projection.EffectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + inertiaB.InverseMass);
            projection.InverseMassA = inertiaA.InverseMass;
            projection.InverseMassB = inertiaB.InverseMass;

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            projection.BiasVelocity = (distance - prestep.TargetDistance) * positionErrorToVelocity;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ApplyImpulse(ref BodyVelocities a, ref BodyVelocities b, ref CenterDistanceProjection projection, ref Vector<float> impulse)
        {
            Vector3Wide.Scale(projection.JacobianA, impulse * projection.InverseMassA, out var changeA);
            Vector3Wide.Scale(projection.JacobianA, impulse * projection.InverseMassB, out var negatedChangeB);
            Vector3Wide.Add(a.Linear, changeA, out a.Linear);
            Vector3Wide.Subtract(b.Linear, negatedChangeB, out b.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CenterDistanceProjection projection, ref Vector<float> accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CenterDistanceProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(velocityA.Linear, projection.JacobianA, out var linearCSVA);
            Vector3Wide.Dot(velocityB.Linear, projection.JacobianA, out var negatedCSVB);
            var csi = (projection.BiasVelocity - (linearCSVA - negatedCSVB)) * projection.EffectiveMass - accumulatedImpulse * projection.SoftnessImpulseScale;
            accumulatedImpulse += csi;
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref csi);
        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class CenterDistanceTypeProcessor : TwoBodyTypeProcessor<CenterDistancePrestepData, CenterDistanceProjection, Vector<float>, CenterDistanceConstraintFunctions>
    {
        public const int BatchTypeId = 35;
    }
}
