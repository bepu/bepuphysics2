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

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return CenterDistanceTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(CenterDistanceTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(TargetDistance >= 0, "CenterDistanceConstraint.TargetDistance must be nonnegative.");
            ConstraintChecker.AssertValid(SpringSettings, nameof(CenterDistanceConstraint));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<CenterDistancePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GatherScatter.GetFirst(ref target.TargetDistance) = TargetDistance;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out CenterDistanceConstraint description)
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

    public struct CenterDistanceConstraintFunctions : ITwoBodyConstraintFunctions<CenterDistancePrestepData, CenterDistanceProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            float dt, float inverseDt, ref CenterDistancePrestepData prestep, out CenterDistanceProjection projection)
        {
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
        static void ApplyImpulse(ref BodyVelocityWide a, ref BodyVelocityWide b, ref CenterDistanceProjection projection, ref Vector<float> impulse)
        {
            Vector3Wide.Scale(projection.JacobianA, impulse * projection.InverseMassA, out var changeA);
            Vector3Wide.Scale(projection.JacobianA, impulse * projection.InverseMassB, out var negatedChangeB);
            Vector3Wide.Add(a.Linear, changeA, out a.Linear);
            Vector3Wide.Subtract(b.Linear, negatedChangeB, out b.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref CenterDistanceProjection projection, ref Vector<float> accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, ref CenterDistanceProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(velocityA.Linear, projection.JacobianA, out var linearCSVA);
            Vector3Wide.Dot(velocityB.Linear, projection.JacobianA, out var negatedCSVB);
            var csi = (projection.BiasVelocity - (linearCSVA - negatedCSVB)) * projection.EffectiveMass - accumulatedImpulse * projection.SoftnessImpulseScale;
            accumulatedImpulse += csi;
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref csi);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ApplyImpulse(in Vector3Wide jacobianA, in Vector<float> inverseMassA, in Vector<float> inverseMassB, in Vector<float> impulse, ref BodyVelocityWide a, ref BodyVelocityWide b)
        {
            Vector3Wide.Scale(jacobianA, impulse * inverseMassA, out var changeA);
            Vector3Wide.Scale(jacobianA, impulse * inverseMassB, out var negatedChangeB);
            Vector3Wide.Add(a.Linear, changeA, out a.Linear);
            Vector3Wide.Subtract(b.Linear, negatedChangeB, out b.Linear);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            ref CenterDistancePrestepData prestep, ref Vector<float> accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            var ab = positionB - positionA;
            var lengthSquared = ab.LengthSquared();
            var inverseDistance = MathHelper.FastReciprocalSquareRoot(lengthSquared);
            var useFallback = Vector.LessThan(lengthSquared, new Vector<float>(1e-10f));
            Vector3Wide.Scale(ab, inverseDistance, out var jacobianA);
            jacobianA.X = Vector.ConditionalSelect(useFallback, Vector<float>.One, jacobianA.X);
            jacobianA.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, jacobianA.Y);
            jacobianA.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, jacobianA.Z);

            ApplyImpulse(jacobianA, inertiaA.InverseMass, inertiaB.InverseMass, accumulatedImpulses, ref wsvA, ref wsvB);
        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve2(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt,
            ref CenterDistancePrestepData prestep, ref Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //Note that we need the actual length for error calculation.
            var ab = positionB - positionA;
            var distance = ab.Length();
            var inverseDistance = MathHelper.FastReciprocal(distance);
            var useFallback = Vector.LessThan(distance, new Vector<float>(1e-5f));
            Vector3Wide.Scale(ab, inverseDistance, out var jacobianA);
            jacobianA.X = Vector.ConditionalSelect(useFallback, Vector<float>.One, jacobianA.X);
            jacobianA.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, jacobianA.Y);
            jacobianA.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, jacobianA.Z);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            //Jacobian is just the unit length direction, so the effective mass is simple:
            var effectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + inertiaB.InverseMass);  

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            var biasVelocity = (distance - prestep.TargetDistance) * positionErrorToVelocity;

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(wsvA.Linear, jacobianA, out var linearCSVA);
            Vector3Wide.Dot(wsvB.Linear, jacobianA, out var negatedCSVB);
            var csi = (biasVelocity - (linearCSVA - negatedCSVB)) * effectiveMass - accumulatedImpulse * softnessImpulseScale;
            accumulatedImpulse += csi;
            ApplyImpulse(jacobianA, inertiaA.InverseMass, inertiaB.InverseMass, csi, ref wsvA, ref wsvB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateForNewPose(
            in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in BodyVelocityWide wsvA, 
            in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, in BodyVelocityWide wsvB,
            in Vector<float> dt, in Vector<float> accumulatedImpulses, ref CenterDistancePrestepData prestep)
        {
        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class CenterDistanceTypeProcessor : TwoBodyTypeProcessor<CenterDistancePrestepData, CenterDistanceProjection, Vector<float>, CenterDistanceConstraintFunctions, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear>
    {
        public const int BatchTypeId = 35;
    }
}
