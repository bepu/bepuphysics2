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

    public struct CenterDistanceConstraintFunctions : ITwoBodyConstraintFunctions<CenterDistancePrestepData, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3Wide jacobianA, in Vector<float> inverseMassA, in Vector<float> inverseMassB, in Vector<float> impulse, ref BodyVelocityWide a, ref BodyVelocityWide b)
        {
            Vector3Wide.Scale(jacobianA, impulse * inverseMassA, out var changeA);
            Vector3Wide.Scale(jacobianA, impulse * inverseMassB, out var negatedChangeB);
            Vector3Wide.Add(a.Linear, changeA, out a.Linear);
            Vector3Wide.Subtract(b.Linear, negatedChangeB, out b.Linear);
        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
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
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt,
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

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref CenterDistancePrestepData prestepData) { }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of distance servos.
    /// </summary>
    public class CenterDistanceTypeProcessor : TwoBodyTypeProcessor<CenterDistancePrestepData, Vector<float>, CenterDistanceConstraintFunctions, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear, AccessOnlyLinear>
    {
        public const int BatchTypeId = 35;
    }
}
