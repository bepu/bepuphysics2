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
    /// Constrains a point on one body to a point on another body.
    /// </summary>
    public struct BallSocket : ITwoBodyConstraintDescription<BallSocket>
    {
        /// <summary>
        /// Offset from the center of body A to its attachment in A's local space.
        /// </summary>
        public Vector3 LocalOffsetA;
        /// <summary>
        /// Offset from the center of body B to its attachment in B's local space.
        /// </summary>
        public Vector3 LocalOffsetB;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return BallSocketTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(BallSocketTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(SpringSettings, nameof(BallSocket));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<BallSocketPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffsetA, ref target.LocalOffsetA);
            Vector3Wide.WriteFirst(LocalOffsetB, ref target.LocalOffsetB);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out BallSocket description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<BallSocketPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffsetA, out description.LocalOffsetA);
            Vector3Wide.ReadFirst(source.LocalOffsetB, out description.LocalOffsetB);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct BallSocketPrestepData
    {
        public Vector3Wide LocalOffsetA;
        public Vector3Wide LocalOffsetB;
        public SpringSettingsWide SpringSettings;
    }
    public struct BallSocketFunctions : ITwoBodyConstraintFunctions<BallSocketPrestepData, Vector3Wide>
    {
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB,
            ref BallSocketPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationA, out var offsetA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out var offsetB);
            BallSocketShared.ApplyImpulse(ref wsvA, ref wsvB, offsetA, offsetB, inertiaA, inertiaB, accumulatedImpulses);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt,
            ref BallSocketPrestepData prestep, ref Vector3Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetA, orientationA, out var offsetA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffsetB, orientationB, out var offsetB);
            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            BallSocketShared.ComputeEffectiveMass(inertiaA, inertiaB, offsetA, offsetB, effectiveMassCFMScale, out var effectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            var ab = positionB - positionA;
            Vector3Wide.Add(ab, offsetB, out var anchorB);
            Vector3Wide.Subtract(anchorB, offsetA, out var error);
            Vector3Wide.Scale(error, positionErrorToVelocity, out var biasVelocity);

            BallSocketShared.Solve(ref wsvA, ref wsvB, offsetA, offsetB, biasVelocity, effectiveMass, softnessImpulseScale, ref accumulatedImpulses, inertiaA, inertiaB);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref BallSocketPrestepData prestepData) { }
    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket constraints.
    /// </summary>
    public class BallSocketTypeProcessor : TwoBodyTypeProcessor<BallSocketPrestepData, Vector3Wide, BallSocketFunctions, AccessNoPosition, AccessNoPosition, AccessAll, AccessAll>
    {
        public const int BatchTypeId = 22;
    }
}
