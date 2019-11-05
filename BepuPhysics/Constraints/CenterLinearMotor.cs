using BepuPhysics;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
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
    /// Constrains the relative linear velocity between two bodies to a target. Has no offset anchor.
    /// </summary>
    public struct CenterLinearMotor : ITwoBodyConstraintDescription<CenterLinearMotor>
    {
        /// <summary>
        /// Target relative linear velocity between A and B, stored in A's local space. Target world space linear velocity of B is LinearVelocityA + TargetVelocityLocalA * OrientationA.
        /// </summary>
        public Vector3 TargetVelocityLocalA;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return CenterLinearMotorTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(CenterLinearMotorTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertValid(Settings, nameof(CenterLinearMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<CenterLinearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(TargetVelocityLocalA, ref target.TargetVelocityLocalA);
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out CenterLinearMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<CenterLinearMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.TargetVelocityLocalA, out description.TargetVelocityLocalA);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct CenterLinearMotorPrestepData
    {
        public Vector3Wide TargetVelocityLocalA;
        public MotorSettingsWide Settings;
    }

    public struct CenterLinearMotorProjection
    {
        public Vector<float> EffectiveMass;
        public Vector3Wide BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public Vector<float> InverseMassA;
        public Vector<float> InverseMassB;
    }

    public struct CenterLinearMotorFunctions : IConstraintFunctions<CenterLinearMotorPrestepData, CenterLinearMotorProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref CenterLinearMotorPrestepData prestep, out CenterLinearMotorProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA, out var orientationB);
            projection.InverseMassA = inertiaA.InverseMass;
            projection.InverseMassB = inertiaB.InverseMass;
            QuaternionWide.Transform(prestep.TargetVelocityLocalA, orientationA, out var targetRelativeVelocity);

            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);
            projection.EffectiveMass = effectiveMassCFMScale / (inertiaA.InverseMass + inertiaB.InverseMass);

            Vector3Wide.Scale(targetRelativeVelocity, -projection.EffectiveMass, out projection.BiasImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CenterLinearMotorProjection projection, ref Vector3Wide csi)
        {
            Vector3Wide.Scale(csi, projection.InverseMassA, out var velocityChangeA);
            Vector3Wide.Scale(csi, projection.InverseMassB, out var velocityChangeB);
            Vector3Wide.Add(velocityA.Linear, velocityChangeA, out velocityA.Linear);
            Vector3Wide.Subtract(velocityB.Linear, velocityChangeB, out velocityB.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CenterLinearMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CenterLinearMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //Note subtraction; jLinearB = -I.
            Vector3Wide.Subtract(velocityA.Linear, velocityB.Linear, out var csv);
            Vector3Wide.Scale(csv, projection.EffectiveMass, out var csiBodyContribution);
            Vector3Wide.Scale(accumulatedImpulse, projection.SoftnessImpulseScale, out var softness);
            Vector3Wide.Subtract(projection.BiasImpulse, softness, out var csi);
            Vector3Wide.Subtract(csi, csiBodyContribution, out csi);
            ServoSettingsWide.ClampImpulse(projection.MaximumImpulse, ref accumulatedImpulse, ref csi);

            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref csi);
        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket constraints.
    /// </summary>
    public class CenterLinearMotorTypeProcessor : TwoBodyTypeProcessor<CenterLinearMotorPrestepData, CenterLinearMotorProjection, Vector3Wide, CenterLinearMotorFunctions>
    {
        public const int BatchTypeId = 52;
    }
}
