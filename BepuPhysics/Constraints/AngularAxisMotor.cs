using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Constrains two bodies to rotate around a local axis attached to the first body at a target velocity.
    /// </summary>
    public struct AngularAxisMotor : ITwoBodyConstraintDescription<AngularAxisMotor>
    {
        /// <summary>
        /// Axis of rotation in body A's local space.
        /// </summary>
        public Vector3 LocalAxisA;
        /// <summary>
        /// Target relative angular velocity around the axis.
        /// </summary>
        public float TargetVelocity;
        /// <summary>
        /// Motor control parameters.
        /// </summary>
        public MotorSettings Settings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularAxisMotorTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(AngularAxisMotorTypeProcessor);
        
        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalAxisA, nameof(AngularAxisMotor), nameof(LocalAxisA));
            ConstraintChecker.AssertValid(Settings, nameof(AngularAxisMotor));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularAxisMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalAxisA, ref target.LocalAxisA);
            GatherScatter.GetFirst(ref target.TargetVelocity) = TargetVelocity;
            MotorSettingsWide.WriteFirst(Settings, ref target.Settings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularAxisMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularAxisMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalAxisA, out description.LocalAxisA);
            description.TargetVelocity = GatherScatter.GetFirst(ref source.TargetVelocity);
            MotorSettingsWide.ReadFirst(source.Settings, out description.Settings);
        }
    }

    public struct AngularAxisMotorPrestepData
    {
        public Vector3Wide LocalAxisA;
        public Vector<float> TargetVelocity;
        public MotorSettingsWide Settings;
    }

    public struct AngularAxisMotorProjection
    {
        public Vector3Wide VelocityToImpulseA;
        public Vector<float> BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public Vector3Wide ImpulseToVelocityA;
        public Vector3Wide NegatedImpulseToVelocityB;
    }


    public struct AngularAxisMotorFunctions : IConstraintFunctions<AngularAxisMotorPrestepData, AngularAxisMotorProjection, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref AngularAxisMotorPrestepData prestep, out AngularAxisMotorProjection projection)
        {
            //Velocity level constraint that acts directly on the given axes. Jacobians just the axes, nothing complicated. 1DOF, so we do premultiplication.
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA, out var orientationB);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalAxisA, orientationA, out var axis);
            Symmetric3x3Wide.TransformWithoutOverlap(axis, inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            Vector3Wide.Dot(axis, projection.ImpulseToVelocityA, out var contributionA);
            Symmetric3x3Wide.TransformWithoutOverlap(axis, inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Vector3Wide.Dot(axis, projection.NegatedImpulseToVelocityB, out var contributionB);
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);
            var effectiveMass = effectiveMassCFMScale / (contributionA + contributionB);

            Vector3Wide.Scale(axis, effectiveMass, out projection.VelocityToImpulseA);

            projection.BiasImpulse = prestep.TargetVelocity * effectiveMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, in AngularAxisMotorProjection projection, in Vector<float> csi)
        {
            Vector3Wide.Scale(projection.ImpulseToVelocityA, csi, out var velocityChangeA);
            Vector3Wide.Scale(projection.NegatedImpulseToVelocityB, csi, out var negatedVelocityChangeB);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularAxisMotorProjection projection, ref Vector<float> accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection, accumulatedImpulse);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularAxisMotorProjection projection, ref Vector<float> accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector3Wide.Dot(velocityA.Angular, projection.VelocityToImpulseA, out var csiA);
            Vector3Wide.Dot(velocityB.Angular, projection.VelocityToImpulseA, out var negatedCSIB);
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiA - negatedCSIB);
            ServoSettingsWide.ClampImpulse(projection.MaximumImpulse, ref accumulatedImpulse, ref csi);
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, projection, csi);

        }

    }

    public class AngularAxisMotorTypeProcessor : TwoBodyTypeProcessor<AngularAxisMotorPrestepData, AngularAxisMotorProjection, Vector<float>, AngularAxisMotorFunctions>
    {
        public const int BatchTypeId = 41;
    }
}

