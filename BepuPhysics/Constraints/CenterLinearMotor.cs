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
    /// Constrains the relative linear velocity between two bodies to a target.
    /// Conceptually, controls the relative velocity by a virtual lever arm attached to the center of A and leading to the center of B.
    /// The connection to A is motorized to allow changing both the direction of the lever arm and the length (allowing full 3 DOF velocity control), while the connection to B allows unrestricted rotation.
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
        public Vector3Wide OffsetFromAToB;
        public Vector3Wide BiasVelocity;
        public Symmetric3x3Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumImpulse;
        public BodyInertias InertiaA;
        public Vector<float> InverseMassB;
    }

    public struct CenterLinearMotorFunctions : IConstraintFunctions<CenterLinearMotorPrestepData, CenterLinearMotorProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref CenterLinearMotorPrestepData prestep, out CenterLinearMotorProjection projection)
        {
            bodies.GatherPose(ref bodyReferences, count, out var offsetB, out var orientationA, out var orientationB);
            projection.InertiaA = inertiaA;
            projection.InverseMassB = inertiaB.InverseMass;

            projection.OffsetFromAToB = offsetB;
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(projection.OffsetFromAToB, projection.InertiaA.InverseInertiaTensor, out var inverseEffectiveMass);
            //The offset for B is zero, so there is no angular contribution.

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            var linearContribution = projection.InertiaA.InverseMass + projection.InverseMassB;
            inverseEffectiveMass.XX += linearContribution;
            inverseEffectiveMass.YY += linearContribution;
            inverseEffectiveMass.ZZ += linearContribution;
            Symmetric3x3Wide.Invert(inverseEffectiveMass, out projection.EffectiveMass);
            MotorSettingsWide.ComputeSoftness(prestep.Settings, dt, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale, out projection.MaximumImpulse);
            Symmetric3x3Wide.Scale(projection.EffectiveMass, effectiveMassCFMScale, out projection.EffectiveMass);

            QuaternionWide.Transform(prestep.TargetVelocityLocalA, orientationA, out projection.BiasVelocity);
            Vector3Wide.Negate(projection.BiasVelocity, out projection.BiasVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref CenterLinearMotorProjection projection, ref Vector3Wide csi)
        {
            Vector3Wide.CrossWithoutOverlap(projection.OffsetFromAToB, csi, out var wsi);
            Symmetric3x3Wide.TransformWithoutOverlap(wsi, projection.InertiaA.InverseInertiaTensor, out var angularChangeA);
            Vector3Wide.Add(velocityA.Angular, angularChangeA, out velocityA.Angular);

            Vector3Wide.Scale(csi, projection.InertiaA.InverseMass, out var linearChangeA);
            Vector3Wide.Add(velocityA.Linear, linearChangeA, out velocityA.Linear);
            
            Vector3Wide.Scale(csi, projection.InverseMassB, out var negativeLinearChangeB);
            Vector3Wide.Subtract(velocityB.Linear, negativeLinearChangeB, out velocityB.Linear); //note subtraction; the jacobian is -I
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
            Vector3Wide.CrossWithoutOverlap(velocityA.Angular, projection.OffsetFromAToB, out var angularCSV);
            Vector3Wide.Add(csv, angularCSV, out csv);
            Vector3Wide.Subtract(projection.BiasVelocity, csv, out csv);

            Symmetric3x3Wide.TransformWithoutOverlap(csv, projection.EffectiveMass, out var csi);
            Vector3Wide.Scale(accumulatedImpulse, projection.SoftnessImpulseScale, out var softness);
            Vector3Wide.Subtract(csi, softness, out csi);

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
