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
    public struct MotorSettings
    {
        public float MaximumSpeed;
        public float BaseSpeed;
        public float MaximumForce;
    }
    public struct MotorSettingsWide
    {
        public Vector<float> MaximumSpeed;
        public Vector<float> BaseSpeed;
        public Vector<float> MaximumForce;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(ref MotorSettings source, ref MotorSettingsWide target)
        {
            GetFirst(ref target.MaximumSpeed) = source.MaximumSpeed;
            GetFirst(ref target.BaseSpeed) = source.BaseSpeed;
            GetFirst(ref target.MaximumForce) = source.MaximumForce;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(ref MotorSettingsWide source, out MotorSettings target)
        {
            target.MaximumSpeed = source.MaximumSpeed[0];
            target.BaseSpeed = source.BaseSpeed[0];
            target.MaximumForce = source.MaximumForce[0];
        }
    }

    public struct GrabMotor : IConstraintDescription<GrabMotor>
    {
        public Vector3 LocalOffset;
        public SpringSettings SpringSettings;
        public MotorSettings MotorSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return GrabMotorTypeProcessor.BatchTypeId;
            }
        }

        public Type BatchType => typeof(GrabMotorTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<GrabMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GetFirst(ref target.LocalOffset.X) = LocalOffset.X;
            GetFirst(ref target.LocalOffset.Y) = LocalOffset.Y;
            GetFirst(ref target.LocalOffset.Z) = LocalOffset.Z;
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
            MotorSettingsWide.WriteFirst(ref MotorSettings, ref target.MotorSettings);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out GrabMotor description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<GrabMotorPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.LocalOffset.X = GetFirst(ref source.LocalOffset.X);
            description.LocalOffset.Y = GetFirst(ref source.LocalOffset.Y);
            description.LocalOffset.Z = GetFirst(ref source.LocalOffset.Z);
            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
            MotorSettingsWide.ReadFirst(ref source.MotorSettings, out description.MotorSettings);
        }
    }

    public struct GrabMotorPrestepData
    {
        public Vector3Wide LocalOffset;
        //TODO: This depends upon position being represented as a 32 bit floating point number.
        //That's a little problematic in the long run. You could avoid this dependency by representing the target as a relative target, but that is quite a bit more difficult to use-
        //you would have to update the target location every single time step, or else it would just continually accelerate.
        public Vector3Wide Target;
        public SpringSettingsWide SpringSettings;
        public MotorSettingsWide MotorSettings;
    }

    public struct GrabMotorProjection
    {
        public Vector3Wide Offset;
        public Vector3Wide BiasVelocity;
        public Triangular3x3Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> MaximumForce;
        public BodyInertias Inertia;
    }

    public struct GrabMotorFunctions : IOneBodyConstraintFunctions<GrabMotorPrestepData, GrabMotorProjection, Vector3Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref Vector<int> bodyReferences, int count, float dt, float inverseDt, ref GrabMotorPrestepData prestep,
            out GrabMotorProjection projection)
        {
            //TODO: Note that this grabs a world position. That poses a problem for different position representations.
            bodies.GatherInertiaAndPose(ref bodyReferences, count, out var position, out var orientation, out projection.Inertia);

            //The grabber is roughly equivalent to a ball socket joint with a nonzero goal (and only one body).

            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffset, orientation, out projection.Offset);
            Triangular3x3Wide.SkewSandwichWithoutOverlap(ref projection.Offset, ref projection.Inertia.InverseInertiaTensor, out var inverseEffectiveMass);

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            inverseEffectiveMass.XX += projection.Inertia.InverseMass;
            inverseEffectiveMass.YY += projection.Inertia.InverseMass;
            inverseEffectiveMass.ZZ += projection.Inertia.InverseMass;
            Triangular3x3Wide.SymmetricInvert(ref inverseEffectiveMass, out projection.EffectiveMass);
            SpringSettingsWide.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Triangular3x3Wide.Scale(ref projection.EffectiveMass, ref effectiveMassCFMScale, out projection.EffectiveMass);

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Add(ref projection.Offset, ref position, out var worldGrabPoint);
            Vector3Wide.Subtract(ref prestep.Target, ref worldGrabPoint, out var error);
            Vector3Wide.Scale(ref error, ref positionErrorToVelocity, out projection.BiasVelocity);
            Vector3Wide.Length(ref projection.BiasVelocity, out var speed);
            var newSpeed = Vector.Min(prestep.MotorSettings.MaximumSpeed, Vector.Max(prestep.MotorSettings.BaseSpeed, speed));
            var scale = newSpeed / speed;
            Vector3Wide.Scale(ref projection.BiasVelocity, ref scale, out var scaledVelocity);
            Vector3Wide.ConditionalSelect(Vector.GreaterThan(speed, Vector<float>.Zero), scaledVelocity, projection.BiasVelocity, out projection.BiasVelocity);

            projection.MaximumForce = prestep.MotorSettings.MaximumForce;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref GrabMotorProjection projection, ref Vector3Wide csi)
        {
            Vector3Wide.CrossWithoutOverlap(ref projection.Offset, ref csi, out var wsi);
            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref wsi, ref projection.Inertia.InverseInertiaTensor, out var change);
            Vector3Wide.Add(ref velocityA.Angular, ref change, out velocityA.Angular);

            Vector3Wide.Scale(ref csi, ref projection.Inertia.InverseMass, out change);
            Vector3Wide.Add(ref velocityA.Linear, ref change, out velocityA.Linear);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref GrabMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref GrabMotorProjection projection, ref Vector3Wide accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular);
            Vector3Wide.CrossWithoutOverlap(ref velocityA.Angular, ref projection.Offset, out var angularCSV);
            Vector3Wide.Add(ref velocityA.Linear, ref angularCSV, out var csv);

            Triangular3x3Wide.TransformBySymmetricWithoutOverlap(ref csv, ref projection.EffectiveMass, out var csi);
            Vector3Wide.Scale(ref accumulatedImpulse, ref projection.SoftnessImpulseScale, out var softness);
            Vector3Wide.Subtract(ref csi, ref softness, out csi);

            //The motor has a limited maximum force, so clamp the accumulated impulse. Watch out for division by zero.
            var previous = accumulatedImpulse;
            Vector3Wide.Add(ref accumulatedImpulse, ref csi, out accumulatedImpulse);
            Vector3Wide.Length(ref accumulatedImpulse, out var impulseMagnitude);
            var newMagnitude = Vector.Min(impulseMagnitude, projection.MaximumForce);
            var scale = newMagnitude / impulseMagnitude;
            Vector3Wide.Scale(ref accumulatedImpulse, ref scale, out accumulatedImpulse);
            Vector3Wide.ConditionalSelect(Vector.GreaterThan(impulseMagnitude, Vector<float>.Zero), accumulatedImpulse, previous, out accumulatedImpulse);
            Vector3Wide.Subtract(ref accumulatedImpulse, ref previous, out csi);

            ApplyImpulse(ref velocityA, ref projection, ref csi);

        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket constraints.
    /// </summary>
    public class GrabMotorTypeProcessor : OneBodyTypeProcessor<GrabMotorPrestepData, GrabMotorProjection, Vector3Wide, GrabMotorFunctions>
    {
        public const int BatchTypeId = 26;
    }
}
