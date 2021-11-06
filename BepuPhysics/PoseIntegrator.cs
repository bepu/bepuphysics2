using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using BepuUtilities.Collections;
using BepuPhysics.Constraints;

namespace BepuPhysics
{
    public interface IPoseIntegrator
    {
        void IntegrateBodiesAndUpdateBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void PredictBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void IntegrateVelocitiesBoundsAndInertias(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void IntegrateVelocitiesAndUpdateInertias(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void IntegratePoses(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void IntegrateAfterSubstepping(IndexSet constrainedBodies, float dt, int substepCount, IThreadDispatcher threadDispatcher = null);
    }

    /// <summary>
    /// Defines how a pose integrator should handle angular velocity integration.
    /// </summary>
    public enum AngularIntegrationMode
    {
        /// <summary>
        /// Angular velocity is directly integrated and does not change as the body pose changes. Does not conserve angular momentum.
        /// </summary>
        Nonconserving,
        /// <summary>
        /// Approximately conserves angular momentum by updating the angular velocity according to the change in orientation. Does a decent job for gyroscopes, but angular velocities will tend to drift towards a minimal inertia axis.
        /// </summary>
        ConserveMomentum,
        /// <summary>
        /// Approximately conserves angular momentum by including an implicit gyroscopic torque. Best option for Dzhanibekov effect simulation, but applies a damping effect that can make gyroscopes less useful.
        /// </summary>
        ConserveMomentumWithGyroscopicTorque,
    }

    /// <summary>
    /// Defines a type that handles callbacks for body pose integration.
    /// </summary>
    public interface IPoseIntegratorCallbacks
    {
        /// <summary>
        /// Gets how the pose integrator should handle angular velocity integration.
        /// </summary>
        AngularIntegrationMode AngularIntegrationMode { get; }

        /// <summary>
        /// Gets whether the integrator should use only one step for unconstrained bodies when using a substepping solver.
        /// If true, unconstrained bodies use a single step of length equal to the dt provided to Simulation.Timestep. 
        /// If false, unconstrained bodies will be integrated with the same number of substeps as the constrained bodies in the solver.
        /// </summary>
        bool AllowSubstepsForUnconstrainedBodies { get; }

        /// <summary>
        /// Gets whether the velocity integration callback should be called for kinematic bodies.
        /// If true, IntegrateVelocity will be called for bundles including kinematic bodies.
        /// If false, kinematic bodies will just continue using whatever velocity they have set.
        /// Most use cases should set this to false.
        /// </summary>
        bool IntegrateVelocityForKinematics { get; }


        /// <summary>
        /// Performs any required initialization logic after the Simulation instance has been constructed.
        /// </summary>
        /// <param name="simulation">Simulation that owns these callbacks.</param>
        void Initialize(Simulation simulation);

        /// <summary>
        /// Called prior to integrating the simulation's active bodies. When used with a substepping timestepper, this could be called multiple times per frame with different time step values.
        /// </summary>
        /// <param name="dt">Current time step duration.</param>
        void PrepareForIntegration(float dt);

        /// <summary>
        /// Callback called for each active body within the simulation during body integration.
        /// </summary>
        /// <param name="bodyIndex">Index of the body being visited.</param>
        /// <param name="pose">Body's current pose.</param>
        /// <param name="localInertia">Body's current local inertia.</param>
        /// <param name="workerIndex">Index of the worker thread processing this body.</param>
        /// <param name="velocity">Reference to the body's current velocity to integrate.</param>
        void IntegrateVelocity(int bodyIndex, in RigidPose pose, in BodyInertia localInertia, int workerIndex, ref BodyVelocity velocity);


        /// <summary>
        /// Callback for a bundle of bodies being integrated.
        /// </summary>
        /// <param name="bodyIndices">Indices of the bodies being integrated in this bundle.</param>
        /// <param name="position">Current body positions.</param>
        /// <param name="orientation">Current body orientations.</param>
        /// <param name="localInertia">Body's current local inertia.</param>
        /// <param name="integrationMask">Mask indicating which lanes are active in the bundle. Active lanes will contain 0xFFFFFFFF, inactive lanes will contain 0. Lanes beyond bodyIndices.Length are undefined.</param>
        /// <param name="workerIndex">Index of the worker thread processing this bundle.</param>
        /// <param name="dt">Durations to integrate the velocity over. Can vary over lanes.</param>
        /// <param name="velocity">Velocity of bodies in the bundle. Any changes to lanes which are not active by the integrationMask will be discarded.</param>
        void IntegrateVelocity(
            in Vector<int> bodyIndices, in Vector3Wide position, in QuaternionWide orientation, in BodyInertiaWide localInertia,
            in Vector<int> integrationMask, int workerIndex, in Vector<float> dt, ref BodyVelocityWide velocity);
    }

    /// <summary>
    /// Provides helper functions for integrating body poses.
    /// </summary>
    public static class PoseIntegration
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotateInverseInertia(in Symmetric3x3 localInverseInertiaTensor, in Quaternion orientation, out Symmetric3x3 rotatedInverseInertiaTensor)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var orientationMatrix);
            //I^-1 = RT * Ilocal^-1 * R 
            //NOTE: If you were willing to confuse users a little bit, the local inertia could be required to be diagonal.
            //This would be totally fine for all the primitive types which happen to have diagonal inertias, but for more complex shapes (convex hulls, meshes), 
            //there would need to be a reorientation step. That could be confusing, and it's probably not worth it.
            Symmetric3x3.RotationSandwich(orientationMatrix, localInverseInertiaTensor, out rotatedInverseInertiaTensor);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Integrate(in Vector3 position, in Vector3 linearVelocity, float dt, out Vector3 integratedPosition)
        {
            position.Validate();
            linearVelocity.Validate();
            var displacement = linearVelocity * dt;
            integratedPosition = position + displacement;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Integrate(in Quaternion orientation, in Vector3 angularVelocity, float dt, out Quaternion integratedOrientation)
        {
            orientation.ValidateOrientation();
            angularVelocity.Validate();
            //Note that we don't bother with conservation of angular momentum or the gyroscopic term or anything else. All orientation integration assumes a series of piecewise linear integrations
            //That's not entirely correct, but it's a reasonable approximation that means we don't have to worry about conservation of angular momentum or gyroscopic terms when dealing with CCD sweeps.
            var speed = angularVelocity.Length();
            if (speed > 1e-15f)
            {
                var halfAngle = speed * dt * 0.5f;
                Unsafe.SkipInit(out Quaternion q);
                Unsafe.As<Quaternion, Vector3>(ref q) = angularVelocity * (MathHelper.Sin(halfAngle) / speed);
                q.W = MathHelper.Cos(halfAngle);
                //Note that the input and output may overlap.
                QuaternionEx.Concatenate(orientation, q, out integratedOrientation);
                QuaternionEx.Normalize(ref integratedOrientation);
            }
            else
            {
                integratedOrientation = orientation;
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Integrate(in QuaternionWide start, in Vector3Wide angularVelocity, in Vector<float> halfDt, out QuaternionWide integrated)
        {
            Vector3Wide.Length(angularVelocity, out var speed);
            var halfAngle = speed * halfDt;
            QuaternionWide q;
            MathHelper.Sin(halfAngle, out var s);
            var scale = s / speed;
            q.X = angularVelocity.X * scale;
            q.Y = angularVelocity.Y * scale;
            q.Z = angularVelocity.Z * scale;
            MathHelper.Cos(halfAngle, out q.W);
            QuaternionWide.ConcatenateWithoutOverlap(start, q, out var end);
            end = QuaternionWide.Normalize(end);
            var speedValid = Vector.GreaterThan(speed, new Vector<float>(1e-15f));
            integrated.X = Vector.ConditionalSelect(speedValid, end.X, start.X);
            integrated.Y = Vector.ConditionalSelect(speedValid, end.Y, start.Y);
            integrated.Z = Vector.ConditionalSelect(speedValid, end.Z, start.Z);
            integrated.W = Vector.ConditionalSelect(speedValid, end.W, start.W);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotateInverseInertia(in Symmetric3x3Wide localInverseInertiaTensor, in QuaternionWide orientation, out Symmetric3x3Wide rotatedInverseInertiaTensor)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientation, out var orientationMatrix);
            //I^-1 = RT * Ilocal^-1 * R 
            //NOTE: If you were willing to confuse users a little bit, the local inertia could be required to be diagonal.
            //This would be totally fine for all the primitive types which happen to have diagonal inertias, but for more complex shapes (convex hulls, meshes), 
            //there would need to be a reorientation step. That could be confusing, and it's probably not worth it.
            Symmetric3x3Wide.RotationSandwich(orientationMatrix, localInverseInertiaTensor, out rotatedInverseInertiaTensor);
        }

        /// <summary>
        /// Uses the previous angular velocity if attempting to conserve angular momentum introduced infinities or NaNs. Happens when attempting to conserve momentum with a kinematic or partially inertia locked body.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FallbackIfInertiaIncompatible(in Vector3Wide previousAngularVelocity, ref Vector3Wide angularVelocity)
        {
            var infinity = new Vector<float>(float.PositiveInfinity);
            var useNewVelocity = Vector.BitwiseAnd(Vector.LessThan(Vector.Abs(angularVelocity.X), infinity), Vector.BitwiseAnd(
                Vector.LessThan(Vector.Abs(angularVelocity.Y), infinity), 
                Vector.LessThan(Vector.Abs(angularVelocity.Z), infinity)));
            angularVelocity.X = Vector.ConditionalSelect(useNewVelocity, angularVelocity.X, previousAngularVelocity.X);
            angularVelocity.Y = Vector.ConditionalSelect(useNewVelocity, angularVelocity.Y, previousAngularVelocity.Y);
            angularVelocity.Z = Vector.ConditionalSelect(useNewVelocity, angularVelocity.Z, previousAngularVelocity.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void IntegrateAngularVelocityConserveMomentum(in QuaternionWide previousOrientation, in Symmetric3x3Wide localInverseInertia, in Symmetric3x3Wide worldInverseInertia, ref Vector3Wide angularVelocity)
        {
            //Note that this effectively recomputes the previous frame's inertia. There may not have been a previous inertia stored in the inertias buffer.
            //This just avoids the need for quite a bit of complexity around keeping the world inertias buffer updated with adds/removes/moves and other state changes that we can't easily track.
            //Also, even if it were cached, the memory bandwidth requirements of loading another inertia tensor would hurt multithreaded scaling enough to eliminate any performance advantage.
            Matrix3x3Wide.CreateFromQuaternion(previousOrientation, out var previousOrientationMatrix);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(angularVelocity, previousOrientationMatrix, out var localPreviousAngularVelocity);
            Symmetric3x3Wide.Invert(localInverseInertia, out var localInertiaTensor);
            Symmetric3x3Wide.TransformWithoutOverlap(localPreviousAngularVelocity, localInertiaTensor, out var localAngularMomentum);
            Matrix3x3Wide.Transform(localAngularMomentum, previousOrientationMatrix, out var angularMomentum);
            var previousVelocity = angularVelocity;
            Symmetric3x3Wide.TransformWithoutOverlap(angularMomentum, worldInverseInertia, out angularVelocity);
            FallbackIfInertiaIncompatible(previousVelocity, ref angularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void IntegrateAngularVelocityConserveMomentumWithGyroscopicTorque(
            in QuaternionWide orientation, in Symmetric3x3Wide localInverseInertia, ref Vector3Wide angularVelocity, in Vector<float> dt)
        {
            //Integrating the gyroscopic force explicitly can result in some instability, so we'll use an approximate implicit approach.
            //angularVelocity1 * inertia1 = angularVelocity0 * inertia1 + dt * ((angularVelocity1 * inertia1) x angularVelocity1)
            //Note that this includes a reference to inertia1 which doesn't exist yet. We do, however, have the local inertia, so we'll 
            //transform all velocities into local space using the current orientation for the calculation.
            //So:
            //localAngularVelocity1 * localInertia = localAngularVelocity0 * localInertia - dt * (localAngularVelocity1 x (localAngularVelocity1 * localInertia))
            //localAngularVelocity1 * localInertia - localAngularVelocity0 * localInertia + dt * (localAngularVelocity1 x (localAngularVelocity1 * localInertia)) = 0 
            //f(localAngularVelocity1) = (localAngularVelocity1 - localAngularVelocity0) * localInertia + dt * (localAngularVelocity1 x (localAngularVelocity1 * localInertia))
            //Not trivial to solve for localAngularVelocity1 so we'll do so numerically with a newton iteration.
            //(For readers familiar with Bullet's BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY, this is basically identical.)

            //We'll start with an initial guess of localAngularVelocity1 = localAngularVelocity0, and update with a newton step of f(localAngularVelocity1) * invert(df/dw1(localAngularVelocity1))
            //df/dw1x(localAngularVelocity1) * localInertia + dt * (df/dw1x(localAngularVelocity1) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x df/dw1x(localAngularVelocity1 * localInertia))
            //df/dw1x(localAngularVelocity1) = (1,0,0)
            //df/dw1x(f(localAngularVelocity1)) = (1, 0, 0) * localInertia + dt * ((1, 0, 0) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x ((1, 0, 0) * localInertia))
            //df/dw1x(f(localAngularVelocity1)) = (0, 1, 0) * localInertia + dt * ((0, 1, 0) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x ((0, 1, 0) * localInertia))
            //df/dw1x(f(localAngularVelocity1)) = (0, 0, 1) * localInertia + dt * ((0, 0, 1) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x ((0, 0, 1) * localInertia))
            //This can be expressed a bit more concisely, given a x b = skew(a) * b, where skew(a) is a skew symmetric matrix representing a cross product: 
            //df/dw1(f(localAngularVelocity1)) = localInertia + dt * (skew(localAngularVelocity1) * localInertia - skew(localAngularVelocity1 * localInertia))
            Matrix3x3Wide.CreateFromQuaternion(orientation, out var orientationMatrix);
            //Using localAngularVelocity0 as the first guess for localAngularVelocity1.
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(angularVelocity, orientationMatrix, out var localAngularVelocity);
            Symmetric3x3Wide.Invert(localInverseInertia, out var localInertiaTensor);

            Symmetric3x3Wide.TransformWithoutOverlap(localAngularVelocity, localInertiaTensor, out var localAngularMomentum);
            var residual = dt * Vector3Wide.Cross(localAngularMomentum, localAngularVelocity);

            Matrix3x3Wide.CreateCrossProduct(localAngularMomentum, out var skewMomentum);
            Matrix3x3Wide.CreateCrossProduct(localAngularVelocity, out var skewVelocity);
            var transformedSkewVelocity = skewVelocity * localInertiaTensor;
            Matrix3x3Wide.Subtract(transformedSkewVelocity, skewMomentum, out var changeOverDt);
            Matrix3x3Wide.Scale(changeOverDt, dt, out var change);
            var jacobian = localInertiaTensor + change;

            Matrix3x3Wide.Invert(jacobian, out var inverseJacobian);
            Matrix3x3Wide.Transform(residual, inverseJacobian, out var newtonStep);
            localAngularVelocity -= newtonStep;

            var previousVelocity = angularVelocity;
            Matrix3x3Wide.Transform(localAngularVelocity, orientationMatrix, out angularVelocity);
            FallbackIfInertiaIncompatible(previousVelocity, ref angularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void Integrate(in RigidPose pose, in BodyVelocity velocity, float dt, out RigidPose integratedPose)
        {
            Integrate(pose.Position, velocity.Linear, dt, out integratedPose.Position);
            Integrate(pose.Orientation, velocity.Angular, dt, out integratedPose.Orientation);
        }
    }


    /// <summary>
    /// Integrates the velocity of mobile bodies over time into changes in position and orientation. Also applies gravitational acceleration to dynamic bodies.
    /// </summary>
    /// <remarks>
    /// This variant of the integrator uses a single global gravity. Other integrators that provide per-entity gravity could exist later.
    /// This integrator also assumes that the bodies positions are stored in terms of single precision floats. Later on, we will likely modify the Bodies
    /// storage to allow different representations for larger simulations. That will require changes in this integrator, the relative position calculation of collision detection,
    /// the bounding box calculation, and potentially even in the broadphase in extreme cases (64 bit per component positions).
    /// </remarks>
    public class PoseIntegrator<TCallbacks> : IPoseIntegrator where TCallbacks : IPoseIntegratorCallbacks
    {
        Bodies bodies;
        Shapes shapes;
        BroadPhase broadPhase;

        public TCallbacks Callbacks;

        Action<int> integrateBodiesAndUpdateBoundingBoxesWorker;
        Action<int> predictBoundingBoxesWorker;
        Action<int> integrateVelocitiesBoundsAndInertiasWorker;
        Action<int> integrateVelocitiesWorker;
        Action<int> integratePosesWorker;
        public PoseIntegrator(Bodies bodies, Shapes shapes, BroadPhase broadPhase, TCallbacks callbacks)
        {
            this.bodies = bodies;
            this.shapes = shapes;
            this.broadPhase = broadPhase;
            Callbacks = callbacks;
            integrateBodiesAndUpdateBoundingBoxesWorker = IntegrateBodiesAndUpdateBoundingBoxesWorker;
            predictBoundingBoxesWorker = PredictBoundingBoxesWorker;
            integrateVelocitiesBoundsAndInertiasWorker = IntegrateVelocitiesBoundsAndInertiasWorker;
            integrateVelocitiesWorker = IntegrateVelocitiesWorker;
            integratePosesWorker = IntegratePosesWorker;
            integrateAfterSubsteppingWorker = IntegrateAfterSubsteppingWorker;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateSleepCandidacy(ref BodyVelocity velocity, ref BodyActivity activity)
        {
            var velocityHeuristic = velocity.Linear.LengthSquared() + velocity.Angular.LengthSquared();
            if (velocityHeuristic > activity.SleepThreshold)
            {
                activity.TimestepsUnderThresholdCount = 0;
                activity.SleepCandidate = false;
            }
            else
            {
                if (activity.TimestepsUnderThresholdCount < byte.MaxValue)
                {
                    ++activity.TimestepsUnderThresholdCount;
                    if (activity.TimestepsUnderThresholdCount >= activity.MinimumTimestepsUnderThreshold)
                    {
                        activity.SleepCandidate = true;
                    }
                }
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void IntegrateAngularVelocityConserving(in Quaternion previousOrientation, in RigidPose pose, in BodyInertia localInertia, in BodyInertia inertia, ref Vector3 angularVelocity, float dt)
        {
            previousOrientation.ValidateOrientation();
            pose.Orientation.ValidateOrientation();
            angularVelocity.Validate();

            if (Callbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentum)
            {
                //Note that this effectively recomputes the previous frame's inertia. There may not have been a previous inertia stored in the inertias buffer.
                //This just avoids the need for quite a bit of complexity around keeping the world inertias buffer updated with adds/removes/moves and other state changes that we can't easily track.
                //Also, even if it were cached, the memory bandwidth requirements of loading another inertia tensor would hurt multithreaded scaling enough to eliminate any performance advantage.
                Matrix3x3.CreateFromQuaternion(previousOrientation, out var previousOrientationMatrix);
                Matrix3x3.TransformTranspose(angularVelocity, previousOrientationMatrix, out var localPreviousAngularVelocity);
                Symmetric3x3.Invert(localInertia.InverseInertiaTensor, out var localInertiaTensor);
                Symmetric3x3.TransformWithoutOverlap(localPreviousAngularVelocity, localInertiaTensor, out var localAngularMomentum);
                Matrix3x3.Transform(localAngularMomentum, previousOrientationMatrix, out var angularMomentum);
                Symmetric3x3.TransformWithoutOverlap(angularMomentum, inertia.InverseInertiaTensor, out angularVelocity);
            }

            //Note that this mode branch is optimized out for any callbacks that return a constant value.
            if (Callbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque)
            {
                //Integrating the gyroscopic force explicitly can result in some instability, so we'll use an approximate implicit approach.
                //angularVelocity1 * inertia1 = angularVelocity0 * inertia1 + dt * ((angularVelocity1 * inertia1) x angularVelocity1)
                //Note that this includes a reference to inertia1 which doesn't exist yet. We do, however, have the local inertia, so we'll 
                //transform all velocities into local space using the current orientation for the calculation.
                //So:
                //localAngularVelocity1 * localInertia = localAngularVelocity0 * localInertia - dt * (localAngularVelocity1 x (localAngularVelocity1 * localInertia))
                //localAngularVelocity1 * localInertia - localAngularVelocity0 * localInertia + dt * (localAngularVelocity1 x (localAngularVelocity1 * localInertia)) = 0 
                //f(localAngularVelocity1) = (localAngularVelocity1 - localAngularVelocity0) * localInertia + dt * (localAngularVelocity1 x (localAngularVelocity1 * localInertia))
                //Not trivial to solve for localAngularVelocity1 so we'll do so numerically with a newton iteration.
                //(For readers familiar with Bullet's BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY, this is basically identical.)

                //We'll start with an initial guess of localAngularVelocity1 = localAngularVelocity0, and update with a newton step of f(localAngularVelocity1) * invert(df/dw1(localAngularVelocity1))
                //df/dw1x(localAngularVelocity1) * localInertia + dt * (df/dw1x(localAngularVelocity1) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x df/dw1x(localAngularVelocity1 * localInertia))
                //df/dw1x(localAngularVelocity1) = (1,0,0)
                //df/dw1x(f(localAngularVelocity1)) = (1, 0, 0) * localInertia + dt * ((1, 0, 0) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x ((1, 0, 0) * localInertia))
                //df/dw1x(f(localAngularVelocity1)) = (0, 1, 0) * localInertia + dt * ((0, 1, 0) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x ((0, 1, 0) * localInertia))
                //df/dw1x(f(localAngularVelocity1)) = (0, 0, 1) * localInertia + dt * ((0, 0, 1) x (localAngularVelocity1 * localInertia) + localAngularVelocity1 x ((0, 0, 1) * localInertia))
                //This can be expressed a bit more concisely, given a x b = skew(a) * b, where skew(a) is a skew symmetric matrix representing a cross product: 
                //df/dw1(f(localAngularVelocity1)) = localInertia + dt * (skew(localAngularVelocity1) * localInertia - skew(localAngularVelocity1 * localInertia))
                Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientationMatrix);
                //Using localAngularVelocity0 as the first guess for localAngularVelocity1.
                Matrix3x3.TransformTranspose(angularVelocity, orientationMatrix, out var localAngularVelocity);
                Symmetric3x3.Invert(localInertia.InverseInertiaTensor, out var localInertiaTensor);

                Symmetric3x3.TransformWithoutOverlap(localAngularVelocity, localInertiaTensor, out var localAngularMomentum);
                var residual = dt * Vector3.Cross(localAngularMomentum, localAngularVelocity);

                Matrix3x3.CreateCrossProduct(localAngularMomentum, out var skewMomentum);
                Matrix3x3.CreateCrossProduct(localAngularVelocity, out var skewVelocity);
                Symmetric3x3.Multiply(skewVelocity, localInertiaTensor, out var transformedSkewVelocity);
                Matrix3x3.Subtract(transformedSkewVelocity, skewMomentum, out var changeOverDt);
                Matrix3x3.Scale(changeOverDt, dt, out var change);
                Symmetric3x3.Add(change, localInertiaTensor, out var jacobian);

                Matrix3x3.Invert(jacobian, out var inverseJacobian);
                Matrix3x3.Transform(residual, inverseJacobian, out var newtonStep);
                localAngularVelocity -= newtonStep;

                Matrix3x3.Transform(localAngularVelocity, orientationMatrix, out angularVelocity);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void IntegrateAngularVelocity(in Quaternion previousOrientation, in RigidPose pose, in BodyInertia localInertia, in BodyInertia inertia, ref Vector3 angularVelocity, float dt)
        {
            //Note that this mode branch is optimized out for any callbacks that return a constant value.
            if ((int)Callbacks.AngularIntegrationMode >= (int)AngularIntegrationMode.ConserveMomentum)
            {
                if (!Bodies.HasLockedInertia(localInertia.InverseInertiaTensor))
                {
                    IntegrateAngularVelocityConserving(previousOrientation, pose, localInertia, inertia, ref angularVelocity, dt);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void IntegrateAngularVelocity(in RigidPose pose, in BodyInertia localInertia, in BodyInertia inertia, ref Vector3 angularVelocity, float dt)
        {
            //We didn't have a previous orientation available. Reconstruct it by integrating backwards.
            //(In single threaded terms, caching this information could be faster, but it adds a lot of complexity and could end up reducing performance on higher core counts.)
            //Note that this mode branch is optimized out for any callbacks that return a constant value.
            if ((int)Callbacks.AngularIntegrationMode >= (int)AngularIntegrationMode.ConserveMomentum)
            {
                if (!Bodies.HasLockedInertia(localInertia.InverseInertiaTensor))
                {
                    PoseIntegration.Integrate(pose.Orientation, angularVelocity, -dt, out var previousOrientation);
                    IntegrateAngularVelocityConserving(previousOrientation, pose, localInertia, inertia, ref angularVelocity, dt);
                }
            }
        }

        unsafe void IntegrateBodiesAndUpdateBoundingBoxes(int startIndex, int endIndex, float dt, ref BoundingBoxBatcher boundingBoxBatcher, int workerIndex)
        {
            ref var baseStates = ref bodies.ActiveSet.SolverStates[0];
            ref var baseActivity = ref bodies.ActiveSet.Activity[0];
            ref var baseCollidable = ref bodies.ActiveSet.Collidables[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var state = ref Unsafe.Add(ref baseStates, i);

                var previousOrientation = state.Motion.Pose.Orientation; //This is unused if conservation of angular momentum is disabled... compiler *may* remove it...
                ref var motion = ref state.Motion;
                ref var inertia = ref state.Inertia;
                PoseIntegration.Integrate(motion.Pose, motion.Velocity, dt, out motion.Pose);

                //Note that this generally is used before velocity integration. That means an object can go inactive with gravity-induced velocity.
                //That is actually intended: when the narrowphase wakes up an island, the accumulated impulses in the island will be ready for gravity's influence.
                //To do otherwise would hurt the solver's guess, reducing the quality of the solve and possibly causing a little bump.
                //This is only relevant when the update order actually puts the sleeper after gravity. For ease of use, this fact may be ignored by the simulation update order.
                UpdateSleepCandidacy(ref state.Motion.Velocity, ref Unsafe.Add(ref baseActivity, i));

                //Update the inertia tensors for the new orientation.
                //TODO: If the pose integrator is positioned at the end of an update, the first frame after any out-of-timestep orientation change or local inertia change
                //has to get is inertia tensors calculated elsewhere. Either they would need to be computed on addition or something- which is a bit gross, but doable-
                //or we would need to move this calculation to the beginning of the frame to guarantee that all inertias are up to date. 
                //This would require a scan through all pose memory to support, but if you do it at the same time as AABB update, that's fine- that stage uses the pose too.Inertias, i);
                PoseIntegration.RotateInverseInertia(inertia.Local.InverseInertiaTensor, motion.Pose.Orientation, out inertia.World.InverseInertiaTensor);
                //While it's a bit goofy just to copy over the inverse mass every frame even if it doesn't change,
                //it's virtually always gathered together with the inertia tensor and having a duplicate means we can sometimes avoid loading a lane
                //(i.e. loading only the last 32 bytes of the cache line into a Vector256).
                inertia.World.InverseMass = inertia.Local.InverseMass;

                IntegrateAngularVelocity(previousOrientation, motion.Pose, inertia.Local, inertia.World, ref motion.Velocity.Angular, dt);
                Callbacks.IntegrateVelocity(i, motion.Pose, inertia.Local, workerIndex, ref motion.Velocity);

                //Bounding boxes are accumulated in a scalar fashion, but the actual bounding box calculations are deferred until a sufficient number of collidables are accumulated to make
                //executing a bundle worthwhile. This does two things: 
                //1) SIMD can be used to do the mathy bits of bounding box calculation. The calculations are usually pretty cheap, 
                //but they will often be more expensive than the pose stuff above.
                //2) The number of virtual function invocations required is reduced by a factor equal to the size of the accumulator cache.
                //Note that the accumulator caches are kept relatively small so that it is very likely that the pose and velocity of the collidable's body will still be in L1 cache
                //when it comes time to actually compute bounding boxes.

                //Note that any collidable that lacks a collidable, or any reference that is beyond the set of collidables, will have a specially formed index.
                //The accumulator will detect that and not try to add a nonexistent collidable.
                boundingBoxBatcher.Add(i, motion.Pose, motion.Velocity, Unsafe.Add(ref baseCollidable, i));

                //It's helpful to do the bounding box update here in the pose integrator because they share information. If the phases were split, there could be a penalty
                //associated with loading all the body poses and velocities from memory again. Even if the L3 cache persisted, it would still be worse than looking into L1 or L2.
                //Also, the pose integrator in isolation is extremely memory bound to the point where it can hardly benefit from multithreading. By interleaving some less memory bound
                //work into the mix, we can hopefully fill some execution gaps.
            }
        }

        unsafe void PredictBoundingBoxes(int startIndex, int endIndex, float dt, ref BoundingBoxBatcher boundingBoxBatcher, int workerIndex)
        {
            ref var baseStates = ref bodies.ActiveSet.SolverStates[0];
            ref var baseActivity = ref bodies.ActiveSet.Activity[0];
            ref var baseCollidable = ref bodies.ActiveSet.Collidables[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var state = ref Unsafe.Add(ref baseStates, i);
                ref var motion = ref state.Motion;
                motion.Pose.Position.Validate();
                motion.Pose.Orientation.ValidateOrientation();
                motion.Velocity.Linear.Validate();
                motion.Velocity.Angular.Validate();

                UpdateSleepCandidacy(ref motion.Velocity, ref Unsafe.Add(ref baseActivity, i));

                //Bounding box prediction does not need to update inertia tensors.                
                var integratedVelocity = motion.Velocity;
                Callbacks.IntegrateVelocity(i, motion.Pose, state.Inertia.Local, workerIndex, ref integratedVelocity);

                //Note that we do not include fancier angular integration for the bounding box prediction- it's not very important.
                boundingBoxBatcher.Add(i, motion.Pose, integratedVelocity, Unsafe.Add(ref baseCollidable, i));
            }
        }

        unsafe void IntegrateVelocitiesBoundsAndInertias(int startIndex, int endIndex, float dt, ref BoundingBoxBatcher boundingBoxBatcher, int workerIndex)
        {
            ref var baseStates = ref bodies.ActiveSet.SolverStates[0];
            ref var baseActivity = ref bodies.ActiveSet.Activity[0];
            ref var baseCollidable = ref bodies.ActiveSet.Collidables[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var state = ref Unsafe.Add(ref baseStates, i);
                ref var motion = ref state.Motion;
                motion.Pose.Position.Validate();
                motion.Pose.Orientation.ValidateOrientation();
                motion.Velocity.Linear.Validate();
                motion.Velocity.Angular.Validate();

                UpdateSleepCandidacy(ref motion.Velocity, ref Unsafe.Add(ref baseActivity, i));

                ref var inertia = ref state.Inertia;
                PoseIntegration.RotateInverseInertia(inertia.Local.InverseInertiaTensor, motion.Pose.Orientation, out inertia.World.InverseInertiaTensor);
                inertia.World.InverseMass = inertia.Local.InverseMass;

                IntegrateAngularVelocity(motion.Pose, inertia.Local, inertia.World, ref motion.Velocity.Angular, dt);
                Callbacks.IntegrateVelocity(i, motion.Pose, inertia.Local, workerIndex, ref motion.Velocity);

                boundingBoxBatcher.Add(i, motion.Pose, motion.Velocity, Unsafe.Add(ref baseCollidable, i));
            }
        }


        unsafe void IntegrateVelocities(int startIndex, int endIndex, float dt, int workerIndex)
        {
            ref var baseStates = ref bodies.ActiveSet.SolverStates[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var state = ref Unsafe.Add(ref baseStates, i);
                ref var motion = ref state.Motion;
                motion.Pose.Position.Validate();
                motion.Pose.Orientation.ValidateOrientation();
                motion.Velocity.Linear.Validate();
                motion.Velocity.Angular.Validate();

                ref var inertia = ref state.Inertia;
                PoseIntegration.RotateInverseInertia(inertia.Local.InverseInertiaTensor, motion.Pose.Orientation, out inertia.World.InverseInertiaTensor);
                inertia.World.InverseMass = inertia.Local.InverseMass;

                IntegrateAngularVelocity(motion.Pose, inertia.Local, inertia.World, ref motion.Velocity.Angular, dt);
                Callbacks.IntegrateVelocity(i, motion.Pose, inertia.Local, workerIndex, ref motion.Velocity);
            }
        }

        unsafe void IntegratePoses(int startIndex, int endIndex, float dt, int workerIndex)
        {
            ref var baseStates = ref bodies.ActiveSet.SolverStates[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var state = ref Unsafe.Add(ref baseStates, i);
                ref var motion = ref state.Motion;
                motion.Pose.Position.Validate();
                motion.Pose.Orientation.ValidateOrientation();
                motion.Velocity.Linear.Validate();
                motion.Velocity.Angular.Validate();

                PoseIntegration.Integrate(motion.Pose, motion.Velocity, dt, out motion.Pose);
            }
        }


        float cachedDt;
        int jobSize;
        int substepCount;
        IThreadDispatcher threadDispatcher;

        //Note that we aren't using a very cache-friendly work distribution here.
        //This is working on the assumption that the jobs will be large enough that border region cache misses won't be a big concern.
        //If this turns out to be false, this could be swapped over to a system similar to the solver-
        //preschedule offset regions for each worker to allow each one to consume a contiguous region before workstealing.
        int availableJobCount;
        bool TryGetJob(int maximumJobInterval, out int start, out int exclusiveEnd)
        {
            var jobIndex = Interlocked.Decrement(ref availableJobCount);
            if (jobIndex < 0)
            {
                start = 0;
                exclusiveEnd = 0;
                return false;
            }
            start = jobIndex * jobSize;
            exclusiveEnd = start + jobSize;
            if (exclusiveEnd > maximumJobInterval)
                exclusiveEnd = maximumJobInterval;
            Debug.Assert(exclusiveEnd > start, "Jobs that would involve bundles beyond the body count should not be created.");
            return true;
        }

        void IntegrateBodiesAndUpdateBoundingBoxesWorker(int workerIndex)
        {
            var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, threadDispatcher.GetThreadMemoryPool(workerIndex), cachedDt);
            var bodyCount = bodies.ActiveSet.Count;
            while (TryGetJob(bodyCount, out var start, out var exclusiveEnd))
            {
                IntegrateBodiesAndUpdateBoundingBoxes(start, exclusiveEnd, cachedDt, ref boundingBoxUpdater, workerIndex);
            }
            boundingBoxUpdater.Flush();

        }

        void PredictBoundingBoxesWorker(int workerIndex)
        {
            var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, threadDispatcher.GetThreadMemoryPool(workerIndex), cachedDt);
            var bodyCount = bodies.ActiveSet.Count;
            while (TryGetJob(bodyCount, out var start, out var exclusiveEnd))
            {
                PredictBoundingBoxes(start, exclusiveEnd, cachedDt, ref boundingBoxUpdater, workerIndex);
            }
            boundingBoxUpdater.Flush();
        }

        void IntegrateVelocitiesBoundsAndInertiasWorker(int workerIndex)
        {
            var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, threadDispatcher.GetThreadMemoryPool(workerIndex), cachedDt);
            var bodyCount = bodies.ActiveSet.Count;
            while (TryGetJob(bodyCount, out var start, out var exclusiveEnd))
            {
                IntegrateVelocitiesBoundsAndInertias(start, exclusiveEnd, cachedDt, ref boundingBoxUpdater, workerIndex);
            }
            boundingBoxUpdater.Flush();
        }

        void IntegrateVelocitiesWorker(int workerIndex)
        {
            var bodyCount = bodies.ActiveSet.Count;
            while (TryGetJob(bodyCount, out var start, out var exclusiveEnd))
            {
                IntegrateVelocities(start, exclusiveEnd, cachedDt, workerIndex);
            }
        }

        void IntegratePosesWorker(int workerIndex)
        {
            var bodyCount = bodies.ActiveSet.Count;
            while (TryGetJob(bodyCount, out var start, out var exclusiveEnd))
            {
                IntegratePoses(start, exclusiveEnd, cachedDt, workerIndex);
            }
        }

        void PrepareForMultithreadedExecution(int loopIterationCount, float dt, int workerCount, int substepCount = 1)
        {
            cachedDt = dt;
            this.substepCount = substepCount;
            const int jobsPerWorker = 2;
            var targetJobCount = workerCount * jobsPerWorker;
            jobSize = loopIterationCount / targetJobCount;
            if (jobSize == 0)
                jobSize = 1;
            availableJobCount = loopIterationCount / jobSize;
            if (jobSize * availableJobCount < loopIterationCount)
                ++availableJobCount;
        }


        public void IntegrateBodiesAndUpdateBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            Callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                //While we do technically support multithreading here, scaling is going to be really, really bad if the simulation gets kicked out of L3 cache in between frames.
                //The ratio of memory loads to actual compute work in this stage is extremely high, so getting scaling of 1.2x on a quad core is quite possible.
                //On the upside, it is a very short stage. With any luck, one or more of the following will hold:
                //1) the system has silly fast RAM,
                //2) the CPU supports octochannel memory and just brute forces the issue,
                //3) whatever the application is doing doesn't evict the entire L3 cache between frames.

                //Note that this bottleneck means the fact that we're working through bodies in a nonvectorized fashion (in favor of optimizing storage for solver access) is not a problem.

                PrepareForMultithreadedExecution(bodies.ActiveSet.Count, dt, threadDispatcher.ThreadCount);
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(integrateBodiesAndUpdateBoundingBoxesWorker);
                this.threadDispatcher = null;
            }
            else
            {
                var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, pool, dt);
                IntegrateBodiesAndUpdateBoundingBoxes(0, bodies.ActiveSet.Count, dt, ref boundingBoxUpdater, 0);
                boundingBoxUpdater.Flush();
            }
        }

        public void PredictBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            //No need to ensure inertias capacity here; world inertias are not computed during bounding box prediction.
            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            Callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(bodies.ActiveSet.Count, dt, threadDispatcher.ThreadCount);
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(predictBoundingBoxesWorker);
                this.threadDispatcher = null;
            }
            else
            {
                var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, pool, dt);
                PredictBoundingBoxes(0, bodies.ActiveSet.Count, dt, ref boundingBoxUpdater, 0);
                boundingBoxUpdater.Flush();
            }

        }

        public void IntegrateVelocitiesBoundsAndInertias(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            Callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(bodies.ActiveSet.Count, dt, threadDispatcher.ThreadCount);
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(integrateVelocitiesBoundsAndInertiasWorker);
                this.threadDispatcher = null;
            }
            else
            {
                var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, pool, dt);
                IntegrateVelocitiesBoundsAndInertias(0, bodies.ActiveSet.Count, dt, ref boundingBoxUpdater, 0);
                boundingBoxUpdater.Flush();
            }
        }

        public void IntegrateVelocitiesAndUpdateInertias(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            Callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(bodies.ActiveSet.Count, dt, threadDispatcher.ThreadCount);
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(integrateVelocitiesWorker);
                this.threadDispatcher = null;
            }
            else
            {
                IntegrateVelocities(0, bodies.ActiveSet.Count, dt, 0);
            }
        }

        public void IntegratePoses(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            //This path is used with some other velocity/bounding box integration that handles world inertia calculation, so we don't need to worry about it.
            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            Callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(bodies.ActiveSet.Count, dt, threadDispatcher.ThreadCount);
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(integratePosesWorker);
                this.threadDispatcher = null;
            }
            else
            {
                IntegratePoses(0, bodies.ActiveSet.Count, dt, 0);
            }
        }


        /// <summary>
        /// Integrates the velocities of kinematic bodies as a prepass to the first substep during solving.
        /// Kinematics have to be integrated ahead of time since they don't block constraint batch membership; the same kinematic could appear in the batch multiple times.
        /// </summary>
        internal unsafe void IntegrateKinematicVelocities(Buffer<int> bodyHandles, int bundleStartIndex, int bundleEndIndex, float substepDt, int workerIndex)
        {
            var bodyCount = bodyHandles.Length;
            var bundleCount = BundleIndexing.GetBundleCount(bodyCount);
            var bundleDt = new Vector<float>(substepDt);
            var halfDt = bundleDt * new Vector<float>(0.5f);

            int* bodyIndices = stackalloc int[Vector<int>.Count];
            ref var callbacks = ref Callbacks;
            var handleToLocation = bodies.HandleToLocation;
            BodyInertiaWide zeroInertia = default;

            for (int bundleIndex = bundleStartIndex; bundleIndex < bundleEndIndex; ++bundleIndex)
            {
                var bundleBaseIndex = bundleIndex * Vector<float>.Count;
                var countInBundle = Math.Min(bodyCount - bundleBaseIndex, Vector<float>.Count);
                for (int i = 0; i < countInBundle; ++i)
                {
                    bodyIndices[i] = handleToLocation[bodyHandles[bundleBaseIndex + i]].Index;
                }

                var existingMask = BundleIndexing.CreateMaskForCountInBundle(countInBundle);
                var trailingMask = Vector.OnesComplement(existingMask);
                var bodyIndicesVector = new Vector<int>(new Span<int>(bodyIndices, Vector<int>.Count));
                bodyIndicesVector = Vector.BitwiseOr(trailingMask, bodyIndicesVector);
                //Slightly unfortunate sacrifice to API simplicity: 
                //We're doing a full gather so we can use the vectorized IntegrateVelocity callback even though the amount of work we're doing is absolutely trivial.
                //With luck, the user sets the appropriate flag on the callbacks so this is never called in the first place. (Kinematics are generally not subject to user velocity integration!)
                bodies.GatherState<AccessNoInertia>(bodyIndicesVector, false, out var position, out var orientation, out var velocity, out _);
                callbacks.IntegrateVelocity(bodyIndicesVector, position, orientation, zeroInertia, existingMask, workerIndex, bundleDt, ref velocity);
                //Writes to the empty lanes won't matter (scatter is masked), so we don't need to clean them up.
                //Kinematic bodies have infinite inertia, so using the momentum conserving codepaths would hit a singularity.
                bodies.ScatterVelocities<AccessAll>(ref velocity, ref bodyIndicesVector);

            }
        }

        /// <summary>
        /// Integrates the poses *then* velocities of kinematic bodies as a prepass to the second or later substeps during solving.
        /// Kinematics have to be integrated ahead of time since they don't block constraint batch membership; the same kinematic could appear in the batch multiple times.
        /// </summary>
        internal unsafe void IntegrateKinematicPosesAndVelocities(Buffer<int> bodyHandles, int bundleStartIndex, int bundleEndIndex, float substepDt, int workerIndex)
        {
            var bodyCount = bodyHandles.Length;
            var bundleCount = BundleIndexing.GetBundleCount(bodyCount);
            var bundleDt = new Vector<float>(substepDt);
            var halfDt = bundleDt * new Vector<float>(0.5f);

            int* bodyIndices = stackalloc int[Vector<int>.Count];
            ref var callbacks = ref Callbacks;
            var handleToLocation = bodies.HandleToLocation;
            BodyInertiaWide zeroInertia = default;

            for (int bundleIndex = bundleStartIndex; bundleIndex < bundleEndIndex; ++bundleIndex)
            {
                var bundleBaseIndex = bundleIndex * Vector<float>.Count;
                var countInBundle = Math.Min(bodyCount - bundleBaseIndex, Vector<float>.Count);
                for (int i = 0; i < countInBundle; ++i)
                {
                    bodyIndices[i] = handleToLocation[bodyHandles[bundleBaseIndex + i]].Index;
                }

                var existingMask = BundleIndexing.CreateMaskForCountInBundle(countInBundle);
                var trailingMask = Vector.OnesComplement(existingMask);
                var bodyIndicesVector = new Vector<int>(new Span<int>(bodyIndices, Vector<int>.Count));
                bodyIndicesVector = Vector.BitwiseOr(trailingMask, bodyIndicesVector);
                bodies.GatherState<AccessNoInertia>(bodyIndicesVector, false, out var position, out var orientation, out var velocity, out _);
                //Note that we integrate pose, THEN velocity. This is executing in the context of the second (or beyond) substep, which are effectively completing the previous substep's frame.
                //In other words, the pose integration completes the last substep, and then velocity integration prepares for the current substep.
                //The last substep's pose integration is handled in the IntegrateBundlesAfterSubstepping.
                position += velocity.Linear * bundleDt;
                //Kinematic bodies have infinite inertia, so using the angular momentum conserving codepaths would hit a singularity.
                PoseIntegration.Integrate(orientation, velocity.Angular, halfDt, out orientation);
                bodies.ScatterPose(ref position, ref orientation, bodyIndicesVector, existingMask);
                if (callbacks.IntegrateVelocityForKinematics)
                {
                    callbacks.IntegrateVelocity(bodyIndicesVector, position, orientation, zeroInertia, existingMask, workerIndex, bundleDt, ref velocity);
                    //Writes to the empty lanes won't matter (scatter is masked), so we don't need to clean them up.
                    bodies.ScatterVelocities<AccessAll>(ref velocity, ref bodyIndicesVector);
                }

            }
        }

        unsafe void IntegrateBundlesAfterSubstepping(ref IndexSet mergedConstrainedBodyHandles, int bundleStartIndex, int bundleEndIndex, float dt, float substepDt, int substepCount, int workerIndex)
        {
            var bodyCount = bodies.ActiveSet.Count;
            var bundleCount = BundleIndexing.GetBundleCount(bodyCount);
            var bundleDt = new Vector<float>(dt);
            var bundleSubstepDt = new Vector<float>(substepDt);

            Vector<int> unconstrainedMask;
            Vector<int> bodyIndices;
            int* unconstrainedMaskPointer = (int*)&unconstrainedMask;
            int* bodyIndicesPointer = (int*)&bodyIndices;
            var negativeOne = new Vector<int>(-1);
            ref var callbacks = ref Callbacks;
            ref var indexToHandle = ref bodies.ActiveSet.IndexToHandle;

            for (int i = bundleStartIndex; i < bundleEndIndex; ++i)
            {
                var bundleBaseIndex = i * Vector<float>.Count;
                var countInBundle = Math.Min(bodyCount - bundleBaseIndex, Vector<float>.Count);
                //This is executed at the end of the frame, after all constraints are complete.
                //It covers both constrained and unconstrained bodies.
                //There is no need to write world inertia, since the solver is done.
                //Bodies that are unconstrained should undergo velocity callbacks, velocity integration, and pose integration.
                //Unconstrained bodies can optionally perform a single step for the whole timestep, or do multiple steps to match the integration behavior of constrained bodies.
                //Bodies that are constrained should only undergo one substep of pose integration.
                bool anyBodyInBundleIsUnconstrained = false;
                bodyIndices = negativeOne; //Initialize bundles to -1 so that inactive lanes are consistent with the active set's storage of body references (empty lanes are -1)
                for (int innerIndex = 0; innerIndex < countInBundle; ++innerIndex)
                {
                    var bodyIndex = bundleBaseIndex + innerIndex;
                    bodyIndicesPointer[innerIndex] = bodyIndex;
                    var bodyHandle = indexToHandle[bodyIndex].Value;
                    //Note the use of the solver-merged body handles set. In principle, you could check the body constraints list- if it's empty, then you know all you need to know.
                    //The merged set is preferred here just for the sake of less memory bandwidth.
                    if (mergedConstrainedBodyHandles.Contains(bodyHandle))
                    {
                        unconstrainedMaskPointer[innerIndex] = 0;
                    }
                    else
                    {
                        unconstrainedMaskPointer[innerIndex] = -1;
                        anyBodyInBundleIsUnconstrained = true;
                    }
                }


                Vector<float> bundleEffectiveDt;
                if (callbacks.AllowSubstepsForUnconstrainedBodies)
                {
                    bundleEffectiveDt = bundleSubstepDt;
                }
                else
                {
                    bundleEffectiveDt = Vector.ConditionalSelect(unconstrainedMask, bundleDt, bundleSubstepDt);
                }
                var halfDt = bundleEffectiveDt * new Vector<float>(0.5f);
                bodies.GatherState<AccessAll>(bodyIndices, false, out var position, out var orientation, out var velocity, out var localInertia);


                Vector<int> unconstrainedVelocityIntegrationMask;
                bool anyBodyInBundleNeedsVelocityIntegration;
                if (callbacks.IntegrateVelocityForKinematics)
                {
                    unconstrainedVelocityIntegrationMask = unconstrainedMask;
                    anyBodyInBundleNeedsVelocityIntegration = anyBodyInBundleIsUnconstrained;
                }
                else
                {
                    var isKinematic =
                    Vector.Equals(Vector.BitwiseOr(
                        Vector.BitwiseOr(Vector.BitwiseOr(localInertia.InverseMass, localInertia.InverseInertiaTensor.XX), Vector.BitwiseOr(localInertia.InverseInertiaTensor.YX, localInertia.InverseInertiaTensor.YY)),
                        Vector.BitwiseOr(Vector.BitwiseOr(localInertia.InverseInertiaTensor.ZX, localInertia.InverseInertiaTensor.ZY), localInertia.InverseInertiaTensor.ZZ)), Vector<float>.Zero);
                    unconstrainedVelocityIntegrationMask = Vector.AndNot(unconstrainedMask, isKinematic);
                    anyBodyInBundleNeedsVelocityIntegration = Vector.LessThanAny(unconstrainedVelocityIntegrationMask, Vector<int>.Zero);
                }
                //We don't want to scatter velocities into any slots that don't want velocity writes. By setting all the bits in such lanes, velocity scatter will skip them.
                var velocityMaskedBodyIndices = Vector.BitwiseOr(bodyIndices, Vector.OnesComplement(unconstrainedVelocityIntegrationMask));

                if (anyBodyInBundleIsUnconstrained)
                {
                    int integrationStepCount;
                    if (callbacks.AllowSubstepsForUnconstrainedBodies)
                    {
                        integrationStepCount = substepCount;
                    }
                    else
                    {
                        integrationStepCount = 1;
                    }
                    for (int stepIndex = 0; stepIndex < integrationStepCount; ++stepIndex)
                    {
                        //Note that the following integrates velocities, then poses.
                        var previousVelocity = velocity;

                        if (anyBodyInBundleNeedsVelocityIntegration)
                        {
                            callbacks.IntegrateVelocity(bodyIndices, position, orientation, localInertia, unconstrainedVelocityIntegrationMask, workerIndex, bundleEffectiveDt, ref velocity);
                            //It would be annoying to make the user handle masking velocity writes to inactive lanes, so we handle it internally.
                            Vector3Wide.ConditionalSelect(unconstrainedVelocityIntegrationMask, velocity.Linear, previousVelocity.Linear, out velocity.Linear);
                            Vector3Wide.ConditionalSelect(unconstrainedVelocityIntegrationMask, velocity.Angular, previousVelocity.Angular, out velocity.Angular);
                        }

                        position += velocity.Linear * bundleEffectiveDt;

                        //(Note that the constraints in the embedded substepper integrate pose, then velocity- this is because the first substep only integrates velocity,
                        //so in reality, the full loop for constrained bodies with 3 substeps looks like:
                        //(velocity -> solve) -> (pose -> velocity -> solve) -> (pose -> velocity -> solve) -> pose
                        //For unconstrained bodies, it's a tight loop of just:
                        //(velocity -> pose) -> (velocity -> pose) -> (velocity -> pose)
                        if (callbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentum)
                        {
                            var previousOrientation = orientation;
                            PoseIntegration.Integrate(orientation, velocity.Angular, halfDt, out orientation);
                            PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out var inverseInertiaTensor);
                            PoseIntegration.IntegrateAngularVelocityConserveMomentum(previousOrientation, localInertia.InverseInertiaTensor, inverseInertiaTensor, ref velocity.Angular);
                        }
                        else if (callbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque)
                        {
                            PoseIntegration.Integrate(orientation, velocity.Angular, halfDt, out orientation);
                            PoseIntegration.IntegrateAngularVelocityConserveMomentumWithGyroscopicTorque(orientation, localInertia.InverseInertiaTensor, ref velocity.Angular, bundleEffectiveDt);
                        }
                        else
                        {
                            PoseIntegration.Integrate(orientation, velocity.Angular, halfDt, out orientation);
                        }
                        var integratePoseMask = BundleIndexing.CreateMaskForCountInBundle(countInBundle);
                        if (callbacks.AllowSubstepsForUnconstrainedBodies)
                        {
                            if (stepIndex > 0)
                            {
                                //Only the first substep should integrate poses for the constrained bodies, so mask them out for later substeps.
                                integratePoseMask = Vector.BitwiseAnd(integratePoseMask, unconstrainedMask);
                            }
                        }
                        bodies.ScatterPose(ref position, ref orientation, bodyIndices, integratePoseMask);
                        if (anyBodyInBundleNeedsVelocityIntegration)
                        {
                            bodies.ScatterVelocities<AccessAll>(ref velocity, ref velocityMaskedBodyIndices);
                        }
                    }
                }
                else
                {
                    //All bodies in the bundle are constrained, so we do not need to do any kind of velocity integration.
                    PoseIntegration.Integrate(orientation, velocity.Angular, halfDt, out orientation);
                    position += velocity.Linear * bundleEffectiveDt;
                    var integratePoseMask = BundleIndexing.CreateMaskForCountInBundle(countInBundle);
                    bodies.ScatterPose(ref position, ref orientation, bodyIndices, integratePoseMask);
                }
            }
        }

        Action<int> integrateAfterSubsteppingWorker;
        IndexSet constrainedBodies;
        private void IntegrateAfterSubsteppingWorker(int workerIndex)
        {
            var bundleCount = BundleIndexing.GetBundleCount(bodies.ActiveSet.Count);
            var substepDt = cachedDt / substepCount;
            while (TryGetJob(bundleCount, out var start, out var exclusiveEnd))
            {
                IntegrateBundlesAfterSubstepping(ref constrainedBodies, start, exclusiveEnd, cachedDt, substepDt, substepCount, workerIndex);
            }
        }

        public void IntegrateAfterSubstepping(IndexSet constrainedBodies, float dt, int substepCount, IThreadDispatcher threadDispatcher)
        {
            if (threadDispatcher != null && threadDispatcher.ThreadCount > 1)
            {
                PrepareForMultithreadedExecution(BundleIndexing.GetBundleCount(bodies.ActiveSet.Count), dt, threadDispatcher.ThreadCount, substepCount);
                this.constrainedBodies = constrainedBodies;
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(integrateAfterSubsteppingWorker);
                this.threadDispatcher = null;
                this.constrainedBodies = default;
            }
            else
            {
                IntegrateBundlesAfterSubstepping(ref constrainedBodies, 0, BundleIndexing.GetBundleCount(bodies.ActiveSet.Count), dt, dt / substepCount, substepCount, 0);
            }
        }
    }
}
