using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics
{
    public interface IPoseIntegrator
    {
        void IntegrateBodiesAndUpdateBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void PredictBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void IntegrateVelocitiesBoundsAndInertias(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void IntegrateVelocitiesAndUpdateInertias(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
        void IntegratePoses(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
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
            var displacement = linearVelocity * dt;
            integratedPosition = position + displacement;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Integrate(in Quaternion orientation, in Vector3 angularVelocity, float dt, out Quaternion integratedOrientation)
        {
            //Note that we don't bother with conservation of angular momentum or the gyroscopic term or anything else. All orientation integration assumes a series of piecewise linear integrations
            //That's not entirely correct, but it's a reasonable approximation that means we don't have to worry about conservation of angular momentum or gyroscopic terms when dealing with CCD sweeps.

            var speed = angularVelocity.Length();
            if (speed > 1e-15f)
            {
                var halfAngle = speed * dt * 0.5f;
                Quaternion q;
                Unsafe.As<Quaternion, Vector3>(ref *&q) = angularVelocity * (MathHelper.Sin(halfAngle) / speed);
                q.W = MathHelper.Cos(halfAngle);
                //Note that the input and output may overlap.
                Quaternion.Concatenate(orientation, q, out integratedOrientation);
                Quaternion.Normalize(ref integratedOrientation);
            }
            else
            {
                integratedOrientation = orientation;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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
            QuaternionWide.ConcatenateWithoutOverlap(start, q, out var concatenated);
            QuaternionWide.Normalize(concatenated, out integrated);
            var speedValid = Vector.GreaterThan(speed, new Vector<float>(1e-15f));
            integrated.X = Vector.ConditionalSelect(speedValid, integrated.X, start.X);
            integrated.Y = Vector.ConditionalSelect(speedValid, integrated.Y, start.Y);
            integrated.Z = Vector.ConditionalSelect(speedValid, integrated.Z, start.Z);
            integrated.W = Vector.ConditionalSelect(speedValid, integrated.W, start.W);
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

        TCallbacks callbacks;

        Action<int> integrateBodiesAndUpdateBoundingBoxesWorker;
        Action<int> predictBoundingBoxesWorker;
        Action<int> integrateVelocitiesBoundsAndInertiasWorker;
        Action<int> integrateVelocitiesWorker;
        Action<int> integratePosesWorker;
        public PoseIntegrator(Bodies bodies, Shapes shapes, BroadPhase broadPhase, TCallbacks callback)
        {
            this.bodies = bodies;
            this.shapes = shapes;
            this.broadPhase = broadPhase;
            this.callbacks = callback;
            integrateBodiesAndUpdateBoundingBoxesWorker = IntegrateBodiesAndUpdateBoundingBoxesWorker;
            predictBoundingBoxesWorker = PredictBoundingBoxesWorker;
            integrateVelocitiesBoundsAndInertiasWorker = IntegrateVelocitiesBoundsAndInertiasWorker;
            integrateVelocitiesWorker = IntegrateVelocitiesWorker;
            integratePosesWorker = IntegratePosesWorker;
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
                ++activity.TimestepsUnderThresholdCount;
                if (activity.TimestepsUnderThresholdCount >= activity.MinimumTimestepsUnderThreshold)
                {
                    activity.SleepCandidate = true;
                }
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void IntegrateAngularVelocityConserving(in Quaternion previousOrientation, in RigidPose pose, in BodyInertia localInertia, in BodyInertia inertia, ref Vector3 angularVelocity, float dt)
        {

            if (callbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentum)
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
            if (callbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque)
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
            if ((int)callbacks.AngularIntegrationMode >= (int)AngularIntegrationMode.ConserveMomentum)
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
            if ((int)callbacks.AngularIntegrationMode >= (int)AngularIntegrationMode.ConserveMomentum)
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
            ref var basePoses = ref bodies.ActiveSet.Poses[0];
            ref var baseVelocities = ref bodies.ActiveSet.Velocities[0];
            ref var baseLocalInertia = ref bodies.ActiveSet.LocalInertias[0];
            ref var baseInertias = ref bodies.Inertias[0];
            ref var baseActivity = ref bodies.ActiveSet.Activity[0];
            ref var baseCollidable = ref bodies.ActiveSet.Collidables[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var pose = ref Unsafe.Add(ref basePoses, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);

                var previousOrientation = pose.Orientation; //This is unused if conservation of angular momentum is disabled... compiler *may* remove it...
                PoseIntegration.Integrate(pose, velocity, dt, out pose);

                //Note that this generally is used before velocity integration. That means an object can go inactive with gravity-induced velocity.
                //That is actually intended: when the narrowphase wakes up an island, the accumulated impulses in the island will be ready for gravity's influence.
                //To do otherwise would hurt the solver's guess, reducing the quality of the solve and possibly causing a little bump.
                //This is only relevant when the update order actually puts the sleeper after gravity. For ease of use, this fact may be ignored by the simulation update order.
                UpdateSleepCandidacy(ref velocity, ref Unsafe.Add(ref baseActivity, i));

                //Update the inertia tensors for the new orientation.
                //TODO: If the pose integrator is positioned at the end of an update, the first frame after any out-of-timestep orientation change or local inertia change
                //has to get is inertia tensors calculated elsewhere. Either they would need to be computed on addition or something- which is a bit gross, but doable-
                //or we would need to move this calculation to the beginning of the frame to guarantee that all inertias are up to date. 
                //This would require a scan through all pose memory to support, but if you do it at the same time as AABB update, that's fine- that stage uses the pose too.
                ref var localInertia = ref Unsafe.Add(ref baseLocalInertia, i);
                ref var inertia = ref Unsafe.Add(ref baseInertias, i);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, pose.Orientation, out inertia.InverseInertiaTensor);
                //While it's a bit goofy just to copy over the inverse mass every frame even if it doesn't change,
                //it's virtually always gathered together with the inertia tensor and it really isn't worth a whole extra external system to copy inverse masses only on demand.
                inertia.InverseMass = localInertia.InverseMass;

                IntegrateAngularVelocity(previousOrientation, pose, localInertia, inertia, ref velocity.Angular, dt);
                callbacks.IntegrateVelocity(i, pose, localInertia, workerIndex, ref velocity);

                //Bounding boxes are accumulated in a scalar fashion, but the actual bounding box calculations are deferred until a sufficient number of collidables are accumulated to make
                //executing a bundle worthwhile. This does two things: 
                //1) SIMD can be used to do the mathy bits of bounding box calculation. The calculations are usually pretty cheap, 
                //but they will often be more expensive than the pose stuff above.
                //2) The number of virtual function invocations required is reduced by a factor equal to the size of the accumulator cache.
                //Note that the accumulator caches are kept relatively small so that it is very likely that the pose and velocity of the collidable's body will still be in L1 cache
                //when it comes time to actually compute bounding boxes.

                //Note that any collidable that lacks a collidable, or any reference that is beyond the set of collidables, will have a specially formed index.
                //The accumulator will detect that and not try to add a nonexistent collidable.
                boundingBoxBatcher.Add(i, pose, velocity, Unsafe.Add(ref baseCollidable, i));

                //It's helpful to do the bounding box update here in the pose integrator because they share information. If the phases were split, there could be a penalty
                //associated with loading all the body poses and velocities from memory again. Even if the L3 cache persisted, it would still be worse than looking into L1 or L2.
                //Also, the pose integrator in isolation is extremely memory bound to the point where it can hardly benefit from multithreading. By interleaving some less memory bound
                //work into the mix, we can hopefully fill some execution gaps.
            }
        }

        unsafe void PredictBoundingBoxes(int startIndex, int endIndex, float dt, ref BoundingBoxBatcher boundingBoxBatcher, int workerIndex)
        {
            ref var basePoses = ref bodies.ActiveSet.Poses[0];
            ref var baseVelocities = ref bodies.ActiveSet.Velocities[0];
            ref var baseLocalInertia = ref bodies.ActiveSet.LocalInertias[0];
            ref var baseActivity = ref bodies.ActiveSet.Activity[0];
            ref var baseCollidable = ref bodies.ActiveSet.Collidables[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var pose = ref Unsafe.Add(ref basePoses, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);

                UpdateSleepCandidacy(ref velocity, ref Unsafe.Add(ref baseActivity, i));

                //Bounding box prediction does not need to update inertia tensors.                
                var integratedVelocity = velocity;
                callbacks.IntegrateVelocity(i, pose, Unsafe.Add(ref baseLocalInertia, i), workerIndex, ref integratedVelocity);

                //Note that we do not include fancier angular integration for the bounding box prediction- it's not very important.
                boundingBoxBatcher.Add(i, pose, integratedVelocity, Unsafe.Add(ref baseCollidable, i));
            }
        }

        unsafe void IntegrateVelocitiesBoundsAndInertias(int startIndex, int endIndex, float dt, ref BoundingBoxBatcher boundingBoxBatcher, int workerIndex)
        {
            ref var basePoses = ref bodies.ActiveSet.Poses[0];
            ref var baseVelocities = ref bodies.ActiveSet.Velocities[0];
            ref var baseLocalInertia = ref bodies.ActiveSet.LocalInertias[0];
            ref var baseInertias = ref bodies.Inertias[0];
            ref var baseActivity = ref bodies.ActiveSet.Activity[0];
            ref var baseCollidable = ref bodies.ActiveSet.Collidables[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var pose = ref Unsafe.Add(ref basePoses, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);

                UpdateSleepCandidacy(ref velocity, ref Unsafe.Add(ref baseActivity, i));

                ref var localInertia = ref Unsafe.Add(ref baseLocalInertia, i);
                ref var inertia = ref Unsafe.Add(ref baseInertias, i);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, pose.Orientation, out inertia.InverseInertiaTensor);
                inertia.InverseMass = localInertia.InverseMass;

                IntegrateAngularVelocity(pose, localInertia, inertia, ref velocity.Angular, dt);
                callbacks.IntegrateVelocity(i, pose, localInertia, workerIndex, ref velocity);

                boundingBoxBatcher.Add(i, pose, velocity, Unsafe.Add(ref baseCollidable, i));
            }
        }


        unsafe void IntegrateVelocities(int startIndex, int endIndex, float dt, int workerIndex)
        {
            ref var basePoses = ref bodies.ActiveSet.Poses[0];
            ref var baseVelocities = ref bodies.ActiveSet.Velocities[0];
            ref var baseLocalInertia = ref bodies.ActiveSet.LocalInertias[0];
            ref var baseInertias = ref bodies.Inertias[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var pose = ref Unsafe.Add(ref basePoses, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);

                ref var localInertia = ref Unsafe.Add(ref baseLocalInertia, i);
                ref var inertia = ref Unsafe.Add(ref baseInertias, i);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, pose.Orientation, out inertia.InverseInertiaTensor);
                inertia.InverseMass = localInertia.InverseMass;

                IntegrateAngularVelocity(pose, localInertia, inertia, ref velocity.Angular, dt);
                callbacks.IntegrateVelocity(i, pose, localInertia, workerIndex, ref velocity);
            }
        }

        unsafe void IntegratePoses(int startIndex, int endIndex, float dt, int workerIndex)
        {
            ref var basePoses = ref bodies.ActiveSet.Poses[0];
            ref var baseVelocities = ref bodies.ActiveSet.Velocities[0];
            for (int i = startIndex; i < endIndex; ++i)
            {
                ref var pose = ref Unsafe.Add(ref basePoses, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);

                PoseIntegration.Integrate(pose, velocity, dt, out pose);
            }
        }


        float cachedDt;
        int bodiesPerJob;
        IThreadDispatcher threadDispatcher;

        //Note that we aren't using a very cache-friendly work distribution here.
        //This is working on the assumption that the jobs will be large enough that border region cache misses won't be a big concern.
        //If this turns out to be false, this could be swapped over to a system similar to the solver-
        //preschedule offset regions for each worker to allow each one to consume a contiguous region before workstealing.
        int availableJobCount;
        bool TryGetJob(int bodyCount, out int start, out int exclusiveEnd)
        {
            var jobIndex = Interlocked.Decrement(ref availableJobCount);
            if (jobIndex < 0)
            {
                start = 0;
                exclusiveEnd = 0;
                return false;
            }
            start = jobIndex * bodiesPerJob;
            exclusiveEnd = start + bodiesPerJob;
            if (exclusiveEnd > bodyCount)
                exclusiveEnd = bodyCount;
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

        void PrepareForMultithreadedExecution(float dt, int workerCount)
        {
            cachedDt = dt;
            const int jobsPerWorker = 4;
            var targetJobCount = workerCount * jobsPerWorker;
            bodiesPerJob = bodies.ActiveSet.Count / targetJobCount;
            if (bodiesPerJob == 0)
                bodiesPerJob = 1;
            availableJobCount = bodies.ActiveSet.Count / bodiesPerJob;
            if (bodiesPerJob * availableJobCount < bodies.ActiveSet.Count)
                ++availableJobCount;
        }


        public void IntegrateBodiesAndUpdateBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            //Users of this codepath are expecting all integration related work to be done at once, so we need to update inertias.
            bodies.EnsureInertiasCapacity(Math.Max(1, bodies.ActiveSet.Count));

            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                //While we do technically support multithreading here, scaling is going to be really, really bad if the simulation gets kicked out of L3 cache in between frames.
                //The ratio of memory loads to actual compute work in this stage is extremely high, so getting scaling of 1.2x on a quad core is quite possible.
                //On the upside, it is a very short stage. With any luck, one or more of the following will hold:
                //1) the system has silly fast RAM,
                //2) the CPU supports octochannel memory and just brute forces the issue,
                //3) whatever the application is doing doesn't evict the entire L3 cache between frames.

                //Note that this bottleneck means the fact that we're working through bodies in a nonvectorized fashion (in favor of optimizing storage for solver access) is not a problem.

                PrepareForMultithreadedExecution(dt, threadDispatcher.ThreadCount);
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

            callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(dt, threadDispatcher.ThreadCount);
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
            bodies.EnsureInertiasCapacity(Math.Max(1, bodies.ActiveSet.Count));

            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(dt, threadDispatcher.ThreadCount);
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
            //Isolated velocity integration is used by substeppers that also expect an inertia update.
            bodies.EnsureInertiasCapacity(Math.Max(1, bodies.ActiveSet.Count));

            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(dt, threadDispatcher.ThreadCount);
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

            callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(dt, threadDispatcher.ThreadCount);
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(integratePosesWorker);
                this.threadDispatcher = null;
            }
            else
            {
                IntegratePoses(0, bodies.ActiveSet.Count, dt, 0);
            }
        }
    }
}
