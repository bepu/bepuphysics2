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
        void PredictBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null);
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
        /// If true, unconstrained bodies use a single step of length equal to the dt provided to <see cref="Simulation.Timestep"/>. 
        /// If false, unconstrained bodies will be integrated with the same number of substeps as the constrained bodies in the solver.
        /// </summary>
        bool AllowSubstepsForUnconstrainedBodies { get; }

        /// <summary>
        /// Gets whether the velocity integration callback should be called for kinematic bodies.
        /// If true, <see cref="IntegrateVelocity"/> will be called for bundles including kinematic bodies.
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
        /// Callback invoked ahead of dispatches that may call into <see cref="IntegrateVelocity"/>.
        /// It may be called more than once with different values over a frame. For example, when performing bounding box prediction, velocity is integrated with a full frame time step duration.
        /// During substepped solves, integration is split into substepCount steps, each with fullFrameDuration / substepCount duration.
        /// The final integration pass for unconstrained bodies may be either fullFrameDuration or fullFrameDuration / substepCount, depending on the value of AllowSubstepsForUnconstrainedBodies. 
        /// </summary>
        /// <param name="dt">Current integration time step duration.</param>
        /// <remarks>This is typically used for precomputing anything expensive that will be used across velocity integration.</remarks>
        void PrepareForIntegration(float dt);

        /// <summary>
        /// Callback for a bundle of bodies being integrated.
        /// </summary>
        /// <param name="bodyIndices">Indices of the bodies being integrated in this bundle.</param>
        /// <param name="position">Current body positions.</param>
        /// <param name="orientation">Current body orientations.</param>
        /// <param name="localInertia">Body's current local inertia.</param>
        /// <param name="integrationMask">Mask indicating which lanes are active in the bundle. Active lanes will contain 0xFFFFFFFF, inactive lanes will contain 0.</param>
        /// <param name="workerIndex">Index of the worker thread processing this bundle.</param>
        /// <param name="dt">Durations to integrate the velocity over. Can vary over lanes.</param>
        /// <param name="velocity">Velocity of bodies in the bundle. Any changes to lanes which are not active by the integrationMask will be discarded.</param>
        void IntegrateVelocity(
            Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia,
            Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity);
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
            var s = MathHelper.Sin(halfAngle);
            var scale = s / speed;
            q.X = angularVelocity.X * scale;
            q.Y = angularVelocity.Y * scale;
            q.Z = angularVelocity.Z * scale;
            q.W = MathHelper.Cos(halfAngle);
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
    /// Handles body integration work that isn't bundled into the solver's execution. Predicts bounding boxes, integrates velocity and poses for unconstrained bodies, and does final post-substepping pose integration for constrained bodies.
    /// </summary>
    public class PoseIntegrator<TCallbacks> : IPoseIntegrator where TCallbacks : IPoseIntegratorCallbacks
    {
        Bodies bodies;
        Shapes shapes;
        BroadPhase broadPhase;

        public TCallbacks Callbacks;

        Action<int> predictBoundingBoxesWorker;
        public PoseIntegrator(Bodies bodies, Shapes shapes, BroadPhase broadPhase, TCallbacks callbacks)
        {
            this.bodies = bodies;
            this.shapes = shapes;
            this.broadPhase = broadPhase;
            Callbacks = callbacks;
            predictBoundingBoxesWorker = PredictBoundingBoxesWorker;
            integrateAfterSubsteppingWorker = IntegrateAfterSubsteppingWorker;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateSleepCandidacy(float velocityHeuristic, ref BodyActivity activity)
        {
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

        unsafe void PredictBoundingBoxes(int startBundleIndex, int endBundleIndex, float dt, ref BoundingBoxBatcher boundingBoxBatcher, int workerIndex)
        {
            var activities = bodies.ActiveSet.Activity;
            var collidables = bodies.ActiveSet.Collidables;

            Helpers.FillVectorWithLaneIndices(out var laneIndexOffsets);
            var dtWide = new Vector<float>(dt);
            var bodyCount = bodies.ActiveSet.Count;
            for (int bundleIndex = startBundleIndex; bundleIndex < endBundleIndex; ++bundleIndex)
            {
                var bundleStartBodyIndex = bundleIndex * Vector<float>.Count;
                var countInBundle = bodyCount - bundleStartBodyIndex;
                if (countInBundle > Vector<int>.Count)
                    countInBundle = Vector<int>.Count;
                //Note that this is bundle-fied primarily to avoid requiring velocity integration callbacks to implement a scalar and vector version.
                //Performance wise, I don't expect a meaningful improvement over a scalar version; there's too little work being done.
                var laneIndices = new Vector<int>(bundleStartBodyIndex) + laneIndexOffsets;
                bodies.GatherState<AccessAll>(laneIndices, false, out var position, out var orientation, out var velocity, out var inertia);

                Vector<int> integrationMask;
                if (Callbacks.IntegrateVelocityForKinematics)
                {
                    integrationMask = BundleIndexing.CreateMaskForCountInBundle(countInBundle);
                }
                else
                {
                    integrationMask = Vector.AndNot(BundleIndexing.CreateMaskForCountInBundle(countInBundle), Bodies.IsKinematic(inertia));
                }
                var sleepEnergy = velocity.Linear.LengthSquared() + velocity.Angular.LengthSquared();

                //Note that we're not storing out the integrated velocities. The integrated velocities are only used for bounding box prediction.
                if (Vector.LessThanAny(integrationMask, Vector<int>.Zero))
                    Callbacks.IntegrateVelocity(laneIndices, position, orientation, inertia, integrationMask, workerIndex, dtWide, ref velocity);

                for (int i = 0; i < countInBundle; ++i)
                {
                    var bodyIndex = i + bundleStartBodyIndex;
                    UpdateSleepCandidacy(sleepEnergy[i], ref activities[bodyIndex]);

                    //TODO: A vectorized transposition, like what the GatherState function uses, would speed this up a good bit.
                    //PredictBoundingBoxes isn't a huge cost overall so I didn't do it immediately, but it's an available optimization.
                    RigidPose bodyPose;
                    Vector3Wide.ReadSlot(ref position, i, out bodyPose.Position);
                    QuaternionWide.ReadSlot(ref orientation, i, out bodyPose.Orientation);
                    BodyVelocity bodyVelocity;
                    Vector3Wide.ReadSlot(ref velocity.Linear, i, out bodyVelocity.Linear);
                    Vector3Wide.ReadSlot(ref velocity.Angular, i, out bodyVelocity.Angular);

                    //Bounding boxes are accumulated in a scalar fashion, but the actual bounding box calculations are deferred until a sufficient number of collidables are accumulated to make
                    //executing a bundle worthwhile. This does two things: 
                    //1) SIMD can be used to do the mathy bits of bounding box calculation. The calculations are usually pretty cheap, 
                    //but they will often be more expensive than the pose stuff above.
                    //2) The number of virtual function invocations required is reduced by a factor equal to the size of the accumulator cache.
                    //Note that the accumulator caches are kept relatively small so that it is very likely that the pose and velocity of the collidable's body will still be in L1 cache
                    //when it comes time to actually compute bounding boxes.

                    //Note that any collidable that lacks a collidable, or any reference that is beyond the set of collidables, will have a specially formed index.
                    //The accumulator will detect that and not try to add a nonexistent collidable.
                    boundingBoxBatcher.Add(bodyIndex, bodyPose, bodyVelocity, collidables[bodyIndex]);
                }
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
            return true;
        }

        void PredictBoundingBoxesWorker(int workerIndex)
        {
            var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, threadDispatcher.GetThreadMemoryPool(workerIndex), cachedDt);
            var bundleCount = BundleIndexing.GetBundleCount(bodies.ActiveSet.Count);
            while (TryGetJob(bundleCount, out var start, out var exclusiveEnd))
            {
                PredictBoundingBoxes(start, exclusiveEnd, cachedDt, ref boundingBoxUpdater, workerIndex);
            }
            boundingBoxUpdater.Flush();
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

        public void PredictBoundingBoxes(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            Callbacks.PrepareForIntegration(dt);
            if (threadDispatcher != null)
            {
                PrepareForMultithreadedExecution(BundleIndexing.GetBundleCount(bodies.ActiveSet.Count), dt, threadDispatcher.ThreadCount);
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(predictBoundingBoxesWorker, availableJobCount);
                //predictBoundingBoxesWorker(0);
                this.threadDispatcher = null;
            }
            else
            {
                var boundingBoxUpdater = new BoundingBoxBatcher(bodies, shapes, broadPhase, pool, dt);
                PredictBoundingBoxes(0, BundleIndexing.GetBundleCount(bodies.ActiveSet.Count), dt, ref boundingBoxUpdater, 0);
                boundingBoxUpdater.Flush();
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
            var bodyIndicesSpan = new Span<int>(bodyIndices, Vector<int>.Count);
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
                var bodyIndicesVector = Vector.BitwiseOr(trailingMask, new Vector<int>(bodyIndicesSpan));

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
            var bodyIndicesSpan = new Span<int>(bodyIndices, Vector<int>.Count);
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
                var bodyIndicesVector = new Vector<int>(bodyIndicesSpan);
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

            int* unconstrainedMaskPointer = stackalloc int[Vector<int>.Count];
            int* bodyIndicesPointer = stackalloc int[Vector<int>.Count];
            var unconstrainedMaskSpan = new Span<int>(unconstrainedMaskPointer, Vector<int>.Count);
            var bodyIndicesSpan = new Span<int>(bodyIndicesPointer, Vector<int>.Count);
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
                var unconstrainedMask = new Vector<int>(unconstrainedMaskSpan);
                var bodyIndices = new Vector<int>(bodyIndicesSpan);
                if (countInBundle < Vector<int>.Count)
                {
                    //Set empty body index lanes to -1 so that inactive lanes are consistent with the active set's storage of body references (empty lanes are -1)
                    var trailingMask = BundleIndexing.CreateTrailingMaskForCountInBundle(countInBundle);
                    bodyIndices = Vector.BitwiseOr(bodyIndices, trailingMask);
                    //Empty slots should not be considered here; clear the mask slot.
                    unconstrainedMask = Vector.AndNot(unconstrainedMask, trailingMask);
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
                    var isKinematic = Bodies.IsKinematic(localInertia);
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
            //The only bodies undergoing *velocity* integration during the post-integration step are unconstrained.
            var substepDt = dt / substepCount;
            var velocityIntegrationTimestep = Callbacks.AllowSubstepsForUnconstrainedBodies ? substepDt : dt;
            Callbacks.PrepareForIntegration(velocityIntegrationTimestep);
            if (threadDispatcher != null && threadDispatcher.ThreadCount > 1)
            {
                PrepareForMultithreadedExecution(BundleIndexing.GetBundleCount(bodies.ActiveSet.Count), dt, threadDispatcher.ThreadCount, substepCount);
                this.constrainedBodies = constrainedBodies;
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(integrateAfterSubsteppingWorker, availableJobCount);
                this.threadDispatcher = null;
                this.constrainedBodies = default;
            }
            else
            {
                IntegrateBundlesAfterSubstepping(ref constrainedBodies, 0, BundleIndexing.GetBundleCount(bodies.ActiveSet.Count), dt, substepDt, substepCount, 0);
            }
        }
    }
}
