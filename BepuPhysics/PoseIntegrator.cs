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

    /// <summary>
    /// Integrates the velocity of mobile bodies over time into changes in position and orientation. Also applies gravitational acceleration to dynamic bodies.
    /// </summary>
    /// <remarks>
    /// This variant of the integrator uses a single global gravity. Other integrators that provide per-entity gravity could exist later.
    /// This integrator also assumes that the bodies positions are stored in terms of single precision floats. Later on, we will likely modify the Bodies
    /// storage to allow different representations for larger simulations. That will require changes in this integrator, the relative position calculation of collision detection,
    /// the bounding box calculation, and potentially even in the broadphase in extreme cases (64 bit per component positions).
    /// </remarks>
    public class PoseIntegrator
    {
        Bodies bodies;
        Shapes shapes;
        BroadPhase broadPhase;

        /// <summary>
        /// Acceleration of gravity to apply to all dynamic bodies in the simulation.
        /// </summary>
        public Vector3 Gravity;

        Action<int> workerDelegate;
        public PoseIntegrator(Bodies bodies, Shapes shapes, BroadPhase broadPhase)
        {
            this.bodies = bodies;
            this.shapes = shapes;
            this.broadPhase = broadPhase;
            workerDelegate = Worker;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotateInertia(ref Triangular3x3 localInertiaTensor, ref Quaternion orientation, out Triangular3x3 rotatedInertiaTensor)
        {
            Matrix3x3.CreateFromQuaternion(ref orientation, out var orientationMatrix);
            //I^-1 = RT * Ilocal^-1 * R 
            //NOTE: If you were willing to confuse users a little bit, the local inertia could be required to be diagonal.
            //This would be totally fine for all the primitive types which happen to have diagonal inertias, but for more complex shapes (convex hulls, meshes), 
            //there would need to be a reorientation step. That could be confusing, and it's probably not worth it.
            Triangular3x3.RotationSandwich(ref orientationMatrix, ref localInertiaTensor, out rotatedInertiaTensor);
        }

        unsafe void IntegrateBodies(int startIndex, int endIndex, float dt, ref BoundingBoxUpdater boundingBoxUpdater)
        {
            ref var basePoses = ref bodies.ActiveSet.Poses[0];
            ref var baseVelocities = ref bodies.ActiveSet.Velocities[0];
            ref var baseLocalInertias = ref bodies.ActiveSet.LocalInertias[0];
            ref var baseInertias = ref bodies.Inertias[0];
            ref var baseActivity = ref bodies.ActiveSet.Activity[0];
            var halfDt = dt * 0.5f;
            for (int i = startIndex; i < endIndex; ++i)
            {
                //Integrate position with the latest linear velocity. Note that gravity is integrated afterwards.
                ref var pose = ref Unsafe.Add(ref basePoses, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);
                var displacement = velocity.Linear * dt;
                pose.Position += displacement;

                //Update deactivation candidacy. Note that this comes before velocity integration. That means an object can go inactive with gravity-induced velocity.
                //That is actually intended: when the narrowphase wakes up an island, the accumulated impulses in the island will be ready for gravity's influence.
                //To do otherwise would hurt the solver's guess, reducing the quality of the solve and possibly causing a little bump.
                //This is only relevant when the update order actually puts the deactivator after gravity. For ease of use, this fact may be ignored by the simulation update order.
                ref var activity = ref Unsafe.Add(ref baseActivity, i);
                var velocityHeuristic = velocity.Linear.LengthSquared() + velocity.Angular.LengthSquared();
                if (velocityHeuristic > activity.DeactivationThreshold)
                {
                    activity.TimestepsUnderThresholdCount = 0;
                    activity.DeactivationCandidate = false;
                }
                else
                {
                    ++activity.TimestepsUnderThresholdCount;
                    if (activity.TimestepsUnderThresholdCount >= activity.MinimumTimestepsUnderThreshold)
                    {
                        activity.DeactivationCandidate = true;
                    }
                }

                //Integrate orientation with the latest angular velocity.
                //Note that we don't bother with conservation of angular momentum or the gyroscopic term or anything else- 
                //it's not exactly correct, but it's stable, fast, and no one really notices. Unless they're trying to spin a multitool in space or something.
                //(But frankly, that just looks like reality has a bug.)

                //TODO: This is a pretty strong combo-hack. Check codegen.
                Quaternion multiplier;
                Unsafe.As<float, Vector3>(ref *&multiplier.X) = velocity.Angular * halfDt;
                multiplier.W = 0;
                Quaternion.ConcatenateWithoutOverlap(ref pose.Orientation, ref *&multiplier, out var increment);
                Quaternion.Add(ref pose.Orientation, ref increment, out pose.Orientation);
                Quaternion.Normalize(ref pose.Orientation);

                //Update the inertia tensors for the new orientation.
                //TODO: If the pose integrator is positioned at the end of an update, the first frame after any out-of-timestep orientation change or local inertia change
                //has to get is inertia tensors calculated elsewhere. Either they would need to be computed on addition or something- which is a bit gross, but doable-
                //or we would need to move this calculation to the beginning of the frame to guarantee that all inertias are up to date. 
                //This would require a scan through all pose memory to support, but if you do it at the same time as AABB update, that's fine- that stage uses the pose too.
                ref var localInertias = ref Unsafe.Add(ref baseLocalInertias, i);
                ref var inertias = ref Unsafe.Add(ref baseInertias, i);
                RotateInertia(ref localInertias.InverseInertiaTensor, ref pose.Orientation, out inertias.InverseInertiaTensor);
                //While it's a bit goofy just to copy over the inverse mass every frame even if it doesn't change,
                //it's virtually always gathered together with the inertia tensor and it really isn't worth a whole extra external system to copy inverse masses only on demand.
                inertias.InverseMass = localInertias.InverseMass;

                //Note that we apply gravity during this phase. That means, if the integrator is put at the end of the frame, the velocity will be nonzero.
                //For now, we're assuming that the integrator runs at the beginning of the frame. This is a tradeoff, with some pros:
                //1) Contacts generated in a given frame will match the position. You could draw effects at the contact points without worrying about them being one frame offset.
                //However, if we detect contacts from a predicted transform, this mostly goes out the window.
                //2) There is no need for a second 'force application' stage before the AABB update. Realistically, the force application would be bundled into the AABB update.
                //3) Advanced users are free to trivially intervene in the velocity modifications caused by constraints before they are integrated. (A callback would be just as good.)
                //4) PoseIntegrator->AABBUpdate allows the pose/velocity information required by pose integration to remain in LLC for use by AABB update.
                //Having collision detection and the solver in between would likely evict pose and velocity on larger simulations. Given that we assume that the cache is evicted
                //in between frames, the AABB->coldet->solve->poseintegrate could end up paying the cost of pose/velocity loads twice. It's not THAT bad, but it's not free- 
                //about 0.15-0.25ms on a simulation with 32768 bodies.
                //And one big con:
                //If you modify velocity outside of the update, it will be directly used to integrate position during the next frame, ignoring all constraints.
                //This WILL be a problem, especially since BEPUphysics v1 trained people to use velocity modifications to control motion.

                //This isn't an unsolvable problem- making it easy to handle velocity modifications mid-update by exposing a callback of some sort would work. But that's one step more
                //than the v1 'just set the velocity' style.                

                //Note that we avoid accelerating kinematics. Kinematics are any body with an inverse mass of zero (so a mass of ~infinity). No force can move them.
                if (localInertias.InverseMass > 0)
                    velocity.Linear += gravityDt;
                //Implementation sidenote: Why aren't kinematics all bundled together separately from dynamics to avoid this condition?
                //Because kinematics can have a velocity- that is what distinguishes them from a static object. The solver must read velocities of all bodies involved in a constraint.
                //Under ideal conditions, those bodies will be near in memory to increase the chances of a cache hit. If kinematics are separately bundled, the the number of cache
                //misses necessarily increases. Slowing down the solver in order to speed up the pose integrator is a really, really bad trade, especially when the benefit is a few ALU ops.


                //Bounding boxes are accumulated in a scalar fashion, but the actual bounding box calculations are deferred until a sufficient number of collidables are accumulated to make
                //executing a bundle worthwhile. This does two things: 
                //1) SIMD can be used to do the mathy bits of bounding box calculation. The calculations are usually pretty cheap, 
                //but they will often be more expensive than the pose stuff above.
                //2) The number of virtual function invocations required is reduced by a factor equal to the size of the accumulator cache.
                //Note that the accumulator caches are kept relatively small so that it is very likely that the pose and velocity of the collidable's body will still be in L1 cache
                //when it comes time to actually compute bounding boxes.

                //Note that any collidable that lacks a collidable, or any reference that is beyond the set of collidables, will have a specially formed index.
                //The accumulator will detect that and not try to add a nonexistent collidable- hence, "TryAdd".
                boundingBoxUpdater.TryAdd(i);

                //It's helpful to do the bounding box update here in the pose integrator because they share information. If the phases were split, there could be a penalty
                //associated with loading all the body poses and velocities from memory again. Even if the L3 cache persisted, it would still be worse than looking into L1 or L2.
                //Also, the pose integrator in isolation is extremely memory bound to the point where it can hardly benefit from multithreading. By interleaving some less memory bound
                //work into the mix, we can hopefully fill some execution gaps.
            }
        }

        float cachedDt;
        Vector3 gravityDt;
        int bodiesPerJob;
        IThreadDispatcher threadDispatcher;

        //Note that we aren't using a very cache-friendly work distribution here.
        //This is working on the assumption that the jobs will be large enough that border region cache misses won't be a big concern.
        //If this turns out to be false, this could be swapped over to a system similar to the solver-
        //preschedule offset regions for each worker to allow each one to consume a contiguous region before workstealing.
        int availableJobCount;
        void Worker(int workerIndex)
        {
            var boundingBoxUpdater = new BoundingBoxUpdater(bodies, shapes, broadPhase, threadDispatcher.GetThreadMemoryPool(workerIndex), cachedDt);
            var bodyCount = bodies.ActiveSet.Count;
            while (true)
            {
                var jobIndex = Interlocked.Decrement(ref availableJobCount);
                if (jobIndex < 0)
                    break;
                var start = jobIndex * bodiesPerJob;
                var exclusiveEnd = start + bodiesPerJob;
                if (exclusiveEnd > bodyCount)
                    exclusiveEnd = bodyCount;
                Debug.Assert(exclusiveEnd > start, "Jobs that would involve bundles beyond the body count should not be created.");

                IntegrateBodies(start, exclusiveEnd, cachedDt, ref boundingBoxUpdater);

            }
            boundingBoxUpdater.FlushAndDispose();

        }
        public void Update(float dt, BufferPool pool, IThreadDispatcher threadDispatcher = null)
        {
            //For now, the pose integrator (as the first stage that references them in any way) is responsible for ensuring that the bodies have a reasonable size inertias buffer.
            //Note that ownership (for purposes of final disposal) still belongs to the Bodies set.
            bodies.EnsureInertiasCapacity(bodies.ActiveSet.Count);

            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            gravityDt = Gravity * dt;
            if (threadDispatcher != null)
            {
                //While we do technically support multithreading here, scaling is going to be really, really bad if the simulation gets kicked out of L3 cache in between frames.
                //The ratio of memory loads to actual compute work in this stage is extremely high, so getting scaling of 1.2x on a quad core is quite possible.
                //On the upside, it is a very short stage. With any luck, one or more of the following will hold:
                //1) the system has silly fast RAM,
                //2) the CPU supports octochannel memory and just brute forces the issue,
                //3) whatever the application is doing doesn't evict the entire L3 cache between frames.

                //Note that this bottleneck means the fact that we're working through bodies in a nonvectorized fashion (in favor of optimizing storage for solver access) is not a problem.

                cachedDt = dt;
                const int jobsPerWorker = 4;
                var targetJobCount = workerCount * jobsPerWorker;
                bodiesPerJob = bodies.ActiveSet.Count / targetJobCount;
                if (bodiesPerJob == 0)
                    bodiesPerJob = 1;
                availableJobCount = bodies.ActiveSet.Count / bodiesPerJob;
                if (bodiesPerJob * availableJobCount < bodies.ActiveSet.Count)
                    ++availableJobCount;
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(workerDelegate);
                this.threadDispatcher = null;
            }
            else
            {
                var boundingBoxUpdater = new BoundingBoxUpdater(bodies, shapes, broadPhase, pool, dt);
                IntegrateBodies(0, bodies.ActiveSet.Count, dt, ref boundingBoxUpdater);
                boundingBoxUpdater.FlushAndDispose();
            }

        }
    }
}
