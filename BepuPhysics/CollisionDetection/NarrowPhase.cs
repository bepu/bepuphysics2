﻿using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System.Runtime.CompilerServices;
using System;
using System.Diagnostics;
using System.Threading;
using BepuUtilities;
using BepuPhysics.Constraints.Contact;

namespace BepuPhysics.CollisionDetection
{
    /*
     * The narrow phase operates on overlaps generated by the broad phase. 
     * Its job is to compute contact manifolds for overlapping collidables and to manage the constraints produced by those manifolds. 
     * 
     * The scheduling of collision detection jobs is conceptually asynchronous. There is no guarantee that a broad phase overlap provided to the narrow phase
     * will result in an immediate calculation of the manifold. This is useful for batching together many collidable pairs of the same type for simultaneous SIMD-friendly execution.
     * (Not all pairs are ideal fits for wide SIMD, but many common and simple ones are.)
     * 
     * The interface to the broad phase makes no guarantees about the nature of this batching. The narrow phase could immediately execute, or it could batch up Vector<float>.Count,
     * or maybe 32 in a row, or it could wait until all overlaps have been submitted before actually beginning work.
     * 
     * This deferred execution requires that the pending work be stored somehow. This is complicated by the fact that there are a variety of different top level pairs that handle
     * incoming contact manifold data and the resulting constraints in different ways. There are two main distinctions:
     * 1) Continuous collision detection mode. For the purposes of the narrow phase, each collidable can be thought of as discrete or continuous.
     * -Discrete pairs take the result of the underlying manifold and directly manipulate regular contact constraints. 
     * -Continuous pairs perform a sweep to approximate the time of impact between two objects, then submits a pair to the CollisionBatcher with poses integrated to that time.
     * After the collision finishes, the resulting manifold is warped back to the current time to provide high quality speculative contacts.
     * 2) Individual versus compound types. Compound pairs will tend to create child convex pairs and wait for their completion. This ensures the greatest number of simultaneous
     * SIMD-friendly manifold calculations. For example, four compound-compound pairs could result in 60 sphere-capsule subpairs which can then all be executed in a SIMD fashion.
     *  
     * Note that its possible for the evaluation of a pair to generate more pairs. This is currently only seen in compound pairs, but we do permit less obvious cases. 
     * Imagine a high-complexity convex-convex test that has highly divergent execution, but with smaller pieces which are not as divergent.
     * SIMD operations don't map well to divergent execution, so if the individual jobs are large enough, it could be worth it to spawn new pairs for the nondivergent pieces.
     * Most convexes aren't complicated enough to warrant this (often it's faster to simply execute all paths), but it's potentially useful.
     * 
     * In any case where more pairs are generated, evaluating just the current set of pairs is insufficient to guarantee completion. Instead, execution can be thought of like traversing a graph.
     * Each work-creating pair may create an entry on the execution stack if its 'execution threshold' is reached (the arbitrary size which, when reached, results in the execution of the 
     * stored pairs). When no jobs remain on the stack, take any available stored pair set and try to execute it- even if it hasn't yet reached its execution threshold. In this situation,
     * without further action it won't ever fill up, so there's no reason to wait. That execution may then spawn more work, which could create an element on the execution stack, and so on. 
     * Ideally, job sets are consumed in order of their probability of creating new work. That maximizes the number of SIMD-friendly executions.
     * 
     * In practice, there are two phases. The first phase takes in the broad phase-generated top level pairs. At this stage, we do not need to resort to executing incomplete bundles. 
     * Instead, we just continue to work on the top level pairs until none remain. The second phase kicks in here. Since no further top-level work is being generated, we start trying to 
     * flush all the remaining pairs, even if they are not at the execution threshold.
     * 
     * All of the above works within the context of a single thread. There may be many threads in flight, but each one is guaranteed to be handling different top level pairs.
     * That means all of the pair storage is thread local and requires no synchronization. It is also mostly ephemeral- once the thread finishes, only a small amount of information needs
     * to be persisted to globally accessed memory. (Overlap->ConstraintHandle is one common piece of data, but some pairs may also persist other data like separating axes for early outs.
     * Such extra data is fairly rare, since it implies divergence in execution- which is something you don't want in a SIMD-friendly implementation. Likely only in things like hull-hull.)
     * 
     * Every narrow phase pair is responsible for managing the constraints that its computed manifolds require. 
     * This requires the ability to look up existing overlap->constraint relationships for three reasons:
     * 1) Any existing constraint, if it has the same number of contacts as the new manifold, should have its contact data updated.
     * 2) Any accumulated impulse from the previous frame's contact solve should be distributed over the new set of contacts for warm starting this frame's solve.
     * 3) Any change in contact count should result in the removal of the previous constraint (if present) and the addition of the new constraint (if above zero contacts).
     * This mapping is stored in a single dictionary. The previous frame's mapping is treated as read-only during the new frame's narrow phase execution, 
     * so no synchronization is required to read it. The current frame updates pointerse in the dictionary and collects deferred adds on each worker thread for later flushing.
     * 
     * Constraints associated with 'stale' overlaps (those which were not updated during the current frame) are removed in a postpass.
     */


    public enum NarrowPhaseFlushJobType
    {
        RemoveConstraintsFromBodyLists,
        ReturnConstraintHandles,
        RemoveConstraintFromBatchReferencedHandles,
        RemoveConstraintsFromFallbackBatch,
        RemoveConstraintFromTypeBatch,
        FlushPairCacheChanges
    }

    public struct NarrowPhaseFlushJob
    {
        public NarrowPhaseFlushJobType Type;
        public int Index;
    }

    public unsafe abstract class NarrowPhase
    {
        public Simulation Simulation;
        public BufferPool Pool;
        public Bodies Bodies;
        public Statics Statics;
        public Solver Solver;
        public Shapes Shapes;
        public SweepTaskRegistry SweepTaskRegistry;
        public CollisionTaskRegistry CollisionTaskRegistry;
        public ConstraintRemover ConstraintRemover;
        internal FreshnessChecker FreshnessChecker;
        //TODO: It is possible that some types will benefit from per-overlap data, like separating axes. For those, we should have type-dedicated overlap dictionaries.
        //The majority of type pairs, however, only require a constraint handle.
        public PairCache PairCache;
        internal float timestepDuration;

        internal ContactConstraintAccessor[] contactConstraintAccessors;
        public void RegisterContactConstraintAccessor(ContactConstraintAccessor contactConstraintAccessor)
        {
            var id = contactConstraintAccessor.ConstraintTypeId;
            if (contactConstraintAccessors == null || contactConstraintAccessors.Length <= id)
                contactConstraintAccessors = new ContactConstraintAccessor[id + 1];
            if (contactConstraintAccessors[id] != null)
            {
                throw new InvalidOperationException($"Cannot register accessor for type id {id}; it is already registered by {contactConstraintAccessors[id]}.");
            }
            contactConstraintAccessors[id] = contactConstraintAccessor;
        }

        /// <summary>
        /// Looks up the contact constraint accessor for the given constraint type id if it exists.
        /// </summary>
        /// <param name="constraintTypeId">Constraint type id to look up a constraint accessor for.</param>
        /// <param name="accessor">Accessor for the given type id.</param>
        /// <returns>True if the constraint type id refers to a registered accessor, false otherwise.</returns>
        public bool TryGetContactConstraintAccessor(int constraintTypeId, out ContactConstraintAccessor accessor)
        {
            if (IsContactConstraintType(constraintTypeId) && contactConstraintAccessors.Length > constraintTypeId)
            {
                accessor = contactConstraintAccessors[constraintTypeId];
                if (accessor != null)
                    return true;
            }
            accessor = null;
            return false;
        }

        /// <summary>
        /// Tries to extract contact prestep, impulse, and body reference data from the given handle. If it's not a contact constraint, returns false.
        /// </summary>
        /// <typeparam name="TExtractor">Type of the extractor used to collect contact data from the solver.</typeparam>
        /// <param name="constraintHandle">Constraint to try to extract data from.</param>
        /// <param name="extractor">Extractor used to collect contact data from the solver.</param>
        /// <returns>True if the constraint was a contact type, false otherwise.</returns>
        public bool TryExtractSolverContactData<TExtractor>(ConstraintHandle constraintHandle, ref TExtractor extractor) where TExtractor : struct, ISolverContactDataExtractor
        {
            ref var constraintLocation = ref Solver.HandleToConstraint[constraintHandle.Value];
            if (TryGetContactConstraintAccessor(constraintLocation.TypeId, out var accessor))
            {
                accessor.ExtractContactData(constraintLocation, Solver, ref extractor);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Tries to extract prestep and impulse contact data from the given handle. If it's not a contact constraint, returns false.
        /// </summary>
        /// <typeparam name="TExtractor">Type of the extractor used to collect contact data from the solver.</typeparam>
        /// <param name="constraintHandle">Constraint to try to extract data from.</param>
        /// <param name="extractor">Extractor used to collect contact data from the solver.</param>
        /// <returns>True if the constraint was a contact type, false otherwise.</returns>
        public bool TryExtractSolverContactPrestepAndImpulses<TExtractor>(ConstraintHandle constraintHandle, ref TExtractor extractor) where TExtractor : struct, ISolverContactPrestepAndImpulsesExtractor
        {
            ref var constraintLocation = ref Solver.HandleToConstraint[constraintHandle.Value];
            if (TryGetContactConstraintAccessor(constraintLocation.TypeId, out var accessor))
            {
                accessor.ExtractContactPrestepAndImpulses(constraintLocation, Solver, ref extractor);
                return true;
            }
            return false;
        }

        protected NarrowPhase()
        {
            flushWorkerLoop = FlushWorkerLoop;
        }

        /// <summary>
        /// Gets whether a constraint type id maps to a contact constraint.
        /// </summary>
        /// <param name="constraintTypeId">Id of the constraint to check.</param>
        /// <returns>True if the type id refers to a contact constraint. False otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsContactConstraintType(int constraintTypeId)
        {
            Debug.Assert(constraintTypeId >= 0);
            return constraintTypeId < PairCache.CollisionConstraintTypeCount;
        }

        public void Prepare(float dt, IThreadDispatcher threadDispatcher = null)
        {
            timestepDuration = dt;
            OnPrepare(threadDispatcher);
            PairCache.Prepare(threadDispatcher);
            ConstraintRemover.Prepare(threadDispatcher);
        }

        protected abstract void OnPrepare(IThreadDispatcher threadDispatcher);
        protected abstract void OnPreflush(IThreadDispatcher threadDispatcher, bool deterministic);
        protected abstract void OnPostflush(IThreadDispatcher threadDispatcher);


        int flushJobIndex;
        QuickList<NarrowPhaseFlushJob> flushJobs;
        IThreadDispatcher threadDispatcher;
        Action<int> flushWorkerLoop;
        void FlushWorkerLoop(int workerIndex)
        {
            int jobIndex;
            while ((jobIndex = Interlocked.Increment(ref flushJobIndex)) < flushJobs.Count)
            {
                ExecuteFlushJob(ref flushJobs[jobIndex]);
            }
        }
        void ExecuteFlushJob(ref NarrowPhaseFlushJob job)
        {
            switch (job.Type)
            {
                case NarrowPhaseFlushJobType.RemoveConstraintsFromBodyLists:
                    ConstraintRemover.RemoveConstraintsFromBodyLists();
                    break;
                case NarrowPhaseFlushJobType.ReturnConstraintHandles:
                    ConstraintRemover.ReturnConstraintHandles();
                    break;
                case NarrowPhaseFlushJobType.RemoveConstraintFromBatchReferencedHandles:
                    ConstraintRemover.RemoveConstraintsFromBatchReferencedHandles();
                    break;
                case NarrowPhaseFlushJobType.RemoveConstraintsFromFallbackBatch:
                    ConstraintRemover.RemoveConstraintsFromFallbackBatchReferencedHandles();
                    break;
                case NarrowPhaseFlushJobType.RemoveConstraintFromTypeBatch:
                    ConstraintRemover.RemoveConstraintsFromTypeBatch(job.Index);
                    break;
                case NarrowPhaseFlushJobType.FlushPairCacheChanges:
                    PairCache.FlushMappingChanges();
                    break;
            }

        }

        public void Flush(IThreadDispatcher threadDispatcher = null)
        {
            var deterministic = threadDispatcher != null && Simulation.Deterministic;
            //var start = Stopwatch.GetTimestamp();
            OnPreflush(threadDispatcher, deterministic);
            //var end = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Preflush time (us): {1e6 * (end - start) / Stopwatch.Frequency}");
            flushJobs = new QuickList<NarrowPhaseFlushJob>(128, Pool);
            PairCache.PrepareFlushJobs(ref flushJobs);
            var removalBatchJobCount = ConstraintRemover.CreateFlushJobs(deterministic);
            //Note that we explicitly add the constraint remover jobs here. 
            //The constraint remover can be used in two ways- sleeper style, and narrow phase style.
            //In sleeping, we're not actually removing constraints from the simulation completely, so it requires fewer jobs.
            //The constraint remover just lets you choose which jobs to call. The narrow phase needs all of them.
            flushJobs.EnsureCapacity(flushJobs.Count + removalBatchJobCount + 4, Pool);
            flushJobs.AddUnsafely(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.RemoveConstraintsFromBodyLists });
            flushJobs.AddUnsafely(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.ReturnConstraintHandles });
            flushJobs.AddUnsafely(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.RemoveConstraintFromBatchReferencedHandles });
            if (Solver.ActiveSet.Batches.Count > Solver.FallbackBatchThreshold)
            {
                flushJobs.AddUnsafely(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.RemoveConstraintsFromFallbackBatch });
            }
            for (int i = 0; i < removalBatchJobCount; ++i)
            {
                flushJobs.AddUnsafely(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.RemoveConstraintFromTypeBatch, Index = i });
            }

            if (threadDispatcher == null)
            {
                for (int i = 0; i < flushJobs.Count; ++i)
                {
                    ExecuteFlushJob(ref flushJobs[i]);
                }
            }
            else
            {
                flushJobIndex = -1;
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(flushWorkerLoop, flushJobs.Count);
                //flushWorkerLoop(0);
                this.threadDispatcher = null;
            }
            flushJobs.Dispose(Pool);

            PairCache.Postflush();
            ConstraintRemover.MarkAffectedConstraintsAsRemovedFromSolver();
            ConstraintRemover.Postflush();

            OnPostflush(threadDispatcher);
        }

        public void Clear()
        {
            PairCache.Clear();
        }
        public void Dispose()
        {
            PairCache.Dispose();
            OnDispose();
        }

        protected abstract void OnDispose();


        /// <summary>
        /// Sorts references to guarantee that two collidables in the same pair will always be in the same order.
        /// </summary>
        /// <param name="a">First collidable reference to sort.</param>
        /// <param name="b">First collidable reference to sort.</param>
        /// <param name="aMobility">Mobility extracted from collidable A.</param>
        /// <param name="bMobility">Mobility extracted from collidable B.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SortCollidableReferencesForPair(CollidableReference a, CollidableReference b, out CollidableMobility aMobility, out CollidableMobility bMobility, out CollidableReference sortedA, out CollidableReference sortedB)
        {
            //In order to guarantee contact manifold and constraint consistency across multiple frames, the order of collidables submitted to collision testing must be
            //the same every time. Since the provided handles do not move for the lifespan of the collidable in the simulation, they can be used as an ordering.
            //Between two bodies, simply put the lower handle in slot A always.
            //If one of the two objects is static, stick it in the second slot.      
            aMobility = a.Mobility;
            bMobility = b.Mobility;
            if ((aMobility != CollidableMobility.Static && bMobility != CollidableMobility.Static && a.BodyHandle.Value > b.BodyHandle.Value) || aMobility == CollidableMobility.Static)
            {
                sortedA = b;
                sortedB = a;
            }
            else
            {
                sortedA = a;
                sortedB = b;
            }
        }
    }

    /// <summary>
    /// Turns broad phase overlaps into contact manifolds and uses them to manage constraints in the solver.
    /// </summary>
    /// <typeparam name="TCallbacks">Type of the callbacks to use.</typeparam>
    public unsafe partial class NarrowPhase<TCallbacks> : NarrowPhase where TCallbacks : struct, INarrowPhaseCallbacks
    {
        public TCallbacks Callbacks;
        public struct OverlapWorker
        {
            public CollisionBatcher<CollisionCallbacks> Batcher;
            public PendingConstraintAddCache PendingConstraints;
            public QuickList<int> PendingSetAwakenings;

            public OverlapWorker(int workerIndex, BufferPool pool, NarrowPhase<TCallbacks> narrowPhase)
            {
                Batcher = new CollisionBatcher<CollisionCallbacks>(pool, narrowPhase.Shapes, narrowPhase.CollisionTaskRegistry, narrowPhase.timestepDuration,
                    new CollisionCallbacks(workerIndex, pool, narrowPhase));
                PendingConstraints = new PendingConstraintAddCache(pool);
                PendingSetAwakenings = new QuickList<int>(16, pool);
            }
        }

        internal OverlapWorker[] overlapWorkers;

        public NarrowPhase(Simulation simulation, CollisionTaskRegistry collisionTaskRegistry, SweepTaskRegistry sweepTaskRegistry, TCallbacks callbacks,
             int initialSetCapacity, int minimumMappingSize = 2048, int minimumPendingSize = 128)
            : base()
        {
            Simulation = simulation;
            Pool = simulation.BufferPool;
            Shapes = simulation.Shapes;
            Bodies = simulation.Bodies;
            Statics = simulation.Statics;
            Solver = simulation.Solver;
            ConstraintRemover = simulation.constraintRemover;
            Callbacks = callbacks;
            CollisionTaskRegistry = collisionTaskRegistry;
            SweepTaskRegistry = sweepTaskRegistry;
            PairCache = new PairCache(simulation.BufferPool, initialSetCapacity, minimumMappingSize, minimumPendingSize);
            FreshnessChecker = new FreshnessChecker(this);
            preflushWorkerLoop = PreflushWorkerLoop;
        }

        protected override void OnPrepare(IThreadDispatcher threadDispatcher)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            //Resizes should be very rare, and having a single extra very small array isn't concerning.
            //(It's not an unmanaged type because it contains nonblittable references.)
            if (overlapWorkers == null || overlapWorkers.Length < threadCount)
                Array.Resize(ref overlapWorkers, threadCount);
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i] = new OverlapWorker(i, threadDispatcher != null ? threadDispatcher.WorkerPools[i] : Pool, this);
            }
        }

        protected override void OnPostflush(IThreadDispatcher threadDispatcher)
        {
            //TODO: Constraint generators can actually be disposed immediately once the overlap finding process completes.
            //Here, we are disposing them late- that means we suffer a little more wasted memory use. 
            //If you actually wanted to address this, you could add in an OnPreflush or similar.
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i].Batcher.Callbacks.Dispose();
            }
        }

        protected override void OnDispose()
        {
            Callbacks.Dispose();
        }

        public void HandleOverlap(int workerIndex, CollidableReference a, CollidableReference b)
        {
            Debug.Assert(a.Packed != b.Packed, "Excuse me, broad phase, but an object cannot collide with itself!");
            SortCollidableReferencesForPair(a, b, out var aMobility, out var bMobility, out a, out b);
            Debug.Assert(aMobility != CollidableMobility.Static || bMobility != CollidableMobility.Static, "Broad phase should not be able to generate static-static pairs.");

            //Two static pairs are impossible (the broad phase doesn't test stuff in the static/sleeping tree against itself), and any pair with a static will put the body in slot A.
            var twoBodies = bMobility != CollidableMobility.Static;
            ref var bodyLocationA = ref Bodies.HandleToLocation[a.BodyHandle.Value];
            ref var setA = ref Bodies.Sets[bodyLocationA.SetIndex];
            ref var stateA = ref setA.DynamicsState[bodyLocationA.Index];
            ref var collidableA = ref setA.Collidables[bodyLocationA.Index];
            float speculativeMarginB;
            if (twoBodies)
            {
                ref var bodyLocationB = ref Bodies.HandleToLocation[b.BodyHandle.Value];
                ref var collidableB = ref Bodies.Sets[bodyLocationB.SetIndex].Collidables[bodyLocationB.Index];
                speculativeMarginB = collidableB.SpeculativeMargin;
            }
            else
            {
                //Slot B is a static.
                speculativeMarginB = 0;
            }

            //Add the speculative margins. This is conservative; the speculative margins were computed as a worst case based on the velocity of the body,
            //then clamped by the collidable's min/max margin values. Adding them together means an unlimited margin will result in speculative contacts
            //being generated for the pair if the velocity would bring them into contact.

            //Note that this margin *could* be kept smaller within a pair by only storing out the angular contribution to the speculative margin target
            //and then expanding the pair by the magnitude of the relative linear velocity.
            //However, loading the velocities here isn't free. In tests, it usually came out slower than just using the more generous speculative margin.
            var speculativeMargin = collidableA.SpeculativeMargin + speculativeMarginB;

            //By precalculating the speculative margin, we give the narrow phase callbacks the option of modifying it.
            if (!Callbacks.AllowContactGeneration(workerIndex, a, b, ref speculativeMargin))
                return;
            ref var overlapWorker = ref overlapWorkers[workerIndex];
            var pair = new CollidablePair(a, b);
            if (twoBodies)
            {
                //Both references are bodies.
                ref var bodyLocationB = ref Bodies.HandleToLocation[b.BodyHandle.Value];
                Debug.Assert(bodyLocationA.SetIndex == 0 || bodyLocationB.SetIndex == 0, "One of the two bodies must be active. Otherwise, something is busted!");
                ref var setB = ref Bodies.Sets[bodyLocationB.SetIndex];
                ref var stateB = ref setB.DynamicsState[bodyLocationB.Index];
                ref var collidableB = ref setB.Collidables[bodyLocationB.Index];
                AddBatchEntries(workerIndex, ref overlapWorker, ref pair,
                    ref collidableA.Continuity, ref collidableB.Continuity,
                    collidableA.Shape, collidableB.Shape,
                    collidableA.BroadPhaseIndex, collidableB.BroadPhaseIndex,
                    speculativeMargin,
                    ref stateA.Motion.Pose, ref stateB.Motion.Pose,
                    ref stateA.Motion.Velocity, ref stateB.Motion.Velocity);
            }
            else
            {
                //Since we disallow 2-static pairs and we guarantee the second slot holds the static if it exists, we know that A is a body and B is a static.
                //Further, we know that the body must be an *active* body, because inactive bodies and statics exist within the same static/inactive broad phase tree and are not tested
                //against each other.
                Debug.Assert(aMobility != CollidableMobility.Static && bMobility == CollidableMobility.Static);
                ref var bodyLocation = ref Bodies.HandleToLocation[a.BodyHandle.Value];
                Debug.Assert(bodyLocation.SetIndex == 0, "The body of a body-static pair must be active.");

                //TODO: Ideally, the compiler would see this and optimize away the relevant math in AddBatchEntries. That's a longshot, though. May want to abuse some generics to force it.
                var zeroVelocity = default(BodyVelocity);
                ref var staticB = ref Statics.GetDirectReference(b.StaticHandle);
                AddBatchEntries(workerIndex, ref overlapWorker, ref pair,
                    ref collidableA.Continuity, ref staticB.Continuity,
                    collidableA.Shape, staticB.Shape,
                    collidableA.BroadPhaseIndex, staticB.BroadPhaseIndex,
                    speculativeMargin,
                    ref stateA.Motion.Pose, ref staticB.Pose,
                    ref stateA.Motion.Velocity, ref zeroVelocity);
            }

        }

        struct CCDSweepFilter : ISweepFilter
        {
            public NarrowPhase<TCallbacks> NarrowPhase;
            public CollidablePair Pair;
            public int WorkerIndex;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowTest(int childA, int childB)
            {
                return NarrowPhase.Callbacks.AllowContactGeneration(WorkerIndex, Pair, childA, childB);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddBatchEntries(int workerIndex, ref OverlapWorker overlapWorker,
            ref CollidablePair pair,
            ref ContinuousDetection continuityA, ref ContinuousDetection continuityB,
            TypedIndex shapeA, TypedIndex shapeB,
            int broadPhaseIndexA, int broadPhaseIndexB,
            float speculativeMargin,
            ref RigidPose poseA, ref RigidPose poseB,
            ref BodyVelocity velocityA, ref BodyVelocity velocityB)
        {
            Debug.Assert(pair.A.Packed != pair.B.Packed);
            var allowExpansion = continuityA.AllowExpansionBeyondSpeculativeMargin | continuityB.AllowExpansionBeyondSpeculativeMargin;
            //Note that we pick float.MaxValue for the maximum bounds expansion passive-involving pairs.
            //This is a compromise- looser bounds are not a correctness issue, so we're trading off potentially more subpairs
            //and the need to compute a tighter maximum bound. That's not incredibly expensive, but it does add up. For now, we use the looser bound under the assumption
            //that the vast majority of pairs won't benefit from the tighter bound.
            var maximumExpansion = allowExpansion ? float.MaxValue : speculativeMargin;

            //Create a continuation for the pair given the CCD state.
            //Note that we never create 'unilateral' CCD pairs. That is, if either collidable in a pair enables a CCD feature, we just act like both are using it.
            //That keeps things a little simpler. Unlike v1, we don't have to worry about the implications of 'motion clamping' here- no need for deeper configuration.            
            CCDContinuationIndex continuationIndex = default;
            if (continuityA.Mode == ContinuousDetectionMode.Continuous || continuityB.Mode == ContinuousDetectionMode.Continuous)
            {
                var sweepTask = SweepTaskRegistry.GetTask(shapeA.Type, shapeB.Type);
                if (sweepTask != null)
                {
                    //Not every continuous pair requires an actual sweep test. If the maximum approaching displacement for any point on the involved shapes isn't any larger
                    //than the speculative margin, then we don't need to perform a sweep- we can assume that the speculative margin will take care of it.
                    //sweepRequired = (||angularVelocityA|| * maximumRadiusA + ||angularVelocityB|| * maximumRadiusB + ||relativeLinearVelocity||) * dt > speculativeMargin
                    //Unfortunately, there's no easy and quick way to grab a reliable maximum radius for all shape types. Convexes have a relatively cheap value (though it may
                    //involve a square root sometimes), but compounds tend to require heavier lifting and iteration.
                    //Given that this should be a pretty rarely used loose optimization, we'll instead make use of the bounding boxes to create an estimate.
                    //Note that the broad phase already touched this data on this thread, so it's still available in L1. (This function is called from broad phase collision testing.)
                    //TODO: May want to reconsider this approach if you end up caching more properties on the shape (or if profiling suggests it is a concern).
                    var aInStaticTree = pair.A.Mobility == CollidableMobility.Static || Simulation.Bodies.HandleToLocation[pair.A.BodyHandle.Value].SetIndex > 0;
                    var bInStaticTree = pair.B.Mobility == CollidableMobility.Static || Simulation.Bodies.HandleToLocation[pair.B.BodyHandle.Value].SetIndex > 0;
                    ref var aTree = ref aInStaticTree ? ref Simulation.BroadPhase.StaticTree : ref Simulation.BroadPhase.ActiveTree;
                    ref var bTree = ref bInStaticTree ? ref Simulation.BroadPhase.StaticTree : ref Simulation.BroadPhase.ActiveTree;
                    aTree.GetBoundsPointers(broadPhaseIndexA, out var aMin, out var aMax);
                    bTree.GetBoundsPointers(broadPhaseIndexB, out var bMin, out var bMax);
                    var maximumRadiusA = (*aMax - *aMin).Length() * 0.5f;
                    var maximumRadiusB = (*bMax - *bMin).Length() * 0.5f;
                    if ((velocityA.Angular.Length() * maximumRadiusA + velocityB.Angular.Length() * maximumRadiusB + (velocityB.Linear - velocityA.Linear).Length()) * timestepDuration > speculativeMargin)
                    {
                        Simulation.Shapes[shapeA.Type].GetShapeData(shapeA.Index, out var shapeDataA, out var shapeSizeA);
                        Simulation.Shapes[shapeB.Type].GetShapeData(shapeB.Index, out var shapeDataB, out var shapeSizeB);
                        float minimumSweepTimestepA, sweepConvergenceThresholdA;
                        if (continuityA.Mode == ContinuousDetectionMode.Continuous)
                        {
                            minimumSweepTimestepA = continuityA.MinimumSweepTimestep;
                            sweepConvergenceThresholdA = continuityA.SweepConvergenceThreshold;
                        }
                        else
                        {
                            minimumSweepTimestepA = float.MaxValue;
                            sweepConvergenceThresholdA = float.MaxValue;
                        }
                        float minimumSweepTimestepB, sweepConvergenceThresholdB;
                        if (continuityB.Mode == ContinuousDetectionMode.Continuous)
                        {
                            minimumSweepTimestepB = continuityB.MinimumSweepTimestep;
                            sweepConvergenceThresholdB = continuityB.SweepConvergenceThreshold;
                        }
                        else
                        {
                            minimumSweepTimestepB = float.MaxValue;
                            sweepConvergenceThresholdB = float.MaxValue;
                        }
                        var filter = new CCDSweepFilter { NarrowPhase = this, Pair = pair, WorkerIndex = workerIndex };
                        if (sweepTask.Sweep(
                            shapeDataA, shapeA.Type, poseA.Orientation, velocityA,
                            shapeDataB, shapeB.Type, poseB.Position - poseA.Position, poseB.Orientation, velocityB,
                            timestepDuration,
                            //Note that we use the *smaller* thresholds. This allows high fidelity objects to demand more time even if paired with low fidelity objects.
                            Math.Min(minimumSweepTimestepA, minimumSweepTimestepB),
                            Math.Min(sweepConvergenceThresholdA, sweepConvergenceThresholdB), 25, //Note the fixed but high iteration threshold.
                            ref filter, Simulation.Shapes, SweepTaskRegistry, overlapWorker.Batcher.Pool, out _, out var t1, out _, out _))
                        {
                            //Create the pair at a position known to be intersecting from the sweep test. t0 and t1 are the bounding region of the first time of impact,
                            //so we pick the later one (t1). The continuation handler will 'rewind' the depths to create speculative contacts.
                            continuationIndex = overlapWorker.Batcher.Callbacks.AddContinuous(ref pair, velocityB.Linear - velocityA.Linear, velocityA.Angular, velocityB.Angular, t1);

                            //The poses should be as they will be at t1, not where they are now. Velocity is treated as constant throughout the the timestep.
                            PoseIntegration.Integrate(poseA.Orientation, velocityA.Angular, t1, out var integratedOrientationA);
                            PoseIntegration.Integrate(poseB.Orientation, velocityB.Angular, t1, out var integratedOrientationB);
                            var offsetB = poseB.Position - poseA.Position + (velocityB.Linear - velocityA.Linear) * t1;
                            overlapWorker.Batcher.Add(
                               shapeA, shapeB,
                               offsetB, integratedOrientationA, integratedOrientationB, velocityA, velocityB,
                               speculativeMargin, maximumExpansion, new PairContinuation((int)continuationIndex.Packed));
                        }
                    }
                }
            }
            if (!continuationIndex.Exists)
            {
                //No CCD continuation was created, so create a discrete one.
                continuationIndex = overlapWorker.Batcher.Callbacks.AddDiscrete(ref pair);
                overlapWorker.Batcher.Add(
                   shapeA, shapeB,
                   poseB.Position - poseA.Position, poseA.Orientation, poseB.Orientation, velocityA, velocityB,
                   speculativeMargin, maximumExpansion, new PairContinuation((int)continuationIndex.Packed));
            }
        }
    }
}