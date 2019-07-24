using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuPhysics.Trees;

namespace BepuPhysics
{
    /// <summary>
    /// Orchestrates the bookkeeping and execution of a full dynamic simulation.
    /// </summary>
    public partial class Simulation : IDisposable
    {
        public IslandAwakener Awakener { get; private set; }
        public IslandSleeper Sleeper { get; private set; }
        public Bodies Bodies { get; private set; }
        public Statics Statics { get; private set; }
        public Shapes Shapes { get; private set; }
        public BodyLayoutOptimizer BodyLayoutOptimizer { get; private set; }
        public ConstraintLayoutOptimizer ConstraintLayoutOptimizer { get; private set; }
        public BatchCompressor SolverBatchCompressor { get; private set; }
        public Solver Solver { get; private set; }
        public IPoseIntegrator PoseIntegrator { get; private set; }
        public BroadPhase BroadPhase { get; private set; }
        public CollidableOverlapFinder BroadPhaseOverlapFinder { get; private set; }
        public NarrowPhase NarrowPhase { get; private set; }

        SimulationProfiler profiler = new SimulationProfiler(13);
        /// <summary>
        /// Gets the simulation profiler. Note that the SimulationProfiler implementation only exists when the library is compiled with the PROFILE compilation symbol; if not defined, returned times are undefined.
        /// </summary>
        public SimulationProfiler Profiler { get { return profiler; } }

        //Helpers shared across at least two stages.
        internal ConstraintRemover constraintRemover;

        /// <summary>
        /// Gets the main memory pool used to fill persistent structures and main thread ephemeral resources across the engine.
        /// </summary>
        public BufferPool BufferPool { get; private set; }

        /// <summary>
        /// Gets the timestepper used to update the simulation state.
        /// </summary>
        public ITimestepper Timestepper { get; private set; }

        /// <summary>
        /// Gets or sets whether to use a deterministic time step when using multithreading. When set to true, additional time is spent sorting constraint additions and transfers.
        /// Note that this can only affect determinism locally- different processor architectures may implement instructions differently.
        /// </summary>
        public bool Deterministic { get; set; }

        protected Simulation(BufferPool bufferPool, SimulationAllocationSizes initialAllocationSizes, int solverIterationCount, int solverFallbackBatchThreshold, ITimestepper timestepper)
        {
            BufferPool = bufferPool;
            Shapes = new Shapes(bufferPool, initialAllocationSizes.ShapesPerType);
            BroadPhase = new BroadPhase(bufferPool, initialAllocationSizes.Bodies, initialAllocationSizes.Bodies + initialAllocationSizes.Statics);
            Bodies = new Bodies(bufferPool, Shapes, BroadPhase,
                initialAllocationSizes.Bodies,
                initialAllocationSizes.Islands,
                initialAllocationSizes.ConstraintCountPerBodyEstimate);
            Statics = new Statics(bufferPool, Shapes, Bodies, BroadPhase, initialAllocationSizes.Statics);

            Solver = new Solver(Bodies, BufferPool, solverIterationCount, solverFallbackBatchThreshold,
                initialCapacity: initialAllocationSizes.Constraints,
                initialIslandCapacity: initialAllocationSizes.Islands,
                minimumCapacityPerTypeBatch: initialAllocationSizes.ConstraintsPerTypeBatch);
            constraintRemover = new ConstraintRemover(BufferPool, Bodies, Solver);
            Sleeper = new IslandSleeper(Bodies, Solver, BroadPhase, constraintRemover, BufferPool);
            Awakener = new IslandAwakener(Bodies, Statics, Solver, BroadPhase, Sleeper, bufferPool);
            Statics.awakener = Awakener;
            Solver.awakener = Awakener;
            Bodies.Initialize(Solver, Awakener, Sleeper);
            SolverBatchCompressor = new BatchCompressor(Solver, Bodies);
            BodyLayoutOptimizer = new BodyLayoutOptimizer(Bodies, BroadPhase, Solver, bufferPool);
            ConstraintLayoutOptimizer = new ConstraintLayoutOptimizer(Bodies, Solver);
            Timestepper = timestepper;

        }

        /// <summary>
        /// Constructs a simulation supporting dynamic movement and constraints with the specified narrow phase callbacks.
        /// </summary>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        /// <param name="narrowPhaseCallbacks">Callbacks to use in the narrow phase.</param>
        /// <param name="poseIntegratorCallbacks">Callbacks to use in the pose integrator.</param>
        /// <param name="timestepper">Timestepper that defines how the simulation state should be updated. If null, defaults to PositionFirstTimestepper.</param>
        /// <param name="solverIterationCount">Number of iterations the solver should use.</param>
        /// <param name="solverFallbackBatchThreshold">Number of synchronized batches the solver should maintain before falling back to a lower quality jacobi hybrid solver.</param>
        /// <param name="initialAllocationSizes">Allocation sizes to initialize the simulation with. If left null, default values are chosen.</param>
        /// <returns>New simulation.</returns>
        public static Simulation Create<TNarrowPhaseCallbacks, TPoseIntegratorCallbacks>(
            BufferPool bufferPool, TNarrowPhaseCallbacks narrowPhaseCallbacks, TPoseIntegratorCallbacks poseIntegratorCallbacks, ITimestepper timestepper = null,
            int solverIterationCount = 8, int solverFallbackBatchThreshold = 64, SimulationAllocationSizes? initialAllocationSizes = null)
            where TNarrowPhaseCallbacks : struct, INarrowPhaseCallbacks
            where TPoseIntegratorCallbacks : struct, IPoseIntegratorCallbacks
        {
            if (initialAllocationSizes == null)
            {
                initialAllocationSizes = new SimulationAllocationSizes
                {
                    Bodies = 4096,
                    Statics = 4096,
                    ShapesPerType = 128,
                    ConstraintCountPerBodyEstimate = 8,
                    Constraints = 16384,
                    ConstraintsPerTypeBatch = 256
                };
            }

            var simulation = new Simulation(bufferPool, initialAllocationSizes.Value, solverIterationCount, solverFallbackBatchThreshold, timestepper == null ? new PositionFirstTimestepper() : timestepper);
            simulation.PoseIntegrator = new PoseIntegrator<TPoseIntegratorCallbacks>(simulation.Bodies, simulation.Shapes, simulation.BroadPhase, poseIntegratorCallbacks);
            var narrowPhase = new NarrowPhase<TNarrowPhaseCallbacks>(simulation,
                DefaultTypes.CreateDefaultCollisionTaskRegistry(), DefaultTypes.CreateDefaultSweepTaskRegistry(),
                narrowPhaseCallbacks, initialAllocationSizes.Value.Islands + 1);
            DefaultTypes.RegisterDefaults(simulation.Solver, narrowPhase);
            simulation.NarrowPhase = narrowPhase;
            simulation.Sleeper.pairCache = narrowPhase.PairCache;
            simulation.Awakener.pairCache = narrowPhase.PairCache;
            simulation.Solver.pairCache = narrowPhase.PairCache;
            simulation.BroadPhaseOverlapFinder = new CollidableOverlapFinder<TNarrowPhaseCallbacks>(narrowPhase, simulation.BroadPhase);

            return simulation;
        }



        private int ValidateAndCountShapefulBodies(ref BodySet bodySet, ref Tree tree, ref Buffer<CollidableReference> leaves)
        {
            int shapefulBodyCount = 0;
            for (int i = 0; i < bodySet.Count; ++i)
            {
                ref var collidable = ref bodySet.Collidables[i];
                if (collidable.Shape.Exists)
                {
                    Debug.Assert(collidable.BroadPhaseIndex >= 0 && collidable.BroadPhaseIndex < tree.LeafCount);
                    ref var leaf = ref leaves[collidable.BroadPhaseIndex];
                    Debug.Assert(leaf.Handle == bodySet.IndexToHandle[i]);
                    Debug.Assert(leaf.Mobility == CollidableMobility.Dynamic || leaf.Mobility == CollidableMobility.Kinematic);
                    ++shapefulBodyCount;
                }
            }
            return shapefulBodyCount;
        }


        [Conditional("DEBUG")]
        internal void ValidateCollidables()
        {
            var activeShapefulBodyCount = ValidateAndCountShapefulBodies(ref Bodies.ActiveSet, ref BroadPhase.ActiveTree, ref BroadPhase.activeLeaves);
            Debug.Assert(BroadPhase.ActiveTree.LeafCount == activeShapefulBodyCount);

            int inactiveShapefulBodyCount = 0;

            for (int setIndex = 1; setIndex < Bodies.Sets.Length; ++setIndex)
            {
                ref var set = ref Bodies.Sets[setIndex];
                if (set.Allocated)
                {
                    inactiveShapefulBodyCount += ValidateAndCountShapefulBodies(ref set, ref BroadPhase.StaticTree, ref BroadPhase.staticLeaves);
                }
            }
            Debug.Assert(inactiveShapefulBodyCount + Statics.Count == BroadPhase.StaticTree.LeafCount);
            for (int i = 0; i < Statics.Count; ++i)
            {
                ref var collidable = ref Statics.Collidables[i];
                Debug.Assert(collidable.Shape.Exists, "All static collidables must have shapes. That's their only purpose.");

                Debug.Assert(collidable.BroadPhaseIndex >= 0 && collidable.BroadPhaseIndex < BroadPhase.StaticTree.LeafCount);
                ref var leaf = ref BroadPhase.staticLeaves[collidable.BroadPhaseIndex];
                Debug.Assert(leaf.Handle == Statics.IndexToHandle[i]);
                Debug.Assert(leaf.Mobility == CollidableMobility.Static);
            }

            //Ensure there are no duplicates between the two broad phase trees.
            for (int i = 0; i < BroadPhase.ActiveTree.LeafCount; ++i)
            {
                var activeLeaf = BroadPhase.activeLeaves[i];
                for (int j = 0; j < BroadPhase.StaticTree.LeafCount; ++j)
                {
                    Debug.Assert(BroadPhase.staticLeaves[j].Packed != activeLeaf.Packed);
                }
            }

        }

        //These functions act as convenience wrappers around common execution patterns. They can be mixed and matched in custom timesteps, or for certain advanced use cases, called directly.
        /// <summary>
        /// Executes the sleep stage, moving candidate
        /// </summary>
        /// <param name="threadDispatcher">Thread dispatcher to use for the sleeper execution, if any.</param>
        public void Sleep(IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(Sleeper);
            Sleeper.Update(threadDispatcher, Deterministic);
            profiler.End(Sleeper);
        }

        /// <summary>
        /// Updates the position, velocity, world inertia, deactivation candidacy and bounding boxes of active bodies.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void IntegrateBodiesAndUpdateBoundingBoxes(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(PoseIntegrator);
            PoseIntegrator.IntegrateBodiesAndUpdateBoundingBoxes(dt, BufferPool, threadDispatcher);
            profiler.End(PoseIntegrator);
        }

        /// <summary>
        /// Predicts the bounding boxes of active bodies by speculatively integrating velocity. Does not actually modify body velocities. Updates deactivation candidacy.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void PredictBoundingBoxes(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(PoseIntegrator);
            PoseIntegrator.PredictBoundingBoxes(dt, BufferPool, threadDispatcher);
            profiler.End(PoseIntegrator);
        }

        /// <summary>
        /// Updates the velocities, world space inertias, bounding boxes, and deactivation candidacy of active bodies.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void IntegrateVelocitiesBoundsAndInertias(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(PoseIntegrator);
            PoseIntegrator.IntegrateVelocitiesBoundsAndInertias(dt, BufferPool, threadDispatcher);
            profiler.End(PoseIntegrator);
        }

        /// <summary>
        /// Updates the velocities and world space inertias of active bodies.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void IntegrateVelocitiesAndUpdateInertias(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(PoseIntegrator);
            PoseIntegrator.IntegrateVelocitiesAndUpdateInertias(dt, BufferPool, threadDispatcher);
            profiler.End(PoseIntegrator);
        }

        /// <summary>
        /// Updates the poses of active bodies.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void IntegratePoses(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(PoseIntegrator);
            PoseIntegrator.IntegratePoses(dt, BufferPool, threadDispatcher);
            profiler.End(PoseIntegrator);
        }

        /// <summary>
        /// Updates the broad phase structure for the current body bounding boxes, finds potentially colliding pairs, and then executes the narrow phase for all such pairs. Generates contact constraints for the solver.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void CollisionDetection(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(BroadPhase);
            BroadPhase.Update(threadDispatcher);
            profiler.End(BroadPhase);

            profiler.Start(BroadPhaseOverlapFinder);
            BroadPhaseOverlapFinder.DispatchOverlaps(dt, threadDispatcher);
            profiler.End(BroadPhaseOverlapFinder);

            profiler.Start(NarrowPhase);
            NarrowPhase.Flush(threadDispatcher);
            profiler.End(NarrowPhase);
        }

        /// <summary>
        /// Uses the current body velocities to incrementally update all active contact constraint penetration depths.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void IncrementallyUpdateContactConstraints(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(Solver);
            Solver.IncrementallyUpdateContactConstraints(dt, threadDispatcher);
            profiler.End(Solver);
        }

        /// <summary>
        /// Solves all active constraints in the simulation.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void Solve(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Start(Solver);
            Solver.Solve(dt, threadDispatcher);
            profiler.End(Solver);
        }

        /// <summary>
        /// Incrementally improves body and constraint storage for better performance.
        /// </summary>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void IncrementallyOptimizeDataStructures(IThreadDispatcher threadDispatcher = null)
        {
            //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies - and so affects the optimal constraint position.
            //TODO: The order of these optimizer stages is performance relevant, even though they don't have any effect on correctness.
            //You may want to try them in different locations to see how they impact cache residency.
            profiler.Start(BodyLayoutOptimizer);
            BodyLayoutOptimizer.IncrementalOptimize();
            profiler.End(BodyLayoutOptimizer);

            profiler.Start(ConstraintLayoutOptimizer);
            ConstraintLayoutOptimizer.Update(BufferPool, threadDispatcher);
            profiler.End(ConstraintLayoutOptimizer);

            profiler.Start(SolverBatchCompressor);
            SolverBatchCompressor.Compress(BufferPool, threadDispatcher, threadDispatcher != null && Deterministic);
            profiler.End(SolverBatchCompressor);
        }

        //TODO: I wonder if people will abuse the dt-as-parameter to the point where we should make it a field instead, like it effectively was in v1.
        /// <summary>
        /// Performs one timestep of the given length.
        /// </summary>
        /// <remarks>
        /// Be wary of variable timesteps. They can harm stability. Whenever possible, keep the timestep the same across multiple frames unless you have a specific reason not to.
        /// </remarks>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void Timestep(float dt, IThreadDispatcher threadDispatcher = null)
        {
            profiler.Clear();
            profiler.Start(this);

            Timestepper.Timestep(this, dt, threadDispatcher);

            profiler.End(this);
        }

        /// <summary>
        /// Clears the simulation of every object, only returning memory to the pool that would be returned by sequential removes. 
        /// Other persistent allocations, like those in the Bodies set, will remain.
        /// </summary>
        public void Clear()
        {
            Solver.Clear();
            Bodies.Clear();
            Statics.Clear();
            Shapes.Clear();
            BroadPhase.Clear();
            NarrowPhase.Clear();
            Sleeper.Clear();
        }

        /// <summary>
        /// Increases the allocation size of any buffers too small to hold the allocation target.
        /// </summary>
        /// <remarks>
        /// <para>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </para>
        /// <para>
        /// This is primarily a convenience function. Everything it does internally can be done externally.
        /// For example, if only type batches need to be resized, the solver's own functions can be used directly.
        /// </para>
        /// </remarks>
        /// <param name="allocationTarget">Allocation sizes to guarantee sufficient size for.</param>
        public void EnsureCapacity(SimulationAllocationSizes allocationTarget)
        {
            Solver.EnsureSolverCapacities(allocationTarget.Bodies, allocationTarget.Constraints);
            Solver.MinimumCapacityPerTypeBatch = Math.Max(allocationTarget.ConstraintsPerTypeBatch, Solver.MinimumCapacityPerTypeBatch);
            Solver.EnsureTypeBatchCapacities();
            NarrowPhase.PairCache.EnsureConstraintToPairMappingCapacity(Solver, allocationTarget.Constraints);
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.EnsureCapacity(allocationTarget.Bodies);
            Bodies.MinimumConstraintCapacityPerBody = allocationTarget.ConstraintCountPerBodyEstimate;
            Bodies.EnsureConstraintListCapacities();
            Sleeper.EnsureSetsCapacity(allocationTarget.Islands + 1);
            Statics.EnsureCapacity(allocationTarget.Statics);
            Shapes.EnsureBatchCapacities(allocationTarget.ShapesPerType);
            BroadPhase.EnsureCapacity(allocationTarget.Bodies, allocationTarget.Bodies + allocationTarget.Statics);
        }


        /// <summary>
        /// Increases the allocation size of any buffers too small to hold the allocation target, and decreases the allocation size of any buffers that are unnecessarily large.
        /// </summary>
        /// <remarks>
        /// <para>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </para>
        /// <para>
        /// This is primarily a convenience function. Everything it does internally can be done externally.
        /// For example, if only type batches need to be resized, the solver's own functions can be used directly.
        /// </para>
        /// </remarks>
        /// <param name="allocationTarget">Allocation sizes to guarantee sufficient size for.</param>
        public void Resize(SimulationAllocationSizes allocationTarget)
        {
            Solver.ResizeSolverCapacities(allocationTarget.Bodies, allocationTarget.Constraints);
            Solver.MinimumCapacityPerTypeBatch = allocationTarget.ConstraintsPerTypeBatch;
            Solver.ResizeTypeBatchCapacities();
            NarrowPhase.PairCache.ResizeConstraintToPairMappingCapacity(Solver, allocationTarget.Constraints);
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.Resize(allocationTarget.Bodies);
            Bodies.MinimumConstraintCapacityPerBody = allocationTarget.ConstraintCountPerBodyEstimate;
            Bodies.ResizeConstraintListCapacities();
            Sleeper.ResizeSetsCapacity(allocationTarget.Islands + 1);
            Statics.Resize(allocationTarget.Statics);
            Shapes.ResizeBatches(allocationTarget.ShapesPerType);
            BroadPhase.Resize(allocationTarget.Bodies, allocationTarget.Bodies + allocationTarget.Statics);
        }

        /// <summary>
        /// Clears the simulation of every object and returns all pooled memory to the buffer pool. Leaves the simulation in an unusable state.
        /// </summary>
        public void Dispose()
        {
            Clear();
            Sleeper.Dispose();
            Solver.Dispose();
            BroadPhase.Dispose();
            NarrowPhase.Dispose();
            Bodies.Dispose();
            Statics.Dispose();
            Shapes.Dispose();
        }
    }
}
