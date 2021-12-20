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

#if !DEBUG
[module: SkipLocalsInit]
#endif

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
        public BatchCompressor SolverBatchCompressor { get; private set; }
        public Solver Solver { get; private set; }
        public IPoseIntegrator PoseIntegrator { get; private set; }
        public BroadPhase BroadPhase { get; private set; }
        public CollidableOverlapFinder BroadPhaseOverlapFinder { get; private set; }
        public NarrowPhase NarrowPhase { get; private set; }

        SimulationProfiler profiler = new(13);
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

        /// <summary>
        /// Constructs a simulation supporting dynamic movement and constraints with the specified narrow phase callbacks.
        /// </summary>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        /// <param name="narrowPhaseCallbacks">Callbacks to use in the narrow phase.</param>
        /// <param name="poseIntegratorCallbacks">Callbacks to use in the pose integrator.</param>
        /// <param name="timestepper">Timestepper that defines how the simulation state should be updated. If null, <see cref="DefaultTimestepper"/> is used.</param>
        /// <param name="solveDescription">Describes how the solver should execute, including the number of substeps and the number of velocity iterations per substep.</param>
        /// <param name="initialAllocationSizes">Allocation sizes to initialize the simulation with. If left null, default values are chosen.</param>
        /// <returns>New simulation.</returns>
        public static Simulation Create<TNarrowPhaseCallbacks, TPoseIntegratorCallbacks>(
            BufferPool bufferPool, TNarrowPhaseCallbacks narrowPhaseCallbacks, TPoseIntegratorCallbacks poseIntegratorCallbacks, SolveDescription solveDescription, ITimestepper timestepper = null, SimulationAllocationSizes? initialAllocationSizes = null)
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

            //var simulation = new Simulation(bufferPool, initialAllocationSizes.Value, solverIterationCount, solverFallbackBatchThreshold, timestepper);
            var simulation = new Simulation();
            simulation.BufferPool = bufferPool;
            simulation.Shapes = new Shapes(bufferPool, initialAllocationSizes.Value.ShapesPerType);
            simulation.BroadPhase = new BroadPhase(bufferPool, initialAllocationSizes.Value.Bodies, initialAllocationSizes.Value.Bodies + initialAllocationSizes.Value.Statics);
            simulation.Bodies = new Bodies(bufferPool, simulation.Shapes, simulation.BroadPhase,
                initialAllocationSizes.Value.Bodies,
                initialAllocationSizes.Value.Islands,
                initialAllocationSizes.Value.ConstraintCountPerBodyEstimate);
            simulation.Statics = new Statics(bufferPool, simulation.Shapes, simulation.Bodies, simulation.BroadPhase, initialAllocationSizes.Value.Statics);

            var poseIntegrator = new PoseIntegrator<TPoseIntegratorCallbacks>(simulation.Bodies, simulation.Shapes, simulation.BroadPhase, poseIntegratorCallbacks);
            simulation.PoseIntegrator = poseIntegrator;

            simulation.Solver = new Solver<TPoseIntegratorCallbacks>(simulation.Bodies, simulation.BufferPool, solveDescription,
                initialCapacity: initialAllocationSizes.Value.Constraints,
                initialIslandCapacity: initialAllocationSizes.Value.Islands,
                minimumCapacityPerTypeBatch: initialAllocationSizes.Value.ConstraintsPerTypeBatch, poseIntegrator);
            simulation.constraintRemover = new ConstraintRemover(simulation.BufferPool, simulation.Bodies, simulation.Solver);
            simulation.Sleeper = new IslandSleeper(simulation.Bodies, simulation.Solver, simulation.BroadPhase, simulation.constraintRemover, simulation.BufferPool);
            simulation.Awakener = new IslandAwakener(simulation.Bodies, simulation.Statics, simulation.Solver, simulation.BroadPhase, simulation.Sleeper, bufferPool);
            simulation.Statics.awakener = simulation.Awakener;
            simulation.Solver.awakener = simulation.Awakener;
            simulation.Bodies.Initialize(simulation.Solver, simulation.Awakener, simulation.Sleeper);
            simulation.SolverBatchCompressor = new BatchCompressor(simulation.Solver, simulation.Bodies);
            simulation.Timestepper = timestepper ?? new DefaultTimestepper();

            var narrowPhase = new NarrowPhase<TNarrowPhaseCallbacks>(simulation,
                DefaultTypes.CreateDefaultCollisionTaskRegistry(), DefaultTypes.CreateDefaultSweepTaskRegistry(),
                narrowPhaseCallbacks, initialAllocationSizes.Value.Islands + 1);
            DefaultTypes.RegisterDefaults(simulation.Solver, narrowPhase);
            simulation.NarrowPhase = narrowPhase;
            simulation.Sleeper.pairCache = narrowPhase.PairCache;
            simulation.Awakener.pairCache = narrowPhase.PairCache;
            simulation.Solver.pairCache = narrowPhase.PairCache;
            simulation.BroadPhaseOverlapFinder = new CollidableOverlapFinder<TNarrowPhaseCallbacks>(narrowPhase, simulation.BroadPhase);

            //We defer initialization until after all the other simulation bits are constructed.
            poseIntegrator.Callbacks.Initialize(simulation);
            narrowPhase.Callbacks.Initialize(simulation);

            return simulation;
        }



        private static int ValidateAndCountShapefulBodies(ref BodySet bodySet, ref Tree tree, ref Buffer<CollidableReference> leaves)
        {
            int shapefulBodyCount = 0;
            for (int i = 0; i < bodySet.Count; ++i)
            {
                ref var collidable = ref bodySet.Collidables[i];
                if (collidable.Shape.Exists)
                {
                    Debug.Assert(collidable.BroadPhaseIndex >= 0 && collidable.BroadPhaseIndex < tree.LeafCount);
                    ref var leaf = ref leaves[collidable.BroadPhaseIndex];
                    Debug.Assert(leaf.StaticHandle.Value == bodySet.IndexToHandle[i].Value);
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
                ref var collidable = ref Statics[i];
                Debug.Assert(collidable.Shape.Exists, "All static collidables must have shapes. That's their only purpose.");

                Debug.Assert(collidable.BroadPhaseIndex >= 0 && collidable.BroadPhaseIndex < BroadPhase.StaticTree.LeafCount);
                ref var leaf = ref BroadPhase.staticLeaves[collidable.BroadPhaseIndex];
                Debug.Assert(leaf.StaticHandle.Value == Statics.IndexToHandle[i].Value);
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
        /// Updates the broad phase structure for the current body bounding boxes, finds potentially colliding pairs, and then executes the narrow phase for all such pairs. Generates contact constraints for the solver.
        /// </summary>
        /// <param name="dt">Duration of the time step.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void Solve(float dt, IThreadDispatcher threadDispatcher = null)
        {
            Profiler.Start(Solver);
            var constrainedBodySet = Solver.PrepareConstraintIntegrationResponsibilities(threadDispatcher);
            Solver.Solve(dt, threadDispatcher);
            Profiler.End(Solver);

            Profiler.Start(PoseIntegrator);
            PoseIntegrator.IntegrateAfterSubstepping(constrainedBodySet, dt, Solver.SubstepCount, threadDispatcher);
            Profiler.End(PoseIntegrator);

            Solver.DisposeConstraintIntegrationResponsibilities();
        }

        /// <summary>
        /// Incrementally improves body and constraint storage for better performance.
        /// </summary>
        /// <param name="threadDispatcher">Thread dispatcher to use for execution, if any.</param>
        public void IncrementallyOptimizeDataStructures(IThreadDispatcher threadDispatcher = null)
        {
            //Previously, this handled body and constraint memory layout optimization. 2.4 significantly changed how memory accesses work in the solver
            //and the optimizers were no longer net wins, so all that's left is the batch compressor.
            //It pulls constraints currently living in high constraint batch indices to lower constraint batches if possible.
            //Over time, that'll tend to reduce sync points in the solver and improve performance.
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
            if (dt <= 0)
                throw new ArgumentException("Timestep duration must be positive.", nameof(dt));
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
