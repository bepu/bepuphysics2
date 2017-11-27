using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Orchestrates the bookkeeping and execution of a full dynamic simulation.
    /// </summary>
    public partial class Simulation : IDisposable
    {
        public IslandActivator Activator { get; private set; }
        public Deactivator Deactivator { get; private set; }
        public Bodies Bodies { get; private set; }
        public Statics Statics { get; private set; }
        public Shapes Shapes { get; private set; }
        public BodyLayoutOptimizer BodyLayoutOptimizer { get; private set; }
        public ConstraintLayoutOptimizer ConstraintLayoutOptimizer { get; private set; }
        public BatchCompressor SolverBatchCompressor { get; private set; }
        public Solver Solver { get; private set; }
        public PoseIntegrator PoseIntegrator { get; private set; }
        public BroadPhase BroadPhase { get; private set; }
        public CollidableOverlapFinder BroadPhaseOverlapFinder { get; private set; }
        public NarrowPhase NarrowPhase { get; private set; }

        //Helpers shared across at least two stages.
        internal ConstraintRemover constraintRemover;

        /// <summary>
        /// Gets the main memory pool used to fill persistent structures and main thread ephemeral resources across the engine.
        /// </summary>
        public BufferPool BufferPool { get; private set; }

        /// <summary>
        /// Gets or sets whether to use a deterministic time step when using multithreading. When set to true, additional time is spent sorting constraint additions and transfers.
        /// Note that this can only affect determinism locally- different processor architectures may implement instructions differently.
        /// </summary>
        public bool Deterministic { get; set; }

        protected Simulation(BufferPool bufferPool, SimulationAllocationSizes initialAllocationSizes)
        {
            BufferPool = bufferPool;
            Shapes = new Shapes(bufferPool, initialAllocationSizes.ShapesPerType);
            BroadPhase = new BroadPhase(bufferPool, initialAllocationSizes.Bodies, initialAllocationSizes.Bodies + initialAllocationSizes.Statics);
            Activator = new IslandActivator();
            Bodies = new Bodies(bufferPool, Activator, Shapes, BroadPhase,
                initialAllocationSizes.Bodies,
                initialAllocationSizes.Islands, 
                initialAllocationSizes.ConstraintCountPerBodyEstimate);
            Statics = new Statics(bufferPool, Shapes, Bodies, BroadPhase, Activator, initialAllocationSizes.Statics);
            
            Solver = new Solver(Bodies, BufferPool, 8,
                initialCapacity: initialAllocationSizes.Constraints,
                initialIslandCapacity: initialAllocationSizes.Islands,
                minimumCapacityPerTypeBatch: initialAllocationSizes.ConstraintsPerTypeBatch);
            Bodies.Initialize(Solver);
            constraintRemover = new ConstraintRemover(BufferPool, Bodies, Solver);
            Deactivator = new Deactivator(Bodies, Solver, BroadPhase, constraintRemover, BufferPool);
            PoseIntegrator = new PoseIntegrator(Bodies, Shapes, BroadPhase);
            SolverBatchCompressor = new BatchCompressor(Solver, Bodies);
            BodyLayoutOptimizer = new BodyLayoutOptimizer(Bodies, BroadPhase, Solver, bufferPool);
            ConstraintLayoutOptimizer = new ConstraintLayoutOptimizer(Bodies, Solver);

        }

        /// <summary>
        /// Constructs a simulation supporting dynamic movement and constraints with the specified narrow phase callbacks.
        /// </summary>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        /// <param name="narrowPhaseCallbacks">Callbacks to use in the narrow phase.</param>
        /// <param name="initialAllocationSizes">Allocation sizes to initialize the simulation with. If left null, default values are chosen.</param>
        /// <returns>New simulation.</returns>
        public static Simulation Create<TNarrowPhaseCallbacks>(BufferPool bufferPool, TNarrowPhaseCallbacks narrowPhaseCallbacks,
            SimulationAllocationSizes? initialAllocationSizes = null)
            where TNarrowPhaseCallbacks : struct, INarrowPhaseCallbacks
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

            var simulation = new Simulation(bufferPool, initialAllocationSizes.Value);
            DefaultTypes.Register(simulation.Solver, out var defaultTaskRegistry);
            var narrowPhase = new NarrowPhase<TNarrowPhaseCallbacks>(simulation, defaultTaskRegistry, narrowPhaseCallbacks, initialAllocationSizes.Value.Islands + 1);
            simulation.NarrowPhase = narrowPhase;
            simulation.Deactivator.pairCache = narrowPhase.PairCache;
            simulation.BroadPhaseOverlapFinder = new CollidableOverlapFinder<TNarrowPhaseCallbacks>(narrowPhase, simulation.BroadPhase);

            return simulation;
        }

      
        //     CONSTRAINTS
        //TODO: Push this into the Solver, like we did with Bodies and Statics.

        /// <summary>
        /// Allocates a constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandles">First body handle in a list of body handles used by the constraint.</param>
        /// <param name="bodyCount">Number of bodies used by the constraint.</param>
        /// <returns>Allocated constraint handle.</returns>
        public int Add<TDescription>(ref int bodyHandles, int bodyCount, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            Solver.Add(ref bodyHandles, bodyCount, ref description, out int constraintHandle);
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyHandle = Unsafe.Add(ref bodyHandles, i);
                Bodies.ValidateExistingHandle(bodyHandle);
                Bodies.AddConstraint(Bodies.HandleToLocation[bodyHandle].Index, constraintHandle, i);
            }
            return constraintHandle;
        }

        /// <summary>
        /// Allocates a two-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandleA">First body of the pair.</param>
        /// <param name="bodyHandleB">Second body of the pair.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe int Add<TDescription>(int bodyHandleA, int bodyHandleB, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            //Don't really want to take a dependency on the stack layout of parameters, so...
            var bodyReferences = stackalloc int[2];
            bodyReferences[0] = bodyHandleA;
            bodyReferences[1] = bodyHandleB;
            return Add(ref bodyReferences[0], 2, ref description);
        }


        public void RemoveConstraint(int constraintHandle)
        {
            ConstraintGraphRemovalEnumerator enumerator;
            enumerator.bodies = Bodies;
            enumerator.constraintHandle = constraintHandle;
            Solver.EnumerateConnectedBodies(constraintHandle, ref enumerator);
            Solver.Remove(constraintHandle);
        }

        //TODO: I wonder if people will abuse the dt-as-parameter to the point where we should make it a field instead, like it effectively was in v1.
        /// <summary>
        /// Performs one timestep of the given length.
        /// </summary>
        /// <remarks>
        /// Be wary of variable timesteps. They can harm stability. Whenever possible, keep the timestep the same across multiple frames unless you have a specific reason not to.
        /// </remarks>
        /// <param name="dt">Duration of the time step in time.</param>
        public void Timestep(float dt, IThreadDispatcher threadDispatcher = null)
        {
            ProfilerClear();
            ProfilerStart(this);
            //Note that the first behavior-affecting stage is actually the pose integrator. This is a shift from v1, where collision detection went first.
            //This is a tradeoff:
            //1) Any externally set velocities will be integrated without input from the solver. The v1-style external velocity control won't work as well-
            //the user would instead have to change velocities after the pose integrator runs. This isn't perfect either, since the pose integrator is also responsible
            //for updating the bounding boxes used for collision detection.
            //2) By bundling bounding box calculation with pose integration, you avoid redundant pose and velocity memory accesses.
            //3) Generated contact positions are in sync with the integrated poses. 
            //That's often helpful for gameplay purposes- you don't have to reinterpret contact data when creating graphical effects or positioning sound sources.

            //TODO: This is something that is possibly worth exposing as one of the generic type parameters. Users could just choose the order arbitrarily.
            //Or, since you're talking about something that happens once per frame instead of once per collision pair, just provide a simple callback.
            //(Or maybe an enum even?)
            //#1 is a difficult problem, though. There is no fully 'correct' place to change velocities. We might just have to bite the bullet and create a
            //inertia tensor/bounding box update separate from pose integration. If the cache gets evicted in between (virtually guaranteed unless no stages run),
            //this basically means an extra 100-200 microseconds per frame on a processor with ~20GBps bandwidth simulating 32768 bodies.

            //Note that the reason why the pose integrator comes first instead of, say, the solver, is that the solver relies on world space inertias calculated by the pose integration.
            //If the pose integrator doesn't run first, we either need 
            //1) complicated on demand updates of world inertia when objects are added or local inertias are changed or 
            //2) local->world inertia calculation before the solver.        

            ProfilerStart(PoseIntegrator);
            PoseIntegrator.Update(dt, BufferPool, threadDispatcher);
            ProfilerEnd(PoseIntegrator);

            //Note that the deactivator comes *after* velocity integration. That looks a little weird, but it's for a reason:
            //When the narrow phase activates a bunch of objects in a pile, their accumulated impulses will represent all forces acting on them at the time of deactivation.
            //That includes gravity. If we deactivate objects *before* gravity is applied in a given frame, then when those bodies are activated, the accumulated impulses
            //will be less accurate because they assume that gravity has already been applied. This can cause a small bump.
            //So instead, velocity integration (and deactivation candidacy management) comes before deactivation.
            ProfilerStart(Deactivator);
            Deactivator.Update(threadDispatcher, Deterministic);
            ProfilerEnd(Deactivator);

            ProfilerStart(BroadPhase);
            BroadPhase.Update(threadDispatcher);
            ProfilerEnd(BroadPhase);

            ProfilerStart(BroadPhaseOverlapFinder);
            BroadPhaseOverlapFinder.DispatchOverlaps(threadDispatcher);
            ProfilerEnd(BroadPhaseOverlapFinder);

            ProfilerStart(NarrowPhase);
            NarrowPhase.Flush(threadDispatcher, threadDispatcher != null && Deterministic);
            ProfilerEnd(NarrowPhase);

            ProfilerStart(Solver);
            if (threadDispatcher == null)
                Solver.Update(dt);
            else
                Solver.MultithreadedUpdate(threadDispatcher, BufferPool, dt);
            ProfilerEnd(Solver);

            //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
            //TODO: The order of these optimizer stages is performance relevant, even though they don't have any effect on correctness.
            //You may want to try them in different locations to see how they impact cache residency.
            ProfilerStart(BodyLayoutOptimizer);
            if (threadDispatcher == null)
                BodyLayoutOptimizer.IncrementalOptimize();
            else
                BodyLayoutOptimizer.IncrementalOptimize(BufferPool, threadDispatcher);
            ProfilerEnd(BodyLayoutOptimizer);

            ProfilerStart(ConstraintLayoutOptimizer);
            ConstraintLayoutOptimizer.Update(BufferPool, threadDispatcher);
            ProfilerEnd(ConstraintLayoutOptimizer);

            ProfilerStart(SolverBatchCompressor);
            SolverBatchCompressor.Compress(BufferPool, threadDispatcher, threadDispatcher != null && Deterministic);
            ProfilerEnd(SolverBatchCompressor);

            ProfilerEnd(this);
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
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.EnsureCapacity(allocationTarget.Bodies);
            Bodies.MinimumConstraintCapacityPerBody = allocationTarget.ConstraintCountPerBodyEstimate;
            Bodies.EnsureConstraintListCapacities();
            Deactivator.EnsureSetsCapacity(allocationTarget.Islands + 1);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);
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
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.Resize(allocationTarget.Bodies);
            Bodies.MinimumConstraintCapacityPerBody = allocationTarget.ConstraintCountPerBodyEstimate;
            Bodies.ResizeConstraintListCapacities();
            Deactivator.ResizeSetsCapacity(allocationTarget.Islands + 1);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);
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
            Activator.Dispose();
            Deactivator.Dispose();
            Solver.Dispose();
            BroadPhase.Dispose();
            NarrowPhase.Dispose();
            Bodies.Dispose();
            Statics.Dispose();
            BodyLayoutOptimizer.Dispose(BufferPool);
            Shapes.Dispose();
        }
    }
}
