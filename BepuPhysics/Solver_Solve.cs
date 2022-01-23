using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Runtime.Intrinsics.X86;
using System.Numerics;
using System.Runtime.Intrinsics;
using static BepuPhysics.Solver;
using BepuPhysics.CollisionDetection;

namespace BepuPhysics
{
    public partial class Solver
    {
        protected enum SolverStageType
        {
            IncrementalUpdate,
            IntegrateConstrainedKinematics,
            WarmStart,
            Solve,
        }

        protected struct SolverSyncStage
        {
            public Buffer<int> Claims;
            public int BatchIndex;
            public int WorkBlockStartIndex;
            public SolverStageType StageType;

            public SolverSyncStage(Buffer<int> claims, int workBlockStartIndex, SolverStageType type, int batchIndex = -1)
            {
                Claims = claims;
                BatchIndex = batchIndex;
                WorkBlockStartIndex = workBlockStartIndex;
                StageType = type;
            }
        }

        protected struct WorkBlock
        {
            public int BatchIndex;
            public int TypeBatchIndex;
            /// <summary>
            /// Index of the first bundle in the block.
            /// </summary>
            public int StartBundle;
            /// <summary>
            /// Exlusive end index of the bundle. Index of the last bundle in the block is End - 1.
            /// </summary>
            public int End;
        }

        protected struct IntegrationWorkBlock
        {
            public int StartBundleIndex;
            public int EndBundleIndex;
        }

        //This is up in Solver, instead of Solver<T>, due to explicit layout.
        [StructLayout(LayoutKind.Explicit)]
        protected struct SubstepMultithreadingContext
        {
            [FieldOffset(0)]
            public Buffer<SolverSyncStage> Stages;
            [FieldOffset(16)]
            public Buffer<WorkBlock> IncrementalUpdateBlocks;
            [FieldOffset(32)]
            public Buffer<IntegrationWorkBlock> KinematicIntegrationBlocks;
            [FieldOffset(48)]
            public Buffer<WorkBlock> ConstraintBlocks;
            [FieldOffset(64)]
            public Buffer<int> ConstraintBatchBoundaries;
            [FieldOffset(80)]
            public float Dt;
            [FieldOffset(84)]
            public float InverseDt;
            [FieldOffset(88)]
            public int WorkerCount;
            [FieldOffset(92)]
            public int HighestVelocityIterationCount;
            [FieldOffset(96)]
            public Buffer<int> VelocityIterationCounts;


            //This index is written during multithreaded execution; don't want to infest any of the more frequently read properties, so it's shoved out of any dangerous cache line.
            /// <summary>
            /// Monotonically increasing index of executed stages during a frame.
            /// </summary>
            [FieldOffset(256)]
            public int SyncIndex;

            /// <summary>
            /// Counter of work completed for the current stage.
            /// </summary>
            [FieldOffset(384)]
            public int CompletedWorkBlockCount;


        }

        public abstract IndexSet PrepareConstraintIntegrationResponsibilities(IThreadDispatcher threadDispatcher = null);
        public abstract void DisposeConstraintIntegrationResponsibilities();
        public abstract void Solve(float dt, IThreadDispatcher threadDispatcher = null);
    }


    /// <summary>
    /// Handles integration-aware substepped solving.
    /// </summary>
    /// <typeparam name="TIntegrationCallbacks">Type of integration callbacks being used during the substepped solve.</typeparam>
    public class Solver<TIntegrationCallbacks> : Solver where TIntegrationCallbacks : struct, IPoseIntegratorCallbacks
    {
        /*
        There are two significant sources of complexity here:
        1. The solver takes substeps, which means velocity/pose integration must be embedded into the solving process.
        2. There are many sync points, and thread dispatches must manage these in a low overhead way.
        
        Looking at the single threaded implementation at the very bottom would be helpful for understanding the general flow of execution.

        In order to reduce overall dispatch count and to share memory loads as much as possible, the first execution of a constraint affecting a body is responsible for that body's integration.
        There are some special cases around this- kinematics must be handled in a separate prepass, since kinematics can appear in a given constraint batch more than once.
        Unconstrained bodies will be integrated separately outside of the solver.

        To reduce sync point overhead, worker threads do not enter blocking states. The main orchestrator thread never even yields- it only spins.
        The main thread kicks off jobs to all available workers. If a worker has yielded previously, it may miss waking up, but that will not block the execution of other threads.
        Work is scheduled such that the same thread operates on the same data each iteration/substep, if possible. This makes it more likely that a core will find relevant data in its local caches.
        There is a tradeoff with workstealing- to keep overhead low while maintaining this consistent scheduling, threads only look for incrementally adjacent blocks. This can sometimes result in imbalanced workloads.
        (This will likely need to be updated to be cleverer as heterogeneous architectures gain popularity.)
        */

        public Solver(Bodies bodies, BufferPool pool, SolveDescription solveDescription,
            int initialCapacity,
            int initialIslandCapacity,
            int minimumCapacityPerTypeBatch, PoseIntegrator<TIntegrationCallbacks> poseIntegrator)
            : base(bodies, pool, solveDescription, initialCapacity, initialIslandCapacity, minimumCapacityPerTypeBatch)
        {
            PoseIntegrator = poseIntegrator;
            solveWorker = SolveWorker;
            constraintIntegrationResponsibilitiesWorker = ConstraintIntegrationResponsibilitiesWorker;
        }

        /// <summary>
        /// Pose integrator used by the simulation.
        /// </summary>
        public PoseIntegrator<TIntegrationCallbacks> PoseIntegrator { get; private set; }
        protected interface ITypeBatchSolveFilter
        {
            bool IncludeFallbackBatchForWorkBlocks { get; }
            bool AllowType(int typeId);
        }
        protected struct IncrementalUpdateForSubstepFilter : ITypeBatchSolveFilter
        {
            public TypeProcessor[] TypeProcessors;
            public bool IncludeFallbackBatchForWorkBlocks { get { return true; } }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowType(int typeId)
            {
                return TypeProcessors[typeId].RequiresIncrementalSubstepUpdates;
            }
        }
        protected struct MainSolveFilter : ITypeBatchSolveFilter
        {
            public bool IncludeFallbackBatchForWorkBlocks
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    return false;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowType(int typeId)
            {
                return true;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void WarmStartBlock<TBatchShouldIntegratePoses>(int workerIndex, int batchIndex, int typeBatchIndex, int startBundle, int endBundle, ref TypeBatch typeBatch, TypeProcessor typeProcessor, float dt, float inverseDt)
            where TBatchShouldIntegratePoses : unmanaged, IBatchPoseIntegrationAllowed
        {
            if (batchIndex == 0)
            {
                Buffer<IndexSet> noFlagsRequired = default;
                typeProcessor.WarmStart<TIntegrationCallbacks, BatchShouldAlwaysIntegrate, TBatchShouldIntegratePoses>(
                    ref typeBatch, ref noFlagsRequired, bodies, ref PoseIntegrator.Callbacks,
                    dt, inverseDt, startBundle, endBundle, workerIndex);
            }
            else
            {
                if (coarseBatchIntegrationResponsibilities[batchIndex][typeBatchIndex])
                {
                    typeProcessor.WarmStart<TIntegrationCallbacks, BatchShouldConditionallyIntegrate, TBatchShouldIntegratePoses>(
                        ref typeBatch, ref integrationFlags[batchIndex][typeBatchIndex], bodies, ref PoseIntegrator.Callbacks,
                        dt, inverseDt, startBundle, endBundle, workerIndex);
                }
                else
                {
                    typeProcessor.WarmStart<TIntegrationCallbacks, BatchShouldNeverIntegrate, TBatchShouldIntegratePoses>(
                        ref typeBatch, ref integrationFlags[batchIndex][typeBatchIndex], bodies, ref PoseIntegrator.Callbacks,
                        dt, inverseDt, startBundle, endBundle, workerIndex);
                }
            }
        }
        protected interface IStageFunction
        {
            void Execute(Solver solver, int blockIndex, int workerIndex);
        }

        //Split the solve process into a warmstart and solve, where warmstart doesn't try to store out anything. It just computes jacobians and modifies velocities according to the accumulated impulse.
        //The solve step then *recomputes* jacobians from prestep data and pose information.
        //Why? Memory bandwidth. Redoing the calculation is cheaper than storing it out.
        struct WarmStartStageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;
            public int SubstepIndex;
            public Solver<TIntegrationCallbacks> solver;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex, int workerIndex)
            {
                ref var block = ref this.solver.substepContext.ConstraintBlocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                if (SubstepIndex == 0)
                {
                    this.solver.WarmStartBlock<DisallowPoseIntegration>(workerIndex, block.BatchIndex, block.TypeBatchIndex, block.StartBundle, block.End, ref typeBatch, typeProcessor, Dt, InverseDt);
                }
                else
                {
                    this.solver.WarmStartBlock<AllowPoseIntegration>(workerIndex, block.BatchIndex, block.TypeBatchIndex, block.StartBundle, block.End, ref typeBatch, typeProcessor, Dt, InverseDt);
                }

            }
        }

        struct SolveStageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;
            public Solver<TIntegrationCallbacks> solver;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex, int workerIndex)
            {
                ref var block = ref this.solver.substepContext.ConstraintBlocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                typeProcessor.Solve(ref typeBatch, solver.bodies, Dt, InverseDt, block.StartBundle, block.End);
            }
        }

        struct IncrementalUpdateStageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;
            public Solver<TIntegrationCallbacks> solver;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex, int workerIndex)
            {
                ref var block = ref this.solver.substepContext.IncrementalUpdateBlocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                solver.TypeProcessors[typeBatch.TypeId].IncrementallyUpdateForSubstep(ref typeBatch, solver.bodies, Dt, InverseDt, block.StartBundle, block.End);
            }
        }

        struct IntegrateConstrainedKinematicsStageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;
            public int SubstepIndex;
            public Solver<TIntegrationCallbacks> solver;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex, int workerIndex)
            {
                ref var block = ref this.solver.substepContext.KinematicIntegrationBlocks[blockIndex];
                if (SubstepIndex == 0)
                {
                    this.solver.PoseIntegrator.IntegrateKinematicVelocities(solver.ConstrainedKinematicHandles.Span.Slice(solver.ConstrainedKinematicHandles.Count), block.StartBundleIndex, block.EndBundleIndex, Dt, workerIndex);
                }
                else
                {
                    this.solver.PoseIntegrator.IntegrateKinematicPosesAndVelocities(solver.ConstrainedKinematicHandles.Span.Slice(solver.ConstrainedKinematicHandles.Count), block.StartBundleIndex, block.EndBundleIndex, Dt, workerIndex);
                }
            }
        }

        unsafe void ExecuteWorkerStage<TStageFunction>(ref TStageFunction stageFunction, int workerIndex, int workerStart, int availableBlocksStartIndex, ref Buffer<int> claims, int previousSyncIndex, int syncIndex, ref int completedWorkBlocks) where TStageFunction : IStageFunction
        {
            if (workerStart == -1)
            {
                //Thread count exceeds work block count; nothing for this worker to do.
                //(Technically, there's a possibility that an earlier thread would fail to wake and allowing this thread to steal that block COULD help,
                //but on average it's better to keep jobs scheduled to the same core to avoid excess memory traffic, and we can rely on some other active worker to take care of it.)
                return;
            }
            int workBlockIndex = workerStart;
            int locallyCompletedCount = 0;
            //Try to claim blocks by traversing forward until we're blocked by another claim.
            while (Interlocked.CompareExchange(ref claims[workBlockIndex], syncIndex, previousSyncIndex) == previousSyncIndex)
            {
                //Successfully claimed a work block.
                stageFunction.Execute(this, availableBlocksStartIndex + workBlockIndex, workerIndex);
                ++locallyCompletedCount;
                ++workBlockIndex;
                if (workBlockIndex >= claims.Length)
                {
                    //Wrap around.
                    workBlockIndex = 0;
                }
            }
            //Try to claim work blocks going backward.
            workBlockIndex = workerStart - 1;
            while (true)
            {
                if (workBlockIndex < 0)
                {
                    //Wrap around.
                    workBlockIndex = claims.Length - 1;
                }
                //Note the comparison: equal *or greater* blocks.
                //Consider what happens if this thread was heavily delayed and the stage it was dispatched for has already ended.
                //Other threads could be working on the next sync index. A mere equality test could result in this thread thinking there's work to be done, so it starts claiming for an *earlier* stage.
                //Then everything dies.
                if (Interlocked.CompareExchange(ref claims[workBlockIndex], syncIndex, previousSyncIndex) != previousSyncIndex)
                {
                    break;
                }
                //Successfully claimed a work block.
                stageFunction.Execute(this, availableBlocksStartIndex + workBlockIndex, workerIndex);
                ++locallyCompletedCount;
                workBlockIndex--;
            }
            //No more adjacent work blocks are available. This thread is done!
            Interlocked.Add(ref completedWorkBlocks, locallyCompletedCount);


            //debugStageWorkBlocksCompleted[syncIndex - 1][workerIndex] = locallyCompletedCount;
            //if (workerIndex == 3)
            //{
            //Console.WriteLine($"Worker {workerIndex}, stage {typeof(TStageFunction).Name}, sync index {syncIndex} completed {locallyCompletedCount / (double)claims.Length:G2} ({locallyCompletedCount} of {claims.Length}).");
            //}
            //for (int i = 0; i < claims.Length; ++i)
            //{
            //    if (claims[i] != syncIndex)
            //    {
            //        Console.WriteLine($"Failed to claim index {i}, claim value is {claims[i]} instead of {syncIndex}, previous claim should have been {previousSyncIndex}, worker start {workerStart}");
            //    }
            //}

        }
        void ExecuteMainStage<TStageFunction>(ref TStageFunction stageFunction, int workerIndex, int workerStart, ref SolverSyncStage stage, int previousSyncIndex, int syncIndex) where TStageFunction : IStageFunction
        {
            var availableBlocksCount = stage.Claims.Length;
            if (availableBlocksCount == 0)
                return;

            //for (int i = 0; i < availableBlocksCount; ++i)
            //{
            //    stageFunction.Execute(this, stage.WorkBlockStartIndex + i, workerIndex);
            //}
            //return;
            //Console.WriteLine($"Main executing {typeof(TStageFunction).Name} for sync index {syncIndex}, expected claim {syncIndex - previousSyncIndexOffset}");
            if (availableBlocksCount == 1)
            {
                //Console.WriteLine($"Main thread is executing {syncIndex} by itself; stage function: {stageFunction.GetType().Name}");
                //There is only one work block available. There's no reason to notify other threads about it or do any claims management; just execute it sequentially.
                stageFunction.Execute(this, stage.WorkBlockStartIndex, workerIndex);
            }
            else
            {
                //Console.WriteLine($"Main thread is requesting workers begin for sync index {syncIndex}; stage function: {stageFunction.GetType().Name}");
                //Write the new stage index so other spinning threads will begin work on it.
                Volatile.Write(ref substepContext.SyncIndex, syncIndex);
                ExecuteWorkerStage(ref stageFunction, workerIndex, workerStart, stage.WorkBlockStartIndex, ref stage.Claims, previousSyncIndex, syncIndex, ref substepContext.CompletedWorkBlockCount);

                //Since we asked other threads to do work, we must wait until the requested work is done before proceeding.
                //Note that we DO NOT yield on the main thread! 
                //This significantly increases the chance *some* progress will be made on the available work, even if all other workers are stuck unscheduled.
                //The reasoning here is that the OS is not likely to unschedule an active thread, but will be far less aggressive about scheduling a *currently unscheduled* thread.
                //Critically, yielding threads are not in any kind of execution queue- from the OS's perspective, they aren't asking to be woken up.
                //If another thread comes in with significant work, they could be stalled for (from the solver's perspective) an arbitrarily long time.
                //By having the main thread never yield, the only way for all progress to halt is for the OS to aggressively unschedule the main thread.
                //That is very rare when dealing with CPUs with plenty of cores to go around relative to the scheduled work.                
                //(Why not notify the OS that waiting threads actually want to be executed? Just overhead. Feel free to experiment with different approaches, but so far this has won empirically.)
                while (Volatile.Read(ref substepContext.CompletedWorkBlockCount) != availableBlocksCount)
                {
                    Thread.SpinWait(3);
                }
                //Console.WriteLine($"Completed blocks count: {substepContext.CompletedWorkBlockCount}.");
                //All workers are done. We can safely reset the counter for the next time this stage is used.
                substepContext.CompletedWorkBlockCount = 0;
            }
        }
        protected SubstepMultithreadingContext substepContext;



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int GetPreviousSyncIndexForIncrementalUpdate(int substepIndex, int syncIndex, int syncStagesPerSubstep)
        {
            return substepIndex == 1 ? 0 : Math.Max(0, syncIndex - syncStagesPerSubstep);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int GetPreviousSyncIndexForIntegrateConstrainedKinematics(int substepIndex, int syncIndex, int syncStagesPerSubstep)
        {
            //If kinematics have their velocities integrated, then the first substep will have executed and left the claims at 1. Otherwise, the first substep will leave them cleared at 0.
            //The second substep and later will always run (since kinematics need their poses integrated regardless) so their sync index isn't weirdly conditional.
            return substepIndex == 1 ? PoseIntegrator.Callbacks.IntegrateVelocityForKinematics ? 2 : 0 : Math.Max(0, syncIndex - syncStagesPerSubstep);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int GetWarmStartLookback(int substepIndex, int synchronizedBatchCount)
        {
            //Warm start and solve share the same claims buffer, so we want to look back to the last execution of the solver.
            //Variable velocity iteration counts make this slightly tricky- we must skip over 
            //"+ 2" is just for the first two stages- incremental update and integration of constrained kinematics.
            var warmStartLookback = synchronizedBatchCount + 2;
            if (substepIndex > 0)
            {
                warmStartLookback += synchronizedBatchCount * (substepContext.HighestVelocityIterationCount - substepContext.VelocityIterationCounts[substepIndex - 1]);
            }
            return warmStartLookback;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int GetPreviousSyncIndexForWarmStart(int syncIndex, int warmStartLookback)
        {
            //The claims for warmstarts and solves are shared. So we want to look back to the last solve's claims, which would be beyond the incremental update and integrate constrained kinematics.
            return Math.Max(0, syncIndex - warmStartLookback);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int GetPreviousSyncIndexForSolve(int syncIndex, int synchronizedBatchCount)
        {
            return Math.Max(0, syncIndex - synchronizedBatchCount);
        }

        protected static int GetUniformlyDistributedStart(int workerIndex, int blockCount, int workerCount, int offset)
        {
            if (blockCount <= workerCount)
            {
                //Too few blocks to give every worker a job; give the jobs to the first context.WorkBlocks.Count workers.
                return workerIndex < blockCount ? offset + workerIndex : -1;
            }
            var blocksPerWorker = blockCount / workerCount;
            var remainder = blockCount - blocksPerWorker * workerCount;
            return offset + blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
        }

        Action<int> solveWorker;
        void SolveWorker(int workerIndex)
        {
            //The solver has two codepaths: one thread, acting as an orchestrator, and the others, just waiting to be used.
            //There is no requirement that a worker thread above index 0 actually runs at all for a given dispatch.
            //If a worker fails to schedule for a long time because the OS went with a different thread, that's perfectly fine- 
            //another thread will consume the work that would have otherwise been handled by it, and the execution as a whole
            //will continue on unimpeded.
            //There's still nothing done if the OS unschedules an active worker that claimed work, but that's a far, far rarer concern.
            //Note that this attempts to maintain a given worker's relationship to a set of work blocks. This increases the probability that 
            //data will remain in some cache that's reasonably close to the core.
            int workerCount = substepContext.WorkerCount;
            var incrementalUpdateWorkerStart = GetUniformlyDistributedStart(workerIndex, substepContext.IncrementalUpdateBlocks.Length, workerCount, 0);
            var kinematicIntegrationWorkerStart = GetUniformlyDistributedStart(workerIndex, substepContext.KinematicIntegrationBlocks.Length, workerCount, 0);
            Buffer<int> batchStarts;
            ref var activeSet = ref ActiveSet;
            unsafe
            {
                var batchStartsData = stackalloc int[activeSet.Batches.Count];
                batchStarts = new Buffer<int>(batchStartsData, activeSet.Batches.Count);
            }
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                var batchOffset = batchIndex > 0 ? substepContext.ConstraintBatchBoundaries[batchIndex - 1] : 0;
                var batchCount = substepContext.ConstraintBatchBoundaries[batchIndex] - batchOffset;
                batchStarts[batchIndex] = GetUniformlyDistributedStart(workerIndex, batchCount, workerCount, 0);
            }

            Debug.Assert(activeSet.Batches.Count > 0, "Don't dispatch if there are no constraints.");

            //TODO: Every single one of these offers up the same parameters. Could avoid the need to initialize any of them.
            var incrementalUpdateStage = new IncrementalUpdateStageFunction
            {
                Dt = substepContext.Dt,
                InverseDt = substepContext.InverseDt,
                solver = this
            };
            var integrateConstrainedKinematicsStage = new IntegrateConstrainedKinematicsStageFunction
            {
                Dt = substepContext.Dt,
                InverseDt = substepContext.InverseDt,
                solver = this
            };
            var warmstartStage = new WarmStartStageFunction
            {
                Dt = substepContext.Dt,
                InverseDt = substepContext.InverseDt,
                solver = this
            };
            var solveStage = new SolveStageFunction
            {
                Dt = substepContext.Dt,
                InverseDt = substepContext.InverseDt,
                solver = this
            };

            var maximumSyncStagesPerSubstep = 2 + synchronizedBatchCount * (1 + substepContext.HighestVelocityIterationCount);
            if (workerIndex == 0)
            {
                //This is the main 'orchestrator' thread. It tracks execution progress and notifies other threads that's it's time to work.
                for (int substepIndex = 0; substepIndex < substepCount; ++substepIndex)
                {
                    OnSubstepStarted(substepIndex);
                    //Note that variable velocity iteration counts per substep means that not every substep will exhaust the entirety of the allocated sync points.
                    //That's fine; we just need to ensure that each substep starts at a point that the worker threads can recognize is in the appropriate substep.
                    //Easiest to have a consistent size for each substep so the workers can simply divide the sync index to get the substep index.
                    //(The +1 here is just because the first dispatch expects 0 as a previous value and goes to 1, and the current sync index is what's going to be written as a claim next.)
                    int syncIndex = substepIndex * maximumSyncStagesPerSubstep + 1;
                    //Note that the main thread's view of the sync index increments every single dispatch, even if there is no work.
                    //This ensures that the workers are able to advance to the appropriate stage by examining the sync index snapshot.
                    if (substepIndex > 0)
                    {
                        ExecuteMainStage(ref incrementalUpdateStage, workerIndex, incrementalUpdateWorkerStart, ref substepContext.Stages[0], GetPreviousSyncIndexForIncrementalUpdate(substepIndex, syncIndex, maximumSyncStagesPerSubstep), syncIndex);
                    }
                    //Note that we do not invoke velocity integration on the first substep if kinematics do not need velocity integration.
                    ++syncIndex;
                    if (substepIndex > 0 || PoseIntegrator.Callbacks.IntegrateVelocityForKinematics)
                    {
                        integrateConstrainedKinematicsStage.SubstepIndex = substepIndex;
                        ExecuteMainStage(ref integrateConstrainedKinematicsStage, workerIndex, kinematicIntegrationWorkerStart, ref substepContext.Stages[1], GetPreviousSyncIndexForIntegrateConstrainedKinematics(substepIndex, syncIndex, maximumSyncStagesPerSubstep), syncIndex);
                    }
                    warmstartStage.SubstepIndex = substepIndex;
                    var warmStartLookback = GetWarmStartLookback(substepIndex, synchronizedBatchCount);
                    for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                    {
                        ++syncIndex;
                        ExecuteMainStage(ref warmstartStage, workerIndex, batchStarts[batchIndex], ref substepContext.Stages[batchIndex + 2], GetPreviousSyncIndexForWarmStart(syncIndex, warmStartLookback), syncIndex);
                    }
                    if (fallbackExists)
                    {
                        //The fallback runs only on the main thread.
                        ref var batch = ref activeSet.Batches[FallbackBatchThreshold];
                        ref var integrationFlagsForBatch = ref integrationFlags[FallbackBatchThreshold];
                        for (int j = 0; j < batch.TypeBatches.Count; ++j)
                        {
                            ref var typeBatch = ref batch.TypeBatches[j];
                            if (substepIndex == 0)
                            {
                                WarmStartBlock<DisallowPoseIntegration>(0, FallbackBatchThreshold, j, 0, typeBatch.BundleCount, ref typeBatch, TypeProcessors[typeBatch.TypeId], substepContext.Dt, substepContext.InverseDt);
                            }
                            else
                            {
                                WarmStartBlock<AllowPoseIntegration>(0, FallbackBatchThreshold, j, 0, typeBatch.BundleCount, ref typeBatch, TypeProcessors[typeBatch.TypeId], substepContext.Dt, substepContext.InverseDt);
                            }
                        }
                    }
                    var velocityIterationCountForSubstep = substepContext.VelocityIterationCounts[substepIndex];
                    for (int iterationIndex = 0; iterationIndex < velocityIterationCountForSubstep; ++iterationIndex)
                    {
                        for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                        {
                            //Note that this is using a 'different' stage by index than the worker thread if the iteration index > 1. 
                            //That's totally fine- the warmstart/iteration stages share the same claims buffers per batch. They're redundant for the sake of easier indexing.
                            ++syncIndex;
                            ExecuteMainStage(ref solveStage, workerIndex, batchStarts[batchIndex], ref substepContext.Stages[batchIndex + 2], GetPreviousSyncIndexForSolve(syncIndex, synchronizedBatchCount), syncIndex);
                        }
                        if (fallbackExists)
                        {
                            //The fallback runs only on the main thread.
                            ref var batch = ref activeSet.Batches[FallbackBatchThreshold];
                            for (int j = 0; j < batch.TypeBatches.Count; ++j)
                            {
                                ref var typeBatch = ref batch.TypeBatches[j];
                                TypeProcessors[typeBatch.TypeId].Solve(ref typeBatch, bodies, substepContext.Dt, substepContext.InverseDt, 0, typeBatch.BundleCount);
                            }
                        }
                    }
                    OnSubstepEnded(substepIndex);
                }
                //All done; notify waiting threads to join.
                Volatile.Write(ref substepContext.SyncIndex, int.MinValue);
            }
            else
            {
                //This is a worker thread. It does not need to track execution progress; it only checks to see if there's any work that needs to be done, and if there is, does it, then goes back into a wait.
                int latestCompletedSyncIndex = 0;
                int syncIndexInSubstep = -1;
                int substepIndex = 0;

                while (true)
                {
                    var spinWait = new LocalSpinWait();
                    int syncIndex;
                    while (latestCompletedSyncIndex == (syncIndex = Volatile.Read(ref substepContext.SyncIndex)))
                    {
                        //No work yet available.
                        spinWait.SpinOnce();
                    }
                    //Stages were set up prior to execution. Note that we don't attempt to ping pong buffers or anything; workblock claim indices monotonically increase across the execution of the solver.
                    //This guarantees that a worker thread can go idle and miss an arbitrary number of stages without blocking any progress.
                    if (syncIndex == int.MinValue)
                    {
                        //No more stages; exit the work loop.
                        break;
                    }
                    //Extract the job type, stage index, and substep index from the sync index.
                    var syncStepsSinceLast = syncIndex - latestCompletedSyncIndex;
                    syncIndexInSubstep += syncStepsSinceLast;
                    while (true)
                    {
                        if (syncIndexInSubstep >= maximumSyncStagesPerSubstep)
                        {
                            syncIndexInSubstep -= maximumSyncStagesPerSubstep;
                            ++substepIndex;
                        }
                        else
                        {
                            break;
                        }
                    }
                    //Console.WriteLine($"Worker working on sync index {syncIndex}, sync index in substep: {syncIndexInSubstep}");
                    //Note that we're going to do a compare exchange that prevents any claim on work blocks that *arent* of the previous sync index, which means we need the previous sync index.
                    //Storing that in a reliable way is annoying, so we derive it from syncIndex.
                    ref var stage = ref substepContext.Stages[syncIndexInSubstep];
                    //Console.WriteLine($"Worker {workerIndex} executing {stage.StageType} for sync index {syncIndex}, stage index {syncIndexInSubstep}");
                    switch (stage.StageType)
                    {
                        case SolverStageType.IncrementalUpdate:
                            ExecuteWorkerStage(ref incrementalUpdateStage, workerIndex, incrementalUpdateWorkerStart, 0, ref stage.Claims, GetPreviousSyncIndexForIncrementalUpdate(substepIndex, syncIndex, maximumSyncStagesPerSubstep), syncIndex, ref substepContext.CompletedWorkBlockCount);
                            break;
                        case SolverStageType.IntegrateConstrainedKinematics:
                            integrateConstrainedKinematicsStage.SubstepIndex = substepIndex;
                            ExecuteWorkerStage(ref integrateConstrainedKinematicsStage, workerIndex, kinematicIntegrationWorkerStart, 0, ref stage.Claims, GetPreviousSyncIndexForIntegrateConstrainedKinematics(substepIndex, syncIndex, maximumSyncStagesPerSubstep), syncIndex, ref substepContext.CompletedWorkBlockCount);
                            break;
                        case SolverStageType.WarmStart:
                            warmstartStage.SubstepIndex = substepIndex;
                            ExecuteWorkerStage(ref warmstartStage, workerIndex, batchStarts[stage.BatchIndex], stage.WorkBlockStartIndex, ref stage.Claims, GetPreviousSyncIndexForWarmStart(syncIndex, GetWarmStartLookback(substepIndex, synchronizedBatchCount)), syncIndex, ref substepContext.CompletedWorkBlockCount);
                            break;
                        case SolverStageType.Solve:
                            ExecuteWorkerStage(ref solveStage, workerIndex, batchStarts[stage.BatchIndex], stage.WorkBlockStartIndex, ref stage.Claims, GetPreviousSyncIndexForSolve(syncIndex, synchronizedBatchCount), syncIndex, ref substepContext.CompletedWorkBlockCount);
                            break;
                    }
                    latestCompletedSyncIndex = syncIndex;

                }
            }
        }
        Buffer<IntegrationWorkBlock> BuildKinematicIntegrationWorkBlocks(int minimumBlockSizeInBundles, int maximumBlockSizeInBundles, int targetBlockCount)
        {
            var bundleCount = BundleIndexing.GetBundleCount(ConstrainedKinematicHandles.Count);
            if (bundleCount > 0)
            {
                var targetBundlesPerBlock = bundleCount / targetBlockCount;
                if (targetBundlesPerBlock < minimumBlockSizeInBundles)
                    targetBundlesPerBlock = minimumBlockSizeInBundles;
                if (targetBundlesPerBlock > maximumBlockSizeInBundles)
                    targetBundlesPerBlock = maximumBlockSizeInBundles;
                var blockCount = (bundleCount + targetBundlesPerBlock - 1) / targetBundlesPerBlock;
                var bundlesPerBlock = bundleCount / blockCount;
                var remainder = bundleCount - bundlesPerBlock * blockCount;
                var previousEnd = 0;
                pool.Take(blockCount, out Buffer<IntegrationWorkBlock> workBlocks);
                for (int i = 0; i < blockCount; ++i)
                {
                    var bundleCountForBlock = bundlesPerBlock;
                    if (i < remainder)
                        ++bundleCountForBlock;
                    workBlocks[i] = new IntegrationWorkBlock { StartBundleIndex = previousEnd, EndBundleIndex = previousEnd += bundleCountForBlock };
                }
                return workBlocks;

            }
            return default;
        }

        protected unsafe void BuildWorkBlocks<TTypeBatchFilter>(
            BufferPool pool, int minimumBlockSizeInBundles, int maximumBlockSizeInBundles, int targetBlocksPerBatch, ref TTypeBatchFilter typeBatchFilter,
            out QuickList<WorkBlock> workBlocks, out Buffer<int> batchBoundaries) where TTypeBatchFilter : ITypeBatchSolveFilter
        {
            ref var activeSet = ref ActiveSet;
            int batchCount;
            if (typeBatchFilter.IncludeFallbackBatchForWorkBlocks)
            {
                batchCount = activeSet.Batches.Count;
            }
            else
            {
                GetSynchronizedBatchCount(out batchCount, out _);
            }
            workBlocks = new QuickList<WorkBlock>(targetBlocksPerBatch * batchCount, pool);
            pool.Take(batchCount, out batchBoundaries);
            var inverseMinimumBlockSizeInBundles = 1f / minimumBlockSizeInBundles;
            var inverseMaximumBlockSizeInBundles = 1f / maximumBlockSizeInBundles;
            for (int batchIndex = 0; batchIndex < batchCount; ++batchIndex)
            {
                ref var typeBatches = ref activeSet.Batches[batchIndex].TypeBatches;
                var bundleCount = 0;
                for (int typeBatchIndex = 0; typeBatchIndex < typeBatches.Count; ++typeBatchIndex)
                {
                    if (typeBatchFilter.AllowType(typeBatches[typeBatchIndex].TypeId))
                    {
                        bundleCount += typeBatches[typeBatchIndex].BundleCount;
                    }
                }
                for (int typeBatchIndex = 0; typeBatchIndex < typeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref typeBatches[typeBatchIndex];
                    if (!typeBatchFilter.AllowType(typeBatch.TypeId))
                    {
                        continue;
                    }
                    var typeBatchSizeFraction = typeBatch.BundleCount / (float)bundleCount; //note: pre-inverting this doesn't necessarily work well due to numerical issues.
                    var typeBatchMaximumBlockCount = typeBatch.BundleCount * inverseMinimumBlockSizeInBundles;
                    var typeBatchMinimumBlockCount = typeBatch.BundleCount * inverseMaximumBlockSizeInBundles;
                    var typeBatchBlockCount = Math.Max(1, (int)Math.Min(typeBatchMaximumBlockCount, Math.Max(typeBatchMinimumBlockCount, targetBlocksPerBatch * typeBatchSizeFraction)));
                    int previousEnd = 0;
                    var baseBlockSizeInBundles = typeBatch.BundleCount / typeBatchBlockCount;
                    var remainder = typeBatch.BundleCount - baseBlockSizeInBundles * typeBatchBlockCount;
                    for (int newBlockIndex = 0; newBlockIndex < typeBatchBlockCount; ++newBlockIndex)
                    {
                        ref var block = ref workBlocks.Allocate(pool);
                        var blockBundleCount = newBlockIndex < remainder ? baseBlockSizeInBundles + 1 : baseBlockSizeInBundles;
                        block.BatchIndex = batchIndex;
                        block.TypeBatchIndex = typeBatchIndex;
                        block.StartBundle = previousEnd;
                        block.End = previousEnd + blockBundleCount;
                        previousEnd = block.End;
                        Debug.Assert(block.StartBundle >= 0 && block.StartBundle < typeBatch.BundleCount);
                        Debug.Assert(block.End >= block.StartBundle + Math.Min(minimumBlockSizeInBundles, typeBatch.BundleCount) && block.End <= typeBatch.BundleCount);
                    }
                }
                batchBoundaries[batchIndex] = workBlocks.Count;
            }
        }

        int GetVelocityIterationCountForSubstepIndex(int substepIndex)
        {
            if (VelocityIterationScheduler != null)
            {
                var scheduledCount = VelocityIterationScheduler(substepIndex);
                return scheduledCount < 1 ? VelocityIterationCount : scheduledCount;
            }
            return VelocityIterationCount;
        }
        //Buffer<Buffer<int>> debugStageWorkBlocksCompleted;
        protected void ExecuteMultithreaded(float dt, IThreadDispatcher threadDispatcher, Action<int> workDelegate)
        {
            var workerCount = substepContext.WorkerCount = threadDispatcher.ThreadCount;
            substepContext.Dt = dt;
            substepContext.InverseDt = 1f / dt;
            pool.Take(substepCount, out substepContext.VelocityIterationCounts);
            //Each substep can have a different number of velocity iterations.
            if (VelocityIterationScheduler == null)
            {
                for (int i = 0; i < substepCount; ++i)
                {
                    substepContext.VelocityIterationCounts[i] = VelocityIterationCount;
                }
            }
            else
            {
                for (int i = 0; i < substepCount; ++i)
                {
                    substepContext.VelocityIterationCounts[i] = GetVelocityIterationCountForSubstepIndex(i);
                }
            }

            //First build a set of work blocks.
            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
            //These values are found by empirical tuning. The optimal values may vary by architecture.
            //The goal here is to have just enough blocks that, in the event that we end up some underpowered threads (due to competition or hyperthreading), 
            //there are enough blocks that workstealing will still generally allow the extra threads to be useful.
            const int targetBlocksPerBatchPerWorker = 4;
            const int minimumBlockSizeInBundles = 1;
            const int maximumBlockSizeInBundles = 1024;

            var targetBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
            var mainFilter = new MainSolveFilter();
            var incrementalUpdateFilter = new IncrementalUpdateForSubstepFilter { TypeProcessors = TypeProcessors };
            BuildWorkBlocks(pool, minimumBlockSizeInBundles, maximumBlockSizeInBundles, targetBlocksPerBatch, ref mainFilter, out var constraintBlocks, out substepContext.ConstraintBatchBoundaries);
            BuildWorkBlocks(pool, minimumBlockSizeInBundles, maximumBlockSizeInBundles, targetBlocksPerBatch, ref incrementalUpdateFilter, out var incrementalBlocks, out var incrementalUpdateBatchBoundaries);
            pool.Return(ref incrementalUpdateBatchBoundaries); //TODO: No need to create this in the first place. Doesn't really cost anything, but...
            substepContext.ConstraintBlocks = constraintBlocks.Span.Slice(constraintBlocks.Count);
            substepContext.IncrementalUpdateBlocks = incrementalBlocks.Span.Slice(incrementalBlocks.Count);
            substepContext.KinematicIntegrationBlocks = BuildKinematicIntegrationWorkBlocks(minimumBlockSizeInBundles, maximumBlockSizeInBundles, targetBlocksPerBatch);

            //Not every batch will actually have work blocks associated with it; the batch compressor could be falling behind, which means older constraints could be at higher batches than they need to be, leaving gaps.
            //We don't want to include those empty batches as sync points in the solver.
            var batchCount = ActiveSet.Batches.Count;
            substepContext.SyncIndex = 0;
            var totalConstraintBatchWorkBlockCount = substepContext.ConstraintBatchBoundaries.Length == 0 ? 0 : substepContext.ConstraintBatchBoundaries[^1];
            var totalClaimCount = incrementalBlocks.Count + substepContext.KinematicIntegrationBlocks.Length + totalConstraintBatchWorkBlockCount;
            GetSynchronizedBatchCount(out var stagesPerIteration, out var fallbackExists);
            substepContext.HighestVelocityIterationCount = 0;
            for (int i = 0; i < substepContext.VelocityIterationCounts.Length; ++i)
            {
                substepContext.HighestVelocityIterationCount = Math.Max(substepContext.VelocityIterationCounts[i], substepContext.HighestVelocityIterationCount);
            }
            pool.Take(2 + stagesPerIteration * (1 + substepContext.HighestVelocityIterationCount), out substepContext.Stages);
            //Claims will be monotonically increasing throughout execution. All should start at zero to match with the initial sync index.
            pool.Take<int>(totalClaimCount, out var claims);
            claims.Clear(0, claims.Length);
            substepContext.Stages[0] = new(claims.Slice(incrementalBlocks.Count), 0, SolverStageType.IncrementalUpdate);
            substepContext.Stages[1] = new(claims.Slice(incrementalBlocks.Count, substepContext.KinematicIntegrationBlocks.Length), 0, SolverStageType.IntegrateConstrainedKinematics);
            //Note that we create redundant stages that share the same workblock targets and claims buffers.
            //This is just to make indexing a little simpler during the multithreaded work.
            int targetStageIndex = 2;
            //Warm start.
            var preambleClaimCount = incrementalBlocks.Count + substepContext.KinematicIntegrationBlocks.Length;
            int claimStart = preambleClaimCount;
            int highestJobCountInSolve = 0;
            for (int batchIndex = 0; batchIndex < stagesPerIteration; ++batchIndex)
            {
                var stageIndex = targetStageIndex++;
                var batchStart = batchIndex == 0 ? 0 : substepContext.ConstraintBatchBoundaries[batchIndex - 1];
                var workBlocksInBatch = substepContext.ConstraintBatchBoundaries[batchIndex] - batchStart;
                substepContext.Stages[stageIndex] = new(claims.Slice(claimStart, workBlocksInBatch), batchStart, SolverStageType.WarmStart, batchIndex);
                claimStart += workBlocksInBatch;
                highestJobCountInSolve = Math.Max(highestJobCountInSolve, workBlocksInBatch);
            }
            for (int iterationIndex = 0; iterationIndex < substepContext.HighestVelocityIterationCount; ++iterationIndex)
            {
                //Solve. Note that we're reusing the same claims as were used in the warm start for these stages; the stages just tell the workers what kind of work to do.
                claimStart = preambleClaimCount;
                for (int batchIndex = 0; batchIndex < stagesPerIteration; ++batchIndex)
                {
                    var stageIndex = targetStageIndex++;
                    var batchStart = batchIndex == 0 ? 0 : substepContext.ConstraintBatchBoundaries[batchIndex - 1];
                    var workBlocksInBatch = substepContext.ConstraintBatchBoundaries[batchIndex] - batchStart;
                    substepContext.Stages[stageIndex] = new(claims.Slice(claimStart, workBlocksInBatch), batchStart, SolverStageType.Solve, batchIndex);
                    claimStart += workBlocksInBatch;
                    highestJobCountInSolve = Math.Max(highestJobCountInSolve, workBlocksInBatch);
                }
            }

            //for (int i = 0; i < iterationCountPlusOne; ++i)
            //{
            //    int claimStart = incrementalBlocks.Count;
            //    for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            //    {
            //        var stageIndex = targetStageIndex++;
            //        var batchStart = batchIndex == 0 ? 0 : substepContext.ConstraintBatchBoundaries[batchIndex - 1];
            //        var workBlocksInBatch = substepContext.ConstraintBatchBoundaries[batchIndex] - batchStart;
            //        substepContext.Stages[stageIndex] = new(claims.Slice(claimStart, workBlocksInBatch), batchStart,  batchIndex);
            //        claimStart += workBlocksInBatch;
            //    }
            //}

            //var syncCount = substepCount * (1 + synchronizedBatchCount * (1 + IterationCount)) - 1;
            //pool.Take(syncCount, out debugStageWorkBlocksCompleted);
            //pool.Take<int>(syncCount * workerCount, out var workBlocksCompleted);
            //workBlocksCompleted.Clear(0, workBlocksCompleted.Length);
            //for (int i = 0; i < syncCount; ++i)
            //{
            //    debugStageWorkBlocksCompleted[i] = workBlocksCompleted.Slice(i * workerCount, workerCount);
            //}

            //While we could be a little more aggressive about culling work with this condition, it doesn't matter much. Have to do it for correctness; worker relies on it.
            if (ActiveSet.Batches.Count > 0)
            {
                //workDelegate(0);
                threadDispatcher.DispatchWorkers(workDelegate, highestJobCountInSolve);
            }

            //pool.Take<int>(syncCount, out var availableCountPerSync);
            //var syncIndex = 0;
            //for (int substepIndex = 0; substepIndex < substepCount; ++substepIndex)
            //{
            //    if (substepIndex > 0)
            //    {
            //        availableCountPerSync[syncIndex] = incrementalBlocks.Count;
            //        ++syncIndex;
            //    }
            //    for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            //    {
            //        var batchStart = batchIndex == 0 ? 0 : substepContext.ConstraintBatchBoundaries[batchIndex - 1];
            //        var workBlocksInBatch = substepContext.ConstraintBatchBoundaries[batchIndex] - batchStart;
            //        availableCountPerSync[syncIndex] = workBlocksInBatch;
            //        ++syncIndex;
            //    }
            //    for (int i = 0; i < IterationCount; ++i)
            //    {
            //        for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            //        {
            //            var batchStart = batchIndex == 0 ? 0 : substepContext.ConstraintBatchBoundaries[batchIndex - 1];
            //            var workBlocksInBatch = substepContext.ConstraintBatchBoundaries[batchIndex] - batchStart;
            //            availableCountPerSync[syncIndex] = workBlocksInBatch;
            //            ++syncIndex;
            //        }
            //    }
            //}

            //pool.Take<int>(workerCount, out var workerBlocksCompletedSums);
            //pool.Take<double>(workerCount, out var workerFractionSum);
            //workerBlocksCompletedSums.Clear(0, workerBlocksCompletedSums.Length);
            //var availableCountSum = 0;
            //for (int i = 0; i < syncCount; ++i)
            //{
            //    if (availableCountPerSync[i] <= 1)
            //        continue;
            //    Console.WriteLine($"Sync {i}, available {availableCountPerSync[i]}, ideal {(availableCountPerSync[i] / (double)workerCount):G2}:");
            //    var stageWorkerBlockCounts = debugStageWorkBlocksCompleted[i];
            //    for (int j = 0; j < workerCount; ++j)
            //    {
            //        workerBlocksCompletedSums[j] += stageWorkerBlockCounts[j];
            //        var expectedCount = availableCountPerSync[i] / (double)workerCount;
            //        if (j >= availableCountPerSync[i])
            //            workerFractionSum[j] += 1;
            //        else
            //            workerFractionSum[j] += stageWorkerBlockCounts[j] / expectedCount;
            //        Console.WriteLine($"{j}: {stageWorkerBlockCounts[j]}");
            //    }
            //    availableCountSum += availableCountPerSync[i];
            //}
            ////var idealOccupancy = 1.0 / workerCount;
            ////Console.WriteLine($"Worker occupancy (ideal {idealOccupancy}):");
            ////for (int i = 0; i < workerCount; ++i)
            ////{
            ////    //Console.WriteLine($"{i}: {(workerBlocksCompletedSums[i] / (double)availableCountSum):G3}");
            ////    Console.WriteLine($"{i}: {workerFractionSum[i] / syncCount:G3}");
            ////}

            //pool.Return(ref workerBlocksCompletedSums);
            //pool.Return(ref workBlocksCompleted);
            //pool.Return(ref debugStageWorkBlocksCompleted);
            //pool.Return(ref availableCountPerSync);

            pool.Return(ref claims);
            pool.Return(ref substepContext.Stages);
            pool.Return(ref substepContext.ConstraintBatchBoundaries);
            pool.Return(ref substepContext.IncrementalUpdateBlocks);
            if (substepContext.KinematicIntegrationBlocks.Allocated)
                pool.Return(ref substepContext.KinematicIntegrationBlocks);
            pool.Return(ref substepContext.ConstraintBlocks);
            pool.Return(ref substepContext.VelocityIterationCounts);



        }

        struct IsFallbackBatch { }
        struct IsNotFallbackBatch { }
        unsafe bool ComputeIntegrationResponsibilitiesForConstraintRegion<TFallbackness>(int batchIndex, int typeBatchIndex, int constraintStart, int exclusiveConstraintEnd) where TFallbackness : unmanaged
        {
            ref var firstObservedForBatch = ref bodiesFirstObservedInBatches[batchIndex];
            ref var integrationFlagsForTypeBatch = ref integrationFlags[batchIndex][typeBatchIndex];
            ref var typeBatch = ref ActiveSet.Batches[batchIndex].TypeBatches[typeBatchIndex];
            var typeBatchBodyReferences = typeBatch.BodyReferences.As<int>();
            var bodiesPerConstraintInTypeBatch = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
            var intsPerBundle = Vector<int>.Count * bodiesPerConstraintInTypeBatch;
            var bundleStartIndex = constraintStart / Vector<float>.Count;
            var bundleEndIndex = (exclusiveConstraintEnd + Vector<float>.Count - 1) / Vector<float>.Count;
            Debug.Assert(bundleStartIndex >= 0 && bundleEndIndex <= typeBatch.BundleCount);
            ref var activeSet = ref bodies.ActiveSet;

            for (int bundleIndex = bundleStartIndex; bundleIndex < bundleEndIndex; ++bundleIndex)
            {
                int bundleStartIndexInConstraints = bundleIndex * Vector<int>.Count;
                int countInBundle = Math.Min(Vector<float>.Count, typeBatch.ConstraintCount - bundleStartIndexInConstraints);
                //Body references are stored in AOSOA layout.
                var bundleBodyReferencesStart = typeBatchBodyReferences.Memory + bundleIndex * intsPerBundle;
                for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraintInTypeBatch; ++bodyIndexInConstraint)
                {
                    ref var integrationFlagsForBodyInConstraint = ref integrationFlagsForTypeBatch[bodyIndexInConstraint];
                    var bundleStart = bundleBodyReferencesStart + bodyIndexInConstraint * Vector<int>.Count;
                    for (int bundleInnerIndex = 0; bundleInnerIndex < countInBundle; ++bundleInnerIndex)
                    {
                        //Constraints refer to bodies by index when they're in the active set, so we need to transform to handle to look up our merged batch results.
                        int bodyIndex;
                        if (typeof(TFallbackness) == typeof(IsFallbackBatch))
                        {
                            //Fallback batches can contain empty lanes; there's no guarantee of constraint contiguity. Such lanes are marked with -1 in the body references.
                            //Just skip over them.
                            var rawBodyIndex = bundleStart[bundleInnerIndex];
                            if (rawBodyIndex == -1)
                            {
                                continue;
                            }
                            bodyIndex = rawBodyIndex & Bodies.BodyReferenceMask;
                        }
                        else
                        {
                            bodyIndex = bundleStart[bundleInnerIndex] & Bodies.BodyReferenceMask;
                        }
                        var bodyHandle = activeSet.IndexToHandle[bodyIndex].Value;
                        if (firstObservedForBatch.Contains(bodyHandle))
                        {
                            if (typeof(TFallbackness) == typeof(IsFallbackBatch))
                            {
                                //This is a fallback. Being contained is not sufficient to require integration; it must also be the *first* constraint that will be executed.
                                //This is guaranteed by the index of the constraint in the type batch, and the type batch's index.
                                //Note that, since the fallback batch was the first time this body was seen, we know that *all* constraints associated with this body must be in the fallback batch.
                                //This could be significantly optimized by not recalculating the earliest candidate every single time a body is encountered in the fallback batch, but
                                //the fallback batch should effectively *never* contain any integration responsibilities.
                                ulong earliestIndex = ulong.MaxValue;
                                ref var constraintsForBody = ref activeSet.Constraints[bodyIndex];
                                for (int constraintIndexInBody = 0; constraintIndexInBody < constraintsForBody.Count; ++constraintIndexInBody)
                                {
                                    ref var fallbackBatch = ref ActiveSet.Batches[FallbackBatchThreshold];
                                    ref var location = ref HandleToConstraint[constraintsForBody[constraintIndexInBody].ConnectingConstraintHandle.Value];
                                    var typeBatchIndexForCandidate = fallbackBatch.TypeIndexToTypeBatchIndex[location.TypeId];
                                    var candidate = ((ulong)typeBatchIndexForCandidate << 32) | (uint)location.IndexInTypeBatch;
                                    if (candidate < earliestIndex)
                                        earliestIndex = candidate;
                                }
                                var indexInTypeBatch = bundleStartIndexInConstraints + bundleInnerIndex;
                                var currentSlot = ((ulong)typeBatchIndex << 32) | (uint)indexInTypeBatch;
                                if (currentSlot == earliestIndex)
                                {
                                    integrationFlagsForBodyInConstraint.AddUnsafely(indexInTypeBatch);
                                }
                            }
                            else
                            {
                                //Not a fallback; being contained in the observed set is sufficient.
                                integrationFlagsForBodyInConstraint.AddUnsafely(bundleStartIndexInConstraints + bundleInnerIndex);
                            }
                        }
                    }
                }
            }
            //Precompute which type batches have *any* integration responsibilities, allowing us to use a all-or-nothing test before dispatching a workblock.
            //Note that this could be vectorized the same way we did in the batch merging, but... less likely to be useful. Less constraints in sequence in type batches,
            //and we're already within a multithreaded context. Saving 1 microsecond not terribly meaningful.
            var flagBundleCount = IndexSet.GetBundleCapacity(typeBatch.ConstraintCount);
            ulong mergedFlagBundles = 0;
            for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraintInTypeBatch; ++bodyIndexInConstraint)
            {
                ref var integrationFlagsForBodyInTypeBatch = ref integrationFlagsForTypeBatch[bodyIndexInConstraint];
                for (int i = 0; i < flagBundleCount; ++i)
                {
                    mergedFlagBundles |= integrationFlagsForBodyInTypeBatch.Flags[i];
                }
            }
            return mergedFlagBundles != 0;
        }

        void ConstraintIntegrationResponsibilitiesWorker(int workerIndex)
        {
            int jobIndex;
            while ((jobIndex = Interlocked.Increment(ref nextConstraintIntegrationResponsibilityJobIndex) - 1) < integrationResponsibilityPrepassJobs.Count)
            {
                ref var job = ref integrationResponsibilityPrepassJobs[jobIndex];
                if (job.batch == FallbackBatchThreshold)
                    jobAlignedIntegrationResponsibilities[jobIndex] = ComputeIntegrationResponsibilitiesForConstraintRegion<IsFallbackBatch>(job.batch, job.typeBatch, job.start, job.end);
                else
                    jobAlignedIntegrationResponsibilities[jobIndex] = ComputeIntegrationResponsibilitiesForConstraintRegion<IsNotFallbackBatch>(job.batch, job.typeBatch, job.start, job.end);
            }
        }

        int nextConstraintIntegrationResponsibilityJobIndex;
        QuickList<(int batch, int typeBatch, int start, int end)> integrationResponsibilityPrepassJobs;
        Buffer<bool> jobAlignedIntegrationResponsibilities;
        Buffer<IndexSet> bodiesFirstObservedInBatches;
        Buffer<Buffer<Buffer<IndexSet>>> integrationFlags;
        /// <summary>
        /// Caches a single bool for whether type batches within batches have constraints with any integration responsibilities.
        /// Type batches with no integration responsibilities can use a codepath with no integration checks at all.
        /// </summary>
        Buffer<Buffer<bool>> coarseBatchIntegrationResponsibilities;
        Action<int> constraintIntegrationResponsibilitiesWorker;
        IndexSet mergedConstrainedBodyHandles;

        public override unsafe IndexSet PrepareConstraintIntegrationResponsibilities(IThreadDispatcher threadDispatcher = null)
        {
            if (ActiveSet.Batches.Count > 0)
            {
                pool.Take(ActiveSet.Batches.Count, out integrationFlags);
                integrationFlags[0] = default;
                pool.Take(ActiveSet.Batches.Count, out coarseBatchIntegrationResponsibilities);
                for (int batchIndex = 1; batchIndex < integrationFlags.Length; ++batchIndex)
                {
                    ref var batch = ref ActiveSet.Batches[batchIndex];
                    ref var flagsForBatch = ref integrationFlags[batchIndex];
                    pool.Take(batch.TypeBatches.Count, out flagsForBatch);
                    pool.Take(batch.TypeBatches.Count, out coarseBatchIntegrationResponsibilities[batchIndex]);
                    for (int typeBatchIndex = 0; typeBatchIndex < flagsForBatch.Length; ++typeBatchIndex)
                    {
                        ref var flagsForTypeBatch = ref flagsForBatch[typeBatchIndex];
                        ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                        var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                        pool.Take(bodiesPerConstraint, out flagsForTypeBatch);
                        for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraint; ++bodyIndexInConstraint)
                        {
                            flagsForTypeBatch[bodyIndexInConstraint] = new IndexSet(pool, typeBatch.ConstraintCount);
                        }
                    }
                }

                //Brute force fallback for debugging:
                //for (int i = 0; i < bodies.ActiveSet.Count; ++i)
                //{
                //    ref var constraints = ref bodies.ActiveSet.Constraints[i];
                //    ConstraintHandle minimumConstraint;
                //    minimumConstraint.Value = -1;
                //    int minimumBatchIndex = int.MaxValue;
                //    int minimumIndexInConstraint = -1;
                //    for (int j = 0; j < constraints.Count; ++j)
                //    {
                //        ref var constraint = ref constraints[j];
                //        var batchIndex = HandleToConstraint[constraint.ConnectingConstraintHandle.Value].BatchIndex;
                //        if (batchIndex < minimumBatchIndex)
                //        {
                //            minimumBatchIndex = batchIndex;
                //            minimumIndexInConstraint = constraint.BodyIndexInConstraint;
                //            minimumConstraint = constraint.ConnectingConstraintHandle;
                //        }
                //    }
                //    if (minimumConstraint.Value >= 0)
                //    {
                //        ref var location = ref HandleToConstraint[minimumConstraint.Value];
                //        var typeBatchIndex = ActiveSet.Batches[location.BatchIndex].TypeIndexToTypeBatchIndex[location.TypeId];
                //        ref var indexSet = ref integrationFlags[location.BatchIndex][typeBatchIndex][minimumIndexInConstraint];
                //        indexSet.AddUnsafely(location.IndexInTypeBatch);
                //    }
                //}

                pool.Take(batchReferencedHandles.Count, out bodiesFirstObservedInBatches);
                //We don't have to consider the first batch, since we know ahead of time that the first batch will be the first time we see any bodies in it.
                //Just copy directly from the first batch into the merged to initialize it.
                //Note "+ 64" instead of "+ 63": the highest possibly claimed id is inclusive!
                pool.Take((bodies.HandlePool.HighestPossiblyClaimedId + 64) / 64, out mergedConstrainedBodyHandles.Flags);
                var copyLength = Math.Min(mergedConstrainedBodyHandles.Flags.Length, batchReferencedHandles[0].Flags.Length);
                batchReferencedHandles[0].Flags.CopyTo(0, mergedConstrainedBodyHandles.Flags, 0, copyLength);
                batchReferencedHandles[0].Flags.Clear(copyLength, batchReferencedHandles[0].Flags.Length - copyLength);

                //Yup, we're just leaving the first slot unallocated to avoid having to offset indices all over the place. Slight wonk, but not a big deal.
                bodiesFirstObservedInBatches[0] = default;
                pool.Take<bool>(batchReferencedHandles.Count, out var batchHasAnyIntegrationResponsibilities);
                for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
                {
                    ref var batchHandles = ref batchReferencedHandles[batchIndex];
                    var bundleCount = Math.Min(mergedConstrainedBodyHandles.Flags.Length, batchHandles.Flags.Length);
                    //Note that we bypass the constructor to avoid zeroing unnecessarily. Every bundle will be fully assigned.
                    pool.Take(bundleCount, out bodiesFirstObservedInBatches[batchIndex].Flags);
                }
                GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
                //Note that we are not multithreading the batch merging phase. This typically takes a handful of microseconds.
                //You'd likely need millions of bodies before you'd see any substantial benefit from multithreading this.
                var batchCount = ActiveSet.Batches.Count;
                for (int batchIndex = 1; batchIndex < batchCount; ++batchIndex)
                {
                    ref var batchHandles = ref batchReferencedHandles[batchIndex];
                    ref var firstObservedInBatch = ref bodiesFirstObservedInBatches[batchIndex];
                    var flagBundleCount = Math.Min(mergedConstrainedBodyHandles.Flags.Length, batchHandles.Flags.Length);

                    if (Avx2.IsSupported)
                    {
                        var avxBundleCount = flagBundleCount / 4;
                        var horizontalAvxMerge = Vector256<ulong>.Zero;
                        for (int avxBundleIndex = 0; avxBundleIndex < avxBundleCount; ++avxBundleIndex)
                        {
                            //These will *almost* always be aligned, but guaranteeing it is not worth the complexity.
                            var mergeBundle = Avx2.LoadVector256((ulong*)((Vector256<ulong>*)mergedConstrainedBodyHandles.Flags.Memory + avxBundleIndex));
                            var batchBundle = Avx2.LoadVector256((ulong*)((Vector256<ulong>*)batchHandles.Flags.Memory + avxBundleIndex));
                            Avx.Store((ulong*)((Vector256<ulong>*)mergedConstrainedBodyHandles.Flags.Memory + avxBundleIndex), Avx2.Or(mergeBundle, batchBundle));
                            //If this batch contains a body, and the merged set does not, then it's the first batch that sees a body and it will have integration responsibility.
                            var firstObservedBundle = Avx2.AndNot(mergeBundle, batchBundle);
                            horizontalAvxMerge = Avx2.Or(firstObservedBundle, horizontalAvxMerge);
                            Avx.Store((ulong*)((Vector256<ulong>*)firstObservedInBatch.Flags.Memory + avxBundleIndex), firstObservedBundle);
                        }
                        var notEqual = Avx2.Xor(Avx2.CompareEqual(horizontalAvxMerge, Vector256<ulong>.Zero), Vector256<ulong>.AllBitsSet);
                        ulong horizontalMerge = (ulong)Avx2.MoveMask(notEqual.AsDouble());

                        //Cleanup loop.
                        for (int flagBundleIndex = avxBundleCount * 4; flagBundleIndex < flagBundleCount; ++flagBundleIndex)
                        {
                            var mergeBundle = mergedConstrainedBodyHandles.Flags[flagBundleIndex];
                            var batchBundle = batchHandles.Flags[flagBundleIndex];
                            mergedConstrainedBodyHandles.Flags[flagBundleIndex] = mergeBundle | batchBundle;
                            //If this batch contains a body, and the merged set does not, then it's the first batch that sees a body and it will have integration responsibility.
                            var firstObservedBundle = ~mergeBundle & batchBundle;
                            horizontalMerge |= firstObservedBundle;
                            firstObservedInBatch.Flags[flagBundleIndex] = firstObservedBundle;
                        }
                        batchHasAnyIntegrationResponsibilities[batchIndex] = horizontalMerge != 0;
                    }
                    else
                    {
                        ulong horizontalMerge = 0;
                        for (int flagBundleIndex = 0; flagBundleIndex < flagBundleCount; ++flagBundleIndex)
                        {
                            var mergeBundle = mergedConstrainedBodyHandles.Flags[flagBundleIndex];
                            var batchBundle = batchHandles.Flags[flagBundleIndex];
                            mergedConstrainedBodyHandles.Flags[flagBundleIndex] = mergeBundle | batchBundle;
                            //If this batch contains a body, and the merged set does not, then it's the first batch that sees a body and it will have integration responsibility.
                            var firstObservedBundle = ~mergeBundle & batchBundle;
                            horizontalMerge |= firstObservedBundle;
                            firstObservedInBatch.Flags[flagBundleIndex] = firstObservedBundle;
                        }
                        batchHasAnyIntegrationResponsibilities[batchIndex] = horizontalMerge != 0;
                    }
                }

                //var start = Stopwatch.GetTimestamp();
                //We now have index sets representing the first time each body handle is observed in a batch.
                //This process is significantly more expensive than the batch merging phase and can benefit from multithreading.
                //It is still fairly cheap, though- we can't use really fine grained jobs or the cost of swapping jobs will exceed productive work.

                //Note that we arbitrarily use single threaded execution if the job is small enough. Dispatching isn't free.
                bool useSingleThreadedPath = true;
                if (threadDispatcher != null && threadDispatcher.ThreadCount > 1)
                {
                    integrationResponsibilityPrepassJobs = new(128, pool);
                    int constraintCount = 0;
                    const int targetJobSize = 2048;
                    Debug.Assert(targetJobSize % 64 == 0, "Target job size must be a multiple of the index set bundles to avoid threads working on the same flag bundle.");
                    for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
                    {
                        if (!batchHasAnyIntegrationResponsibilities[batchIndex])
                            continue;
                        ref var batch = ref ActiveSet.Batches[batchIndex];
                        for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                        {
                            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                            constraintCount += typeBatch.ConstraintCount;
                            int jobCountForTypeBatch = (typeBatch.ConstraintCount + targetJobSize - 1) / targetJobSize;
                            for (int i = 0; i < jobCountForTypeBatch; ++i)
                            {
                                var jobStart = i * targetJobSize;
                                var jobEnd = Math.Min(jobStart + targetJobSize, typeBatch.ConstraintCount);
                                integrationResponsibilityPrepassJobs.Allocate(pool) = (batchIndex, typeBatchIndex, jobStart, jobEnd);
                            }
                        }
                    }
                    if (constraintCount > 4096 + threadDispatcher.ThreadCount * 1024)
                    {
                        nextConstraintIntegrationResponsibilityJobIndex = 0;
                        useSingleThreadedPath = false;
                        pool.Take(integrationResponsibilityPrepassJobs.Count, out jobAlignedIntegrationResponsibilities);
                        //for (int i = 0; i < integrationResponsibilityPrepassJobs.Count; ++i)
                        //{
                        //    ref var job = ref integrationResponsibilityPrepassJobs[i];
                        //    jobAlignedIntegrationResponsibilities[i] = ComputeIntegrationResponsibilitiesForConstraintRegion(job.batch, job.typeBatch, job.start, job.end);
                        //}
                        threadDispatcher.DispatchWorkers(constraintIntegrationResponsibilitiesWorker, integrationResponsibilityPrepassJobs.Count);

                        //Coarse batch integration responsibilities start uninitialized. Possible to have multiple jobs per type batch in multithreaded case, so we need to init to merge.
                        for (int i = 1; i < ActiveSet.Batches.Count; ++i)
                        {
                            ref var batch = ref ActiveSet.Batches[i];
                            for (int j = 0; j < batch.TypeBatches.Count; ++j)
                            {
                                coarseBatchIntegrationResponsibilities[i][j] = false;
                            }
                        }
                        for (int i = 0; i < integrationResponsibilityPrepassJobs.Count; ++i)
                        {
                            ref var job = ref integrationResponsibilityPrepassJobs[i];
                            coarseBatchIntegrationResponsibilities[job.batch][job.typeBatch] |= jobAlignedIntegrationResponsibilities[i];
                        }
                        pool.Return(ref jobAlignedIntegrationResponsibilities);
                    }
                    integrationResponsibilityPrepassJobs.Dispose(pool);
                }
                if (useSingleThreadedPath)
                {
                    for (int i = 1; i < synchronizedBatchCount; ++i)
                    {
                        if (!batchHasAnyIntegrationResponsibilities[i])
                            continue;
                        ref var batch = ref ActiveSet.Batches[i];
                        for (int j = 0; j < batch.TypeBatches.Count; ++j)
                        {
                            ref var typeBatch = ref batch.TypeBatches[j];
                            coarseBatchIntegrationResponsibilities[i][j] = ComputeIntegrationResponsibilitiesForConstraintRegion<IsNotFallbackBatch>(i, j, 0, typeBatch.ConstraintCount);
                        }
                    }
                    if (fallbackExists && batchHasAnyIntegrationResponsibilities[FallbackBatchThreshold])
                    {
                        ref var batch = ref ActiveSet.Batches[FallbackBatchThreshold];
                        for (int j = 0; j < batch.TypeBatches.Count; ++j)
                        {
                            ref var typeBatch = ref batch.TypeBatches[j];
                            coarseBatchIntegrationResponsibilities[FallbackBatchThreshold][j] = ComputeIntegrationResponsibilitiesForConstraintRegion<IsFallbackBatch>(FallbackBatchThreshold, j, 0, typeBatch.ConstraintCount);
                        }
                    }
                }
                //var end = Stopwatch.GetTimestamp();
                //Console.WriteLine($"time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");

                ////Validation:
                //for (int i = 0; i < bodies.ActiveSet.Count; ++i)
                //{
                //    ref var constraints = ref bodies.ActiveSet.Constraints[i];
                //    ConstraintHandle minimumConstraint;
                //    minimumConstraint.Value = -1;
                //    int minimumBatchIndex = int.MaxValue;
                //    int minimumBodyIndexInConstraint = -1;
                //    ulong earliestSlotInFallback = ulong.MaxValue;
                //    ConstraintHandle detectedConstraint;
                //    detectedConstraint.Value = -1;
                //    for (int j = 0; j < constraints.Count; ++j)
                //    {
                //        ref var constraint = ref constraints[j];
                //        ref var location = ref HandleToConstraint[constraint.ConnectingConstraintHandle.Value];
                //        var typeBatchIndex = ActiveSet.Batches[location.BatchIndex].TypeIndexToTypeBatchIndex[location.TypeId];

                //        if (location.BatchIndex <= minimumBatchIndex) //Note that the only time it can be equal is if this is in the fallback batch.
                //        {
                //            if (location.BatchIndex == FallbackBatchThreshold)
                //            {
                //                var encodedSlotCandidate = ((ulong)typeBatchIndex << 32) | (uint)location.IndexInTypeBatch;
                //                if (encodedSlotCandidate < earliestSlotInFallback)
                //                {
                //                    earliestSlotInFallback = encodedSlotCandidate;
                //                    //We should only accept another fallback constraint as minimal if it has an earlier typebatch index/index in type batch.
                //                    minimumBatchIndex = location.BatchIndex;
                //                    minimumBodyIndexInConstraint = constraint.BodyIndexInConstraint;
                //                    minimumConstraint = constraint.ConnectingConstraintHandle;
                //                }
                //            }
                //            else
                //            {
                //                minimumBatchIndex = location.BatchIndex;
                //                minimumBodyIndexInConstraint = constraint.BodyIndexInConstraint;
                //                minimumConstraint = constraint.ConnectingConstraintHandle;
                //            }
                //        }

                //        if (location.BatchIndex > 0)
                //        {
                //            ref var indexSet = ref integrationFlags[location.BatchIndex][typeBatchIndex][constraint.BodyIndexInConstraint];
                //            if (indexSet.Contains(location.IndexInTypeBatch))
                //            {
                //                //This constraint has integration responsibility for this body.
                //                Debug.Assert(detectedConstraint.Value == -1, "Only one constraint should have integration responsibility for a given body.");
                //                detectedConstraint = constraint.ConnectingConstraintHandle;
                //            }
                //        }
                //    }
                //    if (constraints.Count > 0 && minimumBatchIndex > 0)
                //        Debug.Assert(detectedConstraint.Value >= 0, "At least one constraint must have integration responsibility for a body if it has a constraint.");
                //    if (minimumConstraint.Value >= 0)
                //    {
                //        Debug.Assert(minimumBatchIndex == 0 || detectedConstraint.Value == minimumConstraint.Value);
                //        ref var location = ref HandleToConstraint[minimumConstraint.Value];
                //        var typeBatchIndex = ActiveSet.Batches[location.BatchIndex].TypeIndexToTypeBatchIndex[location.TypeId];
                //        if (location.BatchIndex > 0)
                //        {
                //            ref var indexSet = ref integrationFlags[location.BatchIndex][typeBatchIndex][minimumBodyIndexInConstraint];
                //            Debug.Assert(indexSet.Contains(location.IndexInTypeBatch));
                //        }
                //    }
                //}

                pool.Return(ref batchHasAnyIntegrationResponsibilities);

                Debug.Assert(!bodiesFirstObservedInBatches[0].Flags.Allocated, "Remember, we're assuming we're just leaving the first batch's slot empty to avoid indexing complexity.");
                for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
                {
                    bodiesFirstObservedInBatches[batchIndex].Dispose(pool);
                }
                pool.Return(ref bodiesFirstObservedInBatches);

                //Add the constrained kinematics to the constrained body handles. The kinematics were absent from batch referenced handles.
                //TODO: This assumes the number of kinematics is low relative to the number of bodies and does not need to be multithreaded.
                //This assumption is *usually* fine, but we should probably have a fallback that is more efficient if this assumption is wrong.
                //Could maintain an indexset parallel to the ConstrainedKinematicHandles- same set, just different format.
                //If we detect a lot of constrained kinematics, just do an indexset merge.
                //That would be fast enough even if all bodies were kinematic.
                for (int i = 0; i < ConstrainedKinematicHandles.Count; ++i)
                {
                    mergedConstrainedBodyHandles.AddUnsafely(ConstrainedKinematicHandles[i]);
                }
                return mergedConstrainedBodyHandles;
            }
            else
            {
                return new IndexSet();
            }
        }
        public override void DisposeConstraintIntegrationResponsibilities()
        {
            if (ActiveSet.Batches.Count > 0)
            {
                Debug.Assert(!integrationFlags[0].Allocated, "Remember, we're assuming we're just leaving the first batch's slot empty to avoid indexing complexity.");
                for (int batchIndex = 1; batchIndex < integrationFlags.Length; ++batchIndex)
                {
                    ref var flagsForBatch = ref integrationFlags[batchIndex];
                    for (int typeBatchIndex = 0; typeBatchIndex < flagsForBatch.Length; ++typeBatchIndex)
                    {
                        ref var flagsForTypeBatch = ref flagsForBatch[typeBatchIndex];
                        for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < flagsForTypeBatch.Length; ++bodyIndexInConstraint)
                        {
                            flagsForTypeBatch[bodyIndexInConstraint].Dispose(pool);
                        }
                        pool.Return(ref flagsForTypeBatch);
                    }
                    pool.Return(ref flagsForBatch);
                    pool.Return(ref coarseBatchIntegrationResponsibilities[batchIndex]);
                }
                pool.Return(ref integrationFlags);
                pool.Return(ref coarseBatchIntegrationResponsibilities);
                mergedConstrainedBodyHandles.Dispose(pool);
            }
        }

        public override void Solve(float totalDt, IThreadDispatcher threadDispatcher = null)
        {
            var substepDt = totalDt / substepCount;
            PoseIntegrator.Callbacks.PrepareForIntegration(substepDt);
            if (threadDispatcher == null)
            {
                var inverseDt = 1f / substepDt;
                ref var activeSet = ref ActiveSet;
                var batchCount = activeSet.Batches.Count;
                for (int substepIndex = 0; substepIndex < substepCount; ++substepIndex)
                {
                    OnSubstepStarted(substepIndex);
                    if (substepIndex > 0)
                    {
                        for (int i = 0; i < batchCount; ++i)
                        {
                            ref var batch = ref activeSet.Batches[i];
                            for (int j = 0; j < batch.TypeBatches.Count; ++j)
                            {
                                ref var typeBatch = ref batch.TypeBatches[j];
                                var processor = TypeProcessors[typeBatch.TypeId];
                                if (processor.RequiresIncrementalSubstepUpdates)
                                    processor.IncrementallyUpdateForSubstep(ref typeBatch, bodies, substepDt, inverseDt, 0, typeBatch.BundleCount);
                            }
                        }
                        PoseIntegrator.IntegrateKinematicPosesAndVelocities(ConstrainedKinematicHandles.Span.Slice(ConstrainedKinematicHandles.Count), 0, BundleIndexing.GetBundleCount(ConstrainedKinematicHandles.Count), substepDt, 0);
                    }
                    else
                    {
                        if (PoseIntegrator.Callbacks.IntegrateVelocityForKinematics)
                            PoseIntegrator.IntegrateKinematicVelocities(ConstrainedKinematicHandles.Span.Slice(ConstrainedKinematicHandles.Count), 0, BundleIndexing.GetBundleCount(ConstrainedKinematicHandles.Count), substepDt, 0);
                    }
                    for (int i = 0; i < batchCount; ++i)
                    {
                        ref var batch = ref activeSet.Batches[i];
                        ref var integrationFlagsForBatch = ref integrationFlags[i];
                        for (int j = 0; j < batch.TypeBatches.Count; ++j)
                        {
                            ref var typeBatch = ref batch.TypeBatches[j];
                            if (substepIndex == 0)
                            {
                                WarmStartBlock<DisallowPoseIntegration>(0, i, j, 0, typeBatch.BundleCount, ref typeBatch, TypeProcessors[typeBatch.TypeId], substepDt, inverseDt);
                            }
                            else
                            {
                                WarmStartBlock<AllowPoseIntegration>(0, i, j, 0, typeBatch.BundleCount, ref typeBatch, TypeProcessors[typeBatch.TypeId], substepDt, inverseDt);
                            }
                        }
                    }
                    var velocityIterationCount = GetVelocityIterationCountForSubstepIndex(substepIndex);
                    for (int iterationIndex = 0; iterationIndex < velocityIterationCount; ++iterationIndex)
                    {
                        for (int i = 0; i < batchCount; ++i)
                        {
                            ref var batch = ref activeSet.Batches[i];
                            for (int j = 0; j < batch.TypeBatches.Count; ++j)
                            {
                                ref var typeBatch = ref batch.TypeBatches[j];
                                TypeProcessors[typeBatch.TypeId].Solve(ref typeBatch, bodies, substepDt, inverseDt, 0, typeBatch.BundleCount);
                            }
                        }
                    }
                    OnSubstepEnded(substepIndex);
                }
            }
            else
            {
                ExecuteMultithreaded(substepDt, threadDispatcher, solveWorker);
            }
        }
    }
}
