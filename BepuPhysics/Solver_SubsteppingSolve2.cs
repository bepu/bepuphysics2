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

namespace BepuPhysics
{
    internal enum SolverJobType
    {
        IncrementalUpdate,
        WarmStart,
        WarmStartFallback,
        Solve,
        SolveFallback,
    }

    internal struct SolverSyncStage
    {
        public Buffer<int> Claims;
        public int BatchIndex;
        public int WorkBlockStartIndex;

        public SolverSyncStage(Buffer<int> claims, int workBlockStartIndex, int batchIndex = -1)
        {
            Claims = claims;
            BatchIndex = batchIndex;
            WorkBlockStartIndex = workBlockStartIndex;
        }
    }

    [StructLayout(LayoutKind.Explicit)]
    internal struct SubstepMultithreadingContext
    {
        [FieldOffset(0)]
        public Buffer<SolverSyncStage> Stages;
        [FieldOffset(16)]
        public Buffer<WorkBlock> IncrementalUpdateBlocks;
        [FieldOffset(32)]
        public Buffer<WorkBlock> ConstraintBlocks;
        [FieldOffset(48)]
        public Buffer<FallbackScatterWorkBlock> FallbackBlocks;
        [FieldOffset(64)]
        public Buffer<int> ConstraintBatchBoundaries;
        [FieldOffset(80)]
        public float Dt;
        [FieldOffset(84)]
        public float InverseDt;
        [FieldOffset(88)]
        public int WorkerCount;


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


    public partial class Solver
    {
        public virtual IndexSet PrepareConstraintIntegrationResponsibilities(int substepCount, IThreadDispatcher threadDispatcher = null)
        {
            throw new NotImplementedException();
        }
        public virtual void DisposeConstraintIntegrationResponsibilities()
        {
            throw new NotImplementedException();
        }

        public virtual void SolveStep2(float dt, IThreadDispatcher threadDispatcher = null)
        {
            throw new NotImplementedException();
        }
    }
    public class Solver<TIntegrationCallbacks> : Solver where TIntegrationCallbacks : struct, IPoseIntegratorCallbacks
    {
        public Solver(Bodies bodies, BufferPool pool, int iterationCount, int fallbackBatchThreshold,
            int initialCapacity,
            int initialIslandCapacity,
            int minimumCapacityPerTypeBatch, PoseIntegrator<TIntegrationCallbacks> poseIntegrator)
            : base(bodies, pool, iterationCount, fallbackBatchThreshold, initialCapacity, initialIslandCapacity, minimumCapacityPerTypeBatch)
        {
            PoseIntegrator = poseIntegrator;
            solveStep2Worker2 = SolveStep2Worker2;
            constraintIntegrationResponsibilitiesWorker = ConstraintIntegrationResponsibilitiesWorker;
        }

        public PoseIntegrator<TIntegrationCallbacks> PoseIntegrator { get; private set; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void WarmStartBlock<TBatchShouldIntegratePoses>(int workerIndex, int batchIndex, int typeBatchIndex, int startBundle, int endBundle, ref TypeBatch typeBatch, TypeProcessor typeProcessor, float dt, float inverseDt)
            where TBatchShouldIntegratePoses : unmanaged, IBatchPoseIntegrationAllowed
        {
            if (batchIndex == 0)
            {
                Buffer<IndexSet> noFlagsRequired = default;
                typeProcessor.WarmStart2<TIntegrationCallbacks, BatchShouldAlwaysIntegrate, TBatchShouldIntegratePoses>(
                    ref typeBatch, ref noFlagsRequired, bodies, ref PoseIntegrator.Callbacks,
                    dt, inverseDt, startBundle, endBundle, workerIndex);
            }
            else
            {
                if (coarseBatchIntegrationResponsibilities[batchIndex][typeBatchIndex])
                {
                    typeProcessor.WarmStart2<TIntegrationCallbacks, BatchShouldConditionallyIntegrate, TBatchShouldIntegratePoses>(
                        ref typeBatch, ref integrationFlags[batchIndex][typeBatchIndex], bodies, ref PoseIntegrator.Callbacks,
                        dt, inverseDt, startBundle, endBundle, workerIndex);
                }
                else
                {
                    typeProcessor.WarmStart2<TIntegrationCallbacks, BatchShouldNeverIntegrate, TBatchShouldIntegratePoses>(
                        ref typeBatch, ref integrationFlags[batchIndex][typeBatchIndex], bodies, ref PoseIntegrator.Callbacks,
                        dt, inverseDt, startBundle, endBundle, workerIndex);
                }
            }
        }


        //Split the solve process into a warmstart and solve, where warmstart doesn't try to store out anything. It just computes jacobians and modifies velocities according to the accumulated impulse.
        //The solve step then *recomputes* jacobians from prestep data and pose information.
        //Why? Memory bandwidth. Redoing the calculation is cheaper than storing it out.
        struct WarmStartStep2StageFunction : IStageFunction
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

        struct SolveStep2StageFunction : IStageFunction
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
                typeProcessor.SolveStep2(ref typeBatch, solver.bodies, Dt, InverseDt, block.StartBundle, block.End);
            }
        }

        //struct FallbackSolveStep2StageFunction : IStageFunction
        //{
        //    public float Dt;
        //    public float InverseDt;

        //    [MethodImpl(MethodImplOptions.AggressiveInlining)]
        //    public void Execute(Solver solver, int blockIndex)
        //    {
        //        ref var block = ref solver.context.ConstraintBlocks.Blocks[blockIndex];
        //        ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
        //        var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
        //        typeProcessor.JacobiSolveStep2(ref typeBatch, solver.bodies, ref solver.ActiveSet.Fallback, ref solver.context.FallbackResults[block.TypeBatchIndex], Dt, InverseDt, block.StartBundle, block.End);
        //    }
        //}

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
                solver.TypeProcessors[typeBatch.TypeId].IncrementallyUpdateContactData(ref typeBatch, solver.bodies, Dt, InverseDt, block.StartBundle, block.End);
            }
        }

        void ExecuteWorkerStage<TStageFunction>(ref TStageFunction stageFunction, int workerIndex, int workerStart, int availableBlocksStartIndex, ref Buffer<int> claims, int previousSyncIndexOffset, int syncIndex, ref int completedWorkBlocks) where TStageFunction : IStageFunction
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
            var previousSyncIndex = Math.Max(0, syncIndex - previousSyncIndexOffset);
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
            //TODO: Technically, given wrap around of forward traversal, backwards looping seems.. questionable. Verify.
            //Try to claim work blocks going backward.
            workBlockIndex = workerStart - 1;
            while (true)
            {
                if (workBlockIndex < 0)
                {
                    //Wrap around.
                    workBlockIndex = claims.Length - 1;
                }
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
            //if (workerIndex == 3)
            //{
            //    Console.WriteLine($"Worker {workerIndex} completed {locallyCompletedCount / (double)claims.Length:G2} ({locallyCompletedCount} of {claims.Length}).");
            //}

        }
        void ExecuteMainStage<TStageFunction>(ref TStageFunction stageFunction, int workerIndex, int workerStart, ref SolverSyncStage stage, int previousSyncIndexOffset, ref int syncIndex) where TStageFunction : IStageFunction
        {
            //Note that the main thread's view of the sync index increments every single time, even if there is no work.
            //This ensures that the workers are able to advance to the appropriate stage by examining the sync index snapshot.
            ++syncIndex;
            var availableBlocksCount = stage.Claims.Length;
            if (availableBlocksCount == 0)
                return;

            //for (int i = 0; i < availableBlocksCount; ++i)
            //{
            //    stageFunction.Execute(this, stage.WorkBlockStartIndex + i, workerIndex);
            //}
            //return;

            if (availableBlocksCount == 1)
            {
                //There is only one work block available. There's no reason to notify other threads about it or do any claims management; just execute it sequentially.
                stageFunction.Execute(this, stage.WorkBlockStartIndex, workerIndex);
            }
            else
            {
                //Write the new stage index so other spinning threads will begin work on it.
                Volatile.Write(ref substepContext.SyncIndex, syncIndex);
                ExecuteWorkerStage(ref stageFunction, workerIndex, workerStart, stage.WorkBlockStartIndex, ref stage.Claims, previousSyncIndexOffset, syncIndex, ref substepContext.CompletedWorkBlockCount);

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
                //All workers are done. We can safely reset the counter for the next time this stage is used.
                substepContext.CompletedWorkBlockCount = 0;
            }
        }
        SubstepMultithreadingContext substepContext;


        Action<int> solveStep2Worker2;
        void SolveStep2Worker2(int workerIndex)
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
            int fallbackStart = GetUniformlyDistributedStart(workerIndex, substepContext.FallbackBlocks.Length, workerCount, 0);
            Buffer<int> batchStarts;
            ref var activeSet = ref ActiveSet;
            unsafe
            {
                var batchStartsData = stackalloc int[activeSet.Batches.Count];
                batchStarts = new Buffer<int>(batchStartsData, activeSet.Batches.Count);
            }
            for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
            {
                var batchOffset = batchIndex > 0 ? substepContext.ConstraintBatchBoundaries[batchIndex - 1] : 0;
                var batchCount = substepContext.ConstraintBatchBoundaries[batchIndex] - batchOffset;
                batchStarts[batchIndex] = GetUniformlyDistributedStart(workerIndex, batchCount, workerCount, 0);
            }

            Debug.Assert(activeSet.Batches.Count > 0, "Don't dispatch if there are no constraints.");
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);

            var incrementalUpdateStage = new IncrementalUpdateStageFunction
            {
                Dt = substepContext.Dt,
                InverseDt = substepContext.InverseDt,
                solver = this
            };
            var warmstartStage = new WarmStartStep2StageFunction
            {
                Dt = substepContext.Dt,
                InverseDt = substepContext.InverseDt,
                solver = this
            };
            var solveStage = new SolveStep2StageFunction
            {
                Dt = substepContext.Dt,
                InverseDt = substepContext.InverseDt,
                solver = this
            };

            var baseStageCountInSubstep = synchronizedBatchCount * (1 + IterationCount);
            if (workerIndex == 0)
            {
                //This is the main 'orchestrator' thread. It tracks execution progress and notifies other threads that's it's time to work.
                int syncIndex = 0;
                for (int substepIndex = 0; substepIndex < substepCount; ++substepIndex)
                {
                    if (substepIndex > 0)
                    {
                        ExecuteMainStage(ref incrementalUpdateStage, workerIndex, incrementalUpdateWorkerStart, ref substepContext.Stages[0], 1 + baseStageCountInSubstep, ref syncIndex);
                    }
                    for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                    {
                        warmstartStage.SubstepIndex = substepIndex;
                        ExecuteMainStage(ref warmstartStage, workerIndex, batchStarts[batchIndex], ref substepContext.Stages[batchIndex + 1], 1 + synchronizedBatchCount, ref syncIndex);
                    }
                    for (int iterationIndex = 0; iterationIndex < IterationCount; ++iterationIndex)
                    {
                        for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                        {
                            ExecuteMainStage(ref solveStage, workerIndex, batchStarts[batchIndex], ref substepContext.Stages[batchIndex + 1], synchronizedBatchCount, ref syncIndex);
                        }
                    }
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
                    //Stages were set up prior to execution. Note that we don't attempt to ping pong buffers or anything; there are unique entries for every single stage.
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
                        var stageCountInSubstep = substepIndex > 0 ? baseStageCountInSubstep + 1 : baseStageCountInSubstep;
                        if (syncIndexInSubstep >= stageCountInSubstep)
                        {
                            syncIndexInSubstep -= stageCountInSubstep;
                            ++substepIndex;
                        }
                        else
                        {
                            break;
                        }
                    }
                    var stageIndex = substepIndex == 0 ? syncIndexInSubstep + 1 : syncIndexInSubstep;
                    //Note that we're going to do a compare exchange that prevents any claim on work blocks that *arent* of the previous sync index, which means we need the previous sync index.
                    //Storing that in a reliable way is annoying, so we derive it from syncIndex.
                    if (stageIndex == 0)
                    {
                        //Incremental update.
                        ref var stage = ref substepContext.Stages[stageIndex];
                        ExecuteWorkerStage(ref incrementalUpdateStage, workerIndex, incrementalUpdateWorkerStart, 0, ref stage.Claims, 1 + baseStageCountInSubstep, syncIndex, ref substepContext.CompletedWorkBlockCount);
                    }
                    else if (stageIndex < 1 + synchronizedBatchCount)
                    {
                        //Warm start.
                        ref var stage = ref substepContext.Stages[stageIndex];
                        warmstartStage.SubstepIndex = substepIndex;
                        ExecuteWorkerStage(ref warmstartStage, workerIndex, batchStarts[stage.BatchIndex], stage.WorkBlockStartIndex, ref stage.Claims, 1 + synchronizedBatchCount, syncIndex, ref substepContext.CompletedWorkBlockCount);
                    }
                    else
                    {
                        //Solve.
                        stageIndex -= synchronizedBatchCount;
                        ref var stage = ref substepContext.Stages[stageIndex];
                        ExecuteWorkerStage(ref solveStage, workerIndex, batchStarts[stage.BatchIndex], stage.WorkBlockStartIndex, ref stage.Claims, synchronizedBatchCount, syncIndex, ref substepContext.CompletedWorkBlockCount);
                    }
                    latestCompletedSyncIndex = syncIndex;

                }
            }

        }

        protected void ExecuteMultithreaded2(float dt, IThreadDispatcher threadDispatcher, Action<int> workDelegate)
        {
            var workerCount = substepContext.WorkerCount = threadDispatcher.ThreadCount;
            substepContext.Dt = dt;
            substepContext.InverseDt = 1f / dt;
            //First build a set of work blocks.
            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
            //These values are found by empirical tuning. The optimal values may vary by architecture.
            //The goal here is to have just enough blocks that, in the event that we end up some underpowered threads (due to competition or hyperthreading), 
            //there are enough blocks that workstealing will still generally allow the extra threads to be useful.
            const int targetBlocksPerBatchPerWorker = 32;
            const int minimumBlockSizeInBundles = 1;
            const int maximumBlockSizeInBundles = 1024;

            var targetBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
            var mainFilter = new MainSolveFilter();
            var incrementalFilter = new IncrementalContactDataUpdateFilter();
            BuildWorkBlocks(pool, minimumBlockSizeInBundles, maximumBlockSizeInBundles, targetBlocksPerBatch, ref mainFilter, out var constraintBlocks, out substepContext.ConstraintBatchBoundaries);
            BuildWorkBlocks(pool, minimumBlockSizeInBundles, maximumBlockSizeInBundles, targetBlocksPerBatch, ref incrementalFilter, out var incrementalBlocks, out var incrementalUpdateBatchBoundaries);
            pool.Return(ref incrementalUpdateBatchBoundaries); //TODO: No need to create this in the first place. Doesn't really cost anything, but...
            substepContext.ConstraintBlocks = constraintBlocks.Span.Slice(constraintBlocks.Count);
            substepContext.IncrementalUpdateBlocks = incrementalBlocks.Span.Slice(incrementalBlocks.Count);

            //Not every batch will actually have work blocks associated with it; the batch compressor could be falling behind, which means older constraints could be at higher batches than they need to be, leaving gaps.
            //We don't want to include those empty batches as sync points in the solver.
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            pool.Take(1 + synchronizedBatchCount, out substepContext.Stages);
            substepContext.SyncIndex = 0;

            var totalConstraintBatchWorkBlockCount = substepContext.ConstraintBatchBoundaries[^1];
            var totalClaimCount = incrementalBlocks.Count + totalConstraintBatchWorkBlockCount;
            //Claims will be monotonically increasing throughout execution. All should start at zero to match with the initial sync index.
            pool.Take<int>(totalClaimCount, out var claims);
            claims.Clear(0, claims.Length);
            substepContext.Stages[0] = new(claims.Slice(incrementalBlocks.Count), 0);
            int claimStart = incrementalBlocks.Count;
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                var stageIndex = batchIndex + 1;
                var batchStart = batchIndex == 0 ? 0 : substepContext.ConstraintBatchBoundaries[batchIndex - 1];
                var workBlocksInBatch = substepContext.ConstraintBatchBoundaries[batchIndex] - batchStart;
                substepContext.Stages[stageIndex] = new(claims.Slice(claimStart, workBlocksInBatch), batchStart, batchIndex);
                claimStart += workBlocksInBatch;
            }


            //While we could be a little more aggressive about culling work with this condition, it doesn't matter much. Have to do it for correctness; worker relies on it.
            if (ActiveSet.Batches.Count > 0)
                threadDispatcher.DispatchWorkers(workDelegate);

            pool.Return(ref claims);
            pool.Return(ref substepContext.Stages);
            pool.Return(ref substepContext.ConstraintBatchBoundaries);
            pool.Return(ref substepContext.IncrementalUpdateBlocks);
            pool.Return(ref substepContext.ConstraintBlocks);

        }


        unsafe bool ComputeIntegrationResponsibilitiesForConstraintRegion(int batchIndex, int typeBatchIndex, int constraintStart, int exclusiveConstraintEnd)
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
                        var bodyHandle = bodies.ActiveSet.IndexToHandle[bundleStart[bundleInnerIndex]].Value;
                        if (firstObservedForBatch.Contains(bodyHandle))
                        {
                            integrationFlagsForBodyInConstraint.AddUnsafely(bundleStartIndexInConstraints + bundleInnerIndex);
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
                jobAlignedIntegrationResponsibilities[jobIndex] = ComputeIntegrationResponsibilitiesForConstraintRegion(job.batch, job.typeBatch, job.start, job.end);
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

        int substepCount;
        public override unsafe IndexSet PrepareConstraintIntegrationResponsibilities(int substepCount, IThreadDispatcher threadDispatcher = null)
        {
            //TODO: we're caching it on a per call basis because we are still using the old substeppingtimestepper frame externally. Once we bite the bullet 100% on bundling, we can make it equivalent to IterationCount.
            this.substepCount = substepCount;
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
                //Note that we are not multithreading the batch merging phase. This typically takes a handful of microseconds.
                //You'd likely need millions of bodies before you'd see any substantial benefit from multithreading this.
                for (int batchIndex = 1; batchIndex < ActiveSet.Batches.Count; ++batchIndex)
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
                            var mergeBundle = ((Vector256<ulong>*)mergedConstrainedBodyHandles.Flags.Memory)[avxBundleIndex];
                            var batchBundle = ((Vector256<ulong>*)batchHandles.Flags.Memory)[avxBundleIndex];
                            ((Vector256<ulong>*)mergedConstrainedBodyHandles.Flags.Memory)[avxBundleIndex] = Avx2.Or(mergeBundle, batchBundle);
                            //If this batch contains a body, and the merged set does not, then it's the first batch that sees a body and it will have integration responsibility.
                            var firstObservedBundle = Avx2.AndNot(mergeBundle, batchBundle);
                            horizontalAvxMerge = Avx2.Or(firstObservedBundle, horizontalAvxMerge);
                            ((Vector256<ulong>*)firstObservedInBatch.Flags.Memory)[avxBundleIndex] = firstObservedBundle;
                        }
                        var notEqual = Avx2.CompareNotEqual(horizontalAvxMerge.AsDouble(), Vector256<double>.Zero);
                        ulong horizontalMerge = (ulong)Avx.MoveMask(notEqual);

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
                        threadDispatcher.DispatchWorkers(constraintIntegrationResponsibilitiesWorker);

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
                    for (int i = 1; i < ActiveSet.Batches.Count; ++i)
                    {
                        if (!batchHasAnyIntegrationResponsibilities[i])
                            continue;
                        ref var batch = ref ActiveSet.Batches[i];
                        for (int j = 0; j < batch.TypeBatches.Count; ++j)
                        {
                            ref var typeBatch = ref batch.TypeBatches[j];
                            coarseBatchIntegrationResponsibilities[i][j] = ComputeIntegrationResponsibilitiesForConstraintRegion(i, j, 0, typeBatch.ConstraintCount);
                        }
                    }
                }
                //var end = Stopwatch.GetTimestamp();
                //Console.WriteLine($"time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
                pool.Return(ref batchHasAnyIntegrationResponsibilities);

                ////Validation:
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
                //        if (location.BatchIndex > 0)
                //        {
                //            ref var indexSet = ref integrationFlags[location.BatchIndex][typeBatchIndex][minimumIndexInConstraint];
                //            Debug.Assert(indexSet.Contains(location.IndexInTypeBatch));
                //        }
                //    }
                //}

                Debug.Assert(!bodiesFirstObservedInBatches[0].Flags.Allocated, "Remember, we're assuming we're just leaving the first batch's slot empty to avoid indexing complexity.");
                for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
                {
                    bodiesFirstObservedInBatches[batchIndex].Dispose(pool);
                }
                pool.Return(ref bodiesFirstObservedInBatches);
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

        public override void SolveStep2(float totalDt, IThreadDispatcher threadDispatcher = null)
        {
            var substepDt = totalDt / substepCount;
            if (threadDispatcher == null)
            {
                var inverseDt = 1f / substepDt;
                ref var activeSet = ref ActiveSet;
                GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
                Debug.Assert(!fallbackExists, "Not handling this yet.");

                var incrementalUpdateFilter = default(IncrementalContactDataUpdateFilter);
                for (int substepIndex = 0; substepIndex < substepCount; ++substepIndex)
                {
                    if (substepIndex > 0)
                    {
                        for (int i = 0; i < ActiveSet.Batches.Count; ++i)
                        {
                            ref var batch = ref activeSet.Batches[i];
                            for (int j = 0; j < batch.TypeBatches.Count; ++j)
                            {
                                ref var typeBatch = ref batch.TypeBatches[j];
                                if (incrementalUpdateFilter.AllowType(typeBatch.TypeId))
                                    TypeProcessors[typeBatch.TypeId].IncrementallyUpdateContactData(ref typeBatch, bodies, substepDt, inverseDt, 0, typeBatch.BundleCount);
                            }
                        }
                    }
                    for (int i = 0; i < synchronizedBatchCount; ++i)
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

                    for (int iterationIndex = 0; iterationIndex < IterationCount; ++iterationIndex)
                    {
                        for (int i = 0; i < synchronizedBatchCount; ++i)
                        {
                            ref var batch = ref activeSet.Batches[i];
                            for (int j = 0; j < batch.TypeBatches.Count; ++j)
                            {
                                ref var typeBatch = ref batch.TypeBatches[j];
                                TypeProcessors[typeBatch.TypeId].SolveStep2(ref typeBatch, bodies, substepDt, inverseDt, 0, typeBatch.BundleCount);
                            }
                        }
                    }
                }
            }
            else
            {
                //ExecuteMultithreaded<MainSolveFilter>(substepDt, threadDispatcher, solveStep2Worker, includeIncrementalUpdate: true);
                ExecuteMultithreaded2(substepDt, threadDispatcher, solveStep2Worker2);
            }
        }
    }
}
