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

namespace BepuPhysics
{
    public partial class Solver
    {


        struct Prestep2StageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex)
            {
                ref var block = ref solver.context.ConstraintBlocks.Blocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                typeProcessor.Prestep2(ref typeBatch, solver.bodies, Dt, InverseDt, block.StartBundle, block.End);
            }
        }

        struct Prestep2FallbackStageFunction : IStageFunction
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex)
            {
                ref var block = ref solver.context.ConstraintBlocks.Blocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                //typeProcessor.JacobiPrestep2(ref typeBatch, ref solver.bodies.ActiveSet.Velocities, solver ref solver.context.FallbackResults[block.TypeBatchIndex], block.StartBundle, block.End);

            }
        }

        void Solve2Worker(int workerIndex)
        {
            int prestepStart = GetUniformlyDistributedStart(workerIndex, context.ConstraintBlocks.Blocks.Count, context.WorkerCount, 0);
            int fallbackStart = GetUniformlyDistributedStart(workerIndex, context.FallbackBlocks.Blocks.Count, context.WorkerCount, 0);
            Buffer<int> batchStarts;
            ref var activeSet = ref ActiveSet;
            unsafe
            {
                //stackalloc is actually a little bit slow since the localsinit behavior forces a zeroing.
                //Fortunately, this executes once per thread per frame. With 32 batches, it would add... a few nanoseconds per frame. We can accept that overhead.
                //This is preferred over preallocating on the heap- we might write to these values and we don't want to risk false sharing for no reason. 
                //A single instance of false sharing would cost far more than the overhead of zeroing out the array.
                var batchStartsData = stackalloc int[activeSet.Batches.Count];
                batchStarts = new Buffer<int>(batchStartsData, activeSet.Batches.Count);
            }
            for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
            {
                var batchOffset = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                var batchCount = context.BatchBoundaries[batchIndex] - batchOffset;
                batchStarts[batchIndex] = GetUniformlyDistributedStart(workerIndex, batchCount, context.WorkerCount, batchOffset);
            }


            int syncStage = 0;
            //The claimed and unclaimed state swap after every usage of both pingpong claims buffers.
            int claimedState = 1;
            int unclaimedState = 0;
            var bounds = context.WorkerBoundsA;
            var boundsBackBuffer = context.WorkerBoundsB;
            Debug.Assert(activeSet.Batches.Count > 0, "Don't dispatch if there are no constraints.");

            //Note that every batch has a different start position. Each covers a different subset of constraints, so they require different start locations.
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            //claimedState ^= 1;
            //unclaimedState ^= 1;
            var prestepStage = new Prestep2StageFunction { Dt = context.Dt, InverseDt = 1f / context.Dt };
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                var batchOffset = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                //Don't use the warm start to guess at the solve iteration work distribution.
                var workerBatchStartCopy = batchStarts[batchIndex];
                ExecuteStage(ref prestepStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchOffset, context.BatchBoundaries[batchIndex],
                    ref workerBatchStartCopy, ref syncStage, claimedState, unclaimedState);
            }
            var fallbackScatterStage = new FallbackScatterStageFunction();
            if (fallbackExists)
            {
                var prestepFallbackStage = new Prestep2FallbackStageFunction();
                var batchStart = FallbackBatchThreshold > 0 ? context.BatchBoundaries[FallbackBatchThreshold - 1] : 0;
                //Don't use the warm start to guess at the solve iteration work distribution.
                var workerBatchStartCopy = batchStarts[FallbackBatchThreshold];
                ExecuteStage(ref prestepFallbackStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchStart, context.BatchBoundaries[FallbackBatchThreshold],
                    ref workerBatchStartCopy, ref syncStage, claimedState, unclaimedState);
                ExecuteStage(ref fallbackScatterStage, ref context.FallbackBlocks, ref bounds, ref boundsBackBuffer,
                    workerIndex, 0, context.FallbackBlocks.Blocks.Count, ref fallbackStart, ref syncStage, unclaimedState, claimedState); //note claim state swap: fallback scatter claims have no prestep, so it's off by one cycle
            }
            claimedState ^= 1;
            unclaimedState ^= 1;

            var solveStage = new SolveStageFunction();
            var solveFallbackStage = new SolveFallbackStageFunction();
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                {
                    var batchOffset = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                    ExecuteStage(ref solveStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchOffset, context.BatchBoundaries[batchIndex],
                        ref batchStarts[batchIndex], ref syncStage, claimedState, unclaimedState);
                }
                if (fallbackExists)
                {
                    var batchOffset = FallbackBatchThreshold > 0 ? context.BatchBoundaries[FallbackBatchThreshold - 1] : 0;
                    ExecuteStage(ref solveFallbackStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchOffset, context.BatchBoundaries[FallbackBatchThreshold],
                        ref batchStarts[FallbackBatchThreshold], ref syncStage, claimedState, unclaimedState);
                    ExecuteStage(ref fallbackScatterStage, ref context.FallbackBlocks, ref bounds, ref boundsBackBuffer,
                        workerIndex, 0, context.FallbackBlocks.Blocks.Count, ref fallbackStart, ref syncStage, unclaimedState, claimedState); //note claim state swap: fallback scatter claims have no prestep, so it's off by one cycle
                }
                claimedState ^= 1;
                unclaimedState ^= 1;
            }
        }

        public void Solve2(float dt, IThreadDispatcher threadDispatcher = null)
        {
            if (threadDispatcher == null)
            {
                var inverseDt = 1f / dt;
                ref var activeSet = ref ActiveSet;
                GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
                //TODO: May want to consider executing warmstart immediately following the prestep. Multithreading can't do that, so there could be some bitwise differences introduced.
                //On the upside, it would make use of cached data.
                for (int i = 0; i < synchronizedBatchCount; ++i)
                {
                    ref var batch = ref activeSet.Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].Prestep2(ref typeBatch, bodies, dt, inverseDt, 0, typeBatch.BundleCount);
                    }
                }
                Buffer<FallbackTypeBatchResults> fallbackResults = default;
                if (fallbackExists)
                {
                    ref var batch = ref activeSet.Batches[FallbackBatchThreshold];
                    FallbackBatch.AllocateResults(this, pool, ref batch, out fallbackResults);
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].JacobiPrestep2(ref typeBatch, bodies, ref activeSet.Fallback, ref fallbackResults[j], dt, inverseDt, 0, typeBatch.BundleCount);
                    }
                    activeSet.Fallback.ScatterVelocities(bodies, this, ref fallbackResults, 0, activeSet.Fallback.BodyCount);
                }
                for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
                {
                    for (int i = 0; i < synchronizedBatchCount; ++i)
                    {
                        ref var batch = ref activeSet.Batches[i];
                        for (int j = 0; j < batch.TypeBatches.Count; ++j)
                        {
                            ref var typeBatch = ref batch.TypeBatches[j];
                            TypeProcessors[typeBatch.TypeId].SolveIteration(ref typeBatch, bodies, 0, typeBatch.BundleCount);
                        }
                    }
                    if (fallbackExists)
                    {
                        ref var batch = ref activeSet.Batches[FallbackBatchThreshold];
                        for (int j = 0; j < batch.TypeBatches.Count; ++j)
                        {
                            ref var typeBatch = ref batch.TypeBatches[j];
                            TypeProcessors[typeBatch.TypeId].JacobiSolveIteration(ref typeBatch, bodies, ref fallbackResults[j], 0, typeBatch.BundleCount);
                        }
                        activeSet.Fallback.ScatterVelocities(bodies, this, ref fallbackResults, 0, activeSet.Fallback.BodyCount);
                    }
                }
                if (fallbackExists)
                {
                    FallbackBatch.DisposeResults(this, pool, ref activeSet.Batches[FallbackBatchThreshold], ref fallbackResults);
                }
            }
            else
            {
                ExecuteMultithreaded<MainSolveFilter>(dt, threadDispatcher, solve2Worker);
            }
        }

    }
}
