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
        //Split the solve process into a warmstart and solve, where warmstart doesn't try to store out anything. It just computes jacobians and modifies velocities according to the accumulated impulse.
        //The solve step then *recomputes* jacobians from prestep data and pose information.
        //Why? Memory bandwidth. Redoing the calculation is cheaper than storing it out.
        struct WarmStartStep2StageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex)
            {
                ref var block = ref solver.context.ConstraintBlocks.Blocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                typeProcessor.WarmStart2(ref typeBatch, solver.bodies, Dt, InverseDt, block.StartBundle, block.End);
            }
        }
        //no fallback warmstart; the last constraint batch is always handled by the solve instead, and if the fallback batch exists, it's guaranteed to be the last batch.

        struct SolveStep2StageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex)
            {
                ref var block = ref solver.context.ConstraintBlocks.Blocks[blockIndex];
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

        Action<int> solveStep2Worker;


        void SolveStep2Worker(int workerIndex)
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
            //Note that every batch has a different start position. Each covers a different subset of constraints, so they require different start locations.
            //The same concept applies to the prestep- the prestep covers all constraints at once, rather than batch by batch.
            Debug.Assert(activeSet.Batches.Count > 0, "Don't dispatch if there are no constraints.");
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);

            var warmstartStage = new WarmStartStep2StageFunction();
            warmstartStage.Dt = context.Dt;
            warmstartStage.InverseDt = 1f / context.Dt;
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                var batchOffset = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                ExecuteStage(ref warmstartStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchOffset, context.BatchBoundaries[batchIndex],
                    ref batchStarts[batchIndex], ref syncStage, claimedState, unclaimedState);
            }
            claimedState ^= 1;
            unclaimedState ^= 1;
            var solveStage = new SolveStep2StageFunction();
            solveStage.Dt = context.Dt;
            solveStage.InverseDt = 1f / context.Dt;
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                var batchOffset = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                ExecuteStage(ref solveStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchOffset, context.BatchBoundaries[batchIndex],
                    ref batchStarts[batchIndex], ref syncStage, claimedState, unclaimedState);
            }
            //if (fallbackExists)
            //{
            //    var solveFallbackStage = new FallbackSolveStepStageFunction();
            //    var fallbackScatterStage = new FallbackScatterStageFunction();
            //    var batchOffset = FallbackBatchThreshold > 0 ? context.BatchBoundaries[FallbackBatchThreshold - 1] : 0;
            //    ExecuteStage(ref solveFallbackStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchOffset, context.BatchBoundaries[FallbackBatchThreshold],
            //        ref batchStarts[FallbackBatchThreshold], ref syncStage, claimedState, unclaimedState);
            //    ExecuteStage(ref fallbackScatterStage, ref context.FallbackBlocks, ref bounds, ref boundsBackBuffer,
            //        workerIndex, 0, context.FallbackBlocks.Blocks.Count, ref fallbackStart, ref syncStage, claimedState, unclaimedState);
            //}
            claimedState ^= 1;
            unclaimedState ^= 1;
        }

        public void SolveStep2(float dt, IThreadDispatcher threadDispatcher = null)
        {
            if (threadDispatcher == null)
            {
                var inverseDt = 1f / dt;
                ref var activeSet = ref ActiveSet;
                GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
                Debug.Assert(!fallbackExists, "Not handling this yet.");

                for (int i = 0; i < synchronizedBatchCount; ++i)
                {
                    ref var batch = ref activeSet.Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].WarmStart2(ref typeBatch, bodies, dt, inverseDt, 0, typeBatch.BundleCount);
                    }
                }

                for (int i = 0; i < synchronizedBatchCount; ++i)
                {
                    ref var batch = ref activeSet.Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].SolveStep2(ref typeBatch, bodies, dt, inverseDt, 0, typeBatch.BundleCount);
                    }
                }
            }
            else
            {
                ExecuteMultithreaded<MainSolveFilter>(dt, threadDispatcher, solveStep2Worker);
            }
        }

    }
}
