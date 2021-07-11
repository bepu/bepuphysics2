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
        public virtual void PrepareConstraintIntegrationResponsibilities()
        {
        }
        public virtual void DisposeConstraintIntegrationResponsibilities()
        {
        }

        public virtual void SolveStep2(float dt, IThreadDispatcher threadDispatcher = null)
        {

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
            solveStep2Worker = SolveStep2Worker;
        }

        public PoseIntegrator<TIntegrationCallbacks> PoseIntegrator { get; private set; }

        //Split the solve process into a warmstart and solve, where warmstart doesn't try to store out anything. It just computes jacobians and modifies velocities according to the accumulated impulse.
        //The solve step then *recomputes* jacobians from prestep data and pose information.
        //Why? Memory bandwidth. Redoing the calculation is cheaper than storing it out.
        struct WarmStartStep2StageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;
            public Solver<TIntegrationCallbacks> solver;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex, int workerIndex )
            {
                ref var block = ref this.solver.context.ConstraintBlocks.Blocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                typeProcessor.WarmStart2(ref typeBatch, ref this.solver.integrationFlags[block.BatchIndex][block.TypeBatchIndex], this.solver.bodies, ref this.solver.PoseIntegrator.Callbacks, Dt, InverseDt, block.StartBundle, block.End, workerIndex);
            }
        }
        //no fallback warmstart; the last constraint batch is always handled by the solve instead, and if the fallback batch exists, it's guaranteed to be the last batch.

        struct SolveStep2StageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;
            public Solver<TIntegrationCallbacks> solver;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex, int workerIndex)
            {
                ref var block = ref this.solver.context.ConstraintBlocks.Blocks[blockIndex];
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
            int fallbackStart = GetUniformlyDistributedStart(workerIndex, context.FallbackBlocks.Blocks.Count, context.WorkerCount, 0);
            Buffer<int> batchStarts;
            ref var activeSet = ref ActiveSet;
            unsafe
            {
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

            var warmstartStage = new WarmStartStep2StageFunction
            {
                Dt = context.Dt,
                InverseDt = 1f / context.Dt,
                solver = this
            };
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                var batchOffset = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                ExecuteStage(ref warmstartStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, batchOffset, context.BatchBoundaries[batchIndex],
                    ref batchStarts[batchIndex], ref syncStage, claimedState, unclaimedState);
            }
            claimedState ^= 1;
            unclaimedState ^= 1;
            var solveStage = new SolveStep2StageFunction
            {
                Dt = context.Dt,
                InverseDt = 1f / context.Dt,
                solver = this
            };
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

        Buffer<Buffer<Buffer<IndexSet>>> integrationFlags;

        public override void PrepareConstraintIntegrationResponsibilities()
        {
            //var start = Stopwatch.GetTimestamp();
            pool.Take(ActiveSet.Batches.Count, out integrationFlags);
            for (int i = 0; i < integrationFlags.Length; ++i)
            {
                ref var batch = ref ActiveSet.Batches[i];
                ref var flagsForBatch = ref integrationFlags[i];
                pool.Take(batch.TypeBatches.Count, out flagsForBatch);
                for (int j = 0; j < flagsForBatch.Length; ++j)
                {
                    ref var flagsForTypeBatch = ref flagsForBatch[j];
                    ref var typeBatch = ref batch.TypeBatches[j];
                    var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    pool.Take(bodiesPerConstraint, out flagsForTypeBatch);
                    for (int k = 0; k < bodiesPerConstraint; ++k)
                    {
                        flagsForTypeBatch[k] = new IndexSet(pool, typeBatch.ConstraintCount);
                    }
                }
            }
            for (int i = 0; i < bodies.ActiveSet.Count; ++i)
            {
                ref var constraints = ref bodies.ActiveSet.Constraints[i];
                ConstraintHandle minimumConstraint;
                minimumConstraint.Value = -1;
                int minimumBatchIndex = int.MaxValue;
                int minimumIndexInConstraint = -1;
                for (int j = 0; j < constraints.Count; ++j)
                {
                    ref var constraint = ref constraints[j];
                    var batchIndex = HandleToConstraint[constraint.ConnectingConstraintHandle.Value].BatchIndex;
                    if (batchIndex < minimumBatchIndex)
                    {
                        minimumBatchIndex = batchIndex;
                        minimumIndexInConstraint = constraint.BodyIndexInConstraint;
                        minimumConstraint = constraint.ConnectingConstraintHandle;
                    }
                }
                if (minimumConstraint.Value >= 0)
                {
                    ref var location = ref HandleToConstraint[minimumConstraint.Value];
                    var typeBatchIndex = ActiveSet.Batches[location.BatchIndex].TypeIndexToTypeBatchIndex[location.TypeId];
                    ref var indexSet = ref integrationFlags[location.BatchIndex][typeBatchIndex][minimumIndexInConstraint];
                    indexSet.AddUnsafely(location.IndexInTypeBatch);
                }
            }
            //Console.WriteLine($"body count: {bodies.ActiveSet.Count}, constraint count: {CountConstraints()}");
            //var end = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Brute force time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
        }
        public override void DisposeConstraintIntegrationResponsibilities()
        {
            for (int i = 0; i < integrationFlags.Length; ++i)
            {
                ref var flagsForBatch = ref integrationFlags[i];
                for (int j = 0; j < flagsForBatch.Length; ++j)
                {
                    ref var flagsForTypeBatch = ref flagsForBatch[j];
                    for (int k = 0; k < flagsForTypeBatch.Length; ++k)
                    {
                        flagsForTypeBatch[k].Dispose(pool);
                    }
                    pool.Return(ref flagsForTypeBatch);
                }
                pool.Return(ref flagsForBatch);
            }
            pool.Return(ref integrationFlags);
        }

        public override void SolveStep2(float dt, IThreadDispatcher threadDispatcher = null)
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
                    ref var integrationFlagsForBatch = ref integrationFlags[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].WarmStart2(ref typeBatch, ref integrationFlagsForBatch[j], bodies, ref PoseIntegrator.Callbacks, dt, inverseDt, 0, typeBatch.BundleCount, 0);
                    }
                }

                for (int i = 0; i < synchronizedBatchCount; ++i)
                {
                    ref var batch = ref activeSet.Batches[i];
                    ref var integrationFlagsForBatch = ref integrationFlags[i];
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
