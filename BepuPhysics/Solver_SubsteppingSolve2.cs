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
            public void Execute(Solver solver, int blockIndex, int workerIndex)
            {
                ref var block = ref this.solver.context.ConstraintBlocks.Blocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                if (block.BatchIndex == 0)
                {
                    Buffer<IndexSet> noFlagsRequired = default;
                    typeProcessor.WarmStart2<TIntegrationCallbacks, BatchShouldAlwaysIntegrate>(
                        ref typeBatch, ref noFlagsRequired, this.solver.bodies, ref this.solver.PoseIntegrator.Callbacks,
                        Dt, InverseDt, block.StartBundle, block.End, workerIndex);
                }
                else
                {
                    if (this.solver.coarseBatchIntegrationResponsibilities[block.BatchIndex][block.TypeBatchIndex])
                    {
                        typeProcessor.WarmStart2<TIntegrationCallbacks, BatchShouldConditionallyIntegrate>(
                            ref typeBatch, ref this.solver.integrationFlags[block.BatchIndex][block.TypeBatchIndex], this.solver.bodies, ref this.solver.PoseIntegrator.Callbacks,
                            Dt, InverseDt, block.StartBundle, block.End, workerIndex);
                    }
                    else
                    {
                        typeProcessor.WarmStart2<TIntegrationCallbacks, BatchShouldNeverIntegrate>(
                            ref typeBatch, ref this.solver.integrationFlags[block.BatchIndex][block.TypeBatchIndex], this.solver.bodies, ref this.solver.PoseIntegrator.Callbacks,
                            Dt, InverseDt, block.StartBundle, block.End, workerIndex);
                    }
                }
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
        /// <summary>
        /// Caches a single bool for whether type batches within batches have constraints with any integration responsibilities.
        /// Type batches with no integration responsibilities can use a codepath with no integration checks at all.
        /// </summary>
        Buffer<Buffer<bool>> coarseBatchIntegrationResponsibilities;

        public override unsafe void PrepareConstraintIntegrationResponsibilities()
        {
            //var start = Stopwatch.GetTimestamp();
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
            //Console.WriteLine($"body count: {bodies.ActiveSet.Count}, constraint count: {CountConstraints()}");
            //var end = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Brute force time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");

            pool.Take<IndexSet>(batchReferencedHandles.Count, out var bodiesFirstObservedInBatches);
            IndexSet merged;
            //We don't have to consider the first batch, since we know ahead of time that the first batch will be the first time we see any bodies in it.
            //Just copy directly from the first batch into the merged to initialize it.
            pool.Take((bodies.HandlePool.HighestPossiblyClaimedId + 63) / 64, out merged.Flags);
            var copyLength = Math.Min(merged.Flags.Length, batchReferencedHandles[0].Flags.Length);
            batchReferencedHandles[0].Flags.CopyTo(0, merged.Flags, 0, copyLength);
            batchReferencedHandles[0].Flags.Clear(copyLength, batchReferencedHandles[0].Flags.Length - copyLength);

            //Yup, we're just leaving the first slot unallocated to avoid having to offset indices all over the place. Slight wonk, but not a big deal.
            bodiesFirstObservedInBatches[0] = default;
            pool.Take<bool>(batchReferencedHandles.Count, out var batchHasAnyIntegrationResponsibilities);
            for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
            {
                ref var batchHandles = ref batchReferencedHandles[batchIndex];
                var bundleCount = Math.Min(merged.Flags.Length, batchHandles.Flags.Length);
                //Note that we bypass the constructor to avoid zeroing unnecessarily. Every bundle will be fully assigned.
                pool.Take(bundleCount, out bodiesFirstObservedInBatches[batchIndex].Flags);
            }
            for (int batchIndex = 1; batchIndex < ActiveSet.Batches.Count; ++batchIndex)
            {
                ref var batchHandles = ref batchReferencedHandles[batchIndex];
                ref var firstObservedInBatch = ref bodiesFirstObservedInBatches[batchIndex];
                var bundleCount = Math.Min(merged.Flags.Length, batchHandles.Flags.Length);
                ulong horizontalMerge = 0;
                for (int flagBundleIndex = 0; flagBundleIndex < bundleCount; ++flagBundleIndex)
                {
                    var mergeBundle = merged.Flags[flagBundleIndex];
                    var batchBundle = batchHandles.Flags[flagBundleIndex];
                    merged.Flags[flagBundleIndex] = mergeBundle | batchBundle;
                    //If this batch contains a body, and the merged set does not, then it's the first batch that sees a body and it will have integration responsibility.
                    var firstObservedBundle = ~mergeBundle & batchBundle;
                    horizontalMerge |= firstObservedBundle;
                    firstObservedInBatch.Flags[flagBundleIndex] = firstObservedBundle;
                }
                batchHasAnyIntegrationResponsibilities[batchIndex] = horizontalMerge != 0;
            }
            var start = Stopwatch.GetTimestamp();
            //We now have index sets representing the first time each body handle is observed in a batch.
            for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
            {
                if (!batchHasAnyIntegrationResponsibilities[batchIndex])
                    continue;
                ref var integrationFlagsForBatch = ref integrationFlags[batchIndex];
                ref var firstObservedForBatch = ref bodiesFirstObservedInBatches[batchIndex];
                ref var batch = ref ActiveSet.Batches[batchIndex];

                //ulong totalConstraintCount = 0;
                //ulong integratingConstraintCount = 0;
                //ulong totalLanes = 0;
                //ulong integratingLanes = 0;

                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var integrationFlagsForTypeBatch = ref integrationFlagsForBatch[typeBatchIndex];
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    var typeBatchBodyReferences = typeBatch.BodyReferences.As<int>();
                    var bodiesPerConstraintInTypeBatch = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    var intsPerBundle = Vector<int>.Count * bodiesPerConstraintInTypeBatch;
                    for (int bundleIndex = 0; bundleIndex < typeBatch.BundleCount; ++bundleIndex)
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
                    coarseBatchIntegrationResponsibilities[batchIndex][typeBatchIndex] = mergedFlagBundles != 0;

                    //for (int i = 0; i < flagBundleCount; ++i)
                    //{
                    //    ulong countMerge = 0;
                    //    for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraintInTypeBatch; ++bodyIndexInConstraint)
                    //    {
                    //        var flagsForBody = integrationFlagsForTypeBatch[bodyIndexInConstraint].Flags[i];
                    //        countMerge |= flagsForBody;
                    //        integratingLanes += Popcnt.X64.PopCount(flagsForBody);
                    //        totalLanes += (ulong)Math.Min(64, typeBatch.ConstraintCount - i * 64);
                    //    }
                    //    integratingConstraintCount += Popcnt.X64.PopCount(countMerge);
                    //    totalConstraintCount += (ulong)Math.Min(64, typeBatch.ConstraintCount - i * 64);
                    //}
                }
                //Console.WriteLine($"Batch {batchIndex} integrating constraints: {integratingConstraintCount} over {totalConstraintCount}, {integratingConstraintCount / (double)totalConstraintCount}");
                //Console.WriteLine($"Batch {batchIndex} integrating lanes:       {integratingLanes} over {totalLanes}, {integratingLanes / (double)totalLanes}");
            }
            pool.Return(ref batchHasAnyIntegrationResponsibilities);
            //for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
            //{
            //    ref var integrationFlagsForBatch = ref integrationFlags[batchIndex];
            //    ref var firstObservedForBatch = ref bodiesFirstObservedInBatches[batchIndex];
            //    ref var batch = ref ActiveSet.Batches[batchIndex];
            //    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
            //    {
            //        ref var integrationFlagsForTypeBatch = ref integrationFlagsForBatch[typeBatchIndex];
            //        ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
            //        var typeBatchBodyReferences = typeBatch.BodyReferences.As<int>();
            //        var bodiesPerConstraintInTypeBatch = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
            //        var intsPerBundle = Vector<int>.Count * bodiesPerConstraintInTypeBatch;
            //        //We process over one strip of bodies in the constraint in each pass, constructing contiguous flag sequences as we go.
            //        for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraintInTypeBatch; ++bodyIndexInConstraint)
            //        {
            //            ref var integrationFlagsForBodyInConstraint = ref integrationFlagsForTypeBatch[bodyIndexInConstraint];
            //            for (int flagBundleIndex = 0; flagBundleIndex < integrationFlagsForBodyInConstraint.Flags.Length; ++flagBundleIndex)
            //            {
            //                ulong flagBundle = 0;
            //                var constraintBundleStartIndexInConstraints = flagBundleIndex * 64;
            //                int countInFlagBundle = Math.Min(64, typeBatch.ConstraintCount - constraintBundleStartIndexInConstraints);
            //                for (int indexInFlagBundle = 0; indexInFlagBundle < countInFlagBundle; ++indexInFlagBundle)
            //                {
            //                    var constraintIndex = constraintBundleStartIndexInConstraints + indexInFlagBundle;
            //                    BundleIndexing.GetBundleIndices(constraintIndex, out var constraintBundleIndex, out var constraintIndexInBundle);
            //                    var bodyHandle = bodies.ActiveSet.IndexToHandle[typeBatchBodyReferences.Memory[constraintBundleIndex * intsPerBundle + Vector<int>.Count * bodyIndexInConstraint]].Value;
            //                    if (firstObservedForBatch.Contains(bodyHandle))
            //                    {
            //                        integrationFlagsForBodyInConstraint.AddUnsafely(constraintIndex);
            //                    }
            //                }
            //                integrationFlagsForBodyInConstraint.Flags[flagBundleIndex] = flagBundle;
            //            }
            //        }
            //    }
            //}
            var end = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
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

            merged.Dispose(pool);
            Debug.Assert(!bodiesFirstObservedInBatches[0].Flags.Allocated, "Remember, we're assuming we're just leaving the first batch's slot empty to avoid indexing complexity.");
            for (int batchIndex = 1; batchIndex < bodiesFirstObservedInBatches.Length; ++batchIndex)
            {
                bodiesFirstObservedInBatches[batchIndex].Dispose(pool);
            }
            pool.Return(ref bodiesFirstObservedInBatches);
        }
        public override void DisposeConstraintIntegrationResponsibilities()
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
                        if (i == 0)
                        {
                            Buffer<IndexSet> noFlagsRequired = default;
                            TypeProcessors[typeBatch.TypeId].WarmStart2<TIntegrationCallbacks, BatchShouldAlwaysIntegrate>(ref typeBatch, ref noFlagsRequired, bodies, ref PoseIntegrator.Callbacks,
                                dt, inverseDt, 0, typeBatch.BundleCount, 0);
                        }
                        else
                        {
                            if (coarseBatchIntegrationResponsibilities[i][j])
                            {
                                TypeProcessors[typeBatch.TypeId].WarmStart2<TIntegrationCallbacks, BatchShouldConditionallyIntegrate>(ref typeBatch, ref integrationFlagsForBatch[j], bodies, ref PoseIntegrator.Callbacks,
                                    dt, inverseDt, 0, typeBatch.BundleCount, 0);
                            }
                            else
                            {
                                TypeProcessors[typeBatch.TypeId].WarmStart2<TIntegrationCallbacks, BatchShouldNeverIntegrate>(ref typeBatch, ref integrationFlagsForBatch[j], bodies, ref PoseIntegrator.Callbacks,
                                    dt, inverseDt, 0, typeBatch.BundleCount, 0);
                            }
                        }
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
