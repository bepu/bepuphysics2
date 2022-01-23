using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics
{
    /// <summary>
    /// Handles the movement of constraints from higher indexed batches into lower indexed batches to avoid accumulating a bunch of unnecessary ConstraintBatches.
    /// </summary>
    public class BatchCompressor
    {
        //We want to keep removes as fast as possible. So, when removing constraints, no attempt is made to pull constraints from higher constraint batches into the revealed slot.
        //Over time, this could result in lots of extra constraint batches that ruin multithreading performance.
        //This batch compressor solves this problem over multiple frames.
        //The dedicated batch analysis has some pretty nice advantages:
        //0) Removes stay (relatively) fast- no O(n) searching or complex logic.
        //1) High churn adds/removes are extremely common during chaotic collisions, which is exactly when you need as little overhead as possible. 
        //1.5) High churn situations will tend to rapidly invalidate the 'optimization' effort of extremely aggressive on-remove swaps.
        //2) On-removal will often fail to make any change due to other reference blockages.
        //3) Dedicated batch analysis can be deferred over multiple frames because the intermediate results are all fine from a correctness standpoint. 
        //3.5) Deferred costs can be kept consistently low no matter what kind of add/remove churn is happening.
        //4) Dedicated batch analysis can be performed asynchronously and hidden behind other stally stages which aren't the solver and don't modify the solver (e.g. broadphase, midphase).
        //5) Even if we are in a 'suboptimal' constraint configuration (i.e. some pulldowns exist), it will rarely have an effect on performance unless it actually results in extra batches.
        //6) Dedicated analysis could afford to perform more complex heuristics to optimize batches. This doesn't do anything clever, but in theory, it could.

        //Note that this is trying to solve an NP hard problem (graph edge coloring) with a strictly greedy approach, so it won't produce optimal results.
        //(In fact, it could use almost twice as many batches as would be optimal!)
        //There's probably an argument to be made for a slightly more clever approach later on- consider that the BVH SAH optimization is NP hard too, but the 
        //broadphase's incremental refinement does pretty well (and quite a bit better than pure greedy approaches). It is likely we can create something 
        //similar here. Some possibilities:
        //1) Grab a random set of N constraints from each batch, speculatively remove all of them and 'rebatch' them with a more expensive local
        //optimizer. The rebatching would have to take into account all the other bodies represented in the batches at each level. There is no guarantee this
        //would make progress unless the 'rebatching' algorithm was provably better than greedy and you had a sufficiently large rebatching region.
        //2) Claim a region of the constraint graph by traversing it. Execute misra & gries or similar not-exponential offline operations on it.
        //3) Maybe claim a smaller region and use an optimal (exponential) algorithm. I suspect covering a large region would work better.
        //This would be similar to treelet rotations in BVH refinement.

        //(Keep in mind that the difference between an optimal coloring and the greedy coloring is pretty small as far as solve time goes.
        //Expect less than 5% impact. In other words, it's not worth spending a month on a research project when there's so many other options to pursue.)

        public Solver Solver { get; private set; }
        public Bodies Bodies { get; private set; }
        float targetCandidateFraction;
        float maximumCompressionFraction;
        /// <summary>
        /// Gets or sets the desired number of candidates to analyze as a fraction of the total number of constraints.
        /// </summary>
        public float TargetCandidateFraction
        {
            get { return targetCandidateFraction; }
            set
            {
                if (value < 0 || value > 1) throw new ArgumentException("Fraction must be from 0 to 1.");
                targetCandidateFraction = value;
            }
        }
        /// <summary>
        /// Gets or sets the maximum number of constraint moves that can occur in a single execution of Compress as a fraction of the total number of constraints.
        /// </summary>       
        public float MaximumCompressionFraction
        {
            get { return maximumCompressionFraction; }
            set
            {
                if (value < 0 || value > 1) throw new ArgumentException("Fraction must be from 0 to 1.");
                maximumCompressionFraction = value;
            }
        }
        struct Compression
        {
            public ConstraintHandle ConstraintHandle;
            public int TargetBatch;
        }
        struct AnalysisRegion
        {
            public int TypeBatchIndex;
            public int StartIndexInTypeBatch;
            public int EndIndexInTypeBatch;
        }
        /// <summary>
        /// Index of the constraint batch to optimize.
        /// </summary>
        int nextBatchIndex;
        int nextTypeBatchIndex;

        //Note that these lists do not contain any valid information between frames- they only exist for the duration of the optimization.
        //Note that we allocate all of the contexts and their compressions on demand. It's all pointer backed, so there's no worries about GC reference tracing.
        Buffer<QuickList<Compression>> workerCompressions;
        IThreadDispatcher threadDispatcher;
        int analysisJobIndex;
        QuickList<AnalysisRegion> analysisJobs;


        Action<int> analysisWorkerDelegate;
        public BatchCompressor(Solver solver, Bodies bodies, float targetCandidateFraction = 0.005f, float maximumCompressionFraction = 0.0005f)
        {
            this.Solver = solver;
            this.Bodies = bodies;
            TargetCandidateFraction = targetCandidateFraction;
            this.MaximumCompressionFraction = maximumCompressionFraction;
            analysisWorkerDelegate = AnalysisWorker;
        }



        void AnalysisWorker(int workerIndex)
        {
            int jobIndex;
            while ((jobIndex = Interlocked.Increment(ref analysisJobIndex)) < analysisJobs.Count)
            {
                DoJob(ref analysisJobs[jobIndex], workerIndex, threadDispatcher.GetThreadMemoryPool(workerIndex));
            }
        }


        //Note that we split the find and apply stages conceptually just because there is a decent chance that they'll be scheduled at different times.
        //That is, while you can run the analysis phase in parallel, you can't run the application in parallel. So, if you can hide the sequential application in an unrelated
        //phase, it could be a net win. However, oversubscribing a parallel analysis phase on top of another parallel phase might not be quite as wise.
        //It'll just require some testing.
        //(The broad phase is a pretty likely candidate for this overlay- it both causes no changes in constraints and is very stally compared to most other phases.)


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void TryToFindBetterBatchForConstraint(
            BufferPool pool, ref QuickList<Compression> compressions, ref TypeBatch typeBatch, int* bodyHandles, ref ActiveConstraintDynamicBodyHandleCollector handleAccumulator, TypeProcessor typeProcessor, int constraintIndex)
        {
            handleAccumulator.Count = 0;
            Solver.EnumerateConnectedRawBodyReferences(ref typeBatch, constraintIndex, ref handleAccumulator);
            var dynamicBodyHandles = new Span<int>(bodyHandles, handleAccumulator.Count);
            for (int batchIndex = nextBatchIndex - 1; batchIndex >= 0; --batchIndex)
            {
                if (Solver.batchReferencedHandles[batchIndex].CanFit(dynamicBodyHandles))
                {
                    compressions.Add(new Compression { ConstraintHandle = typeBatch.IndexToHandle[constraintIndex], TargetBatch = batchIndex }, pool);
                    return;
                }
            }

        }


        unsafe void DoJob(ref AnalysisRegion region, int workerIndex, BufferPool pool)
        {
            ref var compressions = ref this.workerCompressions[workerIndex];
            ref var batch = ref Solver.ActiveSet.Batches[nextBatchIndex];
            var typeBatchIndex = region.TypeBatchIndex;
            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
            var typeProcessor = Solver.TypeProcessors[typeBatch.TypeId];

            //Each job only works on a subset of a single type batch.
            var bodiesPerConstraint = typeProcessor.BodiesPerConstraint;
            var bodyHandles = stackalloc int[bodiesPerConstraint];
            ActiveConstraintDynamicBodyHandleCollector handleAccumulator;
            handleAccumulator.Bodies = Bodies;
            handleAccumulator.Handles = bodyHandles;
            handleAccumulator.Count = 0;
            if (nextBatchIndex == Solver.FallbackBatchThreshold)
            {
                for (int i = region.StartIndexInTypeBatch; i < region.EndIndexInTypeBatch; ++i)
                {
                    //This is a fallback batch; the rules are a little different.
                    //Not all constraint slots up to the typeBatch.ConstraintCount are guaranteed to actually exist. It's potentially sparse.
                    //Just skip them.
                    if (typeBatch.IndexToHandle[i].Value >= 0)
                        TryToFindBetterBatchForConstraint(pool, ref compressions, ref typeBatch, bodyHandles, ref handleAccumulator, typeProcessor, i);
                }
            }
            else
            {
                for (int i = region.StartIndexInTypeBatch; i < region.EndIndexInTypeBatch; ++i)
                {
                    TryToFindBetterBatchForConstraint(pool, ref compressions, ref typeBatch, bodyHandles, ref handleAccumulator, typeProcessor, i);
                }
            }
        }


        struct CompressionTarget
        {
            public ushort WorkerIndex;
            public ushort Index;
            public ConstraintHandle ConstraintHandle;
        }
        struct CompressionComparer : IComparerRef<CompressionTarget>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref CompressionTarget a, ref CompressionTarget b)
            {
                return a.ConstraintHandle.Value.CompareTo(b.ConstraintHandle.Value);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void ApplyCompression(int sourceBatchIndex, ref ConstraintBatch sourceBatch, ref Compression compression)
        {
            var constraintLocation = Solver.HandleToConstraint[compression.ConstraintHandle.Value];
            var typeProcessor = Solver.TypeProcessors[constraintLocation.TypeId];
            if (sourceBatchIndex == Solver.FallbackBatchThreshold)
            {
                //We're optimizing the fallback batch, so we need to be careful about compressions interfering with each other. The parallel analysis assumed each batch
                //contained at most one instance of each body, which doesn't hold for the fallback batch.
                //Easy enough to address: check to see if the target batch can still hold the constraint.
                var dynamicBodyHandles = stackalloc int[typeProcessor.BodiesPerConstraint];
                ActiveConstraintDynamicBodyHandleCollector handleAccumulator;
                handleAccumulator.Bodies = Bodies;
                handleAccumulator.Handles = dynamicBodyHandles;
                handleAccumulator.Count = 0;
                Solver.EnumerateConnectedRawBodyReferences(compression.ConstraintHandle, ref handleAccumulator);
                if (!Solver.batchReferencedHandles[compression.TargetBatch].CanFit(new Span<int>(dynamicBodyHandles, handleAccumulator.Count)))
                {
                    //Another compression from the fallback batch has blocked this compression.
                    //Note that this isn't really a problem- batch compression is an incremental process. If some other compression was possible, a future frame will find it pretty quickly.
                    return;
                }
            }
            //Console.WriteLine($"Compressing: {compression.ConstraintHandle} moving from {Solver.Batches.IndexOf(sourceBatch)} to {compression.TargetBatch}");

            //Note that we do not simply remove and re-add the constraint; while that would work, it would redo a lot of work that isn't necessary.
            //Instead, since we already know exactly where the constraint is and what constraint batch it should go to, we can avoid a lot of abstractions
            //and do more direct copies.
            Solver.TypeProcessors[constraintLocation.TypeId].TransferConstraint(
                ref sourceBatch.GetTypeBatch(constraintLocation.TypeId), nextBatchIndex, constraintLocation.IndexInTypeBatch, Solver, Bodies, compression.TargetBatch);
        }

        /// <summary>
        /// Incrementally finds and applies a set of compressions to apply to the constraints in the solver's batches.
        /// Constraints in higher index batches try to move to lower index batches whenever possible.
        /// </summary>
        public void Compress(BufferPool pool, IThreadDispatcher threadDispatcher = null, bool deterministic = false)
        {
            var workerCount = threadDispatcher != null ? threadDispatcher.ThreadCount : 1;
            var constraintCount = Solver.ActiveSet.ConstraintCount;
            //Early out if there are no constraints to compress. The existence of constraints is assumed in some of the subsequent stages, so this is not merely an optimization.
            if (constraintCount == 0)
                return;
            var maximumCompressionCount = (int)Math.Max(1, Math.Round(MaximumCompressionFraction * constraintCount));
            var targetCandidateCount = (int)Math.Max(1, Math.Round(TargetCandidateFraction * constraintCount));

            pool.Take(workerCount, out workerCompressions);
            for (int i = 0; i < workerCount; ++i)
            {
                //Be careful: the jobs may require resizes on the compression count list. That requires the use of per-worker pools.
                workerCompressions[i] = new QuickList<Compression>(Math.Max(8, maximumCompressionCount), threadDispatcher == null ? pool : threadDispatcher.GetThreadMemoryPool(i));
            }

            //In any given compression attempt, we only optimize over one ConstraintBatch.
            //This provides a guarantee that every optimization that occurs over the course of the compression
            //does not affect any other optimization, because a ConstraintBatch guarantees that bodies are only referenced by a single constraint.
            //That's useful when multithreading- we don't have to worry about what candidates other threads have found.
            //The exception is the fallback batch; if we're currently optimizing the fallback batch, we must protect against multiple compressions interfering with one another.

            //Note that the application of compression is sequential. Solver.Add and Solver.Remove can't be called from multiple threads. So when multithreading,
            //only the candidate analysis is actually multithreaded. That's fine- actual compressions are actually pretty rare in nonpathological cases!

            //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
            Debug.Assert(Solver.ActiveSet.Batches.Count > 0);
            if (nextBatchIndex >= Solver.ActiveSet.Batches.Count)
            {
                //Invalid batch; wrap to the first one.
                nextBatchIndex = 0;
                nextTypeBatchIndex = 0;
            }
            //Note that we must handle the case where a batch has zero type batches (because we haven't compressed it yet!).
            while (nextTypeBatchIndex >= Solver.ActiveSet.Batches[nextBatchIndex].TypeBatches.Count)
            {
                //Invalid type batch; move to the next batch.
                ++nextBatchIndex;
                if (nextBatchIndex >= Solver.ActiveSet.Batches.Count)
                    nextBatchIndex = 0;
                nextTypeBatchIndex = 0;
            }
            //Console.WriteLine($"start: batch {nextBatchIndex}, type batch {nextTarget.TypeBatchIndex}, index {nextTarget.StartIndexInTypeBatch}");

            //Build the analysis regions.
            ref var batch = ref Solver.ActiveSet.Batches[nextBatchIndex];

            //Just make a generous estimate as to the number of jobs we'll need. 512 is huge in context, but trivial in terms of ephemeral memory required.
            analysisJobs = new QuickList<AnalysisRegion>(512, pool);

            //Jobs are created as subsets of type batches. Note that we never leave a type batch partially covered. This helps with determinism-
            //if we detect all compressions required within the entire type batch, the compressions list won't be sensitive to the order of constraints in memory within that type batch.
            //While we could relax this for a nondeterministic path, this phase is too cheap to bother with a bunch of custom logic.
            const int targetConstraintsPerJob = 64;

            int totalConstraintsScheduled = 0;
            for (; nextTypeBatchIndex < batch.TypeBatches.Count && totalConstraintsScheduled < targetCandidateCount; ++nextTypeBatchIndex)
            {
                ref var typeBatch = ref batch.TypeBatches[nextTypeBatchIndex];
                var jobCount = 1 + typeBatch.ConstraintCount / targetConstraintsPerJob;
                var baseConstraintsPerJob = typeBatch.ConstraintCount / jobCount;
                var remainder = typeBatch.ConstraintCount - baseConstraintsPerJob * jobCount;

                var previousEnd = 0;
                analysisJobs.EnsureCapacity(analysisJobs.Count + jobCount, pool);
                for (int j = 0; j < jobCount; ++j)
                {
                    var constraintsInJob = j < remainder ? baseConstraintsPerJob + 1 : baseConstraintsPerJob;
                    ref var job = ref analysisJobs.AllocateUnsafely();
                    job.TypeBatchIndex = nextTypeBatchIndex;
                    job.StartIndexInTypeBatch = previousEnd;
                    previousEnd += constraintsInJob;
                    job.EndIndexInTypeBatch = previousEnd;
                }

                totalConstraintsScheduled += typeBatch.ConstraintCount;
            }

            //var analyzeStart = Stopwatch.GetTimestamp();
            if (threadDispatcher != null)
            {
                analysisJobIndex = -1;
                this.threadDispatcher = threadDispatcher;
                threadDispatcher.DispatchWorkers(analysisWorkerDelegate, analysisJobs.Count);
                this.threadDispatcher = null;
            }
            else
            {
                for (int i = 0; i < analysisJobs.Count; ++i)
                {
                    DoJob(ref analysisJobs[i], 0, pool);
                }
            }
            //var analyzeEnd = Stopwatch.GetTimestamp();

            analysisJobs.Dispose(pool);

            ref var sourceBatch = ref Solver.ActiveSet.Batches[nextBatchIndex];
            int compressionsApplied = 0;

            //var applyStart = Stopwatch.GetTimestamp();
            if (deterministic)
            {
                //In deterministic mode, we must first sort the compressions found by every thread.
                //Sorting by constraint handle gives a unique order regardless of memory layout.
                //When combined with the analysis covering all or none of a type batch, the batch compressor becomes insensitive to memory layouts and is deterministic.
                var totalCompressionCount = 0;
                for (int i = 0; i < workerCount; ++i)
                {
                    totalCompressionCount += workerCompressions[i].Count;
                }
                if (totalCompressionCount > 0)
                {
                    var compressionsToSort = new QuickList<CompressionTarget>(totalCompressionCount, pool);

                    for (int i = 0; i < workerCount; ++i)
                    {
                        ref var compressions = ref workerCompressions[i];
                        for (int j = 0; j < compressions.Count; ++j)
                        {
                            ref var target = ref compressionsToSort.AllocateUnsafely();
                            target.WorkerIndex = (ushort)i;
                            target.Index = (ushort)j;
                            //Note that we precache the handle here rather than continually running off to pull data from the source lists. Doesn't matter much for performance in context-
                            //there are never many compressions- but it does simplify things a little.
                            target.ConstraintHandle = compressions[j].ConstraintHandle;
                        }
                    }

                    var comparer = new CompressionComparer();
                    QuickSort.Sort(ref compressionsToSort[0], 0, compressionsToSort.Count - 1, ref comparer);

                    //Now that they're sorted, we can go back and apply some of them.
                    //We can stop early- since this isn't sensitive to memory layout and we'll eventually swing back around to these type batches eventually, skipping some compressions
                    //occasionally doesn't have any long term harm.
                    for (int i = 0; i < compressionsToSort.Count && i < maximumCompressionCount; ++i)
                    {
                        ref var target = ref compressionsToSort[i];
                        ApplyCompression(nextBatchIndex, ref sourceBatch, ref workerCompressions[target.WorkerIndex][target.Index]);
                    }

                    compressionsToSort.Dispose(pool);
                }

            }
            else
            {
                //In nondeterministic mode, we can just walk through the worker results in any order.
                for (int i = workerCount - 1; i >= 0 && compressionsApplied < maximumCompressionCount; --i)
                {
                    ref var compressions = ref workerCompressions[i];
                    for (int j = compressions.Count - 1; j >= 0 && compressionsApplied < maximumCompressionCount; --j)
                    {
                        ApplyCompression(nextBatchIndex, ref sourceBatch, ref compressions[j]);
                        ++compressionsApplied;
                    }

                }
            }
            //var applyEnd = Stopwatch.GetTimestamp();


            //Console.WriteLine($"Batch count: {Solver.Batches.Count}, compression count: {compressionsApplied}, candidates analyzed: {totalConstraintsScheduled}");
            //var analyzeTime = 1e6 * (analyzeEnd - analyzeStart) / Stopwatch.Frequency;
            //Console.WriteLine($"Analyze time (us): {analyzeTime}, per constraint scheduled (us): {analyzeTime / totalConstraintsScheduled}");
            //var applyTime = 1e6 * (applyEnd - applyStart) / Stopwatch.Frequency;
            //Console.WriteLine($"Apply time (us): {applyTime}, per applied: {applyTime / compressionsApplied}, (maximum: {maximumCompressionCount})");

            for (int i = 0; i < workerCount; ++i)
            {
                //Be careful: the jobs may require resizes on the compression count list. That requires the use of per-worker pools.
                workerCompressions[i].Dispose((threadDispatcher == null ? pool : threadDispatcher.GetThreadMemoryPool(i)));
            }
            pool.Return(ref workerCompressions);
        }


    }
}
