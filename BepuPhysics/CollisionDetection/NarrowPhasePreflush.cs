using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace BepuPhysics.CollisionDetection
{
    internal enum PreflushJobType
    {
        /// <summary>
        /// Phase one job in the awakener. JobIndex used to identify sub-job.
        /// </summary>
        AwakenerPhaseOne,
        /// <summary>
        /// Sorts the constraints of a single type across all workers. Used by deterministic preflushes to schedule adds.
        /// Accesses no buffer pools; memory is allocated and returned on main thread.
        /// </summary>
        SortContactConstraintType,
        /// <summary>
        /// Identifies a first guess at the constraint batch to which every new constraint should be added to. 
        /// Accesses no buffer pools; memory is allocated and returned on main thread.
        /// </summary>
        SpeculativeConstraintBatchSearch,

        //The deterministic constraint add is split into two dispatches. The to-add sort, speculative batch search, and body list add all happen first.
        //The actual addition process occurs afterward alongside the freshness checker.
        //It's slower than the nondeterministic path, but that's the bullet to bite.

        /// <summary>
        /// Adds constraints to the solver and constraint graph in an order determined by the previous sorts and with the help of the speculatively computed batch targets. Locally sequential.
        /// Accesses main thread buffer pool when type batches are created or resized.
        /// </summary>
        DeterministicConstraintAdd,
        /// <summary>
        /// Adds constraints to the solver and constraint graph in an order determined by the collision detection phase. If the collision detection phase is nondeterministic due to threading, then 
        /// this will result in nondeterministic adds to the solver.
        /// Accesses main thread buffer pool when type batches are created or resized.
        /// </summary>
        NondeterministicConstraintAdd,
        /// <summary>
        /// Phase two job in the awakener. JobIndex used to identify sub-job.
        /// </summary>
        AwakenerPhaseTwo,
        /// <summary>
        /// Check the freshness bytes in a region to remove stale pairs.
        /// </summary>
        CheckFreshness,
    }

    [StructLayout(LayoutKind.Explicit)]
    internal struct PreflushJob
    {
        [FieldOffset(0)]
        public PreflushJobType Type;
        /// <summary>
        /// Start region of a CheckFreshness or SpeculativeConstraintBatchSearch job.
        /// </summary>
        [FieldOffset(4)]
        public int Start;
        /// <summary>
        /// End region of a CheckFreshness or SpeculativeConstraintBatchSearch job.
        /// </summary>
        [FieldOffset(8)]
        public int End;
        /// <summary>
        /// Narrow phase constraint type index targeted by a SpeculativeConstraintBatchSearch or SortContactConstraintType.
        /// </summary>
        [FieldOffset(12)]
        public int TypeIndex;
        /// <summary>
        /// Index of the worker in which a range of constraints starts. 
        /// Used by SpeculativeConstraintBatchSearch.
        /// </summary>
        [FieldOffset(16)]
        public int WorkerIndex;
        /// <summary>
        /// Number of worker threads containing constraints to read in the SortContactConstraintType and NondeterministicConstraintAdd tasks.
        /// </summary>
        [FieldOffset(16)]
        public int WorkerCount;
        /// <summary>
        /// Index of the job. Used by AwakenerPhaseOne and AwakenerPhaseTwo tasks.
        /// </summary>
        [FieldOffset(4)]
        public int JobIndex;
    }

    public partial class NarrowPhase<TCallbacks>
    {
        public struct SortConstraintTarget
        {
            public int WorkerIndex;
            public int ByteIndexInCache;
            //Note that we cache the handles as we do the initial pass to collect worker indices and byte indices.
            //While this does inflate the memory usage, note that 1024 pending constraints would only cost 16384 bytes. 
            //So extremely large simulations undergoing significant chaos may result in the sorting thread spilling out of L1, but virtually never L2.
            //This caching avoids the need to repeatedly pull in cache lines from the different worker sets. 
            //In chaotic large simulations, the poor cache line utilization (since we're only looking up 4-8 bytes) could result in spilling into L3.
            //As an added bonus, the access pattern during the initial pass is prefetchable, while a comparison-time lookup is not.
            public ulong SortKey;
        }
        Buffer<QuickList<SortConstraintTarget>> sortedConstraints;

        int preflushJobIndex;
        QuickList<PreflushJob> preflushJobs;
        Action<int> preflushWorkerLoop;
        void PreflushWorkerLoop(int workerIndex)
        {
            int jobIndex;
            while ((jobIndex = Interlocked.Increment(ref preflushJobIndex)) < preflushJobs.Count)
            {
                ExecutePreflushJob(workerIndex, ref preflushJobs[jobIndex]);
            }
        }

        struct PendingConstraintComparer : IComparerRef<SortConstraintTarget>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe int Compare(ref SortConstraintTarget a, ref SortConstraintTarget b)
            {
                return a.SortKey.CompareTo(b.SortKey);
            }
        }

        unsafe void BuildSortingTargets(ref QuickList<SortConstraintTarget> list, int typeIndex, int workerCount)
        {
            for (int i = 0; i < workerCount; ++i)
            {
                ref var workerList = ref overlapWorkers[i].PendingConstraints.pendingConstraintsByType[typeIndex];
                if (workerList.Count > 0)
                {
                    //This is doing redundant integer divides, but we lack type knowledge and it doesn't require any extra memory loading.
                    var entrySizeInBytes = workerList.ByteCount / workerList.Count;
                    int indexInBytes = 0;
                    for (int j = 0; j < workerList.Count; ++j)
                    {
                        ref var constraint = ref list.AllocateUnsafely();
                        constraint.WorkerIndex = i;
                        constraint.ByteIndexInCache = indexInBytes;
                        //Note two details:
                        //1) We rely on the layout of memory in the pending constraint add. If the collidable pair doesn't occupy the first 8 bytes, this breaks.
                        //2) We rely on the order of collidable pair references. The narrow phase should always guarantee a consistent order.
                        constraint.SortKey = *(ulong*)(workerList.Buffer.Memory + indexInBytes);
                        indexInBytes += entrySizeInBytes;
                    }
                }
            }
        }

        unsafe void ExecutePreflushJob(int workerIndex, ref PreflushJob job)
        {
            switch (job.Type)
            {
                case PreflushJobType.CheckFreshness:
                    FreshnessChecker.CheckFreshnessInRegion(workerIndex, job.Start, job.End);
                    break;
                case PreflushJobType.SortContactConstraintType:
                    {
                        //The main thread has already allocated lists of appropriate capacities for all types that exist.
                        //We initialize and sort those lists on multiple threads.
                        ref var list = ref sortedConstraints[job.TypeIndex];
                        BuildSortingTargets(ref list, job.TypeIndex, job.WorkerCount);

                        //Since duplicates are impossible (as that would imply the narrow phase generated two constraints for one pair), the non-threeway sort is used.
                        PendingConstraintComparer comparer;
                        QuickSort.Sort(ref list.Span[0], 0, list.Count - 1, ref comparer);
                    }
                    break;
                case PreflushJobType.SpeculativeConstraintBatchSearch:
                    {
                        overlapWorkers[job.WorkerIndex].PendingConstraints.SpeculativeConstraintBatchSearch(Solver, job.TypeIndex, job.Start, job.End);
                    }
                    break;
                case PreflushJobType.NondeterministicConstraintAdd:
                    {
                        for (int i = 0; i < job.WorkerCount; ++i)
                        {
                            overlapWorkers[i].PendingConstraints.FlushWithSpeculativeBatches(Simulation, ref PairCache);
                        }
                    }
                    break;
                case PreflushJobType.DeterministicConstraintAdd:
                    {
                        for (int typeIndex = 0; typeIndex < contactConstraintAccessors.Length; ++typeIndex)
                        {
                            contactConstraintAccessors[typeIndex]?.DeterministicallyAdd(typeIndex, overlapWorkers, ref sortedConstraints[typeIndex], Simulation, PairCache);
                        }
                    }
                    break;
                case PreflushJobType.AwakenerPhaseOne:
                    {
                        Simulation.Awakener.ExecutePhaseOneJob(job.JobIndex);
                    }
                    break;
                case PreflushJobType.AwakenerPhaseTwo:
                    {
                        Simulation.Awakener.ExecutePhaseTwoJob(job.JobIndex);
                    }
                    break;
            }
        }

        protected override void OnPreflush(IThreadDispatcher threadDispatcher, bool deterministic)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            //Before we complete the addition of constraints, the pair cache's constraint handle->pair mapping must be made large enough to hold all existing constraints plus
            //any that we are about to add. There's no guarantee that we will use them (some earlier handles may be available), but we have no good way to know ahead of time.
            int newConstraintCount = 0;
            int setsToAwakenCapacity = 0;
            for (int i = 0; i < threadCount; ++i)
            {
                newConstraintCount += overlapWorkers[i].PendingConstraints.CountConstraints();
                //This will tend to significantly overestimate the true set requirement, but that's not concerning- the maximum allocation won't be troublesome regardless.
                setsToAwakenCapacity += overlapWorkers[i].PendingSetAwakenings.Count;
            }
            var targetConstraintCapacity = Solver.HandlePool.HighestPossiblyClaimedId + 1 + newConstraintCount;
            PairCache.EnsureConstraintToPairMappingCapacity(Solver, targetConstraintCapacity);
            //Do the same for the main solver handle set. We make use of the Solver.HandleToLocation frequently; a resize would break stuff.
            Solver.EnsureSolverCapacities(1, targetConstraintCapacity);
            var setsToAwaken = new QuickList<int>(setsToAwakenCapacity, Pool);
            var uniqueAwakeningsSet = new IndexSet(Pool, Simulation.Bodies.Sets.Length);
            for (int i = 0; i < threadCount; ++i)
            {
                Simulation.Awakener.AccumulateUniqueIndices(ref overlapWorkers[i].PendingSetAwakenings, ref uniqueAwakeningsSet, ref setsToAwaken);
            }
            uniqueAwakeningsSet.Dispose(Pool);
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i].PendingSetAwakenings.Dispose(overlapWorkers[i].Batcher.Pool);
            }
            (int awakenerPhaseOneJobCount, int awakenerPhaseTwoJobCount) = Simulation.Awakener.PrepareJobs(ref setsToAwaken, false, threadCount);
            if (threadCount > 1)
            {
                //Given the sizes involved, a fixed guess of 128 should be just fine for essentially any simulation. Overkill, but not in a concerning way.
                //Temporarily allocating 1KB of memory isn't a big deal, and we will only touch the necessary subset of it anyway.
                //(There are pathological cases where resizes are still possible, but the constraint remover handles them by not adding unsafely.)
                preflushJobs = new QuickList<PreflushJob>(128 + Math.Max(awakenerPhaseOneJobCount, awakenerPhaseTwoJobCount), Pool);

                //FIRST PHASE: 
                //1) If deterministic, sort each type batch.
                //2) Perform any awakener phase one jobs (pair cache awakenings, update awakened batch referenced handles, copy awakened body regions).
                //3) Speculatively search for best-guess constraint batches for each new constraint in parallel.

                //Following the goals of the first phase, we have to responsibilities during job creation:
                //1) Scan through the workers and allocate space for the sorting handles to be added, if deterministic.
                //2) Speculative job creation walks through the types contained within each worker. Larger contiguous lists are subdivided into more than one job.
                //However, we never bother creating jobs which span boundaries between workers or types. While this can decrease the size of individual jobs below a useful level in some cases,
                //those are also the cases where the performance deficit doesn't matter. The simplicity of a single job operating only within a single list is worth more- plus, there
                //is a slight cache miss cost to jumping to a different area of memory in the middle of a job. Bundling that cost into the overhead of a new multithreaded task isn't a terrible thing.

                for (int i = 0; i < threadCount; ++i)
                {
                    overlapWorkers[i].PendingConstraints.AllocateForSpeculativeSearch();
                }
                for (int i = 0; i < awakenerPhaseOneJobCount; ++i)
                {
                    preflushJobs.Add(new PreflushJob { Type = PreflushJobType.AwakenerPhaseOne, JobIndex = i }, Pool);
                }
                //Note that we create the sort jobs ahead of batch finder. 
                //They tend to be individually much heftier than the constraint batch finder phase, and we'd like to be able to fill in the execution gaps.
                //TODO: It would be nice to have all the jobs semi-sorted by heftiness- that would just split the awakener job creator loop. Only bother if profiling suggests it.
                if (deterministic)
                {
                    Pool.TakeAtLeast(PairCache.CollisionConstraintTypeCount, out sortedConstraints);
                    sortedConstraints.Clear(0, PairCache.CollisionConstraintTypeCount);
                    for (int typeIndex = 0; typeIndex < PairCache.CollisionConstraintTypeCount; ++typeIndex)
                    {
                        int countInType = 0;
                        for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
                        {
                            countInType += overlapWorkers[workerIndex].PendingConstraints.pendingConstraintsByType[typeIndex].Count;
                        }
                        if (countInType > 0)
                        {
                            //Note that we don't actually add any constraint targets here- we let the actual worker threads do that. No reason not to, and it extracts a tiny bit of extra parallelism.
                            sortedConstraints[typeIndex] = new QuickList<SortConstraintTarget>(countInType, Pool);
                            preflushJobs.Add(new PreflushJob { Type = PreflushJobType.SortContactConstraintType, TypeIndex = typeIndex, WorkerCount = threadCount }, Pool);
                        }
                    }
                }
                const int maximumConstraintsPerJob = 16; //TODO: Empirical tuning.

                for (int typeIndex = 0; typeIndex < PairCache.CollisionConstraintTypeCount; ++typeIndex)
                {
                    for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
                    {
                        var count = overlapWorkers[workerIndex].PendingConstraints.pendingConstraintsByType[typeIndex].Count;
                        if (count > 0)
                        {
                            var jobCount = 1 + count / maximumConstraintsPerJob;
                            var jobSize = count / jobCount;
                            var remainder = count - jobCount * jobSize;
                            int previousEnd = 0;
                            for (int i = 0; i < jobCount; ++i)
                            {
                                var jobStart = previousEnd;
                                var constraintsInJob = jobSize;
                                if (i < remainder)
                                    ++constraintsInJob;
                                previousEnd += constraintsInJob;
                                preflushJobs.Add(new PreflushJob
                                {
                                    Type = PreflushJobType.SpeculativeConstraintBatchSearch,
                                    Start = jobStart,
                                    End = previousEnd,
                                    TypeIndex = typeIndex,
                                    WorkerIndex = workerIndex
                                }, Pool);
                            }
                            Debug.Assert(previousEnd == count);
                        }
                    }
                }
                //This must be cached so that the freshness checker does not schedule jobs over newly awakened entries.
                var originalPairCacheMappingCount = PairCache.Mapping.Count;
                //var start = Stopwatch.GetTimestamp();
                preflushJobIndex = -1;
                threadDispatcher.DispatchWorkers(preflushWorkerLoop, preflushJobs.Count);
                //preflushWorkerLoop(0);
                //var end = Stopwatch.GetTimestamp();
                //Console.WriteLine($"Preflush phase 1 time (us): {1e6 * (end - start) / Stopwatch.Frequency}");

                //SECOND PHASE:
                //Here we just handle the awakener's second phase jobs, if any. These can't be stacked in phase one (since phase one handles awakener phase one jobs),
                //and they can't be stacked (easily) with the narrow phase constraint add because it could trigger type batch resizes.
                preflushJobs.Clear(); //Note job clear. We're setting up new jobs.
                for (int i = 0; i < awakenerPhaseTwoJobCount; ++i)
                {
                    preflushJobs.Add(new PreflushJob { Type = PreflushJobType.AwakenerPhaseTwo, JobIndex = i }, Pool);
                }
                //start = Stopwatch.GetTimestamp();
                preflushJobIndex = -1;
                threadDispatcher.DispatchWorkers(preflushWorkerLoop, preflushJobs.Count);
                //preflushWorkerLoop(0);
                //end = Stopwatch.GetTimestamp();
                //Console.WriteLine($"Preflush phase 2 time (us): {1e6 * (end - start) / Stopwatch.Frequency}");

                //THIRD PHASE:
                //1) Locally sequential constraint adds. This is the beefiest single task, and it runs on one thread. It can be deterministic or nondeterministic.
                //2) Freshness checker. Lots of smaller jobs that can hopefully fill the gap while the constraint adds finish. The wider the CPU, the less this will be possible.
                preflushJobs.Clear(); //Note job clear. We're setting up new jobs.
                if (deterministic)
                {
                    preflushJobs.Add(new PreflushJob { Type = PreflushJobType.DeterministicConstraintAdd }, Pool);
                    //var job = new PreflushJob { Type = PreflushJobType.DeterministicConstraintAdd };
                    //ExecutePreflushJob(0, ref job);
                }
                else
                {
                    preflushJobs.Add(new PreflushJob { Type = PreflushJobType.NondeterministicConstraintAdd, WorkerCount = threadCount }, Pool);
                    //var job = new PreflushJob { Type = PreflushJobType.NondeterministicConstraintAdd, WorkerCount = threadCount };
                    //ExecutePreflushJob(0, ref job);
                }
                FreshnessChecker.CreateJobs(threadCount, ref preflushJobs, Pool, originalPairCacheMappingCount);
                //start = Stopwatch.GetTimestamp();
                preflushJobIndex = -1;
                threadDispatcher.DispatchWorkers(preflushWorkerLoop, preflushJobs.Count);
                //preflushWorkerLoop(0);
                //end = Stopwatch.GetTimestamp();
                //Console.WriteLine($"Preflush phase 3 time (us): {1e6 * (end - start) / Stopwatch.Frequency}");

                for (int i = 0; i < threadCount; ++i)
                {
                    overlapWorkers[i].PendingConstraints.DisposeSpeculativeSearch();
                }
                if (deterministic)
                {
                    for (int i = 0; i < PairCache.CollisionConstraintTypeCount; ++i)
                    {
                        ref var typeList = ref sortedConstraints[i];
                        if (typeList.Span.Allocated)
                            typeList.Dispose(Pool);
                    }
                    Pool.Return(ref sortedConstraints);
                }
                preflushJobs.Dispose(Pool);
            }
            else
            {
                //Single threaded. Quite a bit simpler!
                //Three tasks: awakenings, freshness checker, and add all pending constraints.
                //Note that phase one changes the PairCache.Mapping.Count; the count must be cached so that the freshness checker doesn't bother analyzing the newly awakened pairs.
                var originalMappingCount = PairCache.Mapping.Count;
                for (int i = 0; i < awakenerPhaseOneJobCount; ++i)
                    Simulation.Awakener.ExecutePhaseOneJob(i);
                //Note that phase one of awakener must occur before the constraint flush. Phase one registers the newly awakened constraints in constraint batches.
                //This this was not done, pending adds might end up in the same batches as newly awake constraints that share bodies.
                for (int i = 0; i < awakenerPhaseTwoJobCount; ++i)
                    Simulation.Awakener.ExecutePhaseTwoJob(i);
                overlapWorkers[0].PendingConstraints.FlushSequentially(Simulation, PairCache);
                FreshnessChecker.CheckFreshnessInRegion(0, 0, originalMappingCount);
            }
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i].PendingConstraints.Dispose();
            }
            if (setsToAwaken.Count > 0)
            {
                Simulation.Awakener.DisposeForCompletedAwakenings(ref setsToAwaken);
            }
            setsToAwaken.Dispose(Pool);
        }
    }
}
