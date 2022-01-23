using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace BepuPhysics
{
    /// <summary>
    /// Provides functionality for efficiently waking up sleeping bodies.
    /// </summary>
    public class IslandAwakener
    {
        Solver solver;
        Statics statics;
        Bodies bodies;
        BroadPhase broadPhase;
        IslandSleeper sleeper;
        BufferPool pool;
        internal PairCache pairCache;

        public IslandAwakener(Bodies bodies, Statics statics, Solver solver, BroadPhase broadPhase, IslandSleeper sleeper, BufferPool pool)
        {
            this.bodies = bodies;
            this.statics = statics;
            this.solver = solver;
            this.broadPhase = broadPhase;
            this.sleeper = sleeper;
            this.pool = pool;

            this.phaseOneWorkerDelegate = PhaseOneWorker;
            this.phaseTwoWorkerDelegate = PhaseTwoWorker;
        }

        /// <summary>
        /// Wakes up a body if it is sleeping. All bodies that can be found by traversing the constraint graph from the body will also be awakened.
        /// If the body is already awake, this does nothing.
        /// </summary>
        /// <param name="bodyHandle">Handle of the body to awaken.</param>
        public void AwakenBody(BodyHandle bodyHandle)
        {
            bodies.ValidateExistingHandle(bodyHandle);
            AwakenSet(bodies.HandleToLocation[bodyHandle.Value].SetIndex);
        }

        /// <summary>
        /// Wakes up any sleeping bodies associated with a constraint. All bodies that can be found by traversing the constraint graph from the constraint referenced bodies will also be awakened.
        /// If all bodies associated with the constraint are already awake, this does nothing.
        /// </summary>
        /// <param name="constraintHandle">Handle of the constraint to awaken.</param>
        public void AwakenConstraint(ConstraintHandle constraintHandle)
        {
            AwakenSet(solver.HandleToConstraint[constraintHandle.Value].SetIndex);
        }

        /// <summary>
        /// Wakes up all bodies and constraints within a set. Doesn't do anything if the set is awake (index zero).
        /// </summary>
        /// <param name="setIndex">Index of the set to awaken.</param>
        public void AwakenSet(int setIndex)
        {
            if (setIndex > 0)
            {
                ValidateSleepingSetIndex(setIndex);
                //TODO: Some fairly pointless work here- spans or other approaches could help with the API.
                var list = new QuickList<int>(1, pool);
                list.AddUnsafely(setIndex);
                AwakenSets(ref list);
                list.Dispose(pool);
            }
        }

        /// <summary>
        /// Awakens a list of set indices.
        /// </summary>
        /// <param name="setIndices">List of set indices to wake up.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use when waking the bodies. Pass null to run on a single thread.</param>
        public void AwakenSets(ref QuickList<int> setIndices, IThreadDispatcher threadDispatcher = null)
        {
            var uniqueSetIndices = new QuickList<int>(setIndices.Count, pool);
            var uniqueSet = new IndexSet(pool, bodies.Sets.Length);
            AccumulateUniqueIndices(ref setIndices, ref uniqueSet, ref uniqueSetIndices);
            uniqueSet.Dispose(pool);

            //Note that we use the same codepath as multithreading, we just don't use a multithreaded dispatch to execute jobs.
            //TODO: It would probably be a good idea to add a little heuristic to avoid doing multithreaded dispatches if there are only like 5 total bodies.
            //Shouldn't matter too much- the threaded variant should only really be used when doing big batched changes, so having a fixed constant cost isn't that bad.
            int threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            //Note that direct wakes always reset activity states. I suspect this is sufficiently universal that no one will ever want the alternative,
            //even though the narrowphase does avoid resetting activity states for the sake of faster resleeping when possible.      
            var (phaseOneJobCount, phaseTwoJobCount) = PrepareJobs(ref uniqueSetIndices, true, threadCount);

            if (threadCount > 1)
            {
                this.jobIndex = -1;
                this.jobCount = phaseOneJobCount;
                threadDispatcher.DispatchWorkers(phaseOneWorkerDelegate, phaseOneJobCount);
            }
            else
            {
                for (int i = 0; i < phaseOneJobCount; ++i)
                {
                    ExecutePhaseOneJob(i);
                }
            }

            if (threadCount > 1)
            {
                this.jobIndex = -1;
                this.jobCount = phaseTwoJobCount;
                threadDispatcher.DispatchWorkers(phaseTwoWorkerDelegate, phaseTwoJobCount);
            }
            else
            {
                for (int i = 0; i < phaseTwoJobCount; ++i)
                {
                    ExecutePhaseTwoJob(i);
                }
            }

            DisposeForCompletedAwakenings(ref uniqueSetIndices);

            uniqueSetIndices.Dispose(pool);
        }

        //Note that the worker loop and its supporting fields are only used when the island awakener is used in isolation by an external call.
        //The engine's own use of the awakener takes place in the narrowphase, where the awakener jobs are scheduled alongside other jobs for greater parallelism.
        int jobIndex;
        int jobCount;
        //TODO: once again, we repeat this worker pattern. We've done this, what, seven times? It wouldn't even be that difficult to centralize it. We'll get around to that at some point.
        Action<int> phaseOneWorkerDelegate;
        Action<int> phaseTwoWorkerDelegate;
        internal void PhaseOneWorker(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref jobIndex);
                if (index >= jobCount)
                    break;
                ExecutePhaseOneJob(index);
            }
        }
        internal void PhaseTwoWorker(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref jobIndex);
                if (index >= jobCount)
                    break;
                ExecutePhaseTwoJob(index);
            }
        }
        enum PhaseOneJobType
        {
            PairCache,
            MoveFallbackBatchBodies,
            UpdateBatchReferencedHandles,
            CopyBodyRegion
        }

        struct PhaseOneJob
        {
            public PhaseOneJobType Type;
            public int BatchIndex; //Only used by batch reference update jobs.
            //Only used by body region copy.
            public int SourceSet;
            public int SourceStart;
            public int TargetStart;
            public int Count;
            //could union these, but it's preeeetty pointless.
        }
        enum PhaseTwoJobType
        {
            BroadPhase,
            CopyConstraintRegion,
            AddFallbackTypeBatchConstraints
        }
        struct CopyConstraintRegionJob
        {
            public int SourceStart;
            public int TargetStart;
            public int Count;
            public int SourceSet;
            public int TypeId;
            public int Batch;
            public int SourceTypeBatch;
            public int TargetTypeBatch;
        }

        struct FallbackAddSource
        {
            public int SourceSet;
            public int SourceTypeBatchIndex;
        }
        struct AddFallbackTypeBatchConstraintsJob
        {
            public Buffer<FallbackAddSource> Sources;
            public int TypeId;
            public int TargetTypeBatch;
        }

        [StructLayout(LayoutKind.Explicit)]
        struct PhaseTwoJob
        {
            [FieldOffset(0)]
            public PhaseTwoJobType Type;
            [FieldOffset(4)]
            public CopyConstraintRegionJob CopyConstraintRegion;
            [FieldOffset(8)]
            public AddFallbackTypeBatchConstraintsJob AddFallbackTypeBatchConstraints;
        }

        bool resetActivityStates;
        QuickList<int> uniqueSetIndices;
        QuickList<PhaseOneJob> phaseOneJobs;
        QuickList<PhaseTwoJob> phaseTwoJobs;
        internal unsafe void ExecutePhaseOneJob(int index)
        {
            ref var job = ref phaseOneJobs[index];
            switch (job.Type)
            {
                case PhaseOneJobType.PairCache:
                    {
                        //Updating the pair cache is locally sequential because it modifies the global overlap mapping, which at the moment is a hash table.
                        //The other per-type caches could be separated from this job and handled in an internally multithreaded way, but that would add complexity that is likely unnecessary.
                        //We'll assume the other jobs can balance things out until proven otherwise.
                        for (int i = 0; i < uniqueSetIndices.Count; ++i)
                        {
                            pairCache.AwakenSet(uniqueSetIndices[i]);
                        }
                    }
                    break;
                case PhaseOneJobType.UpdateBatchReferencedHandles:
                    {
                        //Note that the narrow phase will schedule this job alongside another worker which reads existing batch referenced handles.
                        //There will be some cache line sharing sometimes. That's not wonderful for performance, but it should not cause bugs:
                        //1) The narrowphase's speculative batch search is readonly, so there are no competing writes.
                        //2) Awakening requires that a new constraint has been created involving an sleeping body.
                        //The speculative search will then try to find a batch for that constraint, and it may end up erroneously
                        //finding a batch slot that gets blocked by this waking procedure. 
                        //But the speculative process is *speculative*; it is fine for it to be wrong, so long as it isn't wrong in a way that makes it choose a higher batch index.

                        //Note that this is parallel over different batches, regardless of which source set they're from.
                        ref var targetBatchReferencedHandles = ref solver.batchReferencedHandles[job.BatchIndex];
                        for (int i = 0; i < uniqueSetIndices.Count; ++i)
                        {
                            var setIndex = uniqueSetIndices[i];
                            ref var sourceSet = ref solver.Sets[setIndex];
                            if (sourceSet.Batches.Count > job.BatchIndex)
                            {
                                ref var batch = ref sourceSet.Batches[job.BatchIndex];
                                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                                {
                                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                                    solver.TypeProcessors[typeBatch.TypeId].AddWakingBodyHandlesToBatchReferences(ref typeBatch, ref targetBatchReferencedHandles);
                                }
                            }
                        }
                    }
                    break;
                case PhaseOneJobType.MoveFallbackBatchBodies:
                    {
                        for (int i = 0; i < uniqueSetIndices.Count; ++i)
                        {
                            Debug.Assert(uniqueSetIndices[i] > 0);
                            ref var source = ref solver.Sets[uniqueSetIndices[i]].SequentialFallback;
                            ref var target = ref solver.ActiveSet.SequentialFallback;
                            if (source.dynamicBodyConstraintCounts.Count > 0)
                            {
                                for (int j = 0; j < source.dynamicBodyConstraintCounts.Count; ++j)
                                {
                                    //Inactive sets refer to body handles. Active set refers to body indices. Make the transition.
                                    //The HandleToLocation was updated during job setup, so we can use it.
                                    ref var bodyLocation = ref bodies.HandleToLocation[source.dynamicBodyConstraintCounts.Keys[j]];
                                    Debug.Assert(bodyLocation.SetIndex == 0, "Any batch moved into the active set should be dealing with bodies which have already been moved into the active set.");
                                    var added = target.dynamicBodyConstraintCounts.AddUnsafely(bodyLocation.Index, source.dynamicBodyConstraintCounts.Values[j]);
                                    Debug.Assert(added, "Any body moving from an inactive set to the active set should not already be present in the active set's fallback batch.");
                                }
                            }
                        }
                    }
                    break;
                case PhaseOneJobType.CopyBodyRegion:
                    {
                        //Since we already preallocated everything during the job preparation, all we have to do is copy from the sleeping set location.
                        //Note that the broad phase index associated with a body is not updated here. That's handled in phase 2; this just puts the rest of the data into position.
                        ref var sourceSet = ref bodies.Sets[job.SourceSet];
                        ref var targetSet = ref bodies.ActiveSet;
                        sourceSet.Collidables.CopyTo(job.SourceStart, targetSet.Collidables, job.TargetStart, job.Count);
                        sourceSet.Constraints.CopyTo(job.SourceStart, targetSet.Constraints, job.TargetStart, job.Count);
                        sourceSet.SolverStates.CopyTo(job.SourceStart, targetSet.SolverStates, job.TargetStart, job.Count);
                        //This rescans the memory, but it should be still floating in cache ready to access.
                        for (int i = 0; i < job.Count; ++i)
                        {
                            var sourceBodyIndex = i + job.SourceStart;
                            if (Bodies.IsKinematicUnsafeGCHole(ref sourceSet.SolverStates[sourceBodyIndex].Inertia.Local) && sourceSet.Constraints[sourceBodyIndex].Count > 0)
                            {
                                bool taken = false;
                                solver.constrainedKinematicLock.Enter(ref taken);
                                solver.ConstrainedKinematicHandles.AddUnsafely(sourceSet.IndexToHandle[sourceBodyIndex].Value);
                                solver.constrainedKinematicLock.Exit();
                            }
                        }
                        sourceSet.Activity.CopyTo(job.SourceStart, targetSet.Activity, job.TargetStart, job.Count);
                        if (resetActivityStates)
                        {
                            for (int targetIndex = job.TargetStart + job.Count - 1; targetIndex >= job.TargetStart; --targetIndex)
                            {
                                ref var targetActivity = ref targetSet.Activity[targetIndex];
                                targetActivity.TimestepsUnderThresholdCount = 0;
                                targetActivity.SleepCandidate = false;
                            }
                        }
                        sourceSet.IndexToHandle.CopyTo(job.SourceStart, targetSet.IndexToHandle, job.TargetStart, job.Count);
                    }
                    break;
            }
        }


        internal unsafe void ExecutePhaseTwoJob(int index)
        {
            ref var phaseTwoJob = ref phaseTwoJobs[index];
            switch (phaseTwoJob.Type)
            {
                case PhaseTwoJobType.BroadPhase:
                    {
                        //Note that the broad phase add/remove has a dependency on the body copies; that's why it's in the second phase.
                        //Also note that the broad phase update cannot be split into two parallel jobs because removals update broad phase indices of moved leaves,
                        //which in context might belong to awakened collidables that have not yet been removed.
                        //If the indices are sorted, this could be avoided and this could be split into two jobs. Only bother if performance numbers suggest it.
                        ref var activeSet = ref bodies.ActiveSet;
                        for (int i = 0; i < uniqueSetIndices.Count; ++i)
                        {
                            ref var sleepingBodySet = ref bodies.Sets[uniqueSetIndices[i]];
                            for (int j = 0; j < sleepingBodySet.Count; ++j)
                            {
                                //Note that we have to go grab the active version of the collidable, since the active version is potentially modified by removals.
                                ref var bodyLocation = ref bodies.HandleToLocation[sleepingBodySet.IndexToHandle[j].Value];
                                Debug.Assert(bodyLocation.SetIndex == 0);
                                //The broad phase index value is currently the static index, either copied from the sleeping set or modified by a removal below.
                                //We'll update it, so just take a reference.
                                ref var broadPhaseIndex = ref activeSet.Collidables[bodyLocation.Index].BroadPhaseIndex;
                                if (broadPhaseIndex >= 0)
                                {
                                    broadPhase.GetStaticBoundsPointers(broadPhaseIndex, out var minPointer, out var maxPointer);
                                    BoundingBox bounds;
                                    bounds.Min = *minPointer;
                                    bounds.Max = *maxPointer;
                                    var staticBroadPhaseIndexToRemove = broadPhaseIndex;
                                    broadPhaseIndex = broadPhase.AddActive(broadPhase.staticLeaves[broadPhaseIndex], ref bounds);

                                    if (broadPhase.RemoveStaticAt(staticBroadPhaseIndexToRemove, out var movedLeaf))
                                    {
                                        if (movedLeaf.Mobility == Collidables.CollidableMobility.Static)
                                        {
                                            statics.GetDirectReference(movedLeaf.StaticHandle).BroadPhaseIndex = staticBroadPhaseIndexToRemove;
                                        }
                                        else
                                        {
                                            //Note that the moved leaf cannot refer to one of the collidables that we've already moved into the active set, because all such collidables
                                            //have already been removed. 
                                            bodies.UpdateCollidableBroadPhaseIndex(movedLeaf.BodyHandle, staticBroadPhaseIndexToRemove);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
                case PhaseTwoJobType.CopyConstraintRegion:
                    {
                        //Constraints are a little more complicated than bodies for a few reasons:
                        //1) Constraints are stored in AOSOA format. We cannot simply copy with bundle alignment with no preparation, since that may leave a gap.
                        //To avoid this issue, we instead use the last bundle of the source batch to pad out any incomplete bundles ahead of the target copy region.
                        //The scheduler attempts to maximize the number of jobs which are pure bundle copies.
                        //2) Sleeping constraints store their body references as body *handles* rather than body indices.
                        //Pulling the type batches back into the active set requires translating those body handles to body indices.  
                        //3) The translation from body handle to body index requires that the bodies already have an active set identity, which is why the constraints wait until the second phase.
                        ref var job = ref phaseTwoJob.CopyConstraintRegion;
                        Debug.Assert(solver.ActiveSet.Batches[job.Batch].TypeBatches[job.TargetTypeBatch].TypeId == solver.Sets[job.SourceSet].Batches[job.Batch].TypeBatches[job.SourceTypeBatch].TypeId);
                        Debug.Assert(job.Batch != solver.FallbackBatchThreshold, "Fallback batches must only be handled by the fallback-specific job.");
                        solver.TypeProcessors[job.TypeId].CopySleepingToActive(
                            job.SourceSet, job.Batch, job.SourceTypeBatch, job.TargetTypeBatch,
                            job.SourceStart, job.TargetStart, job.Count, bodies, solver);
                        //solver.ValidateConstraintMaps(0, job.Batch, job.TargetTypeBatch, job.TargetStart, job.Count);
                    }
                    break;
                case PhaseTwoJobType.AddFallbackTypeBatchConstraints:
                    {
                        ref var job = ref phaseTwoJob.AddFallbackTypeBatchConstraints;
                        for (int i = 0; i < job.Sources.Length; ++i)
                        {
                            var source = job.Sources[i];
                            solver.TypeProcessors[job.TypeId].AddSleepingToActiveForFallback(source.SourceSet, source.SourceTypeBatchIndex, job.TargetTypeBatch, bodies, solver);
                        }
                    }
                    break;
            }

        }

        internal void AccumulateUniqueIndices(ref QuickList<int> candidateSetIndices, ref IndexSet uniqueSet, ref QuickList<int> uniqueSetIndices)
        {
            for (int i = 0; i < candidateSetIndices.Count; ++i)
            {
                var candidateSetIndex = candidateSetIndices[i];
                if (!uniqueSet.Contains(candidateSetIndex))
                {
                    uniqueSet.AddUnsafely(candidateSetIndex);
                    uniqueSetIndices.AllocateUnsafely() = candidateSetIndex;
                }
            }
            if (uniqueSetIndices.Count > 0)
            {
                //The number of unique set indices being awakened is typically very small (<4), so sorting even when the simulation is running nondeterministically really isn't a concern.
                //Determinism requires source sets are awakened in order when the fallback batch may be involved, since constraint order and typebatch order within the sequential fallback matter.
                var comparer = new PrimitiveComparer<int>();
                QuickSort.Sort(ref uniqueSetIndices[0], 0, uniqueSetIndices.Count - 1, ref comparer);
            }
        }
        [Conditional("DEBUG")]
        void ValidateSleepingSetIndex(int setIndex)
        {
            Debug.Assert(setIndex >= 1 && setIndex < bodies.Sets.Length && setIndex < solver.Sets.Length && setIndex < pairCache.SleepingSets.Length);
            //Note that pair cache sets are not guaranteed to be allocated if there are no pairs, and solver sets are not guaranteed to exist if there are no constraints.
            Debug.Assert(bodies.Sets[setIndex].Allocated);
        }
        [Conditional("DEBUG")]
        void ValidateUniqueSets(ref QuickList<int> setIndices)
        {
            var set = new IndexSet(pool, bodies.Sets.Length);
            for (int i = 0; i < setIndices.Count; ++i)
            {
                var setIndex = setIndices[i];
                ValidateSleepingSetIndex(setIndex);
                Debug.Assert(!set.Contains(setIndex));
                set.Add(setIndex, pool);
            }
            set.Dispose(pool);

        }

        //This is getting into the realm of Fizzbuzz Enterprise. 
        interface ITypeCount
        {
            void Add<T>(T other) where T : ITypeCount;
        }
        struct ConstraintCount : ITypeCount
        {
            public int Count;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Add<T>(T other) where T : ITypeCount
            {
                Debug.Assert(typeof(T) == typeof(ConstraintCount));
                Count += Unsafe.As<T, ConstraintCount>(ref other).Count;
            }
        }
        struct PairCacheCount : ITypeCount
        {
            public int ElementSizeInBytes;
            public int ByteCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Add<T>(T other) where T : ITypeCount
            {
                Debug.Assert(typeof(T) == typeof(PairCacheCount));
                ref var pairCacheOther = ref Unsafe.As<T, PairCacheCount>(ref other);
                Debug.Assert(ElementSizeInBytes == 0 || ElementSizeInBytes == pairCacheOther.ElementSizeInBytes);
                ElementSizeInBytes = pairCacheOther.ElementSizeInBytes;
                ByteCount += pairCacheOther.ByteCount;
            }
        }

        struct TypeAllocationSizes<T> where T : unmanaged, ITypeCount
        {
            public Buffer<T> TypeCounts;
            public int HighestOccupiedTypeIndex;
            public TypeAllocationSizes(BufferPool pool, int maximumTypeCount)
            {
                pool.Take(maximumTypeCount, out TypeCounts);
                TypeCounts.Clear(0, maximumTypeCount);
                HighestOccupiedTypeIndex = 0;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Add(int typeId, T typeCount)
            {
                TypeCounts[typeId].Add(typeCount);
                if (typeId > HighestOccupiedTypeIndex)
                    HighestOccupiedTypeIndex = typeId;
            }
            public void Dispose(BufferPool pool)
            {
                pool.Return(ref TypeCounts);
            }
        }


        unsafe internal (int phaseOneJobCount, int phaseTwoJobCount) PrepareJobs(ref QuickList<int> setIndices, bool resetActivityStates, int threadCount)
        {
            if (setIndices.Count == 0)
                return (0, 0);
            ValidateUniqueSets(ref setIndices);
            this.uniqueSetIndices = setIndices;
            this.resetActivityStates = resetActivityStates;

            //We have three main jobs in this function:
            //1) Ensure that the active set in the bodies, solver, and pair cache can hold the newly awakened islands without resizing or accessing a buffer pool.
            //2) Allocate space in the active set for the bodies and constraints.
            //(Pair caches are left to be handled in a locally sequential task right now due to the overlap mapping being global.
            //The type caches could be updated in parallel, but we just didn't split it out. You can consider doing that if there appears to be a reason to do so.)
            //3) Schedule actual jobs for the two work phases.

            int newBodyCount = 0;
            int highestNewBatchCount = 0;
            int highestRequiredTypeCapacity = 0;
            int additionalRequiredFallbackCapacity = 0;
            for (int i = 0; i < setIndices.Count; ++i)
            {
                var setIndex = setIndices[i];
                newBodyCount += bodies.Sets[setIndex].Count;
                var setBatchCount = solver.Sets[setIndex].Batches.Count;
                if (highestNewBatchCount < setBatchCount)
                    highestNewBatchCount = setBatchCount;
                ref var constraintSet = ref solver.Sets[setIndex];
                additionalRequiredFallbackCapacity += constraintSet.SequentialFallback.BodyCount;
                for (int batchIndex = 0; batchIndex < constraintSet.Batches.Count; ++batchIndex)
                {
                    ref var batch = ref constraintSet.Batches[batchIndex];
                    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                    {
                        ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                        if (highestRequiredTypeCapacity < typeBatch.TypeId)
                            highestRequiredTypeCapacity = typeBatch.TypeId;
                    }
                }

            }
            //We accumulated indices above; add one to get the capacity requirement.
            ++highestRequiredTypeCapacity;
            pool.Take<TypeAllocationSizes<ConstraintCount>>(highestNewBatchCount, out var constraintCountPerTypePerBatch);
            for (int batchIndex = 0; batchIndex < highestNewBatchCount; ++batchIndex)
            {
                constraintCountPerTypePerBatch[batchIndex] = new TypeAllocationSizes<ConstraintCount>(pool, highestRequiredTypeCapacity);
            }
            var narrowPhaseConstraintCaches = new TypeAllocationSizes<PairCacheCount>(pool, PairCache.CollisionConstraintTypeCount);
            var narrowPhaseCollisionCaches = new TypeAllocationSizes<PairCacheCount>(pool, PairCache.CollisionTypeCount);

            void AccumulatePairCacheTypeCounts(ref Buffer<SleepingCache> sourceTypeCaches, ref TypeAllocationSizes<PairCacheCount> counts)
            {
                for (int j = 0; j < sourceTypeCaches.Length; ++j)
                {
                    ref var sourceCache = ref sourceTypeCaches[j];
                    if (sourceCache.List.Buffer.Allocated)
                        counts.Add(sourceCache.TypeId, new PairCacheCount { ByteCount = sourceCache.List.ByteCount, ElementSizeInBytes = sourceCache.List.ElementSizeInBytes });
                    else
                        break; //Encountering an unallocated slot is a termination condition. Used instead of explicitly storing cache counts, which are only rarely useful.
                }
            }
            int newPairCount = 0;
            for (int i = 0; i < setIndices.Count; ++i)
            {
                var setIndex = setIndices[i];
                ref var constraintSet = ref solver.Sets[setIndex];
                for (int batchIndex = 0; batchIndex < constraintSet.Batches.Count; ++batchIndex)
                {
                    ref var constraintCountPerType = ref constraintCountPerTypePerBatch[batchIndex];
                    ref var batch = ref constraintSet.Batches[batchIndex];
                    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                    {
                        ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                        constraintCountPerType.Add(typeBatch.TypeId, new ConstraintCount { Count = typeBatch.ConstraintCount });
                    }
                }

                ref var sourceSet = ref pairCache.SleepingSets[setIndex];
                newPairCount += sourceSet.Pairs.Count;
                AccumulatePairCacheTypeCounts(ref sourceSet.ConstraintCaches, ref narrowPhaseConstraintCaches);
                AccumulatePairCacheTypeCounts(ref sourceSet.CollisionCaches, ref narrowPhaseCollisionCaches);
            }

            //We now know how many new bodies, constraint batch entries, and pair cache entries are going to be added.
            //Ensure capacities on all systems:
            //bodies,
            bodies.EnsureCapacity(bodies.ActiveSet.Count + newBodyCount);
            solver.ConstrainedKinematicHandles.EnsureCapacity(solver.ConstrainedKinematicHandles.Count + newBodyCount, pool); //TODO: This could be FAR more conservative. Few bodies are typically kinematic.
            //broad phase, (technically overestimating, not every body has a collidable, but vast majority do and shrug)
            broadPhase.EnsureCapacity(broadPhase.ActiveTree.LeafCount + newBodyCount, broadPhase.StaticTree.LeafCount);
            //constraints,
            solver.ActiveSet.Batches.EnsureCapacity(highestNewBatchCount, pool);
            if (additionalRequiredFallbackCapacity > 0)
                solver.ActiveSet.SequentialFallback.EnsureCapacity(solver.ActiveSet.SequentialFallback.BodyCount + additionalRequiredFallbackCapacity, pool);
            Debug.Assert(highestNewBatchCount <= solver.FallbackBatchThreshold + 1, "Shouldn't have any batches beyond the fallback batch.");
            solver.batchReferencedHandles.EnsureCapacity(highestNewBatchCount, pool);
            for (int batchIndex = solver.ActiveSet.Batches.Count; batchIndex < highestNewBatchCount; ++batchIndex)
            {
                solver.ActiveSet.Batches.AllocateUnsafely() = new ConstraintBatch(pool);
                solver.batchReferencedHandles.AllocateUnsafely() = new IndexSet(pool, bodies.HandlePool.HighestPossiblyClaimedId + 1);
            }
            for (int batchIndex = 0; batchIndex < highestNewBatchCount; ++batchIndex)
            {
                ref var constraintCountPerType = ref constraintCountPerTypePerBatch[batchIndex];
                ref var batch = ref solver.ActiveSet.Batches[batchIndex];
                batch.EnsureTypeMapSize(pool, constraintCountPerType.HighestOccupiedTypeIndex);
                solver.batchReferencedHandles[batchIndex].EnsureCapacity(bodies.HandlePool.HighestPossiblyClaimedId + 1, pool);
                for (int typeId = 0; typeId <= constraintCountPerType.HighestOccupiedTypeIndex; ++typeId)
                {
                    var countForType = constraintCountPerType.TypeCounts[typeId].Count;
                    //The fallback batch must allocate a worst case scenario assuming that every new constraint needs its own bundle.
                    //It's difficult to be more conservative ahead of time; we don't know which existing partial bundles will be able to accept the new constraints.
                    //Fallback batches should tend to be rarely used and relatively small, and the extra memory won't be touched, so this isn't a major concern.
                    if (batchIndex == solver.FallbackBatchThreshold)
                        countForType *= Vector<float>.Count;
                    if (countForType > 0)
                    {
                        var typeProcessor = solver.TypeProcessors[typeId];
                        ref var typeBatch = ref *batch.GetOrCreateTypeBatch(typeId, typeProcessor, countForType, pool);
                        var targetCapacity = countForType + typeBatch.ConstraintCount;
                        if (targetCapacity > typeBatch.IndexToHandle.Length)
                        {
                            typeProcessor.Resize(ref typeBatch, targetCapacity, pool);
                        }
                    }
                }
                constraintCountPerType.Dispose(pool);
            }
            pool.Return(ref constraintCountPerTypePerBatch);
            //and narrow phase pair caches.
            ref var targetPairCache = ref pairCache.GetCacheForAwakening();
            void EnsurePairCacheTypeCapacities(ref TypeAllocationSizes<PairCacheCount> cacheSizes, ref Buffer<UntypedList> targetCaches, BufferPool cachePool)
            {
                for (int typeIndex = 0; typeIndex <= cacheSizes.HighestOccupiedTypeIndex; ++typeIndex)
                {
                    ref var pairCacheCount = ref cacheSizes.TypeCounts[typeIndex];
                    if (pairCacheCount.ByteCount > 0)
                    {
                        ref var targetSubCache = ref targetCaches[typeIndex];
                        targetSubCache.EnsureCapacityInBytes(pairCacheCount.ElementSizeInBytes, targetSubCache.ByteCount + pairCacheCount.ByteCount, cachePool);
                    }
                }
            }
            EnsurePairCacheTypeCapacities(ref narrowPhaseConstraintCaches, ref targetPairCache.constraintCaches, targetPairCache.pool);
            EnsurePairCacheTypeCapacities(ref narrowPhaseCollisionCaches, ref targetPairCache.collisionCaches, targetPairCache.pool);
            narrowPhaseConstraintCaches.Dispose(pool);
            narrowPhaseCollisionCaches.Dispose(pool);
            pairCache.Mapping.EnsureCapacity(pairCache.Mapping.Count + newPairCount, pool);

            phaseOneJobs = new QuickList<PhaseOneJob>(Math.Max(32, highestNewBatchCount + 1), pool);
            phaseTwoJobs = new QuickList<PhaseTwoJob>(32, pool);
            //Finally, create actual jobs. Note that this involves actually allocating space in the bodies set and in type batches for the workers to fill in.
            //(Pair caches are currently handled in a locally sequential way and do not require preallocation.)
            phaseOneJobs.AllocateUnsafely() = new PhaseOneJob { Type = PhaseOneJobType.PairCache };
            phaseOneJobs.AllocateUnsafely() = new PhaseOneJob { Type = PhaseOneJobType.MoveFallbackBatchBodies };
            for (int batchIndex = 0; batchIndex < highestNewBatchCount; ++batchIndex)
            {
                phaseOneJobs.AllocateUnsafely() = new PhaseOneJob { Type = PhaseOneJobType.UpdateBatchReferencedHandles, BatchIndex = batchIndex };
            }
            phaseTwoJobs.AllocateUnsafely() = new PhaseTwoJob { Type = PhaseTwoJobType.BroadPhase };

            ref var activeBodySet = ref bodies.ActiveSet;
            ref var activeSolverSet = ref solver.ActiveSet;
            //TODO: The job sizes are a little goofy for single threaded execution. Easy enough to resolve with special case or dynamic size.

            //Multiple source sets can contribute to the same target type batch. Track those as we enumerate sets so we can create a single job for each target after the loop.
            QuickDictionary<int, QuickList<FallbackAddSource>, PrimitiveComparer<int>> targetFallbackTypeBatchesToSources = highestNewBatchCount > solver.FallbackBatchThreshold ? targetFallbackTypeBatchesToSources = new(8, pool) : default;
            for (int i = 0; i < uniqueSetIndices.Count; ++i)
            {
                var sourceSetIndex = uniqueSetIndices[i];
                {
                    const int bodyJobSize = 64;
                    ref var sourceSet = ref bodies.Sets[sourceSetIndex];
                    var setJobCount = Math.Max(1, sourceSet.Count / bodyJobSize);
                    var baseBodiesPerJob = sourceSet.Count / setJobCount;
                    var remainder = sourceSet.Count - baseBodiesPerJob * setJobCount;
                    phaseOneJobs.EnsureCapacity(phaseOneJobs.Count + setJobCount, pool);
                    var previousSourceEnd = 0;
                    for (int jobIndex = 0; jobIndex < setJobCount; ++jobIndex)
                    {
                        ref var job = ref phaseOneJobs.AllocateUnsafely();
                        job.Type = PhaseOneJobType.CopyBodyRegion;
                        job.SourceSet = sourceSetIndex;
                        job.SourceStart = previousSourceEnd;
                        job.TargetStart = activeBodySet.Count;
                        job.Count = jobIndex >= remainder ? baseBodiesPerJob : baseBodiesPerJob + 1;
                        previousSourceEnd += job.Count;
                        activeBodySet.Count += job.Count;
                        //We perform the body handle update up front because it's cheap, and because it makes some things simpler:
                        //the narrow phase flush adds constraints in the second stage, and we want the awakener to have already modified the fallback batches by the time that happens.
                        //So, we do fallback batch modification in phase one of the IslandAwakener (which happens in phase one of the narrow phase flush), and that relies on the 
                        //body handle->location mapping being up to date.
                        for (int j = 0; j < job.Count; ++j)
                        {
                            var sourceIndex = job.SourceStart + j;
                            var targetIndex = job.TargetStart + j;
                            ref var bodyLocation = ref bodies.HandleToLocation[sourceSet.IndexToHandle[sourceIndex].Value];
                            bodyLocation.SetIndex = 0;
                            bodyLocation.Index = targetIndex;
                        }
                    }
                    Debug.Assert(previousSourceEnd == sourceSet.Count);
                    Debug.Assert(activeBodySet.Count <= activeBodySet.IndexToHandle.Length);
                }
                {
                    const int constraintJobSize = 32;
                    ref var sourceSet = ref solver.Sets[sourceSetIndex];
                    var fallbackIndex = solver.FallbackBatchThreshold;

                    int synchronizedBatchCountInSource = sourceSet.Batches.Count > solver.FallbackBatchThreshold ? solver.FallbackBatchThreshold : sourceSet.Batches.Count;
                    for (int batchIndex = 0; batchIndex < synchronizedBatchCountInSource; ++batchIndex)
                    {
                        ref var sourceBatch = ref sourceSet.Batches[batchIndex];
                        ref var targetBatch = ref activeSolverSet.Batches[batchIndex];
                        for (int sourceTypeBatchIndex = 0; sourceTypeBatchIndex < sourceBatch.TypeBatches.Count; ++sourceTypeBatchIndex)
                        {
                            ref var sourceTypeBatch = ref sourceBatch.TypeBatches[sourceTypeBatchIndex];
                            var targetTypeBatchIndex = targetBatch.TypeIndexToTypeBatchIndex[sourceTypeBatch.TypeId];
                            ref var targetTypeBatch = ref targetBatch.TypeBatches[targetTypeBatchIndex];
                            //TODO: It would be nice to be a little more clever about scheduling start and end points for the sake of avoiding partial bundles.
                            var jobCount = Math.Max(1, sourceTypeBatch.ConstraintCount / constraintJobSize);
                            var baseConstraintsPerJob = sourceTypeBatch.ConstraintCount / jobCount;
                            var remainder = sourceTypeBatch.ConstraintCount - baseConstraintsPerJob * jobCount;
                            phaseTwoJobs.EnsureCapacity(phaseTwoJobs.Count + jobCount, pool);
                            var previousSourceEnd = 0;
                            for (int jobIndex = 0; jobIndex < jobCount; ++jobIndex)
                            {
                                ref var job = ref phaseTwoJobs.AllocateUnsafely();
                                job.Type = PhaseTwoJobType.CopyConstraintRegion;
                                job.CopyConstraintRegion = new CopyConstraintRegionJob
                                {
                                    TypeId = sourceTypeBatch.TypeId,
                                    Batch = batchIndex,
                                    SourceSet = sourceSetIndex,
                                    SourceTypeBatch = sourceTypeBatchIndex,
                                    TargetTypeBatch = targetTypeBatchIndex,
                                    Count = jobIndex >= remainder ? baseConstraintsPerJob : baseConstraintsPerJob + 1,
                                    SourceStart = previousSourceEnd,
                                    TargetStart = targetTypeBatch.ConstraintCount
                                };
                                previousSourceEnd += job.CopyConstraintRegion.Count;
                                var oldBundleCount = targetTypeBatch.BundleCount;
                                targetTypeBatch.ConstraintCount += job.CopyConstraintRegion.Count;
                                if (targetTypeBatch.BundleCount != oldBundleCount)
                                {
                                    //A new bundle was created; guarantee any trailing slots are set to -1.
                                    //Since it's a whole new bundle that has not yet had any data set to it, we can safely just initialize the whole bundle's body references to -1.
                                    var vectorCount = solver.TypeProcessors[job.CopyConstraintRegion.TypeId].BodiesPerConstraint;
                                    var bundleStart = (Vector<int>*)(targetTypeBatch.BodyReferences.Memory + (targetTypeBatch.BundleCount - 1) * vectorCount * Unsafe.SizeOf<Vector<int>>());
                                    var negativeOne = new Vector<int>(-1);
                                    for (int vectorIndex = 0; vectorIndex < vectorCount; ++vectorIndex)
                                    {
                                        bundleStart[vectorIndex] = negativeOne;
                                    }
                                }
                            }
                            Debug.Assert(previousSourceEnd == sourceTypeBatch.ConstraintCount);
                            Debug.Assert(targetTypeBatch.ConstraintCount <= targetTypeBatch.IndexToHandle.Length);
                        }
                    }
                    if (sourceSet.Batches.Count > fallbackIndex)
                    {
                        ref var sourceBatch = ref sourceSet.Batches[fallbackIndex];
                        ref var targetBatch = ref activeSolverSet.Batches[fallbackIndex];
                        for (int sourceTypeBatchIndex = 0; sourceTypeBatchIndex < sourceBatch.TypeBatches.Count; ++sourceTypeBatchIndex)
                        {
                            ref var sourceTypeBatch = ref sourceBatch.TypeBatches[sourceTypeBatchIndex];
                            var targetTypeBatchIndex = targetBatch.TypeIndexToTypeBatchIndex[sourceTypeBatch.TypeId];
                            if (!targetFallbackTypeBatchesToSources.FindOrAllocateSlot(targetTypeBatchIndex, pool, out var slotIndex))
                            {
                                targetFallbackTypeBatchesToSources.Values[slotIndex] = new QuickList<FallbackAddSource>(8, pool);
                            }
                            targetFallbackTypeBatchesToSources.Values[slotIndex].Allocate(pool) = new FallbackAddSource { SourceSet = sourceSetIndex, SourceTypeBatchIndex = sourceTypeBatchIndex };
                        }
                    }
                }
            }
            if (targetFallbackTypeBatchesToSources.Keys.Allocated)
            {
                phaseTwoJobs.EnsureCapacity(phaseTwoJobs.Count + targetFallbackTypeBatchesToSources.Count, pool);
                for (int i = 0; i < targetFallbackTypeBatchesToSources.Count; ++i)
                {
                    ref var job = ref phaseTwoJobs.AllocateUnsafely();
                    job.Type = PhaseTwoJobType.AddFallbackTypeBatchConstraints;
                    ref var list = ref targetFallbackTypeBatchesToSources.Values[i];
                    job.AddFallbackTypeBatchConstraints.Sources = list.Span.Slice(list.Count);
                    job.AddFallbackTypeBatchConstraints.TargetTypeBatch = targetFallbackTypeBatchesToSources.Keys[i];
                    job.AddFallbackTypeBatchConstraints.TypeId = solver.ActiveSet.Batches[solver.FallbackBatchThreshold].TypeBatches[job.AddFallbackTypeBatchConstraints.TargetTypeBatch].TypeId;
                }
                //Note that the per target lists will be disposed in the DisposeForCompletedAwakenings, since the spans created for the per-target lists are used in the PhaseTwoJobs.
                targetFallbackTypeBatchesToSources.Dispose(pool);
            }
            return (phaseOneJobs.Count, phaseTwoJobs.Count);
        }


        internal void DisposeForCompletedAwakenings(ref QuickList<int> setIndices)
        {
            Debug.Assert(setIndices.Count > 0 == phaseOneJobs.Span.Allocated && setIndices.Count > 0 == phaseTwoJobs.Span.Allocated);
            for (int i = 0; i < setIndices.Count; ++i)
            {
                var setIndex = setIndices[i];
                ref var bodySet = ref bodies.Sets[setIndex];
                //Note that neither the constraint set nor the pair cache set necessarily exist. It is possible for bodies to go to sleep by themselves.
                ref var constraintSet = ref solver.Sets[setIndex];
                ref var pairCacheSet = ref pairCache.SleepingSets[setIndex];
                Debug.Assert(bodySet.Allocated);
                bodySet.DisposeBuffers(pool);
                if (constraintSet.Allocated)
                {
                    constraintSet.Dispose(pool);
                }
                if (pairCacheSet.Allocated)
                    pairCacheSet.Dispose(pool);
                this.uniqueSetIndices = new QuickList<int>();
                sleeper.ReturnSetId(setIndex);

            }
            if (phaseOneJobs.Span.Allocated)
            {
                phaseOneJobs.Dispose(pool);
                for (int i = 0; i < phaseTwoJobs.Count; ++i)
                {
                    ref var job = ref phaseTwoJobs[i];
                    if (job.Type == PhaseTwoJobType.AddFallbackTypeBatchConstraints)
                    {
                        pool.Return(ref job.AddFallbackTypeBatchConstraints.Sources);
                    }
                }
                phaseTwoJobs.Dispose(pool);
            }
        }
    }
}
