using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
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
        public void AwakenBody(int bodyHandle)
        {
            bodies.ValidateExistingHandle(bodyHandle);
            AwakenSet(bodies.HandleToLocation[bodyHandle].SetIndex);
        }

        /// <summary>
        /// Wakes up any sleeping bodies associated with a constraint. All bodies that can be found by traversing the constraint graph from the constraint referenced bodies will also be awakened.
        /// If all bodies associated with the constraint are already awake, this does nothing.
        /// </summary>
        /// <param name="constraintHandle">Handle of the constraint to awaken.</param>
        public void AwakenConstraint(int constraintHandle)
        {
            AwakenSet(solver.HandleToConstraint[constraintHandle].SetIndex);
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
                threadDispatcher.DispatchWorkers(phaseOneWorkerDelegate);
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
                threadDispatcher.DispatchWorkers(phaseTwoWorkerDelegate);
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
        }
        struct PhaseTwoJob
        {
            public PhaseTwoJobType Type;
            public int SourceStart;
            public int TargetStart;
            public int Count;
            public int SourceSet;
            public int TypeId;
            public int Batch;
            public int SourceTypeBatch;
            public int TargetTypeBatch;
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
                        Debug.Assert(job.BatchIndex < solver.FallbackBatchThreshold, "The fallback batch doesn't have any referenced handles to update!");
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
                            ref var source = ref solver.Sets[uniqueSetIndices[i]].Fallback;
                            ref var target = ref solver.ActiveSet.Fallback;
                            if (source.bodyConstraintReferences.Count > 0)
                            {
                                for (int j = 0; j < source.bodyConstraintReferences.Count; ++j)
                                {
                                    //Inactive sets refer to body handles. Active set refers to body indices. Make the transition.
                                    //The HandleToLocation was updated during job setup, so we can use it.
                                    ref var bodyLocation = ref bodies.HandleToLocation[source.bodyConstraintReferences.Keys[j]];
                                    Debug.Assert(bodyLocation.SetIndex == 0, "Any batch moved into the active set should be dealing with bodies which have already been moved into the active set.");
                                    var added = target.bodyConstraintReferences.AddUnsafelyRef(ref bodyLocation.Index, source.bodyConstraintReferences.Values[j]);
                                    Debug.Assert(added, "Any body moving from an inactive set to the active set should not already be present in the active set's fallback batch.");
                                }
                                //We've reused the lists. Set the count to zero so they don't get disposed later.
                                source.bodyConstraintReferences.Count = 0;
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
                        sourceSet.Collidables.CopyTo(job.SourceStart, ref targetSet.Collidables, job.TargetStart, job.Count);
                        sourceSet.Constraints.CopyTo(job.SourceStart, ref targetSet.Constraints, job.TargetStart, job.Count);
                        //The world inertias must be updated as well. They are stored outside the sets.
                        //Note that we use a manual loop copy for the local inertias and poses since we're accessing them during the world inertia calculation anyway.
                        //This can worsen the copy codegen a little, but it means we only have to scan the memory once.
                        //(Realistically, either option is fast- these regions won't tend to fill L1.) 
                        for (int i = 0; i < job.Count; ++i)
                        {
                            var sourceIndex = job.SourceStart + i;
                            var targetIndex = job.TargetStart + i;
                            ref var targetWorldInertia = ref bodies.Inertias[targetIndex];
                            ref var sourceLocalInertia = ref sourceSet.LocalInertias[sourceIndex];
                            ref var targetLocalInertia = ref targetSet.LocalInertias[targetIndex];
                            ref var sourcePose = ref sourceSet.Poses[sourceIndex];
                            ref var targetPose = ref targetSet.Poses[targetIndex];
                            targetPose = sourcePose;
                            targetLocalInertia = sourceLocalInertia;
                            PoseIntegration.RotateInverseInertia(sourceLocalInertia.InverseInertiaTensor, sourcePose.Orientation, out targetWorldInertia.InverseInertiaTensor);
                            targetWorldInertia.InverseMass = sourceLocalInertia.InverseMass;
                        }
                        sourceSet.Velocities.CopyTo(job.SourceStart, ref targetSet.Velocities, job.TargetStart, job.Count);
                        sourceSet.Activity.CopyTo(job.SourceStart, ref targetSet.Activity, job.TargetStart, job.Count);
                        if (resetActivityStates)
                        {
                            for (int targetIndex = job.TargetStart + job.Count - 1; targetIndex >= job.TargetStart; --targetIndex)
                            {
                                ref var targetActivity = ref targetSet.Activity[targetIndex];
                                targetActivity.TimestepsUnderThresholdCount = 0;
                                targetActivity.SleepCandidate = false;
                            }
                        }
                        sourceSet.IndexToHandle.CopyTo(job.SourceStart, ref targetSet.IndexToHandle, job.TargetStart, job.Count);
                    }
                    break;
            }
        }


        internal unsafe void ExecutePhaseTwoJob(int index)
        {
            ref var job = ref phaseTwoJobs[index];
            switch (job.Type)
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
                                ref var bodyLocation = ref bodies.HandleToLocation[sleepingBodySet.IndexToHandle[j]];
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
                                            statics.Collidables[statics.HandleToIndex[movedLeaf.Handle]].BroadPhaseIndex = staticBroadPhaseIndexToRemove;
                                        }
                                        else
                                        {
                                            //Note that the moved leaf cannot refer to one of the collidables that we've already moved into the active set, because all such collidables
                                            //have already been removed. 
                                            bodies.UpdateCollidableBroadPhaseIndex(movedLeaf.Handle, staticBroadPhaseIndexToRemove);
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
                        ref var sourceTypeBatch = ref solver.Sets[job.SourceSet].Batches[job.Batch].TypeBatches[job.SourceTypeBatch];
                        ref var targetTypeBatch = ref solver.ActiveSet.Batches[job.Batch].TypeBatches[job.TargetTypeBatch];
                        Debug.Assert(targetTypeBatch.TypeId == sourceTypeBatch.TypeId);
                        solver.TypeProcessors[job.TypeId].CopySleepingToActive(
                            job.SourceSet, job.Batch, job.SourceTypeBatch, job.Batch, job.TargetTypeBatch,
                            job.SourceStart, job.TargetStart, job.Count, bodies, solver);
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

        struct TypeAllocationSizes<T> where T : struct, ITypeCount
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


        internal (int phaseOneJobCount, int phaseTwoJobCount) PrepareJobs(ref QuickList<int> setIndices, bool resetActivityStates, int threadCount)
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
                additionalRequiredFallbackCapacity += constraintSet.Fallback.BodyCount;
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
            //broad phase, (technically overestimating, not every body has a collidable, but vast majority do and shrug)
            broadPhase.EnsureCapacity(broadPhase.ActiveTree.LeafCount + newBodyCount, broadPhase.StaticTree.LeafCount);
            //constraints,
            solver.ActiveSet.Batches.EnsureCapacity(highestNewBatchCount, pool);
            if (additionalRequiredFallbackCapacity > 0)
                solver.ActiveSet.Fallback.EnsureCapacity(solver.ActiveSet.Fallback.BodyCount + additionalRequiredFallbackCapacity, pool);
            solver.batchReferencedHandles.EnsureCapacity(Math.Min(solver.FallbackBatchThreshold, highestNewBatchCount), pool);
            for (int batchIndex = solver.ActiveSet.Batches.Count; batchIndex < highestNewBatchCount; ++batchIndex)
            {
                solver.ActiveSet.Batches.AllocateUnsafely() = new ConstraintBatch(pool);
                //The fallback batch has no batch referenced handles.
                if (batchIndex < solver.FallbackBatchThreshold)
                {
                    solver.batchReferencedHandles.AllocateUnsafely() = new IndexSet(pool, bodies.HandlePool.HighestPossiblyClaimedId + 1);
                }
            }
            for (int batchIndex = 0; batchIndex < highestNewBatchCount; ++batchIndex)
            {
                ref var constraintCountPerType = ref constraintCountPerTypePerBatch[batchIndex];
                ref var batch = ref solver.ActiveSet.Batches[batchIndex];
                batch.EnsureTypeMapSize(pool, constraintCountPerType.HighestOccupiedTypeIndex);
                //The fallback batch has no batch referenced handles.
                if (batchIndex < solver.FallbackBatchThreshold)
                {
                    solver.batchReferencedHandles[batchIndex].EnsureCapacity(bodies.HandlePool.HighestPossiblyClaimedId + 1, pool);
                }
                for (int typeId = 0; typeId <= constraintCountPerType.HighestOccupiedTypeIndex; ++typeId)
                {
                    var countForType = constraintCountPerType.TypeCounts[typeId].Count;
                    if (countForType > 0)
                    {
                        var typeProcessor = solver.TypeProcessors[typeId];
                        ref var typeBatch = ref batch.GetOrCreateTypeBatch(typeId, typeProcessor, countForType, pool);
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
            //Don't create batch referenced handles update jobs for the fallback batch; it has no referenced handles!
            var highestSynchronizedBatchCount = Math.Min(solver.FallbackBatchThreshold, highestNewBatchCount);
            for (int batchIndex = 0; batchIndex < highestSynchronizedBatchCount; ++batchIndex)
            {
                phaseOneJobs.AllocateUnsafely() = new PhaseOneJob { Type = PhaseOneJobType.UpdateBatchReferencedHandles, BatchIndex = batchIndex };
            }
            phaseTwoJobs.AllocateUnsafely() = new PhaseTwoJob { Type = PhaseTwoJobType.BroadPhase };

            ref var activeBodySet = ref bodies.ActiveSet;
            ref var activeSolverSet = ref solver.ActiveSet;
            //TODO: The job sizes are a little goofy for single threaded execution. Easy enough to resolve with special case or dynamic size.
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
                            ref var bodyLocation = ref bodies.HandleToLocation[sourceSet.IndexToHandle[sourceIndex]];
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
                    for (int batchIndex = 0; batchIndex < sourceSet.Batches.Count; ++batchIndex)
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
                                job.TypeId = sourceTypeBatch.TypeId;
                                job.Batch = batchIndex;
                                job.SourceSet = sourceSetIndex;
                                job.SourceTypeBatch = sourceTypeBatchIndex;
                                job.TargetTypeBatch = targetTypeBatchIndex;
                                job.Count = jobIndex >= remainder ? baseConstraintsPerJob : baseConstraintsPerJob + 1;
                                job.SourceStart = previousSourceEnd;
                                job.TargetStart = targetTypeBatch.ConstraintCount;
                                previousSourceEnd += job.Count;
                                targetTypeBatch.ConstraintCount += job.Count;
                            }
                            Debug.Assert(previousSourceEnd == sourceTypeBatch.ConstraintCount);
                            Debug.Assert(targetTypeBatch.ConstraintCount <= targetTypeBatch.IndexToHandle.Length);
                        }
                    }
                }
            }
            return (phaseOneJobs.Count, phaseTwoJobs.Count);
        }


        internal void DisposeForCompletedAwakenings(ref QuickList<int> setIndices)
        {
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
            phaseOneJobs.Dispose(pool);
            phaseTwoJobs.Dispose(pool);
        }
    }
}
