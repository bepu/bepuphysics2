using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;

namespace BepuPhysics.CollisionDetection
{
    struct TypeBatchIndex
    {
        public short TypeBatch;
        public short Batch;
    }

    /// <summary>
    /// Accumulates constraints to remove from multiple threads, and efficiently removes them all as a batch.
    /// </summary>
    public class ConstraintRemover
    {
        internal Solver solver;
        internal Bodies bodies;
        BufferPool pool;

        internal struct PerBodyRemovalTarget
        {
            public int EncodedBodyIndex;
            public ConstraintHandle ConstraintHandle;

            public int BatchIndex;
            public BodyHandle BodyHandle;
        }


        struct RemovalsForTypeBatch
        {
            public QuickList<ConstraintHandle> ConstraintHandlesToRemove;
            public QuickList<PerBodyRemovalTarget> PerBodyRemovalTargets;
        }

        struct RemovalCache
        {
            public int BatchCount;
            public Buffer<TypeBatchIndex> TypeBatches;
            public Buffer<RemovalsForTypeBatch> RemovalsForTypeBatches;

            //Storing this stuff by constraint batch is an option, but it would require more prefiltering that isn't free.
            int minimumCapacityPerBatch;

            public RemovalCache(BufferPool pool, int batchCapacity, int minimumCapacityPerBatch)
            {
                this.minimumCapacityPerBatch = minimumCapacityPerBatch;

                pool.TakeAtLeast(batchCapacity, out TypeBatches);
                pool.TakeAtLeast(batchCapacity, out RemovalsForTypeBatches);
                BatchCount = 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int IndexOf(TypeBatchIndex typeBatchIndex)
            {
                //Working under the assumption that the number of batches/type batches will be pretty small, so using linear enumeration should beat a dictionary.
                ref var indexAsInt = ref Unsafe.As<TypeBatchIndex, int>(ref typeBatchIndex);
                for (int i = 0; i < BatchCount; ++i)
                {
                    if (indexAsInt == Unsafe.As<TypeBatchIndex, int>(ref TypeBatches[i]))
                        return i;
                }
                return -1;
            }
            public int AllocateSpaceForTargets(TypeBatchIndex typeBatchIndex, int constraintHandleCount, int perBodyRemovalCount, BufferPool pool)
            {
                var index = IndexOf(typeBatchIndex);
                if (index >= 0)
                {
                    ref var slot = ref RemovalsForTypeBatches[index];
                    Debug.Assert(slot.ConstraintHandlesToRemove.Span.Allocated && slot.PerBodyRemovalTargets.Span.Allocated);
                    slot.PerBodyRemovalTargets.EnsureCapacity(slot.PerBodyRemovalTargets.Count + perBodyRemovalCount, pool);
                    slot.ConstraintHandlesToRemove.EnsureCapacity(slot.ConstraintHandlesToRemove.Count + constraintHandleCount, pool);
                    return index;
                }
                index = BatchCount;
                BatchCount = BatchCount + 1;
                if (TypeBatches.Length == index)
                    pool.ResizeToAtLeast(ref TypeBatches, BatchCount, index);
                if (RemovalsForTypeBatches.Length == index)
                    pool.ResizeToAtLeast(ref RemovalsForTypeBatches, BatchCount, index);
                TypeBatches[index] = typeBatchIndex;
                ref var newSlot = ref RemovalsForTypeBatches[index];
                newSlot.ConstraintHandlesToRemove = new QuickList<ConstraintHandle>(Math.Max(constraintHandleCount, minimumCapacityPerBatch), pool);
                newSlot.PerBodyRemovalTargets = new QuickList<PerBodyRemovalTarget>(Math.Max(perBodyRemovalCount, minimumCapacityPerBatch), pool);
                return index;
            }

            public void Dispose(BufferPool pool)
            {
                pool.Return(ref TypeBatches);
                for (int i = 0; i < BatchCount; ++i)
                {
                    ref var removal = ref RemovalsForTypeBatches[i];
                    removal.PerBodyRemovalTargets.Dispose(pool);
                    removal.ConstraintHandlesToRemove.Dispose(pool);
                }
                pool.Return(ref RemovalsForTypeBatches);
                this = default;
            }
        }


        struct WorkerCache
        {

            internal BufferPool pool;
            public RemovalCache Removals;

            public WorkerCache(BufferPool pool, int batchCapacity, int minimumCapacityPerBatch)
            {
                this.pool = pool;
                Debug.Assert(minimumCapacityPerBatch > 0);
                Removals = new RemovalCache(pool, batchCapacity, minimumCapacityPerBatch);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void EnqueueForRemoval(ConstraintHandle constraintHandle, Solver solver, Bodies bodies)
            {
                ref var constraint = ref solver.HandleToConstraint[constraintHandle.Value];
                Debug.Assert(constraint.SetIndex == 0, "The constraint remover requires that the target constraint is active.");
                TypeBatchIndex typeBatchIndex;
                //Parallel removes are guaranteed to not change the constraint indices until all removes complete, so we can precache the type batch index here.
                //This allows us to collect the constraints to remove by type batch. Removes in different type batches can proceed in parallel.
                typeBatchIndex.Batch = (short)constraint.BatchIndex;
                ref var constraintBatch = ref solver.ActiveSet.Batches[constraint.BatchIndex];
                typeBatchIndex.TypeBatch = (short)constraintBatch.TypeIndexToTypeBatchIndex[constraint.TypeId];

                var typeProcessor = solver.TypeProcessors[constraint.TypeId];
                var bodiesPerConstraint = typeProcessor.BodiesPerConstraint;
                var linearTypeBatchIndex = Removals.AllocateSpaceForTargets(typeBatchIndex, 1, bodiesPerConstraint, pool);

                ref var typeBatchRemovals = ref Removals.RemovalsForTypeBatches[linearTypeBatchIndex];
                typeBatchRemovals.ConstraintHandlesToRemove.AllocateUnsafely() = constraintHandle;

                //Now extract and enqueue the body list constraint removal targets and the constraint batch body handle removal targets.
                //We have to perform the enumeration here rather than in the later flush. Removals from type batches make enumerating connected body indices a race condition there.
                ref var typeBatch = ref constraintBatch.TypeBatches[typeBatchIndex.TypeBatch];
                var encodedBodyIndices = stackalloc int[bodiesPerConstraint];
                var enumerator = new PassthroughReferenceCollector(encodedBodyIndices);
                solver.EnumerateConnectedRawBodyReferences(ref typeBatch, constraint.IndexInTypeBatch, ref enumerator);

                for (int i = 0; i < bodiesPerConstraint; ++i)
                {
                    ref var target = ref typeBatchRemovals.PerBodyRemovalTargets.AllocateUnsafely();
                    target.EncodedBodyIndex = encodedBodyIndices[i];
                    target.ConstraintHandle = constraintHandle;

                    target.BatchIndex = typeBatchIndex.Batch;
                    target.BodyHandle = bodies.ActiveSet.IndexToHandle[target.EncodedBodyIndex & Bodies.BodyReferenceMask];
                }
            }

            public void Dispose()
            {
                Removals.Dispose(pool);
            }
        }

        int previousCapacityPerBatch;
        int previousBatchCapacity;
        float previousCapacityMultiplier;
        int minimumConstraintCapacity;
        int minimumTypeCapacity;
        WorkerCache[] workerCaches; //there is a reference within the worker cache for the pool, so this can't be a buffer.
        int threadCount;

        public ConstraintRemover(BufferPool pool, Bodies bodies, Solver solver, int minimumTypeCapacity = 4, int minimumRemovalCapacity = 128, float previousCapacityMultiplier = 1.25f)
        {
            this.pool = pool;
            this.bodies = bodies;
            this.solver = solver;
            this.minimumConstraintCapacity = minimumRemovalCapacity;
            this.minimumTypeCapacity = minimumTypeCapacity;
            this.previousCapacityMultiplier = previousCapacityMultiplier;
        }


        public void Prepare(IThreadDispatcher dispatcher)
        {
            threadCount = dispatcher == null ? 1 : dispatcher.ThreadCount;
            //There aren't going to be that many workers or resizes of this array, so a managed reference is fine. Makes the storage of the buffer pool easier.
            if (workerCaches == null || workerCaches.Length < threadCount)
            {
                workerCaches = new WorkerCache[threadCount];
            }
            var batchCapacity = (int)Math.Max(minimumTypeCapacity, previousBatchCapacity * previousCapacityMultiplier);
            var capacityPerBatch = (int)Math.Max(minimumConstraintCapacity, previousCapacityPerBatch * previousCapacityMultiplier);
            if (dispatcher != null)
            {
                for (int i = 0; i < threadCount; ++i)
                {
                    //Note the use of per-thread pools. It is possible for the workers to resize the collections.
                    workerCaches[i] = new WorkerCache(dispatcher.GetThreadMemoryPool(i), batchCapacity, capacityPerBatch);
                }
            }
            else
            {
                workerCaches[0] = new WorkerCache(pool, batchCapacity, capacityPerBatch);
            }
            //The island sleeper job order requires this allocation to be done in the Prepare instead of CreateFlushJobs.
            if (solver.ActiveSet.Batches.Count > solver.FallbackBatchThreshold)
            {
                //Ensure that the fallback deallocation list is also large enough. The fallback batch may result in 3 returned buffers for the primary dictionary.
                //TODO: Since this is no longer a variable count, there's no reason to allocate a list like this.
                allocationIdsToFree = new QuickList<int>(3, pool);
            }
        }

        struct PerBodyRemovalTargetComparer : IComparerRef<PerBodyRemovalTarget>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref PerBodyRemovalTarget a, ref PerBodyRemovalTarget b)
            {
                //Constraint handles take precedence, but we have to distinguish between body handles for a globally unique sort.
                //(There are potentially multiple PerBodyRemovalTargets per constraint, since it creates one for each body associated with the constraint.)
                var aLong = ((long)a.ConstraintHandle.Value << 32) | (long)a.BodyHandle.Value;
                var bLong = ((long)b.ConstraintHandle.Value << 32) | (long)b.BodyHandle.Value;
                return aLong.CompareTo(bLong);
            }
        }
        struct TypeBatchIndexComparer : IComparerRef<TypeBatchIndex>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref TypeBatchIndex a, ref TypeBatchIndex b)
            {
                return Unsafe.As<TypeBatchIndex, int>(ref a).CompareTo(Unsafe.As<TypeBatchIndex, int>(ref b));
            }
        }

        //The general idea for multithreaded constraint removal is that there are varying levels of parallelism across the process.
        //You can't call solver.RemoveConstraint from multiple threads without locks, and we don't want to pay the price of constant syncs.
        //So instead, we identify the pieces of pretty-much-sequential work, and stick them into locally sequential jobs.
        //We then run them alongside other more parallel work. The hope is that the parallel work will fill in the gaps, balancing the work across all the threads
        //and limiting the worst case.

        //It's important to note that this is massively overkill when the simulation is only dealing with <20 removals. It's likely slower than just doing regular removes
        //in sequence at that point- sequential removes would cost around 5us in that case, so any kind of multithreaded overhead can overwhelm the work being done.
        //Doubling the cost of the best case, resulting in handfuls of wasted microseconds, isn't concerning (and we could special case it if we really wanted to).
        //Cutting the cost of the worst case when thousands of constraints get removed by a factor of ~ThreadCount is worth this complexity. Frame spikes are evil!

        RemovalCache batches;
        /// <summary>
        /// Processes enqueued constraint removals and prepares removal jobs.
        /// </summary>
        /// <param name="deterministic">True if the constraint remover should maintain determinism at an added cost, false otherwise.</param>
        /// <returns>The number of removal jobs created. To complete the jobs, execute RemoveConstraintsFromTypeBatch for every index from 0 to the returned job count.</returns>
        public int CreateFlushJobs(bool deterministic)
        {
            //Accumulate the set of unique type batches in a contiguous list so we can easily execute multithreaded jobs over them.
            //Note that we're actually copying here, rather than merely creating references. This simplifies the deterministic/nondeterministic split at a pretty tiny cost.
            batches = new RemovalCache(pool, 32, 8);
            var removedConstraintCount = 0;
            for (int i = 0; i < threadCount; ++i)
            {
                ref var cache = ref workerCaches[i];
                for (int j = 0; j < cache.Removals.BatchCount; ++j)
                {
                    ref var typeBatchIndex = ref cache.Removals.TypeBatches[j];
                    ref var workerRemovals = ref cache.Removals.RemovalsForTypeBatches[j];
                    removedConstraintCount += workerRemovals.ConstraintHandlesToRemove.Count;
                    var batchIndex = batches.AllocateSpaceForTargets(typeBatchIndex, workerRemovals.ConstraintHandlesToRemove.Count, workerRemovals.PerBodyRemovalTargets.Count, pool);

                    ref var combinedRemovalsForBatch = ref batches.RemovalsForTypeBatches[batchIndex];
                    combinedRemovalsForBatch.ConstraintHandlesToRemove.AddRangeUnsafely(workerRemovals.ConstraintHandlesToRemove.Span, 0, workerRemovals.ConstraintHandlesToRemove.Count);
                    combinedRemovalsForBatch.PerBodyRemovalTargets.AddRangeUnsafely(workerRemovals.PerBodyRemovalTargets.Span, 0, workerRemovals.PerBodyRemovalTargets.Count);

                }
            }
            if (deterministic)
            {
                //To ensure determinism, sort within each type batch by constraint handle (and, for removal targets, also by the body handle).
                //The handles are unique and deterministic, so processing the sorted list in order will produce deterministic results.
                //This (and perhaps the contiguous list gathering phase) could be punted to a multithreaded dispatch, but only do that if it actually seems beneficial. 
                var constraintHandleComparer = new PrimitiveComparer<int>();
                var perBodyRemovalTargetComparer = new PerBodyRemovalTargetComparer();
                for (int i = 0; i < batches.BatchCount; ++i)
                {
                    ref var batchRemovals = ref batches.RemovalsForTypeBatches[i];
                    QuickSort.Sort(ref batchRemovals.ConstraintHandlesToRemove.Span[0].Value, 0, batchRemovals.ConstraintHandlesToRemove.Count - 1, ref constraintHandleComparer);
                    QuickSort.Sort(ref batchRemovals.PerBodyRemovalTargets[0], 0, batchRemovals.PerBodyRemovalTargets.Count - 1, ref perBodyRemovalTargetComparer);
                }
                //Also sort the batches according to the type batch index. Note that batch indices/type batch indices are deterministic if the simulation is deterministic;
                //if they weren't, the solver would produce nondeterministic results.
                var typeBatchIndexComparer = new TypeBatchIndexComparer();
                QuickSort.Sort(ref batches.TypeBatches[0], ref batches.RemovalsForTypeBatches[0], 0, batches.BatchCount - 1, ref typeBatchIndexComparer);
            }

            //Ensure that the solver's id pool is large enough to hold all constraint handles being removed.
            //(Note that we do this even if we end up using this for sleeping, where we don't actually return the handles.
            //There's no functional reason for that- it's just simpler to not have a conditional API, and it has no significant impact on performance. Might change later.)
            solver.HandlePool.EnsureCapacity(solver.HandlePool.AvailableIdCount + removedConstraintCount, pool);

            //Ensure that the removal list is large enough to hold every single type batch in the worst case. This prevents the need to resize during execution.
            //That's valuable because every access to the main thread's buffer pool is a potential race condition when other tasks are also using it.
            //We don't want to have to lock every use of the buffer pool in other tasks just because we didn't preallocate a trivial amount here.
            int typeBatchCount = 0;
            ref var activeSet = ref solver.ActiveSet;
            for (int i = 0; i < activeSet.Batches.Count; ++i)
            {
                typeBatchCount += activeSet.Batches[i].TypeBatches.Count;
            }
            removedTypeBatches = new QuickList<TypeBatchIndex>(typeBatchCount, pool);
            return batches.BatchCount;
        }

        /// <summary>
        /// Returns the handles associated with all removed constraints to the solver's handle pool.
        /// </summary>
        public void ReturnConstraintHandles()
        {
            //Note that this does not zero out the slot associated with the handle. It is assumed that the typebatch removal is proceeding in parallel.
            //It will attempt to look up handle->index mappings, so we can't corrupt them.
            for (int i = 0; i < batches.BatchCount; ++i)
            {
                ref var batchHandles = ref batches.RemovalsForTypeBatches[i].ConstraintHandlesToRemove;
                for (int j = 0; j < batchHandles.Count; ++j)
                {
                    solver.HandlePool.ReturnUnsafely(batchHandles[j].Value);
                }
            }
        }

        public void RemoveConstraintsFromBodyLists()
        {
            //While body list removal could technically be internally multithreaded, it would be pretty complex- you would have to do one dispatch per solver.Batches batch
            //to guarantee that no two threads hit the same body constraint list at the same time. 
            //That is more complicated and would almost certainly be slower than this locally sequential version.
            for (int i = 0; i < batches.BatchCount; ++i)
            {
                ref var removals = ref batches.RemovalsForTypeBatches[i].PerBodyRemovalTargets;
                for (int j = 0; j < removals.Count; ++j)
                {
                    ref var target = ref removals[j];
                    if (bodies.RemoveConstraintReference(target.EncodedBodyIndex & Bodies.BodyReferenceMask, target.ConstraintHandle) && (target.EncodedBodyIndex & Bodies.KinematicMask) != 0)
                    {
                        //This is a kinematic, and it has no remaining constraint connections. Remove it from the solver constrained kinematic set.
                        var removed = solver.ConstrainedKinematicHandles.FastRemove(target.BodyHandle.Value);
                        Debug.Assert(removed, "The last constraint removed from a kinematic should see the body removed from the constrained kinematic set.");
                    }
                }
            }
        }

        public void RemoveConstraintsFromBatchReferencedHandles()
        {
            for (int i = 0; i < batches.BatchCount; ++i)
            {
                if (batches.TypeBatches[i].Batch == solver.FallbackBatchThreshold)
                {
                    //Batch referenced handles for the fallback are handled in RemoveConstraintsFromFallbackBatch. 
                    continue;
                }
                ref var removals = ref batches.RemovalsForTypeBatches[i].PerBodyRemovalTargets;
                for (int j = 0; j < removals.Count; ++j)
                {
                    ref var target = ref removals[j];
                    //Debug.Assert(solver.batchReferencedHandles[target.BatchIndex].Contains(target.BodyHandle.Value) || bodies.GetBodyReference(target.BodyHandle).Kinematic,
                    //    "The batch referenced handles must include all constraint-involved dynamics, but will not include kinematics.");
                    solver.batchReferencedHandles[target.BatchIndex].Unset(target.BodyHandle.Value);
                }
            }
        }

        QuickList<int> allocationIdsToFree;
        public void RemoveConstraintsFromFallbackBatchReferencedHandles()
        {
            Debug.Assert(solver.ActiveSet.Batches.Count > solver.FallbackBatchThreshold);
            for (int i = 0; i < batches.BatchCount; ++i)
            {
                ref var removals = ref batches.RemovalsForTypeBatches[i].PerBodyRemovalTargets;
                if (batches.TypeBatches[i].Batch == solver.FallbackBatchThreshold)
                {
                    for (int j = 0; j < removals.Count; ++j)
                    {
                        ref var target = ref removals[j];
                        if (solver.ActiveSet.SequentialFallback.RemoveOneBodyReferenceFromDynamicsSet(target.EncodedBodyIndex & Bodies.BodyReferenceMask, ref allocationIdsToFree))
                        {
                            //No more constraints for this body in the fallback set; it should not exist in the fallback batch's referenced handles anymore.
                            //Debug.Assert(solver.batchReferencedHandles[target.BatchIndex].Contains(target.BodyHandle.Value) || bodies.GetBodyReference(target.BodyHandle).Kinematic,
                            //    "The batch referenced handles must include all constraint-involved dynamics, but will not include kinematics.");
                            solver.batchReferencedHandles[target.BatchIndex].Unset(target.BodyHandle.Value);
                        }
                    }
                }
            }
        }
        public void TryRemoveBodyFromConstrainedKinematicsAndRemoveAllConstraintsForBodyFromFallbackBatch(BodyHandle bodyHandle, int bodyIndex)
        {
            solver.TryRemoveDynamicBodyFromFallback(bodyHandle, bodyIndex, ref allocationIdsToFree);
            //Note that we don't check kinematicity here. If it's dynamic, that's fine, this won't do anything.
            solver.ConstrainedKinematicHandles.FastRemove(bodyHandle.Value);
        }

        QuickList<TypeBatchIndex> removedTypeBatches;
        object batchRemovalLocker = new object();
        public void RemoveConstraintsFromTypeBatch(int index)
        {
            var batch = batches.TypeBatches[index];
            ref var constraintBatch = ref solver.ActiveSet.Batches[batch.Batch];
            ref var typeBatch = ref constraintBatch.TypeBatches[batch.TypeBatch];
            var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
            ref var removals = ref batches.RemovalsForTypeBatches[index];
            for (int i = 0; i < removals.ConstraintHandlesToRemove.Count; ++i)
            {
                var handle = removals.ConstraintHandlesToRemove[i];
                //Note that we look up the index in the type batch dynamically even though we could have cached it alongside batch and typebatch indices.
                //That's because removals can change the index, so caching indices would require sorting the indices for each type batch before removing.
                //That's very much doable, but not doing it is simpler, and the performance difference is likely trivial.
                //TODO: Likely worth testing.
                ref var location = ref solver.HandleToConstraint[handle.Value];
                typeProcessor.Remove(ref typeBatch, location.IndexInTypeBatch, ref solver.HandleToConstraint, location.BatchIndex == solver.FallbackBatchThreshold);
                if (typeBatch.ConstraintCount == 0)
                {
                    //This batch-typebatch needs to be removed.
                    //Note that we just use a lock here, nothing tricky- the number of typebatch/batch removals should tend to be extremely low (averaging 0),
                    //so it's not worth doing a bunch of per worker accumulators and stuff.
                    lock (batchRemovalLocker)
                    {
                        removedTypeBatches.AddUnsafely(batch);
                    }
                }
            }
        }

        struct TypeBatchComparer : IComparerRef<TypeBatchIndex>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref TypeBatchIndex a, ref TypeBatchIndex b)
            {
                return Unsafe.As<TypeBatchIndex, int>(ref b).CompareTo(Unsafe.As<TypeBatchIndex, int>(ref a));
            }
        }

        /// <summary>
        /// For uses of the ConstraintRemover that fully remove a constraint from the simulation (rather than simply moving it somewhere else),
        /// the handle->constraint mapping must be updated. This has to wait until after the multithreaded operations actually complete to avoid corrupting parallel operations.
        /// </summary>
        public void MarkAffectedConstraintsAsRemovedFromSolver()
        {
            for (int i = 0; i < batches.BatchCount; ++i)
            {
                ref var batchHandles = ref batches.RemovalsForTypeBatches[i].ConstraintHandlesToRemove;
                for (int j = 0; j < batchHandles.Count; ++j)
                {
                    //A negative set index is our marker for nonexistence.
                    solver.HandleToConstraint[batchHandles[j].Value].SetIndex = -1;
                }
            }
        }

        public void Postflush()
        {
            if (removedTypeBatches.Count > 0)
            {
                //Get rid of any type batches (and constraint batches) that became empty due to removals.
                //Sort the removed batches from highest to lowest so that higher index type batches and constraint batches get removed first.
                //This allows remove-by-pulling-last-index without corrupting other indices.
                var comparer = new TypeBatchComparer();
                QuickSort.Sort(ref removedTypeBatches[0], 0, removedTypeBatches.Count - 1, ref comparer);
                ref var activeSet = ref solver.ActiveSet;
                for (int i = 0; i < removedTypeBatches.Count; ++i)
                {
                    var batchIndices = removedTypeBatches[i];
                    ref var batch = ref activeSet.Batches[batchIndices.Batch];
                    ref var typeBatch = ref batch.TypeBatches[batchIndices.TypeBatch];
                    batch.RemoveTypeBatchIfEmpty(ref typeBatch, batchIndices.TypeBatch, solver.pool);
                    solver.RemoveBatchIfEmpty(ref batch, batchIndices.Batch);
                }
            }
            removedTypeBatches.Dispose(pool);

            if (allocationIdsToFree.Span.Allocated)
            {
                for (int i = 0; i < allocationIdsToFree.Count; ++i)
                {
                    pool.ReturnUnsafely(allocationIdsToFree[i]);
                }
                allocationIdsToFree.Dispose(pool);
            }
            batches.Dispose(pool);

            //Get rid of the worker cache allocations and store the capacities for next frame initialization.
            previousCapacityPerBatch = 0;
            for (int i = 0; i < threadCount; ++i)
            {
                ref var workerCache = ref workerCaches[i];
                for (int j = 0; j < workerCache.Removals.BatchCount; ++j)
                {
                    if (previousCapacityPerBatch < workerCache.Removals.RemovalsForTypeBatches[j].ConstraintHandlesToRemove.Count)
                        previousCapacityPerBatch = workerCache.Removals.RemovalsForTypeBatches[j].ConstraintHandlesToRemove.Count;
                }
                if (previousBatchCapacity < workerCache.Removals.BatchCount)
                    previousBatchCapacity = workerCache.Removals.BatchCount;
                workerCache.Dispose();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnqueueRemoval(int workerIndex, ConstraintHandle constraintHandle)
        {
            workerCaches[workerIndex].EnqueueForRemoval(constraintHandle, solver, bodies);
        }
    }
}
