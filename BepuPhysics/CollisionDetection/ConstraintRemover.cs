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
    //generiiiiiiiiiiiics
    using Batches = QuickDictionary<
        TypeBatchIndex,
        QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>,
        Buffer<TypeBatchIndex>,
        Buffer<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>,
        Buffer<int>, TypeBatchIndexComparer>;

    struct WorkerBatchReference
    {
        public short WorkerIndex;
        public short WorkerBatchIndex;
    }

    struct TypeBatchIndex
    {
        public short TypeBatch;
        public short Batch;
    }
    struct TypeBatchIndexComparer : IEqualityComparerRef<TypeBatchIndex>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref TypeBatchIndex a, ref TypeBatchIndex b)
        {
            return Unsafe.As<TypeBatchIndex, int>(ref a) == Unsafe.As<TypeBatchIndex, int>(ref b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref TypeBatchIndex item)
        {
            return Unsafe.As<TypeBatchIndex, int>(ref item);
        }
    }
    public enum NarrowPhaseFlushJobType
    {
        UpdateConstraintBookkeeping,
        RemoveConstraintFromTypeBatch,
        FlushPairCacheChanges
    }

    public struct NarrowPhaseFlushJob
    {
        public NarrowPhaseFlushJobType Type;
        public int Index;
    }

    /// <summary>
    /// Accumulates constraints to remove from multiple threads, and efficiently removes them all as a batch.
    /// </summary>
    public class ConstraintRemover
    {
        Solver solver;
        Bodies bodies;
        BufferPool pool;

        struct WorkerCache
        {
            internal struct RemovalTarget
            {
                public int BodyIndex;
                public int ConstraintHandle;

                public int BatchIndex;
                public int BodyHandle;
            }

            internal BufferPool pool;
            internal QuickList<TypeBatchIndex, Buffer<TypeBatchIndex>> Batches;
            internal QuickList<QuickList<int, Buffer<int>>, Buffer<QuickList<int, Buffer<int>>>> BatchHandles;
            internal QuickList<RemovalTarget, Buffer<RemovalTarget>> RemovalTargets;

            //Storing this stuff by constraint batch is an option, but it would require more prefiltering that isn't free.
            int minimumCapacityPerBatch;

            public WorkerCache(BufferPool pool, int batchCapacity, int minimumCapacityPerBatch)
            {
                this.pool = pool;
                Debug.Assert(minimumCapacityPerBatch > 0);
                this.minimumCapacityPerBatch = minimumCapacityPerBatch;
                QuickList<TypeBatchIndex, Buffer<TypeBatchIndex>>.Create(pool.SpecializeFor<TypeBatchIndex>(), batchCapacity, out Batches);
                QuickList<QuickList<int, Buffer<int>>, Buffer<QuickList<int, Buffer<int>>>>.Create(pool.SpecializeFor<QuickList<int, Buffer<int>>>(), batchCapacity, out BatchHandles);
                QuickList<RemovalTarget, Buffer<RemovalTarget>>.Create(pool.SpecializeFor<RemovalTarget>(), batchCapacity, out RemovalTargets);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void EnqueueForRemoval(int constraintHandle, Solver solver, Bodies bodies)
            {
                ref var constraint = ref solver.HandleToConstraint[constraintHandle];
                Debug.Assert(constraint.SetIndex == 0, "The constraint remover requires that the target constraint is active.");
                TypeBatchIndex typeBatchIndex;
                //Parallel removes are guaranteed to not change the constraint indices until all removes complete, so we can precache the type batch index here.
                //This allows us to collect the constraints to remove by type batch. Removes in different type batches can proceed in parallel.
                typeBatchIndex.Batch = (short)constraint.BatchIndex;
                ref var constraintBatch = ref solver.ActiveSet.Batches[constraint.BatchIndex];
                typeBatchIndex.TypeBatch = (short)constraintBatch.TypeIndexToTypeBatchIndex[constraint.TypeId];

                int index = -1;
                //Note that we just scan for an existing type batch entry that matches the new one.
                //Given the limited number of constraint types and removes happening, this brute force approach will tend to be faster than a full dictionary.
                //(TODO: That's worth testing.)
                for (int i = 0; i < Batches.Count; ++i)
                {
                    if (Unsafe.As<TypeBatchIndex, int>(ref typeBatchIndex) == Unsafe.As<TypeBatchIndex, int>(ref Batches[i]))
                    {
                        //Guarantee we can hold the new handle before we add it. Ensuring capacity beforehand avoids the need for creating pools, and shares the add
                        //with the other no-index-found path.
                        ref var handles = ref BatchHandles[i];
                        if (handles.Span.Length == handles.Count)
                        {
                            //Buffers resize by powers of 2, no worry of incremental resizes.
                            handles.EnsureCapacity(handles.Count + 1, pool.SpecializeFor<int>());
                        }
                        index = i;
                        break;
                    }
                }
                if (index == -1)
                {
                    index = Batches.Count;
                    //Note that the spans are not of equal length; just test them independently.
                    //In the extremely unlikely event that this shows up in profiling, you can simply ensure batch handles capacity on the post-ensurecapacity Batches capacity.
                    if (Batches.Span.Length == Batches.Count)
                    {
                        var newCount = Batches.Count + 1;
                        Batches.EnsureCapacity(newCount, pool.SpecializeFor<TypeBatchIndex>());
                    }
                    if (BatchHandles.Span.Length == BatchHandles.Count)
                    {
                        var newCount = Batches.Count + 1;
                        BatchHandles.EnsureCapacity(newCount, pool.SpecializeFor<QuickList<int, Buffer<int>>>());
                    }
                    Batches.AllocateUnsafely() = typeBatchIndex;
                    ref var handles = ref BatchHandles.AllocateUnsafely();
                    QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), minimumCapacityPerBatch, out handles);
                }
                BatchHandles[index].AllocateUnsafely() = constraintHandle;

                //Now extract and enqueue the body list constraint removal targets and the constraint batch body handle removal targets.
                //We have to perform the enumeration here rather than in the later flush. Removals from type batches make enumerating connected body indices a race condition there.
                ref var typeBatch = ref constraintBatch.TypeBatches[typeBatchIndex.TypeBatch];
                BodyIndexCollector enumerator;
                var typeProcessor = solver.TypeProcessors[constraint.TypeId];
                var bodiesPerConstraint = typeProcessor.BodiesPerConstraint;
                var bodyIndices = stackalloc int[bodiesPerConstraint];
                enumerator.BodyIndices = bodyIndices;
                enumerator.IndexInConstraint = 0;
                typeProcessor.EnumerateConnectedBodyIndices(ref typeBatch, constraint.IndexInTypeBatch, ref enumerator);

                RemovalTargets.EnsureCapacity(RemovalTargets.Count + bodiesPerConstraint, pool.SpecializeFor<RemovalTarget>());
                for (int i = 0; i < bodiesPerConstraint; ++i)
                {
                    ref var target = ref RemovalTargets.AllocateUnsafely();
                    target.BodyIndex = bodyIndices[i];
                    target.ConstraintHandle = constraintHandle;

                    target.BatchIndex = typeBatchIndex.Batch;
                    target.BodyHandle = bodies.ActiveSet.IndexToHandle[bodyIndices[i]];
                }
            }

            unsafe struct BodyIndexCollector : IForEach<int>
            {
                internal int* BodyIndices;
                internal int IndexInConstraint;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void LoopBody(int bodyIndex)
                {
                    BodyIndices[IndexInConstraint++] = bodyIndex;
                }
            }


            public void Dispose()
            {
                Batches.Dispose(pool.SpecializeFor<TypeBatchIndex>());
                var intPool = pool.SpecializeFor<int>();
                for (int i = 0; i < BatchHandles.Count; ++i)
                {
                    BatchHandles[i].Dispose(intPool);
                }
                BatchHandles.Dispose(pool.SpecializeFor<QuickList<int, Buffer<int>>>());
                RemovalTargets.Dispose(pool.SpecializeFor<RemovalTarget>());
                this = new WorkerCache();
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

        Batches batches;
        public void CreateFlushJobs(ref QuickList<NarrowPhaseFlushJob, Buffer<NarrowPhaseFlushJob>> jobs)
        {
            //Add the locally sequential jobs. Put them first in the hope that the usually-smaller per-typebatch jobs will balance out the remainder of the work.
            var jobPool = pool.SpecializeFor<NarrowPhaseFlushJob>();
            jobs.Add(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.UpdateConstraintBookkeeping }, jobPool);
            //TODO: For deactivation, you don't actually want to create a body list removal request. The bodies would be getting removed, so it would be redundant.
            //Simple enough to adapt for that use case later. Probably need to get rid of the narrow phase specific reference.

            //Accumulate the set of unique type batches in a contiguous list so we can easily execute multithreaded jobs over them.
            //Note that we're not actually copying over the contents of the per-worker lists here- just storing a reference to the per-worker lists.
            Batches.Create(pool.SpecializeFor<TypeBatchIndex>(), pool.SpecializeFor<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>(), pool.SpecializeFor<int>(),
                7, 3, out batches);
            var typeBatchIndexPool = pool.SpecializeFor<TypeBatchIndex>();
            var quickListPool = pool.SpecializeFor<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>();
            var intPool = pool.SpecializeFor<int>();

            for (int i = 0; i < threadCount; ++i)
            {
                ref var cache = ref workerCaches[i];
                for (int j = 0; j < cache.Batches.Count; ++j)
                {
                    var batchIndex = batches.IndexOf(cache.Batches[j]);
                    if (batchIndex >= 0)
                    {
                        ref var slot = ref batches.Values[batchIndex].AllocateUnsafely();
                        slot.WorkerIndex = (short)i;
                        slot.WorkerBatchIndex = (short)j;
                    }
                    else
                    {
                        //This worker batch doesn't exist in the combined set. Add it now.
                        QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>.Create(pool.SpecializeFor<WorkerBatchReference>(), threadCount - i, out var references);
                        WorkerBatchReference reference;
                        reference.WorkerIndex = (short)i;
                        reference.WorkerBatchIndex = (short)j;
                        references.AddUnsafely(reference);
                        jobs.Add(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.RemoveConstraintFromTypeBatch, Index = batches.Count }, jobPool);
                        batches.Add(ref cache.Batches[j], ref references, typeBatchIndexPool, quickListPool, intPool);
                    }
                }
            }

            //Ensure that the removal list is large enough to hold every single type batch in the worst case. This prevents the need to resize during execution.
            //That's valuable because every access to the main thread's buffer pool is a potential race condition when other tasks are also using it.
            //We don't want to have to lock every use of the buffer pool in other tasks just because we didn't preallocate a trivial amount here.
            int typeBatchCount = 0;
            ref var activeSet = ref solver.ActiveSet;
            for (int i = 0; i < activeSet.Batches.Count; ++i)
            {
                typeBatchCount += activeSet.Batches[i].TypeBatches.Count;
            }
            QuickList<TypeBatchIndex, Buffer<TypeBatchIndex>>.Create(pool.SpecializeFor<TypeBatchIndex>(), typeBatchCount, out removedTypeBatches);
        }


        public void UpdateConstraintBookkeeping(bool deterministic)
        {
            //While body list removal could technically be internally multithreaded, it would be pretty complex- you would have to do one dispatch per solver.Batches batch
            //to guarantee that no two threads hit the same body constraint list at the same time. 
            //That is more complicated and would almost certainly be slower than this locally sequential version.
            for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
            {
                ref var workerCache = ref workerCaches[workerIndex];
                for (int removalTargetIndex = 0; removalTargetIndex < workerCache.RemovalTargets.Count; ++removalTargetIndex)
                {
                    ref var target = ref workerCache.RemovalTargets[removalTargetIndex];
                    bodies.RemoveConstraint(target.BodyIndex, target.ConstraintHandle);
                    solver.batchReferencedHandles[target.BatchIndex].Remove(target.BodyHandle);
                }
            }
            var intPool = pool.SpecializeFor<int>();

            //Note that the handles are also removed here. Even though the action is independent, any resizes of the internal id pool structure would share acceses to the main thread's
            //buffer pool. Removing constraints is the other place where the main thread's buffer pool is used.
            //Doesn't matter too much; the total cost of this stage is very low.
            if (deterministic)
            {
                //The batch compressor requires constraint handles to be deterministic. While that could be changed, ensuring handle determinism fits conceptually
                //with the user-managed constraint handles- they are all deterministic, so the contact ones might as well be too.
                int count = 0;
                for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
                {
                    ref var workerCache = ref workerCaches[workerIndex];
                    for (int batchIndex = 0; batchIndex < workerCache.BatchHandles.Count; ++batchIndex)
                    {
                        count += workerCache.BatchHandles[batchIndex].Count;
                    }
                }
                if (count > 0)
                {
                    intPool.Take(count, out var sortedHandles);
                    int sortIndex = 0;
                    for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
                    {
                        ref var workerCache = ref workerCaches[workerIndex];
                        for (int batchIndex = 0; batchIndex < workerCache.BatchHandles.Count; ++batchIndex)
                        {
                            ref var handles = ref workerCache.BatchHandles[batchIndex];
                            for (int handleIndex = 0; handleIndex < handles.Count; ++handleIndex)
                            {
                                sortedHandles[sortIndex++] = handles[handleIndex];
                            }
                        }
                    }
                    var comparer = new PrimitiveComparer<int>();
                    QuickSort.Sort(ref sortedHandles[0], 0, count - 1, ref comparer);
                    for (int i = 0; i < count; ++i)
                    {
                        solver.HandlePool.Return(sortedHandles[i], intPool);
                    }
                    intPool.Return(ref sortedHandles);
                }
            }
            else
            {
                for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
                {
                    ref var workerCache = ref workerCaches[workerIndex];
                    //If it's nondeterministic anyway, don't worry about the handle ordering.
                    for (int batchIndex = 0; batchIndex < workerCache.BatchHandles.Count; ++batchIndex)
                    {
                        ref var handles = ref workerCache.BatchHandles[batchIndex];
                        for (int handleIndex = 0; handleIndex < handles.Count; ++handleIndex)
                        {
                            solver.HandlePool.Return(handles[handleIndex], intPool);
                        }
                    }
                }
            }

        }

        QuickList<TypeBatchIndex, Buffer<TypeBatchIndex>> removedTypeBatches;
        SpinLock removedTypeBatchLocker = new SpinLock();
        public void RemoveConstraintsFromTypeBatch(int index)
        {
            var batch = batches.Keys[index];
            ref var constraintBatch = ref solver.ActiveSet.Batches[batch.Batch];
            ref var typeBatch = ref constraintBatch.TypeBatches[batch.TypeBatch];
            var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
            ref var batchReferences = ref batches.Values[index];
            bool lockTaken = false;
            for (int i = 0; i < batchReferences.Count; ++i)
            {
                ref var reference = ref batchReferences[i];
                ref var workerCache = ref workerCaches[reference.WorkerIndex];
                ref var handles = ref workerCache.BatchHandles[reference.WorkerBatchIndex];
                for (int j = 0; j < handles.Count; ++j)
                {
                    var handle = handles[j];
                    //Note that we look up the index in the type batch dynamically even though we could have cached it alongside batch and typebatch indices.
                    //That's because removals can change the index, so caching indices would require sorting the indices for each type batch before removing.
                    //That's very much doable, but not doing it is simpler, and the performance difference is likely trivial.
                    //TODO: Likely worth testing.
                    typeProcessor.Remove(ref typeBatch, solver.HandleToConstraint[handle].IndexInTypeBatch, ref solver.HandleToConstraint);
                    if (typeBatch.ConstraintCount == 0)
                    {
                        //This batch-typebatch needs to be removed.
                        //Note that we just use a spinlock here, nothing tricky- the number of typebatch/batch removals should tend to be extremely low (averaging 0),
                        //so it's not worth doing a bunch of per worker accumulators and stuff.
                        removedTypeBatchLocker.Enter(ref lockTaken);
                        removedTypeBatches.AddUnsafely(batch);
                        removedTypeBatchLocker.Exit();
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
                    batch.RemoveTypeBatchIfEmpty(ref typeBatch, batchIndices.TypeBatch, solver.bufferPool);
                    solver.RemoveBatchIfEmpty(ref batch, batchIndices.Batch);
                }
            }
            removedTypeBatches.Dispose(pool.SpecializeFor<TypeBatchIndex>());

            //Get rid of the worker batch reference collections.
            for (int i = 0; i < batches.Count; ++i)
            {
                batches.Values[i].Dispose(pool.SpecializeFor<WorkerBatchReference>());
            }
            batches.Dispose(pool.SpecializeFor<TypeBatchIndex>(), pool.SpecializeFor<QuickList<WorkerBatchReference, Buffer<WorkerBatchReference>>>(), pool.SpecializeFor<int>());

            //Get rid of the worker cache allocations and store the capacities for next frame initialization.
            previousCapacityPerBatch = 0;
            for (int i = 0; i < threadCount; ++i)
            {
                ref var workerCache = ref workerCaches[i];
                for (int j = 0; j < workerCache.BatchHandles.Count; ++j)
                {
                    if (previousCapacityPerBatch < workerCache.BatchHandles[j].Count)
                        previousCapacityPerBatch = workerCache.BatchHandles[j].Count;
                }
                if (previousBatchCapacity < workerCache.BatchHandles.Count)
                    previousBatchCapacity = workerCache.BatchHandles.Count;
                workerCache.Dispose();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnqueueRemoval(int workerIndex, int constraintHandle)
        {
            workerCaches[workerIndex].EnqueueForRemoval(constraintHandle, solver, bodies);
        }
    }
}
