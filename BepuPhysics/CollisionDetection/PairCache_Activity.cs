using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    partial class PairCache
    {
        internal struct CollisionPairLocation
        {
            public CollidablePair Pair;
            //Used only when the collision pair was moved into a sleeping set.
            public int InactiveSetIndex;
            public int InactivePairIndex;
        }

        /// <summary>
        /// Mapping from constraint handle back to collision detection pair cache locations.
        /// </summary>
        internal Buffer<CollisionPairLocation> ConstraintHandleToPair;


        //This buffer is filled in parallel with the Bodies.Sets and Solver.Sets.
        //Note that this does not include the active set, so index 0 is always empty.
        internal Buffer<SleepingSet> SleepingSets;

        internal void ResizeSetsCapacity(int setsCapacity, int potentiallyAllocatedCount)
        {
            Debug.Assert(setsCapacity >= potentiallyAllocatedCount && potentiallyAllocatedCount <= SleepingSets.Length);
            setsCapacity = BufferPool.GetCapacityForCount<SleepingSet>(setsCapacity);
            if (SleepingSets.Length != setsCapacity)
            {
                var oldCapacity = SleepingSets.Length;
                pool.ResizeToAtLeast(ref SleepingSets, setsCapacity, potentiallyAllocatedCount);
                if (oldCapacity < SleepingSets.Length)
                    SleepingSets.Clear(oldCapacity, SleepingSets.Length - oldCapacity); //We rely on unused slots being default initialized.
            }
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateConstraintHandleToPairMapping()
        {
            ValidateConstraintHandleToPairMapping(ref workerCaches, false);
        }
        [Conditional("DEBUG")]
        internal unsafe void ValidateConstraintHandleToPairMappingInProgress(bool ignoreStale)
        {
            ValidateConstraintHandleToPairMapping(ref NextWorkerCaches, ignoreStale);
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateConstraintHandleToPairMapping(ref ArrayList<WorkerPairCache> caches, bool ignoreStale)
        {
            for (int i = 0; i < Mapping.Count; ++i)
            {
                if (!ignoreStale || PairFreshness[i] > 0)
                {
                    var existingCache = Mapping.Values[i].ConstraintCache;
                    var existingHandle = *(int*)(caches[existingCache.Cache].constraintCaches[existingCache.Type].Buffer.Memory + existingCache.Index);
                    Debug.Assert(existingCache.Active, "The overlap mapping should only contain references to constraints which are active.");
                    ref var pairLocation = ref ConstraintHandleToPair[existingHandle];
                    Debug.Assert(new CollidablePairComparer().Equals(ref ConstraintHandleToPair[existingHandle].Pair, ref Mapping.Keys[i]),
                        "The overlap mapping and handle mapping should match.");
                }
            }
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateHandleCountInMapping(ConstraintHandle constraintHandle, int expectedCount)
        {
            int count = 0;
            for (int i = 0; i < Mapping.Count; ++i)
            {
                var existingCache = Mapping.Values[i].ConstraintCache;
                var existingHandle = *(int*)(workerCaches[existingCache.Cache].constraintCaches[existingCache.Type].Buffer.Memory + existingCache.Index);
                if (existingHandle == constraintHandle.Value)
                {
                    ++count;
                    Debug.Assert(count <= expectedCount && count <= 1, "Expected count violated.");
                }
            }
            Debug.Assert(count == expectedCount, "Expected count for this handle not found!");
        }

        internal unsafe void SleepTypeBatchPairs(ref SleepingSetBuilder builder, int setIndex, Solver solver)
        {
            ref var constraintSet = ref solver.Sets[setIndex];
            for (int batchIndex = 0; batchIndex < constraintSet.Batches.Count; ++batchIndex)
            {
                ref var batch = ref constraintSet.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    Debug.Assert(typeBatch.ConstraintCount > 0, "If a type batch exists, it should contain constraints.");
                    if (NarrowPhase.IsContactConstraintType(typeBatch.TypeId))
                    {
                        for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.ConstraintCount; ++indexInTypeBatch)
                        {
                            var handle = typeBatch.IndexToHandle[indexInTypeBatch];
                            ref var pairLocation = ref ConstraintHandleToPair[handle.Value];
                            Mapping.GetTableIndices(ref pairLocation.Pair, out var tableIndex, out var elementIndex);
                            ref var cacheLocations = ref Mapping.Values[elementIndex];
                            Debug.Assert(cacheLocations.ConstraintCache.Exists);

                            pairLocation.InactiveSetIndex = setIndex;
                            pairLocation.InactivePairIndex = builder.Add(ref workerCaches, pool, ref Mapping.Keys[elementIndex], ref cacheLocations);

                            //Now that any existing cache data has been moved into the inactive set, we should remove the overlap from the overlap mapping.
                            Mapping.FastRemove(tableIndex, elementIndex);
                        }
                    }
                }
            }
            builder.FinalizeSet(pool, out SleepingSets[setIndex]);
        }

        internal ref WorkerPairCache GetCacheForAwakening()
        {
            //Note that the target location for the set depends on whether the awakening is being executed from within the context of the narrow phase.
            //Either way, we need to put the data into the most recently updated cache. If this is happening inside the narrow phase, that is the NextWorkerCaches,
            //because we haven't yet flipped the buffers. If it's outside of the narrow phase, then it's the current workerCaches. 
            //We can distinguish between the two by checking whether the NextWorkerCaches are allocated. They don't exist outside of the narrowphase's execution.

            //Also note that we only deal with one worker cache. Wake ups just dump new caches into the first thread. This works out since
            //the actual pair cache modification is locally sequential right now.
            if (NextWorkerCaches.Allocated && NextWorkerCaches.Count > 0 && NextWorkerCaches[0].collisionCaches.Allocated)
                return ref NextWorkerCaches[0];
            if (workerCaches.Allocated)
                return ref workerCaches[0];
            //No caches exist yet; this must be an external call taking place before the first update. Lazily initialize one worker cache.
            workerCaches = new ArrayList<WorkerPairCache>(1);
            var constraints = new QuickList<WorkerPairCache.PreallocationSizes>(1, pool);
            var collisions = new QuickList<WorkerPairCache.PreallocationSizes>(1, pool);
            workerCaches.AllocateUnsafely() = new WorkerPairCache(0, pool, ref constraints, ref collisions, 0);
            constraints.Dispose(pool);
            collisions.Dispose(pool);
            return ref workerCaches[0];
        }

        private unsafe PairCacheIndex CopyCacheForAwakening(ref Buffer<SleepingCache> inactiveCaches, ref Buffer<UntypedList> activeCaches, TypedIndex sourceCacheIndex)
        {
            ref var sourceCache = ref inactiveCaches[sourceCacheIndex.Type];
            //Note that the sourceCacheIndex.Type refers to the index of the type in a packed list, not the noncontiguous type id. 
            //The unpacked active caches use the noncontiguous type id, so the sourceCache.TypeId is now referenced rather than the sourceCacheIndex.Type.
            ref var targetCache = ref activeCaches[sourceCache.TypeId];
            var targetByteIndex = targetCache.Allocate(sourceCache.List.ElementSizeInBytes, sourceCache.List.Count, pool);
            Unsafe.CopyBlockUnaligned(targetCache.Buffer.Memory + targetByteIndex, sourceCache.List.Buffer.Memory + sourceCacheIndex.Index, (uint)sourceCache.List.ElementSizeInBytes);
            //Note that the cache chosen for activated entries is always the first one, so the cache index is simply 0.
            return new PairCacheIndex(0, sourceCache.TypeId, targetByteIndex);
        }
        internal unsafe void AwakenSet(int setIndex)
        {
            ref var sleepingSet = ref SleepingSets[setIndex];
            //If there are no pairs, there is no need for an inactive set, so it's not guaranteed to be allocated.
            if (sleepingSet.Allocated)
            {
                ref var activeSet = ref GetCacheForAwakening();
                //For simplicity, awakening simply walks the pairs list in the sleeping set.
                //By construction of the inactive set, the cache accesses will be highly cache coherent, so the fact that it doesn't do bulk copies isn't that bad.
                //(we COULD make it do bulk copies, but only bother with that if there is any reason to.)
                for (int i = 0; i < sleepingSet.Pairs.Count; ++i)
                {
                    ref var pair = ref sleepingSet.Pairs[i];
                    CollidablePairPointers pointers;
                    pointers.ConstraintCache = CopyCacheForAwakening(ref sleepingSet.ConstraintCaches, ref activeSet.constraintCaches, pair.ConstraintCache);
                    if (pair.CollisionCache.Exists)
                    {
                        pointers.CollisionDetectionCache = CopyCacheForAwakening(ref sleepingSet.CollisionCaches, ref activeSet.collisionCaches, pair.CollisionCache);
                    }
                    else
                    {
                        pointers.CollisionDetectionCache = new PairCacheIndex();
                    }
                    Mapping.AddUnsafely(ref pair.Pair, pointers);
                }
            }
        }

        internal void RemoveReferenceIfContactConstraint(ConstraintHandle handle, int typeId)
        {
            if (NarrowPhase.IsContactConstraintType(typeId))
            {
                var removed = Mapping.FastRemove(ref ConstraintHandleToPair[handle.Value].Pair);
                Debug.Assert(removed, "If a contact constraint is being directly removed, it must exist within the pair mapping- " +
                    "all *active* contact constraints do, and it's not valid to attempt to remove an inactive constraint.");
            }
        }
    }
}
