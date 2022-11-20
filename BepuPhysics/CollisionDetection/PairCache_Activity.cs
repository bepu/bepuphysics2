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
        internal unsafe void ValidateHandleCountInMapping(ConstraintHandle constraintHandle, int expectedCount)
        {
            int count = 0;
            for (int i = 0; i < Mapping.Count; ++i)
            {
                var existingCache = Mapping.Values[i];
                if (existingCache.ConstraintHandle == constraintHandle)
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
                            ref var cache = ref Mapping.Values[elementIndex];
                            pairLocation.InactiveSetIndex = setIndex;
                            pairLocation.InactivePairIndex = builder.Add(pool, Mapping.Keys[elementIndex], cache);

                            //Now that any existing cache data has been moved into the inactive set, we should remove the overlap from the overlap mapping.
                            Mapping.FastRemove(tableIndex, elementIndex);
                        }
                    }
                }
            }
            builder.FinalizeSet(pool, out SleepingSets[setIndex]);
        }

        internal unsafe void AwakenSet(int setIndex)
        {
            ref var sleepingSet = ref SleepingSets[setIndex];
            //If there are no pairs, there is no need for an inactive set, so it's not guaranteed to be allocated.
            if (sleepingSet.Allocated)
            {
                for (int i = 0; i < sleepingSet.Pairs.Count; ++i)
                {
                    ref var pair = ref sleepingSet.Pairs[i];
                    Mapping.AddUnsafely(pair.Pair, pair.Cache);
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
