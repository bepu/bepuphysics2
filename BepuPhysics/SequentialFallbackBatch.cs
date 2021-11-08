using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    interface IBodyReferenceGetter
    {
        int GetBodyReference(Bodies bodies, BodyHandle handle);
    }

    struct ActiveSetGetter : IBodyReferenceGetter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetBodyReference(Bodies bodies, BodyHandle bodyHandle)
        {
            ref var bodyLocation = ref bodies.HandleToLocation[bodyHandle.Value];
            Debug.Assert(bodyLocation.SetIndex == 0, "When creating a fallback batch for the active set, all bodies associated with it must be active.");
            return bodyLocation.Index;
        }
    }
    struct InactiveSetGetter : IBodyReferenceGetter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetBodyReference(Bodies bodies, BodyHandle bodyHandle)
        {
            return bodyHandle.Value;
        }
    }

    /// <summary>
    /// Contains constraints that could not belong to any lower constraint batch due to their involved bodies. All of the contained constraints will be solved using a fallback solver that
    /// trades rigidity for parallelism.
    /// </summary>
    public struct SequentialFallbackBatch
    {
        /// <summary>
        /// Gets the number of bodies in the fallback batch.
        /// </summary>
        public readonly int BodyCount { get { return dynamicBodyConstraintCounts.Count; } }

        //In order to maintain the batch referenced handles for the fallback batch (which can have the same body appear more than once),
        //every body must maintain a count of fallback constraints associated with it.
        //Note that this dictionary uses active set body *indices* while active, but body *handles* when associated with an inactive set.
        //This is consistent with the body references stored by active/inactive constraints.
        internal QuickDictionary<int, int, PrimitiveComparer<int>> dynamicBodyConstraintCounts;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Allocate<TBodyReferenceGetter>(Span<BodyHandle> dynamicBodyHandles, Bodies bodies,
            BufferPool pool, TBodyReferenceGetter bodyReferenceGetter, int minimumBodyCapacity)
            where TBodyReferenceGetter : struct, IBodyReferenceGetter
        {
            EnsureCapacity(Math.Max(dynamicBodyConstraintCounts.Count + dynamicBodyHandles.Length, minimumBodyCapacity), pool);
            for (int i = 0; i < dynamicBodyHandles.Length; ++i)
            {
                var bodyReference = bodyReferenceGetter.GetBodyReference(bodies, dynamicBodyHandles[i]);

                if (dynamicBodyConstraintCounts.FindOrAllocateSlotUnsafely(bodyReference, out var slotIndex))
                {
                    ++dynamicBodyConstraintCounts.Values[slotIndex];
                }
                else
                {
                    dynamicBodyConstraintCounts.Values[slotIndex] = 1;
                }
            }
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void AllocateForActive(Span<BodyHandle> dynamicBodyHandles, Bodies bodies,
           BufferPool pool, int minimumBodyCapacity = 8)
        {
            Allocate(dynamicBodyHandles, bodies, pool, new ActiveSetGetter(), minimumBodyCapacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void AllocateForInactive(Span<BodyHandle> dynamicBodyHandles, Bodies bodies,
          BufferPool pool, int minimumBodyCapacity = 8)
        {
            Allocate(dynamicBodyHandles, bodies, pool, new InactiveSetGetter(), minimumBodyCapacity);
        }


        /// <summary>
        /// Removes a constraint from a body in the fallback batch.
        /// </summary>
        /// <param name="bodyReference">Body associated with a constraint in the fallback batch.</param>
        /// <param name="allocationIdsToFree">Allocations that should be freed once execution is back in a safe context.</param>
        /// <returns>True if the body was dynamic and no longer has any constraints associated with it in the fallback batch, false otherwise.</returns>
        internal unsafe bool RemoveOneBodyReferenceFromDynamicsSet(int bodyReference, ref QuickList<int> allocationIdsToFree)
        {
            if (!dynamicBodyConstraintCounts.GetTableIndices(ref bodyReference, out var tableIndex, out var bodyReferencesIndex))
                return false;
            ref var constraintCount = ref dynamicBodyConstraintCounts.Values[bodyReferencesIndex];
            --constraintCount;
            if (constraintCount == 0)
            {
                //If there are no more constraints associated with this body, get rid of the body list.
                constraintCount = default;
                dynamicBodyConstraintCounts.FastRemove(tableIndex, bodyReferencesIndex);
                if (dynamicBodyConstraintCounts.Count == 0)
                {
                    //No constraints remain in the fallback batch. Drop the dictionary.
                    allocationIdsToFree.AllocateUnsafely() = dynamicBodyConstraintCounts.Keys.Id;
                    allocationIdsToFree.AllocateUnsafely() = dynamicBodyConstraintCounts.Values.Id;
                    allocationIdsToFree.AllocateUnsafely() = dynamicBodyConstraintCounts.Table.Id;
                    dynamicBodyConstraintCounts = default;
                }
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes a body from the fallback batch's dynamic body constraint counts if it is present.
        /// </summary>
        /// <param name="bodyReference">Reference to the body to remove from the fallback batch.</param>
        /// <param name="allocationIdsToFree">Allocations that should be freed once execution is back in a safe context.</param>
        /// <returns>True if the body was present in the fallback batch and was removed, false otherwise.</returns>
        internal unsafe bool TryRemoveDynamicBodyFromTracking(int bodyReference, ref QuickList<int> allocationIdsToFree)
        {
            if (dynamicBodyConstraintCounts.Keys.Allocated && dynamicBodyConstraintCounts.GetTableIndices(ref bodyReference, out var tableIndex, out var bodyReferencesIndex))
            {
                ref var constraintReferences = ref dynamicBodyConstraintCounts.Values[bodyReferencesIndex];
                //If there are no more constraints associated with this body, get rid of the body list.
                dynamicBodyConstraintCounts.FastRemove(tableIndex, bodyReferencesIndex);
                if (dynamicBodyConstraintCounts.Count == 0)
                {
                    //No constraints remain in the fallback batch. Drop the dictionary.
                    allocationIdsToFree.AllocateUnsafely() = dynamicBodyConstraintCounts.Keys.Id;
                    allocationIdsToFree.AllocateUnsafely() = dynamicBodyConstraintCounts.Values.Id;
                    allocationIdsToFree.AllocateUnsafely() = dynamicBodyConstraintCounts.Table.Id;
                    dynamicBodyConstraintCounts = default;
                }
                return true;
            }
            return false;
        }


        internal unsafe void Remove(Solver solver, BufferPool bufferPool, ref ConstraintBatch batch, ref IndexSet fallbackBatchHandles, int typeId, int indexInTypeBatch)
        {
            var typeProcessor = solver.TypeProcessors[typeId];
            var bodyCount = typeProcessor.BodiesPerConstraint;
            var bodyIndices = stackalloc int[bodyCount];
            var enumerator = new PassthroughReferenceCollector(bodyIndices);
            var maximumAllocationIdsToFree = 3 + bodyCount * 2;
            var allocationIdsToRemoveMemory = stackalloc int[maximumAllocationIdsToFree];
            var initialSpan = new Buffer<int>(allocationIdsToRemoveMemory, maximumAllocationIdsToFree);
            var allocationIdsToFree = new QuickList<int>(initialSpan);
            solver.EnumerateConnectedRawBodyReferences(ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[typeId]], indexInTypeBatch, ref enumerator);
            for (int i = 0; i < bodyCount; ++i)
            {
                var rawBodyIndex = bodyIndices[i];
                if (Bodies.IsEncodedDynamicReference(rawBodyIndex))
                {
                    var bodyIndex = rawBodyIndex & Bodies.BodyReferenceMask;
                    if (RemoveOneBodyReferenceFromDynamicsSet(bodyIndex, ref allocationIdsToFree))
                    {
                        fallbackBatchHandles.Remove(solver.bodies.ActiveSet.IndexToHandle[bodyIndex].Value);
                    }
                }
            }
            for (int i = 0; i < allocationIdsToFree.Count; ++i)
            {
                bufferPool.ReturnUnsafely(allocationIdsToFree[i]);
            }
        }

        [Conditional("DEBUG")]
        public static unsafe void ValidateSetReferences(Solver solver, int setIndex)
        {
            ref var set = ref solver.Sets[setIndex];
            Debug.Assert(set.Allocated);
            if (set.Batches.Count > solver.FallbackBatchThreshold)
            {
                Debug.Assert(set.SequentialFallback.dynamicBodyConstraintCounts.Keys.Allocated);
                ref var bodyConstraintCounts = ref set.SequentialFallback.dynamicBodyConstraintCounts;
                for (int i = 0; i < bodyConstraintCounts.Count; ++i)
                {
                    //This is a handle on inactive sets, and an index for active sets.
                    var bodyReference = bodyConstraintCounts.Keys[i];
                    var count = bodyConstraintCounts.Values[i];
                    Debug.Assert(count > 0, "If there exists a body reference set, it should be populated.");
                }
                ref var batch = ref set.Batches[solver.FallbackBatchThreshold];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    var bodiesPerConstraint = solver.TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    var connectedBodies = stackalloc int[bodiesPerConstraint];
                    for (int constraintIndex = 0; constraintIndex < typeBatch.ConstraintCount; ++constraintIndex)
                    {
                        var constraintHandle = typeBatch.IndexToHandle[constraintIndex];
                        var collector = new PassthroughReferenceCollector(connectedBodies);
                        solver.EnumerateConnectedDynamicBodies(constraintHandle, ref collector);
                        for (int i = 0; i < bodiesPerConstraint; ++i)
                        {
                            var localBodyIndex = bodyConstraintCounts.IndexOf(connectedBodies[i]);
                            Debug.Assert(localBodyIndex >= 0, "Any dynamic body referenced by a constraint in the fallback batch should exist within the fallback batch's dynamic body listing.");
                            var count = bodyConstraintCounts.Values[localBodyIndex];
                        }
                    }
                }
            }
        }
        [Conditional("DEBUG")]
        public static unsafe void ValidateReferences(Solver solver)
        {
            for (int i = 0; i < solver.Sets.Length; ++i)
            {
                if (solver.Sets[i].Allocated)
                    ValidateSetReferences(solver, i);
            }
        }
        internal void UpdateForDynamicBodyMemoryMove(int originalBodyIndex, int newBodyLocation)
        {
            Debug.Assert(dynamicBodyConstraintCounts.Keys.Allocated && !dynamicBodyConstraintCounts.ContainsKey(newBodyLocation), "If a body is being moved, as opposed to swapped, then the target index should not be present.");
            dynamicBodyConstraintCounts.GetTableIndices(ref originalBodyIndex, out var tableIndex, out var elementIndex);
            var references = dynamicBodyConstraintCounts.Values[elementIndex];
            dynamicBodyConstraintCounts.FastRemove(tableIndex, elementIndex);
            dynamicBodyConstraintCounts.AddUnsafely(ref newBodyLocation, references);
        }

        internal void UpdateForBodyMemorySwap(int a, int b)
        {
            var indexA = dynamicBodyConstraintCounts.IndexOf(a);
            var indexB = dynamicBodyConstraintCounts.IndexOf(b);
            Debug.Assert(indexA >= 0 && indexB >= 0, "A swap requires that both indices are already present.");
            Helpers.Swap(ref dynamicBodyConstraintCounts.Values[indexA], ref dynamicBodyConstraintCounts.Values[indexB]);
        }

        internal static void CreateFrom(ref SequentialFallbackBatch sourceBatch, BufferPool pool, out SequentialFallbackBatch targetBatch)
        {
            //Copy over non-buffer state. This copies buffer references pointlessly, but that doesn't matter.
            targetBatch.dynamicBodyConstraintCounts = sourceBatch.dynamicBodyConstraintCounts;
            pool.TakeAtLeast(sourceBatch.dynamicBodyConstraintCounts.Count, out targetBatch.dynamicBodyConstraintCounts.Keys);
            pool.TakeAtLeast(targetBatch.dynamicBodyConstraintCounts.Keys.Length, out targetBatch.dynamicBodyConstraintCounts.Values);
            pool.TakeAtLeast(sourceBatch.dynamicBodyConstraintCounts.TableMask + 1, out targetBatch.dynamicBodyConstraintCounts.Table);
            sourceBatch.dynamicBodyConstraintCounts.Keys.CopyTo(0, targetBatch.dynamicBodyConstraintCounts.Keys, 0, sourceBatch.dynamicBodyConstraintCounts.Count);
            sourceBatch.dynamicBodyConstraintCounts.Values.CopyTo(0, targetBatch.dynamicBodyConstraintCounts.Values, 0, sourceBatch.dynamicBodyConstraintCounts.Count);
            sourceBatch.dynamicBodyConstraintCounts.Table.CopyTo(0, targetBatch.dynamicBodyConstraintCounts.Table, 0, sourceBatch.dynamicBodyConstraintCounts.TableMask + 1);
        }

        internal void EnsureCapacity(int bodyCapacity, BufferPool pool)
        {
            if (dynamicBodyConstraintCounts.Keys.Allocated)
            {
                //This is conservative since there's no guarantee that we'll actually need to resize at all if these bodies are already present, but that's fine. 
                dynamicBodyConstraintCounts.EnsureCapacity(bodyCapacity, pool);
            }
            else
            {
                dynamicBodyConstraintCounts = new QuickDictionary<int, int, PrimitiveComparer<int>>(bodyCapacity, pool);
            }

        }

        public void Compact(BufferPool pool)
        {
            if (dynamicBodyConstraintCounts.Keys.Allocated)
            {
                dynamicBodyConstraintCounts.Compact(pool);
            }
        }


        public void Dispose(BufferPool pool)
        {
            if (dynamicBodyConstraintCounts.Keys.Allocated)
            {
                dynamicBodyConstraintCounts.Dispose(pool);
            }
        }
    }
}
