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
        public readonly int BodyCount { get { return bodyConstraintCounts.Count; } }

        //In order to maintain the batch referenced handles for the fallback batch (which can have the same body appear more than once),
        //every body must maintain a count of fallback constraints associated with it.
        //Note that this dictionary uses active set body *indices* while active, but body *handles* when associated with an inactive set.
        //This is consistent with the body references stored by active/inactive constraints.
        internal QuickDictionary<int, int, PrimitiveComparer<int>> bodyConstraintCounts;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Allocate<TBodyReferenceGetter>(Span<BodyHandle> constraintBodyHandles, Bodies bodies,
            BufferPool pool, TBodyReferenceGetter bodyReferenceGetter, int minimumBodyCapacity)
            where TBodyReferenceGetter : struct, IBodyReferenceGetter
        {
            EnsureCapacity(Math.Max(bodyConstraintCounts.Count + constraintBodyHandles.Length, minimumBodyCapacity), pool);
            for (int i = 0; i < constraintBodyHandles.Length; ++i)
            {
                var bodyReference = bodyReferenceGetter.GetBodyReference(bodies, constraintBodyHandles[i]);

                if (bodyConstraintCounts.FindOrAllocateSlotUnsafely(bodyReference, out var slotIndex))
                {
                    ++bodyConstraintCounts.Values[slotIndex];
                }
                else
                {
                    bodyConstraintCounts.Values[slotIndex] = 1;
                }
            }
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void AllocateForActive(Span<BodyHandle> constraintBodyHandles, Bodies bodies,
           BufferPool pool, int minimumBodyCapacity = 8)
        {
            Allocate(constraintBodyHandles, bodies, pool, new ActiveSetGetter(), minimumBodyCapacity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void AllocateForInactive(Span<BodyHandle> constraintBodyHandles, Bodies bodies,
          BufferPool pool, int minimumBodyCapacity = 8)
        {
            Allocate(constraintBodyHandles, bodies, pool, new InactiveSetGetter(), minimumBodyCapacity);
        }


        /// <summary>
        /// Removes a constraint from a body in the fallback batch.
        /// </summary>
        /// <param name="bodyReference">Body associated with a constraint in the fallback batch.</param>
        /// <param name="allocationIdsToFree">Allocations that should be freed once execution is back in a safe context.</param>
        /// <returns>True if the body no longer has any constraints associated with it in the fallback batch, false otherwise.</returns>
        internal unsafe bool Remove(int bodyReference, ref QuickList<int> allocationIdsToFree)
        {
            var bodyPresent = bodyConstraintCounts.GetTableIndices(ref bodyReference, out var tableIndex, out var bodyReferencesIndex);
            Debug.Assert(bodyPresent, "If we've been asked to remove a constraint associated with a body, that body must be in this batch.");
            ref var constraintCount = ref bodyConstraintCounts.Values[bodyReferencesIndex];
            --constraintCount;
            if (constraintCount == 0)
            {
                //If there are no more constraints associated with this body, get rid of the body list.
                constraintCount = default;
                bodyConstraintCounts.FastRemove(tableIndex, bodyReferencesIndex);
                if (bodyConstraintCounts.Count == 0)
                {
                    //No constraints remain in the fallback batch. Drop the dictionary.
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintCounts.Keys.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintCounts.Values.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintCounts.Table.Id;
                    bodyConstraintCounts = default;
                }
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes a body from the fallback batch if it is present.
        /// </summary>
        /// <param name="bodyReference">Reference to the body to remove from the fallback batch.</param>
        /// <param name="allocationIdsToFree">Allocations that should be freed once execution is back in a safe context.</param>
        /// <returns>True if the body was present in the fallback batch and was removed, false otherwise.</returns>
        internal unsafe bool TryRemove(int bodyReference, ref QuickList<int> allocationIdsToFree)
        {
            if (bodyConstraintCounts.Keys.Allocated && bodyConstraintCounts.GetTableIndices(ref bodyReference, out var tableIndex, out var bodyReferencesIndex))
            {
                ref var constraintReferences = ref bodyConstraintCounts.Values[bodyReferencesIndex];
                //If there are no more constraints associated with this body, get rid of the body list.
                bodyConstraintCounts.FastRemove(tableIndex, bodyReferencesIndex);
                if (bodyConstraintCounts.Count == 0)
                {
                    //No constraints remain in the fallback batch. Drop the dictionary.
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintCounts.Keys.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintCounts.Values.Id;
                    allocationIdsToFree.AllocateUnsafely() = bodyConstraintCounts.Table.Id;
                    bodyConstraintCounts = default;
                }
                return true;
            }
            return false;
        }


        internal unsafe void Remove(Solver solver, BufferPool bufferPool, ref ConstraintBatch batch, ConstraintHandle constraintHandle, ref IndexSet fallbackBatchHandles, int typeId, int indexInTypeBatch)
        {
            var typeProcessor = solver.TypeProcessors[typeId];
            var bodyCount = typeProcessor.BodiesPerConstraint;
            var bodyIndices = stackalloc int[bodyCount];
            var enumerator = new ReferenceCollector(bodyIndices);
            solver.EnumerateConnectedBodies(constraintHandle, ref enumerator);
            var maximumAllocationIdsToFree = 3 + bodyCount * 2;
            var allocationIdsToRemoveMemory = stackalloc int[maximumAllocationIdsToFree];
            var initialSpan = new Buffer<int>(allocationIdsToRemoveMemory, maximumAllocationIdsToFree);
            var allocationIdsToFree = new QuickList<int>(initialSpan);
            typeProcessor.EnumerateConnectedBodyIndices(ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[typeId]], indexInTypeBatch, ref enumerator);
            for (int i = 0; i < bodyCount; ++i)
            {
                if (Remove(bodyIndices[i], ref allocationIdsToFree))
                {
                    fallbackBatchHandles.Remove(solver.bodies.ActiveSet.IndexToHandle[bodyIndices[i]].Value);
                }
            }
            for (int i = 0; i < allocationIdsToFree.Count; ++i)
            {
                bufferPool.ReturnUnsafely(allocationIdsToFree[i]);
            }
        }


        public static void AllocateResults(Solver solver, BufferPool pool, ref ConstraintBatch batch, out Buffer<JacobiFallbackTypeBatchResults> results)
        {
            pool.TakeAtLeast(batch.TypeBatches.Count, out results);
            for (int i = 0; i < batch.TypeBatches.Count; ++i)
            {
                ref var typeBatch = ref batch.TypeBatches[i];
                var bodiesPerConstraint = solver.TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                ref var typeBatchResults = ref results[i];
                pool.TakeAtLeast(bodiesPerConstraint, out typeBatchResults.BodyVelocities);
                for (int j = 0; j < bodiesPerConstraint; ++j)
                {
                    pool.TakeAtLeast(typeBatch.BundleCount, out typeBatchResults.GetVelocitiesForBody(j));
                }
            }
        }

        [Conditional("DEBUG")]
        unsafe static void ValidateBodyConstraintReference(Solver solver, int setIndex, int bodyReference, ConstraintHandle constraintHandle, int expectedIndexInConstraint)
        {
            ref var constraintLocation = ref solver.HandleToConstraint[constraintHandle.Value];
            Debug.Assert(constraintLocation.SetIndex == setIndex);
            Debug.Assert(constraintLocation.BatchIndex == solver.FallbackBatchThreshold, "Should only be working on constraints which are members of the active fallback batch.");
            var debugReferences = stackalloc int[solver.TypeProcessors[constraintLocation.TypeId].BodiesPerConstraint];
            var debugBodyReferenceCollector = new ReferenceCollector(debugReferences);
            solver.EnumerateConnectedBodies(constraintHandle, ref debugBodyReferenceCollector);
            Debug.Assert(debugReferences[expectedIndexInConstraint] == bodyReference, "The constraint's true body references must agree with the fallback batch.");
        }
        [Conditional("DEBUG")]
        public static unsafe void ValidateSetReferences(Solver solver, int setIndex)
        {
            ref var set = ref solver.Sets[setIndex];
            Debug.Assert(set.Allocated);
            if (set.Batches.Count > solver.FallbackBatchThreshold)
            {
                Debug.Assert(set.SequentialFallback.bodyConstraintCounts.Keys.Allocated);
                ref var bodyConstraintCounts = ref set.SequentialFallback.bodyConstraintCounts;
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
                        var collector = new ReferenceCollector(connectedBodies);
                        solver.EnumerateConnectedBodies(constraintHandle, ref collector);
                        for (int i = 0; i < bodiesPerConstraint; ++i)
                        {
                            var localBodyIndex = bodyConstraintCounts.IndexOf(connectedBodies[i]);
                            Debug.Assert(localBodyIndex >= 0, "Any body referenced by a constraint in the fallback batch should exist within the fallback batch's body listing.");
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
        internal void UpdateForBodyMemoryMove(int originalBodyIndex, int newBodyLocation)
        {
            Debug.Assert(bodyConstraintCounts.Keys.Allocated && !bodyConstraintCounts.ContainsKey(newBodyLocation), "If a body is being moved, as opposed to swapped, then the target index should not be present.");
            bodyConstraintCounts.GetTableIndices(ref originalBodyIndex, out var tableIndex, out var elementIndex);
            var references = bodyConstraintCounts.Values[elementIndex];
            bodyConstraintCounts.FastRemove(tableIndex, elementIndex);
            bodyConstraintCounts.AddUnsafely(ref newBodyLocation, references);
        }

        internal void UpdateForBodyMemorySwap(int a, int b)
        {
            var indexA = bodyConstraintCounts.IndexOf(a);
            var indexB = bodyConstraintCounts.IndexOf(b);
            Debug.Assert(indexA >= 0 && indexB >= 0, "A swap requires that both indices are already present.");
            Helpers.Swap(ref bodyConstraintCounts.Values[indexA], ref bodyConstraintCounts.Values[indexB]);
        }

        internal static void CreateFrom(ref JacobiFallbackBatch sourceBatch, BufferPool pool, out JacobiFallbackBatch targetBatch)
        {
            //Copy over non-buffer state. This copies buffer references pointlessly, but that doesn't matter.
            targetBatch.bodyConstraintReferences = sourceBatch.bodyConstraintReferences;
            pool.TakeAtLeast(sourceBatch.bodyConstraintReferences.Count, out targetBatch.bodyConstraintReferences.Keys);
            pool.TakeAtLeast(targetBatch.bodyConstraintReferences.Keys.Length, out targetBatch.bodyConstraintReferences.Values);
            pool.TakeAtLeast(sourceBatch.bodyConstraintReferences.TableMask + 1, out targetBatch.bodyConstraintReferences.Table);
            sourceBatch.bodyConstraintReferences.Keys.CopyTo(0, targetBatch.bodyConstraintReferences.Keys, 0, sourceBatch.bodyConstraintReferences.Count);
            sourceBatch.bodyConstraintReferences.Values.CopyTo(0, targetBatch.bodyConstraintReferences.Values, 0, sourceBatch.bodyConstraintReferences.Count);
            sourceBatch.bodyConstraintReferences.Table.CopyTo(0, targetBatch.bodyConstraintReferences.Table, 0, sourceBatch.bodyConstraintReferences.TableMask + 1);

            for (int i = 0; i < sourceBatch.bodyConstraintReferences.Count; ++i)
            {
                ref var source = ref sourceBatch.bodyConstraintReferences.Values[i];
                ref var target = ref targetBatch.bodyConstraintReferences.Values[i];
                target = source;
                pool.TakeAtLeast(source.Count, out target.Span);
                pool.TakeAtLeast(source.TableMask + 1, out target.Table);
                source.Span.CopyTo(0, target.Span, 0, source.Count);
                source.Table.CopyTo(0, target.Table, 0, source.TableMask + 1);
            }
        }

        internal void EnsureCapacity(int bodyCapacity, BufferPool pool)
        {
            if (bodyConstraintCounts.Keys.Allocated)
            {
                //This is conservative since there's no guarantee that we'll actually need to resize at all if these bodies are already present, but that's fine. 
                bodyConstraintCounts.EnsureCapacity(bodyCapacity, pool);
            }
            else
            {
                bodyConstraintCounts = new QuickDictionary<int, int, PrimitiveComparer<int>>(bodyCapacity, pool);
            }

        }

        public void Compact(BufferPool pool)
        {
            if (bodyConstraintCounts.Keys.Allocated)
            {
                bodyConstraintCounts.Compact(pool);
            }
        }


        public void Dispose(BufferPool pool)
        {
            if (bodyConstraintCounts.Keys.Allocated)
            {
                bodyConstraintCounts.Dispose(pool);
            }
        }
    }
}
