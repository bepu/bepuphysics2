using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuUtilities;

namespace BepuPhysics
{
    /// <summary>
    /// Contains a set of type batches whose constraints share no body references.
    /// </summary>
    public struct ConstraintBatch
    {
        //Note that both active and inactive constraint batches share the same data layout.
        //This means we have a type id->index mapping in inactive islands. 
        //Reasoning:
        //The type id->index mapping is required because the solver's handle->constraint indirection stores a type id. If it stored a batch-specific type *index*, 
        //then all mappings associated with a type batch's constraints would have to be updated when a type batch changes slots due to a removal.
        //Given that there could be hundreds or thousands of such changes required, we instead rely on this last-second remapping.

        //However, an inactive island will never undergo removals under normal conditions, and they can only happen at all by direct user request.
        //In other words, the risk of moving type batches in inactive islands is pretty much irrelevant. In fact, you could instead store a direct handle->type *index* mapping
        //pretty safely, even if it meant either not moving type batches when one becomes empty or just brute force updating the mapping associated with every constraint in the type batch.

        //The cost of storing this extra data is not completely trivial. Assuming an average of 128 bytes per type id->index mapping, consider what happens
        //when you have 65536 inactive islands: ~10 megabytes of wasted mapping data.

        //Right now, we make no attempt to split the storage layout and bite the bullet on the waste, because:
        //1) While it does require some memory, 10 megabytes is fairly trivial in terms of *capacity* for any platform where you're going to have a simulation with 65536 inactive islands.
        //2) The memory used by a bunch of different islands isn't really concerning from a bandwidth perspective, because by nature, it is not being accessed every frame (nor all at once).
        //3) Avoiding this waste would require splitting the storage representation and/or complicating the handle->constraint mapping.
        //TODO: So, perhaps one day we'll consider changing this, but for now it's just not worth it.

        //That said, we DO store each active constraint batch's BatchReferencedHandles separately from the ConstraintBatch type.
        //Two reasons for the different choice:
        //1) The handles memory will often end up being an order of magnitude bigger. We're not talking about 10MB for 65536 inactive islands here, but rather 100-400MB and up.
        //2) Storing the referenced handles separately in the solver doesn't really change anything. We just have to pass them as parameters here and there; no significant complication.

        public Buffer<int> TypeIndexToTypeBatchIndex;
        public QuickList<TypeBatch> TypeBatches;

        public ConstraintBatch(BufferPool pool, int initialTypeCountEstimate = 32)
            : this()
        {
            ResizeTypeMap(pool, initialTypeCountEstimate);
            TypeBatches = new QuickList<TypeBatch>(initialTypeCountEstimate, pool);
        }

        void ResizeTypeMap(BufferPool pool, int newSize)
        {
            var oldLength = TypeIndexToTypeBatchIndex.Length;
            Debug.Assert(oldLength != BufferPool.GetCapacityForCount<int>(newSize), "Shouldn't resize if nothing changes.");
            pool.ResizeToAtLeast(ref TypeIndexToTypeBatchIndex, newSize, oldLength);
            for (int i = oldLength; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                TypeIndexToTypeBatchIndex[i] = -1;
            }
        }
        internal void EnsureTypeMapSize(BufferPool pool, int targetSize)
        {
            if (targetSize > TypeIndexToTypeBatchIndex.Length)
                ResizeTypeMap(pool, targetSize);
        }

        [Conditional("DEBUG")]
        void ValidateTypeBatchMappings()
        {
            for (int i = 0; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                var index = TypeIndexToTypeBatchIndex[i];
                if (index >= 0)
                {
                    Debug.Assert(index < TypeBatches.Count);
                    Debug.Assert(TypeBatches[index].TypeId == i);
                }
            }
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                Debug.Assert(TypeIndexToTypeBatchIndex[TypeBatches[i].TypeId] == i);
            }
        }

        /// <summary>
        /// Gets a type batch in the batch matching the given type id.
        /// Requires that there exists at least one constraint in the type batch.
        /// </summary>
        /// <param name="typeId">Id of the TypeBatch's type to retrieve.</param>
        /// <returns>TypeBatch instance associated with the given type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref TypeBatch GetTypeBatch(int typeId)
        {
            return ref *GetTypeBatchPointer(typeId);
        }


        /// <summary>
        /// Gets a pointer to the type batch in the batch matching the given type id.
        /// Requires that there exists at least one constraint in the type batch.
        /// </summary>
        /// <param name="typeId">Id of the TypeBatch's type to retrieve.</param>
        /// <returns>TypeBatch instance associated with the given type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe TypeBatch* GetTypeBatchPointer(int typeId)
        {
            ValidateTypeBatchMappings();
            var typeBatchIndex = TypeIndexToTypeBatchIndex[typeId];
            return TypeBatches.GetPointer(typeBatchIndex);
        }

        internal unsafe TypeBatch* CreateNewTypeBatch(int typeId, TypeProcessor typeProcessor, int initialCapacity, BufferPool pool)
        {
            Debug.Assert(typeProcessor != null, "Can't create a type batch for a nonexistent type processor. Did you forget to call Solver.Register<T> for the constraint type?");
            var newIndex = TypeBatches.Count;
            TypeBatches.EnsureCapacity(TypeBatches.Count + 1, pool);
            TypeIndexToTypeBatchIndex[typeId] = newIndex;
            ref var typeBatch = ref TypeBatches.AllocateUnsafely();
            typeProcessor.Initialize(ref typeBatch, initialCapacity, pool);
            return (TypeBatch*)Unsafe.AsPointer(ref typeBatch);
        }


        internal unsafe TypeBatch* GetOrCreateTypeBatch(int typeId, TypeProcessor typeProcessor, int initialCapacity, BufferPool pool)
        {
            if (typeId >= TypeIndexToTypeBatchIndex.Length)
            {
                //While we only request a capacity one slot larger, buffer pools always return a power of 2, so this isn't going to cause tons of unnecessary resizing.
                ResizeTypeMap(pool, typeId + 1);
                return CreateNewTypeBatch(typeId, typeProcessor, initialCapacity, pool);
            }
            else
            {
                var typeBatchIndex = TypeIndexToTypeBatchIndex[typeId];
                if (typeBatchIndex == -1)
                {
                    return CreateNewTypeBatch(typeId, typeProcessor, initialCapacity, pool);
                }
                else
                {
                    return TypeBatches.GetPointer(typeBatchIndex);
                }
            }
        }

        unsafe struct ActiveBodyHandleRemover : IForEach<int>
        {
            public Bodies Bodies;
            public IndexSet* Handles;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public ActiveBodyHandleRemover(Bodies bodies, IndexSet* handles)
            {
                Bodies = bodies;
                Handles = handles;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int encodedBodyIndex)
            {
                if (Bodies.IsEncodedDynamicReference(encodedBodyIndex))
                {
                    Handles->Remove(Bodies.ActiveSet.IndexToHandle[encodedBodyIndex & Bodies.BodyReferenceMask].Value);
                }
            }
        }

        //Note that we have split the constraint batch removal for the sake of reuse by the multithreaded constraint remover.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveTypeBatchIfEmpty(ref TypeBatch typeBatch, int typeBatchIndexToRemove, BufferPool pool)
        {
            if (typeBatch.ConstraintCount == 0)
            {
                var constraintTypeId = typeBatch.TypeId;
                TypeIndexToTypeBatchIndex[constraintTypeId] = -1;
                typeBatch.Dispose(pool); //Note that the disposal must occur BEFORE the removal, or else we'll end up disposing whatever type batch moves to occupy the newly empty slot.
                TypeBatches.FastRemoveAt(typeBatchIndexToRemove);
                if (typeBatchIndexToRemove < TypeBatches.Count)
                {
                    //If we swapped anything into the removed slot, we should update the type index to type batch mapping.
                    TypeIndexToTypeBatchIndex[TypeBatches[typeBatchIndexToRemove].TypeId] = typeBatchIndexToRemove;
                }
            }
            ValidateTypeBatchMappings();
        }
        public unsafe void RemoveBodyHandlesFromBatchForConstraint(int constraintTypeId, int indexInTypeBatch, int batchIndex, Solver solver)
        {
            Debug.Assert(batchIndex <= solver.FallbackBatchThreshold, "This should only be used for non-fallback batches. The body handles set for a fallback batch should be handled by the fallback batch's remove call.");
            var indexSet = solver.batchReferencedHandles.GetPointer(batchIndex);
            var handleRemover = new ActiveBodyHandleRemover(solver.bodies, indexSet);
            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            solver.EnumerateConnectedRawBodyReferences(ref TypeBatches[typeBatchIndex], indexInTypeBatch, ref handleRemover);
        }

        public unsafe void Remove(int constraintTypeId, int indexInTypeBatch, bool isFallback, Solver solver)
        {
            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            ref var typeBatch = ref TypeBatches[typeBatchIndex];
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");
            Debug.Assert(typeBatch.ConstraintCount > indexInTypeBatch);
            var typeProcessor = solver.TypeProcessors[constraintTypeId];
            typeProcessor.Remove(ref typeBatch, indexInTypeBatch, ref solver.HandleToConstraint, isFallback);
            RemoveTypeBatchIfEmpty(ref typeBatch, typeBatchIndex, solver.pool);
        }

        public void Clear(BufferPool pool)
        {
            for (int typeBatchIndex = 0; typeBatchIndex < TypeBatches.Count; ++typeBatchIndex)
            {
                TypeBatches[typeBatchIndex].Dispose(pool);
            }
            //Since there are no more type batches, the mapping must be cleared out.
            for (int typeId = 0; typeId < TypeIndexToTypeBatchIndex.Length; ++typeId)
            {
                TypeIndexToTypeBatchIndex[typeId] = -1;
            }
            TypeBatches.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int GetTargetCapacity(ref TypeBatch typeBatch, Solver solver)
        {
            return Math.Max(typeBatch.ConstraintCount, solver.GetMinimumCapacityForType(typeBatch.TypeId));
        }

        /// <summary>
        /// Ensures that all type batches within this constraint batch meet or exceed the size requirements of the per-type capacities defined by the solver.
        /// </summary>
        /// <param name="solver">Solver to pull minimum capacities from.</param>
        public void EnsureTypeBatchCapacities(Solver solver)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                ref var typeBatch = ref TypeBatches[i];
                var targetCapacity = GetTargetCapacity(ref typeBatch, solver);
                if (targetCapacity > typeBatch.IndexToHandle.Length)
                    solver.TypeProcessors[TypeBatches[i].TypeId].Resize(ref typeBatch, targetCapacity, solver.pool);
            }
        }

        /// <summary>
        /// Applies the solver-defined minimum capacities to existing type batches.
        /// </summary>
        /// <param name="solver">Solver to pull minimum capacities from.</param>
        public void ResizeTypeBatchCapacities(Solver solver)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                ref var typeBatch = ref TypeBatches[i];
                solver.TypeProcessors[TypeBatches[i].TypeId].Resize(ref typeBatch, GetTargetCapacity(ref typeBatch, solver), solver.pool);
            }
        }
        /// <summary>
        /// Releases all memory used by the batch.
        /// </summary>
        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                TypeBatches[i].Dispose(pool);
            }
            pool.Return(ref TypeIndexToTypeBatchIndex);
            TypeIndexToTypeBatchIndex = new Buffer<int>();
            TypeBatches.Dispose(pool);
            TypeBatches = default;
        }
    }
}
