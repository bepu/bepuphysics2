using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Contains a set of constraints which share no body references.
    /// </summary>
    public class ConstraintBatch
    {
        internal BatchReferencedHandles BodyHandles;
        //Pooling the type index to type batch index is a bit pointless, but we can do it easily, so we do.
        public Buffer<int> TypeIndexToTypeBatchIndex;
        public QuickList<TypeBatch, Array<TypeBatch>> TypeBatches;

        public ConstraintBatch(BufferPool pool, int initialReferencedHandlesEstimate = 128 * 64, int initialTypeCountEstimate = 32)
        {
            BodyHandles = new BatchReferencedHandles(pool, initialReferencedHandlesEstimate);
            ResizeTypeMap(pool, initialTypeCountEstimate);
            QuickList<TypeBatch, Array<TypeBatch>>.Create(new PassthroughArrayPool<TypeBatch>(), initialTypeCountEstimate, out TypeBatches);
        }

        void ResizeTypeMap(BufferPool pool, int newSize)
        {
            var oldLength = TypeIndexToTypeBatchIndex.Length;
            pool.SpecializeFor<int>().Resize(ref TypeIndexToTypeBatchIndex, newSize, oldLength);
            for (int i = oldLength; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                TypeIndexToTypeBatchIndex[i] = -1;
            }
        }

        [Conditional("DEBUG")]
        void ValidateTypeBatchMappings()
        {
            for (int i = 0; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                var index = TypeIndexToTypeBatchIndex[i];
                if(index >= 0)
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
        public TypeBatch GetTypeBatch(int typeId)
        {
            ValidateTypeBatchMappings();
            var typeBatchIndex = TypeIndexToTypeBatchIndex[typeId];
            return TypeBatches[typeBatchIndex];
        }

        TypeBatch CreateNewTypeBatch(int typeId, TypeBatchAllocation typeBatchAllocation)
        {
            var batch = typeBatchAllocation.Take(typeId);
            TypeBatches.Add(batch, new PassthroughArrayPool<TypeBatch>());
            return batch;
        }


        /// <summary>
        /// Gets the TypeBatch associated with the given TypeId in this ConstraintBatch. Creates a new TypeBatch if one does not already exist.
        /// </summary>
        /// <param name="typeId">TypeId of the type batch to look up.</param>
        /// <param name="typeBatchAllocation">Type batch allocation used to initialize constraints.</param>
        /// <returns>TypeBatch associated with the given typeid in this ConstraintBatch.</returns>
        internal TypeBatch GetOrCreateTypeBatch(int typeId, TypeBatchAllocation typeBatchAllocation)
        {
            if (typeId >= TypeIndexToTypeBatchIndex.Length)
            {
                ResizeTypeMap(typeBatchAllocation.BufferPool, 1 << SpanHelper.GetContainingPowerOf2(typeId));
                TypeIndexToTypeBatchIndex[typeId] = TypeBatches.Count;
                return CreateNewTypeBatch(typeId, typeBatchAllocation);
            }
            else
            {
                ref var typeBatchIndex = ref TypeIndexToTypeBatchIndex[typeId];
                if (typeBatchIndex == -1)
                {
                    typeBatchIndex = TypeBatches.Count;
                    return CreateNewTypeBatch(typeId, typeBatchAllocation);
                }
                else
                {
                    return TypeBatches[typeBatchIndex];
                }
            }
        }
        /// <summary>
        /// Gets whether the batch could hold the specified body handles.
        /// </summary>
        /// <param name="constraintBodyHandles">List of body handles to check for in the batch.</param>
        /// <param name="constraintBodyHandleCount">Number of bodies referenced by the constraint.</param>
        /// <returns>True if the body handles are not already present in the batch, false otherwise.</returns>
        public unsafe bool CanFit(ref int constraintBodyHandles, int constraintBodyHandleCount)
        {
            for (int i = 0; i < constraintBodyHandleCount; ++i)
            {
                var bodyHandle = Unsafe.Add(ref constraintBodyHandles, i);
                if (BodyHandles.Contains(bodyHandle))
                {
                    return false;
                }
            }
            return true;
        }

        [Conditional("DEBUG")]
        private void ValidateBodyIndex(int bodyIndex, int expectedCount)
        {
            int referencesToBody = 0;
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                var instancesInTypeBatch = TypeBatches[i].GetBodyIndexInstanceCount(bodyIndex);
                Debug.Assert(instancesInTypeBatch + referencesToBody <= expectedCount,
                    "Found an instance of a body index that wasn't expected. Possible upstream bug or memory corruption.");
                referencesToBody += instancesInTypeBatch;
            }
            Debug.Assert(referencesToBody == expectedCount);
        }

        [Conditional("DEBUG")]
        internal void ValidateExistingHandles(Bodies bodies)
        {
            for (int i = 0; i < bodies.Count; ++i)
            {
                bodies.ValidateExistingHandle(bodies.IndexToHandle[i]);
            }
            for (int i = 0; i < bodies.Count; ++i)
            {
                var handle = bodies.IndexToHandle[i];
                if (BodyHandles.Contains(handle))
                    ValidateBodyIndex(i, 1);
                else
                    ValidateBodyIndex(i, 0);
            }
        }
        [Conditional("DEBUG")]
        internal void ValidateNewHandles(ref int bodyHandles, int bodyCount, Bodies bodies)
        {
            Debug.Assert(CanFit(ref bodyHandles, bodyCount));
            for (int i = 0; i < bodyCount; ++i)
            {
                ValidateBodyIndex(bodies.HandleToIndex[Unsafe.Add(ref bodyHandles, i)], 0);
            }
        }
        public unsafe ref TypeBatchData Allocate(int handle, ref int bodyHandles, int bodyCount, Bodies bodies, TypeBatchAllocation typeBatchAllocation, int typeId, out int indexInTypeBatch)
        {
            Debug.Assert(CanFit(ref bodyHandles, bodyCount));
            //Add all the constraint's body handles to the batch we found (or created) to block future references to the same bodies.
            //Also, convert the handle into a memory index. Constraints store a direct memory reference for performance reasons.
            var bodyIndices = stackalloc int[bodyCount];
            for (int j = 0; j < bodyCount; ++j)
            {
                var bodyHandle = Unsafe.Add(ref bodyHandles, j);
                BodyHandles.Add(bodyHandle, typeBatchAllocation.BufferPool);
                bodyIndices[j] = bodies.HandleToIndex[bodyHandle];
            }
            reference.TypeBatch = GetOrCreateTypeBatch(typeId, typeBatchAllocation);
            reference.IndexInTypeBatch = reference.TypeBatch.Allocate(handle, bodyIndices, typeBatchAllocation);
            //TODO: We could adjust the typeBatchAllocation capacities in response to the allocated index.
            //If it exceeds the current capacity, we could ensure the new size is still included.
            //The idea here would be to avoid resizes later by ensuring that the historically encountered size is always used to initialize.
            //This isn't necessarily beneficial, though- often, higher indexed batches will contain smaller numbers of constraints, so allocating a huge number
            //of constraints into them is very low value. You may want to be a little more clever about the heuristic. Either way, only bother with this once there is 
            //evidence that typebatch resizes are ever a concern. This will require frame spike analysis, not merely average timings.
            //(While resizes will definitely occur, remember that it only really matters for *new* type batches- 
            //and it is rare that a new type batch will be created that actually needs to be enormous.)
        }


        unsafe struct BodyHandleRemover : IForEach<int>
        {
            public Bodies Bodies;
            public ConstraintBatch Batch;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public BodyHandleRemover(Bodies bodies, ConstraintBatch batch)
            {
                Bodies = bodies;
                Batch = batch;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int bodyIndex)
            {
                Batch.BodyHandles.Remove(Bodies.IndexToHandle[bodyIndex]);
            }
        }

        //Note that we have split the constraint batch removal for the sake of reuse by the multithreaded constraint remover.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveTypeBatchIfEmpty(TypeBatch typeBatch, int typeBatchIndexToRemove, TypeBatchAllocation typeBatchAllocation)
        {
            if (typeBatch.ConstraintCount == 0)
            {
                var constraintTypeId = typeBatch.TypeId;
                TypeIndexToTypeBatchIndex[constraintTypeId] = -1;
                TypeBatches.FastRemoveAt(typeBatchIndexToRemove);
                if (typeBatchIndexToRemove < TypeBatches.Count)
                {
                    //If we swapped anything into the removed slot, we should update the type index to type batch mapping.
                    TypeIndexToTypeBatchIndex[TypeBatches[typeBatchIndexToRemove].TypeId] = typeBatchIndexToRemove;
                }
                typeBatchAllocation.Return(typeBatch, constraintTypeId);

            }
            ValidateTypeBatchMappings();
        }

        public unsafe void Remove(int constraintTypeId, int indexInTypeBatch, Bodies bodies, ref Buffer<ConstraintLocation> handlesToConstraints, TypeBatchAllocation typeBatchAllocation)
        {
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");

            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            var typeBatch = TypeBatches[typeBatchIndex];
            var handleRemover = new BodyHandleRemover(bodies, this);
            typeBatch.EnumerateConnectedBodyIndices(indexInTypeBatch, ref handleRemover);

            typeBatch.Remove(indexInTypeBatch, ref handlesToConstraints);

            RemoveTypeBatchIfEmpty(typeBatch, typeBatchIndex, typeBatchAllocation);
        }

        /// <summary>
        /// Clears all constraints from the constraint batch.
        /// </summary>
        public void Clear(TypeBatchAllocation typeBatchAllocation)
        {
            BodyHandles.Clear();
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                //Returning a type batch clears and disposes it.
                typeBatchAllocation.Return(TypeBatches[i], TypeBatches[i].TypeId);
            }
            //Since there are no more type batches, the mapping must be cleared out.
            for (int i = 0; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                TypeIndexToTypeBatchIndex[i] = -1;
            }
            TypeBatches.Clear();
        }
        public void EnsureCapacity(TypeBatchAllocation typeBatchAllocation, int bodiesCount, int constraintTypeCount)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                TypeBatches[i].EnsureCapacity(typeBatchAllocation);
            }
            BodyHandles.EnsureCapacity(bodiesCount, typeBatchAllocation.BufferPool);
            //For now this is mostly just for rehydration.
            if (TypeIndexToTypeBatchIndex.Length < constraintTypeCount)
            {
                ResizeTypeMap(typeBatchAllocation.BufferPool, constraintTypeCount);
                if (!TypeBatches.Span.Allocated)
                    QuickList<TypeBatch, Array<TypeBatch>>.Create(new PassthroughArrayPool<TypeBatch>(), constraintTypeCount, out TypeBatches);
                else
                    TypeBatches.Resize(constraintTypeCount, new PassthroughArrayPool<TypeBatch>());
            }
        }
        public void Compact(TypeBatchAllocation typeBatchAllocation, Bodies bodies, int bodiesCount)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                TypeBatches[i].Compact(typeBatchAllocation);
            }
            //Note that we can't shrink below the bodies handle capacity, since the handle distribution could be arbitrary.
            BodyHandles.Compact(Math.Max(bodies.IndexToHandle.Length, bodiesCount), typeBatchAllocation.BufferPool);
            //Compaction just doesn't change the type batch array sizes. It's a bit complicated and practically irrelevant.
        }
        public void Resize(TypeBatchAllocation typeBatchAllocation, Bodies bodies, int bodiesCount, int constraintTypeCount)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                TypeBatches[i].Resize(typeBatchAllocation);
            }
            //Note that we can't shrink below the bodies handle capacity, since the handle distribution could be arbitrary.
            BodyHandles.Resize(Math.Max(bodies.IndexToHandle.Length, bodiesCount), typeBatchAllocation.BufferPool);
            //For now this is mostly just for rehydration. Note that it's actually an EnsureCapacity. For simplicity, we just don't permit the compaction of the type batch arrays.
            if (TypeIndexToTypeBatchIndex.Length < constraintTypeCount)
            {
                ResizeTypeMap(typeBatchAllocation.BufferPool, constraintTypeCount);
                if (!TypeBatches.Span.Allocated)
                    QuickList<TypeBatch, Array<TypeBatch>>.Create(new PassthroughArrayPool<TypeBatch>(), constraintTypeCount, out TypeBatches);
                else
                    TypeBatches.Resize(constraintTypeCount, new PassthroughArrayPool<TypeBatch>());
            }
        }
        /// <summary>
        /// Disposes the unmanaged resources used by the batch and drops all pooled managed resources.
        /// </summary>
        /// <remarks>Calling EnsureCapacity or Resize will make the batch usable again after disposal.</remarks>
        public void Dispose(TypeBatchAllocation typeBatchAllocation)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                //Returning a type batch clears and disposes it.
                typeBatchAllocation.Return(TypeBatches[i], TypeBatches[i].TypeId);
            }
            TypeBatches.Clear();
            BodyHandles.Dispose(typeBatchAllocation.BufferPool);
            typeBatchAllocation.BufferPool.SpecializeFor<int>().Return(ref TypeIndexToTypeBatchIndex);
            TypeIndexToTypeBatchIndex = new Buffer<int>();
            TypeBatches.Dispose(new PassthroughArrayPool<TypeBatch>());
            TypeBatches = new QuickList<TypeBatch, Array<TypeBatch>>();
        }
    }
}
