using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System;

namespace BepuPhysics
{
    unsafe struct ConstraintHandleEnumerator : IForEach<int>
    {
        public int* BodyIndices;
        public int IndexInConstraint;
        public void LoopBody(int i)
        {
            BodyIndices[IndexInConstraint++] = i;
        }
    }

    public struct IslandProtoTypeBatch
    {
        public int TypeId;
        public QuickList<int, Buffer<int>> Handles;

        public IslandProtoTypeBatch(BufferPool<int> intPool, int typeId, int initialTypeBatchSize)
        {
            TypeId = typeId;
            QuickList<int, Buffer<int>>.Create(intPool, initialTypeBatchSize, out Handles);
        }
    }

    //TODO: There's quite a bit of redundant logic here with the constraint batch and solver. Very likely that we could share more.
    public struct IslandProtoConstraintBatch
    {
        public Buffer<int> TypeIdToIndex;
        public QuickList<IslandProtoTypeBatch, Buffer<IslandProtoTypeBatch>> TypeBatches;
        //Note that we use *indices* during island construction, not handles. This protobatch doesn't have to deal with memory moves in between adds, so indices are fine.
        public IndexSet ReferencedBodyIndices;

        public unsafe IslandProtoConstraintBatch(Solver solver, BufferPool pool)
        {
            pool.SpecializeFor<int>().Take(solver.TypeProcessors.Length, out TypeIdToIndex);
            Unsafe.InitBlockUnaligned(TypeIdToIndex.Memory, 0xFF, (uint)(TypeIdToIndex.Length * sizeof(int)));
            QuickList<IslandProtoTypeBatch, Buffer<IslandProtoTypeBatch>>.Create(pool.SpecializeFor<IslandProtoTypeBatch>(), solver.TypeProcessors.Length, out TypeBatches);
            ReferencedBodyIndices = new IndexSet(pool, solver.bodies.ActiveSet.Count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ref IslandProtoTypeBatch GetOrCreateTypeBatch(int typeId, Solver solver, BufferPool<int> intPool)
        {
            ref var idMap = ref TypeIdToIndex[typeId];
            if (idMap == -1)
            {
                idMap = TypeBatches.Count;
                ref var typeBatch = ref TypeBatches.AllocateUnsafely();
                typeBatch = new IslandProtoTypeBatch(intPool, typeId, solver.GetMinimumCapacityForType(typeId));
                return ref typeBatch;
            }
            return ref TypeBatches[idMap];
        }

        [Conditional("DEBUG")]
        internal void Validate(Solver solver)
        {
            for (int j = 0; j < TypeBatches.Count; ++j)
            {
                ref var typeBatch = ref TypeBatches[j];
                Debug.Assert(typeBatch.Handles.Count > 0, "If we created a type batch, it better have some constraints in it!");
                Debug.Assert(TypeIdToIndex[typeBatch.TypeId] == j);
                for (int k = 0; k < typeBatch.Handles.Count; ++k)
                {
                    var handle = typeBatch.Handles[k];
                    Debug.Assert(solver.HandleToConstraint[handle].TypeId == typeBatch.TypeId,
                        "The handle mapping isn't yet updated, but the type id shouldn't change during a deactivation.");
                }
            }
        }

        public unsafe bool TryAdd(int constraintHandle, Solver solver, BufferPool pool)
        {
            Validate(solver);
            ref var constraintLocation = ref solver.HandleToConstraint[constraintHandle];
            var typeProcessor = solver.TypeProcessors[constraintLocation.TypeId];
            var bodiesPerConstraint = typeProcessor.BodiesPerConstraint;
            var bodyIndices = stackalloc int[bodiesPerConstraint];
            ConstraintHandleEnumerator enumerator;
            enumerator.BodyIndices = bodyIndices;
            enumerator.IndexInConstraint = 0;
            typeProcessor.EnumerateConnectedBodyIndices(
                ref solver.ActiveSet.Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId),
                constraintLocation.IndexInTypeBatch,
                ref enumerator);
            if (ReferencedBodyIndices.CanFit(ref enumerator.BodyIndices[0], bodiesPerConstraint))
            {
                var intPool = pool.SpecializeFor<int>();
                ref var typeBatch = ref GetOrCreateTypeBatch(constraintLocation.TypeId, solver, intPool);
                Debug.Assert(typeBatch.TypeId == constraintLocation.TypeId);
                typeBatch.Handles.Add(constraintHandle, intPool);
                for (int i = 0; i < bodiesPerConstraint; ++i)
                {
                    ReferencedBodyIndices.AddUnsafely(enumerator.BodyIndices[i]);
                }
                Validate(solver);
                return true;
            }
            Validate(solver);
            return false;
        }
        
        public void Dispose(BufferPool pool)
        {
            var intPool = pool.SpecializeFor<int>();
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                TypeBatches[i].Handles.Dispose(intPool);
            }
            TypeBatches.Dispose(pool.SpecializeFor<IslandProtoTypeBatch>());
            intPool.Return(ref TypeIdToIndex);
            ReferencedBodyIndices.Dispose(pool);
        }

    }

    /// <summary>
    /// Represents the constraint batch structure and all references in an island. Holds everything necessary to create and gather a full island.
    /// </summary>
    public struct Island
    {
        public QuickList<int, Buffer<int>> BodyIndices;
        public QuickList<IslandProtoConstraintBatch, Buffer<IslandProtoConstraintBatch>> Protobatches;

        public Island(ref QuickList<int, Buffer<int>> bodyIndices, ref QuickList<int, Buffer<int>> constraintHandles, Solver solver, BufferPool pool) : this()
        {
            Debug.Assert(bodyIndices.Count > 0, "Don't be tryin' to create islands with no bodies in them! That don't make no sense.");
            //Create a copy of the body indices with just enough space to hold the island's indices. The original list will continue to be reused in the caller.
            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), bodyIndices.Count, out BodyIndices);
            bodyIndices.Span.CopyTo(0, ref BodyIndices.Span, 0, bodyIndices.Count);
            BodyIndices.Count = bodyIndices.Count;
            QuickList<IslandProtoConstraintBatch, Buffer<IslandProtoConstraintBatch>>.Create(
                pool.SpecializeFor<IslandProtoConstraintBatch>(), solver.ActiveSet.Batches.Count, out Protobatches);
            for (int i = 0; i < constraintHandles.Count; ++i)
            {
                AddConstraint(constraintHandles[i], solver, pool);
            }
            Validate(solver);
        }

        [Conditional("DEBUG")]
        public void Validate(Solver solver)
        {
            for (int i = 0; i < Protobatches.Count; ++i)
            {
                Protobatches[i].Validate(solver);
            }
        }

        void AddConstraint(int constraintHandle, Solver solver, BufferPool pool)
        {
            Validate(solver);

            for (int batchIndex = 0; batchIndex < Protobatches.Count; ++batchIndex)
            {
                if (Protobatches[batchIndex].TryAdd(constraintHandle, solver, pool))
                {
                    Validate(solver);
                    return;
                }
            }
            if (Protobatches.Span.Length == Protobatches.Count)
                Protobatches.EnsureCapacity(Protobatches.Count + 1, pool.SpecializeFor<IslandProtoConstraintBatch>());
            ref var newBatch = ref Protobatches.AllocateUnsafely();
            newBatch = new IslandProtoConstraintBatch(solver, pool);
            newBatch.TryAdd(constraintHandle, solver, pool);
            Validate(solver);
        }

        internal void Dispose(BufferPool pool)
        {
            BodyIndices.Dispose(pool.SpecializeFor<int>());
            for (int k = 0; k < Protobatches.Count; ++k)
            {
                Protobatches[k].Dispose(pool);
            }
            Protobatches.Dispose(pool.SpecializeFor<IslandProtoConstraintBatch>());
        }
    }

}