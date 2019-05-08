using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuUtilities;

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

    internal struct IslandScaffoldTypeBatch
    {
        public int TypeId;
        public QuickList<int> Handles;

        public IslandScaffoldTypeBatch(BufferPool pool, int typeId, int initialTypeBatchSize)
        {
            TypeId = typeId;
            Handles = new QuickList<int>(initialTypeBatchSize, pool);
        }
    }

    //TODO: There's quite a bit of redundant logic here with the constraint batch and solver. Very likely that we could share more.
    internal struct IslandScaffoldConstraintBatch
    {
        public Buffer<int> TypeIdToIndex;
        public QuickList<IslandScaffoldTypeBatch> TypeBatches;
        //Note that we use *indices* during island construction, not handles. This protobatch doesn't have to deal with memory moves in between adds, so indices are fine.
        public IndexSet ReferencedBodyIndices;

        public unsafe IslandScaffoldConstraintBatch(Solver solver, BufferPool pool, int batchIndex)
        {
            pool.TakeAtLeast(solver.TypeProcessors.Length, out TypeIdToIndex);
            Unsafe.InitBlockUnaligned(TypeIdToIndex.Memory, 0xFF, (uint)(TypeIdToIndex.Length * sizeof(int)));
            TypeBatches = new QuickList<IslandScaffoldTypeBatch>(solver.TypeProcessors.Length, pool);
            ReferencedBodyIndices = batchIndex < solver.FallbackBatchThreshold ? new IndexSet(pool, solver.bodies.ActiveSet.Count) : default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ref IslandScaffoldTypeBatch GetOrCreateTypeBatch(int typeId, Solver solver, BufferPool pool)
        {
            ref var idMap = ref TypeIdToIndex[typeId];
            if (idMap == -1)
            {
                idMap = TypeBatches.Count;
                ref var typeBatch = ref TypeBatches.AllocateUnsafely();
                typeBatch = new IslandScaffoldTypeBatch(pool, typeId, solver.GetMinimumCapacityForType(typeId));
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
                        "The handle mapping isn't yet updated, but the type id shouldn't change during a sleep.");
                }
            }
        }

        public unsafe bool TryAdd(int constraintHandle, int batchIndex, Solver solver, BufferPool pool, ref FallbackBatch fallbackBatch)
        {
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
            if (batchIndex == solver.FallbackBatchThreshold || ReferencedBodyIndices.CanFit(ref enumerator.BodyIndices[0], bodiesPerConstraint))
            {
                ref var typeBatch = ref GetOrCreateTypeBatch(constraintLocation.TypeId, solver, pool);
                Debug.Assert(typeBatch.TypeId == constraintLocation.TypeId);
                typeBatch.Handles.Add(constraintHandle, pool);
                if (batchIndex < solver.FallbackBatchThreshold)
                {
                    for (int i = 0; i < bodiesPerConstraint; ++i)
                    {
                        ReferencedBodyIndices.AddUnsafely(enumerator.BodyIndices[i]);
                    }
                }
                else
                {
                    //This is the fallback batch, so we need to fill the fallback batch with relevant information.
                    var bodyHandles = stackalloc int[bodiesPerConstraint];
                    for (int i = 0; i < bodiesPerConstraint; ++i)
                    {
                        bodyHandles[i] = solver.bodies.ActiveSet.IndexToHandle[bodyIndices[i]];
                    }
                    fallbackBatch.AllocateForInactive(constraintHandle, ref *bodyHandles, bodiesPerConstraint, solver.bodies, constraintLocation.TypeId, pool);
                }
                return true;
            }
            return false;
        }

        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < TypeBatches.Count; ++i)
            {
                TypeBatches[i].Handles.Dispose(pool);
            }
            TypeBatches.Dispose(pool);
            pool.Return(ref TypeIdToIndex);
            if (ReferencedBodyIndices.flags.Allocated)
                ReferencedBodyIndices.Dispose(pool);
        }

    }

    /// <summary>
    /// Represents the constraint batch structure and all references in an island. Holds everything necessary to create and gather a full island.
    /// </summary>
    internal struct IslandScaffold
    {
        public QuickList<int> BodyIndices;
        public QuickList<IslandScaffoldConstraintBatch> Protobatches;
        public FallbackBatch FallbackBatch;

        public IslandScaffold(ref QuickList<int> bodyIndices, ref QuickList<int> constraintHandles, Solver solver, BufferPool pool) : this()
        {
            Debug.Assert(bodyIndices.Count > 0, "Don't be tryin' to create islands with no bodies in them! That don't make no sense.");
            //Create a copy of the body indices with just enough space to hold the island's indices. The original list will continue to be reused in the caller.
            BodyIndices = new QuickList<int>(bodyIndices.Count, pool);
            bodyIndices.Span.CopyTo(0, ref BodyIndices.Span, 0, bodyIndices.Count);
            BodyIndices.Count = bodyIndices.Count;
            Protobatches = new QuickList<IslandScaffoldConstraintBatch>(solver.ActiveSet.Batches.Count, pool);
            for (int i = 0; i < constraintHandles.Count; ++i)
            {
                AddConstraint(constraintHandles[i], solver, pool);
            }
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
            for (int batchIndex = 0; batchIndex < Protobatches.Count; ++batchIndex)
            {
                if (Protobatches[batchIndex].TryAdd(constraintHandle, batchIndex, solver, pool, ref FallbackBatch))
                {
                    return;
                }
            }
            if (Protobatches.Span.Length == Protobatches.Count)
                Protobatches.EnsureCapacity(Protobatches.Count + 1, pool);
            var newBatchIndex = Protobatches.Count;
            ref var newBatch = ref Protobatches.AllocateUnsafely();
            newBatch = new IslandScaffoldConstraintBatch(solver, pool, newBatchIndex);
            newBatch.TryAdd(constraintHandle, newBatchIndex, solver, pool, ref FallbackBatch);
        }

        internal void Dispose(BufferPool pool)
        {
            BodyIndices.Dispose(pool);
            for (int k = 0; k < Protobatches.Count; ++k)
            {
                Protobatches[k].Dispose(pool);
            }
            Protobatches.Dispose(pool);
            FallbackBatch.Dispose(pool);
        }
    }

}