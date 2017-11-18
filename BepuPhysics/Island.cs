using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    unsafe struct ConstraintHandleEnumerator : IForEach<int>
    {
        public int* Indices;
        public int IndexInConstraint;
        public void LoopBody(int i)
        {
            Indices[IndexInConstraint++] = i;
        }
    }

    public struct IslandProtoTypeBatch
    {
        public QuickList<int, Buffer<int>> Handles;

        public IslandProtoTypeBatch(BufferPool pool, int initialTypeBatchSize)
        {
            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), initialTypeBatchSize, out Handles);
        }
    }

    public struct IslandProtoConstraintBatch
    {
        public Buffer<int> TypeIdToIndex;
        public QuickList<IslandProtoTypeBatch, Buffer<IslandProtoTypeBatch>> TypeBatches;
        public HandleSet ReferencedBodyHandles;

        public unsafe IslandProtoConstraintBatch(Solver solver, BufferPool pool)
        {
            pool.SpecializeFor<int>().Take(solver.TypeProcessors.Length, out TypeIdToIndex);
            Unsafe.InitBlockUnaligned(TypeIdToIndex.Memory, 0xFF, (uint)(TypeIdToIndex.Length * sizeof(int)));
            QuickList<IslandProtoTypeBatch, Buffer<IslandProtoTypeBatch>>.Create(pool.SpecializeFor<IslandProtoTypeBatch>(), solver.TypeProcessors.Length, out TypeBatches);
            ReferencedBodyHandles = new HandleSet(pool, solver.bodies.HandlePool.HighestPossiblyClaimedId + 1);
        }

        //There is some roughly-duplicate logic here, compared to the 'real' constraint batch. 
        public unsafe static bool TryAdd(void* batch, int constraintHandle, Solver solver, BufferPool pool)
        {
            ref var constraintLocation = ref solver.HandleToConstraint[constraintHandle];
            var typeProcessor = solver.TypeProcessors[constraintLocation.TypeId];
            var bodiesPerConstraint = typeProcessor.BodiesPerConstraint;
            var bodies = stackalloc int[bodiesPerConstraint];
            ConstraintHandleEnumerator enumerator;
            enumerator.Indices = bodies;
            enumerator.IndexInConstraint = 0;
            typeProcessor.EnumerateConnectedBodyIndices(ref solver.Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId), constraintLocation.IndexInTypeBatch, ref enumerator);
            solver.EnumerateConnectedBodies(constraintHandle, ref enumerator);
            //for (int i =0)
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ref IslandProtoTypeBatch GetOrCreateTypeBatch(int typeId, BufferPool pool, int initialTypeBatchSize)
        {
            ref var idMap = ref TypeIdToIndex[typeId];
            if (idMap == -1)
            {
                idMap = TypeBatches.Count;
                ref var typeBatch = ref TypeBatches.AllocateUnsafely();
                typeBatch = new IslandProtoTypeBatch(pool, initialTypeBatchSize);
                return ref typeBatch;
            }
            return ref TypeBatches[typeId];
        }
        public unsafe bool TryAdd(int constraintHandle, ref ConstraintLocation constraintLocation, int* bodyHandles, int bodyHandleCount, BufferPool pool, int initialTypeBatchSize)
        {
            if (ReferencedBodyHandles.CanFit(ref bodyHandles[0], bodyHandleCount))
            {
                ref var typeBatch = ref GetOrCreateTypeBatch(constraintLocation.TypeId, pool, initialTypeBatchSize);
                typeBatch.Handles.AllocateUnsafely() = constraintHandle;
                for (int i = 0; i < bodyHandleCount; ++i)
                {
                    ReferencedBodyHandles.Add(bodyHandles[i], pool);
                }
                return true;
            }
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
            ReferencedBodyHandles.Dispose(pool);
        }

    }

    /// <summary>
    /// Represents the constraint batch structure and all references in an island. Holds everything necessary to create and gather a full island.
    /// </summary>
    public struct IslandStructure
    {
        public QuickList<int, Buffer<int>> BodyHandles;
        public QuickList<IslandProtoConstraintBatch, Buffer<IslandProtoConstraintBatch>> Batches;
    }

}