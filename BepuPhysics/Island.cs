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
            solver.EnumerateConnectedBodyIndices(constraintHandle, ref enumerator);
            //for (int i =0)
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
            for (int i =0; i < TypeBatches.Count; ++i)
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

    /// <summary>
    /// Stores the data associated with a deactivated set of bodies and constraints.
    /// </summary>
    public struct Island
    {
        public QuickList<ConstraintBatch, Buffer<ConstraintBatch>> ConstraintBatches;
        public int BodyCount;
        public Buffer<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>> BodyConstraintReferences;
        public Buffer<int> Handles;
        public Buffer<RigidPose> Poses;
        public Buffer<BodyVelocity> Velocities;
        public Buffer<BodyInertia> LocalInertias;

        /// <summary>
        /// Gets whether this island instance is actually allocated.
        /// </summary>
        public bool Allocated { get { return Handles.Allocated; } }

        //Removals from islands aren't ideal- they scramble the order of objects. But they should be extremely rare for the most part.
        public void RemoveBodyAt(int bodyIndex, Bodies bodies, BroadPhase broadPhase)
        {
            Debug.Assert(bodyIndex >= 0 && bodyIndex < BodyCount);
            //Note that we require the user to remove all constraints associated with a body before removing it.
            Debug.Assert(BodyConstraintReferences[bodyIndex].Count == 0);

            //A little bit annoying- we duplicate a little bit of the bookkeeping associated with body removal. TODO: see if you can nicely unify this.
            var handle = Handles[bodyIndex];
            ref var bodiesMap = ref bodies.HandleToIndex[handle];
            bodiesMap.Island = -1;
            bodiesMap.Index = -1;
            bodies.HandlePool.Return(handle, bodies.pool.SpecializeFor<int>());

            var lastIndex = BodyCount - 1;
            if (bodyIndex < lastIndex)
            {
                BodyConstraintReferences[bodyIndex] = BodyConstraintReferences[lastIndex];
                Poses[bodyIndex] = Poses[lastIndex];
                Velocities[bodyIndex] = Velocities[lastIndex];
                LocalInertias[bodyIndex] = LocalInertias[lastIndex];
                Handles[bodyIndex] = Handles[lastIndex];
            }
            BodyCount = lastIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveConstraintAt(int batchIndex, int typeBatchIndex, int indexInTypeBatch, Solver solver)
        {
            //Constraint removals require type knowledge. We don't have any within the island, and we'd really rather not duplicate the logic required to extract it.
            //Just make use of the solver's virtual dispatch support.
            //Note that the virtual dispatch really doesn't matter- this should not be something that occurs frequently.
            ref var typeBatch = ref ConstraintBatches[batchIndex].TypeBatches[typeBatchIndex];
            solver.TypeProcessors[typeBatch.TypeId].Remove(ref typeBatch, indexInTypeBatch, ref solver.HandleToConstraint);
            //Note that it's possible for type batches to end up with zero constraints through this function. We make no attempt to prevent that; 
            //we do not enumerate over inactive batches every frame, so empty slots are not a problem. This is a corner case that the island activator must handle, though.
        }

        public void Create(ref QuickList<int, Buffer<int>> bodyHandles, ref QuickList<int, Buffer<int>> constraintHandles,
            BufferPool mainPool, BufferPool threadPool)
        {
            //Note that, while we did encounter the indices associated with the island bodies handles and *could* have cached them, we opted to store the handles instead.
            //This does incur additional (warm) indirections, but we would like to also use the handles again- to remove from the active set.
            //Creating this island does not modify anything about the existing active set. All of that is deferred.

            //Note that, while we have already traversed the constraint's connected bodies to collect the island, we did not cache all required data during the traversal.
            //Doing so would be *usually* wasteful- the vast majority of traversals result in no deactivation.
            //Further, the traversal does not otherwise need to touch the prestep data and accumulated impulses. Those are quite large, so avoiding needless accesses
            //are important for keeping the traversal reasonably speedy.
            //Given that we have to grab that additional information anyway, and given that it is likely in L1 (or failing that, L2) cache, we re-enumerate the constraint body 
            //handles here.

            //We have a bit of an annoyance to deal with:
            //1) By convention, we never hold information in per-thread buffer pools between frames.
            //2) We'd like to be able to run island creation on multiple threads.
            //3) Island creation requires allocating space for all the body and constraint data.
            //Implication:
            //We must synchronize access to the main pool when retrieving persisted buffers. All ephemeral data comes from the thread pool.
            //While this isn't too problematic (time spent retrieving island resources is going to be extremely brief), 
            //the main pool access does restrict job scheduling with other main pool users that are unaware of the synchronization requirement.

            //Unless we perform constraint batching during traversal, the numbers of constraint batches, type batches, and constraints within individual type batches are unknown.
            //We cannot just lock once and allocate a minimally sufficient set of buffers.
            //An option:
            //1) Enumerate each constraint's bodies. Convert them to handles and perform batching, locally creating constraintbatches and type batches, but only fill the body references.
            //2) As you go, store the new handle->island location mapping.
            //3) Using the capacities detected by 

            var batchReferencedHandlesPool = threadPool.SpecializeFor<HandleSet>();
            var intPool = threadPool.SpecializeFor<int>();
            constraintHandles.
            batchReferencedHandlesPool.Take(16, out var batchReferencedHandles);

            for (int i = 0; i < ConstraintBatches.Count; ++i)
            {
                batchReferencedHandles[i].Dispose(threadPool);
            }
            batchReferencedHandlesPool.Return(ref batchReferencedHandles);

        }

    }
}
