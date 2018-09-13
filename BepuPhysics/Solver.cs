using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuPhysics.CollisionDetection;
using BepuUtilities;

namespace BepuPhysics
{
    public unsafe struct ConstraintReference
    {
        //TODO: Once blittable exists, we can give this a proper type. Blocked by generics interference in TypeBatch.
        //May want to just treat this as opaque.
        internal void* typeBatchPointer;
        public ref TypeBatch TypeBatch
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Unsafe.AsRef<TypeBatch>(typeBatchPointer);
            }
        }
        public readonly int IndexInTypeBatch;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ConstraintReference(ref TypeBatch typeBatch, int indexInTypeBatch)
        {
            typeBatchPointer = Unsafe.AsPointer(ref typeBatch);
            IndexInTypeBatch = indexInTypeBatch;
        }
    }

    public struct ConstraintLocation
    {
        //Note that the type id is included, even though we can extract it from a type parameter.
        //This is required for body memory swap induced reference changes- it is not efficient to include type metadata in the per-body connections,
        //so instead we keep a type id cached.
        //(You could pack these a bit- it's pretty reasonable to say you can't have more than 2^24 constraints of a given type and 2^8 constraint types...
        //It's just not that valuable, until proven otherwise.)
        public int SetIndex;
        public int BatchIndex;
        public int TypeId;
        public int IndexInTypeBatch;
    }

    public partial class Solver
    {

        /// <summary>
        /// Buffer containing all constraint sets. The first slot is dedicated to the active set; subsequent slots may be occupied by the constraints associated with inactive islands.
        /// </summary>
        public Buffer<ConstraintSet> Sets;

        /// <summary>
        /// Gets a reference to the active set of constraints, stored in the first set slot.
        /// </summary>
        public ref ConstraintSet ActiveSet { get { return ref Sets[0]; } }

        //Note that the referenced handles for the active set are stored outside the constraint set;
        //inactive islands do not store the referenced handles since no new constraints are ever added.
        internal QuickList<IndexSet, Buffer<IndexSet>> batchReferencedHandles;

        public TypeProcessor[] TypeProcessors;

        internal Bodies bodies;
        internal PairCache pairCache;
        internal IslandAwakener awakener;

        /// <summary>
        /// Pool to retrieve constraint handles from when creating new constraints.
        /// </summary>
        public IdPool<Buffer<int>> HandlePool;
        internal BufferPool bufferPool;
        public Buffer<ConstraintLocation> HandleToConstraint;

        //TODO: Make this settable. Unclear if it's really appropriate for a property- properties are conventionally cheap, and changing the maximum batch count on an existing simulation is far from cheap.
        /// <summary>
        /// Gets the maximum number of solver batches to allow before resorting to a fallback solver.
        /// If a single body is constrained by more than FallbackBatchThreshold constraints, all constraints beyond FallbackBatchThreshold are placed into a fallback batch.
        /// The fallback batch uses a different solver that can handle multiple constraints affecting a single body in a single batch, allowing greater parallelism at the cost of convergence speed.
        /// </summary>
        public int FallbackBatchThreshold { get; private set; } = 0;


        int iterationCount;
        /// <summary>
        /// Gets or sets the number of solver iterations to compute per call to Update.
        /// </summary>
        public int IterationCount
        {
            get { return iterationCount; }
            set
            {
                if (value < 1)
                {
                    throw new ArgumentException("Iteration count must be positive.");
                }
                iterationCount = value;
            }
        }

        int minimumCapacityPerTypeBatch;
        /// <summary>
        /// Gets or sets the minimum amount of space, in constraints, initially allocated in any new type batch.
        /// </summary>
        public int MinimumCapacityPerTypeBatch
        {
            get { return minimumCapacityPerTypeBatch; }
            set
            {
                minimumCapacityPerTypeBatch = Math.Max(1, value);
            }
        }
        int[] minimumInitialCapacityPerTypeBatch = new int[0];

        /// <summary>
        /// Sets the minimum capacity initially allocated to a new type batch of the given type.
        /// </summary>
        /// <param name="typeId">Id of the constraint type to check the initial capacity of.</param>
        /// <param name="minimumInitialCapacityForType">Minimum capacity to use for the type.</param>
        public void SetMinimumCapacityForType(int typeId, int minimumInitialCapacityForType)
        {
            if (typeId < 0)
                throw new ArgumentException("Type id must be nonnegative.");
            if (MinimumCapacityPerTypeBatch < 0)
                throw new ArgumentException("Capacity must be nonnegative.");
            if (typeId >= minimumInitialCapacityPerTypeBatch.Length)
                Array.Resize(ref minimumInitialCapacityPerTypeBatch, typeId + 1);
            minimumInitialCapacityPerTypeBatch[typeId] = minimumInitialCapacityForType;
        }

        /// <summary>
        /// Gets the minimum initial capacity for a given type.
        /// The returned value is the larger of MinimumCapacityPerTypeBatch and the value set by SetMinimumCapacityForType for the given type id.
        /// </summary>
        /// <param name="typeId">Type id to retrieve the minm</param>
        /// <returns>Larger of MinimumCapacityPerTypeBatch and the given type's minimum set by SetMinimumCapacityForType.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetMinimumCapacityForType(int typeId)
        {
            //Note that performance is slightly more important here, hence the use of inlining and an assert over exceptions.
            Debug.Assert(typeId >= 0, "Type ids must be nonnegative.");
            if (typeId >= minimumInitialCapacityPerTypeBatch.Length)
                return minimumCapacityPerTypeBatch;
            return Math.Max(minimumInitialCapacityPerTypeBatch[typeId], minimumCapacityPerTypeBatch);
        }
        /// <summary>
        /// Resets all per-type initial capacities to zero. Leaves the minimum capacity across all constraints unchanged.
        /// </summary>
        public void ResetPerTypeInitialCapacities()
        {
            Array.Clear(minimumInitialCapacityPerTypeBatch, 0, minimumInitialCapacityPerTypeBatch.Length);
        }

        /// <summary>
        /// Gets the total number of constraints across all sets, batches, and types. Requires enumerating
        /// all type batches; this can be expensive.
        /// </summary>
        public int CountConstraints()
        {
            int count = 0;
            for (int setIndex = 0; setIndex < Sets.Length; ++setIndex)
            {
                ref var set = ref Sets[setIndex];
                if (set.Allocated)
                {
                    for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
                    {
                        ref var batch = ref set.Batches[batchIndex];
                        for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                        {
                            count += batch.TypeBatches[typeBatchIndex].ConstraintCount;
                        }
                    }
                }
            }
            return count;
        }


        Action<int> workDelegate;
        //TODO: While 32 batches will likely cover most simulations, this will likely change when the jacobi-hybrid fallback is implemented in favor of a user configurable threshold.
        const int BatchCountEstimate = 32;
        public Solver(Bodies bodies, BufferPool bufferPool, int iterationCount,
            int initialCapacity,
            int initialIslandCapacity,
            int minimumCapacityPerTypeBatch)
        {
            this.iterationCount = iterationCount;
            this.minimumCapacityPerTypeBatch = minimumCapacityPerTypeBatch;
            this.bodies = bodies;
            this.bufferPool = bufferPool;
            IdPool<Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), 128, out HandlePool);
            ResizeSetsCapacity(initialIslandCapacity + 1, 0);
            ActiveSet = new ConstraintSet(bufferPool, BatchCountEstimate);
            QuickList<IndexSet, Buffer<IndexSet>>.Create(bufferPool.SpecializeFor<IndexSet>(), BatchCountEstimate, out batchReferencedHandles);
            bufferPool.SpecializeFor<ConstraintLocation>().Take(initialCapacity, out HandleToConstraint);
            workDelegate = Work;
        }

        public void Register<TDescription>() where TDescription : struct, IConstraintDescription<TDescription>
        {
            var description = default(TDescription);
            Debug.Assert(description.ConstraintTypeId >= 0, "Constraint type ids should never be negative. They're used for array indexing.");
            if (TypeProcessors == null || description.ConstraintTypeId >= TypeProcessors.Length)
            {
                //This will result in some unnecessary resizes, but it hardly matters. It only happens once on registration time.
                //This also means we can just take the current type processors length as an accurate measure of type capacity for constraint batches.
                Array.Resize(ref TypeProcessors, description.ConstraintTypeId + 1);
            }
            if (TypeProcessors[description.ConstraintTypeId] != null)
            {
                throw new ArgumentException(
                    $"Type processor {TypeProcessors[description.ConstraintTypeId].GetType().Name} has already been registered for this description's type id " +
                    $"({typeof(TDescription).Name}, {default(TDescription).ConstraintTypeId}). " +
                    $"Cannot register the same type id more than once.");
            }
            var processor = (TypeProcessor)Activator.CreateInstance(description.BatchType);
            TypeProcessors[description.ConstraintTypeId] = processor;
            processor.Initialize(description.ConstraintTypeId);
        }

        /// <summary>
        /// Gets a direct reference to the constraint associated with a handle.
        /// The reference is temporary; any constraint removals that affect the referenced type batch may invalidate the index.
        /// </summary>
        /// <typeparam name="T">Type of the type batch being referred to.</typeparam>
        /// <param name="handle">Handle index of the constraint.</param>
        /// <param name="reference">Temporary direct reference to the type batch and index in the type batch associated with the constraint handle.
        /// May be invalidated by constraint removals.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetConstraintReference(int handle, out ConstraintReference reference)
        {
            ref var constraintLocation = ref HandleToConstraint[handle];
            reference = new ConstraintReference(ref Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId), constraintLocation.IndexInTypeBatch);
        }

        [Conditional("DEBUG")]
        private void ValidateBodyReference(int body, int expectedCount, ref ConstraintBatch batch)
        {
            int referencesToBody = 0;
            for (int i = 0; i < batch.TypeBatches.Count; ++i)
            {
                var instancesInTypeBatch = TypeProcessors[batch.TypeBatches[i].TypeId].GetBodyReferenceCount(ref batch.TypeBatches[i], body);
                Debug.Assert(instancesInTypeBatch + referencesToBody <= expectedCount,
                    "Found an instance of a body index that wasn't expected. Possible upstream bug or memory corruption.");
                referencesToBody += instancesInTypeBatch;
            }
            Debug.Assert(referencesToBody == expectedCount);
        }

        [Conditional("DEBUG")]
        public unsafe void ValidateExistingHandles(bool activeOnly = false)
        {
            var maxBodySet = activeOnly ? 1 : bodies.Sets.Length;
            for (int i = 0; i < maxBodySet; ++i)
            {
                ref var set = ref bodies.Sets[i];
                if (set.Allocated)
                {
                    for (int j = 0; j < set.Count; ++j)
                    {
                        bodies.ValidateExistingHandle(set.IndexToHandle[j]);
                    }
                }
            }
            //Validate the bodies referenced in the active batchReferencedHandles collections. 
            //Note that this only applies to the active set synchronized batches; inactive sets and the fallback batch do not explicitly track referenced handles.
            for (int batchIndex = 0; batchIndex < Math.Min(ActiveSet.Batches.Count, FallbackBatchThreshold); ++batchIndex)
            {
                ref var handles = ref batchReferencedHandles[batchIndex];
                ref var batch = ref ActiveSet.Batches[batchIndex];
                for (int i = 0; i < bodies.ActiveSet.Count; ++i)
                {
                    var handle = bodies.ActiveSet.IndexToHandle[i];
                    if (handles.Contains(handle))
                        ValidateBodyReference(i, 1, ref batch);
                    else
                        ValidateBodyReference(i, 0, ref batch);
                }
            }
            //Now, for all sets, validate that constraint and body references to each other are consistent and complete.
            ReferenceCollector enumerator;
            int maximumBodiesPerConstraint = 0;
            for (int i = 0; i < TypeProcessors.Length; ++i)
            {
                if (TypeProcessors[i] != null && TypeProcessors[i].BodiesPerConstraint > maximumBodiesPerConstraint)
                    maximumBodiesPerConstraint = TypeProcessors[i].BodiesPerConstraint;
            }
            var constraintBodyReferences = stackalloc int[maximumBodiesPerConstraint];
            enumerator.References = constraintBodyReferences;
            var maxConstraintSet = activeOnly ? 1 : Sets.Length;
            for (int setIndex = 0; setIndex < maxConstraintSet; ++setIndex)
            {
                ref var set = ref Sets[setIndex];
                if (set.Allocated)
                {
                    Debug.Assert(bodies.Sets.Length > setIndex);
                    //We are going to test this in both directions:
                    //For each constraint, for each body within that constraint, confirm that the referenced body includes the constraint in its constraint list.
                    //For each body, for each constraint referenced by that body, confirm that the referenced constraint includes the body in its body list.
                    ref var bodySet = ref bodies.Sets[setIndex];
                    Debug.Assert(bodySet.Allocated, "For any existing constraint set, there must be an aligned existing body set. Constraints don't exist by themselves.");
                    for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
                    {
                        ref var batch = ref set.Batches[batchIndex];
                        for (int typebatchIndex = 0; typebatchIndex < batch.TypeBatches.Count; ++typebatchIndex)
                        {
                            ref var typeBatch = ref batch.TypeBatches[typebatchIndex];
                            var processor = TypeProcessors[typeBatch.TypeId];
                            for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.ConstraintCount; ++indexInTypeBatch)
                            {
                                enumerator.Index = 0;
                                processor.EnumerateConnectedBodyIndices(ref typeBatch, indexInTypeBatch, ref enumerator);

                                for (int i = 0; i < processor.BodiesPerConstraint; ++i)
                                {
                                    //Active constraints refer to bodies with an index. Inactive constraints use a handle.
                                    int bodyIndex;
                                    if (setIndex == 0)
                                    {
                                        bodyIndex = constraintBodyReferences[i];
                                    }
                                    else
                                    {
                                        ref var referencedBodyLocation = ref bodies.HandleToLocation[constraintBodyReferences[i]];
                                        Debug.Assert(referencedBodyLocation.SetIndex == setIndex, "Any body involved with a constraint should be in the same set.");
                                        bodyIndex = referencedBodyLocation.Index;
                                    }
                                    Debug.Assert(bodySet.BodyIsConstrainedBy(bodyIndex, typeBatch.IndexToHandle[indexInTypeBatch]),
                                        "If a constraint refers to a body, the body should refer to the constraint.");
                                }
                            }
                        }
                    }
                    //Now for the body->constraint direction.
                    for (int bodyIndex = 0; bodyIndex < bodySet.Count; ++bodyIndex)
                    {
                        ref var constraintList = ref bodySet.Constraints[bodyIndex];
                        for (int constraintIndex = 0; constraintIndex < constraintList.Count; ++constraintIndex)
                        {
                            ref var constraintLocation = ref HandleToConstraint[constraintList[constraintIndex].ConnectingConstraintHandle];
                            Debug.Assert(constraintLocation.SetIndex == setIndex, "Any constraint involved with a body should be in the same set.");
                            ref var batch = ref set.Batches[constraintLocation.BatchIndex];
                            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
                            var processor = TypeProcessors[typeBatch.TypeId];
                            enumerator.Index = 0;
                            processor.EnumerateConnectedBodyIndices(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
                            //Active constraints refer to bodies by index; inactive constraints use handles.
                            int bodyReference = setIndex == 0 ? bodyIndex : bodySet.IndexToHandle[bodyIndex];
                            Debug.Assert(constraintBodyReferences[constraintList[constraintIndex].BodyIndexInConstraint] == bodyReference,
                                "If a body refers to a constraint, the constraint should refer to the body.");
                        }
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        public void ValidateConstraintMaps(bool activeOnly = false)
        {
            var setCount = activeOnly ? 1 : Sets.Length;
            for (int setIndex = 0; setIndex < setCount; ++setIndex)
            {
                ref var set = ref Sets[setIndex];
                if (set.Allocated)
                {
                    for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
                    {
                        ref var batch = ref set.Batches[batchIndex];
                        for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                        {
                            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                            for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.ConstraintCount; ++indexInTypeBatch)
                            {
                                var handle = typeBatch.IndexToHandle[indexInTypeBatch];
                                ref var constraintLocation = ref HandleToConstraint[handle];
                                Debug.Assert(constraintLocation.BatchIndex == batchIndex);
                                Debug.Assert(constraintLocation.IndexInTypeBatch == indexInTypeBatch);
                                Debug.Assert(constraintLocation.TypeId == typeBatch.TypeId);
                                Debug.Assert(batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId] == typeBatchIndex);
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Attempts to locate a spot for a new constraint. Does not perform allocation for the constraint. If no batch exists, returns the index just beyond the end of the existing list of batches.
        /// </summary>
        /// <param name="batchStartIndex">Index at which to start the search.</param>
        /// <returns>Index of the batch that the constraint would fit in.</returns>
        /// <remarks>This is used by the narrowphase's multithreaded constraint adders to locate a spot for a new constraint without requiring a lock. Only after a candidate is located
        /// do those systems attempt an actual claim, limiting the duration of locks and increasing potential parallelism.</remarks>
        internal unsafe int FindCandidateBatch(int batchStartIndex, ref int bodyHandles, int bodyCount)
        {
            ref var set = ref ActiveSet;
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                if (batchReferencedHandles[batchIndex].CanFit(ref bodyHandles, bodyCount))
                    return batchIndex;
            }
            //No synchronized batch worked. Either there's a fallback batch or there aren't yet enough batches to warrant a fallback batch and none of the existing batches could fit the handles.
            return synchronizedBatchCount;
        }

        internal unsafe void AllocateInBatch(int targetBatchIndex, int constraintHandle, ref int bodyHandles, int bodyCount, int typeId, out ConstraintReference reference)
        {
            ref var batch = ref ActiveSet.Batches[targetBatchIndex];
            batch.Allocate(constraintHandle, ref bodyHandles, bodyCount, bodies, typeId, TypeProcessors[typeId], GetMinimumCapacityForType(typeId), bufferPool, out reference);
            if (targetBatchIndex < FallbackBatchThreshold)
            {
                ref var handlesSet = ref batchReferencedHandles[targetBatchIndex];
                for (int j = 0; j < bodyCount; ++j)
                {
                    handlesSet.Add(Unsafe.Add(ref bodyHandles, j), bufferPool);
                }
            }
            else
            {
                Debug.Assert(targetBatchIndex == FallbackBatchThreshold);
                ActiveSet.Fallback.AllocateForActive(constraintHandle, ref bodyHandles, bodyCount, bodies, typeId, bufferPool);
            }
        }

        internal unsafe bool TryAllocateInBatch(int typeId, int targetBatchIndex, ref int bodyHandles, int bodyCount, out int constraintHandle, out ConstraintReference reference)
        {
            ref var set = ref ActiveSet;
            Debug.Assert(targetBatchIndex <= set.Batches.Count,
                "It should be impossible for a target batch to be generated which is more than one slot beyond the end of the batch list. Possible misuse of FindCandidateBatch.");
            if (targetBatchIndex == set.Batches.Count)
            {
                //No batch available. Have to create a new one.
                if (set.Batches.Count == set.Batches.Span.Length)
                    set.Batches.Resize(set.Batches.Count + 1, bufferPool.SpecializeFor<ConstraintBatch>());
                set.Batches.AllocateUnsafely() = new ConstraintBatch(bufferPool, TypeProcessors.Length);
                if (targetBatchIndex < FallbackBatchThreshold)
                {
                    //This batch is not the fallback batch, so create an index set for it.
                    if (set.Batches.Count == batchReferencedHandles.Span.Length)
                        batchReferencedHandles.Resize(set.Batches.Count + 1, bufferPool.SpecializeFor<IndexSet>());
                    batchReferencedHandles.AllocateUnsafely() = new IndexSet(bufferPool, bodies.ActiveSet.Count);
                }
                //Note that if there is no constraint batch for the given index, there is no way for the constraint add to be blocked. It's guaranteed success.
            }
            else
            {
                //Only non-fallback batches can block an incoming constraint. Additions to the fallback batch cannot be blocked; it permits multiple constraints associated with the same body.
                if (targetBatchIndex < FallbackBatchThreshold)
                {
                    //A non-fallback constraint batch already exists here. This may fail.
                    if (!batchReferencedHandles[targetBatchIndex].CanFit(ref bodyHandles, bodyCount))
                    {
                        //This batch cannot hold the constraint.
                        constraintHandle = -1;
                        reference = default;
                        return false;
                    }
                }
            }
            constraintHandle = HandlePool.Take();
            AllocateInBatch(targetBatchIndex, constraintHandle, ref bodyHandles, bodyCount, typeId, out reference);

            if (constraintHandle >= HandleToConstraint.Length)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, HandleToConstraint.Length * 2, HandleToConstraint.Length);
                Debug.Assert(constraintHandle < HandleToConstraint.Length, "Handle indices should never jump by more than 1 slot, so doubling should always be sufficient.");
            }
            ref var constraintLocation = ref HandleToConstraint[constraintHandle];
            //Note that new constraints are always active. It is assumed at this point that all connected bodies have been forced active.
            for (int i = 0; i < bodyCount; ++i)
            {
                ref var bodyLocation = ref bodies.HandleToLocation[Unsafe.Add(ref bodyHandles, i)];
                Debug.Assert(bodyLocation.SetIndex == 0, "New constraints should only be created involving already-activated bodies.");
            }
            constraintLocation.SetIndex = 0;
            constraintLocation.IndexInTypeBatch = reference.IndexInTypeBatch;
            constraintLocation.TypeId = typeId;
            constraintLocation.BatchIndex = targetBatchIndex;
            return true;
        }


        /// <summary>
        /// Applies a description to a constraint slot without waking up the associated island.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintReference">Reference of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        [MethodImpl(MethodImplOptions.NoInlining)]
        public void ApplyDescriptionWithoutWaking<TDescription>(ref ConstraintReference constraintReference, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            //TODO: Note that it would be pretty nice to allow in parameters to avoid the need for the inefficient value type convenience overloads.
            //The reason why we use ref is that the JIT does not recognize that this instance call is not mutating the instance.
            //It emits a localsinit AND a copy.
            //An ideal solution here (other than raw optimizer improvements) would be some language feature that permits the expression of functions-that-work-on-data
            //in a generic fashion without indirection, and without introducing syntax pain.
            //(If you accept syntax pain, it is possible already- pass a struct type that exposes interface implementations that process descriptions, but contains no data of its own.
            //That 'executor' type has trivial clearing cost which should go away entirely with inlining even with the current optimizer. Compare that level of added complexity
            //with IConstraintDescription simply carrying a requirement to implement a static function. Future versions of C# should make this sort of construct easier to deal with.)
            description.ApplyDescription(ref constraintReference.TypeBatch, bundleIndex, innerIndex);
        }

        /// <summary>
        /// Applies a description to a constraint slot without waking up the associated island.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintHandle">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescriptionWithoutWaking<TDescription>(int constraintHandle, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            GetConstraintReference(constraintHandle, out var constraintReference);
            ApplyDescriptionWithoutWaking(ref constraintReference, ref description);
        }
        /// <summary>
        /// Applies a description to a constraint slot without waking up the associated island.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintHandle">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescriptionWithoutWaking<TDescription>(int constraintHandle, TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            ApplyDescriptionWithoutWaking(constraintHandle, ref description);
        }

        /// <summary>
        /// Applies a description to a constraint slot, waking up the connected bodies if necessary.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintHandle">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescription<TDescription>(int constraintHandle, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            awakener.AwakenConstraint(constraintHandle);
            ApplyDescriptionWithoutWaking(constraintHandle, ref description);
        }
        /// <summary>
        /// Applies a description to a constraint slot, waking up the connected bodies if necessary.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintHandle">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescription<TDescription>(int constraintHandle, TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            ApplyDescriptionWithoutWaking(constraintHandle, ref description);
        }

        void Add<TDescription>(ref int bodyHandles, int bodyCount, ref TDescription description, out int handle)
            where TDescription : IConstraintDescription<TDescription>
        {
            ref var set = ref ActiveSet;
            for (int i = 0; i <= set.Batches.Count; ++i)
            {
                if (TryAllocateInBatch(description.ConstraintTypeId, i, ref bodyHandles, bodyCount, out handle, out var reference))
                {
                    ApplyDescriptionWithoutWaking(ref reference, ref description);
                    return;
                }
            }
            handle = -1;
            Debug.Fail("The above allocation loop checks every batch and also one index beyond all existing batches. It should be guaranteed to succeed.");
        }

        /// <summary>
        /// Allocates a constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandles">First body handle in a list of body handles used by the constraint.</param>
        /// <param name="bodyCount">Number of bodies used by the constraint.</param>
        /// <returns>Allocated constraint handle.</returns>
        public int Add<TDescription>(ref int bodyHandles, int bodyCount, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            //Adding a constraint assumes that the involved bodies are active, so wake up anything that is sleeping.
            for (int i = 0; i < bodyCount; ++i)
            {
                awakener.AwakenBody(Unsafe.Add(ref bodyHandles, i));
            }
            Add(ref bodyHandles, bodyCount, ref description, out int constraintHandle);
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyHandle = Unsafe.Add(ref bodyHandles, i);
                bodies.ValidateExistingHandle(bodyHandle);
                bodies.AddConstraint(bodies.HandleToLocation[bodyHandle].Index, constraintHandle, i);
            }
            return constraintHandle;
        }

        /// <summary>
        /// Allocates a constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandles">First body handle in a list of body handles used by the constraint.</param>
        /// <param name="bodyCount">Number of bodies used by the constraint.</param>
        /// <returns>Allocated constraint handle.</returns>
        public int Add<TDescription>(ref int bodyHandles, int bodyCount, TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            return Add(ref bodyHandles, bodyCount, ref description);
        }

        /// <summary>
        /// Allocates a one-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandle">First body of the pair.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe int Add<TDescription>(int bodyHandle, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            return Add(ref bodyHandle, 1, ref description);
        }

        /// <summary>
        /// Allocates a one-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandle">First body of the pair.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe int Add<TDescription>(int bodyHandle, TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            return Add(ref bodyHandle, 1, ref description);
        }

        /// <summary>
        /// Allocates a two-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandleA">First body of the pair.</param>
        /// <param name="bodyHandleB">Second body of the pair.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe int Add<TDescription>(int bodyHandleA, int bodyHandleB, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            //Don't really want to take a dependency on the stack layout of parameters, so...
            var bodyReferences = stackalloc int[2];
            bodyReferences[0] = bodyHandleA;
            bodyReferences[1] = bodyHandleB;
            return Add(ref bodyReferences[0], 2, ref description);
        }

        /// <summary>
        /// Allocates a two-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandleA">First body of the pair.</param>
        /// <param name="bodyHandleB">Second body of the pair.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe int Add<TDescription>(int bodyHandleA, int bodyHandleB, TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            return Add(bodyHandleA, bodyHandleB, ref description);
        }

        //This is split out for use by the multithreaded constraint remover.
        internal void RemoveBatchIfEmpty(ref ConstraintBatch batch, int batchIndex)
        {
            if (batch.TypeBatches.Count == 0)
            {
                //No more constraints exist within the batch; we may be able to get rid of this batch.
                //Merely having no constraints is insufficient. We would really rather not remove a batch if there are batches 'above' it:
                //the handle->constraint mapping involves a batch index. If we removed this batch, it would move every other batch down one step.
                //Which means, in order to retain correctness, we would have to change the batch index on every single constraint in every single batch 
                //of a higher index.

                //That's not feasible in the worst case.

                //Instead, we will only remove the batch if it is the *last* batch. We then rely on deferred batch compression to move constraints into lower
                //batches over time. Since it only handles a limited number of constraints at a time, it doesn't have the same risk of frame hitching.
                //So, even if a low-index batch gets zeroed out, constraints from higher batches will filter down, leaving the highest index constraint potentially empty instead.

                //This is a pretty safe thing to do. It is extremely difficult for a low index batch to end up empty while a higher index batch is still very full.
                //In fact, almost every instance where a batch goes empty will involve the highest index batch. If it isn't, it's going to be very high, and there won't be 
                //many constraints above it. Deferred compression will handle it easily.
                //Note the use of the cached batch index rather than the ref.
                ref var set = ref ActiveSet;
                if (batchIndex == set.Batches.Count - 1)
                {
                    //Note that when we remove an empty batch, it may reveal another empty batch. If that happens, remove the revealed batch(es) too.
                    while (set.Batches.Count > 0)
                    {
                        var lastBatchIndex = set.Batches.Count - 1;
                        ref var lastBatch = ref set.Batches[lastBatchIndex];
                        if (lastBatch.TypeBatches.Count == 0)
                        {
                            lastBatch.Dispose(bufferPool);
                            //The fallback batch has no batch referenced handles.
                            if (lastBatchIndex < FallbackBatchThreshold)
                            {
                                batchReferencedHandles[lastBatchIndex].Dispose(bufferPool);
                                --batchReferencedHandles.Count;
                            }
                            --set.Batches.Count;
                            Debug.Assert(set.Batches.Count == batchReferencedHandles.Count ||
                                (set.Batches.Count == FallbackBatchThreshold + 1 && batchReferencedHandles.Count == FallbackBatchThreshold),
                                "All synchronized batches should have a 1:1 mapping with batchReferencedHandles entries, but the fallback batch doesn't have one.");
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Removes a constraint from a batch, performing any necessary batch cleanup, but does not return the constraint's handle to the pool.
        /// </summary>
        /// <param name="constraintHandle">Handle of the constraint being removed.</param>
        /// <param name="batchIndex">Index of the batch to remove from.</param>
        /// <param name="typeId">Type id of the constraint to remove.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to remove within its type batch.</param>
        internal void RemoveFromBatch(int constraintHandle, int batchIndex, int typeId, int indexInTypeBatch)
        {
            ref var batch = ref ActiveSet.Batches[batchIndex];
            if (batchIndex == FallbackBatchThreshold)
            {
                //If this is the fallback batch, it does not track any referenced handles.
                //Note that we have to remove from fallback first because it accesses the batch's information.
                ActiveSet.Fallback.Remove(this, bufferPool, ref batch, constraintHandle, typeId, indexInTypeBatch);
                batch.Remove(typeId, indexInTypeBatch, this);
            }
            else
            {
                batch.RemoveWithHandles(typeId, indexInTypeBatch, ref batchReferencedHandles[batchIndex], this);
            }
            RemoveBatchIfEmpty(ref batch, batchIndex);
        }

        /// <summary>
        /// Removes the constraint associated with the given handle. Note that this may invalidate any outstanding direct constraint references
        /// by reordering the constraints within the TypeBatch subject to removal.
        /// </summary>
        /// <param name="handle">Handle of the constraint to remove from the solver.</param>
        public void Remove(int handle)
        {
            ref var constraintLocation = ref HandleToConstraint[handle];
            if (constraintLocation.SetIndex > 0)
            {
                //In order to remove a constraint, it must be active.
                awakener.AwakenConstraint(handle);
            }
            Debug.Assert(constraintLocation.SetIndex == 0);
            ConstraintGraphRemovalEnumerator enumerator;
            enumerator.bodies = bodies;
            enumerator.constraintHandle = handle;
            EnumerateConnectedBodies(handle, ref enumerator);

            pairCache.RemoveReferenceIfContactConstraint(handle, constraintLocation.TypeId);
            RemoveFromBatch(handle, constraintLocation.BatchIndex, constraintLocation.TypeId, constraintLocation.IndexInTypeBatch);
            HandlePool.Return(handle, bufferPool.SpecializeFor<int>());
        }

        public void GetDescription<TConstraintDescription, TTypeBatch>(ref ConstraintReference constraintReference, out TConstraintDescription description)
            where TConstraintDescription : IConstraintDescription<TConstraintDescription>
            where TTypeBatch : TypeProcessor
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            default(TConstraintDescription).BuildDescription(ref constraintReference.TypeBatch, bundleIndex, innerIndex, out description);
        }

        public void GetDescription<TConstraintDescription>(int handle, out TConstraintDescription description)
            where TConstraintDescription : IConstraintDescription<TConstraintDescription>
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            ref var location = ref HandleToConstraint[handle];
            var dummy = default(TConstraintDescription);
            ref var typeBatch = ref Sets[location.SetIndex].Batches[location.BatchIndex].GetTypeBatch(dummy.ConstraintTypeId);
            BundleIndexing.GetBundleIndices(location.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            dummy.BuildDescription(ref typeBatch, bundleIndex, innerIndex, out description);
        }


        private bool UpdateConstraintsForBodyMemoryMove(int originalIndex, int newIndex)
        {
            //Note that this function requires scanning the bodies in the constraint. This will tend to be fine since the vast majority of constraints have no more than 2 bodies.
            //While it's possible to store the index of the body in the constraint to avoid this scan, storing that information requires collecting that information on add.
            //That's not impossible by any means, but consider that this function will tend to be called in a deferred way- we have control over how many cache optimizations
            //we perform. We do not, however, have any control over how many adds must be performed. Those must be performed immediately for correctness.
            //In other words, doing a little more work here can reduce the overall work required, in addition to simplifying the storage requirements.
            ref var list = ref bodies.ActiveSet.Constraints[originalIndex];
            bool bodyShouldBePresentInFallback = false;
            for (int i = 0; i < list.Count; ++i)
            {
                ref var constraint = ref list[i];
                ref var constraintLocation = ref HandleToConstraint[constraint.ConnectingConstraintHandle];
                //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
                //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
                //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
                TypeProcessors[constraintLocation.TypeId].UpdateForBodyMemoryMove(
                    ref ActiveSet.Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId),
                    constraintLocation.IndexInTypeBatch, constraint.BodyIndexInConstraint, newIndex);
                if (constraintLocation.BatchIndex == FallbackBatchThreshold)
                    bodyShouldBePresentInFallback = true;
            }
            return bodyShouldBePresentInFallback;
        }
        /// <summary>
        /// Changes the body references of all constraints associated with a body in response to its movement into a new slot.
        /// Constraints associated with the body now at its old slot, if any, are left untouched.
        /// </summary>
        /// <param name="originalBodyIndex">Memory index that the moved body used to inhabit.</param>
        /// <param name="newBodyLocation">Memory index that the moved body now inhabits.</param>
        internal void UpdateForBodyMemoryMove(int originalBodyIndex, int newBodyLocation)
        {
            if (UpdateConstraintsForBodyMemoryMove(originalBodyIndex, newBodyLocation))
            {
                //One of the moved constraints involved the fallback batch, so we need to update the fallback batch's body indices.
                ActiveSet.Fallback.UpdateForBodyMemoryMove(originalBodyIndex, newBodyLocation);
            }
        }

        /// <summary>
        /// Changes the body references of all constraints associated with two bodies in response to them swapping slots in memory.
        /// </summary>
        /// <param name="a">First swapped body index.</param>
        /// <param name="b">Second swapped body index.</param>
        internal void UpdateForBodyMemorySwap(int a, int b)
        {
            var aInFallback = UpdateConstraintsForBodyMemoryMove(a, b);
            var bInFallback = UpdateConstraintsForBodyMemoryMove(b, a);
            if (aInFallback && bInFallback)
            {
                ActiveSet.Fallback.UpdateForBodyMemorySwap(a, b);
            }
            else if (aInFallback)
            {
                ActiveSet.Fallback.UpdateForBodyMemoryMove(a, b);
            }
            else if (bInFallback)
            {
                ActiveSet.Fallback.UpdateForBodyMemoryMove(b, a);
            }
        }




        /// <summary>
        /// Enumerates the set of bodies associated with a constraint in order of their references within the constraint.
        /// </summary>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to use.</param>
        internal void EnumerateConnectedBodies<TEnumerator>(int constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle];
            ref var typeBatch = ref Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            Debug.Assert(constraintLocation.IndexInTypeBatch >= 0 && constraintLocation.IndexInTypeBatch < typeBatch.ConstraintCount, "Bad constraint location; likely some add/remove bug.");
            TypeProcessors[constraintLocation.TypeId].EnumerateConnectedBodyIndices(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
        }

        internal void GetSynchronizedBatchCount(out int synchronizedBatchCount, out bool fallbackExists)
        {
            synchronizedBatchCount = Math.Min(ActiveSet.Batches.Count, FallbackBatchThreshold);
            fallbackExists = ActiveSet.Batches.Count > FallbackBatchThreshold;
            Debug.Assert(ActiveSet.Batches.Count <= FallbackBatchThreshold + 1,
                "There cannot be more than FallbackBatchThreshold + 1 constraint batches because that +1 is the fallback batch which contains all remaining constraints.");
        }

        public void Update(float dt)
        {
            var inverseDt = 1f / dt;
            ref var activeSet = ref ActiveSet;
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            for (int i = 0; i < synchronizedBatchCount; ++i)
            {
                ref var batch = ref activeSet.Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    ref var typeBatch = ref batch.TypeBatches[j];
                    TypeProcessors[typeBatch.TypeId].Prestep(ref typeBatch, bodies, dt, inverseDt, 0, typeBatch.BundleCount);
                }
            }
            if (fallbackExists)
            {
                ref var batch = ref activeSet.Batches[FallbackBatchThreshold];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    ref var typeBatch = ref batch.TypeBatches[j];
                    TypeProcessors[typeBatch.TypeId].JacobiPrestep(ref typeBatch, bodies, ref activeSet.Fallback, dt, inverseDt, 0, typeBatch.BundleCount);
                }
            }
            //TODO: May want to consider executing warmstart immediately following the prestep. Multithreading can't do that, so there could be some bitwise differences introduced.
            //On the upside, it would make use of cached data.
            for (int i = 0; i < synchronizedBatchCount; ++i)
            {
                ref var batch = ref activeSet.Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    ref var typeBatch = ref batch.TypeBatches[j];
                    TypeProcessors[typeBatch.TypeId].WarmStart(ref typeBatch, ref bodies.ActiveSet.Velocities, 0, typeBatch.BundleCount);
                }
            }
            Buffer<FallbackTypeBatchResults> fallbackResults = default;
            if (fallbackExists)
            {
                ref var batch = ref activeSet.Batches[FallbackBatchThreshold];
                FallbackBatch.AllocateResults(this, bufferPool, ref batch, out fallbackResults);
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    ref var typeBatch = ref batch.TypeBatches[j];
                    TypeProcessors[typeBatch.TypeId].JacobiWarmStart(ref typeBatch, ref bodies.ActiveSet.Velocities, ref fallbackResults[j], 0, typeBatch.BundleCount);
                }
                activeSet.Fallback.ScatterVelocities(bodies, this, ref fallbackResults, 0, activeSet.Fallback.BodyCount);
            }
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < synchronizedBatchCount; ++i)
                {
                    ref var batch = ref activeSet.Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].SolveIteration(ref typeBatch, ref bodies.ActiveSet.Velocities, 0, typeBatch.BundleCount);
                    }
                }
                if (fallbackExists)
                {
                    ref var batch = ref activeSet.Batches[FallbackBatchThreshold];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].JacobiSolveIteration(ref typeBatch, ref bodies.ActiveSet.Velocities, ref fallbackResults[j], 0, typeBatch.BundleCount);
                    }
                    activeSet.Fallback.ScatterVelocities(bodies, this, ref fallbackResults, 0, activeSet.Fallback.BodyCount);
                }
            }
            if (fallbackExists)
            {
                FallbackBatch.DisposeResults(this, bufferPool, ref activeSet.Batches[FallbackBatchThreshold], ref fallbackResults);
            }
        }

        //Note that none of these affect the constraint batch estimates or type batch estimates. The assumption is that those are too small to bother with.
        //In the worst case you might see a couple of kilobytes. The reason why these functions exist is to deal with the potential many *megabytes* worth of constraint and body buffers.
        //Maybe something weird happens where this assumption is invalidated later, but I doubt it. There is a cost in API complexity to support it, so we don't.

        //TODO: given the nuance involved in using Clear here, there may be an argument for making it internal only. Then again, we've used that reasoning before only to result in
        //people having to modify the source.
        /// <summary>
        /// Removes all objects from the solver. This is meant as a fast path to empty a simulation's constraints. It makes no attempt to retain synchronization with other systems
        /// which may depend on the existence of constraints, like the per-body constraint lists.
        /// </summary>
        public void Clear()
        {
            ref var activeSet = ref ActiveSet;
            //Fallback batches don't have any batch referenced handles.
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out _);
            for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
            {
                batchReferencedHandles[batchIndex].Dispose(bufferPool);
            }
            batchReferencedHandles.Clear();
            ActiveSet.Clear(bufferPool);
            //All inactive sets are returned to the pool.
            //Their allocations are always created to fit the actual island size.
            for (int i = 1; i < Sets.Length; ++i)
            {
                if (Sets[i].Allocated)
                {
                    Sets[i].Dispose(bufferPool);
                }
            }
            HandlePool.Clear();
        }

        /// <summary>
        /// Adjusts the size of the the solvers non-typebatch data structures. An allocation will grow if the given capacity exceeds the currently allocated capacity.
        /// </summary>
        /// <param name="bodyHandleCapacity">Size of the span of body handles to allocate space for. Applies to batch referenced handle sets.</param>
        /// <param name="constraintHandleCapacity">Number of constraint handles to allocate space for. Applies to the handle->constraint mapping table.</param>
        public void EnsureSolverCapacities(int bodyHandleCapacity, int constraintHandleCapacity)
        {
            if (HandleToConstraint.Length < constraintHandleCapacity)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, constraintHandleCapacity, HandlePool.HighestPossiblyClaimedId + 1);
            }
            //Note that we can't shrink below the bodies handle capacity, since the handle distribution could be arbitrary.
            var targetBatchReferencedHandleSize = Math.Max(bodies.HandlePool.HighestPossiblyClaimedId + 1, bodyHandleCapacity);
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            //The fallback batch does not have any referenced handles.
            for (int i = 0; i < synchronizedBatchCount; ++i)
            {
                batchReferencedHandles[i].EnsureCapacity(targetBatchReferencedHandleSize, bufferPool);
            }
        }

        /// <summary>
        /// Adjusts the size of the the solvers non-typebatch data structures. An allocation is allowed to shrink if it fits both all existing entries and the given capacity.
        /// An allocation will grow if the given capacity exceeds the currently allocated capacity.
        /// </summary>
        /// <param name="bodyHandleCapacity">Size of the span of body handles to allocate space for. Applies to batch referenced handle sets.</param>
        /// <param name="constraintHandleCapacity">Number of constraint handles to allocate space for. Applies to the handle->constraint mapping table.</param>
        public void ResizeSolverCapacities(int bodyHandleCapacity, int constraintHandleCapacity)
        {
            var targetConstraintCount = BufferPool<ConstraintLocation>.GetLowestContainingElementCount(Math.Max(constraintHandleCapacity, HandlePool.HighestPossiblyClaimedId + 1));
            if (HandleToConstraint.Length != targetConstraintCount)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, targetConstraintCount, HandlePool.HighestPossiblyClaimedId + 1);
            }
            //Note that we can't shrink below the bodies handle capacity, since the handle distribution could be arbitrary.
            var targetBatchReferencedHandleSize = Math.Max(bodies.HandlePool.HighestPossiblyClaimedId + 1, bodyHandleCapacity);
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            //The fallback batch does not have any referenced handles.
            for (int i = 0; i < synchronizedBatchCount; ++i)
            {
                batchReferencedHandles[i].Resize(targetBatchReferencedHandleSize, bufferPool);
            }
        }

        internal void ResizeSetsCapacity(int setsCapacity, int potentiallyAllocatedCount)
        {
            Debug.Assert(setsCapacity >= potentiallyAllocatedCount && potentiallyAllocatedCount <= Sets.Length);
            setsCapacity = BufferPool<ConstraintSet>.GetLowestContainingElementCount(setsCapacity);
            if (Sets.Length != setsCapacity)
            {
                var oldCapacity = Sets.Length;
                bufferPool.SpecializeFor<ConstraintSet>().Resize(ref Sets, setsCapacity, potentiallyAllocatedCount);
                if (oldCapacity < Sets.Length)
                    Sets.Clear(oldCapacity, Sets.Length - oldCapacity); //We rely on unused slots being default initialized.
            }
        }

        //Note that the following do not force resizes on inactive sets. Inactive sets are allocated with just enough space to cover what is in the island.
        //Since inactive islands never undergo incremental adds or removes, there is never any point to resizing their allocations.
        /// <summary>
        /// Ensures all existing active type batches meet or exceed the current solver-defined minimum capacities. Type batches with capacities smaller than the minimums will be enlarged.
        /// </summary>
        public void EnsureTypeBatchCapacities()
        {
            ref var activeSet = ref ActiveSet;
            for (int i = 0; i < activeSet.Batches.Count; ++i)
            {
                activeSet.Batches[i].EnsureTypeBatchCapacities(this);
            }
        }
        /// <summary>
        /// Applies the current solver-defined minimum capacities to existing type batches. Type batches with capacities larger than the minimums and counts less than the minimums may be shrunk.
        /// Type batches with capacities smaller than the minimums will be enlarged.
        /// </summary>
        public void ResizeTypeBatchCapacities()
        {
            ref var activeSet = ref ActiveSet;
            for (int i = 0; i < activeSet.Batches.Count; ++i)
            {
                activeSet.Batches[i].ResizeTypeBatchCapacities(this);
            }
        }

        /// <summary>
        /// Returns all pool-retrieved resources to the pool.
        /// </summary>
        /// <remarks>
        /// The solver cannot be 'rehydrated' for reuse after a disposal. If you want to return bulk data to the pool while leaving the solver in a usable state, consider using Clear instead.
        /// </remarks>
        public void Dispose()
        {
            //Note that the fallback batch does not have a batch referenced handle.
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out _);
            for (int i = 0; i < synchronizedBatchCount; ++i)
            {
                batchReferencedHandles[i].Dispose(bufferPool);
            }
            batchReferencedHandles.Dispose(bufferPool.SpecializeFor<IndexSet>());
            for (int i = 0; i < Sets.Length; ++i)
            {
                if (Sets[i].Allocated)
                    Sets[i].Dispose(bufferPool);
            }
            bufferPool.SpecializeFor<ConstraintSet>().Return(ref Sets);
            bufferPool.SpecializeFor<ConstraintLocation>().Return(ref HandleToConstraint);
            HandlePool.Dispose(bufferPool.SpecializeFor<int>());
        }


    }
}
