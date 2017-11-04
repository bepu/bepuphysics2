using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    public unsafe struct ConstraintReference
    {
        //TODO: Once blittable exists, we can give this a proper type. Blocked by generics interference in TypeBatch.
        //May want to just treat this as opaque.
        internal void* typeBatchPointer;
        public ref TypeBatchData TypeBatch
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Unsafe.AsRef<TypeBatchData>(typeBatchPointer);
            }
        }
        public readonly int IndexInTypeBatch;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ConstraintReference(ref TypeBatchData typeBatch, int indexInTypeBatch)
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
        public int BatchIndex;
        public int TypeId;
        public int IndexInTypeBatch;
    }

    public partial class Solver
    {

        public QuickList<ConstraintBatch, Buffer<ConstraintBatch>> Batches;
        internal QuickList<BatchReferencedHandles, Buffer<BatchReferencedHandles>> batchReferencedHandles;

        public TypeProcessor[] TypeProcessors;

        internal Bodies bodies;

        /// <summary>
        /// Pool to retrieve constraint handles from when creating new constraints.
        /// </summary>
        public IdPool<Buffer<int>> HandlePool;
        internal BufferPool bufferPool;
        public Buffer<ConstraintLocation> HandleToConstraint;


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
        /// Gets the total number of constraints across all types and batches.
        /// </summary>
        public int ConstraintCount
        {
            get
            {
                int count = 0;
                for (int i = 0; i < Batches.Count; ++i)
                {
                    ref var batch = ref Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        count += batch.TypeBatches[j].ConstraintCount;
                    }
                }
                return count;
            }
        }
        /// <summary>
        /// Gets the total number of bundles across all types and batches.
        /// </summary>
        public int BundleCount
        {
            get
            {
                int count = 0;
                for (int i = 0; i < Batches.Count; ++i)
                {
                    ref var batch = ref Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        count += batch.TypeBatches[j].BundleCount;
                    }
                }
                return count;
            }
        }

        Action<int> workDelegate;
        //TODO: While 32 batches will likely cover most simulations, this will likely change when the jacobi-hybrid fallback is implemented in favor of a user configurable threshold.
        const int BatchCountEstimate = 32;
        public Solver(Bodies bodies, BufferPool bufferPool, int iterationCount = 5,
            int initialCapacity = 1024,
            int minimumCapacityPerTypeBatch = 64)
        {
            this.iterationCount = iterationCount;
            this.minimumCapacityPerTypeBatch = minimumCapacityPerTypeBatch;
            this.bodies = bodies;
            this.bufferPool = bufferPool;
            IdPool<Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), 128, out HandlePool);
            //Note that managed arrays must be used to hold the reference types. It's technically possible to bypass this by completely abandoning inheritance in the typebatches, but
            //that would make a variety of things more annoying to handle. We can make use of just a tiny amount of idiomatic C#-ness. This won't be many references anyway.
            //We also don't bother pooling this stuff, and we don't have an API for preallocating it- because we're talking about a very, very small amount of data.
            //It's not worth the introduced API complexity.
            QuickList<ConstraintBatch, Buffer<ConstraintBatch>>.Create(bufferPool.SpecializeFor<ConstraintBatch>(), BatchCountEstimate, out Batches);
            QuickList<BatchReferencedHandles, Buffer<BatchReferencedHandles>>.Create(bufferPool.SpecializeFor<BatchReferencedHandles>(), BatchCountEstimate, out batchReferencedHandles);
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
                throw new ArgumentException($"Type processor {TypeProcessors[description.ConstraintTypeId].GetType().Name} has already been registered for this description's type id. " +
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
            reference = new ConstraintReference(ref Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId), constraintLocation.IndexInTypeBatch);
        }

        [Conditional("DEBUG")]
        private void ValidateBodyIndex(int bodyIndex, int expectedCount, ref ConstraintBatch batch)
        {
            int referencesToBody = 0;
            for (int i = 0; i < batch.TypeBatches.Count; ++i)
            {
                var instancesInTypeBatch = TypeProcessors[batch.TypeBatches[i].TypeId].GetBodyIndexInstanceCount(ref batch.TypeBatches[i], bodyIndex);
                Debug.Assert(instancesInTypeBatch + referencesToBody <= expectedCount,
                    "Found an instance of a body index that wasn't expected. Possible upstream bug or memory corruption.");
                referencesToBody += instancesInTypeBatch;
            }
            Debug.Assert(referencesToBody == expectedCount);
        }

        [Conditional("DEBUG")]
        internal void ValidateExistingHandles()
        {
            for (int i = 0; i < bodies.Count; ++i)
            {
                bodies.ValidateExistingHandle(bodies.IndexToHandle[i]);
            }
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                ref var handles = ref batchReferencedHandles[batchIndex];
                ref var batch = ref Batches[batchIndex];
                for (int i = 0; i < bodies.Count; ++i)
                {
                    var handle = bodies.IndexToHandle[i];
                    if (handles.Contains(handle))
                        ValidateBodyIndex(i, 1, ref batch);
                    else
                        ValidateBodyIndex(i, 0, ref batch);
                }
            }
        }

        [Conditional("DEBUG")]
        internal void ValidateNewHandles(int batchIndex, ref int bodyHandles, int bodyCount, Bodies bodies)
        {
            Debug.Assert(batchReferencedHandles[batchIndex].CanFit(ref bodyHandles, bodyCount));
            for (int i = 0; i < bodyCount; ++i)
            {
                ValidateBodyIndex(bodies.HandleToIndex[Unsafe.Add(ref bodyHandles, i)], 0, ref Batches[batchIndex]);
            }
        }

        [Conditional("DEBUG")]
        internal void ValidateConstraintMaps()
        {
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                ref var handles = ref batchReferencedHandles[batchIndex];
                ref var batch = ref Batches[batchIndex];
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

        /// <summary>
        /// Attempts to locate a spot for a new constraint. Does not perform allocation for the constraint. If no batch exists, returns the index just beyond the end of the existing list of batches.
        /// </summary>
        /// <param name="batchStartIndex">Index at which to start the search.</param>
        /// <returns>Index of the batch that the constraint would fit in.</returns>
        /// <remarks>This is used by the narrowphase's multithreaded constraint adders to locate a spot for a new constraint without requiring a lock. Only after a candidate is located
        /// do those systems attempt an actual claim, limiting the duration of locks and increasing potential parallelism.</remarks>
        internal unsafe int FindCandidateBatch(int batchStartIndex, ref int bodyHandles, int bodyCount)
        {
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                if (batchReferencedHandles[batchIndex].CanFit(ref bodyHandles, bodyCount))
                    return batchIndex;
            }
            return Batches.Count;
        }

        internal unsafe bool TryAllocateInBatch(int typeId, int targetBatchIndex, ref int bodyHandles, int bodyCount, out int constraintHandle, out ConstraintReference reference)
        {
            Debug.Assert(targetBatchIndex <= Batches.Count,
                "It should be impossible for a target batch to be generated which is more than one slot beyond the end of the batch list. Possible misuse of FindCandidateBatch.");
            if (targetBatchIndex == Batches.Count)
            {
                //No batch available. Have to create a new one.
                if (Batches.Count == Batches.Span.Length)
                    Batches.Resize(Batches.Count + 1, bufferPool.SpecializeFor<ConstraintBatch>());
                if (Batches.Count == batchReferencedHandles.Span.Length)
                    batchReferencedHandles.Resize(Batches.Count + 1, bufferPool.SpecializeFor<BatchReferencedHandles>());
                Batches.AllocateUnsafely() = new ConstraintBatch(bufferPool, TypeProcessors.Length);
                batchReferencedHandles.AllocateUnsafely() = new BatchReferencedHandles(bufferPool, bodies.Count);
                //Note that if there is no constraint batch for the given index, there is no way for the constraint add to be blocked. It's guaranteed success.
            }
            else
            {
                //A constraint batch already exists here. This may fail.
                if (!batchReferencedHandles[targetBatchIndex].CanFit(ref bodyHandles, bodyCount))
                {
                    //This batch cannot hold the constraint.
                    constraintHandle = -1;
                    reference = default(ConstraintReference);
                    return false;
                }
            }
            constraintHandle = HandlePool.Take();
            Batches[targetBatchIndex].Allocate(constraintHandle, ref bodyHandles, bodyCount, ref batchReferencedHandles[targetBatchIndex],
                bodies, typeId, TypeProcessors[typeId], GetMinimumCapacityForType(typeId), bufferPool, out reference);

            if (constraintHandle >= HandleToConstraint.Length)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, HandleToConstraint.Length * 2, HandleToConstraint.Length);
                Debug.Assert(constraintHandle < HandleToConstraint.Length, "Handle indices should never jump by more than 1 slot, so doubling should always be sufficient.");
            }
            ref var constraintLocation = ref HandleToConstraint[constraintHandle];
            constraintLocation.IndexInTypeBatch = reference.IndexInTypeBatch;
            constraintLocation.TypeId = typeId;
            constraintLocation.BatchIndex = targetBatchIndex;
            return true;
        }


        /// <summary>
        /// Applies a description to a constraint slot.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintReference">Reference of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescription<TDescription>(ref ConstraintReference constraintReference, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            description.ApplyDescription(ref constraintReference.TypeBatch, bundleIndex, innerIndex);
        }


        /// <summary>
        /// Applies a description to a constraint slot.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintReference">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescription<TDescription>(int constraintHandle, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            GetConstraintReference(constraintHandle, out var constraintReference);
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            description.ApplyDescription(ref constraintReference.TypeBatch, bundleIndex, innerIndex);
        }


        /// <summary>
        /// Allocates a constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <typeparam name="TTypeBatch">Type of the TypeBatch to allocate in.</typeparam>
        /// <param name="bodyHandles">Reference to the start of a list of body handles.</param>
        /// <param name="bodyCount">Number of bodies in the body handles list.</param>
        /// <param name="constraintReference">Reference to the allocated slot.</param>
        /// <param name="handle">Allocated constraint handle.</param>
        public void Add<TDescription>(ref int bodyHandles, int bodyCount, ref TDescription description, out int handle)
            where TDescription : IConstraintDescription<TDescription>
        {
            for (int i = 0; i <= Batches.Count; ++i)
            {
                if (TryAllocateInBatch(description.ConstraintTypeId, i, ref bodyHandles, bodyCount, out handle, out var reference))
                {
                    ApplyDescription(ref reference, ref description);
                    return;
                }
            }
            handle = -1;
            Debug.Fail("The above allocation loop checks every batch and also one index beyond all existing batches. It should be guaranteed to succeed.");
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
                if (batchIndex == Batches.Count - 1)
                {
                    //Note that when we remove an empty batch, it may reveal another empty batch. If that happens, remove the revealed batch(es) too.
                    while (Batches.Count > 0)
                    {
                        var lastBatchIndex = Batches.Count - 1;
                        ref var lastBatch = ref Batches[lastBatchIndex];
                        if (lastBatch.TypeBatches.Count == 0)
                        {
                            lastBatch.Dispose(bufferPool);
                            batchReferencedHandles[lastBatchIndex].Dispose(bufferPool);
                            --batchReferencedHandles.Count; 
                            --Batches.Count;
                            Debug.Assert(Batches.Count == batchReferencedHandles.Count);
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
        /// <param name="batchIndex">Index of the batch to remove from.</param>
        /// <param name="typeId">Type id of the constraint to remove.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to remove within its type batch.</param>
        internal void RemoveFromBatch(int batchIndex, int typeId, int indexInTypeBatch)
        {
            ref var batch = ref Batches[batchIndex];
            batch.RemoveWithHandles(typeId, indexInTypeBatch, ref batchReferencedHandles[batchIndex], this);
            RemoveBatchIfEmpty(ref batch, batchIndex);
        }
        /// <summary>
        /// Removes the constraint associated with the given handle. Note that this may invalidate any outstanding direct constraint references (TypeBatch-index pairs)
        /// by reordering the constraints within the TypeBatch subject to removal.
        /// </summary>
        /// <param name="handle">Handle of the constraint to remove from the solver.</param>
        public void Remove(int handle)
        {
            //Note that we don't use a ref var here. Have to be careful; we make use of the constraint location after removal. Direct ref would be invalidated.
            //(Could cache the batch index, but that's splitting some very fine hairs.)
            ref var constraintLocation = ref HandleToConstraint[handle];
            RemoveFromBatch(constraintLocation.BatchIndex, constraintLocation.TypeId, constraintLocation.IndexInTypeBatch);
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
            ref var typeBatch = ref Batches[location.BatchIndex].GetTypeBatch(dummy.ConstraintTypeId);
            BundleIndexing.GetBundleIndices(location.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            dummy.BuildDescription(ref typeBatch, bundleIndex, innerIndex, out description);
        }



        /// <summary>
        /// Changes the body reference of a constraint in response to a body memory move.
        /// </summary>
        /// <param name="constraintHandle">Handle of the constraint to modify.</param> 
        /// <param name="bodyIndexInConstraint">Index of the moved body in the constraint.</param>
        /// <param name="newBodyLocation">Memory index that the moved body now inhabits.</param>
        public void UpdateForBodyMemoryMove(int constraintHandle, int bodyIndexInConstraint, int newBodyLocation)
        {
            //Note that this function requires scanning the bodies in the constraint. This will tend to be fine since the vast majority of constraints have no more than 2 bodies.
            //While it's possible to store the index of the body in the constraint to avoid this scan, storing that information requires collecting that information on add.
            //That's not impossible by any means, but consider that this function will tend to be called in a deferred way- we have control over how many cache optimizations
            //we perform. We do not, however, have any control over how many adds must be performed. Those must be performed immediately for correctness.
            //In other words, doing a little more work here can reduce the overall work required, in addition to simplifying the storage requirements.
            ref var constraintLocation = ref HandleToConstraint[constraintHandle];
            //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
            //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
            //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
            TypeProcessors[constraintLocation.TypeId].UpdateForBodyMemoryMove(
                ref Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId),
                constraintLocation.IndexInTypeBatch, bodyIndexInConstraint, newBodyLocation);
        }

        /// <summary>
        /// Enumerates the set of body indices associated with a constraint in order of their references within the constraint.
        /// </summary>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to use.</param>
        internal void EnumerateConnectedBodyIndices<TEnumerator>(int constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle];
            //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
            //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
            //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
            ref var typeBatch = ref Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            Debug.Assert(constraintLocation.IndexInTypeBatch >= 0 && constraintLocation.IndexInTypeBatch < typeBatch.ConstraintCount, "Bad constraint location; likely some add/remove bug.");
            TypeProcessors[constraintLocation.TypeId].EnumerateConnectedBodyIndices(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
        }


        public void Update(float dt)
        {
            var inverseDt = 1f / dt;
            for (int i = 0; i < Batches.Count; ++i)
            {
                ref var batch = ref Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    ref var typeBatch = ref batch.TypeBatches[j];
                    TypeProcessors[typeBatch.TypeId].Prestep(ref typeBatch, bodies, dt, inverseDt);
                }
            }
            //TODO: May want to consider executing warmstart immediately following the prestep. Multithreading can't do that, so there could be some bitwise differences introduced.
            //On the upside, it would make use of cached data.
            for (int i = 0; i < Batches.Count; ++i)
            {
                ref var batch = ref Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    ref var typeBatch = ref batch.TypeBatches[j];
                    TypeProcessors[typeBatch.TypeId].WarmStart(ref typeBatch, ref bodies.Velocities);
                }
            }
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < Batches.Count; ++i)
                {
                    ref var batch = ref Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        TypeProcessors[typeBatch.TypeId].SolveIteration(ref typeBatch, ref bodies.Velocities);
                    }
                }
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
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                batchReferencedHandles[batchIndex].Dispose(bufferPool);
                Batches[batchIndex].Dispose(bufferPool);
            }
            batchReferencedHandles.Clear();
            Batches.Clear();
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
            for (int i = 0; i < Batches.Count; ++i)
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
            for (int i = 0; i < Batches.Count; ++i)
            {
                batchReferencedHandles[i].Resize(targetBatchReferencedHandleSize, bufferPool);
            }
        }

        /// <summary>
        /// Ensures all existing type batches meet or exceed the current solver-defined minimum capacities. Type batches with capacities smaller than the minimums will be enlarged.
        /// </summary>
        public void EnsureTypeBatchCapacities()
        {
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].EnsureTypeBatchCapacities(this);
            }
        }
        /// <summary>
        /// Applies the current solver-defined minimum capacities to existing type batches. Type batches with capacities larger than the minimums and counts less than the minimums may be shrunk.
        /// Type batches with capacities smaller than the minimums will be enlarged.
        /// </summary>
        public void ResizeTypeBatchCapacities()
        {
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].ResizeTypeBatchCapacities(this);
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
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Dispose(bufferPool);
                batchReferencedHandles[i].Dispose(bufferPool);
            }
            Batches.Dispose(bufferPool.SpecializeFor<ConstraintBatch>());
            Batches = new QuickList<ConstraintBatch, Buffer<ConstraintBatch>>();
            batchReferencedHandles.Dispose(bufferPool.SpecializeFor<BatchReferencedHandles>());
            batchReferencedHandles = new QuickList<BatchReferencedHandles, Buffer<BatchReferencedHandles>>();
            bufferPool.SpecializeFor<ConstraintLocation>().Return(ref HandleToConstraint);
            HandleToConstraint = new Buffer<ConstraintLocation>();
            HandlePool.Dispose(bufferPool.SpecializeFor<int>());
        }


    }
}
