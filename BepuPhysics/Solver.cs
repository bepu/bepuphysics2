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
        public void* TypeBatch;
        public int IndexInTypeBatch;
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

        internal IdPool<Buffer<int>> handlePool;
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

        int minimumCapacity;
        /// <summary>
        /// Gets or sets the minimum amount of space, in constraints, initially allocated in any new type batch.
        /// </summary>
        public int MinimumCapacityPerTypeBatch
        {
            get { return minimumCapacity; }
            set
            {
                if (value <= 0)
                    throw new ArgumentException("Minimum capacity must be positive.");
                minimumCapacity = value;
            }
        }
        int[] minimumInitialCapacityPerTypeBatch;

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
            if (typeId > minimumInitialCapacityPerTypeBatch.Length)
                return minimumCapacity;
            return Math.Max(minimumInitialCapacityPerTypeBatch[typeId], minimumCapacity);
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
                    var batch = Batches[i];
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
                    var batch = Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        count += batch.TypeBatches[j].BundleCount;
                    }
                }
                return count;
            }
        }

        Action<int> workDelegate;
        const int TypeCountEstimate = 32;
        const int BatchCountEstimate = 32;
        public Solver(Bodies bodies, BufferPool bufferPool, int iterationCount = 5,
            int initialCapacity = 1024,
            int minimumCapacityPerTypeBatch = 64)
        {
            this.iterationCount = iterationCount;
            this.bodies = bodies;
            this.bufferPool = bufferPool;
            IdPool<Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), 128, out handlePool);
            //Note that managed arrays must be used to hold the reference types. It's technically possible to bypass this by completely abandoning inheritance in the typebatches, but
            //that would make a variety of things more annoying to handle. We can make use of just a tiny amount of idiomatic C#-ness. This won't be many references anyway.
            //We also don't bother pooling this stuff, and we don't have an API for preallocating it- because we're talking about a very, very small amount of data.
            //It's not worth the introduced API complexity.
            QuickList<ConstraintBatch, Buffer<ConstraintBatch>>.Create(bufferPool.SpecializeFor<ConstraintBatch>(), BatchCountEstimate, out Batches);
            QuickList<BatchReferencedHandles, Buffer<BatchReferencedHandles>>.Create(bufferPool.SpecializeFor<BatchReferencedHandles>(), BatchCountEstimate, out batchReferencedHandles);
            bufferPool.SpecializeFor<ConstraintLocation>().Take(initialCapacity, out HandleToConstraint);
            workDelegate = Work;
            TypeProcessors = new TypeProcessor[TypeCountEstimate];
        }

        public void Register<TDescription>() where TDescription : struct, IConstraintDescription<TDescription>
        {
            var description = default(TDescription);
            Debug.Assert(description.ConstraintTypeId >= 0, "Constraint type ids should never be negative. They're used for array indexing.");
            if (description.ConstraintTypeId >= TypeProcessors.Length)
            {
                //This may result in some unnecessary resizes, but it hardly matters. It only happens once on registration time.
                Array.Resize(ref TypeProcessors, description.ConstraintTypeId + 1);
            }
            if (TypeProcessors[description.ConstraintTypeId] != null)
            {
                throw new ArgumentException($"Type processor {TypeProcessors[description.ConstraintTypeId].GetType().Name} has already been registered for this description's type id. " +
                    $"Cannot register the same type id more than once.");
            }
            TypeProcessors[description.ConstraintTypeId] = (TypeProcessor)Activator.CreateInstance(description.BatchType);
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
        public ref TypeBatchData GetConstraintReference(int handle, out int indexInTypeBatch)
        {
            ref var constraintLocation = ref HandleToConstraint[handle];
            indexInTypeBatch = constraintLocation.IndexInTypeBatch;
            return ref Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
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
                Batches.AllocateUnsafely() = new ConstraintBatch(bufferPool, bodies.Count, TypeCountEstimate);
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
            constraintHandle = handlePool.Take();
            ref var targetBatch = ref Batches[targetBatchIndex];
            targetBatch.Allocate(constraintHandle, ref bodyHandles, bodyCount, ref batchReferencedHandles[targetBatchIndex], 
                bodies, typeId, TypeProcessors[typeId], GetMinimumCapacityForType(typeId), bufferPool, out reference.IndexInTypeBatch);

            if (constraintHandle >= HandleToConstraint.Length)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, HandleToConstraint.Length * 2, HandleToConstraint.Length);
                Debug.Assert(constraintHandle < HandleToConstraint.Length, "Handle indices should never jump by more than 1 slot, so doubling should always be sufficient.");
            }
            HandleToConstraint[constraintHandle].IndexInTypeBatch = reference.IndexInTypeBatch;
            HandleToConstraint[constraintHandle].TypeId = typeId;
            HandleToConstraint[constraintHandle].BatchIndex = targetBatchIndex;
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
            description.ApplyDescription(constraintReference.TypeBatch, bundleIndex, innerIndex);
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
            description.ApplyDescription(constraintReference.TypeBatch, bundleIndex, innerIndex);
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
        internal void RemoveBatchIfEmpty(ConstraintBatch batch, int batchIndex)
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
                    while (Batches.Count > 0 && Batches[Batches.Count - 1].TypeBatches.Count == 0)
                    {
                        //Note that we do not actually null out the batch slot. It's still there. The backing array of the Batches list acts as a pool. When a new batch is required,
                        //the add function first checks the backing array to see if a batch was already allocated for it. In effect, adding and removing batches behaves like a stack.
                        --Batches.Count;
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
            var batch = Batches[batchIndex];
            batch.Remove(typeId, indexInTypeBatch, bodies, ref HandleToConstraint, TypeBatchCapacities);
            RemoveBatchIfEmpty(batch, batchIndex);
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
            var constraintLocation = HandleToConstraint[handle];
            RemoveFromBatch(constraintLocation.BatchIndex, constraintLocation.TypeId, constraintLocation.IndexInTypeBatch);
            handlePool.Return(handle, bufferPool.SpecializeFor<int>());
        }

        public void GetDescription<TConstraintDescription, TTypeBatch>(ref ConstraintReference constraintReference, out TConstraintDescription description)
            where TConstraintDescription : IConstraintDescription<TConstraintDescription>
            where TTypeBatch : TypeProcessor
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            default(TConstraintDescription).BuildDescription(constraintReference.TypeBatch, bundleIndex, innerIndex, out description);

        }

        public void GetDescription<TConstraintDescription>(int handle, out TConstraintDescription description)
            where TConstraintDescription : IConstraintDescription<TConstraintDescription>
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            ref var location = ref HandleToConstraint[handle];
            var dummy = default(TConstraintDescription);
            var typeBatch = Batches[location.BatchIndex].GetTypeBatch(dummy.ConstraintTypeId);
            BundleIndexing.GetBundleIndices(location.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            dummy.BuildDescription(typeBatch, bundleIndex, innerIndex, out description);

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
            Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId).UpdateForBodyMemoryMove(constraintLocation.IndexInTypeBatch, bodyIndexInConstraint, newBodyLocation);
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
            Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId).EnumerateConnectedBodyIndices(constraintLocation.IndexInTypeBatch, ref enumerator);
        }


        public void Update(float dt)
        {
            var inverseDt = 1f / dt;
            for (int i = 0; i < Batches.Count; ++i)
            {
                var batch = Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches[j].Prestep(bodies, dt, inverseDt);
                }
            }
            //TODO: May want to consider executing warmstart immediately following the prestep. Multithreading can't do that, so there could be some bitwise differences introduced.
            //On the upside, it would make use of cached data.
            for (int i = 0; i < Batches.Count; ++i)
            {
                var batch = Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches[j].WarmStart(ref bodies.Velocities);
                }
            }
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < Batches.Count; ++i)
                {
                    var batch = Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        batch.TypeBatches[j].SolveIteration(ref bodies.Velocities);
                    }
                }
            }
        }

        //Note that none of these affect the constraint batch estimates or type batch estimates. The assumption is that those are too small to bother with.
        //In the worst case you might see a couple of kilobytes. The reason why these functions exist is to deal with the potential many *megabytes* worth of constraint and body buffers.
        //Maybe something weird happens where this assumption is invalidated later, but I doubt it. There is a cost in API complexity to support it, so we don't.

        /// <summary>
        /// Removes all objects from the solver without returning any resources to the memory pool.
        /// </summary>
        public void Clear()
        {
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                batchReferencedHandles[batchIndex].Clear();
                Batches[batchIndex].Clear(bufferPool);
            }
            //Note that the 'pooled' batches are stored in place. By changing the count, we move all existing batches into the pool.
            Batches.Count = 0;
            handlePool.Clear();
        }


        public void EnsureCapacity(int bodiesCount, int constraintCount, int constraintsPerTypeBatch)
        {
            if (!Batches.Span.Allocated)
            {
                //This solver instance was disposed, so we need to explicitly reconstruct the batches array.
                QuickList<ConstraintBatch, Buffer<ConstraintBatch>>.Create(bufferPool.SpecializeFor<ConstraintBatch>(), BatchCountEstimate, out Batches);
            }
            if (HandleToConstraint.Length < constraintCount)
            {
                //Note that the handle pool, if disposed, will have a -1 highest claimed id, corresponding to a 0 length copy region.
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, constraintCount, handlePool.HighestPossiblyClaimedId + 1);
            }
            //Note that we modify the type batch allocation and pass it to the batches. 
            //The idea here is that the user may also have modified the per-type sizes and wants them to affect the result of this call.
            constraintsPerTypeBatch = Math.Max(1, constraintsPerTypeBatch);
            if (TypeBatchCapacities.MinimumCapacity < constraintsPerTypeBatch)
                TypeBatchCapacities.MinimumCapacity = constraintsPerTypeBatch;
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].EnsureCapacity(TypeBatchCapacities, bodiesCount, TypeCountEstimate);
            }
            //Like the bodies set, we lazily handle handlePool internal capacity unless explicitly told to expand by ensure capacity.
            //This will likely be overkill, but it's a pretty small cost (oh no four hundred kilobytes for a simulation with 100,000 constraints).
            //If the user really wants to stop resizes, well, this will do that.
            handlePool.EnsureCapacity(constraintCount, bufferPool.SpecializeFor<int>());
        }

        public void Compact(int bodiesCount, int constraintCount, int constraintsPerTypeBatch)
        {
            constraintsPerTypeBatch = Math.Max(1, constraintsPerTypeBatch);
            if (TypeBatchCapacities.MinimumCapacity > constraintsPerTypeBatch)
                TypeBatchCapacities.MinimumCapacity = constraintsPerTypeBatch;
            //Note that we cannot safely compact the handles array below the highest potentially allocated id. This could be a little disruptive sometimes, but the cost is low.
            //If it ever ecomes a genuine problem, you can change the way the idpool works to permit a tighter maximum.
            var targetConstraintCount = BufferPool<ConstraintLocation>.GetLowestContainingElementCount(Math.Max(constraintCount, handlePool.HighestPossiblyClaimedId + 1));
            if (HandleToConstraint.Length > targetConstraintCount)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, targetConstraintCount, handlePool.HighestPossiblyClaimedId + 1);
            }
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Compact(TypeBatchCapacities, bodies, bodiesCount);
            }
            handlePool.Compact(constraintCount, bufferPool.SpecializeFor<int>());
        }

        public void Resize(int bodiesCount, int constraintCount, int constraintsPerTypeBatch)
        {
            if (!Batches.Span.Allocated)
            {
                //This solver instance was disposed, so we need to explicitly reconstruct the batches array.
                QuickList<ConstraintBatch, Buffer<ConstraintBatch>>.Create(bufferPool.SpecializeFor<ConstraintBatch>(), BatchCountEstimate, out Batches);
            }
            var targetConstraintCount = BufferPool<ConstraintLocation>.GetLowestContainingElementCount(Math.Max(constraintCount, handlePool.HighestPossiblyClaimedId + 1));
            if (HandleToConstraint.Length != targetConstraintCount)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, targetConstraintCount, handlePool.HighestPossiblyClaimedId + 1);
            }
            TypeBatchCapacities.MinimumCapacity = Math.Max(1, constraintsPerTypeBatch);
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Resize(TypeBatchCapacities, bodies, bodiesCount, TypeCountEstimate);
            }
            handlePool.Resize(constraintCount, bufferPool.SpecializeFor<int>());

            //TODO: dont forget this , pulled from constraintbatch
            //Note that we can't shrink below the bodies handle capacity, since the handle distribution could be arbitrary.
            BodyHandles.Resize(Math.Max(bodies.IndexToHandle.Length, bodiesCount), typeBatchAllocation.BufferPool);
        }

        /// <summary>
        /// Disposes all resources in the solver, returning unmanaged resources to the pool and dropping all pooled managed resource references.
        /// </summary>
        /// <remarks>The solver object can be reused if EnsureCapacity or Resize is called to rehydrate the resources.</remarks>
        public void Dispose()
        {
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Dispose(TypeBatchCapacities);
            }
            bufferPool.SpecializeFor<ConstraintLocation>().Return(ref HandleToConstraint);
            HandleToConstraint = new Buffer<ConstraintLocation>();
            handlePool.Dispose(bufferPool.SpecializeFor<int>());
            Batches.Dispose(bufferPool.SpecializeFor<ConstraintBatch>());
            Batches = new QuickList<ConstraintBatch, Buffer<ConstraintBatch>>();
        }


    }
}
