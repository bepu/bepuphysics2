using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Superclass of constraint type batch processors. Responsible for interpreting raw type batches for the purposes of bookkeeping and solving.
    /// </summary>
    /// <remarks>
    /// <para>This class holds no actual state of its own. A solver creates a unique type processor for each registered constraint type, and all instances are held in untyped memory.
    /// Splitting the functionality from the data allows for far fewer GC-tracked instances and allows the raw data layout to be shared more easily.</para>
    /// <para>For example, deactivated simulation islands store type batches, but they are created and used differently- and for convenience, they are stored on a per-island basis.
    /// Using the same system but with reference type TypeBatches, tens of thousands of inactive islands would imply tens of thousands of GC-tracked objects.</para>
    /// That's not acceptable, so here we are. 
    /// <para>Conceptually, you can think of the solver's array of TypeProcessors like C function pointers.</para>
    /// </remarks>
    public abstract class TypeProcessor
    {
        //TODO: Having this in the base class actually complicates the implementation of some special constraint types. Consider an 'articulation' subsolver that involves
        //N bodies, for N > Vector<float>.Count * 2. You may want to do SIMD internally in such a case, so there would be no 'bundles' at this level. Worry about that later.

        protected int typeId;
        public int TypeId { get { return typeId; } }
        public abstract int BodiesPerConstraint { get; }

        public void Initialize(int typeId)
        {
            this.typeId = typeId;
        }

        /// <summary>
        /// Allocates a slot in the batch.
        /// </summary>
        /// <param name="typeBatch">Type batch to allocate in.</param>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <param name="bodyIndices">Pointer to a list of body indices (not handles!) with count equal to the type batch's expected number of involved bodies.</param>
        /// <param name="pool">Allocation provider to use if the type batch has to be resized.</param>
        /// <returns>Index of the slot in the batch.</returns>
        public unsafe abstract int Allocate(ref TypeBatchData typeBatch, int handle, int* bodyIndices, BufferPool pool);
        public abstract void Remove(ref TypeBatchData typeBatch, int index, ref Buffer<ConstraintLocation> handlesToConstraints);

        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceBatchIndex">Index of the batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        public unsafe abstract void TransferConstraint(ref TypeBatchData typeBatch, int sourceBatchIndex, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex);

        public abstract void EnumerateConnectedBodyIndices<TEnumerator>(ref TypeBatchData typeBatch, int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int>;
        public abstract void UpdateForBodyMemoryMove(ref TypeBatchData typeBatch, int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation);

        public abstract void Scramble(ref TypeBatchData typeBatch, Random random, ref Buffer<ConstraintLocation> handlesToConstraints);

        internal abstract void GetBundleTypeSizes(out int bodyReferencesBundleSize, out int prestepBundleSize, out int accumulatedImpulseBundleSize);

        internal abstract void GenerateSortKeysAndCopyReferences(
            ref TypeBatchData typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache);

        internal abstract void CopyToCache(
            ref TypeBatchData typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref Buffer<int> indexToHandleCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache);

        internal abstract void Regather(
            ref TypeBatchData typeBatch,
            int constraintStart, int constraintCount, ref int firstSourceIndex,
            ref Buffer<int> indexToHandleCache, ref RawBuffer bodyReferencesCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache,
            ref Buffer<ConstraintLocation> handlesToConstraints);

        [Conditional("DEBUG")]
        internal abstract void VerifySortRegion(ref TypeBatchData typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices);
        internal abstract int GetBodyIndexInstanceCount(ref TypeBatchData typeBatch, int bodyIndex);

        public abstract void Initialize(ref TypeBatchData typeBatch, int initialCapacity, BufferPool pool);
        public abstract void Resize(ref TypeBatchData typeBatch, int newCapacity, BufferPool pool);

        public abstract void Prestep(ref TypeBatchData typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle);
        public abstract void WarmStart(ref TypeBatchData typeBatch, ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle);
        public abstract void SolveIteration(ref TypeBatchData typeBatch, ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle);


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(ref TypeBatchData typeBatch, Bodies bodies, float dt, float inverseDt)
        {
            Prestep(ref typeBatch, bodies, dt, inverseDt, 0, typeBatch.BundleCount);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref TypeBatchData typeBatch, ref Buffer<BodyVelocity> bodyVelocities)
        {
            WarmStart(ref typeBatch, ref bodyVelocities, 0, typeBatch.BundleCount);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveIteration(ref TypeBatchData typeBatch, ref Buffer<BodyVelocity> bodyVelocities)
        {
            SolveIteration(ref typeBatch, ref bodyVelocities, 0, typeBatch.BundleCount);
        }

    }

    /// <summary>
    /// Defines a function that creates a sort key from body references in a type batch. Used by constraint layout optimization.
    /// </summary>
    public interface ISortKeyGenerator<TBodyReferences>
    {
        int GetSortKey(int constraintIndex, ref Buffer<TBodyReferences> bodyReferences);
    }

    //Note that the only reason to have generics at the type level here is to avoid the need to specify them for each individual function. It's functionally equivalent, but this just
    //cuts down on the syntax noise a little bit. 
    //Really, you could use a bunch of composed static generic helpers.
    public abstract class TypeProcessor<TBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse> : TypeProcessor
    {

        static void IncreaseSize<T>(BufferPool rawPool, ref Buffer<T> buffer)
        {
            var pool = rawPool.SpecializeFor<T>();
            var old = buffer;
            pool.Take(buffer.Length * 2, out buffer);
            old.CopyTo(0, ref buffer, 0, old.Length);
            pool.Return(ref old);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void AddBodyReferencesLane(ref TBodyReferences bundle, int innerIndex, int* bodyIndices)
        {
            //The jit should be able to fold almost all of the size-related calculations and address fiddling.
            ref var start = ref Unsafe.As<TBodyReferences, int>(ref bundle);
            ref var targetLane = ref Unsafe.Add(ref start, innerIndex);
            var stride = Vector<int>.Count;
            //We assume that the body references struct is organized in memory like Bundle0, Inner0, ... BundleN, InnerN, Count
            //Assuming contiguous storage, Count is then located at start + stride * BodyCount.
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / (stride * sizeof(int));
            targetLane = *bodyIndices;
            for (int i = 1; i < bodyCount; ++i)
            {
                Unsafe.Add(ref targetLane, i * stride) = bodyIndices[i];
            }

#if DEBUG
            //The inner index is the last slot in the bundle, since adds always append and constraints are always a contiguous block.
            //Loop through the indices of the bundle and confirm that none of the indices are the same.
            for (int i = 0; i <= innerIndex; ++i)
            {
                var indices = stackalloc int[bodyCount * 2];
                for (int j = 0; j < bodyCount; ++j)
                {
                    indices[j] = Unsafe.Add(ref start, i + stride * j);
                }
                for (int j = i + 1; j <= innerIndex; ++j)
                {
                    for (int k = 0; k < bodyCount; ++k)
                    {
                        indices[bodyCount + k] = Unsafe.Add(ref start, j + stride * k);
                    }
                    for (int k = 0; k < bodyCount * 2; ++k)
                    {
                        for (int l = k + 1; l < bodyCount * 2; ++l)
                        {
                            Debug.Assert(indices[k] != indices[l], "A bundle should not share any body references. If an add causes redundant body references, something upstream broke.");
                        }
                    }
                }

            }
#endif
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static int GetCountInBundle(ref TypeBatchData typeBatch, int bundleStartIndex)
        {
            //TODO: May want to check codegen on this. Min vs explicit branch. Theoretically, it could do this branchlessly...
            return Math.Min(Vector<float>.Count, typeBatch.ConstraintCount - (bundleStartIndex << BundleIndexing.VectorShift));
        }


        public unsafe sealed override int Allocate(ref TypeBatchData typeBatch, int handle, int* bodyIndices, BufferPool pool)
        {
            Debug.Assert(typeBatch.BodyReferences.Allocated, "Should initialize the batch before allocating anything from it.");
            if (typeBatch.ConstraintCount == typeBatch.IndexToHandle.Length)
            {
                InternalResize(ref typeBatch, pool, typeBatch.ConstraintCount * 2);
            }
            var index = typeBatch.ConstraintCount++;
            typeBatch.IndexToHandle[index] = handle;
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            ref var bundle = ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, bundleIndex);
            AddBodyReferencesLane(ref bundle, innerIndex, bodyIndices);
            //Clear the slot's accumulated impulse. The backing memory could be initialized to any value.
            GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref Buffer<TAccumulatedImpulse>.Get(ref typeBatch.AccumulatedImpulses, bundleIndex), innerIndex);
            var bundleCount = typeBatch.BundleCount;
            Debug.Assert(typeBatch.PrestepData.Length >= bundleCount * Unsafe.SizeOf<TPrestepData>());
            Debug.Assert(typeBatch.Projection.Length >= bundleCount * Unsafe.SizeOf<TProjection>());
            Debug.Assert(typeBatch.BodyReferences.Length >= bundleCount * Unsafe.SizeOf<TBodyReferences>());
            Debug.Assert(typeBatch.AccumulatedImpulses.Length >= bundleCount * Unsafe.SizeOf<TAccumulatedImpulse>());
            Debug.Assert(typeBatch.IndexToHandle.Length >= typeBatch.ConstraintCount * Unsafe.SizeOf<int>());
            return index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static void CopyConstraintData(
             ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle, int sourceInner,
             ref TBodyReferences targetReferencesBundle, ref TPrestepData targetPrestepBundle, ref TAccumulatedImpulse targetAccumulatedBundle, int targetInner)
        {
            //Note that we do NOT copy the iteration data. It is regenerated each frame from scratch. 
            GatherScatter.CopyLane(ref sourceReferencesBundle, sourceInner, ref targetReferencesBundle, targetInner);
            GatherScatter.CopyLane(ref sourcePrestepBundle, sourceInner, ref targetPrestepBundle, targetInner);
            GatherScatter.CopyLane(ref sourceAccumulatedBundle, sourceInner, ref targetAccumulatedBundle, targetInner);
        }
        /// <summary>
        /// Overwrites all the data in the target constraint slot with source data.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static void Move(
            ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle, int sourceHandle,
            int sourceInner,
            ref TBodyReferences targetReferencesBundle, ref TPrestepData targetPrestepBundle, ref TAccumulatedImpulse targetAccumulatedBundle, ref int targetIndexToHandle,
            int targetInner, int targetIndex, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            CopyConstraintData(
                ref sourceReferencesBundle, ref sourcePrestepBundle, ref sourceAccumulatedBundle, sourceInner,
                ref targetReferencesBundle, ref targetPrestepBundle, ref targetAccumulatedBundle, targetInner);
            targetIndexToHandle = sourceHandle;
            handlesToConstraints[sourceHandle].IndexInTypeBatch = targetIndex;
        }



        public sealed override unsafe void Scramble(ref TypeBatchData typeBatch, Random random, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            //This is a pure debug function used to compare cache optimization strategies. Performance doesn't matter. 
            TPrestepData aPrestep = default(TPrestepData);
            TAccumulatedImpulse aAccumulated = default(TAccumulatedImpulse);
            TBodyReferences aBodyReferences = default(TBodyReferences);
            int aHandle;

            var prestepData = typeBatch.PrestepData.As<TPrestepData>();
            var accumulatedImpulses = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var bodyReferences = typeBatch.BodyReferences.As<TBodyReferences>();

            for (int a = typeBatch.ConstraintCount - 1; a >= 1; --a)
            {
                BundleIndexing.GetBundleIndices(a, out var aBundle, out var aInner);
                GatherScatter.CopyLane(ref bodyReferences[aBundle], aInner, ref aBodyReferences, 0);
                GatherScatter.CopyLane(ref prestepData[aBundle], aInner, ref aPrestep, 0);
                GatherScatter.CopyLane(ref accumulatedImpulses[aBundle], aInner, ref aAccumulated, 0);
                aHandle = typeBatch.IndexToHandle[a];

                var b = random.Next(a);
                BundleIndexing.GetBundleIndices(b, out var bBundle, out var bInner);
                //Move b into a's slot.
                Move(
                    ref bodyReferences[bBundle], ref prestepData[bBundle], ref accumulatedImpulses[bBundle], typeBatch.IndexToHandle[b], bInner,
                    ref bodyReferences[aBundle], ref prestepData[aBundle], ref accumulatedImpulses[aBundle], ref typeBatch.IndexToHandle[a], aInner, a, ref handlesToConstraints);
                //Move cached a into b's slot.
                Move(
                    ref aBodyReferences, ref aPrestep, ref aAccumulated, aHandle, 0,
                    ref bodyReferences[bBundle], ref prestepData[bBundle], ref accumulatedImpulses[bBundle], ref typeBatch.IndexToHandle[b], bInner, b, ref handlesToConstraints);
            }
        }

        /// <summary>
        /// Removes a constraint from the batch.
        /// </summary>
        /// <param name="index">Index of the constraint to remove.</param>
        /// <param name="handlesToConstraints">The handle to constraint mapping used by the solver that could be modified by a swap on removal.</param>
        public override unsafe void Remove(ref TypeBatchData typeBatch, int index, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            Debug.Assert(index >= 0 && index < typeBatch.ConstraintCount, "Can only remove elements that are actually in the batch!");
            var lastIndex = typeBatch.ConstraintCount - 1;
            typeBatch.ConstraintCount = lastIndex;
            BundleIndexing.GetBundleIndices(lastIndex, out var sourceBundleIndex, out var sourceInnerIndex);
#if DEBUG
            //The Move below overwrites the IndexToHandle, so if we want to use it for debugging, we gotta cache it.
            var removedHandle = typeBatch.IndexToHandle[index];
#endif
            ref var bodyReferences = ref Unsafe.As<byte, TBodyReferences>(ref *typeBatch.BodyReferences.Memory);
            if (index < lastIndex)
            {
                //Need to swap.
                ref var prestepData = ref Unsafe.As<byte, TPrestepData>(ref *typeBatch.PrestepData.Memory);
                ref var accumulatedImpulses = ref Unsafe.As<byte, TAccumulatedImpulse>(ref *typeBatch.AccumulatedImpulses.Memory);
                BundleIndexing.GetBundleIndices(index, out var targetBundleIndex, out var targetInnerIndex);
                Move(
                    ref Unsafe.Add(ref bodyReferences, sourceBundleIndex), ref Unsafe.Add(ref prestepData, sourceBundleIndex), ref Unsafe.Add(ref accumulatedImpulses, sourceBundleIndex),
                    typeBatch.IndexToHandle[lastIndex], sourceInnerIndex,
                    ref Unsafe.Add(ref bodyReferences, targetBundleIndex), ref Unsafe.Add(ref prestepData, targetBundleIndex), ref Unsafe.Add(ref accumulatedImpulses, targetBundleIndex),
                    ref typeBatch.IndexToHandle[lastIndex], targetInnerIndex, lastIndex,
                    ref handlesToConstraints);
            }
            //TODO: Is clearing the body references even required anymore? Pretty sure it's not, but we need to confirm. 
            //If you don't read or write using data outside the count, then you don't need it.
            ref var bundle = ref Unsafe.Add(ref bodyReferences, sourceBundleIndex);
            GatherScatter.ClearLane<TBodyReferences, int>(ref bundle, sourceInnerIndex);

#if DEBUG
            //While it's not necessary to clear these, it can be useful for debugging if any accesses of the old position (that are not refilled immediately)
            //result in some form of index error later upon invalid usage.
            handlesToConstraints[removedHandle].BatchIndex = -1;
            handlesToConstraints[removedHandle].IndexInTypeBatch = -1;
            handlesToConstraints[removedHandle].TypeId = -1;
            typeBatch.IndexToHandle[lastIndex] = -1;
#endif
        }

        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceBatchIndex">Index of the batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        public unsafe override void TransferConstraint(ref TypeBatchData typeBatch, int sourceBatchIndex, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex)
        {
            //Note that the following does some redundant work. It's technically possible to do better than this, but it requires bypassing a lot of bookkeeping.
            //It's not exactly trivial to keep everything straight, especially over time- it becomes a maintenance nightmare.
            //So instead, given that compressions should generally be extremely rare (relatively speaking) and highly deferrable, we'll accept some minor overhead.
            int bodiesPerConstraint = BodiesPerConstraint;
            var bodyHandles = stackalloc int[bodiesPerConstraint];
            var bodyHandleCollector = new ConstraintBodyHandleCollector(bodies, bodyHandles);
            EnumerateConnectedBodyIndices(ref typeBatch, indexInTypeBatch, ref bodyHandleCollector);
            ref var targetBatch = ref solver.Batches[targetBatchIndex];
            //Allocate a spot in the new batch. Note that it does not change the Handle->Constraint mapping in the Solver; that's important when we call Solver.Remove below.
            var constraintHandle = typeBatch.IndexToHandle[indexInTypeBatch];
            targetBatch.Allocate(constraintHandle, ref bodyHandles[0], bodiesPerConstraint,
                ref solver.batchReferencedHandles[sourceBatchIndex], bodies, typeId, solver.TypeProcessors[typeId],
                solver.GetMinimumCapacityForType(typeId), solver.bufferPool, out var targetReference);

            BundleIndexing.GetBundleIndices(targetReference.IndexInTypeBatch, out var targetBundle, out var targetInner);
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var sourceBundle, out var sourceInner);
            //We don't pull a description or anything from the old constraint. That would require having a unique mapping from constraint to 'full description'. 
            //Instead, we just directly copy from lane to lane.
            //Note that we leave out the runtime generated bits- they'll just get regenerated.
            CopyConstraintData(
                ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, sourceBundle),
                ref Buffer<TPrestepData>.Get(ref typeBatch.PrestepData, sourceBundle),
                ref Buffer<TAccumulatedImpulse>.Get(ref typeBatch.AccumulatedImpulses, sourceBundle),
                sourceInner,
                ref Buffer<TBodyReferences>.Get(ref targetReference.TypeBatch.BodyReferences, targetBundle),
                ref Buffer<TPrestepData>.Get(ref targetReference.TypeBatch.PrestepData, targetBundle),
                ref Buffer<TAccumulatedImpulse>.Get(ref targetReference.TypeBatch.AccumulatedImpulses, targetBundle),
                targetInner);

            //Now we can get rid of the old allocation.
            //Note the use of RemoveFromBatch instead of Remove. Solver.Remove returns the handle to the pool, which we do not want!
            //It may look a bit odd to use a solver-level function here, given that we are operating on batches and handling the solver state directly for the most part. 
            //However, removes can result in empty batches that require resource reclamation. 
            //Rather than reimplementing that we just reuse the solver's version. 
            //That sort of resource cleanup isn't required on add- everything that is needed already exists, and nothing is going away.
            solver.RemoveFromBatch(sourceBatchIndex, typeId, indexInTypeBatch);

            //Don't forget to keep the solver's pointers consistent! We bypassed the usual add procedure, so the solver hasn't been notified yet.
            ref var constraintLocation = ref solver.HandleToConstraint[constraintHandle];
            constraintLocation.BatchIndex = targetBatchIndex;
            constraintLocation.IndexInTypeBatch = targetReference.IndexInTypeBatch;
            constraintLocation.TypeId = typeId;

        }

        void InternalResize(ref TypeBatchData typeBatch, BufferPool pool, int constraintCapacity)
        {
            Debug.Assert(constraintCapacity >= 0, "The constraint capacity should have already been validated.");
            pool.SpecializeFor<int>().Resize(ref typeBatch.IndexToHandle, constraintCapacity, typeBatch.ConstraintCount);
            //Note that we construct the bundle capacity from the resized constraint capacity. This means we only have to check the IndexToHandle capacity
            //before allocating, which simplifies things a little bit at the cost of some memory. Could revisit this if memory use is actually a concern.
            var bundleCapacity = BundleIndexing.GetBundleCount(typeBatch.IndexToHandle.Length);
            //Note that the projection is not copied over. It is ephemeral data. (In the same vein as above, if memory is an issue, we could just allocate projections on demand.)
            var bundleCount = typeBatch.BundleCount;
            pool.Resize(ref typeBatch.Projection, bundleCapacity * Unsafe.SizeOf<TProjection>(), 0);
            pool.Resize(ref typeBatch.BodyReferences, bundleCapacity * Unsafe.SizeOf<TBodyReferences>(), bundleCount);
            pool.Resize(ref typeBatch.PrestepData, bundleCapacity * Unsafe.SizeOf<TPrestepData>(), bundleCount);
            pool.Resize(ref typeBatch.AccumulatedImpulses, bundleCapacity * Unsafe.SizeOf<TAccumulatedImpulse>(), bundleCount);
        }

        public override void Initialize(ref TypeBatchData typeBatch, int initialCapacity, BufferPool pool)
        {
            //We default-initialize the type batch because the resize checks existing allocation status when deciding how much to copy over. 
            //Technically we could use an initialization-specific version, but it doesn't matter.
            typeBatch = new TypeBatchData();
            typeBatch.TypeId = TypeId;
            InternalResize(ref typeBatch, pool, initialCapacity);
        }

        public override void Resize(ref TypeBatchData typeBatch, int desiredCapacity, BufferPool pool)
        {
            var desiredConstraintCapacity = BufferPool<int>.GetLowestContainingElementCount(desiredCapacity);
            if (desiredConstraintCapacity != typeBatch.IndexToHandle.Length)
            {
                InternalResize(ref typeBatch, pool, desiredConstraintCapacity);
            }
        }


        internal sealed override void GetBundleTypeSizes(out int bodyReferencesBundleSize, out int prestepBundleSize, out int accumulatedImpulseBundleSize)
        {
            bodyReferencesBundleSize = Unsafe.SizeOf<TBodyReferences>();
            prestepBundleSize = Unsafe.SizeOf<TPrestepData>();
            accumulatedImpulseBundleSize = Unsafe.SizeOf<TAccumulatedImpulse>();
        }


        public sealed override void UpdateForBodyMemoryMove(ref TypeBatchData typeBatch, int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            //Note that this relies on the bodyreferences memory layout. It uses the stride of vectors to skip to the next body based on the bodyIndexInConstraint.
            ref var bundle = ref Unsafe.As<TBodyReferences, Vector<int>>(ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, constraintBundleIndex));
            GatherScatter.Get(ref bundle, constraintInnerIndex + bodyIndexInConstraint * Vector<int>.Count) = newBodyLocation;
        }

        //Note that these next two sort key users require a generic sort key implementation; this avoids virtual dispatch on a per-object level while still sharing the bulk of the logic.
        //Technically, we could force the TSortKeyGenerator to be defined at the generic type level, but this seems a little less... extreme.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void GenerateSortKeysAndCopyReferences<TSortKeyGenerator>(
           ref TypeBatchData typeBatch,
           int bundleStart, int localBundleStart, int bundleCount,
           int constraintStart, int localConstraintStart, int constraintCount,
           ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache)
            where TSortKeyGenerator : struct, ISortKeyGenerator<TBodyReferences>
        {
            var sortKeyGenerator = default(TSortKeyGenerator);
            var bodyReferences = typeBatch.BodyReferences.As<TBodyReferences>();
            for (int i = 0; i < constraintCount; ++i)
            {
                Unsafe.Add(ref firstSourceIndex, i) = localConstraintStart + i;
                Unsafe.Add(ref firstSortKey, i) = sortKeyGenerator.GetSortKey(constraintStart + i, ref bodyReferences);
            }
            var typedBodyReferencesCache = bodyReferencesCache.As<TBodyReferences>();
            bodyReferences.CopyTo(bundleStart, ref typedBodyReferencesCache, localBundleStart, bundleCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void VerifySortRegion<TSortKeyGenerator>(ref TypeBatchData typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
            where TSortKeyGenerator : struct, ISortKeyGenerator<TBodyReferences>
        {
            var sortKeyGenerator = default(TSortKeyGenerator);
            var previousKey = -1;
            var baseIndex = bundleStartIndex << BundleIndexing.VectorShift;
            var bodyReferences = typeBatch.BodyReferences.As<TBodyReferences>();
            for (int i = 0; i < constraintCount; ++i)
            {
                var sourceIndex = sortedSourceIndices[i];
                var targetIndex = baseIndex + i;
                var key = sortKeyGenerator.GetSortKey(baseIndex + i, ref bodyReferences);
                Debug.Assert(key > previousKey, "After the sort and swap completes, all constraints should be in order.");
                Debug.Assert(key == sortedKeys[i], "After the swap goes through, the rederived sort keys should match the previously sorted ones.");
                previousKey = key;

            }
        }

        internal unsafe sealed override void CopyToCache(
            ref TypeBatchData typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref Buffer<int> indexToHandleCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache)
        {
            typeBatch.IndexToHandle.CopyTo(constraintStart, ref indexToHandleCache, localConstraintStart, constraintCount);
            var typedPrestepCache = prestepCache.As<TPrestepData>();
            var typedAccumulatedImpulsesCache = accumulatedImpulsesCache.As<TAccumulatedImpulse>();
            Unsafe.CopyBlockUnaligned(
                prestepCache.Memory + Unsafe.SizeOf<TPrestepData>() * localBundleStart,
                typeBatch.PrestepData.Memory + Unsafe.SizeOf<TPrestepData>() * bundleStart,
                (uint)(Unsafe.SizeOf<TPrestepData>() * bundleCount));
            Unsafe.CopyBlockUnaligned(
                accumulatedImpulsesCache.Memory + Unsafe.SizeOf<TAccumulatedImpulse>() * localBundleStart,
                typeBatch.AccumulatedImpulses.Memory + Unsafe.SizeOf<TAccumulatedImpulse>() * bundleStart,
                (uint)(Unsafe.SizeOf<TAccumulatedImpulse>() * bundleCount));
        }
        internal sealed override void Regather(
            ref TypeBatchData typeBatch,
            int constraintStart, int constraintCount, ref int firstSourceIndex,
            ref Buffer<int> indexToHandleCache, ref RawBuffer bodyReferencesCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache,
            ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            var typedBodyReferencesCache = bodyReferencesCache.As<TBodyReferences>();
            var typedPrestepCache = prestepCache.As<TPrestepData>();
            var typedAccumulatedImpulsesCache = accumulatedImpulsesCache.As<TAccumulatedImpulse>();

            var typedBodyReferencesTarget = typeBatch.BodyReferences.As<TBodyReferences>();
            var typedPrestepTarget = typeBatch.BodyReferences.As<TPrestepData>();
            var typedAccumulatedImpulsesTarget = typeBatch.BodyReferences.As<TAccumulatedImpulse>();
            for (int i = 0; i < constraintCount; ++i)
            {
                var sourceIndex = Unsafe.Add(ref firstSourceIndex, i);
                var targetIndex = constraintStart + i;
                //Note that we do not bother checking whether the source and target are the same.
                //The cost of the branch is large enough in comparison to the frequency of its usefulness that it only helps in practically static situations.
                //Also, its maximum benefit is quite small.
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

                Move(
                    ref typedBodyReferencesCache[sourceBundle], ref typedPrestepCache[sourceBundle], ref typedAccumulatedImpulsesCache[sourceBundle],
                    indexToHandleCache[sourceIndex], sourceInner,
                    ref typedBodyReferencesTarget[targetBundle], ref typedPrestepTarget[targetBundle], ref typedAccumulatedImpulsesTarget[targetBundle],
                    ref typeBatch.IndexToHandle[targetIndex], targetInner, targetIndex, ref handlesToConstraints);

            }
        }


        internal override int GetBodyIndexInstanceCount(ref TypeBatchData typeBatch, int bodyIndexToFind)
        {
            //This is a pure debug function; performance does not matter.
            var bundleCount = typeBatch.BundleCount;
            var bodyReferences = typeBatch.BodyReferences.As<TBodyReferences>();
            int count = 0;
            var bodiesPerConstraint = BodiesPerConstraint;
            for (int bundleIndex = 0; bundleIndex < bundleCount; ++bundleIndex)
            {
                var bundleSize = Math.Min(Vector<float>.Count, typeBatch.ConstraintCount - (bundleIndex << BundleIndexing.VectorShift));
                ref var bundleBase = ref Unsafe.As<TBodyReferences, Vector<int>>(ref bodyReferences[bundleIndex]);
                for (int constraintBodyIndex = 0; constraintBodyIndex < bodiesPerConstraint; ++constraintBodyIndex)
                {
                    ref var bodyVectorBase = ref Unsafe.As<Vector<int>, int>(ref Unsafe.Add(ref bundleBase, constraintBodyIndex));
                    for (int innerIndex = 0; innerIndex < bundleSize; ++innerIndex)
                    {
                        if (Unsafe.Add(ref bodyVectorBase, innerIndex) == bodyIndexToFind)
                            ++count;
                        Debug.Assert(count <= 1);
                    }
                }
            }
            return count;
        }
    }


}
