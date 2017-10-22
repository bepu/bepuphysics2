using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints
{
    //TODO: At the moment, we have a TypeBatch instance for every individual type batch, which means a reference type that has to be tracked by the GC.
    //While vtables are going to be involved at some point, you could get rid of the extra instances by just keeping a single array with unique instances of the
    //type batches. In other words, you're using it as an array of function pointers.
    //The actual constraint data would be held in blittable value types, which isn't much of a change.
    //This would be very similar to how other systems like the narrow phase and bounding box updater work.
    //Fairly low value change; almost no impact on performance. Might be worth doing if you ever do a heavy refactor to support things like a jacobi fallback solver though.
    //(Jacobi fallback would operate on almost all the exact same data, but simply handles the velocity scatter in a different way.)
    //One downside: the typebatch would have to do a cast on the incoming data. Not a big deal.
    public abstract class TypeBatch
    {
        //TODO: Having this in the base class actually complicates the implementation of some special constraint types. Consider an 'articulation' subsolver that involves
        //N bodies, for N > Vector<float>.Count * 2. You may want to do SIMD internally in such a case, so there would be no 'bundles' at this level. Worry about that later.
        protected int bundleCount;
        public int BundleCount => bundleCount;
        protected int constraintCount;
        protected int typeId;
        public int ConstraintCount => constraintCount;

        public abstract int BodiesPerConstraint { get; }
        public int TypeId { get { return typeId; } }

        /// <summary>
        /// The handles for the constraints in this type batch.
        /// </summary>
        public Buffer<int> IndexToHandle;

        /// <summary>
        /// Allocates a slot in the batch.
        /// </summary>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <param name="bodyIndices">Pointer to a list of body indices (not handles!) with count equal to the type batch's expected number of involved bodies.</param>
        /// <param name="typeBatchAllocation">Type batch allocation provider to use if the type batch has to be resized.</param>
        /// <returns>Index of the slot in the batch.</returns>
        public unsafe abstract int Allocate(int handle, int* bodyIndices, TypeBatchAllocation typeBatchAllocation);
        public abstract void Remove(int index, ref Buffer<ConstraintLocation> handlesToConstraints);

        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceBatchIndex">Index of the batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        public unsafe abstract void TransferConstraint(int sourceBatchIndex, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex);

        public abstract void EnumerateConnectedBodyIndices<TEnumerator>(int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int>;
        public abstract void UpdateForBodyMemoryMove(int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation);
        /// <summary>
        /// Swaps two constraints in the type batch by index.
        /// </summary>
        /// <param name="indexA">Index of the first constraint to swap.</param>
        /// <param name="indexB">Index of the second constraint to swap.</param>
        /// <param name="handlesToConstraints">The owning solver's handle to constraint mapping.</param>
        public abstract void SwapConstraints(int a, int b, ref Buffer<ConstraintLocation> handlesToConstraints);

        public abstract void Scramble(Random random, ref Buffer<ConstraintLocation> handlesToConstraints);

        internal abstract void GetBundleTypeSizes(out int bodyReferencesBundleSize, out int prestepBundleSize, out int accumulatedImpulseBundleSize);

        internal abstract void GenerateSortKeysAndCopyReferences(
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache);

        internal abstract void CopyToCache(
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref Buffer<int> indexToHandleCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache);

        internal abstract void Regather(int constraintStart, int constraintCount, ref int firstSourceIndex,
           ref Buffer<int> indexToHandleCache, ref RawBuffer bodyReferencesCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache,
            ref Buffer<ConstraintLocation> handlesToConstraints);

        [Conditional("DEBUG")]
        internal abstract void VerifySortRegion(int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices);
        internal abstract int GetBodyIndexInstanceCount(int bodyIndex);

        public abstract void Initialize(TypeBatchAllocation typeBatchAllocation, int typeId);
        public abstract void EnsureCapacity(TypeBatchAllocation typeBatchAllocation);
        public abstract void Compact(TypeBatchAllocation typeBatchAllocation);
        public abstract void Resize(TypeBatchAllocation typeBatchAllocation);
        public abstract void Dispose(BufferPool rawPool);

        public abstract void Prestep(Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle);
        public abstract void WarmStart(ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle);
        public abstract void SolveIteration(ref Buffer<BodyVelocity> bodyVelocities, int startBundle, int exclusiveEndBundle);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, float dt, float inverseDt)
        {
            Prestep(bodies, dt, inverseDt, 0, bundleCount);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref Buffer<BodyVelocity> bodyVelocities)
        {
            WarmStart(ref bodyVelocities, 0, bundleCount);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveIteration(ref Buffer<BodyVelocity> bodyVelocities)
        {
            SolveIteration(ref bodyVelocities, 0, bundleCount);
        }

    }
    //You are allowed to squint at this triple-class separation.
    //This is only really here because there are cases (e.g. adding a constraint) where it is necessary to have knowledge of TBodyReferences so that the caller (the solver, generally)
    //can communicate to the type batch in a type safe way. The alternative would have been including the TPrestepData, TProjection, and TAccumulatedImpulse, which just gets excessive.
    //Avoiding generic type knowledge would likely have involved some goofy safe-but-Unsafe casting.
    //We might have some issues with this in the future if we have fully unconstrained body reference counts in a constraint. It's wise to avoid that.

    public abstract class TypeBatch<TBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse> : TypeBatch
    {
        public Buffer<TBodyReferences> BodyReferences;
        public Buffer<TPrestepData> PrestepData;
        //Technically, the iteration data does not need to persist outside of the scope of the solve. We let it persist for simplicity- it does not take much space.

        //Projection and unprojection data required by the WarmStart and SolveIteration. Note that this data is conceptually ephemeral.
        //External users should not depend upon it outside of the solver's execution.
        //(At the moment, it does persist, but it becomes unreliable when constraints are removed, and the implementation reserves the right to make it completely temporary.)
        protected Buffer<TProjection> Projection;
        public Buffer<TAccumulatedImpulse> AccumulatedImpulses;


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
        protected int GetCountInBundle(int bundleStartIndex)
        {
            //TODO: May want to check codegen on this. Min vs explicit branch. Theoretically, it could do this branchlessly...
            return Math.Min(Vector<float>.Count, constraintCount - (bundleStartIndex << BundleIndexing.VectorShift));
        }


        public unsafe sealed override int Allocate(int handle, int* bodyIndices, TypeBatchAllocation typeBatchAllocation)
        {
            Debug.Assert(Projection.Memory != null, "Should initialize the batch before allocating anything from it.");
            if (constraintCount == IndexToHandle.Length)
            {
                InternalResize(typeBatchAllocation, constraintCount * 2);
            }
            var index = constraintCount++;
            if ((constraintCount & BundleIndexing.VectorMask) == 1)
                ++bundleCount;
            IndexToHandle[index] = handle;
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            ref var bundle = ref BodyReferences[bundleIndex];
            AddBodyReferencesLane(ref bundle, innerIndex, bodyIndices);
            //Clear the slot's accumulated impulse. The backing memory could be initialized to any value.
            GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref AccumulatedImpulses[bundleIndex], innerIndex);
            Debug.Assert(PrestepData.Length >= bundleCount);
            Debug.Assert(Projection.Length >= bundleCount);
            Debug.Assert(BodyReferences.Length >= bundleCount);
            Debug.Assert(AccumulatedImpulses.Length >= bundleCount);
            Debug.Assert(IndexToHandle.Length >= constraintCount);
            return index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static void CopyConstraintData(
             ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle, int sourceInner,
             ref TBodyReferences targetReferencesBundle, ref TPrestepData targetPrestepBundle, ref TAccumulatedImpulse targetAccumulatedBundle, int targetInner)
        {
            //Note that we do NOT copy the iteration data. It is regenerated each frame from scratch. 
            //We may later decide that this is silly because someone might rely on it, but... it seems very unlikely. 
            //Try to stop people from relying on it, and see if anyone ever complains.
            GatherScatter.CopyLane(ref sourceReferencesBundle, sourceInner, ref targetReferencesBundle, targetInner);
            GatherScatter.CopyLane(ref sourcePrestepBundle, sourceInner, ref targetPrestepBundle, targetInner);
            GatherScatter.CopyLane(ref sourceAccumulatedBundle, sourceInner, ref targetAccumulatedBundle, targetInner);
        }
        /// <summary>
        /// Overwrites all the data in the target constraint slot with source data.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static void Move(
            ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle,
            ref Buffer<TBodyReferences> bodyReferences, ref Buffer<TPrestepData> prestepData, ref Buffer<TAccumulatedImpulse> accumulatedImpulses,
            ref Buffer<int> indexToHandle,
            int sourceInner, int sourceHandle, int targetBundle, int targetInner, int targetIndex, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            CopyConstraintData(
                ref sourceReferencesBundle, ref sourcePrestepBundle, ref sourceAccumulatedBundle, sourceInner,
                ref bodyReferences[targetBundle], ref prestepData[targetBundle], ref accumulatedImpulses[targetBundle], targetInner);
            indexToHandle[targetIndex] = sourceHandle;
            handlesToConstraints[sourceHandle].IndexInTypeBatch = targetIndex;
        }

        /// <summary>
        /// Overwrites all the data in the target constraint slot with source data.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(
            ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle,
            int sourceInner, int sourceHandle, int targetBundle, int targetInner, int targetIndex, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            Move(ref sourceReferencesBundle, ref sourcePrestepBundle, ref sourceAccumulatedBundle, ref BodyReferences, ref PrestepData, ref AccumulatedImpulses, ref IndexToHandle,
                sourceInner, sourceHandle, targetBundle, targetInner, targetIndex, ref handlesToConstraints);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(int sourceBundle, int sourceInner, int sourceIndex, int targetBundle, int targetInner, int targetIndex,
             ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            Move(ref BodyReferences[sourceBundle], ref PrestepData[sourceBundle], ref AccumulatedImpulses[sourceBundle], sourceInner, IndexToHandle[sourceIndex],
                targetBundle, targetInner, targetIndex, ref handlesToConstraints);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(int sourceIndex, int targetIndex,
            ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
            BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);
            Move(ref BodyReferences[sourceBundle], ref PrestepData[sourceBundle], ref AccumulatedImpulses[sourceBundle], sourceInner, IndexToHandle[sourceIndex],
                targetBundle, targetInner, targetIndex, ref handlesToConstraints);
        }

        /// <summary>
        /// Swaps two constraints in the type batch by index.
        /// </summary>
        /// <param name="a">Index of the first constraint to swap.</param>
        /// <param name="b">Index of the second constraint to swap.</param>
        /// <param name="handlesToConstraints">The owning solver's handle to constraint mapping.</param>
        public override sealed void SwapConstraints(int a, int b, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            //TODO: It would probably be a good idea to preallocate a memory blob for use as a cache rather than initializes a bunch of new locals.
            //Maybe newer JITs won't require init, but it doesn't require that much effort to preallocate a blob of sufficient size.
            TPrestepData aPrestep = default(TPrestepData);
            TAccumulatedImpulse aAccumulated = default(TAccumulatedImpulse);
            TBodyReferences aBodyReferences = default(TBodyReferences);

            BundleIndexing.GetBundleIndices(a, out var aBundle, out var aInner);
            GatherScatter.CopyLane(ref BodyReferences[aBundle], aInner, ref aBodyReferences, 0);
            GatherScatter.CopyLane(ref PrestepData[aBundle], aInner, ref aPrestep, 0);
            GatherScatter.CopyLane(ref AccumulatedImpulses[aBundle], aInner, ref aAccumulated, 0);
            var aHandle = IndexToHandle[a];

            BundleIndexing.GetBundleIndices(b, out var bBundle, out var bInner);
            Move(bBundle, bInner, b, aBundle, aInner, a, ref handlesToConstraints);
            Move(ref aBodyReferences, ref aPrestep, ref aAccumulated, 0, aHandle, bBundle, bInner, b, ref handlesToConstraints);
        }


        public sealed override void Scramble(Random random, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            //This is a pure debug function used to compare cache optimization strategies. Performance doesn't matter. 
            TPrestepData aPrestep = default(TPrestepData);
            TAccumulatedImpulse aAccumulated = default(TAccumulatedImpulse);
            TBodyReferences aBodyReferences = default(TBodyReferences);
            int aHandle;

            for (int a = ConstraintCount - 1; a >= 1; --a)
            {
                BundleIndexing.GetBundleIndices(a, out var aBundle, out var aInner);
                GatherScatter.CopyLane(ref BodyReferences[aBundle], aInner, ref aBodyReferences, 0);
                GatherScatter.CopyLane(ref PrestepData[aBundle], aInner, ref aPrestep, 0);
                GatherScatter.CopyLane(ref AccumulatedImpulses[aBundle], aInner, ref aAccumulated, 0);
                aHandle = IndexToHandle[a];

                var b = random.Next(a);
                BundleIndexing.GetBundleIndices(b, out var bBundle, out var bInner);
                Move(bBundle, bInner, b, aBundle, aInner, a, ref handlesToConstraints);
                Move(ref aBodyReferences, ref aPrestep, ref aAccumulated, 0, aHandle, bBundle, bInner, b, ref handlesToConstraints);
            }
        }

        /// <summary>
        /// Removes a constraint from the batch.
        /// </summary>
        /// <param name="index">Index of the constraint to remove.</param>
        /// <param name="handlesToConstraints">The handle to constraint mapping used by the solver that could be modified by a swap on removal.</param>
        public override void Remove(int index, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            Debug.Assert(index >= 0 && index < constraintCount, "Can only remove elements that are actually in the batch!");
            var lastIndex = constraintCount - 1;
            constraintCount = lastIndex;
            if ((constraintCount & BundleIndexing.VectorMask) == 0)
                --bundleCount;
            BundleIndexing.GetBundleIndices(lastIndex, out var sourceBundleIndex, out var sourceInnerIndex);
#if DEBUG
            //The Move below overwrites the IndexToHandle, so if we want to use it for debugging, we gotta cache it.
            var removedHandle = IndexToHandle[index];
#endif
            if (index < lastIndex)
            {
                //Need to swap.
                BundleIndexing.GetBundleIndices(index, out var targetBundleIndex, out var targetInnerIndex);
                Move(sourceBundleIndex, sourceInnerIndex, lastIndex, targetBundleIndex, targetInnerIndex, index, ref handlesToConstraints);
            }
            RemoveBodyReferences(sourceBundleIndex, sourceInnerIndex);

#if DEBUG
            //While it's not necessary to clear these, it can be useful for debugging if any accesses of the old position (that are not refilled immediately)
            //result in some form of index error later upon invalid usage.
            handlesToConstraints[removedHandle].BatchIndex = -1;
            handlesToConstraints[removedHandle].IndexInTypeBatch = -1;
            handlesToConstraints[removedHandle].TypeId = -1;
            IndexToHandle[lastIndex] = -1;
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
        public unsafe override void TransferConstraint(int sourceBatchIndex, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex)
        {
            //Note that the following does some redundant work. It's technically possible to do better than this, but it requires bypassing a lot of bookkeeping.
            //It's not exactly trivial to keep everything straight, especially over time- it becomes a maintenance nightmare.
            //So instead, given that compressions should generally be extremely rare (relatively speaking) and highly deferrable, we'll accept some minor overhead.
            int bodiesPerConstraint = BodiesPerConstraint;
            var bodyHandles = stackalloc int[BodiesPerConstraint];
            var bodyHandleCollector = new ConstraintBodyHandleCollector(bodies, bodyHandles);
            EnumerateConnectedBodyIndices(indexInTypeBatch, ref bodyHandleCollector);
            var targetBatch = solver.Batches[targetBatchIndex];
            //Allocate a spot in the new batch. Note that it does not change the Handle->Constraint mapping in the Solver; that's important when we call Solver.Remove below.
            var constraintHandle = IndexToHandle[indexInTypeBatch];
            targetBatch.Allocate(constraintHandle, ref bodyHandles[0], bodiesPerConstraint, bodies, solver.TypeBatchAllocation, typeId, out var newReference);

            //This cast is pretty gross looking, but guaranteed to work by virtue of the typeid registrations.
            var targetTypeBatch = (TypeBatch<TBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>)newReference.TypeBatch;
            BundleIndexing.GetBundleIndices(newReference.IndexInTypeBatch, out var targetBundle, out var targetInner);
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var sourceBundle, out var sourceInner);
            //We don't pull a description or anything from the old constraint. That would require having a unique mapping from constraint to 'full description'. 
            //Instead, we just directly copy from lane to lane.
            //Note that we leave out the runtime generated bits- they'll just get regenerated.
            GatherScatter.CopyLane(ref BodyReferences[sourceBundle], sourceInner, ref targetTypeBatch.BodyReferences[targetBundle], targetInner);
            GatherScatter.CopyLane(ref PrestepData[sourceBundle], sourceInner, ref targetTypeBatch.PrestepData[targetBundle], targetInner);
            GatherScatter.CopyLane(ref AccumulatedImpulses[sourceBundle], sourceInner, ref targetTypeBatch.AccumulatedImpulses[targetBundle], targetInner);

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
            constraintLocation.IndexInTypeBatch = newReference.IndexInTypeBatch;
            constraintLocation.TypeId = typeId;

        }

        public override void Initialize(TypeBatchAllocation typeBatchAllocation, int typeId)
        {
            this.typeId = typeId;
            InternalResize(typeBatchAllocation, typeBatchAllocation[typeId]);
        }

        void InternalResize(TypeBatchAllocation typeBatchAllocation, int constraintCapacity)
        {
            Debug.Assert(constraintCapacity >= typeBatchAllocation[typeId], "The constraint capacity should have already been validated.");
            typeBatchAllocation.BufferPool.SpecializeFor<int>().Resize(ref IndexToHandle, constraintCapacity, constraintCount);
            //Note that we construct the bundle capacity from the resized constraint capacity. This means we only have to check the IndexToHandle capacity
            //before allocating, which simplifies things a little bit at the cost of some memory. Could revisit this if memory use is actually a concern.
            var bundleCapacity = BundleIndexing.GetBundleCount(IndexToHandle.Length);
            //Note that the projection is not copied over. It is ephemeral data. (In the same vein as above, if memory is an issue, we could just allocate projections on demand.)
            typeBatchAllocation.BufferPool.SpecializeFor<TProjection>().Resize(ref Projection, bundleCapacity, 0);
            typeBatchAllocation.BufferPool.SpecializeFor<TBodyReferences>().Resize(ref BodyReferences, bundleCapacity, bundleCount);
            typeBatchAllocation.BufferPool.SpecializeFor<TPrestepData>().Resize(ref PrestepData, bundleCapacity, bundleCount);
            typeBatchAllocation.BufferPool.SpecializeFor<TAccumulatedImpulse>().Resize(ref AccumulatedImpulses, bundleCapacity, bundleCount);
        }
        public override void EnsureCapacity(TypeBatchAllocation typeBatchAllocation)
        {
            var desiredConstraintCapacity = Math.Max(constraintCount, typeBatchAllocation[typeId]);
            if (desiredConstraintCapacity > IndexToHandle.Length)
            {
                InternalResize(typeBatchAllocation, desiredConstraintCapacity);
            }
        }
        public override void Compact(TypeBatchAllocation typeBatchAllocation)
        {
            var desiredConstraintCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(constraintCount, typeBatchAllocation[typeId]));
            if (desiredConstraintCapacity < IndexToHandle.Length)
            {
                InternalResize(typeBatchAllocation, desiredConstraintCapacity);
            }
        }
        public override void Resize(TypeBatchAllocation typeBatchAllocation)
        {
            var desiredConstraintCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(constraintCount, typeBatchAllocation[typeId]));
            if (desiredConstraintCapacity != IndexToHandle.Length)
            {
                InternalResize(typeBatchAllocation, desiredConstraintCapacity);
            }
        }
        public override void Dispose(BufferPool bufferPool)
        {
            bufferPool.SpecializeFor<TProjection>().Return(ref Projection);
            bufferPool.SpecializeFor<TBodyReferences>().Return(ref BodyReferences);
            bufferPool.SpecializeFor<TPrestepData>().Return(ref PrestepData);
            bufferPool.SpecializeFor<TAccumulatedImpulse>().Return(ref AccumulatedImpulses);
            bufferPool.SpecializeFor<int>().Return(ref IndexToHandle);
            Projection = new Buffer<TProjection>();
            BodyReferences = new Buffer<TBodyReferences>();
            PrestepData = new Buffer<TPrestepData>();
            AccumulatedImpulses = new Buffer<TAccumulatedImpulse>();
            IndexToHandle = new Buffer<int>();
            //While the usual use case for Dispose is returning empty batches to the pool, explicit clears will result in pool returns of nonempty batches.
            //In that case we need to clear the counts to zero.
            bundleCount = 0;
            constraintCount = 0;
        }

        internal sealed override void GetBundleTypeSizes(out int bodyReferencesBundleSize, out int prestepBundleSize, out int accumulatedImpulseBundleSize)
        {
            bodyReferencesBundleSize = Unsafe.SizeOf<TBodyReferences>();
            prestepBundleSize = Unsafe.SizeOf<TPrestepData>();
            accumulatedImpulseBundleSize = Unsafe.SizeOf<TAccumulatedImpulse>();
        }

        protected void RemoveBodyReferences(int bundleIndex, int innerIndex)
        {
            ref var bundle = ref BodyReferences[bundleIndex];
            //This is a little defensive; in the event that the body set actually scales down, you don't want to end up with invalid pointers in some lanes.
            //TODO: This may need to change depending on how we handle kinematic/static/inactive storage and encoding.
            GatherScatter.ClearLane<TBodyReferences, int>(ref bundle, innerIndex);
        }

        public sealed override void UpdateForBodyMemoryMove(int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            //Note that this relies on the bodyreferences memory layout. It uses the stride of vectors to skip to the next body based on the bodyIndexInConstraint.
            ref var bundle = ref Unsafe.As<TBodyReferences, Vector<int>>(ref BodyReferences[constraintBundleIndex]);
            GatherScatter.Get(ref bundle, constraintInnerIndex + bodyIndexInConstraint * Vector<int>.Count) = newBodyLocation;
        }
    }
}
