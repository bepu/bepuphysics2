using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities.Collections;
using BepuUtilities;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Superclass of constraint type batch processors. Responsible for interpreting raw type batches for the purposes of bookkeeping and solving.
    /// </summary>
    /// <remarks>
    /// <para>This class holds no actual state of its own. A solver creates a unique type processor for each registered constraint type, and all instances are held in untyped memory.
    /// Splitting the functionality from the data allows for far fewer GC-tracked instances and allows the raw data layout to be shared more easily.</para>
    /// <para>For example, sleeping simulation islands store type batches, but they are created and used differently- and for convenience, they are stored on a per-island basis.
    /// Using the same system but with reference type TypeBatches, tens of thousands of inactive islands would imply tens of thousands of GC-tracked objects.</para>
    /// That's not acceptable, so here we are. 
    /// <para>Conceptually, you can think of the solver's array of TypeProcessors like C function pointers.</para>
    /// </remarks>
    public abstract class TypeProcessor
    {
        //TODO: Having this in the base class actually complicates the implementation of some special constraint types. Consider an 'articulation' subsolver that involves
        //N bodies, for N > Vector<float>.Count * 2. You may want to do SIMD internally in such a case, so there would be no 'bundles' at this level. Worry about that later.

        //We cache type id and bodies per constraint to avoid virtual calls.
        protected int typeId;
        protected int bodiesPerConstraint;
        public int TypeId { get { return typeId; } }
        /// <summary>
        /// Gets the number of bodies associated with each constraint in this type processor.
        /// </summary>
        public int BodiesPerConstraint { get { return bodiesPerConstraint; } }
        /// <summary>
        /// Gets the number of degrees of freedom that each constraint in this type processor constrains. Equal to the number of entries in the accumulated impulses.
        /// </summary>
        public int ConstrainedDegreesOfFreedom { get; private set; }
        protected abstract int InternalBodiesPerConstraint { get; }
        protected abstract int InternalConstrainedDegreesOfFreedom { get; }

        public void Initialize(int typeId)
        {
            this.typeId = typeId;
            this.bodiesPerConstraint = InternalBodiesPerConstraint;
            this.ConstrainedDegreesOfFreedom = InternalConstrainedDegreesOfFreedom;
        }

        /// <summary>
        /// Allocates a slot in the batch, assuming the batch is not a fallback batch.
        /// </summary>
        /// <param name="typeBatch">Type batch to allocate in.</param>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <param name="encodedBodyIndices">List of body indices (not handles!) with count equal to the type batch's expected number of involved bodies.</param>
        /// <param name="pool">Allocation provider to use if the type batch has to be resized.</param>
        /// <returns>Index of the slot in the batch.</returns>
        public unsafe abstract int AllocateInTypeBatch(ref TypeBatch typeBatch, ConstraintHandle handle, Span<int> encodedBodyIndices, BufferPool pool);

        /// <summary>
        /// Allocates a slot in the batch, assuming the batch is a fallback batch.
        /// </summary>
        /// <param name="typeBatch">Type batch to allocate in.</param>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <param name="encodedBodyIndices">List of body indices (not handles!) with count equal to the type batch's expected number of involved bodies.</param>
        /// <param name="pool">Allocation provider to use if the type batch has to be resized.</param>
        /// <returns>Index of the slot in the batch.</returns>
        public unsafe abstract int AllocateInTypeBatchForFallback(ref TypeBatch typeBatch, ConstraintHandle handle, Span<int> encodedBodyIndices, BufferPool pool);
        public abstract void Remove(ref TypeBatch typeBatch, int index, ref Buffer<ConstraintLocation> handlesToConstraints, bool isFallback);


        /// <summary>
        /// Collects body references from active constraints and converts them into properly flagged constraint kinematic body handles.
        /// </summary>
        unsafe struct ActiveKinematicFlaggedBodyHandleCollector : IForEach<int>
        {
            public Bodies Bodies;
            public int* DynamicBodyHandles;
            public int DynamicCount;
            public int* EncodedBodyIndices;
            public int IndexCount;


            public ActiveKinematicFlaggedBodyHandleCollector(Bodies bodies, int* dynamicHandles, int* encodedBodyIndices)
            {
                Bodies = bodies;
                DynamicBodyHandles = dynamicHandles;
                DynamicCount = 0;
                EncodedBodyIndices = encodedBodyIndices;
                IndexCount = 0;
            }

            public void LoopBody(int encodedBodyIndex)
            {
                if (Bodies.IsEncodedDynamicReference(encodedBodyIndex))
                {
                    DynamicBodyHandles[DynamicCount++] = Bodies.ActiveSet.IndexToHandle[encodedBodyIndex].Value;
                }
                EncodedBodyIndices[IndexCount++] = encodedBodyIndex;
            }
        }
        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceTypeBatch">Source type batch to transfer the constraint out of.</param>
        /// <param name="sourceBatchIndex">Index of the batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        public unsafe void TransferConstraint(ref TypeBatch sourceTypeBatch, int sourceBatchIndex, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex)
        {
            int bodiesPerConstraint = InternalBodiesPerConstraint;
            var dynamicBodyHandles = stackalloc int[bodiesPerConstraint];
            var encodedBodyIndices = stackalloc int[bodiesPerConstraint];
            var bodyHandleCollector = new ActiveKinematicFlaggedBodyHandleCollector(bodies, dynamicBodyHandles, encodedBodyIndices);
            solver.EnumerateConnectedRawBodyReferences(ref sourceTypeBatch, indexInTypeBatch, ref bodyHandleCollector);
            var constraintHandle = sourceTypeBatch.IndexToHandle[indexInTypeBatch];
            TransferConstraint(ref sourceTypeBatch, sourceBatchIndex, indexInTypeBatch, solver, bodies, targetBatchIndex, new Span<BodyHandle>(dynamicBodyHandles, bodyHandleCollector.DynamicCount), new Span<int>(encodedBodyIndices, bodiesPerConstraint));
        }

        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceTypeBatch">Source type batch to transfer the constraint out of.</param>
        /// <param name="sourceBatchIndex">Index of the batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        /// <param name="dynamicBodyHandles">Set of body handles in the constraint referring to dynamic bodies.</param>
        /// <param name="encodedBodyIndices">Set of encoded body indices to use in the new constraint allocation.</param>
        public unsafe abstract void TransferConstraint(ref TypeBatch sourceTypeBatch, int sourceBatchIndex, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex, Span<BodyHandle> dynamicBodyHandles, Span<int> encodedBodyIndices);

        [Conditional("DEBUG")]
        protected abstract void ValidateAccumulatedImpulsesSizeInBytes(int sizeInBytes);
        public unsafe void EnumerateAccumulatedImpulses<TEnumerator>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<float>
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var bundleIndex, out var innerIndex);
            var bundleSizeInFloats = ConstrainedDegreesOfFreedom * Vector<float>.Count;
            ValidateAccumulatedImpulsesSizeInBytes(bundleSizeInFloats * 4);
            var impulseAddress = (float*)typeBatch.AccumulatedImpulses.Memory + (bundleIndex * bundleSizeInFloats + innerIndex);
            enumerator.LoopBody(*impulseAddress);
            for (int i = 1; i < ConstrainedDegreesOfFreedom; ++i)
            {
                impulseAddress += Vector<float>.Count;
                enumerator.LoopBody(*impulseAddress);
            }
        }
        public abstract void ScaleAccumulatedImpulses(ref TypeBatch typeBatch, float scale);

        /// <summary>
        /// Updates a type batch's body index references for the movement of a body in memory.
        /// </summary>
        /// <param name="typeBatch">Type batch containing a constraint that references the body.</param>
        /// <param name="indexInTypeBatch">Index of the constraint in the type batch.</param>
        /// <param name="bodyIndexInConstraint">Index within the constraint of the body.</param>
        /// <param name="newBodyLocation">New index of the body in the bodies active set.</param>
        /// <returns>True if the body being moved was kinematic according to the constraint's reference.</returns>
        public abstract bool UpdateForBodyMemoryMove(ref TypeBatch typeBatch, int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation);

        public abstract void Scramble(ref TypeBatch typeBatch, Random random, ref Buffer<ConstraintLocation> handlesToConstraints);

        internal abstract void GetBundleTypeSizes(out int bodyReferencesBundleSize, out int prestepBundleSize, out int accumulatedImpulseBundleSize);

        internal abstract void GenerateSortKeysAndCopyReferences(
            ref TypeBatch typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref Buffer<byte> bodyReferencesCache);

        internal abstract void CopyToCache(
            ref TypeBatch typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref Buffer<ConstraintHandle> indexToHandleCache, ref Buffer<byte> prestepCache, ref Buffer<byte> accumulatedImpulsesCache);

        internal abstract void Regather(
            ref TypeBatch typeBatch,
            int constraintStart, int constraintCount, ref int firstSourceIndex,
            ref Buffer<ConstraintHandle> indexToHandleCache, ref Buffer<byte> bodyReferencesCache, ref Buffer<byte> prestepCache, ref Buffer<byte> accumulatedImpulsesCache,
            ref Buffer<ConstraintLocation> handlesToConstraints);

        internal unsafe abstract void GatherActiveConstraints(Bodies bodies, Solver solver, ref QuickList<ConstraintHandle> sourceHandles, int startIndex, int endIndex, ref TypeBatch targetTypeBatch);

        internal unsafe abstract void AddSleepingToActiveForFallback(
            int sourceSet, int sourceTypeBatchIndex, int targetTypeBatchIndex, Bodies bodies, Solver solver);

        internal unsafe abstract void CopySleepingToActive(
            int sourceSet, int sourceBatchIndex, int sourceTypeBatchIndex, int targetTypeBatchIndex,
            int sourceStart, int targetStart, int count, Bodies bodies, Solver solver);


        internal unsafe abstract void AddWakingBodyHandlesToBatchReferences(ref TypeBatch typeBatch, ref IndexSet targetBatchReferencedHandles);

        [Conditional("DEBUG")]
        internal abstract void VerifySortRegion(ref TypeBatch typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices);
        internal abstract int GetBodyReferenceCount(ref TypeBatch typeBatch, int body);

        public abstract void Initialize(ref TypeBatch typeBatch, int initialCapacity, BufferPool pool);
        public abstract void Resize(ref TypeBatch typeBatch, int newCapacity, BufferPool pool);

        public abstract void WarmStart<TIntegratorCallbacks, TBatchIntegrationMode, TAllowPoseIntegration>(ref TypeBatch typeBatch, ref Buffer<IndexSet> integrationFlags, Bodies bodies,
            ref TIntegratorCallbacks poseIntegratorCallbacks,
            float dt, float inverseDt, int startBundle, int exclusiveEndBundle, int workerIndex)
            where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
            where TBatchIntegrationMode : unmanaged, IBatchIntegrationMode
            where TAllowPoseIntegration : unmanaged, IBatchPoseIntegrationAllowed;
        public abstract void Solve(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle);

        /// <summary>
        /// Gets whether this type requires incremental updates for each substep in a frame beyond the first.
        /// </summary>
        public abstract bool RequiresIncrementalSubstepUpdates { get; }
        public virtual void IncrementallyUpdateForSubstep(ref TypeBatch typeBatch, Bodies bodies, float dt, float inverseDt, int startBundle, int end)
        {
            Debug.Fail("An incremental update was scheduled for a type batch that does not have a contact data update implementation.");
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static int GetCountInBundle(ref TypeBatch typeBatch, int bundleStartIndex)
        {
            //TODO: May want to check codegen on this. Min vs explicit branch. Theoretically, it could do this branchlessly...
            return Math.Min(Vector<float>.Count, typeBatch.ConstraintCount - (bundleStartIndex << BundleIndexing.VectorShift));
        }

    }

    /// <summary>
    /// Defines a function that creates a sort key from body references in a type batch. Used by constraint layout optimization.
    /// </summary>
    public interface ISortKeyGenerator<TBodyReferences> where TBodyReferences : unmanaged
    {
        int GetSortKey(int constraintIndex, ref Buffer<TBodyReferences> bodyReferences);
    }

    //Note that the only reason to have generics at the type level here is to avoid the need to specify them for each individual function. It's functionally equivalent, but this just
    //cuts down on the syntax noise a little bit. 
    //Really, you could use a bunch of composed static generic helpers.
    public abstract class TypeProcessor<TBodyReferences, TPrestepData, TAccumulatedImpulse> : TypeProcessor where TBodyReferences : unmanaged where TPrestepData : unmanaged where TAccumulatedImpulse : unmanaged
    {
        protected override int InternalConstrainedDegreesOfFreedom
        {
            get
            {
                //We're making an assumption about the layout of memory here. It's not guaranteed to be valid, but it does happen to be for all existing and planned constraints.
                var dofCount = Unsafe.SizeOf<TAccumulatedImpulse>() / (4 * Vector<float>.Count);
                Debug.Assert(dofCount * 4 * Vector<float>.Count == Unsafe.SizeOf<TAccumulatedImpulse>(), "One of the assumptions of this DOF calculator is broken. Fix this!");
                return dofCount;
            }
        }
        protected override void ValidateAccumulatedImpulsesSizeInBytes(int sizeInBytes)
        {
            Debug.Assert(sizeInBytes == Unsafe.SizeOf<TAccumulatedImpulse>(), "Your assumptions about memory layout and size are wrong for this type! Fix it!");
        }

        public override unsafe void ScaleAccumulatedImpulses(ref TypeBatch typeBatch, float scale)
        {
            var dofCount = Unsafe.SizeOf<TAccumulatedImpulse>() / Unsafe.SizeOf<Vector<float>>();
            var broadcastedScale = new Vector<float>(scale);
            ref var impulsesBase = ref Unsafe.AsRef<Vector<float>>(typeBatch.AccumulatedImpulses.Memory);
            for (int i = 0; i < dofCount; ++i)
            {
                Unsafe.Add(ref impulsesBase, i) *= broadcastedScale;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void SetBodyReferencesLane(ref TBodyReferences bundle, int innerIndex, Span<int> bodyIndices)
        {
            //We assume that the body references struct is organized in memory like Bundle0, Inner0, ... BundleN, InnerN, Count
            //Assuming contiguous storage, Count is then located at start + stride * BodyCount.
            ref var start = ref Unsafe.As<TBodyReferences, int>(ref bundle);
            ref var targetLane = ref Unsafe.Add(ref start, innerIndex);
            targetLane = bodyIndices[0];
            for (int i = 0; i < bodyIndices.Length; ++i)
            {
                Unsafe.Add(ref targetLane, i * Vector<int>.Count) = bodyIndices[i];
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void AddBodyReferencesLane(ref TBodyReferences bundle, int innerIndex, Span<int> bodyIndices)
        {
            //The jit should be able to fold almost all of the size-related calculations and address fiddling.
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / (Vector<int>.Count * sizeof(int));
            if (innerIndex == 0)
            {
                //This constraint is the first one in a new bundle; set all body references in the constraint to -1 to mean 'no constraint allocated'.
                var negativeOne = new Vector<int>(-1);
                ref var bodyReferenceBundle = ref Unsafe.As<TBodyReferences, Vector<int>>(ref bundle);
                for (int i = 0; i < bodyCount; ++i)
                {
                    Unsafe.Add(ref bodyReferenceBundle, i) = negativeOne;
                }
            }
            SetBodyReferencesLane(ref bundle, innerIndex, bodyIndices);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void RemoveBodyReferencesLane(ref TBodyReferences bundle, int innerIndex)
        {
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / (Vector<int>.Count * sizeof(int));
            ref var start = ref Unsafe.As<TBodyReferences, int>(ref bundle);
            ref var targetLane = ref Unsafe.Add(ref start, innerIndex);
            targetLane = -1;
            for (int i = 1; i < bodyCount; ++i)
            {
                Unsafe.Add(ref targetLane, i * Vector<int>.Count) = -1;
            }
        }


        public unsafe sealed override int AllocateInTypeBatch(ref TypeBatch typeBatch, ConstraintHandle handle, Span<int> bodyIndices, BufferPool pool)
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
            Debug.Assert(typeBatch.BodyReferences.Length >= bundleCount * Unsafe.SizeOf<TBodyReferences>());
            Debug.Assert(typeBatch.AccumulatedImpulses.Length >= bundleCount * Unsafe.SizeOf<TAccumulatedImpulse>());
            Debug.Assert(typeBatch.IndexToHandle.Length >= typeBatch.ConstraintCount);
            return index;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe static bool AllowFallbackBundleAllocation(ref TBodyReferences bundle, Vector<int>* broadcastedBodyIndices)
        {
            //TODO: depending on codegen, there's a chance that doing special cases for the 1, 2, 3, and 4 body cases would be worth it. No need for loop jumps and such.
            //The type batches are always held in pinned memory, this is not a GC hole.
            var bundleBodyIndices = (Vector<int>*)Unsafe.AsPointer(ref bundle);
            var bodiesPerConstraint = Unsafe.SizeOf<TBodyReferences>() / Unsafe.SizeOf<Vector<int>>(); //redundant, but folds.
            for (int broadcastedBundleBodyInConstraint = 0; broadcastedBundleBodyInConstraint < bodiesPerConstraint; ++broadcastedBundleBodyInConstraint)
            {
                var broadcastedBodies = broadcastedBodyIndices[broadcastedBundleBodyInConstraint];
                for (int bundleBodyIndexInConstraint = 0; bundleBodyIndexInConstraint < bodiesPerConstraint; ++bundleBodyIndexInConstraint)
                {
                    //Note that the broadcastedBodies were created with the kinematic flag stripped, so when comparing against the constraint-held references, they will never return true.
                    //This means that kinematics can appear more than once in a single bundle, which is what we want. Kinematics can appear multiple times in batches, too.
                    if (Vector.EqualsAny(bundleBodyIndices[bundleBodyIndexInConstraint], broadcastedBodies))
                    {
                        return false;
                    }
                }
            }
            //The new allocation is not blocked by matching indices, but is there room? At least one slot would have to have -1s in it.
            return Vector.LessThanAny(*bundleBodyIndices, Vector<int>.Zero);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe static int GetInnerIndexForFallbackAllocation(ref TBodyReferences bundle)
        {
            //The type batches are always held in pinned memory, this is not a GC hole.
            var bundleBodyIndices = Unsafe.As<TBodyReferences, Vector<int>>(ref bundle);
            //Choose the first empty slot as the allocation target. This requires picking the lowest index lane that contains -1.
            return BundleIndexing.GetFirstSetLaneIndex(Vector.LessThan(bundleBodyIndices, Vector<int>.Zero));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe static bool ProbeBundleForFallback(Buffer<TBodyReferences> typeBatchBodyIndices, Vector<int>* broadcastedBodyIndices, Span<int> encodedBodyIndices, int bundleIndex, ref int targetBundleIndex, ref int targetInnerIndex)
        {
            ref var bundle = ref typeBatchBodyIndices[bundleIndex];
            if (AllowFallbackBundleAllocation(ref bundle, broadcastedBodyIndices))
            {
                //We've found a place to put the allocation.
                targetBundleIndex = bundleIndex;
                targetInnerIndex = GetInnerIndexForFallbackAllocation(ref bundle);
                SetBodyReferencesLane(ref bundle, targetInnerIndex, encodedBodyIndices);
                return true;
            }
            return false;
        }

        [Conditional("DEBUG")]
        void ValidateEmptyFallbackSlots(ref TypeBatch typeBatch)
        {
            for (int i = 0; i < typeBatch.ConstraintCount; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                var laneIsEmpty = Unsafe.As<TBodyReferences, Vector<int>>(ref typeBatch.BodyReferences.As<TBodyReferences>()[bundleIndex])[innerIndex] == -1;
                Debug.Assert((typeBatch.IndexToHandle[i].Value == -1) == laneIsEmpty);
            }
        }

        [Conditional("DEBUG")]
        void ValidateFallbackAccessSafety(ref TypeBatch typeBatch, int bodiesPerConstraint)
        {
            var bodyReferencesBundleSize = Unsafe.SizeOf<Vector<int>>() * bodiesPerConstraint;
            for (int bundleIndex = 0; bundleIndex < typeBatch.BundleCount; ++bundleIndex)
            {
                ref var bodyReferenceForFirstBody = ref Unsafe.As<byte, Vector<int>>(ref typeBatch.BodyReferences[bundleIndex * bodyReferencesBundleSize]);
                for (int sourceBodyIndexInConstraint = 0; sourceBodyIndexInConstraint < bodiesPerConstraint; ++sourceBodyIndexInConstraint)
                {
                    var occupiedLaneMask = Vector.GreaterThanOrEqual(bodyReferenceForFirstBody, Vector<int>.Zero);
                    var occupiedLaneCountInBundle = 0;
                    for (int i = 0; i < Vector<int>.Count; ++i)
                    {
                        if (occupiedLaneMask[i] < 0)
                            ++occupiedLaneCountInBundle;
                    }
                    Debug.Assert(occupiedLaneCountInBundle > 0, "For any bundle in the [0, BundleCount) interval, there must be at least one occupied lane.");
                    var bodyReferencesForSource = Unsafe.Add(ref bodyReferenceForFirstBody, sourceBodyIndexInConstraint);
                    for (int innerIndex = 0; innerIndex < Vector<int>.Count; ++innerIndex)
                    {
                        var index = bodyReferencesForSource[innerIndex];
                        if (index >= 0)
                        {
                            var broadcasted = new Vector<int>(bodyReferencesForSource[innerIndex]);
                            int matchesTotal = 0;
                            for (int targetBodyIndexInConstraint = 0; targetBodyIndexInConstraint < bodiesPerConstraint; ++targetBodyIndexInConstraint)
                            {
                                var bodyReferencesForTarget = Unsafe.Add(ref bodyReferenceForFirstBody, targetBodyIndexInConstraint);
                                var matchesInLane = -Vector.Dot(Vector.Equals(broadcasted, bodyReferencesForTarget), Vector<int>.One);
                                matchesTotal += matchesInLane;
                            }
                            Debug.Assert(matchesTotal == 1, "A body reference should occur no more than once in any constraint bundle.");
                        }
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        void ValidateAccumulatedImpulses(ref TypeBatch typeBatch)
        {
            var dofCount = Unsafe.SizeOf<TAccumulatedImpulse>() / Unsafe.SizeOf<Vector<float>>();
            for (int i = 0; i < typeBatch.BundleCount; ++i)
            {
                var impulseBundle = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>()[i];
                ref var impulses = ref Unsafe.As<TAccumulatedImpulse, Vector<float>>(ref impulseBundle);
                var mask = Vector.GreaterThanOrEqual(Unsafe.As<TBodyReferences, Vector<int>>(ref typeBatch.BodyReferences.As<TBodyReferences>()[i]), Vector<int>.Zero);
                for (int dofIndex = 0; dofIndex < dofCount; ++dofIndex)
                {
                    var impulsesForDOF = Unsafe.Add(ref impulses, dofIndex);
                    impulsesForDOF.Validate(mask);
                }
            }
        }

        public unsafe sealed override int AllocateInTypeBatchForFallback(ref TypeBatch typeBatch, ConstraintHandle handle, Span<int> encodedBodyIndices, BufferPool pool)
        {
            //This folds.
            var bodiesPerConstraint = Unsafe.SizeOf<TBodyReferences>() / Unsafe.SizeOf<Vector<int>>();
            //ValidateEmptyFallbackSlots(ref typeBatch);
            //ValidateFallbackAccessSafety(ref typeBatch, bodiesPerConstraint);
            //ValidateAccumulatedImpulses(ref typeBatch);
            Debug.Assert(typeBatch.BodyReferences.Allocated, "Should initialize the batch before allocating anything from it.");
            if (typeBatch.ConstraintCount == typeBatch.IndexToHandle.Length)
            {
                Debug.Assert(pool != null, "Looks like a user that doesn't have access to a pool (the awakener, probably?) tried to add a constraint without preallocating enough room.");
                //This isn't technically required (since probing might find an earlier slot), but it makes things simpler and rarely allocates more than necessary.
                InternalResize(ref typeBatch, pool, typeBatch.ConstraintCount * 2);
            }
            //The sequential fallback batch has different allocation rules.
            //Allocation must guarantee that a constraint does not fall into a bundle which shares any of the same body references.
            //(That responsibility usually falls on batch referenced handles blocking new constraints,
            //but the fallback exists precisely because the simulation is asking for pathological numbers of constraints affecting the same body.)
            const int probeLocationCount = 16;
            //Note that this only ever executes for the active set, so body references are indices.
            var typeBatchBodyIndices = typeBatch.BodyReferences.As<TBodyReferences>();
            int targetBundleIndex = -1;
            int targetInnerIndex = -1;
            var broadcastedBodyIndices = stackalloc Vector<int>[bodiesPerConstraint];
            for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < encodedBodyIndices.Length; ++bodyIndexInConstraint)
            {
                //Note that the broadcastedBodies are created with the kinematic flag stripped, so when comparing against the constraint-held references, they will never return true.
                //This means that kinematics can appear more than once in a single bundle, which is what we want. Kinematics can appear multiple times in batches, too.
                broadcastedBodyIndices[bodyIndexInConstraint] = new Vector<int>(encodedBodyIndices[bodyIndexInConstraint] & Bodies.BodyReferenceMask);
            }
            var bundleCount = typeBatch.BundleCount;
            if (bundleCount <= probeLocationCount + 1) //(we always probe the last bundle)
            {
                //The fallback batch is small; there's no need to do a stochastic insertion. Just enumerate all bundles.
                for (int bundleIndexInTypeBatch = 0; bundleIndexInTypeBatch < bundleCount; ++bundleIndexInTypeBatch)
                {
                    if (ProbeBundleForFallback(typeBatchBodyIndices, broadcastedBodyIndices, encodedBodyIndices, bundleIndexInTypeBatch, ref targetBundleIndex, ref targetInnerIndex))
                        break;
                }
            }
            else
            {
                //The fallback batch is large enough to warrant stochastic probing.
                //The idea here is that fallback batches very often involve a single body over and over and over.
                //Scanning every single bundle to find a spot would be needlessly expensive given the most common use case.
                //Instead, we probe a few locations and then give up.
                //First, probe the final bundle just in case it's nice and simple, then do stochastic probes.
                //Stochastic probing works by pseudorandomly choosing a starting point, then picking probe locations based on that starting point.
                var lastBundleIndex = bundleCount - 1;
                if (!ProbeBundleForFallback(typeBatchBodyIndices, broadcastedBodyIndices, encodedBodyIndices, lastBundleIndex, ref targetBundleIndex, ref targetInnerIndex))
                {
                    //No room in the final bundle; keep looking with stochastic probes.
                    var nextProbeIndex = (HashHelper.Rehash(handle.Value) & 0x7FFF_FFFF) % lastBundleIndex;
                    var bundleJump = bundleCount / probeLocationCount;
                    var remainder = lastBundleIndex - bundleJump * probeLocationCount;
                    for (int probeIndex = 0; probeIndex < probeLocationCount; ++probeIndex)
                    {
                        if (ProbeBundleForFallback(typeBatchBodyIndices, broadcastedBodyIndices, encodedBodyIndices, nextProbeIndex, ref targetBundleIndex, ref targetInnerIndex))
                            break;
                        nextProbeIndex += bundleJump;
                        if (probeIndex < remainder)
                            ++nextProbeIndex;
                        if (nextProbeIndex >= bundleCount)
                            nextProbeIndex -= bundleCount;
                    }
                }
            }
            if (targetBundleIndex == -1)
            {
                //None of the existing bundles can hold the constraint; we need a new one.
                var oldCount = typeBatch.ConstraintCount;

                var indexInTypeBatch = bundleCount * Vector<int>.Count;
                var newConstraintCount = indexInTypeBatch + 1;
                if (newConstraintCount >= typeBatch.IndexToHandle.Length)
                {
                    Debug.Assert(pool != null, "Looks like a user that doesn't have access to a pool (the awakener, probably?) tried to add a constraint without preallocating enough room.");
                    InternalResize(ref typeBatch, pool, newConstraintCount * 2);
                }
                typeBatch.ConstraintCount = newConstraintCount;
                typeBatch.IndexToHandle[indexInTypeBatch] = handle;
                ref var bundle = ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, bundleCount);
                AddBodyReferencesLane(ref bundle, 0, encodedBodyIndices);
                //Clear the slot's accumulated impulse. The backing memory could be initialized to any value.
                GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref Buffer<TAccumulatedImpulse>.Get(ref typeBatch.AccumulatedImpulses, bundleCount), 0);
                Debug.Assert(typeBatch.PrestepData.Length >= typeBatch.BundleCount * Unsafe.SizeOf<TPrestepData>());
                Debug.Assert(typeBatch.BodyReferences.Length >= typeBatch.BundleCount * Unsafe.SizeOf<TBodyReferences>());
                Debug.Assert(typeBatch.AccumulatedImpulses.Length >= typeBatch.BundleCount * Unsafe.SizeOf<TAccumulatedImpulse>());
                Debug.Assert(typeBatch.IndexToHandle.Length >= typeBatch.ConstraintCount);

                //Batch compression relies on all unoccupied slots having a IndexToHandle of -1.
                //We've created a new bundle, which means we're responsible for setting all the slots from the previous count to the new count (excluding our just-added constraint!) to -1.
                Debug.Assert(indexInTypeBatch == typeBatch.ConstraintCount - 1);
                for (int i = oldCount; i < indexInTypeBatch; ++i)
                {
                    typeBatch.IndexToHandle[i].Value = -1;
                }
                //ValidateEmptyFallbackSlots(ref typeBatch);
                //ValidateFallbackAccessSafety(ref typeBatch, bodiesPerConstraint);
                //ValidateAccumulatedImpulses(ref typeBatch);
                return indexInTypeBatch;
            }
            else
            {
                //Clear the slot's accumulated impulse. The backing memory could be initialized to any value.
                GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref Buffer<TAccumulatedImpulse>.Get(ref typeBatch.AccumulatedImpulses, targetBundleIndex), targetInnerIndex);
                var indexInTypeBatch = targetBundleIndex * Vector<int>.Count + targetInnerIndex;
                //If the constraint was added after the highest index currently existing constraint, the constraint count needs to be boosted.
                typeBatch.ConstraintCount = Math.Max(indexInTypeBatch + 1, typeBatch.ConstraintCount);
                Debug.Assert(typeBatch.IndexToHandle.Length >= typeBatch.ConstraintCount);
                typeBatch.IndexToHandle[indexInTypeBatch] = handle;
                Debug.Assert(typeBatch.ConstraintCount <= typeBatch.IndexToHandle.Length);
                Debug.Assert(typeBatch.PrestepData.Length >= bundleCount * Unsafe.SizeOf<TPrestepData>());
                Debug.Assert(typeBatch.BodyReferences.Length >= bundleCount * Unsafe.SizeOf<TBodyReferences>());
                Debug.Assert(typeBatch.AccumulatedImpulses.Length >= bundleCount * Unsafe.SizeOf<TAccumulatedImpulse>());
                //ValidateEmptyFallbackSlots(ref typeBatch);
                //ValidateFallbackAccessSafety(ref typeBatch, bodiesPerConstraint);
                //ValidateAccumulatedImpulses(ref typeBatch);
                return indexInTypeBatch;
            }

        }

        /// <summary>
        /// Overwrites all the data in the target constraint slot with source data.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected static void Move(
            ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle, ConstraintHandle sourceHandle,
            int sourceInner,
            ref TBodyReferences targetReferencesBundle, ref TPrestepData targetPrestepBundle, ref TAccumulatedImpulse targetAccumulatedBundle, ref ConstraintHandle targetIndexToHandle,
            int targetInner, int targetIndex, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            //Note that we do NOT copy the iteration data. It is regenerated each frame from scratch. 
            GatherScatter.CopyLane(ref sourceReferencesBundle, sourceInner, ref targetReferencesBundle, targetInner);
            GatherScatter.CopyLane(ref sourcePrestepBundle, sourceInner, ref targetPrestepBundle, targetInner);
            GatherScatter.CopyLane(ref sourceAccumulatedBundle, sourceInner, ref targetAccumulatedBundle, targetInner);
            targetIndexToHandle = sourceHandle;
            handlesToConstraints[sourceHandle.Value].IndexInTypeBatch = targetIndex;
        }



        public sealed override unsafe void Scramble(ref TypeBatch typeBatch, Random random, ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            //This is a pure debug function used to compare cache optimization strategies. Performance doesn't matter. 
            TPrestepData aPrestep = default;
            TAccumulatedImpulse aAccumulated = default;
            TBodyReferences aBodyReferences = default;
            ConstraintHandle aHandle;

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
        /// <param name="typeBatch">Type batch to remove a constraint from.</param>
        /// <param name="index">Index of the constraint to remove.</param>
        /// <param name="handlesToConstraints">The handle to constraint mapping used by the solver that could be modified by a swap on removal.</param>
        /// <param name="isFallback">True if the type batch being removed from belongs to the fallback batch, false otherwise.</param>
        public override unsafe void Remove(ref TypeBatch typeBatch, int index, ref Buffer<ConstraintLocation> handlesToConstraints, bool isFallback)
        {
            Debug.Assert(index >= 0 && index < typeBatch.ConstraintCount, "Can only remove elements that are actually in the batch!");
            if (isFallback)
            {
                //ValidateEmptyFallbackSlots(ref typeBatch);
                //ValidateFallbackAccessSafety(ref typeBatch, bodiesPerConstraint);
                //ValidateAccumulatedImpulses(ref typeBatch);
                //The fallback batch does not guarantee contiguity of constraints, only contiguity of *bundles*.
                //Bundles may be incomplete.
                //We must guarantee that a bundle never contains references to the same body more than once.
                //So, it's not safe to simply pull the last constraint into the removed slot.
                //Instead, remove the constraint from whatever bundle it's in, and if it is empty afterwards, pull the whole last bundle into its position.
                BundleIndexing.GetBundleIndices(index, out var removedBundleIndex, out var removedInnerIndex);
                var bodyReferences = typeBatch.BodyReferences.As<TBodyReferences>();
                ref var removedBundleSlot = ref bodyReferences[removedBundleIndex];
                //Batch compression relies on unused constraints in the [0, ConstraintCount) interval having their handles pointing to -1.
                typeBatch.IndexToHandle[index].Value = -1;
                RemoveBodyReferencesLane(ref removedBundleSlot, removedInnerIndex);
                var firstBodyReferences = Unsafe.As<TBodyReferences, Vector<int>>(ref removedBundleSlot);
                if (Vector.LessThanAll(firstBodyReferences, Vector<int>.Zero))
                {
                    //All slots in the bundle are now empty; this bundle should be removed.
                    var lastBundleIndex = typeBatch.BundleCount - 1;
                    if (removedBundleIndex != lastBundleIndex)
                    {
                        //There is a bundle to move into the now-dead bundle slot.
                        var prestepData = typeBatch.PrestepData.As<TPrestepData>();
                        var accumulatedImpulses = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
                        prestepData[removedBundleIndex] = prestepData[lastBundleIndex];
                        accumulatedImpulses[removedBundleIndex] = accumulatedImpulses[lastBundleIndex];
                        bodyReferences[removedBundleIndex] = bodyReferences[lastBundleIndex];
                        var firstBodyLaneForMovedBundle = (int*)Unsafe.AsPointer(ref bodyReferences[lastBundleIndex]);
                        //Update all constraint locations for the move.
                        var constraintIndexShift = (lastBundleIndex - removedBundleIndex) * Vector<int>.Count;
                        var bundleStartIndexInConstraints = lastBundleIndex * Vector<int>.Count;
                        for (int i = 0; i < Vector<int>.Count; ++i)
                        {
                            if (firstBodyLaneForMovedBundle[i] >= 0)
                            {
                                //This constraint actually exists.
                                var constraintIndex = bundleStartIndexInConstraints + i;
                                var newConstraintIndex = constraintIndex - constraintIndexShift;
                                var handle = typeBatch.IndexToHandle[constraintIndex];
                                typeBatch.IndexToHandle[constraintIndex].Value = -1; //Removed handles should be set to -1.
                                typeBatch.IndexToHandle[newConstraintIndex] = handle;
                                handlesToConstraints[handle.Value].IndexInTypeBatch = newConstraintIndex;
                            }
                        }
                        //Removed the last bundle index, so drop back by one.
                        --lastBundleIndex;
                    }
                    //Calculate the new constraint count by getting the highest index in the new last bundle.
                    var innerLaneCount = BundleIndexing.GetLastSetLaneCount(Vector.GreaterThanOrEqual(Unsafe.As<TBodyReferences, Vector<int>>(ref bodyReferences[lastBundleIndex]), Vector<int>.Zero));
                    typeBatch.ConstraintCount = lastBundleIndex * Vector<int>.Count + innerLaneCount;

                    //ValidateEmptyFallbackSlots(ref typeBatch);
                    //ValidateFallbackAccessSafety(ref typeBatch, bodiesPerConstraint);
                    //ValidateAccumulatedImpulses(ref typeBatch);
                }
            }
            else
            {
                var lastIndex = typeBatch.ConstraintCount - 1;
                typeBatch.ConstraintCount = lastIndex;
                BundleIndexing.GetBundleIndices(lastIndex, out var sourceBundleIndex, out var sourceInnerIndex);

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
                        ref typeBatch.IndexToHandle[index], targetInnerIndex, index,
                        ref handlesToConstraints);
                }
                //Clear the now-empty last slot of the body references bundle.
                RemoveBodyReferencesLane(ref Unsafe.Add(ref bodyReferences, sourceBundleIndex), sourceInnerIndex);
            }
        }



        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceTypeBatch">Source type batch to transfer the constraint out of.</param>
        /// <param name="sourceBatchIndex">Index of the batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        /// <param name="dynamicBodyHandles">Set of body handles in the constraint referring to dynamic bodies.</param>
        /// <param name="encodedBodyIndices">Set of encoded body indices to use in the new constraint allocation.</param>
        public unsafe override sealed void TransferConstraint(ref TypeBatch sourceTypeBatch, int sourceBatchIndex, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex, Span<BodyHandle> dynamicBodyHandles, Span<int> encodedBodyIndices)
        {
            var constraintHandle = sourceTypeBatch.IndexToHandle[indexInTypeBatch];
            //Allocate a spot in the new batch. Note that it does not change the Handle->Constraint mapping in the Solver; that's important when we call Solver.Remove below.
            solver.AllocateInBatch(targetBatchIndex, constraintHandle, dynamicBodyHandles, encodedBodyIndices, typeId, out var targetReference);

            BundleIndexing.GetBundleIndices(targetReference.IndexInTypeBatch, out var targetBundle, out var targetInner);
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var sourceBundle, out var sourceInner);
            //We don't pull a description or anything from the old constraint. That would require having a unique mapping from constraint to 'full description'. 
            //Instead, we just directly copy from lane to lane. Note that body references are excluded; AllocateInBatch already took care of setting those values.
            GatherScatter.CopyLane(
                ref Buffer<TPrestepData>.Get(ref sourceTypeBatch.PrestepData, sourceBundle), sourceInner,
                ref Buffer<TPrestepData>.Get(ref targetReference.TypeBatch.PrestepData, targetBundle), targetInner);
            GatherScatter.CopyLane(
                ref Buffer<TAccumulatedImpulse>.Get(ref sourceTypeBatch.AccumulatedImpulses, sourceBundle), sourceInner,
                ref Buffer<TAccumulatedImpulse>.Get(ref targetReference.TypeBatch.AccumulatedImpulses, targetBundle), targetInner);

            //Don't forget to keep the solver's pointers consistent! We bypassed the usual add procedure, so the solver hasn't been notified yet.
            ref var constraintLocation = ref solver.HandleToConstraint[constraintHandle.Value];
            constraintLocation.BatchIndex = targetBatchIndex;
            constraintLocation.IndexInTypeBatch = targetReference.IndexInTypeBatch;
            constraintLocation.TypeId = typeId;
            solver.AssertConstraintHandleExists(constraintHandle);

            //Now we can get rid of the old allocation.
            //Note the use of RemoveFromBatch instead of Remove. Solver.Remove returns the handle to the pool, which we do not want!
            //It may look a bit odd to use a solver-level function here, given that we are operating on batches and handling the solver state directly for the most part. 
            //However, removes can result in empty batches that require resource reclamation. 
            //Rather than reimplementing that we just reuse the solver's version. 
            //That sort of resource cleanup isn't required on add- everything that is needed already exists, and nothing is going away.
            solver.RemoveFromBatch(sourceBatchIndex, typeId, indexInTypeBatch);
        }

        void InternalResize(ref TypeBatch typeBatch, BufferPool pool, int constraintCapacity)
        {
            Debug.Assert(constraintCapacity >= 0, "The constraint capacity should have already been validated.");
            pool.ResizeToAtLeast(ref typeBatch.IndexToHandle, constraintCapacity, typeBatch.ConstraintCount);
            //Note that we construct the bundle capacity from the resized constraint capacity. This means we only have to check the IndexToHandle capacity
            //before allocating, which simplifies things a little bit at the cost of some memory. Could revisit this if memory use is actually a concern.
            var bundleCapacity = BundleIndexing.GetBundleCount(typeBatch.IndexToHandle.Length);
            //Note that the projection is not copied over. It is ephemeral data. (In the same vein as above, if memory is an issue, we could just allocate projections on demand.)
            var bundleCount = typeBatch.BundleCount;
            pool.ResizeToAtLeast(ref typeBatch.BodyReferences, bundleCapacity * Unsafe.SizeOf<TBodyReferences>(), bundleCount * Unsafe.SizeOf<TBodyReferences>());
            pool.ResizeToAtLeast(ref typeBatch.PrestepData, bundleCapacity * Unsafe.SizeOf<TPrestepData>(), bundleCount * Unsafe.SizeOf<TPrestepData>());
            pool.ResizeToAtLeast(ref typeBatch.AccumulatedImpulses, bundleCapacity * Unsafe.SizeOf<TAccumulatedImpulse>(), bundleCount * Unsafe.SizeOf<TAccumulatedImpulse>());
        }

        public override void Initialize(ref TypeBatch typeBatch, int initialCapacity, BufferPool pool)
        {
            //We default-initialize the type batch because the resize checks existing allocation status when deciding how much to copy over. 
            //Technically we could use an initialization-specific version, but it doesn't matter.
            typeBatch = new TypeBatch();
            typeBatch.TypeId = TypeId;
            InternalResize(ref typeBatch, pool, initialCapacity);
        }

        public override void Resize(ref TypeBatch typeBatch, int desiredCapacity, BufferPool pool)
        {
            var desiredConstraintCapacity = BufferPool.GetCapacityForCount<int>(desiredCapacity);
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


        public sealed override bool UpdateForBodyMemoryMove(ref TypeBatch typeBatch, int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            //Note that this relies on the bodyreferences memory layout. It uses the stride of vectors to skip to the next body based on the bodyIndexInConstraint.
            ref var bundle = ref Unsafe.As<TBodyReferences, Vector<int>>(ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, constraintBundleIndex));
            ref var referenceLocation = ref GatherScatter.Get(ref bundle, constraintInnerIndex + bodyIndexInConstraint * Vector<int>.Count);
            //Note that the old kinematic mask is preserved so that the caller doesn't have to requery the object for its kinematicity.
            var isKinematic = Bodies.IsEncodedKinematicReference(referenceLocation);
            referenceLocation = newBodyLocation | (referenceLocation & Bodies.KinematicMask);
            return isKinematic;
        }

        //Note that these next two sort key users require a generic sort key implementation; this avoids virtual dispatch on a per-object level while still sharing the bulk of the logic.
        //Technically, we could force the TSortKeyGenerator to be defined at the generic type level, but this seems a little less... extreme.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void GenerateSortKeysAndCopyReferences<TSortKeyGenerator>(
           ref TypeBatch typeBatch,
           int bundleStart, int localBundleStart, int bundleCount,
           int constraintStart, int localConstraintStart, int constraintCount,
           ref int firstSortKey, ref int firstSourceIndex, ref Buffer<byte> bodyReferencesCache)
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
            bodyReferences.CopyTo(bundleStart, typedBodyReferencesCache, localBundleStart, bundleCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void VerifySortRegion<TSortKeyGenerator>(ref TypeBatch typeBatch, int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
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
                //Note that this assert uses >= and not >; in a synchronized constraint batch, it's impossible for body references to be duplicated, but fallback batches CAN have duplicates.
                Debug.Assert(key >= previousKey, "After the sort and swap completes, all constraints should be in order.");
                Debug.Assert(key == sortedKeys[i], "After the swap goes through, the rederived sort keys should match the previously sorted ones.");
                previousKey = key;

            }
        }

        internal unsafe sealed override void CopyToCache(
            ref TypeBatch typeBatch,
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref Buffer<ConstraintHandle> indexToHandleCache, ref Buffer<byte> prestepCache, ref Buffer<byte> accumulatedImpulsesCache)
        {
            typeBatch.IndexToHandle.CopyTo(constraintStart, indexToHandleCache, localConstraintStart, constraintCount);
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
            ref TypeBatch typeBatch,
            int constraintStart, int constraintCount, ref int firstSourceIndex,
            ref Buffer<ConstraintHandle> indexToHandleCache, ref Buffer<byte> bodyReferencesCache, ref Buffer<byte> prestepCache, ref Buffer<byte> accumulatedImpulsesCache,
            ref Buffer<ConstraintLocation> handlesToConstraints)
        {
            var typedBodyReferencesCache = bodyReferencesCache.As<TBodyReferences>();
            var typedPrestepCache = prestepCache.As<TPrestepData>();
            var typedAccumulatedImpulsesCache = accumulatedImpulsesCache.As<TAccumulatedImpulse>();

            var typedBodyReferencesTarget = typeBatch.BodyReferences.As<TBodyReferences>();
            var typedPrestepTarget = typeBatch.PrestepData.As<TPrestepData>();
            var typedAccumulatedImpulsesTarget = typeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
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

        internal unsafe sealed override void GatherActiveConstraints(Bodies bodies, Solver solver, ref QuickList<ConstraintHandle> sourceHandles, int startIndex, int endIndex, ref TypeBatch targetTypeBatch)
        {
            ref var activeConstraintSet = ref solver.ActiveSet;
            ref var activeBodySet = ref bodies.ActiveSet;
            for (int i = startIndex; i < endIndex; ++i)
            {
                var sourceHandle = sourceHandles[i];
                targetTypeBatch.IndexToHandle[i] = sourceHandle;
                ref var location = ref solver.HandleToConstraint[sourceHandle.Value];
                Debug.Assert(targetTypeBatch.TypeId == location.TypeId, "Can only gather from batches of the same type.");
                Debug.Assert(location.SetIndex == 0, "Can only gather from the active set.");

                ref var sourceBatch = ref activeConstraintSet.Batches[location.BatchIndex];
                ref var sourceTypeBatch = ref sourceBatch.TypeBatches[sourceBatch.TypeIndexToTypeBatchIndex[location.TypeId]];
                BundleIndexing.GetBundleIndices(location.IndexInTypeBatch, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(i, out var targetBundle, out var targetInner);

                //Note that we don't directly copy body references or projection information. Projection information is ephemeral and unnecessary for sleeping constraints.
                //Body references get turned into handles so that awakening can easily track down the bodies' readded locations.
                GatherScatter.CopyLane(
                    ref Buffer<TPrestepData>.Get(ref sourceTypeBatch.PrestepData, sourceBundle), sourceInner,
                    ref Buffer<TPrestepData>.Get(ref targetTypeBatch.PrestepData, targetBundle), targetInner);
                GatherScatter.CopyLane(
                    ref Buffer<TAccumulatedImpulse>.Get(ref sourceTypeBatch.AccumulatedImpulses, sourceBundle), sourceInner,
                    ref Buffer<TAccumulatedImpulse>.Get(ref targetTypeBatch.AccumulatedImpulses, targetBundle), targetInner);
                ref var sourceReferencesLaneStart = ref Unsafe.Add(ref Unsafe.As<TBodyReferences, int>(ref Buffer<TBodyReferences>.Get(ref sourceTypeBatch.BodyReferences, sourceBundle)), sourceInner);
                ref var targetReferencesLaneStart = ref Unsafe.Add(ref Unsafe.As<TBodyReferences, int>(ref Buffer<TBodyReferences>.Get(ref targetTypeBatch.BodyReferences, targetBundle)), targetInner);
                var offset = 0;
                for (int j = 0; j < bodiesPerConstraint; ++j)
                {
                    var encodedBodyIndex = Unsafe.Add(ref sourceReferencesLaneStart, offset);
                    //Note that when we transfer the body reference into the sleeping batch, the body reference turns into a handle- but it preserves the kinematic flag.
                    Unsafe.Add(ref targetReferencesLaneStart, offset) = activeBodySet.IndexToHandle[encodedBodyIndex & Bodies.BodyReferenceMask].Value | (encodedBodyIndex & Bodies.KinematicMask);
                    offset += Vector<int>.Count;
                }
            }
        }


        internal unsafe sealed override void AddSleepingToActiveForFallback(int sourceSet, int sourceTypeBatchIndex, int targetTypeBatchIndex, Bodies bodies, Solver solver)
        {
            //Unlike the bulk copies that the awakener can do for non-fallback batches, we have to do the heavyweight allocations for fallbacks.
            //This arises because sleeping constraint sets do not maintain the 'no constraints refer to the same bodies in a given bundle' rule; everything just got packed together.
            var batchIndex = solver.FallbackBatchThreshold;
            ref var sourceTypeBatch = ref solver.Sets[sourceSet].Batches[batchIndex].TypeBatches[sourceTypeBatchIndex];
            ref var targetTypeBatch = ref solver.ActiveSet.Batches[batchIndex].TypeBatches[targetTypeBatchIndex];
            Debug.Assert(sourceTypeBatch.TypeId == targetTypeBatch.TypeId);
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / Unsafe.SizeOf<Vector<int>>();
            //solver.ValidateSetOwnership(ref sourceTypeBatch, sourceSet);
            //ValidateAccumulatedImpulses(ref targetTypeBatch);
            //ValidateEmptyFallbackSlots(ref targetTypeBatch);
            //ValidateFallbackAccessSafety(ref targetTypeBatch, bodyCount);
            //solver.ValidateConstraintMaps(0, batchIndex, targetTypeBatchIndex);
            //solver.ValidateConstraintMaps(sourceSet, batchIndex, sourceTypeBatchIndex);
            Span<int> bodyIndices = stackalloc int[bodyCount];
            var sourceBundleCount = sourceTypeBatch.BundleCount;
            var sourceBodyReferences = sourceTypeBatch.BodyReferences.As<TBodyReferences>();
            var sourcePrestepData = sourceTypeBatch.PrestepData.As<TPrestepData>();
            var sourceAccumulatedImpulses = sourceTypeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var targetPrestepData = targetTypeBatch.PrestepData.As<TPrestepData>();
            var targetAccumulatedImpulses = targetTypeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
            var bodyHandleToLocation = bodies.HandleToLocation;
            var constraintHandleToLocation = solver.HandleToConstraint;
            for (int bundleIndexInSource = 0; bundleIndexInSource < sourceBundleCount; ++bundleIndexInSource)
            {
                //It's possible that the sleeping fallback entries do not have any repeat entries. In that case, we could bulk copy the bundle into the active set.
                //It's unclear if that's the best option- consider that it would always add a new bundle, but the members of the bundle might be able to be inserted in previous bundles.
                var bundleStartConstraintIndex = bundleIndexInSource * Vector<int>.Count;
                var countInBundle = sourceTypeBatch.ConstraintCount - bundleStartConstraintIndex;
                if (countInBundle > Vector<int>.Count)
                    countInBundle = Vector<int>.Count;
                ref var sourceBodyReferencesBundle = ref sourceBodyReferences[bundleIndexInSource];
                ref var sourceAccumulatedImpulsesBundle = ref sourceAccumulatedImpulses[bundleIndexInSource];
                ref var sourcePrestepBundle = ref sourcePrestepData[bundleIndexInSource];
                for (int sourceInnerIndex = 0; sourceInnerIndex < countInBundle; ++sourceInnerIndex)
                {
                    var sourceIndex = bundleStartConstraintIndex + sourceInnerIndex;
                    ref var bodyReferencesLane = ref Unsafe.As<TBodyReferences, int>(ref GatherScatter.GetOffsetInstance(ref sourceBodyReferencesBundle, sourceInnerIndex));
                    //Note that the sleeping set stores body references as handles, while the active set uses indices. We have to translate here.
                    for (int i = 0; i < bodyIndices.Length; ++i)
                    {
                        //Bodies have already been moved into the active set, so we can use the mapping.
                        var encodedBodyHandleValue = Unsafe.Add(ref bodyReferencesLane, Vector<int>.Count * i);
                        var bodyHandleValue = encodedBodyHandleValue & Bodies.BodyReferenceMask;
                        Debug.Assert(bodyHandleToLocation[bodyHandleValue].SetIndex == 0);
                        //Preserve the kinematic flag when converting from handle to index.
                        bodyIndices[i] = bodyHandleToLocation[bodyHandleValue].Index | (encodedBodyHandleValue & Bodies.KinematicMask);
                    }
                    var handle = sourceTypeBatch.IndexToHandle[sourceIndex];
                    Debug.Assert(constraintHandleToLocation[handle.Value].SetIndex == sourceSet);
                    Debug.Assert(constraintHandleToLocation[handle.Value].IndexInTypeBatch == sourceIndex);
                    Debug.Assert(constraintHandleToLocation[handle.Value].TypeId == sourceTypeBatch.TypeId);
                    Debug.Assert(constraintHandleToLocation[handle.Value].BatchIndex == batchIndex);
                    //Note that we pass null for the buffer pool. The user (awakener) must preallocate worst case room in the type batches ahead of time so that multiple threads can proceed at the same time.
                    var targetIndex = AllocateInTypeBatchForFallback(ref targetTypeBatch, handle, bodyIndices, null);
                    BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

                    GatherScatter.CopyLane(ref sourceAccumulatedImpulsesBundle, sourceInnerIndex, ref targetAccumulatedImpulses[targetBundle], targetInner);
                    GatherScatter.CopyLane(ref sourcePrestepBundle, sourceInnerIndex, ref targetPrestepData[targetBundle], targetInner);
                    ref var location = ref constraintHandleToLocation[handle.Value];
                    location.SetIndex = 0;
                    location.BatchIndex = batchIndex;
                    Debug.Assert(sourceTypeBatch.TypeId == location.TypeId);
                    location.IndexInTypeBatch = targetIndex;
                }
            }
            //ValidateAccumulatedImpulses(ref targetTypeBatch);
            //ValidateEmptyFallbackSlots(ref targetTypeBatch);
            //ValidateFallbackAccessSafety(ref targetTypeBatch, bodyCount);
            //solver.ValidateConstraintMaps(0, batchIndex, targetTypeBatchIndex);
        }

        internal unsafe sealed override void CopySleepingToActive(
            int sourceSet, int batchIndex, int sourceTypeBatchIndex, int targetTypeBatchIndex,
            int sourceStart, int targetStart, int count, Bodies bodies, Solver solver)
        {
            ref var sourceTypeBatch = ref solver.Sets[sourceSet].Batches[batchIndex].TypeBatches[sourceTypeBatchIndex];
            ref var targetTypeBatch = ref solver.ActiveSet.Batches[batchIndex].TypeBatches[targetTypeBatchIndex];
            Debug.Assert(sourceStart >= 0 && sourceStart + count <= sourceTypeBatch.ConstraintCount);
            Debug.Assert(targetStart >= 0 && targetStart + count <= targetTypeBatch.ConstraintCount,
                "This function should only be used when a region has been preallocated within the type batch.");
            Debug.Assert(sourceTypeBatch.TypeId == targetTypeBatch.TypeId);
            //TODO: Note that we give up on a bulk copy very easily here.
            //If you see this showing up in profiling to a meaningful extent, consider doing incomplete copies to allow a central bulk copy.
            //The only reasons that such a thing isn't already implemented are simplicity and time.
            if ((targetStart & BundleIndexing.VectorMask) == 0 &&
                (sourceStart & BundleIndexing.VectorMask) == 0 &&
                ((count & BundleIndexing.VectorMask) == 0 || count == targetTypeBatch.ConstraintCount))
            {
                //We can use a simple bulk copy here.      
                Debug.Assert(count > 0);
                var bundleCount = BundleIndexing.GetBundleCount(count);
                var sourcePrestepData = sourceTypeBatch.PrestepData.As<TPrestepData>();
                var sourceAccumulatedImpulses = sourceTypeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
                var targetPrestepData = targetTypeBatch.PrestepData.As<TPrestepData>();
                var targetAccumulatedImpulses = targetTypeBatch.AccumulatedImpulses.As<TAccumulatedImpulse>();
                var sourceBundleStart = sourceStart >> BundleIndexing.VectorShift;
                var targetBundleStart = targetStart >> BundleIndexing.VectorShift;
                sourcePrestepData.CopyTo(sourceBundleStart, targetPrestepData, targetBundleStart, bundleCount);
                sourceAccumulatedImpulses.CopyTo(sourceBundleStart, targetAccumulatedImpulses, targetBundleStart, bundleCount);
            }
            else
            {
                for (int i = 0; i < count; ++i)
                {
                    //Note that this implementation allows two threads to access a single bundle. That would be a pretty bad case of false sharing if it happens, but
                    //it won't cause correctness problems.
                    var sourceIndex = sourceStart + i;
                    var targetIndex = targetStart + i;
                    BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                    BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);
                    GatherScatter.CopyLane(
                        ref Buffer<TPrestepData>.Get(ref sourceTypeBatch.PrestepData, sourceBundle), sourceInner,
                        ref Buffer<TPrestepData>.Get(ref targetTypeBatch.PrestepData, targetBundle), targetInner);
                    GatherScatter.CopyLane(
                        ref Buffer<TAccumulatedImpulse>.Get(ref sourceTypeBatch.AccumulatedImpulses, sourceBundle), sourceInner,
                        ref Buffer<TAccumulatedImpulse>.Get(ref targetTypeBatch.AccumulatedImpulses, targetBundle), targetInner);
                }
            }
            //Note that body reference copies cannot be done in bulk because inactive constraints refer to body handles while active constraints refer to body indices.
            for (int i = 0; i < count; ++i)
            {
                var sourceIndex = sourceStart + i;
                var targetIndex = targetStart + i;
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

                ref var sourceReferencesLaneStart = ref Unsafe.Add(ref Unsafe.As<TBodyReferences, int>(ref Buffer<TBodyReferences>.Get(ref sourceTypeBatch.BodyReferences, sourceBundle)), sourceInner);
                ref var targetReferencesLaneStart = ref Unsafe.Add(ref Unsafe.As<TBodyReferences, int>(ref Buffer<TBodyReferences>.Get(ref targetTypeBatch.BodyReferences, targetBundle)), targetInner);
                var offset = 0;
                for (int j = 0; j < bodiesPerConstraint; ++j)
                {
                    var encodedBodyHandle = Unsafe.Add(ref sourceReferencesLaneStart, offset);
                    //Note that encoded kinematicity flags are carried over to the active index reference.
                    Unsafe.Add(ref targetReferencesLaneStart, offset) = bodies.HandleToLocation[encodedBodyHandle & Bodies.BodyReferenceMask].Index | (encodedBodyHandle & Bodies.KinematicMask);
                    offset += Vector<int>.Count;
                }
                var constraintHandle = sourceTypeBatch.IndexToHandle[sourceIndex];
                ref var location = ref solver.HandleToConstraint[constraintHandle.Value];
                Debug.Assert(location.SetIndex == sourceSet);
                location.SetIndex = 0;
                location.BatchIndex = batchIndex;
                Debug.Assert(sourceTypeBatch.TypeId == location.TypeId);
                location.IndexInTypeBatch = targetIndex;
                //This could be done with a bulk copy, but eh! We already touched the memory.
                targetTypeBatch.IndexToHandle[targetIndex] = constraintHandle;
            }
        }


        internal unsafe sealed override void AddWakingBodyHandlesToBatchReferences(ref TypeBatch typeBatch, ref IndexSet targetBatchReferencedHandles)
        {
            for (int i = 0; i < typeBatch.ConstraintCount; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var sourceBundle, out var sourceInner);
                ref var sourceHandlesStart = ref Unsafe.Add(ref Unsafe.As<TBodyReferences, int>(ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, sourceBundle)), sourceInner);
                var offset = 0;
                for (int j = 0; j < bodiesPerConstraint; ++j)
                {
                    var encodedBodyHandle = Unsafe.Add(ref sourceHandlesStart, offset);
                    if (Bodies.IsEncodedDynamicReference(encodedBodyHandle))
                    {
                        //Note that only dynamic bodies are added to the batch referenced handles.
                        //Given that we're only adding references to bodies that already exist, and therefore were at some point in the active set, it should never be necessary
                        //to resize the batch referenced handles structure.
                        //Note that this will happily set an existing bit if the target batch is the fallback batch.
                        targetBatchReferencedHandles.SetUnsafely(encodedBodyHandle);
                    }
                    offset += Vector<int>.Count;
                }
            }
        }

        internal override int GetBodyReferenceCount(ref TypeBatch typeBatch, int bodyToFind)
        {
            //This is a pure debug function; performance does not matter.
            //Note that this function is used across both active and inactive sets. In the active set, the body references refer to *indices* in the Bodies.ActiveSet.
            //For inactive constraint sets, the body references are instead body *handles*. The user of this function is expected to appreciate the difference.
            var bundleCount = typeBatch.BundleCount;
            var bodyReferences = typeBatch.BodyReferences.As<TBodyReferences>();
            int count = 0;
            var bodiesPerConstraint = InternalBodiesPerConstraint;
            for (int bundleIndex = 0; bundleIndex < bundleCount; ++bundleIndex)
            {
                var bundleSize = Math.Min(Vector<float>.Count, typeBatch.ConstraintCount - (bundleIndex << BundleIndexing.VectorShift));
                ref var bundleBase = ref Unsafe.As<TBodyReferences, Vector<int>>(ref bodyReferences[bundleIndex]);
                for (int constraintBodyIndex = 0; constraintBodyIndex < bodiesPerConstraint; ++constraintBodyIndex)
                {
                    ref var bodyVectorBase = ref Unsafe.As<Vector<int>, int>(ref Unsafe.Add(ref bundleBase, constraintBodyIndex));
                    for (int innerIndex = 0; innerIndex < bundleSize; ++innerIndex)
                    {
                        if (Unsafe.Add(ref bodyVectorBase, innerIndex) == bodyToFind)
                            ++count;
                    }
                }
            }
            return count;
        }


        public enum BundleIntegrationMode
        {
            None = 0,
            Partial = 1,
            All = 2
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BundleIntegrationMode BundleShouldIntegrate(int bundleIndex, in IndexSet integrationFlags, out Vector<int> integrationMask)
        {
            Debug.Assert(Vector<float>.Count <= 32, "Wait, what? The integration mask isn't big enough to handle a vector this big.");
            var constraintStartIndex = bundleIndex * Vector<float>.Count;
            var flagBundleIndex = constraintStartIndex >> 6;
            var flagInnerIndex = constraintStartIndex - (flagBundleIndex << 6);
            var flagMask = (1 << Vector<float>.Count) - 1;
            var scalarIntegrationMask = ((int)(integrationFlags.Flags[flagBundleIndex] >> flagInnerIndex)) & flagMask;
            if (scalarIntegrationMask == flagMask)
            {
                //No need to carefully expand a bitstring into a vector mask if we know that a single broadcast will suffice.
                integrationMask = new Vector<int>(-1);
                return BundleIntegrationMode.All;
            }
            else if (scalarIntegrationMask > 0)
            {
                if (Vector<int>.Count == 4 || Vector<int>.Count == 8)
                {
                    Vector<int> selectors;
                    if (Vector<int>.Count == 8)
                    {
                        selectors = Vector256.Create(1, 2, 4, 8, 16, 32, 64, 128).AsVector();
                    }
                    else
                    {
                        selectors = Vector128.Create(1, 2, 4, 8).AsVector();
                    }
                    var scalarBroadcast = new Vector<int>(scalarIntegrationMask);
                    var selected = Vector.BitwiseAnd(selectors, scalarBroadcast);
                    integrationMask = Vector.Equals(selected, selectors);
                }
                else
                {
                    //This is not a good implementation, but I don't know of any target platforms that will hit this.
                    //TODO: AVX512 being enabled by the runtime could force this path to be taken; it'll require an update!
                    Debug.Assert(Vector<int>.Count <= 8, "The vector path assumes that AVX512 is not supported, so this is hitting a fallback path.");
                    Span<int> mask = stackalloc int[Vector<int>.Count];
                    for (int i = 0; i < Vector<int>.Count; ++i)
                    {
                        mask[i] = (scalarIntegrationMask & (1 << i)) > 0 ? -1 : 0;
                    }
                    integrationMask = new Vector<int>(mask);
                }
                return BundleIntegrationMode.Partial;
            }
            integrationMask = default;
            return BundleIntegrationMode.None;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void IntegratePoseAndVelocity<TIntegratorCallbacks>(
            ref TIntegratorCallbacks integratorCallbacks, ref Vector<int> bodyIndices, in BodyInertiaWide localInertia, float dt, in Vector<int> integrationMask,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity,
            int workerIndex,
            out BodyInertiaWide inertia)
            where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
        {
            //Note that we integrate pose, then velocity.
            //We only use this function where we can guarantee that the external-to-timestep view of velocities and poses looks like the frame starts on a velocity integration and ends on a pose integration.
            //This ensures that velocities set externally are still solved before being integrated.
            //So, the solver runs velocity integration alone on the first substep. All later substeps then run pose + velocity, and then after the last substep, a final pose integration.
            //This is equivalent in ordering to running each substep as velocity, warmstart, solve, pose integration, but just shifting the execution context.
            var dtWide = new Vector<float>(dt);
            var newPosition = position + velocity.Linear * dtWide;
            //Note that we only take results for slots which actually need integration. Reintegration would be an error.
            Vector3Wide.ConditionalSelect(integrationMask, newPosition, position, out position);
            QuaternionWide newOrientation;
            inertia.InverseMass = localInertia.InverseMass;
            var previousVelocity = velocity;
            if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentum)
            {
                var previousOrientation = orientation;
                PoseIntegration.Integrate(orientation, velocity.Angular, dtWide * new Vector<float>(0.5f), out newOrientation);
                QuaternionWide.ConditionalSelect(integrationMask, newOrientation, orientation, out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
                PoseIntegration.IntegrateAngularVelocityConserveMomentum(previousOrientation, localInertia.InverseInertiaTensor, inertia.InverseInertiaTensor, ref velocity.Angular);
            }
            else if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque)
            {
                PoseIntegration.Integrate(orientation, velocity.Angular, dtWide * new Vector<float>(0.5f), out newOrientation);
                QuaternionWide.ConditionalSelect(integrationMask, newOrientation, orientation, out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
                PoseIntegration.IntegrateAngularVelocityConserveMomentumWithGyroscopicTorque(orientation, localInertia.InverseInertiaTensor, ref velocity.Angular, dtWide);
            }
            else
            {
                PoseIntegration.Integrate(orientation, velocity.Angular, dtWide * new Vector<float>(0.5f), out newOrientation);
                QuaternionWide.ConditionalSelect(integrationMask, newOrientation, orientation, out orientation);
                PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
            }
            integratorCallbacks.IntegrateVelocity(bodyIndices, position, orientation, localInertia, integrationMask, workerIndex, new Vector<float>(dt), ref velocity);
            //It would be annoying to make the user handle masking velocity writes to inactive lanes, so we handle it internally.
            Vector3Wide.ConditionalSelect(integrationMask, velocity.Linear, previousVelocity.Linear, out velocity.Linear);
            Vector3Wide.ConditionalSelect(integrationMask, velocity.Angular, previousVelocity.Angular, out velocity.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void IntegrateVelocity<TIntegratorCallbacks, TBatchIntegrationMode>(
            ref TIntegratorCallbacks integratorCallbacks, ref Vector<int> bodyIndices, in BodyInertiaWide localInertia, float dt, in Vector<int> integrationMask,
            in Vector3Wide position, in QuaternionWide orientation, ref BodyVelocityWide velocity,
            int workerIndex,
            out BodyInertiaWide inertia)
            where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
            where TBatchIntegrationMode : unmanaged, IBatchIntegrationMode
        {
            inertia.InverseMass = localInertia.InverseMass;
            PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, orientation, out inertia.InverseInertiaTensor);
            if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentum)
            {
                //Yes, that's integrating backwards to get a previous orientation to convert to momentum. Yup, that's a bit janky.
                PoseIntegration.Integrate(orientation, velocity.Angular, new Vector<float>(dt * -0.5f), out var previousOrientation);
                PoseIntegration.IntegrateAngularVelocityConserveMomentum(previousOrientation, localInertia.InverseInertiaTensor, inertia.InverseInertiaTensor, ref velocity.Angular);
            }
            else if (integratorCallbacks.AngularIntegrationMode == AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque)
            {
                PoseIntegration.IntegrateAngularVelocityConserveMomentumWithGyroscopicTorque(orientation, localInertia.InverseInertiaTensor, ref velocity.Angular, new Vector<float>(dt));
            }
            if (typeof(TBatchIntegrationMode) == typeof(BatchShouldConditionallyIntegrate))
            {
                var previousVelocity = velocity;
                integratorCallbacks.IntegrateVelocity(bodyIndices, position, orientation, localInertia, integrationMask, workerIndex, new Vector<float>(dt), ref velocity);
                //It would be annoying to make the user handle masking velocity writes to inactive lanes, so we handle it internally.
                Vector3Wide.ConditionalSelect(integrationMask, velocity.Linear, previousVelocity.Linear, out velocity.Linear);
                Vector3Wide.ConditionalSelect(integrationMask, velocity.Angular, previousVelocity.Angular, out velocity.Angular);
            }
            else
            {
                integratorCallbacks.IntegrateVelocity(bodyIndices, position, orientation, localInertia, integrationMask, workerIndex, new Vector<float>(dt), ref velocity);
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherAndIntegrate<TIntegratorCallbacks, TBatchIntegrationMode, TAccessFilter, TShouldIntegratePoses>(
            Bodies bodies, ref TIntegratorCallbacks integratorCallbacks, ref Buffer<IndexSet> integrationFlags, int bodyIndexInConstraint, float dt, int workerIndex, int bundleIndex,
            ref Vector<int> bodyIndices, out Vector3Wide position, out QuaternionWide orientation, out BodyVelocityWide velocity, out BodyInertiaWide inertia)
            where TIntegratorCallbacks : struct, IPoseIntegratorCallbacks
            where TBatchIntegrationMode : unmanaged, IBatchIntegrationMode
            where TAccessFilter : unmanaged, IBodyAccessFilter
            where TShouldIntegratePoses : unmanaged, IBatchPoseIntegrationAllowed
        {
            //These type tests are compile time constants and will be specialized.
            if (typeof(TShouldIntegratePoses) == typeof(AllowPoseIntegration))
            {
                if (typeof(TBatchIntegrationMode) == typeof(BatchShouldAlwaysIntegrate))
                {
                    //Avoid slots that are empty (-1) or slots that are kinematic. Both can be tested by checking the unsigned magnitude against the flag lower limit.
                    var integrationMask = Vector.AsVectorInt32(Vector.LessThan(Vector.AsVectorUInt32(bodyIndices), new Vector<uint>(Bodies.DynamicLimit)));
                    bodies.GatherState<AccessAll>(bodyIndices, false, out position, out orientation, out velocity, out var localInertia);
                    IntegratePoseAndVelocity(ref integratorCallbacks, ref bodyIndices, localInertia, dt, integrationMask, ref position, ref orientation, ref velocity, workerIndex, out inertia);
                    bodies.ScatterPose(ref position, ref orientation, bodyIndices, integrationMask);
                    bodies.ScatterInertia(ref inertia, bodyIndices, integrationMask);
                }
                else if (typeof(TBatchIntegrationMode) == typeof(BatchShouldNeverIntegrate))
                {
                    bodies.GatherState<TAccessFilter>(bodyIndices, true, out position, out orientation, out velocity, out inertia);
                }
                else
                {
                    Debug.Assert(typeof(TBatchIntegrationMode) == typeof(BatchShouldConditionallyIntegrate));
                    //This executes in warmstart, and warmstarts are typically quite simple from an instruction stream perspective.
                    //Having a dynamically chosen codepath is unlikely to cause instruction fetching issues.
                    var bundleIntegrationMode = BundleShouldIntegrate(bundleIndex, integrationFlags[bodyIndexInConstraint], out var integrationMask);
                    //Note that this will gather world inertia if there is no integration in the bundle, but that it is guaranteed to load all motion state information.
                    //This avoids complexity around later velocity scattering- we don't have to condition on whether the bundle is integrating.
                    //In practice, since the access filters are only reducing instruction counts and not memory bandwidth,
                    //the slightly increased unnecessary gathering is no worse than the more complex scatter condition in performance, and remains simpler.
                    bodies.GatherState<AccessAll>(bodyIndices, bundleIntegrationMode == BundleIntegrationMode.None, out position, out orientation, out velocity, out var gatheredInertia);
                    if (bundleIntegrationMode != BundleIntegrationMode.None)
                    {
                        //Note that if we take this codepath, the integration routine will reconstruct the world inertias from local inertia given the current pose.
                        //The changes to pose and velocity for integration inactive lanes will be masked out, so it'll just be identical to the world inertia if we had gathered it.
                        //Given that we're running the instructions in a bundle to build it, there's no reason to go out of our way to gather the world inertia.
                        IntegratePoseAndVelocity(ref integratorCallbacks, ref bodyIndices, gatheredInertia, dt, integrationMask, ref position, ref orientation, ref velocity, workerIndex, out inertia);
                        bodies.ScatterPose(ref position, ref orientation, bodyIndices, integrationMask);
                        bodies.ScatterInertia(ref inertia, bodyIndices, integrationMask);
                    }
                    else
                    {
                        inertia = gatheredInertia;
                    }
                }
            }
            else
            {
                Debug.Assert(typeof(TShouldIntegratePoses) == typeof(DisallowPoseIntegration));
                //There is no need to integrate poses; this is the first substep. 
                //Note that the full loop for constrained bodies with 3 substeps looks like:
                //(velocity -> solve) -> (pose -> velocity -> solve) -> (pose -> velocity -> solve) -> pose
                //For unconstrained bodies, it's a tight loop of just:
                //(velocity -> pose) -> (velocity -> pose) -> (velocity -> pose)
                //So we're maintaining the same order.
                //Note that world inertia is still scattered as a part of velocity integration; we need the updated value since we can't trust the cached value across frames.
                if (typeof(TBatchIntegrationMode) == typeof(BatchShouldAlwaysIntegrate))
                {
                    //Avoid slots that are empty (-1) or slots that are kinematic. Both can be tested by checking the unsigned magnitude against the flag lower limit.
                    var integrationMask = Vector.AsVectorInt32(Vector.LessThan(Vector.AsVectorUInt32(bodyIndices), new Vector<uint>(Bodies.DynamicLimit)));
                    bodies.GatherState<AccessAll>(bodyIndices, false, out position, out orientation, out velocity, out var localInertia);
                    IntegrateVelocity<TIntegratorCallbacks, TBatchIntegrationMode>(ref integratorCallbacks, ref bodyIndices, localInertia, dt, integrationMask, position, orientation, ref velocity, workerIndex, out inertia);
                    bodies.ScatterInertia(ref inertia, bodyIndices, integrationMask);
                }
                else if (typeof(TBatchIntegrationMode) == typeof(BatchShouldNeverIntegrate))
                {
                    bodies.GatherState<TAccessFilter>(bodyIndices, true, out position, out orientation, out velocity, out inertia);
                }
                else
                {
                    Debug.Assert(typeof(TBatchIntegrationMode) == typeof(BatchShouldConditionallyIntegrate));
                    var bundleIntegrationMode = BundleShouldIntegrate(bundleIndex, integrationFlags[bodyIndexInConstraint], out var integrationMask);
                    bodies.GatherState<AccessAll>(bodyIndices, bundleIntegrationMode == BundleIntegrationMode.None, out position, out orientation, out velocity, out var gatheredInertia);
                    if (bundleIntegrationMode != BundleIntegrationMode.None)
                    {
                        IntegrateVelocity<TIntegratorCallbacks, TBatchIntegrationMode>(ref integratorCallbacks, ref bodyIndices, gatheredInertia, dt, integrationMask, position, orientation, ref velocity, workerIndex, out inertia);
                        bodies.ScatterInertia(ref inertia, bodyIndices, integrationMask);
                    }
                    else
                    {
                        inertia = gatheredInertia;
                    }
                }
            }

            //var validationMask = Vector.GreaterThanOrEqual(bodyIndices, Vector<int>.Zero);
            //orientation.Validate(validationMask);
            //position.Validate(validationMask);
            //velocity.Linear.Validate(validationMask);
            //velocity.Angular.Validate(validationMask);
        }

    }


}