using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using System.Runtime.InteropServices;
using System.Numerics;
using System.Threading;

namespace BepuPhysics
{

    /// <summary>
    /// Holds and solves constraints between bodies in a simulation.
    /// </summary>
    public abstract partial class Solver
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
        internal QuickList<IndexSet> batchReferencedHandles;

        /// <summary>
        /// Set of processors applied to batches of constraints of particular types, indexed by the constraint type id.
        /// </summary>
        public TypeProcessor[] TypeProcessors;

        internal Bodies bodies;
        internal PairCache pairCache;
        internal IslandAwakener awakener;

        /// <summary>
        /// Pool to retrieve constraint handles from when creating new constraints.
        /// </summary>
        public IdPool HandlePool;
        internal BufferPool pool;
        /// <summary>
        /// Mapping from constraint handle (via its internal integer value) to the location of a constraint in memory.
        /// </summary>
        public Buffer<ConstraintLocation> HandleToConstraint;

        /// <summary>
        /// Gets the maximum number of solver batches to allow before resorting to a fallback solver.
        /// If a single body is constrained by more than FallbackBatchThreshold constraints, all constraints beyond FallbackBatchThreshold are placed into a fallback batch.
        /// The fallback batch uses a different solver that can handle multiple constraints affecting a single body in a single batch, allowing greater parallelism at the cost of convergence speed.
        /// </summary>
        public int FallbackBatchThreshold { get; private set; }

        /// <summary>
        /// Lock used to add to the constrained kinematic handles from multiple threads, if necessary.
        /// </summary>
        internal SpinLock constrainedKinematicLock;
        /// <summary>
        /// Set of body handles associated with constrained kinematic bodies. These will be integrated during substepping.
        /// </summary>
        public QuickSet<int, PrimitiveComparer<int>> ConstrainedKinematicHandles;

        protected int substepCount;
        /// <summary>
        /// Gets or sets the number of substeps the solver will simulate per call to Solve.
        /// </summary>
        public int SubstepCount
        {
            get { return substepCount; }
            set
            {
                if (value < 1)
                    throw new ArgumentException("Substep count must be positive.");
                substepCount = value;
            }
        }

        int velocityIterationCount;
        /// <summary>
        /// Gets or sets the number of solver velocity iterations to compute per substep.
        /// </summary>
        public int VelocityIterationCount
        {
            get { return velocityIterationCount; }
            set
            {
                if (value < 1)
                {
                    throw new ArgumentException("Iteration count must be positive.");
                }
                velocityIterationCount = value;
            }
        }

        /// <summary>
        /// Callback executed to determine how many velocity iterations should be used for a given substep. If null, or if it returns a non-positive value, the <see cref="VelocityIterationCount"/> will be used instead.
        /// </summary>
        public SubstepVelocityIterationScheduler VelocityIterationScheduler { get; set; }

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
        /// Delegate type of solver substep begin/end events.
        /// </summary>
        /// <param name="substepIndex">Index of the substep that the event is about.</param>
        public delegate void SubstepEvent(int substepIndex);
        /// <summary>
        /// Event invoked when the solver begins a substep. If the solver is executing on multiple threads, this will be invoked within the multithreaded dispatch on worker thread 0.
        /// </summary>
        /// <remarks>Take care when attempting to dispatch multithreaded operations from within this event.
        /// If using the same <see cref="IThreadDispatcher"/> instance as the solver, the dispatcher implementation must be reentrant. The demos implementation is not.</remarks>
        public event SubstepEvent SubstepStarted;
        /// <summary>
        /// Event invoked when the solver completes a substep. If the solver is executing on multiple threads, this will be invoked within the multithreaded dispatch on worker thread 0.
        /// </summary>
        /// <remarks>Take care when attempting to dispatch multithreaded operations from within this event.
        /// If using the same <see cref="IThreadDispatcher"/> instance as the solver, the dispatcher implementation must be reentrant. The demos implementation is not.</remarks>
        public event SubstepEvent SubstepEnded;

        protected void OnSubstepStarted(int substepIndex)
        {
            SubstepStarted?.Invoke(substepIndex);
        }
        protected void OnSubstepEnded(int substepIndex)
        {
            SubstepEnded?.Invoke(substepIndex);
        }

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
        /// Counts the number of constraints in a particular type batch.
        /// </summary>
        /// <param name="setIndex">Index of the set containing the type batch.</param>
        /// <param name="batchIndex">Index of the batch containing the type batch.</param>
        /// <param name="typeBatchIndex">Index of the type batch within the batch.</param>
        /// <returns>Number of constraints in the type batch.</returns>
        /// <remarks>This handles whether the type batch is in the fallback batch or not. Active fallback batches are not guaranteed to have contiguous constraints, so the <see cref="TypeBatch.ConstraintCount"/> value is an upper bound and there may be gaps.</remarks>
        public int CountConstraintsInTypeBatch(int setIndex, int batchIndex, int typeBatchIndex)
        {
            Debug.Assert(setIndex >= 0 && setIndex < Sets.Length && Sets[setIndex].Allocated && batchIndex >= 0 && batchIndex < Sets[setIndex].Batches.Count && typeBatchIndex >= 0 && typeBatchIndex < Sets[setIndex].Batches[batchIndex].TypeBatches.Count,
                "Set index, batch index, and type batch index must point at an existing type batch.");
            ref var set = ref Sets[setIndex];
            ref var batch = ref set.Batches[batchIndex];
            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
            if (setIndex == 0 && batchIndex > FallbackBatchThreshold)
            {
                //Sequential fallback batches in the active set currently store constraints noncontiguously to guarantee that each bundle does not share any body references.
                //Counting constraints requires skipping over any empty slots.

                var constraintCount = typeBatch.ConstraintCount;
                var indexToHandle = typeBatch.IndexToHandle;
                int count = 0;
                for (int i = 0; i < constraintCount; ++i)
                {
                    //Empty slots are marked with a -1 in the index to handles mapping (and in body references).
                    if (indexToHandle[i].Value >= 0)
                        ++count;
                }
                return count;
            }
            else
            {
                return typeBatch.ConstraintCount;
            }
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
                    var setIsActiveAndFallbackExists = setIndex == 0 && set.Batches.Count > FallbackBatchThreshold;
                    var contiguousBatchCount = setIsActiveAndFallbackExists ? FallbackBatchThreshold : set.Batches.Count;
                    for (int batchIndex = 0; batchIndex < contiguousBatchCount; ++batchIndex)
                    {
                        ref var batch = ref set.Batches[batchIndex];
                        for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                        {
                            count += batch.TypeBatches[typeBatchIndex].ConstraintCount;
                        }
                    }
                    if (setIsActiveAndFallbackExists)
                    {
                        //Sequential fallback batches in the active set currently store constraints noncontiguously to guarantee that each bundle does not share any body references.
                        //Counting constraints requires skipping over any empty slots.
                        ref var batch = ref set.Batches[FallbackBatchThreshold];
                        for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                        {
                            var constraintCount = batch.TypeBatches[typeBatchIndex].ConstraintCount;
                            var indexToHandle = batch.TypeBatches[typeBatchIndex].IndexToHandle;
                            for (int i = 0; i < constraintCount; ++i)
                            {
                                //Empty slots are marked with a -1 in the index to handles mapping (and in body references).
                                if (indexToHandle[i].Value >= 0)
                                    ++count;
                            }
                        }
                    }

                }
            }
            return count;
        }

        protected Solver(Bodies bodies, BufferPool pool, SolveDescription solveDescription,
            int initialCapacity,
            int initialIslandCapacity,
            int minimumCapacityPerTypeBatch)
        {
            SubstepCount = solveDescription.SubstepCount;
            VelocityIterationCount = solveDescription.VelocityIterationCount;
            VelocityIterationScheduler = solveDescription.VelocityIterationScheduler;
            FallbackBatchThreshold = solveDescription.FallbackBatchThreshold;
            this.minimumCapacityPerTypeBatch = minimumCapacityPerTypeBatch;
            this.bodies = bodies;
            this.pool = pool;
            HandlePool = new IdPool(128, pool);
            ResizeSetsCapacity(initialIslandCapacity + 1, 0);
            ActiveSet = new ConstraintSet(pool, FallbackBatchThreshold + 1);
            batchReferencedHandles = new QuickList<IndexSet>(FallbackBatchThreshold + 1, pool);
            ResizeHandleCapacity(initialCapacity);
            ConstrainedKinematicHandles = new QuickSet<int, PrimitiveComparer<int>>(bodies.HandleToLocation.Length, pool);
        }

        /// <summary>
        /// Registers a constraint type with the solver, creating a type processor for the type internally and allowing constraints of that type to be added to the solver.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint to register with the solver.</typeparam>
        /// <exception cref="ArgumentException">Fired when another constraint type of the same id has already been registered.</exception>
        /// <remarks><see cref="DefaultTypes.RegisterDefaults(Solver, NarrowPhase)"/> is called during simuation creation and registers all the built in types. Calling <see cref="Register"/> manually is only necessary if custom types are used.</remarks>
        public void Register<TDescription>() where TDescription : unmanaged, IConstraintDescription<TDescription>
        {
            var description = default(TDescription);
            Debug.Assert(description.ConstraintTypeId >= 0, "Constraint type ids should never be negative. They're used for array indexing.");
            if (TypeProcessors == null || description.ConstraintTypeId >= TypeProcessors.Length)
            {
                //This will result in some unnecessary resizes, but it hardly matters. It only happens once on registration time.
                //This also means we can just take the current type processors length as an accurate measure of type capacity for constraint batches.
                Array.Resize(ref TypeProcessors, description.ConstraintTypeId + 1);
            }
            if (TypeProcessors[description.ConstraintTypeId] == null)
            {
                var processor = (TypeProcessor)Activator.CreateInstance(description.TypeProcessorType);
                TypeProcessors[description.ConstraintTypeId] = processor;
                processor.Initialize(description.ConstraintTypeId);
            }
            else if (TypeProcessors[description.ConstraintTypeId].GetType() != description.TypeProcessorType)
            {
                throw new ArgumentException(
                    $"Type processor {TypeProcessors[description.ConstraintTypeId].GetType().Name} has already been registered for this description's type id " +
                    $"({typeof(TDescription).Name}, {default(TDescription).ConstraintTypeId}). " +
                    $"Cannot register two types with the same type id.");
            }
        }


        /// <summary>
        /// Gets whether the given constraint handle refers to a constraint in the solver.
        /// </summary>
        /// <param name="constraintHandle">Constraint handle to check for existence in the solver.</param>
        /// <returns>True if the constraint handle exists in the solver, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConstraintExists(ConstraintHandle constraintHandle)
        {
            //A constraint location with a negative set index marks a mapping slot as unused.
            return constraintHandle.Value >= 0 && constraintHandle.Value <= HandlePool.HighestPossiblyClaimedId && HandleToConstraint[constraintHandle.Value].SetIndex >= 0;
        }

        /// <summary>
        /// Gets a direct reference to the constraint associated with a handle.
        /// The reference is temporary; any constraint removals that affect the referenced type batch may invalidate the index.
        /// </summary>
        /// <param name="handle">Handle index of the constraint.</param>
        /// <returns>Temporary direct reference to the type batch and index in the type batch associated with the constraint handle.
        /// May be invalidated by constraint removals.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ConstraintReference GetConstraintReference(ConstraintHandle handle)
        {
            AssertConstraintHandleExists(handle);
            ref var constraintLocation = ref HandleToConstraint[handle.Value];
            return new ConstraintReference(Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatchPointer(constraintLocation.TypeId), constraintLocation.IndexInTypeBatch);
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateConstraintReferenceKinematicity()
        {
            //Only the active set's body indices are flagged for kinematicity; the inactive sets store body handles.
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
                            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                            var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                            for (int i = 0; i < typeBatch.ConstraintCount; ++i)
                            {
                                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                                ref var bodyReferencesBundle = ref typeBatch.BodyReferences[bundleIndex * bodiesPerConstraint * Unsafe.SizeOf<Vector<int>>()];
                                for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraint; ++bodyIndexInConstraint)
                                {
                                    var referencesForBodyIndexInConstraint = Unsafe.Add(ref Unsafe.As<byte, Vector<int>>(ref bodyReferencesBundle), bodyIndexInConstraint);
                                    var encodedBodyReference = referencesForBodyIndexInConstraint[innerIndex];
                                    if (encodedBodyReference > 0)
                                    {
                                        var kinematicByEncodedReference = (encodedBodyReference & Bodies.KinematicMask) != 0;
                                        bool kinematicByInertia;
                                        if (setIndex == 0)
                                        {
                                            //Active set references are indices.
                                            kinematicByInertia = Bodies.IsKinematicUnsafeGCHole(ref bodies.ActiveSet.SolverStates[encodedBodyReference & Bodies.BodyReferenceMask].Inertia.Local);
                                        }
                                        else
                                        {
                                            //Sleeping set references are handles.
                                            kinematicByInertia = bodies[new BodyHandle { Value = encodedBodyReference & Bodies.BodyReferenceMask }].Kinematic;
                                        }
                                        Debug.Assert(kinematicByEncodedReference == kinematicByInertia, "Constraint reference encoded kinematicity must match actual kinematicity by inertia.");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateConstrainedKinematicsSet()
        {
            ref var set = ref bodies.ActiveSet;
            for (int i = 0; i < set.Count; ++i)
            {
                if (Bodies.IsKinematicUnsafeGCHole(ref set.SolverStates[i].Inertia.Local) && set.Constraints[i].Count > 0)
                {
                    var contained = ConstrainedKinematicHandles.Contains(set.IndexToHandle[i].Value);
                    if (!contained)
                        ValidateExistingHandles();
                    Debug.Assert(contained, "Any active kinematic with constraints must appear in the constrained kinematic set.");
                }
            }
            for (int i = 0; i < ConstrainedKinematicHandles.Count; ++i)
            {
                var bodyReference = bodies[new BodyHandle(ConstrainedKinematicHandles[i])];
                Debug.Assert(bodyReference.Kinematic && bodyReference.Constraints.Count > 0, "Any body listed in the constrained kinematics set must be kinematic and constrained.");
            }
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
        internal unsafe void ValidateTrailingTypeBatchBodyReferences()
        {
            ref var set = ref ActiveSet;
            for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
            {
                ref var batch = ref set.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    var expectedEmptyLanesInLastBundle = typeBatch.BundleCount * Vector<int>.Count - typeBatch.ConstraintCount;
                    var firstEmptySlotIndex = Vector<int>.Count - expectedEmptyLanesInLastBundle;
                    ref var lastBodyReferencesBundle = ref typeBatch.BodyReferences[(typeBatch.BundleCount - 1) * bodiesPerConstraint * Unsafe.SizeOf<Vector<int>>()];
                    for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraint; ++bodyIndexInConstraint)
                    {
                        ref var bodyBundleInConstraint = ref Unsafe.Add(ref Unsafe.As<byte, Vector<int>>(ref lastBodyReferencesBundle), bodyIndexInConstraint);
                        ref var bodiesInBundle = ref Unsafe.As<Vector<int>, int>(ref bodyBundleInConstraint);
                        for (int i = firstEmptySlotIndex; i < Vector<int>.Count; ++i)
                        {
                            Debug.Assert(Unsafe.Add(ref bodiesInBundle, i) == -1, "Any awake incomplete bundle should have its trailing values initialized to -1.");
                        }
                    }
                }
            }
        }
        [Conditional("DEBUG")]
        internal unsafe void ValidateFallbackBatchEmptySlotReferences()
        {
            ref var set = ref ActiveSet;
            if (set.Batches.Count > FallbackBatchThreshold)
            {
                ref var batch = ref set.Batches[FallbackBatchThreshold];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    var bodyReferencesBundleSize = Unsafe.SizeOf<Vector<int>>() * bodiesPerConstraint;
                    for (int i = 0; i < typeBatch.ConstraintCount; ++i)
                    {
                        var expectDeadSlot = typeBatch.IndexToHandle[i].Value == -1;
                        BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                        var bodyReferenceForFirstBody = Unsafe.As<byte, int>(ref typeBatch.BodyReferences[bundleIndex * bodyReferencesBundleSize + 4 * innerIndex]);
                        Debug.Assert(expectDeadSlot == (bodyReferenceForFirstBody == -1), "For fallback batches, the IndexToHandle should be -1 when the body lanes are -1, corresponding to empty lanes in the sparse batch.");
                    }
                }
            }
        }


        [Conditional("DEBUG")]
        internal unsafe void ValidateFallbackBodiesAreDynamic()
        {
            ref var set = ref ActiveSet;
            if (set.Batches.Count > FallbackBatchThreshold)
            {
                for (int i = 0; i < set.SequentialFallback.dynamicBodyConstraintCounts.Count; ++i)
                {
                    Debug.Assert(!Bodies.IsKinematicUnsafeGCHole(ref bodies.ActiveSet.SolverStates[set.SequentialFallback.dynamicBodyConstraintCounts.Keys[i]].Inertia.Local),
                        "All ostensibly dynamic bodies tracked by the fallback batch must actually be dynamic.");
                }
                for (int i = 0; i < bodies.ActiveSet.Count; ++i)
                {
                    var constraints = bodies.ActiveSet.Constraints[i];
                    var fallbackConstraintsForDynamicBody = 0;
                    for (int j = 0; j < constraints.Count; ++j)
                    {
                        if (HandleToConstraint[constraints[j].ConnectingConstraintHandle.Value].BatchIndex == FallbackBatchThreshold)
                        {
                            ++fallbackConstraintsForDynamicBody;
                        }
                    }
                    var bodyIsInFallbackDynamicsSet = ActiveSet.SequentialFallback.dynamicBodyConstraintCounts.TryGetValue(i, out var countForBody);
                    if (Bodies.IsKinematicUnsafeGCHole(ref bodies.ActiveSet.SolverStates[i].Inertia.Local))
                    {
                        Debug.Assert(!bodyIsInFallbackDynamicsSet, "Kinematics should not be present in the dynamic bodies referenced by the fallback batch.");
                    }
                    else
                    {
                        Debug.Assert(bodyIsInFallbackDynamicsSet == (fallbackConstraintsForDynamicBody > 0), "The fallback batch should contain a reference to the dynamic body if there are constraints associated with it in the fallback batch.");
                        Debug.Assert(fallbackConstraintsForDynamicBody == countForBody, "If the dynamic body is referenced in the fallback batch, the brute force count should match the cached count.");
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateFallbackBatchAccessSafety()
        {
            ref var set = ref ActiveSet;
            if (set.Batches.Count > FallbackBatchThreshold)
            {
                ref var batch = ref set.Batches[FallbackBatchThreshold];
                int occupiedLaneCountAcrossBatch = 0;
                int totalBundleCount = 0;
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    totalBundleCount += typeBatch.BundleCount;
                    var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    var bodyReferencesBundleSize = Unsafe.SizeOf<Vector<int>>() * bodiesPerConstraint;
                    for (int bundleIndex = 0; bundleIndex < typeBatch.BundleCount; ++bundleIndex)
                    {
                        ref var bodyReferenceForFirstBody = ref Unsafe.As<byte, Vector<int>>(ref typeBatch.BodyReferences[bundleIndex * bodyReferencesBundleSize]);
                        var occupiedLaneMask = Vector.GreaterThanOrEqual(bodyReferenceForFirstBody, Vector<int>.Zero);
                        var occupiedLaneCountInBundle = 0;
                        for (int i = 0; i < Vector<int>.Count; ++i)
                        {
                            if (occupiedLaneMask[i] < 0)
                                ++occupiedLaneCountInBundle;
                        }
                        occupiedLaneCountAcrossBatch += occupiedLaneCountInBundle;
                        Debug.Assert(occupiedLaneCountInBundle > 0, "For any bundle in the [0, BundleCount) interval, there must be at least one occupied lane.");
                        for (int sourceBodyIndexInConstraint = 0; sourceBodyIndexInConstraint < bodiesPerConstraint; ++sourceBodyIndexInConstraint)
                        {
                            var bodyReferencesForSource = Unsafe.Add(ref bodyReferenceForFirstBody, sourceBodyIndexInConstraint);
                            for (int innerIndex = 0; innerIndex < Vector<int>.Count; ++innerIndex)
                            {
                                var index = bodyReferencesForSource[innerIndex];
                                if (index >= 0 && Bodies.IsEncodedDynamicReference(index))
                                {
                                    var broadcasted = new Vector<int>(bodyReferencesForSource[innerIndex]);
                                    int matchesTotal = 0;
                                    for (int targetBodyIndexInConstraint = 0; targetBodyIndexInConstraint < bodiesPerConstraint; ++targetBodyIndexInConstraint)
                                    {
                                        var bodyReferencesForTarget = Unsafe.Add(ref bodyReferenceForFirstBody, targetBodyIndexInConstraint);
                                        var matchesInLane = -Vector.Dot(Vector.Equals(broadcasted, bodyReferencesForTarget), Vector<int>.One);
                                        matchesTotal += matchesInLane;
                                    }
                                    Debug.Assert(matchesTotal == 1, "A dynamic body reference should occur no more than once in any constraint bundle.");
                                }
                            }
                        }
                    }
                }
                //Console.WriteLine($"Average fallback occupancy: {Vector<int>.Count * occupiedLaneCountAcrossBatch / (double)(totalBundleCount * Vector<int>.Count):G3} / {Vector<int>.Count}, total bundle count: {totalBundleCount}");
            }
        }
        [Conditional("DEBUG")]
        internal void ValidateSetOwnership(ref TypeBatch typeBatch, int expectedSetIndex)
        {
            for (int i = 0; i < typeBatch.ConstraintCount; ++i)
            {
                var handle = typeBatch.IndexToHandle[i];
                if (handle.Value >= 0)
                {
                    Debug.Assert(HandleToConstraint[handle.Value].SetIndex == expectedSetIndex);
                }
            }
        }
        [Conditional("DEBUG")]
        internal void ValidateSetOwnership()
        {
            for (int setIndex = 0; setIndex < Sets.Length; ++setIndex)
            {
                ref var set = ref Sets[setIndex];
                if (!set.Allocated)
                    continue;
                for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
                {
                    ref var batch = ref set.Batches[batchIndex];
                    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                    {
                        ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                        ValidateSetOwnership(ref typeBatch, setIndex);
                    }
                }
            }
        }

        unsafe struct ValidateAccumulatedImpulsesEnumerator : IForEach<float>
        {
            public int Index;
            public float* AccumulatedImpulses;
            public void LoopBody(float impulse)
            {
                AccumulatedImpulses[Index] = impulse;
            }
        }


        internal unsafe void ValidateFallbackBodyReferencesByHash(HashDiagnosticType hashDiagnosticType)
        {
            var hashes = InvasiveHashDiagnostics.Instance;
            ref var hash = ref hashes.GetHashForType(hashDiagnosticType);
            if (ActiveSet.Batches.Count > FallbackBatchThreshold)
            {
                ref var batch = ref ActiveSet.Batches[FallbackBatchThreshold];
                for (int i = 0; i < batch.TypeBatches.Count; ++i)
                {
                    ref var typeBatch = ref batch.TypeBatches[i];
                    hashes.ContributeToHash(ref hash, typeBatch.TypeId);
                    hashes.ContributeToHash(ref hash, typeBatch.ConstraintCount);
                    var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
                    var bytesPerBodyReferencesBundle = bodiesPerConstraint * Unsafe.SizeOf<Vector<int>>();
                    for (int bundleIndex = 0; bundleIndex < typeBatch.BundleCount; ++bundleIndex)
                    {
                        var countInBundle = typeBatch.ConstraintCount - bundleIndex * Vector<int>.Count;
                        if (countInBundle > Vector<int>.Count)
                            countInBundle = Vector<int>.Count;
                        ref var bundleStart = ref Unsafe.As<byte, Vector<int>>(ref typeBatch.BodyReferences[bytesPerBodyReferencesBundle * bundleIndex]);
                        for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < bodiesPerConstraint; ++bodyIndexInConstraint)
                        {
                            var bodyVector = Unsafe.Add(ref bundleStart, bodyIndexInConstraint);
                            for (int innerIndex = 0; innerIndex < countInBundle; ++innerIndex)
                            {
                                var bodyIndex = bodyVector[innerIndex];
                                if (bodyIndex >= 0)
                                    hashes.ContributeToHash(ref hash, bodies.ActiveSet.IndexToHandle[bodyIndex & Bodies.BodyReferenceMask].Value);
                                else
                                    hashes.ContributeToHash(ref hash, bodyIndex);
                            }
                        }
                    }
                    for (int constraintIndex = 0; constraintIndex < typeBatch.ConstraintCount; ++constraintIndex)
                    {
                        hashes.ContributeToHash(ref hash, typeBatch.IndexToHandle[constraintIndex].Value);
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateAccumulatedImpulses()
        {
            var impulseMemory = stackalloc float[16];
            var impulsesEnumerator = new ValidateAccumulatedImpulsesEnumerator { Index = 0, AccumulatedImpulses = impulseMemory };
            for (int i = 0; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (!set.Allocated)
                    continue;
                for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
                {
                    ref var batch = ref set.Batches[batchIndex];
                    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                    {
                        ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                        var dofCount = TypeProcessors[typeBatch.TypeId].ConstrainedDegreesOfFreedom;
                        for (int constraintIndex = 0; constraintIndex < typeBatch.ConstraintCount; ++constraintIndex)
                        {
                            if (typeBatch.IndexToHandle[constraintIndex].Value >= 0)
                            {
                                impulsesEnumerator.Index = 0;
                                TypeProcessors[typeBatch.TypeId].EnumerateAccumulatedImpulses(ref typeBatch, constraintIndex, ref impulsesEnumerator);
                                for (int dofIndex = 0; dofIndex < dofCount; ++dofIndex)
                                {
                                    impulseMemory[dofIndex].Validate();
                                }
                            }
                        }
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateBatchReferencedHandlesVersusConstraintStoredReferences()
        {
            const int maximumBodyCountInConstraint = 4;
            int* debugReferences = stackalloc int[maximumBodyCountInConstraint];
            for (int batchIndex = 0; batchIndex < ActiveSet.Batches.Count; ++batchIndex)
            {
                var batch = ActiveSet.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    Debug.Assert(TypeProcessors[typeBatch.TypeId].BodiesPerConstraint <= maximumBodyCountInConstraint);
                    for (int constraintIndex = 0; constraintIndex < typeBatch.ConstraintCount; ++constraintIndex)
                    {
                        var constraintHandle = typeBatch.IndexToHandle[constraintIndex];
                        if (constraintHandle.Value >= 0)
                        {
                            PassthroughReferenceCollector debugEnumerator = new(debugReferences);
                            EnumerateConnectedRawBodyReferences(constraintHandle, ref debugEnumerator);
                            for (int bodyIndexInConstraint = 0; bodyIndexInConstraint < debugEnumerator.Index; ++bodyIndexInConstraint)
                            {
                                var isInReferencedHandles = batchReferencedHandles[batchIndex].Contains(bodies.ActiveSet.IndexToHandle[debugReferences[bodyIndexInConstraint] & Bodies.BodyReferenceMask].Value);
                                Debug.Assert(isInReferencedHandles == Bodies.IsEncodedDynamicReference(debugReferences[bodyIndexInConstraint]));
                            }
                        }
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal unsafe void ValidateExistingHandles(bool activeOnly = false)
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
            //Note that this only applies to the active set batches; inactive sets do not explicitly track referenced handles.
            for (int batchIndex = 0; batchIndex < ActiveSet.Batches.Count; ++batchIndex)
            {
                ref var handles = ref batchReferencedHandles[batchIndex];
                ref var batch = ref ActiveSet.Batches[batchIndex];
                for (int i = 0; i < bodies.ActiveSet.Count; ++i)
                {
                    var handle = bodies.ActiveSet.IndexToHandle[i];
                    int expectedCount = 0;
                    int bodyReference = i;
                    if (Bodies.IsKinematicUnsafeGCHole(ref bodies.ActiveSet.SolverStates[i].Inertia.Local))
                    {
                        //Kinematic bodies may appear more than once in non-fallback batches, so we have to count how many references to expect.
                        var constraints = bodies.ActiveSet.Constraints[i];
                        for (int constraintForBodyIndex = 0; constraintForBodyIndex < constraints.Count; ++constraintForBodyIndex)
                        {
                            if (HandleToConstraint[constraints[constraintForBodyIndex].ConnectingConstraintHandle.Value].BatchIndex == batchIndex)
                                ++expectedCount;
                        }
                        bodyReference |= Bodies.KinematicMask;
                    }
                    else
                    {
                        if (handles.Contains(handle.Value))
                        {
                            if (batchIndex < FallbackBatchThreshold)
                            {
                                //A dynamic body can only appear in a non-fallback batch at most once.
                                expectedCount = 1;
                            }
                            else
                            {
                                //If this is the fallback batch, then the expected count may be more than 1.
                                var foundBody = ActiveSet.SequentialFallback.dynamicBodyConstraintCounts.TryGetValue(i, out var constraintCountInFallbackBatchForBody);
                                Debug.Assert(foundBody, "A body was in the fallback batch's referenced handles, so the fallback batch should have a reference for that body.");
                                expectedCount = foundBody ? constraintCountInFallbackBatchForBody : 0;
                            }
                        }
                        else
                        {
                            expectedCount = 0;
                        }
                    }
                    ValidateBodyReference(bodyReference, expectedCount, ref batch);
                }
                //No inactive bodies should be present in the active set solver batch referenced handles.
                for (int inactiveBodySetIndex = 1; inactiveBodySetIndex < bodies.Sets.Length; ++inactiveBodySetIndex)
                {
                    ref var bodySet = ref bodies.Sets[inactiveBodySetIndex];
                    if (bodies.Sets[inactiveBodySetIndex].Allocated)
                    {
                        for (int i = 0; i < bodySet.Count; ++i)
                        {
                            Debug.Assert(!handles.Contains(bodySet.IndexToHandle[i].Value), "Bodies in an inactive set should not show up in the active solver set's referenced body handles.");
                        }
                    }
                }
            }
            //Now, for all sets, validate that constraint and body references to each other are consistent and complete.
            PassthroughReferenceCollector enumerator;
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
                                EnumerateConnectedRawBodyReferences(ref typeBatch, indexInTypeBatch, ref enumerator);

                                for (int i = 0; i < processor.BodiesPerConstraint; ++i)
                                {
                                    //Active constraints refer to bodies with an index. Inactive constraints use a handle.
                                    int bodyIndex;
                                    if (setIndex == 0)
                                    {
                                        bodyIndex = constraintBodyReferences[i] & Bodies.BodyReferenceMask;
                                    }
                                    else
                                    {
                                        ref var referencedBodyLocation = ref bodies.HandleToLocation[constraintBodyReferences[i] & Bodies.BodyReferenceMask];
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
                            ref var constraintLocation = ref HandleToConstraint[constraintList[constraintIndex].ConnectingConstraintHandle.Value];
                            Debug.Assert(constraintLocation.SetIndex == setIndex, "Any constraint involved with a body should be in the same set.");
                            ref var batch = ref set.Batches[constraintLocation.BatchIndex];
                            ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
                            var processor = TypeProcessors[typeBatch.TypeId];
                            enumerator.Index = 0;
                            EnumerateConnectedRawBodyReferences(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
                            //Active constraints refer to bodies by index; inactive constraints use handles.
                            int bodyReference = setIndex == 0 ? bodyIndex : bodySet.IndexToHandle[bodyIndex].Value;
                            var bodyReferenceInConstraint = constraintBodyReferences[constraintList[constraintIndex].BodyIndexInConstraint];
                            bodyReferenceInConstraint &= Bodies.BodyReferenceMask;
                            Debug.Assert(bodyReferenceInConstraint == bodyReference,
                                "If a body refers to a constraint, the constraint should refer to the body.");
                        }
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal void ValidateConstraintMaps(int setIndex, int batchIndex, int typeBatchIndex, int constraintStart, int constraintCount)
        {
            ref var set = ref Sets[setIndex];
            ref var batch = ref set.Batches[batchIndex];
            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
            var end = constraintStart + constraintCount;
            if (batchIndex == FallbackBatchThreshold)
            {
                for (int indexInTypeBatch = constraintStart; indexInTypeBatch < end; ++indexInTypeBatch)
                {
                    //Fallback batches can have empty slots, marked with a -1 in the handle slot.
                    var handle = typeBatch.IndexToHandle[indexInTypeBatch];
                    if (handle.Value >= 0)
                    {
                        AssertConstraintHandleExists(handle);
                        ref var constraintLocation = ref HandleToConstraint[handle.Value];
                        Debug.Assert(constraintLocation.SetIndex == setIndex);
                        Debug.Assert(constraintLocation.BatchIndex == batchIndex);
                        Debug.Assert(constraintLocation.IndexInTypeBatch == indexInTypeBatch);
                        Debug.Assert(constraintLocation.TypeId == typeBatch.TypeId);
                        Debug.Assert(batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId] == typeBatchIndex);
                    }
                }
            }
            else
            {
                for (int indexInTypeBatch = constraintStart; indexInTypeBatch < end; ++indexInTypeBatch)
                {
                    var handle = typeBatch.IndexToHandle[indexInTypeBatch];
                    AssertConstraintHandleExists(handle);
                    ref var constraintLocation = ref HandleToConstraint[handle.Value];
                    Debug.Assert(constraintLocation.SetIndex == setIndex);
                    Debug.Assert(constraintLocation.BatchIndex == batchIndex);
                    Debug.Assert(constraintLocation.IndexInTypeBatch == indexInTypeBatch);
                    Debug.Assert(constraintLocation.TypeId == typeBatch.TypeId);
                    Debug.Assert(batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId] == typeBatchIndex);
                }
            }
        }
        [Conditional("DEBUG")]
        internal void ValidateConstraintMaps(int setIndex, int batchIndex, int typeBatchIndex)
        {
            ValidateConstraintMaps(setIndex, batchIndex, typeBatchIndex, 0, Sets[setIndex].Batches[batchIndex].TypeBatches[typeBatchIndex].ConstraintCount);
        }

        [Conditional("DEBUG")]
        internal void ValidateActiveFallbackConstraintMaps()
        {
            ref var set = ref Sets[0];
            if (set.Allocated)
            {
                if (set.Batches.Count > FallbackBatchThreshold)
                {
                    ref var batch = ref set.Batches[FallbackBatchThreshold];
                    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                    {
                        ValidateConstraintMaps(0, FallbackBatchThreshold, typeBatchIndex);
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal void ValidateConstraintMaps(bool activeOnly = false)
        {
            for (int i = 0; i < HandleToConstraint.Length; ++i)
            {
                var handle = new ConstraintHandle { Value = i };
                if (ConstraintExists(handle))
                {
                    AssertConstraintHandleExists(handle);
                }
            }
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
                            ValidateConstraintMaps(setIndex, batchIndex, typeBatchIndex);
                        }
                    }
                }
            }
        }

        [Conditional("DEBUG")]
        internal void AssertConstraintHandleExists(ConstraintHandle handle)
        {
            Debug.Assert(ConstraintExists(handle), "Constraint must exist according to Solver.ConstraintExists test.");
            Debug.Assert(handle.Value >= 0 && handle.Value < HandleToConstraint.Length, "Handle must be contained within the handle mapping.");
            ref var location = ref HandleToConstraint[handle.Value];
            Debug.Assert(location.SetIndex >= 0 && location.SetIndex < Sets.Length, "Set index must be within the sets buffer.");
            ref var set = ref Sets[location.SetIndex];
            Debug.Assert(location.BatchIndex >= 0 && location.BatchIndex < set.Batches.Count, "Batch index must be within the set's batches buffer.");
            ref var batch = ref set.Batches[location.BatchIndex];
            Debug.Assert(location.TypeId >= 0 && location.TypeId < batch.TypeIndexToTypeBatchIndex.Length, "Type id must exist within the batch's type id mapping.");
            var typeBatchIndex = batch.TypeIndexToTypeBatchIndex[location.TypeId];
            Debug.Assert(typeBatchIndex >= 0 && typeBatchIndex < batch.TypeBatches.Count, "Type batch index must be a valid index in the type batches list.");
            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
            Debug.Assert(typeBatch.IndexToHandle[location.IndexInTypeBatch].Value == handle.Value, "Index->handle mapping in type batch must agree with handle->index mapping.");
        }

        /// <summary>
        /// Attempts to locate a spot for a new constraint. Does not perform allocation for the constraint. If no batch exists, returns the index just beyond the end of the existing list of batches.
        /// </summary>
        /// <returns>Index of the batch that the constraint would fit in.</returns>
        /// <remarks>This is used by the narrowphase's multithreaded constraint adders to locate a spot for a new constraint without requiring a lock. Only after a candidate is located
        /// do those systems attempt an actual claim, limiting the duration of locks and increasing potential parallelism.</remarks>
        internal unsafe int FindCandidateBatch(CollidablePair collidablePair)
        {
            ref var set = ref ActiveSet;
            GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
            var aIsDynamic = collidablePair.A.Mobility == Collidables.CollidableMobility.Dynamic;
            if (aIsDynamic && collidablePair.B.Mobility == Collidables.CollidableMobility.Dynamic)
            {
                //Both collidables are dynamic.
                var a = collidablePair.A.BodyHandle.Value;
                var b = collidablePair.B.BodyHandle.Value;
                for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                {
                    if (!batchReferencedHandles[batchIndex].Contains(a) && !batchReferencedHandles[batchIndex].Contains(b))
                        return batchIndex;
                }
            }
            else
            {
                //Only one collidable is dynamic. Statics and kinematics will not block batch containment.
                Debug.Assert(aIsDynamic || collidablePair.B.Mobility == Collidables.CollidableMobility.Dynamic,
                    "Constraints can only be created when at least one body in the pair is dynamic.");
                var dynamicHandle = (aIsDynamic ? collidablePair.A.BodyHandle : collidablePair.B.BodyHandle).Value;
                for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                {
                    if (!batchReferencedHandles[batchIndex].Contains(dynamicHandle))
                        return batchIndex;
                }
            }
            //No synchronized batch worked. Either there's a fallback batch or there aren't yet enough batches to warrant a fallback batch and none of the existing batches could fit the handles.
            return synchronizedBatchCount;
        }

        internal unsafe void AllocateInBatch(int targetBatchIndex, ConstraintHandle constraintHandle, Span<BodyHandle> dynamicBodyHandles, Span<int> encodedBodyIndices, int typeId, out ConstraintReference reference)
        {
            ref var batch = ref ActiveSet.Batches[targetBatchIndex];
            for (int j = 0; j < encodedBodyIndices.Length; ++j)
            {
                //Include the body in the constrained kinematics set if necessary.
                var encodedBodyIndex = encodedBodyIndices[j];
                if (Bodies.IsEncodedKinematicReference(encodedBodyIndex))
                {
                    ConstrainedKinematicHandles.Add(bodies.ActiveSet.IndexToHandle[encodedBodyIndex & Bodies.BodyReferenceMask].Value, pool);
                }
            }
            var typeProcessor = TypeProcessors[typeId];
            Debug.Assert(typeProcessor.BodiesPerConstraint == encodedBodyIndices.Length);
            var typeBatch = batch.GetOrCreateTypeBatch(typeId, typeProcessor, GetMinimumCapacityForType(typeId), pool);
            int indexInTypeBatch;
            if (targetBatchIndex == FallbackBatchThreshold)
                indexInTypeBatch = typeProcessor.AllocateInTypeBatchForFallback(ref *typeBatch, constraintHandle, encodedBodyIndices, pool);
            else
                indexInTypeBatch = typeProcessor.AllocateInTypeBatch(ref *typeBatch, constraintHandle, encodedBodyIndices, pool);
            reference = new ConstraintReference(typeBatch, indexInTypeBatch);
            //TODO: We could adjust the typeBatchAllocation capacities in response to the allocated index.
            //If it exceeds the current capacity, we could ensure the new size is still included.
            //The idea here would be to avoid resizes later by ensuring that the historically encountered size is always used to initialize.
            //This isn't necessarily beneficial, though- often, higher indexed batches will contain smaller numbers of constraints, so allocating a huge number
            //of constraints into them is very low value. You may want to be a little more clever about the heuristic. Either way, only bother with this once there is 
            //evidence that typebatch resizes are ever a concern. This will require frame spike analysis, not merely average timings.
            //(While resizes will definitely occur, remember that it only really matters for *new* type batches- 
            //and it is rare that a new type batch will be created that actually needs to be enormous.)

            ref var handlesSet = ref batchReferencedHandles[targetBatchIndex];
            for (int i = 0; i < dynamicBodyHandles.Length; ++i)
            {
                Debug.Assert(targetBatchIndex == FallbackBatchThreshold || !handlesSet.Contains(dynamicBodyHandles[i].Value), "Non-fallback batches should not come to include references to the same dynamic body more than once.");
                handlesSet.Set(dynamicBodyHandles[i].Value, pool);
            }
            if (targetBatchIndex == FallbackBatchThreshold)
            {
                ActiveSet.SequentialFallback.AllocateForActive(dynamicBodyHandles, bodies, pool);
            }
        }

        unsafe internal void GetBlockingBodyHandles(Span<BodyHandle> bodyHandles, ref Span<BodyHandle> blockingBodyHandlesAllocation, Span<int> encodedBodyIndices)
        {
            //Kinematics do not block allocation in a batch; they are treated as read only by the solver.
            int blockingCount = 0;
            var solverStates = bodies.ActiveSet.SolverStates;
            for (int i = 0; i < bodyHandles.Length; ++i)
            {
                var location = bodies.HandleToLocation[bodyHandles[i].Value];
                Debug.Assert(location.SetIndex == 0);
                if (Bodies.IsKinematicUnsafeGCHole(ref solverStates[location.Index].Inertia.Local))
                {
                    encodedBodyIndices[i] = location.Index | Bodies.KinematicMask;
                }
                else
                {
                    blockingBodyHandlesAllocation[blockingCount++] = bodyHandles[i];
                    encodedBodyIndices[i] = location.Index;
                }
            }
            blockingBodyHandlesAllocation = blockingBodyHandlesAllocation.Slice(0, blockingCount);
        }

        internal int AllocateNewConstraintBatch()
        {
            ref var set = ref ActiveSet;
            if (set.Batches.Count == set.Batches.Span.Length)
                set.Batches.Resize(set.Batches.Count + 1, pool);
            set.Batches.AllocateUnsafely() = new ConstraintBatch(pool, TypeProcessors.Length);
            //Create an index set for the new batch.
            if (set.Batches.Count == batchReferencedHandles.Span.Length)
                batchReferencedHandles.Resize(set.Batches.Count + 1, pool);
            batchReferencedHandles.AllocateUnsafely() = new IndexSet(pool, bodies.ActiveSet.Count);
            return set.Batches.Count - 1;
        }

        internal unsafe bool TryAllocateInBatch(int typeId, int targetBatchIndex, Span<BodyHandle> dynamicBodyHandles, Span<int> encodedBodyIndices, out ConstraintHandle constraintHandle, out ConstraintReference reference)
        {
            ref var set = ref ActiveSet;
            Debug.Assert(targetBatchIndex <= set.Batches.Count,
                "It should be impossible for a target batch to be generated which is more than one slot beyond the end of the batch list. Possible misuse of FindCandidateBatch.");
            if (targetBatchIndex == set.Batches.Count)
            {
                //No batch available. Have to create a new one.
                //Note that if there is no constraint batch for the given index, there is no way for the constraint add to be blocked. It's guaranteed success.
                AllocateNewConstraintBatch();
            }
            else
            {
                //Only non-fallback batches can block an incoming constraint. Additions to the fallback batch cannot be blocked; it permits multiple constraints associated with the same body.
                if (targetBatchIndex < FallbackBatchThreshold)
                {
                    //A non-fallback constraint batch already exists here. This may fail.
                    if (!batchReferencedHandles[targetBatchIndex].CanFit(MemoryMarshal.Cast<BodyHandle, int>(dynamicBodyHandles)))
                    {
                        //This batch cannot hold the constraint.
                        constraintHandle = new ConstraintHandle(-1);
                        reference = default;
                        return false;
                    }
                }
            }
            constraintHandle = new ConstraintHandle(HandlePool.Take());
            AllocateInBatch(targetBatchIndex, constraintHandle, dynamicBodyHandles, encodedBodyIndices, typeId, out reference);

            if (constraintHandle.Value >= HandleToConstraint.Length)
            {
                pool.ResizeToAtLeast(ref HandleToConstraint, HandleToConstraint.Length * 2, HandleToConstraint.Length);
                Debug.Assert(constraintHandle.Value < HandleToConstraint.Length, "Handle indices should never jump by more than 1 slot, so doubling should always be sufficient.");
            }
            ref var constraintLocation = ref HandleToConstraint[constraintHandle.Value];
#if DEBUG
            //Note that new constraints are always active. It is assumed at this point that all connected bodies have been forced active.
            for (int i = 0; i < dynamicBodyHandles.Length; ++i)
            {
                Debug.Assert(bodies.HandleToLocation[dynamicBodyHandles[i].Value].SetIndex == 0, "New constraints should only be created involving already-activated bodies.");
            }
#endif
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
        public unsafe void ApplyDescriptionWithoutWaking<TDescription>(in ConstraintReference constraintReference, in TDescription description)
            where TDescription : unmanaged, IConstraintDescription<TDescription>
        {
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            description.ApplyDescription(ref *constraintReference.typeBatchPointer, bundleIndex, innerIndex);
        }

        /// <summary>
        /// Applies a description to a constraint slot without waking up the associated island.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintHandle">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescriptionWithoutWaking<TDescription>(ConstraintHandle constraintHandle, in TDescription description)
            where TDescription : unmanaged, IConstraintDescription<TDescription>
        {
            var constraintReference = GetConstraintReference(constraintHandle);
            ApplyDescriptionWithoutWaking(constraintReference, description);
        }

        /// <summary>
        /// Applies a description to a constraint slot, waking up the connected bodies if necessary.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintHandle">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescription<TDescription>(ConstraintHandle constraintHandle, in TDescription description)
            where TDescription : unmanaged, IConstraintDescription<TDescription>
        {
            awakener.AwakenConstraint(constraintHandle);
            ApplyDescriptionWithoutWaking(constraintHandle, description);
        }

        void Add<TDescription>(Span<BodyHandle> bodyHandles, in TDescription description, out ConstraintHandle handle)
            where TDescription : unmanaged, IConstraintDescription<TDescription>
        {
            ref var set = ref ActiveSet;
            Span<BodyHandle> blockingBodyHandles = stackalloc BodyHandle[bodyHandles.Length];
            Span<int> encodedBodyIndices = stackalloc int[bodyHandles.Length];
            GetBlockingBodyHandles(bodyHandles, ref blockingBodyHandles, encodedBodyIndices);
            for (int i = 0; i <= set.Batches.Count; ++i)
            {
                if (TryAllocateInBatch(description.ConstraintTypeId, i, blockingBodyHandles, encodedBodyIndices, out handle, out var reference))
                {
                    ApplyDescriptionWithoutWaking(reference, description);
                    return;
                }
            }
            handle = new ConstraintHandle(-1);
            Debug.Fail("The above allocation loop checks every batch and also one index beyond all existing batches. It should be guaranteed to succeed.");
        }

        /// <summary>
        /// Allocates a constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandles">Body handles used by the constraint.</param>
        /// <param name="description">Description of the constraint to add.</param>
        /// <returns>Allocated constraint handle.</returns>
        public ConstraintHandle Add<TDescription>(Span<BodyHandle> bodyHandles, in TDescription description)
            where TDescription : unmanaged, IConstraintDescription<TDescription>
        {
            Debug.Assert(description.ConstraintTypeId >= 0 && description.ConstraintTypeId < TypeProcessors.Length &&
                TypeProcessors[description.ConstraintTypeId].GetType() == description.TypeProcessorType,
                "The description's constraint type and type processor don't match what has been registered in the solver. Did you forget to register the constraint type?");
            Debug.Assert(bodyHandles.Length == TypeProcessors[description.ConstraintTypeId].BodiesPerConstraint,
                "The number of bodies supplied to a constraint add must match the expected number of bodies involved in that constraint type. Did you use the wrong Solver.Add overload?");
            //Adding a constraint assumes that the involved bodies are active, so wake up anything that is sleeping.
            for (int i = 0; i < bodyHandles.Length; ++i)
            {
                awakener.AwakenBody(bodyHandles[i]);
            }
            Add(bodyHandles, description, out var constraintHandle);
            for (int i = 0; i < bodyHandles.Length; ++i)
            {
                var bodyHandle = bodyHandles[i];
                bodies.ValidateExistingHandle(bodyHandle);
                bodies.AddConstraint(bodies.HandleToLocation[bodyHandle.Value].Index, constraintHandle, i);
            }
            return constraintHandle;
        }

        /// <summary>
        /// Allocates a one-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandle">Body connected to the constraint.</param>
        /// <param name="description">Description of the constraint to add.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe ConstraintHandle Add<TDescription>(BodyHandle bodyHandle, in TDescription description)
            where TDescription : unmanaged, IOneBodyConstraintDescription<TDescription>
        {
            Span<BodyHandle> bodyHandles = stackalloc BodyHandle[] { bodyHandle };
            return Add(bodyHandles, description);
        }

        /// <summary>
        /// Allocates a two-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandleA">First body of the constraint.</param>
        /// <param name="bodyHandleB">Second body of the constraint.</param>
        /// <param name="description">Description of the constraint to add.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe ConstraintHandle Add<TDescription>(BodyHandle bodyHandleA, BodyHandle bodyHandleB, in TDescription description)
            where TDescription : unmanaged, ITwoBodyConstraintDescription<TDescription>
        {
            Span<BodyHandle> bodyHandles = stackalloc BodyHandle[] { bodyHandleA, bodyHandleB };
            return Add(bodyHandles, description);
        }

        /// <summary>
        /// Allocates a three-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandleA">First body of the constraint.</param>
        /// <param name="bodyHandleB">Second body of the constraint.</param>
        /// <param name="bodyHandleC">Third body of the constraint.</param>
        /// <param name="description">Description of the constraint to add.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe ConstraintHandle Add<TDescription>(BodyHandle bodyHandleA, BodyHandle bodyHandleB, BodyHandle bodyHandleC, in TDescription description)
            where TDescription : unmanaged, IThreeBodyConstraintDescription<TDescription>
        {
            Span<BodyHandle> bodyHandles = stackalloc BodyHandle[] { bodyHandleA, bodyHandleB, bodyHandleC };
            return Add(bodyHandles, description);
        }

        /// <summary>
        /// Allocates a four-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandleA">First body of the constraint.</param>
        /// <param name="bodyHandleB">Second body of the constraint.</param>
        /// <param name="bodyHandleC">Third body of the constraint.</param>
        /// <param name="bodyHandleD">Fourth body of the constraint.</param>
        /// <param name="description">Description of the constraint to add.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe ConstraintHandle Add<TDescription>(BodyHandle bodyHandleA, BodyHandle bodyHandleB, BodyHandle bodyHandleC, BodyHandle bodyHandleD, in TDescription description)
            where TDescription : unmanaged, IFourBodyConstraintDescription<TDescription>
        {
            Span<BodyHandle> bodyHandles = stackalloc BodyHandle[] { bodyHandleA, bodyHandleB, bodyHandleC, bodyHandleD };
            return Add(bodyHandles, description);
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
                            lastBatch.Dispose(pool);
                            batchReferencedHandles[lastBatchIndex].Dispose(pool);
                            --batchReferencedHandles.Count;
                            --set.Batches.Count;
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
        internal unsafe void RemoveFromBatch(int batchIndex, int typeId, int indexInTypeBatch)
        {
            ref var batch = ref ActiveSet.Batches[batchIndex];
            if (batchIndex == FallbackBatchThreshold)
            {
                //Note that we have to remove from fallback first because it accesses the batch's information.
                ActiveSet.SequentialFallback.Remove(this, pool, ref batch, ref batchReferencedHandles[batchIndex], typeId, indexInTypeBatch);
            }
            else
            {
                batch.RemoveBodyHandlesFromBatchForConstraint(typeId, indexInTypeBatch, batchIndex, this);
            }
            batch.Remove(typeId, indexInTypeBatch, batchIndex == FallbackBatchThreshold, this);
            RemoveBatchIfEmpty(ref batch, batchIndex);
        }

        /// <summary>
        /// Enumerates the bodies attached to an active constraint and removes the constraint's handle from all of the connected body constraint reference lists.
        /// </summary>
        struct RemoveConstraintReferencesFromBodiesEnumerator : IForEach<int>
        {
            internal Solver solver;
            internal ConstraintHandle constraintHandle;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int encodedBodyIndex)
            {
                var bodyIndex = encodedBodyIndex & Bodies.BodyReferenceMask;
                //Note that this only looks in the active set. Directly removing inactive objects is unsupported- removals and adds activate all involved islands.
                if (solver.bodies.RemoveConstraintReference(bodyIndex, constraintHandle) && Bodies.IsEncodedKinematicReference(encodedBodyIndex))
                {
                    var removed = solver.ConstrainedKinematicHandles.FastRemove(solver.bodies.ActiveSet.IndexToHandle[bodyIndex].Value);
                    Debug.Assert(removed, "If we just removed the last constraint from a kinematic, then the constrained kinematics set must have contained the body handle so it can be removed.");
                }
            }
        }

        /// <summary>
        /// Removes the constraint associated with the given handle. Note that this may invalidate any outstanding direct constraint references
        /// by reordering the constraints within the TypeBatch subject to removal.
        /// </summary>
        /// <param name="handle">Handle of the constraint to remove from the solver.</param>
        public void Remove(ConstraintHandle handle)
        {
            AssertConstraintHandleExists(handle);
            ref var constraintLocation = ref HandleToConstraint[handle.Value];
            if (constraintLocation.SetIndex > 0)
            {
                //In order to remove a constraint, it must be active.
                awakener.AwakenConstraint(handle);
            }
            Debug.Assert(constraintLocation.SetIndex == 0);
            RemoveConstraintReferencesFromBodiesEnumerator enumerator;
            enumerator.solver = this;
            enumerator.constraintHandle = handle;
            EnumerateConnectedRawBodyReferences(handle, ref enumerator);

            pairCache.RemoveReferenceIfContactConstraint(handle, constraintLocation.TypeId);
            RemoveFromBatch(constraintLocation.BatchIndex, constraintLocation.TypeId, constraintLocation.IndexInTypeBatch);
            //A negative set index marks a slot in the handle->constraint mapping as unused. The other values are considered undefined.
            constraintLocation.SetIndex = -1;
            HandlePool.Return(handle.Value, pool);
        }

        /// <summary>
        /// Gets the constraint description associated with a constraint reference.
        /// </summary>
        /// <typeparam name="TConstraintDescription">Type of the constraint description to retrieve.</typeparam>
        /// <param name="constraintReference">Reference to the constraint to retrieve.</param>
        /// <param name="description">Retrieved description of the constraint.</param>
        public void GetDescription<TConstraintDescription>(ConstraintReference constraintReference, out TConstraintDescription description)
            where TConstraintDescription : unmanaged, IConstraintDescription<TConstraintDescription>
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            Debug.Assert(constraintReference.TypeBatch.TypeId == default(TConstraintDescription).ConstraintTypeId, "Constraint type associated with the TConstraintDescription generic type parameter must match the type of the constraint in the solver.");
            default(TConstraintDescription).BuildDescription(ref constraintReference.TypeBatch, bundleIndex, innerIndex, out description);
        }

        /// <summary>
        /// Gets the constraint description associated with a constraint handle.
        /// </summary>
        /// <typeparam name="TConstraintDescription">Type of the constraint description to retrieve.</typeparam>
        /// <param name="handle">Handle of the constraint to retrieve.</param>
        /// <param name="description">Retrieved description of the constraint.</param>
        public void GetDescription<TConstraintDescription>(ConstraintHandle handle, out TConstraintDescription description)
            where TConstraintDescription : unmanaged, IConstraintDescription<TConstraintDescription>
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            ref var location = ref HandleToConstraint[handle.Value];
            Debug.Assert(default(TConstraintDescription).ConstraintTypeId == location.TypeId, "Constraint type associated with the TConstraintDescription generic type parameter must match the type of the constraint in the solver.");
            ref var typeBatch = ref Sets[location.SetIndex].Batches[location.BatchIndex].GetTypeBatch(location.TypeId);
            BundleIndexing.GetBundleIndices(location.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            default(TConstraintDescription).BuildDescription(ref typeBatch, bundleIndex, innerIndex, out description);
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
                ref var constraintLocation = ref HandleToConstraint[constraint.ConnectingConstraintHandle.Value];
                //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
                //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
                //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
                var bodyIsKinematic = TypeProcessors[constraintLocation.TypeId].UpdateForBodyMemoryMove(
                    ref ActiveSet.Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId),
                    constraintLocation.IndexInTypeBatch, constraint.BodyIndexInConstraint, newIndex);
                //Note that only dynamic bodies 
                if (!bodyIsKinematic && constraintLocation.BatchIndex == FallbackBatchThreshold)
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
                //One of the moved constraints involved the fallback batch, and this body was dynamic, so we need to update the fallback batch's body indices.
                ActiveSet.SequentialFallback.UpdateForDynamicBodyMemoryMove(originalBodyIndex, newBodyLocation);
            }
        }

        //TODO: Using a non-fixed time step isn't ideal to begin with, but these scaling functions are worse than they need to be.
        //Unfortunately, the faster alternative is quite a bit more complex- the accumulated impulses would need to be scaled alongside the warm start to minimize memory bandwidth.
        //Plus, none of this uses multithreading.
        //Inactive sets are more difficult- an option would be to store scale on a per-set basis and accumulate it, and then only handle the scaling when it becomes active.

        /// <summary>
        /// Scales the accumulated impulses associated with a constraint set by a given scale.
        /// </summary>
        /// <param name="set">Set to scale.</param>
        /// <param name="scale">Scale to apply to accumulated impulses.</param>
        public void ScaleAccumulatedImpulses(ref ConstraintSet set, float scale)
        {
            for (int batchIndex = 0; batchIndex < ActiveSet.Batches.Count; ++batchIndex)
            {
                ref var batch = ref ActiveSet.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    TypeProcessors[typeBatch.TypeId].ScaleAccumulatedImpulses(ref typeBatch, scale);
                }
            }
        }

        /// <summary>
        /// Scales all accumulated impulses in the active set.
        /// </summary>
        /// <param name="scale">Scale to apply to accumulated impulses.</param>
        public void ScaleActiveAccumulatedImpulses(float scale)
        {
            ScaleAccumulatedImpulses(ref ActiveSet, scale);
        }

        /// <summary>
        /// Scales all accumulated impulses in all constraint sets.
        /// </summary>
        /// <param name="scale">Scale to apply to accumulated impulses.</param>
        public void ScaleAccumulatedImpulses(float scale)
        {
            for (int i = 0; i < Sets.Length; ++i)
            {
                ref var set = ref Sets[i];
                if (set.Allocated)
                    ScaleAccumulatedImpulses(ref set, scale);
            }
        }

        /// <summary>
        /// Enumerates the accumulated impulses associated with a constraint.
        /// </summary>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to use.</param>
        public void EnumerateAccumulatedImpulses<TEnumerator>(ConstraintHandle constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<float>
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle.Value];
            ref var typeBatch = ref Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            Debug.Assert(constraintLocation.IndexInTypeBatch >= 0 && constraintLocation.IndexInTypeBatch < typeBatch.ConstraintCount, "Bad constraint location; likely some add/remove bug.");
            TypeProcessors[constraintLocation.TypeId].EnumerateAccumulatedImpulses(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
        }

        /// <summary>
        /// Gathers the squared magnitude of the accumulated impulse for a given constraint.
        /// </summary>
        /// <param name="constraintHandle">Constraint to look up the accumulated impulses of.</param>
        /// <returns>Squared magnitude of the accumulated impulses associated with the given constraint.</returns>
        public unsafe float GetAccumulatedImpulseMagnitudeSquared(ConstraintHandle constraintHandle)
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle.Value];
            ref var typeBatch = ref Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            Debug.Assert(constraintLocation.IndexInTypeBatch >= 0 && constraintLocation.IndexInTypeBatch < typeBatch.ConstraintCount, "Bad constraint location; likely some add/remove bug.");
            var typeProcessor = TypeProcessors[constraintLocation.TypeId];
            var impulses = stackalloc float[typeProcessor.ConstrainedDegreesOfFreedom];
            var floatCollector = new FloatCollector(impulses);
            TypeProcessors[constraintLocation.TypeId].EnumerateAccumulatedImpulses(ref typeBatch, constraintLocation.IndexInTypeBatch, ref floatCollector);
            var sumOfSquares = 0f;
            for (int i = 0; i < typeProcessor.ConstrainedDegreesOfFreedom; ++i)
            {
                var impulse = impulses[i];
                sumOfSquares += impulse * impulse;
            }
            return sumOfSquares;
        }

        /// <summary>
        /// Gathers the magnitude of the accumulated impulse for a given constraint.
        /// </summary>
        /// <param name="constraintHandle">Constraint to look up the accumulated impulses of.</param>
        /// <returns>Magnitude of the accumulated impulses associated with the given constraint.</returns>
        public unsafe float GetAccumulatedImpulseMagnitude(ConstraintHandle constraintHandle)
        {
            return (float)Math.Sqrt(GetAccumulatedImpulseMagnitudeSquared(constraintHandle));
        }


        internal void TryRemoveDynamicBodyFromFallback(BodyHandle bodyHandle, int bodyIndex, ref QuickList<int> allocationIdsToFree)
        {
            if (ActiveSet.SequentialFallback.TryRemoveDynamicBodyFromTracking(bodyIndex, ref allocationIdsToFree))
            {
                Debug.Assert(batchReferencedHandles[FallbackBatchThreshold].Contains(bodyHandle.Value) || bodies[bodyHandle].Kinematic,
                    "The batch referenced handles must include all constraint-involved dynamics, but will not include kinematics.");
                batchReferencedHandles[FallbackBatchThreshold].Unset(bodyHandle.Value);
            }
        }

        private struct DynamicToKinematicEnumerator : IForEach<int>
        {
            public int DynamicCount;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void LoopBody(int encodedBodyReference)
            {
                ++DynamicCount;
            }
        }
        internal unsafe void UpdateReferencesForBodyBecomingKinematic(BodyHandle bodyHandle, int bodyIndex)
        {
            Debug.Assert(bodies[bodyHandle].Kinematic);
            //Any constraints that connect only kinematic bodies together should be removed; they'll NaN out.
            //Ideally, the user would handle this for all non-contact constraints, but it would be rather annoying to 
            //have to explicitly enumerate and remove all contact constraints any time you wanted to make a body kinematic.
            DynamicToKinematicEnumerator enumerator;
            ref var constraints = ref bodies.ActiveSet.Constraints[bodyIndex];
            bool presentInFallback = false;
            //Note reverse iteration. If the solver removes a constraint for now being between two kinematics, you don't want to break enumeration.
            for (int i = constraints.Count - 1; i >= 0; --i)
            {
                ref var constraint = ref constraints[i];
                var constraintHandle = constraint.ConnectingConstraintHandle;
                enumerator.DynamicCount = 0;
                EnumerateConnectedDynamicBodies(constraint.ConnectingConstraintHandle, ref enumerator);
                if (enumerator.DynamicCount == 1)
                {
                    //Given that *this* body is becoming kinematic, this constraint now connects only kinematic bodies; keeping it in the solver would cause a singularity.
                    Remove(constraintHandle);
                }
                else
                {
                    //The constraint survived, so update its kinematicity flag for this body.
                    var location = HandleToConstraint[constraintHandle.Value];
                    AssertConstraintHandleExists(constraintHandle);
                    var typeBatch = Sets[location.SetIndex].Batches[location.BatchIndex].GetTypeBatchPointer(location.TypeId);
                    var bodiesPerConstraint = TypeProcessors[location.TypeId].BodiesPerConstraint;
                    var intsPerBundle = Vector<int>.Count * bodiesPerConstraint;
                    BundleIndexing.GetBundleIndices(location.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
                    var firstBodyReference = (uint*)typeBatch->BodyReferences.Memory + intsPerBundle * bundleIndex + innerIndex;
                    ref var bodyReferenceSlot = ref firstBodyReference[constraint.BodyIndexInConstraint * Vector<int>.Count];
                    var oldDynamicIndex = bodyReferenceSlot;
                    bodyReferenceSlot = oldDynamicIndex | Bodies.KinematicMask;
                    if (location.BatchIndex < FallbackBatchThreshold)
                    {
                        //If this isn't a fallback batch, then the former dynamic was the only reference to the body in the batch and the reference should be removed to avoid blocking other bodies.
                        batchReferencedHandles[location.BatchIndex].Remove(bodyHandle.Value);
                    }
                    else
                    {
                        presentInFallback = true;
                    }
                }
            }
            if (presentInFallback)
            {
                //Detected at least one constraint in the fallback. Since the body is now kinematic, *no* constraint in the fallback can have a reference to it, so just remove the body.
                var ids = stackalloc int[3];
                QuickList<int> allocationIdsToFree = new(new Buffer<int>(ids, 3));
                TryRemoveDynamicBodyFromFallback(bodyHandle, bodyIndex, ref allocationIdsToFree);
                for (int i = 0; i < allocationIdsToFree.Count; ++i)
                {
                    pool.ReturnUnsafely(allocationIdsToFree[i]);
                }
            }
            if (constraints.Count > 0)
            {
                //This body is now kinematic, and remains constrained. Stick it in the constrained kinematics set.
                ConstrainedKinematicHandles.Add(bodyHandle.Value, pool);
            }
            ValidateConstrainedKinematicsSet();

        }


        private unsafe struct KinematicToDynamicEnumerator : IForEach<int>
        {
            public const int MaximumBodiesPerConstraint = 4;

            public Buffer<BodyHandle> IndexToHandle;
            public int* DynamicBodyHandles;
            public int DynamicCount;
            public int* EncodedBodyIndices;
            public int EncodedCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void LoopBody(int encodedBodyReference)
            {
                Debug.Assert(EncodedCount < MaximumBodiesPerConstraint, "We assumed that the number of bodies per constraint was limited; if this assumption fails, it could cause a stack overrun.");
                if (Bodies.IsEncodedDynamicReference(encodedBodyReference))
                {
                    DynamicBodyHandles[DynamicCount++] = IndexToHandle[encodedBodyReference].Value;
                }
                EncodedBodyIndices[EncodedCount++] = encodedBodyReference;
            }
        }

        internal unsafe void UpdateReferencesForBodyBecomingDynamic(BodyHandle bodyHandle, int bodyIndex)
        {
            //A kinematic body has become dynamic. Kinematic bodies do not block membership in constraint batches, dynamic bodies do.
            //For any constraint connected to the new dynamic, ensure that it belongs to a constraint batch not shared by any other constraints connected to the same body.
            int* dynamicBodyHandles = stackalloc int[KinematicToDynamicEnumerator.MaximumBodiesPerConstraint];
            int* encodedBodyIndices = stackalloc int[KinematicToDynamicEnumerator.MaximumBodiesPerConstraint];
            KinematicToDynamicEnumerator enumerator;
            enumerator.IndexToHandle = bodies.ActiveSet.IndexToHandle;
            enumerator.DynamicBodyHandles = dynamicBodyHandles;
            enumerator.EncodedBodyIndices = encodedBodyIndices;
            var indexToHandle = bodies.ActiveSet.IndexToHandle;
            var handleToConstraint = HandleToConstraint;
            ref var constraints = ref bodies.ActiveSet.Constraints[bodyIndex];
            for (int constraintIndex = 0; constraintIndex < constraints.Count; ++constraintIndex)
            {
                ref var constraint = ref constraints[constraintIndex];
                enumerator.DynamicCount = 0;
                enumerator.EncodedCount = 0;
                EnumerateConnectedRawBodyReferences(constraint.ConnectingConstraintHandle, ref enumerator);
                //Since we haven't updated the constraint reference to this body's kinematicity yet, it was not included in the dynamicBodyHandles. 
                //Include it here.
                dynamicBodyHandles[enumerator.DynamicCount++] = bodyHandle.Value;
                //Remove the kinematic flag from the body's encoded index. Updating this before attempting to transfer the constraint ensures that the proper flags get stored in the new location.
                encodedBodyIndices[constraint.BodyIndexInConstraint] &= Bodies.BodyReferenceMask;
                var dynamicBodyHandlesSpan = new Span<int>(dynamicBodyHandles, enumerator.DynamicCount);
                var encodedBodyIndicesSpan = new Span<int>(encodedBodyIndices, enumerator.EncodedCount);
                GetSynchronizedBatchCount(out var synchronizedBatchCount, out var fallbackExists);
                var constraintLocation = handleToConstraint[constraint.ConnectingConstraintHandle.Value];
                ref var batch = ref ActiveSet.Batches[constraintLocation.BatchIndex];
                ref var typeBatch = ref batch.TypeBatches[batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId]];
                int targetBatchIndex = -1;

                for (int batchIndex = 0; batchIndex < synchronizedBatchCount; ++batchIndex)
                {
                    if (batchReferencedHandles[batchIndex].CanFit(dynamicBodyHandlesSpan))
                    {
                        //Because we haven't removed the constraint from the simulation, it's currently blocking the constraint batch it previously lived in.
                        //It must have had at least one other dynamic before (otherwise it would have violated the 'constraints must have at least one dynamic in them' rule), so that body will block it.
                        //This causes a little bit of batch inefficiency, but the batch compressor will take care of it eventually and this codepath is reasonably rare- the simplicity of reusing TransferConstraint wins.
                        Debug.Assert(batchIndex != constraintLocation.BatchIndex, "It should not be possible for a newly dynamic reference to insert itself into the same batch it was in while kinematic.");
                        targetBatchIndex = batchIndex;
                        break;
                    }
                }
                if (targetBatchIndex == -1)
                {
                    //Still need a batch. 
                    if (fallbackExists)
                    {
                        targetBatchIndex = FallbackBatchThreshold;
                    }
                    else
                    {
                        //No batch has been found that can hold the constraint, but there is room for additional constraint batches.
                        targetBatchIndex = AllocateNewConstraintBatch();
                    }
                }
                //Perform the transfer!
                //Note that there's no need to strip kinematic flags- we stripped the flag appropriately when we created the encodedBodyIndices earlier, and those were the values that got stuck into the new allocation.
                TypeProcessors[constraintLocation.TypeId].TransferConstraint(ref typeBatch, constraintLocation.BatchIndex, constraintLocation.IndexInTypeBatch, this, bodies, targetBatchIndex,
                    new Span<BodyHandle>(dynamicBodyHandles, enumerator.DynamicCount), encodedBodyIndicesSpan);
            }
            if (constraints.Count > 0)
            {
                ConstrainedKinematicHandles.FastRemove(bodyHandle.Value);
            }
        }

        internal interface IConstraintReferenceReportType { }
        internal struct ReportEncodedReferences : IConstraintReferenceReportType { }
        internal struct ReportDecodedReferences : IConstraintReferenceReportType { }
        internal struct ReportDecodedDynamicReferences : IConstraintReferenceReportType { }

        /// <summary>
        /// Enumerates body references in the constraint. Reports data according to the TReportType.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator called for each body index in the constraint.</typeparam>
        /// <typeparam name="TReportType">Type of information to report to the enumerator.</typeparam>
        /// <param name="typeBatch">Type batch containing the constraint to enumerate.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to enumerate in the type batch.</param>
        /// <param name="enumerator">Enumerator to call for each connected body reference.</param>
        internal unsafe void EnumerateConnectedBodyReferences<TEnumerator, TReportType>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int> where TReportType : unmanaged, IConstraintReferenceReportType
        {
            var bodiesPerConstraint = TypeProcessors[typeBatch.TypeId].BodiesPerConstraint;
            //Type batches store body references in AOSOA format, with one Vector<int> for each constraint body reference in sequence, tightly packed.
            //We can extract directly from memory.
            var bytesPerBundle = bodiesPerConstraint * Unsafe.SizeOf<Vector<int>>();
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var bundleIndex, out var innerIndex);
            Debug.Assert(bytesPerBundle * typeBatch.BundleCount <= typeBatch.BodyReferences.Length, "Buffer must be large enough to hold the bundles of our assumed size. If this fails, an important assumption has been invalidated somewhere.");
            var startByte = bundleIndex * bytesPerBundle + innerIndex * 4;
            for (int i = 0; i < bodiesPerConstraint; ++i)
            {
                var raw = *(int*)(typeBatch.BodyReferences.Memory + startByte + i * Unsafe.SizeOf<Vector<int>>());
                if (typeof(TReportType) == typeof(ReportEncodedReferences))
                {
                    enumerator.LoopBody(raw);
                }
                else if (typeof(TReportType) == typeof(ReportDecodedReferences))
                {
                    enumerator.LoopBody(raw & Bodies.BodyReferenceMask);
                }
                else if (typeof(TReportType) == typeof(ReportDecodedDynamicReferences))
                {
                    if (Bodies.IsEncodedDynamicReference(raw))
                        enumerator.LoopBody(raw & Bodies.BodyReferenceMask);
                }
            }
        }
        /// <summary>
        /// Enumerates the set of body references associated with a constraint in order of their references within the constraint.
        /// This will report the raw body reference (body index if awake, handle if asleep) and any encoded metadata, like whether the body is kinematic.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to call on each connected body reference.</typeparam>
        /// <param name="typeBatch">Type batch containing the constraint to enumerate.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to enumerate in the type batch.</param>
        /// <param name="enumerator">Enumerator to call for each connected body reference.</param>
        public unsafe void EnumerateConnectedRawBodyReferences<TEnumerator>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            EnumerateConnectedBodyReferences<TEnumerator, ReportEncodedReferences>(ref typeBatch, indexInTypeBatch, ref enumerator);
        }

        /// <summary>
        /// Enumerates the set of body references associated with a constraint in order of their references within the constraint.
        /// This will report the raw body reference (body index if awake, handle if asleep) and any encoded metadata, like whether the body is kinematic.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to call on each connected body reference.</typeparam>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to call for each connected body reference.</param>
        public unsafe void EnumerateConnectedRawBodyReferences<TEnumerator>(ConstraintHandle constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle.Value];
            ref var typeBatch = ref Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            Debug.Assert(constraintLocation.IndexInTypeBatch >= 0 && constraintLocation.IndexInTypeBatch < typeBatch.ConstraintCount, "Bad constraint location; likely some add/remove bug.");
            EnumerateConnectedBodyReferences<TEnumerator, ReportEncodedReferences>(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
        }

        /// <summary>
        /// Enumerates the set of body references associated with an active constraint in order of their references within the constraint.
        /// This will report the body reference (body index if awake, handle if asleep) without any encoded kinematicity metadata.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to call on each connected body reference.</typeparam>
        /// <param name="typeBatch">Type batch containing the constraint to enumerate.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to enumerate in the type batch.</param>
        /// <param name="enumerator">Enumerator to call for each connected body reference.</param>
        public unsafe void EnumerateConnectedBodyReferences<TEnumerator>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            EnumerateConnectedBodyReferences<TEnumerator, ReportDecodedReferences>(ref typeBatch, indexInTypeBatch, ref enumerator);
        }

        /// <summary>
        /// Enumerates the set of body references associated with an active constraint in order of their references within the constraint.
        /// This will report the body reference (body index if awake, handle if asleep) without any encoded kinematicity metadata.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to call on each connected body reference.</typeparam>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to call for each connected body reference.</param>
        public unsafe void EnumerateConnectedBodyReferences<TEnumerator>(ConstraintHandle constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle.Value];
            ref var typeBatch = ref Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            Debug.Assert(constraintLocation.IndexInTypeBatch >= 0 && constraintLocation.IndexInTypeBatch < typeBatch.ConstraintCount, "Bad constraint location; likely some add/remove bug.");
            EnumerateConnectedBodyReferences<TEnumerator, ReportDecodedReferences>(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
        }

        /// <summary>
        /// Enumerates the set of dynamic body references associated with a constraint in order of their references within the constraint.
        /// This will report the body reference (body index if awake, handle if asleep) without any encoded kinematicity metadata.
        /// Kinematic references are skipped.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to call on each connected dynamic body reference.</typeparam>
        /// <param name="typeBatch">Type batch containing the constraint to enumerate.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to enumerate in the type batch.</param>
        /// <param name="enumerator">Enumerator to call for each connected dynamic body reference.</param>
        public unsafe void EnumerateConnectedDynamicBodies<TEnumerator>(ref TypeBatch typeBatch, int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            EnumerateConnectedBodyReferences<TEnumerator, ReportDecodedDynamicReferences>(ref typeBatch, indexInTypeBatch, ref enumerator);
        }

        /// <summary>
        /// Enumerates the set of dynamic body references associated with a constraint in order of their references within the constraint.
        /// This will report the body reference (body index if awake, handle if asleep) without any encoded kinematicity metadata.
        /// Kinematic references are skipped.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to call on each connected dynamic body reference.</typeparam>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to call for each connected dynamic body reference.</param>
        public unsafe void EnumerateConnectedDynamicBodies<TEnumerator>(ConstraintHandle constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle.Value];
            ref var typeBatch = ref Sets[constraintLocation.SetIndex].Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            Debug.Assert(constraintLocation.IndexInTypeBatch >= 0 && constraintLocation.IndexInTypeBatch < typeBatch.ConstraintCount, "Bad constraint location; likely some add/remove bug.");
            EnumerateConnectedBodyReferences<TEnumerator, ReportDecodedDynamicReferences>(ref typeBatch, constraintLocation.IndexInTypeBatch, ref enumerator);
        }

        internal void GetSynchronizedBatchCount(out int synchronizedBatchCount, out bool fallbackExists)
        {
            synchronizedBatchCount = Math.Min(ActiveSet.Batches.Count, FallbackBatchThreshold);
            fallbackExists = ActiveSet.Batches.Count > FallbackBatchThreshold;
            Debug.Assert(ActiveSet.Batches.Count <= FallbackBatchThreshold + 1,
                "There cannot be more than FallbackBatchThreshold + 1 constraint batches because that +1 is the fallback batch which contains all remaining constraints.");
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
            for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
            {
                batchReferencedHandles[batchIndex].Dispose(pool);
            }
            ConstrainedKinematicHandles.Clear();
            batchReferencedHandles.Clear();
            ActiveSet.Clear(pool);
            //All inactive sets are returned to the pool.
            //Their allocations are always created to fit the actual island size.
            for (int i = 1; i < Sets.Length; ++i)
            {
                if (Sets[i].Allocated)
                {
                    Sets[i].Dispose(pool);
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
                pool.ResizeToAtLeast(ref HandleToConstraint, constraintHandleCapacity, HandlePool.HighestPossiblyClaimedId + 1);
            }
            //Note that we can't shrink below the bodies handle capacity, since the handle distribution could be arbitrary.
            var targetBatchReferencedHandleSize = Math.Max(bodies.HandlePool.HighestPossiblyClaimedId + 1, bodyHandleCapacity);
            for (int i = 0; i < ActiveSet.Batches.Count; ++i)
            {
                batchReferencedHandles[i].EnsureCapacity(targetBatchReferencedHandleSize, pool);
            }

            ConstrainedKinematicHandles.EnsureCapacity(bodyHandleCapacity, pool);
        }

        void ResizeHandleCapacity(int constraintHandleCapacity)
        {
            pool.ResizeToAtLeast(ref HandleToConstraint, constraintHandleCapacity, HandlePool.HighestPossiblyClaimedId + 1);
            for (int i = HandlePool.HighestPossiblyClaimedId + 1; i < HandleToConstraint.Length; ++i)
            {
                //A negative set index marks a slot as unused.
                HandleToConstraint[i].SetIndex = -1;
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
            var targetConstraintCount = BufferPool.GetCapacityForCount<ConstraintLocation>(Math.Max(constraintHandleCapacity, HandlePool.HighestPossiblyClaimedId + 1));
            if (HandleToConstraint.Length != targetConstraintCount)
            {
                ResizeHandleCapacity(targetConstraintCount);
            }
            //Note that we can't shrink below the bodies handle capacity, since the handle distribution could be arbitrary.
            var targetBatchReferencedHandleSize = Math.Max(bodies.HandlePool.HighestPossiblyClaimedId + 1, bodyHandleCapacity);
            for (int i = 0; i < ActiveSet.Batches.Count; ++i)
            {
                batchReferencedHandles[i].Resize(targetBatchReferencedHandleSize, pool);
            }

            var targetConstrainedKinematicsCapacity = Math.Max(ConstrainedKinematicHandles.Count, bodyHandleCapacity);
            ConstrainedKinematicHandles.Resize(targetConstrainedKinematicsCapacity, pool);
        }

        internal void ResizeSetsCapacity(int setsCapacity, int potentiallyAllocatedCount)
        {
            Debug.Assert(setsCapacity >= potentiallyAllocatedCount && potentiallyAllocatedCount <= Sets.Length);
            setsCapacity = BufferPool.GetCapacityForCount<ConstraintSet>(setsCapacity);
            if (Sets.Length != setsCapacity)
            {
                var oldCapacity = Sets.Length;
                pool.ResizeToAtLeast(ref Sets, setsCapacity, potentiallyAllocatedCount);
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
            for (int i = 0; i < ActiveSet.Batches.Count; ++i)
            {
                batchReferencedHandles[i].Dispose(pool);
            }
            batchReferencedHandles.Dispose(pool);
            ConstrainedKinematicHandles.Dispose(pool);
            for (int i = 0; i < Sets.Length; ++i)
            {
                if (Sets[i].Allocated)
                    Sets[i].Dispose(pool);
            }
            pool.Return(ref Sets);
            pool.Return(ref HandleToConstraint);
            HandlePool.Dispose(pool);
        }
    }
}
