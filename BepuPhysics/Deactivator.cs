using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics
{
    public class Deactivator
    {
        IdPool<Buffer<int>> setIdPool;
        Bodies bodies;
        Solver solver;
        BroadPhase broadPhase;
        internal PairCache pairCache;
        ConstraintRemover constraintRemover;
        BufferPool pool;
        public int InitialIslandBodyCapacity { get; set; } = 1024;
        public int InitialIslandConstraintCapacity { get; set; } = 1024;

        /// <summary>
        /// Gets or sets the multiplier applied to the active body count used to calculate the number of deactivation traversals in a given timestep.
        /// </summary>
        public float TestedFractionPerFrame { get; set; } = 0.01f;
        /// <summary>
        /// Gets or sets the fraction of the active set to target as the number of bodies deactivated in a given frame.
        /// This is only a goal; the actual number of deactivated bodies may be more or less.
        /// </summary>
        public float TargetDeactivatedFraction { get; set; } = 0.005f;
        /// <summary>
        /// Gets or sets the fraction of the active set to target as the number of bodies traversed for deactivation in a given frame.
        /// This is only a goal; the actual number of traversed bodies may be more or less.
        /// </summary>
        public float TargetTraversedFraction { get; set; } = 0.02f;

        public Deactivator(Bodies bodies, Solver solver, BroadPhase broadPhase, ConstraintRemover constraintRemover, BufferPool pool)
        {
            this.bodies = bodies;
            this.solver = solver;
            this.broadPhase = broadPhase;
            this.constraintRemover = constraintRemover;
            this.pool = pool;
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), 16, out setIdPool);
            //We reserve index 0 for the active set.
            setIdPool.Take();
            findIslandsDelegate = FindIslands;
            gatherDelegate = Gather;
            typeBatchConstraintRemovalDelegate = TypeBatchConstraintRemoval;
            executeRemovalWorkDelegate = ExecuteRemovalWork;
        }

        struct ConstraintBodyEnumerator : IForEach<int>
        {
            public QuickList<int, Buffer<int>> ConstraintBodyIndices;
            public BufferPool<int> IntPool;
            public int SourceIndex;
            public void LoopBody(int bodyIndex)
            {
                if (bodyIndex != SourceIndex)
                {
                    ConstraintBodyIndices.Add(bodyIndex, IntPool);
                }
            }
        }



        struct ForcedDeactivationPredicate : IPredicate<int>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Matches(ref int bodyIndex)
            {
                return true;
            }
        }
        struct DeactivationPredicate : IPredicate<int>
        {
            public Bodies Bodies;
            public IndexSet PreviouslyTraversedBodies;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Matches(ref int bodyIndex)
            {
                //Note that we block traversals on a single thread from retreading old ground.
                if (PreviouslyTraversedBodies.Contains(bodyIndex))
                    return false;
                //Note that it is safe to add to the previously traversed body set. The current traversal's consideredBodies set is tested first.
                //If it gets a hit, the predicate isn't executed, but the traversal doesn't stop.
                PreviouslyTraversedBodies.AddUnsafely(bodyIndex);
                return Bodies.ActiveSet.Activity[bodyIndex].DeactivationCandidate;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool PushBody<TTraversalPredicate>(int bodyIndex, ref IndexSet consideredBodies, ref QuickList<int, Buffer<int>> bodyIndices, ref QuickList<int, Buffer<int>> visitationStack,
            ref BufferPool<int> intPool, ref TTraversalPredicate predicate) where TTraversalPredicate : IPredicate<int>
        {
            if (!consideredBodies.Contains(bodyIndex))
            {
                if (!predicate.Matches(ref bodyIndex))
                {
                    return false;
                }
                //This body has not yet been traversed. Push it onto the stack.
                bodyIndices.Add(bodyIndex, intPool);
                consideredBodies.AddUnsafely(bodyIndex);
                visitationStack.Add(bodyIndex, intPool);
            }
            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool EnqueueUnvisitedNeighbors<TTraversalPredicate>(int bodyIndex,
            ref QuickList<int, Buffer<int>> bodyIndices,
            ref QuickList<int, Buffer<int>> constraintHandles,
            ref IndexSet consideredBodies, ref IndexSet consideredConstraints,
            ref QuickList<int, Buffer<int>> visitationStack,
            ref ConstraintBodyEnumerator bodyEnumerator,
            ref BufferPool<int> intPool, ref TTraversalPredicate predicate) where TTraversalPredicate : IPredicate<int>
        {
            bodyEnumerator.SourceIndex = bodyIndex;
            ref var list = ref bodies.ActiveSet.Constraints[bodyIndex];
            for (int i = 0; i < list.Count; ++i)
            {
                ref var entry = ref list[i];
                if (!consideredConstraints.Contains(entry.ConnectingConstraintHandle))
                {
                    //This constraint has not yet been traversed. Follow the constraint to every other connected body.
                    constraintHandles.Add(entry.ConnectingConstraintHandle, intPool);
                    consideredConstraints.AddUnsafely(entry.ConnectingConstraintHandle);
                    bodyEnumerator.ConstraintBodyIndices.Count = 0;
                    solver.EnumerateConnectedBodies(entry.ConnectingConstraintHandle, ref bodyEnumerator);
                    for (int j = 0; j < bodyEnumerator.ConstraintBodyIndices.Count; ++j)
                    {
                        var connectedBodyIndex = bodyEnumerator.ConstraintBodyIndices[j];
                        if (!PushBody(connectedBodyIndex, ref consideredBodies, ref bodyIndices, ref visitationStack, ref intPool, ref predicate))
                            return false;
                    }
                }
            }
            return true;
        }

        /// <summary>
        /// Traverses the active constraint graph collecting bodies that match a predicate. If any body visited during the traversal fails to match the predicate, the traversal terminates.
        /// </summary>
        /// <typeparam name="TTraversalPredicate">Type of the predicate to test each body index with.</typeparam>
        /// <param name="pool">Pool to allocate temporary collections from.</param>
        /// <param name="startingActiveBodyIndex">Index of the active body to start the traversal at.</param>
        /// <param name="predicate">Predicate to test each traversed body with. If any body results in the predicate returning false, the traversal stops and the function returns false.</param>
        /// <param name="bodyIndices">List to fill with body indices traversed during island collection. Bodies failing the predicate will not be included.</param>
        /// <param name="constraintHandles">List to fill with constraint handles traversed during island collection.</param>
        /// <returns>True if the simulation graph was traversed without ever finding a body that made the predicate return false. False if any body failed the predicate.
        /// The bodyIndices and constraintHandles lists will contain all traversed predicate-passing bodies and constraints.</returns>
        public bool CollectIsland<TTraversalPredicate>(BufferPool pool, int startingActiveBodyIndex, ref TTraversalPredicate predicate,
            ref QuickList<int, Buffer<int>> bodyIndices, ref QuickList<int, Buffer<int>> constraintHandles) where TTraversalPredicate : IPredicate<int>
        {
            Debug.Assert(startingActiveBodyIndex >= 0 && startingActiveBodyIndex < bodies.ActiveSet.Count);
            //We'll build the island by working depth-first. This means the bodies and constraints we accumulate will be stored in any inactive island by depth-first order,
            //which happens to be a pretty decent layout for cache purposes. In other words, when we wake these islands back up, bodies near each other in the graph will have 
            //a higher chance of being near each other in memory. Bodies directly connected may often end up adjacent to each other, meaning loading one body may give you the other for 'free'
            //(assuming they share a cache line).
            //The DFS order for constraints is not quite as helpful as the constraint optimizer's sort, but it's not terrible.

            //Despite being DFS, there is no guarantee that the visitation stack will be any smaller than the final island itself, and we have no way of knowing how big the island is 
            //ahead of time- except that it can't be larger than the entire active simulation.
            var intPool = pool.SpecializeFor<int>();
            var initialBodyCapacity = Math.Min(InitialIslandBodyCapacity, bodies.ActiveSet.Count);
            //Note that we track all considered bodies AND constraints. 
            //While we only need to track one of them for the purposes of traversal, tracking both allows low-overhead collection of unique bodies and constraints.
            //Note that the constraint handle set is initialized to cover the entire handle span. 
            //That's actually fine- every single object occupies only a single bit, so 131072 objects only use 16KB.
            var consideredBodies = new IndexSet(pool, bodies.ActiveSet.Count);
            var consideredConstraints = new IndexSet(pool, solver.HandlePool.HighestPossiblyClaimedId + 1);
            //The stack will store body indices.
            QuickList<int, Buffer<int>>.Create(intPool, initialBodyCapacity, out var visitationStack);

            //Start the traversal by pushing the initial body conditionally.
            if (!PushBody(startingActiveBodyIndex, ref consideredBodies, ref bodyIndices, ref visitationStack, ref intPool, ref predicate))
            {
                consideredBodies.Dispose(pool);
                consideredConstraints.Dispose(pool);
                visitationStack.Dispose(intPool);
                return false;
            }
            var enumerator = new ConstraintBodyEnumerator();
            enumerator.IntPool = intPool;
            QuickList<int, Buffer<int>>.Create(intPool, 4, out enumerator.ConstraintBodyIndices);

            bool disqualified = false;
            while (visitationStack.TryPop(out var nextIndexToVisit))
            {
                if (!EnqueueUnvisitedNeighbors(nextIndexToVisit, ref bodyIndices, ref constraintHandles, ref consideredBodies, ref consideredConstraints, ref visitationStack,
                    ref enumerator, ref intPool, ref predicate))
                {
                    disqualified = true;
                    break;
                }
            }
            enumerator.ConstraintBodyIndices.Dispose(intPool);
            consideredBodies.Dispose(pool);
            consideredConstraints.Dispose(pool);
            visitationStack.Dispose(intPool);
            return !disqualified;
        }

        int targetTraversedBodyCountPerThread;
        int targetDeactivatedBodyCountPerThread;
        QuickList<int, Buffer<int>> traversalTargetBodyIndices;
        IThreadDispatcher threadDispatcher;
        int jobIndex;


        struct WorkerTraversalResults
        {
            //Note that all these resources are allocated on per-worker pools. Be careful when disposing them.
            public IndexSet TraversedBodies;
            public QuickList<Island, Buffer<Island>> Islands;

            internal void Dispose(BufferPool pool)
            {
                for (int islandIndex = 0; islandIndex < Islands.Count; ++islandIndex)
                {
                    Islands[islandIndex].Dispose(pool);
                }
                Islands.Dispose(pool.SpecializeFor<Island>());
                TraversedBodies.Dispose(pool);
            }
        }

        Buffer<WorkerTraversalResults> workerTraversalResults;

        struct GatheringJob
        {
            public int TargetSetIndex;
            public QuickList<int, Buffer<int>> SourceIndices;
            public int StartIndex;
            public int EndIndex;
            /// <summary>
            /// If true, this job relates to a subset of body indices. If false, this job relates to a subset of constraint handles.
            /// </summary>
            public bool IsBodyJob;
            //TODO: Could unionize these, but it's not very important.
            public int TargetSetIndexInReferenceList; //Only set in body jobs.
            //Batch/typebatch indices only set in constraint jobs.
            public int TargetBatchIndex;
            public int TargetTypeBatchIndex;
        }

        QuickList<GatheringJob, Buffer<GatheringJob>> gatheringJobs;

        void FindIslands(int workerIndex, BufferPool threadPool)
        {
            Debug.Assert(workerTraversalResults.Allocated && workerTraversalResults.Length > workerIndex);
            ref var results = ref workerTraversalResults[workerIndex];
            var islandPool = threadPool.SpecializeFor<Island>();
            QuickList<Island, Buffer<Island>>.Create(islandPool, 64, out results.Islands);
            var intPool = threadPool.SpecializeFor<int>();

            QuickList<int, Buffer<int>>.Create(intPool, Math.Min(InitialIslandBodyCapacity, bodies.ActiveSet.Count), out var bodyIndices);
            QuickList<int, Buffer<int>>.Create(intPool, Math.Min(InitialIslandConstraintCapacity, solver.HandlePool.HighestPossiblyClaimedId + 1), out var constraintHandles);

            DeactivationPredicate predicate;
            predicate.Bodies = bodies;
            predicate.PreviouslyTraversedBodies = new IndexSet(threadPool, bodies.ActiveSet.Count);
            var traversedBodies = 0;
            var deactivatedBodies = 0;

            while (traversedBodies < targetTraversedBodyCountPerThread && deactivatedBodies < targetDeactivatedBodyCountPerThread)
            {
                //This thread still has some deactivation budget, so try another traversal.
                var targetIndex = Interlocked.Increment(ref jobIndex);
                if (targetIndex >= traversalTargetBodyIndices.Count)
                    break;
                var bodyIndex = traversalTargetBodyIndices[targetIndex];
                if (CollectIsland(threadPool, bodyIndex, ref predicate, ref bodyIndices, ref constraintHandles))
                {
                    //Found an island to deactivate!
                    deactivatedBodies += bodyIndices.Count;

                    //Note that the deactivation predicate refuses to visit any body that was visited in any previous traversal on this thread. 
                    //From that we know that any newly discovered island is unique *on this thread*. It's very possible that a different thread has found the same
                    //island, but we let that happen in favor of avoiding tons of sync overhead.
                    //The gathering phase will check each worker's island against all previous workers. If it's a duplicate, it will get thrown out.
                    var island = new Island(ref bodyIndices, ref constraintHandles, solver, threadPool);
                    results.Islands.Add(ref island, islandPool);
                }
                traversedBodies += bodyIndices.Count;
                bodyIndices.Count = 0;
                constraintHandles.Count = 0;
            }
            bodyIndices.Dispose(intPool);
            constraintHandles.Dispose(intPool);
            results.TraversedBodies = predicate.PreviouslyTraversedBodies;
        }

        Action<int> findIslandsDelegate;
        void FindIslands(int workerIndex)
        {
            //The only reason we separate this out is to make it easier for the main pool to be passed in if there is only a single thread.
            FindIslands(workerIndex, threadDispatcher.GetThreadMemoryPool(workerIndex));
        }

        Action<int> gatherDelegate;
        unsafe void Gather(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref jobIndex);
                if (index >= gatheringJobs.Count)
                {
                    break;
                }
                ref var job = ref gatheringJobs[index];
                if (job.IsBodyJob)
                {
                    //Load a range of bodies from the active set and store them into the target inactive body set.
                    ref var sourceSet = ref bodies.ActiveSet;
                    ref var inactiveSetReference = ref newInactiveSets[job.TargetSetIndexInReferenceList];
                    for (int targetIndex = job.StartIndex; targetIndex < job.EndIndex; ++targetIndex)
                    {
                        var sourceIndex = job.SourceIndices[targetIndex];
                        ref var targetSet = ref bodies.Sets[job.TargetSetIndex];
                        targetSet.IndexToHandle[targetIndex] = sourceSet.IndexToHandle[sourceIndex];
                        targetSet.Activity[targetIndex] = sourceSet.Activity[sourceIndex];
                        ref var sourceCollidable = ref sourceSet.Collidables[sourceIndex];
                        targetSet.Collidables[targetIndex] = sourceCollidable;
                        //Note that we are just copying the constraint list reference; we don't have to reallocate it.
                        //Keep this in mind when removing the object from the active set. We don't want to dispose the list since we're still using it.
                        targetSet.Constraints[targetIndex] = sourceSet.Constraints[sourceIndex];
                        targetSet.LocalInertias[targetIndex] = sourceSet.LocalInertias[sourceIndex];
                        targetSet.Poses[targetIndex] = sourceSet.Poses[sourceIndex];
                        targetSet.Velocities[targetIndex] = sourceSet.Velocities[sourceIndex];

                        //Gather the broad phase data so that the later active set removal phase can stick it into the static broad phase structures.
                        ref var broadPhaseData = ref inactiveSetReference.BroadPhaseData[targetIndex];
                        broadPhaseData.Reference = broadPhase.activeLeaves[sourceCollidable.BroadPhaseIndex];
                        broadPhase.GetActiveBoundsPointers(sourceCollidable.BroadPhaseIndex, out var minPtr, out var maxPtr);
                        broadPhaseData.Bounds.Min = new Vector3(minPtr[0], minPtr[1], minPtr[2]);
                        broadPhaseData.Bounds.Max = new Vector3(maxPtr[0], maxPtr[1], maxPtr[2]);
                    }
                }
                else
                {
                    //Load a range of constraints from the active set and store them into the target inactive constraint set.
                    ref var targetTypeBatch = ref solver.Sets[job.TargetSetIndex].Batches[job.TargetBatchIndex].TypeBatches[job.TargetTypeBatchIndex];
                    //We can share a single virtual dispatch over all the constraints since they are of the same type. They may, however, be in different batches.
                    solver.TypeProcessors[targetTypeBatch.TypeId].GatherActiveConstraints(bodies, solver, ref job.SourceIndices, job.StartIndex, job.EndIndex, ref targetTypeBatch);
                    //Enqueue these constraints for later removal.
                    Debug.Assert(job.StartIndex >= 0 && job.EndIndex <= targetTypeBatch.ConstraintCount && job.StartIndex < job.EndIndex);
                    for (int indexInTypeBatch = job.StartIndex; indexInTypeBatch < job.EndIndex; ++indexInTypeBatch)
                    {
                        constraintRemover.EnqueueRemoval(workerIndex, targetTypeBatch.IndexToHandle[indexInTypeBatch]);
                    }
                }
            }
        }

        int typeBatchConstraintRemovalJobCount;
        Action<int> typeBatchConstraintRemovalDelegate;

        void TypeBatchConstraintRemoval(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref jobIndex);
                if (index >= typeBatchConstraintRemovalJobCount)
                    break;
                constraintRemover.RemoveConstraintsFromTypeBatch(index);
            }
        }

        struct BroadPhaseData
        {
            public CollidableReference Reference;
            public BoundingBox Bounds;
        }

        struct InactiveSetReference
        {
            public int Index;

            public Buffer<BroadPhaseData> BroadPhaseData;
        }

        QuickList<InactiveSetReference, Buffer<InactiveSetReference>> newInactiveSets;
        QuickList<RemovalJob, Buffer<RemovalJob>> removalJobs;

        enum RemovalJobType
        {
            RemoveFromBatchReferencedHandles,
            NotifyNarrowPhasePairCache,
            AddCollidablesToStaticTree,
            RemoveBodiesFromActiveSet
        }

        struct RemovalJob
        {
            public RemovalJobType Type;
        }


        void ExecuteRemoval(ref RemovalJob job)
        {
            switch (job.Type)
            {
                case RemovalJobType.RemoveFromBatchReferencedHandles:
                    constraintRemover.RemoveConstraintsFromBatchReferencedHandles();
                    break;
                case RemovalJobType.RemoveBodiesFromActiveSet:
                    {
                        for (int setReferenceIndex = 0; setReferenceIndex < newInactiveSets.Count; ++setReferenceIndex)
                        {
                            var setIndex = newInactiveSets[setReferenceIndex].Index;
                            ref var set = ref bodies.Sets[setIndex];
                            for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                            {
                                ref var location = ref bodies.HandleToLocation[set.IndexToHandle[bodyIndex]];
                                Debug.Assert(location.SetIndex == 0, "At this point, the deactivation hasn't gone through so the set should still be 0.");
                                bodies.RemoveFromActiveSet(location.Index);
                                //And now we can actually update the handle->body mapping.
                                location.SetIndex = setIndex;
                                location.Index = bodyIndex;
                            }
                        }
                    }
                    break;
                case RemovalJobType.AddCollidablesToStaticTree:
                    {
                        for (int setReferenceIndex = 0; setReferenceIndex < newInactiveSets.Count; ++setReferenceIndex)
                        {
                            ref var setReference = ref newInactiveSets[setReferenceIndex];
                            ref var set = ref bodies.Sets[setReference.Index];
                            for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                            {
                                ref var collidable = ref set.Collidables[bodyIndex];
                                if (collidable.Shape.Exists)
                                {
                                    ref var data = ref setReference.BroadPhaseData[bodyIndex];
                                    collidable.BroadPhaseIndex = broadPhase.AddStatic(data.Reference, ref data.Bounds);
                                }
                            }
                        }
                    }
                    break;
                case RemovalJobType.NotifyNarrowPhasePairCache:
                    {
                        //This must be locally sequential because it results in removals from the pair cache's global overlap mapping and allocates from the main pool.
                        for (int setReferenceIndex = 0; setReferenceIndex < newInactiveSets.Count; ++setReferenceIndex)
                        {
                            pairCache.DeactivateTypeBatchPairs(newInactiveSets[setReferenceIndex].Index, solver);
                        }
                    }
                    break;
            }
        }
        Action<int> executeRemovalWorkDelegate;
        void ExecuteRemovalWork(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref jobIndex);
                if (index >= removalJobs.Count)
                    break;
                ExecuteRemoval(ref removalJobs[index]);
            }
        }

        struct HandleComparer : IComparerRef<int>
        {
            public Buffer<int> Handles;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref int a, ref int b)
            {
                return Handles[a].CompareTo(Handles[b]);
            }
        }

        int scheduleOffset;

        [Conditional("DEBUG")]
        unsafe void PrintIsland(ref Island island)
        {
            Console.Write($"{island.BodyIndices.Count} body handles: ");
            for (int i = 0; i < island.BodyIndices.Count; ++i)
                Console.Write($"{bodies.ActiveSet.IndexToHandle[island.BodyIndices[i]]}, ");
            Console.WriteLine();
            int constraintCount = 0;
            for (int batchIndex = 0; batchIndex < island.Protobatches.Count; ++batchIndex)
            {
                ref var batch = ref island.Protobatches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    constraintCount += typeBatch.Handles.Count;
                }
            }
            Console.Write($"{constraintCount} constraint handles: ");
            ConstraintReferenceCollector bodyIndexEnumerator;
            var intPool = pool.SpecializeFor<int>();
            QuickSet<int, Buffer<int>, Buffer<int>, PrimitiveComparer<int>>.Create(intPool, intPool, 3, 3, out var constraintReferencedBodyHandles);
            for (int batchIndex = 0; batchIndex < island.Protobatches.Count; ++batchIndex)
            {
                ref var batch = ref island.Protobatches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                    var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
                    var references = stackalloc int[typeProcessor.BodiesPerConstraint];
                    bodyIndexEnumerator.References = references;
                    for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.Handles.Count; ++indexInTypeBatch)
                    {
                        var handle = typeBatch.Handles[indexInTypeBatch];
                        ref var location = ref solver.HandleToConstraint[handle];
                        Debug.Assert(location.SetIndex == 0);
                        Debug.Assert(location.TypeId == typeBatch.TypeId);
                        ref var solverBatch = ref solver.Sets[0].Batches[location.BatchIndex];
                        bodyIndexEnumerator.Index = 0;
                        typeProcessor.EnumerateConnectedBodyIndices(
                            ref solverBatch.TypeBatches[solverBatch.TypeIndexToTypeBatchIndex[location.TypeId]], location.IndexInTypeBatch, ref bodyIndexEnumerator);
                        for (int i = 0; i < typeProcessor.BodiesPerConstraint; ++i)
                        {
                            constraintReferencedBodyHandles.Add(ref bodies.ActiveSet.IndexToHandle[references[i]], intPool, intPool);
                        }
                        Console.Write($"{handle}, ");
                    }
                }
            }
            Console.WriteLine();
            Console.Write("Unique body handles referenced by all constraints: ");
            for (int i = 0; i < constraintReferencedBodyHandles.Count; ++i)
            {
                Console.Write($"{constraintReferencedBodyHandles[i]}, ");
            }
            Console.WriteLine();
        }
        public void Update(IThreadDispatcher threadDispatcher, bool deterministic)
        {
            if (bodies.ActiveSet.Count == 0)
                return;

            //var totalHeuristic = 0f;
            //var deactivationCandidateCount = 0;
            //var totalEnergyOfNonCandidates = 0f;
            //var totalEnergyOfCandidates = 0f;
            //var highestEnergy = 0f;
            //var highestEnergyIndex = -1;
            //for (int i = 0; i < bodies.ActiveSet.Count; ++i)
            //{
            //    ref var velocities = ref bodies.ActiveSet.Velocities[i];
            //    var heuristic = velocities.Linear.LengthSquared() + velocities.Angular.LengthSquared();
            //    totalHeuristic += heuristic;
            //    ref var activity = ref bodies.ActiveSet.Activity[i];
            //    if (activity.DeactivationCandidate)
            //    {
            //        ++deactivationCandidateCount;
            //        totalEnergyOfCandidates += heuristic;
            //    }
            //    else
            //    {
            //        totalEnergyOfNonCandidates += heuristic;
            //    }
            //    if (heuristic > highestEnergy)
            //    {
            //        highestEnergyIndex = i;
            //        highestEnergy = heuristic;
            //    }
            //}
            //Console.WriteLine(
            //    $"TE: {totalHeuristic}, " +
            //    $"CC: {deactivationCandidateCount}, " +
            //    $"ACE: {totalEnergyOfCandidates / deactivationCandidateCount}, " +
            //    $"ANCE: {totalEnergyOfNonCandidates / (bodies.ActiveSet.Count - deactivationCandidateCount)}" +
            //    $"HE: {highestEnergy}, HEI: {highestEnergyIndex}");

            //There are four threaded phases to deactivation:
            //1) Traversing the constraint graph to identify 'simulation islands' that satisfy the deactivation conditions.
            //2) Gathering the data backing the bodies and constraints of a simulation island and placing it into an inactive storage representation (a BodySet and ConstraintSet).
            //3) Removing bodies, some solver bookkeeping related to removed constraints, and broad phase work.
            //4) Removing the deactivated constraints from their type batches.
            //Separating it into these phases allows for a fairly direct parallelization.
            //Traversal proceeds in parallel, biting the bullet on the fact that different traversal starting points on separate threads may identify the same island sometimes.
            //Once all islands have been detected, the second phase is able to eliminate duplicates and gather the remaining unique islands in parallel.
            //Finally, while removal involves many sequential operations, there are some fully parallel parts and some of the locally sequential parts can be run in parallel with each other.

            //The goal here isn't necessarily to speed up the best case- using four dispatches basically guarantees 20us to 50us of overhead- 
            //but rather to try to keep the worst case from dropping frames and to improve deactivation responsiveness.

            //1) TRAVERSAL
            int candidateCount = (int)Math.Max(1, bodies.ActiveSet.Count * TestedFractionPerFrame);

            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), candidateCount, out traversalTargetBodyIndices);

            //Uniformly distribute targets across the active set. Each frame, the targets are pushed up by one slot.
            int spacing = bodies.ActiveSet.Count / candidateCount;

            //The schedule offset will gradually walk off into the sunset, and there's also a possibility that changes to the size of the active set (by, say, deactivation)
            //will put the offset so far out that a single subtraction by the active set count would be insufficient. So instead we just wrap it to zero.
            if (scheduleOffset > bodies.ActiveSet.Count)
            {
                scheduleOffset = 0;
            }

            var index = scheduleOffset;
            for (int i = 0; i < candidateCount; ++i)
            {
                if (index >= bodies.ActiveSet.Count)
                {
                    index -= bodies.ActiveSet.Count;
                }
                traversalTargetBodyIndices.AllocateUnsafely() = index;
                index += spacing;
            }
            ++scheduleOffset;


            if (deterministic)
            {
                //The order in which deactivations occurs affects the result of the simulation. To ensure determinism, we need to pin the deactivation order to something
                //which is deterministic. We will use the handle associated with each active body as the order provider.
                pool.SpecializeFor<int>().Take(bodies.ActiveSet.Count, out var sortedIndices);
                for (int i = 0; i < bodies.ActiveSet.Count; ++i)
                {
                    sortedIndices[i] = i;
                }
                //Handles are guaranteed to be unique; no need for three way partitioning.
                HandleComparer comparer;
                comparer.Handles = bodies.ActiveSet.IndexToHandle;
                //TODO: This sort might end up being fairly expensive. On a very large simulation, it might even amount to 5% of the simulation time.
                //It would be nice to come up with a better solution here. Some options include other sources of determinism, hiding the sort, and possibly enumerating directly over handles.
                QuickSort.Sort(ref sortedIndices[0], 0, bodies.ActiveSet.Count - 1, ref comparer);

                //Now that we have a sorted set of indices, we have eliminated nondeterminism related to memory layout. The initial target body indices can be remapped onto the sorted list.
                for (int i = 0; i < traversalTargetBodyIndices.Count; ++i)
                {
                    traversalTargetBodyIndices[i] = sortedIndices[traversalTargetBodyIndices[i]];
                    Debug.Assert(traversalTargetBodyIndices[i] >= 0 && traversalTargetBodyIndices[i] < bodies.ActiveSet.Count);
                }
                pool.SpecializeFor<int>().Return(ref sortedIndices);
            }

            int threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            targetDeactivatedBodyCountPerThread = (int)Math.Max(1, bodies.ActiveSet.Count * TargetDeactivatedFraction / threadCount);
            targetTraversedBodyCountPerThread = (int)Math.Max(1, bodies.ActiveSet.Count * TargetTraversedFraction / threadCount);

            pool.SpecializeFor<WorkerTraversalResults>().Take(threadCount, out workerTraversalResults);
            //Note that all resources within a worker's results set are allocate on the worker's pool since the thread may need to resize things.
            this.threadDispatcher = threadDispatcher;
            jobIndex = -1;
            if (threadCount > 1)
            {
                threadDispatcher.DispatchWorkers(findIslandsDelegate);
            }
            else
            {
                FindIslands(0, pool);
            }
            this.threadDispatcher = null;

            traversalTargetBodyIndices.Dispose(pool.SpecializeFor<int>());

            //In the event that no islands are available for deactivation, early out to avoid the later dispatches.
            int totalIslandCount = 0;
            for (int i = 0; i < threadCount; ++i)
            {
                totalIslandCount += workerTraversalResults[i].Islands.Count;
            }
            void DisposeWorkerTraversalResults()
            {
                if (threadCount > 1)
                {
                    //The source of traversal worker resources is a per-thread pool.
                    for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
                    {
                        workerTraversalResults[workerIndex].Dispose(threadDispatcher.GetThreadMemoryPool(workerIndex));
                    }
                }
                else
                {
                    //The source of traversal worker resources was the main pool since it's running in single threaded mode.
                    workerTraversalResults[0].Dispose(pool);
                }
                pool.SpecializeFor<WorkerTraversalResults>().Return(ref workerTraversalResults);
            }
            if (totalIslandCount == 0)
            {
                DisposeWorkerTraversalResults();
                return;
            }

            //2) GATHERING
            //Traversal is now done. We should have a set of results for each worker in the workerTraversalResults. It's time to gather all the data for the deactivating bodies and constraints.
            //Note that we only preallocate a fixed size. It will often be an overestimate, but that's fine. Resizes are more concerning.
            //(We could precompute the exact number of jobs, but it's not really necessary.) 
            var objectsPerGatherJob = 64;
            var gatheringJobPool = pool.SpecializeFor<GatheringJob>();
            QuickList<GatheringJob, Buffer<GatheringJob>>.Create(gatheringJobPool, 512, out gatheringJobs);
            //We now create jobs only for the set of unique islands. While each worker avoided creating duplicates locally, threads did not communicate and so may have found the same islands.
            //Each worker output a set of their traversed bodies. Using it, we can efficiently check to see if a given island is a duplicate by checking all previous workers.
            //If a previous worker traversed a body, the later worker's island holding that body is considered a duplicate.
            //(Note that a body can only belong to one island. If two threads find the same body, it means they found the exact same island, just using different paths. They are fully redundant.)
            var inactiveSetReferencePool = pool.SpecializeFor<InactiveSetReference>();
            var broadPhaseDataPool = pool.SpecializeFor<BroadPhaseData>();
            QuickList<InactiveSetReference, Buffer<InactiveSetReference>>.Create(inactiveSetReferencePool, 32, out newInactiveSets);
            var deactivatedBodyCount = 0;
            for (int workerIndex = 0; workerIndex < threadCount; ++workerIndex)
            {
                ref var workerIslands = ref workerTraversalResults[workerIndex].Islands;
                for (int j = 0; j < workerIslands.Count; ++j)
                {
                    ref var island = ref workerIslands[j];
                    island.Validate(solver);
                    bool skip = false;
                    for (int previousWorkerIndex = 0; previousWorkerIndex < workerIndex; ++previousWorkerIndex)
                    {
                        Debug.Assert(island.BodyIndices.Count > 0, "Any reported island should have a positive number of bodies in it. Otherwise, there's nothing to deactivate!");
                        if (workerTraversalResults[previousWorkerIndex].TraversedBodies.Contains(island.BodyIndices[0]))
                        {
                            //A previous worker already reported this island. It is a duplicate; skip it.
                            skip = true;
                            break;
                        }
                    }
                    if (!skip)
                    {
                        //Allocate space for a new island.
                        var setIndex = setIdPool.Take();
                        newInactiveSets.EnsureCapacity(newInactiveSets.Count + 1, inactiveSetReferencePool);
                        var referenceIndex = newInactiveSets.Count;
                        ref var newSetReference = ref newInactiveSets.AllocateUnsafely();
                        newSetReference.Index = setIndex;
                        //We allocate the broad phase data buffer here, but the data is gathered on the worker thread.
                        broadPhaseDataPool.Take(island.BodyIndices.Count, out newSetReference.BroadPhaseData);
                        EnsureSetsCapacity(setIndex + 1);
                        bodies.Sets[setIndex] = new BodySet(island.BodyIndices.Count, pool);
                        bodies.Sets[setIndex].Count = island.BodyIndices.Count;
                        deactivatedBodyCount += island.BodyIndices.Count;
                        ref var constraintSet = ref solver.Sets[setIndex];
                        constraintSet = new ConstraintSet(pool, island.Protobatches.Count);
                        for (int batchIndex = 0; batchIndex < island.Protobatches.Count; ++batchIndex)
                        {
                            ref var sourceBatch = ref island.Protobatches[batchIndex];
                            ref var targetBatch = ref constraintSet.Batches.AllocateUnsafely();
                            targetBatch = new ConstraintBatch(pool, sourceBatch.TypeIdToIndex.Length);
                            for (int typeBatchIndex = 0; typeBatchIndex < sourceBatch.TypeBatches.Count; ++typeBatchIndex)
                            {
                                ref var sourceTypeBatch = ref sourceBatch.TypeBatches[typeBatchIndex];
                                ref var targetTypeBatch = ref targetBatch.CreateNewTypeBatch(sourceTypeBatch.TypeId, solver.TypeProcessors[sourceTypeBatch.TypeId], sourceTypeBatch.Handles.Count, pool);
                                targetTypeBatch.ConstraintCount = sourceTypeBatch.Handles.Count;
                            }
                        }

                        //A single island may involve multiple jobs, depending on its size.
                        //For simplicity, a job can only cover contiguous regions. In other words, if there are two type batches, there will be at least two jobs- even if the type batches
                        //only have one constraint each. 
                        //A job also only covers either bodies or constraints, not both at once.
                        //TODO: This job scheduling pattern appears frequently. Would be nice to unify it. Obvious zero overhead approach with generics abuse.
                        {
                            var jobCount = Math.Max(1, island.BodyIndices.Count / objectsPerGatherJob);
                            var bodiesPerJob = island.BodyIndices.Count / jobCount;
                            var remainder = island.BodyIndices.Count - bodiesPerJob * jobCount;
                            var previousEnd = 0;
                            gatheringJobs.EnsureCapacity(gatheringJobs.Count + jobCount, gatheringJobPool);
                            for (int i = 0; i < jobCount; ++i)
                            {
                                var bodiesInJob = i < remainder ? bodiesPerJob + 1 : bodiesPerJob;
                                ref var job = ref gatheringJobs.AllocateUnsafely();
                                job.IsBodyJob = true;
                                job.SourceIndices = island.BodyIndices;
                                job.StartIndex = previousEnd;
                                previousEnd += bodiesInJob;
                                job.EndIndex = previousEnd;
                                job.TargetSetIndex = setIndex;
                                job.TargetSetIndexInReferenceList = referenceIndex;
                            }
                        }

                        for (int batchIndex = 0; batchIndex < island.Protobatches.Count; ++batchIndex)
                        {
                            ref var sourceBatch = ref island.Protobatches[batchIndex];
                            for (int typeBatchIndex = 0; typeBatchIndex < sourceBatch.TypeBatches.Count; ++typeBatchIndex)
                            {
                                ref var sourceTypeBatch = ref sourceBatch.TypeBatches[typeBatchIndex];
                                var jobCount = Math.Max(1, sourceTypeBatch.Handles.Count / objectsPerGatherJob);
                                var constraintsPerJob = sourceTypeBatch.Handles.Count / jobCount;
                                var remainder = sourceTypeBatch.Handles.Count - constraintsPerJob * jobCount;
                                var previousEnd = 0;
                                gatheringJobs.EnsureCapacity(gatheringJobs.Count + jobCount, gatheringJobPool);
                                for (int i = 0; i < jobCount; ++i)
                                {
                                    var constraintsInJob = i < remainder ? constraintsPerJob + 1 : constraintsPerJob;
                                    ref var job = ref gatheringJobs.AllocateUnsafely();
                                    job.IsBodyJob = false;
                                    job.SourceIndices = sourceTypeBatch.Handles;
                                    job.StartIndex = previousEnd;
                                    previousEnd += constraintsInJob;
                                    job.EndIndex = previousEnd;
                                    job.TargetSetIndex = setIndex;
                                    job.TargetBatchIndex = batchIndex;
                                    job.TargetTypeBatchIndex = typeBatchIndex;
                                }
                            }
                        }
                    }
                }
            }

            constraintRemover.Prepare(threadDispatcher);

            jobIndex = -1;
            if (threadCount > 1)
            {
                threadDispatcher.DispatchWorkers(gatherDelegate);
            }
            else
            {
                Gather(0);
            }
            DisposeWorkerTraversalResults();
            gatheringJobs.Dispose(pool.SpecializeFor<GatheringJob>());

            //3) BOOKKEEPING AND REMOVAL FINALIZATION
            //Stage 4 only removes constraints from type batches. Still need to update active set constraint batches, bodies, and so on.
            //Note that while we're using the same ConstraintRemover as the narrow phase, we do not need to perform per-body constraint list removals or handle returns.

            //We don't want the static tree to resize during removals. That would use the main pool and conflict with the NotifyNarrowPhasePairCache job's usage of the main pool.
            broadPhase.EnsureCapacity(broadPhase.ActiveTree.LeafCount, broadPhase.StaticTree.LeafCount + deactivatedBodyCount);

            QuickList<RemovalJob, Buffer<RemovalJob>>.Create(pool.SpecializeFor<RemovalJob>(), 4, out removalJobs);
            //The heavier locally sequential jobs are scheduled up front, leaving the smaller later tasks to fill gaps.
            removalJobs.AllocateUnsafely() = new RemovalJob { Type = RemovalJobType.NotifyNarrowPhasePairCache };
            removalJobs.AllocateUnsafely() = new RemovalJob { Type = RemovalJobType.RemoveBodiesFromActiveSet };
            removalJobs.AllocateUnsafely() = new RemovalJob { Type = RemovalJobType.AddCollidablesToStaticTree };
            removalJobs.AllocateUnsafely() = new RemovalJob { Type = RemovalJobType.RemoveFromBatchReferencedHandles };

            jobIndex = -1;
            if (threadCount > 1)
            {
                threadDispatcher.DispatchWorkers(executeRemovalWorkDelegate);
            }
            else
            {
                for (int i = 0; i < removalJobs.Count; ++i)
                {
                    ExecuteRemoval(ref removalJobs[i]);
                }
            }
            removalJobs.Dispose(pool.SpecializeFor<RemovalJob>());

            //4) CONSTRAINT REMOVAL FROM TYPE BATCHES
            //Removal from the type batches blocks body removal from the active set and the constraint handle mapping update:
            //-Body removal results in moved bodies, and any constraints associated with moved bodies must have their body indices updated to point at the new location.
            //Removing constraints from the type batch in parallel would cause a race condition.
            //-The constraint handle mapping is used by the type batch removal to locate constraints. Updating the handle mapping introduces a race condition.
            //Since the type batch removal for a large island (or multiple islands) will tend to create enough jobs to use the CPU,
            //we punt all other work (except for the constraint handle mapping update, type batch constraint removal relies on constraint handles for the moment)
            //to the third stage. It takes the form of a series of locally sequential jobs, so it won't have wonderful load balancing.
            //But perfect scaling is necessarily required- we just want to reduce the frame drops as much as possible.
            typeBatchConstraintRemovalJobCount = constraintRemover.CreateFlushJobs();
            jobIndex = -1;
            if (threadCount > 1)
            {
                threadDispatcher.DispatchWorkers(typeBatchConstraintRemovalDelegate);
            }
            else
            {
                for (int i = 0; i < typeBatchConstraintRemovalJobCount; ++i)
                {
                    constraintRemover.RemoveConstraintsFromTypeBatch(i);
                }
            }

            //This phase existing after the type batch constraint removal is a byproduct of how the constraint remover works-
            //since removing a constraint in a type batch can cause other constraints to move, it instead caches handles and looks up the current location as it goes.
            //It could be changed to instead work on constraint indices which are sorted prior to removal to guarantee that all indices remain valid.
            //If that was done (and no other constraint handle lookups are required either), the handle mapping update could be pulled alongside the type batch constraint removal
            //as a multithreaded task.
            //May be possible to force the update right after the enqueue in the gather if you did this. (As always, double check, this is tricky business.)
            //For now, we just do it sequentially because it's simple and relatively cheap (and I'd like to get this done sooner than later).

            //This could also be internally multithreaded. The only reason why it's not implemented that way is because it should be extremely cheap.
            //Could swap it over later if measurement suggests otherwise.
            for (int i = 0; i < newInactiveSets.Count; ++i)
            {
                var setIndex = newInactiveSets[i].Index;
                ref var set = ref solver.Sets[setIndex];
                for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
                {
                    ref var batch = ref set.Batches[batchIndex];
                    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                    {
                        ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                        for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.ConstraintCount; ++indexInTypeBatch)
                        {
                            var handle = typeBatch.IndexToHandle[indexInTypeBatch];
                            ref var constraintLocation = ref solver.HandleToConstraint[handle];
                            constraintLocation.SetIndex = setIndex;
                            constraintLocation.BatchIndex = batchIndex;
                            constraintLocation.IndexInTypeBatch = indexInTypeBatch;
                            Debug.Assert(constraintLocation.TypeId == typeBatch.TypeId, "Deactivating a constraint shouldn't change its type!");
                        }
                    }
                }
            }

            for (int i = 0; i < newInactiveSets.Count; ++i)
            {
                broadPhaseDataPool.Return(ref newInactiveSets[i].BroadPhaseData);
            }
            newInactiveSets.Dispose(inactiveSetReferencePool);
            constraintRemover.Postflush();
        }


        /// <summary>
        /// Ensures that the Bodies, Solver, and NarrowPhase can hold at least the given number of sets (BodySets for the Bodies collection, ConstraintSets for the Solver, PairSubcaches for the NarrowPhase.PairCache).
        /// </summary>
        /// <param name="setsCapacity">Number of sets to guarantee space for.</param>
        public void EnsureSetsCapacity(int setsCapacity)
        {
            var potentiallyAllocatedCount = Math.Min(setIdPool.HighestPossiblyClaimedId + 1, Math.Min(bodies.Sets.Length, Math.Min(solver.Sets.Length, pairCache.InactiveSets.Length)));
            if (setsCapacity > bodies.Sets.Length)
            {
                bodies.ResizeSetsCapacity(setsCapacity, potentiallyAllocatedCount);
            }
            if (setsCapacity > solver.Sets.Length)
            {
                solver.ResizeSetsCapacity(setsCapacity, potentiallyAllocatedCount);
            }
            if (setsCapacity > pairCache.InactiveSets.Length)
            {
                pairCache.ResizeSetsCapacity(setsCapacity, potentiallyAllocatedCount);
            }
        }

        /// <summary>
        /// Ensures that the Bodies and Solver can hold the given number of sets. 
        /// If the existing allocation is smaller than the requested sets capacity, the allocation will be enlarged.
        /// If the existing allocation is larger than both the existing potentially allocated set range and the requested sets capacity, the allocation will be shrunk.
        /// Shrinks will never cause an existing set to be lost.
        /// </summary>
        /// <param name="setsCapacity">Target number of sets to allocate space for.</param>
        public void ResizeSetsCapacity(int setsCapacity)
        {
            var potentiallyAllocatedCount = Math.Min(setIdPool.HighestPossiblyClaimedId + 1, Math.Min(bodies.Sets.Length, Math.Min(solver.Sets.Length, pairCache.InactiveSets.Length)));
            setsCapacity = Math.Max(potentiallyAllocatedCount, setsCapacity);
            bodies.ResizeSetsCapacity(setsCapacity, potentiallyAllocatedCount);
            solver.ResizeSetsCapacity(setsCapacity, potentiallyAllocatedCount);
            pairCache.ResizeSetsCapacity(setsCapacity, potentiallyAllocatedCount);
        }

        public void Clear()
        {
            setIdPool.Clear();
            //Slot 0 is reserved for the active set.
            setIdPool.Take();
        }

        public void Dispose()
        {
            setIdPool.Dispose(pool.SpecializeFor<int>());
        }



    }
}
