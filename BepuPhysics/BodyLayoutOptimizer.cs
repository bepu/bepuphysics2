using System.Runtime.CompilerServices;
using System;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Runtime.InteropServices;
using System.Threading;
using BepuUtilities;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;

namespace BepuPhysics
{
    /// <summary>
    /// Incrementally changes the layout of a set of bodies to minimize the cache misses associated with the solver and other systems that rely on connection following.
    /// </summary>
    public partial class BodyLayoutOptimizer
    {
        Bodies bodies;
        BroadPhase broadPhase;
        Solver solver;

        float optimizationFraction;
        /// <summary>
        /// Gets or sets the fraction of all bodies to update each frame.
        /// </summary>
        public float OptimizationFraction
        {
            get
            {
                return optimizationFraction;
            }
            set
            {
                if (value > 1 || value < 0)
                    throw new ArgumentException("Optimization fraction must be a value from 0 to 1.");
                optimizationFraction = value;
            }
        }

        Action<int> incrementalOptimizeWorkDelegate;
        public BodyLayoutOptimizer(Bodies bodies, BroadPhase broadPhase, Solver solver, BufferPool pool, float optimizationFraction = 0.005f)
        {
            this.bodies = bodies;
            this.broadPhase = broadPhase;
            this.solver = solver;
            OptimizationFraction = optimizationFraction;

            incrementalOptimizeWorkDelegate = IncrementalOptimizeWork;
        }

        

        public static void SwapBodyLocation(Bodies bodies, int a, int b)
        {
            Debug.Assert(a != b, "Swapping a body with itself isn't meaningful. Whaddeyer doin?");
            //Enumerate the bodies' current set of constraints, changing the reference in each to the new location.
            //Note that references to both bodies must be changed- both bodies moved!
            //This function does not update the actual position of the list in the graph, so we can modify both without worrying about invalidating indices.
            bodies.UpdateAttachedConstraintsForBodyMemoryMove(a, b);
            bodies.UpdateAttachedConstraintsForBodyMemoryMove(b, a);

            //Update the body locations.
            bodies.ActiveSet.Swap(a, b, ref bodies.HandleToLocation);
        }

        int nextBodyIndex = 0;

        struct IncrementalEnumerator : IForEach<int>
        {
            public Bodies bodies;
            public BroadPhase broadPhase;
            public Solver solver;
            public int slotIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connectedBodyIndex > slotIndex)
                {
                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.

                    //TODO: this implementation can almost certainly be improved-
                    //this version goes through all the effort of diving into the type batches for references, then does it all again to move stuff around.
                    //A hardcoded swapping operation could do both at once, saving a few indirections.
                    //It won't be THAT much faster- every single indirection is already cached. 
                    //Also, before you do that sort of thing, remember how short this stage is.
                    var newLocation = slotIndex++;
                    //Note that graph.EnumerateConnectedBodies explicitly excludes the body whose constraints we are enumerating, 
                    //so we don't have to worry about having the rug pulled by this list swap.
                    //(Also, !(x > x) for many values of x.)
                    SwapBodyLocation(bodies, connectedBodyIndex, newLocation);
                }
            }
        }
        public void IncrementalOptimize()
        {
            //All this does is look for any bodies which are to the right of a given body. If it finds one, it pulls it to be adjacent.
            //This converges at the island level- that is, running this on a static topology of simulation islands will eventually result in 
            //the islands being contiguous in memory, and at least some connected bodies being adjacent to each other.
            //However, within the islands, it may continue to unnecessarily swap objects around as bodies 'fight' for ownership.
            //One body doesn't know that another body has already claimed a body as a child, so this can't produce a coherent unique traversal order.
            //(In fact, it won't generally converge even with a single one dimensional chain of bodies.)

            //This optimization routine requires much less overhead than other options, like full island traversals. We only request the connections of a single body,
            //and the swap count is limited to the number of connected bodies.

            //Don't bother optimizing if no optimizations can be performed. This condition is assumed during worker execution.
            if (bodies.ActiveSet.Count <= 2)
                return;
            int optimizationCount = (int)Math.Max(1, Math.Round(bodies.ActiveSet.Count * optimizationFraction));
            for (int i = 0; i < optimizationCount; ++i)
            {
                //No point trying to optimize the last two bodies. No optimizations are possible.
                if (nextBodyIndex >= bodies.ActiveSet.Count - 2)
                    nextBodyIndex = 0;

                var enumerator = new IncrementalEnumerator();
                enumerator.bodies = bodies;
                enumerator.broadPhase = broadPhase;
                enumerator.solver = solver;
                enumerator.slotIndex = nextBodyIndex + 1;
                bodies.EnumerateConnectedBodyIndices(nextBodyIndex, ref enumerator);

                ++nextBodyIndex;
            }

        }



        //TODO: Note that there are a few ways we can do multithreading. The naive way is to just apply locks on all the nodes affected by an optimization candidate.
        //This is roughly what we do below and it does work reasonably well, but a major fraction of the total execution time will be tied up in intercore communication.
        //It does not scale particularly well; something around 2.5x on an 8 core ryzen and a 4 core 3770K.
        //For now, we aren't too concerned- even if we could speed it up by a factor of infinity, we're talking about a stage that takes <100us for a simulation with 30k+ bodies.

        //Some future possibilities:
        //1) At the cost of convergence speed, you can instead choose to optimize region by region. 
        //An optimizing thread can be given a subset of all bodies with guaranteed exclusive access by scheduling.
        //While that thread can't necessarily swap bodies to where the locking version would, it can make progress toward the goal over time.
        //Rather than 'pulling', it would 'push'- find a parent to the left, and go as far to the left towards it as possible.
        //You'd have to be a bit tricky to ensure that all bodies will move towards it (rather than just one that continually gets swapped around), and the end behavior
        //could be a little different, but it might end up being faster overall due to the lack of contention.
        //The same concept could apply to the broad phase optimizer too, though it's a little easier there (and the naive locking requirements are more complex per swap, too).

        //2) Another parallel-across regions approach: rephrase the optimization as a sort. Compute the target location of every body in a region in a multithreaded prepass.
        //Then, sort the bodies in the region by the target location. There's a high probability that some bodies targets will be invalidated during the movement,
        //but so long as it eventually converges, that's totally fine.

        //Note that external systems which have to respond to body movements will generally be okay without their own synchronization in multithreaded cache optimization.
        //For example, the constraint handle, type batch index, index in type batch, and body index in constraint do not change. If we properly synchronize
        //the body accesses, then the changes to constraint bodies will be synchronized too.

        //Avoid a little pointless false sharing with padding.
        [StructLayout(LayoutKind.Explicit, Size = 128)]
        struct Worker
        {
            [FieldOffset(0)]
            public int Index;
            [FieldOffset(4)]
            public int HighestNeededClaimCount;
            [FieldOffset(8)]
            public int CompletedJobs;
            [FieldOffset(16)]
            public QuickList<int, Buffer<int>> WorkerClaims;
        }
        int remainingOptimizationAttemptCount;
        Buffer<Worker> workers;

        struct ClaimConnectedBodiesEnumerator : IForEach<int>
        {
            public Bodies Bodies;
            /// <summary>
            /// The claim states for every body in the simulation.
            /// </summary>
            public Buffer<int> ClaimStates;
            /// <summary>
            /// The set of claims owned by the current worker.
            /// </summary>
            public QuickList<int, Buffer<int>> WorkerClaims;
            public int ClaimIdentity;
            public bool AllClaimsSucceededSoFar;

            public bool TryClaim(int index)
            {
                var preclaimValue = Interlocked.CompareExchange(ref ClaimStates[index], ClaimIdentity, 0);
                if (preclaimValue == 0)
                {
                    Debug.Assert(WorkerClaims.Count < WorkerClaims.Span.Length,
                        "The claim enumerator should never be invoked if the worker claims buffer is too small to hold all the bodies.");
                    WorkerClaims.AddUnsafely(index);
                }
                else if (preclaimValue != ClaimIdentity)
                {
                    //Note that it only fails when it's both nonzero AND not equal to the claim identity. It means it's claimed by a different worker.
                    return false;
                }
                return true;
            }
            /// <summary>
            /// Because new claims are appended, and a failed claim always results in the removal of a contiguous set of trailing indices, it acts like a stack.
            /// This pops off a number of the most recent claim indices.
            /// </summary>
            /// <param name="workerClaimsToPop">Number of claims to remove from the end of the worker claims list.</param>
            public void Unclaim(int workerClaimsToPop)
            {
                Debug.Assert(workerClaimsToPop <= WorkerClaims.Count, "The pop request should never exceed the accumulated claims.");
                for (int i = 0; i < workerClaimsToPop; ++i)
                {
                    WorkerClaims.Pop(out var poppedIndex);
                    //We don't need to do anything special here, provided a reasonably strong memory model. Just release the slot.
                    ClaimStates[poppedIndex] = 0;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //TODO: If you end up going with this approach, you should probably use a IBreakableForEach instead to avoid unnecessary traversals.
                if (AllClaimsSucceededSoFar)
                {
                    if (!TryClaim(connectedBodyIndex))
                        AllClaimsSucceededSoFar = false;
                }
            }

        }

        struct MultithreadedIncrementalEnumerator : IForEach<int>
        {
            public ClaimConnectedBodiesEnumerator ClaimEnumerator;
            public int slotIndex;
            public int HighestNeededClaimCount;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connectedBodyIndex > slotIndex)
                {
                    var newLocation = slotIndex++;
                    //We must claim both the swap source, target, AND all of the bodies connected to them.
                    //(If we didn't claim the conected bodies, the changes to the constraint body indices could break the traversal.)
                    //(We don't do a constraint claims array directly because there is no single contiguous and easily indexable set of constraints.)
                    var previousClaimCount = ClaimEnumerator.WorkerClaims.Count;
                    var neededSize = previousClaimCount + 2 +
                        ClaimEnumerator.Bodies.ActiveSet.Constraints[connectedBodyIndex].Count +
                        ClaimEnumerator.Bodies.ActiveSet.Constraints[newLocation].Count;
                    //We accumulate the number of claims needed so that later updates can expand the claim array if necessary.
                    //(This should be exceptionally rare so long as a decent initial size is chosen- unless you happen to attach 5000 constraints
                    //to a single object. Which is a really bad idea.)
                    if (neededSize > HighestNeededClaimCount)
                        HighestNeededClaimCount = neededSize;
                    if (neededSize > ClaimEnumerator.WorkerClaims.Span.Length)
                    {
                        //This body can't be claimed; we don't have enough space left.
                        return;
                    }
                    if (!ClaimEnumerator.TryClaim(connectedBodyIndex))
                        return;
                    if (!ClaimEnumerator.TryClaim(newLocation))
                    {
                        //If this claim failed but the previous succeeded, we should unclaim the remote location.
                        ClaimEnumerator.Unclaim(1);
                        return;
                    }
                    ClaimEnumerator.AllClaimsSucceededSoFar = true;
                    ClaimEnumerator.Bodies.EnumerateConnectedBodyIndices(connectedBodyIndex, ref ClaimEnumerator);
                    if (!ClaimEnumerator.AllClaimsSucceededSoFar)
                    {
                        ClaimEnumerator.Unclaim(ClaimEnumerator.WorkerClaims.Count - previousClaimCount);
                        return;
                    }
                    ClaimEnumerator.Bodies.EnumerateConnectedBodyIndices(newLocation, ref ClaimEnumerator);
                    if (!ClaimEnumerator.AllClaimsSucceededSoFar)
                    {
                        ClaimEnumerator.Unclaim(ClaimEnumerator.WorkerClaims.Count - previousClaimCount);
                        return;
                    }

                    //At this point, we have claimed both swap targets and all their satellites. Can actually do the swap.

                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.
                    SwapBodyLocation(ClaimEnumerator.Bodies, connectedBodyIndex, newLocation);

                    //Unclaim all the bodies associated with this swap pair. Don't relinquish the claim origin.
                    ClaimEnumerator.Unclaim(ClaimEnumerator.WorkerClaims.Count - previousClaimCount);

                }
            }
        }


        void IncrementalOptimizeWork(int workerIndex)
        {
            var enumerator = new MultithreadedIncrementalEnumerator();
            ref var worker = ref workers[workerIndex];
            enumerator.ClaimEnumerator = new ClaimConnectedBodiesEnumerator
            {
                Bodies = bodies,
                ClaimStates = claims,
                WorkerClaims = worker.WorkerClaims,
                ClaimIdentity = workerIndex + 1,
            };
            //Note that these are optimization *attempts*. If we fail due to contention, that's totally fine. We'll get around to it later.
            //Remember, this is strictly a performance oriented heuristic. Even if all bodies are completely scrambled, it will still produce perfectly correct results.
            worker.HighestNeededClaimCount = 0;
            while (Interlocked.Decrement(ref remainingOptimizationAttemptCount) >= 0)
            {
                enumerator.HighestNeededClaimCount = 0;
                Debug.Assert(worker.Index < bodies.ActiveSet.Count - 2, "The scheduler shouldn't produce optimizations targeting the last two slots- no swaps are possible.");
                var optimizationTarget = worker.Index++;
                //There's no reason to target the last two slots. No swaps are possible.
                if (worker.Index >= bodies.ActiveSet.Count - 2)
                    worker.Index = 0;
                enumerator.slotIndex = optimizationTarget + 1;
                //We don't want to let other threads yank the claim origin away from us while we're enumerating over its connections. That would be complicated to deal with.
                if (!enumerator.ClaimEnumerator.TryClaim(optimizationTarget))
                    continue; //Couldn't grab the origin; just move on.
                bodies.EnumerateConnectedBodyIndices(optimizationTarget, ref enumerator);

                //The optimization for this object is complete. We should relinquish all existing claims.
                Debug.Assert(enumerator.ClaimEnumerator.WorkerClaims.Count == 1 && enumerator.ClaimEnumerator.WorkerClaims[0] == optimizationTarget,
                    "The only remaining claim should be the optimization target; all others should have been relinquished within the inner enumerator.");
                enumerator.ClaimEnumerator.Unclaim(1);
                worker.HighestNeededClaimCount = enumerator.HighestNeededClaimCount;
                ++worker.CompletedJobs;
            }

        }

        //This claims set is a part of the optimizer for now, but if there is ever a time where you might want a per body claims set for some other multithreaded purpose,
        //move it into the Bodies class instead. It tends to resize with the body count, so it makes sense to bundle it if there is any sharing at all.
        //Note that claims are peristent because clearing a few kilobytes every frame isn't free. (It's not expensive, either, but it's not free.)
        Buffer<int> claims;
        //We pick an extremely generous value to begin with because there's not much reason not to. This avoids problems in most reasonable simulations.
        int workerClaimsBufferSize = 512;
        public void IncrementalOptimize(BufferPool pool, IThreadDispatcher threadPool)
        {
            //TODO: It's possible that the cost associated with setting up multithreading exceeds the cost of the actual optimization for smaller simulations.
            //You might want to fall back to single threaded based on some empirical testing.

            //Don't bother optimizing if no optimizations can be performed. This condition is assumed during worker execution.
            if (bodies.ActiveSet.Count <= 2)
                return;
            //Note that, while we COULD aggressively downsize the claims array in response to body count, we'll instead let it stick around unless the bodies allocations change.
            ResizeForBodiesCapacity(pool);

            pool.SpecializeFor<Worker>().Take(threadPool.ThreadCount, out workers);

            //Note that we ignore the last two slots as optimization targets- no swaps are possible.
            if (nextBodyIndex >= bodies.ActiveSet.Count - 2)
                nextBodyIndex = 0;
            //Each worker is assigned a start location evenly spaced from other workers. The less interference, the better.
            var spacingBetweenWorkers = bodies.ActiveSet.Count / threadPool.ThreadCount;
            var spacingRemainder = spacingBetweenWorkers - spacingBetweenWorkers * threadPool.ThreadCount;
            int optimizationCount = (int)Math.Max(1, Math.Round(bodies.ActiveSet.Count * optimizationFraction));
            var optimizationsPerWorker = optimizationCount / threadPool.ThreadCount;
            var optimizationsRemainder = optimizationCount - optimizationsPerWorker * threadPool.ThreadCount;
            int nextStartIndex = nextBodyIndex;
            for (int i = 0; i < threadPool.ThreadCount; ++i)
            {
                ref var worker = ref workers[i];
                worker.Index = nextStartIndex;
                worker.CompletedJobs = 0;
                QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), workerClaimsBufferSize, out worker.WorkerClaims);

                nextStartIndex += spacingBetweenWorkers;
                if (--spacingRemainder >= 0)
                    ++nextStartIndex;
                //Note that we just wrap to zero rather than taking into acount the amount of spill. 
                //No real downside, and ensures that the potentially longest distance pulls get done.
                //Also note that we ignore the last two slots as optimization targets- no swaps are possible.
                if (nextStartIndex >= bodies.ActiveSet.Count - 2)
                    nextStartIndex = 0;
            }
            //Every worker moves forward from its start location by decrementing the optimization count to claim an optimization job.
            remainingOptimizationAttemptCount = Math.Min(bodies.ActiveSet.Count, optimizationCount);

            threadPool.DispatchWorkers(incrementalOptimizeWorkDelegate);

            int lowestWorkerJobsCompleted = int.MaxValue;
            for (int i = 0; i < threadPool.ThreadCount; ++i)
            {
                ref var worker = ref workers[i];
                //Update the initial buffer size if it was too small this frame.
                if (worker.HighestNeededClaimCount > workerClaimsBufferSize)
                    workerClaimsBufferSize = worker.HighestNeededClaimCount;
                if (worker.CompletedJobs < lowestWorkerJobsCompleted)
                    lowestWorkerJobsCompleted = worker.CompletedJobs;

                Debug.Assert(worker.WorkerClaims.Count == 0, "After execution, all worker claims should be relinquished.");
                worker.WorkerClaims.Dispose(pool.SpecializeFor<int>());
            }
            pool.SpecializeFor<Worker>().Return(ref workers);

            //Push all workers forward by the amount the slowest one got done- or a fixed minimum to avoid stalls.
            //The goal is to ensure each worker gets to keep working on a contiguous blob as much as possible for higher cache coherence.
            //This isn't a hard guarantee- contested claims and thread scheduling can easily result in gaps in the optimization, but that's fine.
            //No harm to correctness; we'll eventually optimize it during a later frame.
            nextBodyIndex += Math.Max(lowestWorkerJobsCompleted, Math.Max(1, optimizationCount / (threadPool.ThreadCount * 4)));
        }


        /// <summary>
        /// Returns the multithreaded claims array to the provided pool.
        /// </summary>
        /// <remarks><para>Apart from its use in the optimization function itself, this only needs to be called when the simulation is being torn down 
        /// and outstanding pinned resources need to be reclaimed (with the assumption that the underlying bufferpool will be reused).</para>
        /// <para>Note that it's possible to reuse the body layout optimizer after disposal. The update function will reallocate a claims array as needed.</para></remarks>
        /// <param name="pool">Pool to return the claims array to.</param>
        public void Dispose(BufferPool pool)
        {
            if (claims.Length > 0)
            {
                pool.SpecializeFor<int>().Return(ref claims);
            }
        }

        //Note that we don't have explicit support for compaction or ensure capacity here.
        //The BodyLayoutOptimizer is effectively a slave of the Bodies set in terms of its resource allocation sizes.
        //(The only reason it's stored in here is because, at the moment, no other system requires a claims array.)

        /// <summary>
        /// Checks the referenced bodies set and resizes the claims array if it does not match the size needed to contain the bodies set.
        /// </summary>
        public void ResizeForBodiesCapacity(BufferPool pool)
        {
            var bodiesCapacity = bodies.ActiveSet.IndexToHandle.Length;
            if (claims.Length != BufferPool<int>.GetLowestContainingElementCount(bodiesCapacity))
            {
                //We need a new claims buffer. Get rid of the old one.
                pool.SpecializeFor<int>().Resize(ref claims, bodiesCapacity, 0);
                //Claims need to be zeroed so that workers can claim by identity.
                //Go ahead and clear the full buffer so we don't have to worry about later body additions resulting in accesses to uncleared memory.
                //Easier to clear upfront than track every single add.
                claims.Clear(0, claims.Length);
            }
        }

    }
}
