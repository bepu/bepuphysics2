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
        
        public BodyLayoutOptimizer(Bodies bodies, BroadPhase broadPhase, Solver solver, BufferPool pool, float optimizationFraction = 0.005f)
        {
            this.bodies = bodies;
            this.broadPhase = broadPhase;
            this.solver = solver;
            OptimizationFraction = optimizationFraction;
            
        }



        public static void SwapBodyLocation(Bodies bodies, Solver solver, int a, int b)
        {
            Debug.Assert(a != b, "Swapping a body with itself isn't meaningful. Whaddeyer doin?");
            //Enumerate the bodies' current set of constraints, changing the reference in each to the new location.
            //Note that references to both bodies must be changed- both bodies moved!
            //This function does not update the actual position of the list in the graph, so we can modify both without worrying about invalidating indices.
            solver.UpdateForBodyMemorySwap(a, b);

            //Update the body locations.
            bodies.ActiveSet.Swap(a, b, ref bodies.HandleToLocation);
            //TODO: If the body layout optimizer occurs before or after all other stages, this swap isn't required. If we move it in between other stages though, we need to keep the inertia 
            //coherent with the other body properties.
            //Helpers.Swap(ref bodies.Inertias[a], ref bodies.Inertias[b]);
        }

        int nextBodyIndex = 0;

        struct IncrementalEnumerator : IForEach<int>
        {
            public Bodies bodies;
            public BroadPhase broadPhase;
            public Solver solver;
            public int Index;
            public int TargetIndexStart;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                ++Index;
                if (Index > 32)
                    return;
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                var newLocationIndex = TargetIndexStart + Index;
                if (connectedBodyIndex > newLocationIndex)
                {
                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.

                    //TODO: this implementation can almost certainly be improved-
                    //this version goes through all the effort of diving into the type batches for references, then does it all again to move stuff around.
                    //A hardcoded swapping operation could do both at once, saving a few indirections.
                    //It won't be THAT much faster- every single indirection is already cached. 
                    //Also, before you do that sort of thing, remember how short this stage is.
                    //Note that graph.EnumerateConnectedBodies explicitly excludes the body whose constraints we are enumerating, 
                    //so we don't have to worry about having the rug pulled by this list swap.
                    //(Also, !(x > x) for many values of x.)
                    SwapBodyLocation(bodies, solver, connectedBodyIndex, newLocationIndex);
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
                enumerator.TargetIndexStart = nextBodyIndex + 1;
                enumerator.Index = 0;
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
            public Solver Solver;
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

    }
}
