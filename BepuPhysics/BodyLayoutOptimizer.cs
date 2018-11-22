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
    }
}
