using System.Runtime.CompilerServices;
using static BepuPhysics.ConstraintGraph;
using System;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Runtime.InteropServices;
using System.Threading;
using BepuPhysics.CollisionDetection;

namespace BepuPhysics
{
    public partial class BodyLayoutOptimizer
    {
        //TODO: This approach is likely going to go away. It actually does slightly worse than the super naive version, and it's more complex.

        struct PartialIslandDFSEnumerator : IForEach<int>
        {
            public Bodies bodies;
            public BroadPhase broadPhase;
            public ConstraintGraph graph;
            public Solver solver;
            //We effectively do a full traversal over multiple frames. We have to store the stack for this to work.
            public QuickList<int, Buffer<int>> traversalStack;
            public BufferPool<int> pool;
            //The target index is just the last recorded exclusive endpoint of the island. In other words, it's the place where the next island body will be put.
            public int targetIndex;
            public int currentBodyIndex;
            public int maximumBodiesToVisit;
            public int visitedBodyCount;

            public void LoopBody(int connectedBodyIndex)
            {
                //Should this node be swapped into position? 
                if (connectedBodyIndex > targetIndex)
                {
                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.
                    var newLocation = targetIndex++;
                    Debug.Assert(newLocation > currentBodyIndex, "The target index should always progress ahead of the traversal. Did something get reset incorrectly?");
                    SwapBodyLocation(bodies, graph, solver, connectedBodyIndex, newLocation);
                    //Note that we mark the new location for traversal, since it was moved.
                    traversalStack.Add(newLocation, pool);
                }
                else if (connectedBodyIndex > currentBodyIndex)
                {
                    //While this body should not be swapped because it has already been swapped by an earlier traversal visit, 
                    //it is still a candidate for continued traversal. It might have children that cannot be reached by other paths.
                    traversalStack.Add(connectedBodyIndex, pool);
                }
            }
        }
        PartialIslandDFSEnumerator islandEnumerator;

        public void PartialIslandOptimizeDFS(BufferPool pool, int maximumBodiesToVisit = 32)
        {
            if (!islandEnumerator.traversalStack.Span.Allocated)
            {
                islandEnumerator = new PartialIslandDFSEnumerator
                {
                    bodies = bodies,
                    broadPhase = broadPhase,
                    graph = graph,
                    solver = solver,
                    pool = pool.SpecializeFor<int>()
                };
                QuickList<int, Buffer<int>>.Create(islandEnumerator.pool, maximumBodiesToVisit * 2, out islandEnumerator.traversalStack);
            }

            //With the observation that the full island DFS traversal is a decent cache optimization heuristic, attempt to do the same thing except spread over multiple frames.
            //This can clearly get invalidated by changes to the topology between frames, but temporary suboptimality will not cause correctness problems.

            //A few important implementation notes:
            //1) The target index advances with every swap performed.
            //2) "Visited" bodies are not explicitly tracked. Instead, the traversal will refuse to visit any bodies to the left of the current body.
            //Further, no swaps will occur with any body to the left of the target index.
            //Any body before the target index has already been swapped into position by an earlier traversal (heuristically speaking).
            //3) Swaps are performed inline, eliminating the need for any temporary body storage (or long-term locks in the multithreaded implementation).
            islandEnumerator.visitedBodyCount = 0;
            islandEnumerator.maximumBodiesToVisit = maximumBodiesToVisit;
            //First, attempt to continue any previous traversals.
            do
            {
                if (islandEnumerator.targetIndex >= bodies.Count)
                {
                    //The target index has walked outside the bounds of the body set. While we could wrap around and continue, that would a different heuristic
                    //for swapping and visitation- currently we use 'to the right' which isn't well defined on a ring.
                    //Instead, for simplicity's sake, the target simply resets to the first index, and the traversal stack gets cleared.
                    islandEnumerator.targetIndex = 0;
                    islandEnumerator.traversalStack.Count = 0;
                }
                if (islandEnumerator.traversalStack.Count == 0)
                {
                    //There is no active traversal. Start one.
                    islandEnumerator.traversalStack.Add(islandEnumerator.targetIndex++, islandEnumerator.pool);
                }
                islandEnumerator.traversalStack.Pop(out islandEnumerator.currentBodyIndex);
                //It's possible for the target index to fall behind if no swaps are needed. 
                islandEnumerator.targetIndex = Math.Max(islandEnumerator.targetIndex, islandEnumerator.currentBodyIndex + 1);
                graph.EnumerateConnectedBodies(islandEnumerator.currentBodyIndex, ref islandEnumerator);
                ++islandEnumerator.visitedBodyCount;
            } while (islandEnumerator.visitedBodyCount < islandEnumerator.maximumBodiesToVisit);
        }

        
    }
}
