using System.Runtime.CompilerServices;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;

namespace BepuPhysics
{
    public partial class BodyLayoutOptimizer
    {
        //TODO: This approach is likely going to go away. It's a testing mechanism for now to confirm that we haven't introduced any dependencies on the body list order.

        int sortingBodyIndex = 0;

        struct CollectingEnumerator : IForEach<int>
        {
            public int minimumIndex;
            public QuickList<int, Buffer<int>> bodyIndices;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connectedBodyIndex > minimumIndex)
                {
                    bodyIndices.AddUnsafely(connectedBodyIndex);
                }
            }
        }
        public void SortingIncrementalOptimize(BufferPool rawPool)
        {
            //All this does is look for any bodies which are to the right of a given body. If it finds one, it pulls it to be adjacent.
            //This converges at the island level- that is, running this on a static topology of simulation islands will eventually result in 
            //the islands being contiguous in memory, and at least some connected bodies being adjacent to each other.
            //However, within the islands, it may continue to unnecessarily swap objects around as bodies 'fight' for ownership.
            //One body doesn't know that another body has already claimed a body as a child, so this can't produce a coherent unique traversal order.
            //(In fact, it won't generally converge even with a single one dimensional chain of bodies.)

            //This optimization routine requires much less overhead than other options, like full island traversals. We only request the connections of a single body,
            //and the swap count is limited to the number of connected bodies.

            //Note that this first implementation really does not care about performance. Just looking for the performance impact on the solver at this point.

            if (sortingBodyIndex >= bodies.Count - 1)
                sortingBodyIndex = 0;

            var enumerator = new CollectingEnumerator();
            enumerator.minimumIndex = sortingBodyIndex;
            var pool = rawPool.SpecializeFor<int>();
            QuickList<int, Buffer<int>>.Create(pool, graph.GetConstraintList(sortingBodyIndex).Count, out enumerator.bodyIndices);

            graph.EnumerateConnectedBodies(sortingBodyIndex, ref enumerator);
            if (enumerator.bodyIndices.Count > 0)
            {
                pool.Take(enumerator.bodyIndices.Count, out var sourceIndices);
                var comparer = new PrimitiveComparer<int>();
                InsertionSort.Sort(ref enumerator.bodyIndices[0], ref sourceIndices[0], 0, enumerator.bodyIndices.Count - 1, ref comparer);
                int targetIndex = sortingBodyIndex;
                for (int i = 0; i < enumerator.bodyIndices.Count; ++i)
                {
                    ++targetIndex;
                    //Because the list is sorted, it's not possible for the swap target location to be greater than the swap source location.
                    Debug.Assert(targetIndex <= enumerator.bodyIndices[i]);
                    SwapBodyLocation(bodies, graph, solver, enumerator.bodyIndices[i], ++targetIndex);
                }
                pool.Return(ref sourceIndices);
            }
            enumerator.bodyIndices.Dispose(pool);
            ++sortingBodyIndex;
        }

    }
}
