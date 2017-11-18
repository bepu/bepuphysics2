using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    public class Deactivator
    {
        IdPool<Buffer<int>> islandIdPool;
        Bodies bodies;
        Solver solver;
        BufferPool pool;
        public int InitialIslandBodyCapacity = 1024;
        public int InitialIslandConstraintCapacity = 1024;
        public Deactivator(Bodies bodies, Solver solver, BufferPool pool)
        {
            this.bodies = bodies;
            this.solver = solver;
            this.pool = pool;
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), 16, out var islandIdPool);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool IsDeactivationCandidate(int bodyIndex)
        {
            //TODO: This heuristic is really poor and must be improved. Most likely, we'll end up with something in the pose integrator (or its descendants) which checks the velocity state
            //over the course of multiple frames. Decent heuristics include nonincreasing energy over some number of frames, combined with a likely per-body deactivation threshold.
            //May want to just use sub-threshold for a framecount- simpler to track, and more aggressive.
            //(Note that things like 'isalwaysactive' can be expressed as a speical case of more general tuning, like a DeactivationVelocityThreshold < 0.)
            ref var bodyVelocity = ref bodies.ActiveSet.Velocities[bodyIndex];
            return bodyVelocity.Linear.LengthSquared() + bodyVelocity.Angular.LengthSquared() < 0.1f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void EnqueueUnvisitedNeighbors(int bodyHandle,
            ref QuickList<int, Buffer<int>> bodyHandles,
            ref QuickList<int, Buffer<int>> constraintHandles,
            ref HandleSet consideredBodies, ref HandleSet consideredConstraints,
            ref QuickList<int, Buffer<int>> visitationStack,
            ref ConstraintBodyEnumerator bodyEnumerator,
            ref BufferPool<int> intPool)
        {
            var bodyIndex = bodies.HandleToLocation[bodyHandle].Index;
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
                    solver.EnumerateConnectedBodyIndices(entry.ConnectingConstraintHandle, ref bodyEnumerator);
                    for (int j = 0; j < bodyEnumerator.ConstraintBodyIndices.Count; ++j)
                    {
                        var connectedBodyHandle = bodies.ActiveSet.IndexToHandle[bodyEnumerator.ConstraintBodyIndices[j]];
                        if(!consideredBodies.Contains(connectedBodyHandle))
                        {
                            //This body has not yet been traversed. Push it onto the stack.
                            bodyHandles.Add(connectedBodyHandle, intPool);
                            consideredBodies.AddUnsafely(connectedBodyHandle);
                            visitationStack.Add(connectedBodyHandle, intPool);

                        }
                    }
                }
            }
        }

        void Traverse(int workerIndex, BufferPool threadPool, int startingBodyHandle)
        {
            //We'll build the island by working depth-first. This means the bodies and constraints we accumulate will be stored in any inactive island by depth-first order,
            //which happens to be a pretty decent layout for cache purposes. In other words, when we wake these islands back up, bodies near each other in the graph will have 
            //a higher chance of being near each other in memory. Bodies directly connected may often end up adjacent to each other, meaning loading one body may give you the other for 'free'
            //(assuming they share a cache line).
            //The DFS order for constraints is not quite as helpful as the constraint optimizer's sort, but it's not terrible.

            //Despite being DFS, there is no guarantee that the visitation stack will be any smaller than the island itself, and we have no way of knowing how big the island is 
            //ahead of time- except that it can't be larger than the entire active simulation.
            var intPool = threadPool.SpecializeFor<int>();
            var initialBodyCapacity = Math.Min(InitialIslandBodyCapacity, bodies.ActiveSet.Count);
            QuickList<int, Buffer<int>>.Create(intPool, initialBodyCapacity, out var bodyHandles);
            QuickList<int, Buffer<int>>.Create(intPool, Math.Min(InitialIslandBodyCapacity, solver.HandlePool.HighestPossiblyClaimedId + 1), out var constraintHandles);
            //Note that we track all considered bodies AND constraints. 
            //While we only need to track one of them for the purposes of traversal, tracking both allows low-overhead collection of unique bodies and constraints.
            //Note that the handle sets are initialized to cover the entire handle span. That's actually fine- every single object occupies only a single bit, so 131072 objects only use 16KB.
            var consideredBodies = new HandleSet(threadPool, bodies.HandlePool.HighestPossiblyClaimedId + 1);
            var consideredConstraints = new HandleSet(threadPool, solver.HandlePool.HighestPossiblyClaimedId + 1);
            //The stack will store bodies.
            consideredBodies.AddUnsafely(startingBodyHandle);
            QuickList<int, Buffer<int>>.Create(intPool, initialBodyCapacity, out var visitationStack);



            var bodyIndex = bodies.HandleToLocation[startingBodyHandle].Index;
            ref var constraintHandleList = ref bodies.ActiveSet.Constraints[bodyIndex];

        }


        //public void Create(ref QuickList<int, Buffer<int>> bodyHandles, ref QuickList<int, Buffer<int>> constraintHandles,
        //    BufferPool mainPool, BufferPool threadPool)
        //{
        //    //Note that, while we did encounter the indices associated with the island bodies handles and *could* have cached them, we opted to store the handles instead.
        //    //This does incur additional (warm) indirections, but we would like to also use the handles again- to remove from the active set.
        //    //Creating this island does not modify anything about the existing active set. All of that is deferred.

        //    //Note that, while we have already traversed the constraint's connected bodies to collect the island, we did not cache all required data during the traversal.
        //    //Doing so would be *usually* wasteful- the vast majority of traversals result in no deactivation.
        //    //Further, the traversal does not otherwise need to touch the prestep data and accumulated impulses. Those are quite large, so avoiding needless accesses
        //    //are important for keeping the traversal reasonably speedy.
        //    //Given that we have to grab that additional information anyway, and given that it is likely in L1 (or failing that, L2) cache, we re-enumerate the constraint body 
        //    //handles here.

        //    //We have a bit of an annoyance to deal with:
        //    //1) By convention, we never hold information in per-thread buffer pools between frames.
        //    //2) We'd like to be able to run island creation on multiple threads.
        //    //3) Island creation requires allocating space for all the body and constraint data.
        //    //Implication:
        //    //We must synchronize access to the main pool when retrieving persisted buffers. All ephemeral data comes from the thread pool.
        //    //While this isn't too problematic (time spent retrieving island resources is going to be extremely brief), 
        //    //the main pool access does restrict job scheduling with other main pool users that are unaware of the synchronization requirement.

        //    //Unless we perform constraint batching during traversal, the numbers of constraint batches, type batches, and constraints within individual type batches are unknown.
        //    //We cannot just lock once and allocate a minimally sufficient set of buffers.
        //    //An option:
        //    //1) Enumerate each constraint's bodies. Convert them to handles and perform batching, locally creating constraintbatches and type batches, but only fill the body references.
        //    //2) As you go, store the new handle->island location mapping.
        //    //3) Using the capacities detected by 

        //    var batchReferencedHandlesPool = threadPool.SpecializeFor<HandleSet>();
        //    var intPool = threadPool.SpecializeFor<int>();
        //    constraintHandles.
        //    batchReferencedHandlesPool.Take(16, out var batchReferencedHandles);

        //    for (int i = 0; i < ConstraintBatches.Count; ++i)
        //    {
        //        batchReferencedHandles[i].Dispose(threadPool);
        //    }
        //    batchReferencedHandlesPool.Return(ref batchReferencedHandles);

        //}


        public void Dispose()
        {
            islandIdPool.Dispose(pool.SpecializeFor<int>());
        }
    }
}
