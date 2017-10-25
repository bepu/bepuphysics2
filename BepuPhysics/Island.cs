using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Stores the data associated with a deactivated set of bodies and constraints.
    /// </summary>
    public struct Island
    {
        public QuickList<ConstraintBatchData, Buffer<ConstraintBatchData>> ConstraintBatches;
        public int BodyCount;
        public Buffer<QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>> BodyConstraintReferences;
        public Buffer<int> Handles;
        public Buffer<RigidPose> Poses;
        public Buffer<BodyVelocity> Velocities;
        public Buffer<BodyInertia> LocalInertias;

        //Removals from islands aren't ideal- they scramble the order of objects. But they should be extremely rare for the most part.
        public void RemoveBodyAt(int bodyIndex, Bodies bodies, BroadPhase broadPhase)
        {
            Debug.Assert(bodyIndex >= 0 && bodyIndex < BodyCount);
            //Note that we require the user to remove all constraints associated with a body before removing it.
            Debug.Assert(BodyConstraintReferences[bodyIndex].Count == 0);

            //A little bit annoying- we duplicate a little bit of the bookkeeping associated with body removal. TODO: see if you can nicely unify this.
            var handle = Handles[bodyIndex];
            ref var bodiesMap = ref bodies.HandleToIndex[handle];
            bodiesMap.Island = -1;
            bodiesMap.Index = -1;
            bodies.HandlePool.Return(handle, bodies.pool.SpecializeFor<int>());

            var lastIndex = BodyCount - 1;
            if (bodyIndex < lastIndex)
            {
                BodyConstraintReferences[bodyIndex] = BodyConstraintReferences[lastIndex];
                Poses[bodyIndex] = Poses[lastIndex];
                Velocities[bodyIndex] = Velocities[lastIndex];
                LocalInertias[bodyIndex] = LocalInertias[lastIndex];
                Handles[bodyIndex] = Handles[lastIndex];
            }
            BodyCount = lastIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveConstraintAt(int batchIndex, int typeBatchIndex, int indexInTypeBatch, Solver solver)
        {
            //Constraint removals require type knowledge. We don't have any within the island, and we'd really rather not duplicate the logic required to extract it.
            //Just make use of the solver's virtual dispatch support.
            //Note that the virtual dispatch really doesn't matter- this should not be something that occurs frequently.
            solver.RemoveFromTypeBatch(ref ConstraintBatches[batchIndex].TypeBatches[typeBatchIndex], indexInTypeBatch);
        }

        public void Create(ref QuickList<int, Buffer<int>> bodyHandles, ref QuickList<int, Buffer<int>> constraintHandles,
            
            BufferPool mainPool, BufferPool threadPool)
        {
            //Note that, while we did encounter the indices associated with the island bodies handles and *could* have cached them, we opted to store the handles instead.
            //This does incur additional (warm) indirections, but we would like to also use the handles again- to remove from the active set.
            //Creating this island does not modify anything about the existing active set. All of that is deferred.

            //Note that, while we have already traversed the constraint's connected bodies to collect the island, we did not cache all required data during the traversal.
            //Doing so would be *usually* wasteful- the vast majority of traversals result in no deactivation.
            //Further, the traversal does not otherwise need to touch the prestep data and accumulated impulses. Those are quite large, so avoiding needless accesses
            //are important for keeping the traversal reasonably speedy.
            //Given that we have to grab that additional information anyway, and given that it is likely in L1 (or failing that, L2) cache, we re-enumerate the constraint body 
            //handles here.

            //We have a bit of an annoyance to deal with:
            //1) By convention, we never hold information in per-thread buffer pools between frames.
            //2) We'd like to be able to run island creation on multiple threads.
            //3) Island creation requires allocating space for all the body and constraint data.
            //Implication:

            var batchReferencedHandlesPool = threadPool.SpecializeFor<BatchReferencedHandles>();
            var intPool = threadPool.SpecializeFor<int>();
            constraintHandles.
            batchReferencedHandlesPool.Take(16, out var batchReferencedHandles);

            for (int i = 0; i < ConstraintBatches.Count; ++i)
            {
                batchReferencedHandles[i].Dispose(threadPool);
            }
            batchReferencedHandlesPool.Return(ref batchReferencedHandles);

        }

    }
}
