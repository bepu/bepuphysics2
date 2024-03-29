﻿using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Contains the pending pair cache changes created by a single worker during the last execution of narrow phase pair processing.
    /// </summary>
    public struct WorkerPendingPairChanges
    {
        public struct PendingAdd
        {
            public CollidablePair Pair;
            public ConstraintCache Cache;
        }

        /// <summary>
        /// The set of pair-pointer associations created by this worker that should be added to the pair mapping.
        /// </summary>
        public QuickList<PendingAdd> PendingAdds;
        /// <summary>
        /// The set of pairs to remove from the pair cache generated by the worker.
        /// </summary>
        public QuickList<CollidablePair> PendingRemoves;

        public WorkerPendingPairChanges(BufferPool pool, int pendingCapacity)
        {
            PendingAdds = new QuickList<PendingAdd>(pendingCapacity, pool);
            PendingRemoves = new QuickList<CollidablePair>(pendingCapacity, pool);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Add(BufferPool pool, CollidablePair pair, in ConstraintCache cache)
        {
            int index = PendingAdds.Count;
            ref var pendingAdd = ref PendingAdds.Allocate(pool);
            pendingAdd.Pair = pair;
            pendingAdd.Cache = cache;
            return index;
        }


        public void Dispose(BufferPool pool)
        {
            PendingAdds.Dispose(pool);
            PendingRemoves.Dispose(pool);
        }
    }
}
