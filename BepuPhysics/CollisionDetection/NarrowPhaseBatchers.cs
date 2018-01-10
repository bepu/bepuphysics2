using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {

        internal struct BatcherFilters : ICollisionSubtaskFilters
        {
            NarrowPhase<TCallbacks> narrowPhase;
            int workerIndex;

            public BatcherFilters(int workerIndex, NarrowPhase<TCallbacks> narrowPhase)
            {
                this.workerIndex = workerIndex;
                this.narrowPhase = narrowPhase;
            }
            
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowCollisionTesting(CollidablePair parent, int childA, int childB)
            {
                return narrowPhase.Callbacks.AllowContactGeneration(workerIndex, parent, childA, childB);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void Configure(CollidablePair parent, int childA, int childB, ContactManifold* manifold)
            {
                narrowPhase.Callbacks.ConfigureContactManifold(workerIndex, parent, childA, childB, manifold);
            }
        }

        internal struct OverlapWorker
        {
            public StreamingBatcher Batcher;
            public BatcherFilters Filters;
            public ConstraintGenerators ConstraintGenerators;
            public PendingConstraintAddCache PendingConstraints;
            public QuickList<int, Buffer<int>> PendingSetAwakenings;

            public OverlapWorker(int workerIndex, BufferPool pool, NarrowPhase<TCallbacks> narrowPhase)
            {
                Batcher = new StreamingBatcher(pool, narrowPhase.CollisionTaskRegistry);
                Filters = new BatcherFilters(workerIndex, narrowPhase);
                ConstraintGenerators = new ConstraintGenerators(workerIndex, pool, narrowPhase);
                PendingConstraints = new PendingConstraintAddCache(pool);
                QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), 16, out PendingSetAwakenings);
            }

        }

        internal OverlapWorker[] overlapWorkers;

        private void PrepareOverlapWorkers(IThreadDispatcher threadDispatcher)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            //Resizes should be very rare, and having a single extra very small array isn't concerning.
            //(It's not an unmanaged type because it contains nonblittable references.)
            if (overlapWorkers == null || overlapWorkers.Length < threadCount)
                Array.Resize(ref overlapWorkers, threadCount);
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i] = new OverlapWorker(i, threadDispatcher != null ? threadDispatcher.GetThreadMemoryPool(i) : Pool, this);
            }
        }


        private void DisposeConstraintGenerators(int threadCount)
        {
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i].ConstraintGenerators.Dispose();
            }

        }
    }


}
