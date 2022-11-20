using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.CollisionDetection
{
    using OverlapMapping = QuickDictionary<CollidablePair, ConstraintCache, CollidablePairComparer>;

    [StructLayout(LayoutKind.Explicit, Size = 8)]
    public struct CollidablePair
    {
        [FieldOffset(0)]
        public CollidableReference A;
        [FieldOffset(4)]
        public CollidableReference B;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public CollidablePair(CollidableReference a, CollidableReference b)
        {
            A = a;
            B = b;
        }

        public override string ToString()
        {
            return $"<{A}, {B}>";
        }
    }

    public struct CollidablePairComparer : IEqualityComparerRef<CollidablePair>
    {
        //Note that pairs are sorted by handle, so we can assume order matters.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref CollidablePair a, ref CollidablePair b)
        {
            return Unsafe.As<CollidablePair, ulong>(ref a) == Unsafe.As<CollidablePair, ulong>(ref b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref CollidablePair item)
        {
            const ulong p1 = 961748927UL;
            const ulong p2 = 899809343UL;
            var hash64 = (ulong)item.A.Packed * (p1 * p2) + (ulong)item.B.Packed * (p2);
            return (int)(hash64 ^ (hash64 >> 32));
        }
    }

    /// <summary>
    /// Refers to a change in a <see cref="PairCache"/>.
    /// </summary>
    public struct PairCacheChangeIndex
    {
        /// <summary>
        /// Index of the <see cref="WorkerPendingPairChanges"/> storing the pending change, if any. If -1, then this pair cache change refers to a change directly to the mapping.
        /// </summary>
        public int WorkerIndex;
        /// <summary>
        /// Index of the change in the cache. For pending changes, refers to the index within the pending cache; for a direct mapping changes, refers to the pair index.
        /// </summary>
        public int Index;

        /// <summary>
        /// Gets whether this change is in the <see cref="PairCache"/>
        /// </summary>
        public bool IsPending => WorkerIndex >= 0;
    }

    /// <summary>
    /// Stores information about a contact constraint from the previous timestep.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ConstraintCache
    {
        /// <summary>
        /// Handle of the contact constraint associated with this cache.
        /// </summary>
        public ConstraintHandle ConstraintHandle;
        /// <summary>
        /// Feature id of the first contact in the constraint associated with this cache.
        /// </summary>
        public int FeatureId0;
        /// <summary>
        /// Feature id of the second contact in the constraint associated with this cache.
        /// </summary>
        public int FeatureId1;
        /// <summary>
        /// Feature id of the third contact in the constraint associated with this cache.
        /// </summary>
        public int FeatureId2;
        /// <summary>
        /// Feature id of the fourth contact in the constraint associated with this cache.
        /// </summary>
        public int FeatureId3;
    }

    public partial class PairCache
    {
        public OverlapMapping Mapping;

        /// <summary>
        /// Per-pair 'freshness' flags set when a pair is added or updated by the narrow phase execution. Only initialized for the duration of the narrowphase's execution.
        /// </summary>
        /// <remarks>
        /// This stores one byte per pair. While it could be compressed to 1 bit, that requires manually ensuring thread safety. By using bytes, we rely on the 
        /// atomic setting behavior for data types no larger than the native pointer size. Further, smaller sizes actually pay a higher price in terms of increased false sharing.
        /// Choice of data type is a balancing act between the memory bandwidth of the post analysis and the frequency of false sharing.
        /// </remarks>
        internal Buffer<byte> PairFreshness;
        internal BufferPool pool;
        int minimumPendingSize;
        int minimumPerTypeCapacity;
        int previousPendingSize;


        //While the current worker caches are read from, changes to the cache are accumulated.
        internal Buffer<WorkerPendingPairChanges> WorkerPendingChanges;
        internal IThreadDispatcher cachedDispatcher;


        public PairCache(BufferPool pool, int initialSetCapacity, int minimumMappingSize, int minimumPendingSize, int minimumPerTypeCapacity)
        {
            this.minimumPendingSize = minimumPendingSize;
            this.minimumPerTypeCapacity = minimumPerTypeCapacity;
            this.pool = pool;
            Mapping = new OverlapMapping(minimumMappingSize, pool);
            ResizeSetsCapacity(initialSetCapacity, 0);
        }

        public void Prepare(IThreadDispatcher threadDispatcher = null)
        {
            var threadCount = threadDispatcher != null ? threadDispatcher.ThreadCount : 1;
            cachedDispatcher = threadDispatcher;

            var pendingSize = Math.Max(minimumPendingSize, previousPendingSize);
            pool.Take(threadCount, out WorkerPendingChanges);
            if (threadDispatcher != null)
            {
                for (int i = 0; i < threadCount; ++i)
                {
                    WorkerPendingChanges[i] = new WorkerPendingPairChanges(threadDispatcher.GetThreadMemoryPool(i), pendingSize);
                }
            }
            else
            {
                WorkerPendingChanges[0] = new WorkerPendingPairChanges(pool, pendingSize);
            }

            //Create the pair freshness array for the existing overlaps.
            pool.TakeAtLeast(Mapping.Count, out PairFreshness);
            //This clears 1 byte per pair. 32768 pairs with 10GBps assumed single core bandwidth means about 3 microseconds.
            //There is a small chance that multithreading this would be useful in larger simulations- but it would be very, very close.
            PairFreshness.Clear(0, Mapping.Count);

        }


        internal void EnsureConstraintToPairMappingCapacity(Solver solver, int targetCapacity)
        {
            targetCapacity = Math.Max(solver.HandlePool.HighestPossiblyClaimedId + 1, targetCapacity);
            if (ConstraintHandleToPair.Length < targetCapacity)
            {
                pool.ResizeToAtLeast(ref ConstraintHandleToPair, targetCapacity, ConstraintHandleToPair.Length);
            }
        }

        internal void ResizeConstraintToPairMappingCapacity(Solver solver, int targetCapacity)
        {
            targetCapacity = BufferPool.GetCapacityForCount<CollisionPairLocation>(Math.Max(solver.HandlePool.HighestPossiblyClaimedId + 1, targetCapacity));
            if (ConstraintHandleToPair.Length != targetCapacity)
            {
                pool.ResizeToAtLeast(ref ConstraintHandleToPair, targetCapacity, Math.Min(targetCapacity, ConstraintHandleToPair.Length));
            }
        }



        /// <summary>
        /// Flush all deferred changes from the last narrow phase execution.
        /// </summary>
        public void PrepareFlushJobs(ref QuickList<NarrowPhaseFlushJob> jobs)
        {
            //The freshness cache should have already been used in order to generate the constraint removal requests and the PendingRemoves that we handle in a moment; dispose it now.
            pool.Return(ref PairFreshness);

            //Ensure the overlap mapping size is sufficient up front. This requires scanning all the pending sizes.
            int largestIntermediateSize = Mapping.Count;
            var newMappingSize = Mapping.Count;
            for (int i = 0; i < WorkerPendingChanges.Length; ++i)
            {
                ref var cache = ref WorkerPendingChanges[i];
                //Removes occur first, so this cache can only result in a larger mapping if there are more adds than removes.
                newMappingSize += cache.PendingAdds.Count - cache.PendingRemoves.Count;
                if (newMappingSize > largestIntermediateSize)
                    largestIntermediateSize = newMappingSize;
            }
            Mapping.EnsureCapacity(largestIntermediateSize, pool);

            jobs.Add(new NarrowPhaseFlushJob { Type = NarrowPhaseFlushJobType.FlushPairCacheChanges }, pool);
        }

        public unsafe void FlushMappingChanges()
        {
            //Flush all pending adds from the new set.
            //Note that this phase accesses no shared memory- it's all pair cache local, and no pool accesses are made.
            //That means we could run it as a job alongside solver constraint removal. That's good, because adding and removing to the hash tables isn't terribly fast.  
            //(On the order of 10-100 nanoseconds per operation, so in pathological cases, it can start showing up in profiles.)
            for (int i = 0; i < WorkerPendingChanges.Length; ++i)
            {
                ref var cache = ref WorkerPendingChanges[i];

                //Walk backwards on the off chance that a swap can be avoided.
                for (int j = cache.PendingRemoves.Count - 1; j >= 0; --j)
                {
                    var removed = Mapping.FastRemove(ref cache.PendingRemoves[j]);
                    Debug.Assert(removed);
                }
                for (int j = 0; j < cache.PendingAdds.Count; ++j)
                {
                    ref var pending = ref cache.PendingAdds[j];
                    var added = Mapping.AddUnsafely(pending.Pair, pending.Cache);
                    Debug.Assert(added);
                }
            }
        }
        public void Postflush()
        {
            //This bookkeeping and disposal phase is trivially cheap compared to the cost of updating the mapping table, so we do it sequentially.
            //The fact that we access the per-worker pools here would prevent easy multithreading anyway; the other threads may use them. 
            int largestPendingSize = 0;
            for (int i = 0; i < WorkerPendingChanges.Length; ++i)
            {
                ref var pendingChanges = ref WorkerPendingChanges[i];
                if (pendingChanges.PendingAdds.Count > largestPendingSize)
                {
                    largestPendingSize = pendingChanges.PendingAdds.Count;
                }
                if (pendingChanges.PendingRemoves.Count > largestPendingSize)
                {
                    largestPendingSize = pendingChanges.PendingRemoves.Count;
                }
            }
            if (WorkerPendingChanges.Length > 1)
            {
                for (int i = 0; i < WorkerPendingChanges.Length; ++i)
                {
                    WorkerPendingChanges[i].Dispose(cachedDispatcher.GetThreadMemoryPool(i));
                }
            }
            else
            {
                WorkerPendingChanges[0].Dispose(pool);
            }
            previousPendingSize = largestPendingSize;

            pool.Return(ref WorkerPendingChanges);
            cachedDispatcher = null;


        }

        internal void Clear()
        {
            for (int i = 1; i < SleepingSets.Length; ++i)
            {
                if (SleepingSets[i].Allocated)
                {
                    SleepingSets[i].Dispose(pool);
                }
            }

            Debug.Assert(!WorkerPendingChanges.Allocated);
        }

        public void Dispose()
        {
            //Note that we do not need to dispose the worker cache arrays themselves- they were just arrays pulled out of a passthrough pool.
            Debug.Assert(!WorkerPendingChanges.Allocated);

            Mapping.Dispose(pool);
            for (int i = 1; i < SleepingSets.Length; ++i)
            {
                ref var set = ref SleepingSets[i];
                if (set.Allocated)
                    set.Dispose(pool);
            }
            pool.Return(ref SleepingSets);
            //The constraint handle to pair is partially slaved to the constraint handle capacity. 
            //It gets ensured every frame, but the gap between construction and the first frame could leave it uninitialized.
            if (ConstraintHandleToPair.Allocated)
                pool.Return(ref ConstraintHandleToPair);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(CollidablePair pair)
        {
            return Mapping.IndexOf(pair);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintCache GetCache(int index)
        {
            return ref Mapping.Values[index];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe PairCacheChangeIndex Add(int workerIndex, CollidablePair pair, in ConstraintCache constraintCache)
        {
            //Note that we do not have to set any freshness bytes here; using this path means there exists no previous overlap to remove anyway.            
            return new PairCacheChangeIndex { WorkerIndex = workerIndex, Index = WorkerPendingChanges[workerIndex].Add(cachedDispatcher == null ? pool : cachedDispatcher.GetThreadMemoryPool(workerIndex), pair, constraintCache) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe PairCacheChangeIndex Update(int pairIndex, in ConstraintCache cache)
        {
            //We're updating an existing pair, so we should prevent this pair from being removed.
            PairFreshness[pairIndex] = 0xFF;
            Mapping.Values[pairIndex] = cache;
            return new PairCacheChangeIndex { WorkerIndex = -1, Index = pairIndex };
        }

        //4 convex one body, 4 convex two body, 7 nonconvex one body, 7 convex two body.
        public const int CollisionConstraintTypeCount = 22;
        public const int CollisionTypeCount = 16;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe ConstraintHandle GetOldConstraintHandle(int pairIndex)
        {
            return Mapping.Values[pairIndex].ConstraintHandle;
        }

        /// <summary>
        /// Completes the addition of a constraint by filling in the narrowphase's pointer to the constraint and by distributing accumulated impulses.
        /// </summary>
        /// <typeparam name="TContactImpulses">Count-specialized type containing cached accumulated impulses.</typeparam>
        /// <param name="narrowPhase">Narrow phase that triggered the constraint add.</param>
        /// <param name="solver">Solver containing the constraint to set the impulses of.</param>
        /// <param name="impulses">Warm starting impulses to apply to the contact constraint.</param>
        /// <param name="pairCacheChangeIndex">Index of the change associated with this constraint in the <see cref="PairCache"/>.</param>
        /// <param name="constraintHandle">Constraint handle associated with the constraint cache being updated.</param>
        /// <param name="pair">Collidable pair associated with the new constraint.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void CompleteConstraintAdd<TContactImpulses>(NarrowPhase narrowPhase, Solver solver, ref TContactImpulses impulses, PairCacheChangeIndex pairCacheChangeIndex,
            ConstraintHandle constraintHandle, ref CollidablePair pair)
        {
            if (pairCacheChangeIndex.IsPending)
            {
                WorkerPendingChanges[pairCacheChangeIndex.WorkerIndex].PendingAdds[pairCacheChangeIndex.Index].Cache.ConstraintHandle = constraintHandle;
            }
            else
            {
                Mapping.Values[pairCacheChangeIndex.Index].ConstraintHandle = constraintHandle;
            }
            var reference = solver.GetConstraintReference(constraintHandle);
            Debug.Assert(reference.IndexInTypeBatch >= 0 && reference.IndexInTypeBatch < reference.TypeBatch.ConstraintCount);
            narrowPhase.contactConstraintAccessors[reference.TypeBatch.TypeId].ScatterNewImpulses(ref reference, ref impulses);
            //This mapping entry had to be deferred until now because no constraint handle was known until now. Now that we have it,
            //we can fill in the pointers back to the overlap mapping.
            ConstraintHandleToPair[constraintHandle.Value].Pair = pair;
        }

    }
}
