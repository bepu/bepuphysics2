using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    internal struct SleepingPair
    {
        public CollidablePair Pair;
        public TypedIndex ConstraintCache;
        public TypedIndex CollisionCache;
    }

    internal struct SleepingCache
    {
        public int TypeId;
        public UntypedList List;
    }

    internal struct SleepingSet
    {
        public bool Allocated { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Pairs.Span.Allocated; } }

        public Buffer<SleepingCache> ConstraintCaches;
        public Buffer<SleepingCache> CollisionCaches;
        public QuickList<SleepingPair> Pairs;

        public void Dispose(BufferPool pool)
        {
            //Note that we use allocation status as an early terminator here. Didn't want to store the extra bytes for counts for no reason.
            //This does require a clear over the unfilled slots in the inactive set builder, though.
            for (int i = 0; i < ConstraintCaches.Length; ++i)
            {
                ref var cache = ref ConstraintCaches[i];
                if (cache.List.Buffer.Allocated)
                    pool.Return(ref cache.List.Buffer);
                else
                    break;
            }
            pool.Return(ref ConstraintCaches);
            //Remember, collision caches are not guaranteed to exist. If none are found during set construction, nothing is allocated for them.
            //This just saves a little bit of extra space for the inactive set.
            if (CollisionCaches.Allocated)
            {
                for (int i = 0; i < CollisionCaches.Length; ++i)
                {
                    ref var cache = ref CollisionCaches[i];
                    if (cache.List.Buffer.Allocated)
                        pool.Return(ref cache.List.Buffer);
                    else
                        break;
                }
                pool.Return(ref CollisionCaches);
            }
            Pairs.Dispose(pool);
        }
    }

    internal struct SleepingSetBuilder
    {
        public Buffer<UntypedList> ConstraintCaches;
        public Buffer<UntypedList> CollisionCaches;
        public QuickList<SleepingPair> Pairs;
        public int InitialCapacityPerCache;
        public SleepingSetBuilder(BufferPool pool, int initialPairCapacity, int initialCapacityPerCache)
        {
            pool.TakeAtLeast(PairCache.CollisionConstraintTypeCount, out ConstraintCaches);
            pool.TakeAtLeast(PairCache.CollisionTypeCount, out CollisionCaches);
            //Original values are used to test for existence; have to clear to avoid undefined values.
            ConstraintCaches.Clear(0, ConstraintCaches.Length);
            CollisionCaches.Clear(0, CollisionCaches.Length);
            Pairs = new QuickList<SleepingPair>(initialPairCapacity, pool);
            InitialCapacityPerCache = initialCapacityPerCache;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe TypedIndex CopyToBuilderCache(ref Buffer<UntypedList> sourceCaches, ref Buffer<UntypedList> targetCaches, int typeId, int sourceByteIndex, BufferPool pool)
        {
            ref var sourceCache = ref sourceCaches[typeId];
            ref var targetCache = ref targetCaches[typeId];
            var targetByteIndex = targetCache.Allocate(sourceCache.ElementSizeInBytes, InitialCapacityPerCache, pool);
            Unsafe.CopyBlockUnaligned(targetCache.Buffer.Memory + targetByteIndex, sourceCache.Buffer.Memory + sourceByteIndex, (uint)sourceCache.ElementSizeInBytes);
            return new TypedIndex(typeId, targetByteIndex);
        }

        public int Add(ref ArrayList<WorkerPairCache> pairCaches, BufferPool pool, ref CollidablePair pair, ref CollidablePairPointers sourcePointers)
        {
            var pairIndex = Pairs.Count;
            Pairs.EnsureCapacity(Pairs.Count + 1, pool);
            ref var entry = ref Pairs.AllocateUnsafely();
            entry.Pair = pair;
            Debug.Assert(sourcePointers.ConstraintCache.Exists);
            var workerIndex = sourcePointers.ConstraintCache.Cache;
            ref var workerCache = ref pairCaches[workerIndex];
            Debug.Assert(!sourcePointers.CollisionDetectionCache.Exists || sourcePointers.CollisionDetectionCache.Cache == workerIndex);
            entry.ConstraintCache = CopyToBuilderCache(ref workerCache.constraintCaches, ref ConstraintCaches,
                sourcePointers.ConstraintCache.Type, sourcePointers.ConstraintCache.Index, pool);
            if (sourcePointers.CollisionDetectionCache.Exists)
            {
                entry.CollisionCache = CopyToBuilderCache(ref workerCache.collisionCaches, ref CollisionCaches,
                    sourcePointers.CollisionDetectionCache.Type, sourcePointers.CollisionDetectionCache.Index, pool);
            }
            else
            {
                entry.CollisionCache = new TypedIndex();
            }
            return pairIndex;
        }


        unsafe void CopyExistingLists(ref Buffer<UntypedList> sourceCaches, BufferPool pool, out Buffer<SleepingCache> inactiveCaches, out Buffer<int> typeRemap)
        {
            int sourceTypeCount = 0;
            for (int i = 0; i < sourceCaches.Length; ++i)
            {
                if (sourceCaches[i].Count > 0)
                {
                    ++sourceTypeCount;
                }
            }
            //Note that collision caches are not guaranteed to exist, so there may be no need to allocate room to store them.
            if (sourceTypeCount > 0)
            {
                pool.TakeAtLeast(sourceTypeCount, out inactiveCaches);
                int index = 0;
                pool.TakeAtLeast(sourceCaches.Length, out typeRemap);
                for (int i = 0; i < sourceCaches.Length; ++i)
                {
                    ref var sourceList = ref sourceCaches[i];
                    if (sourceList.Count > 0)
                    {
                        ref var inactiveCache = ref inactiveCaches[index];
                        inactiveCache.TypeId = i;
                        inactiveCache.List = new UntypedList(sourceList.ElementSizeInBytes, sourceList.Count, pool);
                        inactiveCache.List.ByteCount = sourceList.ByteCount;
                        inactiveCache.List.Count = sourceList.Count;
                        Unsafe.CopyBlockUnaligned(inactiveCache.List.Buffer.Memory, sourceList.Buffer.Memory, (uint)sourceList.ByteCount);
                        typeRemap[i] = index; //Note that unfilled mapping slots won't be accessed; this is only used for pointing pairs to the proper packed locations.
                        ++index;

                        //Clear for the next usage.
                        sourceList.ByteCount = 0;
                        sourceList.Count = 0;
                    }
                }
                //The inactive set's disposal uses allocation status as a loop terminator. Go ahead and clear any empty slots to avoid corrupt allocation state.
                inactiveCaches.Clear(index, inactiveCaches.Length - index);
            }
            else
            {
                typeRemap = new Buffer<int>();
                inactiveCaches = new Buffer<SleepingCache>();
            }
        }
        public void FinalizeSet(BufferPool pool, out SleepingSet set)
        {
            //Repackage the gathered caches into a smaller format for longer term storage.
            //This adds a little extra cost, but it 
            //1) avoids the need for most incremental resizes during inactive set construction by sharing allocations and
            //2) minimizes the memory required for the inactive set. Using the same format as the active set would require about a kilobyte per set, 
            //which gets expensive when you have tens of thousands of isolated islands!

            if (Pairs.Count > 0)
            {
                CopyExistingLists(ref ConstraintCaches, pool, out set.ConstraintCaches, out var constraintTypeRemap);
                CopyExistingLists(ref CollisionCaches, pool, out set.CollisionCaches, out var collisionTypeRemap);
                Debug.Assert(set.ConstraintCaches.Length > 0,
                    "While there may be no collision caches, pair mapping entries only exist for constraintful pairs.");

                set.Pairs = new QuickList<SleepingPair>(Pairs.Count, pool);
                for (int i = 0; i < Pairs.Count; ++i)
                {
                    ref var sourcePair = ref Pairs[i];
                    ref var remappedPair = ref set.Pairs.AllocateUnsafely();
                    remappedPair.Pair = sourcePair.Pair;
                    remappedPair.ConstraintCache = new TypedIndex(constraintTypeRemap[sourcePair.ConstraintCache.Type], sourcePair.ConstraintCache.Index);
                    remappedPair.CollisionCache = sourcePair.CollisionCache.Exists ?
                        new TypedIndex(collisionTypeRemap[sourcePair.CollisionCache.Type], sourcePair.CollisionCache.Index) : new TypedIndex();
                }
                pool.Return(ref constraintTypeRemap);
                if (collisionTypeRemap.Allocated)
                    pool.Return(ref collisionTypeRemap);
                Pairs.Count = 0;
            }
            else
            {
                //No pairs -> no set required.
                set = new SleepingSet();
            }
        }

        public void Dispose(BufferPool pool)
        {
            for (int i = 0; i < ConstraintCaches.Length; ++i)
            {
                if (ConstraintCaches[i].Buffer.Allocated)
                    pool.Return(ref ConstraintCaches[i].Buffer);
            }
            pool.Return(ref ConstraintCaches);
            for (int i = 0; i < CollisionCaches.Length; ++i)
            {
                if (CollisionCaches[i].Buffer.Allocated)
                    pool.Return(ref CollisionCaches[i].Buffer);
            }
            pool.Return(ref CollisionCaches);
            Pairs.Dispose(pool);
        }
    }
}
