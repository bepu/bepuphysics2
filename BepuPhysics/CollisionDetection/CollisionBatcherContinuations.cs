using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    public interface ICollisionTestContinuation
    {
        void Create(int slots, BufferPool pool);

        unsafe void OnChildCompleted<TCallbacks>(ref PairContinuation report, ref ConvexContactManifold manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks;
        unsafe void OnChildCompletedEmpty<TCallbacks>(ref PairContinuation report, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks;
        unsafe bool TryFlush<TCallbacks>(int pairId, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks;


    }

    /// <summary>
    /// Describes the flow control to apply to a convex-convex pair report.
    /// </summary>
    public enum CollisionContinuationType : byte
    {
        /// <summary>
        /// Marks a pair as requiring no further processing before being reported to the user supplied continuations.
        /// </summary>
        Direct,
        /// <summary>
        /// Marks a pair as part of a set of a higher (potentially multi-manifold) pair, potentially requiring contact reduction.
        /// </summary>
        NonconvexReduction,
        /// <summary>
        /// Marks a pair as a part of a set of mesh-convex collisions, potentially requiring mesh boundary smoothing.
        /// </summary>
        MeshReduction,
        /// <summary>
        /// Marks a pair as a part of a set of mesh-convex collisions spawned by a mesh-compound pair, potentially requiring mesh boundary smoothing.
        /// </summary>
        CompoundMeshReduction,
        //TODO: We don't yet support boundary smoothing for meshes or convexes. Most likely, boundary smoothed convexes won't make it into the first release of the engine at all;
        //they're a pretty experimental feature with limited applications.
        ///// <summary>
        ///// Marks a pair as a part of a set of convex-convex collisions, potentially requiring general convex boundary smoothing.
        ///// </summary>
        //BoundarySmoothedConvexes,           

    }

    public struct PairContinuation
    {
        public int PairId;
        public int ChildA;
        public int ChildB;
        public uint Packed;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairContinuation(int pairId, int childA, int childB, CollisionContinuationType continuationType, int continuationIndex, int continuationChildIndex)
        {
            PairId = pairId;
            ChildA = childA;
            ChildB = childB;
            //continuationChildIndex: [0, 17]
            //continuationIndex: [18, 27]
            //continuationType:  [28, 31]
            Debug.Assert(continuationIndex < (1 << 10));
            Debug.Assert(continuationChildIndex < (1 << 18));
            Debug.Assert((int)continuationType < (1 << 4));
            Packed = (uint)(((int)continuationType << 28) | (continuationIndex << 18) | continuationChildIndex);
        }
        public PairContinuation(int pairId)
        {
            PairId = pairId;
            ChildA = 0;
            ChildB = 0;
            Packed = 0;
        }

        public CollisionContinuationType Type { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (CollisionContinuationType)(Packed >> 28); } }
        public int Index { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (int)((Packed >> 18) & ((1 << 10) - 1)); } }
        public int ChildIndex { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (int)(Packed & ((1 << 18) - 1)); } }
    }

    public struct BatcherContinuations<T> where T : unmanaged, ICollisionTestContinuation
    {
        public Buffer<T> Continuations;
        public IdPool IdPool;
        const int InitialCapacity = 64;

        public ref T CreateContinuation(int slotsInContinuation, BufferPool pool, out int index)
        {
            if (!Continuations.Allocated)
            {
                Debug.Assert(!IdPool.Allocated);
                //Lazy initialization.
                pool.TakeAtLeast(InitialCapacity, out Continuations);
                IdPool = new IdPool(InitialCapacity, pool);
            }
            index = IdPool.Take();
            if (index >= Continuations.Length)
            {
                pool.ResizeToAtLeast(ref Continuations, index + 1, index);
            }
            ref var continuation = ref Continuations[index];
            continuation.Create(slotsInContinuation, pool);
            return ref Continuations[index];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ContributeChildToContinuation<TCallbacks>(ref PairContinuation continuation, ref ConvexContactManifold manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var slot = ref Continuations[continuation.Index];
            slot.OnChildCompleted(ref continuation, ref manifold, ref batcher);
            if (slot.TryFlush(continuation.PairId, ref batcher))
            {
                //The entire continuation has completed; free the slot.
                IdPool.Return(continuation.Index, batcher.Pool);
            }
        }


        internal void Dispose(BufferPool pool)
        {
            if (Continuations.Allocated)
            {
                pool.ReturnUnsafely(Continuations.Id);
                Debug.Assert(IdPool.Allocated);
                IdPool.Dispose(pool);
            }
#if DEBUG
            //Makes it a little easier to catch bad accesses.
            this = new BatcherContinuations<T>();
#endif
        }
    }
}
