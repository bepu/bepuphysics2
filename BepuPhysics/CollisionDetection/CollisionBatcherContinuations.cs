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

        unsafe void OnChildCompleted<TCallbacks>(ref PairContinuation report, ConvexContactManifold* manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks;
        unsafe void OnChildCompletedEmpty<TCallbacks>(ref PairContinuation report, ref CollisionBatcher<TCallbacks> batcher)
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
        //TODO: We don't yet support boundary smoothing for meshes or convexes. Most likely, boundary smoothed convexes won't make it into the first release of the engine at all;
        //they're a pretty experimental feature with limited applications.
        ///// <summary>
        ///// Marks a pair as a part of a set of mesh-convex collisions, potentially requiring mesh boundary smoothing.
        ///// </summary>
        //BoundarySmoothedMesh,
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
        public PairContinuation(int pairId, int childA, int childB, CollisionContinuationType continuationType, int continuationIndex)
        {
            PairId = pairId;
            ChildA = childA;
            ChildB = childB;
            Debug.Assert(continuationIndex < (1 << 23));
            Packed = (uint)(((int)continuationType << 24) | continuationIndex);
        }
        public PairContinuation(int pairId)
        {
            PairId = pairId;
            ChildA = 0;
            ChildB = 0;
            Packed = 0;
        }

        public CollisionContinuationType Type { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (CollisionContinuationType)(Packed >> 24); } }
        public int Index { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (int)(Packed & 0x007FFFFF); } }
    }

    public struct BatcherContinuations<T> where T : ICollisionTestContinuation
    {
        public Buffer<T> Continuations;
        public IdPool<Buffer<int>> IdPool;
        const int InitialCapacity = 64;

        public ref T CreateContinuation(int slotsInContinuation, BufferPool pool, out int index)
        {
            if (!Continuations.Allocated)
            {
                Debug.Assert(!IdPool.AvailableIds.Span.Allocated);
                //Lazy initialization.
                pool.Take(InitialCapacity, out Continuations);
                IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), InitialCapacity, out IdPool);
            }
            index = IdPool.Take();
            if (index >= Continuations.Length)
            {
                pool.Resize(ref Continuations, index, index - 1);
            }
            ref var continuation = ref Continuations[index];
            continuation.Create(slotsInContinuation, pool);
            return ref Continuations[index];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ContributeChildToContinuation<TCallbacks>(ref PairContinuation continuation, ConvexContactManifold* manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            Continuations[continuation.Index].OnChildCompleted(ref continuation, manifold, ref batcher);
        }


        internal void Dispose(BufferPool pool)
        {
            if (Continuations.Allocated)
            {
                pool.ReturnUnsafely(Continuations.Id);
                Debug.Assert(IdPool.AvailableIds.Span.Allocated);
                IdPool.Dispose(pool.SpecializeFor<int>());
            }
#if DEBUG
            //Makes it a little easier to catch bad accesses.
            this = new BatcherContinuations<T>();
#endif
        }
    }
}
