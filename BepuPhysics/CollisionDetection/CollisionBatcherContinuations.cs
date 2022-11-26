using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Defines a type which includes information necessary to apply some form of post processing to a collision test result.
    /// </summary>
    public interface ICollisionTestContinuation
    {
        /// <summary>
        /// Creates a collision test continuation with the given number of slots for subpairs.
        /// </summary>
        /// <param name="slots">Number of subpair slots to include in the continuation.</param>
        /// <param name="pool">Pool to take resources from.</param>
        void Create(int slots, BufferPool pool);

        /// <summary>
        /// Handles what to do next when the child pair has finished execution and the resulting manifold is available.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the callbacks used in the batcher.</typeparam>
        /// <param name="report">Continuation instance being considered.</param>
        /// <param name="manifold">Contact manifold for the child pair.</param>
        /// <param name="batcher">Collision batcher processing the pair.</param>
        unsafe void OnChildCompleted<TCallbacks>(ref PairContinuation report, ref ConvexContactManifold manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks;
        /// <summary>
        /// Handles what to do next when the child pair was rejected for testing, and no manifold exists.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the callbacks used in the batcher.</typeparam>
        /// <param name="report">Continuation instance being considered.</param>
        /// <param name="batcher">Collision batcher processing the pair.</param>
        unsafe void OnUntestedChildCompleted<TCallbacks>(ref PairContinuation report, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks;

        /// <summary>
        /// Checks if the parent pair is complete and should be flushed.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the callbacks used in the batcher.</typeparam>
        /// <param name="pairId">Id of the pair to attempt to flush.</param>
        /// <param name="batcher">Collision batcher processing the pair.</param>
        /// <returns>True if the pair was done and got flushed, false otherwise.</returns>
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
        Direct = 0,
        /// <summary>
        /// Marks a pair as part of a set of a higher (potentially multi-manifold) pair, potentially requiring contact reduction.
        /// </summary>
        NonconvexReduction = 1,
        /// <summary>
        /// Marks a pair as a part of a set of mesh-convex collisions, potentially requiring mesh boundary smoothing.
        /// </summary>
        MeshReduction = 2,
        /// <summary>
        /// Marks a pair as a part of a set of mesh-convex collisions spawned by a mesh-compound pair, potentially requiring mesh boundary smoothing.
        /// </summary>
        CompoundMeshReduction = 3,
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

        /// <summary>
        /// Covers bits [0, 20) in the packed representation. Refers to the child pair index in a subtask generating collision task that generated this continuation.
        /// </summary>
        public const int ChildIndexBits = 20;
        /// <summary>
        /// Covers bits [20, 30) in the packed representation. Refers to the index of a subpair in a continuation processor.
        /// Maximum number should be equal to the sum of the batch sizes subtask generating collision tasks, which as of this writing is 384, but we'll include a little buffer.
        /// </summary>
        public const int ContinuationIndexBits = 10;
        /// <summary>
        /// Covers bits [30, 32) in the packed representation. Refers to which continuation processor should be used for this subpair.
        /// </summary>
        public const int ContinuationTypeBits = 2;

        public const int ExclusiveMaximumChildIndex = 1 << ChildIndexBits;
        public const int ExclusiveMaximumContinuationIndex = 1 << ContinuationIndexBits;
        public const int ExclusiveMaximumContinuationType = 1 << ContinuationTypeBits;

        const int TypeShift = ChildIndexBits + ContinuationIndexBits;
        const int IndexShift = ChildIndexBits;
        const int IndexMask = (1 << ContinuationIndexBits) - 1;
        const int ChildIndexMask = (1 << ChildIndexBits) - 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairContinuation(int pairId, int childA, int childB, CollisionContinuationType continuationType, int continuationIndex, int continuationChildIndex)
        {
            PairId = pairId;
            ChildA = childA;
            ChildB = childB;
            Debug.Assert(continuationChildIndex < ExclusiveMaximumChildIndex);
            Debug.Assert(continuationIndex < ExclusiveMaximumContinuationIndex);
            Debug.Assert((int)continuationType < ExclusiveMaximumContinuationType);
            Packed = (uint)(((int)continuationType << TypeShift) | (continuationIndex << IndexShift) | continuationChildIndex);
        }
        public PairContinuation(int pairId)
        {
            PairId = pairId;
            ChildA = 0;
            ChildB = 0;
            Packed = 0;
        }

        public CollisionContinuationType Type { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (CollisionContinuationType)(Packed >> TypeShift); } }
        public int Index { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (int)((Packed >> IndexShift) & IndexMask); } }
        public int ChildIndex { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (int)(Packed & ChildIndexMask); } }
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ContributeUntestedChildToContinuation<TCallbacks>(ref PairContinuation continuation, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var slot = ref Continuations[continuation.Index];
            slot.OnUntestedChildCompleted(ref continuation, ref batcher);
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
