using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;
using System;
using BepuUtilities;

namespace BepuPhysics.CollisionDetection
{
    unsafe struct UntypedBlob
    {
        public Buffer<byte> Buffer;
        public int ByteCount;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public byte* Allocate(int allocationSizeInBytes)
        {
            var newByteCount = ByteCount + allocationSizeInBytes;
            Debug.Assert(newByteCount <= Buffer.Length, "This collection doesn't auto-resize! You forgot to initialize this, or initialized it to an insufficient capacity.");
            var toReturn = Buffer.Memory + ByteCount;
            ByteCount = newByteCount;
            return toReturn;
        }
    }
    struct CollisionBatch
    {
        public UntypedList Pairs;
        public UntypedBlob Shapes;
    }

    public struct CollisionBatcher<TCallbacks> where TCallbacks : struct, ICollisionCallbacks
    {

        public BufferPool Pool;
        public Shapes Shapes;
        CollisionTaskRegistry typeMatrix;
        public TCallbacks Callbacks;
        /// <summary>
        /// Timestep duration used by pairs which rely on velocity to compute local bounding boxes for pruning.
        /// </summary>
        public float Dt;

        int minimumBatchIndex, maximumBatchIndex;
        //The streaming batcher contains batches for pending work submitted by the user.
        //This pending work can be top level pairs like sphere versus sphere, but it may also be subtasks of submitted work.
        //Consider two compound bodies colliding. The pair will decompose into a set of potentially many convex subpairs.
        internal Buffer<CollisionBatch> batches;
        //These collision tasks can then call upon some of the batcher's fixed function post processing stages.
        //For example, compound collisions generate multiple convex-convex manifolds which need to be reduced and combined into a single nonconvex manifold for 
        //efficiency in constraint solving.
        public BatcherContinuations<NonconvexReduction> NonconvexReductions;
        public BatcherContinuations<MeshReduction> MeshReductions;
        public BatcherContinuations<CompoundMeshReduction> CompoundMeshReductions;

        public unsafe CollisionBatcher(BufferPool pool, Shapes shapes, CollisionTaskRegistry collisionTypeMatrix, float dt, TCallbacks callbacks)
        {
            Pool = pool;
            Shapes = shapes;
            typeMatrix = collisionTypeMatrix;
            Dt = dt;
            Callbacks = callbacks;
            pool.TakeAtLeast(collisionTypeMatrix.tasks.Length, out batches);
            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
            batches.Clear(0, collisionTypeMatrix.tasks.Length);
            NonconvexReductions = new BatcherContinuations<NonconvexReduction>();
            MeshReductions = new BatcherContinuations<MeshReduction>();
            CompoundMeshReductions = new BatcherContinuations<CompoundMeshReduction>();
            minimumBatchIndex = collisionTypeMatrix.tasks.Length;
            maximumBatchIndex = -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe ref TPair AllocatePair<TPair>(ref CollisionBatch batch, ref CollisionTaskReference reference) where TPair : ICollisionPair<TPair>
        {
            if (!batch.Pairs.Buffer.Allocated)
            {
                batch.Pairs = new UntypedList(Unsafe.SizeOf<TPair>(), reference.BatchSize, Pool);
                if (minimumBatchIndex > reference.TaskIndex)
                    minimumBatchIndex = reference.TaskIndex;
                if (maximumBatchIndex < reference.TaskIndex)
                    maximumBatchIndex = reference.TaskIndex;
            }
            return ref batch.Pairs.AllocateUnsafely<TPair>();
        }

        private unsafe void Add(ref CollisionTaskReference reference, int flipMask, int shapeTypeA, int shapeTypeB, void* shapeA, void* shapeB,
            in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, in BodyVelocity velocityA, in BodyVelocity velocityB, float speculativeMargin, float maximumExpansion,
            in PairContinuation continuation)
        {
            ref var batch = ref batches[reference.TaskIndex];
            //If you find yourself needing to add a significant number of variations here, you may want to consider moving to a more flexible form of indirection.
            //Giving collision tasks the ability to create their own pairs (which would therefore properly match the ExecuteBatch's expectations by default)
            //would be easy enough, and the performance penatly would be essentially nonexistent.
            switch (reference.PairType)
            {
                case CollisionTaskPairType.StandardPair:
                    {
                        ref var pair = ref AllocatePair<CollisionPair>(ref batch, ref reference);
                        pair.A = shapeA;
                        pair.B = shapeB;
                        pair.FlipMask = flipMask;
                        pair.OffsetB = offsetB;
                        pair.OrientationA = orientationA;
                        pair.OrientationB = orientationB;
                        pair.SpeculativeMargin = speculativeMargin;
                        pair.Continuation = continuation;
                    }
                    break;
                case CollisionTaskPairType.FliplessPair:
                    {
                        ref var pair = ref AllocatePair<FliplessPair>(ref batch, ref reference);
                        pair.A = shapeA;
                        pair.B = shapeB;
                        pair.OffsetB = offsetB;
                        pair.OrientationA = orientationA;
                        pair.OrientationB = orientationB;
                        pair.SpeculativeMargin = speculativeMargin;
                        pair.Continuation = continuation;
                    }
                    break;
                case CollisionTaskPairType.SpherePair:
                    {
                        ref var pair = ref AllocatePair<SpherePair>(ref batch, ref reference);
                        pair.A = Unsafe.AsRef<Sphere>(shapeA);
                        pair.B = Unsafe.AsRef<Sphere>(shapeB);
                        pair.OffsetB = offsetB;
                        pair.SpeculativeMargin = speculativeMargin;
                        pair.Continuation = continuation;
                    }
                    break;
                case CollisionTaskPairType.SphereIncludingPair:
                    {
                        ref var pair = ref AllocatePair<SphereIncludingPair>(ref batch, ref reference);
                        pair.A = Unsafe.AsRef<Sphere>(shapeA);
                        pair.B = shapeB;
                        pair.FlipMask = flipMask;
                        pair.OffsetB = offsetB;
                        pair.OrientationB = orientationB;
                        pair.SpeculativeMargin = speculativeMargin;
                        pair.Continuation = continuation;
                    }
                    break;
                case CollisionTaskPairType.BoundsTestedPair:
                    {
                        ref var pair = ref AllocatePair<BoundsTestedPair>(ref batch, ref reference);
                        pair.A = shapeA;
                        pair.B = shapeB;
                        pair.FlipMask = flipMask;
                        pair.OffsetB = offsetB;
                        pair.OrientationA = orientationA;
                        pair.OrientationB = orientationB;
                        pair.RelativeLinearVelocityA = velocityA.Linear - velocityB.Linear;
                        pair.AngularVelocityA = velocityA.Angular;
                        pair.AngularVelocityB = velocityB.Angular;
                        pair.MaximumExpansion = maximumExpansion;
                        pair.SpeculativeMargin = speculativeMargin;
                        pair.Continuation = continuation;
                    }
                    break;
            }
            Debug.Assert(batch.Pairs.Buffer.Allocated && batch.Pairs.ElementSizeInBytes > 0 && batch.Pairs.ElementSizeInBytes < 131072, "How'd the batch get corrupted?");
            if (batch.Pairs.Count == reference.BatchSize)
            {
                typeMatrix[reference.TaskIndex].ExecuteBatch(ref batch.Pairs, ref this);
                batch.Pairs.Count = 0;
                batch.Pairs.ByteCount = 0;
                batch.Shapes.ByteCount = 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void AddDirectly(
            ref CollisionTaskReference reference, int shapeTypeA, int shapeTypeB, void* shapeA, void* shapeB,
            in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, in BodyVelocity velocityA, in BodyVelocity velocityB, float speculativeMargin, float maximumExpansion,
            in PairContinuation pairContinuation)
        {
            if (reference.TaskIndex < 0)
            {
                //There is no task for this shape type pair. Immediately respond with an empty manifold.
                var manifold = new ConvexContactManifold();
                Callbacks.OnPairCompleted(pairContinuation.PairId, ref manifold);
                return;
            }
            if (shapeTypeA != reference.ExpectedFirstTypeId)
            {
                Debug.Assert(shapeTypeB == reference.ExpectedFirstTypeId);
                Add(ref reference, -1, shapeTypeB, shapeTypeA, shapeB, shapeA, -offsetB, orientationB, orientationA, velocityB, velocityA, speculativeMargin, maximumExpansion, pairContinuation);
            }
            else
            {
                Add(ref reference, 0, shapeTypeA, shapeTypeB, shapeA, shapeB, offsetB, orientationA, orientationB, velocityA, velocityB, speculativeMargin, maximumExpansion, pairContinuation);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddDirectly(
           int shapeTypeA, int shapeTypeB, void* shapeA, void* shapeB,
           in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, in BodyVelocity velocityA, in BodyVelocity velocityB, float speculativeMargin, float maximumExpansion,
           in PairContinuation pairContinuation)
        {
            ref var reference = ref typeMatrix.GetTaskReference(shapeTypeA, shapeTypeB);
            AddDirectly(ref reference, shapeTypeA, shapeTypeB, shapeA, shapeB, offsetB, orientationA, orientationB, velocityA, velocityB, speculativeMargin, maximumExpansion, pairContinuation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void AddDirectly(int shapeTypeA, int shapeTypeB, void* shapeA, void* shapeB,
            in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, float speculativeMargin, in PairContinuation pairContinuation)
        {
            AddDirectly(shapeTypeA, shapeTypeB, shapeA, shapeB, offsetB, orientationA, orientationB, default, default, speculativeMargin, default, pairContinuation);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(TypedIndex shapeIndexA, TypedIndex shapeIndexB,
            in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, in BodyVelocity velocityA, in BodyVelocity velocityB,
            float speculativeMargin, float maximumExpansion,
            in PairContinuation continuation)
        {
            var shapeTypeA = shapeIndexA.Type;
            var shapeTypeB = shapeIndexB.Type;
            Shapes[shapeIndexA.Type].GetShapeData(shapeIndexA.Index, out var shapeA, out var shapeSizeA);
            Shapes[shapeIndexB.Type].GetShapeData(shapeIndexB.Index, out var shapeB, out var shapeSizeB);
            AddDirectly(shapeTypeA, shapeTypeB, shapeA, shapeB, offsetB, orientationA, orientationB, velocityA, velocityB, speculativeMargin, maximumExpansion, continuation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(TypedIndex shapeIndexA, TypedIndex shapeIndexB, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB,
            float speculativeMargin, in PairContinuation continuation)
        {
            Add(shapeIndexA, shapeIndexB, offsetB, orientationA, orientationB, default, default, speculativeMargin, default, continuation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void CacheShapes(ref CollisionTaskReference reference, void* shapeA, void* shapeB, int shapeSizeA, int shapeSizeB, out void* cachedShapeA, out void* cachedShapeB)
        {
            //TODO: This does some pointless allocations and copies for sphere pairs...
            ref var batch = ref batches[reference.TaskIndex];
            if (!batch.Shapes.Buffer.Allocated)
            {
                var size = reference.BatchSize * (shapeSizeA + shapeSizeB);
                Pool.TakeAtLeast(size, out batch.Shapes.Buffer);
                Debug.Assert(batch.Shapes.ByteCount == 0);
            }
            cachedShapeA = batch.Shapes.Allocate(shapeSizeA);
            cachedShapeB = batch.Shapes.Allocate(shapeSizeB);
            //TODO: Given the size of these copies, it's not clear that this copy implementation is ideal. Wouldn't worry too much about it.
            Buffer.MemoryCopy(shapeA, cachedShapeA, shapeSizeA, shapeSizeA);
            Buffer.MemoryCopy(shapeB, cachedShapeB, shapeSizeB, shapeSizeB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void CacheShapeB(int shapeTypeA, int shapeTypeB, void* shapeDataB, int shapeSizeB, out void* cachedShapeDataB)
        {
            ref var reference = ref typeMatrix.GetTaskReference(shapeTypeA, shapeTypeB);
            ref var batch = ref batches[reference.TaskIndex];
            if (!batch.Shapes.Buffer.Allocated)
            {
                var size = reference.BatchSize * (Shapes[shapeTypeA].ShapeDataSize + shapeSizeB);
                Pool.TakeAtLeast(size, out batch.Shapes.Buffer);
                Debug.Assert(batch.Shapes.ByteCount == 0);
            }
            cachedShapeDataB = batch.Shapes.Allocate(shapeSizeB);
            Buffer.MemoryCopy(shapeDataB, cachedShapeDataB, shapeSizeB, shapeSizeB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(
           int shapeTypeA, int shapeTypeB, int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, float speculativeMargin, int pairId)
        {
            ref var reference = ref typeMatrix.GetTaskReference(shapeTypeA, shapeTypeB);
            CacheShapes(ref reference, shapeA, shapeB, shapeSizeA, shapeSizeB, out var cachedShapeA, out var cachedShapeB);
            AddDirectly(ref reference, shapeTypeA, shapeTypeB, cachedShapeA, cachedShapeB, offsetB, orientationA, orientationB, default, default, speculativeMargin, default, new PairContinuation(pairId));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TShapeA, TShapeB>(TShapeA shapeA, TShapeB shapeB, in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, float speculativeMargin, int pairId)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
        {
            //Note that the shapes are passed by copy to avoid a GC hole. This isn't optimal, but it does allow a single code path, and the underlying function is the one
            //that's actually used by the narrowphase (and which will likely be used for most performance sensitive cases).
            //TODO: You could recover the performance and safety once generic pointers exist. By having pointers in the parameter list, we can require that the user handle GC safety.
            //(We could also have an explicit 'unsafe' overload, but that API complexity doesn't seem worthwhile. My guess is nontrivial uses will all use the underlying function directly.)
            Add(shapeA.TypeId, shapeB.TypeId, Unsafe.SizeOf<TShapeA>(), Unsafe.SizeOf<TShapeB>(), Unsafe.AsPointer(ref shapeA), Unsafe.AsPointer(ref shapeB),
                offsetB, orientationA, orientationB, speculativeMargin, pairId);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush()
        {
            //The collision task registry guarantees that tasks which create work for other tasks always appear sooner in the task array than their child tasks.
            //Since there are no cycles, only one flush pass is required.
            for (int i = minimumBatchIndex; i <= maximumBatchIndex; ++i)
            {
                ref var batch = ref batches[i];
                if (batch.Pairs.Count > 0)
                {
                    typeMatrix.tasks[i].ExecuteBatch(ref batch.Pairs, ref this);
                }
                //Dispose of the batch and any associated buffers; since the flush is one pass, we won't be needing this again.
                if (batch.Pairs.Buffer.Allocated)
                {
                    Pool.Return(ref batch.Pairs.Buffer);
                }
                if (batch.Shapes.Buffer.Allocated)
                {
                    Pool.Return(ref batch.Shapes.Buffer);
                }
            }
            Pool.Return(ref batches);
            NonconvexReductions.Dispose(Pool);
            MeshReductions.Dispose(Pool);
            CompoundMeshReductions.Dispose(Pool);
        }

        public unsafe void ProcessConvexResult(ref ConvexContactManifold manifold, ref PairContinuation continuation)
        {
#if DEBUG
            if (manifold.Count > 0)
            {
                manifold.Normal.Validate();
            }
#endif
            if (continuation.Type == CollisionContinuationType.Direct)
            {
                //This result concerns a pair which had no higher level owner. Directly report the manifold result.
                Callbacks.OnPairCompleted(continuation.PairId, ref manifold);
            }
            else
            {
                //This result is associated with another pair and requires additional processing.
                //Before we move to the next stage, notify the submitter that the subpair has completed.
                Callbacks.OnChildPairCompleted(continuation.PairId, continuation.ChildA, continuation.ChildB, ref manifold);
                switch (continuation.Type)
                {
                    case CollisionContinuationType.NonconvexReduction:
                        {
                            NonconvexReductions.ContributeChildToContinuation(ref continuation, ref manifold, ref this);
                        }
                        break;
                    case CollisionContinuationType.MeshReduction:
                        {
                            MeshReductions.ContributeChildToContinuation(ref continuation, ref manifold, ref this);
                        }
                        break;
                    case CollisionContinuationType.CompoundMeshReduction:
                        {
                            CompoundMeshReductions.ContributeChildToContinuation(ref continuation, ref manifold, ref this);
                        }
                        break;
                }

            }
        }
    }
}
