using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.CollisionDetection
{

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
        Buffer<UntypedList> batches;
        //These collision tasks can then call upon some of the batcher's fixed function post processing stages.
        //For example, compound collisions generate multiple convex-convex manifolds which need to be reduced and combined into a single nonconvex manifold for 
        //efficiency in constraint solving.
        public BatcherContinuations<NonconvexReduction> NonconvexReductions;
        public BatcherContinuations<MeshReduction> MeshReductions;

        public unsafe CollisionBatcher(BufferPool pool, Shapes shapes, CollisionTaskRegistry collisionTypeMatrix, float dt, TCallbacks callbacks)
        {
            Pool = pool;
            Shapes = shapes;
            typeMatrix = collisionTypeMatrix;
            Dt = dt;
            Callbacks = callbacks;
            pool.Take(collisionTypeMatrix.tasks.Length, out batches);
            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
            batches.Clear(0, collisionTypeMatrix.tasks.Length);
            NonconvexReductions = new BatcherContinuations<NonconvexReduction>();
            MeshReductions = new BatcherContinuations<MeshReduction>();
            minimumBatchIndex = collisionTypeMatrix.tasks.Length;
            maximumBatchIndex = -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Add(ref CollisionTaskReference reference,
            int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, ref RigidPose poseA, ref RigidPose poseB, float speculativeMargin,
            int flipMask, ref PairContinuation pairContinuationInfo)
        {
            ref var batch = ref batches[reference.TaskIndex];
            var pairData = batch.AllocateUnsafely();
            Unsafe.CopyBlockUnaligned(pairData, shapeA, (uint)shapeSizeA);
            Unsafe.CopyBlockUnaligned(pairData += shapeSizeA, shapeB, (uint)shapeSizeB);
            var poses = (TestPair*)(pairData += shapeSizeB);
            poses->FlipMask = flipMask;
            poses->PoseA = poseA;
            poses->PoseB = poseB;
            poses->SpeculativeMargin = speculativeMargin;
            poses->Continuation = pairContinuationInfo;
            if (batch.Count == reference.BatchSize)
            {
                typeMatrix[reference.TaskIndex].ExecuteBatch(ref batch, ref this);
                batch.Count = 0;
                batch.ByteCount = 0;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(
            int shapeTypeA, int shapeTypeB, int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, 
            ref RigidPose poseA, ref RigidPose poseB, ref BodyVelocity velocityA, ref BodyVelocity velocityB, float speculativeMargin,
            ref PairContinuation pairContinuationInfo)
        {
            ref var reference = ref typeMatrix.GetTaskReference(shapeTypeA, shapeTypeB);
            if (reference.TaskIndex < 0)
            {
                //There is no task for this shape type pair. Immediately respond with an empty manifold.
                var manifold = new ConvexContactManifold();
                Callbacks.OnPairCompleted(pairContinuationInfo.PairId, &manifold);
                return;
            }
            ref var batch = ref batches[reference.TaskIndex];
            //The reference cached which type of pair the collision task expects. Populate it with the provided data.
            switch (reference.PairType)
            {
                case CollisionTaskPairType.StandardPair:
                    break;
                case CollisionTaskPairType.FliplessPair:
                    break;
                case CollisionTaskPairType.SpherePair:
                    break;
                case CollisionTaskPairType.SphereIncludingPair:
                    break;
                case CollisionTaskPairType.BoundsTestedPair:
                    break;
            }
            var pairSize = shapeSizeA + shapeSizeB + Unsafe.SizeOf<TestPair>();
            if (!batch.Buffer.Allocated)
            {
                batch = new UntypedList(pairSize, reference.BatchSize, Pool);
                if (minimumBatchIndex > reference.TaskIndex)
                    minimumBatchIndex = reference.TaskIndex;
                if (maximumBatchIndex < reference.TaskIndex)
                    maximumBatchIndex = reference.TaskIndex;
            }
            Debug.Assert(batch.Buffer.Allocated && batch.ElementSizeInBytes > 0 && batch.ElementSizeInBytes < 131072, "How'd the batch get corrupted?");
            if (shapeTypeA != reference.ExpectedFirstTypeId)
            {
                //The inputs need to be reordered to guarantee that the collision tasks are handed data in the proper order.
                Add(ref reference, shapeSizeB, shapeSizeA, shapeB, shapeA, ref poseB, ref poseA, speculativeMargin, -1, ref pairContinuationInfo);
            }
            else
            {
                Add(ref reference, shapeSizeA, shapeSizeB, shapeA, shapeB, ref poseA, ref poseB, speculativeMargin, 0, ref pairContinuationInfo);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(
           int shapeTypeA, int shapeTypeB, int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, ref RigidPose poseA, ref RigidPose poseB, float speculativeMargin,
           int pairId)
        {
            var pairContinuationInfo = new PairContinuation(pairId);
            Add(shapeTypeA, shapeTypeB, shapeSizeA, shapeSizeB, shapeA, shapeB, ref poseA, ref poseB, speculativeMargin, ref pairContinuationInfo);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(TypedIndex shapeIndexA, TypedIndex shapeIndexB, ref RigidPose poseA, ref RigidPose poseB, float speculativeMargin,
            ref PairContinuation pairContinuationInfo)
        {
            var shapeTypeA = shapeIndexA.Type;
            var shapeTypeB = shapeIndexB.Type;
            Shapes[shapeIndexA.Type].GetShapeData(shapeIndexA.Index, out var shapeA, out var shapeSizeA);
            Shapes[shapeIndexB.Type].GetShapeData(shapeIndexB.Index, out var shapeB, out var shapeSizeB);
            Add(shapeTypeA, shapeTypeB, shapeSizeA, shapeSizeB, shapeA, shapeB, ref poseA, ref poseB, speculativeMargin, ref pairContinuationInfo);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(TypedIndex shapeIndexA, TypedIndex shapeIndexB, ref RigidPose poseA, ref RigidPose poseB, float speculativeMargin, int pairId)
        {
            var pairContinuationInfo = new PairContinuation(pairId);
            Add(shapeIndexA, shapeIndexB, ref poseA, ref poseB, speculativeMargin, ref pairContinuationInfo);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TShapeA, TShapeB>(TShapeA shapeA, TShapeB shapeB, ref RigidPose poseA, ref RigidPose poseB, float speculativeMargin, int pairId)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
        {
            //Note that the shapes are passed by copy to avoid a GC hole. This isn't optimal, but it does allow a single code path, and the underlying function is the one
            //that's actually used by the narrowphase (and which will likely be used for most performance sensitive cases).
            //TODO: You could recover the performance and safety once generic pointers exist. By having pointers in the parameter list, we can require that the user handle GC safety.
            //(We could also have an explicit 'unsafe' overload, but that API complexity doesn't seem worthwhile. My guess is nontrivial uses will all use the underlying function directly.)
            var continuation = new PairContinuation(pairId);
            Add(shapeA.TypeId, shapeB.TypeId, Unsafe.SizeOf<TShapeA>(), Unsafe.SizeOf<TShapeB>(), Unsafe.AsPointer(ref shapeA), Unsafe.AsPointer(ref shapeB),
                ref poseA, ref poseB, speculativeMargin, ref continuation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush()
        {
            //The collision task registry guarantees that tasks which create work for other tasks always appear sooner in the task array than their child tasks.
            //Since there are no cycles, only one flush pass is required.
            for (int i = minimumBatchIndex; i <= maximumBatchIndex; ++i)
            {
                ref var batch = ref batches[i];
                if (batch.Count > 0)
                {
                    typeMatrix.tasks[i].ExecuteBatch(ref batch, ref this);
                }
                //Dispose of the batch and any associated buffers; since the flush is one pass, we won't be needing this again.
                if (batch.Buffer.Allocated)
                {
                    Pool.Return(ref batch.Buffer);
                }
            }
            var listPool = Pool.SpecializeFor<UntypedList>();
            listPool.Return(ref batches);
            NonconvexReductions.Dispose(Pool);
        }

        public unsafe void ProcessConvexResult(ConvexContactManifold* manifold, ref PairContinuation continuation)
        {
            if (continuation.Type == CollisionContinuationType.Direct)
            {
                //This result concerns a pair which had no higher level owner. Directly report the manifold result.
                Callbacks.OnPairCompleted(continuation.PairId, manifold);
            }
            else
            {
                //This result is associated with another pair and requires additional processing.
                //Before we move to the next stage, notify the submitter that the subpair has completed.
                Callbacks.OnChildPairCompleted(continuation.PairId, continuation.ChildA, continuation.ChildB, manifold);
                switch (continuation.Type)
                {
                    case CollisionContinuationType.NonconvexReduction:
                        {
                            NonconvexReductions.ContributeChildToContinuation(ref continuation, manifold, ref this);
                        }
                        break;
                    case CollisionContinuationType.BoundarySmoothedMesh:
                        {
                            MeshReductions.ContributeChildToContinuation(ref continuation, manifold, ref this);
                        }
                        break;
                }

            }
        }
    }
}
