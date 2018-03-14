using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{

    /// <summary>
    /// Describes the flow control to apply to a convex-convex pair report.
    /// </summary>
    public enum PairReportType : byte
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

    public struct TestPairSource
    {
        public int PairId;
        public int ChildA;
        public int ChildB;
        public PairReportType Type;
    }
    public struct TestPair
    {
        /// <summary>
        /// Stores whether the types involved in pair require that the resulting contact manifold be flipped to be consistent with the user-requested pair order.
        /// </summary>
        public int FlipMask;
        public RigidPose PoseA;
        public RigidPose PoseB;
        public TestPairSource Source;
    }

    //Writes by the narrowphase write shape data without type knowledge, so they can't easily operate on regular packing rules. Emulate this with a pack of 1.
    //This allows the reader to still have a quick way to interpret data rather than casting individual shapes.
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct TestPair<TShapeA, TShapeB>
            where TShapeA : struct, IShape where TShapeB : struct, IShape
    {
        public TShapeA A;
        public TShapeB B;
        public TestPair Shared;
    }


    public struct CollisionBatcher<TCallbacks> where TCallbacks : struct, ICollisionCallbacks
    {
        //The streaming batcher contains batches for pending work submitted by the user.
        //This pending work can be top level pairs like sphere versus sphere, but it may also be subtasks of submitted work.
        //Consider two compound bodies colliding. The pair will decompose into a set of potentially many convex subpairs.

        CollisionTaskRegistry typeMatrix;
        internal BufferPool pool;
        public TCallbacks Callbacks;

        int minimumBatchIndex, maximumBatchIndex;
        Buffer<UntypedList> batches;
        //A subset of collision tasks require a place to return information.
        Buffer<UntypedList> localContinuations;

        public unsafe CollisionBatcher(BufferPool pool, CollisionTaskRegistry collisionTypeMatrix, TCallbacks callbacks)
        {
            this.pool = pool;
            Callbacks = callbacks;
            typeMatrix = collisionTypeMatrix;
            pool.SpecializeFor<UntypedList>().Take(collisionTypeMatrix.tasks.Length, out batches);
            pool.SpecializeFor<UntypedList>().Take(collisionTypeMatrix.tasks.Length, out localContinuations);
            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
            batches.Clear(0, collisionTypeMatrix.tasks.Length);
            localContinuations.Clear(0, collisionTypeMatrix.tasks.Length);
            minimumBatchIndex = collisionTypeMatrix.tasks.Length;
            maximumBatchIndex = -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Add(ref CollisionTaskReference reference,
            int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, ref RigidPose poseA, ref RigidPose poseB,
            int flipMask, int pairId)
        {
            ref var batch = ref batches[reference.TaskIndex];
            var pairData = batch.AllocateUnsafely();
            Unsafe.CopyBlockUnaligned(pairData, shapeA, (uint)shapeSizeA);
            Unsafe.CopyBlockUnaligned(pairData += shapeSizeA, shapeB, (uint)shapeSizeB);
            var poses = (TestPair*)(pairData += shapeSizeB);
            poses->FlipMask = flipMask;
            poses->PoseA = poseA;
            poses->PoseB = poseB;
            poses->Source = new TestPairSource { PairId = pairId };
            if (batch.Count == reference.BatchSize)
            {
                typeMatrix[reference.TaskIndex].ExecuteBatch(ref batch, ref this);
                batch.Count = 0;
                batch.ByteCount = 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add(
            int shapeTypeIdA, int shapeTypeIdB, int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, ref RigidPose poseA, ref RigidPose poseB,
            int pairId)
        {
            ref var reference = ref typeMatrix.GetTaskReference(shapeTypeIdA, shapeTypeIdB);
            if (reference.TaskIndex < 0)
            {
                //There is no task for this shape type pair. Immediately respond with an empty manifold.
                var manifold = new ContactManifold();
                Callbacks.OnPairCompleted(pairId, &manifold);
                return;
            }
            ref var batch = ref batches[reference.TaskIndex];
            var pairSize = shapeSizeA + shapeSizeB + Unsafe.SizeOf<TestPair>();
            if (!batch.Buffer.Allocated)
            {
                batch = new UntypedList(pairSize, reference.BatchSize, pool);
                if (minimumBatchIndex > reference.TaskIndex)
                    minimumBatchIndex = reference.TaskIndex;
                if (maximumBatchIndex < reference.TaskIndex)
                    maximumBatchIndex = reference.TaskIndex;
            }
            Debug.Assert(batch.Buffer.Allocated && batch.ElementSizeInBytes > 0 && batch.ElementSizeInBytes < 131072, "How'd the batch get corrupted?");
            if (shapeTypeIdA != reference.ExpectedFirstTypeId)
            {
                //The inputs need to be reordered to guarantee that the collision tasks are handed data in the proper order.
                Add(ref reference, shapeSizeB, shapeSizeA, shapeB, shapeA, ref poseB, ref poseA, -1, pairId);
            }
            else
            {
                Add(ref reference, shapeSizeA, shapeSizeB, shapeA, shapeB, ref poseA, ref poseB, 0, pairId);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Add<TShapeA, TShapeB>(ref CollisionTaskReference reference,
            ref TShapeA shapeA, ref TShapeB shapeB, ref RigidPose poseA, ref RigidPose poseB,
            int flipMask, int pairId)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
        {
            ref var batch = ref batches[reference.TaskIndex];
            ref var pairData = ref batch.AllocateUnsafely<TestPair<TShapeA, TShapeB>>();
            pairData.A = shapeA;
            pairData.B = shapeB;
            pairData.Shared.FlipMask = flipMask;
            pairData.Shared.PoseA = poseA;
            pairData.Shared.PoseB = poseB;
            pairData.Shared.Source = new TestPairSource { PairId = pairId };
            if (batch.Count == reference.BatchSize)
            {
                typeMatrix[reference.TaskIndex].ExecuteBatch(ref batch, ref this);
                batch.Count = 0;
                batch.ByteCount = 0;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TShapeA, TShapeB>(ref TShapeA shapeA, ref TShapeB shapeB, ref RigidPose poseA, ref RigidPose poseB,
            int pairId)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
        {
            ref var reference = ref typeMatrix.GetTaskReference<TShapeA, TShapeB>();
            if (reference.TaskIndex < 0)
            {
                //There is no task for this shape type pair. Immediately respond with an empty manifold.
                var manifold = new ContactManifold();
                Callbacks.OnPairCompleted(pairId, &manifold);
                return;
            }
            ref var batch = ref batches[reference.TaskIndex];
            if (!batch.Buffer.Allocated)
            {
                batch = new UntypedList(Unsafe.SizeOf<TestPair<TShapeA, TShapeB>>(), reference.BatchSize, pool);
                if (minimumBatchIndex > reference.TaskIndex)
                    minimumBatchIndex = reference.TaskIndex;
                if (maximumBatchIndex < reference.TaskIndex)
                    maximumBatchIndex = reference.TaskIndex;
            }
            //The type comparison should be a compilation constant.
            if (typeof(TShapeA) != typeof(TShapeB) && default(TShapeA).TypeId != reference.ExpectedFirstTypeId)
            {
                //The inputs need to be reordered to guarantee that the collision tasks are handed data in the proper order.
                Add(ref reference, ref shapeB, ref shapeA, ref poseB, ref poseA, -1, pairId);
            }
            else
            {
                Add(ref reference, ref shapeA, ref shapeB, ref poseA, ref poseB, 0, pairId);
            }
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
                    pool.Return(ref batch.Buffer);
                }
                //Note that the local continuations are not guaranteed to be allocated when the batch is; many tasks don't have any associated continuations.
                if (localContinuations[i].Buffer.Allocated)
                {
                    pool.Return(ref localContinuations[i].Buffer);
                }
            }
            var listPool = pool.SpecializeFor<UntypedList>();
            listPool.Return(ref batches);
            listPool.Return(ref localContinuations);
        }




        public unsafe void ProcessConvexResult(ContactManifold* manifold, ref TestPairSource report)
        {
            if (report.Type == PairReportType.Direct)
            {
                //This result concerns a pair which had no higher level owner. Directly report the manifold result.
                Callbacks.OnPairCompleted(report.PairId, manifold);
            }
            else
            {
                //This result is associated with another pair and requires additional processing.
                //Before we move to the next stage, notify the submitter that the subpair has completed.
                Callbacks.OnChildPairCompleted(report.PairId, report.ChildA, report.ChildB, manifold);
                switch (report.Type)
                {
                    case PairReportType.NonconvexReduction:
                        {
                        }
                        break;
                }

            }
        }
    }
}
