//using BepuPhysics.Collidables;
//using BepuPhysics.CollisionDetection;
//using BepuUtilities;
//using BepuUtilities.Memory;
//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Runtime.CompilerServices;
//using System.Text;

//namespace BepuPhysics
//{
//    public struct BoundingBoxBatcher<TContinuation> 
//        where TContinuation : struct
//    {
//        internal BufferPool pool;
//        internal Shapes shapes;

//        int minimumBatchIndex, maximumBatchIndex;
//        Buffer<UntypedList> batches;
//        Buffer<UntypedList> localContinuations;

//        public unsafe BoundingBoxBatcher(Shapes shapes, BufferPool pool)
//        {
//            this.shapes = shapes;
//            this.pool = pool;
//            pool.SpecializeFor<UntypedList>().Take(shapes.RegisteredTypeSpan, out batches);
//            pool.SpecializeFor<UntypedList>().Take(shapes.RegisteredTypeSpan, out localContinuations);
//            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
//            batches.Clear(0, shapes.RegisteredTypeSpan);
//            localContinuations.Clear(0, shapes.RegisteredTypeSpan);
//            minimumBatchIndex = shapes.RegisteredTypeSpan;
//            maximumBatchIndex = -1;
//        }

//        public void Notify<TContinuations>(ref ContinuationIndex continuationIndex, BoundingBox boundingBox, ref TContinuations continuations)
//            where TContinuations : struct, IContinuations
//        {
//            if(continuationIndex.Type )
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public void Flush<TContinuations, TFilters>(ref TContinuations continuations)
//            where TContinuations : struct, IContinuations
//            where TFilters : struct, ICollisionSubtaskFilters
//        {
//            //The collision task registry guarantees that tasks which create work for other tasks always appear sooner in the task array than their child tasks.
//            //Since there are no cycles, only one flush pass is required.
//            for (int i = minimumBatchIndex; i <= maximumBatchIndex; ++i)
//            {
//                ref var batch = ref batches[i];
//                if (batch.Count > 0)
//                {
//                    shapes.UpdateBoundsBatch(i, ref batch, ref this, ref continuations);
//                }
//                //Dispose of the batch and any associated buffers; since the flush is one pass, we won't be needing this again.
//                if (batch.Buffer.Allocated)
//                {
//                    pool.Return(ref batch.Buffer);
//                }
//                //Note that the local continuations are not guaranteed to be allocated when the batch is; many tasks don't have any associated continuations.
//                if (localContinuations[i].Buffer.Allocated)
//                {
//                    pool.Return(ref this.localContinuations[i].Buffer);
//                }
//            }
//            var listPool = pool.SpecializeFor<UntypedList>();
//            listPool.Return(ref batches);
//            listPool.Return(ref this.localContinuations);
//        }
//    }
//}
