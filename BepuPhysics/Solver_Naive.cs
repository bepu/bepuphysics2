//using BepuUtilities;
//using BepuUtilities.Memory;
//using BepuPhysics.Constraints;
//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Text;
//using System.Threading;

//namespace BepuPhysics
//{
//    public partial class Solver
//    {

//        int manualNaiveBlockIndex;
//        int manualNaiveExclusiveEndIndex;
//        void ManualNaivePrestep(int workerIndex)
//        {
//            int blockIndex;
//            ref var activeSet = ref ActiveSet;
//            var inverseDt = 1f / context.Dt;
//            while ((blockIndex = Interlocked.Increment(ref manualNaiveBlockIndex)) <= manualNaiveExclusiveEndIndex)
//            {
//                blockIndex -= 1;
//                ref var block = ref context.ConstraintBlocks.Blocks[blockIndex];
//                ref var typeBatch = ref activeSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
//                if (block.BatchIndex < FallbackBatchThreshold)
//                    TypeProcessors[typeBatch.TypeId].Prestep(ref typeBatch, bodies, context.Dt, inverseDt, block.StartBundle, block.End);
//                else
//                    TypeProcessors[typeBatch.TypeId].JacobiPrestep(ref typeBatch, bodies, ref ActiveSet.Fallback, context.Dt, inverseDt, block.StartBundle, block.End);
//            }
//        }
//        void ManualNaiveWarmStart(int workBlockIndex)
//        {
//            int blockIndex;
//            ref var activeSet = ref ActiveSet;
//            while ((blockIndex = Interlocked.Increment(ref manualNaiveBlockIndex)) <= manualNaiveExclusiveEndIndex)
//            {
//                ref var block = ref context.ConstraintBlocks.Blocks[blockIndex - 1];
//                ref var typeBatch = ref activeSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
//                if (block.BatchIndex < FallbackBatchThreshold)
//                {
//                    TypeProcessors[typeBatch.TypeId].JacobiWarmStart(ref typeBatch, ref bodies.ActiveSet.Velocities, ref context.FallbackResults[block.TypeBatchIndex], block.StartBundle, block.End);
//                }
//                else
//                {
//                    TypeProcessors[typeBatch.TypeId].WarmStart(ref typeBatch, ref bodies.ActiveSet.Velocities, block.StartBundle, block.End);
//                }
//            }
//        }

//        void ManualNaiveSolveIteration(int workBlockIndex)
//        {
//            int blockIndex;
//            ref var activeSet = ref ActiveSet;
//            while ((blockIndex = Interlocked.Increment(ref manualNaiveBlockIndex)) <= manualNaiveExclusiveEndIndex)
//            {
//                ref var block = ref context.ConstraintBlocks.Blocks[blockIndex - 1];
//                ref var typeBatch = ref activeSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
//                if (block.BatchIndex < FallbackBatchThreshold)
//                {
//                    TypeProcessors[typeBatch.TypeId].SolveIteration(ref typeBatch, ref bodies.ActiveSet.Velocities, block.StartBundle, block.End);
//                }
//                else
//                {
//                    TypeProcessors[typeBatch.TypeId].JacobiSolveIteration(ref typeBatch, ref bodies.ActiveSet.Velocities, ref context.FallbackResults[block.TypeBatchIndex], block.StartBundle, block.End);
//                }
//            }
//        }




//        public void ManualNaiveMultithreadedUpdate(IThreadDispatcher threadPool, BufferPool bufferPool, float dt, float inverseDt)
//        {
//            var workerCount = context.WorkerCount = threadPool.ThreadCount;
//            context.Dt = dt;
//            //First build a set of work blocks.
//            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
//            //These values are found by empirical tuning. The optimal values may vary by architecture.
//            const int targetBlocksPerBatchPerWorker = 4;
//            const int minimumBlockSizeInBundles = 4;
//            //Note that on a 3770K, the most expensive constraint bundles tend to cost less than 500ns to execute an iteration for. The minimum block size 
//            //is trying to balance having pointless numbers of blocks versus the worst case length of worker idling. For example, with a block size of 8,
//            //and assuming 500ns per bundle, we risk up to 4 microseconds per iteration-batch worth of idle time.
//            //This issue isn't unique to the somewhat odd workstealing scheme we use- it would still be a concern regardless.
//            var maximumBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
//            var filter = new MainSolveFilter();
//            BuildWorkBlocks(bufferPool, minimumBlockSizeInBundles, maximumBlocksPerBatch, ref filter);
//            ValidateWorkBlocks(ref filter);

//            manualNaiveBlockIndex = 0;
//            manualNaiveExclusiveEndIndex = context.ConstraintBlocks.Blocks.Count;
//            threadPool.DispatchWorkers(ManualNaivePrestep);

//            ref var activeSet = ref ActiveSet;
//            for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
//            {
//                manualNaiveBlockIndex = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
//                manualNaiveExclusiveEndIndex = context.BatchBoundaries[batchIndex];
//                threadPool.DispatchWorkers(ManualNaiveWarmStart);
//            }

//            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
//            {
//                for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
//                {
//                    manualNaiveBlockIndex = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
//                    manualNaiveExclusiveEndIndex = context.BatchBoundaries[batchIndex];
//                    threadPool.DispatchWorkers(ManualNaiveSolveIteration);
//                }
//            }


//            context.ConstraintBlocks.Blocks.Dispose(bufferPool);
//            bufferPool.Return(ref context.BatchBoundaries);
//        }

//    }
//}