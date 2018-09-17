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

//        public Buffer<int> StageIndices; //Used by the intermediate dispatcher.
//        void IntermediateWork(int workerIndex)
//        {
//            int syncStage = 0;
//            int blockIndex;
//            var endIndex = context.WorkBlocks.Count;
//            var inverseDt = 1f / context.Dt;
//            ref var activeSet = ref ActiveSet;
//            while ((blockIndex = Interlocked.Increment(ref StageIndices[syncStage])) <= endIndex)
//            {
//                ref var block = ref context.WorkBlocks[blockIndex - 1];
//                ref var typeBatch = ref activeSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
//               TypeProcessors[typeBatch.TypeId].Prestep(ref typeBatch, bodies, context.Dt, inverseDt, block.StartBundle, block.End);
//            }

//            InterstageSync(ref syncStage);
            
//            for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
//            {
//                endIndex = context.BatchBoundaries[batchIndex];
//                while ((blockIndex = Interlocked.Increment(ref StageIndices[syncStage])) <= endIndex)
//                {
//                    ref var block = ref context.WorkBlocks[blockIndex - 1];
//                    ref var typeBatch = ref activeSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
//                    TypeProcessors[typeBatch.TypeId].WarmStart(ref typeBatch, ref bodies.ActiveSet.Velocities, block.StartBundle, block.End);
//                }
//                InterstageSync(ref syncStage);
//            }

//            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
//            {
//                for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
//                {
//                    endIndex = context.BatchBoundaries[batchIndex];
//                    while ((blockIndex = Interlocked.Increment(ref StageIndices[syncStage])) <= endIndex)
//                    {
//                        ref var block = ref context.WorkBlocks[blockIndex - 1];
//                        ref var typeBatch = ref activeSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
//                        TypeProcessors[typeBatch.TypeId].SolveIteration(ref typeBatch, ref bodies.ActiveSet.Velocities, block.StartBundle, block.End);
//                    }
//                    InterstageSync(ref syncStage);
//                }
//            }
//        }





//        public double IntermediateMultithreadedUpdate(IThreadDispatcher threadPool, BufferPool bufferPool, float dt, float inverseDt)
//        {
//            var workerCount = context.WorkerCount = threadPool.ThreadCount;
//            context.WorkerCompletedCount = 0;
//            context.Dt = dt;
//            //First build a set of work blocks.
//            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
//            //These values are found by empirical tuning. The optimal values may vary by architecture.
//            //The goal here is to have just enough blocks that, in the event that we end up some underpowered threads (due to competition or hyperthreading), 
//            //there are enough blocks that workstealing will still generally allow the extra threads to be useful.
//            const int targetBlocksPerBatchPerWorker = 16;
//            const int minimumBlockSizeInBundles = 4;

//            var maximumBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
//            BuildWorkBlocks(bufferPool, minimumBlockSizeInBundles, maximumBlocksPerBatch);
//            ValidateWorkBlocks();

//            ref var activeSet = ref ActiveSet;
//            var stageCount = 1 + activeSet.Batches.Count * (iterationCount + 1);
//            bufferPool.SpecializeFor<int>().Take(stageCount, out StageIndices);

//            StageIndices[0] = 0;
//            int stageIndex = 1;
//            for (int i = 0; i < iterationCount + 1; ++i)
//            {
//                for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
//                {
//                    StageIndices[stageIndex++] = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
//                }
//            }

//            var start = Stopwatch.GetTimestamp();
//            threadPool.DispatchWorkers(IntermediateWork);
//            var end = Stopwatch.GetTimestamp();

//            bufferPool.SpecializeFor<int>().Return(ref StageIndices);
//            context.WorkBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
//            context.BatchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
//            return (end - start) / (double)Stopwatch.Frequency;
//        }



//    }
//}