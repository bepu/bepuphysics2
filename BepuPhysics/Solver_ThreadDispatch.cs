using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace BepuPhysics
{
    public partial class Solver
    {

        //This is going to look a bit more complicated than would be expected for a series of forloops.
        //A naive implementation would look something like:
        //1) PRESTEP: Parallel dispatch over all constraints, regardless of batch. (Presteps do not write shared data, so there is no need to redispatch per-batch.)
        //2) WARMSTART: Loop over all constraint batches, parallel dispatch over all constraints in batch. (Warmstarts read and write velocities, so batch borders must be respected.)
        //3) SOLVE ITERATIONS: Loop over iterations, loop over all constraint batches, parallel dispatch over all constraints in batch. (Solve iterations also read/write.) 

        //There are a few problems with this approach:
        //1) Fork-join dispatches are not free. Expect ~2us overhead on the main thread for each one, regardless of the workload.
        //If there are 10 constraint batches and 10 iterations, you're up to 1 + 10 + 10 * 10 = 111 dispatches. Over a fifth of a millisecond in pure overhead.
        //This is just a byproduct of general purpose dispatchers not being able to make use of extremely fine grained application knowledge.
        //Every dispatch has to get the threads rolling and scheduled, then the threads have to figure out when to go back into a blocked state
        //when no more work is unavailable, and so on. Over and over and over again.

        //2) The forloop provider is not guaranteed to maintain a relationship between forloop index and underlying hardware threads across multiple dispatches.
        //In fact, we should expect the opposite. Work stealing is an important feature for threadpools to avoid pointless idle time.
        //Unfortunately, this can destroy potential cache locality across solver iterations. This matters only a little bit for smaller simulations on a single processor-
        //if a single core's solver iteration data can fit in L2, then having 'sticky' scheduling will help over multiple iterations. For a 256 KiB per-core L2,
        //that would be a simulation of only about 700 big constraints per core. (That is actually quite a few back in BEPUphysics v1 land... not so much in v2.)

        //Sticky scheduling becomes more important when talking about larger simulations. Consider L3; an 8 MiB L3 cache can hold over 20000 heavy constraints
        //worth of solver iteration data. This is usually shared across all cores of a processor, so the stickiness isn't always useful. However, consider
        //a multiprocessor system. When there are multiple processors, there are multiple L3 caches. Limiting the amount of communication between processors
        //(and to potentially remote parts of system memory) is important, since those accesses tend to have longer latency and lower total bandwidth than direct L3 accesses.
        //But you don't have to resort to big servers to see something like this- some processors, notably the recent Ryzen line, actually behave a bit like 
        //multiple processors that happen to be stuck on the same chip. If the application requires tons of intercore communication, performance will suffer.
        //And of course, cache misses just suck.

        //3) Work stealing implementations that lack application knowledge will tend to make a given worker operate across noncontiguous regions, harming locality and forcing cache misses.

        //So what do we do? We have special guarantees:
        //1) We have to do a bunch of solver iterations in sequence, covering the exact same data over and over. Even the prestep and warmstart cover a lot of the same data.
        //2) We can control the dispatch sizes within a frame. They're going to be the same, over and over, and the next dispatch follows immediately after the last.
        //3) We can guarantee that individual work blocks are fairly small. (A handful of microseconds.)

        //So, there's a few parts to the solution as implemented:
        //1) Dispatch *once* and perform fine grained synchronization with busy waits to block at constraint batch borders. Unless the operating system
        //reschedules a thread (which is very possible, but not a constant occurrence), a worker index will stay associated with the same underlying hardware.
        //2) Worker start locations are spaced across the work blocks so that each worker has a high probability of claiming multiple blocks contiguously.
        //3) Workers track the largest contiguous region that they've been able to claim within an iteration. This is used to provide the next iteration a better starting guess.

        //So, for the most part, the same core/processor will tend to work on the same data over the course of the solve. Hooray!

        //A couple of notes:
        //1) We explicitly don't care about maintaining worker-data relationships between frames. The cache will likely be trashed by the rest of the program- even other parts
        //of the physics simulation will evict stuff. We're primarily concerned about scheduling within 
        //2) Note that neither the prestep nor the warmstart are used to modify the work distribution for the solve- neither of those stages is proportional to the solve iteration load.

        //3) Core-data stickiness doesn't really offer much value for L1/L2 caches. It doesn't take much to evict the entirety of the old data- a 3770K only holds 256KB in its L2.
        //Even if we optimized every constraint to require no more than 350B per iteration for the heaviest constraint 
        //(when this was written, it was at 602B per iteration), a single core's L2 could only hold up to about 750 constraints. 
        //So, the 3770K under ideal circumstances would avoid evicting on a per-iteration basis if the simulation had a total of less than 3000 such constraints.
        //A single thread of a 3700K at 4.5ghz could do prestep-warmstart-8iterations for that in ~2.6 milliseconds. In other words, it's a pretty small simulation.

        //Sticky scheduling only becomes more useful when dealing with multiprocessor systems (or multiprocessor-ish systems, like ryzen) and big datasets, like you might find in an MMO server.
        //A 3770K has 8MB of L3 cache shared across all cores, enough to hold a little under 24000 large constraint solves worth of data between iterations, which is a pretty large chunk.
        //If you had four similar processors, you could ideally handle almost 100,000 constraints without suffering significant evictions in each processor's L3 during iterations.
        //Without sticky scheduling, memory bandwidth use could skyrocket during iterations as the L3 gets missed over and over.


        struct WorkBlock
        {
            public int BatchIndex;
            public int TypeBatchIndex;
            /// <summary>
            /// Index of the first bundle in the block.
            /// </summary>
            public int StartBundle;
            /// <summary>
            /// Exlusive end index of the bundle. Index of the last bundle in the block is End - 1.
            /// </summary>
            public int End;
        }



        private void BuildWorkBlocks(BufferPool bufferPool, int minimumBlockSizeInBundles, int targetBlocksPerBatch)
        {
            var blockPool = bufferPool.SpecializeFor<WorkBlock>();

            var maximumBlockCount = 0;
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var typeBatchCount = Batches[batchIndex].TypeBatches.Count;
                maximumBlockCount += typeBatchCount > targetBlocksPerBatch ? typeBatchCount : targetBlocksPerBatch;
            }
            QuickList<WorkBlock, Buffer<WorkBlock>>.Create(blockPool, maximumBlockCount, out context.WorkBlocks);
            QuickList<int, Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), Batches.Count, out context.BatchBoundaries);
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var batch = Batches[batchIndex];
                var bundleCount = 0;
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    bundleCount += batch.TypeBatches[typeBatchIndex].BundleCount;
                }
                //Create a goal size for the blocks based on the number of bundles present.
                var targetBlockSizeInBundles = bundleCount / targetBlocksPerBatch;
                if (bundleCount - targetBlockSizeInBundles * targetBlocksPerBatch > 0)
                    ++targetBlockSizeInBundles;
                if (targetBlockSizeInBundles < minimumBlockSizeInBundles)
                    targetBlockSizeInBundles = minimumBlockSizeInBundles;

                //Walk through the type batches in order. Avoid tiny 'remainder' batches by spreading any remainder over all previous batches.
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    var typeBatchBlockCount = typeBatch.BundleCount / targetBlockSizeInBundles;
                    if (typeBatchBlockCount == 0)
                        typeBatchBlockCount = 1;

                    var blockSize = typeBatch.BundleCount / typeBatchBlockCount;
                    var remainder = typeBatch.BundleCount - blockSize * typeBatchBlockCount;

                    WorkBlock block;
                    block.BatchIndex = batchIndex;
                    block.TypeBatchIndex = typeBatchIndex;
                    block.End = 0;
                    for (int i = 0; i < typeBatchBlockCount; ++i)
                    {
                        int blockBundleCount = remainder-- > 0 ? blockSize + 1 : blockSize;
                        //Use the previous end as the new start.
                        block.StartBundle = block.End;
                        block.End = block.StartBundle + blockBundleCount;
                        Debug.Assert(block.StartBundle >= 0 && block.StartBundle < typeBatch.BundleCount);
                        Debug.Assert(block.End >= block.StartBundle + Math.Min(minimumBlockSizeInBundles, typeBatch.BundleCount) && block.End <= typeBatch.BundleCount);
                        context.WorkBlocks.AddUnsafely(ref block);
                    }
                }
                context.BatchBoundaries.AddUnsafely(context.WorkBlocks.Count);
            }
        }


        struct WorkerBounds
        {
            /// <summary>
            /// Inclusive start of blocks known to be claimed by any worker.
            /// </summary>
            public int Min;
            /// <summary>
            /// Exclusive end of blocks known to be claimed by any worker.
            /// </summary>
            public int Max;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Merge(ref WorkerBounds current, ref WorkerBounds mergeSource)
            {
                if (mergeSource.Min < current.Min)
                    current.Min = mergeSource.Min;
                if (mergeSource.Max > current.Max)
                    current.Max = mergeSource.Max;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool BoundsTouch(ref WorkerBounds a, ref WorkerBounds b)
            {
                //Note that touching is sufficient reason to merge. They don't have to actually intersect.
                return a.Min - b.Max <= 0 && b.Min - a.Max <= 0;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void MergeIfTouching(ref WorkerBounds current, ref WorkerBounds other)
            {
                //Could be a little more clever here if it matters.
                if (BoundsTouch(ref current, ref other))
                    Merge(ref current, ref other);

            }
        }
        //Just bundling these up to avoid polluting the this. intellisense.
        struct MultithreadingParameters
        {
            public float Dt;
            public QuickList<WorkBlock, Buffer<WorkBlock>> WorkBlocks;
            public Buffer<int> BlockClaims;
            public QuickList<int, Buffer<int>> BatchBoundaries;
            public int WorkerCompletedCount;
            public int WorkerCount;

            public Buffer<WorkerBounds> WorkerBoundsA;
            public Buffer<WorkerBounds> WorkerBoundsB;

        }
        MultithreadingParameters context;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void MergeWorkerBounds(ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex)
        {
            for (int i = 0; i < workerIndex; ++i)
            {
                WorkerBounds.MergeIfTouching(ref bounds, ref allWorkerBounds[i]);
            }
            for (int i = workerIndex + 1; i < context.WorkerCount; ++i)
            {
                WorkerBounds.MergeIfTouching(ref bounds, ref allWorkerBounds[i]);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int TraverseForwardUntilBlocked<TStageFunction>(ref TStageFunction stageFunction, int blockIndex, ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex,
            int batchEnd, int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            //If no claim is made, this defaults to an invalid interval endpoint.
            int highestLocallyClaimedIndex = -1;
            while (true)
            {
                if (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    highestLocallyClaimedIndex = blockIndex;
                    bounds.Max = blockIndex + 1; //Exclusive bound.
                    Debug.Assert(blockIndex < batchEnd);
                    ref var block = ref context.WorkBlocks[blockIndex];
                    stageFunction.Execute(Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex], block.StartBundle, block.End);
                    //Increment or exit.
                    if (++blockIndex == batchEnd)
                        break;
                }
                else
                {
                    //Already claimed.
                    bounds.Max = blockIndex + 1; //Exclusive bound.
                    break;
                }
            }
            Debug.Assert(bounds.Max <= batchEnd);
            MergeWorkerBounds(ref bounds, ref allWorkerBounds, workerIndex);
            Debug.Assert(bounds.Max <= batchEnd);
            return highestLocallyClaimedIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int TraverseBackwardUntilBlocked<TStageFunction>(ref TStageFunction stageFunction, int blockIndex, ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex,
            int batchStart, int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            //If no claim is made, this defaults to an invalid interval endpoint.
            int lowestLocallyClaimedIndex = context.WorkBlocks.Count;
            while (true)
            {
                if (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    lowestLocallyClaimedIndex = blockIndex;
                    bounds.Min = blockIndex;
                    Debug.Assert(blockIndex >= batchStart);
                    ref var block = ref context.WorkBlocks[blockIndex];
                    stageFunction.Execute(Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex], block.StartBundle, block.End);
                    //Decrement or exit.
                    if (blockIndex == batchStart)
                        break;
                    --blockIndex;
                }
                else
                {
                    //Already claimed.
                    bounds.Min = blockIndex;
                    break;
                }
            }
            MergeWorkerBounds(ref bounds, ref allWorkerBounds, workerIndex);
            return lowestLocallyClaimedIndex;
        }
        interface IStageFunction
        {
            void Execute(TypeProcessor typeBatch, int start, int end);
        }
        struct PrestepStageFunction : IStageFunction
        {
            public float Dt;
            public Bodies Bodies;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(TypeProcessor typeBatch, int start, int end)
            {
                typeBatch.Prestep(Bodies, Dt, 1 / Dt, start, end);
            }
        }

        struct WarmStartStageFunction : IStageFunction
        {
            public Buffer<BodyVelocity> Velocities;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(TypeProcessor typeBatch, int start, int end)
            {
                typeBatch.WarmStart(ref Velocities, start, end);
            }
        }
        struct SolveStageFunction : IStageFunction
        {
            public Buffer<BodyVelocity> Velocities;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(TypeProcessor typeBatch, int start, int end)
            {
                typeBatch.SolveIteration(ref Velocities, start, end);
            }
        }

        /// <summary>
        /// Behaves like a framework SpinWait, but never voluntarily relinquishes the timeslice to off-core threads.
        /// </summary>
        /// <remarks><para>There are three big reasons for using this over the regular framework SpinWait:</para>
        /// <para>1) The framework spinwait relies on spins for quite a while before resorting to any form of timeslice surrender.
        /// Empirically, this is not ideal for the solver- if the sync condition isn't met within several nanoseconds, it will tend to be some microseconds away.
        /// This spinwait is much more aggressive about moving to yields.</para>
        /// <para>2) After a number of yields, the framework SpinWait will resort to calling Sleep.
        /// This widens the potential set of schedulable threads to those not native to the current core. If we permit that transition, it is likely to evict cached solver data.
        /// (For very large simulations, the use of Sleep(0) isn't that concerning- every iteration can be large enough to evict all of cache- 
        /// but there still isn't much benefit to using it over yields in context.)</para>
        /// <para>3) After a particularly long wait, the framework SpinWait resorts to Sleep(1). This is catastrophic for the solver- worse than merely interfering with cached data,
        /// it also simply prevents the thread from being rescheduled for an extremely long period of time (potentially most of a frame!) under the default clock resolution.</para>
        /// <para>Note that this isn't an indication that the framework SpinWait should be changed, but rather that the solver's requirements are extremely specific and don't match
        /// a general purpose solution very well.</para></remarks>
        struct LocalSpinWait
        {
            public int WaitCount;

            //Empirically, being pretty aggressive about yielding produces the best results. This is pretty reasonable- 
            //a single constraint bundle can take hundreds of nanoseconds to finish.
            //That would be a whole lot of spinning that could be used by some other thread. At worst, we're being friendlier to other applications on the system.
            //This thread will likely be rescheduled on the same core, so it's unlikely that we'll lose any cache warmth (that we wouldn't have lost anyway).
            public const int YieldThreshold = 3;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SpinOnce()
            {
                if (WaitCount >= YieldThreshold)
                {
                    Thread.Yield();
                }
                else
                {
                    //We are sacrificing one important feature of the newer framework provided waits- normalized spinning (RuntimeThread.OptimalMaxSpinWaitsPerSpinIteration).
                    //Different platforms can spin at significantly different speeds, so a single constant value for the maximum spin duration doesn't map well to all hardware.
                    //On the upside, we tend to be concerned about two modes- waiting a very short time, and waiting a medium amount of time.
                    //The specific length of the 'short' time doesn't matter too much, so long as it's fairly short.
                    Thread.SpinWait(1 << WaitCount);
                    ++WaitCount;
                }

            }
        }


        void InterstageSync(ref int syncStageIndex)
        {
            //No more work is available to claim, but not every thread is necessarily done with the work they claimed. So we need a dedicated sync- upon completing its local work,
            //a worker increments the 'workerCompleted' counter, and the spins on that counter reaching workerCount * stageIndex.
            ++syncStageIndex;
            var neededCompletionCount = context.WorkerCount * syncStageIndex;
            if (Interlocked.Increment(ref context.WorkerCompletedCount) != neededCompletionCount)
            {
                var spinWait = new LocalSpinWait();
                while (Volatile.Read(ref context.WorkerCompletedCount) < neededCompletionCount)
                {
                    spinWait.SpinOnce();
                }
            }
        }

        private void ExecuteStage<TStageFunction>(ref TStageFunction stageFunction, ref Buffer<WorkerBounds> allWorkerBounds, ref Buffer<WorkerBounds> previousWorkerBounds, int workerIndex,
            int batchStart, int batchEnd, ref int workerStart, ref int syncStage,
            int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            //It is possible for a worker to not have any job available in a particular batch. This can only happen when there are more workers than work blocks in the batch.
            //The workers with indices beyond the available work blocks will have their starts all set to -1 by the scheduler.
            //All previous workers will have tightly packed contiguous indices and won't be able to worksteal at all.
            if (workerStart > -1)
            {
                Debug.Assert(workerStart >= batchStart && workerStart < batchEnd);
                var blockIndex = workerStart;

                ref var bounds = ref allWorkerBounds[workerIndex];

                //Just assume the min will be claimed. There's a chance the thread will get preempted or the value will be read before it's actually claimed, but 
                //that's a very small risk and doesn't affect long-term correctness. (It would just somewhat reduce workstealing effectiveness, and so performance.)
                bounds.Min = blockIndex;
                Debug.Assert(bounds.Max <= batchEnd);

                //Note that initialization guarantees a start index in the batch; no test required.
                //Note that we track the largest contiguous region over the course of the stage execution. The batch start of this worker will be set to the 
                //minimum slot of the largest contiguous region so that following iterations will tend to have a better initial work distribution with less work stealing.
                Debug.Assert(batchStart <= blockIndex && batchEnd > blockIndex);
                var highestLocalClaim = TraverseForwardUntilBlocked(ref stageFunction, blockIndex, ref bounds, ref allWorkerBounds, workerIndex, batchEnd, claimedState, unclaimedState);

                Debug.Assert(bounds.Max <= batchEnd);
                //By now, we've reached the end of the contiguous region in the forward direction. Try walking the other way.
                blockIndex = workerStart - 1;
                //Note that there is no guarantee that the block will be in the batch- this could be the leftmost worker.
                int lowestLocalClaim;
                if (blockIndex >= batchStart)
                {
                    lowestLocalClaim = TraverseBackwardUntilBlocked(ref stageFunction, blockIndex, ref bounds, ref allWorkerBounds, workerIndex, batchStart, claimedState, unclaimedState);
                }
                else
                {
                    lowestLocalClaim = batchStart;
                }
                Debug.Assert(bounds.Max <= batchEnd);
                //These are actually two inclusive bounds, so this is count - 1, but as long as we're consistent it's fine.
                //For this first region, we need to check that it's actually a valid region- if the claims were blocked, it might not be.
                var largestContiguousRegionSize = highestLocalClaim - lowestLocalClaim;
                if (largestContiguousRegionSize >= 0)
                    workerStart = lowestLocalClaim;
                else
                    largestContiguousRegionSize = 0; //It was an invalid region, but later invalid regions should be rejected by size. Setting to zero guarantees that later regions have to have at least one open slot.


                //All contiguous slots have been claimed. Now just traverse to the end along the right direction.
                while (bounds.Max < batchEnd)
                {
                    //Each of these iterations may find a contiguous region larger than our previous attempt.
                    lowestLocalClaim = bounds.Max;
                    highestLocalClaim = TraverseForwardUntilBlocked(ref stageFunction, bounds.Max, ref bounds, ref allWorkerBounds, workerIndex, batchEnd, claimedState, unclaimedState);
                    //If the claim at index lowestLocalClaim was blocked, highestLocalClaim will be -1, so the size will be negative.
                    var regionSize = highestLocalClaim - lowestLocalClaim; //again, actually count - 1
                    if (regionSize > largestContiguousRegionSize)
                    {
                        workerStart = lowestLocalClaim;
                        largestContiguousRegionSize = regionSize;
                    }
                    Debug.Assert(bounds.Max <= batchEnd);
                }

                //Traverse backwards.
                while (bounds.Min > batchStart)
                {
                    //Note bounds.Min - 1; Min is inclusive, so in order to access a new location, it must be pushed out.
                    //Note that the above condition uses a > to handle this.
                    highestLocalClaim = bounds.Min - 1;
                    lowestLocalClaim = TraverseBackwardUntilBlocked(ref stageFunction, highestLocalClaim, ref bounds, ref allWorkerBounds, workerIndex, batchStart, claimedState, unclaimedState);
                    //If the claim at highestLocalClaim was blocked, lowestLocalClaim will be workblocks.Count, so the size will be negative.
                    var regionSize = highestLocalClaim - lowestLocalClaim; //again, actually count - 1
                    if (regionSize > largestContiguousRegionSize)
                    {
                        workerStart = lowestLocalClaim;
                        largestContiguousRegionSize = regionSize;
                    }
                    Debug.Assert(bounds.Max <= batchEnd);
                }

                Debug.Assert(bounds.Min == batchStart && bounds.Max == batchEnd);

            }
            //Clear the previous bounds array before the sync so the next stage has fresh data.
            //Note that this clear is unconditional- the previous worker data must be cleared out or trash data may find its way into the next stage.
            previousWorkerBounds[workerIndex].Min = int.MaxValue;
            previousWorkerBounds[workerIndex].Max = int.MinValue;

            InterstageSync(ref syncStage);
            //Swap the bounds buffers being used before proceeding.
            var tempWorkerBounds = allWorkerBounds;
            allWorkerBounds = previousWorkerBounds;
            previousWorkerBounds = tempWorkerBounds;


        }

        void Work(int workerIndex)
        {
            int prestepStart;
            if (context.WorkBlocks.Count <= context.WorkerCount)
            {
                //Too few blocks to give every worker a job; give the jobs to the first context.WorkBlocks.Count workers.
                prestepStart = workerIndex < context.WorkBlocks.Count ? workerIndex : -1;
            }
            else
            {
                var blocksPerWorker = context.WorkBlocks.Count / context.WorkerCount;
                var remainder = context.WorkBlocks.Count - blocksPerWorker * context.WorkerCount;
                prestepStart = blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
            }
            Buffer<int> batchStarts;
            unsafe
            {
                //stackalloc is actually a little bit slow since the localsinit behavior forces a zeroing.
                //Fortunately, this executes once per thread per frame. With 32 batches, it would add... a few nanoseconds per frame. We can accept that overhead.
                //This is preferred over preallocating on the heap- we might write to these values and we don't want to risk false sharing for no reason. 
                //A single instance of false sharing would cost far more than the overhead of zeroing out the array.
                var data = stackalloc int[Batches.Count];
                batchStarts = new Buffer<int>(data, Batches.Count, Batches.Count);
            }
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                var batchCount = context.BatchBoundaries[batchIndex] - batchStart;

                if (batchCount <= context.WorkerCount)
                {
                    //Too few blocks to give every worker a job; give the jobs to the first context.WorkBlocks.Count workers.
                    batchStarts[batchIndex] = workerIndex < batchCount ? batchStart + workerIndex : -1;
                }
                else
                {
                    var blocksPerWorker = batchCount / context.WorkerCount;
                    var remainder = batchCount - blocksPerWorker * context.WorkerCount;
                    batchStarts[batchIndex] = batchStart + blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
                }
            }

            int syncStage = 0;
            //The claimed and unclaimed state swap after every usage of both pingpong claims buffers.
            int claimedState = 1;
            int unclaimedState = 0;
            var bounds = context.WorkerBoundsA;
            var boundsBackBuffer = context.WorkerBoundsB;
            //Note that every batch has a different start position. Each covers a different subset of constraints, so they require different start locations.
            //The same concept applies to the prestep- the prestep covers all constraints at once, rather than batch by batch.
            var prestepStage = new PrestepStageFunction { Dt = context.Dt, Bodies = bodies };
            Debug.Assert(Batches.Count > 0, "Don't dispatch if there are no constraints.");
            //Technically this could mutate prestep starts, but at the moment we rebuild starts every frame anyway so it doesn't matter oen way or the other.
            ExecuteStage(ref prestepStage, ref bounds, ref boundsBackBuffer, workerIndex, 0, context.WorkBlocks.Count,
                ref prestepStart, ref syncStage, claimedState, unclaimedState);

            claimedState = 0;
            unclaimedState = 1;
            var warmStartStage = new WarmStartStageFunction { Velocities = bodies.Velocities };
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                //Don't use the warm start to guess at the solve iteration work distribution.
                var workerBatchStartCopy = batchStarts[batchIndex];
                ExecuteStage(ref warmStartStage, ref bounds, ref boundsBackBuffer, workerIndex, batchStart, context.BatchBoundaries[batchIndex],
                    ref workerBatchStartCopy, ref syncStage, claimedState, unclaimedState);
            }
            claimedState = 1;
            unclaimedState = 0;

            var solveStage = new SolveStageFunction { Velocities = bodies.Velocities };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                    ExecuteStage(ref solveStage, ref bounds, ref boundsBackBuffer, workerIndex, batchStart, context.BatchBoundaries[batchIndex],
                        ref batchStarts[batchIndex], ref syncStage, claimedState, unclaimedState);
                }
                claimedState ^= 1;
                unclaimedState ^= 1;
            }
        }

        [Conditional("DEBUG")]
        void ValidateWorkBlocks()
        {
            int[][][] batches = new int[Batches.Count][][];
            for (int i = 0; i < Batches.Count; ++i)
            {
                var typeBatches = batches[i] = new int[Batches[i].TypeBatches.Count][];
                for (int j = 0; j < typeBatches.Length; ++j)
                {
                    typeBatches[j] = new int[Batches[i].TypeBatches[j].BundleCount];
                }
            }

            for (int blockIndex = 0; blockIndex < context.WorkBlocks.Count; ++blockIndex)
            {
                ref var block = ref context.WorkBlocks[blockIndex];
                for (int bundleIndex = block.StartBundle; bundleIndex < block.End; ++bundleIndex)
                {
                    ref var visitedCount = ref batches[block.BatchIndex][block.TypeBatchIndex][bundleIndex];
                    ++visitedCount;
                    Debug.Assert(visitedCount == 1);
                }
            }

            for (int batchIndex = 0; batchIndex < batches.Length; ++batchIndex)
            {
                for (int typeBatchIndex = 0; typeBatchIndex < batches[batchIndex].Length; ++typeBatchIndex)
                {
                    for (int constraintIndex = 0; constraintIndex < batches[batchIndex][typeBatchIndex].Length; ++constraintIndex)
                    {
                        Debug.Assert(batches[batchIndex][typeBatchIndex][constraintIndex] == 1);
                    }
                }
            }

        }

        public void MultithreadedUpdate(IThreadDispatcher threadPool, BufferPool bufferPool, float dt)
        {
            var workerCount = context.WorkerCount = threadPool.ThreadCount;
            context.WorkerCompletedCount = 0;
            context.Dt = dt;
            //First build a set of work blocks.
            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
            //These values are found by empirical tuning. The optimal values may vary by architecture.
            //The goal here is to have just enough blocks that, in the event that we end up some underpowered threads (due to competition or hyperthreading), 
            //there are enough blocks that workstealing will still generally allow the extra threads to be useful.
            const int targetBlocksPerBatchPerWorker = 16;
            const int minimumBlockSizeInBundles = 3;

            var targetBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
            BuildWorkBlocks(bufferPool, minimumBlockSizeInBundles, targetBlocksPerBatch);
            ValidateWorkBlocks();

            //Note the clear; the block claims must be initialized to 0 so that the first worker stage knows that the data is available to claim.
            bufferPool.SpecializeFor<int>().Take(context.WorkBlocks.Count, out context.BlockClaims);
            context.BlockClaims.Clear(0, context.WorkBlocks.Count);
            bufferPool.SpecializeFor<WorkerBounds>().Take(workerCount, out context.WorkerBoundsA);
            bufferPool.SpecializeFor<WorkerBounds>().Take(workerCount, out context.WorkerBoundsB);
            //The worker bounds front buffer should be initialized to avoid trash interval data from messing up the workstealing.
            //The worker bounds back buffer will be cleared by the worker before moving on to the next stage.
            for (int i = 0; i < workerCount; ++i)
            {
                context.WorkerBoundsA[i] = new WorkerBounds { Min = int.MaxValue, Max = int.MinValue };
            }

            //While we could be a little more aggressive about culling work with this condition, it doesn't matter much. Have to do it for correctness; worker relies on it.
            if (Batches.Count > 0)
                threadPool.DispatchWorkers(workDelegate);

            context.WorkBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            context.BatchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
            bufferPool.SpecializeFor<int>().Return(ref context.BlockClaims);
            bufferPool.SpecializeFor<WorkerBounds>().Return(ref context.WorkerBoundsA);
            bufferPool.SpecializeFor<WorkerBounds>().Return(ref context.WorkerBoundsB);
        }


    }
}
