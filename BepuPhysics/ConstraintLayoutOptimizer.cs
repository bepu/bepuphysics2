using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics
{
    public class ConstraintLayoutOptimizer
    {
        Bodies bodies;
        Solver solver;
        struct Optimization
        {
            /// <summary>
            /// Index of the target constraint bundle to optimize.
            /// </summary>
            public int BundleIndex;
            /// <summary>
            /// Index of the last optimized type batch.
            /// </summary>
            public int TypeBatchIndex;
            /// <summary>
            /// Index of the last optimized batch.
            /// </summary>
            public int BatchIndex;

        }

        Optimization nextTarget;

        /// <summary>
        /// If true, regions are offset by a half region width. Toggled each frame. Offsets allow the sorted regions to intermix, eventually converging to a full sort.
        /// </summary>
        bool shouldOffset;

        float optimizationFraction;
        public float OptimizationFraction
        {
            get { return optimizationFraction; }
            set
            {
                if (value < 0 || value > 1)
                    throw new ArgumentException("Optimization fraction must be from 0 to 1.");
                optimizationFraction = value;
            }
        }

        Action<int> generateSortKeysDelegate;
        Action<int> regatherDelegate;
        Action<int> copyToCacheAndSortDelegate;

        public ConstraintLayoutOptimizer(Bodies bodies, Solver solver, float optimizationFraction = 0.044f)
        {
            this.bodies = bodies;
            this.solver = solver;
            OptimizationFraction = optimizationFraction;

            generateSortKeysDelegate = GenerateSortKeys;
            regatherDelegate = Regather;
            copyToCacheAndSortDelegate = CopyToCacheAndSort;
        }

        void Wrap(ref Optimization o)
        {
            ref var activeSet = ref solver.ActiveSet;
            Debug.Assert(activeSet.Batches.Count > 0, "Shouldn't be trying to optimize zero constraints.");
            while (true)
            {
                if (o.BatchIndex >= activeSet.Batches.Count)
                {
                    o = new Optimization();
                }
                else if (o.TypeBatchIndex >= activeSet.Batches[o.BatchIndex].TypeBatches.Count)
                {
                    //It's possible that batches prior to the last constraint batch lack constraints. In that case, try the next one.
                    ++o.BatchIndex;
                    o.TypeBatchIndex = 0;
                    o.BundleIndex = 0;
                }
                else if (o.BundleIndex >= activeSet.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount)
                {
                    ++o.TypeBatchIndex;
                    o.BundleIndex = 0;
                }
                else
                {
                    break;
                }
            }
        }

        Optimization FindOffsetFrameStart(Optimization o, int maximumRegionSizeInBundles)
        {
            Wrap(ref o);

            ref var activeSet = ref solver.ActiveSet;
            var spaceRemaining = activeSet.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount - o.BundleIndex;
            if (spaceRemaining <= maximumRegionSizeInBundles)
            {
                ++o.TypeBatchIndex;
                Wrap(ref o);
            }
            //Note that the bundle count is not cached; the above type batch may differ.
            o.BundleIndex = Math.Max(0,
                Math.Min(
                    o.BundleIndex + maximumRegionSizeInBundles / 2,
                    activeSet.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount - maximumRegionSizeInBundles));

            return o;
        }

        public void Update(BufferPool bufferPool, IThreadDispatcher threadDispatcher = null)
        {
            //TODO: It's possible that the cost associated with setting up multithreading exceeds the cost of the actual optimization for smaller simulations.
            //You might want to fall back to single threaded based on some empirical testing.
            //No point in optimizing if there are no constraints- this is a necessary test since we assume that 0 is a valid batch index later.
            ref var activeSet = ref solver.ActiveSet;
            if (activeSet.Batches.Count == 0)
                return;
            var regionSizeInBundles = (int)Math.Max(2, Math.Round(activeSet.BundleCount * optimizationFraction));
            //The region size in bundles should be divisible by two so that it can be offset by half.
            if ((regionSizeInBundles & 1) == 1)
                ++regionSizeInBundles;
            //Note that we require that all regions are bundle aligned. This is important for the typebatch sorting process, which tends to use bulk copies from bundle arrays to cache.
            //If not bundle aligned, those bulk copies would become complex due to the constraint AOSOA layout.

            Optimization target;
            if (shouldOffset)
            {
                //Use the previous frame's start to create the new target.
                target = FindOffsetFrameStart(nextTarget, regionSizeInBundles);
                Debug.Assert(activeSet.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex].BundleCount <= regionSizeInBundles || target.BundleIndex != 0,
                    "On offset frames, the only time a target bundle can be 0 is if the batch is too small for it to be anything else.");
                //Console.WriteLine($"Offset frame targeting {target.BatchIndex}.{target.TypeBatchIndex}:{target.BundleIndex}");
                Debug.Assert(activeSet.Batches[target.BatchIndex].TypeBatches.Count > target.TypeBatchIndex);
            }
            else
            {
                //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
                target = nextTarget;
                Wrap(ref target);
                Debug.Assert(activeSet.Batches[target.BatchIndex].TypeBatches.Count > target.TypeBatchIndex);
                nextTarget = target;
                nextTarget.BundleIndex += regionSizeInBundles;
                //Console.WriteLine($"Normal frame targeting {target.BatchIndex}.{target.TypeBatchIndex}:{target.BundleIndex}");
            }
            //Note that we have two separate parallel optimizations over multiple frames. Alternating between them on a per frame basis is a fairly simple way to guarantee
            //eventual convergence in the sort. We only ever push forward the non-offset version; the offset position is based on the nonoffset version's last start position.
            shouldOffset = !shouldOffset;


            var maximumRegionSizeInConstraints = regionSizeInBundles * Vector<int>.Count;

            ref var typeBatch = ref activeSet.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex];
            SortByBodyLocation(ref typeBatch, target.BundleIndex, Math.Min(typeBatch.ConstraintCount - target.BundleIndex * Vector<int>.Count, maximumRegionSizeInConstraints),
                solver.HandleToConstraint, bodies.ActiveSet.Count, bufferPool, threadDispatcher);

        }

        //TODO: Generic pointers are blocked in pre-blittable C# versions. We can update this later to avoid unsafe casts.
        unsafe void* typeBatchPointer;
        IThreadDispatcher threadDispatcher;
        Buffer<ConstraintLocation> handlesToConstraints;
        struct MultithreadingContext
        {
            public Buffer<int> SortKeys;
            public Buffer<int> SourceIndices;
            public RawBuffer BodyReferencesCache;
            public int SourceStartBundleIndex;
            public int BundlesPerWorker;
            public int BundlesPerWorkerRemainder;
            public int TypeBatchConstraintCount;

            public Buffer<int> SortedKeys; //This is only really stored for debug use.
            public Buffer<int> SortedSourceIndices;
            public Buffer<int> ScratchKeys;
            public Buffer<int> ScratchValues;
            public Buffer<int> IndexToHandleCache;
            public RawBuffer PrestepDataCache;
            public RawBuffer AccumulatesImpulsesCache;
            public int KeyUpperBound;
            public int ConstraintsInSortRegionCount;
            //Note that these differ from phase 1- one of the threads in the sort is dedicated to a sort. These regard the remaining threads.
            public int CopyBundlesPerWorker;
            public int CopyBundlesPerWorkerRemainder;
        }
        MultithreadingContext context;

        unsafe void GenerateSortKeys(int workerIndex)
        {
            var localWorkerBundleStart = context.BundlesPerWorker * workerIndex + Math.Min(workerIndex, context.BundlesPerWorkerRemainder);
            var workerBundleStart = context.SourceStartBundleIndex + localWorkerBundleStart;
            var workerBundleCount = workerIndex < context.BundlesPerWorkerRemainder ? context.BundlesPerWorker + 1 : context.BundlesPerWorker;
            var workerConstraintStart = workerBundleStart << BundleIndexing.VectorShift;
            //Note that the number of constraints we can iterate over is clamped by the type batch's constraint count. The last bundle may not be full.
            var workerConstraintCount = Math.Min(context.TypeBatchConstraintCount - workerConstraintStart, workerBundleCount << BundleIndexing.VectorShift);
            if (workerConstraintCount <= 0)
                return; //No work remains.
            var localWorkerConstraintStart = localWorkerBundleStart << BundleIndexing.VectorShift;

            ref var typeBatch = ref Unsafe.AsRef<TypeBatch>(typeBatchPointer);
            solver.TypeProcessors[typeBatch.TypeId].GenerateSortKeysAndCopyReferences(ref typeBatch,
                workerBundleStart, localWorkerBundleStart, workerBundleCount,
                workerConstraintStart, localWorkerConstraintStart, workerConstraintCount,
                ref context.SortKeys[localWorkerConstraintStart], ref context.SourceIndices[localWorkerConstraintStart], ref context.BodyReferencesCache);
        }

        unsafe void CopyToCacheAndSort(int workerIndex)
        {
            //Sorting only requires that the sort keys and indices be ready. Caching doesn't need to be done yet. 
            //Given that the sort is already very fast and trying to independently multithread it is a bad idea, we'll just bundle it alongside
            //the remaining cache copies. This phase is extremely memory bound and the sort likely won't match the copy duration, but there is no
            //room for complicated schemes at these timescales (<150us). We just try to get as much benefit as we can with a few simple tricks.
            //Most likely we won't get more than about 2.5x speedup on a computer with bandwidth/compute ratios similar to a 3770K with 1600mhz memory.
            if (workerIndex == threadDispatcher.ThreadCount - 1)
            {
                //TODO: If this ends up being the only place where you actually make use of the thread memory pools, you might as well get rid of it
                //in favor of just preallocating workerCount buffers of 1024 ints each. Its original use of creating the typebatch-specific memory no longer exists.
                LSBRadixSort.Sort(
                    ref context.SortKeys, ref context.SourceIndices,
                    ref context.ScratchKeys, ref context.ScratchValues, 0, context.ConstraintsInSortRegionCount,
                    context.KeyUpperBound, threadDispatcher.GetThreadMemoryPool(workerIndex),
                    out context.SortedKeys, out context.SortedSourceIndices);
            }
            //Note that worker 0 still copies if there's only one thread in the pool. Mainly for debugging purposes.
            if (threadDispatcher.ThreadCount == 1 || workerIndex < threadDispatcher.ThreadCount - 1)
            {
                var localWorkerBundleStart = context.CopyBundlesPerWorker * workerIndex + Math.Min(workerIndex, context.CopyBundlesPerWorkerRemainder);
                var workerBundleStart = context.SourceStartBundleIndex + localWorkerBundleStart;
                var workerBundleCount = workerIndex < context.CopyBundlesPerWorkerRemainder ? context.CopyBundlesPerWorker + 1 : context.CopyBundlesPerWorker;
                var workerConstraintStart = workerBundleStart << BundleIndexing.VectorShift;
                //Note that the number of constraints we can iterate over is clamped by the type batch's constraint count. The last bundle may not be full.
                var workerConstraintCount = Math.Min(context.TypeBatchConstraintCount - workerConstraintStart, workerBundleCount << BundleIndexing.VectorShift);
                if (workerConstraintCount <= 0)
                    return; //No work remains.
                var localWorkerConstraintStart = localWorkerBundleStart << BundleIndexing.VectorShift;

                ref var typeBatch = ref Unsafe.AsRef<TypeBatch>(typeBatchPointer);
                solver.TypeProcessors[typeBatch.TypeId].CopyToCache(ref typeBatch,
                    workerBundleStart, localWorkerBundleStart, workerBundleCount,
                    workerConstraintStart, localWorkerConstraintStart, workerConstraintCount,
                    ref context.IndexToHandleCache, ref context.PrestepDataCache, ref context.AccumulatesImpulsesCache);
            }
        }

        unsafe void CopyToCacheAndSort(BufferPool pool)
        {
            LSBRadixSort.Sort(
                ref context.SortKeys, ref context.SourceIndices,
                ref context.ScratchKeys, ref context.ScratchValues, 0, context.ConstraintsInSortRegionCount,
                context.KeyUpperBound, pool,
                out context.SortedKeys, out context.SortedSourceIndices);

            var workerBundleStart = context.SourceStartBundleIndex;
            var workerBundleCount = 0 < context.CopyBundlesPerWorkerRemainder ? context.CopyBundlesPerWorker + 1 : context.CopyBundlesPerWorker;
            var workerConstraintStart = workerBundleStart << BundleIndexing.VectorShift;
            //Note that the number of constraints we can iterate over is clamped by the type batch's constraint count. The last bundle may not be full.
            var workerConstraintCount = Math.Min(context.TypeBatchConstraintCount - workerConstraintStart, workerBundleCount << BundleIndexing.VectorShift);
            if (workerConstraintCount <= 0)
                return; //No work remains.

            ref var typeBatch = ref Unsafe.AsRef<TypeBatch>(typeBatchPointer);
            solver.TypeProcessors[typeBatch.TypeId].CopyToCache(ref typeBatch,
                workerBundleStart, 0, workerBundleCount,
                workerConstraintStart, 0, workerConstraintCount,
                ref context.IndexToHandleCache, ref context.PrestepDataCache, ref context.AccumulatesImpulsesCache);
        }

        unsafe void Regather(int workerIndex)
        {
            var localWorkerBundleStart = context.BundlesPerWorker * workerIndex + Math.Min(workerIndex, context.BundlesPerWorkerRemainder);
            var workerBundleStart = context.SourceStartBundleIndex + localWorkerBundleStart;
            var workerBundleCount = workerIndex < context.BundlesPerWorkerRemainder ? context.BundlesPerWorker + 1 : context.BundlesPerWorker;
            var workerConstraintStart = workerBundleStart << BundleIndexing.VectorShift;
            //Note that the number of constraints we can iterate over is clamped by the type batch's constraint count. The last bundle may not be full.
            var workerConstraintCount = Math.Min(context.TypeBatchConstraintCount - workerConstraintStart, workerBundleCount << BundleIndexing.VectorShift);
            if (workerConstraintCount <= 0)
                return; //No work remains.
            var localWorkerConstraintStart = localWorkerBundleStart << BundleIndexing.VectorShift;
            ref var firstSourceIndex = ref context.SortedSourceIndices[localWorkerConstraintStart];

            ref var typeBatch = ref Unsafe.AsRef<TypeBatch>(typeBatchPointer);
            solver.TypeProcessors[typeBatch.TypeId].Regather(ref typeBatch, workerConstraintStart, workerConstraintCount, ref firstSourceIndex,
                ref context.IndexToHandleCache, ref context.BodyReferencesCache, ref context.PrestepDataCache, ref context.AccumulatesImpulsesCache, ref handlesToConstraints);

        }



        unsafe void SortByBodyLocation(ref TypeBatch typeBatch, int bundleStartIndex, int constraintCount, Buffer<ConstraintLocation> handlesToConstraints, int bodyCount,
            BufferPool pool, IThreadDispatcher threadDispatcher)
        {
            int bundleCount = (constraintCount >> BundleIndexing.VectorShift);
            if ((constraintCount & BundleIndexing.VectorMask) != 0)
                ++bundleCount;
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;

            pool.TakeAtLeast(constraintCount, out context.SourceIndices);
            pool.TakeAtLeast(constraintCount, out context.SortKeys);
            pool.TakeAtLeast(constraintCount, out context.ScratchKeys);
            pool.TakeAtLeast(constraintCount, out context.ScratchValues);
            pool.TakeAtLeast(constraintCount, out context.IndexToHandleCache);

            var typeProcessor = solver.TypeProcessors[typeBatch.TypeId];
            typeProcessor.GetBundleTypeSizes(out var bodyReferencesBundleSize, out var prestepBundleSize, out var accumulatedImpulseBundleSize);

            //The typebatch invoked by the worker will cast the body references to the appropriate type. 
            //Using typeless buffers makes it easy to cache the buffers here in the constraint optimizer rather than in the individual type batches.
            pool.TakeAtLeast(bundleCount * bodyReferencesBundleSize, out context.BodyReferencesCache);
            pool.TakeAtLeast(bundleCount * prestepBundleSize, out context.PrestepDataCache);
            pool.TakeAtLeast(bundleCount * accumulatedImpulseBundleSize, out context.AccumulatesImpulsesCache);

            context.BundlesPerWorker = bundleCount / threadCount;
            context.BundlesPerWorkerRemainder = bundleCount - context.BundlesPerWorker * threadCount;
            context.TypeBatchConstraintCount = typeBatch.ConstraintCount;
            context.SourceStartBundleIndex = bundleStartIndex;

            //The second phase uses one worker to sort.
            if (threadCount > 1)
            {
                context.CopyBundlesPerWorker = bundleCount / (threadCount - 1);
                context.CopyBundlesPerWorkerRemainder = bundleCount - context.CopyBundlesPerWorker * (threadCount - 1);
            }
            else
            {
                //If there's only one worker (as is the case when this is running single-threaded), the worker will have to do the sort AND the copy.
                context.CopyBundlesPerWorker = bundleCount;
                context.CopyBundlesPerWorkerRemainder = 0;
            }
            context.ConstraintsInSortRegionCount = constraintCount;
            context.KeyUpperBound = bodyCount - 1;

            this.typeBatchPointer = Unsafe.AsPointer(ref typeBatch);
            this.threadDispatcher = threadDispatcher;
            this.handlesToConstraints = handlesToConstraints;

            if (threadDispatcher == null)
            {
                GenerateSortKeys(0);
                CopyToCacheAndSort(pool);
                Regather(0);
            }
            else
            {
                threadDispatcher.DispatchWorkers(generateSortKeysDelegate);
                threadDispatcher.DispatchWorkers(copyToCacheAndSortDelegate);
                threadDispatcher.DispatchWorkers(regatherDelegate);
            }

            this.typeBatchPointer = null;
            this.threadDispatcher = null;
            this.handlesToConstraints = new Buffer<ConstraintLocation>();

            //This is a pure debug function.
            solver.TypeProcessors[typeBatch.TypeId].VerifySortRegion(ref typeBatch, bundleStartIndex, constraintCount, ref context.SortedKeys, ref context.SortedSourceIndices);

            pool.Return(ref context.SourceIndices);
            pool.Return(ref context.SortKeys);
            pool.Return(ref context.ScratchKeys);
            pool.Return(ref context.ScratchValues);
            pool.Return(ref context.IndexToHandleCache);
            pool.Return(ref context.BodyReferencesCache);
            pool.Return(ref context.PrestepDataCache);
            pool.Return(ref context.AccumulatesImpulsesCache);
        }
    }
}
