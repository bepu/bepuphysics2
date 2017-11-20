using System;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using System.Diagnostics;
using BepuUtilities;
using System.Numerics;

namespace DemoRenderer.Constraints
{
    unsafe interface IConstraintLineExtractor<TPrestep>
    {
        int LinesPerConstraint { get; }

        void ExtractLines(ref TPrestep prestepBundle, int innerIndex, BodyLocation* bodyLocations, Bodies bodies, ref QuickList<LineInstance, Array<LineInstance>> lines);
    }
    abstract class TypeLineExtractor
    {
        public abstract int LinesPerConstraint { get; }
        public abstract void ExtractLines(Bodies bodies, ref TypeBatch typeBatch, int constraintStart, int constraintCount, bool active, ref QuickList<LineInstance, Array<LineInstance>> lines);
    }

    class TypeLineExtractor<T, TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses> : TypeLineExtractor
        where T : struct, IConstraintLineExtractor<TPrestep>
    {
        public override int LinesPerConstraint => default(T).LinesPerConstraint;
        public unsafe override void ExtractLines(Bodies bodies, ref TypeBatch typeBatch, int constraintStart, int constraintCount, bool active,
            ref QuickList<LineInstance, Array<LineInstance>> lines)
        {
            ref var prestepStart = ref Buffer<TPrestep>.Get(ref typeBatch.PrestepData, 0);
            ref var referencesStart = ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, 0);
            //For now, we assume that TBodyReferences is always a series of contiguous Vector<int> values.
            //We can extract each body by abusing the memory layout.
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / Unsafe.SizeOf<Vector<int>>();
            Debug.Assert(bodyCount * Unsafe.SizeOf<Vector<int>>() == Unsafe.SizeOf<TBodyReferences>());
            var bodyLocations = stackalloc BodyLocation[bodyCount];
            var extractor = default(T);

            var constraintEnd = constraintStart + constraintCount;
            if (active)
            {
                for (int i = constraintStart; i < constraintEnd; ++i)
                {
                    BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                    ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                    ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                    ref var firstReference = ref Unsafe.As<TBodyReferences, Vector<int>>(ref referencesBundle);
                    for (int j = 0; j < bodyCount; ++j)
                    {
                        ref var location = ref bodyLocations[j];
                        location.SetIndex = 0;
                        location.Index = GatherScatter.Get(ref Unsafe.Add(ref firstReference, j), innerIndex);
                    }
                    extractor.ExtractLines(ref prestepBundle, innerIndex, bodyLocations, bodies, ref lines);
                }
            }
            else
            {
                for (int i = constraintStart; i < constraintEnd; ++i)
                {
                    BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                    ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                    ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                    ref var firstReference = ref Unsafe.As<TBodyReferences, Vector<int>>(ref referencesBundle);
                    for (int j = 0; j < bodyCount; ++j)
                    {
                        //Inactive constraints store body references in the form of handles, so we have to follow the indirection.
                        ref var location = ref bodyLocations[j];
                        bodyLocations[j] = bodies.HandleToLocation[GatherScatter.Get(ref Unsafe.Add(ref firstReference, j), innerIndex)];
                    }
                    extractor.ExtractLines(ref prestepBundle, innerIndex, bodyLocations, bodies, ref lines);
                }
            }
        }
    }

    internal class ConstraintLineExtractor
    {
        TypeLineExtractor[] lineExtractors;
        const int jobsPerThread = 4;
        QuickList<ThreadJob, Array<ThreadJob>> jobs;

        struct ThreadJob
        {
            public int SetIndex;
            public int BatchIndex;
            public int TypeBatchIndex;
            public int ConstraintStart;
            public int ConstraintCount;
            public int LineStart;
            public int LinesPerConstraint;
            public QuickList<LineInstance, Array<LineInstance>> jobLines;
        }

        public bool Enabled { get; set; } = true;

        Action<int> executeJobDelegate;

        ref TypeLineExtractor AllocateSlot(int typeId)
        {
            if (typeId >= lineExtractors.Length)
                Array.Resize(ref lineExtractors, typeId + 1);
            return ref lineExtractors[typeId];
        }
        public ConstraintLineExtractor()
        {
            lineExtractors = new TypeLineExtractor[32];
            AllocateSlot(BallSocketTypeBatch.BatchTypeId) =
                new TypeLineExtractor<BallSocketLineExtractor, TwoBodyReferences, BallSocketPrestepData, BallSocketProjection, Vector3Wide>();
            QuickList<ThreadJob, Array<ThreadJob>>.Create(new PassthroughArrayPool<ThreadJob>(), Environment.ProcessorCount * (jobsPerThread + 1), out jobs);

            executeJobDelegate = ExecuteJob;
        }

        Bodies bodies;
        Solver solver;
        private void ExecuteJob(int jobIndex)
        {
            ref var job = ref jobs[jobIndex];
            ref var typeBatch = ref solver.Sets[job.SetIndex].Batches[job.BatchIndex].TypeBatches[job.TypeBatchIndex];
            Debug.Assert(lineExtractors[typeBatch.TypeId] != null, "Jobs should only be created for types which are registered and used.");
            lineExtractors[typeBatch.TypeId].ExtractLines(bodies, ref typeBatch, job.ConstraintStart, job.ConstraintCount, true, ref job.jobLines);
        }

        bool IsContactBatch(int typeId)
        {
            //TODO: If the nonconvex contact count expands to 8, this will have to change.
            return typeId < 16;
        }

        internal void AddInstances(Bodies bodies, Solver solver, bool showConstraints, bool showContacts, ref QuickList<LineInstance, Array<LineInstance>> lines, ParallelLooper looper)
        {
            int neededLineCapacity = lines.Count;
            jobs.Count = 0;
            var jobPool = new PassthroughArrayPool<ThreadJob>();
            for (int setIndex = 0; setIndex < solver.Sets.Length; ++setIndex)
            {
                ref var set = ref solver.Sets[setIndex];
                if (set.Allocated)
                {
                    for (int batchIndex = 0; batchIndex < set.Batches.Count; ++batchIndex)
                    {
                        ref var batch = ref set.Batches[batchIndex];
                        for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                        {
                            ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                            var extractor = lineExtractors[typeBatch.TypeId];
                            var isContactBatch = IsContactBatch(typeBatch.TypeId);
                            if (extractor != null && ((isContactBatch && showContacts) || (!isContactBatch && showConstraints)))
                            {
                                jobs.Add(new ThreadJob
                                {
                                    BatchIndex = batchIndex,
                                    TypeBatchIndex = typeBatchIndex,
                                    ConstraintStart = 0,
                                    ConstraintCount = typeBatch.ConstraintCount,
                                    LineStart = neededLineCapacity,
                                    LinesPerConstraint = extractor.LinesPerConstraint
                                }, jobPool);
                                neededLineCapacity += extractor.LinesPerConstraint * typeBatch.ConstraintCount;
                            }
                        }
                    }
                }
            }
            var maximumJobSize = Math.Max(1, neededLineCapacity / (jobsPerThread * Environment.ProcessorCount));
            var originalJobCount = jobs.Count;
            //Split jobs if they're larger than desired to help load balancing a little bit. This isn't terribly important, but it's pretty easy.
            for (int i = 0; i < originalJobCount; ++i)
            {
                ref var job = ref jobs[i];
                if (job.ConstraintCount > maximumJobSize)
                {
                    var subjobCount = (int)Math.Round(0.5 + job.ConstraintCount / (double)maximumJobSize);
                    var constraintsPerSubjob = job.ConstraintCount / subjobCount;
                    var remainder = job.ConstraintCount - constraintsPerSubjob * subjobCount;
                    //Modify the first job in place.
                    job.ConstraintCount = constraintsPerSubjob;
                    if (remainder > 0)
                        ++job.ConstraintCount;
                    //Append the remaining jobs.
                    var previousJob = job;
                    for (int j = 1; j < subjobCount; ++j)
                    {
                        var newJob = previousJob;
                        newJob.LineStart += previousJob.ConstraintCount * newJob.LinesPerConstraint;
                        newJob.ConstraintStart += previousJob.ConstraintCount;
                        newJob.ConstraintCount = constraintsPerSubjob;
                        if (remainder > j)
                            ++newJob.ConstraintCount;
                        jobs.Add(newJob, jobPool);
                        previousJob = newJob;
                    }
                }
            }
            lines.EnsureCapacity(neededLineCapacity, new PassthroughArrayPool<LineInstance>());
            lines.Count = neededLineCapacity; //Line additions will be performed on suballocated lists. This count will be used by the renderer when reading line data.
            for (int i = 0; i < jobs.Count; ++i)
            {
                //Creating a local copy of the list reference and count allows additions to proceed in parallel. 
                jobs[i].jobLines = new QuickList<LineInstance, Array<LineInstance>>(ref lines.Span);
                //By setting the count, we work around the fact that Array<T> doesn't support slicing.
                jobs[i].jobLines.Count = jobs[i].LineStart;
            }
            this.bodies = bodies;
            this.solver = solver;
            looper.For(0, jobs.Count, executeJobDelegate);
            this.bodies = null;
            this.solver = solver;
        }

    }
}
