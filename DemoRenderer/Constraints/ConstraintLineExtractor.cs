using System;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using System.Diagnostics;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    interface IConstraintLineExtractor<TBodyReferences, TPrestep>
    {
        int LinesPerConstraint { get; }

        void ExtractLines(ref TPrestep prestepBundle, ref TBodyReferences referencesBundle, int innerIndex, Bodies bodies, ref QuickList<LineInstance, Array<LineInstance>> lines);
    }
    abstract class TypeLineExtractor
    {
        public abstract int LinesPerConstraint { get; }
        public abstract void ExtractLines(Bodies bodies, TypeBatch typeBatch, int constraintStart, int constraintCount, ref QuickList<LineInstance, Array<LineInstance>> lines);
    }

    class TypeLineExtractor<T, TTypeBatch, TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses> : TypeLineExtractor
        where T : struct, IConstraintLineExtractor<TBodyReferences, TPrestep>
        where TTypeBatch : TypeBatch<TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses>
    {
        public override int LinesPerConstraint => default(T).LinesPerConstraint;
        public override void ExtractLines(Bodies bodies, TypeBatch typeBatch, int constraintStart, int constraintCount,
            ref QuickList<LineInstance, Array<LineInstance>> lines)
        {
            var batch = (TTypeBatch)typeBatch;
            ref var prestepStart = ref batch.PrestepData[0];
            ref var referencesStart = ref batch.BodyReferences[0];
            var extractor = default(T);

            var constraintEnd = constraintStart + constraintCount;
            for (int i = constraintStart; i < constraintEnd; ++i)
            {
                BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                extractor.ExtractLines(ref prestepBundle, ref referencesBundle, innerIndex, bodies, ref lines);
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
                new TypeLineExtractor<BallSocketLineExtractor, BallSocketTypeBatch, TwoBodyReferences, BallSocketPrestepData, BallSocketProjection, Vector3Wide>();
            QuickList<ThreadJob, Array<ThreadJob>>.Create(new PassthroughArrayPool<ThreadJob>(), Environment.ProcessorCount * (jobsPerThread + 1), out jobs);

            executeJobDelegate = ExecuteJob;
        }

        Bodies bodies;
        Solver solver;
        private void ExecuteJob(int jobIndex)
        {
            ref var job = ref jobs[jobIndex];
            var typeBatch = solver.Batches[job.BatchIndex].TypeBatches[job.TypeBatchIndex];
            Debug.Assert(lineExtractors[typeBatch.TypeId] != null, "Jobs should only be created for types which are available and active.");
            lineExtractors[typeBatch.TypeId].ExtractLines(bodies, typeBatch, job.ConstraintStart, job.ConstraintCount, ref job.jobLines);
        }

        bool IsContactBatch(TypeBatch typeBatch)
        {
            //TODO: If the nonconvex contact count expands to 8, this will have to change.
            return typeBatch.TypeId < 16;
        }

        internal void AddInstances(Bodies bodies, Solver solver, bool showConstraints, bool showContacts, ref QuickList<LineInstance, Array<LineInstance>> lines, ParallelLooper looper)
        {
            int neededLineCapacity = lines.Count;
            jobs.Count = 0;
            var jobPool = new PassthroughArrayPool<ThreadJob>();
            for (int batchIndex = 0; batchIndex < solver.Batches.Count; ++batchIndex)
            {
                var batch = solver.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    var extractor = lineExtractors[typeBatch.TypeId];
                    var isContactBatch = IsContactBatch(typeBatch);
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
