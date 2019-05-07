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
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints.Contact;

namespace DemoRenderer.Constraints
{
    unsafe interface IConstraintLineExtractor<TPrestep>
    {
        int LinesPerConstraint { get; }

        void ExtractLines(ref TPrestep prestepBundle, int setIndex, int* bodyLocations, Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines);
    }
    abstract class TypeLineExtractor
    {
        public abstract int LinesPerConstraint { get; }
        public abstract void ExtractLines(Bodies bodies, int setIndex, ref TypeBatch typeBatch, int constraintStart, int constraintCount, ref QuickList<LineInstance> lines);
    }

    class TypeLineExtractor<T, TBodyReferences, TPrestep, TProjection, TAccumulatedImpulses> : TypeLineExtractor
        where TBodyReferences : struct
        where TPrestep : struct
        where T : struct, IConstraintLineExtractor<TPrestep>
    {
        public override int LinesPerConstraint => default(T).LinesPerConstraint;
        public unsafe override void ExtractLines(Bodies bodies, int setIndex, ref TypeBatch typeBatch, int constraintStart, int constraintCount,
            ref QuickList<LineInstance> lines)
        {
            ref var prestepStart = ref Buffer<TPrestep>.Get(ref typeBatch.PrestepData, 0);
            ref var referencesStart = ref Buffer<TBodyReferences>.Get(ref typeBatch.BodyReferences, 0);
            //For now, we assume that TBodyReferences is always a series of contiguous Vector<int> values.
            //We can extract each body by abusing the memory layout.
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / Unsafe.SizeOf<Vector<int>>();
            Debug.Assert(bodyCount * Unsafe.SizeOf<Vector<int>>() == Unsafe.SizeOf<TBodyReferences>());
            var bodyIndices = stackalloc int[bodyCount];
            var extractor = default(T);

            var constraintEnd = constraintStart + constraintCount;
            if (setIndex == 0)
            {
                var tint = new Vector3(1, 1, 1);
                for (int i = constraintStart; i < constraintEnd; ++i)
                {
                    BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                    ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                    ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                    ref var firstReference = ref Unsafe.As<TBodyReferences, Vector<int>>(ref referencesBundle);
                    for (int j = 0; j < bodyCount; ++j)
                    {
                        //Active set constraint body references refer directly to the body index.
                        bodyIndices[j] = GatherScatter.Get(ref Unsafe.Add(ref firstReference, j), innerIndex);
                    }
                    extractor.ExtractLines(ref GatherScatter.GetOffsetInstance(ref prestepBundle, innerIndex), setIndex, bodyIndices, bodies, ref tint, ref lines);
                }
            }
            else
            {
                var tint = new Vector3(0.4f, 0.4f, 0.8f);
                for (int i = constraintStart; i < constraintEnd; ++i)
                {
                    BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                    ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                    ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                    ref var firstReference = ref Unsafe.As<TBodyReferences, Vector<int>>(ref referencesBundle);
                    for (int j = 0; j < bodyCount; ++j)
                    {
                        //Inactive constraints store body references in the form of handles, so we have to follow the indirection.
                        var bodyHandle = GatherScatter.Get(ref Unsafe.Add(ref firstReference, j), innerIndex);
                        Debug.Assert(bodies.HandleToLocation[bodyHandle].SetIndex == setIndex);
                        bodyIndices[j] = bodies.HandleToLocation[bodyHandle].Index;
                    }
                    extractor.ExtractLines(ref GatherScatter.GetOffsetInstance(ref prestepBundle, innerIndex), setIndex, bodyIndices, bodies, ref tint, ref lines);
                }
            }
        }
    }

    internal class ConstraintLineExtractor : IDisposable
    {
        TypeLineExtractor[] lineExtractors;
        const int jobsPerThread = 4;
        QuickList<ThreadJob> jobs;
        BufferPool pool;

        struct ThreadJob
        {
            public int SetIndex;
            public int BatchIndex;
            public int TypeBatchIndex;
            public int ConstraintStart;
            public int ConstraintCount;
            public int LineStart;
            public int LinesPerConstraint;
            public QuickList<LineInstance> jobLines;
        }

        public bool Enabled { get; set; } = true;

        Action<int> executeJobDelegate;

        ref TypeLineExtractor AllocateSlot(int typeId)
        {
            if (typeId >= lineExtractors.Length)
                Array.Resize(ref lineExtractors, typeId + 1);
            return ref lineExtractors[typeId];
        }
        public ConstraintLineExtractor(BufferPool pool)
        {
            this.pool = pool;
            lineExtractors = new TypeLineExtractor[32];
            AllocateSlot(BallSocketTypeProcessor.BatchTypeId) = new TypeLineExtractor<BallSocketLineExtractor, TwoBodyReferences, BallSocketPrestepData, BallSocketProjection, Vector3Wide>();
            AllocateSlot(WeldTypeProcessor.BatchTypeId) = new TypeLineExtractor<WeldLineExtractor, TwoBodyReferences, WeldPrestepData, WeldProjection, WeldAccumulatedImpulses>();
            AllocateSlot(DistanceServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<DistanceServoLineExtractor, TwoBodyReferences, DistanceServoPrestepData, DistanceServoProjection, Vector<float>>();
            AllocateSlot(DistanceLimitTypeProcessor.BatchTypeId) = new TypeLineExtractor<DistanceLimitLineExtractor, TwoBodyReferences, DistanceLimitPrestepData, DistanceLimitProjection, Vector<float>>();
            AllocateSlot(CenterDistanceTypeProcessor.BatchTypeId) = new TypeLineExtractor<CenterDistanceLineExtractor, TwoBodyReferences, CenterDistancePrestepData, CenterDistanceProjection, Vector<float>>();
            AllocateSlot(PointOnLineServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<PointOnLineLineExtractor, TwoBodyReferences, PointOnLineServoPrestepData, PointOnLineServoProjection, Vector2Wide>();
            AllocateSlot(LinearAxisServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<LinearAxisServoLineExtractor, TwoBodyReferences, LinearAxisServoPrestepData, LinearAxisServoProjection, Vector<float>>();
            AllocateSlot(AngularSwivelHingeTypeProcessor.BatchTypeId) = new TypeLineExtractor<AngularSwivelHingeLineExtractor, TwoBodyReferences, AngularSwivelHingePrestepData, AngularSwivelHingeProjection, Vector<float>>();
            AllocateSlot(SwivelHingeTypeProcessor.BatchTypeId) = new TypeLineExtractor<SwivelHingeLineExtractor, TwoBodyReferences, SwivelHingePrestepData, SwivelHingeProjection, Vector4Wide>();
            AllocateSlot(HingeTypeProcessor.BatchTypeId) = new TypeLineExtractor<HingeLineExtractor, TwoBodyReferences, HingePrestepData, HingeProjection, HingeAccumulatedImpulses>();
            AllocateSlot(OneBodyLinearServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<OneBodyLinearServoLineExtractor, Vector<int>, OneBodyLinearServoPrestepData, OneBodyLinearServoProjection, Vector<float>>();

            AllocateSlot(Contact1OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact1OneBodyLineExtractor, Vector<int>, Contact1OneBodyPrestepData, Contact1OneBodyProjection, Contact1AccumulatedImpulses>();
            AllocateSlot(Contact2OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2OneBodyLineExtractor, Vector<int>, Contact2OneBodyPrestepData, Contact2OneBodyProjection, Contact2AccumulatedImpulses>();
            AllocateSlot(Contact3OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3OneBodyLineExtractor, Vector<int>, Contact3OneBodyPrestepData, Contact3OneBodyProjection, Contact3AccumulatedImpulses>();
            AllocateSlot(Contact4OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4OneBodyLineExtractor, Vector<int>, Contact4OneBodyPrestepData, Contact4OneBodyProjection, Contact4AccumulatedImpulses>();

            AllocateSlot(Contact1TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact1LineExtractor, TwoBodyReferences, Contact1PrestepData, Contact1Projection, Contact1AccumulatedImpulses>();
            AllocateSlot(Contact2TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2LineExtractor, TwoBodyReferences, Contact2PrestepData, Contact2Projection, Contact2AccumulatedImpulses>();
            AllocateSlot(Contact3TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3LineExtractor, TwoBodyReferences, Contact3PrestepData, Contact3Projection, Contact3AccumulatedImpulses>();
            AllocateSlot(Contact4TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4LineExtractor, TwoBodyReferences, Contact4PrestepData, Contact4Projection, Contact4AccumulatedImpulses>();

            AllocateSlot(Contact2NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2NonconvexOneBodyLineExtractor, Vector<int>, Contact2NonconvexOneBodyPrestepData, Contact2NonconvexOneBodyProjection, Contact2NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact3NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3NonconvexOneBodyLineExtractor, Vector<int>, Contact3NonconvexOneBodyPrestepData, Contact3NonconvexOneBodyProjection, Contact3NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact4NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4NonconvexOneBodyLineExtractor, Vector<int>, Contact4NonconvexOneBodyPrestepData, Contact4NonconvexOneBodyProjection, Contact4NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact5NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact5NonconvexOneBodyLineExtractor, Vector<int>, Contact5NonconvexOneBodyPrestepData, Contact5NonconvexOneBodyProjection, Contact5NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact6NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact6NonconvexOneBodyLineExtractor, Vector<int>, Contact6NonconvexOneBodyPrestepData, Contact6NonconvexOneBodyProjection, Contact6NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact7NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact7NonconvexOneBodyLineExtractor, Vector<int>, Contact7NonconvexOneBodyPrestepData, Contact7NonconvexOneBodyProjection, Contact7NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact8NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact8NonconvexOneBodyLineExtractor, Vector<int>, Contact8NonconvexOneBodyPrestepData, Contact8NonconvexOneBodyProjection, Contact8NonconvexAccumulatedImpulses>();

            AllocateSlot(Contact2NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2NonconvexLineExtractor, TwoBodyReferences, Contact2NonconvexPrestepData, Contact2NonconvexProjection, Contact2NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact3NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3NonconvexLineExtractor, TwoBodyReferences, Contact3NonconvexPrestepData, Contact3NonconvexProjection, Contact3NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact4NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4NonconvexLineExtractor, TwoBodyReferences, Contact4NonconvexPrestepData, Contact4NonconvexProjection, Contact4NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact5NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact5NonconvexLineExtractor, TwoBodyReferences, Contact5NonconvexPrestepData, Contact5NonconvexProjection, Contact5NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact6NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact6NonconvexLineExtractor, TwoBodyReferences, Contact6NonconvexPrestepData, Contact6NonconvexProjection, Contact6NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact7NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact7NonconvexLineExtractor, TwoBodyReferences, Contact7NonconvexPrestepData, Contact7NonconvexProjection, Contact7NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact8NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact8NonconvexLineExtractor, TwoBodyReferences, Contact8NonconvexPrestepData, Contact8NonconvexProjection, Contact8NonconvexAccumulatedImpulses>();

            jobs = new QuickList<ThreadJob>(Environment.ProcessorCount * (jobsPerThread + 1), pool);

            executeJobDelegate = ExecuteJob;
        }

        Bodies bodies;
        Solver solver;
        private void ExecuteJob(int jobIndex)
        {
            ref var job = ref jobs[jobIndex];
            ref var typeBatch = ref solver.Sets[job.SetIndex].Batches[job.BatchIndex].TypeBatches[job.TypeBatchIndex];
            Debug.Assert(lineExtractors[typeBatch.TypeId] != null, "Jobs should only be created for types which are registered and used.");
            lineExtractors[typeBatch.TypeId].ExtractLines(bodies, job.SetIndex, ref typeBatch, job.ConstraintStart, job.ConstraintCount, ref job.jobLines);
        }


        internal void AddInstances(Bodies bodies, Solver solver, bool showConstraints, bool showContacts, ref QuickList<LineInstance> lines, ParallelLooper looper)
        {
            int neededLineCapacity = lines.Count;
            jobs.Count = 0;
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
                            if (typeBatch.TypeId >= lineExtractors.Length)
                                continue; //No registered extractor for this type, clearly.
                            var extractor = lineExtractors[typeBatch.TypeId];
                            var isContactBatch = NarrowPhase.IsContactConstraintType(typeBatch.TypeId);
                            if (extractor != null && ((isContactBatch && showContacts) || (!isContactBatch && showConstraints)))
                            {
                                jobs.Add(new ThreadJob
                                {
                                    SetIndex = setIndex,
                                    BatchIndex = batchIndex,
                                    TypeBatchIndex = typeBatchIndex,
                                    ConstraintStart = 0,
                                    ConstraintCount = typeBatch.ConstraintCount,
                                    LineStart = neededLineCapacity,
                                    LinesPerConstraint = extractor.LinesPerConstraint
                                }, pool);
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
                        jobs.Add(newJob, pool);
                        previousJob = newJob;
                    }
                }
            }
            lines.EnsureCapacity(neededLineCapacity, pool);
            lines.Count = neededLineCapacity; //Line additions will be performed on suballocated lists. This count will be used by the renderer when reading line data.
            for (int i = 0; i < jobs.Count; ++i)
            {
                //Creating a local copy of the list reference and count allows additions to proceed in parallel. 
                jobs[i].jobLines = new QuickList<LineInstance>(lines.Span);
                //By setting the count, we work around the fact that Array<T> doesn't support slicing.
                jobs[i].jobLines.Count = jobs[i].LineStart;
            }
            this.bodies = bodies;
            this.solver = solver;
            looper.For(0, jobs.Count, executeJobDelegate);
            this.bodies = null;
            this.solver = solver;
        }

        public void Dispose()
        {
            jobs.Dispose(pool);
        }
    }
}
