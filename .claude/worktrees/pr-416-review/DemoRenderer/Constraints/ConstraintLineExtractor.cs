using System;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using BepuUtilities;
using System.Numerics;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints.Contact;

namespace DemoRenderer.Constraints
{
    unsafe interface IConstraintLineExtractor<TPrestep>
    {
        static abstract int LinesPerConstraint { get; }

        static abstract void ExtractLines(ref TPrestep prestepBundle, int setIndex, int* bodyLocations, Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines);
    }
    abstract class TypeLineExtractor
    {
        public abstract int LinesPerConstraint { get; }
        public abstract void ExtractLines(Bodies bodies, int setIndex, ref TypeBatch typeBatch, int constraintStart, int constraintCount, ref QuickList<LineInstance> lines);
    }

    class TypeLineExtractor<T, TBodyReferences, TPrestep, TAccumulatedImpulses> : TypeLineExtractor
        where TBodyReferences : unmanaged
        where TPrestep : unmanaged
        where T : struct, IConstraintLineExtractor<TPrestep>
    {
        public override int LinesPerConstraint => T.LinesPerConstraint;
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

            var constraintEnd = constraintStart + constraintCount;
            if (setIndex == 0)
            {
                var tint = new Vector3(1, 1, 1);
                for (int i = constraintStart; i < constraintEnd; ++i)
                {
                    if (typeBatch.IndexToHandle[i].Value >= 0)
                    {
                        BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                        ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                        ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                        ref var firstReference = ref Unsafe.As<TBodyReferences, Vector<int>>(ref referencesBundle);
                        for (int j = 0; j < bodyCount; ++j)
                        {
                            //Active set constraint body references refer directly to the body index.
                            bodyIndices[j] = GatherScatter.Get(ref Unsafe.Add(ref firstReference, j), innerIndex) & Bodies.BodyReferenceMask;
                        }
                        T.ExtractLines(ref GatherScatter.GetOffsetInstance(ref prestepBundle, innerIndex), setIndex, bodyIndices, bodies, ref tint, ref lines);
                    }
                }
            }
            else
            {
                var tint = new Vector3(0.4f, 0.4f, 0.8f);
                for (int i = constraintStart; i < constraintEnd; ++i)
                {
                    if (typeBatch.IndexToHandle[i].Value >= 0)
                    {
                        BundleIndexing.GetBundleIndices(i, out var bundleIndex, out var innerIndex);
                        ref var prestepBundle = ref Unsafe.Add(ref prestepStart, bundleIndex);
                        ref var referencesBundle = ref Unsafe.Add(ref referencesStart, bundleIndex);
                        ref var firstReference = ref Unsafe.As<TBodyReferences, Vector<int>>(ref referencesBundle);
                        for (int j = 0; j < bodyCount; ++j)
                        {
                            //Inactive constraints store body references in the form of handles, so we have to follow the indirection.
                            var bodyHandle = GatherScatter.Get(ref Unsafe.Add(ref firstReference, j), innerIndex) & Bodies.BodyReferenceMask;
                            Debug.Assert(bodies.HandleToLocation[bodyHandle].SetIndex == setIndex);
                            bodyIndices[j] = bodies.HandleToLocation[bodyHandle].Index & Bodies.BodyReferenceMask;
                        }
                        T.ExtractLines(ref GatherScatter.GetOffsetInstance(ref prestepBundle, innerIndex), setIndex, bodyIndices, bodies, ref tint, ref lines);
                    }
                }
            }
        }
    }

    internal class ConstraintLineExtractor
    {
        TypeLineExtractor[] lineExtractors;

        internal struct ThreadJob
        {
            public int SimulationIndex;
            public int SetIndex;
            public int BatchIndex;
            public int TypeBatchIndex;
            public int SourceConstraintStart;
            //Source and target constraint counts can differ because of noncontiguous fallback batches.
            public int SourceConstraintCount;
            public int TargetLineStart;
            public int TargetConstraintCount;
            public int LinesPerConstraint;
        }

        public bool Enabled { get; set; } = true;


        ref TypeLineExtractor AllocateSlot(int typeId)
        {
            if (typeId >= lineExtractors.Length)
                Array.Resize(ref lineExtractors, typeId + 1);
            return ref lineExtractors[typeId];
        }
        public ConstraintLineExtractor()
        {
            lineExtractors = new TypeLineExtractor[32];
            AllocateSlot(BallSocketTypeProcessor.BatchTypeId) = new TypeLineExtractor<BallSocketLineExtractor, TwoBodyReferences, BallSocketPrestepData, Vector3Wide>();
            AllocateSlot(BallSocketServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<BallSocketServoLineExtractor, TwoBodyReferences, BallSocketServoPrestepData, Vector3Wide>();
            AllocateSlot(BallSocketMotorTypeProcessor.BatchTypeId) = new TypeLineExtractor<BallSocketMotorLineExtractor, TwoBodyReferences, BallSocketMotorPrestepData, Vector3Wide>();
            AllocateSlot(WeldTypeProcessor.BatchTypeId) = new TypeLineExtractor<WeldLineExtractor, TwoBodyReferences, WeldPrestepData, WeldAccumulatedImpulses>();
            AllocateSlot(DistanceServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<DistanceServoLineExtractor, TwoBodyReferences, DistanceServoPrestepData, Vector<float>>();
            AllocateSlot(DistanceLimitTypeProcessor.BatchTypeId) = new TypeLineExtractor<DistanceLimitLineExtractor, TwoBodyReferences, DistanceLimitPrestepData, Vector<float>>();
            AllocateSlot(CenterDistanceTypeProcessor.BatchTypeId) = new TypeLineExtractor<CenterDistanceLineExtractor, TwoBodyReferences, CenterDistancePrestepData, Vector<float>>();
            AllocateSlot(CenterDistanceLimitTypeProcessor.BatchTypeId) = new TypeLineExtractor<CenterDistanceLimitLineExtractor, TwoBodyReferences, CenterDistanceLimitPrestepData, Vector<float>>();
            AllocateSlot(PointOnLineServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<PointOnLineLineExtractor, TwoBodyReferences, PointOnLineServoPrestepData, Vector2Wide>();
            AllocateSlot(LinearAxisServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<LinearAxisServoLineExtractor, TwoBodyReferences, LinearAxisServoPrestepData, Vector<float>>();
            AllocateSlot(AngularSwivelHingeTypeProcessor.BatchTypeId) = new TypeLineExtractor<AngularSwivelHingeLineExtractor, TwoBodyReferences, AngularSwivelHingePrestepData, Vector<float>>();
            AllocateSlot(SwivelHingeTypeProcessor.BatchTypeId) = new TypeLineExtractor<SwivelHingeLineExtractor, TwoBodyReferences, SwivelHingePrestepData, Vector4Wide>();
            AllocateSlot(HingeTypeProcessor.BatchTypeId) = new TypeLineExtractor<HingeLineExtractor, TwoBodyReferences, HingePrestepData, HingeAccumulatedImpulses>();
            AllocateSlot(OneBodyLinearServoTypeProcessor.BatchTypeId) = new TypeLineExtractor<OneBodyLinearServoLineExtractor, Vector<int>, OneBodyLinearServoPrestepData, Vector<float>>();

            AllocateSlot(Contact1OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact1OneBodyLineExtractor, Vector<int>, Contact1OneBodyPrestepData, Contact1AccumulatedImpulses>();
            AllocateSlot(Contact2OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2OneBodyLineExtractor, Vector<int>, Contact2OneBodyPrestepData, Contact2AccumulatedImpulses>();
            AllocateSlot(Contact3OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3OneBodyLineExtractor, Vector<int>, Contact3OneBodyPrestepData, Contact3AccumulatedImpulses>();
            AllocateSlot(Contact4OneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4OneBodyLineExtractor, Vector<int>, Contact4OneBodyPrestepData, Contact4AccumulatedImpulses>();

            AllocateSlot(Contact1TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact1LineExtractor, TwoBodyReferences, Contact1PrestepData, Contact1AccumulatedImpulses>();
            AllocateSlot(Contact2TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2LineExtractor, TwoBodyReferences, Contact2PrestepData, Contact2AccumulatedImpulses>();
            AllocateSlot(Contact3TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3LineExtractor, TwoBodyReferences, Contact3PrestepData, Contact3AccumulatedImpulses>();
            AllocateSlot(Contact4TypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4LineExtractor, TwoBodyReferences, Contact4PrestepData, Contact4AccumulatedImpulses>();

            AllocateSlot(Contact2NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2NonconvexOneBodyLineExtractor, Vector<int>, Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact3NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3NonconvexOneBodyLineExtractor, Vector<int>, Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact4NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4NonconvexOneBodyLineExtractor, Vector<int>, Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact5NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact5NonconvexOneBodyLineExtractor, Vector<int>, Contact5NonconvexOneBodyPrestepData, Contact5NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact6NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact6NonconvexOneBodyLineExtractor, Vector<int>, Contact6NonconvexOneBodyPrestepData, Contact6NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact7NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact7NonconvexOneBodyLineExtractor, Vector<int>, Contact7NonconvexOneBodyPrestepData, Contact7NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact8NonconvexOneBodyTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact8NonconvexOneBodyLineExtractor, Vector<int>, Contact8NonconvexOneBodyPrestepData, Contact8NonconvexAccumulatedImpulses>();

            AllocateSlot(Contact2NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact2NonconvexLineExtractor, TwoBodyReferences, Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact3NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact3NonconvexLineExtractor, TwoBodyReferences, Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses>();
            AllocateSlot(Contact4NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact4NonconvexLineExtractor, TwoBodyReferences, Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact5NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact5NonconvexLineExtractor, TwoBodyReferences, Contact5NonconvexPrestepData, Contact5NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact6NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact6NonconvexLineExtractor, TwoBodyReferences, Contact6NonconvexPrestepData, Contact6NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact7NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact7NonconvexLineExtractor, TwoBodyReferences, Contact7NonconvexPrestepData, Contact7NonconvexAccumulatedImpulses>();
            //AllocateSlot(Contact8NonconvexTypeProcessor.BatchTypeId) = new TypeLineExtractor<Contact8NonconvexLineExtractor, TwoBodyReferences, Contact8NonconvexPrestepData, Contact8NonconvexAccumulatedImpulses>();
        }

        internal void ExecuteJob(Buffer<LineInstance> lines, Simulation simulation, ThreadJob job)
        {
            ref var typeBatch = ref simulation.Solver.Sets[job.SetIndex].Batches[job.BatchIndex].TypeBatches[job.TypeBatchIndex];
            Debug.Assert(lineExtractors[typeBatch.TypeId] != null, "Jobs should only be created for types which are registered and used.");
            var targetLines = new QuickList<LineInstance>(lines.Slice(job.TargetLineStart, job.TargetConstraintCount * job.LinesPerConstraint));
            lineExtractors[typeBatch.TypeId].ExtractLines(simulation.Bodies, job.SetIndex, ref typeBatch, job.SourceConstraintStart, job.SourceConstraintCount, ref targetLines);
        }


        internal void CreateJobs(Simulation simulation, int simulationIndex, bool showConstraints, bool showContacts, ref QuickList<LineInstance> lines, ref QuickList<ThreadJob> jobs, BufferPool pool)
        {
            int neededLineCapacity = lines.Count;
            int newLineCount = 0;
            var solver = simulation.Solver;
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
                                continue; //No registered extractor for this type.
                            var extractor = lineExtractors[typeBatch.TypeId];
                            var isContactBatch = NarrowPhase.IsContactConstraintType(typeBatch.TypeId);
                            if (extractor != null && ((isContactBatch && showContacts) || (!isContactBatch && showConstraints)))
                            {
                                //Note that we need to handle the case of fallback batches, hence the call to this helper function rather than directly trusting the potentially conservative typeBatch.ConstraintCount.
                                var constraintCount = solver.CountConstraintsInTypeBatch(setIndex, batchIndex, typeBatchIndex);
                                jobs.Add(new ThreadJob
                                {
                                    SimulationIndex = simulationIndex,
                                    SetIndex = setIndex,
                                    BatchIndex = batchIndex,
                                    TypeBatchIndex = typeBatchIndex,
                                    SourceConstraintStart = 0,
                                    SourceConstraintCount = typeBatch.ConstraintCount,
                                    TargetLineStart = neededLineCapacity,
                                    TargetConstraintCount = constraintCount,
                                    LinesPerConstraint = extractor.LinesPerConstraint
                                }, pool);
                                var linesForTypeBatch = extractor.LinesPerConstraint * constraintCount;
                                neededLineCapacity += linesForTypeBatch;
                                newLineCount += linesForTypeBatch;
                            }
                        }
                    }
                }
            }
            if (newLineCount > 0)
                lines.Allocate(newLineCount, pool);
        }

    }
}
