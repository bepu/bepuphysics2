using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading;

namespace BepuPhysics
{
    /// <summary>
    /// Provides functionality for efficiently activating previously deactivated bodies and their associated islands.
    /// </summary>
    public class IslandActivator
    {
        Solver solver;
        Bodies bodies;
        internal PairCache pairCache;
        BufferPool pool;

        public IslandActivator(Bodies bodies, Solver solver, BufferPool pool)
        {
            this.bodies = bodies;
            this.solver = solver;
            this.pool = pool;

            this.phaseOneWorkerDelegate = PhaseOneWorker;
            this.phaseTwoWorkerDelegate = PhaseTwoWorker;
        }

        /// <summary>
        /// Activates a body if it is inactive. All bodies that can be found by traversing the constraint graph from the body will also be activated.
        /// If the body is already active, this does nothing.
        /// </summary>
        /// <param name="bodyHandle">Handle of the body to activate.</param>
        public void ActivateBody(int bodyHandle)
        {
            bodies.ValidateExistingHandle(bodyHandle);
            ActivateSet(bodies.HandleToLocation[bodyHandle].SetIndex);
        }

        /// <summary>
        /// Activates any inactive bodies associated with a constraint. All bodies that can be found by traversing the constraint graph from the constraint referenced bodies will also be activated.
        /// If all bodies associated with the constraint are already active, this does nothing.
        /// </summary>
        /// <param name="constraintHandle">Handle of the constraint to activate.</param>
        public void ActivateConstraint(int constraintHandle)
        {
            ActivateSet(solver.HandleToConstraint[constraintHandle].SetIndex);
        }

        /// <summary>
        /// Activates all bodies and constraints within a set. Doesn't do anything if the set is active (index zero).
        /// </summary>
        /// <param name="setIndex">Index of the set to activate.</param>
        public void ActivateSet(int setIndex)
        {
            ValidateSetIndex(setIndex);
            if (setIndex > 0)
            {
                //TODO: Some fairly pointless work here- spans or other approaches could help with the API.
                QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), 1, out var list);
                list.AddUnsafely(setIndex);
                ActivateSets(ref list);
                list.Dispose(pool.SpecializeFor<int>());
            }
        }

        /// <summary>
        /// Activates a list of set indices.
        /// </summary>
        /// <param name="setIndices">List of set indices to activate.</param>
        /// <param name="threadDispatcher">Thread dispatcher to use when activating the bodies. Pass null to run on a single thread.</param>
        public void ActivateSets(ref QuickList<int, Buffer<int>> setIndices, IThreadDispatcher threadDispatcher = null)
        {
            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), setIndices.Count, out var uniqueSetIndices);
            var uniqueSet = new IndexSet(pool, bodies.Sets.Length);
            AccumulateUniqueIndices(ref setIndices, ref uniqueSet, ref uniqueSetIndices, pool);
            uniqueSet.Dispose(pool);

            //Note that we use the same codepath as multithreading, we just don't use a multithreaded dispatch to execute jobs.
            //TODO: It would probably be a good idea to add a little heuristic to avoid doing multithreaded dispatches if there are only like 5 total bodies.
            //Shouldn't matter too much- the threaded variant should only really be used when doing big batched changes, so having a fixed constant cost isn't that bad.
            int threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            var (phaseOneJobCount, phaseTwoJobCount) = PrepareJobs(ref uniqueSetIndices, threadCount);

            if (threadCount > 1)
            {
                this.jobIndex = -1;
                this.jobCount = phaseOneJobCount;
                threadDispatcher.DispatchWorkers(phaseOneWorkerDelegate);
            }
            else
            {
                for (int i = 0; i < phaseOneJobCount; ++i)
                {
                    ExecutePhaseOneJob(i);
                }
            }

            if (threadCount > 1)
            {
                this.jobIndex = -1;
                this.jobCount = phaseTwoJobCount;
                threadDispatcher.DispatchWorkers(phaseTwoWorkerDelegate);
            }
            else
            {
                for (int i = 0; i < phaseTwoJobCount; ++i)
                {
                    ExecutePhaseTwoJob(i);
                }
            }

            DisposeSets(ref uniqueSetIndices);

            uniqueSetIndices.Dispose(pool.SpecializeFor<int>());
        }

        //Note that the worker loop and its supporting fields are only used when the island activator is used in isolation by an external call.
        //The engine's own use of the activator takes place in the narrowphase, where the activation jobs are scheduled alongside other jobs for greater parallelism.
        int jobIndex;
        int jobCount;
        //TODO: once again, we repeat this worker pattern. We've done this, what, seven times? It wouldn't even be that difficult to centralize it. We'll get around to that at some point.
        Action<int> phaseOneWorkerDelegate;
        Action<int> phaseTwoWorkerDelegate;
        internal void PhaseOneWorker(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref jobIndex);
                if (index >= jobCount)
                    break;
                ExecutePhaseOneJob(index);
            }
        }
        internal void PhaseTwoWorker(int workerIndex)
        {
            while (true)
            {
                var index = Interlocked.Increment(ref jobIndex);
                if (index >= jobCount)
                    break;
                ExecutePhaseTwoJob(index);
            }
        }
        enum JobType
        {
            PairCache,
            UpdateBatchReferencedHandles,
            CopyBodyRegion
        }

        struct PhaseOneJob
        {
            public JobType Type;
            public int BatchIndex; //Only used by batch reference update jobs.
            //Only used by body region copy.
            public int SourceSet;
            public int SourceStart;
            public int TargetStart;
            public int Count;
            //could union these, but it's preeeetty pointless.
        }
        struct PhaseTwoJob
        {
            public int SourceStart;
            public int TargetStart;
            public int Count;
            public int SourceSet;
            public int TypeId;
            public int SourceBatch;
            public int SourceTypeBatch;
            public int TargetBatch;
            public int TargetTypeBatch;
        }

        QuickList<int, Buffer<int>> uniqueSetIndices;
        QuickList<PhaseOneJob, Buffer<PhaseOneJob>> phaseOneJobs;
        QuickList<PhaseTwoJob, Buffer<PhaseTwoJob>> phaseTwoJobs;
        internal void ExecutePhaseOneJob(int index)
        {
            ref var job = ref phaseOneJobs[index];
            switch (job.Type)
            {
                case JobType.PairCache:
                    {
                        //Updating the pair cache is locally sequential because it modifies the global overlap mapping, which at the moment is a hash table.
                    }
                    break;
                case JobType.UpdateBatchReferencedHandles:
                    {
                        //Note that the narrow phase will schedule this job alongside another worker which reads existing batch referenced handles.
                        //There will be some cache line sharing sometimes. That's not wonderful for performance, but it should not cause bugs:
                        //1) The narrowphase's speculative batch search is readonly, so there are no competing writes.
                        //2) Activation requires that a new constraint has been created involving an inactive body.
                        //The speculative search will then try to find a batch for that constraint, and it may end up erroneously
                        //finding a batch slot that gets blocked by this activation procedure. 
                        //But the speculative process is *speculative*; it is fine for it to be wrong, so long as it isn't wrong in a way that makes it choose a higher batch index.

                        //Note that this is parallel over different batches, regardless of which source set they're from.
                        ref var targetBatchReferencedHandles = ref solver.batchReferencedHandles[job.BatchIndex];
                        for (int i = 0; i < uniqueSetIndices.Count; ++i)
                        {
                            var setIndex = uniqueSetIndices[i];
                            ref var sourceSet = ref solver.Sets[setIndex];
                            if (sourceSet.Batches.Count > job.BatchIndex)
                            {
                                ref var batch = ref sourceSet.Batches[job.BatchIndex];
                                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                                {
                                    ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                                    solver.TypeProcessors[typeBatch.TypeId].AddInactiveBodyHandlesToBatchReferences(ref typeBatch, ref targetBatchReferencedHandles);
                                }
                            }
                        }
                    }
                    break;
                case JobType.CopyBodyRegion:
                    {
                        //Since we already preallocated everything during the job preparation, all we have to do is copy from the inactive set location.
                        ref var sourceSet = ref bodies.Sets[job.SourceSet];
                        ref var targetSet = ref bodies.ActiveSet;
                        sourceSet.Activity.CopyTo(job.SourceStart, ref targetSet.Activity, job.TargetStart, job.Count);
                        sourceSet.Collidables.CopyTo(job.SourceStart, ref targetSet.Collidables, job.TargetStart, job.Count);
                        sourceSet.Constraints.CopyTo(job.SourceStart, ref targetSet.Constraints, job.TargetStart, job.Count);
                        sourceSet.IndexToHandle.CopyTo(job.SourceStart, ref targetSet.IndexToHandle, job.TargetStart, job.Count);
                        sourceSet.LocalInertias.CopyTo(job.SourceStart, ref targetSet.LocalInertias, job.TargetStart, job.Count);
                        sourceSet.Poses.CopyTo(job.SourceStart, ref targetSet.Poses, job.TargetStart, job.Count);
                        sourceSet.Velocities.CopyTo(job.SourceStart, ref targetSet.Velocities, job.TargetStart, job.Count);
                    }
                    break;
            }
        }

        internal void ExecutePhaseTwoJob(int index)
        {
            ref var job = ref phaseTwoJobs[index];
            //Constraints are a little more complicated than bodies for two reasons:
            //1) Constraints are stored in AOSOA format. We cannot simply copy with bundle alignment with no preparation, since that may leave a gap.
            //To avoid this issue, we instead use the last bundle of the source batch to pad out any incomplete bundles ahead of the target copy region.
            //The scheduler attempts to maximize the number of jobs which are pure bundle copies.
            //2) Inactive constraints store their body references as body *handles* rather than body indices.
            //Pulling the type batches back into the active set requires translating those body handles to body indices.  
            //3) The translation from body handle to body index requires that the bodies already have an active set identity, which is why the constraints wait until the second phase.
            ref var sourceTypeBatch = ref solver.Sets[job.SourceSet].Batches[job.SourceBatch].TypeBatches[job.SourceTypeBatch];
            ref var targetTypeBatch = ref solver.ActiveSet.Batches[job.TargetBatch].TypeBatches[job.TargetTypeBatch];
            Debug.Assert(targetTypeBatch.TypeId == sourceTypeBatch.TypeId);
            solver.TypeProcessors[job.TypeId].CopyInactiveToActive(
                job.SourceSet, job.SourceBatch, job.SourceTypeBatch, job.TargetBatch, job.TargetTypeBatch,
                job.SourceStart, job.TargetStart, job.Count, bodies, solver);
        }

        internal void AccumulateUniqueIndices(ref QuickList<int, Buffer<int>> candidateSetIndices, ref IndexSet uniqueSet, ref QuickList<int, Buffer<int>> uniqueSetIndices, BufferPool pool)
        {
            for (int i = 0; i < candidateSetIndices.Count; ++i)
            {
                var candidateSetIndex = candidateSetIndices[i];
                if (!uniqueSet.Contains(candidateSetIndex))
                {
                    uniqueSet.Add(candidateSetIndex, pool);
                    uniqueSetIndices.Add(candidateSetIndex, pool.SpecializeFor<int>());
                }
            }
        }
        [Conditional("DEBUG")]
        void ValidateSetIndex(int setIndex)
        {
            Debug.Assert(setIndex >= 0 && setIndex < bodies.Sets.Length && setIndex < solver.Sets.Length && setIndex < pairCache.InactiveSets.Length);
            Debug.Assert(bodies.Sets[setIndex].Allocated && solver.Sets[setIndex].Allocated && pairCache.InactiveSets[setIndex].Allocated);
        }
        [Conditional("DEBUG")]
        void ValidateUniqueSets(ref QuickList<int, Buffer<int>> setIndices)
        {
            var set = new IndexSet(pool, bodies.Sets.Length);
            for (int i = 0; i < setIndices.Count; ++i)
            {
                var setIndex = setIndices[i];
                ValidateSetIndex(setIndex);
                Debug.Assert(!set.Contains(setIndex));
                set.Add(setIndex, pool);
            }
            set.Dispose(pool);

        }

        internal (int phaseOneJobCount, int phaseTwoJobCount) PrepareJobs(ref QuickList<int, Buffer<int>> setIndices, int threadCount)
        {
            ValidateUniqueSets(ref setIndices);
            this.uniqueSetIndices = setIndices;
            return (0, 0);
        }

        internal void DisposeSets(ref QuickList<int, Buffer<int>> setIndices)
        {
            for (int i = 0; i < setIndices.Count; ++i)
            {
                ref var bodySet = ref bodies.Sets[i];
                ref var constraintSet = ref solver.Sets[i];
                ref var pairCacheSet = ref pairCache.InactiveSets[i];
                Debug.Assert(bodySet.Allocated && constraintSet.Allocated && pairCacheSet.Allocated);
                bodySet.DisposeBuffers(pool);
                constraintSet.Dispose(pool);
                pairCacheSet.Dispose(pool);
                this.uniqueSetIndices = new QuickList<int, Buffer<int>>();
            }
        }


        internal void Dispose()
        {
        }
    }
}
