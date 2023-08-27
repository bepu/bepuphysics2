using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using System;

namespace DemoRenderer.Constraints
{
    public class LineExtractor : IDisposable
    {
        internal QuickList<LineInstance> lines;
        ConstraintLineExtractor constraints;
        BoundingBoxLineExtractor boundingBoxes;

        BufferPool pool;
        ParallelLooper looper;
        LooperAction executeJobAction;

        public bool ShowConstraints = true;
        public bool ShowContacts;
        public bool ShowBoundingBoxes;

        public LineExtractor(BufferPool pool, ParallelLooper looper, int initialLineCapacity = 8192)
        {
            lines = new QuickList<LineInstance>(initialLineCapacity, pool);
            constraints = new ConstraintLineExtractor();
            boundingBoxes = new BoundingBoxLineExtractor();
            this.pool = pool;
            this.looper = looper;
            executeJobAction = ExecuteJob;

        }

        Simulation[] simulations;
        Simulation simulation;
        QuickList<ConstraintLineExtractor.ThreadJob> constraintJobs;
        QuickList<BoundingBoxLineExtractor.ThreadJob> boundingBoxJobs;

        void ExecuteJob(int jobIndex, int workerIndex)
        {
            if (jobIndex >= constraintJobs.Count)
            {
                //This is a bounding box job.
                var job = boundingBoxJobs[jobIndex - constraintJobs.Count];
                var simulation = simulations == null ? this.simulation : simulations[job.SimulationIndex];
                boundingBoxes.ExecuteJob(lines.Span, simulation, job);
            }
            else
            {
                //This is a constraint job.
                var job = constraintJobs[jobIndex];
                var simulation = simulations == null ? this.simulation : simulations[job.SimulationIndex];
                constraints.ExecuteJob(lines.Span, simulation, job);
            }
        }

        public void Extract(Simulation[] simulations, IThreadDispatcher threadDispatcher = null)
        {
            if (ShowConstraints || ShowContacts)
            {
                constraintJobs = new QuickList<ConstraintLineExtractor.ThreadJob>(128, pool);
                for (int i = 0; i < simulations.Length; ++i)
                {
                    constraints.CreateJobs(simulations[i], i, ShowConstraints, ShowContacts, ref lines, ref constraintJobs, pool);
                }
            }
            if (ShowBoundingBoxes)
            {
                boundingBoxJobs = new QuickList<BoundingBoxLineExtractor.ThreadJob>(128, pool);
                for (int i = 0; i < simulations.Length; ++i)
                {
                    boundingBoxes.CreateJobs(simulations[i], i, ref lines, ref boundingBoxJobs, pool);
                }
            }
            this.simulations = simulations;
            looper.Dispatcher = threadDispatcher;
            looper.For(0, constraintJobs.Count + boundingBoxJobs.Count, executeJobAction);
            looper.Dispatcher = null;

            if (constraintJobs.Span.Allocated)
            {
                constraintJobs.Dispose(pool);
                constraintJobs = default;
            }
            if (boundingBoxJobs.Span.Allocated)
            {
                boundingBoxJobs.Dispose(pool);
                boundingBoxJobs = default;
            }
            this.simulations = null;

        }

        public void Extract(Simulation simulation, IThreadDispatcher threadDispatcher = null)
        {
            this.simulation = simulation;
            if (ShowConstraints || ShowContacts)
            {
                constraintJobs = new QuickList<ConstraintLineExtractor.ThreadJob>(128, pool);
                constraints.CreateJobs(simulation, 0, ShowConstraints, ShowContacts, ref lines, ref constraintJobs, pool);
            }
            if (ShowBoundingBoxes)
            {
                boundingBoxJobs = new QuickList<BoundingBoxLineExtractor.ThreadJob>(128, pool);
                boundingBoxes.CreateJobs(simulation, 0, ref lines, ref boundingBoxJobs, pool);
            }
            looper.Dispatcher = threadDispatcher;
            looper.For(0, constraintJobs.Count + boundingBoxJobs.Count, executeJobAction);
            looper.Dispatcher = null;

            if (constraintJobs.Span.Allocated)
            {
                constraintJobs.Dispose(pool);
                constraintJobs = default;
            }
            if (boundingBoxJobs.Span.Allocated)
            {
                boundingBoxJobs.Dispose(pool);
                boundingBoxJobs = default;
            }
            this.simulation = null;
        }

        public ref LineInstance Allocate()
        {
            return ref lines.Allocate(pool);
        }

        public ref LineInstance Allocate(int count)
        {
            return ref lines.Allocate(count, pool);
        }

        public void ClearInstances()
        {
            lines.Count = 0;
        }

        public void Dispose()
        {
            lines.Dispose(pool);
        }
    }
}
