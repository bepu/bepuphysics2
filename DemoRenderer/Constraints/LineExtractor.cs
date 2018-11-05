using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.CollisionDetection;
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
        public LineExtractor(BufferPool pool, ParallelLooper looper, int initialLineCapacity = 8192)
        {
            lines = new QuickList<LineInstance>(initialLineCapacity, pool);
            constraints = new ConstraintLineExtractor(pool);
            boundingBoxes = new BoundingBoxLineExtractor(pool);
            this.pool = pool;
            this.looper = looper;
        }

        public void Extract(Bodies bodies, Solver solver, BroadPhase broadPhase, bool showConstraints = true, bool showContacts = false, bool showBoundingBoxes = false, IThreadDispatcher threadDispatcher = null)
        {
            looper.Dispatcher = threadDispatcher;
            if (showConstraints || showContacts)
                constraints.AddInstances(bodies, solver, showConstraints, showContacts, ref lines, looper);
            if (showBoundingBoxes)
                boundingBoxes.AddInstances(broadPhase, ref lines, looper, pool);
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
            constraints.Dispose();
            boundingBoxes.Dispose();
        }
    }
}
