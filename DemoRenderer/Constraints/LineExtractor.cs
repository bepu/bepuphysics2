using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.CollisionDetection;

namespace DemoRenderer.Constraints
{
    public class LineExtractor
    {
        internal QuickList<LineInstance, Array<LineInstance>> lines;
        ConstraintLineExtractor constraints;
        BoundingBoxLineExtractor boundingBoxes;

        ParallelLooper looper;
        public LineExtractor(ParallelLooper looper, int initialLineCapacity = 8192)
        {
            QuickList<LineInstance, Array<LineInstance>>.Create(new PassthroughArrayPool<LineInstance>(), initialLineCapacity, out lines);
            constraints = new ConstraintLineExtractor();
            boundingBoxes = new BoundingBoxLineExtractor();
            this.looper = looper;
        }

        public void Extract(Bodies bodies, Solver solver, BroadPhase broadPhase, bool showConstraints = true, bool showContacts = false, bool showBoundingBoxes = false, IThreadDispatcher threadDispatcher = null)
        {
            looper.Dispatcher = threadDispatcher;
            if (showConstraints || showContacts)
                constraints.AddInstances(bodies, solver, showConstraints, showContacts, ref lines, looper);
            if (showBoundingBoxes)
                boundingBoxes.AddInstances(broadPhase, ref lines, looper);
        }

        public ref LineInstance Allocate()
        {
            return ref lines.Allocate(new PassthroughArrayPool<LineInstance>());
        }

        public void ClearInstances()
        {
            lines.Count = 0;
        }

    }
}
