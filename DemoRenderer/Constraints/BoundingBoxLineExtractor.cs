using System;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using System.Numerics;
using System.Threading.Tasks;
using System.Threading;
using BepuUtilities;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Trees;

namespace DemoRenderer.Constraints
{
    internal class BoundingBoxLineExtractor
    {
        const int jobsPerThread = 4;
        QuickList<ThreadJob, Array<ThreadJob>> jobs;
        BroadPhase broadPhase;
        int masterLinesCount;
        Array<LineInstance> masterLinesSpan;

        struct ThreadJob
        {
            public int LeafStart;
            public int LeafCount;
            public bool CoversActiveCollidables;
        }

        Action<int> workDelegate;
        public BoundingBoxLineExtractor()
        {
            QuickList<ThreadJob, Array<ThreadJob>>.Create(new PassthroughArrayPool<ThreadJob>(), Environment.ProcessorCount * jobsPerThread, out jobs);
            workDelegate = Work;
        }

        private unsafe void Work(int jobIndex)
        {
            ref var job = ref jobs[jobIndex];
            var end = job.LeafStart + job.LeafCount;
            var lineCount = 12 * job.LeafCount;
            var masterStart = Interlocked.Add(ref masterLinesCount, lineCount) - lineCount;
            var color = new Vector3(0, 1, 0);
            var backgroundColor = new Vector3(0, 0, 0);
            if (!job.CoversActiveCollidables)
            {
                var inactiveTint = new Vector3(0.3f, 0.3f, 0.7f);
                color *= inactiveTint;
                backgroundColor *= inactiveTint;
            }
            var packedColor = Helpers.PackColor(ref color);
            var packedBackgroundColor = Helpers.PackColor(ref backgroundColor);
            for (int i = 0; i < job.LeafCount; ++i)
            {
                var broadPhaseIndex = job.LeafStart + i;

                Vector3* min, max;
                if (job.CoversActiveCollidables)
                    broadPhase.GetActiveBoundsPointers(broadPhaseIndex, out min, out max);
                else
                    broadPhase.GetStaticBoundsPointers(broadPhaseIndex, out min, out max);
                var v001 = new Vector3(min->X, min->Y, max->Z);
                var v010 = new Vector3(min->X, max->Y, min->Z);
                var v011 = new Vector3(min->X, max->Y, max->Z);
                var v100 = new Vector3(max->X, min->Y, min->Z);
                var v101 = new Vector3(max->X, min->Y, max->Z);
                var v110 = new Vector3(max->X, max->Y, min->Z);
                var outputStartIndex = masterStart + i * 12;
                masterLinesSpan[outputStartIndex + 0] = new LineInstance(ref *min, ref v001, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 1] = new LineInstance(ref *min, ref v010, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 2] = new LineInstance(ref *min, ref v100, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 3] = new LineInstance(ref v001, ref v011, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 4] = new LineInstance(ref v001, ref v101, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 5] = new LineInstance(ref v010, ref v011, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 6] = new LineInstance(ref v010, ref v110, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 7] = new LineInstance(ref v011, ref *max, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 8] = new LineInstance(ref v100, ref v101, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 9] = new LineInstance(ref v100, ref v110, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 10] = new LineInstance(ref v101, ref *max, packedColor, packedBackgroundColor);
                masterLinesSpan[outputStartIndex + 11] = new LineInstance(ref v110, ref *max, packedColor, packedBackgroundColor);
            }
        }


        void CreateJobsForTree(Tree tree, bool active, ref QuickList<ThreadJob, Array<ThreadJob>> jobs)
        {
            var maximumJobCount = jobsPerThread * Environment.ProcessorCount;
            var possibleLeavesPerJob = tree.LeafCount / maximumJobCount;
            var remainder = tree.LeafCount - possibleLeavesPerJob * maximumJobCount;
            int jobbedLeafCount = 0;
            jobs.EnsureCapacity(jobs.Count + maximumJobCount, new PassthroughArrayPool<ThreadJob>());
            for (int i = 0; i < maximumJobCount; ++i)
            {
                var jobLeafCount = i < remainder ? possibleLeavesPerJob + 1 : possibleLeavesPerJob;
                if (jobLeafCount > 0)
                {
                    ref var job = ref jobs.AllocateUnsafely();
                    job.LeafCount = jobLeafCount;
                    job.LeafStart = jobbedLeafCount;
                    job.CoversActiveCollidables = active;
                    jobbedLeafCount += jobLeafCount;
                }
                else
                    break;
            }
        }

        internal unsafe void AddInstances(BroadPhase broadPhase, ref QuickList<LineInstance, Array<LineInstance>> lines, ParallelLooper looper)
        {
            //For now, we only pull the bounding boxes of objects that are active.
            lines.EnsureCapacity(lines.Count + 12 * (broadPhase.ActiveTree.LeafCount + broadPhase.StaticTree.LeafCount), new PassthroughArrayPool<LineInstance>());
            CreateJobsForTree(broadPhase.ActiveTree, true, ref jobs);
            CreateJobsForTree(broadPhase.StaticTree, false, ref jobs);
            masterLinesSpan = lines.Span;
            masterLinesCount = lines.Count;
            this.broadPhase = broadPhase;
            looper.For(0, jobs.Count, workDelegate);
            lines.Count = masterLinesCount;
            this.broadPhase = null;
            this.masterLinesSpan = new Array<LineInstance>();
            jobs.Count = 0;

        }

    }
}
