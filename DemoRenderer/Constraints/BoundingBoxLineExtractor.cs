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
using System.Runtime.CompilerServices;

namespace DemoRenderer.Constraints
{
    public class BoundingBoxLineExtractor
    {
        const int jobsPerThread = 4;
        QuickList<ThreadJob> jobs;
        BroadPhase broadPhase;
        int masterLinesCount;
        Buffer<LineInstance> masterLinesSpan;

        struct ThreadJob
        {
            public int LeafStart;
            public int LeafCount;
            public bool CoversActiveCollidables;
        }

        BufferPool pool;
        Action<int> workDelegate;
        public BoundingBoxLineExtractor(BufferPool pool)
        {
            this.pool = pool;
            jobs = new QuickList<ThreadJob>(Environment.ProcessorCount * jobsPerThread, pool);
            workDelegate = Work;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteBoundsLines(in Vector3 min, in Vector3 max, uint packedColor, uint packedBackgroundColor, ref LineInstance targetLines)
        {
            var v001 = new Vector3(min.X, min.Y, max.Z);
            var v010 = new Vector3(min.X, max.Y, min.Z);
            var v011 = new Vector3(min.X, max.Y, max.Z);
            var v100 = new Vector3(max.X, min.Y, min.Z);
            var v101 = new Vector3(max.X, min.Y, max.Z);
            var v110 = new Vector3(max.X, max.Y, min.Z);
            Unsafe.Add(ref targetLines, 0) = new LineInstance(min, v001, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 1) = new LineInstance(min, v010, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 2) = new LineInstance(min, v100, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 3) = new LineInstance(v001, v011, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 4) = new LineInstance(v001, v101, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 5) = new LineInstance(v010, v011, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 6) = new LineInstance(v010, v110, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 7) = new LineInstance(v011, max, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 8) = new LineInstance(v100, v101, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 9) = new LineInstance(v100, v110, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 10) = new LineInstance(v101, max, packedColor, packedBackgroundColor);
            Unsafe.Add(ref targetLines, 11) = new LineInstance(v110, max, packedColor, packedBackgroundColor);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteBoundsLines(in Vector3 min, in Vector3 max, in Vector3 color, in Vector3 backgroundColor, ref LineInstance targetLines)
        {
            WriteBoundsLines(min, max, Helpers.PackColor(color), Helpers.PackColor(backgroundColor), ref targetLines);
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
            var packedColor = Helpers.PackColor(color);
            var packedBackgroundColor = Helpers.PackColor(backgroundColor);
            for (int i = 0; i < job.LeafCount; ++i)
            {
                var broadPhaseIndex = job.LeafStart + i;

                Vector3* min, max;
                if (job.CoversActiveCollidables)
                    broadPhase.GetActiveBoundsPointers(broadPhaseIndex, out min, out max);
                else
                    broadPhase.GetStaticBoundsPointers(broadPhaseIndex, out min, out max);
                var outputStartIndex = masterStart + i * 12;
                WriteBoundsLines(*min, *max, packedColor, packedBackgroundColor, ref masterLinesSpan[outputStartIndex]);
            }
        }


        void CreateJobsForTree(in Tree tree, bool active, ref QuickList<ThreadJob> jobs)
        {
            var maximumJobCount = jobsPerThread * Environment.ProcessorCount;
            var possibleLeavesPerJob = tree.LeafCount / maximumJobCount;
            var remainder = tree.LeafCount - possibleLeavesPerJob * maximumJobCount;
            int jobbedLeafCount = 0;
            jobs.EnsureCapacity(jobs.Count + maximumJobCount, pool);
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

        internal unsafe void AddInstances(BroadPhase broadPhase, ref QuickList<LineInstance> lines, ParallelLooper looper, BufferPool pool)
        {
            //For now, we only pull the bounding boxes of objects that are active.
            lines.EnsureCapacity(lines.Count + 12 * (broadPhase.ActiveTree.LeafCount + broadPhase.StaticTree.LeafCount), pool);
            CreateJobsForTree(broadPhase.ActiveTree, true, ref jobs);
            CreateJobsForTree(broadPhase.StaticTree, false, ref jobs);
            masterLinesSpan = lines.Span;
            masterLinesCount = lines.Count;
            this.broadPhase = broadPhase;
            looper.For(0, jobs.Count, workDelegate);
            lines.Count = masterLinesCount;
            this.broadPhase = null;
            jobs.Count = 0;
        }

        public void Dispose()
        {
            jobs.Dispose(pool);
        }
    }
}
