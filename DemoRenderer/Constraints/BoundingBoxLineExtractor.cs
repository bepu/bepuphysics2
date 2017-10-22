using System;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using System.Numerics;
using System.Threading.Tasks;
using System.Threading;
using BepuUtilities;
using BepuPhysics.CollisionDetection;

namespace DemoRenderer.Constraints
{
    internal class BoundingBoxLineExtractor
    {
        const int jobsPerThread = 4;
        QuickList<ThreadJob, Array<ThreadJob>> jobs;
        Vector3 color = new Vector3(0, 1, 0);
        BroadPhase broadPhase;
        int masterLinesCount;

        struct ThreadJob
        {
            public int LeafStart;
            public int LeafCount;
            public QuickList<LineInstance, Array<LineInstance>> JobLines;
            public Array<LineInstance> MasterLinesSpan;
        }

        Action<int> workDelegate;
        public BoundingBoxLineExtractor()
        {
            QuickList<ThreadJob, Array<ThreadJob>>.Create(new PassthroughArrayPool<ThreadJob>(), Environment.ProcessorCount * jobsPerThread, out jobs);
            //Because we don't know how many lines will be created beforehand, each thread needs a dedicated structure. We'll copy everything together at the end. 
            //Not the most efficient thing, but it doesn't matter much.
            for (int i = 0; i < jobs.Span.Length; ++i)
            {
                QuickList<LineInstance, Array<LineInstance>>.Create(new PassthroughArrayPool<LineInstance>(), Environment.ProcessorCount * jobsPerThread, out jobs.Span[i].JobLines);
            }
            workDelegate = Work;
        }

        private unsafe void Work(int jobIndex)
        {
            ref var job = ref jobs[jobIndex];
            var end = job.LeafStart + job.LeafCount;
            job.JobLines.EnsureCapacity(job.LeafCount * 12, new PassthroughArrayPool<LineInstance>());
            for (int broadPhaseIndex = job.LeafStart; broadPhaseIndex < end; ++broadPhaseIndex)
            {
                broadPhase.GetActiveBoundsPointers(broadPhaseIndex, out var minFloat, out var maxFloat);
                var min = (Vector3*)minFloat;
                var max = (Vector3*)maxFloat;
                var v001 = new Vector3(min->X, min->Y, max->Z);
                var v010 = new Vector3(min->X, max->Y, min->Z);
                var v011 = new Vector3(min->X, max->Y, max->Z);
                var v100 = new Vector3(max->X, min->Y, min->Z);
                var v101 = new Vector3(max->X, min->Y, max->Z);
                var v110 = new Vector3(max->X, max->Y, min->Z);
                job.JobLines.AddUnsafely(new LineInstance(ref *min, ref v001, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref *min, ref v010, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref *min, ref v100, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v001, ref v011, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v001, ref v101, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v010, ref v011, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v010, ref v110, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v011, ref *max, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v100, ref v101, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v100, ref v110, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v101, ref *max, ref color));
                job.JobLines.AddUnsafely(new LineInstance(ref v110, ref *max, ref color));
            }
            var masterStart = Interlocked.Add(ref masterLinesCount, job.JobLines.Count) - job.JobLines.Count;
            job.JobLines.Span.CopyTo(0, ref job.MasterLinesSpan, masterStart, job.JobLines.Count);
        }



        internal unsafe void AddInstances(BroadPhase broadPhase, ref QuickList<LineInstance, Array<LineInstance>> lines, ParallelLooper looper)
        {
            //For now, we only pull the bounding boxes of objects that are active.
            var activeLeafCount = broadPhase.ActiveTree.LeafCount;
            lines.EnsureCapacity(lines.Count + 12 * activeLeafCount, new PassthroughArrayPool<LineInstance>());
            var maximumJobCount = jobsPerThread * Environment.ProcessorCount;
            var possibleLeavesPerJob = activeLeafCount / maximumJobCount;
            var remainder = activeLeafCount - possibleLeavesPerJob * maximumJobCount;
            int jobbedLeafCount = 0;
            for (int i = 0; i < maximumJobCount; ++i)
            {
                ref var job = ref jobs.Span[i];
                job.LeafCount = i < remainder ? possibleLeavesPerJob + 1 : possibleLeavesPerJob;
                if (job.LeafCount > 0)
                {
                    job.LeafStart = jobbedLeafCount;
                    jobbedLeafCount += job.LeafCount;
                    job.JobLines.Count = 0;
                    job.MasterLinesSpan = lines.Span;
                    ++jobs.Count;
                }
                else
                    break;
            }
            this.broadPhase = broadPhase;
            masterLinesCount = lines.Count;
            looper.For(0, jobs.Count, workDelegate);
            lines.Count = masterLinesCount;
            this.broadPhase = null;
            jobs.Count = 0;

        }

    }
}
