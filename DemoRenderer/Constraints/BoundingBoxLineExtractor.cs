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
        internal struct ThreadJob
        {
            public int SimulationIndex;
            public int LeafStart;
            public int LeafCount;
            public int TargetLineStart;
            public bool CoversActiveCollidables;
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
        internal unsafe void ExecuteJob(Buffer<LineInstance> lines, Simulation simulation, ThreadJob job)
        {
            var end = job.LeafStart + job.LeafCount;
            var lineCount = 12 * job.LeafCount;
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
                    simulation.BroadPhase.GetActiveBoundsPointers(broadPhaseIndex, out min, out max);
                else
                    simulation.BroadPhase.GetStaticBoundsPointers(broadPhaseIndex, out min, out max);
                var outputStartIndex = job.TargetLineStart + i * 12;
                WriteBoundsLines(*min, *max, packedColor, packedBackgroundColor, ref lines[outputStartIndex]);
            }
        }

        void CreateJobsForTree(in Tree tree, bool active, int simulationIndex, ref QuickList<ThreadJob> jobs, ref int newLinesCount, ref int nextStart, BufferPool pool)
        {
            var maximumJobCount = 4 * Environment.ProcessorCount;
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
                    job.SimulationIndex = simulationIndex;
                    job.LeafCount = jobLeafCount;
                    job.LeafStart = jobbedLeafCount;
                    job.CoversActiveCollidables = active;
                    job.TargetLineStart = nextStart;
                    jobbedLeafCount += jobLeafCount;
                    newLinesCount += jobLeafCount * 12;
                    nextStart += jobLeafCount * 12;
                }
                else
                    break;
            }
        }

        internal unsafe void CreateJobs(Simulation simulation, int simulationIndex, ref QuickList<LineInstance> lines, ref QuickList<ThreadJob> jobs, BufferPool pool)
        {
            //For now, we only pull the bounding boxes of objects that are active.
            lines.EnsureCapacity(lines.Count + 12 * (simulation.BroadPhase.ActiveTree.LeafCount + simulation.BroadPhase.StaticTree.LeafCount), pool);
            int newLinesCount = 0;
            int nextStart = lines.Count;
            CreateJobsForTree(simulation.BroadPhase.ActiveTree, true, simulationIndex, ref jobs, ref newLinesCount, ref nextStart, pool);
            CreateJobsForTree(simulation.BroadPhase.StaticTree, false, simulationIndex, ref jobs, ref newLinesCount, ref nextStart, pool);
            lines.Allocate(newLinesCount, pool);
        }

    }
}
