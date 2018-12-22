using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using DemoContentLoader;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Demos.SpecializedTests
{
    static class HeadlessTest
    {
        public static void Test<T>(ContentArchive content, int runCount, int warmUpFrames, int frameCount) where T : Demo, new()
        {
            var runFrameTimes = new double[runCount];
            for (int runIndex = 0; runIndex < runCount; ++runIndex)
            {
                var demo = new T();
                demo.Initialize(content, new DemoRenderer.Camera(1, 1, 1, 1));
                GC.Collect(3, GCCollectionMode.Forced, true, true);
                for (int i = 0; i < warmUpFrames; ++i)
                {
                    demo.Update(null, null, null, 1 / 60f);
                }
                Console.WriteLine($"Warmup {runIndex} complete");
                double time = 0;
                int largestOverlapCount = 0;
                Console.Write("Completed frames: ");
                for (int i = 0; i < frameCount; ++i)
                {
                    CacheBlaster.Blast();
                    var start = Stopwatch.GetTimestamp();
                    demo.Update(null, null, null, 1 / 60f);
                    var end = Stopwatch.GetTimestamp();
                    time += (end - start) / (double)Stopwatch.Frequency;
                    if (i % 32 == 0)
                        Console.Write($"{i}, ");
                }
                Console.WriteLine();
                var frameTime = time / frameCount;
                Console.WriteLine($"Time per frame (ms): {1e3 * frameTime}, maximum overlap count: {largestOverlapCount}");
                runFrameTimes[runIndex] = frameTime;
                demo.Dispose();
            }
            var min = double.MaxValue;
            var max = double.MinValue;
            var sum = 0.0;
            var sumOfSquares = 0.0;
            for (int runIndex = 0; runIndex < runCount; ++runIndex)
            {
                var time = runFrameTimes[runIndex];
                min = Math.Min(time, min);
                max = Math.Max(time, max);
                sum += time;
                sumOfSquares += time * time;
            }
            var average = sum / runCount;
            var stdDev = Math.Sqrt(sumOfSquares / runCount - average * average);
            Console.WriteLine($"Average (ms): {average * 1e3}");
            Console.WriteLine($"Min, max (ms): {min * 1e3}, {max * 1e3}");
            Console.WriteLine($"Std Dev (ms): {stdDev * 1e3}");
        }
    }
}
