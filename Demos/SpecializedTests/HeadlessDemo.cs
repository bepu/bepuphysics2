using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Demos.SpecializedTests
{
    static class HeadlessDemo
    {
        public static void Simple()
        {
            var simpleDemo = new SimpleDemo();
            simpleDemo.Initialize(new DemoRenderer.Camera(1, 1, 1, 1));
            for (int i = 0; i < 128; ++i)
            {
                simpleDemo.Update(null, 1 / 60f);
            }
            double time = 0;
            const int frameCount = 1000;
            int largestOverlapCount = 0;
            for (int i = 0; i < frameCount; ++i)
            {
                CacheBlaster.Blast();
                var start = Stopwatch.GetTimestamp();
                simpleDemo.Update(null, 1 / 60f);
                var end = Stopwatch.GetTimestamp();
                time += (end - start) / (double)Stopwatch.Frequency;
                var overlapCount = simpleDemo.Simulation.NarrowPhase.PairCache.Mapping.Count;
                if (overlapCount > largestOverlapCount)
                {
                    largestOverlapCount = overlapCount;
                }
                //Console.WriteLine($"FRAME {i}, time (us): {1e6 * simpleDemo.Simulation.Timings[simpleDemo.Simulation.NarrowPhase]}");
                Console.WriteLine($"FRAME {i}");
            }
            Console.WriteLine($"Time per frame (us): {1e6 * time / frameCount}, maximum overlap count: {largestOverlapCount}");
            simpleDemo.Dispose();
        }
    }
}
