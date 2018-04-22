using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace Demos.SpecializedTests
{
    static class HeadlessDemo
    {
        public static void Test<T>(int warmUpFrames, int frameCount) where T : Demo, new()
        {
            var demo = new T();
            demo.Initialize(new DemoRenderer.Camera(1, 1, 1, 1));
            for (int i = 0; i < warmUpFrames; ++i)
            {
                demo.Update(null, 1 / 60f);
            }
            double time = 0;
            int largestOverlapCount = 0;
            for (int i = 0; i < frameCount; ++i)
            {
                //CacheBlaster.Blast();
                var start = Stopwatch.GetTimestamp();
                demo.Update(null, 1 / 60f);
                var end = Stopwatch.GetTimestamp();
                time += (end - start) / (double)Stopwatch.Frequency;
                Console.WriteLine($"FRAME {i}");
            }
            Console.WriteLine($"Time per frame (us): {1e6 * time / frameCount}, maximum overlap count: {largestOverlapCount}");
            demo.Dispose();
        }
    }
}
