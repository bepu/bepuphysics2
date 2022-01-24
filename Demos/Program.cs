using BepuUtilities;
using DemoContentLoader;
using Demos.Demos;
using Demos.Demos.Cars;
using Demos.SpecializedTests;
using DemoUtilities;
using OpenTK;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace Demos
{
    class Program
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void Test(int testCount)
        {
            int workerCount = 512;
            var dispatcher = new SimpleThreadDispatcher2(workerCount);
            long[] slots = new long[workerCount];

            long total = 0;
            long expectedSum = 0;
            for (int q = 0; q < testCount; ++q)
            {
                var start = Stopwatch.GetTimestamp();
                dispatcher.DispatchWorkers(i => { slots[i] += q * i; }, workerCount);
                var stop = Stopwatch.GetTimestamp();
                total += stop - start;

                long sum = 0;
                for (int i = 0; i < workerCount; ++i)
                {
                    sum += slots[i];
                    expectedSum += q * i;
                }
                Console.WriteLine($"Sum: {sum}, expected: {expectedSum}");
            }
            dispatcher.Dispose();
            Console.WriteLine($"Time per dispatch (us): {total * 1e6 / (testCount * Stopwatch.Frequency)}");
        }
        static void Main(string[] args)
        {
            //for (int i = 0; i < 32; ++i)
            //{
            //    Test(32);
            //}
            //Test(1024);
            //return;

            var window = new Window("pretty cool multicolored window",
                new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = typeof(Program).Assembly.GetManifestResourceStream("Demos.Demos.contentarchive"))
            {
                content = ContentArchive.Load(stream);
            }
            //DeterminismTest<FountainStressTestDemo>.Test(content, 2, 32768);
            //HeadlessTest.Test<CarDemo>(content, 4, 64, 512);
            var demo = new DemoHarness(loop, content);
            loop.Run(demo);
            loop.Dispose();
            window.Dispose();
        }
    }
}