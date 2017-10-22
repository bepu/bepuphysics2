using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading;

namespace Demos.SpecializedTests
{
    public static class InterlockedOverheadTests
    {
        static int contestedAccumulator;
        static int[] uncontestedAccumulators;
        public static void Test()
        {
            const int iterations = 100000000;
            {
                int accumulator = 0;
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < iterations; ++i)
                {
                    Interlocked.Increment(ref accumulator);
                }
                var end = Stopwatch.GetTimestamp();
                Console.WriteLine($"Uncontested 1T increment (ns): {1e9 * (end - start) / (iterations * Stopwatch.Frequency)}");
            }

            var threadPool = new SimpleThreadDispatcher(Environment.ProcessorCount);
            {
                var start = Stopwatch.GetTimestamp();
                threadPool.DispatchWorkers(workerIndex =>
                {
                    while (true)
                    {
                        if (Interlocked.Increment(ref contestedAccumulator) >= iterations)
                            break;
                    }
                });
                var end = Stopwatch.GetTimestamp();
                Console.WriteLine($"Contested {threadPool.ThreadCount}T increment (ns): {1e9 * (end - start) / (iterations * Stopwatch.Frequency)}");
            }

            {
                uncontestedAccumulators = new int[Environment.ProcessorCount * 32];
                var start = Stopwatch.GetTimestamp();
                threadPool.DispatchWorkers(workerIndex =>
                {
                    var offset = workerIndex * 32;
                    var exit = iterations / threadPool.ThreadCount;
                    while (true)
                    {
                        if (Interlocked.Increment(ref uncontestedAccumulators[offset]) >= exit)
                            break;
                    }
                });
                var end = Stopwatch.GetTimestamp();
                Console.WriteLine($"Uncontested {threadPool.ThreadCount}T increment (ns): {1e9 * (end - start) / (iterations * Stopwatch.Frequency)}");
            }
        }
    }
}
