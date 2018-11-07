using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class MarshalMemoryTests
    {
        static void Test(string name, int iterationCount)
        {
            var random = new Random(5);
            const int targetAllocationCount = 16384;
            var pointers = new List<IntPtr>(targetAllocationCount);
            var minimumAllocation = double.MaxValue;
            var maximumAllocation = -double.MaxValue;
            var minimumDeallocation = double.MaxValue;
            var maximumDeallocation = -double.MaxValue;
            var allocationSum = 0.0;
            var deallocationSum = 0.0;
            for (int i = 0; i < iterationCount; ++i)
            {
                var shouldAdd = (targetAllocationCount - pointers.Count) / (double)targetAllocationCount;
                var roll = random.NextDouble();
                if (roll <= shouldAdd)
                {
                    var start = Stopwatch.GetTimestamp();
                    pointers.Add(Marshal.AllocHGlobal(131072));
                    var stop = Stopwatch.GetTimestamp();
                    var time = (stop - start) / (double)Stopwatch.Frequency;
                    if (time > maximumAllocation)
                        maximumAllocation = time;
                    if (time < minimumAllocation)
                        minimumAllocation = time;
                    allocationSum += time;
                }
                else
                {
                    var index = random.Next(pointers.Count);
                    var pointer = pointers[index];
                    var start = Stopwatch.GetTimestamp();
                    Marshal.FreeHGlobal(pointer);
                    var stop = Stopwatch.GetTimestamp();
                    var time = (stop - start) / (double)Stopwatch.Frequency;
                    pointers.RemoveAt(index);
                    if (time > maximumDeallocation)
                        maximumDeallocation = time;
                    if (time < minimumDeallocation)
                        minimumDeallocation = time;
                    deallocationSum += time;
                }
            }
            for (int i = 0; i < pointers.Count; ++i)
            {
                Marshal.FreeHGlobal(pointers[i]);
            }

            Console.WriteLine($"{name} time (us) per allocation, average: {allocationSum * 1e6 / iterationCount}, min: {minimumAllocation * 1e6}, max: {maximumAllocation * 1e6}");
            Console.WriteLine($"{name} time (us) per deallocation, average: {deallocationSum * 1e6 / iterationCount}, min: {minimumDeallocation * 1e6}, max: {maximumDeallocation * 1e6}");


        }
        public static void Test()
        {
            Test("warmup", 128);
            Test("actual", 1 << 20);
        }
    }
}
