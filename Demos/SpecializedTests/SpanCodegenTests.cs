using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Demos.SpecializedTests
{
    public static class SpanCodegenTests
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static double Pointer(int size, int iterations)
        {
            var span = stackalloc int[size + 8];
            var start = Stopwatch.GetTimestamp();
            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                for (int i = 0; i < size; ++i)
                {
                    span[i] += span[i + 1];
                    span[i] += span[i + 2];
                    span[i] += span[i + 3];
                    span[i] += span[i + 4];
                    span[i] += span[i + 5];
                    span[i] += span[i + 6];
                    span[i] += span[i + 7];
                    span[i] += span[i + 8];
                }
            }
            var end = Stopwatch.GetTimestamp();
            return (double)(end - start) / Stopwatch.Frequency;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double Buffer(int size, int iterations)
        {
            var pool = new BufferPool();
            pool.SpecializeFor<int>().Take(size + 8, out var span);
            var start = Stopwatch.GetTimestamp();
            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                for (int i = 0; i < size; ++i)
                {
                    span[i] += span[i + 1];
                    span[i] += span[i + 2];
                    span[i] += span[i + 3];
                    span[i] += span[i + 4];
                    span[i] += span[i + 5];
                    span[i] += span[i + 6];
                    span[i] += span[i + 7];
                    span[i] += span[i + 8];
                }
            }
            var end = Stopwatch.GetTimestamp();
            pool.Clear();
            return (double)(end - start) / Stopwatch.Frequency;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double ArrayT(int size, int iterations)
        {
            var span = new Array<int>(new int[size + 8]);
            var start = Stopwatch.GetTimestamp();
            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                for (int i = 0; i < size; ++i)
                {
                    span[i] += span[i + 1];
                    span[i] += span[i + 2];
                    span[i] += span[i + 3];
                    span[i] += span[i + 4];
                    span[i] += span[i + 5];
                    span[i] += span[i + 6];
                    span[i] += span[i + 7];
                    span[i] += span[i + 8];
                }
            }
            var end = Stopwatch.GetTimestamp();
            return (double)(end - start) / Stopwatch.Frequency;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double Array(int size, int iterations)
        {
            var span = new int[size + 8];
            var start = Stopwatch.GetTimestamp();
            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                for (int i = 0; i < size; ++i)
                {
                    span[i] += span[i + 1];
                    span[i] += span[i + 2];
                    span[i] += span[i + 3];
                    span[i] += span[i + 4];
                    span[i] += span[i + 5];
                    span[i] += span[i + 6];
                    span[i] += span[i + 7];
                    span[i] += span[i + 8];
                }
            }
            var end = Stopwatch.GetTimestamp();
            return (double)(end - start) / Stopwatch.Frequency;
        }
        public static void Test()
        {
            const int size = 10000;
            const int iterations = 10000;

            Console.WriteLine($"Pointer Warmup Time: {Pointer(1, 1)}");
            Console.WriteLine($"Buffer Warmup Time: {Buffer(1, 1)}");
            Console.WriteLine($"ArrayT Warmup Time: {ArrayT(1, 1)}");
            Console.WriteLine($"Array Warmup Time: {Array(1, 1)}");

            Console.WriteLine($"Pointer Time: {Pointer(size, iterations)}");
            Console.WriteLine($"Buffer Time: {Buffer(size, iterations)}");
            Console.WriteLine($"ArrayT Time: {ArrayT(size, iterations)}");
            Console.WriteLine($"Array Time: {Array(size, iterations)}");


        }
    }
}