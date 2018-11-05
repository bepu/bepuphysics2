using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BEPUutilitiesTests
{
    public static class SpanCodeGenTests
    {

        [MethodImpl(MethodImplOptions.NoInlining)]
        static double Test<TSpan>(ref TSpan span, int iterations, out int accumulator) where TSpan : ISpan<int>
        {
            var start = Stopwatch.GetTimestamp();
            accumulator = 0;
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < span.Length; ++j)
                {
                    if (span[j] * 73 == 1)
                        break;
                    else
                        span[j] = 0;
                }
                //for (int j = 10; j < span.Length; ++j)
                //{
                //    accumulator += span[j - 1] + span[j];
                //    accumulator += span[j - 2] + span[j];
                //    accumulator += span[j - 3] + span[j];
                //    accumulator += span[j - 4] + span[j];
                //    accumulator += span[j - 5] + span[j];
                //    accumulator += span[j - 6] + span[j];
                //    accumulator += span[j - 7] + span[j];
                //    accumulator += span[j - 8] + span[j];
                //    accumulator += span[j - 9] + span[j];
                //    accumulator += span[j - 10] + span[j];

                //}
            }

            return (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static double RawPointerTest(int* span, int spanLength, int iterations, out int accumulator)
        {
            var start = Stopwatch.GetTimestamp();
            accumulator = 0;
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < spanLength; ++j)
                {
                    if (span[j] * 73 == 1)
                        break;
                    else
                        span[j] = 0;
                }
                //for (int j = 10; j < spanLength; ++j)
                //{
                //    accumulator += span[j - 1] + span[j];
                //    accumulator += span[j - 2] + span[j];
                //    accumulator += span[j - 3] + span[j];
                //    accumulator += span[j - 4] + span[j];
                //    accumulator += span[j - 5] + span[j];
                //    accumulator += span[j - 6] + span[j];
                //    accumulator += span[j - 7] + span[j];
                //    accumulator += span[j - 8] + span[j];
                //    accumulator += span[j - 9] + span[j];
                //    accumulator += span[j - 10] + span[j];

                //}
            }

            return (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;

        }
        public static unsafe void Test()
        {
            var array = new int[32];

            fixed (int* pointer = array)
            {
                Buffer<int> pSpan = new Buffer<int>(pointer, array.Length);
                Array<int> mSpan = new Array<int>(array);

                Console.WriteLine(
                    $"Warmup: {Test(ref pSpan, 1, out int accumulator)}," +
                    $" {Test(ref mSpan, 1, out accumulator)}" +
                    $" {RawPointerTest(pointer, array.Length, 1, out accumulator)}");
                const int iterations = 20000000;
                Console.WriteLine($"Pointer: {Test(ref pSpan, iterations, out accumulator)}");
                Console.WriteLine($"Managed: {Test(ref mSpan, iterations, out accumulator)}");

                Console.WriteLine($"Raw: {RawPointerTest(pointer, array.Length, iterations, out accumulator)}");
            }
        }
    }
}
