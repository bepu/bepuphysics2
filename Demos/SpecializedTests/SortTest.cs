using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Demos.SpecializedTests
{
    public static class SortTest
    {

        struct Comparer : IComparerRef<int>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref int a, ref int b)
            {
                return a < b ? -1 : a > b ? 1 : 0;
            }
        }

        static void VerifySort(ref Buffer<int> keys)
        {
            for (int i = 1; i < keys.Length; ++i)
            {
                Debug.Assert(keys[i] >= keys[i - 1]);
            }
        }
        public static void Test()
        {
            const int elementCount = 65536;
            const int elementExclusiveUpperBound = 1 << 16;

            var bufferPool = new BufferPool();
            bufferPool.Take<int>(elementCount, out var keys);
            bufferPool.Take<int>(elementCount, out var indexMap);      
            bufferPool.Take<int>(elementCount, out var keys2);
            bufferPool.Take<int>(elementCount, out var indexMap2);
            bufferPool.Take<int>(elementCount, out var keys3);
            bufferPool.Take<int>(elementCount, out var indexMap3);
            bufferPool.Take<int>(elementCount, out var keys4);
            bufferPool.Take<int>(elementCount, out var indexMap4);
            Random random = new Random(5);
         
            for (int iteration = 0; iteration < 4; ++iteration)
            {
                for (int i = 0; i < elementCount; ++i)
                {
                    indexMap[i] = i;
                    //keys[i] = i / (elementCount / elementExclusiveUpperBound);
                    //keys[i] = i % elementExclusiveUpperBound;
                    //keys[i] = i;
                    keys[i] = random.Next(elementExclusiveUpperBound);
                }
                keys.CopyTo(0, ref keys2, 0, elementCount);
                keys.CopyTo(0, ref keys3, 0, elementCount);
                keys.CopyTo(0, ref keys4, 0, elementCount);
                indexMap.CopyTo(0, ref indexMap2, 0, elementCount);
                indexMap.CopyTo(0, ref indexMap3, 0, elementCount);
                indexMap.CopyTo(0, ref indexMap4, 0, elementCount);
                var timer = Stopwatch.StartNew();

                var keysScratch = new int[elementCount];
                var valuesScratch = new int[elementCount];
                var bucketCounts = new int[1024];
                for (int t = 0; t < 16; ++t)
                {
                    var comparer = new Comparer();
                    timer.Restart();
                    QuickSort.Sort(ref keys[0], ref indexMap[0], 0, elementCount - 1, ref comparer);
                    //QuickSort.Sort2(ref keys[0], ref indexMap[0], 0, elementCount - 1, ref comparer);
                    timer.Stop();
                    VerifySort(ref keys);
                    Console.WriteLine($"QuickSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                    //timer.Restart();
                    //Array.Sort(keys2.Memory, indexMap2.Memory, 0, elementCount);
                    //timer.Stop();
                    //VerifySort(ref keys2);
                    //Console.WriteLine($"Array.Sort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                    timer.Restart();
                    Array.Clear(bucketCounts, 0, bucketCounts.Length);
                    LSBRadixSort.SortU16(ref keys3[0], ref indexMap3[0], ref keysScratch[0], ref valuesScratch[0], ref bucketCounts[0], elementCount);
                    timer.Stop();
                    VerifySort(ref keys3);
                    Console.WriteLine($"{t} LSBRadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                    var originalIndices = new int[256];
                    timer.Restart();
                    //MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], ref bucketCounts[0], ref originalIndices[0], elementCount, 24);
                    MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], elementCount, SpanHelper.GetContainingPowerOf2(elementExclusiveUpperBound));
                    timer.Stop();
                    VerifySort(ref keys4);
                    Console.WriteLine($"{t} MSBRadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");
                }
            }
            bufferPool.Clear();

        }
    }
}
