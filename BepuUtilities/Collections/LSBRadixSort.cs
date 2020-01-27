using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BepuUtilities.Collections
{
    public static class LSBRadixSort
    {
        //TODO: If the jit ever managed to handle ISpan indexers optimally, we could use a much more natural ISpan-based implementation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ReorderForByte<T>(ref int sourceKeys, ref int targetKeys, ref T sourceValues, ref T targetValues, int keyCount, ref int indices, int shift)
        {
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref sourceKeys, i);
                ref var bucketStartIndex = ref Unsafe.Add(ref indices, (key >> shift) & 0xFF);
                Unsafe.Add(ref targetKeys, bucketStartIndex) = key;
                Unsafe.Add(ref targetValues, bucketStartIndex) = Unsafe.Add(ref sourceValues, i);
                //Bump the index up to compensate for the new element.
                ++bucketStartIndex;
            }
        }

        public static void SortU32<T>(ref int keys, ref T values, ref int keysScratch, ref T valuesScratch, ref int bucketCounts, int keyCount)
        {
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            ref var byte1Counts = ref Unsafe.Add(ref bucketCounts, 256);
            ref var byte2Counts = ref Unsafe.Add(ref bucketCounts, 512);
            ref var byte3Counts = ref Unsafe.Add(ref bucketCounts, 768);

            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref keys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
                ++Unsafe.Add(ref byte1Counts, (key >> 8) & 0xFF);
                ++Unsafe.Add(ref byte2Counts, (key >> 16) & 0xFF);
                ++Unsafe.Add(ref byte3Counts, key >> 24);
            }

            //Convert the bucket counts to partial sums.
            int sum0 = 0;
            int sum1 = 0;
            int sum2 = 0;
            int sum3 = 0;
            for (int i = 0; i < 256; ++i)
            {
                var previousSum0 = sum0;
                var previousSum1 = sum1;
                var previousSum2 = sum2;
                var previousSum3 = sum3;
                ref var byte0 = ref Unsafe.Add(ref bucketCounts, i);
                ref var byte1 = ref Unsafe.Add(ref byte1Counts, i);
                ref var byte2 = ref Unsafe.Add(ref byte2Counts, i);
                ref var byte3 = ref Unsafe.Add(ref byte3Counts, i);
                sum0 += byte0;
                sum1 += byte1;
                sum2 += byte2;
                sum3 += byte3;
                byte0 = previousSum0;
                byte1 = previousSum1;
                byte2 = previousSum2;
                byte3 = previousSum3;
            }

            ReorderForByte(ref keys, ref keysScratch, ref values, ref valuesScratch, keyCount, ref bucketCounts, 0);
            ReorderForByte(ref keysScratch, ref keys, ref valuesScratch, ref values, keyCount, ref byte1Counts, 8);
            ReorderForByte(ref keys, ref keysScratch, ref values, ref valuesScratch, keyCount, ref byte2Counts, 16);
            ReorderForByte(ref keysScratch, ref keys, ref valuesScratch, ref values, keyCount, ref byte3Counts, 24);
        }

        public static void SortU24<T>(ref int inputKeys, ref T inputValues, ref int outputKeys, ref T outputValues, ref int bucketCounts, int keyCount)
        {
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            ref var byte1Counts = ref Unsafe.Add(ref bucketCounts, 256);
            ref var byte2Counts = ref Unsafe.Add(ref bucketCounts, 512);

            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref inputKeys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
                ++Unsafe.Add(ref byte1Counts, (key >> 8) & 0xFF);
                ++Unsafe.Add(ref byte2Counts, (key >> 16) & 0xFF);
            }

            //Convert the bucket counts to partial sums.
            int sum0 = 0;
            int sum1 = 0;
            int sum2 = 0;
            for (int i = 0; i < 256; ++i)
            {
                var previousSum0 = sum0;
                var previousSum1 = sum1;
                var previousSum2 = sum2;
                ref var byte0 = ref Unsafe.Add(ref bucketCounts, i);
                ref var byte1 = ref Unsafe.Add(ref byte1Counts, i);
                ref var byte2 = ref Unsafe.Add(ref byte2Counts, i);
                sum0 += byte0;
                sum1 += byte1;
                sum2 += byte2;
                byte0 = previousSum0;
                byte1 = previousSum1;
                byte2 = previousSum2;
            }

            ReorderForByte(ref inputKeys, ref outputKeys, ref inputValues, ref outputValues, keyCount, ref bucketCounts, 0);
            ReorderForByte(ref outputKeys, ref inputKeys, ref outputValues, ref inputValues, keyCount, ref byte1Counts, 8);
            ReorderForByte(ref inputKeys, ref outputKeys, ref inputValues, ref outputValues, keyCount, ref byte2Counts, 16);
        }

        public static void SortU16<T>(ref int keys, ref T values, ref int keysScratch, ref T valuesScratch, ref int bucketCounts, int keyCount)
        {
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            ref var byte1Counts = ref Unsafe.Add(ref bucketCounts, 256);

            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref keys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
                ++Unsafe.Add(ref byte1Counts, (key >> 8) & 0xFF);
            }

            //Convert the bucket counts to partial sums.
            int sum0 = 0;
            int sum1 = 0;
            for (int i = 0; i < 256; ++i)
            {
                var previousSum0 = sum0;
                var previousSum1 = sum1;
                ref var byte0 = ref Unsafe.Add(ref bucketCounts, i);
                ref var byte1 = ref Unsafe.Add(ref byte1Counts, i);
                sum0 += byte0;
                sum1 += byte1;
                byte0 = previousSum0;
                byte1 = previousSum1;
            }

            ReorderForByte(ref keys, ref keysScratch, ref values, ref valuesScratch, keyCount, ref bucketCounts, 0);
            ReorderForByte(ref keysScratch, ref keys, ref valuesScratch, ref values, keyCount, ref byte1Counts, 8);
        }

        public static void SortU8<T>(ref int inputKeys, ref T inputValues, ref int outputKeys, ref T outputValues, ref int bucketCounts, int keyCount)
        {
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref inputKeys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
            }

            //Convert the bucket counts to partial sums.
            int sum0 = 0;
            for (int i = 0; i < 256; ++i)
            {
                var previousSum0 = sum0;
                ref var byte0 = ref Unsafe.Add(ref bucketCounts, i);
                sum0 += byte0;
                byte0 = previousSum0;
            }

            ReorderForByte(ref inputKeys, ref outputKeys, ref inputValues, ref outputValues, keyCount, ref bucketCounts, 0);
        }

        /// <summary>
        /// Sorts a set of keys and their associated values using radix sort.
        /// </summary>
        /// <remarks>Only one invocation of the sort can be running at a time on a given instance of the sorter.</remarks>
        /// <typeparam name="TValue">Type of the values to sort.</typeparam>
        /// <typeparam name="TKeySpan">Type of the span that holds the keys to sort.</typeparam>
        /// <typeparam name="TValueSpan">Type of the span that holds the values to sort.</typeparam>
        /// <param name="inputKeys">Span containing the keys to sort.</param>
        /// <param name="inputValues">Span containing the values to sort.</param>
        /// <param name="scratchKeys">Scratch array to write temporary results into.</param>
        /// <param name="scratchValues">Scratch array to write temporary results into.</param>
        /// <param name="startIndex">Start location of the sort.</param>
        /// <param name="count">Number of elements, including the start index, to sort.</param>
        /// <param name="keysUpperBound">Value equal to or greater than the value of any key within the sorted key set. Tighter bounds can allow faster execution.</param>
        /// <param name="bufferPool">Pool to pull temporary buffers from.</param>
        /// <param name="sortedKeys">Span containing the sorted keys. Will be either the input keys span or scratchKeys span depending on the keysUpperBound.</param>
        /// <param name="sortedValues">Span containing the sorted values. Will be either the input values span or scratchValues span depending on the keysUpperBound.
        /// Only the region defined by the startIndex and count is modified; the remainder of the span (whether it be the input span or the scratch span) is unmodified.</param>
        public static void Sort<TValue>(ref Buffer<int> inputKeys, ref Buffer<TValue> inputValues, ref Buffer<int> scratchKeys, ref Buffer<TValue> scratchValues,
            int startIndex, int count, int keysUpperBound, BufferPool bufferPool, out Buffer<int> sortedKeys, out Buffer<TValue> sortedValues)
            where TValue : unmanaged
        {
            //Note that we require the scratch and input spans to contain the offset region. That's because the output could be either the scratch or the input spans.
            Debug.Assert(
                inputKeys.Length >= startIndex + count && inputValues.Length >= startIndex + count &&
                inputKeys.Length >= startIndex + count && inputValues.Length >= startIndex + count,
                "The spans must be able to hold the sort region.");
#if DEBUG
            for (int i = startIndex; i < startIndex + count; ++i)
            {
                //Technically, there are instances where someone may want to force a smaller sort... but... that's extremely rare, and there are other options available.
                Debug.Assert(inputKeys[i] <= keysUpperBound, "The given upper bound must actually bound the elements in the sort region, or else the sort won't sort.");
            }
#endif
            int bucketSetCount = keysUpperBound < (1 << 16) ? keysUpperBound < (1 << 8) ? 1 : 2 : keysUpperBound < (1 << 24) ? 3 : 4;
            bufferPool.Take<int>(bucketSetCount * 256, out var bucketCounts);
            unsafe
            {
                //The bucket counts will be added to, so they need to be zeroed.
                Unsafe.InitBlockUnaligned(bucketCounts.Memory, 0, (uint)(sizeof(int) * 256 * bucketSetCount));
            }

            switch (bucketSetCount)
            {
                case 1:
                    {
                        SortU8(ref inputKeys[startIndex], ref inputValues[startIndex], ref scratchKeys[startIndex], ref scratchValues[startIndex], ref bucketCounts[0], count);
                        sortedKeys = scratchKeys;
                        sortedValues = scratchValues;
                    }
                    break;
                case 2:
                    {
                        SortU16(ref inputKeys[startIndex], ref inputValues[startIndex], ref scratchKeys[startIndex], ref scratchValues[startIndex], ref bucketCounts[0], count);
                        sortedKeys = inputKeys;
                        sortedValues = inputValues;
                    }
                    break;
                case 3:
                    {
                        SortU24(ref inputKeys[startIndex], ref inputValues[startIndex], ref scratchKeys[startIndex], ref scratchValues[startIndex], ref bucketCounts[0], count);
                        sortedKeys = scratchKeys;
                        sortedValues = scratchValues;
                    }
                    break;
                case 4:
                    {
                        SortU32(ref inputKeys[startIndex], ref inputValues[startIndex], ref scratchKeys[startIndex], ref scratchValues[startIndex], ref bucketCounts[0], count);
                        sortedKeys = inputKeys;
                        sortedValues = inputValues;
                    }
                    break;
                default:
                    Debug.Fail("Invalid bucket set count.");
                    sortedKeys = default;
                    sortedValues = default;
                    break;
            }

            bufferPool.Return(ref bucketCounts);
        }

    }
}
