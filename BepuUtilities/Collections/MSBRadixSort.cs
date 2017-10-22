using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BepuUtilities.Collections
{

    public static class MSBRadixSort
    {
        //TODO: If the jit ever managed to handle ISpan indexers optimally, we could use a much more natural ISpan-based implementation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Swap<T>(ref T a, ref T b)
        {
            var temp = a;
            a = b;
            b = temp;
        }

        public static void SortU32<T>(ref int keys, ref T values, ref int bucketCounts, ref int bucketOriginalStartIndices, int keyCount, int shift)
        {
            if (keyCount < 32)
            {
                //There aren't many keys remaining. Use insertion sort.
                for (int i = 1; i < keyCount; ++i)
                {
                    var originalKey = Unsafe.Add(ref keys, i);
                    var originalValue = Unsafe.Add(ref values, i);
                    int compareIndex;
                    for (compareIndex = i - 1; compareIndex >= 0; --compareIndex)
                    {
                        if (originalKey < Unsafe.Add(ref keys, compareIndex))
                        {
                            //Move the element up one slot.
                            var upperSlotIndex = compareIndex + 1;
                            Unsafe.Add(ref keys, upperSlotIndex) = Unsafe.Add(ref keys, compareIndex);
                            Unsafe.Add(ref values, upperSlotIndex) = Unsafe.Add(ref values, compareIndex);
                        }
                        else
                            break;
                    }
                    var targetIndex = compareIndex + 1;
                    if (targetIndex != i)
                    {
                        //Move the original index down.
                        Unsafe.Add(ref keys, targetIndex) = originalKey;
                        Unsafe.Add(ref values, targetIndex) = originalValue;
                    }
                }
                return;
            }
            const int bucketCount = 256;
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            //The bucketCounts array passed into the root function must contain enough space to hold every level of the recursion, which is at maximum 1024 entries for 32 bit sorts.
            //Note that we have to clear the buckets here, since the previous values aren't guaranteed to be zero.
            //TODO: would be nice to do this faster. Hard to do with refs alone, but if we had more info about the datasource we could do initblock or something.
            for (int i = 0; i < bucketCount; ++i)
            {
                Unsafe.Add(ref bucketCounts, i) = 0;
            }
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref keys, i);
                ++Unsafe.Add(ref bucketCounts, (key >> shift) & 0xFF);
            }

            //Convert the bucket counts to partial sums.
            int sum = 0;
            for (int i = 0; i < bucketCount; ++i)
            {
                var previousSum = sum;
                ref var bucketSlotCount = ref Unsafe.Add(ref bucketCounts, i);
                sum += bucketSlotCount;
                //We store the partial sum into both the bucketCounts array and the originalStartIndices array.
                Unsafe.Add(ref bucketOriginalStartIndices, i) = previousSum;
                bucketSlotCount = previousSum;
            }

            //Note that the bucketCounts array is now really a startIndices array. We'll increment the bucketCounts version of the array as we perform swaps.
            //Walk through each bucket's region in the array, and walk through each element within each bucket. For each element, push it to the target bucket.
            //Rather than writing the displaced element into the local bucket, check where it should go. Repeat until an element that fits the current bucket is found.
            //Only upon finding the fitting element should anything be written to the local bucket slot. Note that the counting phase guarantees that an element will be found.
            //Every write into a target bucket should increment that bucket's start index.
            //When the number of elements visited reaches the size of the bucket, move on to the next bucket and start over.
            for (int i = 0; i < bucketCount; ++i)
            {
                ref var bucketStartIndex = ref Unsafe.Add(ref bucketCounts, i);
                //Note the use of the original start index. If it kept going beyond the original start, it will walk into an already sorted region.
                int nextStartIndex = i == bucketCount - 1 ? keyCount : Unsafe.Add(ref bucketOriginalStartIndices, i + 1);
                while (bucketStartIndex < nextStartIndex)
                {
                    var localKey = Unsafe.Add(ref keys, bucketStartIndex);
                    var localValue = Unsafe.Add(ref values, bucketStartIndex);
                    while (true)
                    {
                        var targetBucketIndex = (localKey >> shift) & 0xFF;
                        if (targetBucketIndex == i)
                        {
                            //The local key belongs to the local bucket. We can stop the swaps.
                            //Put the local key into the local bucket.
                            Unsafe.Add(ref keys, bucketStartIndex) = localKey;
                            Unsafe.Add(ref values, bucketStartIndex) = localValue;
                            ++bucketStartIndex;
                            break;
                        }
                        ref var targetBucketStartIndex = ref Unsafe.Add(ref bucketCounts, targetBucketIndex);
                        Debug.Assert((targetBucketIndex < 255 && targetBucketStartIndex < Unsafe.Add(ref bucketCounts, targetBucketIndex + 1)) ||
                            (targetBucketIndex == 255 && targetBucketStartIndex < keyCount));
                        ref var targetKeySlot = ref Unsafe.Add(ref keys, targetBucketStartIndex);
                        ref var targetValueSlot = ref Unsafe.Add(ref values, targetBucketStartIndex);
                        Swap(ref targetKeySlot, ref localKey);
                        Swap(ref targetValueSlot, ref localValue);
                        ++targetBucketStartIndex;
                    }
                }
            }


            //Now every bucket contains the elements that belong to it. There may still be sorting to do.
            if (shift > 0)
            {
                //There is at least one more level of sorting potentially required.
                var newShift = shift - 8;
                ref var nextLevelBucketCounts = ref Unsafe.Add(ref bucketCounts, bucketCount);

                var previousEnd = 0;
                for (int i = 0; i < bucketCount; ++i)
                {
                    var bucketEnd = Unsafe.Add(ref bucketCounts, i);
                    var count = bucketEnd - previousEnd;
                    if (count > 0)
                        SortU32(ref Unsafe.Add(ref keys, previousEnd), ref Unsafe.Add(ref values, previousEnd), ref nextLevelBucketCounts, ref bucketOriginalStartIndices, count, newShift);
                    previousEnd = bucketEnd;
                }
            }
        }





        public unsafe static void SortU32<T>(ref int keys, ref T values, int keyCount, int shift)
        {
            if (keyCount < 256)
            {
                var comparer = default(PrimitiveComparer<int>);
                QuickSort.Sort(ref keys, ref values, 0, keyCount - 1, ref comparer);
                return;
            }
            const int bucketCountPower = 7;
            const int bucketCount = 1 << bucketCountPower;
            const int mask = bucketCount - 1;
            //This stackalloc isn't actually super fast- the default behavior is to zero out the range. But we actually want it to be zeroed, so that's okay.
            var bucketCounts = stackalloc int[bucketCount];
#if RELEASESTRIP
            Unsafe.InitBlockUnaligned(bucketCounts, 0, sizeof(int) * bucketCount);
#endif
#if DEBUG
            for (int i = 0; i < bucketCount; ++i)
            {
                Debug.Assert(bucketCounts[i] == 0,
                    "It's really, really unlikely that they would choose to change the stackalloc init behavior without requiring an attribute or something, " +
                    "but if they do, everything breaks. If this ever gets hit, use Unsafe.InitBlock or similar.");
            }
#endif
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref keys, i);
                ++bucketCounts[(key >> shift) & mask];
            }

            //Unlike the bucketCounts, we don't want to pre-initialize these start indices.
            //But they will be, whether we like it or not. (Unless the compiler realizes that it's impossible for an uninitialized value to be read, but good luck with that.)
            //So, using large bucket counts with this isn't ideal. If you want larger bucket sizes, it's a good idea to preallocate the memory outside the call.
            var bucketOriginalStartIndices = stackalloc int[bucketCount];
            //Convert the bucket counts to partial sums.
            int sum = 0;
            for (int i = 0; i < bucketCount; ++i)
            {
                var previousSum = sum;
                ref var bucketSlotCount = ref bucketCounts[i];
                sum += bucketSlotCount;
                //We store the partial sum into both the bucketCounts array and the originalStartIndices array.
                bucketOriginalStartIndices[i] = previousSum;
                bucketSlotCount = previousSum;
            }

            //Note that the bucketCounts array is now really a startIndices array. We'll increment the bucketCounts version of the array as we perform swaps.
            //Walk through each bucket's region in the array, and walk through each element within each bucket. For each element, push it to the target bucket.
            //Rather than writing the displaced element into the local bucket, check where it should go. Repeat until an element that fits the current bucket is found.
            //Only upon finding the fitting element should anything be written to the local bucket slot. Note that the counting phase guarantees that an element will be found.
            //Every write into a target bucket should increment that bucket's start index.
            //When the number of elements visited reaches the size of the bucket, move on to the next bucket and start over.
            for (int i = 0; i < bucketCount; ++i)
            {
                ref var bucketStartIndex = ref bucketCounts[i];
                //Note the use of the original start index. If it kept going beyond the original start, it will walk into an already sorted region.
                int nextStartIndex = i == bucketCount - 1 ? keyCount : bucketOriginalStartIndices[i + 1];
                while (bucketStartIndex < nextStartIndex)
                {
                    var localKey = Unsafe.Add(ref keys, bucketStartIndex);
                    var localValue = Unsafe.Add(ref values, bucketStartIndex);
                    while (true)
                    {
                        var targetBucketIndex = (localKey >> shift) & mask;
                        if (targetBucketIndex == i)
                        {
                            //The local key belongs to the local bucket. We can stop the swaps.
                            //Put the local key into the local bucket.
                            Unsafe.Add(ref keys, bucketStartIndex) = localKey;
                            Unsafe.Add(ref values, bucketStartIndex) = localValue;
                            ++bucketStartIndex;
                            break;
                        }
                        ref var targetBucketStartIndex = ref bucketCounts[targetBucketIndex];
                        Debug.Assert((targetBucketIndex < bucketCount - 1 && targetBucketStartIndex < bucketCounts[targetBucketIndex + 1]) ||
                            (targetBucketIndex == bucketCount - 1 && targetBucketStartIndex < keyCount));
                        ref var targetKeySlot = ref Unsafe.Add(ref keys, targetBucketStartIndex);
                        ref var targetValueSlot = ref Unsafe.Add(ref values, targetBucketStartIndex);
                        Swap(ref targetKeySlot, ref localKey);
                        Swap(ref targetValueSlot, ref localValue);
                        ++targetBucketStartIndex;
                    }
                }
            }


            //Now every bucket contains the elements that belong to it. There may still be sorting to do.
            if (shift > 0)
            {
                //There is at least one more level of sorting potentially required.
                var newShift = Math.Max(0, shift - bucketCountPower);

                var previousEnd = 0;
                for (int i = 0; i < bucketCount; ++i)
                {
                    var bucketEnd = bucketCounts[i];
                    var count = bucketEnd - previousEnd;
                    if (count > 0)
                        SortU32(ref Unsafe.Add(ref keys, previousEnd), ref Unsafe.Add(ref values, previousEnd), count, newShift);
                    previousEnd = bucketEnd;
                }
            }
        }

    }
}
