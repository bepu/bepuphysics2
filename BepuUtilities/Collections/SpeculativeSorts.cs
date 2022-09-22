using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using System.Text;
using System.Threading.Tasks;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Specialized sorts that can go fast sometimes, but often come with special requirements.
    /// </summary>
    public static class VectorizedSorts
    {
        /// <summary>
        /// Computes the mapping from current index to sorted index for all elements in the paddedKeys. The paddedKeys buffer is not actually sorted.
        /// </summary>
        /// <param name="paddedKeys">Padded keys to compute sorted indices for. This buffer's contents will not be mutated.
        /// The buffer must be at least as long as the targetIndices buffer and must be padded to be divisible by vector length.
        /// Vector length is 8 elements if <see cref="Vector256.IsHardwareAccelerated"/> or 4 elements if only <see cref="Vector128.IsHardwareAccelerated"/>.</param>
        /// <param name="targetIndices">Buffer to be filled with the target indices each slot should be moved to in order to sort the buffer.</param>
        public unsafe static void VectorCountingSort(Buffer<float> paddedKeys, Buffer<int> targetIndices)
        {
            if (Vector256.IsHardwareAccelerated)
            {
                Debug.Assert(paddedKeys.length >= targetIndices.length && paddedKeys.length % 8 == 0, "This implementation assumes the keys are pre-padded.");
                var elementCount = targetIndices.length;
                var indexOffsets = Vector256.Create(0, 1, 2, 3, 4, 5, 6, 7);
                for (int i = 0; i < elementCount; ++i)
                {
                    //Broadcast the current value and test it against all other values.
                    //Count all values smaller than the current value, and all equal values that are in a lower index.
                    var previousCount = 0;
                    var value = Vector256.Create(paddedKeys[i]);
                    var currentIndex = Vector256.Create(i);

                    int testedSlots;
                    for (testedSlots = 0; testedSlots < i; testedSlots += 8)
                    {
                        var testVector = Vector256.Load(paddedKeys.Memory + testedSlots);
                        var vectorIndices = indexOffsets + Vector256.Create(testedSlots);
                        var indexIsLesser = Vector256.LessThan(vectorIndices, currentIndex);
                        var slotIsEqual = Vector256.Equals(testVector, value);
                        var slotIsLesser = Vector256.LessThan(testVector, value);
                        var slotPrecedesValue = Vector256.BitwiseOr(slotIsLesser, Vector256.BitwiseAnd(slotIsEqual, indexIsLesser.AsSingle()));
                        var slotPrecedesValueMask = Vector256.ExtractMostSignificantBits(slotPrecedesValue.AsSingle());
                        var slotsPrecedingValueCount = BitOperations.PopCount(slotPrecedesValueMask);

                        previousCount += slotsPrecedingValueCount;
                    }

                    //For any slots after the query slot, there will be no 'lesser index but equal' entries, so the loop is simpler.
                    for (; testedSlots < paddedKeys.Length; testedSlots += 8)
                    {
                        var testVector = Vector256.Load(paddedKeys.Memory + testedSlots);
                        var slotPrecedesValue = Vector256.LessThan(testVector, value);
                        var slotPrecedesValueMask = Vector256.ExtractMostSignificantBits(slotPrecedesValue.AsSingle());
                        var slotsPrecedingValueCount = BitOperations.PopCount(slotPrecedesValueMask);

                        previousCount += slotsPrecedingValueCount;
                    }
                    targetIndices[i] = previousCount;
                }
            }
            else if (Vector128.IsHardwareAccelerated)
            {
                Debug.Assert(paddedKeys.length >= targetIndices.length && paddedKeys.length % 4 == 0, "This implementation assumes the keys are pre-padded.");
                var elementCount = targetIndices.length;
                var indexOffsets = Vector128.Create(0, 1, 2, 3);
                for (int i = 0; i < elementCount; ++i)
                {
                    //Broadcast the current value and test it against all other values.
                    //Count all values smaller than the current value, and all equal values that are in a lower index.
                    var previousCount = 0;
                    var value = Vector128.Create(paddedKeys[i]);
                    var currentIndex = Vector128.Create(i);

                    int testedSlots;
                    for (testedSlots = 0; testedSlots < i; testedSlots += 4)
                    {
                        var testVector = Vector128.Load(paddedKeys.Memory + testedSlots);
                        var vectorIndices = indexOffsets + Vector128.Create(testedSlots);
                        var indexIsLesser = Vector128.LessThan(vectorIndices, currentIndex);
                        var slotIsEqual = Vector128.Equals(testVector, value);
                        var slotIsLesser = Vector128.LessThan(testVector, value);
                        var slotPrecedesValue = Vector128.BitwiseOr(slotIsLesser, Vector128.BitwiseAnd(slotIsEqual, indexIsLesser.AsSingle()));
                        var slotPrecedesValueMask = Vector128.ExtractMostSignificantBits(slotPrecedesValue.AsSingle());
                        var slotsPrecedingValueCount = BitOperations.PopCount(slotPrecedesValueMask);

                        previousCount += slotsPrecedingValueCount;
                    }

                    //For any slots after the query slot, there will be no 'lesser index but equal' entries, so the loop is simpler.
                    for (; testedSlots < paddedKeys.Length; testedSlots += 4)
                    {
                        var testVector = Vector128.Load(paddedKeys.Memory + testedSlots);
                        var slotPrecedesValue = Vector128.LessThan(testVector, value);
                        var slotPrecedesValueMask = Vector128.ExtractMostSignificantBits(slotPrecedesValue.AsSingle());
                        var slotsPrecedingValueCount = BitOperations.PopCount(slotPrecedesValueMask);

                        previousCount += slotsPrecedingValueCount;
                    }
                    targetIndices[i] = previousCount;
                }
            }
            else
            {
                //Shrug! Nonvector fallback. Really shouldn't call a function called VectorCountingSort if you don't have any vectorization.
                for (int i = 0; i < targetIndices.Length; ++i)
                {
                    targetIndices[i] = i;
                }
                var comparer = new PrimitiveComparer<float>();
                QuickSort.Sort(ref paddedKeys[0], ref targetIndices[0], 0, targetIndices.Length - 1, ref comparer);
            }

        }


        /// <summary>
        /// Computes the mapping from current index to sorted index for all elements in the paddedKeys. The paddedKeys buffer is not actually sorted.
        /// </summary>
        /// <param name="paddedKeys">Padded keys to compute sorted indices for. This buffer's contents will not be mutated.
        /// The buffer must be at least as long as the targetIndices buffer and must be padded to be divisible by vector length.
        /// Vector length is 8 elements if <see cref="Vector256.IsHardwareAccelerated"/> or 4 elements if only <see cref="Vector128.IsHardwareAccelerated"/>.</param>
        /// <param name="targetIndices">Buffer to be filled with the target indices each slot should be moved to in order to sort the buffer.</param>
        public unsafe static void VectorCountingSortTranspose(Buffer<float> paddedKeys, Buffer<int> targetIndices)
        {
            if (Vector256.IsHardwareAccelerated)
            {
                Debug.Assert(paddedKeys.length >= targetIndices.length && paddedKeys.length % 8 == 0, "This implementation assumes the keys are pre-padded.");
                var elementCount = targetIndices.length;
                var indexOffsets = Vector256.Create(0, 1, 2, 3, 4, 5, 6, 7);
                for (int i = 0; i < elementCount; i += 8)
                {
                    //Grab a bundle of values and test them against every other key.
                    var values = Vector256.Load(paddedKeys.Memory + i);
                    var counts = Vector256<int>.Zero;
                    var slotIndex = Vector256.Create(i) + indexOffsets;

                    var endOfEqualityTesting = i + 8;
                    //The first loop tests all vectors up to the start of the current bundle.
                    //These bundles do not require testing to see if the index is lesser; it definitely is.
                    for (int j = 0; j < i; ++j)
                    {
                        var testVector = Vector256.Create(paddedKeys[j]);
                        var slotPrecedesValue = Vector256.LessThanOrEqual(testVector, values);

                    }

                    int testedSlots;
                    for (testedSlots = 0; testedSlots < i; testedSlots += 8)
                    {
                        var testVector = Vector256.Load(paddedKeys.Memory + testedSlots);
                        var vectorIndices = indexOffsets + Vector256.Create(testedSlots);
                        var indexIsLesser = Vector256.LessThan(vectorIndices, currentIndex);
                        var slotIsEqual = Vector256.Equals(testVector, value);
                        var slotIsLesser = Vector256.LessThan(testVector, value);
                        var slotPrecedesValue = Vector256.BitwiseOr(slotIsLesser, Vector256.BitwiseAnd(slotIsEqual, indexIsLesser.AsSingle()));
                        var slotPrecedesValueMask = Vector256.ExtractMostSignificantBits(slotPrecedesValue.AsSingle());
                        var slotsPrecedingValueCount = BitOperations.PopCount(slotPrecedesValueMask);

                        previousCount += slotsPrecedingValueCount;
                    }

                    //For any slots after the query slot, there will be no 'lesser index but equal' entries, so the loop is simpler.
                    for (; testedSlots < paddedKeys.Length; testedSlots += 8)
                    {
                        var testVector = Vector256.Load(paddedKeys.Memory + testedSlots);
                        var slotPrecedesValue = Vector256.LessThan(testVector, value);
                        var slotPrecedesValueMask = Vector256.ExtractMostSignificantBits(slotPrecedesValue.AsSingle());
                        var slotsPrecedingValueCount = BitOperations.PopCount(slotPrecedesValueMask);

                        previousCount += slotsPrecedingValueCount;
                    }
                    targetIndices[i] = previousCount;
                }
            }
            else if (Vector128.IsHardwareAccelerated)
            {

            }
            else
            {
                //Shrug! Nonvector fallback. Really shouldn't call a function called VectorCountingSort if you don't have any vectorization.
                for (int i = 0; i < targetIndices.Length; ++i)
                {
                    targetIndices[i] = i;
                }
                var comparer = new PrimitiveComparer<float>();
                QuickSort.Sort(ref paddedKeys[0], ref targetIndices[0], 0, targetIndices.Length - 1, ref comparer);
            }
        }
    }
