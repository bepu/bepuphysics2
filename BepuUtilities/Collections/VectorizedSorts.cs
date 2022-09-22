using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.Intrinsics;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Specialized sorts that can go fast sometimes, but often come with special requirements.
    /// </summary>
    public static class VectorizedSorts
    {
        /// <summary>
        /// Computes the mapping from current index to ascending sorted index for all elements in the paddedKeys. The paddedKeys buffer is not actually sorted.
        /// <para>This function requires that <see cref="Vector256.IsHardwareAccelerated"/> or <see cref="Vector128.IsHardwareAccelerated"/>.</para>
        /// </summary>
        /// <param name="paddedKeys">Padded keys to compute sorted indices for. This buffer's contents will not be mutated.
        /// The buffer must be at least as long as the targetIndices buffer and must be padded to be divisible by vector length.
        /// Vector length is 8 elements if <see cref="Vector256.IsHardwareAccelerated"/> or 4 elements if only <see cref="Vector128.IsHardwareAccelerated"/>.</param>
        /// <param name="paddedTargetIndices">Buffer to be filled with the target indices each slot should be moved to in order to sort the buffer.
        /// The buffer must be padded to be divisible by vector length.
        /// Vector length is 8 elements if <see cref="Vector256.IsHardwareAccelerated"/> or 4 elements if only <see cref="Vector128.IsHardwareAccelerated"/>.</param>
        /// <param name="elementCount">Number of actual elements in the paddedKeys buffer.</param>
        public unsafe static void VectorCountingSort(Buffer<float> paddedKeys, Buffer<int> paddedTargetIndices, int elementCount)
        {
            if (Vector256.IsHardwareAccelerated)
            {
                Debug.Assert(paddedKeys.length == paddedTargetIndices.length && paddedKeys.length % 8 == 0 && paddedKeys.Length >= elementCount,
                    "This implementation assumes the keys and target indices are pre-padded, and can contain the element count.");
                var indexOffsets = Vector256.Create(0, 1, 2, 3, 4, 5, 6, 7);
                for (int i = 0; i < elementCount; i += 8)
                {
                    //Grab a bundle of values and test them against every other key.
                    var values = Vector256.Load(paddedKeys.Memory + i);
                    var counts = Vector256<int>.Zero;

                    var oneMask = Vector256.Create(1);
                    //The first loop tests all vectors up to the start of the current bundle.
                    //These bundles do not require testing to see if the index is lesser; it definitely is.
                    for (int j = 0; j < i; ++j)
                    {
                        var testVector = Vector256.Create(paddedKeys[j]);
                        var slotPrecedesValue = Vector256.LessThanOrEqual(testVector, values);
                        counts += Vector256.BitwiseAnd(slotPrecedesValue.AsInt32(), oneMask);
                    }

                    //For indices that overlap the current bundle, we need to check their relative position.
                    var endOfEqualityTesting = Math.Min(i + 8, elementCount);
                    var slotIndex = Vector256.Create(i) + indexOffsets;
                    for (int j = i; j < endOfEqualityTesting; ++j)
                    {
                        var testVector = Vector256.Create(paddedKeys[j]);
                        var slotIsLesser = Vector256.LessThan(testVector, values);
                        var slotIndexIsLesser = Vector256.LessThan(Vector256.Create(j), slotIndex);
                        var slotIsEqual = Vector256.Equals(testVector, values);
                        var slotPrecedesValue = Vector256.BitwiseOr(slotIsLesser, Vector256.BitwiseAnd(slotIsEqual, slotIndexIsLesser.AsSingle()));
                        counts += Vector256.BitwiseAnd(slotPrecedesValue.AsInt32(), oneMask);
                    }

                    //For indices that come after the current bundle, the relative position is known.
                    for (int j = endOfEqualityTesting; j < elementCount; ++j)
                    {
                        var testVector = Vector256.Create(paddedKeys[j]);
                        var slotPrecedesValue = Vector256.LessThan(testVector, values);
                        counts += Vector256.BitwiseAnd(slotPrecedesValue.AsInt32(), oneMask);
                    }

                    Vector256.Store(counts, paddedTargetIndices.Memory + i);
                }
            }
            else if (Vector128.IsHardwareAccelerated)
            {
                Debug.Assert(paddedKeys.length == paddedTargetIndices.length && paddedKeys.length % 4 == 0 && paddedKeys.Length >= elementCount,
                    "This implementation assumes the keys and target indices are pre-padded, and can contain the element count.");
                var indexOffsets = Vector128.Create(0, 1, 2, 3);
                for (int i = 0; i < elementCount; i += 4)
                {
                    //Grab a bundle of values and test them against every other key.
                    var values = Vector128.Load(paddedKeys.Memory + i);
                    var counts = Vector128<int>.Zero;

                    var oneMask = Vector128.Create(1);
                    //The first loop tests all vectors up to the start of the current bundle.
                    //These bundles do not require testing to see if the index is lesser; it definitely is.
                    for (int j = 0; j < i; ++j)
                    {
                        var testVector = Vector128.Create(paddedKeys[j]);
                        var slotPrecedesValue = Vector128.LessThanOrEqual(testVector, values);
                        counts += Vector128.BitwiseAnd(slotPrecedesValue.AsInt32(), oneMask);
                    }

                    //For indices that overlap the current bundle, we need to check their relative position.
                    var endOfEqualityTesting = Math.Min(i + 4, elementCount);
                    var slotIndex = Vector128.Create(i) + indexOffsets;
                    for (int j = i; j < endOfEqualityTesting; ++j)
                    {
                        var testVector = Vector128.Create(paddedKeys[j]);
                        var slotIsLesser = Vector128.LessThan(testVector, values);
                        var slotIndexIsLesser = Vector128.LessThan(Vector128.Create(j), slotIndex);
                        var slotIsEqual = Vector128.Equals(testVector, values);
                        var slotPrecedesValue = Vector128.BitwiseOr(slotIsLesser, Vector128.BitwiseAnd(slotIsEqual, slotIndexIsLesser.AsSingle()));
                        counts += Vector128.BitwiseAnd(slotPrecedesValue.AsInt32(), oneMask);
                    }

                    //For indices that come after the current bundle, the relative position is known.
                    for (int j = endOfEqualityTesting; j < elementCount; ++j)
                    {
                        var testVector = Vector128.Create(paddedKeys[j]);
                        var slotPrecedesValue = Vector128.LessThan(testVector, values);
                        counts += Vector128.BitwiseAnd(slotPrecedesValue.AsInt32(), oneMask);
                    }

                    Vector128.Store(counts, paddedTargetIndices.Memory + i);
                }
            }
            else
            {
                throw new NotSupportedException("This sort assumes that Vector256.IsHardwareAccelerated or Vector128.IsHardwareAccelerated. It is the caller's responsibility to guarantee this.");
            }
        }
    }
}
