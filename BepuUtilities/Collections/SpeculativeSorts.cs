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
    public static class SpeculativeSorts
    {
        public unsafe static void VectorCountingSort(Buffer<float> paddedKeys, Buffer<int> newIndices)
        {
            if (Avx2.IsSupported)
            {
                Debug.Assert(paddedKeys.length >= newIndices.length && paddedKeys.length % 8 == 0, "This implementation assumes the keys are pre-padded.");
                var elementCount = newIndices.length;
                var indexOffsets = Vector256.Create(0, 1, 2, 3, 4, 5, 6, 7);
                for (int i = 0; i < elementCount; ++i)
                {
                    //Broadcast the current value and test it against all other values.
                    //Count all values smaller than the current value, and all equal values that are in a lower index.
                    var previousCount = 0;
                    var value = Vector256.Create(paddedKeys[i]);
                    var currentIndex = Vector256.Create(i);

                    //TODO: There's no reason to include the full loop complexity for bundles that come after the current index.
                    for (int j = 0; j < paddedKeys.Length; j += 8)
                    {
                        var testVector = Vector256.Load(paddedKeys.Memory + j);
                        var vectorIndices = indexOffsets + Vector256.Create(j);
                        var indexIsLesser = Vector256.LessThan(vectorIndices, currentIndex);
                        var slotIsEqual = Vector256.Equals(testVector, value);
                        var slotIsLesser = Vector256.LessThan(testVector, value);
                        var slotPrecedesValue = Vector256.BitwiseOr(slotIsLesser, Vector256.BitwiseAnd(slotIsEqual, indexIsLesser.AsSingle()));

                        var slotPrecedesValueMask = Avx.MoveMask(slotPrecedesValue.AsSingle());

                        var slotsPrecedingValueCount = BitOperations.PopCount((uint)slotPrecedesValueMask);

                        previousCount += slotsPrecedingValueCount;
                    }
                    newIndices[i] = previousCount;
                }
            }

        }
    }
}
