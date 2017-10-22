using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuUtilities.Collections
{
    public static class InsertionSort
    {
        //TODO: If the jit ever managed to handle ISpan indexers optimally, we could use a much more natural ISpan-based implementation.

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static void Sort<TKey, TValue, TKeySpan, TValueSpan, TComparer>
        //    (ref TKeySpan keys, ref TValueSpan values, int start, int inclusiveEnd, ref TComparer comparer)
        //   where TComparer : IComparerRef<TKey>
        //   where TKeySpan : ISpan<TKey>
        //   where TValueSpan : ISpan<TValue>
        //{
        //    Debug.Assert(keys.Length <= values.Length);
        //    Debug.Assert(start >= 0 && start < keys.Length);
        //    Debug.Assert(inclusiveEnd < keys.Length); //We don't bother checking if inclusiveEnd is >= start; a zero length region will be caught by the loop condition.

        //    for (int i = start + 1; i <= inclusiveEnd; ++i)
        //    {
        //        var originalKey = keys[i];
        //        var originalValue = values[i];
        //        int compareIndex;
        //        for (compareIndex = i - 1; compareIndex >= start; --compareIndex)
        //        {
        //            if (comparer.Compare(ref originalKey, ref keys[compareIndex]) < 0)
        //            {
        //                //Move the element up one slot.
        //                var upperSlotIndex = compareIndex + 1;
        //                keys[upperSlotIndex] = keys[compareIndex];
        //                values[upperSlotIndex] = values[compareIndex];
        //            }
        //            else
        //                break;
        //        }
        //        var targetIndex = compareIndex + 1;
        //        if (targetIndex != i)
        //        {
        //            //Move the original index down.
        //            keys[targetIndex] = originalKey;
        //            values[targetIndex] = originalValue;
        //        }
        //    }
        //}

        //TODO: You could efficiently eliminate the redundant implementations of sorts by abstracting away the datasource.
        //One implementation could cover keys-only, keys+values, and keys-but-only-memory-move-values.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Sort<TKey, TValue, TComparer>
           (ref TKey keys, ref TValue values, int start, int inclusiveEnd, ref TComparer comparer)
          where TComparer : IComparerRef<TKey>
        {
            for (int i = start + 1; i <= inclusiveEnd; ++i)
            {
                var originalKey = Unsafe.Add(ref keys, i);
                var originalValue = Unsafe.Add(ref values, i);
                int compareIndex;
                for (compareIndex = i - 1; compareIndex >= start; --compareIndex)
                {
                    if (comparer.Compare(ref originalKey, ref Unsafe.Add(ref keys, compareIndex)) < 0)
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
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Sort<TKey, TComparer>
           (ref TKey keys, int start, int inclusiveEnd, ref TComparer comparer)
            where TComparer : IComparerRef<TKey>
        {
            for (int i = start + 1; i <= inclusiveEnd; ++i)
            {
                var originalKey = Unsafe.Add(ref keys, i);
                int compareIndex;
                for (compareIndex = i - 1; compareIndex >= start; --compareIndex)
                {
                    if (comparer.Compare(ref originalKey, ref Unsafe.Add(ref keys, compareIndex)) < 0)
                    {
                        //Move the element up one slot.
                        var upperSlotIndex = compareIndex + 1;
                        Unsafe.Add(ref keys, upperSlotIndex) = Unsafe.Add(ref keys, compareIndex);
                    }
                    else
                        break;
                }
                var targetIndex = compareIndex + 1;
                if (targetIndex != i)
                {
                    //Move the original index down.
                    Unsafe.Add(ref keys, targetIndex) = originalKey;
                }
            }
        }
    }
}
