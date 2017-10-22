using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuUtilities.Collections
{
    public static class QuickSort
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Swap<T>(ref T a, ref T b)
        {
            var temp = a;
            a = b;
            b = temp;
        }

        //TODO: If the jit ever managed to handle ISpan indexers optimally, we could use a much more natural ISpan-based implementation.

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //static void Swap<TKey, TValue, TKeySpan, TValueSpan>(ref TKeySpan keys, ref TValueSpan values, int a, int b)
        //    where TKeySpan : ISpan<TKey>
        //    where TValueSpan : ISpan<TValue>
        //{
        //    Swap(ref keys[a], ref keys[b]);
        //    Swap(ref values[a], ref values[b]);
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static int FindMO3Index<TKey, TKeySpan, TComparer>(ref TKeySpan keys, int l, int r, ref TComparer comparer)
        //    where TComparer : IComparerRef<TKey>
        //    where TKeySpan : ISpan<TKey>
        //{
        //    //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
        //    var first = keys[l];
        //    int middleIndex = (l + r) / 2;
        //    var middle = keys[middleIndex];
        //    var last = keys[r];
        //    if (comparer.Compare(ref first, ref middle) <= 0 && comparer.Compare(ref first, ref last) <= 0)
        //    {
        //        //first is lowest.
        //        if (comparer.Compare(ref middle, ref last) <= 0)
        //        {
        //            return middleIndex;
        //        }
        //        else
        //        {
        //            return r;
        //        }
        //    }
        //    else if (comparer.Compare(ref middle, ref last) <= 0)
        //    {
        //        //middle is lowest.
        //        if (comparer.Compare(ref first, ref last) <= 0)
        //        {
        //            return l;
        //        }
        //        else
        //        {
        //            return r;
        //        }
        //    }
        //    else
        //    {
        //        //last is lowest.
        //        if (comparer.Compare(ref first, ref middle) <= 0)
        //        {
        //            return l;
        //        }
        //        else
        //        {
        //            return middleIndex;
        //        }
        //    }
        //}

        //public static void SortWithThreeWayPartitioning<TKey, TValue, TKeySpan, TValueSpan, TComparer>
        //    (ref TKeySpan keys, ref TValueSpan values, int l, int r, ref TComparer comparer)
        //    where TComparer : IComparerRef<TKey>
        //    where TKeySpan : ISpan<TKey>
        //    where TValueSpan : ISpan<TValue>
        //{
        //    if (r - l <= 30)
        //    {
        //        //The area to address is very small. Use insertion sort.
        //        InsertionSort.Sort<TKey, TValue, TKeySpan, TValueSpan, TComparer>(ref keys, ref values, l, r, ref comparer);
        //    }
        //    else
        //    {
        //        //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
        //        var pivotIndex = FindMO3Index<TKey, TKeySpan, TComparer>(ref keys, l, r, ref comparer);

        //        //Put the pivot into the last slot.
        //        Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, pivotIndex, r);
        //        ref var pivot = ref keys[r];

        //        //Use bentley-mcilroy 3-way partitioning scheme to avoid performance drops in corner cases.
        //        int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
        //        int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
        //        int p = l - 1;
        //        int q = r;
        //        if (r <= l)
        //            return;
        //        while (true)
        //        {
        //            //Claim the chunk of the list which is partitioned on the left and right sides.
        //            while (true)
        //            {
        //                ++i;
        //                if (comparer.Compare(ref keys[i], ref pivot) >= 0)
        //                    break;
        //            }
        //            while (true)
        //            {
        //                --j;
        //                if (comparer.Compare(ref pivot, ref keys[j]) >= 0 || j == l)
        //                    break;

        //            }
        //            //If the claims have met, then the partition is complete.
        //            if (i >= j)
        //            {
        //                break;
        //            }
        //            //By the claiming above and because we did not yet break out of the loop,
        //            //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
        //            //So swap them.
        //            Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, i, j);
        //            if (comparer.Compare(ref keys[i], ref pivot) == 0)
        //            {
        //                p++;
        //                Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, p, i);
        //            }
        //            if (comparer.Compare(ref pivot, ref keys[j]) == 0)
        //            {
        //                q--;
        //                Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, j, q);
        //            }
        //        }
        //        //The pivot at r has not been swapped.
        //        //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
        //        //So, swap the pivot and the first element of the greater-than-pivot side to guarantee sorting.
        //        //Note that this invalidates the 'pivot' variable earlier; it was byref. But we don't use it anymore, so that's okay.
        //        Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, i, r);
        //        j = i - 1;
        //        i = i + 1;
        //        for (int k = l; k < p; k++, j--)
        //        {
        //            Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, k, j);
        //        }
        //        for (int k = r - 1; k > q; k--, i++)
        //        {
        //            Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, i, k);
        //        }
        //        SortWithThreeWayPartitioning<TKey, TValue, TKeySpan, TValueSpan, TComparer>(ref keys, ref values, l, j, ref comparer);
        //        SortWithThreeWayPartitioning<TKey, TValue, TKeySpan, TValueSpan, TComparer>(ref keys, ref values, i, r, ref comparer);
        //    }
        //}


        //public static void Sort<TKey, TValue, TKeySpan, TValueSpan, TComparer>
        //    (ref TKeySpan keys, ref TValueSpan values, int l, int r, ref TComparer comparer)
        //    where TComparer : IComparerRef<TKey>
        //    where TKeySpan : ISpan<TKey>
        //    where TValueSpan : ISpan<TValue>
        //{
        //    if (r - l <= 30)
        //    {
        //        //The area to address is very small. Use insertion sort.
        //        InsertionSort.Sort<TKey, TValue, TKeySpan, TValueSpan, TComparer>(ref keys, ref values, l, r, ref comparer);
        //    }
        //    else
        //    {
        //        //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
        //        var pivotIndex = FindMO3Index<TKey, TKeySpan, TComparer>(ref keys, l, r, ref comparer);

        //        //Put the pivot into the last slot.
        //        Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, pivotIndex, r);
        //        ref var pivot = ref keys[r];

        //        int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
        //        int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
        //        while (true)
        //        {
        //            //Claim the chunk of the list which is partitioned on the left and right sides.
        //            while (true)
        //            {
        //                ++i;
        //                if (comparer.Compare(ref keys[i], ref pivot) >= 0)
        //                    break;
        //            }
        //            while (true)
        //            {
        //                --j;
        //                if (comparer.Compare(ref pivot, ref keys[j]) >= 0 || j <= i)
        //                    break;

        //            }
        //            //If the claims have met, then the partition is complete.
        //            if (i >= j)
        //            {
        //                break;
        //            }
        //            //By the claiming above and because we did not yet break out of the loop,
        //            //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
        //            //So swap them.
        //            Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, i, j);
        //        }
        //        //The pivot has not been reintroduced.
        //        //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
        //        //So, push the first element of the greater-than-pivot side into r and the pivot into the first greater element's slot to guarantee sorting.
        //        Swap<TKey, TValue, TKeySpan, TValueSpan>(ref keys, ref values, i, r);
        //        j = i - 1; //Sort's parameters take an inclusive bound, so push j back.
        //        i = i + 1; //The pivot takes i's spot, and the pivot should not be included in sorting, so push i up.
        //        Debug.Assert(i <= r);
        //        Debug.Assert(j >= l);
        //        Sort<TKey, TValue, TKeySpan, TValueSpan, TComparer>(ref keys, ref values, l, j, ref comparer);
        //        Sort<TKey, TValue, TKeySpan, TValueSpan, TComparer>(ref keys, ref values, i, r, ref comparer);
        //    }
        //}




        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Swap<TKey, TValue>(ref TKey keys, ref TValue values, int a, int b)
        {
            Swap(ref Unsafe.Add(ref keys, a), ref Unsafe.Add(ref keys, b));
            Swap(ref Unsafe.Add(ref values, a), ref Unsafe.Add(ref values, b));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Swap<TKey>(ref TKey keys, int a, int b)
        {
            Swap(ref Unsafe.Add(ref keys, a), ref Unsafe.Add(ref keys, b));
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int FindMO3Index<TKey, TComparer>(ref TKey keys, int l, int r, ref TComparer comparer)
            where TComparer : IComparerRef<TKey>
        {
            //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
            ref var first = ref Unsafe.Add(ref keys, l);
            int middleIndex = (l + r) / 2;
            ref var middle = ref Unsafe.Add(ref keys, middleIndex);
            ref var last = ref Unsafe.Add(ref keys, r);
            if (comparer.Compare(ref first, ref middle) <= 0 && comparer.Compare(ref first, ref last) <= 0)
            {
                //first is lowest.
                if (comparer.Compare(ref middle, ref last) <= 0)
                {
                    return middleIndex;
                }
                else
                {
                    return r;
                }
            }
            else if (comparer.Compare(ref middle, ref last) <= 0)
            {
                //middle is lowest.
                if (comparer.Compare(ref first, ref last) <= 0)
                {
                    return l;
                }
                else
                {
                    return r;
                }
            }
            else
            {
                //last is lowest.
                if (comparer.Compare(ref first, ref middle) <= 0)
                {
                    return l;
                }
                else
                {
                    return middleIndex;
                }
            }
        }

        public static void SortWithThreeWayPartitioning<TKey, TValue, TComparer>(ref TKey keys, ref TValue values, int l, int r, ref TComparer comparer) where TComparer : IComparerRef<TKey>
        {
            if (r - l <= 30)
            {
                //The area to address is very small. Use insertion sort.
                InsertionSort.Sort(ref keys, ref values, l, r, ref comparer);
            }
            else
            {
                //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
                var pivotIndex = FindMO3Index(ref keys, l, r, ref comparer);

                //Put the pivot into the last slot.
                var pivot = Unsafe.Add(ref keys, pivotIndex);
                var pivotValue = Unsafe.Add(ref values, pivotIndex);
                //We cached the pivot key and value. Push the value in 'r' to the pivot's location, leaving r unused for the moment.
                Unsafe.Add(ref keys, pivotIndex) = Unsafe.Add(ref keys, r);
                Unsafe.Add(ref values, pivotIndex) = Unsafe.Add(ref values, r);

                //Use bentley-mcilroy 3-way partitioning scheme to avoid performance drops in corner cases.
                int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
                int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
                int p = l - 1;
                int q = r;
                if (r <= l)
                    return;
                while (true)
                {
                    //Claim the chunk of the list which is partitioned on the left and right sides.
                    while (true)
                    {
                        ++i;
                        if (comparer.Compare(ref Unsafe.Add(ref keys, i), ref pivot) >= 0)
                            break;
                    }
                    while (true)
                    {
                        --j;
                        if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, j)) >= 0 || j == l)
                            break;

                    }
                    //If the claims have met, then the partition is complete.
                    if (i >= j)
                    {
                        break;
                    }
                    //By the claiming above and because we did not yet break out of the loop,
                    //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
                    //So swap them.
                    Swap(ref keys, ref values, i, j);
                    if (comparer.Compare(ref Unsafe.Add(ref keys, i), ref pivot) == 0)
                    {
                        p++;
                        Swap(ref keys, ref values, p, i);
                    }
                    if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, j)) == 0)
                    {
                        q--;
                        Swap(ref keys, ref values, j, q);
                    }
                }
                //The pivot at r has not been swapped.
                //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
                //So, swap the pivot and the first element of the greater-than-pivot side to guarantee sorting.
                ref var firstGreaterKeySlot = ref Unsafe.Add(ref keys, i);
                ref var firstGreaterValueSlot = ref Unsafe.Add(ref values, i);
                Unsafe.Add(ref keys, r) = firstGreaterKeySlot;
                Unsafe.Add(ref values, r) = firstGreaterValueSlot;
                firstGreaterKeySlot = pivot;
                firstGreaterValueSlot = pivotValue;
                j = i - 1;
                i = i + 1;
                for (int k = l; k < p; k++, j--)
                {
                    Swap(ref keys, ref values, k, j);
                }
                for (int k = r - 1; k > q; k--, i++)
                {
                    Swap(ref keys, ref values, i, k);
                }
                SortWithThreeWayPartitioning(ref keys, ref values, l, j, ref comparer);
                SortWithThreeWayPartitioning(ref keys, ref values, i, r, ref comparer);
            }
        }

        public static void Sort<TKey, TValue, TComparer>(ref TKey keys, ref TValue values, int l, int r, ref TComparer comparer) where TComparer : IComparerRef<TKey>
        {
            if (r - l <= 30)
            {
                //The area to address is very small. Use insertion sort.
                InsertionSort.Sort(ref keys, ref values, l, r, ref comparer);
            }
            else
            {
                //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
                var pivotIndex = FindMO3Index(ref keys, l, r, ref comparer);

                var pivot = Unsafe.Add(ref keys, pivotIndex);
                var pivotValue = Unsafe.Add(ref values, pivotIndex);
                //We cached the pivot key and value. Push the value in 'r' to the pivot's location, leaving r unused for the moment.
                Unsafe.Add(ref keys, pivotIndex) = Unsafe.Add(ref keys, r);
                Unsafe.Add(ref values, pivotIndex) = Unsafe.Add(ref values, r);

                int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
                int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
                while (true)
                {
                    //Claim the chunk of the list which is partitioned on the left and right sides.
                    while (true)
                    {
                        ++i;
                        if (comparer.Compare(ref Unsafe.Add(ref keys, i), ref pivot) >= 0)
                            break;
                    }
                    while (true)
                    {
                        --j;
                        if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, j)) >= 0 || j <= i)
                            break;

                    }
                    //If the claims have met, then the partition is complete.
                    if (i >= j)
                    {
                        break;
                    }
                    //By the claiming above and because we did not yet break out of the loop,
                    //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
                    //So swap them.
                    Swap(ref keys, ref values, i, j);
                }
                //The pivot has not been reintroduced.
                //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
                //So, push the first element of the greater-than-pivot side into r and the pivot into the first greater element's slot to guarantee sorting.
                ref var firstGreaterKeySlot = ref Unsafe.Add(ref keys, i);
                ref var firstGreaterValueSlot = ref Unsafe.Add(ref values, i);
                Unsafe.Add(ref keys, r) = firstGreaterKeySlot;
                Unsafe.Add(ref values, r) = firstGreaterValueSlot;
                firstGreaterKeySlot = pivot;
                firstGreaterValueSlot = pivotValue;
                j = i - 1; //Sort's parameters take an inclusive bound, so push j back.
                i = i + 1; //The pivot takes i's spot, and the pivot should not be included in sorting, so push i up.
                //Only bother sorting if there is anything to sort.
                if (j > l)
                    Sort(ref keys, ref values, l, j, ref comparer);
                if (r > i)
                    Sort(ref keys, ref values, i, r, ref comparer);
            }
        }
        public static void Sort<TKey, TComparer>(ref TKey keys, int l, int r, ref TComparer comparer) where TComparer : IComparerRef<TKey>
        {
            if (r - l <= 30)
            {
                //The area to address is very small. Use insertion sort.
                InsertionSort.Sort(ref keys, l, r, ref comparer);
            }
            else
            {
                //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
                var pivotIndex = FindMO3Index(ref keys, l, r, ref comparer);

                var pivot = Unsafe.Add(ref keys, pivotIndex);
                //We cached the pivot key and value. Push the value in 'r' to the pivot's location, leaving r unused for the moment.
                Unsafe.Add(ref keys, pivotIndex) = Unsafe.Add(ref keys, r);

                int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
                int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
                while (true)
                {
                    //Claim the chunk of the list which is partitioned on the left and right sides.
                    while (true)
                    {
                        ++i;
                        if (comparer.Compare(ref Unsafe.Add(ref keys, i), ref pivot) >= 0)
                            break;
                    }
                    while (true)
                    {
                        --j;
                        if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, j)) >= 0 || j <= i)
                            break;

                    }
                    //If the claims have met, then the partition is complete.
                    if (i >= j)
                    {
                        break;
                    }
                    //By the claiming above and because we did not yet break out of the loop,
                    //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
                    //So swap them.
                    Swap(ref keys, i, j);
                }
                //The pivot has not been reintroduced.
                //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
                //So, push the first element of the greater-than-pivot side into r and the pivot into the first greater element's slot to guarantee sorting.
                ref var firstGreaterKeySlot = ref Unsafe.Add(ref keys, i);
                Unsafe.Add(ref keys, r) = firstGreaterKeySlot;
                firstGreaterKeySlot = pivot;
                j = i - 1; //Sort's parameters take an inclusive bound, so push j back.
                i = i + 1; //The pivot takes i's spot, and the pivot should not be included in sorting, so push i up.
                //Only bother sorting if there is anything to sort.
                if (j > l)
                    Sort(ref keys, l, j, ref comparer);
                if (r > i)
                    Sort(ref keys, i, r, ref comparer);
            }
        }

    }
}
