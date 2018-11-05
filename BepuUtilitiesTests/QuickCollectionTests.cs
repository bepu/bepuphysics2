using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;

namespace BEPUutilitiesTests
{
    public static class QuickCollectionTests
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void TestQueueResizing(IUnmanagedMemoryPool pool)
        {
            Random random = new Random(5);

            var queue = new QuickQueue<int>(4, pool);
            Queue<int> controlQueue = new Queue<int>();

            for (int iterationIndex = 0; iterationIndex < 1000000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    queue.Enqueue(iterationIndex, pool);
                    controlQueue.Enqueue(iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    queue.Dequeue();
                    controlQueue.Dequeue();
                }
                if (iterationIndex % 1000 == 0)
                {
                    queue.EnsureCapacity(queue.Count * 3, pool);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    queue.Compact(pool);
                }
            }

            Debug.Assert(queue.Count == controlQueue.Count, "e");
            while (queue.Count > 0)
            {
                var a = queue.Dequeue();
                var b = controlQueue.Dequeue();
                Debug.Assert(a == b);
                Debug.Assert(queue.Count == controlQueue.Count);
            }

            queue.Dispose(pool);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void TestListResizing(IUnmanagedMemoryPool pool)
        {
            Random random = new Random(5);
            var list = new QuickList<int>(4, pool);
            List<int> controlList = new List<int>();

            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    list.Add(iterationIndex, pool);
                    controlList.Add(iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    var indexToRemove = random.Next(list.Count);
                    list.RemoveAt(indexToRemove);
                    controlList.RemoveAt(indexToRemove);
                }
                if (iterationIndex % 1000 == 0)
                {
                    list.EnsureCapacity(list.Count * 3, pool);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    list.Compact(pool);
                }
            }

            Debug.Assert(list.Count == controlList.Count);
            for (int i = 0; i < list.Count; ++i)
            {
                var a = list[i];
                var b = controlList[i];
                Debug.Assert(a == b);
                Debug.Assert(list.Count == controlList.Count);
            }

            list.Dispose(pool);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void TestSetResizing(IUnmanagedMemoryPool pool)
        {
            Random random = new Random(5);
            var set = new QuickSet<int, PrimitiveComparer<int>>(4, pool);
            HashSet<int> controlSet = new HashSet<int>();

            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    set.Add(iterationIndex, pool);
                    controlSet.Add(iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    var indexToRemove = random.Next(set.Count);
                    var toRemove = set[indexToRemove];
                    set.FastRemove(toRemove);
                    controlSet.Remove(toRemove);
                }
                if (iterationIndex % 1000 == 0)
                {
                    set.EnsureCapacity(set.Count * 3, pool);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    set.Compact(pool);
                }
            }

            Debug.Assert(set.Count == controlSet.Count);
            for (int i = 0; i < set.Count; ++i)
            {
                Debug.Assert(controlSet.Contains(set[i]));
            }
            foreach (var element in controlSet)
            {
                Debug.Assert(set.Contains(element));
            }

            set.Dispose(pool);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void TestDictionaryResizing(IUnmanagedMemoryPool pool)
        {
            Random random = new Random(5);
            var dictionary = new QuickDictionary<int, int, PrimitiveComparer<int>>(4, pool);
            Dictionary<int, int> controlDictionary = new Dictionary<int, int>();

            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                if (random.NextDouble() < 0.7)
                {
                    dictionary.Add(iterationIndex, iterationIndex, pool);
                    controlDictionary.Add(iterationIndex, iterationIndex);
                }
                if (random.NextDouble() < 0.2)
                {
                    var indexToRemove = random.Next(dictionary.Count);
                    var toRemove = dictionary.Keys[indexToRemove];
                    dictionary.FastRemove(toRemove);
                    controlDictionary.Remove(toRemove);
                }
                if (iterationIndex % 1000 == 0)
                {
                    dictionary.EnsureCapacity(dictionary.Count * 3, pool);
                }
                else if (iterationIndex % 7777 == 0)
                {
                    dictionary.Compact(pool);
                }
            }

            Debug.Assert(dictionary.Count == controlDictionary.Count);
            for (int i = 0; i < dictionary.Count; ++i)
            {
                Debug.Assert(controlDictionary.ContainsKey(dictionary.Keys[i]));
            }
            foreach (var element in controlDictionary.Keys)
            {
                Debug.Assert(dictionary.ContainsKey(element));
            }
            dictionary.Dispose(pool);
        }

        public static void Test()
        {
            var bufferPool = new BufferPool(256);
            TestQueueResizing(bufferPool);
            TestListResizing(bufferPool);
            TestSetResizing(bufferPool);
            TestDictionaryResizing(bufferPool);
            bufferPool.Clear();
            

        }
    }
}
