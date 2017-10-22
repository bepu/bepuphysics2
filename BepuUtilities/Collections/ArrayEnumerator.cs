using System.Collections;
using System.Collections.Generic;

namespace BepuUtilities.Collections
{
    /// <summary>
    /// Enumerable wrapper of an array interval.
    /// </summary>
    /// <typeparam name="T">Type of the array elements.</typeparam>
    public struct ArrayEnumerable<T> : IEnumerable<T>
    {
        T[] array;
        int start;
        int end;

        public int Count
        {
            get
            {
                return end - start;
            }
        }

        /// <summary>
        /// Creates a enumerable that spans an interval in an array.
        /// </summary>
        /// <param name="array">Array to iterate over.</param>
        /// <param name="start">Inclusive beginning of the iteration interval.</param>
        /// <param name="end">Exclusive end of the iteration interval.</param>
        public ArrayEnumerable(T[] array, int start, int end)
        {
            this.array = array;
            this.start = start;
            this.end = end;
        }

        /// <summary>
        /// Creates a enumerable that starts at the first index of the array.
        /// </summary>
        /// <param name="array">Array to iterate over.</param>
        /// <param name="count">Exclusive end of the iteration interval.</param>
        public ArrayEnumerable(T[] array, int count)
        {
            this.array = array;
            this.start = 0;
            this.end = count;
        }

        /// <summary>
        /// Creates a enumerable over the entire array.
        /// </summary>
        /// <param name="array">Array to iterate over.</param>
        public ArrayEnumerable(T[] array)
        {
            this.array = array;
            this.start = 0;
            this.end = array.Length;
        }

        public ArrayEnumerator<T> GetEnumerator()
        {
            return new ArrayEnumerator<T>(array, start, end);
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return GetEnumerator();
        }
    }

    /// <summary>
    /// Enumerates over a given array region.
    /// </summary>
    /// <typeparam name="T">Type of the array to iterate over.</typeparam>
    public struct ArrayEnumerator<T> : IEnumerator<T>
    {
        int start;
        int end;
        int index;
        T[] array;
        public ArrayEnumerator(T[] array, int inclusiveStart, int exclusiveEnd)
        {
            this.start = inclusiveStart;
            this.end = exclusiveEnd;
            this.array = array;
            index = inclusiveStart - 1;
        }
        public T Current
        {
            get
            {
                return array[index];
            }
        }

        object IEnumerator.Current
        {
            get
            {
                return array[index];
            }
        }

        public void Dispose()
        {
        }

        public bool MoveNext()
        {
            return ++index < end;
        }

        public void Reset()
        {
            index = start - 1;
        }
    }
}
