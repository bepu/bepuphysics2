using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Threading;

namespace BepuPhysics
{
    public class UnconstrainedBodies
    {
        /// <summary>
        /// Indices of bodies in the active set that are unconstrained.
        /// </summary>
        public Buffer<int> BodyIndices;
        public int Count;
        public UnconstrainedBodies(int initialCapacity, BufferPool pool)
        {
            pool.TakeAtLeast(initialCapacity, out BodyIndices);
        }

        internal int AddMultithreaded(int bodyIndex)
        {
            var unconstrainedIndex = Interlocked.Increment(ref Count) - 1;
            Debug.Assert(unconstrainedIndex < BodyIndices.Length, "The multithreaded variant assumes that the body indices buffer has been resized to the maximum required size prior to execution.");
            BodyIndices[unconstrainedIndex] = bodyIndex;
            return unconstrainedIndex;
        }

        internal int Add(int bodyIndex, BufferPool pool)
        {
            if (Count == BodyIndices.Length)
            {
                pool.ResizeToAtLeast(ref BodyIndices, Count * 2, BodyIndices.Length);
            }
            var index = Count++;
            BodyIndices[index] = bodyIndex;
            return index;
        }
        internal bool RemoveAt(int unconstrainedIndex, out int movedBodyIndex)
        {
            Debug.Assert(unconstrainedIndex >= 0 && unconstrainedIndex < Count, "Unconstrained index should fall within the unconstrained set.");
            --Count;
            if (unconstrainedIndex < Count)
            {
                //The removal target is not the last element, so we'll move the current last element to the removal index.
                movedBodyIndex = BodyIndices[Count];
                BodyIndices[unconstrainedIndex] = movedBodyIndex;
                return true;
            }
            else
            {
                movedBodyIndex = -1;
                return false;
            }
        }

        internal void UpdateForBodyMemoryMove(int unconstrainedIndex, int bodyIndex)
        {
            BodyIndices[unconstrainedIndex] = bodyIndex;
        }


        public void EnsureCapacity(int capacity, BufferPool pool)
        {
            capacity = capacity > BodyIndices.Length ? capacity : BodyIndices.Length;
            var target = BufferPool.GetCapacityForCount<int>(capacity);
            if (target != BodyIndices.Length)
            {
                pool.ResizeToAtLeast(ref BodyIndices, target, Count);
            }
        }
        public void Resize(int capacity, BufferPool pool)
        {
            capacity = capacity > Count ? capacity : Count;
            var target = BufferPool.GetCapacityForCount<int>(capacity);
            if (target != BodyIndices.Length)
            {
                pool.ResizeToAtLeast(ref BodyIndices, target, Count);
            }
        }

        public void Clear()
        {
            Count = 0;
        }

        public void Dispose(BufferPool pool)
        {
            pool.Return(ref BodyIndices);
            Count = 0;
        }
    }
}