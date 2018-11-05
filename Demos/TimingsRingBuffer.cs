using BepuUtilities.Collections;
using BepuUtilities.Memory;
using Demos.UI;
using System;

namespace Demos
{
    public class TimingsRingBuffer : IDataSeries, IDisposable
    {
        QuickQueue<double> queue;
        BufferPool pool;

        /// <summary>
        /// Gets or sets the maximum number of time measurements that can be held by the ring buffer.
        /// </summary>
        public int Capacity
        {
            get { return queue.Span.Length; }
            set
            {
                if (value <= 0)
                    throw new ArgumentException("Capacity must be positive.");
                if (Capacity != value)
                {
                    queue.Resize(value, pool);
                }
            }
        }
        public TimingsRingBuffer(int maximumCapacity, BufferPool pool)
        {
            if(maximumCapacity <= 0)
                throw new ArgumentException("Capacity must be positive.");
            this.pool = pool;
            queue = new QuickQueue<double>(maximumCapacity, pool);
        }

        public void Add(double time)
        {
            if(queue.Count == Capacity)
            {
                queue.Dequeue();
            }
            queue.EnqueueUnsafely(time);
        }


        public double this[int index] => queue[index];

        public int Start => 0;

        public int End => queue.Count;

        public TimelineStats ComputeStats()
        {
            TimelineStats stats;
            stats.Total = 0.0;
            var sumOfSquares = 0.0;
            stats.Min = double.MaxValue;
            stats.Max = double.MinValue;
            for (int i = 0; i < queue.Count; ++i)
            {
                var time = queue[i];
                stats.Total += time;
                sumOfSquares += time * time;
                if (time < stats.Min)
                    stats.Min = time;
                if (time > stats.Max)
                    stats.Max = time;
            }
            stats.Average = stats.Total / queue.Count;
            stats.StdDev = Math.Sqrt(Math.Max(0, sumOfSquares / queue.Count - stats.Average * stats.Average));
            return stats;
        }

        public void Dispose()
        {
            queue.Dispose(pool);
        }
    }
}
