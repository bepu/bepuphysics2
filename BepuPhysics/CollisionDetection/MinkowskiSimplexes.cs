using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Debug helper class for storing the simplexes associated with minkowski space method execution.
    /// </summary>
    public class MinkowskiSimplexes
    {
        public QuickList<int> SimplexIndices;
        public QuickList<Vector3> Points;

        BufferPool pool;

        public int SimplexCount { get { return SimplexIndices.Count; } }

        public MinkowskiSimplexes(BufferPool pool, int initialSimplexCapacity = 2048)
        {
            this.pool = pool;
            SimplexIndices = new QuickList<int>(initialSimplexCapacity, pool);
            Points = new QuickList<Vector3>(initialSimplexCapacity * 4, pool);
        }

        public Buffer<Vector3> AllocateSimplex(int count)
        {
            var newCount = Points.Count + count;
            Points.EnsureCapacity(newCount, pool);
            var simplexPoints = Points.Span.Slice(Points.Count, count);
            SimplexIndices.Add(Points.Count, pool);
            Points.Count += count;
            return simplexPoints;
        }

        public Buffer<Vector3> GetSimplex(int simplexIndex)
        {
            var simplexStart = SimplexIndices[simplexIndex];
            var simplexEnd = simplexIndex == SimplexIndices.Count - 1 ? Points.Count : SimplexIndices[simplexIndex + 1];
            return Points.Span.Slice(simplexStart, simplexEnd - simplexStart);
        }

        public void Clear()
        {
            SimplexIndices.Count = 0;
            Points.Count = 0;
        }

        public void Dispose()
        {
            SimplexIndices.Dispose(pool);
            Points.Dispose(pool);
        }
    }
}
