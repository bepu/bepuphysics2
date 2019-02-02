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
        private List<(string tag, int pointsStartIndex)> simplexes;
        private QuickList<Vector3> points;

        BufferPool pool;

        public int SimplexCount { get { return simplexes.Count; } }

        public MinkowskiSimplexes(BufferPool pool, int initialSimplexCapacity = 2048)
        {
            this.pool = pool;
            simplexes = new List<(string, int)>(initialSimplexCapacity);
            points = new QuickList<Vector3>(initialSimplexCapacity * 4, pool);
        }

        public Buffer<Vector3> AllocateSimplex(int count, string tag = null)
        {
            var newCount = points.Count + count;
            points.EnsureCapacity(newCount, pool);
            var simplexPoints = points.Span.Slice(points.Count, count);
            simplexes.Add((tag, points.Count));
            points.Count += count;
            return simplexPoints;
        }

        public (string tag, Buffer<Vector3> points) GetSimplex(int simplexIndex)
        {
            var simplexStart = simplexes[simplexIndex].pointsStartIndex;
            var simplexEnd = simplexIndex == simplexes.Count - 1 ? points.Count : simplexes[simplexIndex + 1].pointsStartIndex;
            return (simplexes[simplexIndex].tag, points.Span.Slice(simplexStart, simplexEnd - simplexStart));
        }

        public void Clear()
        {
            simplexes.Clear();
            points.Count = 0;
        }

        public void Dispose()
        {
            points.Dispose(pool);
        }
    }
}
