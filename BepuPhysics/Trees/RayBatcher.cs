using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace BepuPhysics.Trees
{
    /// <summary>
    /// Reusable structure for testing large numbers of rays against trees.
    /// </summary>
    public struct RayBatcher : IDisposable
    {
        Buffer<ushort> rayIndices;
        struct StackEntry
        {
            public int NodeIndex;
            public int RayCount;
        }
        Buffer<StackEntry> stack;
        BufferPool pool;

        public RayBatcher(BufferPool pool, int rayCountForPreallocation = 2048, int treeDepthForPreallocation = 24)
        {
            this.pool = pool;
            Debug.Assert(rayCountForPreallocation <= ushort.MaxValue, "The number of ");
            //The number of ray pointers on the stack is limited in the worst case to all rays per level of the tree.
            var preallocatedRayPointerCount = rayCountForPreallocation * treeDepthForPreallocation;
            pool.Take(preallocatedRayPointerCount, out rayIndices);
            //The number of stack entries is limited by the number of node entries (tree node count * 3) and the number of ray entries.
            //(Can't have more entries on the stack than total ray pointers, after all.)
            pool.Take(Math.Min(preallocatedRayPointerCount, 3 << Math.Min(16, treeDepthForPreallocation)), out stack);
        }

        public void Dispose()
        {
            pool.ReturnUnsafely(rayIndices.Id);
            pool.ReturnUnsafely(stack.Id);
            //Easier to catch bugs if the references get cleared.
            rayIndices = new Buffer<ushort>();
            stack = new Buffer<StackEntry>();
        }
    }
}
