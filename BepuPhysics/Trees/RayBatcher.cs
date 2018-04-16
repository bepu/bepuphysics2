using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Trees
{

    /// <summary>
    /// Reusable structure for testing large numbers of rays against trees.
    /// </summary>
    public struct RayBatcher : IDisposable
    {
        //If you want a deeper explanation about this implementation, check out the Dynamic Ray Stream Traversal paper by Barringer and Akenine-Moller. It's basically the same.
        int stackPointerA0, stackPointerB, stackPointerA1;
        Buffer<ushort> rayIndicesA0, rayIndicesB, rayIndicesA1;
        struct StackEntry
        {
            public int NodeIndex;
            public short RayCount;
            public short RayStack;
        }
        int stackPointer;
        Buffer<StackEntry> stack;
        BufferPool pool;
        int maximumRaysPerTraversal;

        /// <summary>
        /// Constructs a ray batcher and initializes its backing resources.
        /// </summary>
        /// <param name="pool">Pool to pull resources from.</param>
        /// <param name="maximumRaysPerTraversal">Maximum number of rays to execute in each traversal.
        /// This should typically be chosen as the highest value which avoids spilling data out of L2 cache.</param>
        /// <param name="treeDepthForPreallocation">Tree depth to preallocate ray stack space for. If a traversal finds nodes deeper than this, a dynamic resize will be triggered.</param>
        public RayBatcher(BufferPool pool, int maximumRaysPerTraversal = 2048, int treeDepthForPreallocation = 24)
        {
            this.maximumRaysPerTraversal = maximumRaysPerTraversal;
            this.pool = pool;
            Debug.Assert(maximumRaysPerTraversal <= ushort.MaxValue, $"The number of rays per traversal must be less than {ushort.MaxValue}; ray pointers are stored in 2 bytes.");
            //The number of ray pointers on the stack is limited in the worst case to all rays per level of the tree.
            var preallocatedRayPointerCount = maximumRaysPerTraversal * treeDepthForPreallocation;
            pool.Take(preallocatedRayPointerCount, out rayIndicesA0);
            pool.Take(preallocatedRayPointerCount, out rayIndicesB);
            pool.Take(preallocatedRayPointerCount, out rayIndicesA1);
            //The number of stack entries is limited by the number of node entries (tree node count * 3) and the number of ray entries.
            //(Can't have more entries on the stack than total ray pointers, after all.)
            pool.Take(Math.Min(preallocatedRayPointerCount, 3 << Math.Min(16, treeDepthForPreallocation)), out stack);

            stackPointer = stackPointerA0 = stackPointerB = stackPointerA1 = 0;

        }

        /// <summary>
        /// Ray representation designed for quicker intersection against axis aligned bounding boxes.
        /// </summary>
        public struct TreeRayWide
        {
            public Vector3Wide OriginOverDirection;
            public Vector3Wide InverseDirection;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void GatherIntoFirstSlot(ref TreeRay ray, ref TreeRayWide wide)
            {
                GatherScatter.GetFirst(ref wide.OriginOverDirection.X) = ray.OriginOverDirection.X;
                GatherScatter.GetFirst(ref wide.OriginOverDirection.Y) = ray.OriginOverDirection.Y;
                GatherScatter.GetFirst(ref wide.OriginOverDirection.Z) = ray.OriginOverDirection.Z;
                GatherScatter.GetFirst(ref wide.InverseDirection.X) = ray.InverseDirection.X;
                GatherScatter.GetFirst(ref wide.InverseDirection.Y) = ray.InverseDirection.Y;
                GatherScatter.GetFirst(ref wide.InverseDirection.Z) = ray.InverseDirection.Z;
            }
        }

        struct NodeWide
        {
            public Vector3Wide MinA;
            public Vector3Wide MaxA;
            public Vector3Wide MinB;
            public Vector3Wide MaxB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void BroadcastNode(ref Node node, out NodeWide wide)
        {
            Vector3Wide.CreateFrom(ref node.A.Min, out wide.MinA);
            Vector3Wide.CreateFrom(ref node.A.Max, out wide.MaxA);
            Vector3Wide.CreateFrom(ref node.B.Min, out wide.MinB);
            Vector3Wide.CreateFrom(ref node.B.Max, out wide.MaxB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Intersect(ref TreeRayWide ray, ref Vector3Wide min, ref Vector3Wide max, out Vector<float> tMin, out Vector<int> intersected)
        {
            //TODO: This is a good spot for FMA if/when we swap over to platform intrinsics.
            var tX0 = ray.OriginOverDirection.X + min.X * ray.InverseDirection.X;
            var tX1 = ray.OriginOverDirection.X + max.X * ray.InverseDirection.X;
            var tMinX = Vector.Min(tX0, tX1);
            var tMaxX = Vector.Max(tX0, tX1);

            var tY0 = ray.OriginOverDirection.Y + min.Y * ray.InverseDirection.Y;
            var tY1 = ray.OriginOverDirection.Y + max.Y * ray.InverseDirection.Y;
            var tMinY = Vector.Min(tY0, tY0);
            var tMaxY = Vector.Max(tY0, tY1);

            var tZ0 = ray.OriginOverDirection.Z + min.Z * ray.InverseDirection.Z;
            var tZ1 = ray.OriginOverDirection.Z + max.Z * ray.InverseDirection.Z;
            var tMinZ = Vector.Min(tZ0, tZ1);
            var tMaxZ = Vector.Max(tZ0, tZ1);

            tMin = Vector.Max(tMinX, Vector.Max(tMinY, tMinZ));
            var tMax = Vector.Min(tMaxX, Vector.Min(tMaxY, tMaxZ));
            intersected = Vector.LessThanOrEqual(tMin, tMax);
        }

        /// <summary>
        /// Ray representation designed for quicker intersection against axis aligned bounding boxes.
        /// </summary>
        struct TreeRay
        {
            public Vector3 OriginOverDirection;
            public Vector3 InverseDirection;
        }

        interface ITreeRaySource
        {
            int RayCount { get; }
            ref TreeRay this[int rayIndex] { get; }
        }

        unsafe struct RootRaySource : ITreeRaySource
        {
            TreeRay* rays;
            int rayCount;

            public RootRaySource(TreeRay* rays, int rayCount)
            {
                this.rays = rays;
                this.rayCount = rayCount;
            }

            public int RayCount
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get { return rayCount; }
            }

            public ref TreeRay this[int rayIndex]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    return ref rays[rayIndex];
                }
            }
        }

        public unsafe struct TreeRaySource : ITreeRaySource
        {
            TreeRay* rays;
            ushort* rayPointers;
            int rayCount;

            public TreeRaySource(TreeRay* rays, ushort* rayPointerStack, int rayCount)
            {
                this.rays = rays;
                this.rayPointers = rayPointerStack;
                this.rayCount = rayCount;
            }

            public int RayCount
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get { return rayCount; }
            }

            public ref TreeRay this[int rayIndex]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    Debug.Assert(rayIndex >= 0 && rayIndex < RayCount, "The requested ray index must be within the source's region.");
                    return ref rays[rayPointers[rayIndex]];
                }
            }
        }

        unsafe void TestNode<TRayStackSource>(Node* node, ref TRayStackSource raySource) where TRayStackSource : struct, ITreeRaySource
        {
            int a0Start = stackPointerA0;
            int bStart = stackPointerB;
            int a1Start = stackPointerA1;
            BroadcastNode(ref *node, out var wideNode);
            TreeRayWide rayBundle;

            for (int i = 0; i < raySource.RayCount; i += Vector<float>.Count)
            {
                var count = raySource.RayCount - i;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;
                for (int j = 0; j < count; ++j)
                {
                    TreeRayWide.GatherIntoFirstSlot(ref raySource[i + j], ref GatherScatter.GetOffsetInstance(ref rayBundle, j));
                }
                Intersect(ref rayBundle, ref wideNode.MinA, ref wideNode.MaxA, out var tA, out var aIntersected);
                Intersect(ref rayBundle, ref wideNode.MinB, ref wideNode.MaxB, out var tB, out var bIntersected);
                //There are three potential stack regions that a single ray can go based on when and if each node child is intersected:
                //rayIndicesA0: A first or A only
                //rayIndicesB:  B is intersected at all
                //rayIndicesA1: A after B
                //We assign the role for each ray in a vectorized way.
                var aFirst = Vector.LessThanOrEqual(tA, tB);
                var bothIntersected = Vector.BitwiseAnd(aIntersected, bIntersected);
                var shouldAllocateRayToA0 = Vector.BitwiseOr(Vector.BitwiseAnd(bothIntersected, aFirst), Vector.AndNot(aIntersected, bIntersected));
                for (int j = 0; j < count; ++j)
                {
                    //TODO: Examine codegen. Bounds checks MIGHT be elided, but if they aren't, we can work around them.
                    ushort rayPointerIndex = (ushort)(i + j);
                    if (shouldAllocateRayToA0[j] < 0)
                    {
                        rayIndicesA0[stackPointerA0++] = rayPointerIndex;
                    }
                    else if (aIntersected[j] < 0)
                    {
                        //If A was intersected but did not qualify to fit into the A0 slot, it must come after B.
                        rayIndicesA1[stackPointerA1++] = rayPointerIndex;
                    }
                    if (bIntersected[j] < 0)
                    {
                        rayIndicesB[stackPointerB++] = rayPointerIndex;
                    }

                }
            }
            var a1Count = stackPointerA1 - a1Start;
            if (a1Count > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node->A.Index;
                newEntry.RayCount = (short)a1Count;
            }
            var bCount = stackPointerB - bStart;
            if (bCount > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node->B.Index;
                newEntry.RayCount = (short)bCount;
            }
            var a0Count = stackPointerA0 - a0Start;
            if (a0Count > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node->A.Index;
                newEntry.RayCount = (short)a0Count;
            }

        }

        Buffer<TreeRay> rayData;
        public unsafe void TestRayBatch<TRaySource>(ref TRaySource raySource0, Tree tree) where TRaySource : IRaySource
        {
            if (tree.LeafCount == 0)
                return;

            if (rayData.Length < raySource0.RayCount)
                pool.Resize(ref rayData, raySource0.RayCount, 0);
            for (int i = 0; i < raySource0.RayCount; ++i)
            {
                ref var originalRay = ref raySource0[i];
                ref var ray = ref rayData[i];
                ray.InverseDirection = originalRay.
            }

            //The traversal begins by assuming an implicit stack entry for the root node containing all ray pointers from 0 to rayCount-1.
            TreeRayWide rayBundle = default;

            if (tree.LeafCount >= 2)
            {
                var raySource = new RootRaySource(rays, rayCount);
                TestNode(tree.nodes, ref raySource);
            }
            else
            {
                Debug.Assert(tree.LeafCount == 1);
                //Only one child in the tree. Handle it as a special case.
                int a0Start = stackPointerA0;
                var node = tree.nodes;
                BroadcastNode(ref *node, out var nodeWide);
                for (int i = 0; i < rayCount; i += Vector<float>.Count)
                {
                    var bundleStart = rays + i;

                    var count = rayCount - i;
                    if (count > Vector<float>.Count)
                        count = Vector<float>.Count;
                    for (int j = 0; j < count; ++j)
                    {
                        TreeRayWide.GatherIntoFirstSlot(ref bundleStart[j], ref GatherScatter.GetOffsetInstance(ref rayBundle, j));
                    }
                    Intersect(ref rayBundle, ref nodeWide.MinA, ref nodeWide.MaxA, out var tA, out var aIntersected);
                    for (int j = 0; j < count; ++j)
                    {
                        //TODO: Examine codegen. Bounds checks MIGHT be elided, but if they aren't, we can work around them.
                        if (aIntersected[j] < 0)
                        {
                            rayIndicesA0[stackPointerA0++] = (ushort)(i + j);
                        }

                    }
                }
                var a0Count = stackPointerA0 - a0Start;
                if (a0Count > 0)
                {
                    ref var entry = ref stack[stackPointer++];
                    entry.NodeIndex = node->A.Index;
                    entry.RayCount = (short)a0Count;
                }
            }
            while (stackPointer > 0)
            {
                --stackPointer;
                ref var entry = ref stack[stackPointer];
                ushort* rayStackStart;
                //Move the ray stack pointer back to the start of the popped region. The test will read from the region and potentially push additional elements.
                //The pushes are guaranteed to never go beyond the region that was popped- a node cannot be traversed by more rays than its parent.
                //Further, the reads are guaranteed to complete before being overwritten. Each bundle is popped and processed before any dangerous pushes can occur.
                switch (entry.RayStack)
                {
                    case 0:
                        rayStackStart = (ushort*)rayIndicesA0.Memory + (stackPointerA0 -= entry.NodeIndex);
                        break;
                    case 1:
                        rayStackStart = (ushort*)rayIndicesB.Memory + (stackPointerB -= entry.NodeIndex);
                        break;
                    default:
                        rayStackStart = (ushort*)rayIndicesA1.Memory + (stackPointerA1 -= entry.NodeIndex);
                        break;
                }
                if (entry.NodeIndex >= 0)
                {
                    var rayStackSource = new TreeRaySource(rays, rayStackStart, entry.RayCount);
                    TestNode(tree.nodes + entry.NodeIndex, ref rayStackSource);
                }
                else
                {
                    //This is a leaf node.

                }
            }
        }
        public struct Ray
        {
            public Vector3 Origin;
            public Vector3 Direction;
        }
        public interface IRaySource
        {
            int RayCount { get; }
            ref Ray this[int rayIndex] { get; }
        }

        public void TestRays<TRaySource>(ref TRaySource raySource) where TRaySource : IRaySource
        {

        }


        public void Dispose()
        {
            pool.ReturnUnsafely(rayIndicesA0.Id);
            pool.ReturnUnsafely(rayIndicesB.Id);
            pool.ReturnUnsafely(rayIndicesA1.Id);
            pool.ReturnUnsafely(stack.Id);
            //Easier to catch bugs if the references get cleared.
            rayIndicesA0 = new Buffer<ushort>();
            rayIndicesB = new Buffer<ushort>();
            rayIndicesA1 = new Buffer<ushort>();
            stack = new Buffer<StackEntry>();
        }
    }
}
