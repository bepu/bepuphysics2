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
    public struct Ray
    {
        public Vector3 Origin;
        public Vector3 Direction;
    }

    /// <summary>
    /// Ray representation designed for quicker intersection against axis aligned bounding boxes.
    /// </summary>
    struct TreeRay
    {
        public Vector3 OriginOverDirection;
        public float MaximumT;
        public Vector3 InverseDirection;
    }

    public unsafe struct LeafRaySource
    {
        TreeRay* treeRays;
        Ray* rays;
        ushort* rayPointers;
        int rayCount;

        internal LeafRaySource(TreeRay* treeRays, Ray* rays, ushort* rayPointerStack, int rayCount)
        {
            this.treeRays = treeRays;
            this.rays = rays;
            this.rayPointers = rayPointerStack;
            this.rayCount = rayCount;
        }

        /// <summary>
        /// Gets the number of rays in the batch.
        /// </summary>
        public int RayCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return rayCount; }
        }
        
        /// <summary>
        /// Gets pointers to the data for a ray that intersects the leaf's bounding box..
        /// </summary>
        /// <param name="rayIndex">Index of the ray affecting this leaf to grab.</param>
        /// <param name="ray">Pointer to the ray's origin and direction. Note that changing the ray's origin and direction mid-traversal will not change the path of the traversal, 
        /// but it will be visible by any future leafs impacted by this ray.</param>
        /// <param name="maximumT">Pointer to the maximum length of the ray in units of the ray's length.
        /// Decreasing this value will prevent the traversal from visiting more distant nodes later in the traversal.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetRay(int rayIndex, out Ray* ray, out float* maximumT)
        {
            Debug.Assert(rayIndex >= 0 && rayIndex < rayCount, "The ray index must be within 0 and RayCount - 1.");
            ray = rays + rayIndex;
            maximumT = &treeRays[rayIndex].MaximumT;
        }

    }

    public interface ILeafTester
    {
        void RayTest(ref LeafRaySource rays);
    }


    /// <summary>
    /// Reusable structure for testing large numbers of rays against trees.
    /// </summary>
    public struct RayBatcher<TLeafTester> : IDisposable where TLeafTester : struct, ILeafTester
    {
        //If you want a deeper explanation about this implementation, check out the Dynamic Ray Stream Traversal paper by Barringer and Akenine-Moller. It's basically the same.
        int stackPointerA0, stackPointerB, stackPointerA1;
        Buffer<ushort> rayIndicesA0, rayIndicesB, rayIndicesA1;
        struct StackEntry
        {
            public int NodeIndex;
            public ushort RayCount;
            public byte RayStack;
            public byte Depth;
        }
        int stackPointer;
        Buffer<StackEntry> stack;
        BufferPool pool;
        public Tree Tree;
        int batchRayCount;
        Buffer<TreeRay> batchRays;
        Buffer<Ray> batchOriginalRays;
        int maximumRaysPerTraversal;
        int preallocatedTreeDepth;

        TLeafTester leafTester;

        /// <summary>
        /// Constructs a ray batcher and initializes its backing resources.
        /// </summary>
        /// <param name="pool">Pool to pull resources from.</param>
        /// <param name="tree">Tree to target with rays initially.</param>
        /// <param name="leafTester">Leaf tester used to test leaves found by the tree traversal.</param>
        /// <param name="maximumRaysPerTraversal">Maximum number of rays to execute in each traversal.
        /// This should typically be chosen as the highest value which avoids spilling data out of L2 cache.</param>
        /// <param name="treeDepthForPreallocation">Tree depth to preallocate ray stack space for. If a traversal finds nodes deeper than this, a dynamic resize will be triggered.</param>
        public RayBatcher(BufferPool pool, Tree tree, TLeafTester leafTester, int maximumRaysPerTraversal = 2048, int treeDepthForPreallocation = 24) : this()
        {
            this.Tree = tree;
            this.pool = pool;
            this.leafTester = leafTester;
            batchRayCount = 0;
            pool.Take(maximumRaysPerTraversal, out batchRays);
            pool.Take(maximumRaysPerTraversal, out batchOriginalRays);
            Debug.Assert(maximumRaysPerTraversal <= ushort.MaxValue, $"The number of rays per traversal must be less than {ushort.MaxValue}; ray pointers are stored in 2 bytes.");

            ResizeRayStacks(maximumRaysPerTraversal, treeDepthForPreallocation);

            stackPointer = stackPointerA0 = stackPointerB = stackPointerA1 = 0;

        }

        void ResizeRayStacks(int maximumRaysPerTraversal, int treeDepthForPreallocation)
        {
            this.maximumRaysPerTraversal = maximumRaysPerTraversal;
            this.preallocatedTreeDepth = treeDepthForPreallocation;
            //The number of ray pointers on the stack is limited in the worst case to all rays per level of the tree.
            var preallocatedRayPointerCount = maximumRaysPerTraversal * treeDepthForPreallocation;
            pool.Resize(ref rayIndicesA0, preallocatedRayPointerCount, stackPointerA0);
            pool.Resize(ref rayIndicesB, preallocatedRayPointerCount, stackPointerB);
            pool.Resize(ref rayIndicesA1, preallocatedRayPointerCount, stackPointerA1);
            //The number of stack entries is limited by the number of node entries (tree node count * 3) and the number of ray entries.
            //(Can't have more entries on the stack than total ray pointers, after all.)
            pool.Resize(ref stack, Math.Min(preallocatedRayPointerCount, 3 << Math.Min(16, treeDepthForPreallocation)), stackPointer);
        }

        /// <summary>
        /// Ray representation designed for quicker intersection against axis aligned bounding boxes.
        /// </summary>
        struct TreeRayWide
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



        interface ITreeRaySource
        {
            int RayCount { get; }
            int this[int rayIndex] { get; }
        }

        unsafe struct RootRaySource : ITreeRaySource
        {
            int rayCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public RootRaySource(int rayCount)
            {
                this.rayCount = rayCount;
            }

            public int RayCount
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get { return rayCount; }
            }

            public int this[int rayIndex]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    return rayIndex;
                }
            }
        }

        unsafe struct TreeRaySource : ITreeRaySource
        {
            ushort* rayPointers;
            int rayCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public TreeRaySource(ushort* rayPointerStack, int rayCount)
            {
                this.rayPointers = rayPointerStack;
                this.rayCount = rayCount;
            }

            public int RayCount
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get { return rayCount; }
            }

            public int this[int rayIndex]
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    Debug.Assert(rayIndex >= 0 && rayIndex < RayCount, "The requested ray index must be within the source's region.");
                    return rayPointers[rayIndex];
                }
            }
        }

        unsafe void TestNode<TRayStackSource>(Node* node, byte depth, ref TRayStackSource raySource) where TRayStackSource : struct, ITreeRaySource
        {
            int a0Start = stackPointerA0;
            int bStart = stackPointerB;
            int a1Start = stackPointerA1;
            BroadcastNode(ref *node, out var wideNode);
            TreeRayWide rayBundle;

            for (int bundleStartIndex = 0; bundleStartIndex < raySource.RayCount; bundleStartIndex += Vector<float>.Count)
            {
                var count = raySource.RayCount - bundleStartIndex;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;
                for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                {
                    TreeRayWide.GatherIntoFirstSlot(ref batchRays[raySource[bundleStartIndex + innerIndex]], ref GatherScatter.GetOffsetInstance(ref rayBundle, innerIndex));
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
                for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                {
                    //TODO: Examine codegen. Bounds checks MIGHT be elided, but if they aren't, we can work around them.
                    ushort rayPointerIndex = (ushort)(bundleStartIndex + innerIndex);
                    if (shouldAllocateRayToA0[innerIndex] < 0)
                    {
                        rayIndicesA0[stackPointerA0++] = rayPointerIndex;
                    }
                    else if (aIntersected[innerIndex] < 0)
                    {
                        //If A was intersected but did not qualify to fit into the A0 slot, it must come after B.
                        rayIndicesA1[stackPointerA1++] = rayPointerIndex;
                    }
                    if (bIntersected[innerIndex] < 0)
                    {
                        rayIndicesB[stackPointerB++] = rayPointerIndex;
                    }

                }
            }
            var a1Count = stackPointerA1 - a1Start;
            Debug.Assert(depth < 255,
                "We represent the depth as a byte under the assumption that there won't be any absurdly degenerate tree with extreme depth. This may be a poor assumption." +
                "If you encounter this, consider reporting it as an issue on github. You can work around it by using a larger datatype for StackEntry.Depth, but I should probably also fix" +
                "whatever caused a tree to generate so many levels if at all possible.");
            byte newDepth = (byte)(depth + 1);
            if(newDepth > preallocatedTreeDepth)
            {
                //We were not aggressive enough in preallocating for the ray stacks, apparently. Resize them aggressively.
                ResizeRayStacks(maximumRaysPerTraversal, Math.Max(preallocatedTreeDepth * 2, 1));
            }

            if (a1Count > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node->A.Index;
                newEntry.RayCount = (ushort)a1Count;
                newEntry.RayStack = 2;
                newEntry.Depth = newDepth;
            }
            var bCount = stackPointerB - bStart;
            if (bCount > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node->B.Index;
                newEntry.RayCount = (ushort)bCount;
                newEntry.RayStack = 1;
                newEntry.Depth = newDepth;
            }
            var a0Count = stackPointerA0 - a0Start;
            if (a0Count > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node->A.Index;
                newEntry.RayCount = (ushort)a0Count;
                newEntry.RayStack = 0;
                newEntry.Depth = newDepth;
            }

        }

        /// <summary>
        /// Tests any remaining batched rays against the Tree.
        /// </summary>
        public unsafe void Flush()
        {
            if (Tree.LeafCount == 0)
                return;

            //The traversal begins by assuming an implicit stack entry for the root node containing all ray pointers from 0 to rayCount-1.
            TreeRayWide rayBundle = default;

            if (Tree.LeafCount >= 2)
            {
                var raySource = new RootRaySource(batchRayCount);
                TestNode(Tree.nodes, 0, ref raySource);
            }
            else
            {
                Debug.Assert(Tree.LeafCount == 1);
                //Only one child in the tree. Handle it as a special case.
                int a0Start = stackPointerA0;
                var node = Tree.nodes;
                BroadcastNode(ref *node, out var nodeWide);
                for (int bundleStartIndex = 0; bundleStartIndex < batchRayCount; bundleStartIndex += Vector<float>.Count)
                {
                    var bundleStart = (TreeRay*)batchRays.Memory + bundleStartIndex;

                    var count = batchRayCount - bundleStartIndex;
                    if (count > Vector<float>.Count)
                        count = Vector<float>.Count;
                    for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                    {
                        TreeRayWide.GatherIntoFirstSlot(ref bundleStart[innerIndex], ref GatherScatter.GetOffsetInstance(ref rayBundle, innerIndex));
                    }
                    Intersect(ref rayBundle, ref nodeWide.MinA, ref nodeWide.MaxA, out var tA, out var aIntersected);
                    for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                    {
                        //TODO: Examine codegen. Bounds checks MIGHT be elided, but if they aren't, we can work around them.
                        if (aIntersected[innerIndex] < 0)
                        {
                            rayIndicesA0[stackPointerA0++] = (ushort)(bundleStartIndex + innerIndex);
                        }

                    }
                }
                var a0Count = stackPointerA0 - a0Start;
                if (a0Count > 0)
                {
                    ref var entry = ref stack[stackPointer++];
                    entry.NodeIndex = node->A.Index;
                    entry.RayCount = (byte)a0Count;
                    entry.RayStack = 0;
                    entry.Depth = 1;
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
                    var rayStackSource = new TreeRaySource(rayStackStart, entry.RayCount);
                    TestNode(Tree.nodes + entry.NodeIndex, entry.Depth, ref rayStackSource);
                }
                else
                {
                    //This is a leaf node.
                    var rayStackSource = new LeafRaySource((TreeRay*)batchRays.Memory, (Ray*)batchOriginalRays.Memory, rayStackStart, entry.RayCount);
                    leafTester.RayTest(ref rayStackSource);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(ref Vector3 origin, ref Vector3 direction, float maximumT)
        {
            Debug.Assert(batchRayCount < maximumRaysPerTraversal);
            ref var ray = ref batchRays[batchRayCount++];
            //Note that this division has two odd properties:
            //1) If the local direction has a near zero component, it is clamped to a nonzero but extremely small value. This is a hack, but it works reasonably well.
            //The idea is that any interval computed using such an inverse would be enormous. Those values will not be exactly accurate, but they will never appear as a result
            //because a parallel ray will never actually intersect the surface. The resulting intervals are practical approximations of the 'true' infinite intervals.
            //2) To compensate for the clamp and abs, we reintroduce the sign in the numerator.
            //TODO: There is a small chance that a gather/scatter vectorized implementation would be a win. Pretty questionable, though.
            ray.InverseDirection = new Vector3(direction.X < 0 ? -1 : 1, direction.Y < 0 ? -1 : 1, direction.Z < 0 ? -1 : 1) / Vector3.Max(new Vector3(1e-15f), Vector3.Abs(direction));
            ray.MaximumT = maximumT;
            ray.OriginOverDirection = origin * ray.InverseDirection;
            if (batchRayCount == maximumRaysPerTraversal)
            {
                Flush();
                batchRayCount = 0;
            }
        }

        /// <summary>
        /// Disposes all the resources backing the ray batcher.
        /// </summary>
        public void Dispose()
        {
            pool.ReturnUnsafely(rayIndicesA0.Id);
            pool.ReturnUnsafely(rayIndicesB.Id);
            pool.ReturnUnsafely(rayIndicesA1.Id);
            pool.ReturnUnsafely(stack.Id);
            pool.ReturnUnsafely(batchOriginalRays.Id);
            pool.ReturnUnsafely(batchRays.Id);
            //Easier to catch bugs if the references get cleared.
            this = default;
        }
    }
}
