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
    public struct RayData
    {
        public Vector3 Origin;
        public int Id;
        public Vector3 Direction;
    }

    /// <summary>
    /// Ray representation designed for quicker intersection against axis aligned bounding boxes.
    /// </summary>
    public struct TreeRay
    {
        public Vector3 OriginOverDirection;
        public float MaximumT;
        public Vector3 InverseDirection;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFrom(in Vector3 origin, in Vector3 direction, float maximumT, out TreeRay treeRay)
        {
            //Note that this division has two odd properties:
            //1) If the local direction has a near zero component, it is clamped to a nonzero but extremely small value. This is a hack, but it works reasonably well.
            //The idea is that any interval computed using such an inverse would be enormous. Those values will not be exactly accurate, but they will never appear as a result
            //because a parallel ray will never actually intersect the surface. The resulting intervals are practical approximations of the 'true' infinite intervals.
            //2) To compensate for the clamp and abs, we reintroduce the sign in the numerator.
            //TODO: There is a small chance that a gather/scatter vectorized implementation would be a win. Pretty questionable, though.
            treeRay.InverseDirection = new Vector3(direction.X < 0 ? -1 : 1, direction.Y < 0 ? -1 : 1, direction.Z < 0 ? -1 : 1) / Vector3.Max(new Vector3(1e-15f), Vector3.Abs(direction));
            treeRay.MaximumT = maximumT;
            treeRay.OriginOverDirection = origin * treeRay.InverseDirection;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateFrom(in Vector3 origin, in Vector3 direction, float maximumT, int id, out RayData rayData, out TreeRay treeRay)
        {
            rayData.Origin = origin;
            rayData.Id = id;
            rayData.Direction = direction;
            CreateFrom(origin, direction, maximumT, out treeRay);
        }
    }

    public unsafe interface IRaySource
    {
        int RayCount { get; }
        ref readonly RayData GetRay(int rayIndex);
        void GetRay(int rayIndex, out RayData* ray, out float* maximumT);
    }


    public unsafe struct RaySource : IRaySource
    {
        TreeRay* treeRays;
        RayData* rays;
        ushort* rayPointers;
        int rayCount;

        public RaySource(TreeRay* treeRays, RayData* rays, ushort* rayPointerStack, int rayCount)
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
        /// Gets pointers to the data for a ray.
        /// </summary>
        /// <param name="rayIndex">Index of the ray to grab.</param>
        /// <param name="ray">Pointer to the ray's origin and direction. Note that changing the ray's origin and direction mid-traversal will not change the path of the traversal, 
        /// but it will be visible by any future leafs impacted by this ray.</param>
        /// <param name="maximumT">Pointer to the maximum length of the ray in units of the ray's length.
        /// Decreasing this value will prevent the traversal from visiting more distant nodes later in the traversal.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetRay(int rayIndex, out RayData* ray, out float* maximumT)
        {
            Debug.Assert(rayIndex >= 0 && rayIndex < rayCount, "The ray index must be within 0 and RayCount - 1.");
            var remappedIndex = rayPointers[rayIndex];
            ray = rays + remappedIndex;
            maximumT = &treeRays[remappedIndex].MaximumT;
        }

        /// <summary>
        /// Gets a reference to the data for a ray.
        /// </summary>
        /// <param name="rayIndex">Index of the ray to grab.</param>
        /// <returns>Returns a reference to the ray in the ray source.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref readonly RayData GetRay(int rayIndex)
        {
            Debug.Assert(rayIndex >= 0 && rayIndex < rayCount, "The ray index must be within 0 and RayCount - 1.");
            return ref rays[rayPointers[rayIndex]];
        }
    }

    public interface IRayLeafTester
    {
        unsafe void TestLeaf(int leafIndex, RayData* rayData, float* maximumT);
    }
    public interface IBatchedRayLeafTester : IRayLeafTester
    {
        void RayTest(int leafIndex, ref RaySource rays);
    }


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
            public ushort RayCount;
            public byte RayStack;
            public byte Depth;
        }
        int stackPointer;
        Buffer<StackEntry> stack;
        BufferPool pool;
        int batchRayCount;
        Buffer<TreeRay> batchRays;
        Buffer<RayData> batchOriginalRays;
        int rayCapacity;
        int preallocatedTreeDepth;

        Buffer<int> fallbackStack;

        public readonly int RayCapacity { get { return rayCapacity; } }
        public readonly int RayCount { get { return batchRayCount; } }

        /// <summary>
        /// Constructs a ray batcher and initializes its backing resources.
        /// </summary>
        /// <param name="pool">Pool to pull resources from.</param>
        /// <param name="rayCapacity">Maximum number of rays to execute in each traversal.
        /// This should typically be chosen as the highest value which avoids spilling data out of L2 cache.</param>
        /// <param name="treeDepthForPreallocation">Tree depth to preallocate ray stack space for. If a traversal finds nodes deeper than this, a dynamic resize will be triggered.</param>
        public RayBatcher(BufferPool pool, int rayCapacity = 2048, int treeDepthForPreallocation = 24) : this()
        {
            this.pool = pool;
            batchRayCount = 0;
            pool.TakeAtLeast(rayCapacity, out batchRays);
            pool.TakeAtLeast(rayCapacity, out batchOriginalRays);
            Debug.Assert(rayCapacity <= ushort.MaxValue, $"The number of rays per traversal must be less than {ushort.MaxValue}.");

            //Note that this assumes the tree has a fixed maximum depth. Note a great idea in the long term.
            pool.TakeAtLeast(Tree.TraversalStackCapacity, out fallbackStack);
            ResizeRayStacks(rayCapacity, treeDepthForPreallocation);

            stackPointer = stackPointerA0 = stackPointerB = stackPointerA1 = 0;

        }

        void ResizeRayStacks(int rayCapacity, int treeDepthForPreallocation)
        {
            this.rayCapacity = rayCapacity;
            this.preallocatedTreeDepth = treeDepthForPreallocation;
            //The number of ray pointers on the stack is limited in the worst case to all rays per level of the tree.
            var preallocatedRayPointerCount = rayCapacity * treeDepthForPreallocation;
            pool.ResizeToAtLeast(ref rayIndicesA0, preallocatedRayPointerCount, stackPointerA0);
            pool.ResizeToAtLeast(ref rayIndicesB, preallocatedRayPointerCount, stackPointerB);
            pool.ResizeToAtLeast(ref rayIndicesA1, preallocatedRayPointerCount, stackPointerA1);
            //The number of stack entries is limited by the number of node entries (tree node count * 3) and the number of ray entries.
            //(Can't have more entries on the stack than total ray pointers, after all.)
            pool.ResizeToAtLeast(ref stack, Math.Min(preallocatedRayPointerCount, 3 << Math.Min(16, treeDepthForPreallocation)), stackPointer);
        }

        /// <summary>
        /// Ray representation designed for quicker intersection against axis aligned bounding boxes.
        /// </summary>
        struct TreeRayWide
        {
            public Vector3Wide OriginOverDirection;
            public Vector<float> MaximumT;
            public Vector3Wide InverseDirection;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void GatherIntoFirstSlot(ref TreeRay ray, ref TreeRayWide wide)
            {
                GatherScatter.GetFirst(ref wide.OriginOverDirection.X) = ray.OriginOverDirection.X;
                GatherScatter.GetFirst(ref wide.OriginOverDirection.Y) = ray.OriginOverDirection.Y;
                GatherScatter.GetFirst(ref wide.OriginOverDirection.Z) = ray.OriginOverDirection.Z;
                GatherScatter.GetFirst(ref wide.MaximumT) = ray.MaximumT;
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
            Vector3Wide.Broadcast(node.A.Min, out wide.MinA);
            Vector3Wide.Broadcast(node.A.Max, out wide.MaxA);
            Vector3Wide.Broadcast(node.B.Min, out wide.MinB);
            Vector3Wide.Broadcast(node.B.Max, out wide.MaxB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Intersect(ref TreeRayWide ray, ref Vector3Wide min, ref Vector3Wide max, out Vector<float> tMin, out Vector<int> intersected)
        {
            //TODO: This is a good spot for FMA if/when we swap over to platform intrinsics.
            var tX0 = min.X * ray.InverseDirection.X - ray.OriginOverDirection.X;
            var tX1 = max.X * ray.InverseDirection.X - ray.OriginOverDirection.X;
            var tMinX = Vector.Min(tX0, tX1);
            var tMaxX = Vector.Max(tX0, tX1);

            var tY0 = min.Y * ray.InverseDirection.Y - ray.OriginOverDirection.Y;
            var tY1 = max.Y * ray.InverseDirection.Y - ray.OriginOverDirection.Y;
            var tMinY = Vector.Min(tY0, tY1);
            var tMaxY = Vector.Max(tY0, tY1);

            var tZ0 = min.Z * ray.InverseDirection.Z - ray.OriginOverDirection.Z;
            var tZ1 = max.Z * ray.InverseDirection.Z - ray.OriginOverDirection.Z;
            var tMinZ = Vector.Min(tZ0, tZ1);
            var tMaxZ = Vector.Max(tZ0, tZ1);

            tMin = Vector.Max(Vector.Max(Vector<float>.Zero, tMinX), Vector.Max(tMinY, tMinZ));
            var tMax = Vector.Min(Vector.Min(ray.MaximumT, tMaxX), Vector.Min(tMaxY, tMaxZ));
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

        unsafe void TestNode<TRaySource>(ref Node node, byte depth, ref TRaySource raySource) where TRaySource : struct, ITreeRaySource
        {
            int a0Start = stackPointerA0;
            int bStart = stackPointerB;
            int a1Start = stackPointerA1;
            BroadcastNode(ref node, out var wideNode);
            Unsafe.SkipInit(out TreeRayWide rayBundle);

            for (int bundleStartIndex = 0; bundleStartIndex < raySource.RayCount; bundleStartIndex += Vector<float>.Count)
            {
                var count = raySource.RayCount - bundleStartIndex;
                if (count > Vector<float>.Count)
                    count = Vector<float>.Count;

                for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                {
                    var rayIndex = raySource[bundleStartIndex + innerIndex];
                    TreeRayWide.GatherIntoFirstSlot(ref batchRays[rayIndex], ref GatherScatter.GetOffsetInstance(ref rayBundle, innerIndex));
                }
                Intersect(ref rayBundle, ref wideNode.MinA, ref wideNode.MaxA, out var tA, out var aIntersected);
                Intersect(ref rayBundle, ref wideNode.MinB, ref wideNode.MaxB, out var tB, out var bIntersected);
                //There are three potential stack regions that a single ray can go based on when and if each node child is intersected:
                //rayIndicesA0: A first or A only
                //rayIndicesB:  B is intersected at all
                //rayIndicesA1: A after B
                var aFirst = Vector.LessThanOrEqual(tA, tB);
                for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                {
                    ushort rayPointerIndex = (ushort)raySource[bundleStartIndex + innerIndex];
                    var bSlotIntersected = bIntersected[innerIndex] < 0;
                    if (aIntersected[innerIndex] < 0)
                    {
                        if (bSlotIntersected)
                        {
                            if (aFirst[innerIndex] < 0)
                            {
                                rayIndicesA0[stackPointerA0++] = rayPointerIndex;
                                rayIndicesB[stackPointerB++] = rayPointerIndex;
                            }
                            else
                            {
                                rayIndicesB[stackPointerB++] = rayPointerIndex;
                                rayIndicesA1[stackPointerA1++] = rayPointerIndex;
                            }
                        }
                        else
                        {
                            rayIndicesA0[stackPointerA0++] = rayPointerIndex;
                        }
                    }
                    else if (bSlotIntersected)
                    {
                        rayIndicesB[stackPointerB++] = rayPointerIndex;
                    }
                }
            }
            Debug.Assert(depth < 255,
                "We represent the depth as a byte under the assumption that there won't be any absurdly degenerate tree with extreme depth. This may be a poor assumption." +
                "If you encounter this, consider reporting it as an issue on github. You can work around it by using a larger datatype for StackEntry.Depth, but I should probably also fix" +
                "whatever caused a tree to generate so many levels if at all possible.");
            byte newDepth = (byte)(depth + 1);
            if (newDepth > preallocatedTreeDepth)
            {
                //We were not aggressive enough in preallocating for the ray stacks, apparently. Resize them aggressively.
                ResizeRayStacks(rayCapacity, Math.Max(preallocatedTreeDepth * 2, 1));
            }

            var a1Count = stackPointerA1 - a1Start;
            if (a1Count > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node.A.Index;
                newEntry.RayCount = (ushort)a1Count;
                newEntry.RayStack = 2;
                newEntry.Depth = newDepth;
            }
            var bCount = stackPointerB - bStart;
            if (bCount > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node.B.Index;
                newEntry.RayCount = (ushort)bCount;
                newEntry.RayStack = 1;
                newEntry.Depth = newDepth;
            }
            var a0Count = stackPointerA0 - a0Start;
            if (a0Count > 0)
            {
                ref var newEntry = ref stack[stackPointer++];
                newEntry.NodeIndex = node.A.Index;
                newEntry.RayCount = (ushort)a0Count;
                newEntry.RayStack = 0;
                newEntry.Depth = newDepth;
            }

        }

        /// <summary>
        /// Tests any batched rays against the given tree.
        /// </summary>
        /// <param name="tree">Tree to test the accumulated rays against.</param>
        public unsafe void TestRays<TLeafTester>(ref Tree tree, ref TLeafTester leafTester) where TLeafTester : IBatchedRayLeafTester
        {
            Debug.Assert(stackPointerA0 == 0 && stackPointerB == 0 && stackPointerA1 == 0 && stackPointer == 0,
                "At the beginning of the traversal, there should exist no entries on the traversal stack.");
            Debug.Assert(tree.ComputeMaximumDepth() < fallbackStack.Length, "At the moment, we assume that no tree will have more than 256 levels. " +
                "This isn't a hard guarantee; if you hit this, please report it- it probably means there is some goofy pathological case badness in the builder or refiner." +
                "Would be nice to replace this with a properly tracked tree depth so correctness isn't conditional.");
            if (tree.LeafCount == 0)
                return;

            //The traversal begins by assuming an implicit stack entry for the root node containing all ray pointers from 0 to rayCount-1.
            TreeRayWide rayBundle = default;

            if (tree.LeafCount >= 2)
            {
                var raySource = new RootRaySource(batchRayCount);
                TestNode(ref tree.Nodes[0], 0, ref raySource);
            }
            else
            {
                Debug.Assert(tree.LeafCount == 1);
                //Only one child in the tree. Handle it as a special case.
                int a0Start = stackPointerA0;
                ref var node = ref tree.Nodes[0];
                BroadcastNode(ref node, out var nodeWide);
                for (int bundleStartIndex = 0; bundleStartIndex < batchRayCount; bundleStartIndex += Vector<float>.Count)
                {
                    var bundleStart = batchRays.Memory + bundleStartIndex;

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
                    entry.NodeIndex = node.A.Index;
                    entry.RayCount = (ushort)a0Count;
                    entry.RayStack = 0;
                    entry.Depth = 1;
                }
            }
            while (stackPointer > 0)
            {
                //Move the ray stack pointer back to the start of the popped region. The test will read from the region and potentially push additional elements.
                //The pushes are guaranteed to never go beyond the region that was popped- a node cannot be traversed by more rays than its parent.
                //Further, the reads are guaranteed to complete before being overwritten. Each bundle is popped and processed before any dangerous pushes can occur.
                --stackPointer;
                ref var entry = ref stack[stackPointer];
                var rayStackStart = entry.RayStack switch
                {
                    0 => rayIndicesA0.Memory + (stackPointerA0 -= entry.RayCount),
                    1 => rayIndicesB.Memory + (stackPointerB -= entry.RayCount),
                    _ => rayIndicesA1.Memory + (stackPointerA1 -= entry.RayCount),
                };
                if (entry.RayCount >= 3)
                //if(true)
                {
                    //There are enough rays that we can justify continuing this vectorized approach.
                    if (entry.NodeIndex >= 0)
                    {
                        var rayStackSource = new TreeRaySource(rayStackStart, entry.RayCount);
                        TestNode(ref tree.Nodes[entry.NodeIndex], entry.Depth, ref rayStackSource);
                    }
                    else
                    {
                        //This is a leaf node.
                        var rayStackSource = new RaySource(batchRays.Memory, batchOriginalRays.Memory, rayStackStart, entry.RayCount);
                        leafTester.RayTest(Tree.Encode(entry.NodeIndex), ref rayStackSource);
                    }
                }
                else
                {
                    //Not enough rays remain to justify group tests. Fall back to a per-ray traversal.
                    for (int i = 0; i < entry.RayCount; ++i)
                    {
                        var rayIndex = rayStackStart[i];
                        tree.RayCast(entry.NodeIndex, batchRays.Memory + rayIndex, batchOriginalRays.Memory + rayIndex, fallbackStack.Memory, ref leafTester);
                    }
                }
            }
            Debug.Assert(stackPointerA0 == 0 && stackPointerB == 0 && stackPointerA1 == 0 && stackPointer == 0,
                "By the end of the traversal, there should exist no entries on the traversal stack.");
        }


        /// <summary>
        /// Adds a ray to the batcher. Returns true if the batcher has reached maximum ray capacity and needs to be reset in order to continue adding rays.
        /// </summary>
        /// <param name="origin">Origin of the ray to test against the tree.</param>
        /// <param name="direction">Direction of the ray to test against the tree.</param>
        /// <param name="maximumT">Maximum distance that the ray will travel in units of the ray's length.</param>
        /// <param name="id">Identifier value for the ray. Leaf tests will have access to the id.</param>
        /// <returns>True if the batcher is full and requires a call to ResetRays before adding any more rays, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Add(ref Vector3 origin, ref Vector3 direction, float maximumT, int id = 0)
        {
            Debug.Assert(batchRayCount >= 0 && batchRayCount < rayCapacity,
                "The accumulated rays must not exceed the maximum count per traversal; make sure ResetRays was called following a call to Add that returned true.");
            var rayIndex = batchRayCount++;
            ref var originalRay = ref batchOriginalRays[rayIndex];
            originalRay.Origin = origin;
            originalRay.Id = id;
            originalRay.Direction = direction;
            TreeRay.CreateFrom(origin, direction, maximumT, id, out batchOriginalRays[rayIndex], out batchRays[rayIndex]);
            return batchRayCount == rayCapacity;
        }

        /// <summary>
        /// Resets the accumulated ray count to zero.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ResetRays()
        {
            batchRayCount = 0;
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
