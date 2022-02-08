using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;


namespace BepuPhysics.Trees
{
    public unsafe partial struct Tree
    {
        /// <summary>
        /// Buffer of nodes in the tree.
        /// </summary>
        public Buffer<Node> Nodes;
        /// <summary>
        /// Buffer of metanodes in the tree. Metanodes contain metadata that aren't read during most query operations but are useful for bookkeeping.
        /// </summary>
        public Buffer<Metanode> Metanodes;
        int nodeCount;
        /// <summary>
        /// Gets or sets the number of nodes in the tree.
        /// </summary>
        public int NodeCount
        {
            readonly get
            {
                return nodeCount;
            }
            set
            {
                nodeCount = value;
            }
        }

        /// <summary>
        /// Buffer of leaves in the tree.
        /// </summary>
        public Buffer<Leaf> Leaves;
        int leafCount;
        /// <summary>
        /// Gets or sets the number of leaves in the tree.
        /// </summary>
        public int LeafCount
        {
            readonly get
            {
                return leafCount;
            }
            set
            {
                leafCount = value;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AllocateNode()
        {
            Debug.Assert(Nodes.Length > nodeCount && Metanodes.Length > nodeCount,
                "Any attempt to allocate a node should not overrun the allocated nodes. For all operations that allocate nodes, capacity should be preallocated.");
            return nodeCount++;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AddLeaf(int nodeIndex, int childIndex)
        {
            Debug.Assert(leafCount < Leaves.Length,
                "Any attempt to allocate a leaf should not overrun the allocated leaves. For all operations that allocate leaves, capacity should be preallocated.");
            Leaves[leafCount] = new Leaf(nodeIndex, childIndex);
            return leafCount++;
        }

        /// <summary>
        /// Gets bounds pointerse for a leaf in the tree.
        /// </summary>
        /// <param name="leafIndex">Index of the leaf in the tree.</param>
        /// <param name="minPointer">Pointer to the minimum bounds vector in the tree.</param>
        /// <param name="maxPointer">Pointer to the maximum bounds vector in the tree.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void GetBoundsPointers(int leafIndex, out Vector3* minPointer, out Vector3* maxPointer)
        {
            var leaf = Leaves[leafIndex];
            var nodeChild = (&Nodes.Memory[leaf.NodeIndex].A) + leaf.ChildIndex;
            minPointer = &nodeChild->Min;
            maxPointer = &nodeChild->Max;
        }

        /// <summary>
        /// Applies updated bounds to the given leaf index in the tree, refitting the tree to match.
        /// </summary>
        /// <param name="leafIndex">Index of the leaf in the tree to update.</param>
        /// <param name="min">New minimum bounds for the leaf.</param>
        /// <param name="max">New maximum bounds for the leaf.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe readonly void UpdateBounds(int leafIndex, Vector3 min, Vector3 max)
        {
            GetBoundsPointers(leafIndex, out var minPointer, out var maxPointer);
            *minPointer = min;
            *maxPointer = max;
            RefitForNodeBoundsChange(Leaves[leafIndex].NodeIndex);
        }

        /// <summary>
        /// Constructs an empty tree.
        /// </summary>
        /// <param name="pool">Buffer pool to use to allocate resources in the tree.</param>
        /// <param name="initialLeafCapacity">Initial number of leaves to allocate room for.</param>
        public unsafe Tree(BufferPool pool, int initialLeafCapacity = 4096) : this()
        {
            if (initialLeafCapacity <= 0)
                throw new ArgumentException("Initial leaf capacity must be positive.");

            Resize(pool, initialLeafCapacity);

        }

        /// <summary>
        /// Loads a tree from a byte buffer created by the Serialize function.
        /// </summary>
        /// <param name="data">Data to load into the tree.</param>
        /// <param name="pool">Pool to use to create the tree.</param>
        public Tree(Span<byte> data, BufferPool pool)
        {
            if (data.Length <= 4)
                throw new ArgumentException($"Data is only {data.Length} bytes long; that's too small for even a header.");
            leafCount = Unsafe.As<byte, int>(ref data[0]);
            nodeCount = leafCount - 1;
            var leafByteCount = leafCount * sizeof(Leaf);
            var nodeByteCount = nodeCount * sizeof(Node);
            var metanodeByteCount = nodeCount * sizeof(Metanode);
            const int leavesStartIndex = 4;
            var nodesStartIndex = leavesStartIndex + leafByteCount;
            var metanodesStartIndex = nodesStartIndex + nodeByteCount;
            if (data.Length < leavesStartIndex + leafByteCount + nodeByteCount + metanodeByteCount)
                throw new ArgumentException($"Header suggested there were {leafCount} leaves, but there's not enough room in the data for that.");
            pool.Take(leafCount, out Leaves);
            pool.Take(nodeCount, out Nodes);
            pool.Take(nodeCount, out Metanodes);
            Unsafe.CopyBlockUnaligned(ref *(byte*)Leaves.Memory, ref data[leavesStartIndex], (uint)leafByteCount);
            Unsafe.CopyBlockUnaligned(ref *(byte*)Nodes.Memory, ref data[nodesStartIndex], (uint)nodeByteCount);
            Unsafe.CopyBlockUnaligned(ref *(byte*)Metanodes.Memory, ref data[metanodesStartIndex], (uint)metanodeByteCount);
        }

        /// <summary>
        /// Gets the number of bytes required to store the tree.
        /// </summary>
        /// <returns>Number of bytes required to store the tree.</returns>
        public readonly int GetSerializedByteCount()
        {
            return 4 + sizeof(Leaf) * LeafCount + (sizeof(Node) + sizeof(Metanode)) * NodeCount;
        }

        /// <summary>
        /// Writes a tree into a byte buffer.
        /// </summary>
        /// <param name="bytes">Buffer to hold the tree's data.</param>
        public readonly void Serialize(Span<byte> bytes)
        {
            var requiredSizeInBytes = GetSerializedByteCount();
            if (bytes.Length < requiredSizeInBytes)
                throw new ArgumentException($"Target span size {bytes.Length} is less than the required size of {requiredSizeInBytes}.");
            Unsafe.As<byte, int>(ref bytes[0]) = LeafCount;
            var leafByteCount = LeafCount * sizeof(Leaf);
            var nodeByteCount = NodeCount * sizeof(Node);
            var metanodeByteCount = NodeCount * sizeof(Metanode);
            const int leavesStartIndex = 4;
            var nodesStartIndex = leavesStartIndex + leafByteCount;
            var metanodesStartIndex = nodesStartIndex + nodeByteCount;
            Unsafe.CopyBlockUnaligned(ref bytes[4], ref *(byte*)Leaves.Memory, (uint)leafByteCount);
            Unsafe.CopyBlockUnaligned(ref bytes[nodesStartIndex], ref *(byte*)Nodes.Memory, (uint)nodeByteCount);
            Unsafe.CopyBlockUnaligned(ref bytes[metanodesStartIndex], ref *(byte*)Metanodes.Memory, (uint)metanodeByteCount);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Encode(int index)
        {
            return -1 - index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InitializeRoot()
        {
            //The root always exists, even if there are no children in it. Makes some bookkeeping simpler.
            nodeCount = 1;
            ref var rootMetanode = ref Metanodes[0];
            rootMetanode.Parent = -1;
            rootMetanode.IndexInParent = -1;
        }

        /// <summary>
        /// Resizes the buffers backing the tree's nodes and leaves. Will not shrink the buffers below the size needed by the currently resident nodes and leaves.
        /// </summary>
        /// <param name="pool">Pool from which to take and return resources.</param>
        /// <param name="targetLeafSlotCount">The desired number of available leaf slots.</param>
        public void Resize(BufferPool pool, int targetLeafSlotCount)
        {
            //Note that it's not safe to resize below the size of potentially used leaves. If the user wants to go smaller, they'll need to explicitly deal with the leaves somehow first.
            var leafCapacityForTarget = BufferPool.GetCapacityForCount<Leaf>(Math.Max(leafCount, targetLeafSlotCount));
            //Adding incrementally checks the capacity of leaves, and issues a resize if there isn't enough space. But it doesn't check nodes.
            //You could change that, but for now, we simply ensure that the node array has sufficient room to hold everything in the resized leaf array.
            var nodeCapacityForTarget = BufferPool.GetCapacityForCount<Node>(Math.Max(nodeCount, leafCapacityForTarget - 1));
            var metanodeCapacityForTarget = BufferPool.GetCapacityForCount<Metanode>(Math.Max(nodeCount, leafCapacityForTarget - 1));
            bool wasAllocated = Leaves.Allocated;
            Debug.Assert(Leaves.Allocated == Nodes.Allocated);
            if (leafCapacityForTarget != Leaves.Length)
            {
                pool.ResizeToAtLeast(ref Leaves, leafCapacityForTarget, leafCount);
            }
            if (nodeCapacityForTarget != Nodes.Length)
            {
                pool.ResizeToAtLeast(ref Nodes, nodeCapacityForTarget, nodeCount);
            }
            if (metanodeCapacityForTarget != Metanodes.Length)
            {
                pool.ResizeToAtLeast(ref Metanodes, metanodeCapacityForTarget, nodeCount);
                //A node's RefineFlag must be 0, so just clear out the node set. 
                //TODO: This won't be necessary if we get rid of refineflags as a concept.
                Metanodes.Clear(nodeCount, Nodes.Length - nodeCount);
            }
            if (!wasAllocated)
            {
                InitializeRoot();
            }
        }


        /// <summary>
        /// Resets the tree to a fresh post-construction state, clearing out leaves and nodes but leaving the backing resources intact.
        /// </summary>
        public void Clear()
        {
            leafCount = 0;
            InitializeRoot();
        }

        /// <summary>
        /// Disposes the tree's backing resources, returning them to the Pool currently associated with the tree.
        /// </summary>
        /// <param name="pool">Pool to return resources to.</param>
        /// <remarks>Disposed trees can be reused if EnsureCapacity or Resize is used to rehydrate them.</remarks>
        public void Dispose(BufferPool pool)
        {
            Debug.Assert(Nodes.Allocated == Leaves.Allocated && Nodes.Allocated == Metanodes.Allocated, "Nodes and leaves should have consistent lifetimes.");
            if (Nodes.Allocated)
            {
                pool.Return(ref Nodes);
                pool.Return(ref Metanodes);
                pool.Return(ref Leaves);
            }
        }

        /// <summary>
        /// Tests if two tree references point to the same data.
        /// </summary>
        /// <param name="a">First tree to compare.</param>
        /// <param name="b">Second tree to compare.</param>
        /// <returns>True if the two trees have the same nodes and node count, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Equals(in Tree a, in Tree b)
        {
            return a.Nodes.Memory == b.Nodes.Memory && a.nodeCount == b.nodeCount;
        }

    }

}
