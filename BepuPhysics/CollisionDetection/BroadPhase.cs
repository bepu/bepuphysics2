using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Numerics;
using BepuPhysics.Trees;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Manages scene acceleration structures for collision detection and queries.
    /// </summary>
    public unsafe partial class BroadPhase : IDisposable
    {
        /// <summary>
        /// Collidable references contained within the <see cref="ActiveTree"/>. Note that values at or beyond the <see cref="ActiveTree"/>.LeafCount are not defined.
        /// </summary>
        public Buffer<CollidableReference> ActiveLeaves;
        /// <summary>
        /// Collidable references contained within the <see cref="StaticTree"/>. Note that values at or beyond <see cref="StaticTree"/>.LeafCount are not defined.
        /// </summary>
        public Buffer<CollidableReference> StaticLeaves;
        /// <summary>
        /// Pool used by the broad phase.
        /// </summary>
        public BufferPool Pool { get; private set; }
        /// <summary>
        /// Tree containing wakeful bodies.
        /// </summary>
        public Tree ActiveTree;
        /// <summary>
        /// Tree containing sleeping bodies and statics.
        /// </summary>
        public Tree StaticTree;
        Tree.RefitAndRefineMultithreadedContext activeRefineContext;
        //TODO: static trees do not need to do nearly as much work as the active; this will change in the future.
        Tree.RefitAndRefineMultithreadedContext staticRefineContext;

        public BroadPhase(BufferPool pool, int initialActiveLeafCapacity = 4096, int initialStaticLeafCapacity = 8192)
        {
            Pool = pool;
            ActiveTree = new Tree(pool, initialActiveLeafCapacity);
            StaticTree = new Tree(pool, initialStaticLeafCapacity);
            pool.TakeAtLeast(initialActiveLeafCapacity, out ActiveLeaves);
            pool.TakeAtLeast(initialStaticLeafCapacity, out StaticLeaves);

            activeRefineContext = new Tree.RefitAndRefineMultithreadedContext();
            staticRefineContext = new Tree.RefitAndRefineMultithreadedContext();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int Add(CollidableReference collidable, ref BoundingBox bounds, ref Tree tree, BufferPool pool, ref Buffer<CollidableReference> leaves)
        {
            var leafIndex = tree.Add(ref bounds, pool);
            if (leafIndex >= leaves.Length)
            {
                pool.ResizeToAtLeast(ref leaves, tree.LeafCount + 1, leaves.Length);
            }
            leaves[leafIndex] = collidable;
            return leafIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool RemoveAt(int index, ref Tree tree, ref Buffer<CollidableReference> leaves, out CollidableReference movedLeaf)
        {
            Debug.Assert(index >= 0);
            var movedLeafIndex = tree.RemoveAt(index);
            if (movedLeafIndex >= 0)
            {
                movedLeaf = leaves[movedLeafIndex];
                leaves[index] = movedLeaf;
                return true;
            }
            movedLeaf = new CollidableReference();
            return false;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int AddActive(CollidableReference collidable, ref BoundingBox bounds)
        {
            return Add(collidable, ref bounds, ref ActiveTree, Pool, ref ActiveLeaves);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveActiveAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref ActiveTree, ref ActiveLeaves, out movedLeaf);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int AddStatic(CollidableReference collidable, ref BoundingBox bounds)
        {
            return Add(collidable, ref bounds, ref StaticTree, Pool, ref StaticLeaves);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveStaticAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref StaticTree, ref StaticLeaves, out movedLeaf);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetBoundsPointers(int broadPhaseIndex, ref Tree tree, out Vector3* minPointer, out Vector3* maxPointer)
        {
            var leaf = tree.Leaves[broadPhaseIndex];
            var nodeChild = (&tree.Nodes.Memory[leaf.NodeIndex].A) + leaf.ChildIndex;
            minPointer = &nodeChild->Min;
            maxPointer = &nodeChild->Max;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetActiveBoundsPointers(int index, out Vector3* minPointer, out Vector3* maxPointer)
        {
            GetBoundsPointers(index, ref ActiveTree, out minPointer, out maxPointer);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetStaticBoundsPointers(int index, out Vector3* minPointer, out Vector3* maxPointer)
        {
            GetBoundsPointers(index, ref StaticTree, out minPointer, out maxPointer);
        }

        /// <summary>
        /// Applies updated bounds to the given leaf index in the given tree, refitting the tree to match.
        /// </summary>
        /// <param name="broadPhaseIndex">Index of the leaf in the tree to update.</param>
        /// <param name="tree">Tree containing the leaf to update.</param>
        /// <param name="min">New minimum bounds for the leaf.</param>
        /// <param name="max">New maximum bounds for the leaf.</param>
        public unsafe static void UpdateBounds(int broadPhaseIndex, ref Tree tree, in Vector3 min, in Vector3 max)
        {
            GetBoundsPointers(broadPhaseIndex, ref tree, out var minPointer, out var maxPointer);
            *minPointer = min;
            *maxPointer = max;
            tree.RefitForNodeBoundsChange(tree.Leaves[broadPhaseIndex].NodeIndex);
        }

        /// <summary>
        /// Applies updated bounds to the given active leaf index, refitting the tree to match.
        /// </summary>
        /// <param name="broadPhaseIndex">Index of the leaf to update.</param>
        /// <param name="min">New minimum bounds for the leaf.</param>
        /// <param name="max">New maximum bounds for the leaf.</param>
        public void UpdateActiveBounds(int broadPhaseIndex, in Vector3 min, in Vector3 max)
        {
            UpdateBounds(broadPhaseIndex, ref ActiveTree, min, max);
        }
        /// <summary>
        /// Applies updated bounds to the given active leaf index, refitting the tree to match.
        /// </summary>
        /// <param name="broadPhaseIndex">Index of the leaf to update.</param>
        /// <param name="min">New minimum bounds for the leaf.</param>
        /// <param name="max">New maximum bounds for the leaf.</param>
        public void UpdateStaticBounds(int broadPhaseIndex, in Vector3 min, in Vector3 max)
        {
            UpdateBounds(broadPhaseIndex, ref StaticTree, min, max);
        }

        int frameIndex;
        public void Update(IThreadDispatcher threadDispatcher = null)
        {
            if (frameIndex == int.MaxValue)
                frameIndex = 0;
            if (threadDispatcher != null)
            {
                activeRefineContext.RefitAndRefine(ref ActiveTree, Pool, threadDispatcher, frameIndex);
            }
            else
            {
                ActiveTree.RefitAndRefine(Pool, frameIndex);
            }

            //TODO: for now, the inactive/static tree is simply updated like another active tree. This is enormously inefficient compared to the ideal-
            //by nature, static and inactive objects do not move every frame!
            //This should be replaced by a dedicated inactive/static refinement approach. It should also run alongside the active tree to extract more parallelism;
            //in other words, generate jobs from both trees and dispatch over all of them together. No internal dispatch.
            if (threadDispatcher != null)
            {
                staticRefineContext.RefitAndRefine(ref StaticTree, Pool, threadDispatcher, frameIndex);
            }
            else
            {
                StaticTree.RefitAndRefine(Pool, frameIndex);
            }
            ++frameIndex;
        }

        /// <summary>
        /// Clears out the broad phase's structures without releasing any resources.
        /// </summary>
        public void Clear()
        {
            ActiveTree.Clear();
            StaticTree.Clear();
        }

        void EnsureCapacity(ref Tree tree, ref Buffer<CollidableReference> leaves, int capacity)
        {
            if (tree.Leaves.Length < capacity)
                tree.Resize(Pool, capacity);
            if (leaves.Length < capacity)
                Pool.ResizeToAtLeast(ref leaves, capacity, tree.LeafCount);
        }
        void ResizeCapacity(ref Tree tree, ref Buffer<CollidableReference> leaves, int capacity)
        {
            capacity = Math.Max(capacity, tree.LeafCount);
            if (tree.Leaves.Length != BufferPool.GetCapacityForCount<Leaf>(capacity))
                tree.Resize(Pool, capacity);
            if (leaves.Length != BufferPool.GetCapacityForCount<CollidableReference>(capacity))
                Pool.ResizeToAtLeast(ref leaves, capacity, tree.LeafCount);
        }
        void Dispose(ref Tree tree, ref Buffer<CollidableReference> leaves)
        {
            Pool.Return(ref leaves);
            tree.Dispose(Pool);
        }
        /// <summary>
        /// Ensures that the broad phase structures can hold at least the given number of leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void EnsureCapacity(int activeCapacity, int staticCapacity)
        {
            EnsureCapacity(ref ActiveTree, ref ActiveLeaves, activeCapacity);
            EnsureCapacity(ref StaticTree, ref StaticLeaves, staticCapacity);
        }

        /// <summary>
        /// Resizes the broad phase structures to hold the given number of leaves. Note that this is conservative; it will never orphan any existing leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void Resize(int activeCapacity, int staticCapacity)
        {
            ResizeCapacity(ref ActiveTree, ref ActiveLeaves, activeCapacity);
            ResizeCapacity(ref StaticTree, ref StaticLeaves, staticCapacity);
        }

        /// <summary>
        /// Releases memory used by the broad phase. Leaves the broad phase unusable.
        /// </summary>
        public void Dispose()
        {
            Dispose(ref ActiveTree, ref ActiveLeaves);
            Dispose(ref StaticTree, ref StaticLeaves);
        }

    }
}
