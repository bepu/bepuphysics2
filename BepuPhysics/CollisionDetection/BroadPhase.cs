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
    public unsafe partial class BroadPhase : IDisposable
    {
        internal Buffer<CollidableReference> activeLeaves;
        internal Buffer<CollidableReference> staticLeaves;
        public BufferPool Pool { get; private set; }
        public Tree ActiveTree;
        public Tree StaticTree;
        Tree.RefitAndRefineMultithreadedContext activeRefineContext;
        //TODO: static trees do not need to do nearly as much work as the active; this will change in the future.
        Tree.RefitAndRefineMultithreadedContext staticRefineContext;

        public BroadPhase(BufferPool pool, int initialActiveLeafCapacity = 4096, int initialStaticLeafCapacity = 8192)
        {
            Pool = pool;
            ActiveTree = new Tree(pool, initialActiveLeafCapacity);
            StaticTree = new Tree(pool, initialStaticLeafCapacity);
            pool.TakeAtLeast(initialActiveLeafCapacity, out activeLeaves);
            pool.TakeAtLeast(initialStaticLeafCapacity, out staticLeaves);

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
            return Add(collidable, ref bounds, ref ActiveTree, Pool, ref activeLeaves);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveActiveAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref ActiveTree, ref activeLeaves, out movedLeaf);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int AddStatic(CollidableReference collidable, ref BoundingBox bounds)
        {
            return Add(collidable, ref bounds, ref StaticTree, Pool, ref staticLeaves);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveStaticAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ref StaticTree, ref staticLeaves, out movedLeaf);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetBoundsPointers(int broadPhaseIndex, ref Tree tree, out Vector3* minPointer, out Vector3* maxPointer)
        {
            var leaf = tree.Leaves[broadPhaseIndex];
            var nodeChild = (&tree.NodesPointer[leaf.NodeIndex].A) + leaf.ChildIndex;
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
            EnsureCapacity(ref ActiveTree, ref activeLeaves, activeCapacity);
            EnsureCapacity(ref StaticTree, ref staticLeaves, staticCapacity);
        }

        /// <summary>
        /// Resizes the broad phase structures to hold the given number of leaves. Note that this is conservative; it will never orphan any existing leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void Resize(int activeCapacity, int staticCapacity)
        {
            ResizeCapacity(ref ActiveTree, ref activeLeaves, activeCapacity);
            ResizeCapacity(ref StaticTree, ref staticLeaves, staticCapacity);
        }

        /// <summary>
        /// Releases memory used by the broad phase. Leaves the broad phase unusable.
        /// </summary>
        public void Dispose()
        {
            Dispose(ref ActiveTree, ref activeLeaves);
            Dispose(ref StaticTree, ref staticLeaves);
        }

    }
}
