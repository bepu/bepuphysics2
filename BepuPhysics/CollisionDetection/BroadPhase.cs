using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Numerics;

namespace BepuPhysics.CollisionDetection
{
    public unsafe class BroadPhase : IDisposable
    {
        internal Buffer<CollidableReference> activeLeaves;
        internal Buffer<CollidableReference> staticLeaves;
        public Tree ActiveTree;
        public Tree StaticTree;
        Tree.RefitAndRefineMultithreadedContext activeRefineContext;
        //TODO: static trees do not need to do nearly as much work as the active; this will change in the future.
        Tree.RefitAndRefineMultithreadedContext staticRefineContext;

        public BroadPhase(BufferPool pool, int initialActiveLeafCapacity = 4096, int initialStaticLeafCapacity = 8192)
        {
            ActiveTree = new Tree(pool, initialActiveLeafCapacity);
            StaticTree = new Tree(pool, initialStaticLeafCapacity);
            pool.SpecializeFor<CollidableReference>().Take(initialActiveLeafCapacity, out activeLeaves);
            pool.SpecializeFor<CollidableReference>().Take(initialStaticLeafCapacity, out staticLeaves);

            activeRefineContext = new Tree.RefitAndRefineMultithreadedContext();
            staticRefineContext = new Tree.RefitAndRefineMultithreadedContext();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int Add(CollidableReference collidable, ref BoundingBox bounds, Tree tree, ref Buffer<CollidableReference> leaves)
        {
            var leafIndex = tree.Add(ref bounds);
            if (leafIndex >= leaves.Length)
            {
                tree.Pool.SpecializeFor<CollidableReference>().Resize(ref leaves, tree.LeafCount + 1, leaves.Length);
            }
            leaves[leafIndex] = collidable;
            return leafIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool RemoveAt(int index, Tree tree, ref Buffer<CollidableReference> leaves, out CollidableReference movedLeaf)
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
            return Add(collidable, ref bounds, ActiveTree, ref activeLeaves);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveActiveAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, ActiveTree, ref activeLeaves, out movedLeaf);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int AddStatic(CollidableReference collidable, ref BoundingBox bounds)
        {
            return Add(collidable, ref bounds, StaticTree, ref staticLeaves);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveStaticAt(int index, out CollidableReference movedLeaf)
        {
            return RemoveAt(index, StaticTree, ref staticLeaves, out movedLeaf);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetBoundsPointers(int broadPhaseIndex, Tree tree, out Vector3* minPointer, out Vector3* maxPointer)
        {
            var leaf = tree.Leaves[broadPhaseIndex];
            var nodeChild = (&tree.nodes[leaf.NodeIndex].A) + leaf.ChildIndex;
            minPointer = &nodeChild->Min;
            maxPointer = &nodeChild->Max;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetActiveBoundsPointers(int index, out Vector3* minPointer, out Vector3* maxPointer)
        {
            GetBoundsPointers(index, ActiveTree, out minPointer, out maxPointer);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetStaticBoundsPointers(int index, out Vector3* minPointer, out Vector3* maxPointer)
        {
            GetBoundsPointers(index, StaticTree, out minPointer, out maxPointer);
        }

        int frameIndex;
        public void Update(IThreadDispatcher threadDispatcher = null)
        {
            if (frameIndex == int.MaxValue)
                frameIndex = 0;
            if (threadDispatcher != null)
            {
                activeRefineContext.RefitAndRefine(ActiveTree, threadDispatcher, frameIndex);
            }
            else
            {
                ActiveTree.RefitAndRefine(frameIndex);
            }

            //TODO: for now, the inactive/static tree is simply updated like another active tree. This is enormously inefficient compared to the ideal-
            //by nature, static and inactive objects do not move every frame!
            //This should be replaced by a dedicated inactive/static refinement approach. It should also run alongside the active tree to extract more parallelism;
            //in other words, generate jobs from both trees and dispatch over all of them together. No internal dispatch.
            if (threadDispatcher != null)
            {
                staticRefineContext.RefitAndRefine(StaticTree, threadDispatcher, frameIndex);
            }
            else
            {
                StaticTree.RefitAndRefine(frameIndex);
            }
            //++frameIndex;
        }

        /// <summary>
        /// Clears out the broad phase's structures without releasing any resources.
        /// </summary>
        public void Clear()
        {
            ActiveTree.Clear();
            StaticTree.Clear();
        }

        void EnsureCapacity(Tree tree, ref Buffer<CollidableReference> leaves, int capacity)
        {
            if (tree.Leaves.Length < capacity)
                tree.Resize(capacity);
            if (leaves.Length < capacity)
                tree.Pool.SpecializeFor<CollidableReference>().Resize(ref leaves, capacity, tree.LeafCount);
        }
        void ResizeCapacity(Tree tree, ref Buffer<CollidableReference> leaves, int capacity)
        {
            capacity = Math.Max(capacity, tree.LeafCount);
            if (tree.Leaves.Length != BufferPool<Leaf>.GetLowestContainingElementCount(capacity))
                tree.Resize(capacity);
            if (leaves.Length != BufferPool<CollidableReference>.GetLowestContainingElementCount(capacity))
                tree.Pool.SpecializeFor<CollidableReference>().Resize(ref leaves, capacity, tree.LeafCount);
        }
        void Dispose(Tree tree, ref Buffer<CollidableReference> leaves)
        {
            tree.Pool.SpecializeFor<CollidableReference>().Return(ref leaves);
            tree.Dispose();
        }
        /// <summary>
        /// Ensures that the broad phase structures can hold at least the given number of leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void EnsureCapacity(int activeCapacity, int staticCapacity)
        {
            EnsureCapacity(ActiveTree, ref activeLeaves, activeCapacity);
            EnsureCapacity(StaticTree, ref staticLeaves, staticCapacity);
        }

        /// <summary>
        /// Resizes the broad phase structures to hold the given number of leaves. Note that this is conservative; it will never orphan any existing leaves.
        /// </summary>
        /// <param name="activeCapacity">Number of leaves to allocate space for in the active tree.</param>
        /// <param name="staticCapacity">Number of leaves to allocate space for in the static tree.</param>
        public void Resize(int activeCapacity, int staticCapacity)
        {
            ResizeCapacity(ActiveTree, ref activeLeaves, activeCapacity);
            ResizeCapacity(StaticTree, ref staticLeaves, staticCapacity);
        }

        /// <summary>
        /// Releases memory used by the broad phase. Leaves the broad phase unusable.
        /// </summary>
        public void Dispose()
        {
            Dispose(ActiveTree, ref activeLeaves);
            Dispose(StaticTree, ref staticLeaves);
        }

    }
}
