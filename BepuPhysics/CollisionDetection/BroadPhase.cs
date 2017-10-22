using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

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
        static void GetBoundsPointers(int broadPhaseIndex, Tree tree, out float* minPointer, out float* maxPointer)
        {
            var leaf = tree.Leaves[broadPhaseIndex];
            var nodeChild = (&tree.nodes[leaf.NodeIndex].A) + leaf.ChildIndex;
            minPointer = &nodeChild->Min.X;
            maxPointer = &nodeChild->Max.X;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetActiveBoundsPointers(int index, out float* minPointer, out float* maxPointer)
        {
            GetBoundsPointers(index, ActiveTree, out minPointer, out maxPointer);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetStaticBoundsPointers(int index, out float* minPointer, out float* maxPointer)
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
        }



        //TODO: EnsureCapacity and so on. Need them for the broadphase's own leaves sets. We COULD expose the underlying trees and let their sizes be managed separately,
        //or we can handle them at the level of the broadphase too. 

        public void Dispose()
        {
            ActiveTree.Dispose();
        }

    }
}
