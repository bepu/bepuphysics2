using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public interface IBroadPhaseRayTester
    {
        void RayTest(CollidableReference collidable, ref RaySource rays);
    }

    /// <summary>
    /// Helps test the broad phase's active and static trees with a custom leaf tester.
    /// </summary>
    /// <typeparam name="TRayTester">Type used to test rays against leaves.</typeparam>
    public struct BroadPhaseRayBatcher<TRayTester> : IDisposable where TRayTester : struct, IBroadPhaseRayTester
    {
        BroadPhase broadPhase;
        RayBatcher batcher;
        
        struct LeafTester : ILeafTester
        {
            public TRayTester RayTester;
            public Buffer<CollidableReference> Leaves;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void RayTest(int leafIndex, ref RaySource rays)
            {
                RayTester.RayTest(Leaves[leafIndex], ref rays);
            }
        }

        LeafTester activeTester;
        LeafTester staticTester;

        /// <summary>
        /// Constructs a ray batcher for the broad phase and initializes its backing resources.
        /// </summary>
        /// <param name="pool">Pool to pull resources from.</param>
        /// <param name="rayTester">Ray tester used to test leaves found by the broad phase tree traversals.</param>
        /// <param name="batcherRayCapacity">Maximum number of rays to execute in each traversal.
        /// This should typically be chosen as the highest value which avoids spilling data out of L2 cache.</param>
        public BroadPhaseRayBatcher(BufferPool pool, BroadPhase broadPhase, TRayTester rayTester, int batcherRayCapacity = 2048)
        {
            activeTester = new LeafTester { Leaves = broadPhase.activeLeaves, RayTester = rayTester };
            staticTester = new LeafTester { Leaves = broadPhase.staticLeaves, RayTester = rayTester };
            this.broadPhase = broadPhase;
            batcher = new RayBatcher(pool, batcherRayCapacity,
                Math.Max(8, 2 * SpanHelper.GetContainingPowerOf2(Math.Max(broadPhase.StaticTree.LeafCount, broadPhase.ActiveTree.LeafCount))));
        }

        /// <summary>
        /// Adds a ray to the batcher to test against the broad phase trees.
        /// If the underlying ray batcher hits its maximum capacity, all the accumulated rays will be tested against the broad phase trees and the accumulator will be reset.
        /// </summary>
        /// <param name="origin">Origin of the ray to test against the tree.</param>
        /// <param name="direction">Direction of the ray to test against the tree.</param>
        /// <param name="maximumT">Maximum distance that the ray will travel in units of the ray's length.</param>
        /// <param name="id">Identifier value for the ray. Leaf tests will have access to the id.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(ref Vector3 origin, ref Vector3 direction, float maximumT, int id = 0)
        {
            if (batcher.Add(ref origin, ref direction, maximumT, id))
            {
                //TODO: Note that this order implies we test against the active tree before the static tree. This should be revisited- there are many simulations in which 
                //testing the static tree first would be better because of more conservative maximumT values. Not immediately clear which case is dominant.
                batcher.TestRays(broadPhase.ActiveTree, ref activeTester);
                batcher.TestRays(broadPhase.StaticTree, ref staticTester);
                batcher.ResetRays();
            }
        }

        /// <summary>
        /// Tests any accumulated rays against the broad phase trees and then resets the batcher.
        /// </summary>
        public void Flush()
        {
            if (batcher.RayCount > 0)
            {
                //TODO: Similar to Add- order matters here for performance. Need testing to determine which order tends to be better.
                batcher.TestRays(broadPhase.ActiveTree, ref activeTester);
                batcher.TestRays(broadPhase.StaticTree, ref staticTester);
                batcher.ResetRays();
            }
        }

        /// <summary>
        /// Disposes the underlying batcher resources.
        /// </summary>
        public void Dispose()
        {
            batcher.Dispose();
        }
    }

}
