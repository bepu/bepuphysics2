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
        unsafe void RayTest(CollidableReference collidable, RayData* rayData, float* maximumT);
    }
    public interface IBroadPhaseBatchedRayTester : IBroadPhaseRayTester
    {
        void RayTest(CollidableReference collidable, ref RaySource rays);
    }

    /// <summary>
    /// Helps test the broad phase's active and static trees with a custom leaf tester.
    /// </summary>
    /// <typeparam name="TRayTester">Type used to test rays against leaves.</typeparam>
    public struct BroadPhaseRayBatcher<TRayTester> : IDisposable where TRayTester : struct, IBroadPhaseBatchedRayTester
    {
        BroadPhase broadPhase;
        RayBatcher batcher;

        struct LeafTester : IBatchedRayLeafTester
        {
            public TRayTester RayTester;
            public Buffer<CollidableReference> Leaves;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void RayTest(int leafIndex, ref RaySource rays)
            {
                RayTester.RayTest(Leaves[leafIndex], ref rays);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void TestLeaf(int leafIndex, RayData* rayData, float* maximumT)
            {
                RayTester.RayTest(Leaves[leafIndex], rayData, maximumT);
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
                batcher.TestRays(ref broadPhase.ActiveTree, ref activeTester);
                batcher.TestRays(ref broadPhase.StaticTree, ref staticTester);
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
                batcher.TestRays(ref broadPhase.ActiveTree, ref activeTester);
                batcher.TestRays(ref broadPhase.StaticTree, ref staticTester);
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

    /// <summary>
    /// Tests batches of rays against the simulation.
    /// </summary>
    /// <typeparam name="TRayHitHandler">Type used to handle hits against objects in the simulation.</typeparam>
    public struct SimulationRayBatcher<TRayHitHandler> : IDisposable where TRayHitHandler : struct, IRayHitHandler
    {
        struct Dispatcher : IBroadPhaseBatchedRayTester
        {
            public Simulation Simulation;
            public ShapeHitHandler HitHandler;

            public struct ShapeHitHandler : IShapeRayHitHandler
            {
                public TRayHitHandler HitHandler;
                public CollidableReference Reference;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public bool AllowTest(int childIndex)
                {
                    return HitHandler.AllowTest(Reference, childIndex);
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public unsafe void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, int childIndex)
                {
                    HitHandler.OnRayHit(ray, ref maximumT, t, normal, Reference, childIndex);
                }
            }


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void RayTest(CollidableReference reference, ref RaySource rays)
            {
                if (HitHandler.HitHandler.AllowTest(reference))
                {
                    Simulation.GetPoseAndShape(reference, out var pose, out var shape);
                    HitHandler.Reference = reference;
                    Simulation.Shapes[shape.Type].RayTest(shape.Index, *pose, ref rays, ref HitHandler);
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void RayTest(CollidableReference reference, RayData* rayData, float* maximumT)
            {
                if (HitHandler.HitHandler.AllowTest(reference))
                {
                    Simulation.GetPoseAndShape(reference, out var pose, out var shape);
                    HitHandler.Reference = reference;
                    Simulation.Shapes[shape.Type].RayTest(shape.Index, *pose, *rayData, ref *maximumT, ref HitHandler);
                }
            }

        }

        BroadPhaseRayBatcher<Dispatcher> batcher;

        public SimulationRayBatcher(BufferPool pool, Simulation simulation, TRayHitHandler hitHandler, int batcherRayCapacity = 2048)
        {
            Dispatcher dispatcher = default;
            dispatcher.Simulation = simulation;
            dispatcher.HitHandler.HitHandler = hitHandler;
            batcher = new BroadPhaseRayBatcher<Dispatcher>(pool, simulation.BroadPhase, dispatcher, batcherRayCapacity);
        }

        /// <summary>
        /// Adds a ray to the batcher to test against the simulation.
        /// If the underlying ray batcher hits its maximum capacity, all the accumulated rays will be tested against the simulation and the accumulator will be reset.
        /// </summary>
        /// <param name="origin">Origin of the ray to test against the simulation.</param>
        /// <param name="direction">Direction of the ray to test against the simulation.</param>
        /// <param name="maximumT">Maximum distance that the ray will travel in units of the ray's length.</param>
        /// <param name="id">Identifier value for the ray. Callbacks will have access to the id.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(ref Vector3 origin, ref Vector3 direction, float maximumT, int id = 0)
        {
            batcher.Add(ref origin, ref direction, maximumT, id);
        }

        /// <summary>
        /// Tests any accumulated rays against the broad phase trees and then resets the batcher.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush()
        {
            batcher.Flush();
        }

        public void Dispose()
        {
            batcher.Dispose();
        }
    }
}
