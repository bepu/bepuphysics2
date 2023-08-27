﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos.Demos
{
    /// <summary>
    /// Shows one way of handling collision queries that require contact-level test accuracy.
    /// </summary>
    public class CollisionQueryDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 7, 20);
            camera.Yaw = 0;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(100, 1, 100))));

            var random = new Random(5);
            var shapeToDrop = new Box(1, 1, 1);
            var descriptionToDrop = BodyDescription.CreateDynamic(new Vector3(), shapeToDrop.ComputeInertia(1), Simulation.Shapes.Add(shapeToDrop), 0.01f);
            for (int i = 0; i < 128; ++i)
            {
                descriptionToDrop.Pose.Position = new Vector3(-5 + 10 * random.NextSingle(), 45 + 150 * random.NextSingle(), -5 + 10 * random.NextSingle());
                Simulation.Bodies.Add(descriptionToDrop);
            }

            //Add in a static object to test against. Note that the mesh triangles are one sided, so some of the queries whose centers are beneath the mesh do not generate any contacts.
            var mesh = DemoMeshHelper.CreateDeformedPlane(20, 20, (x, y) => { return new Vector3(x * 5 - 50, 3 * MathF.Sin(x) * MathF.Sin(y), y * 5 - 50); }, Vector3.One, BufferPool);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), Simulation.Shapes.Add(mesh)));
        }

        /// <summary>
        /// Provides callbacks for filtering and data collection to the CollisionBatcher we'll be using to test query shapes against the detected environment.
        /// </summary>
        public struct BatcherCallbacks : ICollisionCallbacks
        {
            /// <summary>
            /// Set to true for a pair if a nonnegative depth is detected by collision testing.
            /// </summary>
            public Buffer<bool> QueryWasTouched;

            //These callbacks provide filtering and reporting for pairs being processed by the collision batcher.
            //"Pair id" refers to the identifier given to the pair when it was added to the batcher.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowCollisionTesting(int pairId, int childA, int childB)
            {
                //If you wanted to filter based on the children of an encountered nonconvex object, here would be the place to do it.
                //The pairId could be used to look up the involved objects and any metadata necessary for filtering.
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold)
            {
                //If you need to do any processing on a child manifold before it goes back to a nonconvex processing pass, this is the place to do it.
                //Convex-convex pairs won't invoke this function at all.
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnPairCompleted<TManifold>(int pairId, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                //This function hands off the completed manifold with all postprocessing (NonconvexReduction, MeshReduction, etc.) complete.
                //For the purposes of this demo, we're interested in boolean collision testing.
                //(This process was a little overkill for a pure boolean test, but there is no pure boolean path right now because the contact manifold generators turned out fast enough.
                //And if you find yourself wanting contact data, well, you've got it handy!)
                for (int i = 0; i < manifold.Count; ++i)
                {
                    //This probably looks a bit odd, but it addresses a limitation of returning references to the struct 'this' instance.
                    //(What we really want here is either the lifting of that restriction, or allowing interfaces to require a static member so that we could call the static function and pass the instance, 
                    //instead of invoking the function on the instance AND passing the instance.)
                    if (manifold.GetDepth(ref manifold, i) >= 0)
                    {
                        QueryWasTouched[pairId] = true;
                        break;
                    }
                }
            }
        }

        /// <summary>
        /// Called by the BroadPhase.GetOverlaps to collect all encountered collidables.
        /// </summary>
        struct BroadPhaseOverlapEnumerator : IBreakableForEach<CollidableReference>
        {
            public QuickList<CollidableReference> References;
            //The enumerator never gets stored into unmanaged memory, so it's safe to include a reference type instance.
            public BufferPool Pool;
            public bool LoopBody(CollidableReference reference)
            {
                References.Allocate(Pool) = reference;
                //If you wanted to do any top-level filtering, this would be a good spot for it.
                //The CollidableReference tells you whether it's a body or a static object and the associated handle. You can look up metadata with that.
                return true;
            }
        }


        void GetPoseAndShape(CollidableReference reference, out RigidPose pose, out TypedIndex shapeIndex)
        {
            //Collidables can be associated with either bodies or statics. We have to look in a different place depending on which it is.
            if (reference.Mobility == CollidableMobility.Static)
            {
                var collidable = Simulation.Statics[reference.StaticHandle];
                pose = collidable.Pose;
                shapeIndex = collidable.Shape;
            }
            else
            {
                var bodyReference = Simulation.Bodies[reference.BodyHandle];
                pose = bodyReference.Pose;
                shapeIndex = bodyReference.Collidable.Shape;
            }
        }

        /// <summary>
        /// Adds a shape query to the collision batcher.
        /// </summary>
        /// <param name="queryShapeType">Type of the shape to test.</param>
        /// <param name="queryShapeData">Shape data to test.</param>
        /// <param name="queryShapeSize">Size of the shape data in bytes.</param>
        /// <param name="queryBoundsMin">Minimum of the query shape's bounding box.</param>
        /// <param name="queryBoundsMax">Maximum of the query shape's bounding box.</param>
        /// <param name="queryPose">Pose of the query shape.</param>
        /// <param name="queryId">Id to use to refer to this query when the collision batcher finishes processing it.</param>
        /// <param name="batcher">Batcher to add the query's tests to.</param>
        public unsafe void AddQueryToBatch(int queryShapeType, void* queryShapeData, int queryShapeSize, Vector3 queryBoundsMin, Vector3 queryBoundsMax, in RigidPose queryPose, int queryId, ref CollisionBatcher<BatcherCallbacks> batcher)
        {
            var broadPhaseEnumerator = new BroadPhaseOverlapEnumerator { Pool = BufferPool, References = new QuickList<CollidableReference>(16, BufferPool) };
            Simulation.BroadPhase.GetOverlaps(queryBoundsMin, queryBoundsMax, ref broadPhaseEnumerator);
            for (int overlapIndex = 0; overlapIndex < broadPhaseEnumerator.References.Count; ++overlapIndex)
            {
                GetPoseAndShape(broadPhaseEnumerator.References[overlapIndex], out var pose, out var shapeIndex);
                Simulation.Shapes[shapeIndex.Type].GetShapeData(shapeIndex.Index, out var shapeData, out _);
                //In this path, we assume that the incoming shape data is ephemeral. The collision batcher may last longer than the data pointer.
                //To avoid undefined access, we cache the query data into the collision batcher and use a pointer to the cache instead.
                batcher.CacheShapeB(shapeIndex.Type, queryShapeType, queryShapeData, queryShapeSize, out var cachedQueryShapeData);
                batcher.AddDirectly(
                    shapeIndex.Type, queryShapeType,
                    shapeData, cachedQueryShapeData,
                    //Because we're using this as a boolean query, we use a speculative margin of 0. Don't care about negative depths.
                    queryPose.Position - pose.Position, pose.Orientation, queryPose.Orientation, 0, new PairContinuation(queryId));
            }
            broadPhaseEnumerator.References.Dispose(BufferPool);
        }

        /// <summary>
        /// Adds a shape query to the collision batcher.
        /// </summary>
        /// <typeparam name="TShape">Type of the query shape.</typeparam>
        /// <param name="shape">Shape to use in the query.</param>
        /// <param name="pose">Pose of the query shape.</param>
        /// <param name="queryId">Id to use to refer to this query when the collision batcher finishes processing it.</param>
        /// <param name="batcher">Batcher to add the query's tests to.</param>
        public unsafe void AddQueryToBatch<TShape>(TShape shape, in RigidPose pose, int queryId, ref CollisionBatcher<BatcherCallbacks> batcher) where TShape : IConvexShape
        {
            var queryShapeData = Unsafe.AsPointer(ref shape);
            var queryShapeSize = Unsafe.SizeOf<TShape>();
            shape.ComputeBounds(pose.Orientation, out var boundingBoxMin, out var boundingBoxMax);
            boundingBoxMin += pose.Position;
            boundingBoxMax += pose.Position;
            AddQueryToBatch(TShape.TypeId, queryShapeData, queryShapeSize, boundingBoxMin, boundingBoxMax, pose, queryId, ref batcher);
        }

        //This version of the query isn't used in the demo, but shows how you could use a simulation-cached shape in a query.
        /// <summary>
        /// Adds a shape query to the collision batcher.
        /// </summary>
        /// <typeparam name="TShape">Type of the query shape.</typeparam>
        /// <param name="shape">Shape to use in the query.</param>
        /// <param name="pose">Pose of the query shape.</param>
        /// <param name="queryId">Id to use to refer to this query when the collision batcher finishes processing it.</param>
        /// <param name="batcher">Batcher to add the query's tests to.</param>
        public unsafe void AddQueryToBatch(Shapes shapes, TypedIndex queryShapeIndex, in RigidPose queryPose, int queryId, ref CollisionBatcher<BatcherCallbacks> batcher)
        {
            var shapeBatch = shapes[queryShapeIndex.Type];
            shapeBatch.ComputeBounds(queryShapeIndex.Index, queryPose, out var queryBoundsMin, out var queryBoundsMax);
            Simulation.Shapes[queryShapeIndex.Type].GetShapeData(queryShapeIndex.Index, out var queryShapeData, out _);
            var broadPhaseEnumerator = new BroadPhaseOverlapEnumerator { Pool = BufferPool, References = new QuickList<CollidableReference>(16, BufferPool) };
            Simulation.BroadPhase.GetOverlaps(queryBoundsMin, queryBoundsMax, ref broadPhaseEnumerator);
            for (int overlapIndex = 0; overlapIndex < broadPhaseEnumerator.References.Count; ++overlapIndex)
            {
                GetPoseAndShape(broadPhaseEnumerator.References[overlapIndex], out var pose, out var shapeIndex);
                //Since both involved shapes are from the simulation cache, we don't need to cache them ourselves.
                Simulation.Shapes[shapeIndex.Type].GetShapeData(shapeIndex.Index, out var shapeData, out _);
                batcher.AddDirectly(
                    shapeIndex.Type, queryShapeIndex.Type,
                    shapeData, queryShapeData,
                    //Because we're using this as a boolean query, we use a speculative margin of 0. Don't care about negative depths.
                    queryPose.Position - pose.Position, queryPose.Orientation, pose.Orientation, 0, new PairContinuation(queryId));
            }
            broadPhaseEnumerator.References.Dispose(BufferPool);
        }

        //For the demo, we'll use a bunch of boxes as queries.
        struct Query
        {
            public Box Box;
            public RigidPose Pose;
        }

        public unsafe override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            //The collision batcher vectorizes over multiple tests at once, so for optimal performance, you'll want to feed it a bunch of tests.
            var collisionBatcher = new CollisionBatcher<BatcherCallbacks>(BufferPool, Simulation.Shapes, Simulation.NarrowPhase.CollisionTaskRegistry, 0, new BatcherCallbacks());

            //Create a set of queries up front. We store them so that we can render them after the fact with colors according to their touched states.
            const int widthInQueries = 5;
            var queries = new QuickList<Query>(widthInQueries * widthInQueries, BufferPool);
            var querySpacing = new Vector3(3, 0, 3);
            var basePosition = new Vector3(0, 2, 0) - new Vector3(widthInQueries - 1) * querySpacing * 0.5f;
            for (int i = 0; i < widthInQueries; ++i)
            {
                for (int j = 0; j < widthInQueries; ++j)
                {
                    queries.Allocate(BufferPool) = new Query
                    {
                        Box = new Box(1, 1, 1),
                        Pose = new RigidPose(basePosition + querySpacing * new Vector3(i, 0, j), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(i - 2.5f, 0, j - 2.5f)), i * j + 0.7457f))
                    };
                }
            }
            //Note that the callbacks and set are value types, so you have to be a little careful about copying.
            ref var queryWasTouched = ref collisionBatcher.Callbacks.QueryWasTouched;
            BufferPool.Take(queries.Count, out queryWasTouched);
            queryWasTouched.Clear(0, queryWasTouched.Length);

            for (int queryIndex = 0; queryIndex < queries.Count; ++queryIndex)
            {
                ref var query = ref queries[queryIndex];
                AddQueryToBatch(query.Box, query.Pose, queryIndex, ref collisionBatcher);
            }
            //While the collision batcher may flush batches here and there when a new test is added if it fills a batch, 
            //it's likely that there remain leftover pairs that didn't fill up a batch completely. Force a complete flush.
            //Note that this also returns all resources used by the batcher to the BufferPool.
            collisionBatcher.Flush();

            //Render the query boxes with the proper color.
            for (int i = 0; i < queries.Count; ++i)
            {
                ref var query = ref queries[i];
                renderer.Shapes.AddShape(query.Box, Simulation.Shapes, query.Pose, queryWasTouched[i] ? new Vector3(0, 1, 0) : new Vector3(1, 0, 0));
            }

            BufferPool.Return(ref queryWasTouched);
            queries.Dispose(BufferPool);

            var bottomY = renderer.Surface.Resolution.Y;
            renderer.TextBatcher.Write(text.Clear().Append("The broad phase exposes queries to collect bodies within bounding volumes, and the CollisionBatcher can be used to perform contact generation."), new Vector2(16, bottomY - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("The boxes in the floating grid represent shape queries against the simulation. Broad phase collected candidates are handed to a CollisionBatcher for testing."), new Vector2(16, bottomY - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("If positive depth contacts are detected, the box turns green."), new Vector2(16, bottomY - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("(Note that triangle backfaces do not generate contacts, so queries intersecting just the mesh from below do not turn green.)"), new Vector2(16, bottomY - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}

