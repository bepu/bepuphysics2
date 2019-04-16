using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    static class ConvexPairDeterminismTests
    {
        struct BatcherCallbacks : ICollisionCallbacks
        {
            public Buffer<ConvexContactManifold> Manifolds;
            public BufferPool Pool;
            public bool AllowCollisionTesting(int pairId, int childA, int childB)
            {
                return true;
            }

            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ConvexContactManifold* manifold)
            {

            }

            public unsafe void OnPairCompleted(int pairId, ConvexContactManifold* manifold)
            {
                Manifolds[pairId] = *manifold;
            }

            public unsafe void OnPairCompleted(int pairId, NonconvexContactManifold* manifold)
            {
            }
        }

        static void ComputeCollisions(CollisionTaskRegistry registry, Shapes shapes, BufferPool pool,
            ref Buffer<ConvexContactManifold> manifolds, CollidableDescription a, CollidableDescription b, ref Buffer<RigidPose> posesA, ref Buffer<RigidPose> posesB, Buffer<int> remapIndices, int pairCount)
        {
            var batcher = new CollisionBatcher<BatcherCallbacks>(pool, shapes, registry, 1 / 60f, new BatcherCallbacks { Pool = pool, Manifolds = manifolds });
            for (int i = 0; i < pairCount; ++i)
            {
                var index = remapIndices[i];
                ref var poseA = ref posesA[index];
                ref var poseB = ref posesB[index];
                batcher.Add(a.Shape, b.Shape, poseB.Position - poseA.Position, poseA.Orientation, poseB.Orientation, Math.Max(a.SpeculativeMargin, b.SpeculativeMargin), new PairContinuation(index));
            }
            batcher.Flush();
        }
        static RigidPose CreateRandomPose(Random random, BoundingBox positionBounds)
        {
            RigidPose pose;
            var span = positionBounds.Max - positionBounds.Min;

            pose.Position = positionBounds.Min + span * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
            var axis = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
            var length = axis.Length();
            if (length > 0)
                axis /= length;
            else
                axis = new Vector3(0, 1, 0);
            pose.Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(axis, 1203f * (float)random.NextDouble());
            return pose;
        }

        private static void Test(CollidableDescription a, CollidableDescription b, BoundingBox positionBounds, int pairCount, int poseIterations, int remapIterations, CollisionTaskRegistry registry, BufferPool pool, Shapes shapes, int randomSeed)
        {
            var random = new Random(randomSeed);
            pool.Take<ConvexContactManifold>(pairCount, out var originalManifolds);
            pool.Take<ConvexContactManifold>(pairCount, out var comparisonManifolds);
            pool.Take<RigidPose>(pairCount, out var posesA);
            pool.Take<RigidPose>(pairCount, out var posesB);
            pool.Take<int>(pairCount, out var remapIndices);
            QuickList<int> remainingIndices = new QuickList<int>(pairCount, pool);
            for (int poseIteration = 0; poseIteration < poseIterations; ++poseIteration)
            {
                for (int i = 0; i < pairCount; ++i)
                {
                    posesA[i] = CreateRandomPose(random, positionBounds);
                    posesB[i] = CreateRandomPose(random, positionBounds);
                    remapIndices[i] = i;
                };
                ComputeCollisions(registry, shapes, pool, ref originalManifolds, a, b, ref posesA, ref posesB, remapIndices, pairCount);

                for (int remapIteration = 0; remapIteration < remapIterations; ++remapIteration)
                {
                    for (int i = 0; i < pairCount; ++i)
                    {
                        remainingIndices.AllocateUnsafely() = i;
                    }
                    for (int i = 0; i < pairCount; ++i)
                    {
                        var toTake = random.Next(0, remainingIndices.Count);
                        remapIndices[i] = remainingIndices[toTake];
                        remainingIndices.FastRemoveAt(toTake);
                    }
                    ComputeCollisions(registry, shapes, pool, ref comparisonManifolds, a, b, ref posesA, ref posesB, remapIndices, pairCount);
                    for (int i = 0; i < pairCount; ++i)
                    {
                        ref var original = ref originalManifolds[i];
                        ref var comparison = ref comparisonManifolds[i];
                        if (original.Count != comparison.Count || (original.Count > 0 && (original.Normal != comparison.Normal || original.OffsetB != comparison.OffsetB)))
                        {
                            Console.WriteLine($"Manifold determinism failed!");
                            Console.WriteLine($"counts: {original.Count} vs {comparison.Count}");
                            Console.WriteLine($"normals: {original.Normal} vs {comparison.Normal}");
                            Console.WriteLine($"offsetBs: {original.OffsetB} vs {comparison.OffsetB}");
                        }
                        for (int j = 0; j < original.Count; ++j)
                        {
                            ref var originalContact = ref Unsafe.Add(ref original.Contact0, j);
                            ref var comparisonContact = ref Unsafe.Add(ref comparison.Contact0, j);
                            if (originalContact.Depth != comparisonContact.Depth ||
                                originalContact.FeatureId != comparisonContact.FeatureId ||
                                originalContact.Offset != comparisonContact.Offset)
                            {
                                Console.WriteLine($"Contact determinism failed!");
                                Console.WriteLine($"depths: {originalContact.Depth} vs {comparisonContact.Depth}");
                                Console.WriteLine($"featureids: {originalContact.FeatureId} vs {comparisonContact.FeatureId}");
                                Console.WriteLine($"offsets: {originalContact.Offset} vs {comparisonContact.Offset}");
                            }
                        }
                    }
                }
            }
            pool.Return(ref originalManifolds);
            pool.Return(ref comparisonManifolds);
            pool.Return(ref posesA);
            pool.Return(ref posesB);
            pool.Return(ref remapIndices);
        }

        public static void Test()
        {
            var pool = new BufferPool();
            var random = new Random(5);
            var registry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
            var sphere = new Sphere(1);
            var capsule = new Capsule(0.5f, 1f);
            var box = new Box(1f, 1f, 1f);
            var triangle = new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1));
            var cylinder = new Cylinder(0.5f, 1f);

            const int pointCount = 32;
            var points = new QuickList<Vector3>(pointCount, pool);
            for (int i = 0; i < pointCount; ++i)
            {
                points.AllocateUnsafely() = new Vector3(3 * (float)random.NextDouble(), 2 * (float)random.NextDouble(), (float)random.NextDouble());
            }
            var pointsBuffer = points.Span.Slice(0, points.Count);
            ConvexHullHelper.CreateShape(pointsBuffer, pool, out _, out var convexHull);

            var shapes = new Shapes(pool, 2);
            const float speculativeMargin = float.MaxValue;
            var sphereCollidable = new CollidableDescription(shapes.Add(sphere), speculativeMargin);
            var capsuleCollidable = new CollidableDescription(shapes.Add(capsule), speculativeMargin);
            var boxCollidable = new CollidableDescription(shapes.Add(box), speculativeMargin);
            var triangleCollidable = new CollidableDescription(shapes.Add(triangle), speculativeMargin);
            var cylinderCollidable = new CollidableDescription(shapes.Add(cylinder), speculativeMargin);
            var hullCollidable = new CollidableDescription(shapes.Add(convexHull), speculativeMargin);
            const int pairCount = 31;
            const int poseIterations = 65536;
            const int remapIterations = 32;
            var bounds = new BoundingBox(new Vector3(-6), new Vector3(6));
            const int randomSeed = 5;

            Test(sphereCollidable, sphereCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, capsuleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, capsuleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(cylinderCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(cylinderCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(hullCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
        }
    }
}
