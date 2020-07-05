using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using Demos;
using Demos.Demos;
using Demos.SpecializedTests;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Xunit;

namespace DemoTests
{
    public static class PairDeterminismTests
    {
        struct Manifolds
        {
            public Buffer<ConvexContactManifold> ConvexManifolds;
            public Buffer<NonconvexContactManifold> NonconvexManifolds;
            public Buffer<bool> ManifoldIsConvex;

            public Manifolds(int count, BufferPool pool)
            {
                pool.Take(count, out ConvexManifolds);
                pool.Take(count, out NonconvexManifolds);
                pool.Take(count, out ManifoldIsConvex);
            }

            public void Dispose(BufferPool pool)
            {
                pool.Return(ref ConvexManifolds);
                pool.Return(ref NonconvexManifolds);
                pool.Return(ref ManifoldIsConvex);
            }
        }
        struct BatcherCallbacks : ICollisionCallbacks
        {
            public BufferPool Pool;
            public Manifolds Manifolds;
            public bool AllowCollisionTesting(int pairId, int childA, int childB)
            {
                return true;
            }

            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold)
            {

            }

            public unsafe void OnPairCompleted<TManifold>(int pairId, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                if (manifold.Convex)
                {
                    Manifolds.ConvexManifolds[pairId] = Unsafe.As<TManifold, ConvexContactManifold>(ref manifold);
                }
                else
                {
                    Manifolds.NonconvexManifolds[pairId] = Unsafe.As<TManifold, NonconvexContactManifold>(ref manifold);
                }
                Manifolds.ManifoldIsConvex[pairId] = manifold.Convex;
            }
        }

        static void ComputeCollisions(CollisionTaskRegistry registry, Shapes shapes, BufferPool pool,
            ref Manifolds manifolds, CollidableDescription a, CollidableDescription b, ref Buffer<RigidPose> posesA, ref Buffer<RigidPose> posesB, Buffer<int> remapIndices, int pairCount)
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

        private static void TestPair(CollidableDescription a, CollidableDescription b, BoundingBox positionBounds, int pairCount, int poseIterations, int remapIterations, CollisionTaskRegistry registry, BufferPool pool, Shapes shapes, int randomSeed)
        {
            string pairName = $"{a.Shape.Type} vs {b.Shape.Type}";
            var random = new Random(randomSeed);
            var originalManifolds = new Manifolds(pairCount, pool);
            var comparisonManifolds = new Manifolds(pairCount, pool);
            pool.Take<RigidPose>(pairCount, out var posesA);
            pool.Take<RigidPose>(pairCount, out var posesB);
            pool.Take<int>(pairCount, out var remapIndices);
            QuickList<int> remainingIndices = new QuickList<int>(pairCount, pool);
            long hash = 0;
            for (int poseIteration = 0; poseIteration < poseIterations; ++poseIteration)
            {
                for (int i = 0; i < pairCount; ++i)
                {
                    posesA[i] = TestHelpers.CreateRandomPose(random, positionBounds);
                    posesB[i] = TestHelpers.CreateRandomPose(random, positionBounds);
                    remapIndices[i] = i;
                };
                ComputeCollisions(registry, shapes, pool, ref originalManifolds, a, b, ref posesA, ref posesB, remapIndices, pairCount);

                for (int i = 0; i < pairCount; ++i)
                {
                    if (originalManifolds.ManifoldIsConvex[i])
                    {
                        ref var manifold = ref originalManifolds.ConvexManifolds[i];
                        long manifoldContribution =
                            manifold.Count * 4993 +
                            TestUtilities.ComputeHash(ref manifold.Normal, 5573) +
                            TestUtilities.ComputeHash(ref manifold.OffsetB, 6829);
                        for (int j = 0; j < manifold.Count; ++j)
                        {
                            ref var contact = ref Unsafe.Add(ref manifold.Contact0, j);
                            manifoldContribution +=
                                TestUtilities.ComputeHash(ref contact.Offset, 4783) +
                                TestUtilities.ComputeHash(ref contact.Depth, 5647) + contact.FeatureId * 3697;
                        }
                        hash += manifoldContribution * i;
                    }
                    else
                    {
                        ref var manifold = ref originalManifolds.NonconvexManifolds[i];
                        long manifoldContribution =
                            manifold.Count * 4993 +
                            TestUtilities.ComputeHash(ref manifold.OffsetB, 6829);
                        for (int j = 0; j < manifold.Count; ++j)
                        {
                            ref var contact = ref Unsafe.Add(ref manifold.Contact0, j);
                            manifoldContribution +=
                                TestUtilities.ComputeHash(ref contact.Offset, 4783) +
                                TestUtilities.ComputeHash(ref contact.Normal, 5821) +
                                TestUtilities.ComputeHash(ref contact.Depth, 5647) + contact.FeatureId * 3697;
                        }
                        hash += manifoldContribution * i;
                    }
                }

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
                        Assert.True(originalManifolds.ManifoldIsConvex[i] == comparisonManifolds.ManifoldIsConvex[i], $"{pairName} manifolds don't even have the same convexity state! {originalManifolds.ManifoldIsConvex[i]} versus {comparisonManifolds.ManifoldIsConvex[i]}");
                        if (originalManifolds.ManifoldIsConvex[i])
                        {
                            ref var original = ref originalManifolds.ConvexManifolds[i];
                            ref var comparison = ref comparisonManifolds.ConvexManifolds[i];
                            Assert.True(original.Count == comparison.Count, $"{pairName} convex manifold properties differ: count {original.Count} vs {comparison.Count}");
                            if (original.Count > 0)
                            {
                                Assert.True(original.Normal == comparison.Normal, $"{pairName} convex manifold properties differ: normals {original.Normal} vs {comparison.Normal}");
                                Assert.True(original.OffsetB == comparison.OffsetB, $"{pairName} convex manifold properties differ: offsetBs {original.OffsetB} vs {comparison.OffsetB}");
                                for (int j = 0; j < original.Count; ++j)
                                {
                                    ref var originalContact = ref Unsafe.Add(ref original.Contact0, j);
                                    ref var comparisonContact = ref Unsafe.Add(ref comparison.Contact0, j);
                                    Assert.True(originalContact.Depth == comparisonContact.Depth, $"{pairName} contact properties differ: depth {originalContact.Depth} vs {comparisonContact.Depth}");
                                    Assert.True(originalContact.FeatureId == comparisonContact.FeatureId, $"{pairName} contact properties differ: featureId {originalContact.FeatureId} vs {comparisonContact.FeatureId}");
                                    Assert.True(originalContact.Offset == comparisonContact.Offset, $"{pairName} contact properties differ: offset {originalContact.Offset} vs {comparisonContact.Offset}");
                                }
                            }
                        }
                        else
                        {
                            ref var original = ref originalManifolds.NonconvexManifolds[i];
                            ref var comparison = ref comparisonManifolds.NonconvexManifolds[i];
                            Assert.True(original.Count == comparison.Count, $"{pairName} nonconvex manifold properties differ: count {original.Count} vs {comparison.Count}");
                            if (original.Count > 0)
                            {
                                Assert.True(original.OffsetB == comparison.OffsetB, $"{pairName} nonconvex manifold properties differ: offsetBs {original.OffsetB} vs {comparison.OffsetB}");
                                for (int j = 0; j < original.Count; ++j)
                                {
                                    ref var originalContact = ref Unsafe.Add(ref original.Contact0, j);
                                    ref var comparisonContact = ref Unsafe.Add(ref comparison.Contact0, j);
                                    if (originalContact.Depth != comparisonContact.Depth ||
                                        originalContact.Normal != comparisonContact.Normal ||
                                        originalContact.FeatureId != comparisonContact.FeatureId ||
                                        originalContact.Offset != comparisonContact.Offset)
                                    {
                                        Assert.True(originalContact.Depth == comparisonContact.Depth, $"{pairName} contact properties differ: depth {originalContact.Depth} vs {comparisonContact.Depth}");
                                        Assert.True(originalContact.Normal == comparisonContact.Normal, $"{pairName} contact properties differ: featureId {originalContact.Normal} vs {comparisonContact.Normal}");
                                        Assert.True(originalContact.FeatureId == comparisonContact.FeatureId, $"{pairName} contact properties differ: featureId {originalContact.FeatureId} vs {comparisonContact.FeatureId}");
                                        Assert.True(originalContact.Offset == comparisonContact.Offset, $"{pairName} contact properties differ: offset {originalContact.Offset} vs {comparisonContact.Offset}");
                                    }
                                }
                            }
                        }
                    }
                }
            }
            Console.WriteLine($"Results hash: {hash}");
            originalManifolds.Dispose(pool);
            comparisonManifolds.Dispose(pool);
            pool.Return(ref posesA);
            pool.Return(ref posesB);
            pool.Return(ref remapIndices);
        }

        [Fact]
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
            var pointsBuffer = points.Span.Slice(points.Count);
            ConvexHullHelper.CreateShape(pointsBuffer, pool, out _, out var convexHull);

            var shapes = new Shapes(pool, 8);
            const float speculativeMargin = 0.1f;
            var sphereCollidable = new CollidableDescription(shapes.Add(sphere), speculativeMargin);
            var capsuleCollidable = new CollidableDescription(shapes.Add(capsule), speculativeMargin);
            var boxCollidable = new CollidableDescription(shapes.Add(box), speculativeMargin);
            var triangleCollidable = new CollidableDescription(shapes.Add(triangle), speculativeMargin);
            var cylinderCollidable = new CollidableDescription(shapes.Add(cylinder), speculativeMargin);
            var hullCollidable = new CollidableDescription(shapes.Add(convexHull), speculativeMargin);


            var compoundBuilder = new CompoundBuilder(pool, shapes, 6);
            compoundBuilder.AddForKinematic(sphereCollidable.Shape, new RigidPose(new Vector3(2, 0, 0)), 1);
            compoundBuilder.AddForKinematic(capsuleCollidable.Shape, new RigidPose(new Vector3(0, 2, 0)), 1);
            compoundBuilder.AddForKinematic(boxCollidable.Shape, new RigidPose(new Vector3(0, 0, 2)), 1);
            compoundBuilder.AddForKinematic(triangleCollidable.Shape, new RigidPose(new Vector3(-2, 0, 1)), 1);
            compoundBuilder.AddForKinematic(cylinderCollidable.Shape, new RigidPose(new Vector3(0, -2, 1)), 1);
            compoundBuilder.AddForKinematic(hullCollidable.Shape, new RigidPose(new Vector3(0, 0, -2)), 1);
            compoundBuilder.BuildKinematicCompound(out var children, out _);
            var compound = new Compound(children);
            var bigCompound = new BigCompound(children, shapes, pool);
            var compoundCollidable = new CollidableDescription(shapes.Add(compound), 0.1f);
            var bigCompoundCollidable = new CollidableDescription(shapes.Add(bigCompound), 0.1f);

            DemoMeshHelper.CreateDeformedPlane(2, 2, (x, y) => new Vector3(x, 0, y), Vector3.One, pool, out var mesh);
            var meshCollidable = new CollidableDescription(shapes.Add(mesh), 0.1f);

            const int pairCount = 31;
            const int poseIterations = 256;
            const int remapIterations = 64;
            var bounds = new BoundingBox(new Vector3(-6), new Vector3(6));
            const int randomSeed = 5;

            TestPair(sphereCollidable, sphereCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, capsuleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(sphereCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(capsuleCollidable, capsuleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(capsuleCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(capsuleCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(capsuleCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(capsuleCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(capsuleCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(capsuleCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(capsuleCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(boxCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(boxCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(boxCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(boxCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(boxCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(boxCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(boxCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(triangleCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(triangleCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(triangleCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(triangleCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(triangleCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(triangleCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(cylinderCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(cylinderCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(cylinderCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(cylinderCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(cylinderCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(hullCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(hullCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(hullCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(hullCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(compoundCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(compoundCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(compoundCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(bigCompoundCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            TestPair(bigCompoundCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            TestPair(meshCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

        }
    }
}
