using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using Demos.Demos;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    static class PairDeterminismTests
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

            public unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ConvexContactManifold* manifold)
            {

            }

            public unsafe void OnPairCompleted(int pairId, ConvexContactManifold* manifold)
            {
                Manifolds.ConvexManifolds[pairId] = *manifold;
                Manifolds.ManifoldIsConvex[pairId] = true;
            }

            public unsafe void OnPairCompleted(int pairId, NonconvexContactManifold* manifold)
            {
                Manifolds.NonconvexManifolds[pairId] = *manifold;
                Manifolds.ManifoldIsConvex[pairId] = false;
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
        public static RigidPose CreateRandomPose(Random random, BoundingBox positionBounds)
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
            Console.WriteLine($"Starting {a.Shape.Type} versus {b.Shape.Type}");
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
                    posesA[i] = CreateRandomPose(random, positionBounds);
                    posesB[i] = CreateRandomPose(random, positionBounds);
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
                            DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref manifold.Normal, 5573) +
                            DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref manifold.OffsetB, 6829);
                        for (int j = 0; j < manifold.Count; ++j)
                        {
                            ref var contact = ref Unsafe.Add(ref manifold.Contact0, j);
                            manifoldContribution += 
                                DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref contact.Offset, 4783) +
                                DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref contact.Depth, 5647) + contact.FeatureId * 3697;
                        }
                        hash += manifoldContribution * i;
                    }
                    else
                    {
                        ref var manifold = ref originalManifolds.NonconvexManifolds[i];
                        long manifoldContribution =
                            manifold.Count * 4993 +
                            DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref manifold.OffsetB, 6829);
                        for (int j = 0; j < manifold.Count; ++j)
                        {
                            ref var contact = ref Unsafe.Add(ref manifold.Contact0, j);
                            manifoldContribution +=
                                DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref contact.Offset, 4783) +
                                DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref contact.Normal, 5821) +
                                DeterminismHashTest<FountainStressTestDemo>.ComputeHash(ref contact.Depth, 5647) + contact.FeatureId * 3697;
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
                        if (originalManifolds.ManifoldIsConvex[i] != comparisonManifolds.ManifoldIsConvex[i])
                        {
                            Console.WriteLine($"Manifolds don't even have the same convexity state! {originalManifolds.ManifoldIsConvex[i]} versus {comparisonManifolds.ManifoldIsConvex[i]}");
                        }
                        else
                        {
                            if (originalManifolds.ManifoldIsConvex[i])
                            {
                                ref var original = ref originalManifolds.ConvexManifolds[i];
                                ref var comparison = ref comparisonManifolds.ConvexManifolds[i];
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
                            else
                            {
                                ref var original = ref originalManifolds.NonconvexManifolds[i];
                                ref var comparison = ref comparisonManifolds.NonconvexManifolds[i];
                                if (original.Count != comparison.Count || (original.Count > 0 && original.OffsetB != comparison.OffsetB))
                                {
                                    Console.WriteLine($"Nonconvex manifold determinism failed!");
                                    Console.WriteLine($"counts: {original.Count} vs {comparison.Count}");
                                    Console.WriteLine($"offsetBs: {original.OffsetB} vs {comparison.OffsetB}");
                                }
                                for (int j = 0; j < original.Count; ++j)
                                {
                                    ref var originalContact = ref Unsafe.Add(ref original.Contact0, j);
                                    ref var comparisonContact = ref Unsafe.Add(ref comparison.Contact0, j);
                                    if (originalContact.Depth != comparisonContact.Depth ||
                                        originalContact.Normal != comparisonContact.Normal ||
                                        originalContact.FeatureId != comparisonContact.FeatureId ||
                                        originalContact.Offset != comparisonContact.Offset)
                                    {
                                        Console.WriteLine($"Nonconvex contact determinism failed!");
                                        Console.WriteLine($"depths: {originalContact.Depth} vs {comparisonContact.Depth}");
                                        Console.WriteLine($"normals: {originalContact.Normal} vs {comparisonContact.Normal}");
                                        Console.WriteLine($"featureids: {originalContact.FeatureId} vs {comparisonContact.FeatureId}");
                                        Console.WriteLine($"offsets: {originalContact.Offset} vs {comparisonContact.Offset}");
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
            const int poseIterations = 2048;
            const int remapIterations = 64;
            var bounds = new BoundingBox(new Vector3(-6), new Vector3(6));
            const int randomSeed = 5;

            Test(sphereCollidable, sphereCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, capsuleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(sphereCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(capsuleCollidable, capsuleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(capsuleCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(boxCollidable, boxCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(boxCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(triangleCollidable, triangleCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(triangleCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(cylinderCollidable, cylinderCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(cylinderCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(cylinderCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(cylinderCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(cylinderCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(hullCollidable, hullCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(hullCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(hullCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(hullCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(compoundCollidable, compoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(compoundCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(compoundCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(bigCompoundCollidable, bigCompoundCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);
            Test(bigCompoundCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

            Test(meshCollidable, meshCollidable, bounds, pairCount, poseIterations, remapIterations, registry, pool, shapes, randomSeed);

        }
    }
}
