# Benchmarks

This project contains a variety of performance tests for different pieces of the library.

For a partial but pretty high coverage set of benchmarks, consider running with a filter of:
```
-f *CollisionBatcherTaskBenchmarks.* *GroupedCollisionTesterBenchmarks.* *GatherScatterBenchmarks.* *OneBodyConstraintBenchmarks.* *TwoBodyConstraintBenchmarks.* *ThreeBodyConstraintBenchmarks.* *FourBodyConstraintBenchmarks.* *SweepBenchmarks.* *ShapeRayBenchmarks.* *ShapePileBenchmark.* *RagdollTubeBenchmark.*
```

For a longer complete set of benchmarks, consider running with a filter of:
```
-f *CollisionBatcherTaskBenchmarks.* *ConvexCollisionTesterBenchmarks.* *GatherScatterBenchmarks.* *OneBodyConstraintBenchmarks.* *OneBodyConstraintBenchmarksDeep.* *TwoBodyConstraintBenchmarks.* *TwoBodyConstraintBenchmarksDeep.* *ThreeBodyConstraintBenchmarks.* *FourBodyConstraintBenchmarks.* *SweepBenchmarks.* *SweepBenchmarksDeep.* *ShapeRayBenchmarksDeep.* *ShapePileBenchmark.* *RagdollTubeBenchmark.*
```

The tests sometimes cover different scopes. Most are microbenchmarks. Benchmarks in the following classes cover specific codepaths, like a single collision pair tester:

[ConvexCollisionTesterBenchmarks](ConvexCollisionTesterBenchmarks.cs): Contact manifold generating convex collision tests, one benchmark per type pair.

[OneBodyConstraintBenchmarks](OneBodyConstraintBenchmarks.cs): One body constraints, one benchmark per type.

[TwoBodyConstraintBenchmarks](TwoBodyConstraintBenchmarks.cs): Two body constraints, one benchmark per type.

[ThreeBodyConstraintBenchmarks](ThreeBodyConstraintBenchmarks.cs): Three body constraints, one benchmark per type.

[FourBodyConstraintBenchmarks](FourBodyConstraintBenchmarks.cs): Four body constraints, one benchmark per type.

[ShapeRayBenchmarksDeep](ShapeRayBenchmarksDeep.cs): Shape-ray tests, one benchmark per type.

[SweepBenchmarks](SweepBenchmarks.cs): Linear + angular sweep tests for shape pairs, one benchmark per type pair.

[GatherScatterBenchmarks](GatherScatterBenchmarks.cs): Tests SIMD body data gathering/scattering for constraint solving. One benchmark for gather and one for scatter.

Some microbenchmarks are excluded from the above because they're reasonably well covered by other benchmarks. The excluded codepaths are found in the classes suffixed with `Deep`:

[OneBodyConstraintBenchmarksDeep](OneBodyConstraintBenchmarksDeep.cs): One body constraints excluded from `OneBodyConstraintBenchmarks`.

[TwoBodyConstraintBenchmarksDeep](TwoBodyConstraintBenchmarksDeep.cs): Two body constraints excluded from `TwoBodyConstraintBenchmarks`.

[SweepBenchmarksDeep](SweepBenchmarksDeep.cs): Sweep tests excluded from `SweepBenchmarks`.

[ShapeRayBenchmarksDeep](ShapeRayBenchmarksDeep.cs): Individual shape-ray tests that make up the `ShapeRayBenchmarks`.

Other tests cover more codepaths at once:

[ShapeRayBenchmarks](ShapeRayBenchmarks.cs): One benchmark for all convex ray tests, another for all compound ray tests.

[GroupedCollisionTesterBenchmarks](GroupedCollisionTesterBenchmarks.cs): A few benchmarks testing all convex shape pair types directly (with no `CollisionBatcher` involvement), but with type pairs grouped together by approximate cost.

[CollisionBatcherTaskBenchmarks](CollisionBatcherTaskBenchmarks.cs): A single benchmark testing all shape pair types (including nonconvex pairs) in the `CollisionBatcher`. Pairs are dynamically batched and dispatched across a loop.

[RagdollTubeBenchmark](RagdollTubeBenchmark.cs): Simpler version of the `RagdollTubeDemo`, testing a bunch of ragdolls in a tumblertube. One execution of the benchmark covers many timesteps of simulation. This covers several collision pairs, many constraint types, and the rest of the simulation code gluing things together.

[ShapePileBenchmark](ShapePileBenchmark.cs): Another full simulation benchmark, this time a simpler version of the `ShapePileTestDemo`. Compared to the RagdollTubeDemo, this focuses less on constraints, but includes more collision types.
