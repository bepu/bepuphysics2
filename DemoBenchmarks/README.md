# Benchmarks

This project contains a variety of performance tests for different pieces of the library.

The tests sometimes cover different scopes. Most are microbenchmarks. Benchmarks in the following classes cover specific codepaths, like a single collision pair tester:

[ConvexCollisionTesters](ConvexCollisionTesters.cs): Contact manifold generating convex collision tests, one benchmark per type pair.

[OneBodyConstraints](OneBodyConstraints.cs): One body constraints, one benchmark per type.

[TwoBodyConstraints](TwoBodyConstraints.cs): Two body constraints, one benchmark per type.

[ThreeBodyConstraints](ThreeBodyConstraints.cs): Three body constraints, one benchmark per type.

[FourBodyConstraints](FourBodyConstraints.cs): Four body constraints, one benchmark per type.

[ShapeRayTests](ShapeRayTests.cs): Shape-ray tests, one benchmark per type.

[Sweeps](Sweeps.cs): Linear + angular sweep tests for shape pairs, one benchmark per type pair.

[GatherScatter](GatherScatter.cs): Tests SIMD body data gathering/scattering for constraint solving. One benchmark for each gather/scatter operation.

Other tests cover more codepaths at once:

[CollisionBatcherTasks](CollisionBatcherTasks.cs): A single benchmark testing all shape pair types (including nonconvex pairs) in the `CollisionBatcher`. Pairs are dynamically batched and dispatched across a loop.

[RagdollTubeBenchmark](RagdollTubeBenchmark.cs): Simpler version of the `RagdollTubeDemo`, testing a bunch of ragdolls in a tumblertube. One execution of the benchmark covers many timesteps of simulation. This covers several collision pairs, many constraint types, and the rest of the simulation code gluing things together.

[ShapePileBenchmark](ShapePileBenchmark.cs): Another full simulation benchmark, this time a simpler version of the `ShapePileTestDemo`. Compared to the RagdollTubeDemo, this focuses less on constraints, but includes more collision types.
