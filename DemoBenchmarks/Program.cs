using BenchmarkDotNet.Running;
using DemoBenchmarks;

BenchmarkRunner.Run(typeof(OneBodyConstraintBenchmarks));
BenchmarkRunner.Run(typeof(TwoBodyConstraintBenchmarks));
BenchmarkRunner.Run(typeof(ThreeBodyConstraintBenchmarks));
BenchmarkRunner.Run(typeof(FourBodyConstraintBenchmarks));
BenchmarkRunner.Run(typeof(ConvexCollisionTesters));
BenchmarkRunner.Run(typeof(CollisionBatcherTasks));
BenchmarkRunner.Run(typeof(Sweeps));
BenchmarkRunner.Run(typeof(ShapeRayTests));
BenchmarkRunner.Run(typeof(GatherScatter));
BenchmarkRunner.Run(typeof(RagdollTubeBenchmark));

