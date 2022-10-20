using BenchmarkDotNet.Running;
using DemoBenchmarks;

var summary = BenchmarkRunner.Run(typeof(OneBodyConstraintBenchmarks));