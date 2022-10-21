using BenchmarkDotNet.Running;
using DemoBenchmarks;

var summaryOneBody = BenchmarkRunner.Run(typeof(OneBodyConstraintBenchmarks));
var summaryTwoBody = BenchmarkRunner.Run(typeof(TwoBodyConstraintBenchmarks));
var summaryThreeBody = BenchmarkRunner.Run(typeof(ThreeBodyConstraintBenchmarks));
var summaryFourBody = BenchmarkRunner.Run(typeof(FourBodyConstraintBenchmarks));