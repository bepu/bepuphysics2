using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities;

namespace SimdTests
{
    /// <summary>
    /// Performance testing harness for SIMD operations.
    /// Measures CPU time spent executing functions many times.
    /// </summary>
    public class PerformanceHarness
    {
        private const int WarmupIterations = 1000;
        private const int BenchmarkIterations = 10_000_000;

        /// <summary>
        /// Runs a benchmark on a function, returning nanoseconds per iteration.
        /// </summary>
        private static double BenchmarkFunction<T>(Func<T> function, string name, int iterations = BenchmarkIterations)
        {
            // Warmup
            for (int i = 0; i < WarmupIterations; i++)
            {
                var _ = function();
            }

            // Force garbage collection before benchmark
            GC.Collect();
            GC.WaitForPendingFinalizers();
            GC.Collect();

            var stopwatch = Stopwatch.StartNew();
            for (int i = 0; i < iterations; i++)
            {
                var _ = function();
            }
            stopwatch.Stop();

            var nsPerIteration = (stopwatch.Elapsed.TotalNanoseconds) / iterations;
            return nsPerIteration;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<float> TestFastReciprocal(Vector<float> input)
        {
            return MathHelper.FastReciprocal(input);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<float> TestFastReciprocal_Reference(Vector<float> input)
        {
            return ReferenceImplementations.FastReciprocal_Reference(input);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<float> TestFastReciprocalSquareRoot(Vector<float> input)
        {
            return MathHelper.FastReciprocalSquareRoot(input);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<float> TestFastReciprocalSquareRoot_Reference(Vector<float> input)
        {
            return ReferenceImplementations.FastReciprocalSquareRoot_Reference(input);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<int> TestCreateTrailingMaskForCountInBundle(int count)
        {
            return BundleIndexing.CreateTrailingMaskForCountInBundle(count);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<int> TestCreateTrailingMaskForCountInBundle_Reference(int count)
        {
            return ReferenceImplementations.CreateTrailingMaskForCountInBundle_Reference(count);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<int> TestCreateMaskForCountInBundle(int count)
        {
            return BundleIndexing.CreateMaskForCountInBundle(count);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static Vector<int> TestCreateMaskForCountInBundle_Reference(int count)
        {
            return ReferenceImplementations.CreateMaskForCountInBundle_Reference(count);
        }

        public static void RunAllBenchmarks()
        {
            Console.WriteLine("==============================================");
            Console.WriteLine("SIMD Performance Benchmark Suite");
            Console.WriteLine("==============================================");
            Console.WriteLine($"Vector<float>.Count: {Vector<float>.Count}");
            Console.WriteLine($"Vector<int>.Count: {Vector<int>.Count}");
            Console.WriteLine($"System.Runtime.Intrinsics.X86.Avx.IsSupported: {System.Runtime.Intrinsics.X86.Avx.IsSupported}");
            Console.WriteLine($"System.Runtime.Intrinsics.X86.Avx2.IsSupported: {System.Runtime.Intrinsics.X86.Avx2.IsSupported}");
            Console.WriteLine($"System.Runtime.Intrinsics.X86.Sse.IsSupported: {System.Runtime.Intrinsics.X86.Sse.IsSupported}");
            Console.WriteLine($"System.Runtime.Intrinsics.Arm.AdvSimd.IsSupported: {System.Runtime.Intrinsics.Arm.AdvSimd.IsSupported}");
            Console.WriteLine($"Iterations per benchmark: {BenchmarkIterations:N0}");
            Console.WriteLine("==============================================\n");

            BenchmarkFastReciprocal();
            BenchmarkFastReciprocalSquareRoot();
            BenchmarkCreateTrailingMaskForCountInBundle();
            BenchmarkCreateMaskForCountInBundle();

            Console.WriteLine("\n==============================================");
            Console.WriteLine("Benchmark Complete");
            Console.WriteLine("==============================================");
        }

        private static void BenchmarkFastReciprocal()
        {
            Console.WriteLine("Benchmarking: FastReciprocal");
            var testInput = new Vector<float>(2.5f);

            var simdTime = BenchmarkFunction(() => TestFastReciprocal(testInput), "FastReciprocal (SIMD)");
            var refTime = BenchmarkFunction(() => TestFastReciprocal_Reference(testInput), "FastReciprocal (Reference)");

            Console.WriteLine($"  SIMD:      {simdTime:F2} ns/iteration");
            Console.WriteLine($"  Reference: {refTime:F2} ns/iteration");
            Console.WriteLine($"  Speedup:   {refTime / simdTime:F2}x");
            Console.WriteLine();
        }

        private static void BenchmarkFastReciprocalSquareRoot()
        {
            Console.WriteLine("Benchmarking: FastReciprocalSquareRoot");
            var testInput = new Vector<float>(4.0f);

            var simdTime = BenchmarkFunction(() => TestFastReciprocalSquareRoot(testInput), "FastReciprocalSquareRoot (SIMD)");
            var refTime = BenchmarkFunction(() => TestFastReciprocalSquareRoot_Reference(testInput), "FastReciprocalSquareRoot (Reference)");

            Console.WriteLine($"  SIMD:      {simdTime:F2} ns/iteration");
            Console.WriteLine($"  Reference: {refTime:F2} ns/iteration");
            Console.WriteLine($"  Speedup:   {refTime / simdTime:F2}x");
            Console.WriteLine();
        }

        private static void BenchmarkCreateTrailingMaskForCountInBundle()
        {
            Console.WriteLine("Benchmarking: CreateTrailingMaskForCountInBundle");
            var testCount = Vector<int>.Count / 2;

            var simdTime = BenchmarkFunction(() => TestCreateTrailingMaskForCountInBundle(testCount), "CreateTrailingMaskForCountInBundle (SIMD)");
            var refTime = BenchmarkFunction(() => TestCreateTrailingMaskForCountInBundle_Reference(testCount), "CreateTrailingMaskForCountInBundle (Reference)");

            Console.WriteLine($"  SIMD:      {simdTime:F2} ns/iteration");
            Console.WriteLine($"  Reference: {refTime:F2} ns/iteration");
            Console.WriteLine($"  Speedup:   {refTime / simdTime:F2}x");
            Console.WriteLine();
        }

        private static void BenchmarkCreateMaskForCountInBundle()
        {
            Console.WriteLine("Benchmarking: CreateMaskForCountInBundle");
            var testCount = Vector<int>.Count / 2;

            var simdTime = BenchmarkFunction(() => TestCreateMaskForCountInBundle(testCount), "CreateMaskForCountInBundle (SIMD)");
            var refTime = BenchmarkFunction(() => TestCreateMaskForCountInBundle_Reference(testCount), "CreateMaskForCountInBundle (Reference)");

            Console.WriteLine($"  SIMD:      {simdTime:F2} ns/iteration");
            Console.WriteLine($"  Reference: {refTime:F2} ns/iteration");
            Console.WriteLine($"  Speedup:   {refTime / simdTime:F2}x");
            Console.WriteLine();
        }
    }
}
