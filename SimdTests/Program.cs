using System;

namespace SimdTests
{
    class Program
    {
        static int Main(string[] args)
        {
            Console.WriteLine("SIMD Testing and Performance Harness");
            Console.WriteLine("====================================\n");

            if (args.Length > 0 && args[0].ToLower() == "benchmark")
            {
                Console.WriteLine("Running performance benchmarks...\n");
                PerformanceHarness.RunAllBenchmarks();
                return 0;
            }
            else if (args.Length > 0 && args[0].ToLower() == "test")
            {
                Console.WriteLine("To run correctness tests, use: dotnet test\n");
                return 0;
            }
            else
            {
                Console.WriteLine("Usage:");
                Console.WriteLine("  dotnet run benchmark  - Run performance benchmarks");
                Console.WriteLine("  dotnet test          - Run correctness tests");
                Console.WriteLine();
                Console.WriteLine("Running benchmarks by default...\n");
                PerformanceHarness.RunAllBenchmarks();
                return 0;
            }
        }
    }
}
