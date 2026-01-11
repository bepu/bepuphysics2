# SIMD Performance Testing and Correctness Framework

This project provides a comprehensive testing framework for SIMD (Single Instruction, Multiple Data) implementations in BepuPhysics2. It helps identify performance bottlenecks and verify correctness of SIMD code against scalar reference implementations.

## Overview

BepuPhysics2 uses .NET hardware intrinsics for performance-critical operations. This framework helps:

1. **Verify Correctness**: Compare SIMD implementations against scalar reference implementations
2. **Measure Performance**: Benchmark SIMD code to identify optimization opportunities
3. **Test Incrementally**: Add new implementations and verify them work correctly

## Identified SIMD Functions

The codebase contains 25+ SIMD-accelerated functions. Key categories include:

### 1. Mathematical Operations (BepuUtilities/MathHelper.cs)
- `FastReciprocal(Vector<float>)` - Vector256/128, **NO ARM64**
- `FastReciprocalSquareRoot(Vector<float>)` - Vector256/128, **NO ARM64**
- `Cos/Sin/Acos(Vector<float>)` - Platform-independent

### 2. Bundle Indexing (BepuUtilities/BundleIndexing.cs)
- `CreateTrailingMaskForCountInBundle(int)` - Vector256/128, **NO ARM64**
- `CreateMaskForCountInBundle(int)` - Vector256/128, **NO ARM64**
- `GetFirstSetLaneIndex(Vector<int>)` - Vector256/128, **NO ARM64**
- `GetLastSetLaneCount(Vector<int>)` - Vector256/128, **NO ARM64**

### 3. Bounding Box Operations (BepuUtilities/BoundingBox.cs)
- Multiple AABB intersection/merging functions - **Vector128 only, NO ARM64**

### 4. Body State Operations (BepuPhysics/Bodies_GatherScatter.cs)
- `TransposeMotionStates()` - Vector256 only, **NO ARM64, NO AVX512**
- `GatherState<TAccessFilter>()` - Vector256 only, **NO ARM64, NO AVX512**

### 5. Constraint Solving (BepuPhysics/Solver_Solve.cs)
- Constraint flag merging - **Vector512/256 with fallbacks** (only Vector512 usage found)

### 6. Tree Operations (BepuPhysics/Trees/)
- `GetLeftPackMask()` - **AVX2 REQUIRED, no fallback**
- `ComputeBinIndex()` - Vector128 limited

## Project Structure

```
SimdTests/
├── SimdTests.csproj          # Project file
├── Program.cs                # Entry point for benchmarks
├── ReferenceImplementations.cs  # Scalar reference implementations
├── CorrectnessTests.cs       # xUnit tests verifying correctness
├── PerformanceHarness.cs     # Performance benchmarking framework
└── README.md                 # This file
```

## Usage

### Running Correctness Tests

```bash
cd SimdTests
dotnet test
```

This runs all xUnit tests that verify SIMD implementations match reference implementations.

### Running Performance Benchmarks

```bash
cd SimdTests
dotnet run benchmark
```

This measures CPU time for SIMD operations and compares against scalar implementations.

Sample output:
```
==============================================
SIMD Performance Benchmark Suite
==============================================
Vector<float>.Count: 8
Vector<int>.Count: 8
System.Runtime.Intrinsics.X86.Avx.IsSupported: True
System.Runtime.Intrinsics.X86.Avx2.IsSupported: True
Iterations per benchmark: 10,000,000
==============================================

Benchmarking: FastReciprocal
  SIMD:      1.23 ns/iteration
  Reference: 8.45 ns/iteration
  Speedup:   6.87x

Benchmarking: FastReciprocalSquareRoot
  SIMD:      1.45 ns/iteration
  Reference: 12.34 ns/iteration
  Speedup:   8.51x
...
```

## Adding New Tests

### 1. Add Reference Implementation

In `ReferenceImplementations.cs`:

```csharp
[MethodImpl(MethodImplOptions.NoInlining)]
public static Vector<float> MyFunction_Reference(Vector<float> input)
{
    var result = new float[Vector<float>.Count];
    for (int i = 0; i < Vector<float>.Count; i++)
    {
        result[i] = /* scalar implementation */;
    }
    return new Vector<float>(result);
}
```

### 2. Add Correctness Test

In `CorrectnessTests.cs`:

```csharp
[Fact]
public void TestMyFunction()
{
    var input = new Vector<float>(2.5f);
    var expected = ReferenceImplementations.MyFunction_Reference(input);
    var actual = MyClass.MyFunction(input);

    AssertVectorsEqual(expected, actual, "MyFunction");
}
```

### 3. Add Performance Benchmark

In `PerformanceHarness.cs`:

```csharp
private static void BenchmarkMyFunction()
{
    Console.WriteLine("Benchmarking: MyFunction");
    var testInput = new Vector<float>(2.5f);

    var simdTime = BenchmarkFunction(() => MyClass.MyFunction(testInput), "MyFunction (SIMD)");
    var refTime = BenchmarkFunction(() => ReferenceImplementations.MyFunction_Reference(testInput), "MyFunction (Reference)");

    Console.WriteLine($"  SIMD:      {simdTime:F2} ns/iteration");
    Console.WriteLine($"  Reference: {refTime:F2} ns/iteration");
    Console.WriteLine($"  Speedup:   {refTime / simdTime:F2}x");
    Console.WriteLine();
}
```

Then add the call in `RunAllBenchmarks()`.

## Current Test Coverage

The framework currently tests:

✅ **FastReciprocal** - Random values and special cases
✅ **FastReciprocalSquareRoot** - Random values
✅ **CreateTrailingMaskForCountInBundle** - All count values
✅ **CreateMaskForCountInBundle** - All count values
✅ **GetFirstSetLaneIndex** - Various mask patterns
✅ **GetLastSetLaneCount** - Various mask patterns

## Future Enhancements

Potential additions to this framework:

1. **AVX512 Support**: Add Vector512 implementations and benchmarks
2. **ARM64/NEON Support**: Add AdvSimd implementations for ARM processors
3. **More Functions**: Add tests for BoundingBox operations, TransposeMotionStates, etc.
4. **Automated Comparison**: Generate reports comparing different SIMD widths
5. **CI Integration**: Run tests automatically on different architectures
6. **Memory Bandwidth Tests**: Measure cache effects and memory access patterns

## Notes on Precision

Hardware reciprocal and reciprocal square root are **approximations** and may have slightly lower precision than software implementations:

- `Sse.Reciprocal` / `Avx.Reciprocal`: ~0.04% max relative error
- `Sse.ReciprocalSqrt` / `Avx.ReciprocalSqrt`: ~0.04% max relative error

Tests use relaxed epsilon (0.1% relative error) to account for this.

## Platform Support Summary

| Category | x86 SSE/AVX | AVX512 | ARM NEON |
|----------|-------------|---------|----------|
| Math Operations | ✅ | ❌ | ❌ |
| Bundle Indexing | ✅ | ❌ | ❌ |
| Bounding Box | ✅ (128-bit only) | ❌ | ❌ |
| Body Operations | ✅ (256-bit only) | ❌ | ❌ |
| Constraint Solving | ✅ | ✅ (partial) | ❌ |
| Tree Operations | ✅ (AVX2 required) | ❌ | ❌ |

## License

This testing framework follows the same license as BepuPhysics2.
