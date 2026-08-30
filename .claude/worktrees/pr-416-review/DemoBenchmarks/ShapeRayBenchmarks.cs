using BenchmarkDotNet.Attributes;
namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of shape ray tests. Performs groups of types in single benchmarks: all convexes in one, compounds in the other.
/// </summary>
public class ShapeRayBenchmarks
{
    ShapeRayBenchmarksDeep deep;
    [GlobalSetup]
    public void Setup()
    {
        deep = new ShapeRayBenchmarksDeep();
        deep.Setup();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        deep.Cleanup();
    }

    [Benchmark]
    public void ConvexRayTests()
    {
        deep.RaySphere();
        deep.RayCapsule();
        deep.RayBox();
        deep.RayTriangle();
        deep.RayCylinder();
        deep.RayConvexHull();
    }

    [Benchmark]
    public void CompoundRayTests()
    {
        deep.RayCompound();
        deep.RayBigCompound();
        deep.RayMesh();
    }
}
