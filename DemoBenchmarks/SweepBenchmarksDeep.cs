using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static DemoBenchmarks.BenchmarkHelper;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all sweep tests excluded from <see cref="SweepBenchmarks"/>.
/// </summary>
public class SweepBenchmarksDeep
{
    Sweeper sweeper;
    [GlobalSetup]
    public unsafe void Setup()
    {
        sweeper = new Sweeper();
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        sweeper.Cleanup();
    }

    [Benchmark]
    public void SphereSphere() => sweeper.Test<Sphere, Sphere>();
    [Benchmark]
    public void SphereBox() => sweeper.Test<Sphere, Box>();
    [Benchmark]
    public void SphereCylinder() => sweeper.Test<Sphere, Cylinder>();
    [Benchmark]
    public void SphereConvexHull() => sweeper.Test<Sphere, ConvexHull>(); //GJK
    [Benchmark]
    public void SphereCompound() => sweeper.Test<Sphere, Compound>();
    [Benchmark]
    public void SphereBigCompound() => sweeper.Test<Sphere, BigCompound>();
    [Benchmark]
    public void SphereMesh() => sweeper.Test<Sphere, Mesh>();
    [Benchmark]
    public void CapsuleTriangle() => sweeper.Test<Capsule, Triangle>(); //GJK
    [Benchmark]
    public void CapsuleConvexHull() => sweeper.Test<Capsule, ConvexHull>(); //GJK
    [Benchmark]
    public void CapsuleCompound() => sweeper.Test<Capsule, Compound>();
    [Benchmark]
    public void CapsuleBigCompound() => sweeper.Test<Capsule, BigCompound>();
    [Benchmark]
    public void CapsuleMesh() => sweeper.Test<Capsule, Mesh>();
    [Benchmark]
    public void BoxBox() => sweeper.Test<Box, Box>(); //GJK
    [Benchmark]
    public void BoxTriangle() => sweeper.Test<Box, Triangle>(); //GJK
    [Benchmark]
    public void BoxCylinder() => sweeper.Test<Box, Cylinder>(); //GJK
    [Benchmark]
    public void BoxConvexHull() => sweeper.Test<Box, ConvexHull>(); //GJK
    [Benchmark]
    public void BoxBigCompound() => sweeper.Test<Box, BigCompound>(); //Well covered by bigcompound-bigcompound.
    [Benchmark]
    public void TriangleTriangle() => sweeper.Test<Triangle, Triangle>(); //GJK
    [Benchmark]
    public void TriangleCylinder() => sweeper.Test<Triangle, Cylinder>(); //GJK
    [Benchmark]
    public void TriangleConvexHull() => sweeper.Test<Triangle, ConvexHull>(); //GJK
    [Benchmark]
    public void TriangleCompound() => sweeper.Test<Triangle, Compound>();
    [Benchmark]
    public void TriangleBigCompound() => sweeper.Test<Triangle, BigCompound>();
    [Benchmark]
    public void TriangleMesh() => sweeper.Test<Triangle, Mesh>();
    [Benchmark]
    public void CylinderCylinder() => sweeper.Test<Cylinder, Cylinder>(); //GJK
    [Benchmark]
    public void CylinderConvexHull() => sweeper.Test<Cylinder, ConvexHull>(); //GJK
    [Benchmark]
    public void CylinderCompound() => sweeper.Test<Cylinder, Compound>();
    [Benchmark]
    public void CylinderBigCompound() => sweeper.Test<Cylinder, BigCompound>();
    [Benchmark]
    public void CylinderMesh() => sweeper.Test<Cylinder, Mesh>();
    [Benchmark]
    public void ConvexHullConvexHull() => sweeper.Test<ConvexHull, ConvexHull>(); //GJK
    [Benchmark]
    public void ConvexHullCompound() => sweeper.Test<ConvexHull, Compound>();
    [Benchmark]
    public void ConvexHullBigCompound() => sweeper.Test<ConvexHull, BigCompound>();
    [Benchmark]
    public void ConvexHullMesh() => sweeper.Test<ConvexHull, Mesh>();
    [Benchmark]
    public void CompoundCompound() => sweeper.Test<Compound, Compound>();
    [Benchmark]
    public void CompoundBigCompound() => sweeper.Test<Compound, BigCompound>();
    [Benchmark]
    public void CompoundMesh() => sweeper.Test<Compound, Mesh>();
    [Benchmark]
    public void BigCompoundMesh() => sweeper.Test<BigCompound, Mesh>();
    //No mesh-mesh!
}
