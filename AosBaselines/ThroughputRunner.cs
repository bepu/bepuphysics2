using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AosBaselines;

/// <summary>
/// Measures ns/pair for the wide (AoSoA) testers against the scalar (AoS) mirrors on identical input data.
/// Both paths iterate over preloaded input arrays with equivalent per-pair memory footprints and accumulate a cheap sink
/// from the manifold results to prevent dead code elimination.
/// </summary>
public static class ThroughputRunner
{
    public static float Sink;

    struct SphereWideBundle
    {
        public SphereWide A;
        public SphereWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
    }

    struct BoxWideBundle
    {
        public BoxWide A;
        public BoxWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct BoxTriangleWideBundle
    {
        public BoxWide A;
        public TriangleWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct CylinderWideBundle
    {
        public CylinderWide A;
        public CylinderWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct BoxCylinderWideBundle
    {
        public BoxWide A;
        public CylinderWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct CapsuleCylinderWideBundle
    {
        public CapsuleWide A;
        public CylinderWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct TriangleCylinderWideBundle
    {
        public TriangleWide A;
        public CylinderWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct CapsulePairWideBundle
    {
        public CapsuleWide A;
        public CapsuleWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct CapsuleBoxWideBundle
    {
        public CapsuleWide A;
        public BoxWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct CapsuleTriangleWideBundle
    {
        public CapsuleWide A;
        public TriangleWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct TrianglePairWideBundle
    {
        public TriangleWide A;
        public TriangleWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct HullWideBundle
    {
        public ConvexHullWide A;
        public ConvexHullWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    /// <summary>
    /// Allocates a 64 byte aligned unmanaged buffer. The engine's BufferPool hands out aligned memory, and managed arrays are only 8 byte aligned,
    /// which makes cache-resident wide benchmarks noisy: 32 byte Vector loads randomly straddle cache lines depending on allocation luck.
    /// </summary>
    static unsafe Span<T> AllocateAligned<T>(int count, List<IntPtr> allocations) where T : unmanaged
    {
        var pointer = NativeMemory.AlignedAlloc((nuint)(count * sizeof(T)), 64);
        allocations.Add((IntPtr)pointer);
        return new Span<T>(pointer, count);
    }

    static unsafe void FreeAll(List<IntPtr> allocations)
    {
        foreach (var pointer in allocations)
            NativeMemory.AlignedFree((void*)pointer);
        allocations.Clear();
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchSphereWide(Span<SphereWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            SpherePairTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, Vector<float>.Count, out var manifold);
            sum += manifold.Depth + manifold.Normal.X + manifold.OffsetA.Y;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchSphereScalar(Span<SphereSphereCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            SpherePairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, out var manifold);
            sum += manifold.Depth + manifold.Normal.X + manifold.OffsetA.Y;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchBoxWide(Span<BoxWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            BoxPairTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchBoxScalarFma(Span<BoxBoxCase> cases)
    {
        var result = 0f;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            BoxPairScalarTesterFma.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            result += manifold.Depth0 + manifold.OffsetA0.X + manifold.Normal.X;
        }
        return result;
    }

    static float BenchBoxScalar(Span<BoxBoxCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            BoxPairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchBoxTriangleWide(Span<BoxTriangleWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            BoxTriangleTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchBoxTriangleScalar(Span<BoxTriangleCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            BoxTriangleScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCylinderWide(Span<CylinderWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            CylinderPairTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCylinderScalar(Span<CylinderCylinderCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            CylinderPairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchBoxCylinderWide(Span<BoxCylinderWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            BoxCylinderTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchBoxCylinderScalar(Span<BoxCylinderCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            BoxCylinderScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleCylinderWide(Span<CapsuleCylinderWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            CapsuleCylinderTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleCylinderScalar(Span<CapsuleCylinderCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            CapsuleCylinderScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchTriangleCylinderWide(Span<TriangleCylinderWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            TriangleCylinderTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchTriangleCylinderScalar(Span<TriangleCylinderCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            TriangleCylinderScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsulePairWide(Span<CapsulePairWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            CapsulePairTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsulePairScalar(Span<CapsulePairCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            CapsulePairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleBoxWide(Span<CapsuleBoxWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            CapsuleBoxTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleBoxScalar(Span<CapsuleBoxCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            CapsuleBoxScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleTriangleWide(Span<CapsuleTriangleWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            CapsuleTriangleTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleTriangleScalar(Span<CapsuleTriangleCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            CapsuleTriangleScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchTrianglePairWide(Span<TrianglePairWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            TrianglePairTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchTrianglePairScalar(Span<TrianglePairCase> cases)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            TrianglePairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchHullWide(Span<HullWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            ConvexHullPairTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchHullScalar(Span<HullPairCase> cases, ConvexHull[] hulls)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            ConvexHullPairScalarTester.Test(ref hulls[pair.A], ref hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    struct CapsuleHullWideBundle
    {
        public CapsuleWide A;
        public ConvexHullWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    struct ShapeHullWideBundle<TShapeWideA> where TShapeWideA : unmanaged
    {
        public TShapeWideA A;
        public ConvexHullWide B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleHullWide(Span<CapsuleHullWideBundle> bundles)
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            CapsuleConvexHullTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchCapsuleHullScalar(Span<ShapeHullCase<Capsule>> cases, ConvexHull[] hulls)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            CapsuleConvexHullScalarTester.Test(pair.A, ref hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.OffsetA0.X;
        }
        return sum;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchShapeHullWide<TShapeWideA, TTester>(Span<ShapeHullWideBundle<TShapeWideA>> bundles)
        where TShapeWideA : unmanaged
        where TTester : IPairTester<TShapeWideA, ConvexHullWide, Convex4ContactManifoldWide>
    {
        var sum = Vector<float>.Zero;
        for (int i = 0; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            TTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationA, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return Vector.Sum(sum);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchShapeHullScalar<TShapeA, TScalarTester>(Span<ShapeHullCase<TShapeA>> cases, ConvexHull[] hulls)
        where TScalarTester : IShapeHullScalarTester<TShapeA>
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            TScalarTester.Test(pair.A, ref hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    public static unsafe void RunCapsuleHull(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.Create(setupRandom, hullCount: 32, minPointCount: 4, maxPointCount: 64);
        var generator = new CapsuleHullGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<CapsuleHullWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<ShapeHullCase<Capsule>>(pairCount, allocations);
        bundles.Clear();
        hullSet.Pool.Take<ConvexHull>(bundleCount * laneCount, out var hullReferenceBuffer);
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            hullReferenceBuffer.Slice(i * laneCount, laneCount, out bundle.B.Hulls);
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, hullSet.Hulls[pair.B]);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                CapsuleConvexHullScalarTester.Test(pair.A, ref hullSet.Hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var hulls = hullSet.Hulls;
        var (wide, scalar) = MeasurePair(
            () => BenchCapsuleHullWide(new Span<CapsuleHullWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchCapsuleHullScalar(new Span<ShapeHullCase<Capsule>>((void*)casesPointer, pairCount), hulls),
            pairCount, repsPerTrial, trialCount, warmupCount: 10);
        Report($"Capsule-hull, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunShapeHull<TShapeA, TShapeWideA, TTester, TScalarTester>(
        string name, int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy,
        Func<int, HullSet, bool, ShapeHullGeneratorBase<TShapeA>> createGenerator)
        where TShapeA : unmanaged, IShape
        where TShapeWideA : unmanaged, IShapeWide<TShapeA>
        where TTester : IPairTester<TShapeWideA, ConvexHullWide, Convex4ContactManifoldWide>
        where TScalarTester : IShapeHullScalarTester<TShapeA>
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.Create(setupRandom, hullCount: 32, minPointCount: 4, maxPointCount: 64);
        var generator = createGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<ShapeHullWideBundle<TShapeWideA>>(bundleCount, allocations);
        var cases = AllocateAligned<ShapeHullCase<TShapeA>>(pairCount, allocations);
        bundles.Clear();
        hullSet.Pool.Take<ConvexHull>(bundleCount * laneCount, out var hullReferenceBuffer);
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            hullReferenceBuffer.Slice(i * laneCount, laneCount, out bundle.B.Hulls);
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, hullSet.Hulls[pair.B]);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                TScalarTester.Test(pair.A, ref hullSet.Hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var hulls = hullSet.Hulls;
        var (wide, scalar) = MeasurePair(
            () => BenchShapeHullWide<TShapeWideA, TTester>(new Span<ShapeHullWideBundle<TShapeWideA>>((void*)bundlesPointer, bundleCount)),
            () => BenchShapeHullScalar<TShapeA, TScalarTester>(new Span<ShapeHullCase<TShapeA>>((void*)casesPointer, pairCount), hulls),
            pairCount, repsPerTrial, trialCount, warmupCount: 10);
        Report($"{name}, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    struct SphereVariantWideBundle<TShapeWideB> where TShapeWideB : unmanaged
    {
        public SphereWide A;
        public TShapeWideB B;
        public Vector<float> SpeculativeMargin;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationB;
    }

    //Both kernels use four independent accumulators: a single accumulator forms a loop-carried dependency chain
    //(three serial adds per iteration) that can throttle the fastest kernels on both sides.
    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchSphereVariantWide<TShapeWideB, TTester>(Span<SphereVariantWideBundle<TShapeWideB>> bundles)
        where TShapeWideB : unmanaged
        where TTester : IPairTester<SphereWide, TShapeWideB, Convex1ContactManifoldWide>
    {
        var sum0 = Vector<float>.Zero;
        var sum1 = Vector<float>.Zero;
        var sum2 = Vector<float>.Zero;
        var sum3 = Vector<float>.Zero;
        int i = 0;
        for (; i + 3 < bundles.Length; i += 4)
        {
            ref var bundle0 = ref bundles[i];
            TTester.Test(ref bundle0.A, ref bundle0.B, ref bundle0.SpeculativeMargin, ref bundle0.OffsetB, ref bundle0.OrientationB, Vector<float>.Count, out var manifold0);
            sum0 += manifold0.Depth + manifold0.Normal.X + manifold0.OffsetA.Y;
            ref var bundle1 = ref bundles[i + 1];
            TTester.Test(ref bundle1.A, ref bundle1.B, ref bundle1.SpeculativeMargin, ref bundle1.OffsetB, ref bundle1.OrientationB, Vector<float>.Count, out var manifold1);
            sum1 += manifold1.Depth + manifold1.Normal.X + manifold1.OffsetA.Y;
            ref var bundle2 = ref bundles[i + 2];
            TTester.Test(ref bundle2.A, ref bundle2.B, ref bundle2.SpeculativeMargin, ref bundle2.OffsetB, ref bundle2.OrientationB, Vector<float>.Count, out var manifold2);
            sum2 += manifold2.Depth + manifold2.Normal.X + manifold2.OffsetA.Y;
            ref var bundle3 = ref bundles[i + 3];
            TTester.Test(ref bundle3.A, ref bundle3.B, ref bundle3.SpeculativeMargin, ref bundle3.OffsetB, ref bundle3.OrientationB, Vector<float>.Count, out var manifold3);
            sum3 += manifold3.Depth + manifold3.Normal.X + manifold3.OffsetA.Y;
        }
        for (; i < bundles.Length; ++i)
        {
            ref var bundle = ref bundles[i];
            TTester.Test(ref bundle.A, ref bundle.B, ref bundle.SpeculativeMargin, ref bundle.OffsetB, ref bundle.OrientationB, Vector<float>.Count, out var manifold);
            sum0 += manifold.Depth + manifold.Normal.X + manifold.OffsetA.Y;
        }
        return Vector.Sum(sum0 + sum1 + sum2 + sum3);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchSphereVariantScalar<TShapeB, TScalarTester>(Span<SphereVariantCase<TShapeB>> cases)
        where TScalarTester : ISphereVariantScalarTester<TShapeB>
    {
        float sum0 = 0;
        float sum1 = 0;
        float sum2 = 0;
        float sum3 = 0;
        int i = 0;
        for (; i + 3 < cases.Length; i += 4)
        {
            ref var pair0 = ref cases[i];
            TScalarTester.Test(pair0.A, pair0.B, pair0.SpeculativeMargin, pair0.OffsetB, pair0.OrientationB, out var manifold0);
            sum0 += manifold0.Depth + manifold0.Normal.X + manifold0.OffsetA.Y;
            ref var pair1 = ref cases[i + 1];
            TScalarTester.Test(pair1.A, pair1.B, pair1.SpeculativeMargin, pair1.OffsetB, pair1.OrientationB, out var manifold1);
            sum1 += manifold1.Depth + manifold1.Normal.X + manifold1.OffsetA.Y;
            ref var pair2 = ref cases[i + 2];
            TScalarTester.Test(pair2.A, pair2.B, pair2.SpeculativeMargin, pair2.OffsetB, pair2.OrientationB, out var manifold2);
            sum2 += manifold2.Depth + manifold2.Normal.X + manifold2.OffsetA.Y;
            ref var pair3 = ref cases[i + 3];
            TScalarTester.Test(pair3.A, pair3.B, pair3.SpeculativeMargin, pair3.OffsetB, pair3.OrientationB, out var manifold3);
            sum3 += manifold3.Depth + manifold3.Normal.X + manifold3.OffsetA.Y;
        }
        for (; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            TScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationB, out var manifold);
            sum0 += manifold.Depth + manifold.Normal.X + manifold.OffsetA.Y;
        }
        return sum0 + sum1 + sum2 + sum3;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchSphereBoxScalar2Way(Span<SphereVariantCase<Box>> cases)
    {
        float sum0 = 0;
        float sum1 = 0;
        for (int i = 0; i + 1 < cases.Length; i += 2)
        {
            SphereBoxScalarTesterPipelined.Test2(cases[i], cases[i + 1], out var manifold0, out var manifold1);
            sum0 += manifold0.Depth + manifold0.Normal.X + manifold0.OffsetA.Y;
            sum1 += manifold1.Depth + manifold1.Normal.X + manifold1.OffsetA.Y;
        }
        return sum0 + sum1;
    }

    /// <summary>
    /// Measures one-pair-at-a-time scalar sphere-box against a two-pair statement-interleaved version to test whether
    /// the out-of-order core can already overlap consecutive pair iterations by itself.
    /// </summary>
    public static unsafe void RunSphereBoxPipelineExperiment(int pairCount, int repsPerTrial, int trialCount, int seed)
    {
        var generator = new SphereBoxGenerator(seed, contactHeavy: true);
        var allocations = new List<IntPtr>();
        var cases = AllocateAligned<SphereVariantCase<Box>>(pairCount, allocations);
        for (int i = 0; i < pairCount; ++i)
            cases[i] = generator.Next();
        //Sanity check: interleaving must not change per-pair results.
        long mismatches = 0;
        for (int i = 0; i + 1 < pairCount; i += 2)
        {
            SphereBoxScalarTesterBranchy.Test(cases[i].A, cases[i].B, cases[i].SpeculativeMargin, cases[i].OffsetB, cases[i].OrientationB, out var reference0);
            SphereBoxScalarTesterBranchy.Test(cases[i + 1].A, cases[i + 1].B, cases[i + 1].SpeculativeMargin, cases[i + 1].OffsetB, cases[i + 1].OrientationB, out var reference1);
            SphereBoxScalarTesterPipelined.Test2(cases[i], cases[i + 1], out var pipelined0, out var pipelined1);
            if (BitConverter.SingleToUInt32Bits(reference0.Depth) != BitConverter.SingleToUInt32Bits(pipelined0.Depth) ||
                BitConverter.SingleToUInt32Bits(reference1.Depth) != BitConverter.SingleToUInt32Bits(pipelined1.Depth) ||
                reference0.ContactExists != pipelined0.ContactExists || reference1.ContactExists != pipelined1.ContactExists)
                ++mismatches;
        }
        Console.WriteLine($"Pipelined bitwise check: {mismatches} mismatches over {pairCount} pairs.");
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var (oneWay, twoWay) = MeasurePair(
            () => BenchSphereVariantScalar<Box, SphereBoxScalarTesterBranchy>(new Span<SphereVariantCase<Box>>((void*)casesPointer, pairCount)),
            () => BenchSphereBoxScalar2Way(new Span<SphereVariantCase<Box>>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount);
        Console.WriteLine($"Sphere-box scalar pipelining experiment ({pairCount} pairs, contact-heavy):");
        Console.WriteLine($"    1-way (branchy):        min {oneWay.minNs,8:F2} ns/pair, median {oneWay.medianNs,8:F2} ns/pair");
        Console.WriteLine($"    2-way interleaved:      min {twoWay.minNs,8:F2} ns/pair, median {twoWay.medianNs,8:F2} ns/pair");
        Console.WriteLine($"    2-way speedup: {oneWay.minNs / twoWay.minNs:F2}x (by min), {oneWay.medianNs / twoWay.medianNs:F2}x (by median)");
        FreeAll(allocations);
    }

    /// <summary>
    /// Generic throughput runner for sphere-versus-X testers. The createWideShape hook exists for shapes whose wide form
    /// needs backing memory per bundle (convex hulls); everything else passes a factory returning default.
    /// </summary>
    public static unsafe void RunSphereVariant<TShapeB, TShapeWideB, TTester, TScalarTester, TGenerator>(
        string name, TGenerator generator, int pairCount, int repsPerTrial, int trialCount, Func<int, TShapeWideB> createWideShape)
        where TShapeB : unmanaged, IShape
        where TShapeWideB : unmanaged, IShapeWide<TShapeB>
        where TTester : IPairTester<SphereWide, TShapeWideB, Convex1ContactManifoldWide>
        where TScalarTester : ISphereVariantScalarTester<TShapeB>
        where TGenerator : ISphereVariantGenerator<TShapeB>
    {
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<SphereVariantWideBundle<TShapeWideB>>(bundleCount, allocations);
        var cases = AllocateAligned<SphereVariantCase<TShapeB>>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            bundle.B = createWideShape(i);
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                TScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationB, out var manifold);
                if (manifold.ContactExists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var (wide, scalar) = MeasurePair(
            () => BenchSphereVariantWide<TShapeWideB, TTester>(new Span<SphereVariantWideBundle<TShapeWideB>>((void*)bundlesPointer, bundleCount)),
            () => BenchSphereVariantScalar<TShapeB, TScalarTester>(new Span<SphereVariantCase<TShapeB>>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount);
        Report($"{name} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static void RunSphereVariants(int pairCount, int trialCount, int seed, bool contactHeavy)
    {
        var label = contactHeavy ? "hot, contact-heavy" : "hot, mixed";
        RunSphereVariant<Capsule, CapsuleWide, SphereCapsuleTester, SphereCapsuleScalarTester, SphereCapsuleGenerator>(
            $"Sphere-capsule, {label}", new SphereCapsuleGenerator(seed, contactHeavy), pairCount, repsPerTrial: 2000, trialCount, _ => default);
        RunSphereVariant<Box, BoxWide, SphereBoxTester, SphereBoxScalarTester, SphereBoxGenerator>(
            $"Sphere-box, {label}", new SphereBoxGenerator(seed + 1, contactHeavy), pairCount, repsPerTrial: 2000, trialCount, _ => default);
        RunSphereVariant<Box, BoxWide, SphereBoxTester, SphereBoxScalarTesterBranchy, SphereBoxGenerator>(
            $"Sphere-box (branchy scalar), {label}", new SphereBoxGenerator(seed + 1, contactHeavy), pairCount, repsPerTrial: 2000, trialCount, _ => default);
        RunSphereVariant<Box, BoxWide, SphereBoxTester, SphereBoxScalarTesterPacked, SphereBoxGenerator>(
            $"Sphere-box (packed scalar), {label}", new SphereBoxGenerator(seed + 1, contactHeavy), pairCount, repsPerTrial: 2000, trialCount, _ => default);
        RunSphereVariant<Triangle, TriangleWide, SphereTriangleTester, SphereTriangleScalarTester, SphereTriangleGenerator>(
            $"Sphere-triangle, {label}", new SphereTriangleGenerator(seed + 2, contactHeavy), pairCount, repsPerTrial: 2000, trialCount, _ => default);
        RunSphereVariant<Cylinder, CylinderWide, SphereCylinderTester, SphereCylinderScalarTester, SphereCylinderGenerator>(
            $"Sphere-cylinder, {label}", new SphereCylinderGenerator(seed + 3, contactHeavy), pairCount, repsPerTrial: 2000, trialCount, _ => default);
        RunSphereVariant<Cylinder, CylinderWide, SphereCylinderTester, SphereCylinderScalarTesterBranchy, SphereCylinderGenerator>(
            $"Sphere-cylinder (branchy scalar), {label}", new SphereCylinderGenerator(seed + 3, contactHeavy), pairCount, repsPerTrial: 2000, trialCount, _ => default);
        {
            var setupRandom = new Random(seed + 4);
            using var hullSet = HullSet.Create(setupRandom, hullCount: 32, minPointCount: 4, maxPointCount: 64);
            int laneCount = Vector<float>.Count;
            hullSet.Pool.Take<ConvexHull>(pairCount / laneCount * laneCount, out var hullReferenceBuffer);
            var createHullWide = (int bundleIndex) =>
            {
                var wide = default(ConvexHullWide);
                hullReferenceBuffer.Slice(bundleIndex * laneCount, laneCount, out wide.Hulls);
                return wide;
            };
            RunSphereVariant<ConvexHull, ConvexHullWide, SphereConvexHullTester, SphereConvexHullScalarTester, SphereHullGenerator>(
                $"Sphere-hull (4-64 pts), {label}", new SphereHullGenerator(seed + 5, hullSet, contactHeavy), pairCount, repsPerTrial: 60, trialCount, createHullWide);
        }
    }

    /// <summary>
    /// Measures both implementations with interleaved trials so that frequency/thermal drift over the run affects both paths equally.
    /// </summary>
    /// <summary>
    /// Milliseconds to idle between timing trials (and after warmup). Laptop turbo budgets deplete under sustained load, so
    /// back-to-back trials measure a monotonically deboosting clock; duty-cycling keeps trials in the boosted regime.
    /// Interleaving already protects ratios against clock drift; this protects the absolutes.
    /// </summary>
    public static int InterTrialCooldownMilliseconds;

    static ((double minNs, double medianNs) first, (double minNs, double medianNs) second) MeasurePair(
        Func<float> runFirst, Func<float> runSecond, long pairsPerInvocation, int repsPerTrial, int trialCount, int warmupCount = 30)
    {
        //Warmup: get everything through tiered compilation and let clocks ramp. Time-based with an invocation floor:
        //call counting toward tier1 doesn't start until ~100ms of activity, and dynamic PGO adds an instrumented stage plus
        //background compilation, so a fixed small invocation count measures pre-steady-state code in early trials.
        //A couple of sustained seconds reliably reaches final PGO'd tier1 (and ramps clocks) before any timing begins.
        const double warmupSeconds = 2.0;
        var warmupStart = Stopwatch.GetTimestamp();
        for (int i = 0; i < warmupCount || (Stopwatch.GetTimestamp() - warmupStart) < warmupSeconds * Stopwatch.Frequency; ++i)
        {
            Sink += runFirst();
            Sink += runSecond();
        }
        if (InterTrialCooldownMilliseconds > 0)
            System.Threading.Thread.Sleep(InterTrialCooldownMilliseconds * 4); //Extra recovery after the sustained warmup burn.
        var resultsFirst = new double[trialCount];
        var resultsSecond = new double[trialCount];
        double Trial(Func<float> run)
        {
            var start = Stopwatch.GetTimestamp();
            for (int rep = 0; rep < repsPerTrial; ++rep)
                Sink += run();
            var end = Stopwatch.GetTimestamp();
            return (end - start) * 1e9 / ((double)Stopwatch.Frequency * repsPerTrial * pairsPerInvocation);
        }
        for (int trial = 0; trial < trialCount; ++trial)
        {
            resultsFirst[trial] = Trial(runFirst);
            resultsSecond[trial] = Trial(runSecond);
            if (InterTrialCooldownMilliseconds > 0)
                System.Threading.Thread.Sleep(InterTrialCooldownMilliseconds);
        }
        Array.Sort(resultsFirst);
        Array.Sort(resultsSecond);
        return ((resultsFirst[0], resultsFirst[trialCount / 2]), (resultsSecond[0], resultsSecond[trialCount / 2]));
    }

    static void Report(string name, (double minNs, double medianNs) wide, (double minNs, double medianNs) scalar)
    {
        Console.WriteLine($"{name}:");
        Console.WriteLine($"    AoSoA (wide):  min {wide.minNs,8:F2} ns/pair, median {wide.medianNs,8:F2} ns/pair");
        Console.WriteLine($"    AoS (scalar):  min {scalar.minNs,8:F2} ns/pair, median {scalar.medianNs,8:F2} ns/pair");
        Console.WriteLine($"    AoSoA speedup: {scalar.minNs / wide.minNs:F2}x (by min), {scalar.medianNs / wide.medianNs:F2}x (by median)");
    }

    public static unsafe void RunSphere(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new SphereSphereGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<SphereWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<SphereSphereCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                margins[j] = pair.SpeculativeMargin;
                SpherePairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, out var manifold);
                if (manifold.ContactExists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        //Scale warmup down for giant streaming sets; tiered compilation only needs a handful of invocations there, and long sustained warmup invites throttling noise.
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchSphereWide(new Span<SphereWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchSphereScalar(new Span<SphereSphereCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Sphere-sphere, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunBoxFma(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new BoxBoxGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<BoxWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<BoxBoxCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                BoxPairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        //Scalar bitwise vs scalar FMA, interleaved for fairness; then wide vs FMA for the cross-representation ratio.
        var (scalar, fma) = MeasurePair(
            () => BenchBoxScalar(new Span<BoxBoxCase>((void*)casesPointer, pairCount)),
            () => BenchBoxScalarFma(new Span<BoxBoxCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        var (wide, fma2) = MeasurePair(
            () => BenchBoxWide(new Span<BoxWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchBoxScalarFma(new Span<BoxBoxCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Console.WriteLine($"Box-box FMA, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact):");
        Console.WriteLine($"    scalar bitwise: min {scalar.minNs,8:F2} ns/pair, median {scalar.medianNs,8:F2} ns/pair");
        Console.WriteLine($"    scalar FMA:     min {fma.minNs,8:F2} ns/pair, median {fma.medianNs,8:F2} ns/pair ({scalar.minNs / fma.minNs:F2}x vs bitwise by min)");
        Console.WriteLine($"    AoSoA (wide):   min {wide.minNs,8:F2} ns/pair, median {wide.medianNs,8:F2} ns/pair (FMA run2 min {fma2.minNs:F2}; wide advantage over FMA {fma2.minNs / wide.minNs:F2}x)");
        FreeAll(allocations);
    }

    public static unsafe void RunBox(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new BoxBoxGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<BoxWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<BoxBoxCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                BoxPairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchBoxWide(new Span<BoxWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchBoxScalar(new Span<BoxBoxCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Box-box, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunBoxTriangle(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new BoxTriangleGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<BoxTriangleWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<BoxTriangleCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                BoxTriangleScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchBoxTriangleWide(new Span<BoxTriangleWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchBoxTriangleScalar(new Span<BoxTriangleCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Box-triangle, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunCylinder(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new CylinderCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<CylinderWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<CylinderCylinderCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                CylinderPairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchCylinderWide(new Span<CylinderWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchCylinderScalar(new Span<CylinderCylinderCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Cylinder-cylinder, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunBoxCylinder(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new BoxCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<BoxCylinderWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<BoxCylinderCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                BoxCylinderScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchBoxCylinderWide(new Span<BoxCylinderWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchBoxCylinderScalar(new Span<BoxCylinderCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Box-cylinder, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunCapsuleCylinder(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new CapsuleCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<CapsuleCylinderWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<CapsuleCylinderCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                CapsuleCylinderScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchCapsuleCylinderWide(new Span<CapsuleCylinderWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchCapsuleCylinderScalar(new Span<CapsuleCylinderCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Capsule-cylinder, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunTriangleCylinder(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new TriangleCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<TriangleCylinderWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<TriangleCylinderCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                TriangleCylinderScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchTriangleCylinderWide(new Span<TriangleCylinderWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchTriangleCylinderScalar(new Span<TriangleCylinderCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Triangle-cylinder, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunCapsulePair(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new CapsulePairGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<CapsulePairWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<CapsulePairCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                CapsulePairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchCapsulePairWide(new Span<CapsulePairWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchCapsulePairScalar(new Span<CapsulePairCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Capsule-capsule, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunCapsuleBox(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new CapsuleBoxGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<CapsuleBoxWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<CapsuleBoxCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                CapsuleBoxScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchCapsuleBoxWide(new Span<CapsuleBoxWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchCapsuleBoxScalar(new Span<CapsuleBoxCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Capsule-box, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunCapsuleTriangle(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new CapsuleTriangleGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<CapsuleTriangleWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<CapsuleTriangleCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                CapsuleTriangleScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchCapsuleTriangleWide(new Span<CapsuleTriangleWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchCapsuleTriangleScalar(new Span<CapsuleTriangleCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Capsule-triangle, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    public static unsafe void RunTrianglePair(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy)
    {
        var generator = new TrianglePairGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<TrianglePairWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<TrianglePairCase>(pairCount, allocations);
        bundles.Clear();
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, pair.A);
                bundle.B.WriteSlot(j, pair.B);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                TrianglePairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var warmupCount = pairCount > 1 << 16 ? 4 : 30;
        var (wide, scalar) = MeasurePair(
            () => BenchTrianglePairWide(new Span<TrianglePairWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchTrianglePairScalar(new Span<TrianglePairCase>((void*)casesPointer, pairCount)),
            pairCount, repsPerTrial, trialCount, warmupCount);
        Report($"Triangle-triangle, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    static float BenchHullCandidate(Span<HullPairCase> cases, ConvexHull[] hulls, HullTopology[] topologies, IRelaxedHullPairTester candidate)
    {
        float sum = 0;
        for (int i = 0; i < cases.Length; ++i)
        {
            ref var pair = ref cases[i];
            candidate.Test(topologies[pair.A], topologies[pair.B], ref hulls[pair.A], ref hulls[pair.B],
                pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
            sum += manifold.Normal.X + manifold.Depth0 + manifold.Depth1 + manifold.Depth2 + manifold.Depth3 + manifold.OffsetA0.X;
        }
        return sum;
    }

    public struct HullScanResult
    {
        public int TargetVertexCount;
        public bool ContactHeavy;
        public double ActualMeanVertexCount;
        public double ContactFraction;
        public double PrecomputeMicrosecondsPerHull;
        public (double minNs, double medianNs) Wide;
        public (double minNs, double medianNs) FrozenScalar;
        public (string name, (double minNs, double medianNs) wide, (double minNs, double medianNs) candidate)[] Candidates;
        /// <summary>Direct interleaved head-to-head: frozen scalar vs candidate in the same MeasurePair (thermally robust candidate-vs-frozen ratio).</summary>
        public (string name, (double minNs, double medianNs) frozen, (double minNs, double medianNs) candidate)[] CandidatesVsFrozen;
    }

    /// <summary>
    /// Hull size scan for the relaxed-equality study: engine wide tester vs the frozen bitwise scalar baseline vs each
    /// requested relaxed candidate, on a hull set targeting a specific vertex count. Each comparison is its own
    /// interleaved MeasurePair (thermally robust ratios); every candidate is interleaved against the wide tester so
    /// candidate-vs-frozen ratios compose through the shared wide anchor. Topology precompute happens before any timing
    /// (amortized per shape) and is reported separately.
    /// </summary>
    public static unsafe HullScanResult RunHullScan(int pairCount, int repsPerTrial, int trialCount, int seed, bool contactHeavy, int targetVertexCount, IRelaxedHullPairTester[] candidates)
    {
        var label = $"target {targetVertexCount} verts, {(contactHeavy ? "contact-heavy" : "mixed")}";
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 32, targetVertexCount);
        var topologies = HullTopology.CreateForSet(hullSet, out var precomputeMilliseconds);
        HullTopology.ReportSetStatistics($"Hull scan set ({label})", topologies, precomputeMilliseconds);
        var generator = new HullHullGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<HullWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<HullPairCase>(pairCount, allocations);
        bundles.Clear();
        hullSet.Pool.Take<ConvexHull>(bundleCount * laneCount * 2, out var hullReferenceBuffer);
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            hullReferenceBuffer.Slice(i * laneCount * 2, laneCount, out bundle.A.Hulls);
            hullReferenceBuffer.Slice(i * laneCount * 2 + laneCount, laneCount, out bundle.B.Hulls);
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, hullSet.Hulls[pair.A]);
                bundle.B.WriteSlot(j, hullSet.Hulls[pair.B]);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                ConvexHullPairScalarTester.Test(ref hullSet.Hulls[pair.A], ref hullSet.Hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var hulls = hullSet.Hulls;

        var result = new HullScanResult
        {
            TargetVertexCount = targetVertexCount,
            ContactHeavy = contactHeavy,
            ActualMeanVertexCount = topologies.Average(t => t.VertexCount),
            ContactFraction = (double)contactCount / pairCount,
            PrecomputeMicrosecondsPerHull = precomputeMilliseconds * 1000.0 / topologies.Length,
            Candidates = new (string, (double, double), (double, double))[candidates.Length],
            CandidatesVsFrozen = new (string, (double, double), (double, double))[candidates.Length],
        };

        var (wide, scalar) = MeasurePair(
            () => BenchHullWide(new Span<HullWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchHullScalar(new Span<HullPairCase>((void*)casesPointer, pairCount), hulls),
            pairCount, repsPerTrial, trialCount, warmupCount: 10);
        Report($"Hull-hull scan, {label} ({pairCount} pairs, {result.ContactFraction:P0} with at least one contact)", wide, scalar);
        result.Wide = wide;
        result.FrozenScalar = scalar;

        for (int c = 0; c < candidates.Length; ++c)
        {
            var candidate = candidates[c];
            var (candidateWide, candidateResult) = MeasurePair(
                () => BenchHullWide(new Span<HullWideBundle>((void*)bundlesPointer, bundleCount)),
                () => BenchHullCandidate(new Span<HullPairCase>((void*)casesPointer, pairCount), hulls, topologies, candidate),
                pairCount, repsPerTrial, trialCount, warmupCount: 10);
            Console.WriteLine($"Candidate '{candidate.Name}', {label}:");
            Console.WriteLine($"    AoSoA (wide):  min {candidateWide.minNs,8:F2} ns/pair, median {candidateWide.medianNs,8:F2} ns/pair");
            Console.WriteLine($"    candidate:     min {candidateResult.minNs,8:F2} ns/pair, median {candidateResult.medianNs,8:F2} ns/pair");
            Console.WriteLine($"    wide/candidate ratio: {candidateResult.minNs / candidateWide.minNs:F2}x (by min), {candidateResult.medianNs / candidateWide.medianNs:F2}x (by median)");
            //Cross-ratio through the shared wide anchor: candidate vs the frozen scalar baseline.
            var crossMin = (candidateResult.minNs / candidateWide.minNs) / (scalar.minNs / wide.minNs);
            var crossMedian = (candidateResult.medianNs / candidateWide.medianNs) / (scalar.medianNs / wide.medianNs);
            Console.WriteLine($"    candidate/frozen-scalar (via wide anchor): {crossMin:F2}x (by min), {crossMedian:F2}x (by median)");
            result.Candidates[c] = (candidate.Name, candidateWide, candidateResult);
            //Direct interleaved head-to-head against the frozen scalar baseline: the thermally robust candidate-vs-frozen ratio
            //(no cross-section anchor drift).
            var (frozenDirect, candidateDirect) = MeasurePair(
                () => BenchHullScalar(new Span<HullPairCase>((void*)casesPointer, pairCount), hulls),
                () => BenchHullCandidate(new Span<HullPairCase>((void*)casesPointer, pairCount), hulls, topologies, candidate),
                pairCount, repsPerTrial, trialCount, warmupCount: 10);
            Console.WriteLine($"    head-to-head frozen {frozenDirect.minNs:F0}/{frozenDirect.medianNs:F0} vs candidate {candidateDirect.minNs:F0}/{candidateDirect.medianNs:F0} ns/pair " +
                $"-> candidate/frozen {candidateDirect.minNs / frozenDirect.minNs:F2}x (min), {candidateDirect.medianNs / frozenDirect.medianNs:F2}x (median)");
            result.CandidatesVsFrozen[c] = (candidate.Name, frozenDirect, candidateDirect);
        }
        FreeAll(allocations);
        return result;
    }

    public static unsafe void RunHull(int pairCount, int repsPerTrial, int trialCount, int seed, string label, bool contactHeavy, int minPointCount, int maxPointCount)
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.Create(setupRandom, hullCount: 32, minPointCount, maxPointCount);
        var generator = new HullHullGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        int bundleCount = pairCount / laneCount;
        pairCount = bundleCount * laneCount;
        var allocations = new List<IntPtr>();
        var bundles = AllocateAligned<HullWideBundle>(bundleCount, allocations);
        var cases = AllocateAligned<HullPairCase>(pairCount, allocations);
        bundles.Clear();
        //Backing memory for the per-bundle hull reference buffers; the hull geometry itself is shared via the hull set.
        hullSet.Pool.Take<ConvexHull>(bundleCount * laneCount * 2, out var hullReferenceBuffer);
        Span<float> margins = stackalloc float[laneCount];
        long contactCount = 0;
        for (int i = 0; i < bundleCount; ++i)
        {
            ref var bundle = ref bundles[i];
            hullReferenceBuffer.Slice(i * laneCount * 2, laneCount, out bundle.A.Hulls);
            hullReferenceBuffer.Slice(i * laneCount * 2 + laneCount, laneCount, out bundle.B.Hulls);
            for (int j = 0; j < laneCount; ++j)
            {
                var pair = generator.Next();
                cases[i * laneCount + j] = pair;
                bundle.A.WriteSlot(j, hullSet.Hulls[pair.A]);
                bundle.B.WriteSlot(j, hullSet.Hulls[pair.B]);
                Vector3Wide.WriteSlot(pair.OffsetB, j, ref bundle.OffsetB);
                QuaternionWide.WriteSlot(pair.OrientationA, j, ref bundle.OrientationA);
                QuaternionWide.WriteSlot(pair.OrientationB, j, ref bundle.OrientationB);
                margins[j] = pair.SpeculativeMargin;
                ConvexHullPairScalarTester.Test(ref hullSet.Hulls[pair.A], ref hullSet.Hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out var manifold);
                if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
                    ++contactCount;
            }
            bundle.SpeculativeMargin = new Vector<float>(margins);
        }
        var bundlesPointer = (IntPtr)Unsafe.AsPointer(ref bundles[0]);
        var casesPointer = (IntPtr)Unsafe.AsPointer(ref cases[0]);
        var hulls = hullSet.Hulls;
        var (wide, scalar) = MeasurePair(
            () => BenchHullWide(new Span<HullWideBundle>((void*)bundlesPointer, bundleCount)),
            () => BenchHullScalar(new Span<HullPairCase>((void*)casesPointer, pairCount), hulls),
            pairCount, repsPerTrial, trialCount, warmupCount: 10);
        Report($"Hull-hull, {label} ({pairCount} pairs, {(double)contactCount / pairCount:P0} with at least one contact)", wide, scalar);
        FreeAll(allocations);
    }
}
