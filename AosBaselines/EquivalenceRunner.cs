using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Text;

namespace AosBaselines;

/// <summary>
/// Runs the wide testers and their scalar mirrors over fuzzed cases and verifies bitwise equality of all observable manifold state.
/// Contact fields are only compared for contacts that exist, and the normal is only compared when at least one contact exists,
/// matching what downstream consumers (ReadFirst) can observe. Existence flags themselves are always compared.
/// </summary>
public static class EquivalenceRunner
{
    static string F(float value) => $"{value:G9} (0x{BitConverter.SingleToUInt32Bits(value):X8})";
    static string F(in Vector3 v) => $"({F(v.X)}, {F(v.Y)}, {F(v.Z)})";
    static string F(in Quaternion q) => $"({F(q.X)}, {F(q.Y)}, {F(q.Z)}, {F(q.W)})";

    static bool BitsEqual(float a, float b) => BitConverter.SingleToUInt32Bits(a) == BitConverter.SingleToUInt32Bits(b);
    static bool BitsEqual(in Vector3 a, in Vector3 b) => BitsEqual(a.X, b.X) && BitsEqual(a.Y, b.Y) && BitsEqual(a.Z, b.Z);

    static void AppendFieldComparison(StringBuilder builder, string name, float wide, float scalar)
    {
        if (!BitsEqual(wide, scalar))
            builder.AppendLine($"    {name}: wide {F(wide)} vs scalar {F(scalar)}");
    }
    static void AppendFieldComparison(StringBuilder builder, string name, in Vector3 wide, in Vector3 scalar)
    {
        AppendFieldComparison(builder, name + ".X", wide.X, scalar.X);
        AppendFieldComparison(builder, name + ".Y", wide.Y, scalar.Y);
        AppendFieldComparison(builder, name + ".Z", wide.Z, scalar.Z);
    }

    static Vector3 ReadLane(in Vector3Wide v, int lane) => new(v.X[lane], v.Y[lane], v.Z[lane]);

    public static int RunSphereSphere(long caseCount, int seed, int maxReports)
    {
        var generator = new SphereSphereGenerator(seed);
        int laneCount = Vector<float>.Count;
        var lanes = new SphereSphereCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(SphereWide);
            var bWide = default(SphereWide);
            var offsetB = default(Vector3Wide);
            //Occasionally use partial bundles to make sure inactive lane garbage can't leak into active lanes.
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            SpherePairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                SpherePairScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, out var scalarManifold);
                ++testedLanes;
                var wideExists = wideManifold.ContactExists[j] < 0;
                if (scalarManifold.ContactExists)
                    ++contactCount;

                var builder = new StringBuilder();
                if (wideExists != scalarManifold.ContactExists)
                    builder.AppendLine($"    ContactExists: wide {wideExists} vs scalar {scalarManifold.ContactExists}");
                else if (wideExists)
                {
                    AppendFieldComparison(builder, "OffsetA", ReadLane(wideManifold.OffsetA, j), scalarManifold.OffsetA);
                    AppendFieldComparison(builder, "Normal", ReadLane(wideManifold.Normal, j), scalarManifold.Normal);
                    AppendFieldComparison(builder, "Depth", wideManifold.Depth[j], scalarManifold.Depth);
                    if (wideManifold.FeatureId[j] != scalarManifold.FeatureId)
                        builder.AppendLine($"    FeatureId: wide {wideManifold.FeatureId[j]} vs scalar {scalarManifold.FeatureId}");
                }
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH sphere-sphere case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    radiusA {F(lanes[j].A.Radius)}, radiusB {F(lanes[j].B.Radius)}, margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Sphere-sphere fuzz: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with contact, {mismatches} mismatches.");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunBoxBox(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new BoxBoxGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new BoxBoxCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];
        long edgeFeatureContacts = 0;
        long vertexFeatureContacts = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(BoxWide);
            var bWide = default(BoxWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            BoxPairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                BoxPairScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount = 0;
                if (scalarManifold.Contact0Exists) { ++laneContactCount; if (scalarManifold.FeatureId0 < 0) ++vertexFeatureContacts; else ++edgeFeatureContacts; }
                if (scalarManifold.Contact1Exists) { ++laneContactCount; if (scalarManifold.FeatureId1 < 0) ++vertexFeatureContacts; else ++edgeFeatureContacts; }
                if (scalarManifold.Contact2Exists) { ++laneContactCount; if (scalarManifold.FeatureId2 < 0) ++vertexFeatureContacts; else ++edgeFeatureContacts; }
                if (scalarManifold.Contact3Exists) { ++laneContactCount; if (scalarManifold.FeatureId3 < 0) ++vertexFeatureContacts; else ++edgeFeatureContacts; }
                ++contactCountHistogram[laneContactCount];

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH box-box case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: {F(lanes[j].A.HalfWidth)}, {F(lanes[j].A.HalfHeight)}, {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b: {F(lanes[j].B.HalfWidth)}, {F(lanes[j].B.HalfHeight)}, {F(lanes[j].B.HalfLength)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Box-box fuzz: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}; " +
            $"contacts from box A vertices {(double)vertexFeatureContacts / Math.Max(1, vertexFeatureContacts + edgeFeatureContacts):P1}, from face B edges {(double)edgeFeatureContacts / Math.Max(1, vertexFeatureContacts + edgeFeatureContacts):P1}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunBoxTriangle(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new BoxTriangleGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new BoxTriangleCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];
        long boxVertexContacts = 0;
        long edgeMinContacts = 0;
        long edgeMaxContacts = 0;
        long faceFlaggedLanes = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(BoxWide);
            var bWide = default(TriangleWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            BoxTriangleTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                BoxTriangleScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount = 0;
                //Feature id classification: strip the face collision flag (only ever added to contact 0), then
                //ids < 4 are box vertex contacts, 4..7 triangle edge interval minima, 12..14 maxima.
                void Classify(bool exists, int featureId, ref long vertexContacts, ref long minContacts, ref long maxContacts, ref int count)
                {
                    if (!exists)
                        return;
                    ++count;
                    var id = featureId & ~BepuPhysics.CollisionDetection.MeshReduction.FaceCollisionFlag;
                    if (id < 4)
                        ++vertexContacts;
                    else if (id < 8)
                        ++minContacts;
                    else
                        ++maxContacts;
                }
                Classify(scalarManifold.Contact0Exists, scalarManifold.FeatureId0, ref boxVertexContacts, ref edgeMinContacts, ref edgeMaxContacts, ref laneContactCount);
                Classify(scalarManifold.Contact1Exists, scalarManifold.FeatureId1, ref boxVertexContacts, ref edgeMinContacts, ref edgeMaxContacts, ref laneContactCount);
                Classify(scalarManifold.Contact2Exists, scalarManifold.FeatureId2, ref boxVertexContacts, ref edgeMinContacts, ref edgeMaxContacts, ref laneContactCount);
                Classify(scalarManifold.Contact3Exists, scalarManifold.FeatureId3, ref boxVertexContacts, ref edgeMinContacts, ref edgeMaxContacts, ref laneContactCount);
                ++contactCountHistogram[laneContactCount];
                if (scalarManifold.Contact0Exists && (scalarManifold.FeatureId0 & BepuPhysics.CollisionDetection.MeshReduction.FaceCollisionFlag) != 0)
                    ++faceFlaggedLanes;

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH box-triangle case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: {F(lanes[j].A.HalfWidth)}, {F(lanes[j].A.HalfHeight)}, {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b.A {F(lanes[j].B.A)}");
                        Console.WriteLine($"    b.B {F(lanes[j].B.B)}");
                        Console.WriteLine($"    b.C {F(lanes[j].B.C)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        var classified = Math.Max(1, boxVertexContacts + edgeMinContacts + edgeMaxContacts);
        Console.WriteLine($"Box-triangle fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}; " +
            $"box-vertex {(double)boxVertexContacts / classified:P1} / tri-edge-min {(double)edgeMinContacts / classified:P1} / tri-edge-max {(double)edgeMaxContacts / classified:P1}; " +
            $"face-flagged lanes {(double)faceFlaggedLanes / testedLanes:P1}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    static void CompareConvex1(in Convex1ContactManifoldWide wide, int lane, in Convex1ManifoldScalar scalar, StringBuilder builder)
    {
        var wideExists = wide.ContactExists[lane] < 0;
        if (wideExists != scalar.ContactExists)
        {
            builder.AppendLine($"    ContactExists: wide {wideExists} vs scalar {scalar.ContactExists}");
            return;
        }
        if (wideExists)
        {
            AppendFieldComparison(builder, "OffsetA", ReadLane(wide.OffsetA, lane), scalar.OffsetA);
            AppendFieldComparison(builder, "Normal", ReadLane(wide.Normal, lane), scalar.Normal);
            AppendFieldComparison(builder, "Depth", wide.Depth[lane], scalar.Depth);
            if (wide.FeatureId[lane] != scalar.FeatureId)
                builder.AppendLine($"    FeatureId: wide {wide.FeatureId[lane]} vs scalar {scalar.FeatureId}");
        }
    }

    /// <summary>
    /// Generic equivalence runner for sphere-versus-X testers with orientation-B-only Test signatures and single contact manifolds.
    /// </summary>
    public static int RunSphereVariant<TShapeB, TShapeWideB, TTester, TScalarTester, TGenerator>(
        string name, TGenerator generator, long caseCount, int maxReports, TShapeWideB bWide, Func<TShapeB, string> describeB)
        where TShapeB : IShape
        where TShapeWideB : IShapeWide<TShapeB>
        where TTester : IPairTester<SphereWide, TShapeWideB, Convex1ContactManifoldWide>
        where TScalarTester : ISphereVariantScalarTester<TShapeB>
        where TGenerator : ISphereVariantGenerator<TShapeB>
    {
        int laneCount = Vector<float>.Count;
        var lanes = new SphereVariantCase<TShapeB>[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(SphereWide);
            var offsetB = default(Vector3Wide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            TTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                TScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                if (scalarManifold.ContactExists)
                    ++contactCount;

                var builder = new StringBuilder();
                CompareConvex1(wideManifold, j, scalarManifold, builder);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH {name} case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    sphere radius {F(lanes[j].A.Radius)}");
                        Console.WriteLine($"    b: {describeB(lanes[j].B)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"{name} fuzz: {testedLanes} lanes tested, {(double)contactCount / testedLanes:P1} with contact, {mismatches} mismatches.");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunSphereCapsule(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunSphereVariant<Capsule, CapsuleWide, SphereCapsuleTester, SphereCapsuleScalarTester, SphereCapsuleGenerator>(
            $"sphere-capsule{(contactHeavy ? " (contact-heavy)" : "")} (seed {seed})", new SphereCapsuleGenerator(seed, contactHeavy), caseCount, maxReports, default,
            b => $"radius {F(b.Radius)}, halfLength {F(b.HalfLength)}");
    }

    public static int RunSphereBox(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunSphereVariant<Box, BoxWide, SphereBoxTester, SphereBoxScalarTester, SphereBoxGenerator>(
            $"sphere-box{(contactHeavy ? " (contact-heavy)" : "")} (seed {seed})", new SphereBoxGenerator(seed, contactHeavy), caseCount, maxReports, default,
            b => $"halfWidth {F(b.HalfWidth)}, halfHeight {F(b.HalfHeight)}, halfLength {F(b.HalfLength)}");
    }

    public static int RunSphereBoxPacked(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunSphereVariant<Box, BoxWide, SphereBoxTester, SphereBoxScalarTesterPacked, SphereBoxGenerator>(
            $"sphere-box packed{(contactHeavy ? " (contact-heavy)" : "")} (seed {seed})", new SphereBoxGenerator(seed, contactHeavy), caseCount, maxReports, default,
            b => $"halfWidth {F(b.HalfWidth)}, halfHeight {F(b.HalfHeight)}, halfLength {F(b.HalfLength)}");
    }

    public static int RunSphereTriangle(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunSphereVariant<Triangle, TriangleWide, SphereTriangleTester, SphereTriangleScalarTester, SphereTriangleGenerator>(
            $"sphere-triangle{(contactHeavy ? " (contact-heavy)" : "")} (seed {seed})", new SphereTriangleGenerator(seed, contactHeavy), caseCount, maxReports, default,
            b => $"A {F(b.A)}, B {F(b.B)}, C {F(b.C)}");
    }

    public static int RunSphereCylinder(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunSphereVariant<Cylinder, CylinderWide, SphereCylinderTester, SphereCylinderScalarTester, SphereCylinderGenerator>(
            $"sphere-cylinder{(contactHeavy ? " (contact-heavy)" : "")} (seed {seed})", new SphereCylinderGenerator(seed, contactHeavy), caseCount, maxReports, default,
            b => $"radius {F(b.Radius)}, halfLength {F(b.HalfLength)}");
    }

    public static int RunSphereHull(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.Create(setupRandom, hullCount: 24, minPointCount: 4, maxPointCount: 64);
        hullSet.Pool.Take<ConvexHull>(Vector<float>.Count, out var hullBuffer);
        var bWide = default(ConvexHullWide);
        hullBuffer.Slice(0, Vector<float>.Count, out bWide.Hulls);
        return RunSphereVariant<ConvexHull, ConvexHullWide, SphereConvexHullTester, SphereConvexHullScalarTester, SphereHullGenerator>(
            $"sphere-hull{(contactHeavy ? " (contact-heavy)" : "")} (seed {seed})", new SphereHullGenerator(seed + 1, hullSet, contactHeavy), caseCount, maxReports, bWide,
            b => $"hull with {b.FaceToVertexIndicesStart.Length} faces");
    }

    public static int RunCylinderCylinder(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new CylinderCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new CylinderCylinderCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(CylinderWide);
            var bWide = default(CylinderWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            CylinderPairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                CylinderPairScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount =
                    (scalarManifold.Contact0Exists ? 1 : 0) + (scalarManifold.Contact1Exists ? 1 : 0) +
                    (scalarManifold.Contact2Exists ? 1 : 0) + (scalarManifold.Contact3Exists ? 1 : 0);
                ++contactCountHistogram[laneContactCount];

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH cylinder-cylinder case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: radius {F(lanes[j].A.Radius)}, halfLength {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b: radius {F(lanes[j].B.Radius)}, halfLength {F(lanes[j].B.HalfLength)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Cylinder-cylinder fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunBoxCylinder(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new BoxCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new BoxCylinderCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(BoxWide);
            var bWide = default(CylinderWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            BoxCylinderTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                BoxCylinderScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount =
                    (scalarManifold.Contact0Exists ? 1 : 0) + (scalarManifold.Contact1Exists ? 1 : 0) +
                    (scalarManifold.Contact2Exists ? 1 : 0) + (scalarManifold.Contact3Exists ? 1 : 0);
                ++contactCountHistogram[laneContactCount];

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH box-cylinder case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: halfWidth {F(lanes[j].A.HalfWidth)}, halfHeight {F(lanes[j].A.HalfHeight)}, halfLength {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b: radius {F(lanes[j].B.Radius)}, halfLength {F(lanes[j].B.HalfLength)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Box-cylinder fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunCapsuleCylinder(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new CapsuleCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new CapsuleCylinderCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[3];

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(CapsuleWide);
            var bWide = default(CylinderWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            CapsuleCylinderTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                CapsuleCylinderScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;

                var builder = new StringBuilder();
                CompareConvex2Manifolds(wideManifold, j, scalarManifold, builder, ref contactCount, contactCountHistogram);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH capsule-cylinder case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: radius {F(lanes[j].A.Radius)}, halfLength {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b: radius {F(lanes[j].B.Radius)}, halfLength {F(lanes[j].B.HalfLength)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Capsule-cylinder fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunTriangleCylinder(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new TriangleCylinderGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new TriangleCylinderCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];
        long faceFlaggedLanes = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(TriangleWide);
            var bWide = default(CylinderWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            TriangleCylinderTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                TriangleCylinderScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount =
                    (scalarManifold.Contact0Exists ? 1 : 0) + (scalarManifold.Contact1Exists ? 1 : 0) +
                    (scalarManifold.Contact2Exists ? 1 : 0) + (scalarManifold.Contact3Exists ? 1 : 0);
                ++contactCountHistogram[laneContactCount];
                if (scalarManifold.Contact0Exists && (scalarManifold.FeatureId0 & BepuPhysics.CollisionDetection.MeshReduction.FaceCollisionFlag) != 0)
                    ++faceFlaggedLanes;

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH triangle-cylinder case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a.A {F(lanes[j].A.A)}");
                        Console.WriteLine($"    a.B {F(lanes[j].A.B)}");
                        Console.WriteLine($"    a.C {F(lanes[j].A.C)}");
                        Console.WriteLine($"    b: radius {F(lanes[j].B.Radius)}, halfLength {F(lanes[j].B.HalfLength)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Triangle-cylinder fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}; " +
            $"face-flagged lanes {(double)faceFlaggedLanes / testedLanes:P1}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunCapsulePair(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new CapsulePairGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new CapsulePairCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[3];

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(CapsuleWide);
            var bWide = default(CapsuleWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            CapsulePairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                CapsulePairScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;

                var builder = new StringBuilder();
                CompareConvex2Manifolds(wideManifold, j, scalarManifold, builder, ref contactCount, contactCountHistogram);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH capsule-capsule case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: radius {F(lanes[j].A.Radius)}, halfLength {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b: radius {F(lanes[j].B.Radius)}, halfLength {F(lanes[j].B.HalfLength)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Capsule-capsule fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunCapsuleBox(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new CapsuleBoxGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new CapsuleBoxCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[3];

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(CapsuleWide);
            var bWide = default(BoxWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            CapsuleBoxTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                CapsuleBoxScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;

                var builder = new StringBuilder();
                CompareConvex2Manifolds(wideManifold, j, scalarManifold, builder, ref contactCount, contactCountHistogram);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH capsule-box case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: radius {F(lanes[j].A.Radius)}, halfLength {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b: halfWidth {F(lanes[j].B.HalfWidth)}, halfHeight {F(lanes[j].B.HalfHeight)}, halfLength {F(lanes[j].B.HalfLength)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Capsule-box fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunCapsuleTriangle(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new CapsuleTriangleGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new CapsuleTriangleCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[3];
        long faceFlaggedLanes = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(CapsuleWide);
            var bWide = default(TriangleWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            CapsuleTriangleTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                CapsuleTriangleScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                if (scalarManifold.Contact0Exists && (scalarManifold.FeatureId0 & BepuPhysics.CollisionDetection.MeshReduction.FaceCollisionFlag) != 0)
                    ++faceFlaggedLanes;

                var builder = new StringBuilder();
                CompareConvex2Manifolds(wideManifold, j, scalarManifold, builder, ref contactCount, contactCountHistogram);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH capsule-triangle case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: radius {F(lanes[j].A.Radius)}, halfLength {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    b.A {F(lanes[j].B.A)}");
                        Console.WriteLine($"    b.B {F(lanes[j].B.B)}");
                        Console.WriteLine($"    b.C {F(lanes[j].B.C)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Capsule-triangle fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}; face-flagged lanes {(double)faceFlaggedLanes / testedLanes:P1}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunTrianglePair(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var generator = new TrianglePairGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new TrianglePairCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];
        long aVertexContacts = 0;
        long edgeEntryContacts = 0;
        long edgeExitContacts = 0;
        long faceFlaggedLanes = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(TriangleWide);
            var bWide = default(TriangleWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            TrianglePairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                TrianglePairScalarTester.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount = 0;
                //Feature id classification: strip the face collision flag (only ever added to contact 0), then
                //ids 0..2 are A-vertex contacts, 3..5 B-edge interval entries, 6..8 exits.
                void Classify(bool exists, int featureId, ref long vertexContacts, ref long entryContacts, ref long exitContacts, ref int count)
                {
                    if (!exists)
                        return;
                    ++count;
                    var id = featureId & ~BepuPhysics.CollisionDetection.MeshReduction.FaceCollisionFlag;
                    if (id < 3)
                        ++vertexContacts;
                    else if (id < 6)
                        ++entryContacts;
                    else
                        ++exitContacts;
                }
                Classify(scalarManifold.Contact0Exists, scalarManifold.FeatureId0, ref aVertexContacts, ref edgeEntryContacts, ref edgeExitContacts, ref laneContactCount);
                Classify(scalarManifold.Contact1Exists, scalarManifold.FeatureId1, ref aVertexContacts, ref edgeEntryContacts, ref edgeExitContacts, ref laneContactCount);
                Classify(scalarManifold.Contact2Exists, scalarManifold.FeatureId2, ref aVertexContacts, ref edgeEntryContacts, ref edgeExitContacts, ref laneContactCount);
                Classify(scalarManifold.Contact3Exists, scalarManifold.FeatureId3, ref aVertexContacts, ref edgeEntryContacts, ref edgeExitContacts, ref laneContactCount);
                ++contactCountHistogram[laneContactCount];
                if (scalarManifold.Contact0Exists && (scalarManifold.FeatureId0 & BepuPhysics.CollisionDetection.MeshReduction.FaceCollisionFlag) != 0)
                    ++faceFlaggedLanes;

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH triangle-triangle case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a.A {F(lanes[j].A.A)}");
                        Console.WriteLine($"    a.B {F(lanes[j].A.B)}");
                        Console.WriteLine($"    a.C {F(lanes[j].A.C)}");
                        Console.WriteLine($"    b.A {F(lanes[j].B.A)}");
                        Console.WriteLine($"    b.B {F(lanes[j].B.B)}");
                        Console.WriteLine($"    b.C {F(lanes[j].B.C)}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        var classified = Math.Max(1, aVertexContacts + edgeEntryContacts + edgeExitContacts);
        Console.WriteLine($"Triangle-triangle fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}; " +
            $"a-vertex {(double)aVertexContacts / classified:P1} / b-edge-entry {(double)edgeEntryContacts / classified:P1} / b-edge-exit {(double)edgeExitContacts / classified:P1}; " +
            $"face-flagged lanes {(double)faceFlaggedLanes / testedLanes:P1}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunHullHull(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.Create(setupRandom, hullCount: 24, minPointCount: 4, maxPointCount: 64);
        var generator = new HullHullGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new HullPairCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];

        hullSet.Pool.Take<ConvexHull>(laneCount, out var aHullBuffer);
        hullSet.Pool.Take<ConvexHull>(laneCount, out var bHullBuffer);
        var aWide = default(ConvexHullWide);
        var bWide = default(ConvexHullWide);
        aHullBuffer.Slice(0, laneCount, out aWide.Hulls);
        bHullBuffer.Slice(0, laneCount, out bWide.Hulls);

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, hullSet.Hulls[lanes[j].A]);
                bWide.WriteSlot(j, hullSet.Hulls[lanes[j].B]);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            ConvexHullPairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                ConvexHullPairScalarTester.Test(ref hullSet.Hulls[lanes[j].A], ref hullSet.Hulls[lanes[j].B],
                    lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount =
                    (scalarManifold.Contact0Exists ? 1 : 0) + (scalarManifold.Contact1Exists ? 1 : 0) +
                    (scalarManifold.Contact2Exists ? 1 : 0) + (scalarManifold.Contact3Exists ? 1 : 0);
                ++contactCountHistogram[laneContactCount];

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH hull-hull case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    hull A index {lanes[j].A}, hull B index {lanes[j].B}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Hull-hull fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    static float MaxManifoldDepth(in Convex4ManifoldScalar manifold)
    {
        var depth = float.MinValue;
        if (manifold.Contact0Exists) depth = MathF.Max(depth, manifold.Depth0);
        if (manifold.Contact1Exists) depth = MathF.Max(depth, manifold.Depth1);
        if (manifold.Contact2Exists) depth = MathF.Max(depth, manifold.Depth2);
        if (manifold.Contact3Exists) depth = MathF.Max(depth, manifold.Depth3);
        return depth;
    }

    /// <summary>
    /// FAIL-focused triage of a relaxed hull-hull candidate vs the engine wide reference: classifies every FAIL by
    /// mechanism using the comparator's own SupportDepth as the impartial axis-quality metric. For SAT candidates the
    /// decisive axis of empty-manifold rejects (SatHullPairTester.LastLocalNormal/LastDepth) is re-verified
    /// independently, turning each reject into a checkable certificate.
    /// </summary>
    public static void RunHullHullTriage(IRelaxedHullPairTester candidate, long caseCount, int seed, int targetVertexCount, bool contactHeavy, int samplesPerClass)
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, targetVertexCount);
        var topologies = HullTopology.CreateForSet(hullSet, out _);
        var generator = new HullHullGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new HullPairCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        var isSatCandidate = candidate.Name.StartsWith("sat", StringComparison.OrdinalIgnoreCase);
        var isWalkCandidate = candidate.Name.StartsWith("walk", StringComparison.OrdinalIgnoreCase);
        var isCsoCandidate = candidate.Name.StartsWith("cso", StringComparison.OrdinalIgnoreCase);

        hullSet.Pool.Take<ConvexHull>(laneCount, out var aHullBuffer);
        hullSet.Pool.Take<ConvexHull>(laneCount, out var bHullBuffer);
        var aWide = default(ConvexHullWide);
        var bWide = default(ConvexHullWide);
        aHullBuffer.Slice(0, laneCount, out aWide.Hulls);
        bHullBuffer.Slice(0, laneCount, out bWide.Hulls);

        long cases = 0, fails = 0;
        long normalCandidateBetter = 0, normalReferenceBetter = 0, normalAxisTie = 0;
        double normalAdvantageSum = 0;
        long depthOnlyFails = 0, depthCandidateCloser = 0, depthReferenceCloser = 0;
        long flipReferenceOnly = 0, flipRefConfirmedReject = 0, flipRefBoundary = 0, flipRefSuspicious = 0;
        long flipCandidateOnly = 0, flipCandOwnAxisBelowMargin = 0, flipCandBoundary = 0, flipCandAboveMargin = 0, flipCandAboveMarginRefAccepted = 0;
        Span<int> printed = stackalloc int[6];

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, hullSet.Hulls[lanes[j].A]);
                bWide.WriteSlot(j, hullSet.Hulls[lanes[j].B]);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            ConvexHullPairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, laneCount, out var wideManifold);

            for (int j = 0; j < laneCount; ++j)
            {
                ref var lane = ref lanes[j];
                candidate.Test(topologies[lane.A], topologies[lane.B], ref hullSet.Hulls[lane.A], ref hullSet.Hulls[lane.B],
                    lane.SpeculativeMargin, lane.OffsetB, lane.OrientationA, lane.OrientationB, out var candidateManifold);
                var referenceManifold = ReadWideLane(wideManifold, j);
                Matrix3x3.CreateFromQuaternion(lane.OrientationA, out Matrix3x3 rA);
                Matrix3x3.CreateFromQuaternion(lane.OrientationB, out Matrix3x3 rB);
                var scale = MathF.Min(hullSet.MaxRadii[lane.A], hullSet.MaxRadii[lane.B]);
                var metrics = RelaxedComparator.Compare(referenceManifold, candidateManifold, topologies[lane.A], topologies[lane.B],
                    rA, rB, lane.OffsetB, lane.SpeculativeMargin, scale);
                ++cases;
                if (metrics.Verdict != RelaxedVerdict.Fail)
                    continue;
                ++fails;
                var referenceHasContacts = referenceManifold.Contact0Exists || referenceManifold.Contact1Exists || referenceManifold.Contact2Exists || referenceManifold.Contact3Exists;
                var candidateHasContacts = candidateManifold.Contact0Exists || candidateManifold.Contact1Exists || candidateManifold.Contact2Exists || candidateManifold.Contact3Exists;
                var referenceDepth = MaxManifoldDepth(referenceManifold);
                var candidateDepth = MaxManifoldDepth(candidateManifold);
                var tieTolerance = MathF.Max(RelaxedComparator.TieDepthAbsoluteTolerance * scale, RelaxedComparator.TieDepthRelativeTolerance * MathF.Abs(referenceDepth));
                var boundaryTolerance = 1e-3f * scale;
                var margin = lane.SpeculativeMargin;
                if (referenceHasContacts && candidateHasContacts)
                {
                    var sdCandidate = RelaxedComparator.SupportDepth(topologies[lane.A], topologies[lane.B], rA, rB, lane.OffsetB, candidateManifold.Normal);
                    var sdReference = RelaxedComparator.SupportDepth(topologies[lane.A], topologies[lane.B], rA, rB, lane.OffsetB, referenceManifold.Normal);
                    if (metrics.NormalAngle > RelaxedComparator.FailNormalAngle)
                    {
                        normalAdvantageSum += (sdReference - sdCandidate) / scale;
                        if (sdCandidate < sdReference - tieTolerance)
                        {
                            ++normalCandidateBetter;
                            if (printed[0]++ < samplesPerClass)
                                Console.WriteLine($"    [normal, candidate better] case {i + j}: angle {metrics.NormalAngle:E2}, axis true depths cand {sdCandidate:G6} vs ref {sdReference:G6} (reported {candidateDepth:G6}/{referenceDepth:G6}), scale {scale:G4}");
                        }
                        else if (sdCandidate > sdReference + tieTolerance)
                        {
                            ++normalReferenceBetter;
                            if (printed[1]++ < samplesPerClass)
                                Console.WriteLine($"    [normal, reference better] case {i + j}: angle {metrics.NormalAngle:E2}, axis true depths cand {sdCandidate:G6} vs ref {sdReference:G6} (reported {candidateDepth:G6}/{referenceDepth:G6}), scale {scale:G4}");
                        }
                        else
                            ++normalAxisTie;
                    }
                    else
                    {
                        ++depthOnlyFails;
                        if (MathF.Abs(candidateDepth - sdCandidate) <= MathF.Abs(referenceDepth - sdReference))
                            ++depthCandidateCloser;
                        else
                            ++depthReferenceCloser;
                        if (printed[2]++ < samplesPerClass)
                            Console.WriteLine($"    [depth-only] case {i + j}: reported cand {candidateDepth:G6} vs ref {referenceDepth:G6}, own-axis true depths {sdCandidate:G6}/{sdReference:G6}, angle {metrics.NormalAngle:E2}, scale {scale:G4}");
                    }
                }
                else if (referenceHasContacts)
                {
                    ++flipReferenceOnly;
                    if (isSatCandidate || isWalkCandidate || isCsoCandidate)
                    {
                        var lastLocalNormal = isSatCandidate ? SatHullPairTester.LastLocalNormal : isCsoCandidate ? CsoWalkHullPairTester.LastLocalNormal : WalkHullPairTester.LastLocalNormal;
                        var lastDepth = isSatCandidate ? SatHullPairTester.LastDepth : isCsoCandidate ? CsoWalkHullPairTester.LastDepth : WalkHullPairTester.LastDepth;
                        var lastWinnerType = isSatCandidate ? SatHullPairTester.LastWinnerType : isCsoCandidate ? CsoWalkHullPairTester.LastWinnerType : WalkHullPairTester.LastWinnerType;
                        var lastClipCandidates = isSatCandidate ? SatHullPairTester.LastClipCandidateCount : isCsoCandidate ? CsoWalkHullPairTester.LastClipCandidateCount : WalkHullPairTester.LastClipCandidateCount;
                        Matrix3x3.Transform(lastLocalNormal, rB, out var worldAxis);
                        var sdAxis = RelaxedComparator.SupportDepth(topologies[lane.A], topologies[lane.B], rA, rB, lane.OffsetB, worldAxis);
                        if (sdAxis < -margin - boundaryTolerance)
                            ++flipRefConfirmedReject;
                        else if (sdAxis <= -margin + boundaryTolerance)
                            ++flipRefBoundary;
                        else
                        {
                            ++flipRefSuspicious;
                            if (printed[3]++ < samplesPerClass)
                                Console.WriteLine($"    [flip ref-only SUSPICIOUS] case {i + j}: candidate empty but its axis's true depth {sdAxis:G6} > -margin {-margin:G6} (claimed {lastDepth:G6}, winnerType {lastWinnerType}, clipCandidates {lastClipCandidates}); ref reported depth {referenceDepth:G6}, ref contacts {(referenceManifold.Contact0Exists ? 1 : 0) + (referenceManifold.Contact1Exists ? 1 : 0) + (referenceManifold.Contact2Exists ? 1 : 0) + (referenceManifold.Contact3Exists ? 1 : 0)}, scale {scale:G4}");
                        }
                    }
                }
                else
                {
                    ++flipCandidateOnly;
                    var sdCandidate = RelaxedComparator.SupportDepth(topologies[lane.A], topologies[lane.B], rA, rB, lane.OffsetB, candidateManifold.Normal);
                    if (sdCandidate < -margin - boundaryTolerance)
                    {
                        ++flipCandOwnAxisBelowMargin;
                        if (printed[4]++ < samplesPerClass)
                            Console.WriteLine($"    [flip cand-only, own axis below margin] case {i + j}: candidate accepted but its axis depth {sdCandidate:G6} < -margin {-margin:G6}, scale {scale:G4}");
                    }
                    else if (sdCandidate <= -margin + boundaryTolerance)
                        ++flipCandBoundary;
                    else
                    {
                        //Ground truth probe: run the frozen-configuration scalar refiner (bitwise-equal to the wide
                        //reference's refiner) to learn whether the reference actually rejected here, or accepted and
                        //then emptied its own clip (a reference-side degeneracy).
                        ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);
                        var localOffsetA = -ScalarMath.TransformByTransposed(lane.OffsetB, rB);
                        var centerDistance = ScalarMath.Length(localOffsetA);
                        var initialNormal = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
                        var refinerEpsilonScale = MathF.Min(
                            (MathF.Abs(topologies[lane.A].Vertices[0].X) + MathF.Abs(topologies[lane.A].Vertices[0].Y) + MathF.Abs(topologies[lane.A].Vertices[0].Z)) / 3f,
                            (MathF.Abs(topologies[lane.B].Vertices[0].X) + MathF.Abs(topologies[lane.B].Vertices[0].Y) + MathF.Abs(topologies[lane.B].Vertices[0].Z)) / 3f);
                        ScalarDepthRefiner<ConvexHull, HullSupportScalar, ConvexHull, HullSupportScalar>.FindMinimumDepth(
                            hullSet.Hulls[lane.B], hullSet.Hulls[lane.A], localOffsetA, bLocalOrientationA, initialNormal,
                            1e-5f * refinerEpsilonScale, -margin, out var refinerDepth, out _, out _);
                        if (refinerDepth >= -margin)
                        {
                            ++flipCandAboveMarginRefAccepted;
                            if (printed[5]++ < samplesPerClass)
                                Console.WriteLine($"    [flip cand-only, reference clip degeneracy] case {i + j}: reference refiner accepted (depth {refinerDepth:G6} >= -margin {-margin:G6}) but its manifold is empty; candidate axis depth {sdCandidate:G6}, scale {scale:G4}");
                        }
                        else
                        {
                            ++flipCandAboveMargin;
                            if (printed[5]++ < samplesPerClass)
                                Console.WriteLine($"    [flip cand-only, refiner rejects] case {i + j}: candidate axis depth {sdCandidate:G6} vs -margin {-margin:G6}, refiner ground-truth depth {refinerDepth:G6} (candidate missed a separating axis or boundary disagreement), scale {scale:G4}");
                        }
                    }
                }
            }
        }
        Console.WriteLine($"Triage '{candidate.Name}' size {targetVertexCount}{(contactHeavy ? " contact-heavy" : " mixed")} (seed {seed}): {cases} cases, {fails} FAILs ({(double)fails / Math.Max(1, cases):P3})");
        Console.WriteLine($"    normal-angle FAILs: candidate-better {normalCandidateBetter}, reference-better {normalReferenceBetter}, axis-tie {normalAxisTie}; mean (sdRef-sdCand)/scale over class {(normalCandidateBetter + normalReferenceBetter + normalAxisTie > 0 ? normalAdvantageSum / (normalCandidateBetter + normalReferenceBetter + normalAxisTie) : 0):E2}");
        Console.WriteLine($"    depth-only FAILs: {depthOnlyFails} (candidate reported depth closer to its own axis's true depth: {depthCandidateCloser}, reference closer: {depthReferenceCloser})");
        Console.WriteLine($"    existence flips, reference-only: {flipReferenceOnly} (certificate-confirmed rejects {flipRefConfirmedReject}, boundary {flipRefBoundary}, SUSPICIOUS {flipRefSuspicious})");
        Console.WriteLine($"    existence flips, candidate-only: {flipCandidateOnly} (own axis below margin {flipCandOwnAxisBelowMargin}, boundary {flipCandBoundary}, reference-clip-degeneracy {flipCandAboveMarginRefAccepted}, refiner-rejects {flipCandAboveMargin})");
        hullSet.Pool.Return(ref aHullBuffer);
        hullSet.Pool.Return(ref bHullBuffer);
    }

    static Convex4ManifoldScalar ReadWideLane(in Convex4ContactManifoldWide wide, int lane)
    {
        Convex4ManifoldScalar manifold = default;
        manifold.Contact0Exists = wide.Contact0Exists[lane] < 0;
        manifold.Contact1Exists = wide.Contact1Exists[lane] < 0;
        manifold.Contact2Exists = wide.Contact2Exists[lane] < 0;
        manifold.Contact3Exists = wide.Contact3Exists[lane] < 0;
        if (manifold.Contact0Exists || manifold.Contact1Exists || manifold.Contact2Exists || manifold.Contact3Exists)
            manifold.Normal = ReadLane(wide.Normal, lane);
        if (manifold.Contact0Exists)
        {
            manifold.OffsetA0 = ReadLane(wide.OffsetA0, lane);
            manifold.Depth0 = wide.Depth0[lane];
            manifold.FeatureId0 = wide.FeatureId0[lane];
        }
        if (manifold.Contact1Exists)
        {
            manifold.OffsetA1 = ReadLane(wide.OffsetA1, lane);
            manifold.Depth1 = wide.Depth1[lane];
            manifold.FeatureId1 = wide.FeatureId1[lane];
        }
        if (manifold.Contact2Exists)
        {
            manifold.OffsetA2 = ReadLane(wide.OffsetA2, lane);
            manifold.Depth2 = wide.Depth2[lane];
            manifold.FeatureId2 = wide.FeatureId2[lane];
        }
        if (manifold.Contact3Exists)
        {
            manifold.OffsetA3 = ReadLane(wide.OffsetA3, lane);
            manifold.Depth3 = wide.Depth3[lane];
            manifold.FeatureId3 = wide.FeatureId3[lane];
        }
        return manifold;
    }

    /// <summary>
    /// Runs a relaxed hull-hull candidate against the engine wide tester through the tolerance comparator
    /// (RelaxedComparator) on a size-targeted hull set. Returns the number of FAIL verdicts.
    /// </summary>
    public static int RunHullHullRelaxed(IRelaxedHullPairTester candidate, long caseCount, int seed, int targetVertexCount, bool contactHeavy, int maxReports)
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, targetVertexCount);
        var topologies = HullTopology.CreateForSet(hullSet, out var precomputeMilliseconds);
        HullTopology.ReportSetStatistics($"    hull set (target {targetVertexCount} verts, seed {seed})", topologies, precomputeMilliseconds);
        var generator = new HullHullGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new HullPairCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        var stats = new RelaxedComparisonStats();
        long failReports = 0;

        hullSet.Pool.Take<ConvexHull>(laneCount, out var aHullBuffer);
        hullSet.Pool.Take<ConvexHull>(laneCount, out var bHullBuffer);
        var aWide = default(ConvexHullWide);
        var bWide = default(ConvexHullWide);
        aHullBuffer.Slice(0, laneCount, out aWide.Hulls);
        bHullBuffer.Slice(0, laneCount, out bWide.Hulls);

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, hullSet.Hulls[lanes[j].A]);
                bWide.WriteSlot(j, hullSet.Hulls[lanes[j].B]);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            ConvexHullPairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                ref var lane = ref lanes[j];
                candidate.Test(topologies[lane.A], topologies[lane.B], ref hullSet.Hulls[lane.A], ref hullSet.Hulls[lane.B],
                    lane.SpeculativeMargin, lane.OffsetB, lane.OrientationA, lane.OrientationB, out var candidateManifold);
                var referenceManifold = ReadWideLane(wideManifold, j);
                Matrix3x3.CreateFromQuaternion(lane.OrientationA, out Matrix3x3 rA);
                Matrix3x3.CreateFromQuaternion(lane.OrientationB, out Matrix3x3 rB);
                var scale = MathF.Min(hullSet.MaxRadii[lane.A], hullSet.MaxRadii[lane.B]);
                var metrics = RelaxedComparator.Compare(referenceManifold, candidateManifold, topologies[lane.A], topologies[lane.B],
                    rA, rB, lane.OffsetB, lane.SpeculativeMargin, scale);
                var referenceHasContacts = referenceManifold.Contact0Exists || referenceManifold.Contact1Exists || referenceManifold.Contact2Exists || referenceManifold.Contact3Exists;
                var candidateHasContacts = candidateManifold.Contact0Exists || candidateManifold.Contact1Exists || candidateManifold.Contact2Exists || candidateManifold.Contact3Exists;
                stats.Add(metrics, referenceHasContacts && candidateHasContacts);
                if (metrics.Verdict == RelaxedVerdict.Fail && failReports++ < maxReports)
                {
                    Console.WriteLine($"FAIL {candidate.Name} hull-hull case {i + j} (lane {j}/{pairCount}, seed {seed}, size {targetVertexCount}{(contactHeavy ? ", contact-heavy" : ", mixed")}): {metrics.Reason}");
                    Console.WriteLine($"    hull A index {lane.A} ({topologies[lane.A].VertexCount} verts), hull B index {lane.B} ({topologies[lane.B].VertexCount} verts), scale {F(scale)}");
                    Console.WriteLine($"    margin {F(lane.SpeculativeMargin)}");
                    Console.WriteLine($"    offsetB {F(lane.OffsetB)}");
                    Console.WriteLine($"    orientationA {F(lane.OrientationA)}");
                    Console.WriteLine($"    orientationB {F(lane.OrientationB)}");
                    Console.WriteLine($"    reference normal {F(referenceManifold.Normal)} depth {F(referenceManifold.Depth0)}, candidate normal {F(candidateManifold.Normal)} depth {F(candidateManifold.Depth0)}");
                }
            }
        }
        stats.Print($"Relaxed hull-hull '{candidate.Name}' vs wide, target {targetVertexCount} verts{(contactHeavy ? ", contact-heavy" : ", mixed")} (seed {seed})");
        return (int)Math.Min(int.MaxValue, stats.Fail);
    }

    static void CompareConvex2Manifolds(in Convex2ContactManifoldWide wide, int lane, in Convex2ManifoldScalar scalar, StringBuilder builder, ref long contactCount, Span<long> contactCountHistogram)
    {
        Span<bool> wideExists = [wide.Contact0Exists[lane] < 0, wide.Contact1Exists[lane] < 0];
        Span<bool> scalarExists = [scalar.Contact0Exists, scalar.Contact1Exists];
        bool anyExists = false;
        for (int i = 0; i < 2; ++i)
        {
            if (wideExists[i] != scalarExists[i])
                builder.AppendLine($"    Contact{i}Exists: wide {wideExists[i]} vs scalar {scalarExists[i]}");
            anyExists |= wideExists[i];
        }
        ++contactCountHistogram[(scalarExists[0] ? 1 : 0) + (scalarExists[1] ? 1 : 0)];
        if (scalarExists[0] || scalarExists[1])
            ++contactCount;

        if (builder.Length > 0)
            return;
        if (!anyExists)
            return;

        AppendFieldComparison(builder, "Normal", ReadLane(wide.Normal, lane), scalar.Normal);
        if (wideExists[0])
        {
            AppendFieldComparison(builder, "OffsetA0", ReadLane(wide.OffsetA0, lane), scalar.OffsetA0);
            AppendFieldComparison(builder, "Depth0", wide.Depth0[lane], scalar.Depth0);
            if (wide.FeatureId0[lane] != scalar.FeatureId0)
                builder.AppendLine($"    FeatureId0: wide {wide.FeatureId0[lane]} vs scalar {scalar.FeatureId0}");
        }
        if (wideExists[1])
        {
            AppendFieldComparison(builder, "OffsetA1", ReadLane(wide.OffsetA1, lane), scalar.OffsetA1);
            AppendFieldComparison(builder, "Depth1", wide.Depth1[lane], scalar.Depth1);
            if (wide.FeatureId1[lane] != scalar.FeatureId1)
                builder.AppendLine($"    FeatureId1: wide {wide.FeatureId1[lane]} vs scalar {scalar.FeatureId1}");
        }
    }

    public static int RunCapsuleHull(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.Create(setupRandom, hullCount: 24, minPointCount: 4, maxPointCount: 64);
        var generator = new CapsuleHullGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new ShapeHullCase<Capsule>[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[3];

        hullSet.Pool.Take<ConvexHull>(laneCount, out var bHullBuffer);
        var bWide = default(ConvexHullWide);
        bHullBuffer.Slice(0, laneCount, out bWide.Hulls);

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(CapsuleWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, hullSet.Hulls[lanes[j].B]);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            CapsuleConvexHullTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                CapsuleConvexHullScalarTester.Test(lanes[j].A, ref hullSet.Hulls[lanes[j].B],
                    lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;

                var builder = new StringBuilder();
                CompareConvex2Manifolds(wideManifold, j, scalarManifold, builder, ref contactCount, contactCountHistogram);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH capsule-hull case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: radius {F(lanes[j].A.Radius)}, halfLength {F(lanes[j].A.HalfLength)}");
                        Console.WriteLine($"    hull index {lanes[j].B}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"Capsule-hull fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    /// <summary>
    /// Generic equivalence runner for the (convex shape)-hull testers with Convex4 manifolds.
    /// </summary>
    public static int RunShapeHull<TShapeA, TShapeWideA, TTester, TScalarTester>(
        string pairName, long caseCount, int seed, int maxReports, bool contactHeavy,
        Func<int, HullSet, bool, ShapeHullGeneratorBase<TShapeA>> createGenerator, Func<TShapeA, string> describeA)
        where TShapeA : unmanaged, IShape
        where TShapeWideA : unmanaged, IShapeWide<TShapeA>
        where TTester : IPairTester<TShapeWideA, ConvexHullWide, Convex4ContactManifoldWide>
        where TScalarTester : IShapeHullScalarTester<TShapeA>
    {
        var setupRandom = new Random(seed);
        using var hullSet = HullSet.Create(setupRandom, hullCount: 24, minPointCount: 4, maxPointCount: 64);
        var generator = createGenerator(seed + 1, hullSet, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new ShapeHullCase<TShapeA>[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long mismatches = 0;
        long contactCount = 0;
        long testedLanes = 0;
        Span<long> contactCountHistogram = stackalloc long[5];

        hullSet.Pool.Take<ConvexHull>(laneCount, out var bHullBuffer);
        var bWide = default(ConvexHullWide);
        bHullBuffer.Slice(0, laneCount, out bWide.Hulls);

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(TShapeWideA);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            int pairCount = generator.Random.Next(8) == 0 ? 1 + generator.Random.Next(laneCount) : laneCount;
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, hullSet.Hulls[lanes[j].B]);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            TTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, pairCount, out var wideManifold);

            for (int j = 0; j < pairCount; ++j)
            {
                TScalarTester.Test(lanes[j].A, ref hullSet.Hulls[lanes[j].B],
                    lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var scalarManifold);
                ++testedLanes;
                int laneContactCount =
                    (scalarManifold.Contact0Exists ? 1 : 0) + (scalarManifold.Contact1Exists ? 1 : 0) +
                    (scalarManifold.Contact2Exists ? 1 : 0) + (scalarManifold.Contact3Exists ? 1 : 0);
                ++contactCountHistogram[laneContactCount];

                var builder = new StringBuilder();
                CompareBoxManifolds(wideManifold, j, scalarManifold, builder, ref contactCount);
                if (builder.Length > 0)
                {
                    ++mismatches;
                    if (mismatches <= maxReports)
                    {
                        Console.WriteLine($"MISMATCH {pairName} case {i + j} (lane {j}/{pairCount}):");
                        Console.WriteLine($"    a: {describeA(lanes[j].A)}");
                        Console.WriteLine($"    hull index {lanes[j].B}");
                        Console.WriteLine($"    margin {F(lanes[j].SpeculativeMargin)}");
                        Console.WriteLine($"    offsetB {F(lanes[j].OffsetB)}");
                        Console.WriteLine($"    orientationA {F(lanes[j].OrientationA)}");
                        Console.WriteLine($"    orientationB {F(lanes[j].OrientationB)}");
                        Console.Write(builder.ToString());
                    }
                }
            }
        }
        Console.WriteLine($"{char.ToUpperInvariant(pairName[0])}{pairName.Substring(1)} fuzz{(contactHeavy ? " (contact-heavy)" : "")}: {testedLanes} lanes tested (seed {seed}), {(double)contactCount / testedLanes:P1} with at least one contact, {mismatches} mismatches.");
        Console.WriteLine($"    contact count distribution 0/1/2/3/4: {string.Join("/", contactCountHistogram.ToArray().Select(c => $"{(double)c / testedLanes:P1}"))}");
        return (int)Math.Min(int.MaxValue, mismatches);
    }

    public static int RunBoxHull(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunShapeHull<Box, BoxWide, BoxConvexHullTester, BoxConvexHullScalarTester>(
            "box-hull", caseCount, seed, maxReports, contactHeavy,
            (s, set, ch) => new BoxHullGenerator(s, set, ch),
            a => $"halfWidth {F(a.HalfWidth)}, halfHeight {F(a.HalfHeight)}, halfLength {F(a.HalfLength)}");
    }

    public static int RunCylinderHull(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunShapeHull<Cylinder, CylinderWide, CylinderConvexHullTester, CylinderConvexHullScalarTester>(
            "cylinder-hull", caseCount, seed, maxReports, contactHeavy,
            (s, set, ch) => new CylinderHullGenerator(s, set, ch),
            a => $"radius {F(a.Radius)}, halfLength {F(a.HalfLength)}");
    }

    public static int RunTriangleHull(long caseCount, int seed, int maxReports, bool contactHeavy = false)
    {
        return RunShapeHull<Triangle, TriangleWide, TriangleConvexHullTester, TriangleConvexHullScalarTester>(
            "triangle-hull", caseCount, seed, maxReports, contactHeavy,
            (s, set, ch) => new TriangleHullGenerator(s, set, ch),
            a => $"A {F(a.A)}, B {F(a.B)}, C {F(a.C)}");
    }

    static void CompareBoxManifolds(in Convex4ContactManifoldWide wide, int lane, in Convex4ManifoldScalar scalar, StringBuilder builder, ref long contactCount)
    {
        Span<bool> wideExists = [wide.Contact0Exists[lane] < 0, wide.Contact1Exists[lane] < 0, wide.Contact2Exists[lane] < 0, wide.Contact3Exists[lane] < 0];
        Span<bool> scalarExists = [scalar.Contact0Exists, scalar.Contact1Exists, scalar.Contact2Exists, scalar.Contact3Exists];
        bool anyExists = false;
        for (int i = 0; i < 4; ++i)
        {
            if (wideExists[i] != scalarExists[i])
                builder.AppendLine($"    Contact{i}Exists: wide {wideExists[i]} vs scalar {scalarExists[i]}");
            anyExists |= wideExists[i];
        }
        if (scalarExists[0] || scalarExists[1] || scalarExists[2] || scalarExists[3])
            ++contactCount;

        if (builder.Length > 0)
            return;
        if (!anyExists)
            return;

        AppendFieldComparison(builder, "Normal", ReadLane(wide.Normal, lane), scalar.Normal);
        if (wideExists[0])
        {
            AppendFieldComparison(builder, "OffsetA0", ReadLane(wide.OffsetA0, lane), scalar.OffsetA0);
            AppendFieldComparison(builder, "Depth0", wide.Depth0[lane], scalar.Depth0);
            if (wide.FeatureId0[lane] != scalar.FeatureId0)
                builder.AppendLine($"    FeatureId0: wide {wide.FeatureId0[lane]} vs scalar {scalar.FeatureId0}");
        }
        if (wideExists[1])
        {
            AppendFieldComparison(builder, "OffsetA1", ReadLane(wide.OffsetA1, lane), scalar.OffsetA1);
            AppendFieldComparison(builder, "Depth1", wide.Depth1[lane], scalar.Depth1);
            if (wide.FeatureId1[lane] != scalar.FeatureId1)
                builder.AppendLine($"    FeatureId1: wide {wide.FeatureId1[lane]} vs scalar {scalar.FeatureId1}");
        }
        if (wideExists[2])
        {
            AppendFieldComparison(builder, "OffsetA2", ReadLane(wide.OffsetA2, lane), scalar.OffsetA2);
            AppendFieldComparison(builder, "Depth2", wide.Depth2[lane], scalar.Depth2);
            if (wide.FeatureId2[lane] != scalar.FeatureId2)
                builder.AppendLine($"    FeatureId2: wide {wide.FeatureId2[lane]} vs scalar {scalar.FeatureId2}");
        }
        if (wideExists[3])
        {
            AppendFieldComparison(builder, "OffsetA3", ReadLane(wide.OffsetA3, lane), scalar.OffsetA3);
            AppendFieldComparison(builder, "Depth3", wide.Depth3[lane], scalar.Depth3);
            if (wide.FeatureId3[lane] != scalar.FeatureId3)
                builder.AppendLine($"    FeatureId3: wide {wide.FeatureId3[lane]} vs scalar {scalar.FeatureId3}");
        }
    }
    static long UlpDistance(float a, float b)
    {
        if (a == b)
            return 0;
        if (float.IsNaN(a) || float.IsNaN(b))
            return long.MaxValue;
        //Map float bits to a monotonic integer line so the difference counts representable values between a and b.
        long ia = BitConverter.SingleToInt32Bits(a);
        long ib = BitConverter.SingleToInt32Bits(b);
        ia = ia >= 0 ? ia : int.MinValue - ia;
        ib = ib >= 0 ? ib : int.MinValue - ib;
        return Math.Abs(ia - ib);
    }

    struct ErrorStat
    {
        public double MaxAbs, SumAbs;
        public long MaxUlp, Samples;
        public long Within2, Within16, Within1024, Within1M;
        public void Add(float reference, float value)
        {
            var abs = Math.Abs((double)reference - value);
            if (abs > MaxAbs) MaxAbs = abs;
            SumAbs += abs;
            var ulp = UlpDistance(reference, value);
            if (ulp > MaxUlp) MaxUlp = ulp;
            if (ulp <= 2) ++Within2;
            if (ulp <= 16) ++Within16;
            if (ulp <= 1024) ++Within1024;
            if (ulp <= 1_000_000) ++Within1M;
            ++Samples;
        }
        public override string ToString() => Samples == 0 ? "no samples" :
            $"max {MaxAbs:E2} ({MaxUlp} ulp), mean {SumAbs / Samples:E2}; ulp buckets: <=2: {(double)Within2 / Samples:P3}, <=16: {(double)Within16 / Samples:P3}, <=1024: {(double)Within1024 / Samples:P3}, <=1M: {(double)Within1M / Samples:P3}, >1M: {(double)(Samples - Within1M) / Samples:P3} (n={Samples})";
    }

    /// <summary>
    /// Characterizes the FMA scalar variant against the wide reference with tolerance-based comparison: structural divergence
    /// (accept/reject flips, contact pattern changes, feature id changes from selection tie flips) is counted and categorized;
    /// numerical drift on structurally-matching lanes is reported as absolute and ulp error statistics.
    /// </summary>
    public static void RunBoxBoxFma(long caseCount, int seed, bool contactHeavy = false, int maxReports = 3)
    {
        var generator = new BoxBoxGenerator(seed, contactHeavy);
        int laneCount = Vector<float>.Count;
        var lanes = new BoxBoxCase[laneCount];
        Span<float> margins = stackalloc float[laneCount];
        long testedLanes = 0, structuralMatches = 0, acceptFlips = 0, patternDiffs = 0, featureDiffs = 0;
        long lanesWithContact = 0;
        var normalStat = default(ErrorStat);
        var depthStat = default(ErrorStat);
        var offsetStat = default(ErrorStat);
        long reported = 0;

        for (long i = 0; i < caseCount; i += laneCount)
        {
            var aWide = default(BoxWide);
            var bWide = default(BoxWide);
            var offsetB = default(Vector3Wide);
            var orientationA = default(QuaternionWide);
            var orientationB = default(QuaternionWide);
            for (int j = 0; j < laneCount; ++j)
            {
                lanes[j] = generator.Next();
                aWide.WriteSlot(j, lanes[j].A);
                bWide.WriteSlot(j, lanes[j].B);
                Vector3Wide.WriteSlot(lanes[j].OffsetB, j, ref offsetB);
                QuaternionWide.WriteSlot(lanes[j].OrientationA, j, ref orientationA);
                QuaternionWide.WriteSlot(lanes[j].OrientationB, j, ref orientationB);
                margins[j] = lanes[j].SpeculativeMargin;
            }
            var speculativeMargin = new Vector<float>(margins);
            BoxPairTester.Test(ref aWide, ref bWide, ref speculativeMargin, ref offsetB, ref orientationA, ref orientationB, laneCount, out var wideManifold);

            for (int j = 0; j < laneCount; ++j)
            {
                BoxPairScalarTesterFma.Test(lanes[j].A, lanes[j].B, lanes[j].SpeculativeMargin, lanes[j].OffsetB, lanes[j].OrientationA, lanes[j].OrientationB, out var fma);
                ++testedLanes;
                Span<bool> wideExists = [wideManifold.Contact0Exists[j] < 0, wideManifold.Contact1Exists[j] < 0, wideManifold.Contact2Exists[j] < 0, wideManifold.Contact3Exists[j] < 0];
                Span<bool> fmaExists = [fma.Contact0Exists, fma.Contact1Exists, fma.Contact2Exists, fma.Contact3Exists];
                Span<int> wideIds = [wideManifold.FeatureId0[j], wideManifold.FeatureId1[j], wideManifold.FeatureId2[j], wideManifold.FeatureId3[j]];
                Span<int> fmaIds = [fma.FeatureId0, fma.FeatureId1, fma.FeatureId2, fma.FeatureId3];
                bool wideAny = wideExists[0] | wideExists[1] | wideExists[2] | wideExists[3];
                bool fmaAny = fmaExists[0] | fmaExists[1] | fmaExists[2] | fmaExists[3];
                if (wideAny)
                    ++lanesWithContact;
                if (wideAny != fmaAny)
                {
                    ++acceptFlips;
                    if (reported++ < maxReports)
                        Console.WriteLine($"    accept/reject flip at case {i + j}: wide any={wideAny}, fma any={fmaAny} (margin {lanes[j].SpeculativeMargin:E2})");
                    continue;
                }
                bool samePattern = true;
                for (int c = 0; c < 4; ++c)
                    samePattern &= wideExists[c] == fmaExists[c];
                if (!samePattern)
                {
                    ++patternDiffs;
                    continue;
                }
                bool sameFeatures = true;
                for (int c = 0; c < 4; ++c)
                    if (wideExists[c])
                        sameFeatures &= wideIds[c] == fmaIds[c];
                if (!sameFeatures)
                {
                    ++featureDiffs;
                    continue;
                }
                ++structuralMatches;
                if (!wideAny)
                    continue;
                var wideNormal = ReadLane(wideManifold.Normal, j);
                normalStat.Add(wideNormal.X, fma.Normal.X);
                normalStat.Add(wideNormal.Y, fma.Normal.Y);
                normalStat.Add(wideNormal.Z, fma.Normal.Z);
                void AddContact(bool exists, in Vector3Wide wideOffset, Vector<float> wideDepth, in Vector3 fmaOffset, float fmaDepth, int lane)
                {
                    if (!exists)
                        return;
                    var wo = ReadLane(wideOffset, lane);
                    offsetStat.Add(wo.X, fmaOffset.X);
                    offsetStat.Add(wo.Y, fmaOffset.Y);
                    offsetStat.Add(wo.Z, fmaOffset.Z);
                    depthStat.Add(wideDepth[lane], fmaDepth);
                }
                AddContact(wideExists[0], wideManifold.OffsetA0, wideManifold.Depth0, fma.OffsetA0, fma.Depth0, j);
                AddContact(wideExists[1], wideManifold.OffsetA1, wideManifold.Depth1, fma.OffsetA1, fma.Depth1, j);
                AddContact(wideExists[2], wideManifold.OffsetA2, wideManifold.Depth2, fma.OffsetA2, fma.Depth2, j);
                AddContact(wideExists[3], wideManifold.OffsetA3, wideManifold.Depth3, fma.OffsetA3, fma.Depth3, j);
            }
        }
        Console.WriteLine($"Box-box FMA characterization (seed {seed}, {(contactHeavy ? "contact-heavy" : "mixed")}): {testedLanes} lanes, {(double)lanesWithContact / testedLanes:P1} with contact.");
        Console.WriteLine($"    structural match {(double)structuralMatches / testedLanes:P4}; accept/reject flips {acceptFlips} ({(double)acceptFlips / testedLanes:P4}), " +
            $"contact pattern diffs {patternDiffs} ({(double)patternDiffs / testedLanes:P4}), feature id diffs {featureDiffs} ({(double)featureDiffs / testedLanes:P4})");
        Console.WriteLine($"    normal error: {normalStat}");
        Console.WriteLine($"    depth  error: {depthStat}");
        Console.WriteLine($"    offset error: {offsetStat}");
    }

}
