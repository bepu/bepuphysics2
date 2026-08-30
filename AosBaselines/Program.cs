using System.Numerics;

namespace AosBaselines;

/// <summary>
/// Harness for comparing the wide (AoSoA) collision testers against scalar (AoS) reference implementations:
/// fuzzes for bitwise equivalence and measures relative throughput.
/// Usage:
///   AosBaselines [fuzz|bench|all] [--count N] [--seed S]
/// </summary>
public static class Program
{
    public static int Main(string[] args)
    {
        var mode = args.Length > 0 && !args[0].StartsWith("--") ? args[0] : "all";
        long count = 4_000_000;
        bool countExplicit = false;
        int seed = 5;
        int size = 32;
        int[] sizes = [8, 16, 32, 64, 128];
        bool sizesExplicit = false;
        string candidateName = "frozen";
        string[] candidateNames = [];
        int maxFailReports = 10;
        for (int i = 0; i < args.Length - 1; ++i)
        {
            if (args[i] == "--count")
            {
                count = long.Parse(args[i + 1]);
                countExplicit = true;
            }
            if (args[i] == "--seed")
                seed = int.Parse(args[i + 1]);
            if (args[i] == "--cooldown")
                ThroughputRunner.InterTrialCooldownMilliseconds = int.Parse(args[i + 1]);
            if (args[i] == "--size")
                size = int.Parse(args[i + 1]);
            if (args[i] == "--sizes")
            {
                sizes = args[i + 1].Split(',').Select(int.Parse).ToArray();
                sizesExplicit = true;
            }
            if (args[i] == "--candidate")
                candidateName = args[i + 1];
            if (args[i] == "--candidates")
                candidateNames = args[i + 1].Split(',', StringSplitOptions.RemoveEmptyEntries);
            if (args[i] == "--reports")
                maxFailReports = int.Parse(args[i + 1]);
        }

        Console.WriteLine($"Vector<float>.Count = {Vector<float>.Count}, Vector.IsHardwareAccelerated = {Vector.IsHardwareAccelerated}");

        int mismatches = 0;
        if (mode is "fuzz" or "all")
        {
            Console.WriteLine($"Fuzzing {count} cases per pair type, seed {seed}...");
            mismatches += EquivalenceRunner.RunSphereSphere(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxBox(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxBox(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunBoxTriangle(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxTriangle(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunSphereCapsule(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunSphereBox(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunSphereBoxPacked(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunSphereBoxPacked(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunSphereTriangle(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunSphereCylinder(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunSphereHull(count / 4, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunSphereCapsule(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunSphereBox(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunSphereTriangle(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunSphereCylinder(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunSphereHull(count / 4, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunCylinderCylinder(count / 2, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCylinderCylinder(count / 2, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunBoxCylinder(count / 2, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxCylinder(count / 2, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunCapsuleCylinder(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleCylinder(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunCapsulePair(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsulePair(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunCapsuleBox(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleBox(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunCapsuleTriangle(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleTriangle(count, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunTriangleCylinder(count / 2, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunTriangleCylinder(count / 2, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunTrianglePair(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunTrianglePair(count, seed + 1, maxReports: 10, contactHeavy: true);
            //Hull-hull cases are much more expensive per case; scale the count down.
            mismatches += EquivalenceRunner.RunHullHull(count / 4, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunHullHull(count / 4, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunCapsuleHull(count / 4, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleHull(count / 4, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunBoxHull(count / 4, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxHull(count / 4, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunCylinderHull(count / 4, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCylinderHull(count / 4, seed + 1, maxReports: 10, contactHeavy: true);
            mismatches += EquivalenceRunner.RunTriangleHull(count / 4, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunTriangleHull(count / 4, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "fuzzboxhull")
        {
            Console.WriteLine($"Fuzzing box-hull, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunBoxHull(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxHull(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "fuzzcylinderhull")
        {
            Console.WriteLine($"Fuzzing cylinder-hull, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunCylinderHull(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCylinderHull(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "fuzztrianglehull")
        {
            Console.WriteLine($"Fuzzing triangle-hull, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunTriangleHull(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunTriangleHull(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "minmaxprobe")
        {
            string B(float v) => $"0x{BitConverter.SingleToUInt32Bits(v):X8}";
            var nz = -0f;
            var pz = 0f;
            Console.WriteLine($"Vector.Max(+0,-0) = {B(Vector.Max(new Vector<float>(pz), new Vector<float>(nz))[0])}");
            Console.WriteLine($"Vector.Max(-0,+0) = {B(Vector.Max(new Vector<float>(nz), new Vector<float>(pz))[0])}");
            Console.WriteLine($"Vector.Min(+0,-0) = {B(Vector.Min(new Vector<float>(pz), new Vector<float>(nz))[0])}");
            Console.WriteLine($"Vector.Min(-0,+0) = {B(Vector.Min(new Vector<float>(nz), new Vector<float>(pz))[0])}");
            Console.WriteLine($"Vector.Max(1,-0)  = {B(Vector.Max(new Vector<float>(1f), new Vector<float>(nz))[0])}");
            Console.WriteLine($"Vector.Min(5,-0)  = {B(Vector.Min(new Vector<float>(5f), new Vector<float>(nz))[0])}");
            Console.WriteLine($"MathF.Max(+0,-0) = {B(MathF.Max(pz, nz))}  MathF.Max(-0,+0) = {B(MathF.Max(nz, pz))}");
            Console.WriteLine($"MathF.Min(+0,-0) = {B(MathF.Min(pz, nz))}  MathF.Min(-0,+0) = {B(MathF.Min(nz, pz))}");
            Console.WriteLine($"MaxNative(+0,-0) = {B(float.MaxNative(pz, nz))}  MaxNative(-0,+0) = {B(float.MaxNative(nz, pz))}");
            Console.WriteLine($"MinNative(+0,-0) = {B(float.MinNative(pz, nz))}  MinNative(-0,+0) = {B(float.MinNative(nz, pz))}");
            var nan1 = BitConverter.UInt32BitsToSingle(0x7FC00001);
            Console.WriteLine($"Vector.Max(1,NaN1) = {B(Vector.Max(new Vector<float>(1f), new Vector<float>(nan1))[0])}  Vector.Max(NaN1,1) = {B(Vector.Max(new Vector<float>(nan1), new Vector<float>(1f))[0])}");
            Console.WriteLine($"MathF.Max(1,NaN1) = {B(MathF.Max(1f, nan1))}  MathF.Max(NaN1,1) = {B(MathF.Max(nan1, 1f))}");
            Console.WriteLine($"MaxNative(1,NaN1) = {B(float.MaxNative(1f, nan1))}  MaxNative(NaN1,1) = {B(float.MaxNative(nan1, 1f))}");
        }
        if (mode is "fuzzcapsulehull")
        {
            Console.WriteLine($"Fuzzing capsule-hull, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunCapsuleHull(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleHull(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "stats")
        {
            //Report DepthRefiner iteration counts per pair type to explain where refiner-based tester time goes.
            if (!ScalarDepthRefiner<BepuPhysics.Collidables.Cylinder, CylinderSupportScalar, BepuPhysics.Collidables.Cylinder, CylinderSupportScalar>.CollectStats)
                Console.WriteLine("WARNING: ScalarDepthRefiner.CollectStats is false (counters compiled out of the timed loop); flip it to true and rebuild for meaningful stats.");
            void Report(string name, long calls, long iterations) =>
                Console.WriteLine($"{name}: {calls} refiner calls, {iterations} support iterations, {(double)iterations / calls:F2} avg iterations/call");
            {
                var setupRandom = new Random(seed);
                using var hullSet = HullSet.Create(setupRandom, 24, 4, 64);
                var generator = new SphereHullGenerator(seed + 1, hullSet, contactHeavy: true);
                for (int i = 0; i < 100000; ++i)
                {
                    var pair = generator.Next();
                    SphereConvexHullScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationB, out _);
                }
                Report("sphere-hull (contact-heavy)",
                    ScalarDepthRefiner<BepuPhysics.Collidables.ConvexHull, HullSupportScalar, BepuPhysics.Collidables.Sphere, SphereSupportScalar>.TotalCalls,
                    ScalarDepthRefiner<BepuPhysics.Collidables.ConvexHull, HullSupportScalar, BepuPhysics.Collidables.Sphere, SphereSupportScalar>.TotalIterations);
            }
            {
                var setupRandom = new Random(seed);
                using var hullSet = HullSet.Create(setupRandom, 24, 4, 64);
                var generator = new HullHullGenerator(seed + 1, hullSet, contactHeavy: true);
                for (int i = 0; i < 20000; ++i)
                {
                    var pair = generator.Next();
                    ConvexHullPairScalarTester.Test(ref hullSet.Hulls[pair.A], ref hullSet.Hulls[pair.B], pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out _);
                }
                Report("hull-hull (contact-heavy)",
                    ScalarDepthRefiner<BepuPhysics.Collidables.ConvexHull, HullSupportScalar, BepuPhysics.Collidables.ConvexHull, HullSupportScalar>.TotalCalls,
                    ScalarDepthRefiner<BepuPhysics.Collidables.ConvexHull, HullSupportScalar, BepuPhysics.Collidables.ConvexHull, HullSupportScalar>.TotalIterations);
            }
            void ReportCylinderConditionStats(string name)
            {
                var calls = ScalarDepthRefiner<BepuPhysics.Collidables.Cylinder, CylinderSupportScalar, BepuPhysics.Collidables.Cylinder, CylinderSupportScalar>.TotalCalls;
                var iterations = ScalarDepthRefiner<BepuPhysics.Collidables.Cylinder, CylinderSupportScalar, BepuPhysics.Collidables.Cylinder, CylinderSupportScalar>.TotalIterations;
                Report($"cylinder-cylinder ({name})", calls, iterations);
                void Line(string label, params (string, long)[] entries)
                {
                    Console.Write($"    {label}:");
                    foreach (var (n, v) in entries)
                        Console.Write($" {n} {100.0 * v / iterations:F1}%");
                    Console.WriteLine();
                }
                var r = typeof(ScalarDepthRefiner<BepuPhysics.Collidables.Cylinder, CylinderSupportScalar, BepuPhysics.Collidables.Cylinder, CylinderSupportScalar>);
                long S(string f) => (long)r.GetField(f).GetValue(null);
                Line("simplex", ("full", S("StatSimplexFull")), ("fillA", S("StatFillA")), ("fillB", S("StatFillB")), ("fillC", S("StatFillC")));
                Line("subtriangle", ("ABD", S("StatSubABD")), ("BCD", S("StatSubBCD")), ("CAD", S("StatSubCAD")), ("fallback", S("StatSubFallback")));
                Line("shape", ("outsideEdges", S("StatOutsideEdges")), ("degenerate", S("StatDegenerate")), ("isVertex", S("StatIsVertex")));
                Line("edge case", ("useEdge", S("StatUseEdge")), ("AB", S("StatEdgeAB")), ("BC", S("StatEdgeBC")), ("CA", S("StatEdgeCA")));
                Line("edge t", ("start", S("StatTStart")), ("end", S("StatTEnd")), ("interior", S("StatTInterior")));
                Line("face/misc", ("faceCase", S("StatTargetContained")), ("calibNegate", S("StatCalibrationNegate")), ("bestDepth<0", S("StatBestDepthNegative")), ("pushCandidate", S("StatPushCandidate")));
                Line("terminated in GetNextNormal", ("terminated", S("StatTerminated")));
            }
            {
                var generator = new CylinderCylinderGenerator(seed, contactHeavy: true);
                for (int i = 0; i < 100000; ++i)
                {
                    var pair = generator.Next();
                    CylinderPairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out _);
                }
                ReportCylinderConditionStats("contact-heavy");
            }
            {
                ScalarDepthRefinerStatReset();
                var generator = new CylinderCylinderGenerator(seed, contactHeavy: false);
                for (int i = 0; i < 100000; ++i)
                {
                    var pair = generator.Next();
                    CylinderPairScalarTester.Test(pair.A, pair.B, pair.SpeculativeMargin, pair.OffsetB, pair.OrientationA, pair.OrientationB, out _);
                }
                ReportCylinderConditionStats("mixed");
            }
            static void ScalarDepthRefinerStatReset()
            {
                var r = typeof(ScalarDepthRefiner<BepuPhysics.Collidables.Cylinder, CylinderSupportScalar, BepuPhysics.Collidables.Cylinder, CylinderSupportScalar>);
                foreach (var field in r.GetFields())
                    if (field.FieldType == typeof(long))
                        field.SetValue(null, 0L);
            }
        }
        if (mode is "dotprobe")
        {
            //Determine Vector3.Dot's actual reduction tree on this runtime/ISA by feeding it order-distinguishing inputs.
            //Candidate trees for sum(m0, m1, m2) with the zero-extended W product m3 = +0:
            //  linear:    ((m0 + m1) + m2)         -- the wide Vector3Wide.Dot order
            //  pairwiseZ: ((m0 + m1) + (m2 + 0))   -- hadd/faddp-style tree
            //  swapped:   ((m0 + m2) + (m1 + 0))   -- shuffle(2,3,0,1)-style tree
            //  rightAssoc: (m0 + (m1 + m2))
            var caseData = new float[][]
            {
                new[] { 1e30f, -1e30f, 1f, 1f, 1f, 1f },
                new[] { 1f, 1e30f, -1e30f, 1f, 1f, 1f },
                new[] { 0f, 0f, 0f, -1f, -1f, -1f },
                new[] { 1e30f, 1f, -1e30f, 1f, 1f, 1f },
            };
            var zero = caseData[2][0];
            string Bits(float x) => BitConverter.SingleToUInt32Bits(x).ToString("X8");
            for (int rep = 0; rep < 2; ++rep)
            {
                //Second pass runs after a warm loop so tier promotion differences would show.
                if (rep == 1)
                {
                    float sink = 0;
                    for (int i = 0; i < 3_000_000; ++i)
                        sink += Vector3.Dot(new Vector3(i, 1, 1), new Vector3(1, 1, 1));
                    Console.WriteLine($"    (warm sink {sink})");
                }
                foreach (var c in caseData)
                {
                    var a = new Vector3(c[0], c[1], c[2]);
                    var b = new Vector3(c[3], c[4], c[5]);
                    var d = Vector3.Dot(a, b);
                    var m0 = c[0] * c[3];
                    var m1 = c[1] * c[4];
                    var m2 = c[2] * c[5];
                    var linear = m0 + m1 + m2;
                    var pairwiseZ = (m0 + m1) + (m2 + zero);
                    var swapped = (m0 + m2) + (m1 + zero);
                    var rightAssoc = m0 + (m1 + m2);
                    Console.WriteLine($"    dot={Bits(d)} linear={Bits(linear)} pairwiseZ={Bits(pairwiseZ)} swapped={Bits(swapped)} rightAssoc={Bits(rightAssoc)}" +
                        $" -> matches: {(Bits(d) == Bits(linear) ? "linear " : "")}{(Bits(d) == Bits(pairwiseZ) ? "pairwiseZ " : "")}{(Bits(d) == Bits(swapped) ? "swapped " : "")}{(Bits(d) == Bits(rightAssoc) ? "rightAssoc" : "")}");
                }
            }
        }
        if (mode is "fma")
        {
            EquivalenceRunner.RunBoxBoxFma(count, seed);
            EquivalenceRunner.RunBoxBoxFma(count, seed + 1, contactHeavy: true);
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunBoxFma(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, mixed", false);
            ThroughputRunner.RunBoxFma(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzzboxtri")
        {
            Console.WriteLine($"Fuzzing box-triangle, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunBoxTriangle(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxTriangle(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchboxtri")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunBoxTriangle(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, mixed", false);
            ThroughputRunner.RunBoxTriangle(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "benchboxch")
        {
            ThroughputRunner.RunBox(1 << 13, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "benchbox")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunBox(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, mixed", false);
            ThroughputRunner.RunBox(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "benchcyl")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunCylinder(hotPairs, repsPerTrial: 60, trialCount: 7, seed: seed + 9, "hot, mixed", false);
            ThroughputRunner.RunCylinder(hotPairs, repsPerTrial: 60, trialCount: 7, seed: seed + 10, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzzboxcyl")
        {
            Console.WriteLine($"Fuzzing box-cylinder, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunBoxCylinder(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunBoxCylinder(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchboxcyl")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunBoxCylinder(hotPairs, repsPerTrial: 60, trialCount: 7, seed: seed + 12, "hot, mixed", false);
            ThroughputRunner.RunBoxCylinder(hotPairs, repsPerTrial: 60, trialCount: 7, seed: seed + 13, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzzcapsulecyl")
        {
            Console.WriteLine($"Fuzzing capsule-cylinder, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunCapsuleCylinder(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleCylinder(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchcapsulecyl")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunCapsuleCylinder(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 14, "hot, mixed", false);
            ThroughputRunner.RunCapsuleCylinder(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 15, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzztricyl")
        {
            Console.WriteLine($"Fuzzing triangle-cylinder, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunTriangleCylinder(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunTriangleCylinder(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchtricyl")
        {
            var hotPairs = 1 << 13;
            ThroughputRunner.RunTriangleCylinder(hotPairs, repsPerTrial: 40, trialCount: 7, seed: seed + 16, "hot, mixed", false);
            ThroughputRunner.RunTriangleCylinder(hotPairs, repsPerTrial: 40, trialCount: 7, seed: seed + 17, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzztripair")
        {
            Console.WriteLine($"Fuzzing triangle-triangle, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunTrianglePair(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunTrianglePair(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchtripair")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunTrianglePair(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 25, "hot, mixed", false);
            ThroughputRunner.RunTrianglePair(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 26, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzzcapsulepair")
        {
            Console.WriteLine($"Fuzzing capsule-capsule, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunCapsulePair(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsulePair(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchcapsulepair")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunCapsulePair(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 18, "hot, mixed", false);
            ThroughputRunner.RunCapsulePair(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 19, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzzcapsulebox")
        {
            Console.WriteLine($"Fuzzing capsule-box, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunCapsuleBox(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleBox(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchcapsulebox")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunCapsuleBox(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 21, "hot, mixed", false);
            ThroughputRunner.RunCapsuleBox(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 22, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzzcapsuletri")
        {
            Console.WriteLine($"Fuzzing capsule-triangle, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunCapsuleTriangle(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCapsuleTriangle(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchcapsuletri")
        {
            const int hotPairs = 1 << 13;
            ThroughputRunner.RunCapsuleTriangle(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 23, "hot, mixed", false);
            ThroughputRunner.RunCapsuleTriangle(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 24, "hot, contact-heavy", true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "fuzzcyl")
        {
            Console.WriteLine($"Fuzzing cylinder-cylinder, {count} cases per config, seed {seed}...");
            mismatches += EquivalenceRunner.RunCylinderCylinder(count, seed, maxReports: 10);
            mismatches += EquivalenceRunner.RunCylinderCylinder(count, seed + 1, maxReports: 10, contactHeavy: true);
        }
        if (mode is "benchpipe")
        {
            ThroughputRunner.RunSphereBoxPipelineExperiment(1 << 13, repsPerTrial: 2000, trialCount: 7, seed: seed + 40);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "benchsphere")
        {
            ThroughputRunner.RunSphereVariants(1 << 13, trialCount: 7, seed: seed + 20, contactHeavy: false);
            ThroughputRunner.RunSphereVariants(1 << 13, trialCount: 7, seed: seed + 30, contactHeavy: true);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "benchhullpairs")
        {
            //The four (convex shape)-hull pairs, hot working set, mixed + contact-heavy. Hull geometry is shared from a pool, so
            //everything is cache-hot; run benchhull separately as the machine-state anchor.
            const int hullPairCount = 1 << 12;
            ThroughputRunner.RunCapsuleHull(hullPairCount, repsPerTrial: 40, trialCount: 7, seed: seed + 12, "hot, mixed", false);
            ThroughputRunner.RunCapsuleHull(hullPairCount, repsPerTrial: 40, trialCount: 7, seed: seed + 13, "hot, contact-heavy", true);
            ThroughputRunner.RunShapeHull<BepuPhysics.Collidables.Box, BepuPhysics.Collidables.BoxWide, BepuPhysics.CollisionDetection.CollisionTasks.BoxConvexHullTester, BoxConvexHullScalarTester>(
                "Box-hull", hullPairCount, repsPerTrial: 15, trialCount: 7, seed: seed + 14, "hot, mixed", false, (s, set, ch) => new BoxHullGenerator(s, set, ch));
            ThroughputRunner.RunShapeHull<BepuPhysics.Collidables.Box, BepuPhysics.Collidables.BoxWide, BepuPhysics.CollisionDetection.CollisionTasks.BoxConvexHullTester, BoxConvexHullScalarTester>(
                "Box-hull", hullPairCount, repsPerTrial: 15, trialCount: 7, seed: seed + 15, "hot, contact-heavy", true, (s, set, ch) => new BoxHullGenerator(s, set, ch));
            ThroughputRunner.RunShapeHull<BepuPhysics.Collidables.Cylinder, BepuPhysics.Collidables.CylinderWide, BepuPhysics.CollisionDetection.CollisionTasks.CylinderConvexHullTester, CylinderConvexHullScalarTester>(
                "Cylinder-hull", hullPairCount, repsPerTrial: 15, trialCount: 7, seed: seed + 16, "hot, mixed", false, (s, set, ch) => new CylinderHullGenerator(s, set, ch));
            ThroughputRunner.RunShapeHull<BepuPhysics.Collidables.Cylinder, BepuPhysics.Collidables.CylinderWide, BepuPhysics.CollisionDetection.CollisionTasks.CylinderConvexHullTester, CylinderConvexHullScalarTester>(
                "Cylinder-hull", hullPairCount, repsPerTrial: 15, trialCount: 7, seed: seed + 17, "hot, contact-heavy", true, (s, set, ch) => new CylinderHullGenerator(s, set, ch));
            ThroughputRunner.RunShapeHull<BepuPhysics.Collidables.Triangle, BepuPhysics.Collidables.TriangleWide, BepuPhysics.CollisionDetection.CollisionTasks.TriangleConvexHullTester, TriangleConvexHullScalarTester>(
                "Triangle-hull", hullPairCount, repsPerTrial: 15, trialCount: 7, seed: seed + 18, "hot, mixed", false, (s, set, ch) => new TriangleHullGenerator(s, set, ch));
            ThroughputRunner.RunShapeHull<BepuPhysics.Collidables.Triangle, BepuPhysics.Collidables.TriangleWide, BepuPhysics.CollisionDetection.CollisionTasks.TriangleConvexHullTester, TriangleConvexHullScalarTester>(
                "Triangle-hull", hullPairCount, repsPerTrial: 15, trialCount: 7, seed: seed + 19, "hot, contact-heavy", true, (s, set, ch) => new TriangleHullGenerator(s, set, ch));
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "benchhull")
        {
            const int hullOnlyPairs = 1 << 12;
            ThroughputRunner.RunHull(hullOnlyPairs, repsPerTrial: 20, trialCount: 9, seed: seed + 5, "small hulls (8-16 pts), contact-heavy", true, 8, 16);
            ThroughputRunner.RunHull(hullOnlyPairs, repsPerTrial: 6, trialCount: 9, seed: seed + 6, "large hulls (40-64 pts), contact-heavy", true, 40, 64);
            ThroughputRunner.RunHull(hullOnlyPairs, repsPerTrial: 10, trialCount: 9, seed: seed + 7, "mixed hulls (4-64 pts), contact-heavy", true, 4, 64);
            ThroughputRunner.RunHull(hullOnlyPairs, repsPerTrial: 10, trialCount: 9, seed: seed + 8, "mixed hulls (4-64 pts), mixed", false, 4, 64);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "relaxfuzzhull")
        {
            //Relaxed-equality hull-hull study: candidate vs the engine wide reference through the tolerance comparator.
            var candidate = RelaxedCandidateRegistry.Find(candidateName);
            if (candidate == null)
            {
                Console.WriteLine($"Unknown candidate '{candidateName}'. Available: {RelaxedCandidateRegistry.AvailableNames}");
                return 2;
            }
            var relaxCount = countExplicit ? count : 250_000;
            Console.WriteLine($"Relaxed hull-hull fuzz: candidate '{candidate.Name}', target size {size}, {relaxCount} cases per profile, seed {seed}...");
            mismatches += EquivalenceRunner.RunHullHullRelaxed(candidate, relaxCount, seed, size, contactHeavy: false, maxReports: maxFailReports);
            mismatches += EquivalenceRunner.RunHullHullRelaxed(candidate, relaxCount, seed + 1, size, contactHeavy: true, maxReports: maxFailReports);
        }
        if (mode is "relaxtriagehull")
        {
            var candidate = RelaxedCandidateRegistry.Find(candidateName);
            if (candidate == null)
            {
                Console.WriteLine($"Unknown candidate '{candidateName}'. Available: {RelaxedCandidateRegistry.AvailableNames}");
                return 2;
            }
            var triageCount = countExplicit ? count : 100_000;
            Console.WriteLine($"Relaxed hull-hull FAIL triage: candidate '{candidate.Name}', target size {size}, {triageCount} cases per profile, seed {seed}...");
            EquivalenceRunner.RunHullHullTriage(candidate, triageCount, seed, size, contactHeavy: false, samplesPerClass: maxFailReports);
            EquivalenceRunner.RunHullHullTriage(candidate, triageCount, seed + 1, size, contactHeavy: true, samplesPerClass: maxFailReports);
        }
        if (mode is "satstats")
        {
            //SAT candidate statistics: winner-type distribution, gauss-arc filter rejection rate, phase reject rates,
            //per hull size and profile. Runs the candidate alone (no wide reference), so counts are cheap to gather.
            var candidate = RelaxedCandidateRegistry.Find(candidateName == "frozen" ? "sat" : candidateName);
            if (candidate == null)
            {
                Console.WriteLine($"Unknown candidate '{candidateName}'. Available: {RelaxedCandidateRegistry.AvailableNames}");
                return 2;
            }
            var statsCount = countExplicit ? count : 100_000;
            foreach (var statsSize in sizes)
            {
                foreach (var heavy in new[] { false, true })
                {
                    var setupRandom = new Random(seed + statsSize);
                    using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, statsSize);
                    var topologies = HullTopology.CreateForSet(hullSet, out _);
                    var generator = new HullHullGenerator(seed + statsSize + (heavy ? 1 : 0), hullSet, heavy);
                    SatStats.Reset();
                    for (long i = 0; i < statsCount; ++i)
                    {
                        var pairCase = generator.Next();
                        candidate.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                            pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, out var m);
                        ThroughputRunner.Sink += m.Depth0;
                    }
                    SatStats.Print($"satstats '{candidate.Name}' size {statsSize} {(heavy ? "contact-heavy" : "mixed")} ({statsCount} cases)");
                }
            }
        }
        if (mode is "walkstats")
        {
            //Walk candidate statistics: face-walk eval/move counts, restricted-edge-phase counts, winner distribution,
            //fallback rate, per hull size and profile.
            var candidate = RelaxedCandidateRegistry.Find(candidateName == "frozen" ? "walk" : candidateName);
            if (candidate == null)
            {
                Console.WriteLine($"Unknown candidate '{candidateName}'. Available: {RelaxedCandidateRegistry.AvailableNames}");
                return 2;
            }
            var statsCount = countExplicit ? count : 100_000;
            foreach (var statsSize in sizes)
            {
                foreach (var heavy in new[] { false, true })
                {
                    var setupRandom = new Random(seed + statsSize);
                    using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, statsSize);
                    var topologies = HullTopology.CreateForSet(hullSet, out _);
                    var generator = new HullHullGenerator(seed + statsSize + (heavy ? 1 : 0), hullSet, heavy);
                    WalkStats.Reset();
                    for (long i = 0; i < statsCount; ++i)
                    {
                        var pairCase = generator.Next();
                        candidate.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                            pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, out var m);
                        ThroughputRunner.Sink += m.Depth0;
                    }
                    WalkStats.Print($"walkstats '{candidate.Name}' size {statsSize} {(heavy ? "contact-heavy" : "mixed")} ({statsCount} cases)");
                }
            }
        }
        if (mode is "walkoracle")
        {
            //Local-vs-global axis validation: the walk candidate's restricted axis search against Track 2's exhaustive
            //SAT as the oracle. Both sides run WITHOUT the refiner fallback so the compared axis is the raw minimum over
            //each side's searched axis set (walk's set is a subset of SAT's, so walkDepth >= satDepth up to fp noise).
            //Penetrating misses are retried at larger edge-restriction rings to price the escalation that would fix them.
            var oracleCount = countExplicit ? count : 50_000;
            foreach (var oracleSize in sizes)
            {
                foreach (var heavy in new[] { false, true })
                {
                    var setupRandom = new Random(seed + oracleSize);
                    using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, oracleSize);
                    var topologies = HullTopology.CreateForSet(hullSet, out _);
                    var generator = new HullHullGenerator(seed + oracleSize + (heavy ? 1 : 0), hullSet, heavy);
                    long cases = 0, bothRejected = 0, satRejectedWalkAccepted = 0, walkRejectedSatAccepted = 0;
                    long penetrating = 0, penExact = 0, penWinnerAgree = 0, penMiss = 0, penFixedRing1 = 0, penFixedRing2 = 0, penUnfixed = 0;
                    long separated = 0, sepExact = 0, sepMiss = 0, sepSignFlips = 0;
                    double maxPenExcess = 0, sumPenExcess = 0, maxSepExcess = 0;
                    for (long i = 0; i < oracleCount; ++i)
                    {
                        var pairCase = generator.Next();
                        var scale = MathF.Min(hullSet.MaxRadii[pairCase.A], hullSet.MaxRadii[pairCase.B]);
                        SatHullPairTester.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                            pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, usePolish: false, out _);
                        var satRejected = SatHullPairTester.LastWinnerType < 0;
                        var satDepth = SatHullPairTester.LastDepth;
                        var satWinner = SatHullPairTester.LastWinnerType;
                        WalkHullPairTester.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                            pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, ringDepth: 0, useFallback: false, out _);
                        var walkRejected = WalkHullPairTester.LastWinnerType < 0;
                        var walkDepth = WalkHullPairTester.LastDepth;
                        ++cases;
                        if (satRejected && walkRejected) { ++bothRejected; continue; }
                        if (satRejected)
                        {
                            //Walk failed to find any axis below -margin that SAT's exhaustive sweep found: with the real
                            //fallback this is only caught when the walk's own best went negative; count it honestly.
                            ++satRejectedWalkAccepted;
                            if (satRejectedWalkAccepted <= maxFailReports)
                                Console.WriteLine($"    [oracle] sat-reject/walk-accept case {i}: satDepth {satDepth:G6}, walkDepth {walkDepth:G6}, margin {pairCase.SpeculativeMargin:G6}, scale {scale:G4}");
                            continue;
                        }
                        if (walkRejected) { ++walkRejectedSatAccepted; continue; }
                        var tolerance = MathF.Max(1e-5f * scale, 1e-4f * MathF.Abs(satDepth));
                        var excess = walkDepth - satDepth;
                        if (satDepth >= 0f)
                        {
                            ++penetrating;
                            if (excess <= tolerance)
                            {
                                ++penExact;
                                if (satWinner == WalkHullPairTester.LastWinnerType)
                                    ++penWinnerAgree;
                            }
                            else
                            {
                                ++penMiss;
                                var normalized = excess / scale;
                                sumPenExcess += normalized;
                                if (normalized > maxPenExcess)
                                    maxPenExcess = normalized;
                                WalkHullPairTester.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                                    pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, ringDepth: 1, useFallback: false, out _);
                                if (WalkHullPairTester.LastWinnerType >= 0 && WalkHullPairTester.LastDepth - satDepth <= tolerance)
                                    ++penFixedRing1;
                                else
                                {
                                    WalkHullPairTester.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                                        pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, ringDepth: 2, useFallback: false, out _);
                                    if (WalkHullPairTester.LastWinnerType >= 0 && WalkHullPairTester.LastDepth - satDepth <= tolerance)
                                        ++penFixedRing2;
                                    else
                                    {
                                        ++penUnfixed;
                                        if (penUnfixed <= maxFailReports)
                                            Console.WriteLine($"    [oracle] unfixed pen miss case {i}: satDepth {satDepth:G6} (winner {satWinner}), walkDepth {walkDepth:G6} (winner {WalkHullPairTester.LastWinnerType}), excess/scale {normalized:E2}");
                                    }
                                }
                            }
                        }
                        else
                        {
                            //Separated within margin: in the real candidate this band goes to the refiner fallback, but the
                            //axis-set agreement is still informative (both sides minimize over the same SAT axis family).
                            ++separated;
                            if (excess <= tolerance)
                                ++sepExact;
                            else
                            {
                                ++sepMiss;
                                var normalized = excess / scale;
                                if (normalized > maxSepExcess)
                                    maxSepExcess = normalized;
                            }
                            if (walkDepth >= 0f)
                                ++sepSignFlips;
                        }
                    }
                    Console.WriteLine($"walkoracle size {oracleSize} {(heavy ? "contact-heavy" : "mixed")} ({cases} cases): both-reject {bothRejected}, sat-reject/walk-accept {satRejectedWalkAccepted}, walk-reject/sat-accept {walkRejectedSatAccepted}");
                    Console.WriteLine($"    penetrating {penetrating}: axis exact {penExact} ({(double)penExact / Math.Max(1, penetrating):P3}, winner-type agree {penWinnerAgree}), misses {penMiss} ({(double)penMiss / Math.Max(1, penetrating):P3}; excess/scale mean {(penMiss > 0 ? sumPenExcess / penMiss : 0):E2} max {maxPenExcess:E2}); fixed by ring1 {penFixedRing1}, ring2 {penFixedRing2}, unfixed {penUnfixed}");
                    Console.WriteLine($"    separated-in-margin {separated}: axis exact {sepExact}, miss {sepMiss} (max excess/scale {maxSepExcess:E2}), would-be sign flips {sepSignFlips} (band uses refiner fallback in the real candidate)");
                }
            }
        }
        if (mode is "csoprobe")
        {
            //Deterministic validation of the CSO walk's geometric claims (winding/inward-direction conventions) before
            //trusting them in the walk: for random poses, checks that (1) FacetB CSO polygons (B loop order) are CCW
            //around the B face normal (inward = cross(n, edge) points at the centroid), (2) FacetA CSO polygons
            //(REVERSED A loop) are CCW around -R*n_faceA, (3) edge-state inward directions cross(nFace, d_ccw) point
            //from each edge midpoint toward the adjacent face's centroid on both hulls' CSO images.
            foreach (var probeSize in new[] { 8, 32 })
            {
                var random = new Random(seed + probeSize);
                using var hullSet = HullSet.CreateSized(random, 4, probeSize);
                var topologies = HullTopology.CreateForSet(hullSet, out _);
                long facetBChecks = 0, facetBViolations = 0, facetAChecks = 0, facetAViolations = 0;
                long edgeBChecks = 0, edgeBViolations = 0, edgeAChecks = 0, edgeAViolations = 0;
                for (int pose = 0; pose < 8; ++pose)
                {
                    var topologyA = topologies[random.Next(topologies.Length)];
                    var topologyB = topologies[random.Next(topologies.Length)];
                    var q = Quaternion.Normalize(new Quaternion(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1));
                    BepuUtilities.Matrix3x3.CreateFromQuaternion(q, out BepuUtilities.Matrix3x3 rotation);
                    //FacetB polygons: raw B loops (translation cancels in the check).
                    for (int f = 0; f < topologyB.FaceCount; ++f)
                    {
                        var n = topologyB.FaceNormals[f];
                        var start = topologyB.FaceStarts[f];
                        var loopCount = topologyB.FaceStarts[f + 1] - start;
                        var centroid = Vector3.Zero;
                        for (int k = 0; k < loopCount; ++k)
                            centroid += topologyB.Vertices[topologyB.FaceVertices[start + k]];
                        centroid *= 1f / loopCount;
                        var previous = topologyB.Vertices[topologyB.FaceVertices[start + loopCount - 1]];
                        for (int k = 0; k < loopCount; ++k)
                        {
                            var current = topologyB.Vertices[topologyB.FaceVertices[start + k]];
                            var inward = Vector3.Cross(n, current - previous);
                            ++facetBChecks;
                            if (Vector3.Dot(inward, centroid - (previous + current) * 0.5f) <= 0)
                                ++facetBViolations;
                            previous = current;
                        }
                    }
                    //FacetA polygons: c_j = -R * a_(reversed j) (vb and offset cancel); n = -R * n_faceA.
                    for (int f = 0; f < topologyA.FaceCount; ++f)
                    {
                        BepuUtilities.Matrix3x3.Transform(topologyA.FaceNormals[f], rotation, out var m);
                        var n = -m;
                        var start = topologyA.FaceStarts[f];
                        var loopCount = topologyA.FaceStarts[f + 1] - start;
                        Vector3 Cso(int reversedPosition)
                        {
                            BepuUtilities.Matrix3x3.Transform(topologyA.Vertices[topologyA.FaceVertices[start + (loopCount - 1 - reversedPosition)]], rotation, out var pa);
                            return -pa;
                        }
                        var centroid = Vector3.Zero;
                        for (int j = 0; j < loopCount; ++j)
                            centroid += Cso(j);
                        centroid *= 1f / loopCount;
                        var previous = Cso(loopCount - 1);
                        for (int j = 0; j < loopCount; ++j)
                        {
                            var current = Cso(j);
                            var inward = Vector3.Cross(n, current - previous);
                            ++facetAChecks;
                            if (Vector3.Dot(inward, centroid - (previous + current) * 0.5f) <= 0)
                                ++facetAViolations;
                            previous = current;
                        }
                    }
                    //EdgeB inward directions: Face0 walks Start->End, so d_ccw = +dB for Face0, -dB for Face1.
                    foreach (ref var edge in topologyB.Edges.AsSpan())
                    {
                        var dB = topologyB.Vertices[edge.End] - topologyB.Vertices[edge.Start];
                        var mid = (topologyB.Vertices[edge.End] + topologyB.Vertices[edge.Start]) * 0.5f;
                        void CheckB(int face, Vector3 dCcw, ref long checks, ref long violations)
                        {
                            var startF = topologyB.FaceStarts[face];
                            var countF = topologyB.FaceStarts[face + 1] - startF;
                            var centroid = Vector3.Zero;
                            for (int k = 0; k < countF; ++k)
                                centroid += topologyB.Vertices[topologyB.FaceVertices[startF + k]];
                            centroid *= 1f / countF;
                            ++checks;
                            if (Vector3.Dot(Vector3.Cross(topologyB.FaceNormals[face], dCcw), centroid - mid) <= 0)
                                ++violations;
                        }
                        CheckB(edge.Face0, dB, ref edgeBChecks, ref edgeBViolations);
                        CheckB(edge.Face1, -dB, ref edgeBChecks, ref edgeBViolations);
                    }
                    //EdgeA inward directions in CSO space: reversed loop traverses the edge along +dAp for Face0.
                    foreach (ref var edge in topologyA.Edges.AsSpan())
                    {
                        BepuUtilities.Matrix3x3.Transform(topologyA.Vertices[edge.End] - topologyA.Vertices[edge.Start], rotation, out var dAp);
                        BepuUtilities.Matrix3x3.Transform((topologyA.Vertices[edge.End] + topologyA.Vertices[edge.Start]) * 0.5f, rotation, out var midA);
                        var midCso = -midA;
                        void CheckA(int face, Vector3 dCcw, ref long checks, ref long violations)
                        {
                            BepuUtilities.Matrix3x3.Transform(topologyA.FaceNormals[face], rotation, out var m);
                            var startF = topologyA.FaceStarts[face];
                            var countF = topologyA.FaceStarts[face + 1] - startF;
                            var centroid = Vector3.Zero;
                            for (int k = 0; k < countF; ++k)
                            {
                                BepuUtilities.Matrix3x3.Transform(topologyA.Vertices[topologyA.FaceVertices[startF + k]], rotation, out var pa);
                                centroid += -pa;
                            }
                            centroid *= 1f / countF;
                            ++checks;
                            if (Vector3.Dot(Vector3.Cross(-m, dCcw), centroid - midCso) <= 0)
                                ++violations;
                        }
                        CheckA(edge.Face0, dAp, ref edgeAChecks, ref edgeAViolations);
                        CheckA(edge.Face1, -dAp, ref edgeAChecks, ref edgeAViolations);
                    }
                }
                Console.WriteLine($"csoprobe size {probeSize}: facetB {facetBViolations}/{facetBChecks} violations, facetA {facetAViolations}/{facetAChecks}, " +
                    $"edgeB inward {edgeBViolations}/{edgeBChecks}, edgeA inward {edgeAViolations}/{edgeAChecks} (expect 0 everywhere)");
            }
        }
        if (mode is "csostats")
        {
            //CSO walk statistics: penetrating-walk step/eval counts, separated-walk step counts and stall rates,
            //winner distribution, per hull size and profile.
            var candidate = RelaxedCandidateRegistry.Find(candidateName == "frozen" ? "csowalk" : candidateName);
            if (candidate == null)
            {
                Console.WriteLine($"Unknown candidate '{candidateName}'. Available: {RelaxedCandidateRegistry.AvailableNames}");
                return 2;
            }
            var statsCount = countExplicit ? count : 100_000;
            foreach (var statsSize in sizes)
            {
                foreach (var heavy in new[] { false, true })
                {
                    var setupRandom = new Random(seed + statsSize);
                    using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, statsSize);
                    var topologies = HullTopology.CreateForSet(hullSet, out _);
                    var generator = new HullHullGenerator(seed + statsSize + (heavy ? 1 : 0), hullSet, heavy);
                    CsoStats.Reset();
                    for (long i = 0; i < statsCount; ++i)
                    {
                        var pairCase = generator.Next();
                        candidate.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                            pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, out var m);
                        ThroughputRunner.Sink += m.Depth0;
                    }
                    CsoStats.Print($"csostats '{candidate.Name}' size {statsSize} {(heavy ? "contact-heavy" : "mixed")} ({statsCount} cases)");
                }
            }
        }
        if (mode is "csooracle")
        {
            //Track 4 basin analysis: the CSO walk vs Track 2's exhaustive SAT (usePolish: false) as the axis oracle.
            //Penetrating cases (satDepth >= 0): the walk's local minimum vs the exact SAT minimum -> global-optimum hit
            //rate, directly comparable to Track 3's walkoracle numbers; the frozen-configuration scalar refiner runs on
            //the same cases so the engine refiner's own miss rate is measured against the same oracle. Separated-within-
            //margin cases: the distance walk should be globally convergent; its claim is checked with the complete
            //support-plane certificate (supportDepth(claimed normal) == claimed depth) plus the walk-vs-sat inequality
            //(exact distance <= SAT's overestimated gap).
            var oracleCount = countExplicit ? count : 20_000;
            foreach (var oracleSize in sizes)
            {
                foreach (var heavy in new[] { false, true })
                {
                    var setupRandom = new Random(seed + oracleSize);
                    using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, oracleSize);
                    var topologies = HullTopology.CreateForSet(hullSet, out _);
                    var generator = new HullHullGenerator(seed + oracleSize + (heavy ? 1 : 0), hullSet, heavy);
                    CsoStats.Reset();
                    long cases = 0, bothRejected = 0, satRejectedCsoAccepted = 0, csoRejectedSatAccepted = 0, csoRejectedSatPenetrating = 0;
                    long penetrating = 0, penExact = 0, penWinnerAgree = 0, penMiss = 0, penMissViaSeparatedClaim = 0;
                    double maxPenExcess = 0, sumPenExcess = 0;
                    long refinerPen = 0, refinerExact = 0, refinerMiss = 0;
                    double sumRefinerExcess = 0, maxRefinerExcess = 0;
                    long separated = 0, sepCertOk = 0, sepCertMiss = 0, sepSatBelowCso = 0;
                    double maxSepCertGap = 0, sumSepCertGap = 0;
                    long certChecks = 0, certViolations = 0;
                    double maxCertGap = 0;
                    for (long i = 0; i < oracleCount; ++i)
                    {
                        var pairCase = generator.Next();
                        var scale = MathF.Min(hullSet.MaxRadii[pairCase.A], hullSet.MaxRadii[pairCase.B]);
                        SatHullPairTester.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                            pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, usePolish: false, out _);
                        var satRejected = SatHullPairTester.LastWinnerType < 0;
                        var satDepth = SatHullPairTester.LastDepth;
                        var satWinner = SatHullPairTester.LastWinnerType;
                        CsoWalkHullPairTester.Test(topologies[pairCase.A], topologies[pairCase.B], ref hullSet.Hulls[pairCase.A], ref hullSet.Hulls[pairCase.B],
                            pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, out _);
                        var csoRejected = CsoWalkHullPairTester.LastWinnerType < 0;
                        var csoDepth = CsoWalkHullPairTester.LastDepth;
                        ++cases;
                        //Internal consistency: every reported axis (accept or reject) should carry its exact depth.
                        {
                            BepuUtilities.Matrix3x3.CreateFromQuaternion(pairCase.OrientationA, out BepuUtilities.Matrix3x3 rA);
                            BepuUtilities.Matrix3x3.CreateFromQuaternion(pairCase.OrientationB, out BepuUtilities.Matrix3x3 rB);
                            BepuUtilities.Matrix3x3.Transform(CsoWalkHullPairTester.LastLocalNormal, rB, out var worldAxis);
                            var cert = RelaxedComparator.SupportDepth(topologies[pairCase.A], topologies[pairCase.B], rA, rB, pairCase.OffsetB, worldAxis);
                            var gap = MathF.Abs(cert - csoDepth) / scale;
                            ++certChecks;
                            if (gap > 1e-4f)
                            {
                                ++certViolations;
                                if (gap > maxCertGap)
                                    maxCertGap = gap;
                            }
                        }
                        if (satRejected && csoRejected) { ++bothRejected; continue; }
                        if (satRejected)
                        {
                            //SAT found a separating axis beyond the margin the CSO walk never certified.
                            ++satRejectedCsoAccepted;
                            if (satRejectedCsoAccepted <= maxFailReports)
                                Console.WriteLine($"    [oracle] sat-reject/cso-accept case {i}: satDepth {satDepth:G6}, csoDepth {csoDepth:G6} (regime {CsoWalkHullPairTester.LastRegime}, stalled {CsoWalkHullPairTester.LastSepStalled}), margin {pairCase.SpeculativeMargin:G6}");
                            continue;
                        }
                        if (csoRejected)
                        {
                            //The walk's rejects are exact-axis certificates; sat accepting here with a negative depth is
                            //sat's own gap OVERestimate (satDepth >= true distance depth). Split out the (bug-indicating)
                            //case where sat claims actual penetration.
                            if (satDepth >= 0f)
                            {
                                ++csoRejectedSatPenetrating;
                                if (csoRejectedSatPenetrating <= maxFailReports)
                                    Console.WriteLine($"    [oracle] cso-reject/sat-PENETRATING case {i}: satDepth {satDepth:G6}, csoDepth {csoDepth:G6} -- investigate");
                            }
                            else
                                ++csoRejectedSatAccepted;
                            continue;
                        }
                        var tolerance = MathF.Max(1e-5f * scale, 1e-4f * MathF.Abs(satDepth));
                        if (satDepth >= 0f)
                        {
                            ++penetrating;
                            if (CsoWalkHullPairTester.LastRegime == 1)
                            {
                                //Walk claimed separation on a SAT-penetrating pair: a basin miss that also flipped the sign.
                                ++penMiss;
                                ++penMissViaSeparatedClaim;
                                var normalizedFlip = (csoDepth - satDepth) < 0 ? (satDepth - csoDepth) / scale : (csoDepth - satDepth) / scale;
                                sumPenExcess += normalizedFlip;
                                if (normalizedFlip > maxPenExcess)
                                    maxPenExcess = normalizedFlip;
                            }
                            else
                            {
                                var excess = csoDepth - satDepth;
                                if (excess <= tolerance)
                                {
                                    ++penExact;
                                    if (satWinner == CsoWalkHullPairTester.LastWinnerType)
                                        ++penWinnerAgree;
                                }
                                else
                                {
                                    ++penMiss;
                                    var normalized = excess / scale;
                                    sumPenExcess += normalized;
                                    if (normalized > maxPenExcess)
                                        maxPenExcess = normalized;
                                }
                            }
                            //Engine refiner (frozen configuration) on the same penetrating case.
                            {
                                BepuUtilities.Matrix3x3.CreateFromQuaternion(pairCase.OrientationA, out BepuUtilities.Matrix3x3 rA);
                                BepuUtilities.Matrix3x3.CreateFromQuaternion(pairCase.OrientationB, out BepuUtilities.Matrix3x3 rB);
                                ScalarMath.MultiplyByTranspose(rA, rB, out var bLocalOrientationA);
                                var localOffsetA = -ScalarMath.TransformByTransposed(pairCase.OffsetB, rB);
                                var centerDistance = ScalarMath.Length(localOffsetA);
                                var initialNormal = centerDistance > 1e-8f ? localOffsetA * (1f / centerDistance) : new Vector3(0f, 1f, 0f);
                                var refinerEpsilonScale = MathF.Min(
                                    (MathF.Abs(topologies[pairCase.A].Vertices[0].X) + MathF.Abs(topologies[pairCase.A].Vertices[0].Y) + MathF.Abs(topologies[pairCase.A].Vertices[0].Z)) / 3f,
                                    (MathF.Abs(topologies[pairCase.B].Vertices[0].X) + MathF.Abs(topologies[pairCase.B].Vertices[0].Y) + MathF.Abs(topologies[pairCase.B].Vertices[0].Z)) / 3f);
                                ScalarDepthRefiner<BepuPhysics.Collidables.ConvexHull, HullSupportScalar, BepuPhysics.Collidables.ConvexHull, HullSupportScalar>.FindMinimumDepth(
                                    hullSet.Hulls[pairCase.B], hullSet.Hulls[pairCase.A], localOffsetA, bLocalOrientationA, initialNormal,
                                    1e-5f * refinerEpsilonScale, -pairCase.SpeculativeMargin, out var refinerDepth, out _, out _);
                                ++refinerPen;
                                var refinerExcess = refinerDepth - satDepth;
                                if (refinerExcess <= tolerance)
                                    ++refinerExact;
                                else
                                {
                                    ++refinerMiss;
                                    var normalized = refinerExcess / scale;
                                    sumRefinerExcess += normalized;
                                    if (normalized > maxRefinerExcess)
                                        maxRefinerExcess = normalized;
                                }
                            }
                        }
                        else
                        {
                            //Separated within margin. The walk's exact distance must satisfy csoDepth <= satDepth (SAT's
                            //axis set overestimates the gap) and its certificate must close (checked above); count the
                            //certificate gap distribution for the claimed-global check.
                            ++separated;
                            BepuUtilities.Matrix3x3.CreateFromQuaternion(pairCase.OrientationA, out BepuUtilities.Matrix3x3 rA);
                            BepuUtilities.Matrix3x3.CreateFromQuaternion(pairCase.OrientationB, out BepuUtilities.Matrix3x3 rB);
                            BepuUtilities.Matrix3x3.Transform(CsoWalkHullPairTester.LastLocalNormal, rB, out var worldAxis);
                            var cert = RelaxedComparator.SupportDepth(topologies[pairCase.A], topologies[pairCase.B], rA, rB, pairCase.OffsetB, worldAxis);
                            //cert >= -distance always; global optimality <=> cert == csoDepth == -distance.
                            var certGap = (cert - csoDepth) / scale;
                            sumSepCertGap += MathF.Abs(certGap);
                            if (MathF.Abs(certGap) > maxSepCertGap)
                                maxSepCertGap = MathF.Abs(certGap);
                            if (MathF.Abs(certGap) * scale <= tolerance)
                                ++sepCertOk;
                            else
                            {
                                ++sepCertMiss;
                                if (sepCertMiss <= maxFailReports)
                                    Console.WriteLine($"    [oracle] sep certificate gap case {i}: claimed {csoDepth:G6}, certificate {cert:G6}, gap/scale {certGap:E2}, stalled {CsoWalkHullPairTester.LastSepStalled}");
                            }
                            if (satDepth < csoDepth - tolerance)
                                ++sepSatBelowCso;
                        }
                    }
                    Console.WriteLine($"csooracle size {oracleSize} {(heavy ? "contact-heavy" : "mixed")} ({cases} cases): both-reject {bothRejected}, sat-reject/cso-accept {satRejectedCsoAccepted}, cso-reject/sat-accept {csoRejectedSatAccepted} (sat-penetrating {csoRejectedSatPenetrating})");
                    Console.WriteLine($"    penetrating {penetrating}: axis exact {penExact} ({(double)penExact / Math.Max(1, penetrating):P3}, winner-type agree {penWinnerAgree}), misses {penMiss} ({(double)penMiss / Math.Max(1, penetrating):P3}, of which claimed-separated {penMissViaSeparatedClaim}; excess/scale mean {(penMiss > 0 ? sumPenExcess / penMiss : 0):E2} max {maxPenExcess:E2})");
                    Console.WriteLine($"    engine refiner on same cases {refinerPen}: exact {refinerExact} ({(double)refinerExact / Math.Max(1, refinerPen):P3}), misses {refinerMiss} ({(double)refinerMiss / Math.Max(1, refinerPen):P3}; excess/scale mean {(refinerMiss > 0 ? sumRefinerExcess / refinerMiss : 0):E2} max {maxRefinerExcess:E2})");
                    Console.WriteLine($"    separated-in-margin {separated}: certificate-closed {sepCertOk} ({(double)sepCertOk / Math.Max(1, separated):P3}), cert misses {sepCertMiss} (|gap|/scale mean {(separated > 0 ? sumSepCertGap / separated : 0):E2} max {maxSepCertGap:E2}); sat-below-cso violations {sepSatBelowCso} (should be 0)");
                    Console.WriteLine($"    axis-consistency checks {certChecks}: violations {certViolations} (max gap/scale {maxCertGap:E2})");
                    CsoStats.Print($"    csostats for this cell");
                }
            }
        }
        if (mode is "polishoracle")
        {
            //Track 4 followup: post-refiner axis polish (P0/P0b representative-face axes, P1/Pk facet descent) vs the
            //exhaustive-SAT oracle on penetrating cases. --count = target PENETRATING cases per cell (default 20k).
            var target = countExplicit ? (int)count : 20_000;
            foreach (var oracleSize in sizes)
            {
                var setupRandom = new Random(seed + oracleSize);
                using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, oracleSize);
                var topologies = HullTopology.CreateForSet(hullSet, out _);
                PolishOracle.RunCell(hullSet, topologies, oracleSize, contactHeavy: false, seed, target, maxFailReports);
                PolishOracle.RunCell(hullSet, topologies, oracleSize, contactHeavy: true, seed, target, maxFailReports);
            }
        }
        if (mode is "polishbench")
        {
            //Interleaved wall-clock check of refiner-alone vs refiner+Pk on one cell (--size, contact-heavy).
            var benchPairs = countExplicit ? (int)count : 4096;
            PolishOracle.RunBench(size, contactHeavy: true, seed, benchPairs, trialCount: 7);
        }
        //Warm-start study modes (A1 certificate+min-compose, A2 epilogue, B1 separated-band seeding); default sizes {8,32,128}.
        int[] warmSizes = sizesExplicit ? sizes : [8, 32, 128];
        if (mode is "warmfuzz")
        {
            //A1 dominance fuzz: i.i.d. poses, synthesized warm axes (previous-output/perturbed/random/garbage/NaN/zero/non-unit).
            //Asserts clamped-depth dominance and bitwise no-op passthrough; violations are implementation bugs and fail the exit code.
            var perCell = countExplicit ? (int)count : 200_000;
            foreach (var warmSize in warmSizes)
            {
                var setupRandom = new Random(seed + warmSize);
                using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, warmSize);
                var topologies = HullTopology.CreateForSet(hullSet, out _);
                mismatches += WarmStartStudy.RunDominanceFuzz(hullSet, topologies, warmSize, contactHeavy: false, seed, perCell, maxFailReports);
                mismatches += WarmStartStudy.RunDominanceFuzz(hullSet, topologies, warmSize, contactHeavy: true, seed, perCell, maxFailReports);
            }
        }
        if (mode is "warmseq")
        {
            //Closed-loop temporally-coherent sequences: A1 (warm state = its own previous-frame output) vs pure c2c,
            //SAT oracle ground truth per frame. --count = frames per pair (default 256).
            var framesPerPair = countExplicit ? (int)count : 256;
            foreach (var warmSize in warmSizes)
            {
                var setupRandom = new Random(seed + warmSize);
                using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, warmSize);
                var topologies = HullTopology.CreateForSet(hullSet, out _);
                foreach (var scenario in new[] { SequenceScenario.Rest, SequenceScenario.Slide, SequenceScenario.Deepen, SequenceScenario.Teleport })
                {
                    WarmStartStudy.RunSequenceCell(hullSet, topologies, warmSize, scenario, delta: 0.002f, pairs: 32, framesPerPair, seed);
                    WarmStartStudy.RunSequenceCell(hullSet, topologies, warmSize, scenario, delta: 0.02f, pairs: 32, framesPerPair, seed);
                }
            }
        }
        if (mode is "warmpolish")
        {
            //A2 adopt-and-polish cells: per-depth-tier SAT excess + witness-colinearity residual + leg sweep cost,
            //arms {c2c, A1, A2 K=2/4/6}, warm sources {previous-output, perturbed-truth}. --count = target penetrating cases per cell.
            var target = countExplicit ? (int)count : 20_000;
            foreach (var warmSize in warmSizes)
            {
                var setupRandom = new Random(seed + warmSize);
                using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, warmSize);
                var topologies = HullTopology.CreateForSet(hullSet, out _);
                WarmStartStudy.RunA2Cell(hullSet, topologies, warmSize, contactHeavy: false, seed, target, maxFailReports);
                WarmStartStudy.RunA2Cell(hullSet, topologies, warmSize, contactHeavy: true, seed, target, maxFailReports);
            }
        }
        if (mode is "warmb1")
        {
            //B1 separated-band seeding: gate rates, seeded-vs-c2c sweep histograms, cap-exit rate, seed-invariance spot check.
            var perCell = countExplicit ? count : 100_000;
            foreach (var warmSize in warmSizes)
            {
                var setupRandom = new Random(seed + warmSize);
                using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, warmSize);
                var topologies = HullTopology.CreateForSet(hullSet, out _);
                WarmStartStudy.RunB1Cell(hullSet, topologies, warmSize, contactHeavy: false, seed, perCell, maxFailReports);
                WarmStartStudy.RunB1Cell(hullSet, topologies, warmSize, contactHeavy: true, seed, perCell, maxFailReports);
            }
        }
        if (mode is "warmbench")
        {
            //Interleaved wall clock (axis search + setup only) on three stream workloads at --size: i.i.d. mixed,
            //coherent contact-heavy sequences, separated-heavy sequences. Arms: c2c, A1, B1.
            WarmStartStudy.RunWarmBench(size, seed, trialCount: 15);
        }
        if (mode is "climbprobe")
        {
            //Direct validation of the hillclimb support finder against the frozen full-scan support, isolated from the refiner:
            //random + correlated direction sequences per hull, reporting the support dot deficit normalized by hull radius.
            foreach (var probeSize in sizes)
            {
                var random = new Random(seed + probeSize);
                var hullSet = HullSet.CreateSized(random, 16, probeSize);
                var topologies = HullTopology.CreateForSet(hullSet, out _);
                double maxWarmDeficit = 0, maxColdDeficit = 0;
                long queries = 0, warmWorse = 0, coldWorse = 0;
                long warmRounds = 0, warmDots = 0, coldRounds = 0, coldDots = 0;
                //Probe-only instrumented twin of HullSupportScalarHillclimb.Climb (counting stays out of the timed path).
                int ClimbCounted(HullTopology topology, int start, Vector3 d, ref long rounds, ref long dots)
                {
                    var best = start;
                    var bestDot = Vector3.Dot(topology.Vertices[best], d);
                    ++dots;
                    for (int round = 0; round < topology.VertexCount; ++round)
                    {
                        ++rounds;
                        var s0 = topology.VertexAdjacencyStarts[best];
                        var s1 = topology.VertexAdjacencyStarts[best + 1];
                        var previousBest = best;
                        for (int i = s0; i < s1; ++i)
                        {
                            var neighbor = topology.AdjacentVertices[i];
                            var dot = Vector3.Dot(topology.Vertices[neighbor], d);
                            ++dots;
                            if (dot > bestDot) { bestDot = dot; best = neighbor; }
                        }
                        if (best == previousBest)
                            break;
                    }
                    return best;
                }
                for (int h = 0; h < hullSet.Hulls.Length; ++h)
                {
                    var topology = topologies[h];
                    var inverseScale = 1.0 / hullSet.MaxRadii[h];
                    int warm = 0;
                    var direction = Vector3.UnitY;
                    for (int q = 0; q < 30000; ++q)
                    {
                        //Half the queries take a small correlated step (mimicking refiner normal sequences), half jump randomly.
                        Vector3 NextUnit(float scale)
                        {
                            var candidate = direction + scale * new Vector3(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1);
                            var length = candidate.Length();
                            return length > 1e-6f ? candidate / length : Vector3.UnitX;
                        }
                        direction = (q & 1) == 0 ? NextUnit(2f) : NextUnit(0.05f);
                        var reference = HullSupportScalar.ComputeLocalSupport(hullSet.Hulls[h], direction);
                        var referenceDot = Vector3.Dot(reference, direction);
                        warm = ClimbCounted(topology, warm, direction, ref warmRounds, ref warmDots);
                        var warmDeficit = (referenceDot - Vector3.Dot(topology.Vertices[warm], direction)) * inverseScale;
                        var cold = ClimbCounted(topology, 0, direction, ref coldRounds, ref coldDots);
                        var coldDeficit = (referenceDot - Vector3.Dot(topology.Vertices[cold], direction)) * inverseScale;
                        if (warmDeficit > 0) ++warmWorse;
                        if (coldDeficit > 0) ++coldWorse;
                        if (warmDeficit > maxWarmDeficit) maxWarmDeficit = warmDeficit;
                        if (coldDeficit > maxColdDeficit) maxColdDeficit = coldDeficit;
                        ++queries;
                    }
                }
                Console.WriteLine($"climbprobe size {probeSize}: {queries} queries; warm deficit max {maxWarmDeficit:E2} ({warmWorse} nonzero), cold deficit max {maxColdDeficit:E2} ({coldWorse} nonzero); " +
                    $"warm avg rounds {(double)warmRounds / queries:F2} dots {(double)warmDots / queries:F1}, cold avg rounds {(double)coldRounds / queries:F2} dots {(double)coldDots / queries:F1}");
            }
        }
        if (mode is "benchhullscan")
        {
            //Hull size scan: engine wide vs frozen scalar baseline vs registered relaxed candidates across hull vertex counts.
            var scanCandidates = new List<IRelaxedHullPairTester>();
            foreach (var name in candidateNames)
            {
                var candidate = RelaxedCandidateRegistry.Find(name);
                if (candidate == null)
                {
                    Console.WriteLine($"Unknown candidate '{name}'. Available: {RelaxedCandidateRegistry.AvailableNames}");
                    return 2;
                }
                scanCandidates.Add(candidate);
            }
            var scanPairCount = countExplicit ? (int)count : 1 << 12;
            var results = new List<ThroughputRunner.HullScanResult>();
            foreach (var scanSize in sizes)
            {
                //Bigger hulls cost more per pair; scale reps down to keep trial durations comparable.
                var repsPerTrial = scanSize <= 8 ? 20 : scanSize <= 16 ? 12 : scanSize <= 32 ? 8 : scanSize <= 64 ? 5 : 3;
                results.Add(ThroughputRunner.RunHullScan(scanPairCount, repsPerTrial, trialCount: 7, seed: seed + 100 + scanSize, contactHeavy: false, scanSize, scanCandidates.ToArray()));
                results.Add(ThroughputRunner.RunHullScan(scanPairCount, repsPerTrial, trialCount: 7, seed: seed + 200 + scanSize, contactHeavy: true, scanSize, scanCandidates.ToArray()));
            }
            Console.WriteLine();
            Console.WriteLine("Hull size scan summary (ns/pair):");
            Console.WriteLine("size | actualVerts | profile | contact% | wide min/med | frozen scalar min/med | wide speedup min/med | precompute us/hull" +
                (scanCandidates.Count > 0 ? " | per-candidate min/med (vs wide)" : ""));
            foreach (var r in results)
            {
                var line = $"{r.TargetVertexCount,4} | {r.ActualMeanVertexCount,10:F1} | {(r.ContactHeavy ? "contact-heavy" : "mixed        ")} | {r.ContactFraction,7:P0} | " +
                    $"{r.Wide.minNs,7:F0}/{r.Wide.medianNs,-7:F0} | {r.FrozenScalar.minNs,7:F0}/{r.FrozenScalar.medianNs,-7:F0} | " +
                    $"{r.FrozenScalar.minNs / r.Wide.minNs,4:F2}x/{r.FrozenScalar.medianNs / r.Wide.medianNs:F2}x | {r.PrecomputeMicrosecondsPerHull,6:F1}";
                for (int c = 0; c < r.Candidates.Length; ++c)
                {
                    var (name, wideAnchor, candidate) = r.Candidates[c];
                    var (_, frozenDirect, candidateDirect) = r.CandidatesVsFrozen[c];
                    line += $" | {name}: {candidate.minNs:F0}/{candidate.medianNs:F0} (vs wide {candidate.minNs / wideAnchor.minNs:F2}x/{candidate.medianNs / wideAnchor.medianNs:F2}x, vs frozen {candidateDirect.minNs / frozenDirect.minNs:F2}x/{candidateDirect.medianNs / frozenDirect.medianNs:F2}x)";
                }
                Console.WriteLine(line);
            }
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        if (mode is "bench" or "all")
        {
            //Hot working set: everything stays in cache; measures pure compute throughput.
            //Streaming working set: inputs exceed cache; includes memory bandwidth effects like the real narrowphase batching regime.
            //Mixed profile includes plenty of misses (favors AoS per-pair early outs; wide bundles only early out when all lanes miss);
            //contact-heavy approximates a post-broadphase workload where most pairs are touching.
            const int hotPairs = 1 << 13;
            const int streamingPairs = 1 << 21;
            Console.WriteLine();
            ThroughputRunner.RunSphere(hotPairs, repsPerTrial: 4000, trialCount: 7, seed: seed + 1, "hot, mixed", false);
            ThroughputRunner.RunSphere(hotPairs, repsPerTrial: 4000, trialCount: 7, seed: seed + 1, "hot, contact-heavy", true);
            ThroughputRunner.RunSphere(streamingPairs, repsPerTrial: 4, trialCount: 7, seed: seed + 2, "streaming, mixed", false);
            ThroughputRunner.RunSphere(streamingPairs, repsPerTrial: 4, trialCount: 7, seed: seed + 2, "streaming, contact-heavy", true);
            ThroughputRunner.RunBox(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, mixed", false);
            ThroughputRunner.RunBox(hotPairs, repsPerTrial: 100, trialCount: 7, seed: seed + 3, "hot, contact-heavy", true);
            ThroughputRunner.RunBox(streamingPairs, repsPerTrial: 2, trialCount: 7, seed: seed + 4, "streaming, mixed", false);
            ThroughputRunner.RunBox(streamingPairs, repsPerTrial: 2, trialCount: 7, seed: seed + 4, "streaming, contact-heavy", true);
            ThroughputRunner.RunCylinder(hotPairs, repsPerTrial: 60, trialCount: 7, seed: seed + 9, "hot, mixed", false);
            ThroughputRunner.RunCylinder(hotPairs, repsPerTrial: 60, trialCount: 7, seed: seed + 10, "hot, contact-heavy", true);
            ThroughputRunner.RunCylinder(streamingPairs, repsPerTrial: 2, trialCount: 7, seed: seed + 11, "streaming, contact-heavy", true);
            //Hull-hull: hull geometry is shared from a pool, so all configs are cache-hot; the axes that matter are hull size and lane divergence.
            const int hullPairs = 1 << 12;
            ThroughputRunner.RunHull(hullPairs, repsPerTrial: 20, trialCount: 7, seed: seed + 5, "small hulls (8-16 pts), contact-heavy", true, 8, 16);
            ThroughputRunner.RunHull(hullPairs, repsPerTrial: 6, trialCount: 7, seed: seed + 6, "large hulls (40-64 pts), contact-heavy", true, 40, 64);
            ThroughputRunner.RunHull(hullPairs, repsPerTrial: 10, trialCount: 7, seed: seed + 7, "mixed hulls (4-64 pts), contact-heavy", true, 4, 64);
            ThroughputRunner.RunHull(hullPairs, repsPerTrial: 10, trialCount: 7, seed: seed + 8, "mixed hulls (4-64 pts), mixed", false, 4, 64);
            Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
        }
        return mismatches == 0 ? 0 : 1;
    }
}
