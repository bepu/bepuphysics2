using BepuUtilities;
using System.Diagnostics;
using System.Numerics;

namespace AosBaselines;

/// <summary>
/// Experiment runners for the warm-start schemes (A1 certificate+min-compose, A2 adopt-and-polish epilogue, B1
/// separated-band seeding) in the Track 1 hull-hull configuration. Modes: warmfuzz (per-frame dominance guarantee,
/// synthesized warm axes), warmseq (closed-loop temporally-coherent sequences vs pure c2c, SAT oracle ground truth),
/// warmpolish (A2 witness-consistency cells), warmb1 (separated-band seeding histograms), warmbench (interleaved
/// wall clock on stream workloads).
/// </summary>
public static class WarmStartStudy
{
    static float Percentile(List<float> values, double p)
    {
        if (values.Count == 0)
            return 0;
        var sorted = values.ToArray();
        Array.Sort(sorted);
        var index = (int)Math.Ceiling(p * sorted.Length) - 1;
        return sorted[Math.Clamp(index, 0, sorted.Length - 1)];
    }

    static (double mean, float p99, float max) Stats(List<float> values)
    {
        if (values.Count == 0)
            return (0, 0, 0);
        double sum = 0;
        float max = float.MinValue;
        foreach (var v in values)
        {
            sum += v;
            if (v > max)
                max = v;
        }
        return (sum / values.Count, Percentile(values, 0.99), max);
    }

    /// <summary>Rotates a unit direction by the given angle about a random perpendicular axis (Rodrigues).</summary>
    static Vector3 PerturbDirection(Random random, Vector3 n, float angle)
    {
        var axis = Vector3.Cross(n, random.UnitDirection() + new Vector3(1e-4f, 2e-4f, 3e-4f));
        var lengthSquared = axis.LengthSquared();
        axis = lengthSquared > 1e-12f ? axis / MathF.Sqrt(lengthSquared) : new Vector3(1f, 0f, 0f);
        var c = MathF.Cos(angle);
        var s = MathF.Sin(angle);
        return n * c + Vector3.Cross(axis, n) * s + axis * (Vector3.Dot(axis, n) * (1f - c));
    }

    //--------------------------------------------------------------------------------------------------
    // warmfuzz: per-frame Q1 dominance certification for A1 on i.i.d. poses with synthesized warm axes.
    //--------------------------------------------------------------------------------------------------
    public static int RunDominanceFuzz(HullSet hullSet, HullTopology[] topologies, int size, bool contactHeavy, int seed, long count, int maxReports)
    {
        var generator = new HullHullGenerator(seed + size + (contactHeavy ? 1 : 0), hullSet, contactHeavy);
        var warmRandom = new Random(seed * 7919 + size);
        string[] categoryNames = ["prev-output", "perturbed-c2c", "random-unit", "garbage", "NaN", "zero/tiny", "non-unit"];
        var categoryCounts = new long[7];
        var categoryCerts = new long[7];
        var categoryAdopts = new long[7];
        var categoryRejected = new long[7];
        long violations = 0, bitwiseMismatches = 0, nanOutputs = 0;
        var previousOutput = Vector3.UnitY;
        var watch = Stopwatch.StartNew();
        for (long i = 0; i < count; ++i)
        {
            var pairCase = generator.Next();
            var topologyA = topologies[pairCase.A];
            var topologyB = topologies[pairCase.B];
            var setup = WarmStartSchemes.CreateSetup(topologyA, topologyB, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, pairCase.SpeculativeMargin);
            //Independent vanilla c2c run: the reference the wrapper must never underperform (clamped) and must match
            //bitwise whenever the compose is a no-op.
            WarmStartSchemes.RunC2C<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, out var c2cDepth, out var c2cNormal, out _);
            var category = (int)(i % 7);
            var warmAxis = category switch
            {
                0 => previousOutput,
                1 => PerturbDirection(warmRandom, c2cNormal, warmRandom.LogUniform(1e-4f, 0.3f)),
                2 => warmRandom.UnitDirection(),
                //Garbage: huge finite components (renormalizable => a direction) or overflow-to-inf components (must be rejected).
                3 => warmRandom.UnitDirection() * (warmRandom.Next(2) == 0 ? 1e12f : 1e30f),
                4 => warmRandom.Next(2) == 0 ? new Vector3(float.NaN, 1f, 0f) : new Vector3(float.NaN, float.NaN, float.NaN),
                5 => warmRandom.Next(2) == 0 ? default : new Vector3(1e-25f, -1e-25f, 1e-26f),
                _ => warmRandom.UnitDirection() * warmRandom.LogUniform(1e-5f, 1e5f),
            };
            var a1 = WarmStartSchemes.RunA1<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, warmAxis);
            ++categoryCounts[category];
            if (a1.CertificateFired)
                ++categoryCerts[category];
            if (a1.WarmAdopted)
                ++categoryAdopts[category];
            if (!a1.EvaluatedWarm)
                ++categoryRejected[category];

            var theta = setup.DepthThreshold;
            var clampedC2C = MathF.Max(c2cDepth, theta);
            var clampedA1 = MathF.Max(a1.Depth, theta);
            //NaN-safe direction: a NaN clampedA1 fails <= and counts as a violation.
            if (!(clampedA1 <= clampedC2C))
            {
                ++violations;
                if (violations <= maxReports)
                    Console.WriteLine($"    [warmfuzz] DOMINANCE VIOLATION case {i} cat {categoryNames[category]}: A1 {a1.Depth:G9} vs c2c {c2cDepth:G9} (theta {theta:G6})");
            }
            if (float.IsNaN(a1.Depth) || !float.IsFinite(a1.Normal.X + a1.Normal.Y + a1.Normal.Z))
            {
                ++nanOutputs;
                if (nanOutputs <= maxReports)
                    Console.WriteLine($"    [warmfuzz] NaN OUTPUT case {i} cat {categoryNames[category]}: depth {a1.Depth}, normal {a1.Normal}");
            }
            if (!a1.CertificateFired && !a1.WarmAdopted)
            {
                //Compose was a no-op: the wrapper's result must be bit-identical to the independent c2c run.
                if (BitConverter.SingleToUInt32Bits(a1.Depth) != BitConverter.SingleToUInt32Bits(c2cDepth)
                    || BitConverter.SingleToUInt32Bits(a1.Normal.X) != BitConverter.SingleToUInt32Bits(c2cNormal.X)
                    || BitConverter.SingleToUInt32Bits(a1.Normal.Y) != BitConverter.SingleToUInt32Bits(c2cNormal.Y)
                    || BitConverter.SingleToUInt32Bits(a1.Normal.Z) != BitConverter.SingleToUInt32Bits(c2cNormal.Z))
                {
                    ++bitwiseMismatches;
                    if (bitwiseMismatches <= maxReports)
                        Console.WriteLine($"    [warmfuzz] BITWISE MISMATCH case {i} cat {categoryNames[category]}: A1 {a1.Depth:G9}/{a1.Normal} vs c2c {c2cDepth:G9}/{c2cNormal}");
                }
            }
            previousOutput = c2cNormal;
        }
        watch.Stop();
        Console.WriteLine($"warmfuzz size {size} {(contactHeavy ? "contact-heavy" : "mixed")} seed {seed}: {count} cases in {watch.Elapsed.TotalSeconds:F1}s; " +
            $"violations {violations}, bitwise mismatches {bitwiseMismatches}, NaN outputs {nanOutputs}");
        for (int c = 0; c < 7; ++c)
            Console.WriteLine($"    {categoryNames[c],-14}: {categoryCounts[c]} cases, cert {(double)categoryCerts[c] / Math.Max(1, categoryCounts[c]):P2}, " +
                $"adopted {(double)categoryAdopts[c] / Math.Max(1, categoryCounts[c]):P2}, guard-rejected {(double)categoryRejected[c] / Math.Max(1, categoryCounts[c]):P2}");
        return (int)Math.Min(int.MaxValue, violations + bitwiseMismatches + nanOutputs);
    }

    //--------------------------------------------------------------------------------------------------
    // warmseq: closed-loop coherent sequences, A1 vs pure c2c, SAT oracle ground truth per frame.
    //--------------------------------------------------------------------------------------------------
    public static void RunSequenceCell(HullSet set, HullTopology[] topologies, int size, SequenceScenario scenario, float delta,
        int pairs, int framesPerPair, int seed)
    {
        var random = new Random(seed * 31 + size * 7 + (int)scenario * 131 + (int)(delta * 1e5f));
        long framesTotal = 0, penFrames = 0, sepInMarginFrames = 0, rejectedFrames = 0;
        long certFired = 0, certUnsound = 0, adoptedCount = 0;
        long dominanceViolations = 0, teleports = 0, teleportDominanceViolations = 0, teleportSweepViolations = 0;
        var sweepsC2C = new List<float>();
        var sweepsA1 = new List<float>();
        //Arm 0 = c2c, arm 1 = A1.
        var excess = new List<float>[2] { new(), new() };
        var missCounts = new long[2];
        long evaluatableFrames = 0;
        var missRunLengths = new List<float>[2] { new(), new() };
        var currentRuns = new int[2];
        var watch = Stopwatch.StartNew();

        void CloseRuns()
        {
            for (int arm = 0; arm < 2; ++arm)
            {
                if (currentRuns[arm] > 0)
                {
                    missRunLengths[arm].Add(currentRuns[arm]);
                    currentRuns[arm] = 0;
                }
            }
        }

        for (int p = 0; p < pairs; ++p)
        {
            var indexA = random.Next(set.Hulls.Length);
            var indexB = random.Next(set.Hulls.Length);
            var scale = MathF.Min(set.MaxRadii[indexA], set.MaxRadii[indexB]);
            var margin = 0.25f * scale;
            var frames = CoherentSequences.Generate(random, set, indexA, indexB, scenario, framesPerPair, delta, margin);
            var topologyA = topologies[indexA];
            var topologyB = topologies[indexB];
            var warmWorld = default(Vector3);
            for (int t = 0; t < frames.Length; ++t)
            {
                ref var frame = ref frames[t];
                ++framesTotal;
                var setup = WarmStartSchemes.CreateSetup(topologyA, topologyB, frame.OffsetB, frame.OrientationA, frame.OrientationB, margin);
                //Oracle: exhaustive SAT. Exact on penetration; sound reject certificates; the separated-in-margin
                //band's depth is a known underestimate of the gap, so those frames are excluded from excess stats.
                SatHullPairTester.Test(topologyA, topologyB, ref set.Hulls[indexA], ref set.Hulls[indexB],
                    margin, frame.OffsetB, frame.OrientationA, frame.OrientationB, usePolish: false, out _);
                var rejected = SatHullPairTester.LastWinnerType < 0;
                var satDepth = SatHullPairTester.LastDepth;
                var pen = !rejected && satDepth >= 0f;
                if (pen)
                    ++penFrames;
                else if (rejected)
                    ++rejectedFrames;
                else
                    ++sepInMarginFrames;

                var climbs0 = HullSupportScalarHillclimbCounted.Climbs;
                WarmStartSchemes.RunC2C<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, out var c2cDepth, out _, out _);
                var swC = (int)((HullSupportScalarHillclimbCounted.Climbs - climbs0) >> 1);

                //A1, closed loop: warm axis is A1's OWN previous-frame output, carried in world space.
                var warmLocal = ScalarMath.TransformByTransposed(warmWorld, setup.RB);
                var a1 = WarmStartSchemes.RunA1<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, warmLocal);
                var swA = a1.RefinerSweeps + (a1.EvaluatedWarm ? 1 : 0);
                Matrix3x3.Transform(a1.Normal, setup.RB, out warmWorld);

                if (a1.CertificateFired)
                {
                    ++certFired;
                    if (pen)
                        ++certUnsound;
                }
                if (a1.WarmAdopted)
                    ++adoptedCount;
                sweepsC2C.Add(swC);
                sweepsA1.Add(swA);

                var theta = setup.DepthThreshold;
                var clampedC2C = MathF.Max(c2cDepth, theta);
                var clampedA1 = MathF.Max(a1.Depth, theta);
                if (!(clampedA1 <= clampedC2C))
                    ++dominanceViolations;
                if (frame.Teleport)
                {
                    ++teleports;
                    if (!(clampedA1 <= clampedC2C))
                        ++teleportDominanceViolations;
                    if (swA > swC + 1)
                        ++teleportSweepViolations;
                }

                if (pen || rejected)
                {
                    ++evaluatableFrames;
                    var truthClamped = pen ? MathF.Max(satDepth, theta) : theta;
                    var tolerance = MathF.Max(1e-5f * scale, 1e-4f * (pen ? MathF.Abs(satDepth) : MathF.Abs(theta)));
                    Span<float> clamped = [clampedC2C, clampedA1];
                    for (int arm = 0; arm < 2; ++arm)
                    {
                        var e = clamped[arm] - truthClamped;
                        excess[arm].Add(MathF.Max(0f, e) / scale);
                        var miss = e > tolerance;
                        if (miss)
                        {
                            ++missCounts[arm];
                            ++currentRuns[arm];
                        }
                        else if (currentRuns[arm] > 0)
                        {
                            missRunLengths[arm].Add(currentRuns[arm]);
                            currentRuns[arm] = 0;
                        }
                    }
                }
                else
                {
                    CloseRuns();
                }
            }
            CloseRuns();
        }
        watch.Stop();

        var (excessMeanC, excessP99C, excessMaxC) = Stats(excess[0]);
        var (excessMeanA, excessP99A, excessMaxA) = Stats(excess[1]);
        var (runMeanC, _, runMaxC) = Stats(missRunLengths[0]);
        var (runMeanA, _, runMaxA) = Stats(missRunLengths[1]);
        Console.WriteLine($"warmseq size {size} {scenario} delta {delta:G3} ({pairs}x{framesPerPair}, {watch.Elapsed.TotalSeconds:F1}s): " +
            $"pen {(double)penFrames / framesTotal:P1}, sep-in-margin {(double)sepInMarginFrames / framesTotal:P1}, beyond-margin {(double)rejectedFrames / framesTotal:P1}");
        Console.WriteLine($"    cert fired {(double)certFired / framesTotal:P2} of frames (unsound {certUnsound}); warm adopted {(double)adoptedCount / framesTotal:P2}; " +
            $"dominance violations {dominanceViolations}");
        Console.WriteLine($"    sweeps mean|p99: c2c {Stats(sweepsC2C).mean:F2}|{Percentile(sweepsC2C, 0.99):F0}, A1 {Stats(sweepsA1).mean:F2}|{Percentile(sweepsA1, 0.99):F0}");
        Console.WriteLine($"    excess-vs-oracle (evaluatable {evaluatableFrames}): dwell>tol c2c {(double)missCounts[0] / Math.Max(1, evaluatableFrames):P2}, A1 {(double)missCounts[1] / Math.Max(1, evaluatableFrames):P2}; " +
            $"clamped excess/scale mean|p99|max c2c {excessMeanC:E2}|{excessP99C:E2}|{excessMaxC:E2}, A1 {excessMeanA:E2}|{excessP99A:E2}|{excessMaxA:E2}");
        Console.WriteLine($"    miss persistence (consecutive-miss runs): c2c {missRunLengths[0].Count} runs mean {runMeanC:F2} max {runMaxC:F0}; " +
            $"A1 {missRunLengths[1].Count} runs mean {runMeanA:F2} max {runMaxA:F0}");
        if (teleports > 0)
            Console.WriteLine($"    teleports {teleports}: dominance violations {teleportDominanceViolations}, sweep-bound (>c2c+1) violations {teleportSweepViolations}");
    }

    //--------------------------------------------------------------------------------------------------
    // warmpolish: A2 adopt-and-polish cells (i.i.d. penetrating cases, synthesized warm axes, SAT oracle).
    //--------------------------------------------------------------------------------------------------
    public static void RunA2Cell(HullSet set, HullTopology[] topologies, int size, bool contactHeavy, int seed, int targetPenetrating, int maxReports)
    {
        var generator = new HullHullGenerator(seed + size + (contactHeavy ? 1 : 0), set, contactHeavy);
        var warmRandom = new Random(seed * 4099 + size);
        int[] ks = [2, 4, 6];
        const int Sources = 2; //0 prev-output, 1 perturbed-truth
        string[] sourceNames = ["prev-output", "perturbed-truth"];
        const int Arms = 5;    //0 c2c, 1 A1, 2..4 A2 K=2/4/6
        string[] armNames = ["c2c", "A1", "A2K2", "A2K4", "A2K6"];
        var hits = new long[Sources, Arms];
        var excess = new List<float>[Sources, Arms];
        var tierCounts = new long[3];
        var tierMisses = new long[Sources, Arms, 3];
        var adoptions = new long[Sources];
        //Witness residuals on the adoption-triggered subset (where the arms actually differ), one list per arm per source;
        //the c2c residual on the same subset is the baseline.
        var residuals = new List<float>[Sources, Arms];
        var legSweeps = new List<float>[Sources, 3];
        var legNatural = new long[Sources, 3];
        var c2cSweeps = new List<float>();
        long certOnPenetrating = 0, certKnifeEdge = 0, legWorseThanSeed = 0;
        double certMaxGap = 0;
        for (int s = 0; s < Sources; ++s)
        {
            for (int a = 0; a < Arms; ++a)
            {
                excess[s, a] = new List<float>();
                residuals[s, a] = new List<float>();
            }
            for (int k = 0; k < 3; ++k)
                legSweeps[s, k] = new List<float>();
        }
        long totalCases = 0, penetrating = 0;
        var previousOutput = Vector3.UnitY;
        var watch = Stopwatch.StartNew();

        while (penetrating < targetPenetrating && totalCases < 80L * targetPenetrating)
        {
            var pairCase = generator.Next();
            ++totalCases;
            var topologyA = topologies[pairCase.A];
            var topologyB = topologies[pairCase.B];
            SatHullPairTester.Test(topologyA, topologyB, ref set.Hulls[pairCase.A], ref set.Hulls[pairCase.B],
                pairCase.SpeculativeMargin, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, usePolish: false, out _);
            if (SatHullPairTester.LastWinnerType < 0)
                continue;
            var satDepth = SatHullPairTester.LastDepth;
            if (satDepth < 0f)
                continue;
            var satNormal = SatHullPairTester.LastLocalNormal;
            ++penetrating;
            var scale = MathF.Min(set.MaxRadii[pairCase.A], set.MaxRadii[pairCase.B]);
            var tolerance = MathF.Max(1e-5f * scale, 1e-4f * MathF.Abs(satDepth));
            var tier = satDepth <= 0.1f * scale ? 0 : satDepth <= 0.25f * scale ? 1 : 2;
            ++tierCounts[tier];
            var setup = WarmStartSchemes.CreateSetup(topologyA, topologyB, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, pairCase.SpeculativeMargin);

            var climbs0 = HullSupportScalarHillclimbCounted.Climbs;
            WarmStartSchemes.RunC2C<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, out var c2cDepth, out var c2cNormal, out var c2cWitness);
            c2cSweeps.Add((int)((HullSupportScalarHillclimbCounted.Climbs - climbs0) >> 1));
            var c2cExcess = MathF.Max(0f, c2cDepth - satDepth) / scale;
            var c2cHit = c2cDepth - satDepth <= tolerance;

            for (int s = 0; s < Sources; ++s)
            {
                var warmAxis = s == 0 ? previousOutput : PerturbDirection(warmRandom, satNormal, warmRandom.LogUniform(2e-3f, 0.2f));
                //c2c arm stats are per source for aligned counting (identical values across sources).
                excess[s, 0].Add(c2cExcess);
                if (c2cHit)
                    ++hits[s, 0];
                else
                    ++tierMisses[s, 0, tier];

                if (!WarmStartSchemes.TryValidateAxis(warmAxis, out var w))
                {
                    //Arms inherit c2c wholesale.
                    for (int a = 1; a < Arms; ++a)
                    {
                        excess[s, a].Add(c2cExcess);
                        if (c2cHit)
                            ++hits[s, a];
                        else
                            ++tierMisses[s, a, tier];
                    }
                    continue;
                }
                var warmDepth = WarmStartSchemes.EvaluateAxis(topologyA, topologyB, setup, w, out _, out var warmSupportOnB);
                if (warmDepth < setup.DepthThreshold)
                {
                    //Interval depths upper-bound the true depth, so a certificate on a SAT-penetrating case implies the
                    //oracle overstated (merged-face granularity / knife-edge); beyond tolerance it would be a real bug.
                    if (warmDepth < setup.DepthThreshold - tolerance)
                        ++certOnPenetrating;
                    else
                        ++certKnifeEdge;
                    var gap = (setup.DepthThreshold - warmDepth) / scale;
                    if (gap > certMaxGap)
                        certMaxGap = gap;
                }
                var adopted = warmDepth < c2cDepth;
                var a1Depth = adopted ? warmDepth : c2cDepth;
                var a1Normal = adopted ? w : c2cNormal;
                var a1Witness = adopted ? warmSupportOnB : c2cWitness;
                var a1Excess = MathF.Max(0f, a1Depth - satDepth) / scale;
                var a1Hit = a1Depth - satDepth <= tolerance;
                excess[s, 1].Add(a1Excess);
                if (a1Hit)
                    ++hits[s, 1];
                else
                    ++tierMisses[s, 1, tier];

                if (adopted)
                {
                    ++adoptions[s];
                    residuals[s, 0].Add(WarmStartSchemes.WitnessResidual(topologyA, topologyB, setup, c2cDepth, c2cNormal, c2cWitness) / scale);
                    residuals[s, 1].Add(WarmStartSchemes.WitnessResidual(topologyA, topologyB, setup, a1Depth, a1Normal, a1Witness) / scale);
                    for (int k = 0; k < 3; ++k)
                    {
                        var legClimbs0 = HullSupportScalarHillclimbCounted.Climbs;
                        WarmStartSchemes.RunRefiner<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, w, ks[k],
                            out var legDepth, out var legNormal, out var legWitness);
                        var legSweepCount = (int)((HullSupportScalarHillclimbCounted.Climbs - legClimbs0) >> 1);
                        legSweeps[s, k].Add(legSweepCount);
                        if (legSweepCount <= ks[k])
                            ++legNatural[s, k];
                        if (legDepth > warmDepth + 1e-6f * scale)
                            ++legWorseThanSeed;
                        //Floor at the A1 min; leg witness adopted either way (the repair is the point).
                        var a2Depth = legDepth < a1Depth ? legDepth : a1Depth;
                        var a2Normal = legDepth < a1Depth ? legNormal : a1Normal;
                        var a2Excess = MathF.Max(0f, a2Depth - satDepth) / scale;
                        excess[s, 2 + k].Add(a2Excess);
                        if (a2Depth - satDepth <= tolerance)
                            ++hits[s, 2 + k];
                        else
                            ++tierMisses[s, 2 + k, tier];
                        residuals[s, 2 + k].Add(WarmStartSchemes.WitnessResidual(topologyA, topologyB, setup, a2Depth, a2Normal, legWitness) / scale);
                    }
                }
                else
                {
                    for (int k = 0; k < 3; ++k)
                    {
                        excess[s, 2 + k].Add(a1Excess);
                        if (a1Hit)
                            ++hits[s, 2 + k];
                        else
                            ++tierMisses[s, 2 + k, tier];
                    }
                }
            }
            previousOutput = c2cNormal;
        }
        watch.Stop();

        Console.WriteLine($"warmpolish size {size} {(contactHeavy ? "contact-heavy" : "mixed")} seed {seed}: {totalCases} cases -> {penetrating} penetrating ({watch.Elapsed.TotalSeconds:F1}s); " +
            $"tiers shallow/mid/deep {(double)tierCounts[0] / Math.Max(1, penetrating):P0}/{(double)tierCounts[1] / Math.Max(1, penetrating):P0}/{(double)tierCounts[2] / Math.Max(1, penetrating):P0}; " +
            $"c2c sweeps mean {Stats(c2cSweeps).mean:F2}; sanity: cert-on-penetrating beyond-tol {certOnPenetrating}, knife-edge {certKnifeEdge} (max gap/scale {certMaxGap:E2}), leg-worse-than-seed {legWorseThanSeed}");
        for (int s = 0; s < Sources; ++s)
        {
            Console.WriteLine($"    source {sourceNames[s]}: adoption {(double)adoptions[s] / Math.Max(1, penetrating):P2} ({adoptions[s]})");
            var hitLine = "        hit%: ";
            for (int a = 0; a < Arms; ++a)
                hitLine += $"{armNames[a]} {(double)hits[s, a] / Math.Max(1, penetrating):P2}{(a < Arms - 1 ? " -> " : "")}";
            Console.WriteLine(hitLine);
            var excessLine = "        excess/scale mean|p99: ";
            for (int a = 0; a < Arms; ++a)
            {
                var (mean, p99, _) = Stats(excess[s, a]);
                excessLine += $"{armNames[a]} {mean:E2}|{p99:E2}{(a < Arms - 1 ? ", " : "")}";
            }
            Console.WriteLine(excessLine);
            for (int tier = 0; tier < 3; ++tier)
            {
                if (tierCounts[tier] == 0)
                    continue;
                var line = $"        tier {(tier == 0 ? "shallow" : tier == 1 ? "mid    " : "deep   ")} miss%: ";
                for (int a = 0; a < Arms; ++a)
                    line += $"{armNames[a]} {(double)tierMisses[s, a, tier] / tierCounts[tier]:P2}{(a < Arms - 1 ? " -> " : "")}";
                Console.WriteLine(line);
            }
            var residualLine = "        witness residual/scale on adopted subset mean|p99|max: ";
            for (int a = 0; a < Arms; ++a)
            {
                var (mean, p99, max) = Stats(residuals[s, a]);
                residualLine += $"{armNames[a]} {mean:E2}|{p99:E2}|{max:E2}{(a < Arms - 1 ? ", " : "")}";
            }
            Console.WriteLine(residualLine);
            var legLine = "        leg sweeps on adopted mean|p99 (natural-termination%): ";
            for (int k = 0; k < 3; ++k)
            {
                var (mean, p99, _) = Stats(legSweeps[s, k]);
                legLine += $"K{ks[k]} {mean:F2}|{p99:F0} ({(double)legNatural[s, k] / Math.Max(1, adoptions[s]):P0}){(k < 2 ? ", " : "")}";
            }
            Console.WriteLine(legLine);
        }
    }

    //--------------------------------------------------------------------------------------------------
    // warmb1: separated-band seeding — gate rates, iteration/sweep histograms seeded vs c2c, cap-exit rate,
    // seed-invariance spot check.
    //--------------------------------------------------------------------------------------------------
    public static void RunB1Cell(HullSet set, HullTopology[] topologies, int size, bool contactHeavy, int seed, long count, int maxReports)
    {
        var generator = new HullHullGenerator(seed + size + (contactHeavy ? 1 : 0), set, contactHeavy);
        var warmRandom = new Random(seed * 6151 + size);
        const int Sources = 3; //0 self (exact zero-delta coherence), 1 perturbed-self, 2 prev-output
        string[] sourceNames = ["self-exact", "perturbed-self", "prev-output"];
        var certs = new long[Sources];
        var certAgrees = new long[Sources];
        var seeded = new long[Sources];
        var fellThroughPen = new long[Sources];
        var guardRejected = new long[Sources];
        var seededSweeps = new List<float>[Sources];
        var pairedC2CSweeps = new List<float>[Sources];
        var capExitsSeeded = new long[Sources];
        var capExitsC2CPaired = new long[Sources];
        var depthDiffs = new List<float>[Sources];
        var diffViolations = new long[Sources];
        var diffViolationsCap = new long[Sources];
        var bothCertified = new long[Sources];
        var c2cBeyondSeededAccept = new long[Sources];
        var seededSepC2CPen = new long[Sources];
        var seededBeyondC2CAccept = new long[Sources];
        for (int s = 0; s < Sources; ++s)
        {
            seededSweeps[s] = new List<float>();
            pairedC2CSweeps[s] = new List<float>();
            depthDiffs[s] = new List<float>();
        }
        long separatedBand = 0;
        var bandC2CSweeps = new List<float>();
        long bandC2CCapExits = 0;
        var previousOutput = Vector3.UnitY;
        var watch = Stopwatch.StartNew();

        for (long i = 0; i < count; ++i)
        {
            var pairCase = generator.Next();
            var topologyA = topologies[pairCase.A];
            var topologyB = topologies[pairCase.B];
            var scale = MathF.Min(set.MaxRadii[pairCase.A], set.MaxRadii[pairCase.B]);
            var setup = WarmStartSchemes.CreateSetup(topologyA, topologyB, pairCase.OffsetB, pairCase.OrientationA, pairCase.OrientationB, pairCase.SpeculativeMargin);
            var climbs0 = HullSupportScalarHillclimbCounted.Climbs;
            WarmStartSchemes.RunC2C<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, out var c2cDepth, out var c2cNormal, out _);
            var swC = (int)((HullSupportScalarHillclimbCounted.Climbs - climbs0) >> 1);
            if (c2cDepth < 0f)
            {
                ++separatedBand;
                bandC2CSweeps.Add(swC);
                if (swC >= 26)
                    ++bandC2CCapExits;
            }
            for (int s = 0; s < Sources; ++s)
            {
                var warmAxis = s switch
                {
                    0 => c2cNormal,
                    1 => PerturbDirection(warmRandom, c2cNormal, warmRandom.LogUniform(1e-3f, 0.1f)),
                    _ => previousOutput,
                };
                var b1 = WarmStartSchemes.RunB1<HullSupportScalarHillclimbCounted>(topologyA, topologyB, setup, warmAxis);
                if (!b1.EvaluatedWarm)
                {
                    ++guardRejected[s];
                }
                else if (b1.CertificateFired)
                {
                    ++certs[s];
                    if (c2cDepth < setup.DepthThreshold)
                        ++certAgrees[s];
                }
                else if (b1.Seeded)
                {
                    ++seeded[s];
                    seededSweeps[s].Add(b1.RefinerSweeps);
                    pairedC2CSweeps[s].Add(swC);
                    if (b1.RefinerSweeps >= 26)
                        ++capExitsSeeded[s];
                    if (swC >= 26)
                        ++capExitsC2CPaired[s];
                    var theta = setup.DepthThreshold;
                    if (c2cDepth < 0f)
                    {
                        var seededBelow = b1.Depth < theta;
                        var c2cBelow = c2cDepth < theta;
                        if (seededBelow && c2cBelow)
                        {
                            //Both certified beyond-margin: one clamped equivalence class, no observable difference.
                            ++bothCertified[s];
                        }
                        else if (seededBelow)
                        {
                            ++seededBeyondC2CAccept[s];
                        }
                        else if (c2cBelow)
                        {
                            ++c2cBeyondSeededAccept[s];
                        }
                        else
                        {
                            //Both accepted-separated: the seed-invariance theorem's domain (modulo cap exits).
                            var diff = MathF.Abs(b1.Depth - c2cDepth);
                            depthDiffs[s].Add(diff / scale);
                            if (diff > MathF.Max(4f * setup.ConvergenceThreshold, 1e-6f * scale))
                            {
                                var capInvolved = b1.RefinerSweeps >= 26 || swC >= 26;
                                if (capInvolved)
                                    ++diffViolationsCap[s];
                                else
                                {
                                    ++diffViolations[s];
                                    if (diffViolations[s] <= maxReports)
                                        Console.WriteLine($"    [warmb1] seed-dependence beyond tolerance (epsilon-terminated) case {i} src {sourceNames[s]}: seeded {b1.Depth:G6} ({b1.RefinerSweeps} sweeps) vs c2c {c2cDepth:G6} ({swC} sweeps), conv {setup.ConvergenceThreshold:G3}");
                                }
                            }
                        }
                    }
                    else
                    {
                        //Seeded run certified separation the c2c run missed: disposition flip toward the correct side
                        //(a negative one-sweep depth is proof of separation).
                        ++seededSepC2CPen[s];
                    }
                }
                else
                {
                    ++fellThroughPen[s];
                }
            }
            previousOutput = c2cNormal;
        }
        watch.Stop();

        Console.WriteLine($"warmb1 size {size} {(contactHeavy ? "contact-heavy" : "mixed")} seed {seed}: {count} cases ({watch.Elapsed.TotalSeconds:F1}s); " +
            $"c2c separated band (depth<0) {(double)separatedBand / count:P1}, band c2c sweeps mean|p99 {Stats(bandC2CSweeps).mean:F2}|{Percentile(bandC2CSweeps, 0.99):F0}, band c2c cap-exits {bandC2CCapExits}");
        for (int s = 0; s < Sources; ++s)
        {
            var (seedMean, seedP99, _) = Stats(seededSweeps[s]);
            var (pairMean, pairP99, _) = Stats(pairedC2CSweeps[s]);
            var (diffMean, diffP99, diffMax) = Stats(depthDiffs[s]);
            Console.WriteLine($"    source {sourceNames[s]}: cert {certs[s]} (c2c-agrees {certAgrees[s]}), gate-passed(seeded) {seeded[s]} ({(double)seeded[s] / count:P2} of all, {(double)seeded[s] / Math.Max(1, separatedBand):P1} of band), " +
                $"pen-fallthrough {fellThroughPen[s]}, guard-rejected {guardRejected[s]}");
            Console.WriteLine($"        seeded sweeps mean|p50|p99 {seedMean:F2}|{Percentile(seededSweeps[s], 0.5):F0}|{seedP99:F0} vs paired c2c {pairMean:F2}|{Percentile(pairedC2CSweeps[s], 0.5):F0}|{pairP99:F0}; " +
                $"cap-exits seeded {capExitsSeeded[s]} vs c2c {capExitsC2CPaired[s]}");
            Console.WriteLine($"        quality (both accepted-separated, {depthDiffs[s].Count}): |depth diff|/scale mean|p99|max {diffMean:E2}|{diffP99:E2}|{diffMax:E2}; beyond-tolerance: epsilon-terminated {diffViolations[s]}, cap-involved {diffViolationsCap[s]}; " +
                $"dispositions: both-certified {bothCertified[s]}, seeded-cert/c2c-accept {seededBeyondC2CAccept[s]}, c2c-cert/seeded-accept {c2cBeyondSeededAccept[s]}, seeded-sep/c2c-pen {seededSepC2CPen[s]}");
        }
    }

    //--------------------------------------------------------------------------------------------------
    // warmbench: interleaved wall clock on stream workloads (axis search + setup only; the manifold path is
    // identical across arms and certificate-skipped frames produce no contacts on any arm).
    //--------------------------------------------------------------------------------------------------
    public struct BenchFrame
    {
        public int A, B;
        public Vector3 OffsetB;
        public Quaternion OrientationA, OrientationB;
        public float SpeculativeMargin;
        public bool ResetWarm;
    }

    public static void RunWarmBench(int size, int seed, int trialCount)
    {
        var setupRandom = new Random(seed + size);
        using var hullSet = HullSet.CreateSized(setupRandom, hullCount: 16, size);
        var topologies = HullTopology.CreateForSet(hullSet, out _);

        //Workload 1: i.i.d. mixed stream — decorrelated warm data carried closed-loop; measures the pure +1 overhead
        //and whatever accidental certificates fire.
        var generator = new HullHullGenerator(seed + 1000 + size, hullSet, contactHeavy: false);
        var iid = new BenchFrame[4096];
        for (int i = 0; i < iid.Length; ++i)
        {
            var c = generator.Next();
            iid[i] = new BenchFrame { A = c.A, B = c.B, OffsetB = c.OffsetB, OrientationA = c.OrientationA, OrientationB = c.OrientationB, SpeculativeMargin = c.SpeculativeMargin, ResetWarm = i == 0 };
        }
        //Workloads 2/3: coherent sequences, pair-major (warm resets at each pair's first frame).
        var coherent = BuildSequenceWorkload(hullSet, seed + 2000 + size, pairs: 64, framesPerPair: 64, mixRestSlideDeepen: true);
        var separatedHeavy = BuildSequenceWorkload(hullSet, seed + 3000 + size, pairs: 64, framesPerPair: 64, mixRestSlideDeepen: false);

        RunWorkload("iid-mixed", hullSet, topologies, iid, trialCount);
        RunWorkload("coherent-contact", hullSet, topologies, coherent, trialCount);
        RunWorkload("separated-heavy", hullSet, topologies, separatedHeavy, trialCount);
        Console.WriteLine($"(sink: {ThroughputRunner.Sink})");
    }

    static BenchFrame[] BuildSequenceWorkload(HullSet set, int seed, int pairs, int framesPerPair, bool mixRestSlideDeepen)
    {
        var random = new Random(seed);
        var stream = new BenchFrame[pairs * framesPerPair];
        var index = 0;
        for (int p = 0; p < pairs; ++p)
        {
            var indexA = random.Next(set.Hulls.Length);
            var indexB = random.Next(set.Hulls.Length);
            var scale = MathF.Min(set.MaxRadii[indexA], set.MaxRadii[indexB]);
            var margin = 0.25f * scale;
            var scenario = mixRestSlideDeepen
                ? (p % 3) switch { 0 => SequenceScenario.Rest, 1 => SequenceScenario.Slide, _ => SequenceScenario.Deepen }
                : SequenceScenario.Hover;
            var frames = CoherentSequences.Generate(random, set, indexA, indexB, scenario, framesPerPair, delta: 0.005f, margin);
            for (int t = 0; t < frames.Length; ++t)
            {
                stream[index++] = new BenchFrame
                {
                    A = indexA,
                    B = indexB,
                    OffsetB = frames[t].OffsetB,
                    OrientationA = frames[t].OrientationA,
                    OrientationB = frames[t].OrientationB,
                    SpeculativeMargin = margin,
                    ResetWarm = t == 0,
                };
            }
        }
        return stream;
    }

    static void RunWorkload(string name, HullSet set, HullTopology[] topologies, BenchFrame[] stream, int trialCount)
    {
        float StreamC2C<TSupport>() where TSupport : IScalarSupportFinder<HillclimbHull>
        {
            float sink = 0;
            for (int i = 0; i < stream.Length; ++i)
            {
                ref var f = ref stream[i];
                var setup = WarmStartSchemes.CreateSetup(topologies[f.A], topologies[f.B], f.OffsetB, f.OrientationA, f.OrientationB, f.SpeculativeMargin);
                WarmStartSchemes.RunC2C<TSupport>(topologies[f.A], topologies[f.B], setup, out var depth, out _, out _);
                sink += depth;
            }
            return sink;
        }
        float StreamWarm<TSupport>(bool useB1, Action<WarmSchemeResult> observer) where TSupport : IScalarSupportFinder<HillclimbHull>
        {
            float sink = 0;
            var warmWorld = default(Vector3);
            for (int i = 0; i < stream.Length; ++i)
            {
                ref var f = ref stream[i];
                if (f.ResetWarm)
                    warmWorld = default;
                var setup = WarmStartSchemes.CreateSetup(topologies[f.A], topologies[f.B], f.OffsetB, f.OrientationA, f.OrientationB, f.SpeculativeMargin);
                var warmLocal = ScalarMath.TransformByTransposed(warmWorld, setup.RB);
                var r = useB1
                    ? WarmStartSchemes.RunB1<TSupport>(topologies[f.A], topologies[f.B], setup, warmLocal)
                    : WarmStartSchemes.RunA1<TSupport>(topologies[f.A], topologies[f.B], setup, warmLocal);
                sink += r.Depth;
                Matrix3x3.Transform(r.Normal, setup.RB, out warmWorld);
                observer?.Invoke(r);
            }
            return sink;
        }

        //Untimed characterization pass with the counting finder: fire rates and sweep means per arm.
        {
            var climbs0 = HullSupportScalarHillclimbCounted.Climbs;
            ThroughputRunner.Sink += StreamC2C<HullSupportScalarHillclimbCounted>();
            var c2cSweeps = (HullSupportScalarHillclimbCounted.Climbs - climbs0) / 2.0 / stream.Length;
            long a1Certs = 0, a1Adopts = 0, a1WarmEvals = 0;
            climbs0 = HullSupportScalarHillclimbCounted.Climbs;
            ThroughputRunner.Sink += StreamWarm<HullSupportScalarHillclimbCounted>(false, r =>
            {
                if (r.CertificateFired) ++a1Certs;
                if (r.WarmAdopted) ++a1Adopts;
                if (r.EvaluatedWarm) ++a1WarmEvals;
            });
            var a1Sweeps = (HullSupportScalarHillclimbCounted.Climbs - climbs0) / 2.0 / stream.Length + (double)a1WarmEvals / stream.Length;
            long b1Certs = 0, b1Seeds = 0, b1WarmEvals = 0;
            climbs0 = HullSupportScalarHillclimbCounted.Climbs;
            ThroughputRunner.Sink += StreamWarm<HullSupportScalarHillclimbCounted>(true, r =>
            {
                if (r.CertificateFired) ++b1Certs;
                if (r.Seeded) ++b1Seeds;
                if (r.EvaluatedWarm) ++b1WarmEvals;
            });
            var b1Sweeps = (HullSupportScalarHillclimbCounted.Climbs - climbs0) / 2.0 / stream.Length + (double)b1WarmEvals / stream.Length;
            Console.WriteLine($"warmbench {name} ({stream.Length} frames): sweeps/frame c2c {c2cSweeps:F2}, A1 {a1Sweeps:F2} (cert {(double)a1Certs / stream.Length:P1}, adopt {(double)a1Adopts / stream.Length:P1}), " +
                $"B1 {b1Sweeps:F2} (cert {(double)b1Certs / stream.Length:P1}, seeded {(double)b1Seeds / stream.Length:P1})");
        }

        //Timed, interleaved, plain finder.
        Func<float>[] arms = [StreamC2C<HullSupportScalarHillclimb>, () => StreamWarm<HullSupportScalarHillclimb>(false, null), () => StreamWarm<HullSupportScalarHillclimb>(true, null)];
        string[] armNames = ["c2c", "A1 ", "B1 "];
        const double warmupSeconds = 2.0;
        var warmupStart = Stopwatch.GetTimestamp();
        for (int i = 0; i < 4 || Stopwatch.GetTimestamp() - warmupStart < warmupSeconds * Stopwatch.Frequency; ++i)
        {
            for (int arm = 0; arm < arms.Length; ++arm)
                ThroughputRunner.Sink += arms[arm]();
        }
        if (ThroughputRunner.InterTrialCooldownMilliseconds > 0)
            Thread.Sleep(ThroughputRunner.InterTrialCooldownMilliseconds * 4);
        const int reps = 2;
        var times = new double[arms.Length][];
        for (int arm = 0; arm < arms.Length; ++arm)
            times[arm] = new double[trialCount];
        for (int trial = 0; trial < trialCount; ++trial)
        {
            for (int arm = 0; arm < arms.Length; ++arm)
            {
                var start = Stopwatch.GetTimestamp();
                for (int rep = 0; rep < reps; ++rep)
                    ThroughputRunner.Sink += arms[arm]();
                var end = Stopwatch.GetTimestamp();
                times[arm][trial] = (end - start) * 1e9 / ((double)Stopwatch.Frequency * reps * stream.Length);
            }
            if (ThroughputRunner.InterTrialCooldownMilliseconds > 0)
                Thread.Sleep(ThroughputRunner.InterTrialCooldownMilliseconds);
        }
        for (int arm = 0; arm < arms.Length; ++arm)
            Array.Sort(times[arm]);
        var c2cMin = times[0][0];
        var c2cMed = times[0][trialCount / 2];
        for (int arm = 0; arm < arms.Length; ++arm)
        {
            var min = times[arm][0];
            var median = times[arm][trialCount / 2];
            Console.WriteLine($"    {armNames[arm]}: min {min:F1} ns/frame, median {median:F1}" +
                (arm == 0 ? "" : $" (vs c2c min {min / c2cMin:F3}x, median {median / c2cMed:F3}x)"));
        }
    }
}
