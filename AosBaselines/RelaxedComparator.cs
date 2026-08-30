#nullable enable
using BepuUtilities;
using System.Numerics;

namespace AosBaselines;

public enum RelaxedVerdict
{
    /// <summary>Below the warn thresholds everywhere.</summary>
    Pass,
    /// <summary>Measurable deviation, but below the fail thresholds.</summary>
    Warn,
    /// <summary>Near-degenerate or boundary case where the candidate legitimately made a different-but-equally-valid choice
    /// (different normal with matching support depth, near-touching sign ambiguity, speculative-accept boundary).</summary>
    Tie,
    /// <summary>Deviation beyond tolerance on a non-degenerate case.</summary>
    Fail,
}

public struct RelaxedCaseMetrics
{
    /// <summary>Angle between reference and candidate manifold normals in radians (0 when either side had no contacts).</summary>
    public float NormalAngle;
    /// <summary>|referenceDepth - candidateDepth| / scale where depth is the maximum over existing contacts.</summary>
    public float DepthError;
    /// <summary>Max over candidate contacts of the distance to the nearest reference contact, / scale.</summary>
    public float PositionDeviation;
    /// <summary>candidateContactCount - referenceContactCount.</summary>
    public int ContactCountDelta;
    public RelaxedVerdict Verdict;
    /// <summary>Set for Fail (and Tie) verdicts: which gate tripped.</summary>
    public string? Reason;
}

/// <summary>
/// Tolerance comparator for the relaxed-equality hull-hull study: compares a candidate tester's manifold against the
/// engine wide tester's manifold for the same pair. The reference is authoritative, but alternative algorithms may
/// legitimately pick different (equally minimal) normals on near-degenerate cases; those are detected via a
/// support-depth evaluation of the candidate's normal and classified as ties rather than failures.
/// </summary>
public static class RelaxedComparator
{
    //Hard gates (FAIL beyond these on non-degenerate cases).
    public const float FailNormalAngle = 1e-2f;          //radians
    public const float FailDepthError = 1e-3f;           //normalized by case scale
    public const float FlipDepthThreshold = 1e-4f;       //pen<->sep flips only fail when |referenceDepth| > this * scale
    //Warn band lower bounds.
    public const float WarnNormalAngle = 1e-3f;
    public const float WarnDepthError = 1e-4f;
    //Tie test: a candidate normal is "equally valid" if its true support depth matches the reference depth this closely.
    public const float TieDepthAbsoluteTolerance = 1e-3f;   //times scale
    public const float TieDepthRelativeTolerance = 5e-2f;   //times |referenceDepth|
    //Existence flips are boundary ties when the accepting side's depth sits this close to the -speculativeMargin accept threshold.
    public const float ExistenceBoundaryTolerance = 1e-3f;  //times scale

    static int CountContacts(in Convex4ManifoldScalar manifold) =>
        (manifold.Contact0Exists ? 1 : 0) + (manifold.Contact1Exists ? 1 : 0) + (manifold.Contact2Exists ? 1 : 0) + (manifold.Contact3Exists ? 1 : 0);

    static float MaxDepth(in Convex4ManifoldScalar manifold)
    {
        var depth = float.MinValue;
        if (manifold.Contact0Exists) depth = float.MaxNative(depth, manifold.Depth0);
        if (manifold.Contact1Exists) depth = float.MaxNative(depth, manifold.Depth1);
        if (manifold.Contact2Exists) depth = float.MaxNative(depth, manifold.Depth2);
        if (manifold.Contact3Exists) depth = float.MaxNative(depth, manifold.Depth3);
        return depth;
    }

    static int GatherContacts(in Convex4ManifoldScalar manifold, Span<Vector3> positions)
    {
        int count = 0;
        if (manifold.Contact0Exists) positions[count++] = manifold.OffsetA0;
        if (manifold.Contact1Exists) positions[count++] = manifold.OffsetA1;
        if (manifold.Contact2Exists) positions[count++] = manifold.OffsetA2;
        if (manifold.Contact3Exists) positions[count++] = manifold.OffsetA3;
        return count;
    }

    static float MaxDotOverVertices(Vector3[] vertices, in Vector3 direction)
    {
        var best = float.MinValue;
        for (int i = 0; i < vertices.Length; ++i)
            best = float.MaxNative(best, Vector3.Dot(vertices[i], direction));
        return best;
    }

    /// <summary>
    /// True interval depth of the pair along an arbitrary world-space unit normal pointing from B to A:
    /// depth(n) = dot(offsetB, n) + max over B of dot(vertex, n) + max over A of dot(vertex, -n).
    /// Positive = overlap along n; the refiner's reported depth is the minimum of this over all normals.
    /// </summary>
    public static float SupportDepth(HullTopology a, HullTopology b, in Matrix3x3 rA, in Matrix3x3 rB, in Vector3 offsetB, in Vector3 normal)
    {
        Matrix3x3.TransformTranspose(normal, rB, out var localNormalB);
        Matrix3x3.TransformTranspose(-normal, rA, out var negatedLocalNormalA);
        return Vector3.Dot(offsetB, normal) + MaxDotOverVertices(b.Vertices, localNormalB) + MaxDotOverVertices(a.Vertices, negatedLocalNormalA);
    }

    public static RelaxedCaseMetrics Compare(in Convex4ManifoldScalar reference, in Convex4ManifoldScalar candidate,
        HullTopology a, HullTopology b, in Matrix3x3 rA, in Matrix3x3 rB, in Vector3 offsetB, float speculativeMargin, float scale)
    {
        RelaxedCaseMetrics metrics = default;
        var referenceCount = CountContacts(reference);
        var candidateCount = CountContacts(candidate);
        metrics.ContactCountDelta = candidateCount - referenceCount;

        if (referenceCount == 0 && candidateCount == 0)
        {
            metrics.Verdict = RelaxedVerdict.Pass;
            return metrics;
        }
        if (referenceCount == 0 || candidateCount == 0)
        {
            //Existence flip. The accept gate is depth >= -speculativeMargin, so a depth near that boundary (or near zero,
            //where contact generation itself can go either way) is a legitimate boundary tie.
            var presentDepth = referenceCount > 0 ? MaxDepth(reference) : MaxDepth(candidate);
            if (MathF.Abs(presentDepth + speculativeMargin) <= ExistenceBoundaryTolerance * scale ||
                MathF.Abs(presentDepth) <= ExistenceBoundaryTolerance * scale)
            {
                metrics.Verdict = RelaxedVerdict.Tie;
                metrics.Reason = "existence flip at accept boundary";
            }
            else
            {
                metrics.Verdict = RelaxedVerdict.Fail;
                metrics.Reason = $"existence flip (reference {referenceCount} contacts, candidate {candidateCount}, present depth {presentDepth})";
            }
            return metrics;
        }

        var referenceDepth = MaxDepth(reference);
        var candidateDepth = MaxDepth(candidate);
        //atan2(|cross|, dot) rather than acos(clamped dot): acos has a ~1e-3 rad precision floor near parallel
        //(bitwise-identical not-exactly-unit normals would read as ~1e-3 rad); the cross form is exact at zero.
        var normalDot = Vector3.Dot(reference.Normal, candidate.Normal);
        metrics.NormalAngle = MathF.Atan2(Vector3.Cross(reference.Normal, candidate.Normal).Length(), normalDot);
        metrics.DepthError = MathF.Abs(referenceDepth - candidateDepth) / scale;

        //Contact position deviation: max over candidate contacts of distance to nearest reference contact, normalized.
        Span<Vector3> referencePositions = stackalloc Vector3[4];
        Span<Vector3> candidatePositions = stackalloc Vector3[4];
        var refPositionCount = GatherContacts(reference, referencePositions);
        var candPositionCount = GatherContacts(candidate, candidatePositions);
        float maxDeviation = 0;
        for (int i = 0; i < candPositionCount; ++i)
        {
            var best = float.MaxValue;
            for (int j = 0; j < refPositionCount; ++j)
                best = float.MinNative(best, Vector3.DistanceSquared(candidatePositions[i], referencePositions[j]));
            maxDeviation = float.MaxNative(maxDeviation, best);
        }
        metrics.PositionDeviation = MathF.Sqrt(maxDeviation) / scale;

        var nearTouching = MathF.Abs(referenceDepth) <= FlipDepthThreshold * scale;
        var penetrationFlip = (referenceDepth >= 0) != (candidateDepth >= 0);

        if (metrics.NormalAngle > FailNormalAngle)
        {
            //Different normal. Equally valid if the candidate's normal supports (nearly) the same depth as the reference's minimum.
            var candidateSupportDepth = SupportDepth(a, b, rA, rB, offsetB, candidate.Normal);
            var tieTolerance = float.MaxNative(TieDepthAbsoluteTolerance * scale, TieDepthRelativeTolerance * MathF.Abs(referenceDepth));
            if (MathF.Abs(candidateSupportDepth - referenceDepth) <= tieTolerance)
            {
                metrics.Verdict = RelaxedVerdict.Tie;
                metrics.Reason = "different normal, equivalent support depth";
            }
            else
            {
                metrics.Verdict = RelaxedVerdict.Fail;
                metrics.Reason = $"normal angle {metrics.NormalAngle:E2} rad, candidate normal support depth {candidateSupportDepth} vs reference depth {referenceDepth}";
            }
            return metrics;
        }
        if (penetrationFlip && !nearTouching)
        {
            metrics.Verdict = RelaxedVerdict.Fail;
            metrics.Reason = $"penetration/separation flip (reference depth {referenceDepth}, candidate depth {candidateDepth})";
            return metrics;
        }
        if (metrics.DepthError > FailDepthError)
        {
            metrics.Verdict = RelaxedVerdict.Fail;
            metrics.Reason = $"normalized depth error {metrics.DepthError:E2}";
            return metrics;
        }
        if (penetrationFlip && nearTouching)
        {
            metrics.Verdict = RelaxedVerdict.Tie;
            metrics.Reason = "sign flip while near-touching";
            return metrics;
        }
        metrics.Verdict = metrics.NormalAngle > WarnNormalAngle || metrics.DepthError > WarnDepthError ? RelaxedVerdict.Warn : RelaxedVerdict.Pass;
        return metrics;
    }
}

/// <summary>Aggregates per-case relaxed comparison metrics into verdict counts and max/p99/p50 distributions.</summary>
public sealed class RelaxedComparisonStats
{
    public long Pass, Warn, Tie, Fail, TotalCases, CasesWithContacts;
    readonly List<float> normalAngles = new();
    readonly List<float> depthErrors = new();
    readonly List<float> positionDeviations = new();
    readonly List<int> contactCountDeltas = new();

    public void Add(in RelaxedCaseMetrics metrics, bool bothHadContacts)
    {
        ++TotalCases;
        switch (metrics.Verdict)
        {
            case RelaxedVerdict.Pass: ++Pass; break;
            case RelaxedVerdict.Warn: ++Warn; break;
            case RelaxedVerdict.Tie: ++Tie; break;
            case RelaxedVerdict.Fail: ++Fail; break;
        }
        if (bothHadContacts)
        {
            ++CasesWithContacts;
            normalAngles.Add(metrics.NormalAngle);
            depthErrors.Add(metrics.DepthError);
            positionDeviations.Add(metrics.PositionDeviation);
            contactCountDeltas.Add(metrics.ContactCountDelta);
        }
    }

    static (float p50, float p99, float max) Distribution(List<float> samples)
    {
        if (samples.Count == 0)
            return (0, 0, 0);
        samples.Sort();
        return (samples[(int)(0.5 * (samples.Count - 1))], samples[(int)(0.99 * (samples.Count - 1))], samples[^1]);
    }

    public void Print(string label)
    {
        Console.WriteLine($"{label}: {TotalCases} cases, {CasesWithContacts} with contacts on both sides.");
        Console.WriteLine($"    verdicts: PASS {Pass} ({100.0 * Pass / TotalCases:F2}%), WARN {Warn} ({100.0 * Warn / TotalCases:F2}%), " +
            $"TIE {Tie} ({100.0 * Tie / TotalCases:F3}%), FAIL {Fail} ({100.0 * Fail / TotalCases:F3}%)");
        var (angleP50, angleP99, angleMax) = Distribution(normalAngles);
        var (depthP50, depthP99, depthMax) = Distribution(depthErrors);
        var (posP50, posP99, posMax) = Distribution(positionDeviations);
        Console.WriteLine($"    normal angle (rad):      p50 {angleP50:E2}, p99 {angleP99:E2}, max {angleMax:E2}");
        Console.WriteLine($"    depth error (/scale):    p50 {depthP50:E2}, p99 {depthP99:E2}, max {depthMax:E2}");
        Console.WriteLine($"    position dev (/scale):   p50 {posP50:E2}, p99 {posP99:E2}, max {posMax:E2}");
        if (contactCountDeltas.Count > 0)
        {
            var histogram = new long[9];
            foreach (var delta in contactCountDeltas)
                ++histogram[Math.Clamp(delta + 4, 0, 8)];
            Console.WriteLine($"    contact count delta -4..+4: {string.Join("/", histogram.Select(h => $"{100.0 * h / contactCountDeltas.Count:F1}%"))}");
        }
    }
}
