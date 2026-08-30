using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace AosBaselines;

/// <summary>
/// Counting twin of HullSupportScalarHillclimb: increments a static climb counter per support query so experiment runners
/// can account refiner sweeps (1 CSO sample = 2 climbs) without touching the frozen ScalarDepthRefiner. Behavior is
/// otherwise identical (same warm-slot protocol). Not for timed paths; the plain finder stays the bench configuration.
/// </summary>
public unsafe struct HullSupportScalarHillclimbCounted : IScalarSupportFinder<HillclimbHull>
{
    public static long Climbs;

    public static Vector3 ComputeLocalSupport(in HillclimbHull shape, Vector3 direction)
    {
        ++Climbs;
        var support = HullSupportScalarHillclimb.Climb(shape.Topology, *shape.WarmStartSlot, direction, out var winner);
        *shape.WarmStartSlot = winner;
        return support;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 ComputeSupport(in HillclimbHull shape, Matrix3x3 orientation, Matrix3x3 orientationTranspose, Vector3 direction)
    {
        Matrix3x3.Transform(direction, orientationTranspose, out var localDirection);
        var localSupport = ComputeLocalSupport(shape, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }
}

/// <summary>
/// Per-pair setup shared by all warm-start scheme arms (mirrors HillclimbHullPairTester's setup; Track 1 configuration).
/// The refiner's shape A is hull B (identity, at origin) and its shape B is hull A (BLocalOrientationA, LocalOffsetA);
/// axes are B-local with the B->A convention throughout.
/// </summary>
public struct WarmPairSetup
{
    public Matrix3x3 RA, RB, BLocalOrientationA;
    public Vector3 LocalOffsetA;
    public Vector3 InitialNormal;
    public float EpsilonScale;
    public float ConvergenceThreshold;
    public float DepthThreshold;
}

/// <summary>
/// Result of a warm-start scheme arm. Depth/Normal use the refiner convention (B-local axis pointing B->A);
/// WitnessOnB is the refiner's witness on hull B (its shape A), B-local.
/// </summary>
public struct WarmSchemeResult
{
    public float Depth;
    public Vector3 Normal;
    public Vector3 WitnessOnB;
    public bool EvaluatedWarm;
    public bool CertificateFired;
    public bool WarmAdopted;
    public bool Seeded;
    public bool LegRan;
    public float WarmDepth;
    public Vector3 WarmCsoSample;
    /// <summary>CSO samples of the main refiner run (counting finder only; 0 with the plain finder).</summary>
    public int RefinerSweeps;
    /// <summary>CSO samples of the A2 epilogue leg (counting finder only).</summary>
    public int LegSweeps;
}

/// <summary>
/// Warm-start schemes for the DepthRefiner, hull-hull, Track 1 (hillclimb-support) configuration:
///   A1: warm-axis certificate + output min-compose ("evaluate, never seed") — the c2c trajectory runs untouched.
///   A2: A1 + adopt-and-polish epilogue (K-iteration refiner leg seeded from the adopted axis, floored by the A1 min).
///   B1: separated-band trajectory seeding (seed iff the warm one-sweep depth is negative; else A1 semantics).
/// The frozen ScalarDepthRefiner is wrapped, never modified: the vanilla c2c run inside A1 is bit-identical to a
/// standalone run (the warm-axis evaluation uses separate fresh climb state and cannot perturb the run's warm slots).
/// Relaxed-equality code except where the A1 guarantee requires exactness (the compose is an explicit compare+select
/// over exact samples; no Min/Max intrinsics — a NaN warm depth must lose, never propagate).
/// </summary>
public static class WarmStartSchemes
{
    public static WarmPairSetup CreateSetup(HullTopology topologyA, HullTopology topologyB,
        in Vector3 offsetB, in Quaternion orientationA, in Quaternion orientationB, float speculativeMargin)
    {
        WarmPairSetup setup;
        Matrix3x3.CreateFromQuaternion(orientationA, out setup.RA);
        Matrix3x3.CreateFromQuaternion(orientationB, out setup.RB);
        ScalarMath.MultiplyByTranspose(setup.RA, setup.RB, out setup.BLocalOrientationA);
        var localOffsetB = ScalarMath.TransformByTransposed(offsetB, setup.RB);
        setup.LocalOffsetA = -localOffsetB;
        var centerDistance = ScalarMath.Length(setup.LocalOffsetA);
        setup.InitialNormal = centerDistance < 1e-8f ? new Vector3(0f, 1f, 0f) : setup.LocalOffsetA * (1f / centerDistance);
        var firstPointA = topologyA.Vertices[0];
        var firstPointB = topologyB.Vertices[0];
        var aEpsilonScale = (MathF.Abs(firstPointA.X) + MathF.Abs(firstPointA.Y) + MathF.Abs(firstPointA.Z)) * (1f / 3f);
        var bEpsilonScale = (MathF.Abs(firstPointB.X) + MathF.Abs(firstPointB.Y) + MathF.Abs(firstPointB.Z)) * (1f / 3f);
        setup.EpsilonScale = float.MinNative(aEpsilonScale, bEpsilonScale);
        setup.ConvergenceThreshold = 1e-5f * setup.EpsilonScale;
        setup.DepthThreshold = -speculativeMargin;
        return setup;
    }

    /// <summary>
    /// Guards a cached warm axis: rejects NaN/infinite/zero axes outright (a zeroed slot must never "win" at depth 0),
    /// renormalizes non-unit but finite axes. The !(x > eps) form is deliberately NaN-rejecting.
    /// </summary>
    public static bool TryValidateAxis(Vector3 axis, out Vector3 validated)
    {
        var lengthSquared = axis.LengthSquared();
        if (!(lengthSquared > 1e-16f) || !float.IsFinite(lengthSquared))
        {
            validated = default;
            return false;
        }
        validated = axis * (1f / MathF.Sqrt(lengthSquared));
        return true;
    }

    /// <summary>
    /// One exact CSO support sample along a B-local B->A axis; returns the interval depth. Uses fresh climb starts
    /// (vertex 0) — deliberately separate walk state, so this evaluation cannot perturb any refiner run's warm slots.
    /// Matches the refiner's own sample math (shape A = hull B at identity; shape B = hull A at BLocalOrientationA/LocalOffsetA),
    /// and climbs are exact global support maxima by convexity, so the value equals what a refiner sample along the
    /// same axis would produce.
    /// </summary>
    public static float EvaluateAxis(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup, Vector3 axis,
        out Vector3 csoSample, out Vector3 supportOnHullB)
    {
        supportOnHullB = HullSupportScalarHillclimb.Climb(topologyB, 0, axis, out _);
        var negatedAxisInA = ScalarMath.TransformByTransposed(-axis, setup.BLocalOrientationA);
        var supportInA = HullSupportScalarHillclimb.Climb(topologyA, 0, negatedAxisInA, out _);
        Matrix3x3.Transform(supportInA, setup.BLocalOrientationA, out var extreme);
        extreme += setup.LocalOffsetA;
        csoSample = supportOnHullB - extreme;
        return Vector3.Dot(csoSample, axis);
    }

    /// <summary>Vanilla refiner run in the Track 1 configuration with an arbitrary initial normal and iteration cap.</summary>
    public static unsafe void RunRefiner<TSupport>(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup,
        Vector3 initialNormal, int maximumIterations, out float depth, out Vector3 normal, out Vector3 witnessOnB)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        int warmStartA = 0, warmStartB = 0;
        var refinerShapeA = new HillclimbHull { Topology = topologyB, WarmStartSlot = &warmStartB };
        var refinerShapeB = new HillclimbHull { Topology = topologyA, WarmStartSlot = &warmStartA };
        ScalarDepthRefiner<HillclimbHull, TSupport, HillclimbHull, TSupport>.FindMinimumDepth(
            refinerShapeA, refinerShapeB, setup.LocalOffsetA, setup.BLocalOrientationA, initialNormal,
            setup.ConvergenceThreshold, setup.DepthThreshold, out depth, out normal, out witnessOnB, maximumIterations);
    }

    /// <summary>The pure c2c baseline: center-to-center initial normal, default cap.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void RunC2C<TSupport>(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup,
        out float depth, out Vector3 normal, out Vector3 witnessOnB)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        RunRefiner<TSupport>(topologyA, topologyB, setup, setup.InitialNormal, 25, out depth, out normal, out witnessOnB);
    }

    static void RunC2CInto<TSupport>(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup, ref WarmSchemeResult r)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        var climbsBefore = HullSupportScalarHillclimbCounted.Climbs;
        RunC2C<TSupport>(topologyA, topologyB, setup, out r.Depth, out r.Normal, out r.WitnessOnB);
        r.RefinerSweeps = (int)((HullSupportScalarHillclimbCounted.Climbs - climbsBefore) >> 1);
    }

    /// <summary>
    /// A1: evaluate, never seed. One support sample along the cached warm axis; below the depth threshold that is a
    /// sound separation certificate for the pair (any axis's interval depth upper-bounds the true minimum). Otherwise
    /// the vanilla c2c refiner runs bit-identically and the reported result is the argmin over {c2c result, warm sample},
    /// via explicit compare+select (NaN-safe; a corrupt warm depth loses).
    /// </summary>
    public static WarmSchemeResult RunA1<TSupport>(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup, Vector3 warmAxis)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        WarmSchemeResult r = default;
        if (TryValidateAxis(warmAxis, out var w))
        {
            r.EvaluatedWarm = true;
            r.WarmDepth = EvaluateAxis(topologyA, topologyB, setup, w, out r.WarmCsoSample, out var warmSupportOnB);
            if (r.WarmDepth < setup.DepthThreshold)
            {
                //Separation certified: today's initialNormal pre-check with the trapping failure mode amputated.
                r.CertificateFired = true;
                r.Depth = r.WarmDepth;
                r.Normal = w;
                r.WitnessOnB = warmSupportOnB;
                return r;
            }
            //The warm axis is discarded from the trajectory; the c2c run below is bit-identical to a standalone c2c run
            //(fresh warm slots inside RunRefiner; the warm evaluation above used its own separate climb state).
            var climbsBefore = HullSupportScalarHillclimbCounted.Climbs;
            RunC2C<TSupport>(topologyA, topologyB, setup, out var c2cDepth, out var c2cNormal, out var c2cWitness);
            r.RefinerSweeps = (int)((HullSupportScalarHillclimbCounted.Climbs - climbsBefore) >> 1);
            //Explicit compare+select — never a Min/Max intrinsic: NaN in the warm depth must lose, never propagate.
            if (r.WarmDepth < c2cDepth)
            {
                r.WarmAdopted = true;
                r.Depth = r.WarmDepth;
                r.Normal = w;
                r.WitnessOnB = warmSupportOnB;
            }
            else
            {
                r.Depth = c2cDepth;
                r.Normal = c2cNormal;
                r.WitnessOnB = c2cWitness;
            }
            return r;
        }
        //Invalid cached axis: pure c2c passthrough, no warm sweep spent.
        RunC2CInto<TSupport>(topologyA, topologyB, setup, ref r);
        return r;
    }

    /// <summary>
    /// A2 epilogue: when the warm sample won the A1 min, a K-iteration refiner leg seeded from the adopted axis with a
    /// fresh single-vertex simplex repairs the witness (natural termination implies support/normal colinearity), floored
    /// by the A1 min. The leg's own running min starts at the re-sampled warm depth, so the floor is a rounding-level guard.
    /// </summary>
    public static void RunEpilogue<TSupport>(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup,
        ref WarmSchemeResult r, int k)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        r.LegRan = true;
        var climbsBefore = HullSupportScalarHillclimbCounted.Climbs;
        RunRefiner<TSupport>(topologyA, topologyB, setup, r.Normal, k, out var legDepth, out var legNormal, out var legWitness);
        r.LegSweeps = (int)((HullSupportScalarHillclimbCounted.Climbs - climbsBefore) >> 1);
        if (legDepth < r.Depth)
        {
            r.Depth = legDepth;
            r.Normal = legNormal;
        }
        //The leg's witness is adopted even at equal depth: witness repair is the leg's purpose, and its simplex is
        //consistent with its own (<= A1 min) result.
        r.WitnessOnB = legWitness;
    }

    public static WarmSchemeResult RunA2<TSupport>(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup,
        Vector3 warmAxis, int k)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        var r = RunA1<TSupport>(topologyA, topologyB, setup, warmAxis);
        if (r.WarmAdopted)
            RunEpilogue<TSupport>(topologyA, topologyB, setup, ref r, k);
        return r;
    }

    /// <summary>
    /// B1: separated-band trajectory seeding. The already-paid A1 warm sweep is the gate: a negative one-sweep depth
    /// proves the pair is separated, and in that regime the search target pins to the origin (no quench coupling) and
    /// epsilon-terminated results are seed-independent within the convergence threshold, so seeding is safe. The
    /// penetrating band falls through to A1 semantics (evaluate, never seed).
    /// </summary>
    public static WarmSchemeResult RunB1<TSupport>(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup, Vector3 warmAxis)
        where TSupport : IScalarSupportFinder<HillclimbHull>
    {
        WarmSchemeResult r = default;
        if (TryValidateAxis(warmAxis, out var w))
        {
            r.EvaluatedWarm = true;
            r.WarmDepth = EvaluateAxis(topologyA, topologyB, setup, w, out r.WarmCsoSample, out var warmSupportOnB);
            if (r.WarmDepth < setup.DepthThreshold)
            {
                r.CertificateFired = true;
                r.Depth = r.WarmDepth;
                r.Normal = w;
                r.WitnessOnB = warmSupportOnB;
                return r;
            }
            if (r.WarmDepth < 0f)
            {
                r.Seeded = true;
                var climbsBefore = HullSupportScalarHillclimbCounted.Climbs;
                RunRefiner<TSupport>(topologyA, topologyB, setup, w, 25, out var depth, out var normal, out var witness);
                r.RefinerSweeps = (int)((HullSupportScalarHillclimbCounted.Climbs - climbsBefore) >> 1);
                //The seeded run's initial sample recomputes the warm sample (identical fresh-slot climbs on the same
                //axis), so its running min already includes depth_w; the guard is belt and suspenders.
                if (r.WarmDepth < depth)
                {
                    r.Depth = r.WarmDepth;
                    r.Normal = w;
                    r.WitnessOnB = warmSupportOnB;
                }
                else
                {
                    r.Depth = depth;
                    r.Normal = normal;
                    r.WitnessOnB = witness;
                }
                return r;
            }
            //Penetrating band: A1 semantics.
            var before = HullSupportScalarHillclimbCounted.Climbs;
            RunC2C<TSupport>(topologyA, topologyB, setup, out var c2cDepth, out var c2cNormal, out var c2cWitness);
            r.RefinerSweeps = (int)((HullSupportScalarHillclimbCounted.Climbs - before) >> 1);
            if (r.WarmDepth < c2cDepth)
            {
                r.WarmAdopted = true;
                r.Depth = r.WarmDepth;
                r.Normal = w;
                r.WitnessOnB = warmSupportOnB;
            }
            else
            {
                r.Depth = c2cDepth;
                r.Normal = c2cNormal;
                r.WitnessOnB = c2cWitness;
            }
            return r;
        }
        RunC2CInto<TSupport>(topologyA, topologyB, setup, ref r);
        return r;
    }

    /// <summary>Signed max face-plane violation of a hull-local point against the true face support offsets (~0 on surface).</summary>
    public static float SurfacePlaneError(HullTopology topology, Vector3 pointLocal)
    {
        var max = float.MinValue;
        var faceNormals = topology.FaceNormals;
        var faceSupportOffsets = topology.FaceSupportOffsets;
        for (int f = 0; f < topology.FaceCount; ++f)
        {
            var v = Vector3.Dot(faceNormals[f], pointLocal) - faceSupportOffsets[f];
            if (v > max)
                max = v;
        }
        return max;
    }

    /// <summary>
    /// Witness-consistency residual of a reported (depth, normal, witnessOnB) triple: the hull manifold path reconstructs
    /// closestOnA = witnessOnB - normal*depth, which is valid only when the CSO support behind the witness is colinear
    /// with the normal. Residual = worst distance-like plane violation of the two implied surface points against their
    /// hulls (witnessOnB on hull B; the reconstruction on hull A). Natural refiner termination keeps this near zero;
    /// a raw adopted warm sample does not.
    /// </summary>
    public static float WitnessResidual(HullTopology topologyA, HullTopology topologyB, in WarmPairSetup setup,
        float depth, Vector3 normal, Vector3 witnessOnB)
    {
        var closestOnA = witnessOnB - normal * depth;
        var closestOnAInA = ScalarMath.TransformByTransposed(closestOnA - setup.LocalOffsetA, setup.BLocalOrientationA);
        var errorA = MathF.Abs(SurfacePlaneError(topologyA, closestOnAInA));
        var errorB = MathF.Abs(SurfacePlaneError(topologyB, witnessOnB));
        return MathF.Max(errorA, errorB);
    }
}
