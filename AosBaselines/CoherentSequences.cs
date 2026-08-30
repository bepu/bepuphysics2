using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace AosBaselines;

public enum SequenceScenario
{
    /// <summary>Resting stack: persistent shallow contact, near-zero per-frame delta (independent jitter around a fixed base pose).</summary>
    Rest,
    /// <summary>Slow tangential slide across a face at constant shallow penetration; the true axis migrates across face/edge basins.</summary>
    Slide,
    /// <summary>Deepening persistent overlap: monotone depth growth from the speculative band into the deep regime.</summary>
    Deepen,
    /// <summary>Slide stream with periodic teleports (segment re-randomization): warm data suddenly garbage.</summary>
    Teleport,
    /// <summary>Separated-heavy hover: separation oscillates between within-margin and beyond-margin, with a slow tangential drift.</summary>
    Hover,
}

public struct SequenceFrame
{
    public Vector3 OffsetB;
    public Quaternion OrientationA, OrientationB;
    public bool Teleport;
}

/// <summary>
/// Temporally-coherent pose-sequence generator for the warm-start study. Each call produces one pair's stream of frames;
/// warm state must evolve closed-loop under the scheme being tested (endogeneity is load-bearing — see the README
/// warm-start brainstorm, impossibility 5). Placement uses support matching like HullHullGenerator: for a world contact
/// normal n (pointing A->B), OffsetB = supportA(n) - supportB(-n) + n*separation puts the pair at the given separation
/// (negative = penetration) along n.
/// </summary>
public static class CoherentSequences
{
    static Vector3 SupportWorld(in ConvexHull hull, in Matrix3x3 orientation, in Vector3 direction)
    {
        var localDirection = ScalarMath.TransformByTransposed(direction, orientation);
        var localSupport = HullSupportScalar.ComputeLocalSupport(hull, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }

    static Vector3 JitterDirection(Random random)
    {
        return new Vector3(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1);
    }

    static Vector3 TangentOf(Random random, Vector3 n)
    {
        var cross = Vector3.Cross(n, random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f));
        var lengthSquared = cross.LengthSquared();
        if (lengthSquared < 1e-12f)
            cross = Vector3.Cross(n, new Vector3(0.317f, 0.755f, -0.571f));
        return Vector3.Normalize(cross);
    }

    /// <summary>
    /// Generates one pair's frame stream. delta = per-frame pose change in units of the pair scale (translation) —
    /// slide periods and jitter radii derive from it. speculativeMargin should be the margin the runner will test with.
    /// </summary>
    public static SequenceFrame[] Generate(Random random, HullSet set, int indexA, int indexB,
        SequenceScenario scenario, int frameCount, float delta, float speculativeMargin)
    {
        var scale = MathF.Min(set.MaxRadii[indexA], set.MaxRadii[indexB]);
        var frames = new SequenceFrame[frameCount];
        ref var a = ref set.Hulls[indexA];
        ref var b = ref set.Hulls[indexB];

        var segmentLength = scenario == SequenceScenario.Teleport ? 64 : frameCount;
        var frame = 0;
        var firstSegment = true;
        while (frame < frameCount)
        {
            var qA = random.UnitQuaternion();
            var qB = random.UnitQuaternion();
            Matrix3x3.CreateFromQuaternion(qA, out var ra);
            Matrix3x3.CreateFromQuaternion(qB, out var rb);
            var n = random.UnitDirection();
            var supportA = SupportWorld(a, ra, n);
            var supportB = SupportWorld(b, rb, -n);
            var tangent = TangentOf(random, n);
            var baseContact = supportA - supportB;
            //Slide amplitude spans past the face into edge/vertex territory; period derives from the per-frame delta.
            var amplitude = 1.2f * scale;
            var period = MathF.Max(8f, amplitude * 2f * MathF.PI / MathF.Max(1e-6f, delta * scale));
            var shallow = -0.02f * scale;

            var end = Math.Min(frameCount, frame + segmentLength);
            for (int local = 0; frame < end; ++frame, ++local)
            {
                ref var f = ref frames[frame];
                f.OrientationA = qA;
                f.OrientationB = qB;
                f.Teleport = local == 0 && !firstSegment;
                switch (scenario)
                {
                    case SequenceScenario.Rest:
                        //Independent jitter around the base resting pose: positional noise delta*scale, tiny orientation noise.
                        f.OffsetB = baseContact + n * shallow + JitterDirection(random) * (delta * scale);
                        f.OrientationB = random.Perturb(qB, delta * 0.25f);
                        break;
                    case SequenceScenario.Slide:
                    case SequenceScenario.Teleport:
                        f.OffsetB = baseContact + n * shallow + tangent * (amplitude * MathF.Sin(2f * MathF.PI * local / period));
                        break;
                    case SequenceScenario.Deepen:
                        {
                            var separation = 0.5f * speculativeMargin - delta * scale * local;
                            if (separation < -0.6f * scale)
                                separation = -0.6f * scale;
                            f.OffsetB = baseContact + n * separation;
                            break;
                        }
                    case SequenceScenario.Hover:
                        {
                            //Separation swings between 0.1x and 1.6x the margin (in-margin speculative <-> certified miss),
                            //with a slow tangential drift so the separating feature migrates too.
                            var separation = speculativeMargin * (0.1f + 0.75f * (1f + MathF.Sin(2f * MathF.PI * local / 32f)));
                            f.OffsetB = baseContact + n * separation + tangent * (delta * scale * local);
                            break;
                        }
                }
            }
            firstSegment = false;
        }
        return frames;
    }
}
