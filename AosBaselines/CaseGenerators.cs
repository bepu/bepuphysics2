using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;

namespace AosBaselines;

public struct SphereSphereCase
{
    public Sphere A;
    public Sphere B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
}

public struct BoxBoxCase
{
    public Box A;
    public Box B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

public struct BoxTriangleCase
{
    public Box A;
    public Triangle B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

public struct HullPairCase
{
    /// <summary>Index of hull A in the hull set.</summary>
    public int A;
    /// <summary>Index of hull B in the hull set.</summary>
    public int B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

public static class RandomExtensions
{
    public static float LogUniform(this Random random, float min, float max)
    {
        return min * MathF.Exp(random.NextSingle() * MathF.Log(max / min));
    }

    public static Vector3 UnitDirection(this Random random)
    {
        //Not perfectly uniform, but plenty for fuzzing.
        while (true)
        {
            var v = new Vector3(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1);
            var lengthSquared = v.LengthSquared();
            if (lengthSquared > 1e-6f && lengthSquared <= 1f)
                return v / MathF.Sqrt(lengthSquared);
        }
    }

    public static Quaternion UnitQuaternion(this Random random)
    {
        while (true)
        {
            var q = new Quaternion(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1);
            var lengthSquared = q.LengthSquared();
            if (lengthSquared > 1e-6f && lengthSquared <= 1f)
                return Quaternion.Normalize(q);
        }
    }

    /// <summary>
    /// One of the 24 axis-aligned box orientations (composed quarter turns).
    /// </summary>
    public static Quaternion AxisAlignedOrientation(this Random random)
    {
        var x = Quaternion.CreateFromAxisAngle(Vector3.UnitX, random.Next(4) * MathF.PI * 0.5f);
        var y = Quaternion.CreateFromAxisAngle(Vector3.UnitY, random.Next(4) * MathF.PI * 0.5f);
        var z = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, random.Next(4) * MathF.PI * 0.5f);
        return Quaternion.Normalize(z * y * x);
    }

    public static Quaternion Perturb(this Random random, Quaternion q, float angle)
    {
        return Quaternion.Normalize(Quaternion.CreateFromAxisAngle(random.UnitDirection(), angle) * q);
    }

    public static float NextSphereRadius(this Random random)
    {
        return random.Next(4) switch
        {
            0 => 1f,
            1 => 0.5f,
            _ => random.LogUniform(1e-2f, 1e2f),
        };
    }

    public static float NextSpeculativeMargin(this Random random, float scale)
    {
        return random.Next(5) switch
        {
            0 => 0f,
            1 => scale * 10f,
            _ => random.LogUniform(1e-3f, 2f) * scale,
        };
    }

    public static Quaternion NextShapeOrientation(this Random random)
    {
        return random.Next(6) switch
        {
            0 => Quaternion.Identity,
            1 => random.AxisAlignedOrientation(),
            2 => random.Perturb(random.AxisAlignedOrientation(), random.LogUniform(1e-8f, 1e-3f)),
            _ => random.UnitQuaternion(),
        };
    }

    public static float NextSeparation(this Random random, float scale, float speculativeMargin, bool contactHeavy)
    {
        return random.Next(contactHeavy ? 6 : 9) switch
        {
            0 => -scale * (0.25f + random.NextSingle()) * 0.5f,
            1 => -scale * 0.05f * random.NextSingle(),
            2 => -scale * 1e-5f,
            3 => 0f,
            4 => scale * 1e-5f,
            5 => speculativeMargin * 0.5f,
            6 => speculativeMargin * (0.99f + random.NextSingle() * 0.02f),
            7 => speculativeMargin + scale * (0.1f + random.NextSingle()),
            _ => scale * (random.NextSingle() * 2 - 1) * 0.2f,
        };
    }

    /// <summary>
    /// Quaternion rotating one unit direction onto another, with an antiparallel fallback.
    /// </summary>
    public static Quaternion ShortestArc(Vector3 from, Vector3 to)
    {
        var dot = Vector3.Dot(from, to);
        if (dot < -0.999999f)
        {
            var axis = Vector3.Cross(from, Vector3.UnitX);
            if (axis.LengthSquared() < 1e-6f)
                axis = Vector3.Cross(from, Vector3.UnitY);
            return Quaternion.Normalize(Quaternion.CreateFromAxisAngle(Vector3.Normalize(axis), MathF.PI));
        }
        var cross = Vector3.Cross(from, to);
        return Quaternion.Normalize(new Quaternion(cross, 1f + dot));
    }

    /// <summary>
    /// Support-matching placement: positions B so its support point toward A meets A's support point with the given separation along n,
    /// plus a tangential slide.
    /// </summary>
    public static Vector3 FeaturePlacement(this Random random, in Vector3 supportA, in Vector3 supportB, in Vector3 n, float separation, float slideSpan, bool contactHeavy)
    {
        var tangent = Vector3.Normalize(Vector3.Cross(n, random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f)));
        var slideScale = random.Next(contactHeavy ? 2 : 4) switch
        {
            0 => 0f,
            1 => random.NextSingle() * 0.5f,
            2 => 0.9f + random.NextSingle() * 0.2f,
            _ => random.NextSingle() * 1.5f,
        };
        return supportA - supportB + n * separation + tangent * (slideScale * slideSpan);
    }
}

/// <summary>Case for a sphere paired with any orientation-B-only shape.</summary>
public struct SphereVariantCase<TShapeB>
{
    public Sphere A;
    public TShapeB B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationB;
}

public interface ISphereVariantGenerator<TShapeB>
{
    Random Random { get; }
    SphereVariantCase<TShapeB> Next();
}

public class SphereCapsuleGenerator : ISphereVariantGenerator<Capsule>
{
    public Random Random { get; }
    public bool ContactHeavy;
    public SphereCapsuleGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    public SphereVariantCase<Capsule> Next()
    {
        SphereVariantCase<Capsule> testCase;
        testCase.A = new Sphere { Radius = Random.NextSphereRadius() };
        var capsuleRadius = Random.LogUniform(0.05f, 5f);
        testCase.B = new Capsule { Radius = capsuleRadius, HalfLength = capsuleRadius * Random.LogUniform(0.1f, 50f) };
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        var scale = MathF.Min(testCase.A.Radius, testCase.B.Radius);
        var span = testCase.A.Radius + testCase.B.Radius + testCase.B.HalfLength;
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);
        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3((Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    //Feature normals: capsule axis endpoints, radial directions, random.
                    var n = Random.Next(5) switch
                    {
                        0 => rb.Y,
                        1 => -rb.Y,
                        2 => Vector3.Normalize(Vector3.Cross(rb.Y, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f))),
                        _ => Random.UnitDirection(),
                    };
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = n * testCase.A.Radius;
                    var negatedN = -n;
                    var endpoint = rb.Y * (Vector3.Dot(rb.Y, negatedN) > 0 ? testCase.B.HalfLength : -testCase.B.HalfLength);
                    var supportB = endpoint + negatedN * testCase.B.Radius;
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public class SphereBoxGenerator : ISphereVariantGenerator<Box>
{
    public Random Random { get; }
    public bool ContactHeavy;
    public SphereBoxGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    public SphereVariantCase<Box> Next()
    {
        SphereVariantCase<Box> testCase;
        testCase.A = new Sphere { Radius = Random.NextSphereRadius() };
        testCase.B = new Box
        {
            HalfWidth = Random.LogUniform(1e-2f, 1e2f),
            HalfHeight = Random.LogUniform(1e-2f, 1e2f),
            HalfLength = Random.LogUniform(1e-2f, 1e2f),
        };
        if (Random.Next(4) == 0)
            testCase.B = new Box { HalfWidth = 1f, HalfHeight = 1f, HalfLength = 1f };
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        var minHalf = MathF.Min(testCase.B.HalfWidth, MathF.Min(testCase.B.HalfHeight, testCase.B.HalfLength));
        var maxHalf = MathF.Max(testCase.B.HalfWidth, MathF.Max(testCase.B.HalfHeight, testCase.B.HalfLength));
        var scale = MathF.Min(testCase.A.Radius, minHalf);
        var span = testCase.A.Radius + maxHalf;
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);
        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Sphere center exactly inside/at the box center: internal normal path.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3((Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span);
                break;
            case 3:
                //Inside the box (internal normal path, off-center).
                {
                    var localOffset = new Vector3(
                        (Random.NextSingle() * 2 - 1) * testCase.B.HalfWidth,
                        (Random.NextSingle() * 2 - 1) * testCase.B.HalfHeight,
                        (Random.NextSingle() * 2 - 1) * testCase.B.HalfLength);
                    Matrix3x3.Transform(localOffset, rb, out testCase.OffsetB);
                    break;
                }
            default:
                {
                    var n = Random.Next(8) switch
                    {
                        0 => rb.X, 1 => -rb.X, 2 => rb.Y, 3 => -rb.Y, 4 => rb.Z, 5 => -rb.Z,
                        _ => Random.UnitDirection(),
                    };
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = n * testCase.A.Radius;
                    var negatedN = -n;
                    var x = Vector3.Dot(rb.X, negatedN) > 0 ? testCase.B.HalfWidth : -testCase.B.HalfWidth;
                    var y = Vector3.Dot(rb.Y, negatedN) > 0 ? testCase.B.HalfHeight : -testCase.B.HalfHeight;
                    var z = Vector3.Dot(rb.Z, negatedN) > 0 ? testCase.B.HalfLength : -testCase.B.HalfLength;
                    var supportB = rb.X * x + rb.Y * y + rb.Z * z;
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public class SphereTriangleGenerator : ISphereVariantGenerator<Triangle>
{
    public Random Random { get; }
    public bool ContactHeavy;
    public SphereTriangleGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    public SphereVariantCase<Triangle> Next()
    {
        SphereVariantCase<Triangle> testCase;
        testCase.A = new Sphere { Radius = Random.NextSphereRadius() };
        var triangleScale = Random.LogUniform(0.1f, 10f);
        Vector3 a = Random.UnitDirection() * triangleScale;
        Vector3 b = Random.UnitDirection() * triangleScale;
        Vector3 c;
        if (Random.Next(8) == 0)
        {
            //Near-degenerate sliver: c close to the ab segment.
            var t = Random.NextSingle();
            c = a + (b - a) * t + Random.UnitDirection() * (triangleScale * Random.LogUniform(1e-8f, 1e-4f));
        }
        else
        {
            c = Random.UnitDirection() * triangleScale;
        }
        //Meshes recenter triangles; mirror that most of the time.
        if (Random.Next(4) != 0)
        {
            var centroid = (a + b + c) * (1f / 3f);
            a -= centroid;
            b -= centroid;
            c -= centroid;
        }
        testCase.B = new Triangle { A = a, B = b, C = c };
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        var scale = MathF.Min(testCase.A.Radius, triangleScale);
        var span = testCase.A.Radius + triangleScale * 2;
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);
        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3((Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    //Note that triangles are one sided; the anti-normal side produces contacts, but we sample both sides for coverage.
                    var triangleNormal = Vector3.Cross(b - a, c - a);
                    var normalLengthSquared = triangleNormal.LengthSquared();
                    Vector3 n;
                    if (normalLengthSquared > 1e-12f)
                    {
                        var unitNormal = triangleNormal / MathF.Sqrt(normalLengthSquared);
                        Matrix3x3.Transform(unitNormal, rb, out var worldNormal);
                        //Bias toward the colliding side for contact-heavy.
                        n = Random.Next(ContactHeavy ? 8 : 2) == 0 ? worldNormal : -worldNormal;
                        if (!ContactHeavy && Random.Next(3) == 0)
                            n = Random.UnitDirection();
                    }
                    else
                    {
                        n = Random.UnitDirection();
                    }
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = n * testCase.A.Radius;
                    var negatedN = -n;
                    var localNegatedN = ScalarMath.TransformByTransposed(negatedN, rb);
                    var dotA = Vector3.Dot(a, localNegatedN);
                    var dotB = Vector3.Dot(b, localNegatedN);
                    var dotC = Vector3.Dot(c, localNegatedN);
                    var localSupport = dotA > dotB ? dotA > dotC ? a : c : dotB > dotC ? b : c;
                    Matrix3x3.Transform(localSupport, rb, out var supportB);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public class SphereCylinderGenerator : ISphereVariantGenerator<Cylinder>
{
    public Random Random { get; }
    public bool ContactHeavy;
    public SphereCylinderGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    public SphereVariantCase<Cylinder> Next()
    {
        SphereVariantCase<Cylinder> testCase;
        testCase.A = new Sphere { Radius = Random.NextSphereRadius() };
        testCase.B = Random.Next(4) switch
        {
            0 => new Cylinder { Radius = 1f, HalfLength = 1f },
            1 => new Cylinder { Radius = Random.LogUniform(0.1f, 10f), HalfLength = Random.LogUniform(0.001f, 0.2f) },
            2 => new Cylinder { Radius = Random.LogUniform(0.05f, 2f), HalfLength = Random.LogUniform(5f, 100f) },
            _ => new Cylinder { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) },
        };
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        var scale = MathF.Min(testCase.A.Radius, MathF.Min(testCase.B.Radius, testCase.B.HalfLength));
        var span = testCase.A.Radius + MathF.Max(testCase.B.Radius, testCase.B.HalfLength);
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);
        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3((Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    var angle = Random.NextSingle() * MathF.PI * 2;
                    var radial = rb.X * MathF.Cos(angle) + rb.Z * MathF.Sin(angle);
                    var n = Random.Next(6) switch
                    {
                        0 => rb.Y,
                        1 => -rb.Y,
                        2 => radial,
                        //Cap edge blend.
                        3 => Vector3.Normalize((Random.Next(2) == 0 ? rb.Y : -rb.Y) * 0.5f + radial * 0.5f),
                        _ => Random.UnitDirection(),
                    };
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = n * testCase.A.Radius;
                    Matrix3x3.Transpose(rb, out var rbTranspose);
                    var supportB = CylinderSupportScalar.ComputeSupport(testCase.B, rb, rbTranspose, -n);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public class SphereHullGenerator : ISphereVariantGenerator<ConvexHull>
{
    public Random Random { get; }
    public HullSet Set;
    public bool ContactHeavy;
    public SphereHullGenerator(int seed, HullSet set, bool contactHeavy = false) { Random = new Random(seed); Set = set; ContactHeavy = contactHeavy; }

    public SphereVariantCase<ConvexHull> Next()
    {
        SphereVariantCase<ConvexHull> testCase;
        testCase.A = new Sphere { Radius = Random.NextSphereRadius() };
        var hullIndex = Random.Next(Set.Hulls.Length);
        testCase.B = Set.Hulls[hullIndex];
        var hullRadius = Set.MaxRadii[hullIndex];
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        var scale = MathF.Min(testCase.A.Radius, hullRadius);
        var span = testCase.A.Radius + hullRadius;
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);
        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3((Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span, (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    Vector3 n;
                    if (Random.Next(2) == 0)
                    {
                        var faceIndex = Random.Next(testCase.B.FaceToVertexIndicesStart.Length);
                        BundleIndexing.GetBundleIndices(faceIndex, out var bundleIndex, out var innerIndex);
                        Vector3Wide.ReadSlot(ref testCase.B.BoundingPlanes[bundleIndex].Normal, innerIndex, out var localFaceNormal);
                        Matrix3x3.Transform(localFaceNormal, rb, out var worldFaceNormal);
                        //Point from B toward where A should be.
                        n = -worldFaceNormal;
                    }
                    else
                    {
                        n = Random.UnitDirection();
                    }
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = n * testCase.A.Radius;
                    Matrix3x3.Transpose(rb, out var rbTranspose);
                    var supportB = HullSupportScalar.ComputeSupport(testCase.B, rb, rbTranspose, -n);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

/// <summary>
/// Generates sphere-sphere test cases spanning random configurations plus distance/feature corner cases:
/// coincident centers, exact touching, boundary-of-speculative-margin, deep penetration, extreme radius ratios.
/// </summary>
public class SphereSphereGenerator
{
    public Random Random;
    /// <summary>
    /// If true, generation is biased toward configurations that produce contacts, approximating a post-broadphase narrowphase workload.
    /// </summary>
    public bool ContactHeavy;
    public SphereSphereGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    float NextRadius()
    {
        return Random.Next(4) switch
        {
            0 => 1f,
            1 => 0.5f,
            _ => Random.LogUniform(1e-3f, 1e3f),
        };
    }

    float NextMargin(float scale)
    {
        return Random.Next(5) switch
        {
            0 => 0f,
            1 => 1e-6f,
            2 => scale * 10f,
            _ => Random.LogUniform(1e-3f, 2f) * scale,
        };
    }

    public SphereSphereCase Next()
    {
        SphereSphereCase testCase;
        testCase.A = new Sphere { Radius = NextRadius() };
        testCase.B = new Sphere { Radius = NextRadius() };
        var radiusSum = testCase.A.Radius + testCase.B.Radius;
        testCase.SpeculativeMargin = NextMargin(radiusSum);

        var direction = Random.Next(5) switch
        {
            0 => Vector3.UnitX,
            1 => Vector3.UnitY,
            2 => Vector3.UnitZ,
            _ => Random.UnitDirection(),
        };

        var distance = Random.Next(ContactHeavy ? 5 : 10) switch
        {
            //Coincident centers: degenerate normal fallback.
            0 => 0f,
            //Exact touching.
            1 => radiusSum,
            //Barely separated / barely penetrating around the surface.
            2 => radiusSum * (1f + (Random.NextSingle() * 2 - 1) * 1e-6f),
            //Just inside/outside the speculative margin boundary.
            3 => radiusSum + testCase.SpeculativeMargin * (0.99f + Random.NextSingle() * 0.02f),
            //Deep penetration; B center inside A.
            4 => Random.NextSingle() * testCase.A.Radius,
            //Separated beyond margin.
            5 => radiusSum + testCase.SpeculativeMargin + Random.LogUniform(1e-3f, 1e3f),
            //Random within a couple of radius sums.
            _ => Random.NextSingle() * radiusSum * 2,
        };
        testCase.OffsetB = direction * distance;
        return testCase;
    }
}

/// <summary>
/// Generates box-box test cases. Mixes purely random poses with feature-informed placement:
/// a contact normal is chosen from face/edge/vertex features, then B is placed by support point matching with a controlled
/// separation and a tangential slide, covering face-face, face-edge, face-vertex, edge-edge, and corner grazing contacts.
/// Orientation categories include identity, shared orientations (parallel edges), exact quarter-turn alignments, and epsilon perturbations.
/// </summary>
public class BoxBoxGenerator
{
    public Random Random;
    /// <summary>
    /// If true, generation is biased toward configurations that produce contacts, approximating a post-broadphase narrowphase workload.
    /// </summary>
    public bool ContactHeavy;
    public BoxBoxGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Box NextBox()
    {
        switch (Random.Next(6))
        {
            case 0:
                return new Box { HalfWidth = 1f, HalfHeight = 1f, HalfLength = 1f };
            case 1:
                {
                    //Plate: one dimension much smaller.
                    var size = Random.LogUniform(0.1f, 10f);
                    var thin = size * Random.LogUniform(1e-3f, 1e-1f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = thin; break;
                        case 1: box.HalfHeight = thin; break;
                        default: box.HalfLength = thin; break;
                    }
                    return box;
                }
            case 2:
                {
                    //Rod: one dimension much larger.
                    var size = Random.LogUniform(0.1f, 10f);
                    var lengthy = size * Random.LogUniform(10f, 1000f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = lengthy; break;
                        case 1: box.HalfHeight = lengthy; break;
                        default: box.HalfLength = lengthy; break;
                    }
                    return box;
                }
            default:
                return new Box
                {
                    HalfWidth = Random.LogUniform(1e-2f, 1e2f),
                    HalfHeight = Random.LogUniform(1e-2f, 1e2f),
                    HalfLength = Random.LogUniform(1e-2f, 1e2f),
                };
        }
    }

    void NextOrientations(out Quaternion orientationA, out Quaternion orientationB)
    {
        switch (Random.Next(8))
        {
            case 0:
                orientationA = Quaternion.Identity;
                orientationB = Quaternion.Identity;
                break;
            case 1:
                //Identical orientations: all edges parallel, degenerate edge-edge cross products.
                orientationA = Random.UnitQuaternion();
                orientationB = orientationA;
                break;
            case 2:
                //Axis-aligned quarter-turn alignments: stacking-like configurations with exact ties in face selection.
                orientationA = Random.AxisAlignedOrientation();
                orientationB = Random.AxisAlignedOrientation();
                break;
            case 3:
                //Nearly identical: near-parallel edges stress the degenerate axis fallbacks.
                orientationA = Random.UnitQuaternion();
                orientationB = Random.Perturb(orientationA, Random.LogUniform(1e-8f, 1e-3f));
                break;
            case 4:
                //Nearly axis aligned.
                orientationA = Random.Perturb(Random.AxisAlignedOrientation(), Random.LogUniform(1e-8f, 1e-3f));
                orientationB = Random.Perturb(Random.AxisAlignedOrientation(), Random.LogUniform(1e-8f, 1e-3f));
                break;
            case 5:
                //45 degree tilt: coplanar-ish edge contacts.
                orientationA = Quaternion.Identity;
                orientationB = Quaternion.CreateFromAxisAngle(Random.UnitDirection(), MathF.PI * 0.25f);
                break;
            default:
                orientationA = Random.UnitQuaternion();
                orientationB = Random.UnitQuaternion();
                break;
        }
    }

    static Vector3 Support(in Box box, in Matrix3x3 orientation, in Vector3 direction)
    {
        var x = Vector3.Dot(orientation.X, direction) > 0 ? box.HalfWidth : -box.HalfWidth;
        var y = Vector3.Dot(orientation.Y, direction) > 0 ? box.HalfHeight : -box.HalfHeight;
        var z = Vector3.Dot(orientation.Z, direction) > 0 ? box.HalfLength : -box.HalfLength;
        return orientation.X * x + orientation.Y * y + orientation.Z * z;
    }

    static Vector3 Axis(in Matrix3x3 orientation, int index) => index switch { 0 => orientation.X, 1 => orientation.Y, _ => orientation.Z };

    /// <summary>
    /// Picks a would-be contact normal from the feature sets of the two boxes.
    /// </summary>
    Vector3 NextFeatureNormal(in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                //Face of A.
                {
                    var axis = Axis(ra, Random.Next(3));
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 2:
            case 3:
                //Face of B.
                {
                    var axis = Axis(rb, Random.Next(3));
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 4:
            case 5:
                //Edge-edge: cross of one edge direction from each box.
                {
                    var cross = Vector3.Cross(Axis(ra, Random.Next(3)), Axis(rb, Random.Next(3)));
                    var lengthSquared = cross.LengthSquared();
                    if (lengthSquared < 1e-10f)
                        return Random.UnitDirection();
                    var n = cross / MathF.Sqrt(lengthSquared);
                    return Random.Next(2) == 0 ? n : -n;
                }
            case 6:
                //Vertex direction of A: pushes toward vertex-face and vertex-vertex configurations.
                {
                    var v = Axis(ra, 0) * (Random.Next(2) == 0 ? 1f : -1f)
                          + Axis(ra, 1) * (Random.Next(2) == 0 ? 1f : -1f)
                          + Axis(ra, 2) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(v);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public BoxBoxCase Next()
    {
        BoxBoxCase testCase;
        testCase.A = NextBox();
        testCase.B = NextBox();
        NextOrientations(out testCase.OrientationA, out testCase.OrientationB);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        var maxHalfA = MathF.Max(testCase.A.HalfWidth, MathF.Max(testCase.A.HalfHeight, testCase.A.HalfLength));
        var maxHalfB = MathF.Max(testCase.B.HalfWidth, MathF.Max(testCase.B.HalfHeight, testCase.B.HalfLength));
        var minHalfA = MathF.Min(testCase.A.HalfWidth, MathF.Min(testCase.A.HalfHeight, testCase.A.HalfLength));
        var minHalfB = MathF.Min(testCase.B.HalfWidth, MathF.Min(testCase.B.HalfHeight, testCase.B.HalfLength));
        var scale = MathF.Min(maxHalfA, maxHalfB);
        var minScale = MathF.Min(minHalfA, minHalfB);

        testCase.SpeculativeMargin = Random.Next(5) switch
        {
            0 => 0f,
            1 => scale * 10f,
            _ => Random.LogUniform(1e-3f, 2f) * scale,
        };

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Exactly coincident: fully degenerate.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                //Purely random offset within the combined bounds; mostly overlapping configurations of every stripe.
                {
                    var span = maxHalfA + maxHalfB;
                    testCase.OffsetB = new Vector3(
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span);
                    break;
                }
            default:
                //Feature-informed: support-match along a feature-derived normal with controlled separation, then slide tangentially.
                {
                    var n = NextFeatureNormal(ra, rb);
                    var separation = Random.Next(ContactHeavy ? 6 : 9) switch
                    {
                        0 => -minScale * (0.25f + Random.NextSingle()),                //deep penetration
                        1 => -scale * 0.05f * Random.NextSingle(),                     //moderate penetration
                        2 => -minScale * 1e-5f,                                        //hairline penetration
                        3 => 0f,                                                       //exact touch
                        4 => minScale * 1e-5f,                                         //hairline separation
                        5 => testCase.SpeculativeMargin * 0.5f,                        //speculative contact
                        6 => testCase.SpeculativeMargin * (0.99f + Random.NextSingle() * 0.02f), //margin boundary
                        7 => testCase.SpeculativeMargin + scale * (0.1f + Random.NextSingle()),  //miss
                        _ => scale * (Random.NextSingle() * 2 - 1) * 0.2f,             //random shallow
                    };
                    var supportA = Support(testCase.A, ra, n);
                    var supportB = Support(testCase.B, rb, -n);

                    //Tangential slide: 0 keeps the features aligned; larger values walk the contact across the face toward edges/corners and off the shape entirely.
                    var tangent = Vector3.Normalize(Vector3.Cross(n, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f)));
                    var slideScale = Random.Next(ContactHeavy ? 2 : 4) switch
                    {
                        0 => 0f,
                        1 => Random.NextSingle() * 0.5f,
                        2 => 0.9f + Random.NextSingle() * 0.2f, //around the edge of overlap
                        _ => Random.NextSingle() * 1.5f,
                    };
                    //Scaling slides by the max extents walks contacts across faces and off the shapes entirely; good for fuzz coverage,
                    //but for the contact-heavy profile use the smallest extent so anisotropic shapes (rods, plates) stay in contact.
                    var slide = tangent * (slideScale * (ContactHeavy ? minScale : maxHalfA + maxHalfB));
                    testCase.OffsetB = supportA - supportB + n * separation + slide;
                    break;
                }
        }
        return testCase;
    }
}

/// <summary>
/// Generates box-triangle test cases. Mixes purely random poses with feature-informed placement along normals drawn from the
/// pair's feature sets (triangle face front/back, box faces, box-edge x triangle-edge crosses, box vertex directions, and
/// backface-threshold skimmers). Triangle categories include axis-aligned right triangles (exact ties versus axis-aligned boxes),
/// near-degenerate slivers straddling the 1e-6 degeneracy epsilon, needles, exactly colinear triangles, and both windings;
/// vertex frames are mostly recentered (mesh convention) with occasional far-offset frames to stress cancellation.
/// Orientation categories align box faces to the triangle plane and box edges to triangle edges with jitter tiers spanning
/// exact ties, epsilon perturbations, and small real tilts, plus 45 degree tilts and axis-aligned orientations.
/// </summary>
public class BoxTriangleGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public BoxTriangleGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Box NextBox()
    {
        switch (Random.Next(6))
        {
            case 0:
                return new Box { HalfWidth = 1f, HalfHeight = 1f, HalfLength = 1f };
            case 1:
                {
                    //Plate: one dimension much smaller.
                    var size = Random.LogUniform(0.1f, 10f);
                    var thin = size * Random.LogUniform(1e-3f, 1e-1f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = thin; break;
                        case 1: box.HalfHeight = thin; break;
                        default: box.HalfLength = thin; break;
                    }
                    return box;
                }
            case 2:
                {
                    //Rod: one dimension much larger.
                    var size = Random.LogUniform(0.1f, 10f);
                    var lengthy = size * Random.LogUniform(10f, 1000f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = lengthy; break;
                        case 1: box.HalfHeight = lengthy; break;
                        default: box.HalfLength = lengthy; break;
                    }
                    return box;
                }
            default:
                return new Box
                {
                    HalfWidth = Random.LogUniform(1e-2f, 1e2f),
                    HalfHeight = Random.LogUniform(1e-2f, 1e2f),
                    HalfLength = Random.LogUniform(1e-2f, 1e2f),
                };
        }
    }

    Triangle NextTriangle(out float triScale)
    {
        triScale = Random.LogUniform(1e-2f, 1e2f);
        Vector3 a, b, c;
        //Contact-heavy keeps the degenerate/near-degenerate categories (slivers, needles, colinear) but at lower weight:
        //they mostly produce structural rejections, which the mixed profile already samples heavily.
        switch (Random.Next(ContactHeavy ? 16 : 8))
        {
            case 0:
                //Axis-aligned right triangle in a coordinate plane: exact ties versus axis-aligned boxes.
                a = default;
                switch (Random.Next(3))
                {
                    case 0: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, 0f, triScale); break;
                    case 1: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, triScale, 0f); break;
                    default: b = new Vector3(0f, triScale, 0f); c = new Vector3(0f, 0f, triScale); break;
                }
                break;
            case 1:
                //Near-degenerate sliver: height fraction straddles the 1e-6 DegenerateTriangleEpsilon boundary.
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = a + (b - a) * Random.NextSingle() + Random.UnitDirection() * (triScale * Random.LogUniform(1e-8f, 1e-4f));
                break;
            case 2:
                //Needle: two vertices nearly coincident, third far.
                a = Random.UnitDirection() * triScale;
                b = a + Random.UnitDirection() * (triScale * Random.LogUniform(1e-6f, 1e-3f));
                c = a + Random.UnitDirection() * triScale;
                break;
            case 3:
                //Exactly colinear: the nondegenerate mask must reject; also hits the 1e-7 edge-SAT guard.
                {
                    a = Random.UnitDirection() * triScale;
                    var d = Random.UnitDirection();
                    b = a + d * triScale;
                    c = a + d * (triScale * Random.NextSingle());
                    break;
                }
            default:
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = Random.UnitDirection() * triScale;
                break;
        }
        //Swap winding half the time so both facings occur structurally.
        if (Random.Next(2) == 0)
            (b, c) = (c, b);
        //Vertex frame placement: meshes recenter triangles; mirror that most of the time.
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                break;
            case 2:
                //Far-offset vertex frame: stresses cancellation in the local transform and clip t values.
                {
                    var off = Random.UnitDirection() * (triScale * Random.LogUniform(10f, 1e3f));
                    a += off; b += off; c += off;
                    break;
                }
            default:
                {
                    var centroid = (a + b + c) * (1f / 3f);
                    a -= centroid; b -= centroid; c -= centroid;
                    break;
                }
        }
        return new Triangle { A = a, B = b, C = c };
    }

    static Vector3 Axis(in Matrix3x3 orientation, int index) => index switch { 0 => orientation.X, 1 => orientation.Y, _ => orientation.Z };

    static Vector3 BoxSupport(in Box box, in Matrix3x3 orientation, in Vector3 direction)
    {
        var x = Vector3.Dot(orientation.X, direction) > 0 ? box.HalfWidth : -box.HalfWidth;
        var y = Vector3.Dot(orientation.Y, direction) > 0 ? box.HalfHeight : -box.HalfHeight;
        var z = Vector3.Dot(orientation.Z, direction) > 0 ? box.HalfLength : -box.HalfLength;
        return orientation.X * x + orientation.Y * y + orientation.Z * z;
    }

    static Vector3 LocalAxis(int index) => index switch { 0 => Vector3.UnitX, 1 => Vector3.UnitY, _ => Vector3.UnitZ };

    Quaternion NextOrientationA(Vector3 nFront, bool hasNormal, Vector3 wa, Vector3 wb, Vector3 wc)
    {
        switch (Random.Next(8))
        {
            case 0:
                return Quaternion.Identity;
            case 1:
                //Box face aligned to the triangle plane (near-parallel face-face; ties in face selection and Reduce).
                {
                    if (!hasNormal)
                        return Random.UnitQuaternion();
                    var localAxis = LocalAxis(Random.Next(3)) * (Random.Next(2) == 0 ? 1f : -1f);
                    var q0 = RandomExtensions.ShortestArc(localAxis, -nFront);
                    var spin = Quaternion.CreateFromAxisAngle(nFront, Random.NextSingle() * MathF.PI * 2);
                    var q = Quaternion.Normalize(spin * q0);
                    return Random.Next(3) switch
                    {
                        0 => q,
                        1 => Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f)),
                        _ => Random.Perturb(q, Random.LogUniform(1e-3f, 1e-1f)),
                    };
                }
            case 2:
                //Box edge parallel to a triangle edge: exact and near-degenerate edge-SAT axes plus clip parallel guards.
                {
                    var edge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var edgeLengthSquared = edge.LengthSquared();
                    if (!float.IsFinite(edgeLengthSquared) || edgeLengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    var e = edge / MathF.Sqrt(edgeLengthSquared);
                    var q0 = RandomExtensions.ShortestArc(LocalAxis(Random.Next(3)), e);
                    var spin = Quaternion.CreateFromAxisAngle(e, Random.NextSingle() * MathF.PI * 2);
                    var q = Quaternion.Normalize(spin * q0);
                    return Random.Next(2) == 0 ? q : Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f));
                }
            case 3:
                //45 degree tilt about an in-plane axis relative to a face-aligned pose: abs-compare ties in face selection.
                {
                    if (!hasNormal)
                        return Random.UnitQuaternion();
                    var q0 = RandomExtensions.ShortestArc(Vector3.UnitY, -nFront);
                    var inPlane = Vector3.Cross(nFront, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f));
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (inPlaneLengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    return Quaternion.Normalize(Quaternion.CreateFromAxisAngle(inPlane / MathF.Sqrt(inPlaneLengthSquared), MathF.PI * 0.25f) * q0);
                }
            case 4:
                return Random.AxisAlignedOrientation();
            case 5:
                return Random.Perturb(Random.AxisAlignedOrientation(), Random.LogUniform(1e-8f, 1e-3f));
            default:
                return Random.UnitQuaternion();
        }
    }

    //n points from A (box) toward B (triangle) per harness convention; the triangle's colliding side is +nFront,
    //so contact-heavy calibration picks the sign with dot(n, nFront) <= 0 (box on the front side).
    Vector3 NextFeatureNormal(in Matrix3x3 ra, Vector3 nFront, bool hasNormal, Vector3 wa, Vector3 wb, Vector3 wc)
    {
        //Real mesh workloads are dominated by triangle-face contacts, and the triangle's one-sidedness plus its lack of volume
        //make the other feature normals miss much more often than they do for solids; weight the face normal up for contact-heavy.
        if (ContactHeavy && hasNormal && Random.Next(2) == 0)
            return Random.Next(8) == 0 ? nFront : -nFront;
        switch (Random.Next(10))
        {
            case 0:
            case 1:
                //Triangle face.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    if (ContactHeavy)
                        return Random.Next(8) == 0 ? nFront : -nFront;
                    return Random.Next(2) == 0 ? nFront : -nFront;
                }
            case 2:
            case 3:
                //Box face: triangle feature digs into a box face.
                {
                    var axis = Axis(ra, Random.Next(3));
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(axis, nFront) <= 0 ? axis : -axis;
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 4:
            case 5:
                //Edge-edge: cross of a box axis and a triangle edge.
                {
                    var triEdge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var cross = Vector3.Cross(Axis(ra, Random.Next(3)), triEdge);
                    var lengthSquared = cross.LengthSquared();
                    if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-10f)
                        return Random.UnitDirection();
                    var n = cross / MathF.Sqrt(lengthSquared);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) <= 0 ? n : -n;
                    return Random.Next(2) == 0 ? n : -n;
                }
            case 6:
                //Box vertex direction: corner into the triangle face.
                {
                    var v = Axis(ra, 0) * (Random.Next(2) == 0 ? 1f : -1f)
                          + Axis(ra, 1) * (Random.Next(2) == 0 ? 1f : -1f)
                          + Axis(ra, 2) * (Random.Next(2) == 0 ? 1f : -1f);
                    var n = Vector3.Normalize(v);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) <= 0 ? n : -n;
                    return n;
                }
            case 7:
                //Backface-threshold skimmer: nearly in-plane normal straddling the -1e-2 rejection threshold
                //and the 1e-10 vertex raycast guard region.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    var triEdge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var inPlane = Vector3.Cross(triEdge, nFront);
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (!float.IsFinite(inPlaneLengthSquared) || inPlaneLengthSquared < 1e-12f)
                        return Random.UnitDirection();
                    inPlane /= MathF.Sqrt(inPlaneLengthSquared);
                    var u = Random.LogUniform(1e-4f, 3e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(inPlane + nFront * u);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public BoxTriangleCase Next()
    {
        BoxTriangleCase testCase;
        testCase.A = NextBox();
        testCase.B = NextTriangle(out var triScale);
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        Matrix3x3.Transform(testCase.B.A, rb, out var wa);
        Matrix3x3.Transform(testCase.B.B, rb, out var wb);
        Matrix3x3.Transform(testCase.B.C, rb, out var wc);
        //The colliding side of the triangle is the +cross(ab, ca) side (ab = B-A, ca = A-C).
        var triangleCross = Vector3.Cross(wb - wa, wa - wc);
        var crossLengthSquared = triangleCross.LengthSquared();
        var hasNormal = float.IsFinite(crossLengthSquared) && crossLengthSquared > 0f;
        var nFront = hasNormal ? triangleCross / MathF.Sqrt(crossLengthSquared) : Vector3.UnitY;
        hasNormal &= float.IsFinite(nFront.X) && float.IsFinite(nFront.Y) && float.IsFinite(nFront.Z);

        testCase.OrientationA = NextOrientationA(nFront, hasNormal, wa, wb, wc);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);

        var maxHalfA = MathF.Max(testCase.A.HalfWidth, MathF.Max(testCase.A.HalfHeight, testCase.A.HalfLength));
        var minHalfA = MathF.Min(testCase.A.HalfWidth, MathF.Min(testCase.A.HalfHeight, testCase.A.HalfLength));
        var vertReach = MathF.Max(wa.Length(), MathF.Max(wb.Length(), wc.Length()));
        var scale = MathF.Min(maxHalfA, triScale);
        var minScale = MathF.Min(minHalfA, triScale);
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Coincident frames: fully degenerate placement.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                //Purely random offset within the combined reach.
                {
                    var span = maxHalfA + vertReach;
                    testCase.OffsetB = new Vector3(
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span);
                    break;
                }
            default:
                //Feature-informed support matching with controlled separation plus a tangential slide.
                {
                    var n = NextFeatureNormal(ra, nFront, hasNormal, wa, wb, wc);
                    //Contact-heavy scales penetrations by the smallest extent: a penetration deeper than the box's thin axis pushes
                    //the box center through the one-sided triangle's plane, flipping the calibrated normal into backface rejection.
                    var separation = Random.NextSeparation(ContactHeavy ? minScale : scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = BoxSupport(testCase.A, ra, n);
                    var negatedN = -n;
                    var dotA = Vector3.Dot(wa, negatedN);
                    var dotB = Vector3.Dot(wb, negatedN);
                    var dotC = Vector3.Dot(wc, negatedN);
                    var supportB = dotA > dotB ? dotA > dotC ? wa : wc : dotB > dotC ? wb : wc;
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation,
                        ContactHeavy ? minScale : maxHalfA + vertReach, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public struct CylinderCylinderCase
{
    public Cylinder A;
    public Cylinder B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates cylinder-cylinder cases. Orientation categories emphasize the tester's branch boundaries: exactly parallel caps,
/// nearly parallel caps (the 0.9999 parallel-blend threshold), and tilts near the 45 degree cap/side threshold.
/// Placement uses support matching along cap normals, radial directions, cap-edge blends, and random directions.
/// </summary>
public class CylinderCylinderGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public CylinderCylinderGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Cylinder NextCylinder()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Cylinder { Radius = 1f, HalfLength = 1f };
            case 1:
                //Disc: much wider than long.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            case 2:
                //Rod: much longer than wide.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            default:
                return new Cylinder { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    static Vector3 SupportWorld(in Cylinder cylinder, in Matrix3x3 orientation, in Vector3 direction)
    {
        var localDirection = ScalarMath.TransformByTransposed(direction, orientation);
        var localSupport = CylinderSupportScalar.ComputeLocalSupport(cylinder, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    Vector3 NextFeatureNormal(in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(9))
        {
            case 0:
            case 1:
                //Cap of A.
                return Random.Next(2) == 0 ? ra.Y : -ra.Y;
            case 2:
            case 3:
                //Cap of B.
                return Random.Next(2) == 0 ? rb.Y : -rb.Y;
            case 4:
                //Radial (side) of A.
                return RadialDirection(ra);
            case 5:
                //Radial (side) of B.
                return RadialDirection(rb);
            case 6:
                //Cap edge of A: blend of cap normal and radial, hitting the cap/side threshold region.
                {
                    var radial = RadialDirection(ra);
                    var axis = Random.Next(2) == 0 ? ra.Y : -ra.Y;
                    var blend = 0.5f + (Random.NextSingle() * 2 - 1) * 0.25f;
                    return Vector3.Normalize(axis * blend + radial * (1f - blend));
                }
            default:
                return Random.UnitDirection();
        }
    }

    public CylinderCylinderCase Next()
    {
        CylinderCylinderCase testCase;
        testCase.A = NextCylinder();
        testCase.B = NextCylinder();

        switch (Random.Next(8))
        {
            case 0:
                //Exactly parallel, axis aligned.
                testCase.OrientationA = Quaternion.Identity;
                testCase.OrientationB = Quaternion.Identity;
                break;
            case 1:
                //Identical random orientations: parallel axes and caps.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = testCase.OrientationA;
                break;
            case 2:
                //Nearly parallel: stresses the 0.9999 parallel-blend threshold in cap-cap generation.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, Random.LogUniform(1e-5f, 3e-2f));
                break;
            case 3:
                //Near the 45 degree cap/side selection threshold.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, MathF.PI * 0.25f + (Random.NextSingle() * 2 - 1) * 0.05f);
                break;
            case 4:
                //Axis-aligned quarter turns: caps parallel or exactly perpendicular.
                testCase.OrientationA = Random.AxisAlignedOrientation();
                testCase.OrientationB = Random.AxisAlignedOrientation();
                break;
            default:
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.UnitQuaternion();
                break;
        }
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        //Match the tester's epsilon scale heuristic.
        var scale = MathF.Min(MathF.Max(testCase.A.HalfLength, testCase.A.Radius), MathF.Max(testCase.B.HalfLength, testCase.B.Radius));
        var span = MathF.Max(testCase.A.HalfLength, testCase.A.Radius) + MathF.Max(testCase.B.HalfLength, testCase.B.Radius);

        testCase.SpeculativeMargin = Random.Next(5) switch
        {
            0 => 0f,
            1 => scale * 10f,
            _ => Random.LogUniform(1e-3f, 2f) * scale,
        };

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    var n = NextFeatureNormal(ra, rb);
                    var separation = Random.Next(ContactHeavy ? 6 : 9) switch
                    {
                        0 => -scale * (0.25f + Random.NextSingle()) * 0.5f,
                        1 => -scale * 0.05f * Random.NextSingle(),
                        2 => -scale * 1e-5f,
                        3 => 0f,
                        4 => scale * 1e-5f,
                        5 => testCase.SpeculativeMargin * 0.5f,
                        6 => testCase.SpeculativeMargin * (0.99f + Random.NextSingle() * 0.02f),
                        7 => testCase.SpeculativeMargin + scale * (0.1f + Random.NextSingle()),
                        _ => scale * (Random.NextSingle() * 2 - 1) * 0.2f,
                    };
                    var supportA = SupportWorld(testCase.A, ra, n);
                    var supportB = SupportWorld(testCase.B, rb, -n);
                    var tangent = Vector3.Normalize(Vector3.Cross(n, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f)));
                    var slideScale = Random.Next(ContactHeavy ? 2 : 4) switch
                    {
                        0 => 0f,
                        1 => Random.NextSingle() * 0.5f,
                        2 => 0.9f + Random.NextSingle() * 0.2f,
                        _ => Random.NextSingle() * 1.5f,
                    };
                    var slide = tangent * (slideScale * (ContactHeavy ? scale : span));
                    testCase.OffsetB = supportA - supportB + n * separation + slide;
                    break;
                }
        }
        return testCase;
    }
}

/// <summary>
/// A pregenerated pool of convex hulls with varied vertex counts and shapes (compact clouds, boxy, flattened, elongated, tetrahedra).
/// Hull creation is expensive, so fuzzing and benchmarking pick random pairs from this set.
/// </summary>
public class HullSet : IDisposable
{
    public BufferPool Pool = new();
    public ConvexHull[] Hulls = [];
    public float[] MaxRadii = [];

    public static HullSet Create(Random random, int hullCount, int minPointCount, int maxPointCount)
    {
        var set = new HullSet
        {
            Hulls = new ConvexHull[hullCount],
            MaxRadii = new float[hullCount],
        };
        for (int i = 0; i < hullCount; ++i)
        {
            var pointCount = minPointCount + random.Next(maxPointCount - minPointCount + 1);
            set.Hulls[i] = CreateHull(random, set.Pool, pointCount);
            set.Hulls[i].ComputeAngularExpansionData(out set.MaxRadii[i], out _);
        }
        return set;
    }

    /// <summary>
    /// Creates a hull set targeting a specific hull vertex count: input points sit exactly on a (mildly anisotropic)
    /// ellipsoid surface, so every point is extreme and the resulting hull keeps ~all of them (the hull builder may
    /// still merge a few near-coplanar vertices; verify actual counts via HullTopology). Used by the relaxed-equality
    /// hull size scan.
    /// </summary>
    public static HullSet CreateSized(Random random, int hullCount, int targetVertexCount)
    {
        var set = new HullSet
        {
            Hulls = new ConvexHull[hullCount],
            MaxRadii = new float[hullCount],
        };
        var points = new Vector3[Math.Max(4, targetVertexCount)];
        for (int i = 0; i < hullCount; ++i)
        {
            while (true)
            {
                var scale = random.LogUniform(0.1f, 10f);
                //Mild anisotropy only: extreme plates/rods would invite face merges that erode the target count.
                var axisScale = new Vector3(
                    0.6f + random.NextSingle(),
                    0.6f + random.NextSingle(),
                    0.6f + random.NextSingle());
                for (int j = 0; j < points.Length; ++j)
                {
                    //Small outward-only radial jitter: keeps every point extreme (outward never creates interior points at
                    //these densities) while breaking the exact coplanarity that can push the hull builder's face merging
                    //into emitting degenerate topology on dense spherical clouds.
                    points[j] = random.UnitDirection() * axisScale * (scale * (1f + random.NextSingle() * 0.02f));
                }
                try
                {
                    set.Hulls[i] = new ConvexHull(points, set.Pool, out _);
                }
                catch (ArgumentException)
                {
                    //Degenerate point set; retry.
                    continue;
                }
                try
                {
                    //Reject hulls whose face data fails the closed-2-manifold topology checks (rare builder epsilon
                    //pathologies on dense near-coplanar clouds); the study requires clean topology.
                    HullTopology.Create(ref set.Hulls[i]);
                    break;
                }
                catch (InvalidOperationException)
                {
                    set.Hulls[i].Dispose(set.Pool);
                }
            }
            set.Hulls[i].ComputeAngularExpansionData(out set.MaxRadii[i], out _);
        }
        return set;
    }

    static ConvexHull CreateHull(Random random, BufferPool pool, int pointCount)
    {
        var points = new Vector3[Math.Max(4, pointCount)];
        while (true)
        {
            var scale = random.LogUniform(0.1f, 10f);
            //Shape categories: compact cloud, boxy, flattened, elongated.
            var axisScale = random.Next(4) switch
            {
                0 => new Vector3(1f, 0.05f + random.NextSingle() * 0.1f, 1f),  //plate-like
                1 => new Vector3(1f, 4f + random.NextSingle() * 4f, 1f),       //rod-like
                _ => new Vector3(1f),
            };
            var boxy = random.Next(3) == 0;
            for (int i = 0; i < points.Length; ++i)
            {
                Vector3 p;
                if (boxy)
                {
                    p = new Vector3(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1);
                }
                else
                {
                    p = random.UnitDirection() * (0.5f + 0.5f * random.NextSingle());
                }
                points[i] = p * axisScale * scale;
            }
            try
            {
                return new ConvexHull(points, pool, out _);
            }
            catch (ArgumentException)
            {
                //Degenerate point set (no volume); extremely rare with these generators, just retry.
            }
        }
    }

    public void Dispose()
    {
        Pool.Clear();
    }
}

/// <summary>Case for any one-shape-versus-hull pair; the hull is stored as an index into the shared HullSet.</summary>
public struct ShapeHullCase<TShapeA>
{
    public TShapeA A;
    /// <summary>Index of the hull in the hull set.</summary>
    public int B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Shared skeleton for the (convex shape)-hull generators: random poses plus feature-informed support matching along normals
/// drawn from the shape's and the hull's feature sets, with controlled separations and tangential slides (mirroring the
/// hull-hull generator's structure).
/// </summary>
public abstract class ShapeHullGeneratorBase<TShapeA>
{
    public Random Random;
    public HullSet Set;
    public bool ContactHeavy;

    protected ShapeHullGeneratorBase(int seed, HullSet set, bool contactHeavy)
    {
        Random = new Random(seed);
        Set = set;
        ContactHeavy = contactHeavy;
    }

    protected Vector3 RandomHullFaceNormalWorld(ref ConvexHull hull, in Matrix3x3 orientation)
    {
        var faceIndex = Random.Next(hull.FaceToVertexIndicesStart.Length);
        BundleIndexing.GetBundleIndices(faceIndex, out var bundleIndex, out var innerIndex);
        Vector3Wide.ReadSlot(ref hull.BoundingPlanes[bundleIndex].Normal, innerIndex, out var localNormal);
        Matrix3x3.Transform(localNormal, orientation, out var worldNormal);
        return worldNormal;
    }

    protected static Vector3 HullSupportWorld(ref ConvexHull hull, in Matrix3x3 orientation, in Vector3 direction)
    {
        Matrix3x3.Transpose(orientation, out var orientationTranspose);
        return HullSupportScalar.ComputeSupport(hull, orientation, orientationTranspose, direction);
    }

    protected Vector3 NormalizeOrRandom(Vector3 v)
    {
        var lengthSquared = v.LengthSquared();
        if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-10f)
            return Random.UnitDirection();
        return v / MathF.Sqrt(lengthSquared);
    }

    /// <summary>Shape A's world support point along the given direction.</summary>
    protected abstract Vector3 ShapeSupportWorld(in TShapeA shape, in Matrix3x3 orientation, in Vector3 direction);

    /// <summary>Feature normal pointing from A toward B (the hull).</summary>
    protected abstract Vector3 NextFeatureNormal(in TShapeA shape, ref ConvexHull hull, in Matrix3x3 ra, in Matrix3x3 rb);

    protected abstract TShapeA NextShape(out float shapeScale, out float shapeSpan);

    /// <summary>Hook for shape-feature-to-hull-face alignment categories; defaults to the generic orientation categories.</summary>
    protected virtual Quaternion NextOrientationA(in TShapeA shape, ref ConvexHull hull, in Matrix3x3 rb) => Random.NextShapeOrientation();

    /// <summary>Orientation aligning the given shape-local direction against a random hull face normal, with a spin about the
    /// normal and jitter tiers spanning exact alignment, epsilon perturbations, and small real tilts.</summary>
    protected Quaternion AlignedOrientation(Vector3 localDirection, ref ConvexHull hull, in Matrix3x3 rb)
    {
        var faceNormal = RandomHullFaceNormalWorld(ref hull, rb);
        var q0 = RandomExtensions.ShortestArc(localDirection, -faceNormal);
        var spin = Quaternion.CreateFromAxisAngle(faceNormal, Random.NextSingle() * MathF.PI * 2);
        var q = Quaternion.Normalize(spin * q0);
        return Random.Next(3) switch
        {
            0 => q,
            1 => Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f)),
            _ => Random.Perturb(q, Random.LogUniform(1e-3f, 1e-1f)),
        };
    }

    public ShapeHullCase<TShapeA> Next()
    {
        ShapeHullCase<TShapeA> testCase;
        testCase.A = NextShape(out var shapeScale, out var shapeSpan);
        testCase.B = Random.Next(Set.Hulls.Length);
        ref var hull = ref Set.Hulls[testCase.B];
        var hullRadius = Set.MaxRadii[testCase.B];
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        testCase.OrientationA = NextOrientationA(testCase.A, ref hull, rb);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        var scale = MathF.Min(shapeScale, hullRadius);
        var span = shapeSpan + hullRadius;
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);
        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Exactly coincident centers: fully degenerate placement.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                //Purely random offset within the combined reach.
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    var n = NextFeatureNormal(testCase.A, ref hull, ra, rb);
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = ShapeSupportWorld(testCase.A, ra, n);
                    var supportB = HullSupportWorld(ref hull, rb, -n);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public class CapsuleHullGenerator : ShapeHullGeneratorBase<Capsule>
{
    public CapsuleHullGenerator(int seed, HullSet set, bool contactHeavy = false) : base(seed, set, contactHeavy) { }

    protected override Capsule NextShape(out float shapeScale, out float shapeSpan)
    {
        var radius = Random.LogUniform(0.05f, 5f);
        var capsule = new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(0.1f, 50f) };
        shapeScale = capsule.Radius;
        shapeSpan = capsule.Radius + capsule.HalfLength;
        return capsule;
    }

    protected override Vector3 ShapeSupportWorld(in Capsule shape, in Matrix3x3 orientation, in Vector3 direction)
    {
        var endpoint = orientation.Y * (Vector3.Dot(orientation.Y, direction) > 0 ? shape.HalfLength : -shape.HalfLength);
        return endpoint + direction * shape.Radius;
    }

    protected override Vector3 NextFeatureNormal(in Capsule shape, ref ConvexHull hull, in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                //Capsule rests on a hull face (n points from A toward B, so negate the outward hull face normal).
                return -RandomHullFaceNormalWorld(ref hull, rb);
            case 2:
                //Capsule axis endpoint digs into the hull.
                return Random.Next(2) == 0 ? ra.Y : -ra.Y;
            case 3:
            case 4:
                //Radial: capsule lying sideways against the hull.
                return NormalizeOrRandom(Vector3.Cross(ra.Y, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f)));
            case 5:
                //Edge-on-edge-ish: cross of the capsule axis with a hull face normal.
                return NormalizeOrRandom(Vector3.Cross(ra.Y, RandomHullFaceNormalWorld(ref hull, rb)));
            default:
                return Random.UnitDirection();
        }
    }
}

public class BoxHullGenerator : ShapeHullGeneratorBase<Box>
{
    public BoxHullGenerator(int seed, HullSet set, bool contactHeavy = false) : base(seed, set, contactHeavy) { }

    protected override Box NextShape(out float shapeScale, out float shapeSpan)
    {
        Box box;
        switch (Random.Next(6))
        {
            case 0:
                box = new Box { HalfWidth = 1f, HalfHeight = 1f, HalfLength = 1f };
                break;
            case 1:
                {
                    //Plate: one dimension much smaller.
                    var size = Random.LogUniform(0.1f, 10f);
                    var thin = size * Random.LogUniform(1e-3f, 1e-1f);
                    box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = thin; break;
                        case 1: box.HalfHeight = thin; break;
                        default: box.HalfLength = thin; break;
                    }
                    break;
                }
            case 2:
                {
                    //Rod: one dimension much larger.
                    var size = Random.LogUniform(0.1f, 10f);
                    var lengthy = size * Random.LogUniform(10f, 1000f);
                    box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = lengthy; break;
                        case 1: box.HalfHeight = lengthy; break;
                        default: box.HalfLength = lengthy; break;
                    }
                    break;
                }
            default:
                box = new Box
                {
                    HalfWidth = Random.LogUniform(1e-2f, 1e2f),
                    HalfHeight = Random.LogUniform(1e-2f, 1e2f),
                    HalfLength = Random.LogUniform(1e-2f, 1e2f),
                };
                break;
        }
        shapeScale = MathF.Min(box.HalfWidth, MathF.Min(box.HalfHeight, box.HalfLength));
        shapeSpan = MathF.Max(box.HalfWidth, MathF.Max(box.HalfHeight, box.HalfLength));
        return box;
    }

    static Vector3 Axis(in Matrix3x3 orientation, int index) => index switch { 0 => orientation.X, 1 => orientation.Y, _ => orientation.Z };

    protected override Vector3 ShapeSupportWorld(in Box shape, in Matrix3x3 orientation, in Vector3 direction)
    {
        var x = Vector3.Dot(orientation.X, direction) > 0 ? shape.HalfWidth : -shape.HalfWidth;
        var y = Vector3.Dot(orientation.Y, direction) > 0 ? shape.HalfHeight : -shape.HalfHeight;
        var z = Vector3.Dot(orientation.Z, direction) > 0 ? shape.HalfLength : -shape.HalfLength;
        return orientation.X * x + orientation.Y * y + orientation.Z * z;
    }

    protected override Vector3 NextFeatureNormal(in Box shape, ref ConvexHull hull, in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(9))
        {
            case 0:
            case 1:
                //Box face into the hull: contact walks across the box face.
                {
                    var axis = Axis(ra, Random.Next(3));
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 2:
            case 3:
                //Box rests on a hull face.
                return -RandomHullFaceNormalWorld(ref hull, rb);
            case 4:
            case 5:
                //Edge-edge: cross of a box axis and a hull face normal.
                return NormalizeOrRandom(Vector3.Cross(Axis(ra, Random.Next(3)), RandomHullFaceNormalWorld(ref hull, rb)));
            case 6:
                //Box vertex direction: corner digs into the hull.
                {
                    var v = Axis(ra, 0) * (Random.Next(2) == 0 ? 1f : -1f)
                          + Axis(ra, 1) * (Random.Next(2) == 0 ? 1f : -1f)
                          + Axis(ra, 2) * (Random.Next(2) == 0 ? 1f : -1f);
                    return NormalizeOrRandom(v);
                }
            default:
                return Random.UnitDirection();
        }
    }
}

public class CylinderHullGenerator : ShapeHullGeneratorBase<Cylinder>
{
    public CylinderHullGenerator(int seed, HullSet set, bool contactHeavy = false) : base(seed, set, contactHeavy) { }

    protected override Cylinder NextShape(out float shapeScale, out float shapeSpan)
    {
        Cylinder cylinder;
        switch (Random.Next(5))
        {
            case 0:
                cylinder = new Cylinder { Radius = 1f, HalfLength = 1f };
                break;
            case 1:
                //Disc: much wider than long.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    cylinder = new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                    break;
                }
            case 2:
                //Rod: much longer than wide.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    cylinder = new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                    break;
                }
            default:
                cylinder = new Cylinder { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
                break;
        }
        shapeScale = MathF.Min(cylinder.Radius, cylinder.HalfLength);
        shapeSpan = MathF.Max(cylinder.Radius, cylinder.HalfLength);
        return cylinder;
    }

    protected override Quaternion NextOrientationA(in Cylinder shape, ref ConvexHull hull, in Matrix3x3 rb)
    {
        //Cap-onto-hull-face alignment stresses the 0.70710678118 cap/side threshold's far side and the 0.9999
        //cap-parallel interior point interpolation boundary in GenerateInteriorPoints.
        if (Random.Next(3) == 0)
            return AlignedOrientation(Random.Next(2) == 0 ? Vector3.UnitY : -Vector3.UnitY, ref hull, rb);
        return Random.NextShapeOrientation();
    }

    protected override Vector3 ShapeSupportWorld(in Cylinder shape, in Matrix3x3 orientation, in Vector3 direction)
    {
        var localDirection = ScalarMath.TransformByTransposed(direction, orientation);
        var localSupport = CylinderSupportScalar.ComputeLocalSupport(shape, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    protected override Vector3 NextFeatureNormal(in Cylinder shape, ref ConvexHull hull, in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(9))
        {
            case 0:
            case 1:
                //Cap of the cylinder digs into the hull.
                return Random.Next(2) == 0 ? ra.Y : -ra.Y;
            case 2:
                //Radial: cylinder side against the hull (side-edge clipping path).
                return RadialDirection(ra);
            case 3:
                //Cap edge: blend of cap normal and radial, straddling the 45 degree cap/side selection threshold.
                {
                    var radial = RadialDirection(ra);
                    var axis = Random.Next(2) == 0 ? ra.Y : -ra.Y;
                    var blend = 0.5f + (Random.NextSingle() * 2 - 1) * 0.25f;
                    return NormalizeOrRandom(axis * blend + radial * (1f - blend));
                }
            case 4:
            case 5:
                //Cylinder rests on a hull face.
                return -RandomHullFaceNormalWorld(ref hull, rb);
            case 6:
                //Edge-edge: cross of the cylinder axis and a hull face normal.
                return NormalizeOrRandom(Vector3.Cross(ra.Y, RandomHullFaceNormalWorld(ref hull, rb)));
            default:
                return Random.UnitDirection();
        }
    }
}

public class TriangleHullGenerator : ShapeHullGeneratorBase<Triangle>
{
    public TriangleHullGenerator(int seed, HullSet set, bool contactHeavy = false) : base(seed, set, contactHeavy) { }

    protected override Triangle NextShape(out float shapeScale, out float shapeSpan)
    {
        var triScale = Random.LogUniform(1e-2f, 1e2f);
        Vector3 a, b, c;
        //Contact-heavy keeps degenerate categories at lower weight: they mostly produce structural rejections.
        switch (Random.Next(ContactHeavy ? 16 : 8))
        {
            case 0:
                //Axis-aligned right triangle in a coordinate plane.
                a = default;
                switch (Random.Next(3))
                {
                    case 0: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, 0f, triScale); break;
                    case 1: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, triScale, 0f); break;
                    default: b = new Vector3(0f, triScale, 0f); c = new Vector3(0f, 0f, triScale); break;
                }
                break;
            case 1:
                //Near-degenerate sliver: height fraction straddles the 1e-6 DegenerateTriangleEpsilon boundary.
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = a + (b - a) * Random.NextSingle() + Random.UnitDirection() * (triScale * Random.LogUniform(1e-8f, 1e-4f));
                break;
            case 2:
                //Needle: two vertices nearly coincident, third far.
                a = Random.UnitDirection() * triScale;
                b = a + Random.UnitDirection() * (triScale * Random.LogUniform(1e-6f, 1e-3f));
                c = a + Random.UnitDirection() * triScale;
                break;
            case 3:
                //Exactly colinear: the nondegenerate mask must reject.
                {
                    a = Random.UnitDirection() * triScale;
                    var d = Random.UnitDirection();
                    b = a + d * triScale;
                    c = a + d * (triScale * Random.NextSingle());
                    break;
                }
            default:
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = Random.UnitDirection() * triScale;
                break;
        }
        //Swap winding half the time so both facings occur structurally.
        if (Random.Next(2) == 0)
            (b, c) = (c, b);
        //Vertex frame placement: meshes recenter triangles; mirror that most of the time, with occasional far offsets to stress cancellation.
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                break;
            case 2:
                {
                    var off = Random.UnitDirection() * (triScale * Random.LogUniform(10f, 1e3f));
                    a += off; b += off; c += off;
                    break;
                }
            default:
                {
                    var centroid = (a + b + c) * (1f / 3f);
                    a -= centroid; b -= centroid; c -= centroid;
                    break;
                }
        }
        shapeScale = triScale;
        shapeSpan = triScale * 2f;
        return new Triangle { A = a, B = b, C = c };
    }

    static bool TryGetLocalNormal(in Triangle shape, out Vector3 localNormal)
    {
        var cross = Vector3.Cross(shape.B - shape.A, shape.A - shape.C);
        var lengthSquared = cross.LengthSquared();
        if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-14f)
        {
            localNormal = default;
            return false;
        }
        localNormal = cross / MathF.Sqrt(lengthSquared);
        return float.IsFinite(localNormal.X) && float.IsFinite(localNormal.Y) && float.IsFinite(localNormal.Z);
    }

    protected override Quaternion NextOrientationA(in Triangle shape, ref ConvexHull hull, in Matrix3x3 rb)
    {
        //Triangle-face-onto-hull-face alignment: face-face ties in PickRepresentativeFace and Reduce.
        if (Random.Next(3) == 0 && TryGetLocalNormal(shape, out var localNormal))
            return AlignedOrientation(localNormal, ref hull, rb);
        return Random.NextShapeOrientation();
    }

    protected override Vector3 ShapeSupportWorld(in Triangle shape, in Matrix3x3 orientation, in Vector3 direction)
    {
        Matrix3x3.Transform(shape.A, orientation, out var wa);
        Matrix3x3.Transform(shape.B, orientation, out var wb);
        Matrix3x3.Transform(shape.C, orientation, out var wc);
        var dotA = Vector3.Dot(wa, direction);
        var dotB = Vector3.Dot(wb, direction);
        var dotC = Vector3.Dot(wc, direction);
        return dotA > dotB ? dotA > dotC ? wa : wc : dotB > dotC ? wb : wc;
    }

    protected override Vector3 NextFeatureNormal(in Triangle shape, ref ConvexHull hull, in Matrix3x3 ra, in Matrix3x3 rb)
    {
        var hasNormal = TryGetLocalNormal(shape, out var localNormal);
        Vector3 nFront = default;
        if (hasNormal)
            Matrix3x3.Transform(localNormal, ra, out nFront);
        //The hull only generates contacts approaching the triangle's front (+normal) side; n points from A (triangle) toward B (hull),
        //so the contact-producing sign is +nFront. Bias contact-heavy toward it.
        if (ContactHeavy && hasNormal && Random.Next(2) == 0)
            return Random.Next(8) == 0 ? -nFront : nFront;
        switch (Random.Next(9))
        {
            case 0:
            case 1:
                //Triangle face.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    if (ContactHeavy)
                        return Random.Next(8) == 0 ? -nFront : nFront;
                    return Random.Next(2) == 0 ? nFront : -nFront;
                }
            case 2:
            case 3:
                //Triangle rests against a hull face.
                {
                    var n = -RandomHullFaceNormalWorld(ref hull, rb);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) >= 0 ? n : -n;
                    return n;
                }
            case 4:
            case 5:
                //Edge-edge: cross of a triangle edge and a hull face normal.
                {
                    var edge = Random.Next(3) switch { 0 => shape.B - shape.A, 1 => shape.C - shape.B, _ => shape.A - shape.C };
                    Matrix3x3.Transform(edge, ra, out var worldEdge);
                    var n = NormalizeOrRandom(Vector3.Cross(worldEdge, RandomHullFaceNormalWorld(ref hull, rb)));
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) >= 0 ? n : -n;
                    return n;
                }
            case 6:
                //Backface-threshold skimmer: nearly in-plane normal straddling the -1e-2 rejection boundary.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    var edge = Random.Next(3) switch { 0 => shape.B - shape.A, 1 => shape.C - shape.B, _ => shape.A - shape.C };
                    Matrix3x3.Transform(edge, ra, out var worldEdge);
                    var inPlane = Vector3.Cross(worldEdge, nFront);
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (!float.IsFinite(inPlaneLengthSquared) || inPlaneLengthSquared < 1e-12f)
                        return Random.UnitDirection();
                    inPlane /= MathF.Sqrt(inPlaneLengthSquared);
                    var u = Random.LogUniform(1e-4f, 3e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(inPlane + nFront * u);
                }
            default:
                return Random.UnitDirection();
        }
    }
}

/// <summary>
/// Generates hull-hull test cases over a pregenerated hull set, mirroring the box-box generator's structure:
/// random poses plus feature-informed support matching along face/vertex-derived normals with controlled separations and tangential slides.
/// </summary>
public class HullHullGenerator
{
    public Random Random;
    public HullSet Set;
    public bool ContactHeavy;

    public HullHullGenerator(int seed, HullSet set, bool contactHeavy = false)
    {
        Random = new Random(seed);
        Set = set;
        ContactHeavy = contactHeavy;
    }

    static Vector3 SupportWorld(in ConvexHull hull, in Matrix3x3 orientation, in Vector3 direction)
    {
        var localDirection = ScalarMath.TransformByTransposed(direction, orientation);
        var localSupport = HullSupportScalar.ComputeLocalSupport(hull, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }

    Vector3 RandomFaceNormalWorld(in ConvexHull hull, in Matrix3x3 orientation)
    {
        var faceIndex = Random.Next(hull.FaceToVertexIndicesStart.Length);
        BundleIndexing.GetBundleIndices(faceIndex, out var bundleIndex, out var innerIndex);
        Vector3Wide.ReadSlot(ref hull.BoundingPlanes[bundleIndex].Normal, innerIndex, out var localNormal);
        Matrix3x3.Transform(localNormal, orientation, out var worldNormal);
        return worldNormal;
    }

    Vector3 NextFeatureNormal(in ConvexHull a, in ConvexHull b, in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                //Face of A: B gets placed against that face.
                return RandomFaceNormalWorld(a, ra);
            case 2:
            case 3:
                //Face of B, negated so the normal points from A toward B.
                return -RandomFaceNormalWorld(b, rb);
            case 4:
                //Cross of two face normals: edge-ish directions.
                {
                    var cross = Vector3.Cross(RandomFaceNormalWorld(a, ra), RandomFaceNormalWorld(b, rb));
                    var lengthSquared = cross.LengthSquared();
                    if (lengthSquared < 1e-10f)
                        return Random.UnitDirection();
                    return cross / MathF.Sqrt(lengthSquared);
                }
            case 5:
                //Vertex direction of A.
                {
                    var direction = Random.UnitDirection();
                    var vertex = SupportWorld(a, ra, direction);
                    var lengthSquared = vertex.LengthSquared();
                    if (lengthSquared < 1e-10f)
                        return direction;
                    return vertex / MathF.Sqrt(lengthSquared);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public HullPairCase Next()
    {
        HullPairCase testCase;
        testCase.A = Random.Next(Set.Hulls.Length);
        testCase.B = Random.Next(Set.Hulls.Length);
        ref var a = ref Set.Hulls[testCase.A];
        ref var b = ref Set.Hulls[testCase.B];
        var radiusA = Set.MaxRadii[testCase.A];
        var radiusB = Set.MaxRadii[testCase.B];
        var scale = MathF.Min(radiusA, radiusB);

        switch (Random.Next(8))
        {
            case 0:
                testCase.OrientationA = Quaternion.Identity;
                testCase.OrientationB = Quaternion.Identity;
                break;
            case 1:
                //Identical orientations: parallel faces/edges everywhere.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = testCase.OrientationA;
                break;
            case 2:
                //Nearly identical.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, Random.LogUniform(1e-8f, 1e-3f));
                break;
            default:
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.UnitQuaternion();
                break;
        }
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        testCase.SpeculativeMargin = Random.Next(5) switch
        {
            0 => 0f,
            1 => scale * 10f,
            _ => Random.LogUniform(1e-3f, 2f) * scale,
        };

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Exactly coincident.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                //Purely random offset within the combined bounds.
                {
                    var span = radiusA + radiusB;
                    testCase.OffsetB = new Vector3(
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span);
                    break;
                }
            default:
                //Feature-informed support matching, as in the box-box generator.
                {
                    var n = NextFeatureNormal(a, b, ra, rb);
                    var separation = Random.Next(ContactHeavy ? 6 : 9) switch
                    {
                        0 => -scale * (0.25f + Random.NextSingle()) * 0.5f,
                        1 => -scale * 0.05f * Random.NextSingle(),
                        2 => -scale * 1e-5f,
                        3 => 0f,
                        4 => scale * 1e-5f,
                        5 => testCase.SpeculativeMargin * 0.5f,
                        6 => testCase.SpeculativeMargin * (0.99f + Random.NextSingle() * 0.02f),
                        7 => testCase.SpeculativeMargin + scale * (0.1f + Random.NextSingle()),
                        _ => scale * (Random.NextSingle() * 2 - 1) * 0.2f,
                    };
                    var supportA = SupportWorld(a, ra, n);
                    var supportB = SupportWorld(b, rb, -n);
                    var tangent = Vector3.Normalize(Vector3.Cross(n, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f)));
                    var slideScale = Random.Next(ContactHeavy ? 2 : 4) switch
                    {
                        0 => 0f,
                        1 => Random.NextSingle() * 0.5f,
                        2 => 0.9f + Random.NextSingle() * 0.2f,
                        _ => Random.NextSingle() * 1.5f,
                    };
                    var slide = tangent * (slideScale * (ContactHeavy ? scale : radiusA + radiusB));
                    testCase.OffsetB = supportA - supportB + n * separation + slide;
                    break;
                }
        }
        return testCase;
    }
}

public struct BoxCylinderCase
{
    public Box A;
    public Cylinder B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates box-cylinder cases. Orientation categories emphasize the tester's branch boundaries: the 45 degree cap/side
/// threshold on |localNormal.Y|, near-parallel cap alignment (the 0.9999 interior-point blend threshold), the side path's
/// edge-alignment unrestrict band (0.01-0.02 rad), and axis-aligned exact ties in the box face selection.
/// Placement uses support matching along box face normals, cylinder cap/radial directions, edge blends, and random directions.
/// </summary>
public class BoxCylinderGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public BoxCylinderGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Box NextBox()
    {
        switch (Random.Next(6))
        {
            case 0:
                return new Box { HalfWidth = 1f, HalfHeight = 1f, HalfLength = 1f };
            case 1:
                {
                    //Plate: one dimension much smaller.
                    var size = Random.LogUniform(0.1f, 10f);
                    var thin = size * Random.LogUniform(1e-3f, 1e-1f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = thin; break;
                        case 1: box.HalfHeight = thin; break;
                        default: box.HalfLength = thin; break;
                    }
                    return box;
                }
            case 2:
                {
                    //Rod: one dimension much larger.
                    var size = Random.LogUniform(0.1f, 10f);
                    var lengthy = size * Random.LogUniform(10f, 1000f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = lengthy; break;
                        case 1: box.HalfHeight = lengthy; break;
                        default: box.HalfLength = lengthy; break;
                    }
                    return box;
                }
            default:
                return new Box
                {
                    HalfWidth = Random.LogUniform(1e-2f, 1e2f),
                    HalfHeight = Random.LogUniform(1e-2f, 1e2f),
                    HalfLength = Random.LogUniform(1e-2f, 1e2f),
                };
        }
    }

    Cylinder NextCylinder()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Cylinder { Radius = 1f, HalfLength = 1f };
            case 1:
                //Disc: much wider than long.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            case 2:
                //Rod: much longer than wide.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            default:
                return new Cylinder { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    static Vector3 BoxSupport(in Box box, in Matrix3x3 orientation, in Vector3 direction)
    {
        var x = Vector3.Dot(orientation.X, direction) > 0 ? box.HalfWidth : -box.HalfWidth;
        var y = Vector3.Dot(orientation.Y, direction) > 0 ? box.HalfHeight : -box.HalfHeight;
        var z = Vector3.Dot(orientation.Z, direction) > 0 ? box.HalfLength : -box.HalfLength;
        return orientation.X * x + orientation.Y * y + orientation.Z * z;
    }

    static Vector3 CylinderSupport(in Cylinder cylinder, in Matrix3x3 orientation, in Vector3 direction)
    {
        var localDirection = ScalarMath.TransformByTransposed(direction, orientation);
        var localSupport = CylinderSupportScalar.ComputeLocalSupport(cylinder, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    static Vector3 Axis(in Matrix3x3 orientation, int index) => index switch { 0 => orientation.X, 1 => orientation.Y, _ => orientation.Z };

    Vector3 NextFeatureNormal(in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(12))
        {
            case 0:
            case 1:
                //Box face of A.
                {
                    var axis = Axis(ra, Random.Next(3));
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 2:
            case 3:
                //Cap of B: drives the cap-path (|localNormal.Y| > 0.7071) branch.
                return Random.Next(2) == 0 ? rb.Y : -rb.Y;
            case 4:
            case 5:
                //Radial (side) of B: drives the side path.
                return RadialDirection(rb);
            case 6:
                //Cap edge of B: blend of cap normal and radial, hitting the cap/side threshold region.
                {
                    var radial = RadialDirection(rb);
                    var axis = Random.Next(2) == 0 ? rb.Y : -rb.Y;
                    var blend = 0.5f + (Random.NextSingle() * 2 - 1) * 0.25f;
                    return Vector3.Normalize(axis * blend + radial * (1f - blend));
                }
            case 7:
                //Near-perpendicular to the cylinder axis with a small axial component: side path near the unrestrict band.
                {
                    var radial = RadialDirection(rb);
                    var tilt = Random.LogUniform(1e-4f, 5e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(radial + rb.Y * tilt);
                }
            case 8:
                //Box edge direction blend: normal in the wedge between two box faces.
                {
                    var first = Axis(ra, Random.Next(3));
                    var second = Axis(ra, Random.Next(3));
                    var blend = Random.NextSingle();
                    var candidate = first * blend + second * (1f - blend);
                    if (candidate.LengthSquared() < 1e-6f)
                        return Random.UnitDirection();
                    return Vector3.Normalize(candidate);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public BoxCylinderCase Next()
    {
        BoxCylinderCase testCase;
        testCase.A = NextBox();
        testCase.B = NextCylinder();

        switch (Random.Next(8))
        {
            case 0:
                //Exactly axis aligned: exact ties in the box-face selection and exactly parallel/perpendicular caps.
                testCase.OrientationA = Quaternion.Identity;
                testCase.OrientationB = Quaternion.Identity;
                break;
            case 1:
                //Identical random orientations.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = testCase.OrientationA;
                break;
            case 2:
                //Nearly parallel: stresses the 0.9999 interior-point blend threshold in cap generation.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, Random.LogUniform(1e-5f, 3e-2f));
                break;
            case 3:
                //Near the 45 degree cap/side selection threshold.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, MathF.PI * 0.25f + (Random.NextSingle() * 2 - 1) * 0.05f);
                break;
            case 4:
                //Axis-aligned quarter turns: exact face ties, caps parallel or exactly perpendicular.
                testCase.OrientationA = Random.AxisAlignedOrientation();
                testCase.OrientationB = Random.AxisAlignedOrientation();
                break;
            default:
                testCase.OrientationA = Random.NextShapeOrientation();
                testCase.OrientationB = Random.NextShapeOrientation();
                break;
        }
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        //Match the tester's epsilon scale heuristic.
        var maxBoxExtent = MathF.Max(testCase.A.HalfWidth, MathF.Max(testCase.A.HalfHeight, testCase.A.HalfLength));
        var scale = MathF.Min(maxBoxExtent, MathF.Max(testCase.B.HalfLength, testCase.B.Radius));
        var span = maxBoxExtent + MathF.Max(testCase.B.HalfLength, testCase.B.Radius);

        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    var n = NextFeatureNormal(ra, rb);
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = BoxSupport(testCase.A, ra, n);
                    var supportB = CylinderSupport(testCase.B, rb, -n);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public struct CapsuleCylinderCase
{
    public Capsule A;
    public Cylinder B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates capsule-cylinder cases. Orientation categories emphasize the tester's branch boundaries: parallel and nearly
/// parallel axes (the segment-segment coplanarity interval fade and the iterative solver's parallel clamps), tilts near the
/// 45 degree cap/side contact threshold, and axis-aligned exact ties. Placement uses support matching along the capsule axis,
/// cylinder cap/radial directions, cap-edge blends, near-perpendicular tilts, and random directions.
/// </summary>
public class CapsuleCylinderGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public CapsuleCylinderGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Capsule NextCapsule()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Capsule { Radius = 0.5f, HalfLength = 1f };
            case 1:
                //Long thin rod: stresses the segment solver and coplanar side intervals.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            case 2:
                //Squat, nearly spherical: halfLength much smaller than radius.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            default:
                return new Capsule { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    Cylinder NextCylinder()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Cylinder { Radius = 1f, HalfLength = 1f };
            case 1:
                //Disc: much wider than long.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            case 2:
                //Rod: much longer than wide.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            default:
                return new Cylinder { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    static Vector3 CapsuleSupport(in Capsule capsule, in Matrix3x3 orientation, in Vector3 direction)
    {
        var endpoint = orientation.Y * (Vector3.Dot(orientation.Y, direction) > 0 ? capsule.HalfLength : -capsule.HalfLength);
        return endpoint + direction * capsule.Radius;
    }

    static Vector3 CylinderSupport(in Cylinder cylinder, in Matrix3x3 orientation, in Vector3 direction)
    {
        var localDirection = ScalarMath.TransformByTransposed(direction, orientation);
        var localSupport = CylinderSupportScalar.ComputeLocalSupport(cylinder, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    Vector3 NextFeatureNormal(in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(10))
        {
            case 0:
            case 1:
                //Capsule axis: endpoint-vs-cap and deep endpoint cases.
                return Random.Next(2) == 0 ? ra.Y : -ra.Y;
            case 2:
            case 3:
                //Cap of B: drives the cap-contact (|localNormal.Y| > 0.7071) branch.
                return Random.Next(2) == 0 ? rb.Y : -rb.Y;
            case 4:
            case 5:
                //Radial (side) of B: drives the side path and the internal segment-segment edge normal.
                return RadialDirection(rb);
            case 6:
                //Radial of A.
                return RadialDirection(ra);
            case 7:
                //Cap edge of B: blend of cap normal and radial, hitting the cap/side threshold region.
                {
                    var radial = RadialDirection(rb);
                    var axis = Random.Next(2) == 0 ? rb.Y : -rb.Y;
                    var blend = 0.5f + (Random.NextSingle() * 2 - 1) * 0.25f;
                    return Vector3.Normalize(axis * blend + radial * (1f - blend));
                }
            case 8:
                //Near-perpendicular to the cylinder axis with a small axial tilt: side path near the coplanarity fade band.
                {
                    var radial = RadialDirection(rb);
                    var tilt = Random.LogUniform(1e-4f, 5e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(radial + rb.Y * tilt);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public CapsuleCylinderCase Next()
    {
        CapsuleCylinderCase testCase;
        testCase.A = NextCapsule();
        testCase.B = NextCylinder();

        switch (Random.Next(8))
        {
            case 0:
                //Exactly parallel, axis aligned.
                testCase.OrientationA = Quaternion.Identity;
                testCase.OrientationB = Quaternion.Identity;
                break;
            case 1:
                //Identical random orientations: parallel axes and caps.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = testCase.OrientationA;
                break;
            case 2:
                //Nearly parallel: stresses the coplanarity interval fade and the parallel clamp in the segment closest-point solve.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, Random.LogUniform(1e-5f, 3e-2f));
                break;
            case 3:
                //Near the 45 degree cap/side selection threshold.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, MathF.PI * 0.25f + (Random.NextSingle() * 2 - 1) * 0.05f);
                break;
            case 4:
                //Axis-aligned quarter turns: axes parallel or exactly perpendicular.
                testCase.OrientationA = Random.AxisAlignedOrientation();
                testCase.OrientationB = Random.AxisAlignedOrientation();
                break;
            default:
                testCase.OrientationA = Random.NextShapeOrientation();
                testCase.OrientationB = Random.NextShapeOrientation();
                break;
        }
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        var maxCapsuleExtent = testCase.A.HalfLength + testCase.A.Radius;
        var scale = MathF.Min(MathF.Max(testCase.A.HalfLength, testCase.A.Radius), MathF.Max(testCase.B.HalfLength, testCase.B.Radius));
        var span = maxCapsuleExtent + MathF.Max(testCase.B.HalfLength, testCase.B.Radius);

        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    var n = NextFeatureNormal(ra, rb);
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = CapsuleSupport(testCase.A, ra, n);
                    var supportB = CylinderSupport(testCase.B, rb, -n);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public struct TriangleCylinderCase
{
    public Triangle A;
    public Cylinder B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates triangle-cylinder cases. Triangle categories mirror BoxTriangleGenerator (axis-aligned right triangles, slivers,
/// needles, exact colinears, random, with vertex-frame recentering/offsets); cylinder categories mirror BoxCylinderGenerator
/// (discs, rods, random). Cylinder orientation categories emphasize the tester's branch boundaries: cap parallel to the
/// triangle plane (the 0.9999 interior-point blend threshold), cylinder axis parallel to a triangle edge (the side path's
/// 0.01-0.02 unrestrict band and dominant-edge ties), the 45 degree cap/side threshold, and axis-aligned exact ties.
/// Placement uses support matching along triangle face normals, cylinder cap/radial directions, rim blends, edge-edge
/// crosses, backface-threshold skimmers, and random directions.
/// </summary>
public class TriangleCylinderGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public TriangleCylinderGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Cylinder NextCylinder()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Cylinder { Radius = 1f, HalfLength = 1f };
            case 1:
                //Disc: much wider than long.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            case 2:
                //Rod: much longer than wide.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Cylinder { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            default:
                return new Cylinder { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    Triangle NextTriangle(out float triScale)
    {
        triScale = Random.LogUniform(1e-2f, 1e2f);
        Vector3 a, b, c;
        //Contact-heavy keeps the degenerate/near-degenerate categories (slivers, needles, colinear) but at lower weight:
        //they mostly produce structural rejections, which the mixed profile already samples heavily.
        switch (Random.Next(ContactHeavy ? 16 : 8))
        {
            case 0:
                //Axis-aligned right triangle in a coordinate plane: exact ties versus axis-aligned cylinders.
                a = default;
                switch (Random.Next(3))
                {
                    case 0: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, 0f, triScale); break;
                    case 1: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, triScale, 0f); break;
                    default: b = new Vector3(0f, triScale, 0f); c = new Vector3(0f, 0f, triScale); break;
                }
                break;
            case 1:
                //Near-degenerate sliver: height fraction straddles the 1e-6 DegenerateTriangleEpsilon boundary.
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = a + (b - a) * Random.NextSingle() + Random.UnitDirection() * (triScale * Random.LogUniform(1e-8f, 1e-4f));
                break;
            case 2:
                //Needle: two vertices nearly coincident, third far.
                a = Random.UnitDirection() * triScale;
                b = a + Random.UnitDirection() * (triScale * Random.LogUniform(1e-6f, 1e-3f));
                c = a + Random.UnitDirection() * triScale;
                break;
            case 3:
                //Exactly colinear: the nondegenerate mask must reject.
                {
                    a = Random.UnitDirection() * triScale;
                    var d = Random.UnitDirection();
                    b = a + d * triScale;
                    c = a + d * (triScale * Random.NextSingle());
                    break;
                }
            default:
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = Random.UnitDirection() * triScale;
                break;
        }
        //Swap winding half the time so both facings occur structurally.
        if (Random.Next(2) == 0)
            (b, c) = (c, b);
        //Vertex frame placement: meshes recenter triangles; mirror that most of the time.
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                break;
            case 2:
                //Far-offset vertex frame: stresses cancellation in the local transform and centroid recentering.
                {
                    var off = Random.UnitDirection() * (triScale * Random.LogUniform(10f, 1e3f));
                    a += off; b += off; c += off;
                    break;
                }
            default:
                {
                    var centroid = (a + b + c) * (1f / 3f);
                    a -= centroid; b -= centroid; c -= centroid;
                    break;
                }
        }
        return new Triangle { A = a, B = b, C = c };
    }

    static Vector3 CylinderSupport(in Cylinder cylinder, in Matrix3x3 orientation, in Vector3 direction)
    {
        var localDirection = ScalarMath.TransformByTransposed(direction, orientation);
        var localSupport = CylinderSupportScalar.ComputeLocalSupport(cylinder, localDirection);
        Matrix3x3.Transform(localSupport, orientation, out var support);
        return support;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    Quaternion NextOrientationB(Vector3 nFront, bool hasNormal, Vector3 wa, Vector3 wb, Vector3 wc)
    {
        switch (Random.Next(8))
        {
            case 0:
                return Quaternion.Identity;
            case 1:
                //Cap aligned to the triangle plane (near-parallel cap-face contact; the 0.9999 interior-point blend threshold).
                {
                    if (!hasNormal)
                        return Random.UnitQuaternion();
                    var localAxis = Vector3.UnitY * (Random.Next(2) == 0 ? 1f : -1f);
                    var q0 = RandomExtensions.ShortestArc(localAxis, -nFront);
                    var spin = Quaternion.CreateFromAxisAngle(nFront, Random.NextSingle() * MathF.PI * 2);
                    var q = Quaternion.Normalize(spin * q0);
                    return Random.Next(3) switch
                    {
                        0 => q,
                        1 => Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f)),
                        _ => Random.Perturb(q, Random.LogUniform(1e-3f, 1e-1f)),
                    };
                }
            case 2:
                //Cylinder axis parallel to a triangle edge: side-path unrestrict band and dominant-edge selection ties.
                {
                    var edge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var edgeLengthSquared = edge.LengthSquared();
                    if (!float.IsFinite(edgeLengthSquared) || edgeLengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    var e = edge / MathF.Sqrt(edgeLengthSquared);
                    var q0 = RandomExtensions.ShortestArc(Vector3.UnitY, e);
                    var spin = Quaternion.CreateFromAxisAngle(e, Random.NextSingle() * MathF.PI * 2);
                    var q = Quaternion.Normalize(spin * q0);
                    return Random.Next(2) == 0 ? q : Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f));
                }
            case 3:
                //45 degree tilt about an in-plane axis relative to a cap-aligned pose: the cap/side selection threshold.
                {
                    if (!hasNormal)
                        return Random.UnitQuaternion();
                    var q0 = RandomExtensions.ShortestArc(Vector3.UnitY, -nFront);
                    var inPlane = Vector3.Cross(nFront, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f));
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (inPlaneLengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    return Quaternion.Normalize(Quaternion.CreateFromAxisAngle(inPlane / MathF.Sqrt(inPlaneLengthSquared), MathF.PI * 0.25f + (Random.NextSingle() * 2 - 1) * 0.05f) * q0);
                }
            case 4:
                return Random.AxisAlignedOrientation();
            case 5:
                return Random.Perturb(Random.AxisAlignedOrientation(), Random.LogUniform(1e-8f, 1e-3f));
            default:
                return Random.UnitQuaternion();
        }
    }

    //n points from A (triangle) toward B (cylinder) per harness convention; the triangle's colliding side is +nFront
    //(the cylinder center must sit on the +cross(ab, ca) side to avoid the inside-and-below and backface rejections),
    //so contact-heavy calibration picks the sign with dot(n, nFront) >= 0.
    Vector3 NextFeatureNormal(in Matrix3x3 rb, Vector3 nFront, bool hasNormal, Vector3 wa, Vector3 wb, Vector3 wc)
    {
        //Real mesh workloads are dominated by triangle-face contacts; weight the face normal up for contact-heavy.
        if (ContactHeavy && hasNormal && Random.Next(2) == 0)
            return Random.Next(8) == 0 ? -nFront : nFront;
        switch (Random.Next(11))
        {
            case 0:
            case 1:
                //Triangle face.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    if (ContactHeavy)
                        return Random.Next(8) == 0 ? -nFront : nFront;
                    return Random.Next(2) == 0 ? nFront : -nFront;
                }
            case 2:
            case 3:
                //Cap of B: drives the cap path (|localNormal.Y| > 0.7071 branch).
                {
                    var axis = rb.Y;
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(axis, nFront) >= 0 ? axis : -axis;
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 4:
            case 5:
                //Radial (side) of B: drives the side path.
                {
                    var radial = RadialDirection(rb);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(radial, nFront) >= 0 ? radial : -radial;
                    return radial;
                }
            case 6:
                //Cap rim of B: blend of cap normal and radial, hitting the cap/side threshold region.
                {
                    var radial = RadialDirection(rb);
                    var axis = Random.Next(2) == 0 ? rb.Y : -rb.Y;
                    var blend = 0.5f + (Random.NextSingle() * 2 - 1) * 0.25f;
                    var candidate = axis * blend + radial * (1f - blend);
                    var lengthSquared = candidate.LengthSquared();
                    if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-10f)
                        return Random.UnitDirection();
                    var n = candidate / MathF.Sqrt(lengthSquared);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) >= 0 ? n : -n;
                    return n;
                }
            case 7:
                //Near-perpendicular to the cylinder axis with a small axial component: side path near the unrestrict band.
                {
                    var radial = RadialDirection(rb);
                    var tilt = Random.LogUniform(1e-4f, 5e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    var n = Vector3.Normalize(radial + rb.Y * tilt);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) >= 0 ? n : -n;
                    return n;
                }
            case 8:
                //Edge-edge: cross of the cylinder axis and a triangle edge.
                {
                    var triEdge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var cross = Vector3.Cross(rb.Y, triEdge);
                    var lengthSquared = cross.LengthSquared();
                    if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-10f)
                        return Random.UnitDirection();
                    var n = cross / MathF.Sqrt(lengthSquared);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) >= 0 ? n : -n;
                    return Random.Next(2) == 0 ? n : -n;
                }
            case 9:
                //Backface-threshold skimmer: nearly in-plane normal straddling the 1e-2 rejection threshold and the
                //0.2 triangle-edge-case threshold's low end.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    var triEdge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var inPlane = Vector3.Cross(triEdge, nFront);
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (!float.IsFinite(inPlaneLengthSquared) || inPlaneLengthSquared < 1e-12f)
                        return Random.UnitDirection();
                    inPlane /= MathF.Sqrt(inPlaneLengthSquared);
                    var u = Random.LogUniform(1e-4f, 3e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(inPlane + nFront * u);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public TriangleCylinderCase Next()
    {
        TriangleCylinderCase testCase;
        testCase.A = NextTriangle(out var triScale);
        testCase.B = NextCylinder();
        testCase.OrientationA = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.Transform(testCase.A.A, ra, out var wa);
        Matrix3x3.Transform(testCase.A.B, ra, out var wb);
        Matrix3x3.Transform(testCase.A.C, ra, out var wc);
        //The colliding side of the triangle is the +cross(ab, ca) side (ab = B-A, ca = A-C).
        var triangleCross = Vector3.Cross(wb - wa, wa - wc);
        var crossLengthSquared = triangleCross.LengthSquared();
        var hasNormal = float.IsFinite(crossLengthSquared) && crossLengthSquared > 0f;
        var nFront = hasNormal ? triangleCross / MathF.Sqrt(crossLengthSquared) : Vector3.UnitY;
        hasNormal &= float.IsFinite(nFront.X) && float.IsFinite(nFront.Y) && float.IsFinite(nFront.Z);

        testCase.OrientationB = NextOrientationB(nFront, hasNormal, wa, wb, wc);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        //Match the tester's epsilon scale heuristic (max of the cylinder's extents) crossed with the triangle scale.
        var cylScale = MathF.Max(testCase.B.HalfLength, testCase.B.Radius);
        var cylMinScale = MathF.Min(testCase.B.HalfLength, testCase.B.Radius);
        var vertReach = MathF.Max(wa.Length(), MathF.Max(wb.Length(), wc.Length()));
        var scale = MathF.Min(triScale, cylScale);
        var minScale = MathF.Min(triScale, cylMinScale);
        var span = vertReach + cylScale;
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Coincident frames: fully degenerate placement.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                //Purely random offset within the combined reach.
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                //Feature-informed support matching with controlled separation plus a tangential slide.
                {
                    var n = NextFeatureNormal(rb, nFront, hasNormal, wa, wb, wc);
                    //Contact-heavy scales penetrations by the smaller extents: a penetration deeper than the cylinder's thin
                    //axis pushes its center through the one-sided triangle's plane, flipping into inside-and-below/backface rejection.
                    var separation = Random.NextSeparation(ContactHeavy ? minScale : scale, testCase.SpeculativeMargin, ContactHeavy);
                    var negatedN = -n;
                    var dotA = Vector3.Dot(wa, n);
                    var dotB = Vector3.Dot(wb, n);
                    var dotC = Vector3.Dot(wc, n);
                    var supportA = dotA > dotB ? dotA > dotC ? wa : wc : dotB > dotC ? wb : wc;
                    var supportB = CylinderSupport(testCase.B, rb, negatedN);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation,
                        ContactHeavy ? minScale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public struct CapsulePairCase
{
    public Capsule A;
    public Capsule B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates capsule-capsule cases. Orientation categories emphasize the closed-form tester's branch boundaries: exactly and
/// nearly parallel axes (the 1e-15 parallel clamp and the coplanarity interval fade, whose squared-angle thresholds correspond
/// to axis angles of 0.01 to 0.05 radians), exactly and nearly perpendicular axes (the |dadb| &lt; 1e-7 unprojection select),
/// and axis-aligned exact ties. Placement uses support matching along the capsule axes, radial directions, endpoint blends,
/// small-tilt fade-band directions, and random directions; coincident centers hit the touching-segment normal fallback.
/// </summary>
public class CapsulePairGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public CapsulePairGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Capsule NextCapsule()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Capsule { Radius = 0.5f, HalfLength = 1f };
            case 1:
                //Long thin rod: stresses the parallel clamps and the coplanar interval expansion.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            case 2:
                //Squat, nearly spherical: halfLength much smaller than radius; Contact1's aMax - aMin > 1e-7 * halfLength threshold.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            default:
                return new Capsule { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    static Vector3 CapsuleSupport(in Capsule capsule, in Matrix3x3 orientation, in Vector3 direction)
    {
        var endpoint = orientation.Y * (Vector3.Dot(orientation.Y, direction) > 0 ? capsule.HalfLength : -capsule.HalfLength);
        return endpoint + direction * capsule.Radius;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    Vector3 NextFeatureNormal(in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(10))
        {
            case 0:
            case 1:
                //Axis of A: endpoint-vs-endpoint and deep endpoint cases.
                return Random.Next(2) == 0 ? ra.Y : -ra.Y;
            case 2:
                //Axis of B.
                return Random.Next(2) == 0 ? rb.Y : -rb.Y;
            case 3:
            case 4:
                //Radial (side) of A: side-side contact, the two-contact coplanar interval when axes are near parallel.
                return RadialDirection(ra);
            case 5:
                //Radial of B.
                return RadialDirection(rb);
            case 6:
                //Endpoint blend of A: between axis and radial, sliding contacts across the segment clamp boundaries.
                {
                    var radial = RadialDirection(ra);
                    var axis = Random.Next(2) == 0 ? ra.Y : -ra.Y;
                    var blend = 0.5f + (Random.NextSingle() * 2 - 1) * 0.25f;
                    return Vector3.Normalize(axis * blend + radial * (1f - blend));
                }
            case 7:
            case 8:
                //Near-perpendicular to A's axis with a small axial tilt: normals in and around the coplanarity fade band.
                {
                    var radial = RadialDirection(ra);
                    var tilt = Random.LogUniform(1e-4f, 5e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(radial + ra.Y * tilt);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public CapsulePairCase Next()
    {
        CapsulePairCase testCase;
        testCase.A = NextCapsule();
        testCase.B = NextCapsule();

        switch (Random.Next(10))
        {
            case 0:
                //Exactly parallel, axis aligned.
                testCase.OrientationA = Quaternion.Identity;
                testCase.OrientationB = Quaternion.Identity;
                break;
            case 1:
                //Identical random orientations: exactly parallel axes.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = testCase.OrientationA;
                break;
            case 2:
            case 3:
                //Nearly parallel: spans the 1e-15 denominator clamp and the 0.01..0.05 radian coplanarity fade band.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, Random.LogUniform(1e-6f, 1.5e-1f));
                break;
            case 4:
                //Exactly or nearly perpendicular axes: the |dadb| < 1e-7 unprojection select.
                {
                    testCase.OrientationA = Random.UnitQuaternion();
                    var quarterTurn = Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI * 0.5f);
                    var perpendicular = Quaternion.Normalize(testCase.OrientationA * quarterTurn);
                    testCase.OrientationB = Random.Next(2) == 0 ? perpendicular : Random.Perturb(perpendicular, Random.LogUniform(1e-8f, 1e-2f));
                    break;
                }
            case 5:
                //Axis-aligned quarter turns: axes exactly parallel or exactly perpendicular.
                testCase.OrientationA = Random.AxisAlignedOrientation();
                testCase.OrientationB = Random.AxisAlignedOrientation();
                break;
            default:
                testCase.OrientationA = Random.NextShapeOrientation();
                testCase.OrientationB = Random.NextShapeOrientation();
                break;
        }
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        var scale = MathF.Min(MathF.Max(testCase.A.HalfLength, testCase.A.Radius), MathF.Max(testCase.B.HalfLength, testCase.B.Radius));
        var span = testCase.A.HalfLength + testCase.A.Radius + testCase.B.HalfLength + testCase.B.Radius;

        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Coincident centers: distance can be below the 1e-7 normal validity threshold, forcing the xa fallback.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    var n = NextFeatureNormal(ra, rb);
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = CapsuleSupport(testCase.A, ra, n);
                    var supportB = CapsuleSupport(testCase.B, rb, -n);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public struct CapsuleBoxCase
{
    public Capsule A;
    public Box B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates capsule-box cases. Orientation categories emphasize the tester's branch boundaries: capsule axis parallel to box
/// faces and edges (degenerate edge normals and their (-axisY, axisX, 0) then (1,0,0) fallbacks, plus the |tangentSpaceAxis| &lt; 1e-15
/// slab fallbacks in the face interval clip), axis-aligned exact ties in the representative-face selection, and near-parallel
/// perturbations spanning the epsilon bands. Placement uses support matching along box face normals, box edge wedge blends,
/// capsule axis/radial directions, and random directions.
/// </summary>
public class CapsuleBoxGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public CapsuleBoxGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Capsule NextCapsule()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Capsule { Radius = 0.5f, HalfLength = 1f };
            case 1:
                //Long thin rod: stresses the edge closest-point solves and long face intervals.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            case 2:
                //Squat, nearly spherical: halfLength much smaller than radius.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            default:
                return new Capsule { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    Box NextBox()
    {
        switch (Random.Next(6))
        {
            case 0:
                return new Box { HalfWidth = 1f, HalfHeight = 1f, HalfLength = 1f };
            case 1:
                {
                    //Plate: one dimension much smaller.
                    var size = Random.LogUniform(0.1f, 10f);
                    var thin = size * Random.LogUniform(1e-3f, 1e-1f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = thin; break;
                        case 1: box.HalfHeight = thin; break;
                        default: box.HalfLength = thin; break;
                    }
                    return box;
                }
            case 2:
                {
                    //Rod: one dimension much larger.
                    var size = Random.LogUniform(0.1f, 10f);
                    var lengthy = size * Random.LogUniform(10f, 1000f);
                    var box = new Box { HalfWidth = size, HalfHeight = size, HalfLength = size };
                    switch (Random.Next(3))
                    {
                        case 0: box.HalfWidth = lengthy; break;
                        case 1: box.HalfHeight = lengthy; break;
                        default: box.HalfLength = lengthy; break;
                    }
                    return box;
                }
            default:
                return new Box
                {
                    HalfWidth = Random.LogUniform(1e-2f, 1e2f),
                    HalfHeight = Random.LogUniform(1e-2f, 1e2f),
                    HalfLength = Random.LogUniform(1e-2f, 1e2f),
                };
        }
    }

    static Vector3 CapsuleSupport(in Capsule capsule, in Matrix3x3 orientation, in Vector3 direction)
    {
        var endpoint = orientation.Y * (Vector3.Dot(orientation.Y, direction) > 0 ? capsule.HalfLength : -capsule.HalfLength);
        return endpoint + direction * capsule.Radius;
    }

    static Vector3 BoxSupport(in Box box, in Matrix3x3 orientation, in Vector3 direction)
    {
        var x = Vector3.Dot(orientation.X, direction) > 0 ? box.HalfWidth : -box.HalfWidth;
        var y = Vector3.Dot(orientation.Y, direction) > 0 ? box.HalfHeight : -box.HalfHeight;
        var z = Vector3.Dot(orientation.Z, direction) > 0 ? box.HalfLength : -box.HalfLength;
        return orientation.X * x + orientation.Y * y + orientation.Z * z;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    static Vector3 Axis(in Matrix3x3 orientation, int index) => index switch { 0 => orientation.X, 1 => orientation.Y, _ => orientation.Z };

    Vector3 NextFeatureNormal(in Matrix3x3 ra, in Matrix3x3 rb)
    {
        switch (Random.Next(12))
        {
            case 0:
            case 1:
            case 2:
                //Box face of B: drives the face candidates and the representative-face interval clip.
                {
                    var axis = Axis(rb, Random.Next(3));
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 3:
            case 4:
                //Box edge wedge of B: normal in the wedge between two box faces, driving the edge candidates.
                {
                    var first = Axis(rb, Random.Next(3));
                    var second = Axis(rb, Random.Next(3));
                    var blend = Random.NextSingle();
                    var candidate = first * blend + second * (1f - blend);
                    if (candidate.LengthSquared() < 1e-6f)
                        return Random.UnitDirection();
                    return Vector3.Normalize(candidate);
                }
            case 5:
            case 6:
                //Capsule axis: endpoint-vs-face and deep endpoint cases.
                return Random.Next(2) == 0 ? ra.Y : -ra.Y;
            case 7:
                //Radial of A: side contact against faces and edges.
                return RadialDirection(ra);
            case 8:
                //Box face of B with a tiny tilt: near-parallel face contact, spanning the tangent-axis slab fallback band.
                {
                    var axis = Axis(rb, Random.Next(3));
                    if (Random.Next(2) == 0)
                        axis = -axis;
                    var tilt = Random.LogUniform(1e-4f, 5e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    var tangent = Axis(rb, Random.Next(3));
                    var candidate = axis + tangent * tilt;
                    return Vector3.Normalize(candidate);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public CapsuleBoxCase Next()
    {
        CapsuleBoxCase testCase;
        testCase.A = NextCapsule();
        testCase.B = NextBox();

        switch (Random.Next(8))
        {
            case 0:
                //Exactly axis aligned: exact ties in the representative-face selection and degenerate edge normals.
                testCase.OrientationA = Quaternion.Identity;
                testCase.OrientationB = Quaternion.Identity;
                break;
            case 1:
                //Identical random orientations: capsule axis parallel to box face normal Y.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = testCase.OrientationA;
                break;
            case 2:
                //Nearly parallel: stresses the 1e-15 parallel clamps in the edge solve and the slab fallbacks.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, Random.LogUniform(1e-6f, 3e-2f));
                break;
            case 3:
                //Around 45 degrees: edge-vs-face depth ties in the select chain.
                testCase.OrientationA = Random.UnitQuaternion();
                testCase.OrientationB = Random.Perturb(testCase.OrientationA, MathF.PI * 0.25f + (Random.NextSingle() * 2 - 1) * 0.05f);
                break;
            case 4:
                //Axis-aligned quarter turns: capsule axis exactly parallel or perpendicular to box faces.
                testCase.OrientationA = Random.AxisAlignedOrientation();
                testCase.OrientationB = Random.AxisAlignedOrientation();
                break;
            default:
                testCase.OrientationA = Random.NextShapeOrientation();
                testCase.OrientationB = Random.NextShapeOrientation();
                break;
        }
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);

        //Match the tester's epsilon scale heuristic: min(max box extent, max capsule extent).
        var maxBoxExtent = MathF.Max(testCase.B.HalfWidth, MathF.Max(testCase.B.HalfHeight, testCase.B.HalfLength));
        var scale = MathF.Min(maxBoxExtent, MathF.Max(testCase.A.HalfLength, testCase.A.Radius));
        var span = testCase.A.HalfLength + testCase.A.Radius + maxBoxExtent;

        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Coincident centers: deep intersection, degenerate edge normals and their fallbacks.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                {
                    var n = NextFeatureNormal(ra, rb);
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = CapsuleSupport(testCase.A, ra, n);
                    var supportB = BoxSupport(testCase.B, rb, -n);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation, ContactHeavy ? scale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public struct CapsuleTriangleCase
{
    public Capsule A;
    public Triangle B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates capsule-triangle cases. Orientation categories emphasize the tester's branch boundaries: capsule axis in the
/// triangle plane and parallel to triangle edges (the 0.01-0.05 rad coplanarity fade band, the parallel edge normal fallbacks,
/// and the 1e-15 parallel clamps), axis along the face normal (the |faceNormalADotLocalNormal| &lt; 1e-7 manifold collapse), and
/// near-axis-aligned perturbations. Placement uses support matching along the triangle face normal (weighted up for
/// contact-heavy; triangles are one-sided), capsule axis/radial directions, axis-edge crosses, backface skimmers, and random
/// directions. Triangle categories keep slivers/needles/colinear degenerates for the nondegenerate mask and fallback paths.
/// </summary>
public class CapsuleTriangleGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public CapsuleTriangleGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Capsule NextCapsule()
    {
        switch (Random.Next(5))
        {
            case 0:
                return new Capsule { Radius = 0.5f, HalfLength = 1f };
            case 1:
                //Long thin rod: stresses the edge closest-point solves and long clipped face intervals.
                {
                    var radius = Random.LogUniform(0.05f, 2f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(5f, 100f) };
                }
            case 2:
                //Squat, nearly spherical: halfLength much smaller than radius.
                {
                    var radius = Random.LogUniform(0.1f, 10f);
                    return new Capsule { Radius = radius, HalfLength = radius * Random.LogUniform(0.01f, 0.2f) };
                }
            default:
                return new Capsule { Radius = Random.LogUniform(0.05f, 20f), HalfLength = Random.LogUniform(0.05f, 20f) };
        }
    }

    Triangle NextTriangle(out float triScale)
    {
        triScale = Random.LogUniform(1e-2f, 1e2f);
        Vector3 a, b, c;
        //Contact-heavy keeps the degenerate/near-degenerate categories (slivers, needles, colinear) but at lower weight:
        //they mostly produce structural rejections, which the mixed profile already samples heavily.
        switch (Random.Next(ContactHeavy ? 16 : 8))
        {
            case 0:
                //Axis-aligned right triangle in a coordinate plane: exact ties versus axis-aligned capsule axes.
                a = default;
                switch (Random.Next(3))
                {
                    case 0: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, 0f, triScale); break;
                    case 1: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, triScale, 0f); break;
                    default: b = new Vector3(0f, triScale, 0f); c = new Vector3(0f, 0f, triScale); break;
                }
                break;
            case 1:
                //Near-degenerate sliver: height fraction straddles the 1e-6 DegenerateTriangleEpsilon boundary.
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = a + (b - a) * Random.NextSingle() + Random.UnitDirection() * (triScale * Random.LogUniform(1e-8f, 1e-4f));
                break;
            case 2:
                //Needle: two vertices nearly coincident, third far.
                a = Random.UnitDirection() * triScale;
                b = a + Random.UnitDirection() * (triScale * Random.LogUniform(1e-6f, 1e-3f));
                c = a + Random.UnitDirection() * triScale;
                break;
            case 3:
                //Exactly colinear: the nondegenerate mask must reject; also drives the second normal fallback in TestEdge.
                {
                    a = Random.UnitDirection() * triScale;
                    var d = Random.UnitDirection();
                    b = a + d * triScale;
                    c = a + d * (triScale * Random.NextSingle());
                    break;
                }
            default:
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = Random.UnitDirection() * triScale;
                break;
        }
        //Swap winding half the time so both facings occur structurally.
        if (Random.Next(2) == 0)
            (b, c) = (c, b);
        //Vertex frame placement: meshes recenter triangles; mirror that most of the time.
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                break;
            case 2:
                //Far-offset vertex frame: stresses cancellation in the centroid recentering and clip t values.
                {
                    var off = Random.UnitDirection() * (triScale * Random.LogUniform(10f, 1e3f));
                    a += off; b += off; c += off;
                    break;
                }
            default:
                {
                    var centroid = (a + b + c) * (1f / 3f);
                    a -= centroid; b -= centroid; c -= centroid;
                    break;
                }
        }
        return new Triangle { A = a, B = b, C = c };
    }

    static Vector3 CapsuleSupport(in Capsule capsule, in Matrix3x3 orientation, in Vector3 direction)
    {
        var endpoint = orientation.Y * (Vector3.Dot(orientation.Y, direction) > 0 ? capsule.HalfLength : -capsule.HalfLength);
        return endpoint + direction * capsule.Radius;
    }

    Vector3 RadialDirection(in Matrix3x3 orientation)
    {
        var angle = Random.NextSingle() * MathF.PI * 2;
        return orientation.X * MathF.Cos(angle) + orientation.Z * MathF.Sin(angle);
    }

    Quaternion NextOrientationA(Vector3 nFront, bool hasNormal, Vector3 wa, Vector3 wb, Vector3 wc)
    {
        switch (Random.Next(8))
        {
            case 0:
                return Quaternion.Identity;
            case 1:
                //Capsule axis lying in the triangle plane: coplanar edge intervals; perturbations span the 0.01-0.05 rad
                //coplanarity fade band and the near-parallel clamps.
                {
                    if (!hasNormal)
                        return Random.UnitQuaternion();
                    var inPlane = Vector3.Cross(nFront, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f));
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (inPlaneLengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    var q = RandomExtensions.ShortestArc(Vector3.UnitY, inPlane / MathF.Sqrt(inPlaneLengthSquared));
                    return Random.Next(3) switch
                    {
                        0 => q,
                        1 => Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f)),
                        _ => Random.Perturb(q, Random.LogUniform(1e-3f, 1.5e-1f)),
                    };
                }
            case 2:
                //Capsule axis parallel to a triangle edge: parallel segment-segment solves (1e-15 clamp), zero-length
                //cross fallbacks, and full-interval coplanar contacts.
                {
                    var edge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var edgeLengthSquared = edge.LengthSquared();
                    if (!float.IsFinite(edgeLengthSquared) || edgeLengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    var e = edge / MathF.Sqrt(edgeLengthSquared);
                    var q = RandomExtensions.ShortestArc(Vector3.UnitY, e);
                    return Random.Next(3) switch
                    {
                        0 => q,
                        1 => Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f)),
                        _ => Random.Perturb(q, Random.LogUniform(1e-3f, 1.5e-1f)),
                    };
                }
            case 3:
                //Capsule axis along the face normal: endpoint-vs-face and the |faceNormalADotLocalNormal| < 1e-7 collapse.
                {
                    if (!hasNormal)
                        return Random.UnitQuaternion();
                    var q = RandomExtensions.ShortestArc(Vector3.UnitY, Random.Next(2) == 0 ? nFront : -nFront);
                    return Random.Next(2) == 0 ? q : Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f));
                }
            case 4:
                return Random.AxisAlignedOrientation();
            case 5:
                return Random.Perturb(Random.AxisAlignedOrientation(), Random.LogUniform(1e-8f, 1e-3f));
            default:
                return Random.UnitQuaternion();
        }
    }

    //n points from A (capsule) toward B (triangle) per harness convention; the triangle's colliding side is +nFront,
    //so contact-heavy calibration picks the sign with dot(n, nFront) <= 0 (capsule on the front side).
    Vector3 NextFeatureNormal(in Matrix3x3 ra, Vector3 nFront, bool hasNormal, Vector3 wa, Vector3 wb, Vector3 wc)
    {
        //Real mesh workloads are dominated by triangle-face contacts, and the triangle's one-sidedness plus its lack of
        //volume make the other feature normals miss much more often than for solids; weight the face normal up for contact-heavy.
        if (ContactHeavy && hasNormal && Random.Next(2) == 0)
            return Random.Next(8) == 0 ? nFront : -nFront;
        switch (Random.Next(10))
        {
            case 0:
            case 1:
                //Triangle face.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    if (ContactHeavy)
                        return Random.Next(8) == 0 ? nFront : -nFront;
                    return Random.Next(2) == 0 ? nFront : -nFront;
                }
            case 2:
            case 3:
                //Capsule axis: endpoint digs into the triangle face or an edge.
                {
                    var axis = ra.Y;
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(axis, nFront) <= 0 ? axis : -axis;
                    return Random.Next(2) == 0 ? axis : -axis;
                }
            case 4:
            case 5:
                //Edge-edge: cross of the capsule axis and a triangle edge.
                {
                    var triEdge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var cross = Vector3.Cross(ra.Y, triEdge);
                    var lengthSquared = cross.LengthSquared();
                    if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-10f)
                        return Random.UnitDirection();
                    var n = cross / MathF.Sqrt(lengthSquared);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) <= 0 ? n : -n;
                    return Random.Next(2) == 0 ? n : -n;
                }
            case 6:
                //Radial of A: capsule side against the face or an edge.
                {
                    var n = RadialDirection(ra);
                    if (ContactHeavy & hasNormal)
                        return Vector3.Dot(n, nFront) <= 0 ? n : -n;
                    return n;
                }
            case 7:
                //Backface-threshold skimmer: nearly in-plane normal straddling the -1e-2 rejection threshold.
                {
                    if (!hasNormal)
                        return Random.UnitDirection();
                    var triEdge = Random.Next(3) switch { 0 => wb - wa, 1 => wc - wb, _ => wa - wc };
                    var inPlane = Vector3.Cross(triEdge, nFront);
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (!float.IsFinite(inPlaneLengthSquared) || inPlaneLengthSquared < 1e-12f)
                        return Random.UnitDirection();
                    inPlane /= MathF.Sqrt(inPlaneLengthSquared);
                    var u = Random.LogUniform(1e-4f, 3e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(inPlane + nFront * u);
                }
            default:
                return Random.UnitDirection();
        }
    }

    public CapsuleTriangleCase Next()
    {
        CapsuleTriangleCase testCase;
        testCase.A = NextCapsule();
        testCase.B = NextTriangle(out var triScale);
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        Matrix3x3.Transform(testCase.B.A, rb, out var wa);
        Matrix3x3.Transform(testCase.B.B, rb, out var wb);
        Matrix3x3.Transform(testCase.B.C, rb, out var wc);
        //The colliding side of the triangle is the +cross(ab, ca) side (ab = B-A, ca = A-C), matching cross(ac, ab) in the tester.
        var triangleCross = Vector3.Cross(wb - wa, wa - wc);
        var crossLengthSquared = triangleCross.LengthSquared();
        var hasNormal = float.IsFinite(crossLengthSquared) && crossLengthSquared > 0f;
        var nFront = hasNormal ? triangleCross / MathF.Sqrt(crossLengthSquared) : Vector3.UnitY;
        hasNormal &= float.IsFinite(nFront.X) && float.IsFinite(nFront.Y) && float.IsFinite(nFront.Z);

        testCase.OrientationA = NextOrientationA(nFront, hasNormal, wa, wb, wc);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);

        var maxCapsuleExtent = MathF.Max(testCase.A.HalfLength, testCase.A.Radius);
        var vertReach = MathF.Max(wa.Length(), MathF.Max(wb.Length(), wc.Length()));
        var scale = MathF.Min(maxCapsuleExtent, triScale);
        //Contact-heavy scales penetrations by the smallest extent: a penetration deeper than the capsule's radius pushes the
        //capsule center through the one-sided triangle's plane, flipping into backface rejection.
        var minScale = MathF.Min(MathF.Min(testCase.A.Radius, testCase.A.HalfLength), triScale);
        var span = testCase.A.HalfLength + testCase.A.Radius + vertReach;
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 9 : Random.Next(10))
        {
            case 0:
                //Coincident frames: fully degenerate placement, capsule axis intersecting the triangle.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                //Purely random offset within the combined reach.
                testCase.OffsetB = new Vector3(
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span,
                    (Random.NextSingle() * 2 - 1) * span);
                break;
            default:
                //Feature-informed support matching with controlled separation plus a tangential slide.
                {
                    var n = NextFeatureNormal(ra, nFront, hasNormal, wa, wb, wc);
                    var separation = Random.NextSeparation(ContactHeavy ? minScale : scale, testCase.SpeculativeMargin, ContactHeavy);
                    var supportA = CapsuleSupport(testCase.A, ra, n);
                    var negatedN = -n;
                    var dotA = Vector3.Dot(wa, negatedN);
                    var dotB = Vector3.Dot(wb, negatedN);
                    var dotC = Vector3.Dot(wc, negatedN);
                    var supportB = dotA > dotB ? dotA > dotC ? wa : wc : dotB > dotC ? wb : wc;
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation,
                        ContactHeavy ? minScale : span, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}

public struct TrianglePairCase
{
    public Triangle A;
    public Triangle B;
    public float SpeculativeMargin;
    public Vector3 OffsetB;
    public Quaternion OrientationA;
    public Quaternion OrientationB;
}

/// <summary>
/// Generates triangle-triangle cases. Triangle categories mirror BoxTriangleGenerator (axis-aligned right triangles, slivers,
/// needles, colinear, random; winding swaps; vertex-frame offsets). The pair only collides when the one-sided triangles face
/// each other — the engine requires dot(localNormal, faceNormalA) &lt; 1e-2 and dot(localNormal, faceNormalB) &gt; -1e-2 with
/// localNormal pointing B to A — so orientationA categories align A's face or edges against B's world features, and
/// contact-heavy feature normals are calibrated to keep A's front toward B and B's front toward A.
/// </summary>
public class TrianglePairGenerator
{
    public Random Random;
    public bool ContactHeavy;
    public TrianglePairGenerator(int seed, bool contactHeavy = false) { Random = new Random(seed); ContactHeavy = contactHeavy; }

    Triangle NextTriangle(out float triScale)
    {
        triScale = Random.LogUniform(1e-2f, 1e2f);
        Vector3 a, b, c;
        //Contact-heavy keeps the degenerate/near-degenerate categories (slivers, needles, colinear) but at much lower weight:
        //both triangles must be nondegenerate for any contact, so the degeneracy rejection probability compounds across the pair.
        switch (Random.Next(ContactHeavy ? 24 : 8))
        {
            case 0:
                //Axis-aligned right triangle in a coordinate plane: exact ties in the edge-edge SAT and clipping.
                a = default;
                switch (Random.Next(3))
                {
                    case 0: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, 0f, triScale); break;
                    case 1: b = new Vector3(triScale, 0f, 0f); c = new Vector3(0f, triScale, 0f); break;
                    default: b = new Vector3(0f, triScale, 0f); c = new Vector3(0f, 0f, triScale); break;
                }
                break;
            case 1:
                //Near-degenerate sliver: height fraction straddles the 1e-6 DegenerateTriangleEpsilon boundary.
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = a + (b - a) * Random.NextSingle() + Random.UnitDirection() * (triScale * Random.LogUniform(1e-8f, 1e-4f));
                break;
            case 2:
                //Needle: two vertices nearly coincident, third far.
                a = Random.UnitDirection() * triScale;
                b = a + Random.UnitDirection() * (triScale * Random.LogUniform(1e-6f, 1e-3f));
                c = a + Random.UnitDirection() * triScale;
                break;
            case 3:
                //Exactly colinear: the nondegenerate mask must reject; also hits the 1e-10 edge-SAT guard.
                {
                    a = Random.UnitDirection() * triScale;
                    var d = Random.UnitDirection();
                    b = a + d * triScale;
                    c = a + d * (triScale * Random.NextSingle());
                    break;
                }
            default:
                a = Random.UnitDirection() * triScale;
                b = Random.UnitDirection() * triScale;
                c = Random.UnitDirection() * triScale;
                break;
        }
        //Swap winding half the time so both facings occur structurally.
        if (Random.Next(2) == 0)
            (b, c) = (c, b);
        //Vertex frame placement: meshes recenter triangles; mirror that most of the time.
        switch (Random.Next(8))
        {
            case 0:
            case 1:
                break;
            case 2:
                //Far-offset vertex frame: stresses cancellation in the local transform and clip t values.
                {
                    var off = Random.UnitDirection() * (triScale * Random.LogUniform(10f, 1e3f));
                    a += off; b += off; c += off;
                    break;
                }
            default:
                {
                    var centroid = (a + b + c) * (1f / 3f);
                    a -= centroid; b -= centroid; c -= centroid;
                    break;
                }
        }
        return new Triangle { A = a, B = b, C = c };
    }

    //Front normal convention matching the tester: faceNormal = cross(ab, ca) with ab = B - A, ca = A - C.
    static bool TryGetFrontNormal(Vector3 a, Vector3 b, Vector3 c, out Vector3 nFront)
    {
        var cross = Vector3.Cross(b - a, a - c);
        var lengthSquared = cross.LengthSquared();
        if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-14f)
        {
            nFront = Vector3.UnitY;
            return false;
        }
        nFront = cross / MathF.Sqrt(lengthSquared);
        return float.IsFinite(nFront.X) && float.IsFinite(nFront.Y) && float.IsFinite(nFront.Z);
    }

    //Orientation for A relative to B's world features. A's colliding pose wants its front normal roughly opposing B's.
    Quaternion NextOrientationA(Vector3 nFrontALocal, bool hasNormalA, Vector3 nFrontBWorld, bool hasNormalB, Vector3 wbA, Vector3 wbB, Vector3 wbC, Triangle a)
    {
        switch (Random.Next(ContactHeavy ? 4 : 8))
        {
            case 0:
            case 1 when ContactHeavy:
                //Face-face: A's front normal rotated onto -B's front normal (plus spin and optional perturbation).
                {
                    if (!hasNormalA || !hasNormalB)
                        return Random.UnitQuaternion();
                    var q0 = RandomExtensions.ShortestArc(nFrontALocal, -nFrontBWorld);
                    var spin = Quaternion.CreateFromAxisAngle(nFrontBWorld, Random.NextSingle() * MathF.PI * 2);
                    var q = Quaternion.Normalize(spin * q0);
                    return Random.Next(3) switch
                    {
                        0 => q,
                        1 => Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f)),
                        _ => Random.Perturb(q, Random.LogUniform(1e-3f, 1e-1f)),
                    };
                }
            case 1:
            case 2:
                //Edge parallel to a B edge: exact and near-degenerate edge-edge SAT axes plus the 1e-20 clip parallel guard.
                {
                    var edgeB = Random.Next(3) switch { 0 => wbB - wbA, 1 => wbC - wbB, _ => wbA - wbC };
                    var edgeBLengthSquared = edgeB.LengthSquared();
                    var edgeA = Random.Next(3) switch { 0 => a.B - a.A, 1 => a.C - a.B, _ => a.A - a.C };
                    var edgeALengthSquared = edgeA.LengthSquared();
                    if (!float.IsFinite(edgeBLengthSquared) || edgeBLengthSquared < 1e-12f || !float.IsFinite(edgeALengthSquared) || edgeALengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    var eB = edgeB / MathF.Sqrt(edgeBLengthSquared);
                    var eA = edgeA / MathF.Sqrt(edgeALengthSquared);
                    var q0 = RandomExtensions.ShortestArc(eA, Random.Next(2) == 0 ? eB : -eB);
                    var spin = Quaternion.CreateFromAxisAngle(eB, Random.NextSingle() * MathF.PI * 2);
                    var q = Quaternion.Normalize(spin * q0);
                    return Random.Next(2) == 0 ? q : Random.Perturb(q, Random.LogUniform(1e-8f, 1e-3f));
                }
            case 3:
                //Near-coplanar tilt: face-face pose tilted by an angle spanning the |dot| < 0.2 edge-case threshold
                //and the backface rejection band.
                {
                    if (!hasNormalA || !hasNormalB)
                        return Random.UnitQuaternion();
                    var q0 = Quaternion.Normalize(Quaternion.CreateFromAxisAngle(nFrontBWorld, Random.NextSingle() * MathF.PI * 2) * RandomExtensions.ShortestArc(nFrontALocal, -nFrontBWorld));
                    var inPlane = Vector3.Cross(nFrontBWorld, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f));
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (inPlaneLengthSquared < 1e-12f)
                        return Random.UnitQuaternion();
                    var angle = Random.Next(3) switch
                    {
                        0 => MathF.PI * 0.25f,
                        1 => MathF.Acos(MathF.Min(1f, 0.2f + (Random.NextSingle() * 2 - 1) * 0.05f)),
                        _ => MathF.PI * 0.5f + (Random.NextSingle() * 2 - 1) * 3e-2f,
                    };
                    return Quaternion.Normalize(Quaternion.CreateFromAxisAngle(inPlane / MathF.Sqrt(inPlaneLengthSquared), angle) * q0);
                }
            case 4:
                return Quaternion.Identity;
            case 5:
                return Random.AxisAlignedOrientation();
            case 6:
                return Random.Perturb(Random.AxisAlignedOrientation(), Random.LogUniform(1e-8f, 1e-3f));
            default:
                return Random.UnitQuaternion();
        }
    }

    //n points from A toward B per harness convention. Contact needs A's front along n and B's front against n,
    //so contact-heavy calibration picks dot(n, nFrontA) >= 0 (and prefers the B face normal case's -nFrontB).
    Vector3 NextFeatureNormal(Vector3 nFrontAWorld, bool hasNormalA, Vector3 nFrontBWorld, bool hasNormalB,
        Vector3 waA, Vector3 waB, Vector3 waC, Vector3 wbA, Vector3 wbB, Vector3 wbC)
    {
        //Real mesh workloads are dominated by face contacts; weight the face normals up for contact-heavy.
        if (ContactHeavy && hasNormalA && Random.Next(2) == 0)
            return Random.Next(8) == 0 ? -nFrontAWorld : nFrontAWorld;
        switch (Random.Next(10))
        {
            case 0:
            case 1:
                //Triangle A face.
                {
                    if (!hasNormalA)
                        return Random.UnitDirection();
                    if (ContactHeavy)
                        return Random.Next(8) == 0 ? -nFrontAWorld : nFrontAWorld;
                    return Random.Next(2) == 0 ? nFrontAWorld : -nFrontAWorld;
                }
            case 2:
            case 3:
                //Triangle B face: n opposing B's front so B faces back toward A.
                {
                    if (!hasNormalB)
                        return Random.UnitDirection();
                    if (ContactHeavy)
                        return Random.Next(8) == 0 ? nFrontBWorld : -nFrontBWorld;
                    return Random.Next(2) == 0 ? nFrontBWorld : -nFrontBWorld;
                }
            case 4:
            case 5:
                //Edge-edge: cross of an A edge and a B edge.
                {
                    var edgeA = Random.Next(3) switch { 0 => waB - waA, 1 => waC - waB, _ => waA - waC };
                    var edgeB = Random.Next(3) switch { 0 => wbB - wbA, 1 => wbC - wbB, _ => wbA - wbC };
                    var cross = Vector3.Cross(edgeA, edgeB);
                    var lengthSquared = cross.LengthSquared();
                    if (!float.IsFinite(lengthSquared) || lengthSquared < 1e-10f)
                        return Random.UnitDirection();
                    var n = cross / MathF.Sqrt(lengthSquared);
                    if (ContactHeavy & hasNormalA)
                        return Vector3.Dot(n, nFrontAWorld) >= 0 ? n : -n;
                    return Random.Next(2) == 0 ? n : -n;
                }
            case 6:
            case 7:
                //Backface-threshold skimmer: nearly in-plane normal straddling the +-1e-2 rejection thresholds.
                {
                    var useB = Random.Next(2) == 0;
                    var hasNormal = useB ? hasNormalB : hasNormalA;
                    var nFront = useB ? nFrontBWorld : nFrontAWorld;
                    if (!hasNormal)
                        return Random.UnitDirection();
                    var edge = useB
                        ? Random.Next(3) switch { 0 => wbB - wbA, 1 => wbC - wbB, _ => wbA - wbC }
                        : Random.Next(3) switch { 0 => waB - waA, 1 => waC - waB, _ => waA - waC };
                    var inPlane = Vector3.Cross(edge, nFront);
                    var inPlaneLengthSquared = inPlane.LengthSquared();
                    if (!float.IsFinite(inPlaneLengthSquared) || inPlaneLengthSquared < 1e-12f)
                        return Random.UnitDirection();
                    inPlane /= MathF.Sqrt(inPlaneLengthSquared);
                    var u = Random.LogUniform(1e-4f, 3e-2f) * (Random.Next(2) == 0 ? 1f : -1f);
                    return Vector3.Normalize(inPlane + nFront * u);
                }
            default:
                return Random.UnitDirection();
        }
    }

    static Vector3 TriangleSupport(Vector3 a, Vector3 b, Vector3 c, Vector3 direction)
    {
        var dotA = Vector3.Dot(a, direction);
        var dotB = Vector3.Dot(b, direction);
        var dotC = Vector3.Dot(c, direction);
        return dotA > dotB ? dotA > dotC ? a : c : dotB > dotC ? b : c;
    }

    public TrianglePairCase Next()
    {
        TrianglePairCase testCase;
        testCase.A = NextTriangle(out var triScaleA);
        testCase.B = NextTriangle(out var triScaleB);
        testCase.OrientationB = Random.NextShapeOrientation();
        Matrix3x3.CreateFromQuaternion(testCase.OrientationB, out Matrix3x3 rb);
        Matrix3x3.Transform(testCase.B.A, rb, out var wbA);
        Matrix3x3.Transform(testCase.B.B, rb, out var wbB);
        Matrix3x3.Transform(testCase.B.C, rb, out var wbC);
        var hasNormalB = TryGetFrontNormal(wbA, wbB, wbC, out var nFrontBWorld);
        var hasNormalALocal = TryGetFrontNormal(testCase.A.A, testCase.A.B, testCase.A.C, out var nFrontALocal);

        testCase.OrientationA = NextOrientationA(nFrontALocal, hasNormalALocal, nFrontBWorld, hasNormalB, wbA, wbB, wbC, testCase.A);
        Matrix3x3.CreateFromQuaternion(testCase.OrientationA, out Matrix3x3 ra);
        Matrix3x3.Transform(testCase.A.A, ra, out var waA);
        Matrix3x3.Transform(testCase.A.B, ra, out var waB);
        Matrix3x3.Transform(testCase.A.C, ra, out var waC);
        var hasNormalA = TryGetFrontNormal(waA, waB, waC, out var nFrontAWorld);

        var vertReachA = MathF.Max(waA.Length(), MathF.Max(waB.Length(), waC.Length()));
        var vertReachB = MathF.Max(wbA.Length(), MathF.Max(wbB.Length(), wbC.Length()));
        var scale = MathF.Min(triScaleA, triScaleB);
        testCase.SpeculativeMargin = Random.NextSpeculativeMargin(scale);

        switch (ContactHeavy ? 8 + Random.Next(2) : Random.Next(10))
        {
            case 0:
                //Coincident frames: fully degenerate placement.
                testCase.OffsetB = default;
                break;
            case 1:
            case 2:
                //Purely random offset within the combined reach.
                {
                    var span = vertReachA + vertReachB;
                    testCase.OffsetB = new Vector3(
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span,
                        (Random.NextSingle() * 2 - 1) * span);
                    break;
                }
            case 8:
                //Face-face stack: centroid-aligned speculative face contact, the dominant real mesh configuration.
                //Vertex-support matching aligns corner-to-corner for face contacts (all of a triangle's plane ties in the
                //support direction), which mostly yields empty clip regions; aligning centroids guarantees footprint overlap.
                {
                    if (!hasNormalA || !hasNormalB)
                        goto default;
                    var n = Random.Next(8) == 0 ? -nFrontAWorld : nFrontAWorld;
                    if (Vector3.Dot(n, nFrontAWorld) < 0f)
                        (testCase.A.B, testCase.A.C) = (testCase.A.C, testCase.A.B);
                    if (Vector3.Dot(n, nFrontBWorld) > 0f)
                        (testCase.B.B, testCase.B.C) = (testCase.B.C, testCase.B.B);
                    //Zero-thickness face contact is inherently speculative (penetration flips the centroid calibration into
                    //backface rejection), so keep the separation inside a guaranteed-positive margin window.
                    if (testCase.SpeculativeMargin <= 0f)
                        testCase.SpeculativeMargin = scale * Random.LogUniform(1e-3f, 2e-1f);
                    var stackSeparation = testCase.SpeculativeMargin * (0.05f + Random.NextSingle() * 0.85f);
                    var centroidA = (waA + waB + waC) * (1f / 3f);
                    var centroidB = (wbA + wbB + wbC) * (1f / 3f);
                    var stackTangent = Vector3.Normalize(Vector3.Cross(n, Random.UnitDirection() + new Vector3(1e-3f, 2e-3f, 3e-3f)));
                    var slide = Random.Next(2) == 0 ? 0f : Random.NextSingle() * 0.5f * scale;
                    testCase.OffsetB = centroidA - centroidB + n * stackSeparation + stackTangent * slide;
                    break;
                }
            default:
                //Feature-informed support matching with controlled separation plus a tangential slide.
                {
                    var n = NextFeatureNormal(nFrontAWorld, hasNormalA, nFrontBWorld, hasNormalB, waA, waB, waC, wbA, wbB, wbC);
                    //Both one-sided triangles must face each other for the engine to emit contacts (A's front along the
                    //A-to-B normal, B's front against it). Swapping a triangle's B/C vertices negates its face normal while
                    //leaving the vertex set (and thus supports and placement) unchanged, so enforce mutual facing by winding:
                    //always for contact-heavy, half the time for the mixed profile.
                    if (ContactHeavy || Random.Next(2) == 0)
                    {
                        if (hasNormalA && Vector3.Dot(n, nFrontAWorld) < 0f)
                            (testCase.A.B, testCase.A.C) = (testCase.A.C, testCase.A.B);
                        if (hasNormalB && Vector3.Dot(n, nFrontBWorld) > 0f)
                            (testCase.B.B, testCase.B.C) = (testCase.B.C, testCase.B.B);
                    }
                    var separation = Random.NextSeparation(scale, testCase.SpeculativeMargin, ContactHeavy);
                    //Zero-thickness shapes make face-face PENETRATION self-rejecting: B's centroid falls behind A's plane,
                    //the centroid-based calibration flips the normal, and both backface conditions fail. Face-face triangle
                    //contact is inherently speculative — a small POSITIVE separation inside the margin. Bias contact-heavy
                    //toward that window half the time (rerolling a zero margin so the window exists).
                    if (ContactHeavy && Random.Next(2) == 0)
                    {
                        if (testCase.SpeculativeMargin <= 0f)
                            testCase.SpeculativeMargin = scale * Random.LogUniform(1e-3f, 2e-1f);
                        separation = testCase.SpeculativeMargin * (0.05f + Random.NextSingle() * 0.85f);
                    }
                    var supportA = TriangleSupport(waA, waB, waC, n);
                    var negatedN = -n;
                    var supportB = TriangleSupport(wbA, wbB, wbC, negatedN);
                    testCase.OffsetB = Random.FeaturePlacement(supportA, supportB, n, separation,
                        ContactHeavy ? scale : vertReachA + vertReachB, ContactHeavy);
                    break;
                }
        }
        return testCase;
    }
}
