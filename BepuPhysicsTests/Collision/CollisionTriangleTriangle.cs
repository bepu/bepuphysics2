using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionTriangleTriangle : CollisionBase<Triangle, TriangleWide, Triangle, TriangleWide, Convex4ContactManifoldWide, TrianglePairTester>
{
    protected override Triangle CreateShapeA() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));

    protected override Triangle CreateShapeB() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));

    [Fact]
    public void Concentric_Colliding()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertColliding(new Vector3(0, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void CrossingWithOffset_Colliding()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertColliding(new Vector3(0, 0.25f, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void OffsetAlongDiagonal_Separated()
    {
        AssertSeparated(new Vector3(1.1f, 0, 1.1f));
    }

    [Fact]
    public void OffsetAlongY_Separated()
    {
        AssertSeparated(new Vector3(0, 0.1f, 0));
    }

    [Fact]
    public void CoplanarIdentical_ValidManifold()
    {
        var manifold = Collide(Vector3.Zero);
        AssertValidManifold(manifold);
    }

    [Fact]
    public void CoplanarOffset_ValidManifold()
    {
        AssertValidManifold(Collide(new Vector3(0.5f, 0, 0.25f)));
    }

    [Fact]
    public void Coplanar_FarApart_NoContacts()
    {
        Assert.Equal(0, Collide(new Vector3(5, 0, 0)).Count);
        Assert.Equal(0, Collide(new Vector3(0, 0, -5)).Count);
    }

    [Fact]
    public void ParallelStacked_SeparatedByGap_NoContacts()
    {
        Assert.Equal(0, Collide(new Vector3(0, 0.5f, 0)).Count);
        Assert.Equal(0, Collide(new Vector3(0, -0.5f, 0)).Count);
    }

    [Fact]
    public void ParallelStacked_WithinMargin_ValidSpeculativeManifold()
    {
        foreach (var verticalOffset in new[] { 0.25f, -0.25f })
        {
            var manifold = Collide(new Vector3(0, verticalOffset, 0), 0.5f);
            AssertValidManifold(manifold, 0.5f);
            if (manifold.Count > 0)
                Assert.True(float.Abs(float.Abs(manifold.Normal.Y) - 1) < 1e-3f);
        }
    }

    [Fact]
    public void ParallelStacked_BeyondMargin_NoContacts()
    {
        Assert.Equal(0, Collide(new Vector3(0, 1f, 0), 0.5f).Count);
        Assert.Equal(0, Collide(new Vector3(0, -1f, 0), 0.5f).Count);
    }

    [Fact]
    public void PerpendicularPiercing_ValidManifold()
    {
        foreach (var verticalOffset in new[] { -0.5f, -0.1f, 0f, 0.1f, 0.5f })
            AssertValidManifold(Collide(new Vector3(0, verticalOffset, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f)));
    }

    [Fact]
    public void PerpendicularPiercing_FarAway_NoContacts()
    {
        Assert.Equal(0, Collide(new Vector3(0, 0, 5), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f)).Count);
        Assert.Equal(0, Collide(new Vector3(0, 5, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f)).Count);
    }

    [Fact]
    public void FacingEachOther_FlippedB_ValidManifold()
    {
        AssertValidManifold(Collide(new Vector3(0, -0.01f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi)));
        AssertValidManifold(Collide(new Vector3(0, 0.01f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi)));
    }

    [Fact]
    public void TiltedAboutVertex_ValidManifold()
    {
        foreach (var angle in new[] { 0.1f, 0.5f, 1f, 2f, 3f })
            AssertValidManifold(Collide(new Vector3(0, 0.05f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(angle)), 0f);
    }

    [Fact]
    public void RotatedInPlane_ValidManifold()
    {
        foreach (var angle in new[] { 0.25f, float.Pi * 0.25f, float.Pi * 0.5f, float.Pi })
            AssertValidManifold(Collide(Vector3.Zero, Quaternion.Identity, Quaternion.CreateFromYAxisAngle(angle)));
    }

    [Fact]
    public void EdgeEdgeCrossing_ValidManifold()
    {
        //Rotate B a quarter turn about Y and raise so edges cross like an X.
        var manifold = Collide(new Vector3(0, 0.1f, 0), Quaternion.Identity, Quaternion.CreateFromYAxisAngle(float.Pi * 0.5f) * Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f));
        AssertValidManifold(manifold);
    }

    [Fact]
    public void VertexTouchingFace_ValidManifold()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertValidManifold(Collide(new Vector3(0, 0, 0), Quaternion.Identity, orientationB));
        AssertValidManifold(Collide(new Vector3(-0.5f, 0.0f, -0.5f), Quaternion.Identity, orientationB));
    }

    [Fact]
    public void DegenerateTriangles_ValidManifold()
    {
        var degenerate = new Triangle(Vector3.Zero, Vector3.Zero, Vector3.Zero);
        var line = new Triangle(new Vector3(-1, 0, 0), new Vector3(0, 0, 0), new Vector3(1, 0, 0));
        var normal = CreateShapeA();
        foreach (var offset in new[] { Vector3.Zero, new Vector3(0, 0.1f, 0), new Vector3(0.5f, 0, 0) })
        {
            AssertValidManifold(Collide(degenerate, normal, 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
            AssertValidManifold(Collide(normal, degenerate, 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
            AssertValidManifold(Collide(line, normal, 0.1f, offset, Quaternion.Identity, Quaternion.Identity), 0.1f);
        }
    }

    [Fact]
    public void DifferentSizes_ValidManifold()
    {
        var largerTriangle = new Triangle(new Vector3(-10, 0, -10), new Vector3(10, 0, -10), new Vector3(-10, 0, 10));
        var smallerTriangle = new Triangle(new Vector3(-0.1f, 0, -0.1f), new Vector3(0.1f, 0, -0.1f), new Vector3(-0.1f, 0, 0.1f));
        AssertValidManifold(Collide(largerTriangle, smallerTriangle, 0.1f, new Vector3(0, 0.05f, 0), Quaternion.Identity, Quaternion.Identity), 0.1f);
        AssertValidManifold(Collide(smallerTriangle, largerTriangle, 0.1f, new Vector3(0, 0.05f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(0.5f)), 0.1f);
    }

    [Fact]
    public void SwappedAssignment_BothValid()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        var offset = new Vector3(0, 0.25f, 0);
        var firstThenSecondManifold = Collide(offset, Quaternion.Identity, orientationB);
        AssertValidManifold(firstThenSecondManifold);
        var secondThenFirstManifold = Collide(CreateShapeB(), CreateShapeA(), 0f, Vector3.Transform(-offset, Quaternion.Conjugate(orientationB)), orientationB, Quaternion.Identity);
        AssertValidManifold(secondThenFirstManifold);
    }

    [Fact]
    public void RandomOrientations_ValidManifolds()
    {
        var random = new System.Random(9);
        for (int iteration = 0; iteration < 300; ++iteration)
        {
            var offset = new Vector3((float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f) * 2;
            AssertValidManifold(Collide(offset, Quaternion.CreateRandom(random), Quaternion.CreateRandom(random), 0.1f), 0.1f);
        }
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        AssertBundleConsistent(new Vector3(0, 0.25f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f));
        AssertBundleConsistent(new Vector3(0, 0.01f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(0.3f), 0.1f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var rotation = Quaternion.Normalize(Quaternion.CreateFromYawPitchRoll(0.7f, 0.3f, -0.5f));
        AssertRotationInvariant(new Vector3(0, 0.25f, 0), Quaternion.Identity, Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f), rotation);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 2, 0), Quaternion.Identity, orientationB);
    }

}
