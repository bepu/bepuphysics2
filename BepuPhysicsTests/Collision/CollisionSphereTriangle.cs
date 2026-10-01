using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereTriangle : CollisionBase<Sphere, SphereWide, Triangle, TriangleWide, Convex1ContactManifoldWide, SphereTriangleTester>
{
    protected override Sphere CreateShapeA() => new(1f);

    protected override Triangle CreateShapeB() => new(
        new Vector3(-1, 0, -1),
        new Vector3(1, 0, -1),
        new Vector3(-1, 0, 1));

    [Fact]
    public void FaceRegion_Colliding()
    {
        AssertColliding(new Vector3(0, -0.5f, -0.25f));
    }

    [Fact]
    public void EdgeRegion_Colliding()
    {
        AssertColliding(new Vector3(-0.5f, 0, -0.5f));
    }

    [Fact]
    public void OppositeSide_Colliding()
    {
        AssertColliding(new Vector3(0, -0.5f, -0.25f));
    }

    [Fact]
    public void FaceRegion_Separated()
    {
        AssertSeparated(new Vector3(0, 1.1f, -0.25f));
    }

    [Fact]
    public void OutsideTriangle_Separated()
    {
        AssertSeparated(new Vector3(0, 0, 2.1f));
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.5f);
        AssertSeparated(new Vector3(0, 0, 2.1f), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Face_NormalDepthAndContactPosition()
    {
        var offset = new Vector3(0.3f, -0.5f, 0.3f);
        AssertSingleContact(offset, new Vector3(0, 1, 0), 0.5f);
        AssertHasContactAt(new Vector3(0, -0.5f, 0), offset);
    }

    [Fact]
    public void Face_SphereBarelyTouching()
    {
        AssertSingleContact(new Vector3(0.3f, -0.99f, 0.3f), new Vector3(0, 1, 0), 0.01f);
        AssertSeparated(new Vector3(0.3f, -1.01f, 0.3f));
    }

    [Fact]
    public void Face_DeepPenetration_CenterJustAboveSurface()
    {
        AssertSingleContact(new Vector3(0.3f, -0.01f, 0.3f), new Vector3(0, 1, 0), 0.99f);
    }

    [Fact]
    public void Face_SphereCenterOnSurface_NoContact()
    {
        AssertSeparated(new Vector3(0.3f, 0, 0.3f));
    }

    [Fact]
    public void Backface_NoContactEvenWhenPenetrating()
    {
        AssertSeparated(new Vector3(0.3f, 0.5f, 0.3f));
        AssertSeparated(new Vector3(0.3f, 0.01f, 0.3f));
        AssertSeparated(new Vector3(0.3f, 1.5f, 0.3f));
        AssertSeparated(new Vector3(0.3f, 0.5f, 0.3f), 0.5f);
    }

    [Fact]
    public void Backface_SpeculativeMarginDoesNotCreateContacts()
    {
        AssertSeparated(new Vector3(0.3f, 1.1f, 0.3f), 0.3f);
    }

    [Fact]
    public void EdgeBetweenFirstAndSecondVertices_NormalPointsAwayFromEdge()
    {
        AssertSingleContact(new Vector3(0, -0.5f, 1.5f), Vector3.Normalize(new Vector3(0, 1, -1)), 1f - float.Sqrt(0.5f));
    }

    [Fact]
    public void EdgeBetweenFirstAndThirdVertices_NormalPointsAwayFromEdge()
    {
        AssertSingleContact(new Vector3(1.5f, -0.5f, 0), Vector3.Normalize(new Vector3(-1, 1, 0)), 1f - float.Sqrt(0.5f));
    }

    [Fact]
    public void HypotenuseEdge_NormalPointsAwayFromEdge()
    {
        AssertSingleContact(new Vector3(-0.5f, -0.5f, -0.5f), Vector3.Normalize(new Vector3(1, 1, 1)), 1f - SquareRootOfThreeOverTwo);
    }

    [Fact]
    public void Edge_ContactLiesOnEdge()
    {
        AssertHasContactAt(new Vector3(0, -0.5f, 0.5f), new Vector3(0, -0.5f, 1.5f));
    }

    [Fact]
    public void FirstVertex_NormalPointsAwayFromVertex()
    {
        AssertSingleContact(new Vector3(1.5f, -0.5f, 1.5f), Vector3.Normalize(new Vector3(-1, 1, -1)), 1f - SquareRootOfThreeOverTwo);
        AssertHasContactAt(new Vector3(0.5f, -0.5f, 0.5f), new Vector3(1.5f, -0.5f, 1.5f));
    }

    [Fact]
    public void SecondVertex_NormalPointsAwayFromVertex()
    {
        AssertSingleContact(new Vector3(-1.5f, -0.5f, 1.5f), Vector3.Normalize(new Vector3(1, 1, -1)), 1f - SquareRootOfThreeOverTwo);
    }

    [Fact]
    public void ThirdVertex_NormalPointsAwayFromVertex()
    {
        AssertSingleContact(new Vector3(1.5f, -0.5f, -1.5f), Vector3.Normalize(new Vector3(-1, 1, 1)), 1f - SquareRootOfThreeOverTwo);
    }

    [Fact]
    public void BeyondVertexRegion_Separated()
    {
        AssertSeparated(new Vector3(2.0f, -0.5f, 2.0f));
        AssertSeparated(new Vector3(-2.0f, -0.5f, 2.0f));
        AssertSeparated(new Vector3(2.0f, -0.5f, -2.0f));
    }

    [Fact]
    public void EdgeRegion_BackSide_NoContact()
    {
        AssertSeparated(new Vector3(0, 0.5f, 1.5f));
        AssertSeparated(new Vector3(1.5f, 0.5f, 0));
        AssertSeparated(new Vector3(-0.5f, 0.5f, -0.5f));
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        var offset = new Vector3(0.3f, -1.1f, 0.3f);
        AssertSeparated(offset);
        AssertSingleContact(offset, new Vector3(0, 1, 0), -0.1f, 0.2f);
    }

    [Fact]
    public void EdgeSeparatedWithinMargin_ProducesSpeculativeContact()
    {
        //Sphere at (0, 0.8, -1.8) relative to the triangle: distance to edge point (0,0,-1) is sqrt(0.64 + 0.64).
        var offset = new Vector3(0, -0.8f, 1.8f);
        var distance = float.Sqrt(1.28f);
        AssertSeparated(offset);
        AssertSingleContact(offset, Vector3.Normalize(new Vector3(0, 1, -1)), 1f - distance, 0.3f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(0.3f, -1.3f, 0.3f), 0.2f);
        AssertSeparated(new Vector3(0, 0, 2.3f), 0.2f);
        AssertSeparated(new Vector3(2.3f, 0, 0), 0.2f);
    }

    [Fact]
    public void ReversedWinding_SwapsFrontAndBack()
    {
        var reversed = new Triangle(new Vector3(-1, 0, -1), new Vector3(-1, 0, 1), new Vector3(1, 0, -1));
        Assert.Equal(0, Collide(new Sphere(1), reversed, 0f, new Vector3(0.3f, -0.5f, 0.3f), Quaternion.Identity, Quaternion.Identity).Count);
        var manifold = Collide(new Sphere(1), reversed, 0f, new Vector3(0.3f, 0.5f, 0.3f), Quaternion.Identity, Quaternion.Identity);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, -1, 0), manifold.Normal, 1e-3f);
    }

    [Fact]
    public void DegenerateTriangles_NoContact()
    {
        var sphere = new Sphere(1);
        var collinear = new Triangle(new Vector3(-1, 0, 0), new Vector3(0, 0, 0), new Vector3(1, 0, 0));
        var point = new Triangle(Vector3.Zero, Vector3.Zero, Vector3.Zero);
        var twoCoincident = new Triangle(Vector3.Zero, Vector3.Zero, new Vector3(1, 0, 1));
        foreach (var triangle in new[] { collinear, point, twoCoincident })
            Assert.Equal(0, Collide(sphere, triangle, 0f, new Vector3(0, -0.5f, 0.1f), Quaternion.Identity, Quaternion.Identity).Count);
    }

    [Fact]
    public void TinyAndSliverTriangles_ValidManifold()
    {
        var sphere = new Sphere(1);
        var tiny = new Triangle(new Vector3(-1e-3f, 0, -1e-3f), new Vector3(1e-3f, 0, -1e-3f), new Vector3(-1e-3f, 0, 1e-3f));
        var sliver = new Triangle(new Vector3(-1, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1e-3f));
        AssertValidManifold(Collide(sphere, tiny, 0f, new Vector3(0, -0.5f, 0), Quaternion.Identity, Quaternion.Identity));
        AssertValidManifold(Collide(sphere, sliver, 0f, new Vector3(0, -0.5f, 0), Quaternion.Identity, Quaternion.Identity));
    }

    [Fact]
    public void LargeTriangle_FaceContact()
    {
        var large = new Triangle(new Vector3(-100, 0, -100), new Vector3(300, 0, -100), new Vector3(-100, 0, 300));
        var manifold = Collide(new Sphere(1), large, 0f, new Vector3(0, -0.5f, 0), Quaternion.Identity, Quaternion.Identity);
        Assert.Equal(1, manifold.Count);
        AssertEqual(new Vector3(0, 1, 0), manifold.Normal, 1e-3f);
        Assert.Equal(0.5f, manifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void DifferentSphereRadius_DepthChanges()
    {
        var manifold = Collide(new Sphere(0.25f), DefaultTriangle(), 0f, new Vector3(0.3f, -0.1f, 0.3f), Quaternion.Identity, Quaternion.Identity);
        Assert.Equal(1, manifold.Count);
        Assert.Equal(0.15f, manifold.GetDepth(0), 1e-3f);
        Assert.Equal(0, Collide(new Sphere(0.25f), DefaultTriangle(), 0f, new Vector3(0.3f, -0.3f, 0.3f), Quaternion.Identity, Quaternion.Identity).Count);
    }

    [Fact]
    public void SphereOrientation_DoesNotMatter()
    {
        var spin = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.3f);
        var offset = new Vector3(0, -0.5f, 1.5f);
        var originalManifold = Collide(offset);
        var rotatedManifold = Collide(offset, spin, Quaternion.Identity);
        AssertEqual(originalManifold.Normal, rotatedManifold.Normal, 1e-3f);
        Assert.Equal(originalManifold.GetDepth(0), rotatedManifold.GetDepth(0), 1e-3f);
    }

    [Fact]
    public void TiltedTriangle_ValidManifold()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.6f);
        var manifold = Collide(new Vector3(0.1f, -0.5f, 0.1f), Quaternion.Identity, orientationB);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertBundleConsistent(new Vector3(0.3f, -0.5f, 0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.3f, 0.5f, 0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0, -0.5f, 1.5f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(1.5f, -0.5f, 1.5f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(0.1f, 0.2f, 0.1f), Quaternion.Identity, orientationB);
        AssertBundleConsistent(new Vector3(0.3f, -1.1f, 0.3f), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        AssertRotationInvariant(new Vector3(0.3f, -0.5f, 0.3f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(0, -0.5f, 1.5f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(1.5f, -0.5f, 1.5f), Quaternion.Identity, Quaternion.Identity, world);
        AssertRotationInvariant(new Vector3(0.3f, 0.5f, 0.3f), Quaternion.Identity, Quaternion.Identity, world);
    }

    private static float SquareRootOfThreeOverTwo => float.Sqrt(3f) / 2f;

    private static Triangle DefaultTriangle() => new(new Vector3(-1, 0, -1), new Vector3(1, 0, -1), new Vector3(-1, 0, 1));

    private void AssertSingleContact(Vector3 secondShapeOffset, Vector3 expectedNormal, float expectedDepth, float speculativeMargin = 0f)
    {
        var manifold = Collide(secondShapeOffset, speculativeMargin);
        AssertValidManifold(manifold, speculativeMargin);
        Assert.Equal(1, manifold.Count);
        AssertEqual(expectedNormal, manifold.Normal, 1e-3f);
        Assert.Equal(expectedDepth, manifold.GetDepth(0), 1e-3f);
    }
}
