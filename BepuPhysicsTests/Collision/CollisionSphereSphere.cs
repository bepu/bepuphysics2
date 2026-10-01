using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereSphere : CollisionBase<Sphere, SphereWide, Sphere, SphereWide, Convex1ContactManifoldWide, SpherePairTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Sphere CreateShapeB() => new(1f);

    [Fact]
    public void Concentric_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Colliding()
    {
        AssertColliding(new Vector3(1.5f, 0, 0));
    }

    [Fact]
    public void OffsetAlongX_Separated()
    {
        AssertSeparated(new Vector3(2.1f, 0, 0));
    }

    [Fact]
    public void OffsetAlongY_Colliding()
    {
        AssertColliding(new Vector3(0, 1.5f, 0));
    }

    [Fact]
    public void OffsetAlongY_Separated()
    {
        AssertSeparated(new Vector3(0, 2.1f, 0));
    }

    [Fact]
    public void OffsetAlongZ_Colliding()
    {
        AssertColliding(new Vector3(0, 0, 1.5f));
    }

    [Fact]
    public void OffsetAlongZ_Separated()
    {
        AssertSeparated(new Vector3(0, 0, 2.1f));
    }

    [Fact]
    public void OffsetAlongXYZ_Colliding()
    {
        AssertColliding(new Vector3(1.1f, 1.1f, 1.1f));
    }

    [Fact]
    public void OffsetAlongXYZ_Separated()
    {
        AssertSeparated(new Vector3(1.5f, 1.5f, 1.5f));
    }

    [Fact]
    public void OverlappingPair_HasSingleContact()
    {
        AssertContactCount(1, new Vector3(1.5f, 0, 0));
    }

    [Fact]
    public void Normal_PointsFromBToA()
    {
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.5f, 0, 0));
        AssertNormal(new Vector3(0, 0, 1), new Vector3(0, 0, -1.5f));
        AssertNormal(new Vector3(-1, -1, -1), new Vector3(1, 1, 1));
    }

    [Fact]
    public void Concentric_UsesFallbackNormal()
    {
        AssertNormal(new Vector3(0, 1, 0), Vector3.Zero);
    }

    [Fact]
    public void Concentric_DepthIsSumOfRadii()
    {
        AssertMaximumDepth(2f, Vector3.Zero);
    }

    [Fact]
    public void Depth_MatchesPenetration()
    {
        AssertMaximumDepth(0.5f, new Vector3(1.5f, 0, 0));
        AssertMaximumDepth(0.25f, new Vector3(0, 1.75f, 0));
    }

    [Fact]
    public void ContactPosition_IsMidpointOfOverlap()
    {
        //Overlap spans x in [0.5, 1], so the midpoint is 0.75 from A.
        AssertHasContactAt(new Vector3(0.75f, 0, 0), new Vector3(1.5f, 0, 0));
        AssertHasContactAt(new Vector3(0, -0.75f, 0), new Vector3(0, -1.5f, 0));
    }

    [Fact]
    public void Touching_IsColliding()
    {
        AssertMaximumDepth(0f, new Vector3(2f, 0, 0), speculativeMargin: 0.01f);
    }

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        AssertColliding(new Vector3(2.1f, 0, 0), 0.2f);
        AssertMaximumDepth(-0.1f, new Vector3(2.1f, 0, 0), speculativeMargin: 0.2f);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(2.1f, 0, 0), speculativeMargin: 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated()
    {
        AssertSeparated(new Vector3(2.3f, 0, 0), 0.2f);
    }

    [Fact]
    public void Orientation_DoesNotAffectResult()
    {
        var firstSphereRotation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var secondSphereRotation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertColliding(new Vector3(1.5f, 0, 0), firstSphereRotation, secondSphereRotation);
        AssertSeparated(new Vector3(2.1f, 0, 0), firstSphereRotation, secondSphereRotation);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.5f, 0, 0), firstSphereRotation, secondSphereRotation);
    }

    [Fact]
    public void BundleResult_MatchesSingleLane()
    {
        AssertBundleConsistent(new Vector3(1.5f, 0.2f, -0.3f), Quaternion.Identity, Quaternion.Identity);
        AssertBundleConsistent(new Vector3(2.1f, 0, 0), Quaternion.Identity, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        AssertRotationInvariant(new Vector3(1.2f, 0.4f, -0.5f), Quaternion.Identity, Quaternion.Identity, world);
    }

    [Fact]
    public void DifferentRadii_DepthAndContact()
    {
        var manifold = Collide(new Sphere(2f), new Sphere(0.5f), 0f, new Vector3(2f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        Assert.InRange(manifold.Contact0.Depth, 0.5f - 1e-4f, 0.5f + 1e-4f);
        //Overlap spans x in [1.5, 2], so the midpoint is 1.75 from A.
        AssertEqual(new Vector3(1.75f, 0, 0), manifold.Contact0.Offset);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
    }
}
