using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;

namespace BepuPhysicsTests.Collision;

public class CollisionSphereCapsule : CollisionBase<Sphere, SphereWide, Capsule, CapsuleWide, Convex1ContactManifoldWide, SphereCapsuleTester>
{
    protected override Sphere CreateShapeA() => new(1f);
    protected override Capsule CreateShapeB() => new(1, 2);

    [Fact]
    public void Concentric_Colliding() => AssertColliding(new Vector3(0, 0, 0));

    [Fact]
    public void OffsetAlongX_Colliding() => AssertColliding(new Vector3(1.5f, 0, 0));

    [Fact]
    public void OffsetAlongX_Separated() => AssertSeparated(new Vector3(2.1f, 0, 0));

    [Fact]
    public void OffsetAlongY_Colliding() => AssertColliding(new Vector3(0, 2.5f, 0));

    [Fact]
    public void OffsetAlongY_Separated() => AssertSeparated(new Vector3(0, 3.1f, 0));

    [Fact]
    public void OffsetAlongZ_Colliding() => AssertColliding(new Vector3(0, 0, 1.5f));

    [Fact]
    public void OffsetAlongZ_Separated() => AssertSeparated(new Vector3(0, 0, 2.1f));

    [Fact]
    public void OffsetAlongXYZ_Colliding() => AssertColliding(new Vector3(1.1f, 1.1f, 1.1f));

    [Fact]
    public void OffsetAlongXYZ_Separated() => AssertSeparated(new Vector3(1.5f, 2.5f, 1.5f));

    [Fact]
    public void Rotated_Colliding()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.25f);
        AssertColliding(new Vector3(1.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void Rotated_Separated()
    {
        var orientationB = Quaternion.CreateFromXAxisAngle(float.Pi * 0.25f);
        AssertSeparated(new Vector3(2.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void OverlappingPair_HasSingleContact() => AssertContactCount(1, new Vector3(1.5f, 0, 0));

    [Fact]
    public void Side_NormalPointsFromCapsuleToSphere()
    {
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.5f, 0, 0));
        AssertNormal(new Vector3(0, 0, 1), new Vector3(0, 0, -1.5f));
        //The sphere is beside the cylindrical section, so the closest point is level with the sphere center.
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(1.5f, 0.5f, 0));
    }

    [Fact]
    public void Side_DepthAndContactPosition()
    {
        AssertMaximumDepth(0.5f, new Vector3(1.5f, 0, 0));
        AssertHasContactAt(new Vector3(0.75f, 0, 0), new Vector3(1.5f, 0, 0));
    }

    [Fact]
    public void EndCap_NormalDepthAndContactPosition()
    {
        //Capsule segment spans y in [1.5, 3.5] for offset 2.5; the closest point is (0, 1.5, 0).
        AssertNormal(new Vector3(0, -1, 0), new Vector3(0, 2.5f, 0));
        AssertMaximumDepth(0.5f, new Vector3(0, 2.5f, 0));
        AssertHasContactAt(new Vector3(0, 0.75f, 0), new Vector3(0, 2.5f, 0));

        AssertNormal(new Vector3(0, 1, 0), new Vector3(0, -2.5f, 0));
        AssertHasContactAt(new Vector3(0, -0.75f, 0), new Vector3(0, -2.5f, 0));
    }

    [Fact]
    public void EndCapDiagonal_ClampsToSegmentEnd()
    {
        //The closest point on the segment is its lower endpoint at (1, 1, 0) relative to the sphere, so the distance is sqrt(2).
        var offset = new Vector3(1, 2, 0);
        AssertNormal(new Vector3(-1, -1, 0), offset);
        AssertMaximumDepth(2f - float.Sqrt(2f), offset);
    }

    [Fact]
    public void Concentric_UsesCapsuleLocalXAsFallbackNormal()
    {
        AssertNormal(new Vector3(1, 0, 0), Vector3.Zero);
        AssertMaximumDepth(2f, Vector3.Zero);
        //Rotating the capsule about Z by 90 degrees takes its local X to world Y.
        AssertNormal(new Vector3(0, 1, 0), Vector3.Zero, Quaternion.Identity, Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f));
    }

    [Fact]
    public void SphereOnSegmentInterior_HasValidManifold()
    {
        var manifold = Collide(new Vector3(0, 0.5f, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
    }

    [Fact]
    public void Touching_IsColliding() => AssertMaximumDepth(0f, new Vector3(2f, 0, 0), speculativeMargin: 0.01f);

    [Fact]
    public void SeparatedWithinMargin_ProducesSpeculativeContact()
    {
        AssertSeparated(new Vector3(0, 3.1f, 0));
        AssertColliding(new Vector3(0, 3.1f, 0), 0.2f);
        AssertMaximumDepth(-0.1f, new Vector3(0, 3.1f, 0), speculativeMargin: 0.2f);
        AssertNormal(new Vector3(0, -1, 0), new Vector3(0, 3.1f, 0), speculativeMargin: 0.2f);
    }

    [Fact]
    public void SeparatedBeyondMargin_Separated() => AssertSeparated(new Vector3(0, 3.3f, 0), 0.2f);

    [Fact]
    public void CapsuleRotatedToLieAlongX_ContactOnEndCap()
    {
        //Rotating about Z by 90 degrees puts the segment along X, spanning [1.5, 3.5] for offset 2.5.
        var orientationB = Quaternion.CreateFromZAxisAngle(float.Pi * 0.5f);
        AssertNormal(new Vector3(-1, 0, 0), new Vector3(2.5f, 0, 0), Quaternion.Identity, orientationB);
        AssertMaximumDepth(0.5f, new Vector3(2.5f, 0, 0), Quaternion.Identity, orientationB);
        AssertHasContactAt(new Vector3(0.75f, 0, 0), new Vector3(2.5f, 0, 0), Quaternion.Identity, orientationB);
    }

    [Fact]
    public void SphereOrientation_DoesNotAffectResult()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 1.1f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-2, 1, 0.5f)), 2.3f);
        AssertValidManifold(Collide(new Vector3(1.2f, 0.3f, 0.4f), orientationA, orientationB));
        AssertBundleConsistent(new Vector3(1.2f, 0.3f, 0.4f), orientationA, orientationB);
        AssertBundleConsistent(new Vector3(0, 3.1f, 0), orientationA, Quaternion.Identity, 0.2f);
    }

    [Fact]
    public void RotatingWholeConfiguration_RotatesNormal()
    {
        var world = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.3f, 1, -0.6f)), 0.9f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0.7f);
        AssertRotationInvariant(new Vector3(1.2f, 0.4f, -0.5f), Quaternion.Identity, orientationB, world);
    }

    [Fact]
    public void DifferentSizes_DepthAndContact()
    {
        var manifold = Collide(new Sphere(0.5f), new Capsule(0.5f, 4f), 0f, new Vector3(0.8f, 0, 0), Quaternion.Identity, Quaternion.Identity);
        AssertValidManifold(manifold);
        Assert.Equal(1, manifold.Count);
        Assert.InRange(manifold.Contact0.Depth, 0.2f - 1e-4f, 0.2f + 1e-4f);
        AssertEqual(new Vector3(0.4f, 0, 0), manifold.Contact0.Offset);
        AssertEqual(new Vector3(-1, 0, 0), manifold.Normal);
    }
}
